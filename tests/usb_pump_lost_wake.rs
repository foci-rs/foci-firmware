//! Regression test: the USB pump must never park with an unread packet in the
//! OTG RX FIFO while the RXFLVL interrupt is masked.
//!
//! Real code under test: `foci_firmware::usb_pump::run_once` and
//! `ankyra::transport::Transport::receive`. Modelled after the board and
//! `foci-usb-stm32` code: the CDC RX buffer (`cdc::bulk_out_task` /
//! `rx_consume` semantics), the OTG RX FIFO with its level-triggered RXFLVL
//! interrupt and the RXFLVLM mask that the ISR clears and only a NotReady read
//! re-arms, the board `USB_WAKE` signal, a bulk OUT endpoint that NAKs until
//! re-enabled, a lossy cable, and a Klippy-like host (serialqueue.c) that acks,
//! naks and retransmits SYNC + every unacked frame as one write that the host
//! controller splits into 64-byte packets.
//!
//! Before the pump re-ran on a pending driver notify, a retransmit burst with a
//! dropped packet could leave the task asleep with data in the RX buffer, a
//! packet in the FIFO and the interrupt masked; the host then saw no acks until
//! an unrelated transmit woke the task, which Klippy reports as "Lost
//! communication with MCU".

use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::string::String;

use ankyra::encoding::ReadError;
use ankyra::transport::{Config, ShutdownState, Transport};
use ankyra::{InputBuffer, ScratchOutput, SliceInputBuffer, TransportOutput};
use foci_firmware::usb_pump::{UsbPumpBackend, UsbPumpCounters, run_once};

const SYNC: u8 = 0x7E;
const DEST: u8 = 0x10;
const SEQ_MASK: u8 = 0x0F;
const MAX_PENDING_BLOCKS: usize = 12;
const PKT: usize = 64;
const RX_BUF: usize = 128;

thread_local! {
    static TRACE: RefCell<VecDeque<String>> = const { RefCell::new(VecDeque::new()) };
    static DEV_TX: RefCell<Vec<u8>> = const { RefCell::new(Vec::new()) };
    static USB_WAKE: Cell<bool> = const { Cell::new(false) };
    static DISPATCHED: RefCell<Vec<u16>> = const { RefCell::new(Vec::new()) };
}

struct Out;
impl TransportOutput for Out {
    type Output = ScratchOutput;
    fn output(&self, f: impl FnOnce(&mut Self::Output)) {
        // Mirrors foci_firmware::transport::AnkyraUsbTransport: write, then wake.
        let mut s = ScratchOutput::new();
        f(&mut s);
        DEV_TX.with(|t| t.borrow_mut().extend_from_slice(s.result()));
        USB_WAKE.with(|w| w.set(true));
    }
}
struct Ctx;
impl ShutdownState for Ctx {
    fn is_shutdown(&self) -> bool {
        false
    }
}
struct Cfg;
impl Config for Cfg {
    type TransportOutput = Out;
    type Context<'c> = Ctx;
    fn dispatch(cmd: u16, frame: &mut &[u8], _ctx: &mut Ctx) -> Result<(), ReadError> {
        // Every test command carries a 4-byte payload after the VLQ id.
        if frame.len() < 4 {
            return Err(ReadError);
        }
        if cmd != PAD_CMD {
            DISPATCHED.with(|d| d.borrow_mut().push(cmd));
        }
        *frame = &frame[4..];
        Ok(())
    }
}
static CFG: Cfg = Cfg;

fn tr(msg: String) {
    TRACE.with(|t| {
        let mut t = t.borrow_mut();
        if t.len() > 60 {
            t.pop_front();
        }
        t.push_back(msg);
    });
}

fn crc16(buf: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for b in buf {
        let b = *b ^ ((crc & 0xFF) as u8);
        let b = b ^ (b << 4);
        let b16 = b as u16;
        crc = ((b16 << 8) | (crc >> 8)) ^ (b16 >> 4) ^ (b16 << 3);
    }
    crc
}

/// Build a Klipper frame: len, seq, payload (repeated cmd id + 4 byte arg), crc, sync.
fn frame(seq: u8, cmds: &[(u16, u32)]) -> Vec<u8> {
    let mut v = vec![0u8, (seq & SEQ_MASK) | DEST];
    for (id, arg) in cmds {
        // VLQ encode id (ids < 64 are one byte in Klipper's VLQ)
        assert!(*id < 64);
        v.push(*id as u8);
        v.extend_from_slice(&arg.to_le_bytes());
    }
    v[0] = (v.len() + 3) as u8;
    let c = crc16(&v);
    v.push((c >> 8) as u8);
    v.push(c as u8);
    v.push(SYNC);
    v
}

/// Padding command id; ignored by the dispatch recorder.
const PAD_CMD: u16 = 63;

/// Re-encode `f` (len, seq, cmds.., crc, sync) as an exact `target`-byte frame
/// by appending padding commands, when the sizes allow it.
fn pad_frame_to(f: Vec<u8>, target: usize) -> Vec<u8> {
    let payload = &f[2..f.len() - 3];
    let needed = target.saturating_sub(f.len());
    if needed == 0 || !needed.is_multiple_of(5) {
        return f;
    }
    let seq = f[1];
    let mut v = vec![0u8, seq];
    v.extend_from_slice(payload);
    for _ in 0..needed / 5 {
        v.push(PAD_CMD as u8);
        v.extend_from_slice(&0u32.to_le_bytes());
    }
    v[0] = (v.len() + 3) as u8;
    let c = crc16(&v);
    v.push((c >> 8) as u8);
    v.push(c as u8);
    v.push(SYNC);
    v
}

enum Entry {
    Data(Vec<u8>),
    XferCompl,
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn chance(&mut self, p: f64) -> bool {
        (self.next() % 10_000) as f64 / 10_000.0 < p
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

/// Device-side model: OTG RX FIFO + CDC RX buffer.
struct Dev {
    fifo: VecDeque<Entry>,
    rxflvlm: bool,
    ep_enabled: bool,
    inflight: VecDeque<Vec<u8>>, // packets the host is offering (NAKed until ep enabled)
    rx: [u8; RX_BUF],
    rpos: usize,
    bulk_out_wake: bool,
    report_pending: bool,
    deliveries: u32,
    driver_wake: bool,
}

impl Dev {
    fn new() -> Self {
        Self {
            fifo: VecDeque::new(),
            rxflvlm: true,
            ep_enabled: true,
            inflight: VecDeque::new(),
            rx: [0; RX_BUF],
            rpos: 0,
            bulk_out_wake: false,
            report_pending: false,
            deliveries: 0,
            driver_wake: false,
        }
    }
    /// Hardware: accept one offered packet if the endpoint is enabled.
    fn deliver(&mut self) {
        if self.ep_enabled
            && let Some(p) = self.inflight.pop_front()
        {
            self.fifo.push_back(Entry::Data(p));
            self.fifo.push_back(Entry::XferCompl);
            self.ep_enabled = false; // PKTCNT=1 exhausted
            self.deliveries += 1;
        }
    }
    /// irq_handler(): RXFLVL is level-triggered on a non-empty FIFO.
    fn isr(&mut self) {
        if !self.fifo.is_empty() && self.rxflvlm {
            tr(format!("ISR fires (fifo_len={})", self.fifo.len()));
            self.rxflvlm = false;
            self.bulk_out_wake = true;
            self.driver_wake = true;
            USB_WAKE.with(|w| w.set(true));
        }
    }
    /// otg::usb_read_bulk_out() under usb_irq_disable/enable.
    fn usb_read_bulk_out(&mut self) -> Option<Vec<u8>> {
        // peek_rx_queue: pop informational entries
        let pkt = loop {
            match self.fifo.front() {
                None => {
                    self.rxflvlm = true;
                    tr("read: NotReady, RXFLVLM armed".into());
                    self.isr();
                    return None;
                }
                Some(Entry::XferCompl) => {
                    self.fifo.pop_front();
                }
                Some(Entry::Data(_)) => {
                    if let Some(Entry::Data(p)) = self.fifo.pop_front() {
                        break p;
                    } else {
                        unreachable!()
                    }
                }
            }
        };
        // enable_rx_endpoint: the hardware may take the next packet immediately
        self.ep_enabled = true;
        tr(format!(
            "read: got {} bytes {:02x?}..",
            pkt.len(),
            &pkt[..pkt.len().min(6)]
        ));
        self.deliver();
        // usb_irq_enable
        self.isr();
        Some(pkt)
    }
    /// cdc::bulk_out_task()
    fn bulk_out_task(&mut self) -> bool {
        let woke = std::mem::replace(&mut self.bulk_out_wake, false);
        if !woke {
            return self.rpos > 0;
        }
        if self.rpos + PKT <= RX_BUF {
            if let Some(p) = self.usb_read_bulk_out() {
                // cdc::bulk_out_task re-notifies after every successful read,
                // including a zero-length packet, so the next poll re-arms RXFLVL.
                self.rx[self.rpos..self.rpos + p.len()].copy_from_slice(&p);
                self.rpos += p.len();
                self.bulk_out_wake = true;
                self.driver_wake = true;
            }
        } else {
            self.bulk_out_wake = true;
            self.driver_wake = true;
        }
        self.rpos > 0
    }
    /// cdc::rx_consume()
    fn rx_consume(&mut self, len: usize) {
        if len >= self.rpos {
            self.rpos = 0;
            return;
        }
        let need = self.rpos - len;
        self.rx.copy_within(len..self.rpos, 0);
        self.rpos = need;
        // Proposed fix 2 (matches Klipper usb_bulk_out_task): only self-notify
        // when a block was actually consumed and bytes remain.
        if len > 0 && need > 0 {
            self.bulk_out_wake = true;
            self.driver_wake = true;
        }
    }
}

struct Backend<'a> {
    dev: &'a RefCell<Dev>,
    host_steps_during_yield: u32,
    transport: &'a Transport<Cfg>,
}

impl UsbPumpBackend for Backend<'_> {
    async fn wait_wake(&mut self) {
        tr("run_once: wake consumed".into());
        USB_WAKE.with(|w| w.set(false));
    }
    fn poll(&mut self) -> bool {
        self.dev.borrow_mut().bulk_out_task()
    }
    fn rx_data(&self) -> &[u8] {
        // Safety-free trick: leak a copy for the borrow; the pump copies immediately anyway.
        let d = self.dev.borrow();
        Box::leak(d.rx[..d.rpos].to_vec().into_boxed_slice())
    }
    fn rx_consume(&mut self, n: usize) {
        self.dev.borrow_mut().rx_consume(n);
    }
    fn dispatch(&mut self, chunk: &[u8]) -> usize {
        let mut input = SliceInputBuffer::new(chunk);
        let before = DEV_TX.with(|t| t.borrow().len());
        self.transport.receive(&mut input, Ctx);
        let consumed = chunk.len() - input.available();
        let out = DEV_TX.with(|t| t.borrow().len()) - before;
        tr(format!(
            "dispatch {} bytes {:02x?}.. consumed={} output_bytes={}",
            chunk.len(),
            &chunk[..chunk.len().min(6)],
            consumed,
            out
        ));
        consumed
    }
    fn report_pending(&self) -> bool {
        self.dev.borrow().report_pending
    }
    fn wake(&mut self) {
        USB_WAKE.with(|w| w.set(true));
    }
    async fn yield_now(&mut self) {
        // Other tasks run; the host may land packets meanwhile.
        for _ in 0..self.host_steps_during_yield {
            let mut d = self.dev.borrow_mut();
            d.deliver();
            d.isr();
        }
    }
    fn final_poll(&mut self) {
        tr("final_poll".into());
        self.dev.borrow_mut().bulk_out_task();
        tr(format!(
            "run_once ends: rpos={} usb_wake={}",
            self.dev.borrow().rpos,
            USB_WAKE.with(|w| w.get())
        ));
    }
    fn driver_wake_pending(&mut self) -> bool {
        std::mem::replace(&mut self.dev.borrow_mut().driver_wake, false)
    }
    fn on_wake(&mut self) {}
    fn on_drain_complete(&mut self) {}
}

fn block_on<F: std::future::Future>(mut fut: F) -> F::Output {
    use std::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
    fn noop(_: *const ()) {}
    fn clone(_: *const ()) -> RawWaker {
        RawWaker::new(std::ptr::null(), &VT)
    }
    static VT: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
    let waker = unsafe { Waker::from_raw(RawWaker::new(std::ptr::null(), &VT)) };
    let mut cx = Context::from_waker(&waker);
    let mut fut = unsafe { std::pin::Pin::new_unchecked(&mut fut) };
    loop {
        if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return v;
        }
    }
}

/// Klippy-like host.
struct Host {
    send_seq: u64,    // next sequence to assign
    receive_seq: u64, // highest ack seen (1-based like serialqueue)
    sent: VecDeque<(u64, Vec<u8>)>,
    ignore_nak_seq: u64,
    retransmit_seq: u64,
    retransmits: u32,
    bytes_retransmit: usize,
    next_cmd_arg: u32,
    idle_ticks: u32,
    rx: Vec<u8>,
    progress: u64,
    lost_frames: Vec<u64>,
    zero_length_packets: bool,
    frames: Vec<(u64, Vec<u16>)>,
}

impl Host {
    fn new() -> Self {
        Self {
            send_seq: 16,
            receive_seq: 16,
            sent: VecDeque::new(),
            ignore_nak_seq: 0,
            retransmit_seq: 0,
            retransmits: 0,
            bytes_retransmit: 0,
            next_cmd_arg: 0,
            idle_ticks: 0,
            rx: Vec::new(),
            progress: 0,
            lost_frames: Vec::new(),
            zero_length_packets: false,
            frames: Vec::new(),
        }
    }
    fn write(&self, dev: &mut Dev, rng: &mut Rng, buf: &[u8], drop_p: f64) {
        // cdc_acm: one write -> one URB -> max-size packets. A lost packet
        // fails the whole URB; the remainder of that write is discarded.
        for chunk in buf.chunks(PKT) {
            if rng.chance(drop_p) {
                return;
            }
            dev.inflight.push_back(chunk.to_vec());
        }
        // Hosts with URB_ZERO_PACKET semantics terminate a transfer that is a
        // multiple of the packet size with a zero-length packet.
        if self.zero_length_packets && buf.len().is_multiple_of(PKT) {
            dev.inflight.push_back(Vec::new());
        }
    }
    fn send_new(&mut self, dev: &mut Dev, rng: &mut Rng, drop_p: f64) {
        if self.sent.len() >= MAX_PENDING_BLOCKS {
            return;
        }
        let ncmd = 1 + rng.below(8) as usize; // 1..8 commands of 5 bytes => 10..45 byte frames
        let mut cmds = Vec::new();
        for _ in 0..ncmd {
            let id = (self.next_cmd_arg % 50) as u16 + 1;
            cmds.push((id, self.next_cmd_arg));
            self.next_cmd_arg += 1;
        }
        self.frames
            .push((self.send_seq, cmds.iter().map(|c| c.0).collect()));
        let mut f = frame(self.send_seq as u8, &cmds);
        if self.zero_length_packets && rng.chance(0.3) {
            f = pad_frame_to(f, PKT);
        }
        self.sent.push_back((self.send_seq, f.clone()));
        self.send_seq += 1;
        self.write(dev, rng, &f, drop_p);
    }
    fn retransmit(&mut self, dev: &mut Dev, rng: &mut Rng, drop_p: f64, nak: bool) {
        let mut buf = vec![SYNC];
        for (_, f) in &self.sent {
            buf.extend_from_slice(f);
        }
        self.write(dev, rng, &buf, drop_p);
        self.bytes_retransmit += buf.len();
        self.retransmits += 1;
        if nak {
            self.ignore_nak_seq = self.receive_seq;
            if self.receive_seq < self.retransmit_seq {
                self.ignore_nak_seq = self.retransmit_seq;
            }
        }
        self.retransmit_seq = self.send_seq;
    }
    /// Parse device->host stream; returns true if a NAK asks for fast retransmit.
    fn process_rx(&mut self) -> bool {
        let mut want_retransmit = false;
        loop {
            while !self.rx.is_empty() && self.rx[0] == SYNC {
                self.rx.remove(0);
            }
            if self.rx.len() < 5 {
                break;
            }
            let len = self.rx[0] as usize;
            if !(5..=64).contains(&len) {
                self.rx.remove(0);
                continue;
            }
            if self.rx.len() < len {
                break;
            }
            let f: Vec<u8> = self.rx.drain(..len).collect();
            assert_eq!(f[len - 1], SYNC, "device frame not sync-terminated");
            let c = ((f[len - 3] as u16) << 8) | f[len - 2] as u16;
            assert_eq!(c, crc16(&f[..len - 3]), "device frame crc");
            let rseq_delta = ((f[1] as u64).wrapping_sub(self.receive_seq)) & SEQ_MASK as u64;
            let rseq = self.receive_seq + rseq_delta;
            if rseq != self.receive_seq {
                if rseq > self.send_seq && self.receive_seq != 1 {
                    continue;
                } // bytes_invalid
                // update_receive_seq (serialqueue.c): pop sent messages until the
                // one matching rseq; an ack for a message never sent re-syncs send_seq.
                let mut sent_seq = self.receive_seq;
                let mut popped = Vec::new();
                loop {
                    if self.sent.is_empty() {
                        // Connection-init quirk: these frames are silently lost.
                        self.send_seq = rseq;
                        for sq in popped.drain(..) {
                            self.lost_frames.push(sq);
                        }
                        break;
                    }
                    let (sq, _) = self.sent.pop_front().unwrap();
                    popped.push(sq);
                    sent_seq += 1;
                    if rseq == sent_seq {
                        break;
                    }
                }
                self.receive_seq = rseq;
                self.progress += 1;
                self.idle_ticks = 0;
            } else if len == 5 {
                // Duplicate ack is a nak
                if rseq > self.ignore_nak_seq && !self.sent.is_empty() {
                    want_retransmit = true;
                }
            }
        }
        want_retransmit
    }
}

struct Outcome {
    wedged: bool,
    detail: String,
}

fn run(
    seed: u64,
    drop_p: f64,
    host_latency: u32,
    report_p: f64,
    zlp: bool,
    iterations: u32,
) -> Outcome {
    DEV_TX.with(|t| t.borrow_mut().clear());
    TRACE.with(|t| t.borrow_mut().clear());
    DISPATCHED.with(|d| d.borrow_mut().clear());
    USB_WAKE.with(|w| w.set(false));
    let dev = RefCell::new(Dev::new());
    let mut host = Host::new();
    host.zero_length_packets = zlp;
    let mut rng = Rng(seed | 1);
    let counters = UsbPumpCounters::new();
    let transport: Transport<Cfg> = Transport::new(&CFG, Out);
    let mut backend = Backend {
        dev: &dev,
        host_steps_during_yield: 1,
        transport: &transport,
    };
    let mut wedge_ticks = 0u32;
    let mut last_progress = 0u64;

    for tick in 0..iterations {
        // Host side: generate traffic, deliver, let the ISR fire.
        if rng.chance(0.8) {
            host.send_new(&mut dev.borrow_mut(), &mut rng, drop_p);
        }
        {
            let mut d = dev.borrow_mut();
            if host_latency == 0 || tick % host_latency == 0 {
                d.deliver();
            }
            d.report_pending = rng.chance(report_p);
            d.isr();
        }
        // Device side: run the pump while there is a wake.
        let mut guard = 0;
        while USB_WAKE.with(|w| w.get()) {
            block_on(run_once(&mut backend, &counters));
            {
                let mut d = dev.borrow_mut();
                d.deliver();
                d.isr();
            }
            guard += 1;
            if guard > 10_000 {
                return Outcome {
                    wedged: true,
                    detail: "device livelock".into(),
                };
            }
        }
        // Host reads device output.
        DEV_TX.with(|t| host.rx.extend(t.borrow_mut().drain(..)));
        if host.process_rx() {
            host.retransmit(&mut dev.borrow_mut(), &mut rng, drop_p, true);
        }
        // RTO: no progress for a while -> retransmit.
        if !host.sent.is_empty() {
            host.idle_ticks += 1;
        } else {
            host.idle_ticks = 0;
        }
        if host.idle_ticks > 20 {
            host.retransmit(&mut dev.borrow_mut(), &mut rng, drop_p, false);
            host.idle_ticks = 0;
        }
        // Wedge detection: the host has unacked frames and keeps retransmitting
        // but the ack sequence makes no progress for a long time.
        let d = dev.borrow();
        if host.progress == last_progress && !host.sent.is_empty() {
            wedge_ticks += 1;
        } else {
            wedge_ticks = 0;
        }
        last_progress = host.progress;
        if wedge_ticks > 400 {
            let detail = format!(
                "seed={seed} tick={tick} rpos={} fifo_len={} rxflvlm={} ep_enabled={} inflight={} bulk_out_wake={} host.sent={} retransmits={}",
                d.rpos,
                d.fifo.len(),
                d.rxflvlm,
                d.ep_enabled,
                d.inflight.len(),
                d.bulk_out_wake,
                host.sent.len(),
                host.retransmits
            );
            let mut trace = String::new();
            TRACE.with(|t| {
                for l in t.borrow().iter() {
                    trace.push_str("  ");
                    trace.push_str(l);
                    trace.push('\n');
                }
            });
            return Outcome {
                wedged: true,
                detail: format!("{detail}\n{trace}  rx[..rpos]={:02x?}", &d.rx[..d.rpos]),
            };
        }
    }
    // Verify that the dispatched command stream is a prefix-consistent, loss-free, duplicate-free sequence.
    let got = DISPATCHED.with(|d| d.borrow().clone());
    let expected: Vec<u16> = host
        .frames
        .iter()
        .filter(|(s, _)| !host.lost_frames.contains(s))
        .flat_map(|(_, c)| c.clone())
        .collect();
    let exp = &expected[..got.len().min(expected.len())];
    if got.as_slice() != exp {
        let first_bad = got
            .iter()
            .zip(exp.iter())
            .position(|(a, b)| a != b)
            .unwrap_or(got.len().min(exp.len()));
        return Outcome {
            wedged: true,
            detail: format!(
                "seed={seed} dispatch stream diverged at {first_bad}: got {:?} expected {:?}",
                &got[first_bad.saturating_sub(3)..(first_bad + 3).min(got.len())],
                &exp[first_bad.saturating_sub(3)..(first_bad + 3).min(exp.len())]
            ),
        };
    }
    Outcome {
        wedged: false,
        detail: format!(
            "seed={seed} ok: dispatched={} retransmits={} bytes_retransmit={} deliveries={}",
            got.len(),
            host.retransmits,
            host.bytes_retransmit,
            dev.borrow().deliveries
        ),
    }
}

fn sweep(drop_p: f64, latency: u32, report_p: f64, seeds: u64) {
    sweep_with(drop_p, latency, report_p, false, seeds);
}

fn sweep_with(drop_p: f64, latency: u32, report_p: f64, zlp: bool, seeds: u64) {
    for seed in 1..=seeds {
        let o = run(seed * 7919, drop_p, latency, report_p, zlp, 3000);
        assert!(
            !o.wedged,
            "drop={drop_p} latency={latency} report_p={report_p}: {}",
            o.detail
        );
    }
}

#[test]
fn lossless_link_streams_without_loss_or_duplication() {
    sweep(0.0, 0, 0.0, 40);
}

#[test]
fn retransmit_bursts_with_lost_packets_never_park_the_pump() {
    // The reported field failure: a poor cable, Klippy retransmits, the next
    // packet lands while the task is still draining the previous one.
    sweep(0.2, 3, 0.0, 60);
}

#[test]
fn heavy_loss_with_report_preemption_still_makes_progress() {
    sweep(0.4, 2, 0.3, 40);
}

#[test]
fn zero_length_packets_do_not_strand_the_following_frame() {
    // A host that terminates 64-byte-multiple writes with a zero-length packet
    // (URB_ZERO_PACKET semantics): the empty read must still lead to a poll
    // that re-arms RXFLVL, or the next frame sits in the FIFO unread.
    sweep_with(0.0, 3, 0.0, true, 40);
    sweep_with(0.2, 3, 0.0, true, 40);
}
