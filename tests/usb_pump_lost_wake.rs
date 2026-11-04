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
        if frame.len() < 2 {
            return Err(ReadError);
        }
        if cmd == PAD_CMD {
            let k = frame[0] as usize;
            if frame.len() < 1 + k {
                return Err(ReadError);
            }
            *frame = &frame[1 + k..];
            return Ok(());
        }
        DISPATCHED.with(|d| d.borrow_mut().push(cmd));
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

const PAD_CMD: u16 = 63;

fn pad_to_packet(f: &[u8]) -> Vec<u8> {
    assert!(
        f.len() + 2 <= PKT,
        "a {}-byte frame leaves no room for the padding command",
        f.len()
    );
    let needed = PKT - f.len();
    let payload = &f[2..f.len() - 3];
    let seq = f[1];
    let mut v = vec![0u8, seq];
    v.extend_from_slice(payload);
    v.push(PAD_CMD as u8);
    v.push((needed - 2) as u8);
    v.extend(std::iter::repeat_n(0u8, needed - 2));
    v[0] = (v.len() + 3) as u8;
    let c = crc16(&v);
    v.push((c >> 8) as u8);
    v.push(c as u8);
    v.push(SYNC);
    debug_assert_eq!(v.len(), PKT);
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
    zlps_delivered: u32,
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
            zlps_delivered: 0,
            driver_wake: false,
        }
    }
    fn deliver(&mut self) {
        if self.ep_enabled
            && let Some(p) = self.inflight.pop_front()
        {
            if p.is_empty() {
                self.zlps_delivered += 1;
            }
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
    fn usb_read_bulk_out(&mut self) -> Option<Vec<u8>> {
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
        self.isr();
        Some(pkt)
    }
    fn bulk_out_task(&mut self) -> bool {
        let woke = std::mem::replace(&mut self.bulk_out_wake, false);
        if !woke {
            return self.rpos > 0;
        }
        if self.rpos + PKT <= RX_BUF {
            if let Some(p) = self.usb_read_bulk_out() {
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
    fn rx_consume(&mut self, len: usize) {
        if len >= self.rpos {
            self.rpos = 0;
            return;
        }
        let need = self.rpos - len;
        self.rx.copy_within(len..self.rpos, 0);
        self.rpos = need;
        if len > 0 && need > 0 {
            self.bulk_out_wake = true;
            self.driver_wake = true;
        }
    }
}

struct Backend<'a> {
    dev: Dev,
    host_steps_during_yield: u32,
    transport: &'a Transport<Cfg>,
}

impl UsbPumpBackend for Backend<'_> {
    async fn wait_wake(&mut self) {
        tr("run_once: wake consumed".into());
        USB_WAKE.with(|w| w.set(false));
    }
    fn poll(&mut self) -> bool {
        self.dev.bulk_out_task()
    }
    fn rx_data(&self) -> &[u8] {
        &self.dev.rx[..self.dev.rpos]
    }
    fn rx_consume(&mut self, n: usize) {
        self.dev.rx_consume(n);
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
        self.dev.report_pending
    }
    fn wake(&mut self) {
        USB_WAKE.with(|w| w.set(true));
    }
    async fn yield_now(&mut self) {
        for _ in 0..self.host_steps_during_yield {
            self.dev.deliver();
            self.dev.isr();
        }
    }
    fn final_poll(&mut self) {
        tr("final_poll".into());
        self.dev.bulk_out_task();
        tr(format!(
            "run_once ends: rpos={} usb_wake={}",
            self.dev.rpos,
            USB_WAKE.with(|w| w.get())
        ));
    }
    fn driver_wake_pending(&mut self) -> bool {
        std::mem::replace(&mut self.dev.driver_wake, false)
    }
    fn on_wake(&mut self) {}
    fn on_drain_complete(&mut self) {}
}

struct Host {
    send_seq: u64,
    receive_seq: u64,
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
    padded_frames: u32,
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
            padded_frames: 0,
            frames: Vec::new(),
        }
    }
    fn write(&self, dev: &mut Dev, rng: &mut Rng, buf: &[u8], drop_p: f64) {
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
        let ncmd = 1 + rng.below(8) as usize;
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
            f = pad_to_packet(&f);
            self.padded_frames += 1;
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
                }
                let mut sent_seq = self.receive_seq;
                let mut popped = Vec::new();
                loop {
                    if self.sent.is_empty() {
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
    padded_frames: u32,
    zlps_delivered: u32,
    dispatched: usize,
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
    let mut host = Host::new();
    host.zero_length_packets = zlp;
    let mut rng = Rng(seed | 1);
    let counters = UsbPumpCounters::new();
    let transport: Transport<Cfg> = Transport::new(&CFG, Out);
    let mut backend = Backend {
        dev: Dev::new(),
        host_steps_during_yield: 1,
        transport: &transport,
    };
    let mut wedge_ticks = 0u32;
    let mut last_progress = 0u64;

    for tick in 0..iterations {
        if rng.chance(0.8) {
            host.send_new(&mut backend.dev, &mut rng, drop_p);
        }
        let d = &mut backend.dev;
        if host_latency == 0 || tick % host_latency == 0 {
            d.deliver();
        }
        d.report_pending = rng.chance(report_p);
        d.isr();
        let mut guard = 0;
        while USB_WAKE.with(|w| w.get()) {
            pollster::block_on(run_once(&mut backend, &counters));
            backend.dev.deliver();
            backend.dev.isr();
            guard += 1;
            if guard > 10_000 {
                return Outcome {
                    wedged: true,
                    detail: "device livelock".into(),
                    padded_frames: 0,
                    zlps_delivered: 0,
                    dispatched: 0,
                };
            }
        }
        DEV_TX.with(|t| host.rx.extend(t.borrow_mut().drain(..)));
        if host.process_rx() {
            host.retransmit(&mut backend.dev, &mut rng, drop_p, true);
        }
        if !host.sent.is_empty() {
            host.idle_ticks += 1;
        } else {
            host.idle_ticks = 0;
        }
        if host.idle_ticks > 20 {
            host.retransmit(&mut backend.dev, &mut rng, drop_p, false);
            host.idle_ticks = 0;
        }
        let d = &backend.dev;
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
                padded_frames: 0,
                zlps_delivered: 0,
                dispatched: 0,
            };
        }
    }
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
            padded_frames: 0,
            zlps_delivered: 0,
            dispatched: 0,
        };
    }
    Outcome {
        wedged: false,
        detail: format!(
            "seed={seed} ok: dispatched={} retransmits={} bytes_retransmit={} deliveries={}",
            got.len(),
            host.retransmits,
            host.bytes_retransmit,
            backend.dev.deliveries
        ),
        padded_frames: host.padded_frames,
        zlps_delivered: backend.dev.zlps_delivered,
        dispatched: got.len(),
    }
}

fn sweep(drop_p: f64, latency: u32, report_p: f64, seeds: u64) {
    sweep_with(drop_p, latency, report_p, false, seeds);
}

fn sweep_with(
    drop_p: f64,
    latency: u32,
    report_p: f64,
    zlp: bool,
    seeds: u64,
) -> (u32, u32, usize) {
    let mut totals = (0u32, 0u32, 0usize);
    for seed in 1..=seeds {
        let o = run(seed * 7919, drop_p, latency, report_p, zlp, 3000);
        assert!(
            !o.wedged,
            "drop={drop_p} latency={latency} report_p={report_p} zlp={zlp}: {}",
            o.detail
        );
        totals.0 += o.padded_frames;
        totals.1 += o.zlps_delivered;
        totals.2 += o.dispatched;
    }
    totals
}

#[test]
fn lossless_link_streams_without_loss_or_duplication() {
    sweep(0.0, 0, 0.0, 40);
}

#[test]
fn retransmit_bursts_with_lost_packets_never_park_the_pump() {
    sweep(0.2, 3, 0.0, 60);
}

#[test]
fn heavy_loss_with_report_preemption_still_makes_progress() {
    sweep(0.4, 2, 0.3, 40);
}

#[test]
fn zero_length_packets_do_not_strand_the_following_frame() {
    let (padded, zlps, dispatched) = sweep_with(0.0, 3, 0.0, true, 40);
    assert!(padded > 0, "no frame was padded to a full packet");
    assert!(zlps > 0, "no zero-length packet was delivered");
    assert!(dispatched > 0);
    sweep_with(0.2, 3, 0.0, true, 40);
}

#[test]
fn a_zero_length_packet_followed_by_a_frame_is_dispatched() {
    DEV_TX.with(|t| t.borrow_mut().clear());
    DISPATCHED.with(|d| d.borrow_mut().clear());
    USB_WAKE.with(|w| w.set(false));
    let transport: Transport<Cfg> = Transport::new(&CFG, Out);
    let mut backend = Backend {
        dev: Dev::new(),
        host_steps_during_yield: 1,
        transport: &transport,
    };
    let counters = UsbPumpCounters::new();
    backend.dev.inflight.push_back(Vec::new());
    backend.dev.inflight.push_back(frame(0, &[(7, 1), (8, 2)]));
    backend.dev.deliver();
    backend.dev.isr();
    let mut guard = 0;
    while USB_WAKE.with(|w| w.get()) {
        pollster::block_on(run_once(&mut backend, &counters));
        backend.dev.deliver();
        backend.dev.isr();
        guard += 1;
        assert!(guard < 100, "pump did not settle");
    }
    assert_eq!(backend.dev.zlps_delivered, 1);
    assert_eq!(DISPATCHED.with(|d| d.borrow().clone()), vec![7, 8]);
    assert!(
        backend.dev.rxflvlm,
        "RXFLVL must be re-armed before the task sleeps"
    );
}
