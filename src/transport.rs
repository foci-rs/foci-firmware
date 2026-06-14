//! Shared Ankyra USB transport adapter.

use core::marker::PhantomData;

use ankyra::{ScratchOutput, TransportOutput};

/// Board-local USB transmit sink used by [`AnkyraUsbTransport`].
pub trait UsbTx {
    /// Write encoded protocol bytes to the USB TX buffer.
    fn write(data: &[u8]);
}

/// Board-local wake signal used by [`AnkyraUsbTransport`].
pub trait UsbWake {
    /// Wake the USB task so it can drain pending TX bytes.
    fn wake();
}

/// An Ankyra [`TransportOutput`] backed by board-provided USB TX and wake hooks.
pub struct AnkyraUsbTransport<Tx, Wake> {
    _marker: PhantomData<fn() -> (Tx, Wake)>,
}

impl<Tx, Wake> AnkyraUsbTransport<Tx, Wake> {
    /// Create a transport adapter.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl<Tx, Wake> Default for AnkyraUsbTransport<Tx, Wake> {
    fn default() -> Self {
        Self::new()
    }
}

impl<Tx, Wake> TransportOutput for AnkyraUsbTransport<Tx, Wake>
where
    Tx: UsbTx,
    Wake: UsbWake,
{
    type Output = ScratchOutput;

    fn output(&self, f: impl FnOnce(&mut Self::Output)) {
        let mut scratch = ScratchOutput::new();
        f(&mut scratch);
        Tx::write(scratch.result());
        Wake::wake();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use ankyra::OutputBuffer;
    use std::sync::{Mutex, MutexGuard, OnceLock};
    use std::vec::Vec;

    static TEST_LOCK: Mutex<()> = Mutex::new(());
    static WRITES: OnceLock<Mutex<Vec<Vec<u8>>>> = OnceLock::new();
    static WAKES: OnceLock<Mutex<u32>> = OnceLock::new();

    struct RecordingTx;

    impl UsbTx for RecordingTx {
        fn write(data: &[u8]) {
            writes()
                .lock()
                .expect("writes mutex is available")
                .push(data.to_vec());
        }
    }

    struct RecordingWake;

    impl UsbWake for RecordingWake {
        fn wake() {
            let mut wakes = wakes().lock().expect("wakes mutex is available");
            *wakes += 1;
        }
    }

    fn writes() -> &'static Mutex<Vec<Vec<u8>>> {
        WRITES.get_or_init(|| Mutex::new(Vec::new()))
    }

    fn wakes() -> &'static Mutex<u32> {
        WAKES.get_or_init(|| Mutex::new(0))
    }

    fn reset() {
        writes().lock().expect("writes mutex is available").clear();
        *wakes().lock().expect("wakes mutex is available") = 0;
    }

    fn capture_writes() -> MutexGuard<'static, Vec<Vec<u8>>> {
        writes().lock().expect("writes mutex is available")
    }

    #[test]
    fn transport_writes_encoded_bytes_and_wakes_usb_task() {
        let _guard = TEST_LOCK.lock().expect("transport test lock is available");
        reset();

        let transport = AnkyraUsbTransport::<RecordingTx, RecordingWake>::new();
        transport.output(|output| output.output(&[0x10, 0x20, 0x30]));

        let wake_count = *wakes().lock().expect("wakes mutex is available");
        assert_eq!(wake_count, 1);
        let writes = capture_writes();
        assert_eq!(writes.len(), 1);
        assert_eq!(writes[0].as_slice(), &[0x10, 0x20, 0x30]);
    }

    #[test]
    fn default_transport_has_same_behavior() {
        let _guard = TEST_LOCK.lock().expect("transport test lock is available");
        reset();

        let transport = AnkyraUsbTransport::<RecordingTx, RecordingWake>::default();
        transport.output(|output| output.output(&[0xAB]));

        let wake_count = *wakes().lock().expect("wakes mutex is available");
        assert_eq!(wake_count, 1);
        let writes = capture_writes();
        assert_eq!(writes.len(), 1);
        assert_eq!(writes[0].as_slice(), &[0xAB]);
    }
}
