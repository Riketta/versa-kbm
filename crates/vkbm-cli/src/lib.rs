//! Host-side client library for the versa-kbm vendor control channel.
//!
//! The wire contract lives in `vkbm_proto`; this crate adds the transport
//! (HID output/input reports through `hidapi`) and the ack policy: one
//! round-trip, one resend on a lost ack, stale replies discarded.

use core::time::Duration;

use hidapi::{
    HidApi,
    HidDevice,
    HidError,
};
use vkbm_proto::{
    Command,
    Reply,
    frame::{
        self,
        FRAME_LEN,
    },
};

/// Default USB identity to look for (must match the firmware's constants).
pub const DEFAULT_VID: u16 = 0x046d;
/// Default USB identity to look for (must match the firmware's constants).
pub const DEFAULT_PID: u16 = 0xc52b;
/// Vendor control channel usage page, from the firmware report descriptor.
pub const VENDOR_USAGE_PAGE: u16 = 0xff00;
/// Vendor control channel usage, from the firmware report descriptor.
pub const VENDOR_USAGE: u16 = 0x01;

/// How long one ack may take before the attempt counts as lost.
pub const ACK_TIMEOUT: Duration = Duration::from_millis(250);

/// Client failures.
#[derive(Debug)]
pub enum Error {
    /// Device lookup or open failed.
    Open(HidError),
    /// Report write failed.
    Write(HidError),
    /// Report read failed.
    Read(HidError),
    /// The OS write accepted fewer bytes than the full report.
    ShortWrite {
        /// Bytes the OS accepted.
        wrote: usize,
        /// Bytes of the full report.
        want: usize,
    },
    /// No ack within the timeout, even after the single resend.
    Timeout,
    /// A reply arrived but could not be decoded.
    Decode(frame::FrameError),
    /// No device with the vendor control channel was found.
    NotFound,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Open(e) => write!(f, "device open failed: {e}"),
            Self::Write(e) => write!(f, "report write failed: {e}"),
            Self::Read(e) => write!(f, "report read failed: {e}"),
            Self::ShortWrite { wrote, want } => {
                write!(f, "short write: {wrote} of {want} bytes accepted")
            }
            Self::Timeout => write!(f, "no ack within the timeout"),
            Self::Decode(e) => write!(f, "undecodable reply: {e}"),
            Self::NotFound => write!(f, "no versa-kbm vendor control channel found"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Open(e) | Self::Write(e) | Self::Read(e) => Some(e),
            Self::ShortWrite { .. } | Self::Timeout | Self::Decode(_) | Self::NotFound => None,
        }
    }
}

/// Finds the vendor control channel interface among HID devices.
#[must_use]
pub fn find_device(api: &HidApi, vid: u16, pid: u16) -> Option<hidapi::DeviceInfo> {
    api.device_list()
        .find(|d| {
            d.vendor_id() == vid
                && d.product_id() == pid
                && d.usage_page() == VENDOR_USAGE_PAGE
                && d.usage() == VENDOR_USAGE
        })
        .cloned()
}

/// One host-to-device report write plus one device-to-host report read.
/// Abstracted so the ack/retry logic is testable without hardware.
trait Link {
    /// Writes one 64-byte frame.
    fn send(&mut self, report: &[u8; FRAME_LEN]) -> Result<(), Error>;
    /// Reads one 64-byte report, or [`Error::Timeout`].
    fn recv(&mut self, timeout: Duration) -> Result<[u8; FRAME_LEN], Error>;
}

struct HidLink {
    device: HidDevice,
}

impl Link for HidLink {
    fn send(&mut self, report: &[u8; FRAME_LEN]) -> Result<(), Error> {
        // Report-id prefix: the device uses unnumbered reports, so the id
        // byte is 0.
        let mut out = [0u8; FRAME_LEN + 1];
        let (_, dst) = out.split_at_mut(1);
        dst.copy_from_slice(report);
        let wrote = self.device.write(&out).map_err(Error::Write)?;
        if wrote != out.len() {
            return Err(Error::ShortWrite { wrote, want: out.len() });
        }
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> Result<[u8; FRAME_LEN], Error> {
        let mut buf = [0u8; FRAME_LEN];
        let millis = i32::try_from(timeout.as_millis()).unwrap_or(i32::MAX);
        match self.device.read_timeout(&mut buf, millis) {
            Ok(n) if n > 0 => Ok(buf),
            Ok(_) => Err(Error::Timeout),
            Err(e) => Err(Error::Read(e)),
        }
    }
}

/// Request client over the vendor control channel.
pub struct Client {
    link: Box<dyn Link>,
    next_seq: u8,
}

impl Client {
    /// Opens the vendor control channel of the first matching device.
    ///
    /// # Errors
    /// [`Error::NotFound`] when no device matches, [`Error::Open`] when the
    /// OS refuses the handle.
    pub fn open(api: &HidApi, vid: u16, pid: u16) -> Result<Self, Error> {
        let info = find_device(api, vid, pid).ok_or(Error::NotFound)?;
        let device = api.open_path(info.path()).map_err(Error::Open)?;
        Ok(Self { link: Box::new(HidLink { device }), next_seq: 1 })
    }

    /// Sends one command and waits for its ack; a lost ack is resent once.
    ///
    /// # Errors
    /// [`Error::Timeout`] after send plus one resend, transport errors from
    /// [`Error`], decode errors for malformed replies.
    pub fn request(&mut self, cmd: &Command, timeout: Duration) -> Result<Reply, Error> {
        let seq = self.next_seq;
        self.next_seq = if seq == u8::MAX { 1 } else { seq + 1 };
        let mut raw = [0u8; FRAME_LEN];
        frame::encode(cmd, seq, &mut raw);
        issue(&mut *self.link, &raw, seq, timeout)
    }
}

/// Sends one command frame with the ack policy: a lost ack gets exactly one
/// resend before the command is given up on.
fn issue(
    link: &mut dyn Link,
    raw: &[u8; FRAME_LEN],
    seq: u8,
    timeout: Duration,
) -> Result<Reply, Error> {
    for attempt in 0..2 {
        link.send(raw)?;
        match wait_ack(link, seq, timeout) {
            Ok(reply) => return Ok(reply),
            Err(Error::Timeout) if attempt == 0 => {}
            Err(e) => return Err(e),
        }
    }
    Err(Error::Timeout)
}

fn wait_ack(link: &mut dyn Link, seq: u8, timeout: Duration) -> Result<Reply, Error> {
    loop {
        let raw = link.recv(timeout)?;
        let (got, reply) = frame::decode_reply(&raw).map_err(Error::Decode)?;
        if got == seq {
            return Ok(reply);
        }
        // A reply for an earlier (timed-out) request: discard it.
    }
}

#[cfg(test)]
mod tests {
    use core::{
        cell::RefCell,
        time::Duration,
    };
    use std::rc::Rc;

    use vkbm_proto::{
        Command,
        Reply,
        frame::{
            self,
            FRAME_LEN,
        },
    };

    use super::{
        Error,
        Link,
        issue,
    };

    const TIMEOUT: Duration = Duration::from_millis(250);

    type Replies = Rc<RefCell<std::collections::VecDeque<Result<[u8; FRAME_LEN], Error>>>>;

    #[derive(Default, Clone)]
    struct FakeLink {
        sent: Rc<RefCell<Vec<[u8; FRAME_LEN]>>>,
        replies: Replies,
    }

    impl FakeLink {
        fn sent(&self) -> usize {
            self.sent.borrow().len()
        }

        fn sent_seq(&self, attempt: usize) -> Option<u8> {
            self.sent.borrow().get(attempt).map(|frame| frame[0])
        }
    }

    impl Link for FakeLink {
        fn send(&mut self, report: &[u8; FRAME_LEN]) -> Result<(), Error> {
            self.sent.borrow_mut().push(*report);
            Ok(())
        }

        fn recv(&mut self, _timeout: Duration) -> Result<[u8; FRAME_LEN], Error> {
            self.replies.borrow_mut().pop_front().unwrap_or(Err(Error::Timeout))
        }
    }

    fn ack_for(seq: u8) -> [u8; FRAME_LEN] {
        let mut buf = [0u8; FRAME_LEN];
        frame::encode_reply(seq, &Reply::Ok, &mut buf);
        buf
    }

    fn frame_for(seq: u8) -> [u8; FRAME_LEN] {
        let mut buf = [0u8; FRAME_LEN];
        frame::encode(&Command::Ping, seq, &mut buf);
        buf
    }

    #[test]
    fn ack_on_first_attempt() {
        let mut link = FakeLink {
            replies: Rc::new(RefCell::new(std::collections::VecDeque::from([Ok(ack_for(1))]))),
            ..FakeLink::default()
        };
        let reply = issue(&mut link, &frame_for(1), 1, TIMEOUT);
        assert!(matches!(reply, Ok(Reply::Ok)));
        assert_eq!(link.sent(), 1, "no resend on a clean ack");
    }

    #[test]
    fn lost_ack_is_resent_once() {
        let mut link = FakeLink {
            replies: Rc::new(RefCell::new(std::collections::VecDeque::from([
                Err(Error::Timeout),
                Ok(ack_for(1)),
            ]))),
            ..FakeLink::default()
        };
        let reply = issue(&mut link, &frame_for(1), 1, TIMEOUT);
        assert!(matches!(reply, Ok(Reply::Ok)));
        assert_eq!(link.sent(), 2, "exactly one resend");
    }

    #[test]
    fn double_timeout_gives_up() {
        let mut link = FakeLink {
            replies: Rc::new(RefCell::new(std::collections::VecDeque::from([
                Err(Error::Timeout),
                Err(Error::Timeout),
            ]))),
            ..FakeLink::default()
        };
        let reply = issue(&mut link, &frame_for(1), 1, TIMEOUT);
        assert!(matches!(reply, Err(Error::Timeout)));
        assert_eq!(link.sent(), 2);
    }

    #[test]
    fn stale_replies_are_discarded() {
        let mut link = FakeLink {
            replies: Rc::new(RefCell::new(std::collections::VecDeque::from([
                Ok(ack_for(9)),
                Ok(ack_for(1)),
            ]))),
            ..FakeLink::default()
        };
        let reply = issue(&mut link, &frame_for(1), 1, TIMEOUT);
        assert!(matches!(reply, Ok(Reply::Ok)));
        assert_eq!(link.sent(), 1, "stale ack must not consume the resend");
    }

    #[test]
    fn client_advances_sequence_numbers() {
        let fake = FakeLink {
            replies: Rc::new(RefCell::new(std::collections::VecDeque::from([
                Ok(ack_for(1)),
                Ok(ack_for(2)),
            ]))),
            ..FakeLink::default()
        };
        let mut client = super::Client { link: Box::new(fake.clone()), next_seq: 1 };
        let _ = client.request(&Command::Ping, TIMEOUT);
        let _ = client.request(&Command::Info, TIMEOUT);
        assert_eq!(fake.sent_seq(0), Some(1));
        assert_eq!(fake.sent_seq(1), Some(2));
    }
}
