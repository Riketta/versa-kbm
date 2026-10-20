//! Host-side client library for the versa-kbm vendor control channel.
//!
//! The wire contract lives in `vkbm_proto`; this crate adds the transport
//! (HID output/input reports through `hidapi`) and the ack policy: one
//! round-trip, one resend on a lost ack, stale replies discarded.

use core::time::Duration;
use std::time::{
    SystemTime,
    UNIX_EPOCH,
};

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
pub const DEFAULT_VID: u16 = 0x258a;
/// Default USB identity to look for (must match the firmware's constants).
pub const DEFAULT_PID: u16 = 0x0100;
/// Vendor control channel usage page, from the firmware report descriptor.
pub const VENDOR_USAGE_PAGE: u16 = 0xff00;
/// Vendor control channel usage, from the firmware report descriptor.
pub const VENDOR_USAGE: u16 = 0x01;

/// Default ack timeout, in milliseconds (the CLI's `--timeout-ms` default).
pub const ACK_TIMEOUT_MS: u64 = 250;

/// How long one ack may take before the attempt counts as lost.
pub const ACK_TIMEOUT: Duration = Duration::from_millis(ACK_TIMEOUT_MS);

/// Deadline for [`Client::open`]'s sacrificial ping. A present device acks
/// in about a millisecond; only a silent device pays the full wait.
const OPEN_BURN_TIMEOUT: Duration = Duration::from_millis(50);

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
    /// No ack within the timeout (or the reply stream stayed junk), even
    /// after the single resend.
    Timeout,
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
            Self::NotFound => write!(f, "no versa-kbm vendor control channel found"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Open(e) | Self::Write(e) | Self::Read(e) => Some(e),
            Self::ShortWrite { .. } | Self::Timeout | Self::NotFound => None,
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
    /// Reads one 64-byte report, or [`Error::Timeout`]. `Ok(None)` marks a
    /// short read: a partial report must count as junk downstream, never
    /// decode as a plausible reply with a zero-filled tail.
    fn recv(&mut self, timeout: Duration) -> Result<Option<[u8; FRAME_LEN]>, Error>;
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
        // Windows counts the report-id prefix byte (`FRAME_LEN + 1`); Linux
        // hidraw strips it and reports the payload length (`FRAME_LEN`)
        // only. Both mean the whole frame was handed to the kernel.
        if wrote != FRAME_LEN && wrote != out.len() {
            return Err(Error::ShortWrite { wrote, want: out.len() });
        }
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> Result<Option<[u8; FRAME_LEN]>, Error> {
        let mut buf = [0u8; FRAME_LEN];
        let millis = i32::try_from(timeout.as_millis()).unwrap_or(i32::MAX);
        match self.device.read_timeout(&mut buf, millis) {
            Ok(FRAME_LEN) => Ok(Some(buf)),
            Ok(0) => Err(Error::Timeout),
            // A short report is junk, not a decodable reply.
            Ok(_) => Ok(None),
            Err(e) => Err(Error::Read(e)),
        }
    }
}

/// Seeds the per-process sequence counter. The device dedups by sequence
/// number, so a fresh CLI process must not start at the number the previous
/// session last used - the device would replay that session's cached reply
/// and the command would be swallowed. Clock subsecond nanoseconds and the
/// OS process id, mixed byte by byte, spread the starting points; zero maps
/// to 1 because the host never sends 0. [`Client::open`]'s sacrificial
/// ping absorbs the residual collision risk.
fn initial_seq() -> u8 {
    let nanos = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.subsec_nanos());
    let pid = std::process::id();
    #[allow(clippy::cast_possible_truncation)] // a byte-wide mix is the point
    {
        let mixed = (nanos as u8)
            ^ ((nanos >> 8) as u8)
            ^ (pid as u8)
            ^ ((pid >> 8) as u8)
            ^ ((pid >> 16) as u8);
        if mixed == 0 { 1 } else { mixed }
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
    /// Sends one sacrificial [`Command::Ping`] before returning: the device
    /// replays a repeated sequence number for a short window after its
    /// previous session, so a rapid host restart could collide with the
    /// cached reply and silently swallow this session's first real command.
    /// The throwaway ping burns that replay slot; its outcome is irrelevant
    /// (a silent device fails again on the first real command either way),
    /// so the result is discarded.
    ///
    /// # Errors
    /// [`Error::NotFound`] when no device matches, [`Error::Open`] when the
    /// OS refuses the handle.
    pub fn open(api: &HidApi, vid: u16, pid: u16) -> Result<Self, Error> {
        let info = find_device(api, vid, pid).ok_or(Error::NotFound)?;
        let device = api.open_path(info.path()).map_err(Error::Open)?;
        let mut client = Self { link: Box::new(HidLink { device }), next_seq: initial_seq() };
        let _ = client.request(&Command::Ping, OPEN_BURN_TIMEOUT);
        Ok(client)
    }

    /// Sends one command and waits for its ack; a lost ack is resent once.
    /// Stale and undecodable replies are discarded (bounded; see
    /// [`wait_ack`]).
    ///
    /// # Errors
    /// [`Error::Timeout`] after send plus one resend, transport errors from
    /// [`Error`].
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

/// Waits for the ack of one attempt.
///
/// One deadline spans the whole wait: stale replies (for an earlier,
/// timed-out request) must not extend it, and a stream of stale or
/// undecodable frames is capped instead of looping forever.
fn wait_ack(link: &mut dyn Link, seq: u8, timeout: Duration) -> Result<Reply, Error> {
    /// How many junk frames one attempt tolerates before declaring the
    /// attempt lost.
    const MAX_JUNK: u32 = 8;
    let deadline = std::time::Instant::now() + timeout;
    let mut junk = 0u32;
    loop {
        let now = std::time::Instant::now();
        if now >= deadline {
            return Err(Error::Timeout);
        }
        // A short read is junk like any undecodable frame.
        let Some(raw) = link.recv(deadline - now)? else {
            junk += 1;
            continue;
        };
        match frame::decode_reply(&raw) {
            Ok((got, reply)) if got == seq => return Ok(reply),
            // Stale (an earlier request's ack) or undecodable: tolerate a
            // bounded number, then declare the attempt lost.
            _ => junk += 1,
        }
        if junk >= MAX_JUNK {
            return Err(Error::Timeout);
        }
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
        initial_seq,
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

        fn recv(&mut self, _timeout: Duration) -> Result<Option<[u8; FRAME_LEN]>, Error> {
            match self.replies.borrow_mut().pop_front() {
                Some(reply) => reply.map(Some),
                None => Err(Error::Timeout),
            }
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
    fn undecodable_reply_is_tolerated_then_the_ack_found() {
        let mut junk = [0u8; FRAME_LEN];
        junk[1] = 0x7F; // unknown reply tag
        let mut link = FakeLink {
            replies: Rc::new(RefCell::new(std::collections::VecDeque::from([
                Ok(junk),
                Ok(ack_for(1)),
            ]))),
            ..FakeLink::default()
        };
        let reply = issue(&mut link, &frame_for(1), 1, TIMEOUT);
        assert!(matches!(reply, Ok(Reply::Ok)), "one junk frame must not fail the request");
        assert_eq!(link.sent(), 1);
    }

    #[test]
    fn junk_reply_flood_times_out_instead_of_looping() {
        // More stale frames than one attempt can tolerate, for both
        // attempts: the request must end instead of waiting forever.
        let flood: std::collections::VecDeque<_> = (0..32).map(|_| Ok(ack_for(9))).collect();
        let mut link = FakeLink { replies: Rc::new(RefCell::new(flood)), ..FakeLink::default() };
        let reply = issue(&mut link, &frame_for(1), 1, TIMEOUT);
        assert!(matches!(reply, Err(Error::Timeout)));
        assert_eq!(link.sent(), 2, "the resend still happens once, then give up");
    }

    #[test]
    fn initial_seq_stays_in_host_range() {
        // The wire contract reserves 0 as "never executed" on the device;
        // the seed must never produce it.
        for _ in 0..256 {
            let seq = initial_seq();
            assert!((1..=u8::MAX).contains(&seq), "seeded seq {seq} outside 1..=255");
        }
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

    #[test]
    fn client_wraps_sequence_numbers_past_u8_max() {
        let fake = FakeLink {
            replies: Rc::new(RefCell::new(std::collections::VecDeque::from([
                Ok(ack_for(u8::MAX)),
                Ok(ack_for(1)),
            ]))),
            ..FakeLink::default()
        };
        let mut client = super::Client { link: Box::new(fake.clone()), next_seq: u8::MAX };
        let _ = client.request(&Command::Ping, TIMEOUT);
        let _ = client.request(&Command::Ping, TIMEOUT);
        assert_eq!(fake.sent_seq(0), Some(u8::MAX));
        assert_eq!(fake.sent_seq(1), Some(1), "seq wraps MAX -> 1");
    }
}
