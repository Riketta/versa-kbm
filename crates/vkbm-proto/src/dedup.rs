//! Device-side duplicate suppression for the one-resend host policy.
//!
//! The host resends a command once when its ack is lost. The device acks
//! after execution, so a paced command can outlive the host's ack timeout -
//! without suppression, the resend would execute twice (a double key tap,
//! a double click). The fix is one register per channel: when a frame's
//! sequence number repeats the last executed one, replay the cached reply
//! instead of executing again. Normal traffic increments the sequence, so
//! only true resends ever match.
//!
//! The register stamps the execution time and replays only within
//! [`REPLAY_WINDOW_MS`]. That keeps the replay bounded to true resends - a
//! resend arrives within the host's ack timeout, at most about a second
//! even for max-paced commands - so a NEW host session that starts at the
//! same sequence number the register still holds executes normally
//! instead of swallowing its first command.

use crate::Reply;

/// How long a cached reply stays eligible for replay, in milliseconds. A
/// true resend arrives within the host ack timeout (under a second even
/// for max-paced commands); a repeat past the window is treated as a new
/// command that happens to reuse the sequence number and must execute.
pub const REPLAY_WINDOW_MS: u64 = 2_000;

/// Last-executed sequence register for one command channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SeqDedup {
    last: Option<(u8, Reply, u64)>,
}

impl SeqDedup {
    /// Creates an empty register.
    #[must_use]
    pub const fn new() -> Self {
        Self { last: None }
    }

    /// Returns the cached reply when `seq` repeats the last executed
    /// command within [`REPLAY_WINDOW_MS`] of its execution (a resend
    /// after a lost ack); `None` when the caller must execute the command.
    /// `now_ms` may wrap; the age is computed with wrapping subtraction.
    #[must_use]
    pub fn replay(&self, seq: u8, now_ms: u64) -> Option<Reply> {
        match self.last {
            Some((last_seq, reply, at_ms))
                if last_seq == seq && now_ms.wrapping_sub(at_ms) <= REPLAY_WINDOW_MS =>
            {
                Some(reply)
            }
            _ => None,
        }
    }

    /// Records the reply for an executed command, stamped with `now_ms`.
    pub fn remember(&mut self, seq: u8, reply: Reply, now_ms: u64) {
        self.last = Some((seq, reply, now_ms));
    }
}

impl Default for SeqDedup {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::{
        REPLAY_WINDOW_MS,
        SeqDedup,
    };
    use crate::{
        ErrorCode,
        Reply,
    };

    #[test]
    fn replays_only_the_repeated_sequence() {
        let mut dedup = SeqDedup::new();
        assert_eq!(dedup.replay(1, 100), None);
        dedup.remember(1, Reply::Ok, 100);
        assert_eq!(dedup.replay(1, 150), Some(Reply::Ok), "resend of the last seq replays");
        assert_eq!(dedup.replay(2, 150), None, "a fresh seq must execute");
    }

    #[test]
    fn remembers_only_the_latest_command() {
        let mut dedup = SeqDedup::new();
        dedup.remember(5, Reply::Ok, 10);
        dedup.remember(6, Reply::Err(ErrorCode::Disarmed), 20);
        assert_eq!(dedup.replay(5, 30), None, "older seqs are not remembered");
        assert_eq!(dedup.replay(6, 30), Some(Reply::Err(ErrorCode::Disarmed)));
    }

    #[test]
    fn sequence_wraparound_is_not_a_duplicate() {
        // The host wraps u8::MAX -> 1; the single-slot register keeps that
        // safe because only the immediately-previous seq can replay.
        let mut dedup = SeqDedup::new();
        dedup.remember(u8::MAX, Reply::Ok, 0);
        assert_eq!(dedup.replay(1, 1), None);
    }

    #[test]
    fn first_frame_always_executes() {
        let dedup = SeqDedup::new();
        assert_eq!(dedup.replay(0, 0), None);
    }

    #[test]
    fn replays_within_the_window() {
        let mut dedup = SeqDedup::new();
        dedup.remember(3, Reply::Ok, 1_000);
        assert_eq!(
            dedup.replay(3, 1_000 + REPLAY_WINDOW_MS),
            Some(Reply::Ok),
            "the last millisecond of the window still replays"
        );
    }

    #[test]
    fn replay_expires_after_the_window() {
        let mut dedup = SeqDedup::new();
        dedup.remember(7, Reply::Ok, 0);
        assert_eq!(
            dedup.replay(7, REPLAY_WINDOW_MS + 1),
            None,
            "an expired repeat is not a resend: the command must execute"
        );
    }

    #[test]
    fn clock_wrap_does_not_look_recent() {
        // The age is `now.wrapping_sub(at)`, so a wrap crossing replays
        // briefly after it, but past the window it must not.
        let mut dedup = SeqDedup::new();
        dedup.remember(9, Reply::Ok, u64::MAX - 100);
        assert_eq!(dedup.replay(9, 300), Some(Reply::Ok), "just across the wrap replays");
        assert_eq!(
            dedup.replay(9, 3_000),
            None,
            "three seconds across the wrap is past the window"
        );
    }
}
