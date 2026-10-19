//! Device-side duplicate suppression for the one-resend host policy.
//!
//! The host resends a command once when its ack is lost. The device acks
//! after execution, so a paced command can outlive the host's ack timeout -
//! without suppression, the resend would execute twice (a double key tap,
//! a double click). The fix is one register per channel: when a frame's
//! sequence number repeats the last executed one, replay the cached reply
//! instead of executing again. Normal traffic increments the sequence, so
//! only true resends ever match.

use crate::Reply;

/// Last-executed sequence register for one command channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SeqDedup {
    last: Option<(u8, Reply)>,
}

impl SeqDedup {
    /// Creates an empty register.
    #[must_use]
    pub const fn new() -> Self {
        Self { last: None }
    }

    /// Returns the cached reply when `seq` repeats the last executed
    /// command (a resend after a lost ack); `None` when the caller must
    /// execute the command.
    #[must_use]
    pub fn replay(&self, seq: u8) -> Option<Reply> {
        match self.last {
            Some((last_seq, reply)) if last_seq == seq => Some(reply),
            _ => None,
        }
    }

    /// Records the reply for an executed command.
    pub fn remember(&mut self, seq: u8, reply: Reply) {
        self.last = Some((seq, reply));
    }
}

impl Default for SeqDedup {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::SeqDedup;
    use crate::{
        ErrorCode,
        Reply,
    };

    #[test]
    fn replays_only_the_repeated_sequence() {
        let mut dedup = SeqDedup::new();
        assert_eq!(dedup.replay(1), None);
        dedup.remember(1, Reply::Ok);
        assert_eq!(dedup.replay(1), Some(Reply::Ok), "resend of the last seq replays");
        assert_eq!(dedup.replay(2), None, "a fresh seq must execute");
    }

    #[test]
    fn remembers_only_the_latest_command() {
        let mut dedup = SeqDedup::new();
        dedup.remember(5, Reply::Ok);
        dedup.remember(6, Reply::Err(ErrorCode::Disarmed));
        assert_eq!(dedup.replay(5), None, "older seqs are not remembered");
        assert_eq!(dedup.replay(6), Some(Reply::Err(ErrorCode::Disarmed)));
    }

    #[test]
    fn sequence_wraparound_is_not_a_duplicate() {
        // The host wraps u8::MAX -> 1; the single-slot register keeps that
        // safe because only the immediately-previous seq can replay.
        let mut dedup = SeqDedup::new();
        dedup.remember(u8::MAX, Reply::Ok);
        assert_eq!(dedup.replay(1), None);
    }

    #[test]
    fn first_frame_always_executes() {
        let dedup = SeqDedup::new();
        assert_eq!(dedup.replay(0), None);
    }
}
