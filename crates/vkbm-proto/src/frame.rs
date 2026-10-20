//! Fixed-size binary framing for the HID vendor control channel.
//!
//! Every output report is exactly [`FRAME_LEN`] bytes: `seq`, command tag,
//! little-endian payload, zero padding. Every input-report reply mirrors that
//! shape. USB already CRC-protects transfers, so there is no application-level
//! checksum; reliability comes from the ack plus the one-resend policy in the
//! host client.

use crate::{
    Button,
    Command,
    ErrorCode,
    Reply,
};

/// Report size of the vendor control channel.
pub const FRAME_LEN: usize = 64;

/// Decoding failures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// Fewer than the two header bytes arrived.
    TooShort,
    /// Unknown command or reply tag.
    UnknownTag,
    /// Payload length did not match the tag.
    BadPayload,
}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TooShort => write!(f, "frame shorter than the 2-byte header"),
            Self::UnknownTag => write!(f, "unknown frame tag"),
            Self::BadPayload => write!(f, "payload does not match the tag"),
        }
    }
}

// Command and reply tags live in disjoint, direction-specific spaces:
// `decode` reads command tags, `decode_reply` reads reply tags. They must
// never be mixed - a command tag is meaningless as a reply tag.
mod tag {
    pub(super) const PING: u8 = 0x01;
    pub(super) const INFO: u8 = 0x02;
    pub(super) const KEY_DOWN: u8 = 0x10;
    pub(super) const KEY_UP: u8 = 0x11;
    pub(super) const KEY_TAP: u8 = 0x12;
    pub(super) const MOVE_ABS: u8 = 0x20;
    pub(super) const MOVE_REL: u8 = 0x21;
    pub(super) const MOUSE_DOWN: u8 = 0x22;
    pub(super) const MOUSE_UP: u8 = 0x23;
    pub(super) const CLICK_ABS: u8 = 0x24;
    pub(super) const WHEEL: u8 = 0x25;
    pub(super) const RELEASE_ALL: u8 = 0x30;
    pub(super) const ARM: u8 = 0x31;
    pub(super) const DISARM: u8 = 0x32;
    pub(super) const SET_PACING: u8 = 0x33;
    pub(super) const DRY_RUN: u8 = 0x34;

    pub(super) const REPLY_OK: u8 = 0x00;
    pub(super) const REPLY_ECHO: u8 = 0x01;
    pub(super) const REPLY_INFO: u8 = 0x02;
    pub(super) const REPLY_ERR: u8 = 0xFF;

    pub(super) const ERR_UNKNOWN_COMMAND: u8 = 0x01;
    pub(super) const ERR_BAD_PAYLOAD: u8 = 0x02;
    pub(super) const ERR_DISARMED: u8 = 0x03;
    pub(super) const ERR_NOT_IMPLEMENTED: u8 = 0x04;
    pub(super) const ERR_KEYBOARD_FULL: u8 = 0x05;
    pub(super) const ERR_DISABLED: u8 = 0x06;
    pub(super) const ERR_UNDELIVERABLE: u8 = 0x07;
}

/// Encodes `cmd` into `out` under sequence number `seq`, zero-padding the
/// remainder.
pub fn encode(cmd: &Command, seq: u8, out: &mut [u8; FRAME_LEN]) {
    *out = [0u8; FRAME_LEN];
    out[0] = seq;
    match *cmd {
        Command::Ping => out[1] = tag::PING,
        Command::Info => out[1] = tag::INFO,
        Command::KeyDown { usage } => {
            out[1] = tag::KEY_DOWN;
            out[2] = usage;
        }
        Command::KeyUp { usage } => {
            out[1] = tag::KEY_UP;
            out[2] = usage;
        }
        Command::KeyTap { usage } => {
            out[1] = tag::KEY_TAP;
            out[2] = usage;
        }
        Command::MouseMoveAbs { x, y } => {
            out[1] = tag::MOVE_ABS;
            out[2..4].copy_from_slice(&x.to_le_bytes());
            out[4..6].copy_from_slice(&y.to_le_bytes());
        }
        Command::MouseMoveRel { dx, dy } => {
            out[1] = tag::MOVE_REL;
            out[2..4].copy_from_slice(&dx.to_le_bytes());
            out[4..6].copy_from_slice(&dy.to_le_bytes());
        }
        Command::MouseDown { button } => {
            out[1] = tag::MOUSE_DOWN;
            out[2] = button as u8;
        }
        Command::MouseUp { button } => {
            out[1] = tag::MOUSE_UP;
            out[2] = button as u8;
        }
        Command::ClickAbs { x, y } => {
            out[1] = tag::CLICK_ABS;
            out[2..4].copy_from_slice(&x.to_le_bytes());
            out[4..6].copy_from_slice(&y.to_le_bytes());
        }
        #[allow(clippy::cast_sign_loss)] // for an i8, the low byte *is* the wire byte
        Command::Wheel { v, h } => {
            out[1] = tag::WHEEL;
            out[2] = v as u8;
            out[3] = h as u8;
        }
        Command::ReleaseAll => out[1] = tag::RELEASE_ALL,
        Command::Arm => out[1] = tag::ARM,
        Command::Disarm => out[1] = tag::DISARM,
        Command::SetPacing { inter_ms, hold_ms } => {
            out[1] = tag::SET_PACING;
            out[2] = inter_ms;
            out[3] = hold_ms;
        }
        Command::DryRun { enabled } => {
            out[1] = tag::DRY_RUN;
            out[2] = u8::from(enabled);
        }
    }
}

/// Decodes one frame; returns `(seq, command)`.
///
/// # Errors
/// [`FrameError::TooShort`] for fewer than two header bytes,
/// [`FrameError::UnknownTag`] for an unknown tag, [`FrameError::BadPayload`]
/// for a payload length or value outside the tag's contract.
pub fn decode(data: &[u8]) -> Result<(u8, Command), FrameError> {
    let Some((&seq, rest)) = data.split_first() else {
        return Err(FrameError::TooShort);
    };
    let Some((&kind, payload)) = rest.split_first() else {
        return Err(FrameError::TooShort);
    };
    let cmd = match kind {
        tag::PING => Command::ping_or(payload)?,
        tag::INFO => Command::info_or(payload)?,
        tag::KEY_DOWN => Command::KeyDown { usage: one(payload)? },
        tag::KEY_UP => Command::KeyUp { usage: one(payload)? },
        tag::KEY_TAP => Command::KeyTap { usage: one(payload)? },
        tag::MOVE_ABS => {
            let [x0, x1, y0, y1] = four(payload)?;
            Command::MouseMoveAbs {
                x: u16::from_le_bytes([x0, x1]),
                y: u16::from_le_bytes([y0, y1]),
            }
        }
        tag::MOVE_REL => {
            let [dx0, dx1, dy0, dy1] = four(payload)?;
            Command::MouseMoveRel {
                dx: i16::from_le_bytes([dx0, dx1]),
                dy: i16::from_le_bytes([dy0, dy1]),
            }
        }
        tag::MOUSE_DOWN => Command::MouseDown { button: button_or(payload)? },
        tag::MOUSE_UP => Command::MouseUp { button: button_or(payload)? },
        tag::CLICK_ABS => {
            let [x0, x1, y0, y1] = four(payload)?;
            Command::ClickAbs { x: u16::from_le_bytes([x0, x1]), y: u16::from_le_bytes([y0, y1]) }
        }
        tag::WHEEL => {
            let [vs, hs] = two(payload)?;
            Command::Wheel { v: i8::from_le_bytes([vs]), h: i8::from_le_bytes([hs]) }
        }
        tag::RELEASE_ALL => Command::release_all_or(payload)?,
        tag::ARM => Command::arm_or(payload)?,
        tag::DISARM => Command::disarm_or(payload)?,
        tag::SET_PACING => {
            let [inter_ms, hold_ms] = two(payload)?;
            Command::SetPacing { inter_ms, hold_ms }
        }
        tag::DRY_RUN => Command::dry_run_or(payload)?,
        _ => return Err(FrameError::UnknownTag),
    };
    Ok((seq, cmd))
}

/// Encodes a reply under sequence number `seq`.
pub fn encode_reply(seq: u8, reply: &Reply, out: &mut [u8; FRAME_LEN]) {
    let mut buf = [0u8; FRAME_LEN];
    buf[0] = seq;
    match *reply {
        Reply::Ok => buf[1] = tag::REPLY_OK,
        Reply::Echo => buf[1] = tag::REPLY_ECHO,
        Reply::Info { major, minor, flags } => {
            buf[1] = tag::REPLY_INFO;
            buf[2] = major;
            buf[3] = minor;
            buf[4] = flags;
        }
        Reply::Err(code) => {
            buf[1] = tag::REPLY_ERR;
            buf[2] = match code {
                ErrorCode::UnknownCommand => tag::ERR_UNKNOWN_COMMAND,
                ErrorCode::BadPayload => tag::ERR_BAD_PAYLOAD,
                ErrorCode::Disarmed => tag::ERR_DISARMED,
                ErrorCode::NotImplemented => tag::ERR_NOT_IMPLEMENTED,
                ErrorCode::KeyboardFull => tag::ERR_KEYBOARD_FULL,
                ErrorCode::Disabled => tag::ERR_DISABLED,
                ErrorCode::Undeliverable => tag::ERR_UNDELIVERABLE,
            };
        }
    }
    *out = buf;
}

/// Decodes one reply frame; returns `(seq, reply)`.
///
/// # Errors
/// Same contract as [`decode`].
pub fn decode_reply(data: &[u8]) -> Result<(u8, Reply), FrameError> {
    let Some((&seq, rest)) = data.split_first() else {
        return Err(FrameError::TooShort);
    };
    let Some((&t, payload)) = rest.split_first() else {
        return Err(FrameError::TooShort);
    };
    let reply = match t {
        tag::REPLY_OK => Reply::ok_or(payload)?,
        tag::REPLY_ECHO => Reply::echo_or(payload)?,
        tag::REPLY_INFO => {
            let [major, minor, flags] = three(payload)?;
            Reply::Info { major, minor, flags }
        }
        tag::REPLY_ERR => {
            let code = one(payload)?;
            Reply::Err(match code {
                tag::ERR_UNKNOWN_COMMAND => ErrorCode::UnknownCommand,
                tag::ERR_BAD_PAYLOAD => ErrorCode::BadPayload,
                tag::ERR_DISARMED => ErrorCode::Disarmed,
                tag::ERR_NOT_IMPLEMENTED => ErrorCode::NotImplemented,
                tag::ERR_KEYBOARD_FULL => ErrorCode::KeyboardFull,
                tag::ERR_DISABLED => ErrorCode::Disabled,
                tag::ERR_UNDELIVERABLE => ErrorCode::Undeliverable,
                _ => return Err(FrameError::UnknownTag),
            })
        }
        _ => return Err(FrameError::UnknownTag),
    };
    Ok((seq, reply))
}

// Payload helpers accept trailing zero padding: the wire always carries
// full 64-byte reports, so a zero-payload command arrives with 62 zero bytes
// behind the header. Nonzero bytes beyond the expected payload are rejected.

fn empty(payload: &[u8]) -> Result<(), FrameError> {
    if payload.iter().all(|&b| b == 0) { Ok(()) } else { Err(FrameError::BadPayload) }
}

fn one(payload: &[u8]) -> Result<u8, FrameError> {
    match payload {
        [b] => Ok(*b),
        [b, rest @ ..] if rest.iter().all(|&x| x == 0) => Ok(*b),
        _ => Err(FrameError::BadPayload),
    }
}

fn two(payload: &[u8]) -> Result<[u8; 2], FrameError> {
    match payload {
        [a, b] => Ok([*a, *b]),
        [a, b, rest @ ..] if rest.iter().all(|&x| x == 0) => Ok([*a, *b]),
        _ => Err(FrameError::BadPayload),
    }
}

fn three(payload: &[u8]) -> Result<[u8; 3], FrameError> {
    match payload {
        [a, b, c] => Ok([*a, *b, *c]),
        [a, b, c, rest @ ..] if rest.iter().all(|&x| x == 0) => Ok([*a, *b, *c]),
        _ => Err(FrameError::BadPayload),
    }
}

fn four(payload: &[u8]) -> Result<[u8; 4], FrameError> {
    match payload {
        [a, b, c, d] => Ok([*a, *b, *c, *d]),
        [a, b, c, d, rest @ ..] if rest.iter().all(|&x| x == 0) => Ok([*a, *b, *c, *d]),
        _ => Err(FrameError::BadPayload),
    }
}

fn button_or(payload: &[u8]) -> Result<Button, FrameError> {
    Button::from_tag(one(payload)?).ok_or(FrameError::BadPayload)
}

// The `*_or` helpers keep the zero-payload arms of `decode` one line each.
impl Command {
    fn ping_or(payload: &[u8]) -> Result<Self, FrameError> {
        empty(payload).map(|()| Self::Ping)
    }

    fn info_or(payload: &[u8]) -> Result<Self, FrameError> {
        empty(payload).map(|()| Self::Info)
    }

    fn release_all_or(payload: &[u8]) -> Result<Self, FrameError> {
        empty(payload).map(|()| Self::ReleaseAll)
    }

    fn arm_or(payload: &[u8]) -> Result<Self, FrameError> {
        empty(payload).map(|()| Self::Arm)
    }

    fn disarm_or(payload: &[u8]) -> Result<Self, FrameError> {
        empty(payload).map(|()| Self::Disarm)
    }

    fn dry_run_or(payload: &[u8]) -> Result<Self, FrameError> {
        bool_or(payload).map(|enabled| Self::DryRun { enabled })
    }
}

impl Reply {
    fn ok_or(payload: &[u8]) -> Result<Self, FrameError> {
        empty(payload).map(|()| Self::Ok)
    }

    fn echo_or(payload: &[u8]) -> Result<Self, FrameError> {
        empty(payload).map(|()| Self::Echo)
    }
}

/// A strict wire boolean: only `0x00` and `0x01` decode.
fn bool_or(payload: &[u8]) -> Result<bool, FrameError> {
    match one(payload)? {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(FrameError::BadPayload),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        FRAME_LEN,
        FrameError,
        decode,
        decode_reply,
        encode,
        encode_reply,
        tag,
    };
    use crate::{
        Button,
        Command,
        ErrorCode,
        Reply,
    };

    const SEQ: u8 = 7;

    fn roundtrip(cmd: Command) {
        let mut buf = [0u8; FRAME_LEN];
        encode(&cmd, SEQ, &mut buf);
        assert_eq!(decode(&buf), Ok((SEQ, cmd)));
    }

    #[test]
    fn encodes_header_and_zero_padding() {
        let mut buf = [1u8; FRAME_LEN];
        encode(&Command::Ping, SEQ, &mut buf);
        assert_eq!(buf[0], SEQ);
        assert_eq!(buf[1], tag::PING);
        assert!(buf[2..].iter().all(|&b| b == 0));
    }

    #[test]
    fn roundtrips_every_command() {
        let samples = [
            Command::Ping,
            Command::Info,
            Command::KeyDown { usage: 41 },
            Command::KeyUp { usage: 0xE0 },
            Command::KeyTap { usage: 0xFF },
            Command::MouseMoveAbs { x: 0, y: crate::ABS_MAX },
            Command::MouseMoveRel { dx: -32_768, dy: 32_767 },
            Command::MouseDown { button: Button::Left },
            Command::MouseUp { button: Button::Forward },
            Command::ClickAbs { x: 16_384, y: 4_096 },
            Command::Wheel { v: -127, h: 127 },
            Command::ReleaseAll,
            Command::Arm,
            Command::Disarm,
            Command::SetPacing { inter_ms: 5, hold_ms: 10 },
            Command::DryRun { enabled: true },
            Command::DryRun { enabled: false },
        ];
        for cmd in samples {
            roundtrip(cmd);
        }
    }

    #[test]
    fn rejects_short_and_unknown() {
        assert_eq!(decode(&[]), Err(FrameError::TooShort));
        assert_eq!(decode(&[SEQ]), Err(FrameError::TooShort));
        assert_eq!(decode(&[SEQ, 0x7F]), Err(FrameError::UnknownTag));
        assert_eq!(decode(&[SEQ, tag::PING, 0xFF]), Err(FrameError::BadPayload));
        assert_eq!(decode(&[SEQ, tag::MOUSE_DOWN, 0]), Err(FrameError::BadPayload));
        assert_eq!(decode(&[SEQ, tag::MOUSE_DOWN, 6]), Err(FrameError::BadPayload));
        assert_eq!(decode(&[SEQ, tag::MOVE_ABS, 1, 2, 3]), Err(FrameError::BadPayload));
        assert_eq!(decode(&[SEQ, tag::DRY_RUN, 2]), Err(FrameError::BadPayload));
    }

    #[test]
    fn replies_roundtrip() {
        let samples = [
            Reply::Ok,
            Reply::Echo,
            Reply::Info { major: 0, minor: 1, flags: crate::info_flags::SWITCH_ON },
            Reply::Err(ErrorCode::UnknownCommand),
            Reply::Err(ErrorCode::BadPayload),
            Reply::Err(ErrorCode::Disarmed),
            Reply::Err(ErrorCode::NotImplemented),
            Reply::Err(ErrorCode::KeyboardFull),
            Reply::Err(ErrorCode::Disabled),
            Reply::Err(ErrorCode::Undeliverable),
        ];
        for reply in samples {
            let mut buf = [0u8; FRAME_LEN];
            encode_reply(SEQ, &reply, &mut buf);
            assert_eq!(decode_reply(&buf), Ok((SEQ, reply)));
        }
    }

    #[test]
    fn rejects_bad_replies() {
        assert_eq!(decode_reply(&[SEQ]), Err(FrameError::TooShort));
        assert_eq!(decode_reply(&[SEQ, 0x7F]), Err(FrameError::UnknownTag));
        assert_eq!(decode_reply(&[SEQ, tag::REPLY_OK, 0xFF]), Err(FrameError::BadPayload));
        assert_eq!(decode_reply(&[SEQ, tag::REPLY_ERR, 0x40]), Err(FrameError::UnknownTag));
    }
}
