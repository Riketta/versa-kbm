//! ASCII line encoding for the `dev-cdc` bench channel.
//!
//! Same [`Command`]/[`Reply`] vocabulary as [`crate::frame`], one command per
//! `\n`-terminated line. Keywords are uppercase (decoding is
//! case-insensitive); numbers are decimal; separators are single spaces.

use core::fmt::Write as _;

use crate::{
    Button,
    Command,
    ErrorCode,
    Reply,
};

/// Longest accepted input line, in bytes.
pub const MAX_LINE_LEN: usize = 96;

/// Decode failures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineError {
    /// Nothing to decode.
    Empty,
    /// Line exceeded [`MAX_LINE_LEN`].
    TooLong,
    /// Unknown keyword.
    UnknownCommand,
    /// Wrong number of arguments.
    BadArguments,
    /// Argument was not a number in range.
    BadNumber,
    /// Argument was not a button name.
    BadButton,
}

/// Output buffer too small for the encoded line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    /// The buffer could not fit the line.
    TooSmall,
}

impl core::fmt::Display for LineError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Empty => write!(f, "empty line"),
            Self::TooLong => write!(f, "line longer than {MAX_LINE_LEN} bytes"),
            Self::UnknownCommand => write!(f, "unknown command keyword"),
            Self::BadArguments => write!(f, "wrong number of arguments"),
            Self::BadNumber => write!(f, "argument is not a number in range"),
            Self::BadButton => write!(f, "argument is not a button name"),
        }
    }
}

impl core::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TooSmall => write!(f, "output buffer too small for the line"),
        }
    }
}

/// Decodes one command line (terminator excluded by the caller).
///
/// # Errors
/// [`LineError::Empty`] for a blank line, [`LineError::TooLong`] beyond
/// [`MAX_LINE_LEN`], and the argument errors listed on [`LineError`].
pub fn decode(line: &[u8]) -> Result<Command, LineError> {
    // CRLF terminals: strip trailing carriage returns before parsing.
    let mut line = line;
    while let Some((&b'\r', rest)) = line.split_last() {
        line = rest;
    }
    if line.len() > MAX_LINE_LEN {
        return Err(LineError::TooLong);
    }
    let mut words = line.split(|&b| b == b' ').filter(|w| !w.is_empty());
    let Some(head) = words.next() else {
        return Err(LineError::Empty);
    };
    if head.eq_ignore_ascii_case(b"PING") {
        no_more(&mut words)?;
        Ok(Command::Ping)
    } else if head.eq_ignore_ascii_case(b"INFO") {
        no_more(&mut words)?;
        Ok(Command::Info)
    } else if head.eq_ignore_ascii_case(b"KD") {
        let usage = num_u8(arg(&mut words)?)?;
        no_more(&mut words)?;
        Ok(Command::KeyDown { usage })
    } else if head.eq_ignore_ascii_case(b"KU") {
        let usage = num_u8(arg(&mut words)?)?;
        no_more(&mut words)?;
        Ok(Command::KeyUp { usage })
    } else if head.eq_ignore_ascii_case(b"KT") {
        let usage = num_u8(arg(&mut words)?)?;
        no_more(&mut words)?;
        Ok(Command::KeyTap { usage })
    } else if head.eq_ignore_ascii_case(b"MA") {
        let (x, y) = xy(&mut words)?;
        no_more(&mut words)?;
        Ok(Command::MouseMoveAbs { x, y })
    } else if head.eq_ignore_ascii_case(b"MR") {
        let (dx, dy) = dxy(&mut words)?;
        no_more(&mut words)?;
        Ok(Command::MouseMoveRel { dx, dy })
    } else if head.eq_ignore_ascii_case(b"MD") {
        let button = button(arg(&mut words)?)?;
        no_more(&mut words)?;
        Ok(Command::MouseDown { button })
    } else if head.eq_ignore_ascii_case(b"MU") {
        let button = button(arg(&mut words)?)?;
        no_more(&mut words)?;
        Ok(Command::MouseUp { button })
    } else if head.eq_ignore_ascii_case(b"CA") {
        let (x, y) = xy(&mut words)?;
        no_more(&mut words)?;
        Ok(Command::ClickAbs { x, y })
    } else if head.eq_ignore_ascii_case(b"W") {
        let (v, h) = wh(&mut words)?;
        no_more(&mut words)?;
        Ok(Command::Wheel { v, h })
    } else if head.eq_ignore_ascii_case(b"RA") {
        no_more(&mut words)?;
        Ok(Command::ReleaseAll)
    } else if head.eq_ignore_ascii_case(b"ARM") {
        no_more(&mut words)?;
        Ok(Command::Arm)
    } else if head.eq_ignore_ascii_case(b"DISARM") {
        no_more(&mut words)?;
        Ok(Command::Disarm)
    } else if head.eq_ignore_ascii_case(b"SP") {
        let inter_ms = num_u8(arg(&mut words)?)?;
        let hold_ms = num_u8(arg(&mut words)?)?;
        no_more(&mut words)?;
        Ok(Command::SetPacing { inter_ms, hold_ms })
    } else if head.eq_ignore_ascii_case(b"DRY") {
        let tok = arg(&mut words)?;
        let enabled = if tok.eq_ignore_ascii_case(b"on") {
            true
        } else if tok.eq_ignore_ascii_case(b"off") {
            false
        } else {
            return Err(LineError::BadArguments);
        };
        no_more(&mut words)?;
        Ok(Command::DryRun { enabled })
    } else {
        Err(LineError::UnknownCommand)
    }
}

/// Encodes a command into `out`; returns the written line.
///
/// # Errors
/// [`EncodeError::TooSmall`] when `out` cannot fit the line.
pub fn encode<'a>(cmd: &Command, out: &'a mut [u8]) -> Result<&'a str, EncodeError> {
    let mut w = LineOut { buf: out, pos: 0 };
    let ok = match *cmd {
        Command::Ping => core::write!(w, "PING"),
        Command::Info => core::write!(w, "INFO"),
        Command::KeyDown { usage } => core::write!(w, "KD {usage}"),
        Command::KeyUp { usage } => core::write!(w, "KU {usage}"),
        Command::KeyTap { usage } => core::write!(w, "KT {usage}"),
        Command::MouseMoveAbs { x, y } => core::write!(w, "MA {x} {y}"),
        Command::MouseMoveRel { dx, dy } => core::write!(w, "MR {dx} {dy}"),
        Command::MouseDown { button } => core::write!(w, "MD {}", name(button)),
        Command::MouseUp { button } => core::write!(w, "MU {}", name(button)),
        Command::ClickAbs { x, y } => core::write!(w, "CA {x} {y}"),
        Command::Wheel { v, h } => core::write!(w, "W {v} {h}"),
        Command::ReleaseAll => core::write!(w, "RA"),
        Command::Arm => core::write!(w, "ARM"),
        Command::Disarm => core::write!(w, "DISARM"),
        Command::SetPacing { inter_ms, hold_ms } => core::write!(w, "SP {inter_ms} {hold_ms}"),
        Command::DryRun { enabled } => {
            core::write!(w, "DRY {}", if enabled { "on" } else { "off" })
        }
    };
    finish(w, ok)
}

/// Encodes a reply into `out`; returns the written line.
///
/// # Errors
/// [`EncodeError::TooSmall`] when `out` cannot fit the line.
pub fn encode_reply<'a>(reply: &Reply, out: &'a mut [u8]) -> Result<&'a str, EncodeError> {
    let mut w = LineOut { buf: out, pos: 0 };
    let ok = match *reply {
        Reply::Ok => core::write!(w, "OK"),
        Reply::Echo => core::write!(w, "ECHO"),
        Reply::Info { major, minor, flags } => core::write!(
            w,
            "VKBM {major}.{minor} switch={} arm={} dry={}",
            if flags & crate::info_flags::SWITCH_ON != 0 { "on" } else { "off" },
            if flags & crate::info_flags::ARMED != 0 { "on" } else { "off" },
            if flags & crate::info_flags::DRY_RUN != 0 { "on" } else { "off" }
        ),
        Reply::Err(code) => core::write!(
            w,
            "ERR {}",
            match code {
                ErrorCode::UnknownCommand => "UNKNOWN-COMMAND",
                ErrorCode::BadPayload => "BAD-PAYLOAD",
                ErrorCode::Disarmed => "DISARMED",
                ErrorCode::NotImplemented => "NOT-IMPLEMENTED",
                ErrorCode::KeyboardFull => "KEYBOARD-FULL",
                ErrorCode::Disabled => "DISABLED",
                ErrorCode::Undeliverable => "UNDELIVERABLE",
            }
        ),
    };
    finish(w, ok)
}

struct LineOut<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl core::fmt::Write for LineOut<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let Some(rest) = self.buf.get_mut(self.pos..) else {
            return Err(core::fmt::Error);
        };
        if bytes.len() > rest.len() {
            return Err(core::fmt::Error);
        }
        let (dst, _) = rest.split_at_mut(bytes.len());
        for (d, &b) in dst.iter_mut().zip(bytes.iter()) {
            *d = b;
        }
        self.pos += bytes.len();
        Ok(())
    }
}

fn finish(w: LineOut<'_>, ok: core::fmt::Result) -> Result<&str, EncodeError> {
    ok.map_err(|_| EncodeError::TooSmall)?;
    let LineOut { buf, pos } = w;
    let shared: &[u8] = buf;
    let Some(bytes) = shared.get(..pos) else {
        return Err(EncodeError::TooSmall);
    };
    core::str::from_utf8(bytes).map_err(|_| EncodeError::TooSmall)
}

fn name(button: Button) -> &'static str {
    match button {
        Button::Left => "left",
        Button::Right => "right",
        Button::Middle => "middle",
        Button::Back => "back",
        Button::Forward => "forward",
    }
}

fn arg<'a, I>(words: &mut I) -> Result<&'a [u8], LineError>
where
    I: Iterator<Item = &'a [u8]>,
{
    words.next().ok_or(LineError::BadArguments)
}

fn no_more<'a, I>(words: &mut I) -> Result<(), LineError>
where
    I: Iterator<Item = &'a [u8]>,
{
    if words.next().is_some() { Err(LineError::BadArguments) } else { Ok(()) }
}

fn num_u8(tok: &[u8]) -> Result<u8, LineError> {
    let s = core::str::from_utf8(tok).map_err(|_| LineError::BadNumber)?;
    s.parse().map_err(|_| LineError::BadNumber)
}

fn num_u16(tok: &[u8]) -> Result<u16, LineError> {
    let s = core::str::from_utf8(tok).map_err(|_| LineError::BadNumber)?;
    s.parse().map_err(|_| LineError::BadNumber)
}

fn num_i16(tok: &[u8]) -> Result<i16, LineError> {
    let s = core::str::from_utf8(tok).map_err(|_| LineError::BadNumber)?;
    s.parse().map_err(|_| LineError::BadNumber)
}

fn num_i8(tok: &[u8]) -> Result<i8, LineError> {
    let s = core::str::from_utf8(tok).map_err(|_| LineError::BadNumber)?;
    s.parse().map_err(|_| LineError::BadNumber)
}

fn xy<'a>(words: &mut impl Iterator<Item = &'a [u8]>) -> Result<(u16, u16), LineError> {
    let x = num_u16(arg(words)?)?;
    let y = num_u16(arg(words)?)?;
    Ok((x, y))
}

fn dxy<'a>(words: &mut impl Iterator<Item = &'a [u8]>) -> Result<(i16, i16), LineError> {
    let dx = num_i16(arg(words)?)?;
    let dy = num_i16(arg(words)?)?;
    Ok((dx, dy))
}

fn wh<'a>(words: &mut impl Iterator<Item = &'a [u8]>) -> Result<(i8, i8), LineError> {
    let v = num_i8(arg(words)?)?;
    let h = num_i8(arg(words)?)?;
    Ok((v, h))
}

fn button(tok: &[u8]) -> Result<Button, LineError> {
    for (name, b) in [
        ("left", Button::Left),
        ("right", Button::Right),
        ("middle", Button::Middle),
        ("back", Button::Back),
        ("forward", Button::Forward),
    ] {
        if tok.eq_ignore_ascii_case(name.as_bytes()) {
            return Ok(b);
        }
    }
    Err(LineError::BadButton)
}

/// Assembles `\n`-terminated command lines from arbitrary byte chunks: the
/// CDC bench channel's byte feeder. A line may split across any number of
/// writes; bytes beyond [`MAX_LINE_LEN`] are dropped, and the line is
/// reported as [`LineError::TooLong`] when its terminator finally arrives.
/// Completed lines still go through [`decode`], so an empty line decodes to
/// [`LineError::Empty`] and stays silently ignorable at the caller.
#[derive(Debug)]
pub struct LineAssembler {
    /// Bytes of the line in progress; once full, further input overflows.
    line: [u8; MAX_LINE_LEN],
    /// Bytes of `line` in use; never above `MAX_LINE_LEN`.
    len: usize,
    /// Set once the line in progress exceeded `MAX_LINE_LEN`.
    overflow: bool,
}

impl LineAssembler {
    /// Creates an empty assembler.
    #[must_use]
    pub const fn new() -> Self {
        Self { line: [0; MAX_LINE_LEN], len: 0, overflow: false }
    }

    /// Discards any partial line (a fresh serial connection starts empty).
    pub fn reset(&mut self) {
        self.len = 0;
        self.overflow = false;
    }

    /// Feeds one byte. `Some(result)` when the byte terminated a line:
    /// `Ok(line)` carries the accumulated bytes (terminator excluded,
    /// possibly empty), `Err(LineError::TooLong)` reports a line that
    /// overflowed and was dropped. The returned line borrows the assembler
    /// and is invalidated by the next call.
    pub fn feed_byte(&mut self, byte: u8) -> Option<Result<&[u8], LineError>> {
        if byte == b'\n' {
            if self.overflow {
                self.reset();
                return Some(Err(LineError::TooLong));
            }
            let len = self.len;
            self.reset();
            let (used, _) = self.line.split_at(len);
            return Some(Ok(used));
        }
        match self.line.get_mut(self.len) {
            Some(slot) => {
                *slot = byte;
                self.len += 1;
            }
            None => self.overflow = true,
        }
        None
    }
}

impl Default for LineAssembler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::{
        LineError,
        MAX_LINE_LEN,
        decode,
        encode,
        encode_reply,
    };
    use crate::{
        Button,
        Command,
        ErrorCode,
        Reply,
    };

    #[test]
    fn decodes_keywords_case_insensitively() {
        assert_eq!(decode(b"PING"), Ok(Command::Ping));
        assert_eq!(decode(b"ping"), Ok(Command::Ping));
        assert_eq!(decode(b"  ping  "), Ok(Command::Ping));
        assert_eq!(decode(b"Info"), Ok(Command::Info));
        assert_eq!(decode(b"RA"), Ok(Command::ReleaseAll));
        assert_eq!(decode(b"ARM"), Ok(Command::Arm));
        assert_eq!(decode(b"DISARM"), Ok(Command::Disarm));
    }

    #[test]
    fn decodes_crlf_terminated_lines() {
        // Serial terminals commonly send CR LF; the firmware strips the LF
        // and the decoder must tolerate the leftover CR.
        assert_eq!(decode(b"PING\r"), Ok(Command::Ping));
        assert_eq!(decode(b"MD left\r"), Ok(Command::MouseDown { button: Button::Left }));
        assert_eq!(decode(b"CA 1 2\r\r"), Ok(Command::ClickAbs { x: 1, y: 2 }));
    }

    #[test]
    fn decodes_arguments() {
        assert_eq!(decode(b"KD 41"), Ok(Command::KeyDown { usage: 41 }));
        assert_eq!(decode(b"KU 224"), Ok(Command::KeyUp { usage: 224 }));
        assert_eq!(decode(b"KT 255"), Ok(Command::KeyTap { usage: 255 }));
        assert_eq!(decode(b"MA 0 32767"), Ok(Command::MouseMoveAbs { x: 0, y: crate::ABS_MAX }));
        assert_eq!(
            decode(b"MR -32768 32767"),
            Ok(Command::MouseMoveRel { dx: -32_768, dy: 32_767 })
        );
        assert_eq!(decode(b"MD LEFT"), Ok(Command::MouseDown { button: Button::Left }));
        assert_eq!(decode(b"MU Forward"), Ok(Command::MouseUp { button: Button::Forward }));
        assert_eq!(decode(b"CA 100 200"), Ok(Command::ClickAbs { x: 100, y: 200 }));
        assert_eq!(decode(b"W -127 127"), Ok(Command::Wheel { v: -127, h: 127 }));
        assert_eq!(decode(b"SP 5 10"), Ok(Command::SetPacing { inter_ms: 5, hold_ms: 10 }));
        assert_eq!(decode(b"DRY on"), Ok(Command::DryRun { enabled: true }));
        assert_eq!(decode(b"dry OFF"), Ok(Command::DryRun { enabled: false }));
    }

    #[test]
    fn rejects_bad_lines() {
        assert_eq!(decode(b""), Err(LineError::Empty));
        assert_eq!(decode(b"    "), Err(LineError::Empty));
        assert_eq!(decode(b"FOO"), Err(LineError::UnknownCommand));
        assert_eq!(decode(b"PING 1"), Err(LineError::BadArguments));
        assert_eq!(decode(b"MA 1"), Err(LineError::BadArguments));
        assert_eq!(decode(b"MA 1 2 3"), Err(LineError::BadArguments));
        assert_eq!(decode(b"MA a b"), Err(LineError::BadNumber));
        assert_eq!(decode(b"KD 300"), Err(LineError::BadNumber));
        assert_eq!(decode(b"KD -1"), Err(LineError::BadNumber));
        assert_eq!(decode(b"MD toe"), Err(LineError::BadButton));
        assert_eq!(decode(b"DRY"), Err(LineError::BadArguments));
        assert_eq!(decode(b"DRY maybe"), Err(LineError::BadArguments));
        let long = [b'A'; MAX_LINE_LEN + 1];
        assert_eq!(decode(&long), Err(LineError::TooLong));
    }

    #[test]
    fn encodes_and_roundtrips() {
        let samples = [
            Command::Ping,
            Command::Info,
            Command::KeyDown { usage: 41 },
            Command::KeyUp { usage: 224 },
            Command::KeyTap { usage: 255 },
            Command::MouseMoveAbs { x: 0, y: crate::ABS_MAX },
            Command::MouseMoveRel { dx: -1, dy: 1 },
            Command::MouseDown { button: Button::Left },
            Command::MouseUp { button: Button::Forward },
            Command::ClickAbs { x: 16_384, y: 4_096 },
            Command::Wheel { v: -127, h: 127 },
            Command::ReleaseAll,
            Command::Arm,
            Command::Disarm,
            Command::SetPacing { inter_ms: 5, hold_ms: 10 },
            Command::DryRun { enabled: true },
        ];
        let mut buf = [0u8; MAX_LINE_LEN];
        for cmd in samples {
            let line = encode(&cmd, &mut buf).unwrap_or_default();
            assert_eq!(decode(line.as_bytes()), Ok(cmd), "line {line}");
        }
    }

    #[test]
    fn encodes_replies() {
        let mut buf = [0u8; MAX_LINE_LEN];
        assert_eq!(encode_reply(&Reply::Ok, &mut buf), Ok("OK"));
        assert_eq!(encode_reply(&Reply::Echo, &mut buf), Ok("ECHO"));
        assert_eq!(
            encode_reply(
                &Reply::Info {
                    major: 0,
                    minor: 1,
                    flags: crate::info_flags::SWITCH_ON | crate::info_flags::ARMED
                },
                &mut buf
            ),
            Ok("VKBM 0.1 switch=on arm=on dry=off")
        );
        assert_eq!(
            encode_reply(
                &Reply::Info {
                    major: 0,
                    minor: 1,
                    flags: crate::info_flags::SWITCH_ON
                        | crate::info_flags::ARMED
                        | crate::info_flags::DRY_RUN
                },
                &mut buf
            ),
            Ok("VKBM 0.1 switch=on arm=on dry=on")
        );
        assert_eq!(
            encode_reply(&Reply::Info { major: 0, minor: 1, flags: 0 }, &mut buf),
            Ok("VKBM 0.1 switch=off arm=off dry=off")
        );
        assert_eq!(encode_reply(&Reply::Err(ErrorCode::Disarmed), &mut buf), Ok("ERR DISARMED"));
        assert_eq!(
            encode_reply(&Reply::Err(ErrorCode::NotImplemented), &mut buf),
            Ok("ERR NOT-IMPLEMENTED")
        );
        assert_eq!(
            encode_reply(&Reply::Err(ErrorCode::Undeliverable), &mut buf),
            Ok("ERR UNDELIVERABLE")
        );
        assert_eq!(
            encode_reply(&Reply::Err(ErrorCode::KeyboardFull), &mut buf),
            Ok("ERR KEYBOARD-FULL")
        );
        assert_eq!(encode_reply(&Reply::Err(ErrorCode::Disabled), &mut buf), Ok("ERR DISABLED"));
    }

    #[test]
    fn exactly_max_line_len_is_accepted() {
        // Boundary: MAX_LINE_LEN itself is fine, only longer is rejected.
        // `RA` plus padding spaces stays a valid single-word command.
        let mut line = [b' '; MAX_LINE_LEN];
        let (head, _) = line.split_at_mut(2);
        head.copy_from_slice(b"RA");
        assert_eq!(decode(&line), Ok(Command::ReleaseAll));
    }

    #[test]
    fn flags_tiny_output_buffers() {
        let mut tiny = [0u8; 2];
        assert_eq!(encode(&Command::Ping, &mut tiny), Err(super::EncodeError::TooSmall));
    }
}

#[cfg(test)]
mod assembler_tests {
    extern crate std;

    use std::{
        string::String,
        vec,
        vec::Vec,
    };

    use super::{
        LineAssembler,
        LineError,
        MAX_LINE_LEN,
        decode,
    };
    use crate::Command;

    /// Feeds every chunk in order and collects the completed lines as owned
    /// strings, so table tests can read like transcripts.
    fn assemble(chunks: &[&str]) -> Vec<Result<String, LineError>> {
        let mut asm = LineAssembler::new();
        let mut lines = Vec::new();
        for chunk in chunks {
            for &byte in chunk.as_bytes() {
                if let Some(result) = asm.feed_byte(byte) {
                    lines.push(result.map(|line| String::from_utf8_lossy(line).into_owned()));
                }
            }
        }
        lines
    }

    #[test]
    fn assembles_a_line_from_one_write() {
        assert_eq!(assemble(&["PING\n"]), vec![Ok("PING".into())]);
    }

    #[test]
    fn assembles_a_line_split_across_writes() {
        assert_eq!(assemble(&["MO", "VE ", "1 2", "\n"]), vec![Ok("MOVE 1 2".into())]);
    }

    #[test]
    fn assembles_every_line_in_one_write() {
        assert_eq!(
            assemble(&["PING\nRA\n\n"]),
            vec![Ok("PING".into()), Ok("RA".into()), Ok(String::new())]
        );
    }

    #[test]
    fn empty_line_decodes_to_empty() {
        assert_eq!(assemble(&["\n"]), vec![Ok(String::new())]);
        assert_eq!(decode(b""), Err(LineError::Empty));
    }

    #[test]
    fn exactly_max_line_len_still_assembles() {
        let line = "K".repeat(MAX_LINE_LEN);
        assert_eq!(assemble(&[&line, "\n"]), vec![Ok(line.clone())]);
        assert_eq!(decode(line.as_bytes()), Err(LineError::UnknownCommand));
    }

    #[test]
    fn one_byte_past_max_overflows() {
        let line = "K".repeat(MAX_LINE_LEN + 1);
        assert_eq!(assemble(&[&line, "\n"]), vec![Err(LineError::TooLong)]);
    }

    #[test]
    fn bytes_past_max_do_not_leak_into_the_next_line() {
        let long = "K".repeat(MAX_LINE_LEN) + "XY";
        assert_eq!(
            assemble(&[&long, "\n", "RA\n"]),
            vec![Err(LineError::TooLong), Ok("RA".into())]
        );
    }

    #[test]
    fn overflow_across_writes_stays_overflowed_until_the_newline() {
        let full = "K".repeat(MAX_LINE_LEN);
        assert_eq!(assemble(&[&full, "XY", "more", "\n"]), vec![Err(LineError::TooLong)]);
    }

    #[test]
    fn recovers_after_an_overflowed_line() {
        let long = "K".repeat(MAX_LINE_LEN + 5);
        assert_eq!(
            assemble(&[&long, "\n", "PING\n"]),
            vec![Err(LineError::TooLong), Ok("PING".into())]
        );
    }

    #[test]
    fn reset_discards_the_partial_line() {
        let mut asm = LineAssembler::new();
        assert!(b"PI".iter().all(|&byte| asm.feed_byte(byte).is_none()));
        asm.reset();
        let mut lines = Vec::new();
        for &byte in b"NG\n" {
            if let Some(result) = asm.feed_byte(byte) {
                lines.push(result.map(|line| String::from_utf8_lossy(line).into_owned()));
            }
        }
        assert_eq!(lines, vec![Ok("NG".into())]);
    }

    #[test]
    fn assembled_line_feeds_decode() {
        let mut asm = LineAssembler::new();
        let mut cmd = None;
        for &byte in b"CA 16384 4096\n" {
            if let Some(Ok(line)) = asm.feed_byte(byte) {
                cmd = decode(line).ok();
            }
        }
        assert_eq!(cmd, Some(Command::ClickAbs { x: 16_384, y: 4_096 }));
    }
}
