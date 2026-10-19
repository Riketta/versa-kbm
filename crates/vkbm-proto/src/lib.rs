//! Wire contract between the versa-kbm host tools (sphinx backend, companion
//! CLI) and the RP2040 firmware.
//!
//! Two encodings carry the same [`Command`]/[`Reply`] vocabulary:
//!
//! - [`frame`]: fixed 64-byte binary reports for the production HID vendor
//!   control channel (usage page `0xFF00`).
//! - [`line`]: human-readable ASCII lines for the `dev-cdc` bench channel.
//!
//! The crate is `no_std` so the firmware parses exactly what the host sends;
//! both sides link the same codec and the tests run on the host.

#![no_std]

pub mod frame;
pub mod line;

/// Highest coordinate accepted by the absolute-mouse mapping. This is the HID
/// logical maximum; the OS scales the range across the desktop.
pub const ABS_MAX: u16 = 32_767;

/// A mouse button, in absolute-mouse report bit order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Button {
    /// Primary button.
    Left = 1,
    /// Secondary button.
    Right = 2,
    /// Middle button.
    Middle = 3,
    /// Fourth button (back).
    Back = 4,
    /// Fifth button (forward).
    Forward = 5,
}

impl Button {
    /// Parses a button from its wire tag (`1..=5`).
    #[must_use]
    pub fn from_tag(tag: u8) -> Option<Self> {
        match tag {
            1 => Some(Self::Left),
            2 => Some(Self::Right),
            3 => Some(Self::Middle),
            4 => Some(Self::Back),
            5 => Some(Self::Forward),
            _ => None,
        }
    }
}

/// One host-to-device command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    /// Liveness probe.
    Ping,
    /// Firmware version query.
    Info,
    /// Presses a keyboard usage (HID usage code) down.
    KeyDown {
        /// HID keyboard usage code.
        usage: u8,
    },
    /// Releases a keyboard usage.
    KeyUp {
        /// HID keyboard usage code.
        usage: u8,
    },
    /// Presses and releases a keyboard usage.
    KeyTap {
        /// HID keyboard usage code.
        usage: u8,
    },
    /// Moves the pointer to an absolute position (`0..=ABS_MAX` per axis).
    MouseMoveAbs {
        /// Absolute X.
        x: u16,
        /// Absolute Y.
        y: u16,
    },
    /// Moves the pointer by a relative delta.
    MouseMoveRel {
        /// Signed X delta.
        dx: i16,
        /// Signed Y delta.
        dy: i16,
    },
    /// Presses a mouse button down.
    MouseDown {
        /// Which button.
        button: Button,
    },
    /// Releases a mouse button.
    MouseUp {
        /// Which button.
        button: Button,
    },
    /// Moves to an absolute position, then clicks the primary button.
    ClickAbs {
        /// Absolute X.
        x: u16,
        /// Absolute Y.
        y: u16,
    },
    /// Scrolls: vertical `v` and horizontal `h` wheel ticks.
    Wheel {
        /// Vertical ticks.
        v: i8,
        /// Horizontal (pan) ticks.
        h: i8,
    },
    /// Releases every held key and button.
    ReleaseAll,
    /// Re-enables actuation after a disarm.
    Arm,
    /// Soft killswitch: actuation commands are refused until [`Command::Arm`].
    Disarm,
    /// Sets actuation pacing: gap between sequenced steps, button-hold time.
    SetPacing {
        /// Gap between sequenced report steps, in milliseconds.
        inter_ms: u8,
        /// Button-hold time for clicks and taps, in milliseconds.
        hold_ms: u8,
    },
}

/// Device-to-host reply.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reply {
    /// Command accepted and executed.
    Ok,
    /// Command rejected.
    Err(ErrorCode),
    /// Answer to [`Command::Info`].
    Info {
        /// Semver major.
        major: u8,
        /// Semver minor.
        minor: u8,
    },
}

/// Rejection reasons.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorCode {
    /// Unknown command tag or line keyword.
    UnknownCommand,
    /// Payload had the wrong shape.
    BadPayload,
    /// Device is disarmed ([`Command::Disarm`]); only `Arm`, `Ping`, `Info`
    /// and the configuration commands are served.
    Disarmed,
    /// Command recognized but not wired into this firmware build yet.
    NotImplemented,
}
