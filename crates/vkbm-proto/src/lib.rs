//! Wire contract between the versa-kbm host tools and the RP2040 firmware.
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

pub mod dedup;
pub mod frame;
pub mod line;

/// Highest coordinate accepted by the absolute-mouse mapping. This is the HID
/// logical maximum; the OS scales the range across the desktop.
pub const ABS_MAX: u16 = 32_767;

/// Bits of the [`Reply::Info`] `flags` byte.
pub mod info_flags {
    /// The hardware switch on the board is on: the device may actuate (the
    /// soft arm gate permitting).
    pub const SWITCH_ON: u8 = 0x01;

    /// The soft arm gate is open ([`Command::Arm`] is in effect).
    pub const ARMED: u8 = 0x02;
}

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
    /// Moves the pointer to an absolute position (`0..=ABS_MAX` per axis;
    /// values above that are clamped by the device).
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
    /// Moves to an absolute position, then clicks the primary button
    /// (`0..=ABS_MAX` per axis; values above that are clamped by the device).
    ClickAbs {
        /// Absolute X.
        x: u16,
        /// Absolute Y.
        y: u16,
    },
    /// Scrolls: vertical `v` and horizontal `h` wheel ticks. Values outside
    /// `-127..=127` are clamped by the firmware (the HID descriptor declares
    /// that logical range).
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
        /// [`info_flags`] bits: hardware switch and soft arm gate state.
        flags: u8,
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
    /// Keyboard 6KRO budget already full: the usage was dropped, not
    /// latched, so the tap would have been a no-op.
    KeyboardFull,
    /// The hardware switch on the board is off: actuation is refused until
    /// someone toggles it back on at the device. Software cannot clear this
    /// state - that is the point of the switch.
    Disabled,
}

/// Command routing shared by host and firmware: which commands start new
/// actuation, and how the two gates (hardware switch, soft arm) reject them.
pub mod routing {
    use crate::{
        Command,
        ErrorCode,
    };

    /// Whether `cmd` starts NEW actuation (moves, presses, taps, scrolls).
    /// Only these are subject to the gates; pure releases, queries and
    /// configuration always execute - you must be able to let go of a held
    /// key without arming first.
    #[must_use]
    pub fn is_new_actuation(cmd: &Command) -> bool {
        matches!(
            cmd,
            Command::KeyDown { .. }
                | Command::KeyTap { .. }
                | Command::MouseMoveAbs { .. }
                | Command::MouseMoveRel { .. }
                | Command::MouseDown { .. }
                | Command::ClickAbs { .. }
                | Command::Wheel { .. }
        )
    }

    /// Gate decision for a new-actuation command: `None` when it may run,
    /// otherwise the code to reject it with. The hardware switch outranks
    /// the soft arm gate: while it is off, nothing actuates no matter what
    /// the host armed.
    #[must_use]
    pub fn gate_error(switch_on: bool, armed: bool) -> Option<ErrorCode> {
        if !switch_on {
            Some(ErrorCode::Disabled)
        } else if !armed {
            Some(ErrorCode::Disarmed)
        } else {
            None
        }
    }

    #[cfg(test)]
    mod tests {
        use super::{
            gate_error,
            is_new_actuation,
        };
        use crate::{
            Button,
            Command,
            ErrorCode,
        };

        #[test]
        fn classifies_every_command_variant() {
            let actuating = [
                Command::KeyDown { usage: 41 },
                Command::KeyTap { usage: 41 },
                Command::MouseMoveAbs { x: 0, y: 0 },
                Command::MouseMoveRel { dx: 0, dy: 0 },
                Command::MouseDown { button: Button::Left },
                Command::ClickAbs { x: 0, y: 0 },
                Command::Wheel { v: 0, h: 0 },
            ];
            for cmd in &actuating {
                assert!(is_new_actuation(cmd), "{cmd:?} must count as new actuation");
            }
            let passing = [
                Command::Ping,
                Command::Info,
                Command::Arm,
                Command::Disarm,
                Command::ReleaseAll,
                Command::KeyUp { usage: 41 },
                Command::MouseUp { button: Button::Left },
                Command::SetPacing { inter_ms: 0, hold_ms: 0 },
            ];
            for cmd in &passing {
                assert!(!is_new_actuation(cmd), "{cmd:?} must bypass the gates");
            }
        }

        #[test]
        fn gate_precedence_and_open_state() {
            assert_eq!(gate_error(false, false), Some(ErrorCode::Disabled), "switch outranks arm");
            assert_eq!(gate_error(false, true), Some(ErrorCode::Disabled));
            assert_eq!(gate_error(true, false), Some(ErrorCode::Disarmed));
            assert_eq!(gate_error(true, true), None, "both gates open: actuation runs");
        }
    }
}
