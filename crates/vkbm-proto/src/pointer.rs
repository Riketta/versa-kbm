//! Button-latch bookkeeping for the device's two pointer interfaces.
//!
//! The OS combines button state across HID mice, so every interface must
//! always report the truth about its own buttons and never carry another
//! interface's press. Each press is **owned** by the interface it was
//! pressed on: moves and wheel reports retransmit only their own interface's
//! bits, and a release always rides the owner - a press followed by a move
//! on the other pointer can therefore never latch a button at the OS.

use crate::Button;

/// Number of buttons the descriptors declare (usage tags `1..=5`).
const BUTTONS: usize = 5;

/// Which button bits each pointer interface currently reports down.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PointerLatches {
    /// Bits currently reported down on the absolute pointer.
    abs: u8,
    /// Bits currently reported down on the relative pointer.
    rel: u8,
    /// Owner of each button's press (`Some(true)` = absolute pointer).
    owner: [Option<bool>; BUTTONS],
}

impl PointerLatches {
    /// Creates an all-released state.
    #[must_use]
    pub const fn new() -> Self {
        Self { abs: 0, rel: 0, owner: [None; BUTTONS] }
    }

    /// Latches `button` down, preferring the absolute pointer when
    /// `preferred_abs` is set - unless the button is already held, in which
    /// case its owner keeps it. Returns `(on_abs, bits)`: the interface that
    /// owns the press and the button bits to report **on that interface**.
    pub fn press(&mut self, button: Button, preferred_abs: bool) -> (bool, u8) {
        let slot = usize::from(button as u8 - 1);
        let owned = self.owner.get(slot).copied().flatten();
        let on_abs = owned.unwrap_or_else(|| {
            if let Some(s) = self.owner.get_mut(slot) {
                *s = Some(preferred_abs);
            }
            preferred_abs
        });
        let bit = 1u8 << slot;
        if on_abs {
            self.abs |= bit;
        } else {
            self.rel |= bit;
        }
        (on_abs, self.bits(on_abs))
    }

    /// Releases `button` from whichever interface owns it. Returns
    /// `(on_abs, bits)` for that interface's report, or `None` when the
    /// button was not held (the caller falls back to the active pointer).
    pub fn release(&mut self, button: Button) -> Option<(bool, u8)> {
        let slot = usize::from(button as u8 - 1);
        let on_abs = self.owner.get(slot).copied().flatten()?;
        let keep = !(1u8 << slot);
        if on_abs {
            self.abs &= keep;
        } else {
            self.rel &= keep;
        }
        if let Some(s) = self.owner.get_mut(slot) {
            *s = None;
        }
        Some((on_abs, self.bits(on_abs)))
    }

    /// Button bits to report on one interface right now: zero unless that
    /// interface owns a held press.
    #[must_use]
    pub const fn bits(&self, on_abs: bool) -> u8 {
        if on_abs { self.abs } else { self.rel }
    }

    /// The owner of `button`'s press, if held: `Some(true)` = absolute
    /// pointer, `Some(false)` = relative pointer, `None` = not held.
    #[must_use]
    pub fn owner(&self, button: Button) -> Option<bool> {
        let slot = usize::from(button as u8 - 1);
        self.owner.get(slot).copied().flatten()
    }

    /// Releases everything; both interfaces report zero buttons next.
    pub fn clear(&mut self) {
        self.abs = 0;
        self.rel = 0;
        self.owner = [None; BUTTONS];
    }
}

#[cfg(test)]
mod tests {
    use super::PointerLatches;
    use crate::Button;

    #[test]
    fn press_release_roundtrips_on_one_interface() {
        let mut l = PointerLatches::new();
        assert_eq!(l.press(Button::Left, true), (true, 1));
        assert_eq!(l.bits(true), 1, "abs reports the press");
        assert_eq!(l.release(Button::Left), Some((true, 0)));
        assert_eq!(l.release(Button::Left), None, "double release is a fallback");
    }

    #[test]
    fn moves_on_the_other_interface_never_carry_the_press() {
        let mut l = PointerLatches::new();
        l.press(Button::Left, true);
        assert_eq!(l.bits(false), 0, "relative reports carry no abs press");
        let mut r = PointerLatches::new();
        r.press(Button::Left, false);
        assert_eq!(r.bits(true), 0, "absolute reports carry no rel press");
    }

    #[test]
    fn release_rides_the_owner_not_the_active_pointer() {
        let mut l = PointerLatches::new();
        assert_eq!(l.press(Button::Left, false), (false, 1));
        // A press on the other pointer afterwards must not steal the owner.
        assert_eq!(l.press(Button::Right, true), (true, 2));
        assert_eq!(l.release(Button::Left), Some((false, 0)));
        assert_eq!(l.release(Button::Right), Some((true, 0)));
    }

    #[test]
    fn press_while_held_keeps_the_owner() {
        let mut l = PointerLatches::new();
        l.press(Button::Left, true);
        assert_eq!(l.press(Button::Left, false), (true, 1), "owner keeps the press");
        assert_eq!(l.owner(Button::Left), Some(true));
        assert_eq!(l.release(Button::Left), Some((true, 0)));
        assert_eq!(l.bits(false), 0, "no stranded bits on the losing interface");
    }

    #[test]
    fn owner_reports_the_holding_interface() {
        let mut l = PointerLatches::new();
        assert_eq!(l.owner(Button::Left), None, "released: no owner");
        l.press(Button::Left, false);
        assert_eq!(l.owner(Button::Left), Some(false), "relative owner");
        l.release(Button::Left);
        l.press(Button::Left, true);
        assert_eq!(l.owner(Button::Left), Some(true), "absolute owner");
    }

    #[test]
    fn clear_drops_every_interface() {
        let mut l = PointerLatches::new();
        l.press(Button::Left, true);
        l.press(Button::Right, false);
        l.press(Button::Middle, false);
        l.clear();
        assert_eq!(l.bits(true), 0);
        assert_eq!(l.bits(false), 0);
        assert_eq!(l.release(Button::Left), None);
        assert_eq!(l.release(Button::Right), None);
        assert_eq!(l.release(Button::Middle), None);
    }

    #[test]
    fn multiple_buttons_stack_on_one_interface() {
        let mut l = PointerLatches::new();
        assert_eq!(l.press(Button::Left, true).1, 1);
        assert_eq!(l.press(Button::Forward, true).1, 0b1_0001);
        assert_eq!(l.release(Button::Left), Some((true, 0b1_0000)));
        assert_eq!(l.release(Button::Forward), Some((true, 0)));
    }
}
