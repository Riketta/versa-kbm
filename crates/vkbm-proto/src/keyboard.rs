//! Latched keyboard state shared by the firmware's actuator and the host
//! tests: the HID boot-keyboard report shape (modifier bitmap plus up to six
//! regular usages). Kept here so the host suite covers exactly the state
//! machine the firmware executes.

/// Latched keyboard state: modifier usages (`0xE0..=0xE7`) as a bitmap plus
/// up to six regular usages, mirroring the boot-keyboard report layout.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct KeyboardState {
    /// Modifier usages `0xE0..=0xE7` as a bitmap.
    modifiers: u8,
    /// Regular usages; `0` is an empty slot.
    keycodes: [u8; 6],
}

impl KeyboardState {
    /// Creates an all-released state.
    #[must_use]
    pub const fn new() -> Self {
        Self { modifiers: 0, keycodes: [0; 6] }
    }

    /// Latches `usage` down; `false` when the 6KRO budget is already full
    /// and the usage was dropped (the caller reports
    /// [`crate::ErrorCode::KeyboardFull`] instead of acking).
    pub fn down(&mut self, usage: u8) -> bool {
        if (0xE0..=0xE7).contains(&usage) {
            self.modifiers |= 1u8 << u32::from(usage - 0xE0);
            true
        } else if self.keycodes.contains(&usage) {
            true
        } else if let Some(slot) = self.keycodes.iter_mut().find(|slot| **slot == 0) {
            *slot = usage;
            true
        } else {
            false
        }
    }

    /// Releases `usage` (a modifier bit or a regular slot). Releasing
    /// something not held is a no-op.
    pub fn up(&mut self, usage: u8) {
        if (0xE0..=0xE7).contains(&usage) {
            self.modifiers &= !(1u8 << u32::from(usage - 0xE0));
        } else if let Some(slot) = self.keycodes.iter_mut().find(|slot| **slot == usage) {
            *slot = 0;
        }
    }

    /// Releases everything.
    pub fn clear(&mut self) {
        self.modifiers = 0;
        self.keycodes = [0; 6];
    }

    /// The report-ready parts: `(modifier bitmap, keycodes)`.
    #[must_use]
    pub const fn report(&self) -> (u8, [u8; 6]) {
        (self.modifiers, self.keycodes)
    }
}

#[cfg(test)]
mod tests {
    use super::KeyboardState;

    #[test]
    fn down_up_roundtrips_a_regular_usage() {
        let mut kb = KeyboardState::new();
        assert!(kb.down(41));
        assert_eq!(kb.report(), (0, [41, 0, 0, 0, 0, 0]));
        kb.up(41);
        assert_eq!(kb.report(), (0, [0; 6]));
    }

    #[test]
    fn modifiers_use_the_bitmap_not_the_6kro_slots() {
        let mut kb = KeyboardState::new();
        assert!(kb.down(0xE1)); // left ctrl
        assert!(kb.down(0xE5)); // right shift
        assert_eq!(kb.report().0, 0b0010_0010);
        assert_eq!(kb.report().1, [0; 6], "modifiers never occupy 6KRO slots");
        kb.up(0xE1);
        assert_eq!(kb.report().0, 0b0010_0000);
    }

    #[test]
    fn duplicate_down_stays_a_single_slot() {
        let mut kb = KeyboardState::new();
        assert!(kb.down(41));
        assert!(kb.down(41), "re-latching a held usage succeeds");
        assert_eq!(kb.report(), (0, [41, 0, 0, 0, 0, 0]), "one slot, not two");
    }

    #[test]
    fn sixkro_overflow_drops_not_latches() {
        let mut kb = KeyboardState::new();
        for usage in 4..=9 {
            assert!(kb.down(usage), "five slots must fill, usage {usage}");
        }
        assert_eq!(kb.report().1, [4, 5, 6, 7, 8, 9]);
        assert!(!kb.down(10), "the sixth regular usage is dropped");
        assert_eq!(kb.report().1, [4, 5, 6, 7, 8, 9], "dropped, not latched");
    }

    #[test]
    fn up_of_a_released_usage_is_a_noop() {
        let mut kb = KeyboardState::new();
        kb.up(41);
        assert_eq!(kb.report(), (0, [0; 6]));
        kb.down(41);
        kb.up(42);
        assert_eq!(kb.report(), (0, [41, 0, 0, 0, 0, 0]), "only the held usage counts");
    }

    #[test]
    fn clear_releases_everything() {
        let mut kb = KeyboardState::new();
        let _ = kb.down(0xE0);
        let _ = kb.down(4);
        let _ = kb.down(5);
        kb.clear();
        assert_eq!(kb.report(), (0, [0; 6]));
    }
}
