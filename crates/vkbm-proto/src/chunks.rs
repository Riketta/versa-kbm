//! Splitting of wire-sized relative deltas into the 8-bit reports the
//! relative-mouse interface carries.
//!
//! The relative pointer's descriptor declares `-127..=127` per axis, so a
//! full-range [`Command::MouseMoveRel`](crate::Command::MouseMoveRel) is
//! emitted as a paced stream of up to 259 reports. The split is pure
//! arithmetic and lives here so the host test suite covers exactly what the
//! firmware executes.

/// Largest magnitude of one report axis (the descriptor's logical range).
pub const CHUNK_MAX: i32 = 127;

/// Splits `(dx, dy)` into `(i8, i8)` chunks that sum back to exactly the
/// input. A zero delta yields no chunks - and no USB traffic - which still
/// toggles the firmware's pointer routing (see the `MouseMoveRel` handler).
#[must_use]
pub fn plan(dx: i16, dy: i16) -> impl Iterator<Item = (i8, i8)> {
    let mut rest_dx = i32::from(dx);
    let mut rest_dy = i32::from(dy);
    core::iter::from_fn(move || {
        if rest_dx == 0 && rest_dy == 0 {
            return None;
        }
        let step_dx = clamp(rest_dx);
        let step_dy = clamp(rest_dy);
        rest_dx -= i32::from(step_dx);
        rest_dy -= i32::from(step_dy);
        Some((step_dx, step_dy))
    })
}

/// Clamps one axis' remainder into a single report's range.
fn clamp(rest: i32) -> i8 {
    i8::try_from(rest.clamp(-CHUNK_MAX, CHUNK_MAX)).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::{
        CHUNK_MAX,
        plan,
    };

    /// Sums a plan back to `(i32, i32)` and asserts every chunk is in range.
    fn drain(dx: i16, dy: i16) -> (i32, i32, usize) {
        let (mut sx, mut sy, mut n) = (0, 0, 0);
        for (x, y) in plan(dx, dy) {
            assert!(i32::from(x).abs() <= CHUNK_MAX);
            assert!(i32::from(y).abs() <= CHUNK_MAX);
            assert!(x != 0 || y != 0, "no all-zero chunks");
            sx += i32::from(x);
            sy += i32::from(y);
            n += 1;
        }
        (sx, sy, n)
    }

    #[test]
    fn zero_delta_emits_nothing() {
        assert_eq!(drain(0, 0), (0, 0, 0));
    }

    #[test]
    fn unit_deltas_are_single_chunks() {
        assert_eq!(drain(1, -1), (1, -1, 1));
        assert_eq!(drain(127, -127), (127, -127, 1));
    }

    #[test]
    fn extremes_sum_back_exactly() {
        assert_eq!(drain(i16::MAX, 0), (i32::from(i16::MAX), 0, 259));
        assert_eq!(drain(0, i16::MIN), (0, i32::from(i16::MIN), 259));
        assert_eq!(drain(i16::MIN, i16::MAX), (i32::from(i16::MIN), i32::from(i16::MAX), 259));
    }

    #[test]
    fn exact_multiple_of_the_chunk_size_stays_exact() {
        assert_eq!(drain(254, -508), (254, -508, 4));
    }

    #[test]
    fn mixed_signs_and_axes_terminate() {
        assert_eq!(drain(300, -2), (300, -2, 3));
        assert_eq!(drain(-129, 129), (-129, 129, 2));
    }
}
