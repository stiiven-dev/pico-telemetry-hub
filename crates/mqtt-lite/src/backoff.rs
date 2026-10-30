/// Exponential backoff: base, 2·base, 4·base, … capped at `max_ms`.
pub struct Backoff {
    base_ms: u32,
    max_ms: u32,
    attempt: u8,
}

impl Backoff {
    pub const fn new(base_ms: u32, max_ms: u32) -> Self {
        Self {
            base_ms,
            max_ms,
            attempt: 0,
        }
    }

    /// Delay before the next retry: advances the couner.
    pub fn next_delay_ms(&mut self) -> u32 {
        let shift = self.attempt.min(20) as u32; // keeps the shift from overflowing
        let delay = (self.base_ms as u64) << shift;
        self.attempt = self.attempt.saturating_add(1);
        delay.min(self.max_ms as u64) as u32
    }

    pub fn reset(&mut self) {
        self.attempt = 0;
    }
}

/// "Equal jitter": keep half the delay, randomise the other half.
/// `rnd` is any random u32. Delays under 2 ms collapse to 0, which is fine.
pub fn jitter(delay_ms: u32, rnd: u32) -> u32 {
    let half = delay_ms / 2;
    if half == 0 {
        0
    } else {
        half + rnd % (half + 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn doubles_then_caps() {
        let mut b = Backoff::new(1000, 30_000);
        let got: [u32; 7] = core::array::from_fn(|_| b.next_delay_ms());
        assert_eq!(got, [1000, 2000, 4000, 8000, 16_000, 30_000, 30_000]);
    }

    #[test]
    fn reset_starts_over() {
        let mut b = Backoff::new(1000, 30_000);
        for _ in 0..4 {
            b.next_delay_ms();
        }
        b.reset();
        assert_eq!(b.next_delay_ms(), 1000);
    }

    #[test]
    fn never_overflows() {
        let mut b = Backoff::new(u32::MAX, u32::MAX);
        for _ in 0..300 {
            b.next_delay_ms();
        }
        assert_eq!(b.next_delay_ms(), u32::MAX);
    }

    #[test]
    fn jitter_stays_in_upper_half() {
        for rnd in [0, 1, 12_345, u32::MAX] {
            let j = jitter(1000, rnd);
            assert!((500..=1000).contains(&j), "rnd {rnd} gave {j}");
        }
        assert_eq!(jitter(0, 7), 0);
    }
}
