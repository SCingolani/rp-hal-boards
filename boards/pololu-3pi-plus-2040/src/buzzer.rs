// Hacky, stolen from micropython https://github.com/micropython/micropython/blob/094b52b8ad13be1e919354b188ad3dbe94fc4bf5/ports/rp2/machine_pwm.c
pub fn set_freq(pwm: &mut hal::pwm::Slice<hal::pwm::Pwm3, hal::pwm::FreeRunning>, freq: u32) {
    // Returns: floor((16*F + offset) / div16)
    // Avoids overflow in the numerator that would occur if
    //   16*F + offset > 2**32
    //   F + offset/16 > 2**28 = 268435456 (approximately, due to flooring)
    fn get_slice_hz(offset: u32, div16: u32) -> u32 {
        let source_hz = 12_000_000;
        if source_hz + offset / 16 > 268000000 {
            return ((16 * (source_hz as u64) + offset as u64) / div16 as u64) as u32;
        } else {
            return (16 * source_hz + offset) / div16;
        }
    }

    // Returns 16*F / denom, rounded.
    fn get_slice_hz_round(div16: u32) -> u32 {
        return get_slice_hz(div16 / 2, div16);
    }

    // Returns ceil(16*F / denom).
    fn get_slice_hz_ceil(div16: u32) -> u32 {
        return get_slice_hz(div16 - 1, div16);
    }

    const TOP_MAX: u16 = 65534;
    let div16: u32;
    let top: u16;
    let source_hz: u32 = 12_000_000;
    if (source_hz + freq / 2) / freq < TOP_MAX as u32 {
        // If possible (based on the formula for TOP below), use a DIV of 1.
        // This also prevents overflow in the DIV calculation.
        div16 = 16;

        // Same as get_slice_hz_round() below but canceling the 16s
        // to avoid overflow for high freq.
        top = ((source_hz + freq / 2) / freq - 1) as u16;
    } else {
        // Otherwise, choose the smallest possible DIV for maximum
        // duty cycle resolution.
        // Constraint: 16*F/(div16*freq) < TOP_MAX
        // So:
        div16 = get_slice_hz_ceil(TOP_MAX as u32 * freq as u32);

        // Set TOP as accurately as possible using rounding.
        top = (get_slice_hz_round(div16 as u32 * freq as u32) - 1) as u16;
    }
    if div16 < 16 {
        return;
    } else if div16 >= 256 * 16 {
        return;
    }
    pwm.set_div_int(div16 as u8);
    pwm.set_div_frac(16);
    pwm.set_top(top);
}
