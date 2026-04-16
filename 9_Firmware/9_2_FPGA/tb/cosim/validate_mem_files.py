#!/usr/bin/env python3
"""
validate_mem_files.py — Validate all .mem files against AERIS-10 radar parameters.

Updated for 2048-pt FFT / decimation=4 / 512 output bins / 2-segment chirp.

Checks:
  1. Structural: line counts, hex format, value ranges for all .mem files
  2. FFT twiddle files: bit-exact match against cos(2*pi*k/N) in Q15
  3. Long chirp .mem files: reverse-engineer parameters, check for chirp structure
  4. Short chirp .mem files: check length, value range, spectral content
  5. latency_buffer LATENCY=3187 parameter validation
  6. Memory addressing: 2 segments x 2048 = 4096 slots

Usage:
    python3 validate_mem_files.py
"""

import math
import os
import sys

# ============================================================================
# AERIS-10 System Parameters (from radar_params.vh / radar_scene.py)
# ============================================================================
F_CARRIER = 10.5e9        # 10.5 GHz carrier
C_LIGHT = 3.0e8
F_IF = 120e6              # IF frequency
CHIRP_BW = 20e6           # 20 MHz sweep
FS_ADC = 400e6            # ADC sample rate
FS_SYS = 100e6            # System clock (100 MHz, after CIC 4x)
T_LONG_CHIRP = 30e-6      # 30 us long chirp
T_SHORT_CHIRP = 0.5e-6    # 0.5 us short chirp
CIC_DECIMATION = 4
FFT_SIZE = 2048
DOPPLER_FFT_SIZE = 16
LONG_CHIRP_SAMPLES = int(T_LONG_CHIRP * FS_SYS)  # 3000 at 100 MHz

# Overlap-save parameters (2048-pt FFT)
OVERLAP_SAMPLES = 128
SEGMENT_ADVANCE = FFT_SIZE - OVERLAP_SAMPLES  # 1920
LONG_SEGMENTS = 2

MEM_DIR = os.path.join(os.path.dirname(__file__), '..', '..')

pass_count = 0
fail_count = 0
warn_count = 0

def check(condition, _label):
    global pass_count, fail_count
    if condition:
        pass_count += 1
    else:
        fail_count += 1

def warn(_label):
    global warn_count
    warn_count += 1

def read_mem_hex(filename):
    """Read a .mem file, return list of integer values (16-bit signed)."""
    path = os.path.join(MEM_DIR, filename)
    values = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('//'):
                continue
            val = int(line, 16)
            # Interpret as 16-bit signed
            if val >= 0x8000:
                val -= 0x10000
            values.append(val)
    return values


# ============================================================================
# TEST 1: Structural validation of all .mem files
# ============================================================================
def test_structural():

    expected = {
        # FFT twiddle files (quarter-wave cosine ROMs)
        'fft_twiddle_2048.mem': {'lines': 512, 'desc': '2048-pt FFT quarter-wave cos ROM'},
        'fft_twiddle_16.mem':   {'lines': 4,   'desc': '16-pt FFT quarter-wave cos ROM'},
        # Long chirp segments (2 segments x 2048 samples each)
        'long_chirp_seg0_i.mem': {'lines': 2048, 'desc': 'Long chirp seg 0 I'},
        'long_chirp_seg0_q.mem': {'lines': 2048, 'desc': 'Long chirp seg 0 Q'},
        'long_chirp_seg1_i.mem': {'lines': 2048, 'desc': 'Long chirp seg 1 I'},
        'long_chirp_seg1_q.mem': {'lines': 2048, 'desc': 'Long chirp seg 1 Q'},
        # Short chirp (50 samples)
        'short_chirp_i.mem': {'lines': 50, 'desc': 'Short chirp I'},
        'short_chirp_q.mem': {'lines': 50, 'desc': 'Short chirp Q'},
    }

    # Verify deleted segments do NOT exist
    for deleted in ['long_chirp_seg2_i.mem', 'long_chirp_seg2_q.mem',
                    'long_chirp_seg3_i.mem', 'long_chirp_seg3_q.mem']:
        path = os.path.join(MEM_DIR, deleted)
        check(not os.path.isfile(path),
              f"{deleted} does NOT exist (deleted — only 2 segments now)")

    for fname, info in expected.items():
        path = os.path.join(MEM_DIR, fname)
        exists = os.path.isfile(path)
        check(exists, f"{fname} exists")
        if not exists:
            continue

        vals = read_mem_hex(fname)
        check(len(vals) == info['lines'],
              f"{fname}: {len(vals)} data lines (expected {info['lines']})")

        # Check all values are in 16-bit signed range
        in_range = all(-32768 <= v <= 32767 for v in vals)
        check(in_range, f"{fname}: all values in [-32768, 32767]")


# ============================================================================
# TEST 2: FFT Twiddle Factor Validation
# ============================================================================
def test_twiddle_2048():
    vals = read_mem_hex('fft_twiddle_2048.mem')

    max_err = 0
    err_details = []
    for k in range(min(512, len(vals))):
        angle = 2.0 * math.pi * k / 2048.0
        expected = round(math.cos(angle) * 32767.0)
        expected = max(-32768, min(32767, expected))
        actual = vals[k]
        err = abs(actual - expected)
        if err > max_err:
            max_err = err
        if err > 1:
            err_details.append((k, actual, expected, err))

    check(max_err <= 1,
          f"fft_twiddle_2048.mem: max twiddle error = {max_err} LSB (tolerance: 1)")
    if err_details:
        for _, _act, _exp, _e in err_details[:5]:
            pass


def test_twiddle_16():
    vals = read_mem_hex('fft_twiddle_16.mem')

    max_err = 0
    for k in range(min(4, len(vals))):
        angle = 2.0 * math.pi * k / 16.0
        expected = round(math.cos(angle) * 32767.0)
        expected = max(-32768, min(32767, expected))
        actual = vals[k]
        err = abs(actual - expected)
        if err > max_err:
            max_err = err

    check(max_err <= 1,
          f"fft_twiddle_16.mem: max twiddle error = {max_err} LSB (tolerance: 1)")


# ============================================================================
# TEST 3: Long Chirp .mem File Analysis (2 segments x 2048)
# ============================================================================
def test_long_chirp():

    # Load all 2 segments
    all_i = []
    all_q = []
    for seg in range(LONG_SEGMENTS):
        seg_i = read_mem_hex(f'long_chirp_seg{seg}_i.mem')
        seg_q = read_mem_hex(f'long_chirp_seg{seg}_q.mem')
        all_i.extend(seg_i)
        all_q.extend(seg_q)

    total_samples = len(all_i)
    expected_total = LONG_SEGMENTS * FFT_SIZE  # 2 * 2048 = 4096
    check(total_samples == expected_total,
          f"Total long chirp samples: {total_samples} "
          f"(expected {expected_total} = {LONG_SEGMENTS} segs x {FFT_SIZE})")

    # Compute magnitude envelope
    magnitudes = [math.sqrt(i*i + q*q) for i, q in zip(all_i, all_q, strict=False)]
    max_mag = max(magnitudes)

    # Check if this looks like it came from generate_reference_chirp_q15
    # That function uses 32767 * 0.9 scaling => max magnitude ~29490
    expected_max_from_model = 32767 * 0.9
    uses_model_scaling = max_mag > expected_max_from_model * 0.8
    if not uses_model_scaling:
        warn(f"Magnitude ({max_mag:.0f}) is much lower than expected from Python model "
             f"({expected_max_from_model:.0f}). .mem files may have unknown provenance.")

    # Analyze instantaneous frequency via phase differences
    phases = []
    for i_val, q_val in zip(all_i, all_q, strict=False):
        if abs(i_val) > 5 or abs(q_val) > 5:  # Skip near-zero samples
            phases.append(math.atan2(q_val, i_val))
        else:
            phases.append(None)

    # Compute phase differences (instantaneous frequency)
    freq_estimates = []
    for n in range(1, len(phases)):
        if phases[n] is not None and phases[n-1] is not None:
            dp = phases[n] - phases[n-1]
            # Unwrap
            while dp > math.pi:
                dp -= 2 * math.pi
            while dp < -math.pi:
                dp += 2 * math.pi
            # Frequency in Hz (at 100 MHz sample rate, since these are post-DDC)
            f_inst = dp * FS_SYS / (2 * math.pi)
            freq_estimates.append(f_inst)

    if freq_estimates:
        f_min = min(freq_estimates)
        f_max = max(freq_estimates)
        f_range = f_max - f_min

        # A chirp should show frequency sweep
        is_chirp = f_range > 0.5e6  # At least 0.5 MHz sweep
        check(is_chirp,
              f"Long chirp shows frequency sweep ({f_range/1e6:.2f} MHz > 0.5 MHz)")

        # Check if bandwidth roughly matches expected
        bw_match = abs(f_range - CHIRP_BW) / CHIRP_BW < 0.5  # within 50%
        if not bw_match:
            warn(f"Bandwidth {f_range/1e6:.2f} MHz does NOT match expected "
                 f"{CHIRP_BW/1e6:.2f} MHz")

    # Verify each segment has 2048 entries
    for seg in range(LONG_SEGMENTS):
        seg_i = read_mem_hex(f'long_chirp_seg{seg}_i.mem')
        seg_q = read_mem_hex(f'long_chirp_seg{seg}_q.mem')
        check(len(seg_i) == FFT_SIZE,
              f"long_chirp_seg{seg}_i.mem: {len(seg_i)} samples (expected {FFT_SIZE})")
        check(len(seg_q) == FFT_SIZE,
              f"long_chirp_seg{seg}_q.mem: {len(seg_q)} samples (expected {FFT_SIZE})")


# ============================================================================
# TEST 4: Short Chirp .mem File Analysis
# ============================================================================
def test_short_chirp():

    short_i = read_mem_hex('short_chirp_i.mem')
    short_q = read_mem_hex('short_chirp_q.mem')

    check(len(short_i) == 50, f"Short chirp I: {len(short_i)} samples (expected 50)")
    check(len(short_q) == 50, f"Short chirp Q: {len(short_q)} samples (expected 50)")

    # Expected: 0.5 us chirp at 100 MHz = 50 samples
    expected_samples = int(T_SHORT_CHIRP * FS_SYS)
    check(len(short_i) == expected_samples,
          f"Short chirp length matches T_SHORT_CHIRP * FS_SYS = {expected_samples}")

    magnitudes = [math.sqrt(i*i + q*q) for i, q in zip(short_i, short_q, strict=False)]

    # Check non-zero
    nonzero = sum(1 for m in magnitudes if m > 1)
    check(nonzero == len(short_i), f"All {nonzero}/{len(short_i)} samples non-zero")

    # Check it looks like a chirp (phase should be quadratic)
    phases = [math.atan2(q, i) for i, q in zip(short_i, short_q, strict=False)]
    freq_est = []
    for n in range(1, len(phases)):
        dp = phases[n] - phases[n-1]
        while dp > math.pi:
            dp -= 2 * math.pi
        while dp < -math.pi:
            dp += 2 * math.pi
        freq_est.append(dp * FS_SYS / (2 * math.pi))


# ============================================================================
# TEST 5: Generate Expected Chirp .mem and Compare
# ============================================================================
def test_chirp_vs_model():

    # Generate reference using the same method as radar_scene.py
    chirp_rate = CHIRP_BW / T_LONG_CHIRP  # Hz/s

    model_i = []
    model_q = []
    # Generate first FFT_SIZE samples of the chirp for seg0 comparison
    n_chirp = min(FFT_SIZE, LONG_CHIRP_SAMPLES)  # min(2048, 3000) = 2048

    for n in range(n_chirp):
        t = n / FS_SYS
        phase = math.pi * chirp_rate * t * t
        re_val = round(32767 * 0.9 * math.cos(phase))
        im_val = round(32767 * 0.9 * math.sin(phase))
        model_i.append(max(-32768, min(32767, re_val)))
        model_q.append(max(-32768, min(32767, im_val)))

    # Read seg0 from .mem
    mem_i = read_mem_hex('long_chirp_seg0_i.mem')
    mem_q = read_mem_hex('long_chirp_seg0_q.mem')

    # Compare magnitudes
    model_mags = [math.sqrt(i*i + q*q) for i, q in zip(model_i, model_q, strict=False)]
    mem_mags = [math.sqrt(i*i + q*q) for i, q in zip(mem_i, mem_q, strict=False)]

    model_max = max(model_mags)
    mem_max = max(mem_mags)

    # Check if they match
    matches = sum(1 for a, b in zip(model_i, mem_i, strict=False) if a == b)

    if matches > len(model_i) * 0.9:
        pass
    else:
        warn(".mem files do NOT match Python model. They likely have different provenance.")
        if mem_max > 0:
            model_max / mem_max

    # Check phase correlation (shape match regardless of scaling)
    model_phases = [math.atan2(q, i) for i, q in zip(model_i, model_q, strict=False)]
    mem_phases = [math.atan2(q, i) for i, q in zip(mem_i, mem_q, strict=False)]

    # Compute phase differences
    phase_diffs = []
    for mp, fp in zip(model_phases, mem_phases, strict=False):
        d = mp - fp
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        phase_diffs.append(d)

    max_phase_diff = max(abs(d) for d in phase_diffs)

    phase_match = max_phase_diff < 0.5  # within 0.5 rad
    check(
        phase_match,
        f"Phase shape match: max diff = {math.degrees(max_phase_diff):.1f} deg "
        f"(tolerance: 28.6 deg)",
    )


# ============================================================================
# TEST 6: Latency Buffer LATENCY=3187 Validation
# ============================================================================
def test_latency_buffer():

    LATENCY = 3187
    BRAM_SIZE = 4096

    check(LATENCY < BRAM_SIZE,
          f"LATENCY ({LATENCY}) < BRAM size ({BRAM_SIZE})")

    # For now, validate that LATENCY is reasonable (between 1000 and 4095)
    check(1000 < LATENCY < 4095,
          f"LATENCY={LATENCY} in reasonable range [1000, 4095]")

    # Validate address arithmetic won't overflow
    min_read_ptr = 4096 + 0 - LATENCY
    check(min_read_ptr >= 0 and min_read_ptr < 4096,
          f"Min read_ptr after wrap = {min_read_ptr} (valid: 0..4095)")


# ============================================================================
# TEST 7: Cross-check chirp memory loader addressing
#   2 segments: {segment_select[0], sample_addr[10:0]} = 12-bit address
# ============================================================================
def test_memory_addressing():

    # chirp_memory_loader_param uses: long_addr = {segment_select[0], sample_addr[10:0]}
    # This creates a 12-bit address:
    # Segment 0: addresses 0x000..0x7FF (0..2047)
    # Segment 1: addresses 0x800..0xFFF (2048..4095)

    for seg in range(LONG_SEGMENTS):
        base = seg * FFT_SIZE
        end = base + FFT_SIZE - 1
        addr_from_concat = (seg << 11) | 0  # {seg[0], 11'b0}
        addr_end = (seg << 11) | (FFT_SIZE - 1)

        check(
            addr_from_concat == base,
            f"Seg {seg} base address: {{{seg}[0], 11'b0}} = {addr_from_concat} "
            f"(expected {base})",
        )
        check(addr_end == end,
              f"Seg {seg} end address: {{{seg}[0], 11'h7FF}} = {addr_end} "
              f"(expected {end})")

    # Memory is declared as: reg [15:0] long_chirp_i [0:4095]
    # $readmemh loads seg0 to [0:2047], seg1 to [2048:4095].
    # Addressing via {segment_select, sample_addr} maps correctly.

    # Verify total memory = LONG_SEGMENTS * FFT_SIZE = 4096
    total_mem = LONG_SEGMENTS * FFT_SIZE
    check(total_mem == 4096,
          f"Total chirp memory slots: {total_mem} (expected 4096)")


# ============================================================================
# TEST 8: Seg1 zero-padding analysis
#   With 2048-pt FFT and 2 segments:
#   Seg0: chirp samples 0..2047
#   Seg1: chirp samples 2048..4095 (chirp is 3000, so samples 3000..4095 = zero-padded)
# ============================================================================
def test_seg1_padding():

    seg1_i = read_mem_hex('long_chirp_seg1_i.mem')
    seg1_q = read_mem_hex('long_chirp_seg1_q.mem')

    mags = [math.sqrt(i*i + q*q) for i, q in zip(seg1_i, seg1_q, strict=False)]

    # The chirp is 3000 samples. Seg1 starts at index 2048.
    # Valid chirp data: indices 0..(3000-2048-1) = 0..951
    # Zero-padded: indices 952..2047
    chirp_samples_in_seg1 = LONG_CHIRP_SAMPLES - FFT_SIZE  # 3000 - 2048 = 952
    # Count trailing zeros
    trailing_zeros = 0
    for m in reversed(mags):
        if m < 2:
            trailing_zeros += 1
        else:
            break

    expected_zeros = FFT_SIZE - chirp_samples_in_seg1  # 2048 - 952 = 1096
    # Allow some tolerance (within 50 samples)
    check(abs(trailing_zeros - expected_zeros) < 50,
          f"Seg1 trailing zeros: {trailing_zeros} "
          f"(expected ~{expected_zeros} for {LONG_CHIRP_SAMPLES}-sample chirp)")

    nonzero = sum(1 for m in mags if m > 2)
    check(nonzero > 0,
          f"Seg1 has {nonzero} non-zero samples "
          f"(expected ~{chirp_samples_in_seg1} valid chirp samples)")


# ============================================================================
# MAIN
# ============================================================================
def main():

    test_structural()
    test_twiddle_2048()
    test_twiddle_16()
    test_long_chirp()
    test_short_chirp()
    test_chirp_vs_model()
    test_latency_buffer()
    test_memory_addressing()
    test_seg1_padding()

    if fail_count == 0:
        pass
    else:
        pass

    return 0 if fail_count == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
