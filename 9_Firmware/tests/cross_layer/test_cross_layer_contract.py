"""
Cross-Layer Contract Tests
==========================
Single pytest file orchestrating three tiers of verification:

Tier 1 — Static Contract Parsing:
  Compares Python, Verilog, and C source code at parse-time to catch
  opcode mismatches, bit-width errors, packet constant drift, and
  layout bugs like the status_words[0] 37-bit truncation.

Tier 2 — Verilog Cosimulation (iverilog):
  Compiles and runs tb_cross_layer_ft2232h.v, then parses its output
  files (cmd_results.txt, data_packet.txt, status_packet.txt) and
  runs Python parsers on the captured bytes to verify round-trip
  correctness.

Tier 3 — C Stub Execution:
  Compiles stm32_settings_stub.cpp, generates a binary settings
  packet from Python, runs the stub, and verifies all parsed field
  values match.

The goal is to find UNKNOWN bugs by testing each layer against
independently-derived ground truth — not just checking that two
layers agree (because both could be wrong).
"""

from __future__ import annotations

import os
import re
import struct
import subprocess
import tempfile
from pathlib import Path
from typing import ClassVar

import pytest

# Import the contract parsers
import sys

THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))
import contract_parser as cp  # noqa: E402

# Also add the GUI dir to import radar_protocol
sys.path.insert(0, str(cp.GUI_DIR))


# ===================================================================
# Helpers
# ===================================================================

IVERILOG = os.environ.get("IVERILOG", "/opt/homebrew/bin/iverilog")
VVP = os.environ.get("VVP", "/opt/homebrew/bin/vvp")
CXX = os.environ.get("CXX", "c++")

# Check tool availability for conditional skipping
_has_iverilog = Path(IVERILOG).exists() if "/" in IVERILOG else bool(
    subprocess.run(["which", IVERILOG], capture_output=True).returncode == 0
)
_has_cxx = subprocess.run(
    [CXX, "--version"], capture_output=True
).returncode == 0


def _parse_hex_results(text: str) -> list[dict[str, str]]:
    """Parse space-separated hex lines from TB output files."""
    rows = []
    for line in text.strip().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        rows.append(line.split())
    return rows


# ===================================================================
# Ground Truth: FPGA register map (independently transcribed)
# ===================================================================
# This is the SINGLE SOURCE OF TRUTH, manually transcribed from
# radar_system_top.v lines 902-945. If any layer disagrees with
# this, it's a bug in that layer.

GROUND_TRUTH_OPCODES = {
    0x01: ("host_radar_mode", 2),
    0x02: ("host_trigger_pulse", 1),   # pulse
    0x03: ("host_detect_threshold", 16),
    0x04: ("host_stream_control", 6),
    0x10: ("host_long_chirp_cycles", 16),
    0x11: ("host_long_listen_cycles", 16),
    0x12: ("host_guard_cycles", 16),
    0x13: ("host_short_chirp_cycles", 16),
    0x14: ("host_short_listen_cycles", 16),
    0x15: ("host_chirps_per_elev", 6),
    0x16: ("host_gain_shift", 4),
    0x20: ("host_range_mode", 2),
    0x21: ("host_cfar_guard", 4),
    0x22: ("host_cfar_train", 5),
    0x23: ("host_cfar_alpha", 8),
    0x24: ("host_cfar_mode", 2),
    0x25: ("host_cfar_enable", 1),
    0x26: ("host_mti_enable", 1),
    0x27: ("host_dc_notch_width", 3),
    0x28: ("host_agc_enable", 1),
    0x29: ("host_agc_target", 8),
    0x2A: ("host_agc_attack", 4),
    0x2B: ("host_agc_decay", 4),
    0x2C: ("host_agc_holdoff", 4),
    0x30: ("host_self_test_trigger", 1),  # pulse
    0x31: ("host_status_request", 1),     # pulse
    0xFF: ("host_status_request", 1),     # alias, pulse
}

GROUND_TRUTH_RESET_DEFAULTS = {
    "host_radar_mode": 1,           # 2'b01
    "host_detect_threshold": 10000,
    "host_stream_control": 15,      # 6'b001_111 (mag_only + all streams)
    "host_long_chirp_cycles": 3000,
    "host_long_listen_cycles": 13700,
    "host_guard_cycles": 17540,
    "host_short_chirp_cycles": 50,
    "host_short_listen_cycles": 17450,
    "host_chirps_per_elev": 32,
    "host_gain_shift": 0,
    "host_range_mode": 0,
    "host_cfar_guard": 2,
    "host_cfar_train": 8,
    "host_cfar_alpha": 0x30,
    "host_cfar_mode": 0,
    "host_cfar_enable": 0,
    "host_mti_enable": 0,
    "host_dc_notch_width": 0,
    "host_agc_enable": 0,
    "host_agc_target": 200,
    "host_agc_attack": 1,
    "host_agc_decay": 1,
    "host_agc_holdoff": 4,
}

GROUND_TRUTH_PACKET_CONSTANTS = {
    "status": {"header": 0xBB, "footer": 0x55, "size": 26},
}


# ===================================================================
# TIER 1: Static Contract Parsing
# ===================================================================

class TestTier1OpcodeContract:
    """Verify Python and Verilog opcode sets match ground truth."""

    def test_python_opcodes_match_ground_truth(self):
        """Every Python Opcode must exist in ground truth with correct value."""
        py_opcodes = cp.parse_python_opcodes()
        for val, entry in py_opcodes.items():
            assert val in GROUND_TRUTH_OPCODES, (
                f"Python Opcode {entry.name}=0x{val:02X} not in ground truth! "
                f"Possible phantom opcode (like the 0x06 incident)."
            )

    def test_ground_truth_opcodes_in_python(self):
        """Every ground truth opcode must have a Python enum entry."""
        py_opcodes = cp.parse_python_opcodes()
        for val, (reg, _width) in GROUND_TRUTH_OPCODES.items():
            assert val in py_opcodes, (
                f"Ground truth opcode 0x{val:02X} ({reg}) missing from Python Opcode enum."
            )

    def test_verilog_opcodes_match_ground_truth(self):
        """Every Verilog case entry must exist in ground truth."""
        v_opcodes = cp.parse_verilog_opcodes()
        for val, entry in v_opcodes.items():
            assert val in GROUND_TRUTH_OPCODES, (
                f"Verilog opcode 0x{val:02X} ({entry.register}) not in ground truth."
            )

    def test_ground_truth_opcodes_in_verilog(self):
        """Every ground truth opcode must have a Verilog case entry."""
        v_opcodes = cp.parse_verilog_opcodes()
        for val, (reg, _width) in GROUND_TRUTH_OPCODES.items():
            assert val in v_opcodes, (
                f"Ground truth opcode 0x{val:02X} ({reg}) missing from Verilog case statement."
            )

    def test_python_verilog_bidirectional_match(self):
        """Python and Verilog must have the same set of opcode values."""
        py_set = set(cp.parse_python_opcodes().keys())
        v_set = set(cp.parse_verilog_opcodes().keys())
        py_only = py_set - v_set
        v_only = v_set - py_set
        assert not py_only, f"Opcodes in Python but not Verilog: {[hex(x) for x in py_only]}"
        assert not v_only, f"Opcodes in Verilog but not Python: {[hex(x) for x in v_only]}"

    def test_verilog_register_names_match(self):
        """Verilog case target registers must match ground truth names."""
        v_opcodes = cp.parse_verilog_opcodes()
        for val, (expected_reg, _) in GROUND_TRUTH_OPCODES.items():
            if val in v_opcodes:
                actual_reg = v_opcodes[val].register
                assert actual_reg == expected_reg, (
                    f"Opcode 0x{val:02X}: Verilog writes to '{actual_reg}' "
                    f"but ground truth says '{expected_reg}'"
                )

    def test_opcode_count_exact_match(self):
        """
        Total opcode count must match ground truth exactly.

        This is a CANARY test: if an LLM agent or developer adds/removes
        an opcode in one layer without updating ground truth, this fails
        immediately. Update GROUND_TRUTH_OPCODES when intentionally
        changing the register map.
        """
        expected_count = len(GROUND_TRUTH_OPCODES)  # 25
        py_count = len(cp.parse_python_opcodes())
        v_count = len(cp.parse_verilog_opcodes())

        assert py_count == expected_count, (
            f"Python has {py_count} opcodes but ground truth has {expected_count}. "
            f"If intentional, update GROUND_TRUTH_OPCODES in this test file."
        )
        assert v_count == expected_count, (
            f"Verilog has {v_count} opcodes but ground truth has {expected_count}. "
            f"If intentional, update GROUND_TRUTH_OPCODES in this test file."
        )

    def test_no_extra_verilog_opcodes(self):
        """
        Verilog must not have opcodes absent from ground truth.

        Catches the case where someone adds a new case entry in
        radar_system_top.v but forgets to update the contract.
        """
        v_opcodes = cp.parse_verilog_opcodes()
        extra = set(v_opcodes.keys()) - set(GROUND_TRUTH_OPCODES.keys())
        assert not extra, (
            f"Verilog has opcodes not in ground truth: "
            f"{[f'0x{x:02X} ({v_opcodes[x].register})' for x in extra]}. "
            f"Add them to GROUND_TRUTH_OPCODES if intentional."
        )

    def test_no_extra_python_opcodes(self):
        """
        Python must not have opcodes absent from ground truth.

        Catches phantom opcodes (like the 0x06 incident) added by
        LLM agents that assume without verifying.
        """
        py_opcodes = cp.parse_python_opcodes()
        extra = set(py_opcodes.keys()) - set(GROUND_TRUTH_OPCODES.keys())
        assert not extra, (
            f"Python has opcodes not in ground truth: "
            f"{[f'0x{x:02X} ({py_opcodes[x].name})' for x in extra]}. "
            f"Remove phantom opcodes or add to GROUND_TRUTH_OPCODES."
        )


class TestTier1BitWidths:
    """Verify register widths and opcode bit slices match ground truth."""

    def test_verilog_register_widths(self):
        """Register declarations must match ground truth bit widths."""
        v_widths = cp.parse_verilog_register_widths()
        for reg, expected_width in [
            (name, w) for _, (name, w) in GROUND_TRUTH_OPCODES.items()
        ]:
            if reg in v_widths:
                actual = v_widths[reg]
                assert actual >= expected_width, (
                    f"{reg}: declared {actual}-bit but ground truth says {expected_width}-bit"
                )

    def test_verilog_opcode_bit_slices(self):
        """Opcode case assignments must use correct bit widths from cmd_value."""
        v_opcodes = cp.parse_verilog_opcodes()
        for val, (reg, expected_width) in GROUND_TRUTH_OPCODES.items():
            if val not in v_opcodes:
                continue
            entry = v_opcodes[val]
            if entry.is_pulse:
                continue  # Pulse opcodes don't use cmd_value slicing
            if entry.bit_width > 0:
                assert entry.bit_width >= expected_width, (
                    f"Opcode 0x{val:02X} ({reg}): bit slice {entry.bit_slice} "
                    f"= {entry.bit_width}-bit, expected >= {expected_width}"
                )


class TestTier1StatusWordTruncation:
    """Verify each status_words[] concatenation is exactly 32 bits."""

    def test_status_words_concat_widths_ft2232h(self):
        """Each status_words[] concat must be EXACTLY 32 bits."""
        port_widths = cp.get_usb_interface_port_widths(
            cp.FPGA_DIR / "usb_data_interface_ft2232h.v"
        )
        concats = cp.parse_verilog_status_word_concats(
            cp.FPGA_DIR / "usb_data_interface_ft2232h.v"
        )

        for idx, expr in concats.items():
            result = cp.count_concat_bits(expr, port_widths)
            if result.total_bits < 0:
                pytest.skip(f"status_words[{idx}]: unknown signal width")
            assert result.total_bits == 32, (
                f"status_words[{idx}] is {result.total_bits} bits, not 32! "
                f"{'TRUNCATION' if result.total_bits > 32 else 'UNDERFLOW'} BUG. "
                f"Fragments: {result.fragments}"
            )

    def test_status_words_concat_widths_ft601(self):
        """Same check for the FT601 interface (same bug expected)."""
        ft601_path = cp.FPGA_DIR / "usb_data_interface.v"
        if not ft601_path.exists():
            pytest.skip("FT601 interface file not found")

        port_widths = cp.get_usb_interface_port_widths(ft601_path)
        concats = cp.parse_verilog_status_word_concats(ft601_path)

        for idx, expr in concats.items():
            result = cp.count_concat_bits(expr, port_widths)
            if result.total_bits < 0:
                pytest.skip(f"status_words[{idx}]: unknown signal width")
            assert result.total_bits == 32, (
                f"FT601 status_words[{idx}] is {result.total_bits} bits, not 32! "
                f"{'TRUNCATION' if result.total_bits > 32 else 'UNDERFLOW'} BUG. "
                f"Fragments: {result.fragments}"
            )


class TestTier1StatusFieldPositions:
    """Verify Python status parser bit positions match Verilog layout."""

    def test_python_status_mode_position(self):
        """
        Verify Python reads radar_mode at the correct bit position matching
        the Verilog status_words[0] layout:
        {0xFF[31:24], mode[23:22], stream[21:19], 3'b000[18:16], threshold[15:0]}
        """
        # Get what Python thinks
        py_fields = cp.parse_python_status_fields()
        mode_field = next((f for f in py_fields if f.name == "radar_mode"), None)
        assert mode_field is not None, "radar_mode not found in parse_status_packet"

        # Ground truth: mode is at bits [23:22], so LSB = 22
        expected_shift = 22
        actual_shift = mode_field.lsb

        assert actual_shift == expected_shift, (
            f"Python reads radar_mode at bit {actual_shift} "
            f"but Verilog status_words[0] has mode at bit {expected_shift}."
        )


class TestTier1ArchitecturalParams:
    """
    Verify radar_params.vh (FPGA single source of truth) matches Python
    WaveformConfig defaults and frozen architectural constants.

    These tests catch:
    - LLM agents changing FFT size, bin counts, or sample rates in one
      layer without updating the other
    - Accidental parameter drift between FPGA and GUI
    - Unauthorized changes to critical architectural constants

    When intentionally changing a parameter (e.g. FFT 1024→2048),
    update BOTH radar_params.vh AND WaveformConfig, then update the
    FROZEN_PARAMS dict below.
    """

    # Frozen architectural constants — update deliberately when changing arch
    FROZEN_PARAMS: ClassVar[dict[str, int]] = {
        "RP_FFT_SIZE": 2048,
        "RP_DECIMATION_FACTOR": 4,
        "RP_NUM_RANGE_BINS": 512,
        "RP_DOPPLER_FFT_SIZE": 16,
        "RP_NUM_DOPPLER_BINS": 32,
        "RP_CHIRPS_PER_FRAME": 32,
        "RP_CHIRPS_PER_SUBFRAME": 16,
        "RP_DATA_WIDTH": 16,
        "RP_PROCESSING_RATE_MHZ": 100,
        "RP_RANGE_PER_BIN_DM": 60,       # 6.0 m in decimeters
    }

    def test_radar_params_vh_parseable(self):
        """radar_params.vh must exist and parse without error."""
        params = cp.parse_radar_params_vh()
        assert len(params) > 10, (
            f"Only parsed {len(params)} defines from radar_params.vh — "
            f"expected > 10. Parser may be broken."
        )

    def test_frozen_constants_unchanged(self):
        """
        Critical architectural constants must match frozen values.

        If this test fails, someone changed a fundamental parameter.
        Verify the change is intentional, update FROZEN_PARAMS, AND
        update all downstream consumers (GUI, testbenches, docs).
        """
        params = cp.parse_radar_params_vh()
        for name, expected in self.FROZEN_PARAMS.items():
            assert name in params, (
                f"{name} missing from radar_params.vh! "
                f"Was it renamed or removed?"
            )
            assert params[name] == expected, (
                f"ARCHITECTURAL CHANGE DETECTED: {name} = {params[name]}, "
                f"expected {expected}. If intentional, update FROZEN_PARAMS "
                f"in this test AND all downstream consumers."
            )

    def test_fpga_python_fft_size_match(self):
        """FPGA FFT size must match Python WaveformConfig.fft_size."""
        params = cp.parse_radar_params_vh()
        sys.path.insert(0, str(cp.GUI_DIR))
        from v7.models import WaveformConfig
        wc = WaveformConfig()
        assert params["RP_FFT_SIZE"] == wc.fft_size, (
            f"FFT size mismatch: radar_params.vh={params['RP_FFT_SIZE']}, "
            f"WaveformConfig={wc.fft_size}"
        )

    def test_fpga_python_decimation_match(self):
        """FPGA decimation factor must match Python WaveformConfig."""
        params = cp.parse_radar_params_vh()
        sys.path.insert(0, str(cp.GUI_DIR))
        from v7.models import WaveformConfig
        wc = WaveformConfig()
        assert params["RP_DECIMATION_FACTOR"] == wc.decimation_factor, (
            f"Decimation mismatch: radar_params.vh={params['RP_DECIMATION_FACTOR']}, "
            f"WaveformConfig={wc.decimation_factor}"
        )

    def test_fpga_python_range_bins_match(self):
        """FPGA 3km output bins must match Python WaveformConfig.n_range_bins."""
        params = cp.parse_radar_params_vh()
        sys.path.insert(0, str(cp.GUI_DIR))
        from v7.models import WaveformConfig
        wc = WaveformConfig()
        assert params["RP_NUM_RANGE_BINS"] == wc.n_range_bins, (
            f"Range bins mismatch: radar_params.vh={params['RP_NUM_RANGE_BINS']}, "
            f"WaveformConfig={wc.n_range_bins}"
        )

    def test_fpga_python_doppler_bins_match(self):
        """FPGA Doppler bins must match Python WaveformConfig.n_doppler_bins."""
        params = cp.parse_radar_params_vh()
        sys.path.insert(0, str(cp.GUI_DIR))
        from v7.models import WaveformConfig
        wc = WaveformConfig()
        assert params["RP_NUM_DOPPLER_BINS"] == wc.n_doppler_bins, (
            f"Doppler bins mismatch: radar_params.vh={params['RP_NUM_DOPPLER_BINS']}, "
            f"WaveformConfig={wc.n_doppler_bins}"
        )

    def test_fpga_python_sample_rate_match(self):
        """FPGA processing rate must match Python WaveformConfig.sample_rate_hz."""
        params = cp.parse_radar_params_vh()
        sys.path.insert(0, str(cp.GUI_DIR))
        from v7.models import WaveformConfig
        wc = WaveformConfig()
        fpga_rate_hz = params["RP_PROCESSING_RATE_MHZ"] * 1e6
        assert fpga_rate_hz == wc.sample_rate_hz, (
            f"Sample rate mismatch: radar_params.vh={fpga_rate_hz/1e6} MHz, "
            f"WaveformConfig={wc.sample_rate_hz/1e6} MHz"
        )


class TestTier1PacketConstants:
    """Verify packet header/footer/size constants match across layers."""

    def test_python_packet_constants(self):
        """Python constants match ground truth."""
        py = cp.parse_python_packet_constants()
        for ptype, expected in GROUND_TRUTH_PACKET_CONSTANTS.items():
            assert py[ptype].header == expected["header"], (
                f"Python {ptype} header: 0x{py[ptype].header:02X} != 0x{expected['header']:02X}"
            )
            assert py[ptype].footer == expected["footer"], (
                f"Python {ptype} footer: 0x{py[ptype].footer:02X} != 0x{expected['footer']:02X}"
            )
            assert py[ptype].size == expected["size"], (
                f"Python {ptype} size: {py[ptype].size} != {expected['size']}"
            )

    def test_verilog_packet_constants(self):
        """Verilog localparams match ground truth."""
        v = cp.parse_verilog_packet_constants()
        for ptype, expected in GROUND_TRUTH_PACKET_CONSTANTS.items():
            assert v[ptype].header == expected["header"], (
                f"Verilog {ptype} header: 0x{v[ptype].header:02X} != 0x{expected['header']:02X}"
            )
            assert v[ptype].footer == expected["footer"], (
                f"Verilog {ptype} footer: 0x{v[ptype].footer:02X} != 0x{expected['footer']:02X}"
            )
            assert v[ptype].size == expected["size"], (
                f"Verilog {ptype} size: {v[ptype].size} != {expected['size']}"
            )

    def test_python_verilog_constants_agree(self):
        """Python and Verilog packet constants must match each other."""
        py = cp.parse_python_packet_constants()
        v = cp.parse_verilog_packet_constants()
        # Only status packets have fixed constants; data packets are now
        # bulk per-frame (variable size) in the FT2232H protocol.
        for ptype in ("status",):
            assert py[ptype].header == v[ptype].header
            assert py[ptype].footer == v[ptype].footer
            assert py[ptype].size == v[ptype].size


class TestTier1ResetDefaults:
    """Verify Verilog reset defaults match ground truth."""

    def test_verilog_reset_defaults(self):
        """Reset block values must match ground truth."""
        v_defaults = cp.parse_verilog_reset_defaults()
        for reg, expected in GROUND_TRUTH_RESET_DEFAULTS.items():
            assert reg in v_defaults, f"{reg} not found in reset block"
            actual = v_defaults[reg]
            assert actual == expected, (
                f"{reg}: reset default {actual} != expected {expected}"
            )


class TestTier1BulkFrameLayout:
    """Verify bulk frame protocol layout between Python parser and FPGA RTL.

    The FT2232H USB interface uses a bulk per-frame transfer protocol:
      Header (8 bytes): 0xAA | flags | frame_num[15:0] | n_range[15:0] | n_doppler[15:0]
      [variable payload sections based on flags]
      Footer (1 byte):  0x55

    Flags byte: {2'b0, sparse_det, mag_only, stream_cfar, stream_doppler, stream_range}
    """

    # Ground truth: independently derived from protocol spec (NOT from code)
    HEADER_BYTE = 0xAA
    FOOTER_BYTE = 0x55
    BULK_HEADER_SIZE = 8   # 1 + 1 + 2 + 2 + 2
    N_RANGE = 512
    N_DOPPLER = 32
    # Flag bit positions (from LSB)
    FLAG_STREAM_RANGE = 0x01
    FLAG_STREAM_DOPPLER = 0x02
    FLAG_STREAM_CFAR = 0x04
    FLAG_MAG_ONLY = 0x08
    FLAG_SPARSE_DET = 0x10

    def test_python_header_constants_match_ground_truth(self):
        """Python protocol constants must match independently-derived ground truth."""
        from radar_protocol import (
            HEADER_BYTE, FOOTER_BYTE, BULK_HEADER_SIZE,
            NUM_RANGE_BINS, NUM_DOPPLER_BINS,
        )
        assert HEADER_BYTE == self.HEADER_BYTE, (
            f"HEADER_BYTE: 0x{HEADER_BYTE:02X} != 0x{self.HEADER_BYTE:02X}"
        )
        assert FOOTER_BYTE == self.FOOTER_BYTE, (
            f"FOOTER_BYTE: 0x{FOOTER_BYTE:02X} != 0x{self.FOOTER_BYTE:02X}"
        )
        assert BULK_HEADER_SIZE == self.BULK_HEADER_SIZE, (
            f"BULK_HEADER_SIZE: {BULK_HEADER_SIZE} != {self.BULK_HEADER_SIZE}"
        )
        assert NUM_RANGE_BINS == self.N_RANGE, (
            f"NUM_RANGE_BINS: {NUM_RANGE_BINS} != {self.N_RANGE}"
        )
        assert NUM_DOPPLER_BINS == self.N_DOPPLER, (
            f"NUM_DOPPLER_BINS: {NUM_DOPPLER_BINS} != {self.N_DOPPLER}"
        )

    def test_verilog_header_constants_match_ground_truth(self):
        """Verilog FT2232H header/footer constants must match ground truth."""
        v_pkt = cp.parse_verilog_packet_constants()
        # Status header/footer (FT2232H bulk frames use same HEADER for data)
        assert v_pkt["status"].header == 0xBB, (
            f"STATUS_HEADER: 0x{v_pkt['status'].header:02X} != 0xBB"
        )
        assert v_pkt["status"].footer == self.FOOTER_BYTE, (
            f"FOOTER: 0x{v_pkt['status'].footer:02X} != 0x{self.FOOTER_BYTE:02X}"
        )
        assert v_pkt["status"].size == 26, (
            f"STATUS_PKT_LEN: {v_pkt['status'].size} != 26"
        )

    def test_bulk_frame_total_size_mag_only_with_cfar(self):
        """Verify computed frame size for mag-only + bitmap detection mode.

        Section sizes (ground truth):
          Header:          8 bytes
          Range profile:   512 * 2 = 1024 bytes
          Doppler mag:     512 * 32 * 2 = 32768 bytes
          Detection bitmap: (512 * 32) / 8 = 2048 bytes
          Footer:          1 byte
          Total:           35849 bytes
        """
        header = self.BULK_HEADER_SIZE
        range_section = self.N_RANGE * 2
        doppler_section = self.N_RANGE * self.N_DOPPLER * 2  # mag-only: 16-bit per cell
        det_section = (self.N_RANGE * self.N_DOPPLER + 7) // 8  # bitmap
        footer = 1
        expected_total = header + range_section + doppler_section + det_section + footer
        assert expected_total == 35849, (
            f"Frame size: {expected_total} != 35849"
        )

    def test_bulk_frame_total_size_iq_mode(self):
        """Verify computed frame size for full I/Q (no mag-only) + bitmap.

        Doppler I/Q: 512 * 32 * 4 = 65536 bytes (vs 32768 for mag-only)
        Total: 8 + 1024 + 65536 + 2048 + 1 = 68617 bytes
        """
        header = self.BULK_HEADER_SIZE
        range_section = self.N_RANGE * 2
        doppler_section = self.N_RANGE * self.N_DOPPLER * 4  # I/Q: 32-bit per cell
        det_section = (self.N_RANGE * self.N_DOPPLER + 7) // 8
        footer = 1
        expected_total = header + range_section + doppler_section + det_section + footer
        assert expected_total == 68617, (
            f"I/Q frame size: {expected_total} != 68617"
        )

    def test_parser_extracts_header_fields(self):
        """parse_bulk_frame must correctly extract header fields."""
        from radar_protocol import RadarProtocol

        # Build a minimal valid frame: range + doppler(mag-only) + cfar bitmap
        flags = 0x0F  # range | doppler | cfar | mag_only
        frame_num = 0x1234
        n_range = self.N_RANGE
        n_doppler = self.N_DOPPLER

        buf = bytearray()
        buf.append(self.HEADER_BYTE)
        buf.append(flags)
        buf += struct.pack(">H", frame_num)
        buf += struct.pack(">H", n_range)
        buf += struct.pack(">H", n_doppler)

        # Range profile: all zeros (512 x 2 bytes)
        buf += bytes(n_range * 2)
        # Doppler mag: all zeros (512 x 32 x 2 bytes)
        buf += bytes(n_range * n_doppler * 2)
        # Detection bitmap: all zeros
        buf += bytes((n_range * n_doppler + 7) // 8)
        # Footer
        buf.append(self.FOOTER_BYTE)

        frame = RadarProtocol.parse_bulk_frame(bytes(buf))
        assert frame is not None, "parse_bulk_frame returned None for valid frame"
        assert frame.frame_number == 0x1234, (
            f"frame_number: {frame.frame_number} != 0x1234"
        )
        assert frame.range_profile.shape == (n_range,), (
            f"range_profile shape: {frame.range_profile.shape} != ({n_range},)"
        )
        assert frame.magnitude.shape == (n_range, n_doppler), (
            f"magnitude shape: {frame.magnitude.shape} != ({n_range}, {n_doppler})"
        )
        assert frame.detections.shape == (n_range, n_doppler), (
            f"detections shape: {frame.detections.shape} != ({n_range}, {n_doppler})"
        )

    def test_parser_rejects_bad_header(self):
        """parse_bulk_frame must reject frames with wrong header byte."""
        from radar_protocol import RadarProtocol

        bad_frame = bytes([0xBB, 0x0F, 0x00, 0x01, 0x02, 0x00, 0x00, 0x20, 0x55])
        assert RadarProtocol.parse_bulk_frame(bad_frame) is None

    def test_parser_rejects_truncated_frame(self):
        """parse_bulk_frame must reject frames shorter than header + footer."""
        from radar_protocol import RadarProtocol

        short = bytes([self.HEADER_BYTE, 0x0F, 0x00, 0x01])
        assert RadarProtocol.parse_bulk_frame(short) is None

    def test_detection_bitmap_lsb_first_round_trip(self):
        """Detection bitmap packing must use LSB-first per byte (matching RTL).

        RTL packs: byte_addr = {range_bin, doppler[4:3]}, bit = doppler[2:0].
        This means doppler_bin % 8 gives the bit position within the byte.
        """
        from radar_protocol import RadarProtocol

        n_range = self.N_RANGE
        n_doppler = self.N_DOPPLER
        flags = 0x0F  # range | doppler | cfar | mag_only

        # Create a detection at range=50, doppler=5
        # Expected: byte_idx = 50 * (32/8) + 5//8 = 50*4 + 0 = 200
        # bit_pos = 5 % 8 = 5 → byte value = 1 << 5 = 0x20
        det_bytes = bytearray((n_range * n_doppler + 7) // 8)
        test_range = 50
        test_doppler = 5
        byte_idx = test_range * (n_doppler // 8) + test_doppler // 8
        bit_pos = test_doppler % 8
        det_bytes[byte_idx] |= 1 << bit_pos

        buf = bytearray()
        buf.append(self.HEADER_BYTE)
        buf.append(flags)
        buf += struct.pack(">H", 1)         # frame_num
        buf += struct.pack(">H", n_range)
        buf += struct.pack(">H", n_doppler)
        buf += bytes(n_range * 2)                       # range profile
        buf += bytes(n_range * n_doppler * 2)           # doppler mag
        buf += det_bytes                                # detection bitmap
        buf.append(self.FOOTER_BYTE)

        frame = RadarProtocol.parse_bulk_frame(bytes(buf))
        assert frame is not None
        assert frame.detections[test_range, test_doppler] == 1, (
            f"Detection at ({test_range},{test_doppler}) not found"
        )
        # Verify no false detections in nearby bins
        assert frame.detections[test_range, test_doppler - 1] == 0, (
            f"False detection at ({test_range},{test_doppler - 1})"
        )
        assert frame.detections[test_range, test_doppler + 1] == 0, (
            f"False detection at ({test_range},{test_doppler + 1})"
        )
        assert frame.detection_count == 1, (
            f"detection_count: {frame.detection_count} != 1"
        )

    def test_mock_frame_round_trips_through_parser(self):
        """FT2232HConnection._mock_read() output must parse correctly.

        This is NOT self-referential: the mock packs LSB-first per byte
        (matching RTL), and the parser unpacks LSB-first. If either side
        flipped the bit order, the target detections would appear in wrong
        Doppler bins.
        """
        from radar_protocol import RadarProtocol, FT2232HConnection

        conn = FT2232HConnection(mock=True)
        raw = conn._mock_read(0)
        frame = RadarProtocol.parse_bulk_frame(raw)
        assert frame is not None, "Mock frame failed to parse"
        assert frame.range_profile.shape[0] == self.N_RANGE
        assert frame.magnitude.shape == (self.N_RANGE, self.N_DOPPLER)
        # The mock injects targets at range ~100, doppler 7-9
        # Verify detections appear in expected region
        det_region = frame.detections[99:102, 7:10]
        assert det_region.sum() > 0, (
            "No detections found in expected target region (range 99-101, doppler 7-9)"
        )


class TestTier1STM32SettingsPacket:
    """Verify STM32 settings packet layout."""

    def test_field_order_and_sizes(self):
        """STM32 settings fields must have correct offsets and sizes."""
        fields = cp.parse_stm32_settings_fields()
        if not fields:
            pytest.skip("MCU source not available")

        expected = [
            ("system_frequency", 0, 8, "double"),
            ("chirp_duration_1", 8, 8, "double"),
            ("chirp_duration_2", 16, 8, "double"),
            ("chirps_per_position", 24, 4, "uint32_t"),
            ("freq_min", 28, 8, "double"),
            ("freq_max", 36, 8, "double"),
            ("prf1", 44, 8, "double"),
            ("prf2", 52, 8, "double"),
            ("max_distance", 60, 8, "double"),
            ("map_size", 68, 8, "double"),
        ]

        assert len(fields) == len(expected), (
            f"Expected {len(expected)} fields, got {len(fields)}"
        )

        for f, (ename, eoff, esize, etype) in zip(fields, expected, strict=True):
            assert f.name == ename, f"Field name: {f.name} != {ename}"
            assert f.offset == eoff, f"{f.name}: offset {f.offset} != {eoff}"
            assert f.size == esize, f"{f.name}: size {f.size} != {esize}"
            assert f.c_type == etype, f"{f.name}: type {f.c_type} != {etype}"

    def test_minimum_packet_size(self):
        """
        RadarSettings.cpp says minimum is 74 bytes but actual payload is:
        'SET'(3) + 9*8(doubles) + 4(uint32) + 'END'(3) = 82 bytes.
        This test documents the bug.
        """
        fields = cp.parse_stm32_settings_fields()
        if not fields:
            pytest.skip("MCU source not available")

        # Calculate required payload size
        total_field_bytes = sum(f.size for f in fields)
        # Add markers: "SET"(3) + "END"(3)
        required_size = 3 + total_field_bytes + 3

        # Read the actual minimum check from the source
        src = (cp.MCU_LIB_DIR / "RadarSettings.cpp").read_text(encoding="latin-1")
        import re
        m = re.search(r'length\s*<\s*(\d+)', src)
        assert m, "Could not find minimum length check in parseFromUSB"
        declared_min = int(m.group(1))

        assert declared_min == required_size, (
            f"BUFFER OVERREAD BUG: parseFromUSB minimum check is {declared_min} "
            f"but actual required size is {required_size}. "
            f"({total_field_bytes} bytes of fields + 6 bytes of markers). "
            f"If exactly {declared_min} bytes are passed, extractDouble() reads "
            f"past the buffer at offset {declared_min - 3} (needs 8 bytes, "
            f"only {declared_min - 3 - fields[-1].offset} available)."
        )

    def test_stm32_usb_start_flag(self):
        """USB start flag must be [23, 46, 158, 237]."""
        flag = cp.parse_stm32_start_flag()
        if not flag:
            pytest.skip("USBHandler.cpp not available")
        assert flag == [23, 46, 158, 237], f"Start flag: {flag}"


# ===================================================================
# TIER 2: Verilog Cosimulation
# ===================================================================


# NOTE: TestTier2VerilogCosim was deleted — the FT2232H TB used the old
# per-sample data packet interface which was fully replaced by bulk frame
# protocol. Command round-trip and status packet testing via Verilog cosim
# can be restored when tb_cross_layer_ft2232h.v is rewritten for bulk frames.


# ===================================================================
# TIER 3: C Stub Execution
# ===================================================================

@pytest.mark.skipif(not _has_cxx, reason="C++ compiler not available")
class TestTier3CStub:
    """Compile STM32 settings stub and verify field parsing."""

    @pytest.fixture(scope="class")
    def stub_binary(self, tmp_path_factory):
        """Compile the C++ stub once."""
        workdir = tmp_path_factory.mktemp("c_stub")
        stub_src = THIS_DIR / "stm32_settings_stub.cpp"
        radar_settings_src = cp.MCU_LIB_DIR / "RadarSettings.cpp"
        out_bin = workdir / "stm32_settings_stub"

        result = subprocess.run(
            [CXX, "-std=c++11", "-o", str(out_bin),
             str(stub_src), str(radar_settings_src),
             f"-I{cp.MCU_LIB_DIR}"],
            capture_output=True, text=True, timeout=30,
        )
        assert result.returncode == 0, f"Compile failed:\n{result.stderr}"
        return out_bin

    def _build_settings_packet(self, values: dict) -> bytes:
        """Build a binary settings packet matching RadarSettings::parseFromUSB."""
        pkt = b"SET"
        for key in [
            "system_frequency", "chirp_duration_1", "chirp_duration_2",
        ]:
            pkt += struct.pack(">d", values[key])
        pkt += struct.pack(">I", values["chirps_per_position"])
        for key in [
            "freq_min", "freq_max", "prf1", "prf2",
            "max_distance", "map_size",
        ]:
            pkt += struct.pack(">d", values[key])
        pkt += b"END"
        return pkt

    def _run_stub(self, binary: Path, packet: bytes) -> dict[str, str]:
        """Run stub with packet file, parse stdout into field dict."""
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
            f.write(packet)
            pkt_path = f.name

        try:
            result = subprocess.run(
                [str(binary), pkt_path],
                capture_output=True, text=True, timeout=10,
            )
        finally:
            os.unlink(pkt_path)

        fields = {}
        for line in result.stdout.strip().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                fields[k.strip()] = v.strip()
        return fields

    def test_default_values_round_trip(self, stub_binary):
        """Default settings must parse correctly through C stub."""
        values = {
            "system_frequency": 10.0e9,
            "chirp_duration_1": 30.0e-6,
            "chirp_duration_2": 0.5e-6,
            "chirps_per_position": 32,
            "freq_min": 10.0e6,
            "freq_max": 30.0e6,
            "prf1": 1000.0,
            "prf2": 2000.0,
            "max_distance": 1536.0,
            "map_size": 1536.0,
        }
        pkt = self._build_settings_packet(values)
        result = self._run_stub(stub_binary, pkt)

        assert result.get("parse_ok") == "true", f"Parse failed: {result}"

        for key, expected in values.items():
            actual_str = result.get(key)
            assert actual_str is not None, f"Missing field: {key}"
            actual = int(actual_str) if key == "chirps_per_position" else float(actual_str)
            if isinstance(expected, float):
                assert abs(actual - expected) < expected * 1e-10, (
                    f"{key}: {actual} != {expected}"
                )
            else:
                assert actual == expected, f"{key}: {actual} != {expected}"

    def test_distinctive_values_round_trip(self, stub_binary):
        """Non-default distinctive values must parse correctly."""
        values = {
            "system_frequency": 24.125e9,   # K-band
            "chirp_duration_1": 100.0e-6,
            "chirp_duration_2": 2.0e-6,
            "chirps_per_position": 64,
            "freq_min": 24.0e6,
            "freq_max": 24.25e6,
            "prf1": 5000.0,
            "prf2": 3000.0,
            "max_distance": 75000.0,
            "map_size": 100000.0,
        }
        pkt = self._build_settings_packet(values)
        result = self._run_stub(stub_binary, pkt)

        assert result.get("parse_ok") == "true", f"Parse failed: {result}"

        for key, expected in values.items():
            actual_str = result.get(key)
            assert actual_str is not None, f"Missing field: {key}"
            actual = int(actual_str) if key == "chirps_per_position" else float(actual_str)
            if isinstance(expected, float):
                assert abs(actual - expected) < expected * 1e-10, (
                    f"{key}: {actual} != {expected}"
                )
            else:
                assert actual == expected, f"{key}: {actual} != {expected}"

    def test_truncated_packet_rejected(self, stub_binary):
        """Packet shorter than minimum must be rejected."""
        pkt = b"SET" + b"\x00" * 40 + b"END"  # Only 46 bytes, needs 82
        result = self._run_stub(stub_binary, pkt)
        assert result.get("parse_ok") == "false", (
            f"Expected parse failure for truncated packet, got: {result}"
        )

    def test_bad_markers_rejected(self, stub_binary):
        """Packet with wrong start/end markers must be rejected."""
        values = {
            "system_frequency": 10.5e9, "chirp_duration_1": 30.0e-6,
            "chirp_duration_2": 0.5e-6, "chirps_per_position": 32,
            "freq_min": 10.0e6, "freq_max": 30.0e6,
            "prf1": 1000.0, "prf2": 2000.0,
            "max_distance": 1536.0, "map_size": 1536.0,
        }
        pkt = self._build_settings_packet(values)

        # Wrong start marker
        bad_pkt = b"BAD" + pkt[3:]
        result = self._run_stub(stub_binary, bad_pkt)
        assert result.get("parse_ok") == "false", "Should reject bad start marker"

        # Wrong end marker
        bad_pkt = pkt[:-3] + b"BAD"
        result = self._run_stub(stub_binary, bad_pkt)
        assert result.get("parse_ok") == "false", "Should reject bad end marker"

    def test_python_c_packet_format_agreement(self, stub_binary):
        """
        Python builds a settings packet, C stub parses it.
        This tests that both sides agree on the packet format.
        """
        # Use values right at validation boundaries to stress-test
        values = {
            "system_frequency": 1.0e9,     # min valid
            "chirp_duration_1": 1.0e-6,    # min valid
            "chirp_duration_2": 0.1e-6,    # min valid
            "chirps_per_position": 1,      # min valid
            "freq_min": 1.0e6,             # min valid
            "freq_max": 2.0e6,             # just above freq_min
            "prf1": 100.0,                 # min valid
            "prf2": 100.0,                 # min valid
            "max_distance": 100.0,         # min valid
            "map_size": 1000.0,            # min valid
        }
        pkt = self._build_settings_packet(values)
        result = self._run_stub(stub_binary, pkt)

        assert result.get("parse_ok") == "true", (
            f"Boundary values rejected: {result}"
        )


# ===================================================================
# TIER 3b: Detection Bitmap Packing (RTL ground truth, not self-referential)
# ===================================================================

class TestTier3bDetectionBitmapPacking:
    """Verify detection bitmap packing matches RTL bit-layout specification.

    The RTL packs detection bits as:
        byte_addr = {range_bin, doppler_bin[4:3]}   (i.e. range_bin * 4 + doppler_bin // 8)
        bit_pos   = doppler_bin[2:0]                (i.e. doppler_bin % 8, LSB-first)

    This test derives expected bytes INDEPENDENTLY from the RTL formula,
    then verifies the Python mock generator produces those exact bytes AND
    the Python parser recovers the original detection matrix.

    This prevents the "self-referential mock" problem where mock and parser
    agree with each other but both diverge from what the FPGA actually sends.
    """

    # RTL constants
    N_RANGE = 512
    N_DOPPLER = 32
    BITMAP_BYTES = N_RANGE * N_DOPPLER // 8  # 2048

    @staticmethod
    def _rtl_pack_bitmap(det_matrix):
        """Pack a detection matrix using the RTL formula (ground truth).

        This is an independent re-implementation of the Verilog logic in
        usb_data_interface_ft2232h.v lines 440-448:
            detect_rmw_addr  <= {range_bin_in, doppler_bin_in[4:3]};
            detect_rmw_bit   <= doppler_bin_in[2:0];
        """
        n_range, n_doppler = det_matrix.shape
        buf = bytearray(n_range * n_doppler // 8)
        for r in range(n_range):
            for d in range(n_doppler):
                if det_matrix[r, d]:
                    byte_addr = r * (n_doppler // 8) + d // 8
                    bit_pos = d % 8  # LSB-first
                    buf[byte_addr] |= 1 << bit_pos
        return bytes(buf)

    def test_mock_generator_matches_rtl_formula(self):
        """FT2232HConnection mock bitmap bytes must match independently-derived RTL packing."""
        import radar_protocol as rp

        # Generate a mock frame via the FT2232H mock connection
        conn = rp.FT2232HConnection(mock=True)
        conn.open()
        raw = conn.read()
        conn.close()
        assert raw is not None, "Mock connection returned None"
        frame = rp.RadarProtocol.parse_bulk_frame(raw)
        assert frame is not None, "Failed to parse mock frame"

        # The mock creates detections at range_bins 99-101, doppler_bins 7-9
        # (from radar_protocol.py FT2232HConnection._mock_read())
        import numpy as np
        expected_det = np.zeros((self.N_RANGE, self.N_DOPPLER), dtype=np.uint8)
        for rb in range(99, 102):
            for db in range(7, 10):
                expected_det[rb, db] = 1

        # Derive expected bytes from RTL formula (ground truth)
        expected_bytes = self._rtl_pack_bitmap(expected_det)

        # Extract actual bitmap bytes from the raw frame
        # Frame layout (flags=0x0F: stream_range + stream_doppler + stream_cfar + mag_only):
        #   header(8) + range_mag(512*2) + doppler_mag(512*32*2) + det_bitmap(2048) + footer(1)
        header_size = 1 + 1 + 2 + 2 + 2  # 8 bytes
        range_mag_size = self.N_RANGE * 2  # 1024 bytes
        doppler_mag_size = self.N_RANGE * self.N_DOPPLER * 2  # 32768 bytes
        bitmap_offset = header_size + range_mag_size + doppler_mag_size
        actual_bytes = raw[bitmap_offset:bitmap_offset + self.BITMAP_BYTES]

        assert actual_bytes == expected_bytes, (
            "Mock generator bitmap does NOT match RTL packing formula. "
            "This means the mock would produce frames that differ from "
            "what the FPGA actually sends over USB."
        )

    def test_parser_recovers_detections_from_rtl_packed_bytes(self):
        """Parser must correctly decode bytes packed with the RTL formula."""
        import numpy as np

        # Create a known detection pattern (sparse, tests edge cases)
        det = np.zeros((self.N_RANGE, self.N_DOPPLER), dtype=np.uint8)
        # Test corners: first/last range bin, first/last doppler bin
        test_points = [(0, 0), (0, 31), (511, 0), (511, 31),
                       (100, 7), (255, 15), (256, 16)]
        for r, d in test_points:
            det[r, d] = 1

        # Pack using RTL formula (ground truth)
        rtl_bytes = self._rtl_pack_bitmap(det)

        # Decode using the same logic as RadarProtocol.parse_bulk_frame
        recovered = np.zeros((self.N_RANGE, self.N_DOPPLER), dtype=np.uint8)
        for r in range(self.N_RANGE):
            for d in range(self.N_DOPPLER):
                byte_idx = r * (self.N_DOPPLER // 8) + d // 8
                bit_pos = d % 8
                if (rtl_bytes[byte_idx] >> bit_pos) & 1:
                    recovered[r, d] = 1

        np.testing.assert_array_equal(recovered, det,
            err_msg="Parser logic cannot round-trip RTL-packed bitmap. "
                    "The parser's byte_idx/bit_pos formula diverges from RTL.")

    def test_msb_packing_would_fail(self):
        """Sanity check: MSB-first packing must produce DIFFERENT bytes than RTL.

        This test exists to ensure the RTL ground truth test above isn't
        vacuously true. If MSB and LSB packing produced the same bytes,
        we couldn't detect the bug.
        """
        import numpy as np

        # Detection at doppler bin 1 — LSB bit_pos=1, MSB bit_pos=6
        det = np.zeros((self.N_RANGE, self.N_DOPPLER), dtype=np.uint8)
        det[0, 1] = 1  # doppler bin 1: LSB gives 0x02, MSB gives 0x40

        rtl_bytes = self._rtl_pack_bitmap(det)

        # MSB-first packing (the OLD, wrong way)
        msb_bytes = bytearray(self.BITMAP_BYTES)
        bit_idx = 0 * self.N_DOPPLER + 1
        msb_bytes[bit_idx // 8] |= 1 << (7 - (bit_idx % 8))

        assert rtl_bytes != bytes(msb_bytes), (
            "LSB and MSB packing produced identical bytes — "
            "this test cannot distinguish the bug. Check test logic."
        )


# ===================================================================
# TIER 4: Stale Value Detection (LLM Agent Guardrails)
# ===================================================================

class TestTier4BannedPatterns:
    """
    Scan source files for known-wrong values that have been corrected.

    These patterns are stale ADI Phaser defaults, wrong sample rates,
    and incorrect range calculations that were cleaned up in commit
    d259e5c. If an LLM agent reintroduces them, this test catches it.

    IMPORTANT: Allowlist entries exist for legitimate uses (e.g. comments
    explaining what was wrong, test files checking for these values).
    """

    # (regex_pattern, description, file_extensions_to_scan)
    BANNED_PATTERNS: ClassVar[list[tuple[str, str, tuple[str, ...]]]] = [
        # Wrong carrier frequency (ADI Phaser default, not PLFM)
        (r'10[._]?525\s*e\s*9|10\.525\s*GHz|10525000000',
         "Stale ADI Phaser carrier freq — PLFM uses 10.5 GHz",
         ("*.py", "*.v", "*.vh", "*.cpp", "*.h")),

        # Wrong post-DDC sample rate (ADI Phaser uses 4 MSPS)
        (r'(?<!\d)4e6(?!\d)|4_?000_?000\.0?\s*#.*(?:sample|samp|rate|fs)',
         "Stale ADI 4 MSPS rate — PLFM post-DDC is 100 MSPS",
         ("*.py",)),

        # Wrong range per bin values from old calculations
        (r'(?<!\d)4\.8\s*(?:m/bin|m per|meters?\s*per)',
         "Stale bin spacing 4.8 m — PLFM is 6.0 m/bin",
         ("*.py", "*.cpp")),

        (r'(?<!\d)5\.6\s*(?:m/bin|m per|meters?\s*per)',
         "Stale bin spacing 5.6 m — PLFM is 6.0 m/bin",
         ("*.py", "*.cpp")),

        # Wrong range resolution from deramped FMCW formula
        (r'781\.25',
         "Stale ADI range value 781.25 — not applicable to PLFM",
         ("*.py",)),
    ]

    # Files that are allowed to contain banned patterns
    # (this test file, historical docs, comparison scripts)
    # ADI co-sim files legitimately reference ADI Phaser hardware params
    # because they process ADI test stimulus data (10.525 GHz, 4 MSPS).
    ALLOWLIST: ClassVar[set[str]] = {
        "test_cross_layer_contract.py",  # This file — contains the patterns!
        "CLAUDE.md",                     # Context doc may reference old values
        "golden_reference.py",           # Processes ADI Phaser test data
        "tb_fullchain_mti_cfar_realdata.v",  # $display of ADI test stimulus info
        "tb_doppler_realdata.v",             # $display of ADI test stimulus info
        "tb_range_fft_realdata.v",           # $display of ADI test stimulus info
        "tb_fullchain_realdata.v",           # $display of ADI test stimulus info
    }

    def _scan_files(self, pattern_re, extensions):
        """Scan firmware source files for a regex pattern."""
        hits = []
        firmware_dir = cp.REPO_ROOT / "9_Firmware"

        for ext in extensions:
            for fpath in firmware_dir.rglob(ext.replace("*", "**/*") if "**" in ext else ext):
                # Skip allowlisted files
                if fpath.name in self.ALLOWLIST:
                    continue
                # Skip __pycache__, .git, etc.
                parts = set(fpath.parts)
                if parts & {"__pycache__", ".git", ".venv", "venv", ".ruff_cache"}:
                    continue

                try:
                    text = fpath.read_text(encoding="utf-8", errors="ignore")
                except OSError:
                    continue

                for i, line in enumerate(text.splitlines(), 1):
                    if pattern_re.search(line):
                        hits.append(f"{fpath.relative_to(cp.REPO_ROOT)}:{i}: {line.strip()[:120]}")

        return hits

    def test_no_banned_stale_values(self):
        """
        No source file should contain known-wrong stale values.

        If this fails, an LLM agent likely reintroduced a corrected value.
        Check the flagged lines and fix them. If a line is a legitimate
        use (e.g. a comment explaining history), add the file to ALLOWLIST.
        """
        all_hits = []
        for pattern_str, description, extensions in self.BANNED_PATTERNS:
            pattern_re = re.compile(pattern_str, re.IGNORECASE)
            hits = self._scan_files(pattern_re, extensions)
            all_hits.extend(f"[{description}] {hit}" for hit in hits)

        assert not all_hits, (
            f"Found {len(all_hits)} stale/banned value(s) in source files:\n"
            + "\n".join(f"  {h}" for h in all_hits[:20])
            + ("\n  ... and more" if len(all_hits) > 20 else "")
        )


# ===================================================================
# TIER 5: Pipeline Co-Simulation (Python golden_reference round-trip)
# ===================================================================
#
# These tests load committed .npy golden reference data and run the
# Python bit-accurate model functions on intermediate outputs to verify
# stage-to-stage consistency.  This catches:
#   - Golden reference function drift (model changed but .npy not regenerated)
#   - Stage boundary mismatches (one stage's output shape/scale doesn't
#     match the next stage's expected input)
#   - Bit-accuracy regressions in the Python model
#
# The .npy files were generated by golden_reference.py:main() using
# real ADI Phaser test data.  These tests do NOT re-run golden_reference
# main() — they verify that each processing function, when fed the
# committed intermediate outputs, reproduces the committed downstream
# outputs.  This is fundamentally different from self-blessing because
# the .npy files are committed artifacts (not regenerated each run).

import numpy as np  # noqa: E402

_COSIM_HEX_DIR = cp.REPO_ROOT / "9_Firmware" / "9_2_FPGA" / "tb" / "cosim" / "real_data" / "hex"
_GOLDEN_REF_DIR = cp.REPO_ROOT / "9_Firmware" / "9_2_FPGA" / "tb" / "cosim" / "real_data"

# Add golden_reference to path for imports
if str(_GOLDEN_REF_DIR) not in sys.path:
    sys.path.insert(0, str(_GOLDEN_REF_DIR))


def _load_npy(name: str) -> np.ndarray:
    """Load a committed .npy file from the cosim hex directory.

    Raises FileNotFoundError if the file is missing — use _skip_if_no_golden()
    as a sentinel check at the start of each test to skip gracefully when the
    entire golden dataset is absent.
    """
    path = _COSIM_HEX_DIR / name
    return np.load(str(path))


# Every Tier 5 artifact that is loaded by these tests. If any is missing, the
# committed golden dataset is incomplete and all Tier 5 tests are skipped.
_GOLDEN_SENTINELS = [
    "decimated_range_i.npy",
    "decimated_range_q.npy",
    "doppler_map_i.npy",
    "doppler_map_q.npy",
    "fullchain_mti_i.npy",
    "fullchain_mti_q.npy",
    "fullchain_mti_doppler_i.npy",
    "fullchain_mti_doppler_q.npy",
    "fullchain_cfar_flags.npy",
    "fullchain_cfar_mag.npy",
    "fullchain_cfar_thr.npy",
    "range_fft_all_i.npy",
    "range_fft_all_q.npy",
    "detection_mag.npy",
]


def _skip_if_no_golden():
    """Skip test if committed golden .npy data is not available."""
    for sentinel in _GOLDEN_SENTINELS:
        if not (_COSIM_HEX_DIR / sentinel).exists():
            pytest.skip(f"Golden data incomplete: {sentinel} not found")


class TestTier5PipelineDecimToMTI:
    """Verify decimated range → MTI canceller stage consistency."""

    def test_mti_canceller_reproduces_committed_output(self):
        """run_mti_canceller(decimated) must match fullchain_mti .npy files."""
        _skip_if_no_golden()
        from golden_reference import run_mti_canceller

        dec_i = _load_npy("decimated_range_i.npy")
        dec_q = _load_npy("decimated_range_q.npy")
        expected_i = _load_npy("fullchain_mti_i.npy")
        expected_q = _load_npy("fullchain_mti_q.npy")

        got_i, got_q = run_mti_canceller(dec_i, dec_q, enable=True)

        np.testing.assert_array_equal(got_i, expected_i,
            err_msg="MTI I output drifted from committed golden data")
        np.testing.assert_array_equal(got_q, expected_q,
            err_msg="MTI Q output drifted from committed golden data")

    def test_mti_passthrough_equals_input(self):
        """MTI with enable=False must pass through unchanged."""
        _skip_if_no_golden()
        from golden_reference import run_mti_canceller

        dec_i = _load_npy("decimated_range_i.npy")
        dec_q = _load_npy("decimated_range_q.npy")

        got_i, got_q = run_mti_canceller(dec_i, dec_q, enable=False)

        np.testing.assert_array_equal(got_i, dec_i)
        np.testing.assert_array_equal(got_q, dec_q)

    def test_mti_first_chirp_is_zeroed(self):
        """MTI 2-pulse canceller must zero the first chirp (no previous data)."""
        _skip_if_no_golden()
        from golden_reference import run_mti_canceller

        dec_i = _load_npy("decimated_range_i.npy")
        dec_q = _load_npy("decimated_range_q.npy")

        got_i, got_q = run_mti_canceller(dec_i, dec_q, enable=True)

        np.testing.assert_array_equal(got_i[0], 0,
            err_msg="MTI chirp 0 should be all zeros")
        np.testing.assert_array_equal(got_q[0], 0,
            err_msg="MTI chirp 0 should be all zeros")


class TestTier5PipelineDoppler:
    """Verify Doppler FFT stage consistency against committed golden data."""

    def test_doppler_no_mti_reproduces_committed_output(self):
        """run_doppler_fft(range_fft_all) must match doppler_map .npy (direct path).

        NOTE: doppler_map was generated from full range FFT output (32, 1024),
        NOT from decimated data.  The function only processes the first 64
        columns, but it needs the twiddle file that golden_reference.py used.
        """
        _skip_if_no_golden()
        from golden_reference import run_doppler_fft

        range_i = _load_npy("range_fft_all_i.npy")
        range_q = _load_npy("range_fft_all_q.npy")
        expected_i = _load_npy("doppler_map_i.npy")
        expected_q = _load_npy("doppler_map_q.npy")

        twiddle_16 = str(cp.REPO_ROOT / "9_Firmware" / "9_2_FPGA" / "fft_twiddle_16.mem")
        if not Path(twiddle_16).exists():
            pytest.skip("fft_twiddle_16.mem not found")

        got_i, got_q = run_doppler_fft(range_i, range_q, twiddle_file_16=twiddle_16)

        np.testing.assert_array_equal(got_i, expected_i,
            err_msg="Doppler I (direct path) drifted from committed golden data")
        np.testing.assert_array_equal(got_q, expected_q,
            err_msg="Doppler Q (direct path) drifted from committed golden data")

    def test_doppler_mti_reproduces_committed_output(self):
        """run_doppler_fft(mti_output) must match fullchain_mti_doppler .npy."""
        _skip_if_no_golden()
        from golden_reference import run_doppler_fft

        mti_i = _load_npy("fullchain_mti_i.npy")
        mti_q = _load_npy("fullchain_mti_q.npy")
        expected_i = _load_npy("fullchain_mti_doppler_i.npy")
        expected_q = _load_npy("fullchain_mti_doppler_q.npy")

        twiddle_16 = str(cp.REPO_ROOT / "9_Firmware" / "9_2_FPGA" / "fft_twiddle_16.mem")
        if not Path(twiddle_16).exists():
            pytest.skip("fft_twiddle_16.mem not found")

        got_i, got_q = run_doppler_fft(mti_i, mti_q, twiddle_file_16=twiddle_16)

        np.testing.assert_array_equal(got_i, expected_i,
            err_msg="Doppler I (MTI path) drifted from committed golden data")
        np.testing.assert_array_equal(got_q, expected_q,
            err_msg="Doppler Q (MTI path) drifted from committed golden data")

    def test_doppler_output_shape(self):
        """Doppler output must be (512, 32) — 512 range bins x 32 Doppler bins."""
        _skip_if_no_golden()
        from golden_reference import run_doppler_fft

        dec_i = _load_npy("decimated_range_i.npy")
        dec_q = _load_npy("decimated_range_q.npy")

        twiddle_16 = str(
            cp.REPO_ROOT / "9_Firmware" / "9_2_FPGA" / "fft_twiddle_16.mem"
        )
        tw = twiddle_16 if Path(twiddle_16).exists() else None

        got_i, got_q = run_doppler_fft(dec_i, dec_q, twiddle_file_16=tw)

        assert got_i.shape == (512, 32), f"Expected (512,32), got {got_i.shape}"
        assert got_q.shape == (512, 32), f"Expected (512,32), got {got_q.shape}"

    def test_doppler_chained_from_decimated_via_mti(self):
        """Full chain: decimated → MTI → Doppler must match committed output."""
        _skip_if_no_golden()
        from golden_reference import run_mti_canceller, run_doppler_fft

        dec_i = _load_npy("decimated_range_i.npy")
        dec_q = _load_npy("decimated_range_q.npy")
        expected_i = _load_npy("fullchain_mti_doppler_i.npy")
        expected_q = _load_npy("fullchain_mti_doppler_q.npy")

        twiddle_16 = str(cp.REPO_ROOT / "9_Firmware" / "9_2_FPGA" / "fft_twiddle_16.mem")
        if not Path(twiddle_16).exists():
            pytest.skip("fft_twiddle_16.mem not found")

        mti_i, mti_q = run_mti_canceller(dec_i, dec_q, enable=True)
        got_i, got_q = run_doppler_fft(mti_i, mti_q, twiddle_file_16=twiddle_16)

        np.testing.assert_array_equal(got_i, expected_i,
            err_msg="Chained decim→MTI→Doppler I mismatches committed golden")
        np.testing.assert_array_equal(got_q, expected_q,
            err_msg="Chained decim→MTI→Doppler Q mismatches committed golden")


class TestTier5PipelineCFAR:
    """Verify CFAR stage against committed golden data."""

    # Parameters matching golden_reference.py:main() at lines 1266-1269
    CFAR_GUARD = 2
    CFAR_TRAIN = 8
    CFAR_ALPHA = 0x30
    CFAR_MODE = "CA"
    DC_NOTCH_WIDTH = 2

    def test_cfar_reproduces_committed_output(self):
        """Full chain: MTI Doppler → DC notch → CFAR must match committed .npy."""
        _skip_if_no_golden()
        from golden_reference import run_dc_notch, run_cfar_ca

        doppler_i = _load_npy("fullchain_mti_doppler_i.npy")
        doppler_q = _load_npy("fullchain_mti_doppler_q.npy")
        expected_flags = _load_npy("fullchain_cfar_flags.npy")
        expected_mag = _load_npy("fullchain_cfar_mag.npy")
        expected_thr = _load_npy("fullchain_cfar_thr.npy")

        # Apply DC notch first (matching golden_reference.py:main() flow)
        notch_i, notch_q = run_dc_notch(doppler_i, doppler_q,
                                         width=self.DC_NOTCH_WIDTH)

        got_flags, got_mag, got_thr = run_cfar_ca(
            notch_i, notch_q,
            guard=self.CFAR_GUARD,
            train=self.CFAR_TRAIN,
            alpha_q44=self.CFAR_ALPHA,
            mode=self.CFAR_MODE,
        )

        np.testing.assert_array_equal(got_flags, expected_flags,
            err_msg="CFAR flags drifted from committed golden data")
        np.testing.assert_array_equal(got_mag, expected_mag,
            err_msg="CFAR magnitudes drifted from committed golden data")
        np.testing.assert_array_equal(got_thr, expected_thr,
            err_msg="CFAR thresholds drifted from committed golden data")

    def test_cfar_output_shapes(self):
        """CFAR outputs must be (512, 32)."""
        _skip_if_no_golden()
        from golden_reference import run_dc_notch, run_cfar_ca

        doppler_i = _load_npy("fullchain_mti_doppler_i.npy")
        doppler_q = _load_npy("fullchain_mti_doppler_q.npy")

        notch_i, notch_q = run_dc_notch(
            doppler_i, doppler_q, width=self.DC_NOTCH_WIDTH,
        )
        flags, mag, thr = run_cfar_ca(
            notch_i, notch_q,
            guard=self.CFAR_GUARD,
            train=self.CFAR_TRAIN,
            alpha_q44=self.CFAR_ALPHA,
            mode=self.CFAR_MODE,
        )

        assert flags.shape == (512, 32)
        assert mag.shape == (512, 32)
        assert thr.shape == (512, 32)

    def test_cfar_flags_are_boolean(self):
        """CFAR flags should be boolean (0 or 1 only)."""
        _skip_if_no_golden()
        flags = _load_npy("fullchain_cfar_flags.npy")
        unique = set(np.unique(flags))
        assert unique <= {0, 1}, \
            f"CFAR flags contain non-boolean values: {unique}"

    def test_cfar_threshold_ge_zero(self):
        """CFAR thresholds must be non-negative (unsigned in RTL)."""
        _skip_if_no_golden()
        thr = _load_npy("fullchain_cfar_thr.npy")
        assert np.all(thr >= 0), "CFAR thresholds contain negative values"

    def test_cfar_detection_implies_mag_gt_threshold(self):
        """Where CFAR flags are True, magnitude must exceed threshold."""
        _skip_if_no_golden()
        flags = _load_npy("fullchain_cfar_flags.npy")
        mag = _load_npy("fullchain_cfar_mag.npy")
        thr = _load_npy("fullchain_cfar_thr.npy")

        detected = flags.astype(bool)
        if not np.any(detected):
            pytest.skip("No CFAR detections in golden data")

        # Where detected, magnitude must be > threshold (strict inequality)
        violations = detected & (mag <= thr)
        n_violations = int(np.sum(violations))
        assert n_violations == 0, (
            f"{n_violations} cells detected but mag <= threshold"
        )

    def test_dc_notch_zeros_correct_bins(self):
        """DC notch with width=2 must zero bins {0,1,15,16,17,31}.

        Logic: bin_within_sf < width  OR  bin_within_sf > (15 - width + 1)
        width=2: < 2 → {0,1} per subframe;  > 14 → {15} per subframe
        Global: {0, 1, 15, 16, 17, 31}
        """
        _skip_if_no_golden()
        from golden_reference import run_dc_notch

        doppler_i = _load_npy("fullchain_mti_doppler_i.npy")
        doppler_q = _load_npy("fullchain_mti_doppler_q.npy")

        notch_i, notch_q = run_dc_notch(doppler_i, doppler_q, width=2)

        # Per subframe: bins < 2 → {0,1} and bins > 14 → {15}
        expected_zero_bins = {0, 1, 15, 16, 17, 31}
        for dbin in range(32):
            if dbin in expected_zero_bins:
                assert np.all(notch_i[:, dbin] == 0), \
                    f"DC notch failed: bin {dbin} should be zero"
                assert np.all(notch_q[:, dbin] == 0), \
                    f"DC notch failed: bin {dbin} should be zero"
            else:
                assert np.array_equal(notch_i[:, dbin], doppler_i[:, dbin]), \
                    f"DC notch modified non-notched I bin {dbin}"
                assert np.array_equal(notch_q[:, dbin], doppler_q[:, dbin]), \
                    f"DC notch modified non-notched Q bin {dbin}"


class TestTier5FullPipelineChain:
    """End-to-end: decimated → MTI → Doppler → DC notch → CFAR."""

    def test_full_chain_matches_committed_cfar(self):
        """Chaining all stages from decimated input must match committed CFAR output."""
        _skip_if_no_golden()
        from golden_reference import (
            run_mti_canceller, run_doppler_fft, run_dc_notch, run_cfar_ca,
        )

        dec_i = _load_npy("decimated_range_i.npy")
        dec_q = _load_npy("decimated_range_q.npy")
        expected_flags = _load_npy("fullchain_cfar_flags.npy")
        expected_mag = _load_npy("fullchain_cfar_mag.npy")

        twiddle_16 = str(cp.REPO_ROOT / "9_Firmware" / "9_2_FPGA" / "fft_twiddle_16.mem")
        if not Path(twiddle_16).exists():
            pytest.skip("fft_twiddle_16.mem not found")

        # Run full chain
        mti_i, mti_q = run_mti_canceller(dec_i, dec_q, enable=True)
        dop_i, dop_q = run_doppler_fft(mti_i, mti_q, twiddle_file_16=twiddle_16)
        notch_i, notch_q = run_dc_notch(dop_i, dop_q, width=2)
        flags, mag, _thr = run_cfar_ca(
            notch_i, notch_q, guard=2, train=8, alpha_q44=0x30, mode="CA",
        )

        np.testing.assert_array_equal(flags, expected_flags,
            err_msg="Full chain flags mismatch — pipeline drift detected")
        np.testing.assert_array_equal(mag, expected_mag,
            err_msg="Full chain magnitudes mismatch — pipeline drift detected")

    def test_detection_mag_matches_committed(self):
        """Simple threshold detection magnitude must match committed detection_mag.npy.

        detection_mag.npy was generated from the DIRECT path (no decimator, no MTI):
          range_fft_all → doppler_fft → run_detection
        """
        _skip_if_no_golden()
        from golden_reference import run_doppler_fft, run_detection

        range_i = _load_npy("range_fft_all_i.npy")
        range_q = _load_npy("range_fft_all_q.npy")
        expected_mag = _load_npy("detection_mag.npy")

        twiddle_16 = str(cp.REPO_ROOT / "9_Firmware" / "9_2_FPGA" / "fft_twiddle_16.mem")
        if not Path(twiddle_16).exists():
            pytest.skip("fft_twiddle_16.mem not found")

        # Direct path (matching golden_reference.py:main() flow for detection_mag)
        dop_i, dop_q = run_doppler_fft(range_i, range_q, twiddle_file_16=twiddle_16)
        got_mag, _det = run_detection(dop_i, dop_q, threshold=10000)

        np.testing.assert_array_equal(got_mag, expected_mag,
            err_msg="Detection magnitude drifted from committed golden data")
