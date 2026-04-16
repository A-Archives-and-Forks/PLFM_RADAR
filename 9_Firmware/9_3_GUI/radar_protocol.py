#!/usr/bin/env python3
"""
AERIS-10 Radar Protocol Layer
===============================
Pure-logic module for USB packet parsing and command building.
No GUI dependencies — safe to import from tests and headless scripts.

USB Interface: FT2232H USB 2.0 (8-bit, 50T production board) via pyftdi

USB Packet Protocol:
  TX (FPGA→Host):
    Bulk frame (FT2232H):
      [0xAA] [flags 1B] [frame# 2B] [range_bins 2B] [doppler_bins 2B]
      [range profile (opt)] [doppler mag/IQ (opt)] [detect flags (opt)] [0x55]
    Status packet: [0xBB] [status 6x32b] [0x55]
  RX (Host→FPGA):
    Command: 4 bytes received sequentially {opcode, addr, value_hi, value_lo}
"""

import struct
import time
import threading
import queue
import logging
import contextlib
from dataclasses import dataclass, field
from typing import Any
from enum import IntEnum


import numpy as np

log = logging.getLogger("radar_protocol")

# ============================================================================
# Constants matching usb_data_interface.v
# ============================================================================

HEADER_BYTE = 0xAA
FOOTER_BYTE = 0x55
STATUS_HEADER_BYTE = 0xBB

# Packet sizes
BULK_HEADER_SIZE = 8                 # 1(AA)+1(flags)+2(frame#)+2(Rbins)+2(Dbins)
STATUS_PACKET_SIZE = 26              # 1 + 24 + 1

# Legacy per-sample protocol (FT601 USB 3.0 only)
DATA_PACKET_SIZE = 11               # 1 + 4 + 2 + 2 + 1 + 1

NUM_RANGE_BINS = 512
NUM_DOPPLER_BINS = 32
NUM_CELLS = NUM_RANGE_BINS * NUM_DOPPLER_BINS  # 16384

WATERFALL_DEPTH = 64


class Opcode(IntEnum):
    """Host register opcodes — must match radar_system_top.v case(usb_cmd_opcode).

    FPGA truth table (from radar_system_top.v lines 902-944):
        0x01  host_radar_mode        0x14  host_short_listen_cycles
        0x02  host_trigger_pulse     0x15  host_chirps_per_elev
        0x03  host_detect_threshold  0x16  host_gain_shift
        0x04  host_stream_control    0x20  host_range_mode
        0x10  host_long_chirp_cycles 0x21-0x27  CFAR / MTI / DC-notch
        0x11  host_long_listen_cycles 0x28-0x2C  AGC control
        0x12  host_guard_cycles      0x30  host_self_test_trigger
        0x13  host_short_chirp_cycles 0x31/0xFF  host_status_request
    """
    # --- Basic control (0x01-0x04) ---
    RADAR_MODE          = 0x01  # 2-bit mode select
    TRIGGER_PULSE       = 0x02  # self-clearing one-shot trigger
    DETECT_THRESHOLD    = 0x03  # 16-bit detection threshold value
    STREAM_CONTROL      = 0x04  # 6-bit stream/format control

    # --- Digital gain (0x16) ---
    GAIN_SHIFT          = 0x16  # 4-bit digital gain shift

    # --- Chirp timing (0x10-0x15) ---
    LONG_CHIRP          = 0x10
    LONG_LISTEN         = 0x11
    GUARD               = 0x12
    SHORT_CHIRP         = 0x13
    SHORT_LISTEN        = 0x14
    CHIRPS_PER_ELEV     = 0x15

    # --- Signal processing (0x20-0x27) ---
    RANGE_MODE          = 0x20
    CFAR_GUARD          = 0x21
    CFAR_TRAIN          = 0x22
    CFAR_ALPHA          = 0x23
    CFAR_MODE           = 0x24
    CFAR_ENABLE         = 0x25
    MTI_ENABLE          = 0x26
    DC_NOTCH_WIDTH      = 0x27

    # --- AGC (0x28-0x2C) ---
    AGC_ENABLE          = 0x28
    AGC_TARGET          = 0x29
    AGC_ATTACK          = 0x2A
    AGC_DECAY           = 0x2B
    AGC_HOLDOFF         = 0x2C

    # --- Board self-test / status (0x30-0x31, 0xFF) ---
    SELF_TEST_TRIGGER   = 0x30
    SELF_TEST_STATUS    = 0x31
    STATUS_REQUEST      = 0xFF


# ============================================================================
# Data Structures
# ============================================================================

@dataclass
class RadarFrame:
    """One complete radar frame (512 range x 32 Doppler)."""
    timestamp: float = 0.0
    range_doppler_i: np.ndarray = field(
        default_factory=lambda: np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=np.int16))
    range_doppler_q: np.ndarray = field(
        default_factory=lambda: np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=np.int16))
    magnitude: np.ndarray = field(
        default_factory=lambda: np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=np.float64))
    detections: np.ndarray = field(
        default_factory=lambda: np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=np.uint8))
    range_profile: np.ndarray = field(
        default_factory=lambda: np.zeros(NUM_RANGE_BINS, dtype=np.float64))
    detection_count: int = 0
    frame_number: int = 0


@dataclass
class StatusResponse:
    """Parsed status response from FPGA (6-word / 26-byte packet)."""
    radar_mode: int = 0
    stream_ctrl: int = 0
    cfar_threshold: int = 0
    long_chirp: int = 0
    long_listen: int = 0
    guard: int = 0
    short_chirp: int = 0
    short_listen: int = 0
    chirps_per_elev: int = 0
    range_mode: int = 0
    # Self-test results (word 5, added in Build 26)
    self_test_flags: int = 0     # 5-bit result flags [4:0]
    self_test_detail: int = 0    # 8-bit detail code [7:0]
    self_test_busy: int = 0      # 1-bit busy flag
    # AGC metrics (word 4, added for hybrid AGC)
    agc_current_gain: int = 0    # 4-bit current gain encoding [3:0]
    agc_peak_magnitude: int = 0  # 8-bit peak magnitude [7:0]
    agc_saturation_count: int = 0  # 8-bit saturation count [7:0]
    agc_enable: int = 0          # 1-bit AGC enable readback


# ============================================================================
# Protocol: Packet Parsing & Building
# ============================================================================

def _to_signed16(val: int) -> int:
    """Convert unsigned 16-bit integer to signed (two's complement)."""
    val = val & 0xFFFF
    return val - 0x10000 if val >= 0x8000 else val


class RadarProtocol:
    """
    Parse FPGA→Host packets and build Host→FPGA command words.
    Matches usb_data_interface.v packet format exactly.
    """

    @staticmethod
    def build_command(opcode: int, value: int, addr: int = 0) -> bytes:
        """
        Build a 32-bit command word: {opcode[31:24], addr[23:16], value[15:0]}.
        Returns 4 bytes, big-endian (MSB first).
        """
        word = ((opcode & 0xFF) << 24) | ((addr & 0xFF) << 16) | (value & 0xFFFF)
        return struct.pack(">I", word)

    @staticmethod
    def parse_data_packet(raw: bytes) -> dict[str, Any] | None:
        """
        Parse an 11-byte data packet from the FT2232H byte stream.
        Returns dict with keys: 'range_i', 'range_q', 'doppler_i', 'doppler_q',
        'detection', or None if invalid.

        Packet format (11 bytes):
          Byte 0:    0xAA (header)
          Bytes 1-2: range_q[15:0] MSB first
          Bytes 3-4: range_i[15:0] MSB first
          Bytes 5-6: doppler_real[15:0] MSB first
          Bytes 7-8: doppler_imag[15:0] MSB first
          Byte 9:    {7'b0, cfar_detection}
          Byte 10:   0x55 (footer)
        """
        if len(raw) < DATA_PACKET_SIZE:
            return None
        if raw[0] != HEADER_BYTE:
            return None
        if raw[10] != FOOTER_BYTE:
            return None

        range_q = _to_signed16(struct.unpack_from(">H", raw, 1)[0])
        range_i = _to_signed16(struct.unpack_from(">H", raw, 3)[0])
        doppler_i = _to_signed16(struct.unpack_from(">H", raw, 5)[0])
        doppler_q = _to_signed16(struct.unpack_from(">H", raw, 7)[0])
        detection = raw[9] & 0x01

        return {
            "range_i": range_i,
            "range_q": range_q,
            "doppler_i": doppler_i,
            "doppler_q": doppler_q,
            "detection": detection,
        }

    @staticmethod
    def parse_status_packet(raw: bytes) -> StatusResponse | None:
        """
        Parse a status response packet.
        Format: [0xBB] [6x4B status words] [0x55] = 1 + 24 + 1 = 26 bytes
        """
        if len(raw) < 26:
            return None
        if raw[0] != STATUS_HEADER_BYTE:
            return None

        words = []
        for i in range(6):
            w = struct.unpack_from(">I", raw, 1 + i * 4)[0]
            words.append(w)

        if raw[25] != FOOTER_BYTE:
            return None

        sr = StatusResponse()
        # Word 0: {0xFF[31:24], mode[23:22], stream[21:16], threshold[15:0]}
        sr.cfar_threshold = words[0] & 0xFFFF
        sr.stream_ctrl = (words[0] >> 16) & 0x3F
        sr.radar_mode = (words[0] >> 22) & 0x03
        # Word 1: {long_chirp[31:16], long_listen[15:0]}
        sr.long_listen = words[1] & 0xFFFF
        sr.long_chirp = (words[1] >> 16) & 0xFFFF
        # Word 2: {guard[31:16], short_chirp[15:0]}
        sr.short_chirp = words[2] & 0xFFFF
        sr.guard = (words[2] >> 16) & 0xFFFF
        # Word 3: {short_listen[31:16], 10'd0, chirps_per_elev[5:0]}
        sr.chirps_per_elev = words[3] & 0x3F
        sr.short_listen = (words[3] >> 16) & 0xFFFF
        # Word 4: {agc_current_gain[31:28], agc_peak_magnitude[27:20],
        #          agc_saturation_count[19:12], agc_enable[11], 9'd0, range_mode[1:0]}
        sr.range_mode = words[4] & 0x03
        sr.agc_enable = (words[4] >> 11) & 0x01
        sr.agc_saturation_count = (words[4] >> 12) & 0xFF
        sr.agc_peak_magnitude = (words[4] >> 20) & 0xFF
        sr.agc_current_gain = (words[4] >> 28) & 0x0F
        # Word 5: {7'd0, self_test_busy, 8'd0, self_test_detail[7:0],
        #           3'd0, self_test_flags[4:0]}
        sr.self_test_flags = words[5] & 0x1F
        sr.self_test_detail = (words[5] >> 8) & 0xFF
        sr.self_test_busy = (words[5] >> 24) & 0x01
        return sr

    @staticmethod
    def parse_bulk_frame(raw: bytes) -> RadarFrame | None:
        """Parse a bulk per-frame transfer from the FT2232H USB interface.

        Frame format (from usb_data_interface_ft2232h.v):
          Byte 0:    0xAA (header)
          Byte 1:    Format flags {2'b0, sparse_det, mag_only,
                     stream_cfar, stream_doppler, stream_range}
          Byte 2-3:  Frame number (16-bit, MSB first)
          Byte 4-5:  Range bin count (16-bit, MSB first)
          Byte 6-7:  Doppler bin count (16-bit, MSB first)
          [variable payload based on flags]
          Last byte: 0x55 (footer)
        """
        if len(raw) < BULK_HEADER_SIZE + 1:  # header + footer minimum
            return None
        if raw[0] != HEADER_BYTE:
            return None

        flags = raw[1]
        frame_num = struct.unpack_from(">H", raw, 2)[0]
        n_range = struct.unpack_from(">H", raw, 4)[0]
        n_doppler = struct.unpack_from(">H", raw, 6)[0]

        stream_range = bool(flags & 0x01)
        stream_doppler = bool(flags & 0x02)
        stream_cfar = bool(flags & 0x04)
        mag_only = bool(flags & 0x08)
        sparse_det = bool(flags & 0x10)

        offset = BULK_HEADER_SIZE
        frame = RadarFrame()
        frame.frame_number = frame_num
        frame.timestamp = time.time()

        # --- Range profile section ---
        if stream_range:
            nbytes = n_range * 2
            if offset + nbytes > len(raw):
                return None
            range_data = np.frombuffer(raw[offset:offset + nbytes],
                                       dtype=">u2").astype(np.float64)
            frame.range_profile = range_data
            offset += nbytes

        # --- Doppler magnitude or I/Q section ---
        if stream_doppler:
            if mag_only:
                nbytes = n_range * n_doppler * 2
                if offset + nbytes > len(raw):
                    return None
                mag_flat = np.frombuffer(raw[offset:offset + nbytes],
                                         dtype=">u2").astype(np.float64)
                frame.magnitude = mag_flat.reshape(n_range, n_doppler)
                # No I/Q available in mag-only mode
                frame.range_doppler_i = np.zeros((n_range, n_doppler), dtype=np.int16)
                frame.range_doppler_q = np.zeros((n_range, n_doppler), dtype=np.int16)
                offset += nbytes
            else:
                # Full I/Q: 32-bit per cell (I16, Q16)
                nbytes = n_range * n_doppler * 4
                if offset + nbytes > len(raw):
                    return None
                iq_data = np.frombuffer(raw[offset:offset + nbytes], dtype=">i2")
                iq_data = iq_data.reshape(n_range, n_doppler, 2)
                frame.range_doppler_i = iq_data[:, :, 0].astype(np.int16)
                frame.range_doppler_q = iq_data[:, :, 1].astype(np.int16)
                frame.magnitude = (
                    np.abs(frame.range_doppler_i.astype(np.float64))
                    + np.abs(frame.range_doppler_q.astype(np.float64))
                )
                offset += nbytes

        # --- Detection flags section ---
        if stream_cfar:
            if sparse_det:
                if offset + 2 > len(raw):
                    return None
                det_count = struct.unpack_from(">H", raw, offset)[0]
                offset += 2
                nbytes = det_count * 6
                if offset + nbytes > len(raw):
                    return None
                det_arr = np.zeros((n_range, n_doppler), dtype=np.uint8)
                for d in range(det_count):
                    base = offset + d * 6
                    rbin = struct.unpack_from(">H", raw, base)[0]
                    dbin = struct.unpack_from(">H", raw, base + 2)[0]
                    if rbin < n_range and dbin < n_doppler:
                        det_arr[rbin, dbin] = 1
                frame.detections = det_arr
                frame.detection_count = det_count
                offset += nbytes
            else:
                # Packed bitmap: n_range * n_doppler bits, LSB-first per byte
                # RTL packs: byte_addr = {range_bin, doppler[4:3]}, bit = doppler[2:0]
                nbytes = (n_range * n_doppler + 7) // 8
                if offset + nbytes > len(raw):
                    return None
                det_bytes = raw[offset:offset + nbytes]
                det_arr = np.zeros((n_range, n_doppler), dtype=np.uint8)
                for r in range(n_range):
                    for db in range(n_doppler):
                        byte_idx = r * (n_doppler // 8) + db // 8
                        bit_pos = db % 8  # LSB-first: doppler[2:0] = bit position
                        if byte_idx < len(det_bytes) and (det_bytes[byte_idx] >> bit_pos) & 1:
                            det_arr[r, db] = 1
                frame.detections = det_arr
                frame.detection_count = int(det_arr.sum())
                offset += nbytes

        # Footer check
        if offset >= len(raw) or raw[offset] != FOOTER_BYTE:
            return None

        # Derive range_profile from magnitude if not streamed directly
        if not stream_range and stream_doppler:
            frame.range_profile = np.sum(frame.magnitude, axis=1)

        return frame

    @staticmethod
    def compute_bulk_frame_size(flags: int, n_range: int = NUM_RANGE_BINS,
                                n_doppler: int = NUM_DOPPLER_BINS) -> int:
        """Compute expected bulk frame size in bytes for given flags."""
        size = BULK_HEADER_SIZE  # header
        if flags & 0x01:  # stream_range
            size += n_range * 2
        if flags & 0x02:  # stream_doppler
            if flags & 0x08:  # mag_only
                size += n_range * n_doppler * 2
            else:
                size += n_range * n_doppler * 4
        if flags & 0x04:  # stream_cfar
            if flags & 0x10:  # sparse_det — variable, use bitmap estimate
                size += 2  # count field minimum
            else:
                size += (n_range * n_doppler + 7) // 8
        size += 1  # footer
        return size

    @staticmethod
    def find_packet_boundaries(buf: bytes) -> list[tuple[int, int, str]]:
        """
        Scan buffer for packet start markers.
        Supports bulk frames (0xAA with 8-byte header), status (0xBB),
        and legacy 11-byte data packets (0xAA with footer at offset 10).
        Returns list of (start_idx, expected_end_idx, packet_type).
        """
        packets = []
        i = 0
        while i < len(buf):
            if buf[i] == HEADER_BYTE:
                # Try bulk frame first (8-byte header with range/doppler counts)
                if i + BULK_HEADER_SIZE <= len(buf):
                    flags = buf[i + 1]
                    n_range = struct.unpack_from(">H", buf, i + 4)[0]
                    n_doppler = struct.unpack_from(">H", buf, i + 6)[0]
                    # Sanity: valid bulk frame has reasonable dimensions
                    if (1 <= n_range <= 2048 and 1 <= n_doppler <= 64
                            and flags <= 0x3F):
                        expected_size = RadarProtocol.compute_bulk_frame_size(
                            flags, n_range, n_doppler)
                        end = i + expected_size
                        if end <= len(buf) and buf[end - 1] == FOOTER_BYTE:
                            packets.append((i, end, "bulk"))
                            i = end
                            continue
                        if end > len(buf):
                            break  # partial bulk frame
                # Fallback: legacy 11-byte per-sample packet (FT601)
                end = i + DATA_PACKET_SIZE
                if end <= len(buf) and buf[end - 1] == FOOTER_BYTE:
                    packets.append((i, end, "data"))
                    i = end
                else:
                    if end > len(buf):
                        break
                    i += 1
            elif buf[i] == STATUS_HEADER_BYTE:
                end = i + STATUS_PACKET_SIZE
                if end <= len(buf) and buf[end - 1] == FOOTER_BYTE:
                    packets.append((i, end, "status"))
                    i = end
                else:
                    if end > len(buf):
                        break  # partial status packet — leave for residual
                    i += 1  # footer mismatch — skip
            else:
                i += 1
        return packets


# ============================================================================
# FT2232H USB 2.0 Connection (pyftdi, 245 Synchronous FIFO)
# ============================================================================

# Optional pyftdi import
try:
    from pyftdi.ftdi import Ftdi, FtdiError
    PyFtdi = Ftdi
    PYFTDI_AVAILABLE = True
except ImportError:
    class FtdiError(Exception):
        """Fallback FTDI error type when pyftdi is unavailable."""

    PYFTDI_AVAILABLE = False


class FT2232HConnection:
    """
    FT2232H USB 2.0 Hi-Speed FIFO bridge communication.
    Uses pyftdi in 245 Synchronous FIFO mode (Channel A).
    VID:PID = 0x0403:0x6010 (FTDI default for FT2232H).
    """

    VID = 0x0403
    PID = 0x6010

    def __init__(self, mock: bool = True):
        self._mock = mock
        self._ftdi = None
        self._lock = threading.Lock()
        self.is_open = False
        # Mock state
        self._mock_frame_num = 0
        self._mock_rng = np.random.RandomState(42)

    def open(self, device_index: int = 0) -> bool:
        if self._mock:
            self.is_open = True
            log.info("FT2232H mock device opened (no hardware)")
            return True

        if not PYFTDI_AVAILABLE:
            log.error("pyftdi not installed — cannot open real FT2232H device")
            return False

        try:
            self._ftdi = PyFtdi()
            url = f"ftdi://0x{self.VID:04x}:0x{self.PID:04x}/{device_index + 1}"
            self._ftdi.open_from_url(url)
            # Configure for 245 Synchronous FIFO mode
            self._ftdi.set_bitmode(0xFF, PyFtdi.BitMode.SYNCFF)
            # Set USB transfer size for throughput
            self._ftdi.read_data_set_chunksize(65536)
            self._ftdi.write_data_set_chunksize(65536)
            # Purge buffers
            self._ftdi.purge_buffers()
            self.is_open = True
            log.info(f"FT2232H device opened: {url}")
            return True
        except FtdiError as e:
            log.error(f"FT2232H open failed: {e}")
            return False

    def close(self):
        if self._ftdi is not None:
            with contextlib.suppress(Exception):
                self._ftdi.close()
            self._ftdi = None
        self.is_open = False

    def read(self, size: int = 4096) -> bytes | None:
        """Read raw bytes from FT2232H. Returns None on error/timeout."""
        if not self.is_open:
            return None

        if self._mock:
            return self._mock_read(size)

        with self._lock:
            try:
                data = self._ftdi.read_data(size)
                return bytes(data) if data else None
            except FtdiError as e:
                log.error(f"FT2232H read error: {e}")
                return None

    def write(self, data: bytes) -> bool:
        """Write raw bytes to FT2232H (4-byte commands)."""
        if not self.is_open:
            return False

        if self._mock:
            log.info(f"FT2232H mock write: {data.hex()}")
            return True

        with self._lock:
            try:
                written = self._ftdi.write_data(data)
                return written == len(data)
            except FtdiError as e:
                log.error(f"FT2232H write error: {e}")
                return False

    def _mock_read(self, _size: int) -> bytes:
        """
        Generate a synthetic bulk radar frame for testing.
        Matches the FT2232H sectioned transfer format: header (8B) →
        range profile → doppler magnitude → detection flags → footer.
        A target is injected near range bin 100, Doppler bin 8.
        """
        time.sleep(0.05)
        self._mock_frame_num += 1

        # Stream flags: range + doppler + cfar, mag-only, bitmap detections
        flags = 0x0F  # stream_range | stream_doppler | stream_cfar | mag_only

        buf = bytearray()
        # --- Header (8 bytes) ---
        buf.append(HEADER_BYTE)
        buf.append(flags)
        buf += struct.pack(">H", self._mock_frame_num & 0xFFFF)
        buf += struct.pack(">H", NUM_RANGE_BINS)
        buf += struct.pack(">H", NUM_DOPPLER_BINS)

        # --- Range profile (512 x 16-bit) ---
        range_profile = self._mock_rng.randint(50, 200, size=NUM_RANGE_BINS).astype(np.uint16)
        # Inject target peak at range bin ~100
        for rb in range(98, 103):
            range_profile[rb] = 8000
        buf += range_profile.astype(">u2").tobytes()

        # --- Doppler magnitude (512 x 32 x 16-bit, mag-only) ---
        mag = self._mock_rng.randint(
            10, 100, size=(NUM_RANGE_BINS, NUM_DOPPLER_BINS),
        ).astype(np.uint16)
        for rb in range(98, 103):
            for db in range(7, 10):
                mag[rb, db] = 8000
        buf += mag.astype(">u2").tobytes()

        # --- Detection flags (bitmap: 512*32/8 = 2048 bytes) ---
        det = np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=np.uint8)
        for rb in range(99, 102):
            for db in range(7, 10):
                det[rb, db] = 1
        det_bytes = bytearray((NUM_RANGE_BINS * NUM_DOPPLER_BINS + 7) // 8)
        for r in range(NUM_RANGE_BINS):
            for d in range(NUM_DOPPLER_BINS):
                if det[r, d]:
                    # LSB-first per byte, matching RTL: byte_addr = {range_bin, doppler[4:3]},
                    # bit = doppler[2:0]. This matches the parser at line ~368.
                    byte_idx = r * (NUM_DOPPLER_BINS // 8) + d // 8
                    bit_pos = d % 8
                    det_bytes[byte_idx] |= 1 << bit_pos
        buf += det_bytes

        # --- Footer ---
        buf.append(FOOTER_BYTE)

        return bytes(buf)





# ============================================================================
# Data Recorder (HDF5)
# ============================================================================

try:
    import h5py
    HDF5_AVAILABLE = True
except ImportError:
    HDF5_AVAILABLE = False


class DataRecorder:
    """Record radar frames to HDF5 files for offline analysis."""

    def __init__(self):
        self._file = None
        self._grp = None
        self._frame_count = 0
        self._recording = False

    @property
    def recording(self) -> bool:
        return self._recording

    def start(self, filepath: str):
        if not HDF5_AVAILABLE:
            log.error("h5py not installed — HDF5 recording unavailable")
            return
        try:
            self._file = h5py.File(filepath, "w")
            self._file.attrs["creator"] = "AERIS-10 Radar Dashboard"
            self._file.attrs["start_time"] = time.time()
            self._file.attrs["range_bins"] = NUM_RANGE_BINS
            self._file.attrs["doppler_bins"] = NUM_DOPPLER_BINS

            self._grp = self._file.create_group("frames")
            self._frame_count = 0
            self._recording = True
            log.info(f"Recording started: {filepath}")
        except (OSError, ValueError) as e:
            log.error(f"Failed to start recording: {e}")

    def record_frame(self, frame: RadarFrame):
        if not self._recording or self._file is None:
            return
        try:
            fg = self._grp.create_group(f"frame_{self._frame_count:06d}")
            fg.attrs["timestamp"] = frame.timestamp
            fg.attrs["frame_number"] = frame.frame_number
            fg.attrs["detection_count"] = frame.detection_count
            fg.create_dataset("magnitude", data=frame.magnitude, compression="gzip")
            fg.create_dataset("range_doppler_i", data=frame.range_doppler_i, compression="gzip")
            fg.create_dataset("range_doppler_q", data=frame.range_doppler_q, compression="gzip")
            fg.create_dataset("detections", data=frame.detections, compression="gzip")
            fg.create_dataset("range_profile", data=frame.range_profile, compression="gzip")
            self._frame_count += 1
        except (OSError, ValueError, TypeError) as e:
            log.error(f"Recording error: {e}")

    def stop(self):
        if self._file is not None:
            try:
                self._file.attrs["end_time"] = time.time()
                self._file.attrs["total_frames"] = self._frame_count
                self._file.close()
            except (OSError, ValueError, RuntimeError):
                pass
            self._file = None
        self._recording = False
        log.info(f"Recording stopped ({self._frame_count} frames)")


# ============================================================================
# Radar Data Acquisition Thread
# ============================================================================

class RadarAcquisition(threading.Thread):
    """
    Background thread: reads from USB (FT2232H), parses bulk frames
    or legacy 11-byte packets, and pushes complete frames to the display queue.
    """

    def __init__(self, connection, frame_queue: queue.Queue,
                 recorder: DataRecorder | None = None,
                 status_callback=None):
        super().__init__(daemon=True)
        self.conn = connection
        self.frame_queue = frame_queue
        self.recorder = recorder
        self._status_callback = status_callback
        self._stop_event = threading.Event()
        self._frame = RadarFrame()
        self._sample_idx = 0
        self._frame_num = 0

    def stop(self):
        self._stop_event.set()

    def run(self):
        log.info("Acquisition thread started")
        residual = b""
        while not self._stop_event.is_set():
            chunk = self.conn.read(65536)
            if chunk is None or len(chunk) == 0:
                time.sleep(0.01)
                continue

            raw = residual + chunk
            packets = RadarProtocol.find_packet_boundaries(raw)

            # Keep unparsed tail bytes for next iteration
            if packets:
                last_end = packets[-1][1]
                residual = raw[last_end:]
            else:
                # No packets found — keep entire buffer as residual
                # but cap to avoid unbounded growth
                max_residual = 65536
                residual = raw[-max_residual:] if len(raw) > max_residual else raw
            for start, end, ptype in packets:
                if ptype == "bulk":
                    frame = RadarProtocol.parse_bulk_frame(raw[start:end])
                    if frame is not None:
                        self._emit_frame(frame)
                elif ptype == "data":
                    # Legacy per-sample protocol (FT601)
                    parsed = RadarProtocol.parse_data_packet(
                        raw[start:end])
                    if parsed is not None:
                        self._ingest_sample(parsed)
                elif ptype == "status":
                    status = RadarProtocol.parse_status_packet(raw[start:end])
                    if status is not None:
                        log.info(f"Status: mode={status.radar_mode} "
                                 f"stream={status.stream_ctrl}")
                        if status.self_test_busy or status.self_test_flags:
                            log.info(f"Self-test: busy={status.self_test_busy} "
                                     f"flags=0b{status.self_test_flags:05b} "
                                     f"detail=0x{status.self_test_detail:02X}")
                        if self._status_callback is not None:
                            try:
                                self._status_callback(status)
                            except Exception as e:  # noqa: BLE001
                                log.error(f"Status callback error: {e}")

        log.info("Acquisition thread stopped")

    def _emit_frame(self, frame: RadarFrame):
        """Push a complete bulk frame to the display queue."""
        # Push to display queue (drop old if backed up)
        try:
            self.frame_queue.put_nowait(frame)
        except queue.Full:
            with contextlib.suppress(queue.Empty):
                self.frame_queue.get_nowait()
            self.frame_queue.put_nowait(frame)

        if self.recorder and self.recorder.recording:
            self.recorder.record_frame(frame)

    def _ingest_sample(self, sample: dict):
        """Place sample into current frame and emit when complete (legacy FT601 path)."""
        rbin = self._sample_idx // NUM_DOPPLER_BINS
        dbin = self._sample_idx % NUM_DOPPLER_BINS

        if rbin < NUM_RANGE_BINS and dbin < NUM_DOPPLER_BINS:
            self._frame.range_doppler_i[rbin, dbin] = sample["doppler_i"]
            self._frame.range_doppler_q[rbin, dbin] = sample["doppler_q"]
            mag = abs(int(sample["doppler_i"])) + abs(int(sample["doppler_q"]))
            self._frame.magnitude[rbin, dbin] = mag
            if sample.get("detection", 0):
                self._frame.detections[rbin, dbin] = 1
                self._frame.detection_count += 1

        self._sample_idx += 1

        if self._sample_idx >= NUM_CELLS:
            self._finalize_frame()

    def _finalize_frame(self):
        """Complete frame: compute range profile, push to queue, record (legacy path)."""
        self._frame.timestamp = time.time()
        self._frame.frame_number = self._frame_num
        # Range profile = sum of magnitude across Doppler bins
        self._frame.range_profile = np.sum(self._frame.magnitude, axis=1)

        self._emit_frame(self._frame)

        self._frame_num += 1
        self._frame = RadarFrame()
        self._sample_idx = 0
