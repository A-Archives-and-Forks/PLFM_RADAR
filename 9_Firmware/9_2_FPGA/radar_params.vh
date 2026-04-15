// ============================================================================
// radar_params.vh — Single Source of Truth for AERIS-10 FPGA Parameters
// ============================================================================
//
// ALL modules in the FPGA processing chain MUST `include this file instead of
// hardcoding range bins, segment counts, chirp samples, or timing values.
//
// This file uses `define macros (not localparam) so it can be included at any
// scope. Each consuming module should include this file inside its body and
// optionally alias macros to localparams for readability.
//
// BOARD VARIANTS:
//   SUPPORT_LONG_RANGE = 0  (50T, USB_MODE=1)  — 3 km mode only, 64 range bins
//   SUPPORT_LONG_RANGE = 1  (200T, USB_MODE=0) — 3 km + 20 km modes, up to 1024 bins
//
// RANGE MODES (runtime, via host_range_mode register, opcode 0x20):
//   2'b00 = 3 km  (default on both boards)
//   2'b01 = 20 km (200T only; clamped to 3 km on 50T)
//   2'b10 = Reserved
//   2'b11 = Reserved
//
// USAGE:
//   `include "radar_params.vh"
//   Then reference `RP_FFT_SIZE, `RP_MAX_OUTPUT_BINS, etc.
//
// PHYSICAL CONSTANTS (derived from hardware):
//   ADC clock:           400 MSPS
//   CIC decimation:      4x
//   Processing rate:     100 MSPS (post-DDC)
//   Range per sample:    c / (2 * 100e6) = 1.5 m
//   Decimation factor:   16 (1024 FFT bins -> 64 output bins per segment)
//   Range per dec. bin:  1.5 m * 16 = 24.0 m
//   Carrier frequency:   10.5 GHz
//
// CHIRP BANDWIDTH (Phase 1 target — currently 20 MHz, planned 30 MHz):
//   Range resolution:    c / (2 * BW)
//     20 MHz -> 7.5 m
//     30 MHz -> 5.0 m
//   NOTE: Range resolution is independent of range-per-bin. Resolution
//   determines the minimum separation between two targets; range-per-bin
//   determines the spatial sampling grid.
// ============================================================================

`ifndef RADAR_PARAMS_VH
`define RADAR_PARAMS_VH

// ============================================================================
// BOARD VARIANT — set at synthesis time, NOT runtime
// ============================================================================
// Default to 50T (conservative). Override in top-level or synthesis script:
//   +define+SUPPORT_LONG_RANGE
// or via Vivado: set_property verilog_define {SUPPORT_LONG_RANGE} [current_fileset]

// Note: SUPPORT_LONG_RANGE is a flag define (ifdef/ifndef), not a value.
// `ifndef SUPPORT_LONG_RANGE means 50T (no long range).
// `ifdef SUPPORT_LONG_RANGE means 200T (long range supported).

// ============================================================================
// FFT AND PROCESSING CONSTANTS (fixed, both modes)
// ============================================================================

`define RP_FFT_SIZE             1024    // Range FFT points per segment
`define RP_OVERLAP_SAMPLES      128     // Overlap between adjacent segments
`define RP_SEGMENT_ADVANCE      896     // FFT_SIZE - OVERLAP = 1024 - 128
`define RP_DECIMATION_FACTOR    16      // Range bin decimation (1024 -> 64)
`define RP_BINS_PER_SEGMENT     64      // FFT_SIZE / DECIMATION_FACTOR
`define RP_DOPPLER_FFT_SIZE     16      // Per sub-frame Doppler FFT
`define RP_CHIRPS_PER_FRAME     32      // Total chirps (16 long + 16 short)
`define RP_CHIRPS_PER_SUBFRAME  16      // Chirps per Doppler sub-frame
`define RP_NUM_DOPPLER_BINS     32      // 2 sub-frames * 16 = 32
`define RP_DATA_WIDTH           16      // ADC/processing data width

// ============================================================================
// 3 KM MODE PARAMETERS (both 50T and 200T)
// ============================================================================

`define RP_LONG_CHIRP_SAMPLES_3KM   3000    // 30 us at 100 MSPS
`define RP_LONG_SEGMENTS_3KM        4       // ceil((3000-1024)/896) + 1 = 4
`define RP_OUTPUT_RANGE_BINS_3KM    64      // Downstream pipeline expects 64 range bins (NOTE: will become 128 after 2048-pt FFT upgrade)
`define RP_SHORT_CHIRP_SAMPLES      50      // 0.5 us at 100 MSPS (same both modes)
`define RP_SHORT_SEGMENTS           1       // Single segment for short chirp

// Derived 3 km limits
`define RP_MAX_RANGE_3KM            1536    // 64 bins * 24 m = 1536 m

// ============================================================================
// 20 KM MODE PARAMETERS (200T only)
// ============================================================================

`define RP_LONG_CHIRP_SAMPLES_20KM  13700   // 137 us at 100 MSPS (= listen window)
`define RP_LONG_SEGMENTS_20KM       16      // ceil((13700-1024)/896) + 1 = 16
`define RP_OUTPUT_RANGE_BINS_20KM   1024    // 16 segments * 64 dec. bins each

// Derived 20 km limits
`define RP_MAX_RANGE_20KM           24576   // 1024 bins * 24 m = 24576 m

// ============================================================================
// MAX VALUES (for sizing buffers — compile-time, based on board variant)
// ============================================================================

`ifdef SUPPORT_LONG_RANGE
  `define RP_MAX_SEGMENTS           16
  `define RP_MAX_OUTPUT_BINS        1024
  `define RP_MAX_CHIRP_SAMPLES      13700
`else
  `define RP_MAX_SEGMENTS           4
  `define RP_MAX_OUTPUT_BINS        64
  `define RP_MAX_CHIRP_SAMPLES      3000
`endif

// ============================================================================
// BIT WIDTHS (derived from MAX values)
// ============================================================================

// Segment index: ceil(log2(MAX_SEGMENTS))
//   50T:  log2(4)  = 2 bits
//   200T: log2(16) = 4 bits
`ifdef SUPPORT_LONG_RANGE
  `define RP_SEGMENT_IDX_WIDTH      4
  `define RP_RANGE_BIN_WIDTH        10
  `define RP_CHIRP_MEM_ADDR_W       14      // log2(16*1024) = 14
  `define RP_DOPPLER_MEM_ADDR_W     15      // log2(1024*32) = 15
  `define RP_CFAR_MAG_ADDR_W        15      // log2(1024*32) = 15
`else
  `define RP_SEGMENT_IDX_WIDTH      2
  `define RP_RANGE_BIN_WIDTH        6
  `define RP_CHIRP_MEM_ADDR_W       12      // log2(4*1024) = 12
  `define RP_DOPPLER_MEM_ADDR_W     11      // log2(64*32) = 11
  `define RP_CFAR_MAG_ADDR_W        11      // log2(64*32) = 11
`endif

// Derived depths (for memory declarations)
// Usage: reg [15:0] mem [0:`RP_CHIRP_MEM_DEPTH-1];
`define RP_CHIRP_MEM_DEPTH      (`RP_MAX_SEGMENTS * `RP_FFT_SIZE)
`define RP_DOPPLER_MEM_DEPTH    (`RP_MAX_OUTPUT_BINS * `RP_CHIRPS_PER_FRAME)
`define RP_CFAR_MAG_DEPTH       (`RP_MAX_OUTPUT_BINS * `RP_NUM_DOPPLER_BINS)

// ============================================================================
// CHIRP TIMING DEFAULTS (100 MHz clock cycles)
// ============================================================================
// Reset defaults for host-configurable timing registers.
// Match radar_mode_controller.v parameters and main.cpp STM32 defaults.

`define RP_DEF_LONG_CHIRP_CYCLES    3000    // 30 us
`define RP_DEF_LONG_LISTEN_CYCLES   13700   // 137 us
`define RP_DEF_GUARD_CYCLES         17540   // 175.4 us
`define RP_DEF_SHORT_CHIRP_CYCLES   50      // 0.5 us
`define RP_DEF_SHORT_LISTEN_CYCLES  17450   // 174.5 us
`define RP_DEF_CHIRPS_PER_ELEV      32

// ============================================================================
// BLIND ZONE CONSTANTS (informational, for comments and GUI)
// ============================================================================
// Long chirp blind zone:  c * 30 us / 2 = 4500 m
// Short chirp blind zone: c * 0.5 us / 2 = 75 m

`define RP_LONG_BLIND_ZONE_M        4500
`define RP_SHORT_BLIND_ZONE_M       75

// ============================================================================
// PHYSICAL CONSTANTS (integer-scaled for Verilog — use in comments/assertions)
// ============================================================================
// Range per ADC sample:     1.5 m  (stored as 15 in units of 0.1 m)
// Range per decimated bin: 24.0 m  (stored as 240 in units of 0.1 m)
// Processing rate:         100 MSPS

`define RP_RANGE_PER_SAMPLE_DM      15      // 1.5 m in decimeters
`define RP_RANGE_PER_BIN_DM         240     // 24.0 m in decimeters
`define RP_PROCESSING_RATE_MHZ      100

// ============================================================================
// AGC DEFAULTS
// ============================================================================
`define RP_DEF_AGC_TARGET           200
`define RP_DEF_AGC_ATTACK           1
`define RP_DEF_AGC_DECAY            1
`define RP_DEF_AGC_HOLDOFF          4

// ============================================================================
// CFAR DEFAULTS
// ============================================================================
`define RP_DEF_CFAR_GUARD           2
`define RP_DEF_CFAR_TRAIN           8
`define RP_DEF_CFAR_ALPHA           8'h30   // 3.0 in Q4.4
`define RP_DEF_CFAR_MODE            2'b00   // CA-CFAR

// ============================================================================
// DETECTION DEFAULTS
// ============================================================================
`define RP_DEF_DETECT_THRESHOLD     10000

// ============================================================================
// RANGE MODE ENCODING
// ============================================================================
`define RP_RANGE_MODE_3KM           2'b00
`define RP_RANGE_MODE_20KM          2'b01
`define RP_RANGE_MODE_RSVD2         2'b10
`define RP_RANGE_MODE_RSVD3         2'b11

`endif // RADAR_PARAMS_VH
