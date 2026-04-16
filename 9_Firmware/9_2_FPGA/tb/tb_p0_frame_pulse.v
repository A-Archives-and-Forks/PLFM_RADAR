`timescale 1ns / 1ps

// ============================================================================
// ADVERSARIAL TESTBENCH: frame_complete Pulse Width (P0 Fix #7)
// ============================================================================
// Tests the falling-edge pulse detection pattern used in doppler_processor.v
// (lines 533-551) for the frame_complete signal.
//
// The OLD code held frame_complete as a continuous level whenever the
// Doppler processor was idle. This caused the AGC (rx_gain_control) to
// re-evaluate every clock with zeroed accumulators, collapsing gain control.
//
// The FIX detects the falling edge of processing_active:
//   assign processing_active = (state != S_IDLE);
//   reg processing_active_prev;
//   always @(posedge clk or negedge reset_n)
//       processing_active_prev <= processing_active;
//   assign frame_complete = (~processing_active & processing_active_prev);
//
// This DUT wrapper replicates the EXACT pattern from doppler_processor.v.
// The adversarial tests drive the state input and verify:
//   - Pulse width is EXACTLY 1 clock cycle
//   - No pulse during extended idle
//   - No pulse on reset deassertion
//   - Back-to-back frame completions produce distinct pulses
//   - State transitions not touching S_IDLE produce no pulse
//   - OLD behavior (continuous level) is regressed
// ============================================================================

// ── DUT: Exact replica of doppler_processor.v frame_complete logic ──
module frame_complete_dut (
    input wire clk,
    input wire reset_n,
    input wire [3:0] state,         // Mimic doppler FSM state input
    output wire processing_active,
    output wire frame_complete
);
    // S_IDLE encoding from doppler_processor_optimized
    localparam [3:0] S_IDLE = 4'd0;

    assign processing_active = (state != S_IDLE);

    reg processing_active_prev;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            processing_active_prev <= 1'b0;
        else
            processing_active_prev <= processing_active;
    end

    assign frame_complete = (~processing_active & processing_active_prev);
endmodule


// ── TESTBENCH ────────────────────────────────────────────────
module tb_p0_frame_pulse;

    localparam CLK_PERIOD = 10.0;   // 100 MHz

    // Doppler FSM state encodings (from doppler_processor_optimized)
    localparam [3:0] S_IDLE       = 4'd0;
    localparam [3:0] S_ACCUMULATE = 4'd1;
    localparam [3:0] S_WINDOW     = 4'd2;
    localparam [3:0] S_FFT        = 4'd3;
    localparam [3:0] S_OUTPUT     = 4'd4;
    localparam [3:0] S_NEXT_BIN   = 4'd5;

    // ── Test bookkeeping ─────────────────────────────────────
    integer pass_count = 0;
    integer fail_count = 0;
    integer test_num   = 0;
    integer i;

    task check;
        input cond;
        input [511:0] label;
    begin
        test_num = test_num + 1;
        if (cond) begin
            $display("[PASS] Test %0d: %0s", test_num, label);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Test %0d: %0s", test_num, label);
            fail_count = fail_count + 1;
        end
    end
    endtask

    // ── DUT signals ──────────────────────────────────────────
    reg        clk = 0;
    reg        reset_n = 0;
    reg  [3:0] state = S_IDLE;
    wire       processing_active;
    wire       frame_complete;

    always #(CLK_PERIOD/2) clk = ~clk;

    frame_complete_dut dut (
        .clk(clk),
        .reset_n(reset_n),
        .state(state),
        .processing_active(processing_active),
        .frame_complete(frame_complete)
    );

    // ── Helper ───────────────────────────────────────────────
    task wait_n;
        input integer n;
        integer k;
    begin
        for (k = 0; k < n; k = k + 1) @(posedge clk);
    end
    endtask

    // ── Count frame_complete pulses over N clocks ────────────
    integer pulse_count;

    task count_pulses;
        input integer n_clocks;
        output integer count;
        integer c;
    begin
        count = 0;
        for (c = 0; c < n_clocks; c = c + 1) begin
            @(posedge clk);
            if (frame_complete) count = count + 1;
        end
    end
    endtask

    // ══════════════════════════════════════════════════════════
    // MAIN TEST SEQUENCE
    // ══════════════════════════════════════════════════════════
    initial begin
        $dumpfile("tb_p0_frame_pulse.vcd");
        $dumpvars(0, tb_p0_frame_pulse);

        // ── RESET ────────────────────────────────────────────
        state   = S_IDLE;
        reset_n = 0;
        #100;
        reset_n = 1;
        @(posedge clk);
        @(posedge clk);

        // ──────────────────────────────────────────────────────
        // TEST 1: No pulse on reset deassertion
        // ──────────────────────────────────────────────────────
        $display("\n=== TEST 1: Reset deassertion ===");
        // processing_active = 0 (state = S_IDLE)
        // processing_active_prev was reset to 0
        // frame_complete = ~0 & 0 = 0
        check(frame_complete == 0, "No pulse on reset deassertion (both 0)");

        // ──────────────────────────────────────────────────────
        // TEST 2: No pulse during extended idle
        // ──────────────────────────────────────────────────────
        $display("\n=== TEST 2: Extended idle ===");
        count_pulses(200, pulse_count);
        check(pulse_count == 0, "No pulse during 200 clocks of continuous idle");

        // ──────────────────────────────────────────────────────
        // TEST 3: Single frame completion — pulse width = 1
        // ──────────────────────────────────────────────────────
        $display("\n=== TEST 3: Single frame completion ===");

        // Enter active state
        @(posedge clk); #1;
        state = S_ACCUMULATE;
        wait_n(5);
        check(processing_active == 1, "Active: processing_active = 1");
        check(frame_complete == 0,    "Active: no frame_complete while active");

        // Stay active for 50 clocks (various states)
        #1; state = S_WINDOW;   wait_n(10);
        #1; state = S_FFT;      wait_n(10);
        #1; state = S_OUTPUT;   wait_n(10);
        #1; state = S_NEXT_BIN; wait_n(10);
        check(frame_complete == 0, "Active (multi-state): no frame_complete");

        // Return to idle — should produce exactly 1 pulse
        #1; state = S_IDLE;
        @(posedge clk);
        // On this edge: processing_active = 0, processing_active_prev = 1
        // frame_complete = ~0 & 1 = 1
        check(frame_complete == 1, "Completion: frame_complete fires");

        @(posedge clk);
        // Now: processing_active_prev catches up to 0
        // frame_complete = ~0 & 0 = 0
        check(frame_complete == 0, "Completion: pulse is EXACTLY 1 cycle wide");

        // Verify no more pulses
        count_pulses(100, pulse_count);
        check(pulse_count == 0, "Post-completion: no re-fire during idle");

        // ──────────────────────────────────────────────────────
        // TEST 4: Back-to-back frame completions
        // ──────────────────────────────────────────────────────
        $display("\n=== TEST 4: Back-to-back completions ===");

        begin : backtoback_test
            integer total_pulses;
            total_pulses = 0;

            // Do 5 rapid frame cycles
            for (i = 0; i < 5; i = i + 1) begin
                // Go active
                @(posedge clk); #1;
                state = S_ACCUMULATE;
                wait_n(3);

                // Return to idle
                #1; state = S_IDLE;
                @(posedge clk);
                if (frame_complete) total_pulses = total_pulses + 1;
                @(posedge clk); // pulse should be gone
                if (frame_complete) begin
                    $display("  [WARN] frame %0d: pulse persisted > 1 cycle", i);
                end
            end

            check(total_pulses == 5, "Back-to-back: exactly 5 pulses for 5 completions");
        end

        // ──────────────────────────────────────────────────────
        // TEST 5: State transitions not touching S_IDLE
        // ──────────────────────────────────────────────────────
        $display("\n=== TEST 5: Non-idle transitions ===");

        @(posedge clk); #1;
        state = S_ACCUMULATE;
        wait_n(3);

        // Cycle through active states without returning to idle
        begin : nonidle_test
            integer nonidle_pulses;
            nonidle_pulses = 0;

            #1; state = S_WINDOW;
            @(posedge clk);
            if (frame_complete) nonidle_pulses = nonidle_pulses + 1;

            #1; state = S_FFT;
            @(posedge clk);
            if (frame_complete) nonidle_pulses = nonidle_pulses + 1;

            #1; state = S_OUTPUT;
            @(posedge clk);
            if (frame_complete) nonidle_pulses = nonidle_pulses + 1;

            #1; state = S_NEXT_BIN;
            @(posedge clk);
            if (frame_complete) nonidle_pulses = nonidle_pulses + 1;

            #1; state = S_ACCUMULATE;
            wait_n(10);
            count_pulses(10, pulse_count);
            nonidle_pulses = nonidle_pulses + pulse_count;

            check(nonidle_pulses == 0,
                  "Non-idle transitions: zero pulses (all states active)");
        end

        // Return to idle (one pulse expected)
        #1; state = S_IDLE;
        @(posedge clk);
        check(frame_complete == 1, "Cleanup: pulse on final idle transition");
        @(posedge clk);

        // ──────────────────────────────────────────────────────
        // TEST 6: Long active period — no premature pulse
        // ──────────────────────────────────────────────────────
        $display("\n=== TEST 6: Long active period ===");

        @(posedge clk); #1;
        state = S_FFT;

        count_pulses(500, pulse_count);
        check(pulse_count == 0, "Long active (500 clocks): no premature pulse");

        #1; state = S_IDLE;
        @(posedge clk);
        check(frame_complete == 1, "Long active → idle: pulse fires");
        @(posedge clk);
        check(frame_complete == 0, "Long active → idle: single cycle only");

        // ──────────────────────────────────────────────────────
        // TEST 7: Reset during active state
        // ──────────────────────────────────────────────────────
        $display("\n=== TEST 7: Reset during active ===");

        @(posedge clk); #1;
        state = S_ACCUMULATE;
        wait_n(5);

        // Assert reset while active
        reset_n = 0;
        #50;
        // During reset: processing_active_prev forced to 0
        // state still = S_ACCUMULATE, processing_active = 1
        reset_n = 1;
        @(posedge clk);
        @(posedge clk);
        // After reset release: prev = 0, active = 1
        // frame_complete = ~1 & 0 = 0 (no spurious pulse)
        check(frame_complete == 0, "Reset during active: no spurious pulse");

        // Now go idle — should pulse
        #1; state = S_IDLE;
        @(posedge clk);
        check(frame_complete == 1, "Reset recovery: pulse on idle after active");
        @(posedge clk);

        // ──────────────────────────────────────────────────────
        // TEST 8: REGRESSION — old continuous-level behavior
        // ──────────────────────────────────────────────────────
        $display("\n=== TEST 8: REGRESSION ===");
        // OLD code: frame_complete = (state == S_IDLE && frame_buffer_full == 0)
        // This held frame_complete HIGH for the entire idle period.
        // With AGC sampling frame_complete, this caused re-evaluation every clock.
        //
        // The FIX produces a 1-cycle pulse. We've proven:
        //   - Pulse width = 1 cycle (Test 3)
        //   - No re-fire during idle (Test 2, 3)
        //   - Old behavior would have frame_complete = 1 for 200+ clocks (Test 2)
        //
        // Quantify: old code would produce 200 "events" over 200 idle clocks.
        // New code produces 0. This is the fix.

        state = S_IDLE;
        count_pulses(200, pulse_count);
        check(pulse_count == 0,
              "REGRESSION: 0 pulses in 200 idle clocks (old code: 200)");

        // ══════════════════════════════════════════════════════
        // SUMMARY
        // ══════════════════════════════════════════════════════
        $display("\n============================================");
        $display("  P0 Fix #7: frame_complete Pulse Tests");
        $display("============================================");
        $display("  PASSED: %0d", pass_count);
        $display("  FAILED: %0d", fail_count);
        $display("============================================");

        if (fail_count > 0)
            $display("RESULT: FAIL");
        else
            $display("RESULT: PASS");

        $finish;
    end

    // Timeout watchdog
    initial begin
        #500000;
        $display("[FAIL] TIMEOUT: simulation exceeded 500us");
        $finish;
    end

endmodule
