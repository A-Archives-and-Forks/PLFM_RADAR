`timescale 1ns / 1ps

// ============================================================================
// ADVERSARIAL TESTBENCH: cdc_async_fifo (P0 Fix #1)
// ============================================================================
// Actively tries to BREAK the async FIFO that replaced the flawed
// Gray-encoded CDC for the DDC 400→100 MHz sample path.
//
// Attack vectors:
//   1. Read on empty FIFO — no spurious rd_valid
//   2. Single write/read — basic data integrity
//   3. Fill to capacity — wr_full asserts correctly
//   4. Overflow — write-when-full must be rejected, no corruption
//   5. Ordered streaming — FIFO order preserved under sustained load
//   6. Reset mid-transfer — clean recovery, no stale data
//   7. Burst writes at max wr_clk rate — stress back-pressure
//   8. wr_full deasserts promptly after read
//   9. Alternating single-entry traffic — throughput = 1
//  10. Pathological data patterns — all-ones, alternating bits
// ============================================================================

module tb_p0_async_fifo;

    localparam WR_PERIOD = 2.5;     // 400 MHz source clock
    localparam RD_PERIOD = 10.0;    // 100 MHz destination clock
    localparam WIDTH = 18;
    localparam DEPTH = 8;

    // ── Test bookkeeping ─────────────────────────────────────
    integer pass_count = 0;
    integer fail_count = 0;
    integer test_num   = 0;
    integer i, j;

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
    reg              wr_clk = 0;
    reg              rd_clk = 0;
    reg              wr_reset_n = 0;
    reg              rd_reset_n = 0;
    reg  [WIDTH-1:0] wr_data = 0;
    reg              wr_en = 0;
    wire             wr_full;
    wire [WIDTH-1:0] rd_data;
    wire             rd_valid;
    reg              rd_ack = 0;

    always #(WR_PERIOD/2) wr_clk = ~wr_clk;
    always #(RD_PERIOD/2) rd_clk = ~rd_clk;

    cdc_async_fifo #(
        .WIDTH(WIDTH), .DEPTH(DEPTH), .ADDR_BITS(3)
    ) dut (
        .wr_clk(wr_clk),   .wr_reset_n(wr_reset_n),
        .wr_data(wr_data),  .wr_en(wr_en),  .wr_full(wr_full),
        .rd_clk(rd_clk),    .rd_reset_n(rd_reset_n),
        .rd_data(rd_data),  .rd_valid(rd_valid), .rd_ack(rd_ack)
    );

    // ── Helper tasks ─────────────────────────────────────────
    task do_reset;
    begin
        wr_en = 0; rd_ack = 0; wr_data = 0;
        wr_reset_n = 0; rd_reset_n = 0;
        #100;
        wr_reset_n = 1; rd_reset_n = 1;
        #50;
    end
    endtask

    task wait_wr_n;
        input integer n;
        integer k;
    begin
        for (k = 0; k < n; k = k + 1) @(posedge wr_clk);
    end
    endtask

    task wait_rd_n;
        input integer n;
        integer k;
    begin
        for (k = 0; k < n; k = k + 1) @(posedge rd_clk);
    end
    endtask

    // ── Read one entry with timeout ──────────────────────────
    reg [WIDTH-1:0] read_result;
    reg read_ok;

    task read_one;
        output [WIDTH-1:0] data_out;
        output             valid_out;
        integer timeout;
    begin
        rd_ack = 1;
        valid_out = 0;
        data_out = {WIDTH{1'bx}};
        for (timeout = 0; timeout < 20; timeout = timeout + 1) begin
            @(posedge rd_clk);
            if (rd_valid) begin
                data_out  = rd_data;
                valid_out = 1;
                timeout   = 999; // break
            end
        end
        @(posedge rd_clk);
        rd_ack = 0;
    end
    endtask

    // ── Drain FIFO, return count of entries read ─────────────
    integer drain_count;
    reg [WIDTH-1:0] drain_buf [0:15];

    task drain_fifo;
        output integer count;
        integer t;
    begin
        count = 0;
        rd_ack = 1;
        for (t = 0; t < 60; t = t + 1) begin
            @(posedge rd_clk);
            if (rd_valid && count < 16) begin
                drain_buf[count] = rd_data;
                count = count + 1;
            end
        end
        rd_ack = 0;
        wait_rd_n(3);
    end
    endtask

    // ══════════════════════════════════════════════════════════
    // MAIN TEST SEQUENCE
    // ══════════════════════════════════════════════════════════
    initial begin
        $dumpfile("tb_p0_async_fifo.vcd");
        $dumpvars(0, tb_p0_async_fifo);

        do_reset;

        // ──────────────────────────────────────────────────────
        // GROUP 1: Empty FIFO — no spurious rd_valid
        // ──────────────────────────────────────────────────────
        $display("\n=== GROUP 1: Empty FIFO behavior ===");

        // 1a: rd_valid must be 0 when nothing written
        wait_rd_n(10);
        check(rd_valid == 0, "Empty FIFO: rd_valid is 0 (no writes)");

        // 1b: rd_ack on empty must not produce spurious valid
        rd_ack = 1;
        wait_rd_n(10);
        check(rd_valid == 0, "Empty FIFO: rd_ack on empty produces no valid");
        rd_ack = 0;
        wait_rd_n(3);

        // ──────────────────────────────────────────────────────
        // GROUP 2: Single write/read
        // ──────────────────────────────────────────────────────
        $display("\n=== GROUP 2: Single write/read ===");

        @(posedge wr_clk); #1;
        wr_data = 18'h2ABCD;
        wr_en   = 1;
        @(posedge wr_clk); #1;
        wr_en = 0;

        // Wait for CDC propagation
        wait_rd_n(6);
        check(rd_valid == 1, "Single write: rd_valid asserted");
        check(rd_data == 18'h2ABCD, "Single write: data integrity");

        // ACK and verify deassert
        #1; rd_ack = 1;
        @(posedge rd_clk); #1;
        rd_ack = 0;
        wait_rd_n(6);
        check(rd_valid == 0, "Single write: rd_valid deasserts after ack+empty");

        // ──────────────────────────────────────────────────────
        // GROUP 3: Fill to capacity
        // ──────────────────────────────────────────────────────
        // NOTE: This FIFO uses a pre-fetch show-ahead architecture.
        // When the FIFO goes from empty to non-empty, the read domain
        // auto-presents the first entry into rd_data_reg, advancing
        // rd_ptr by 1.  This frees one slot in the underlying memory,
        // so wr_full requires DEPTH+1 writes (DEPTH in mem + 1 in the
        // output register).  This is necessary because a combinational
        // read from mem across clock domains would be CDC-unsafe.
        $display("\n=== GROUP 3: Fill to capacity ===");
        do_reset;

        // Write DEPTH entries
        for (i = 0; i < DEPTH; i = i + 1) begin
            @(posedge wr_clk); #1;
            wr_data = i[17:0] + 18'h100;
            wr_en   = 1;
        end
        @(posedge wr_clk); #1;
        wr_en = 0;

        // Wait for auto-present round-trip through both synchronizers
        wait_wr_n(12);

        // After auto-present, rd_ptr advanced by 1 → 1 slot freed → not full yet
        check(wr_full == 0, "Pre-fetch show-ahead: DEPTH writes, 1 auto-present frees slot");

        // Write one more entry into the freed slot → now truly full
        @(posedge wr_clk); #1;
        wr_data = 18'hFACE;
        wr_en   = 1;
        @(posedge wr_clk); #1;
        wr_en = 0;

        wait_wr_n(6);
        check(wr_full == 1, "Fill-to-full: wr_full asserted after DEPTH+1 writes");

        // ──────────────────────────────────────────────────────
        // GROUP 4: Overflow — write when full
        // ──────────────────────────────────────────────────────
        $display("\n=== GROUP 4: Overflow protection ===");

        // Attempt to write 3 more entries while full
        for (i = 0; i < 3; i = i + 1) begin
            @(posedge wr_clk); #1;
            wr_data = 18'h3DEAD + i[17:0];
            wr_en   = 1;
        end
        @(posedge wr_clk); #1;
        wr_en = 0;

        // Drain and verify DEPTH+1 entries (DEPTH mem + 1 output register)
        drain_fifo(drain_count);
        check(drain_count == DEPTH + 1, "Overflow: exactly DEPTH+1 entries (overflow rejected)");

        // Verify data integrity — check first DEPTH entries + the extra FACE entry
        begin : overflow_data_check
            reg data_ok;
            data_ok = 1;
            // First entry is the auto-presented one (index 0 from Group 3)
            if (drain_buf[0] !== 18'h100) begin
                $display("  overflow corruption at [0]: expected %h, got %h",
                         18'h100, drain_buf[0]);
                data_ok = 0;
            end
            // Next DEPTH-1 entries are indices 1..DEPTH-1
            for (i = 1; i < DEPTH; i = i + 1) begin
                if (drain_buf[i] !== i[17:0] + 18'h100) begin
                    $display("  overflow corruption at [%0d]: expected %h, got %h",
                             i, i[17:0] + 18'h100, drain_buf[i]);
                    data_ok = 0;
                end
            end
            // Last entry is the FACE entry from the +1 write
            if (drain_buf[DEPTH] !== 18'hFACE) begin
                $display("  overflow corruption at [%0d]: expected %h, got %h",
                         DEPTH, 18'hFACE, drain_buf[DEPTH]);
                data_ok = 0;
            end
            check(data_ok, "Overflow: all DEPTH+1 entries data intact (no corruption)");
        end

        // ──────────────────────────────────────────────────────
        // GROUP 5: Data ordering under sustained streaming
        // ──────────────────────────────────────────────────────
        $display("\n=== GROUP 5: Sustained streaming order ===");
        do_reset;

        // Simulate CIC-decimated DDC output: 1 sample per 4 wr_clks
        // Reader continuously ACKs (rate-matched at 100 MHz)
        begin : stream_test
            reg [WIDTH-1:0] expected_val;
            integer read_idx;
            reg ordering_ok;

            ordering_ok = 1;
            read_idx   = 0;

            fork
                // Writer: 32 samples, 1 per 4 wr_clks (rate-matched to rd_clk)
                begin : stream_writer
                    integer w;
                    for (w = 0; w < 32; w = w + 1) begin
                        @(posedge wr_clk); #1;
                        wr_data = w[17:0] + 18'h1000;
                        wr_en   = 1;
                        @(posedge wr_clk); #1;
                        wr_en = 0;
                        wait_wr_n(2); // 4 wr_clks total per sample
                    end
                end

                // Reader: continuously consume at rd_clk rate
                begin : stream_reader
                    integer rd_t;
                    rd_ack = 1;
                    for (rd_t = 0; rd_t < 500 && read_idx < 32; rd_t = rd_t + 1) begin
                        @(posedge rd_clk);
                        if (rd_valid) begin
                            expected_val = read_idx[17:0] + 18'h1000;
                            if (rd_data !== expected_val) begin
                                $display("  stream order error at [%0d]: expected %h, got %h",
                                         read_idx, expected_val, rd_data);
                                ordering_ok = 0;
                            end
                            read_idx = read_idx + 1;
                        end
                    end
                    #1; rd_ack = 0;
                end
            join

            check(read_idx == 32, "Streaming: all 32 samples received");
            check(ordering_ok,    "Streaming: FIFO order preserved");
        end

        // ──────────────────────────────────────────────────────
        // GROUP 6: Reset mid-transfer
        // ──────────────────────────────────────────────────────
        $display("\n=== GROUP 6: Reset mid-transfer ===");
        do_reset;

        // Write 4 entries
        for (i = 0; i < 4; i = i + 1) begin
            @(posedge wr_clk); #1;
            wr_data = i[17:0] + 18'hAA00;
            wr_en   = 1;
        end
        @(posedge wr_clk); #1;
        wr_en = 0;
        wait_wr_n(3);

        // Assert reset while data is in FIFO
        wr_reset_n = 0; rd_reset_n = 0;
        #50;
        wr_reset_n = 1; rd_reset_n = 1;
        #50;

        // 6a: FIFO must be empty after reset
        wait_rd_n(10);
        check(rd_valid == 0, "Reset mid-xfer: FIFO empty (no stale data)");
        check(wr_full == 0,  "Reset mid-xfer: wr_full deasserted");

        // 6b: New write after reset must work
        @(posedge wr_clk); #1;
        wr_data = 18'h3CAFE;
        wr_en   = 1;
        @(posedge wr_clk); #1;
        wr_en = 0;

        wait_rd_n(6);
        check(rd_valid == 1,           "Reset recovery: rd_valid for new write");
        check(rd_data == 18'h3CAFE,    "Reset recovery: correct data");
        #1; rd_ack = 1; @(posedge rd_clk); #1; rd_ack = 0;
        wait_rd_n(5);

        // ──────────────────────────────────────────────────────
        // GROUP 7: Burst writes at max wr_clk rate
        // ──────────────────────────────────────────────────────
        $display("\n=== GROUP 7: Max-rate burst ===");
        do_reset;

        // Write 7 entries back-to-back (1 per wr_clk, no decimation)
        for (i = 0; i < 7; i = i + 1) begin
            @(posedge wr_clk); #1;
            wr_data = i[17:0] + 18'hB000;
            wr_en   = 1;
        end
        @(posedge wr_clk); #1;
        wr_en = 0;

        // Drain and count
        drain_fifo(drain_count);
        check(drain_count == 7, "Burst: all 7 entries received (no drops)");

        // ──────────────────────────────────────────────────────
        // GROUP 8: wr_full deasserts after read
        // ──────────────────────────────────────────────────────
        $display("\n=== GROUP 8: wr_full release ===");
        do_reset;

        // Fill FIFO: DEPTH entries first
        for (i = 0; i < DEPTH; i = i + 1) begin
            @(posedge wr_clk); #1;
            wr_data = i[17:0];
            wr_en   = 1;
        end
        @(posedge wr_clk); #1;
        wr_en = 0;

        // Wait for auto-present round-trip
        wait_wr_n(12);

        // Write the +1 entry (into the slot freed by auto-present)
        @(posedge wr_clk); #1;
        wr_data = 18'h3BEEF;
        wr_en   = 1;
        @(posedge wr_clk); #1;
        wr_en = 0;
        wait_wr_n(6);
        check(wr_full == 1, "wr_full release: initially full (DEPTH+1 writes)");

        // Read one entry (ACK the auto-presented data)
        #1; rd_ack = 1;
        wait_rd_n(2);
        #1; rd_ack = 0;

        // Wait for rd_ptr sync back to wr domain (2 wr_clk cycles + margin)
        wait_wr_n(10);
        check(wr_full == 0, "wr_full release: deasserts after 1 read");

        // Drain rest
        drain_fifo(drain_count);
        wait_rd_n(5);

        // ──────────────────────────────────────────────────────
        // GROUP 9: Alternating single-entry throughput
        // ──────────────────────────────────────────────────────
        $display("\n=== GROUP 9: Alternating single-entry ===");
        do_reset;

        begin : alt_test
            reg alt_ok;
            reg alt_got_valid;
            integer rd_w;
            alt_ok = 1;
            for (i = 0; i < 12; i = i + 1) begin
                // Write 1
                @(posedge wr_clk); #1;
                wr_data = i[17:0] + 18'hC000;
                wr_en   = 1;
                @(posedge wr_clk); #1;
                wr_en = 0;

                // Read 1 — wait for auto-present with rd_ack=0, then pulse ack
                rd_ack = 0;
                alt_got_valid = 0;
                for (rd_w = 0; rd_w < 20; rd_w = rd_w + 1) begin
                    @(posedge rd_clk);
                    if (rd_valid && !alt_got_valid) begin
                        alt_got_valid = 1;
                        if (rd_data !== i[17:0] + 18'hC000) begin
                            $display("  alt[%0d]: data mismatch", i);
                            alt_ok = 0;
                        end
                        rd_w = 999; // break
                    end
                end
                if (!alt_got_valid) begin
                    $display("  alt[%0d]: no rd_valid after write", i);
                    alt_ok = 0;
                end
                // Consume the entry
                #1; rd_ack = 1;
                @(posedge rd_clk); #1;
                rd_ack = 0;
                wait_rd_n(2);
            end
            check(alt_ok, "Alternating: 12 single-entry cycles all correct");
        end

        // ──────────────────────────────────────────────────────
        // GROUP 10: Pathological data patterns
        // ──────────────────────────────────────────────────────
        $display("\n=== GROUP 10: Pathological data patterns ===");
        do_reset;

        begin : patho_test
            reg patho_ok;
            reg patho_seen;
            reg [WIDTH-1:0] patterns [0:4];
            integer rd_w;
            patterns[0] = 18'h3FFFF;   // all ones
            patterns[1] = 18'h00000;   // all zeros
            patterns[2] = 18'h2AAAA;   // alternating 10...
            patterns[3] = 18'h15555;   // alternating 01...
            patterns[4] = 18'h20001;   // MSB + LSB set

            patho_ok = 1;
            // Write all 5 patterns
            for (i = 0; i < 5; i = i + 1) begin
                @(posedge wr_clk); #1;
                wr_data = patterns[i];
                wr_en   = 1;
            end
            @(posedge wr_clk); #1;
            wr_en = 0;

            // Read one at a time: wait for auto-present, check, ack
            rd_ack = 0;
            for (i = 0; i < 5; i = i + 1) begin
                patho_seen = 0;
                for (rd_w = 0; rd_w < 30; rd_w = rd_w + 1) begin
                    @(posedge rd_clk);
                    if (rd_valid && !patho_seen) begin
                        patho_seen = 1;
                        if (rd_data !== patterns[i]) begin
                            $display("  pattern[%0d]: expected %h got %h",
                                     i, patterns[i], rd_data);
                            patho_ok = 0;
                        end
                        rd_w = 999; // break
                    end
                end
                if (!patho_seen) begin
                    $display("  pattern[%0d]: no valid", i);
                    patho_ok = 0;
                end
                // Consume the entry
                #1; rd_ack = 1;
                @(posedge rd_clk); #1;
                rd_ack = 0;
            end
            check(patho_ok, "Pathological: all 5 bit-patterns survive CDC");
        end

        // ══════════════════════════════════════════════════════
        // SUMMARY
        // ══════════════════════════════════════════════════════
        $display("\n============================================");
        $display("  P0 Fix #1: Async FIFO Adversarial Tests");
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
        #1000000;
        $display("[FAIL] TIMEOUT: simulation exceeded 1ms");
        $finish;
    end

endmodule
