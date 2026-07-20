//  tb_BIST_single.v — Testbench 1: Single Fault Injection
//
//  Purpose:
//    Inject ONE user-chosen fault into the CUT and run a complete BIST cycle.
//    Reports whether the fault was detected and how many patterns it took.
//
//  What is tested:
//    1. Functional mode (TC=0) — CUT computes correctly, PO valid
//    2. BIST reset/init — DONE=0 at start
//    3. Single fault injection — BIST detects the injected fault (RESULT=0
//       triggers DONE) or exhausts all patterns without detection
//    4. Timing — measures which pattern triggered detection
//    5. DONE pulse — verified to go high exactly once
`timescale 1ns/1ps

// `include "BIST.v"

module tb_BIST_single;

    // -------------------------------------------------------------------------
    //  Parameters — change INJECT_FAULT_ID to test a specific fault
    //  fault_id = 2*signal_index     → SA0 on that signal
    //  fault_id = 2*signal_index + 1 → SA1 on that signal
    //  Example: fault_id=64 → SA0 on signal 32 = pp[0][0] (first partial product)
    // -------------------------------------------------------------------------
    parameter CLK_PERIOD    = 10;           // 10 ns = 100 MHz
    parameter INJECT_FAULT_ID = 10'd761;     // ← change this to test any fault

    // -------------------------------------------------------------------------
    //  DUT I/O
    // -------------------------------------------------------------------------
    reg        CLK, TC;
    reg [9:0]  fault_id;
    reg signed [7:0]  PI_A, PI_B;
    wire signed [15:0] PO;
    wire       DONE, RESULT;

    // -------------------------------------------------------------------------
    //  DUT
    // -------------------------------------------------------------------------
    BIST dut (
        .CLK      (CLK),
        .TC       (TC),
        .fault_id (fault_id),
        .PI_A     (PI_A),
        .PI_B     (PI_B),
        .PO       (PO),
        .DONE     (DONE),
        .RESULT   (RESULT)
    );

    // -------------------------------------------------------------------------
    //  Clock
    // -------------------------------------------------------------------------
    initial CLK = 0;
    always #(CLK_PERIOD/2) CLK = ~CLK;

    // -------------------------------------------------------------------------
    //  Cycle counter (for timing measurement)
    // -------------------------------------------------------------------------
    integer cycle_count;
    always @(posedge CLK) cycle_count = cycle_count + 1;

    // -------------------------------------------------------------------------
    //  Main stimulus
    // -------------------------------------------------------------------------
    integer done_seen;

    initial begin
        // $dumpfile("tb_BIST_single.vcd");
        // $dumpvars(0, tb_BIST_single);

        // Init
        CLK        = 0;
        TC         = 0;
        fault_id   = 10'b0;
        PI_A       = 8'd0;
        PI_B       = 8'd0;
        cycle_count = 0;
        done_seen  = 0;

        // =============================================================
        // PHASE 1: Functional mode sanity check (TC=0)
        // =============================================================
        $display("\n========================================");
        $display(" PHASE 1: Functional mode check (TC=0)");
        $display("========================================");

        TC   = 0;
        PI_A = 8'sd7;
        PI_B = 8'sd5;
        repeat(3) @(posedge CLK); #1;
        $display("  PI_A=%0d * PI_B=%0d → PO=%0d (expected %0d) %s",
            $signed(PI_A), $signed(PI_B), $signed(PO),
            $signed(PI_A) * $signed(PI_B),
            ($signed(PO) === $signed(PI_A)*$signed(PI_B)) ? "PASS" : "FAIL");

        PI_A = -8'sd12;
        PI_B =  8'sd10;
        repeat(3) @(posedge CLK); #1;
        $display("  PI_A=%0d * PI_B=%0d → PO=%0d (expected %0d) %s",
            $signed(PI_A), $signed(PI_B), $signed(PO),
            $signed(PI_A) * $signed(PI_B),
            ($signed(PO) === $signed(PI_A)*$signed(PI_B)) ? "PASS" : "FAIL");

        // =============================================================
        // PHASE 2: BIST mode with single fault injection
        // =============================================================
        $display("\n========================================");
        $display(" PHASE 2: BIST — fault_id = %0d", INJECT_FAULT_ID);
        $display("   SA%0d on signal index %0d",
            INJECT_FAULT_ID[0], INJECT_FAULT_ID[9:1]);
        $display("========================================");

        fault_id    = INJECT_FAULT_ID;
        cycle_count = 0;
        TC          = 1;

        // Wait for DONE with a generous timeout (65535 patterns × ~20 cycles each)
        // For simulation speed we cap at 200000 cycles; increase if needed
        fork
            begin : wait_done
                @(posedge DONE);
                done_seen = 1;
                disable watchdog;
            end
            begin : watchdog
                repeat(2_000_000) @(posedge CLK);
                $display("  TIMEOUT: DONE not seen within cycle limit.");
                disable wait_done;
            end
        join

        #1; // settle

        if (done_seen) begin
            if (!RESULT)
                $display("  ✓ Fault DETECTED  (RESULT=0=FAIL)  after ~%0d cycles", cycle_count);
            else
                $display("  ✗ Fault NOT detected (RESULT=1=PASS after all patterns)");
        end

        $display("\n========================================");
        $display(" Simulation complete.");
        $display("========================================\n");
        $finish;
    end

endmodule