/* BIST — Built-In Self-Test Top Module
 Integrates: LFSR_16 (pattern generator) + MUL8_FAULT_INJECT (CUT) + ORA

 FSM state diagram:

   !TC ──────────────────────────────────────────────────────────► RESET

   RESET ──(always)──► RUN

   RUN ──(count==65535)──► RESET  (all patterns exhausted, no fault)
         ──(else)──────────► LFSR

   LFSR ──(always)──► ORA_EN     (advance LFSR, pulse enable next state)

   ORA_EN ──(always)──► ORA_WAIT (enable ORA for exactly 1 cycle)

   ORA_WAIT ──(ORA_DONE & !RESULT)──► RESET  (fault detected → stop)
              ──(ORA_DONE &  RESULT)──► RUN    (pass → next pattern)
              ──(!ORA_DONE)──────────► ORA_WAIT
*/
`timescale 1ns/1ps

`include "LFSR_16.v"
`include "CUT.v"
`include "ORA.v"

module BIST (
    input             CLK,
    input             TC,           // 1 = BIST mode, 0 = normal/functional mode
    input      [9:0]  fault_id,     // Fault to inject (0..769); used when TC=1
    input  signed [7:0] PI_A,       // Primary input A (used when TC=0)
    input  signed [7:0] PI_B,       // Primary input B (used when TC=0)
    output reg signed [15:0] PO,    // Primary output (functional mode)
    output reg        DONE,         // Pulses high when BIST completes
    output            RESULT        // 1=PASS (no fault found), 0=FAIL (fault found)
);

    reg [2:0] state;
    localparam RESET    = 3'd0;   // Reset all sub-modules
    localparam RUN      = 3'd1;   // Check if done; decide next pattern
    localparam LFSR     = 3'd2;   // Advance LFSR by one step
    localparam ORA_EN   = 3'd3;   // Pulse ORA enable for one cycle
    localparam ORA_WAIT = 3'd4;   // Wait for ORA to finish
    localparam FAULT_DONE = 3'd5; 

    // Control Registers
    reg LFSR_RESET;
    reg ORA_RESET;
    reg ORA_ENABLE;
    reg LFSR_NEXT;
    reg CUT_FAULT_EN;

    reg [15:0] count;
    // reg [7:0]  pat_addr;

    // Wires to connect between two modules
    wire [15:0] LFSR_OUT;
    wire [15:0] CUT_IN;
    wire [15:0] CUT_OUT;
    wire        ORA_DONE;

    // 2x1 MUX to select between primary input LFSR input
    assign CUT_IN = TC ? LFSR_OUT : {PI_A, PI_B};


    //  Sub-module instantiation
    LFSR_16 lfsr(.CLK(CLK), .RST(LFSR_RESET), .INCREMENT(LFSR_NEXT), .Q(LFSR_OUT));
    CUT cut(.M(CUT_OUT), .A(CUT_IN[15:8]), .B(CUT_IN[7:0]), .fault_en(CUT_FAULT_EN), .fault_id(fault_id));
    ORA ora(.CLK(CLK), .RST(ORA_RESET), .M(CUT_OUT), .mem_address(count), .enable(ORA_ENABLE), .result(RESULT), .done(ORA_DONE));

    always @(posedge CLK) begin

        // Default: deassert all one-cycle pulses every clock
        LFSR_NEXT  <= 1'b0;
        ORA_ENABLE <= 1'b0;
        LFSR_RESET <= 1'b0;
        ORA_RESET  <= 1'b0;

        if (!TC) begin
            // Functional mode: disable BIST, hold in reset
            CUT_FAULT_EN <= 1'b0;
            DONE         <= 1'b0;
            count        <= 16'b0;
            // pat_addr     <= 8'b0;
            state        <= RESET;
            PO <= CUT_OUT; // Functional-mode output (TC=0)
        end
        else begin
            case (state)
                //  RESET: pulse resets to all sub-modules for one cycle
                RESET: begin
                    LFSR_RESET   <= 1'b1;
                    ORA_RESET    <= 1'b1;
                    CUT_FAULT_EN <= 1'b0;
                    DONE         <= 1'b0;
                    count        <= 16'b0;
                    // pat_addr     <= 8'b0;
                    state        <= RUN;
                end

                //  RUN: check if all 65535 patterns have been applied.
                //  If yes → BIST done with no fault detected (PASS).
                //  If no  → advance to next pattern.
                RUN: begin
                    CUT_FAULT_EN <= 1'b1;
                    if (count == 16'd65535) begin
                        DONE  <= 1'b1;
                        state <= FAULT_DONE;   // return to idle/reset
                    end
                    else begin
                        state <= LFSR;
                    end
                end

                //  LFSR: advance the LFSR by one step; bump counters
                LFSR: begin
                    LFSR_NEXT <= 1'b1;
                    count     <= count    + 1'b1;
                    // pat_addr  <= pat_addr + 1'b1;  // wraps naturally at 255
                    state     <= ORA_EN;
                end

                //  ORA_EN: assert ORA enable for exactly one clock cycle.
                //  ORA latches CUT_OUT and begins its 16-cycle SISR computation.
                ORA_EN: begin
                    ORA_ENABLE <= 1'b1;
                    state      <= ORA_WAIT;
                end

                //  ORA_WAIT: wait until ORA signals done.
                //  RESULT=1 → signatures match → no fault for this pattern → next
                //  RESULT=0 → mismatch → fault detected → stop BIST
                ORA_WAIT: begin
                    if (ORA_DONE) begin
                        if (!RESULT) begin
                            // Signature mismatch — fault detected
                            $display("[BIST] Fault DETECTED by fault_id=%0d at pattern %0d",
                                     fault_id, count);
                            DONE         <= 1'b1;
                            CUT_FAULT_EN <= 1'b0;
                            state        <= FAULT_DONE;
                        end
                        else begin
                            // Signature matched — continue to next pattern
                            state <= RUN;
                        end
                    end
                    // else: stay in ORA_WAIT
                end

                FAULT_DONE: begin
                    DONE <= 1'b1;   // keep DONE asserted
                end

                default: state <= RESET;
            endcase
        end
    end
endmodule