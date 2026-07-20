/*
The LFSR is controlled by the test controller as following:
-> Whenever BIST starts it resets the LFSR.
-> The test controller gives the signal to change the value of LFSR, and then it wait for another
    increment signal (we need to stop the LFSR until the CUT and ORA completes their tasks).
-> The test controller will have a counter to count the number of patterns generated and stop the BIST
    when all patterns are generated.
*/
module LFSR_16 (
    input  wire        CLK,
    input  wire        RST,           // Active high reset
    input  wire        INCREMENT,     // Signal to change the value of LFSR
    output reg  [15:0] Q              // Q[1] = stage 1, Q[16] = stage 16
);
    parameter SEED = 16'b1000_0000_0000_0000;

    wire feedback = Q[15] ^ Q[4] ^ Q[2] ^ Q[1]; // Taps for x^16 + x^5 + x^3 + x^2 + 1

    always @(posedge CLK) begin
        if (RST) begin
            Q <= SEED;                        // Load seed on reset
        end else if (INCREMENT) begin
            Q <= {Q[14:0], feedback};         // Shift left, insert feedback at LSB
        end
    end
endmodule