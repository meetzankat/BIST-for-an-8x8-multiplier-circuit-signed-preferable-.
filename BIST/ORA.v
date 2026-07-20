/*
The ORA functions as follows:
 SISR polynomial: x^8 + x^6 + x^5 + x + 1
 (taps at stages 0, 1, 5, 6 with feedback from stage 7)

 Operation sequence (controlled by BIST controller via `enable`):
   1. enable=0 → IDLE: captures M into CUT_OUTPUT on every clock edge.
   2. enable=1 → SHIFT: serially feeds CUT_OUTPUT[15..0] into SISR over
                 16 clock cycles.
   3. After 16th shift → COMPARE: SIGNATURE compared to mem[mem_address];
                 result latched; module returns to IDLE.

 Ports:
   CLK         — system clock
   RST         — active-high synchronous reset
   M [15:0]    — 16-bit CUT output
   mem_address — 16-bit ROM address of the expected (good) signature
   enable      — rising edge triggers capture + SISR computation
   result      — 1 = PASS, 0 = FAIL (valid one cycle after COMPARE)
   done        — pulses high for one cycle when comparison is complete
*/

module ORA (
    input        CLK,
    input        RST,
    input [15:0] M,
    input [15:0]  mem_address,
    input        enable,
    output reg   result,
    output reg   done          // one-cycle pulse: comparison is valid
);
 
    localparam IDLE    = 2'd0;
    localparam SHIFT   = 2'd1;
    localparam COMPARE = 2'd2;
 
    reg [1:0]  state;
    reg [15:0] CUT_OUTPUT;             // Latched copy of M
    reg [7:0]  SIGNATURE;              // 8-bit SISR output
    reg [3:0]  count;                  // Shift counter, 15 down to 0
 
    wire [7:0] good_sig;
    ORA_ROM rom (.address(mem_address), .data(good_sig));
 
    always @(posedge CLK) begin
        if (RST) begin
            state      <= IDLE;
            CUT_OUTPUT <= 16'b0;
            SIGNATURE  <= 8'b0;
            count      <= 4'd15;
            result     <= 1'b1;
            done       <= 1'b0;
        end
        else begin
            done <= 1'b0;   // default: done is low; set explicitly in COMPARE
 
            case (state)
                // IDLE: continuously latch M so it is stable when enable arrives. Transition to SHIFT on enable.
                IDLE: begin
                    CUT_OUTPUT <= M;
                    SIGNATURE  <= 8'b0;
                    count      <= 4'd15;
                    if (enable)
                        state <= SHIFT;
                end
 
                SHIFT: begin
                    // Serial input is CUT_OUTPUT[count] (MSB first)
                    SIGNATURE[0] <= CUT_OUTPUT[count] ^ SIGNATURE[7];
                    SIGNATURE[1] <= SIGNATURE[0]      ^ SIGNATURE[7];
                    SIGNATURE[2] <= SIGNATURE[1];
                    SIGNATURE[3] <= SIGNATURE[2];
                    SIGNATURE[4] <= SIGNATURE[3];
                    SIGNATURE[5] <= SIGNATURE[4]      ^ SIGNATURE[7];
                    SIGNATURE[6] <= SIGNATURE[5]      ^ SIGNATURE[7];
                    SIGNATURE[7] <= SIGNATURE[6];
 
                    if (count == 4'd0) begin
                        state <= COMPARE; // All 16 bits fed in; move to compare next cycle
                        count <= 4'd15;         // pre-load for next run
                    end
                    else begin
                        count <= count - 1'b1;
                    end
                end

                COMPARE: begin
                    result <= (SIGNATURE == good_sig) ? 1'b1 : 1'b0;
                    done   <= 1'b1;
                    state  <= IDLE;
                end
 
                default: state <= IDLE;
            endcase
        end
    end
endmodule

module ORA_ROM (
    input      [15:0] address,
    output reg [7:0] data
);
    reg [7:0] mem [0:65534];
    initial $readmemb("ROM.txt", mem);
    always @(*) data = mem[address];
endmodule