// =============================================================================
//  MUL8 — Efficient Single Stuck-At Fault Injection System
//
//  KEY OPTIMIZATIONS vs original:
//  1. Registered fault decode: fi_en bus is latched into a register once
//     when fault_id changes — eliminates the 385-bit barrel shifter from
//     the combinational simulation path entirely.
//  2. FA internal wires handled inline (no round-trip): the FI_LUT2s for
//     w1/w2/w3 live inside FA_FI itself, removing 294 extra top-level nets.
//  3. fi_en is now a plain reg — simulator only wakes up FI_LUT2 cells
//     whose enable bit actually transitions (event-driven efficiency).
//  4. fault_id decode is a single always block that runs ONCE per test,
//     not on every gate evaluation.
//
//  Signal index map (385 signals → 770 faults):  [unchanged from original]
//  Signal index  0..7   : A[0..7]
//  Signal index  8..15  : B[0..7]
//  Signal index 16..31  : M[0..15]
//  Signal index 32..95  : pp[row][col]  (64 signals)
//  Signal index 96..159 : s[0..63]
//  Signal index 160..223: c[0..63]
//  Signal index 224..237: inv[0..13]
//  Signal index 238..384: FA internal wires (49 FAs × 3 = 147)
//
//  fault_id = 2*signal_index     → SA0
//  fault_id = 2*signal_index + 1 → SA1
// =============================================================================

// ---------------------------------------------------------------------------
//  FI_LUT2 — unchanged interface, same MUX semantics
// ---------------------------------------------------------------------------
module FI_LUT2 (
    input  wire in,
    input  wire fi_en,
    input  wire stuck,
    output wire out
);
    assign out = fi_en ? stuck : in;
endmodule

// ---------------------------------------------------------------------------
//  Half Adder
// ---------------------------------------------------------------------------
module HA (S, C, A, B);
    input  A, B;
    output S, C;
    and x1(C, A, B);
    xor x2(S, A, B);
endmodule

// ---------------------------------------------------------------------------
//  FA_FI_v2 — Full Adder with INTERNAL fault injection on w1/w2/w3
//  
//  FI_LUT2s for internal wires now live HERE, not at the top level.
//  This eliminates 6 ports per FA (w1_raw/w2_raw/w3_raw/w1_f/w2_f/w3_f)
//  and removes 294 top-level wires from the simulator's event queue.
//
//  The three enable bits are passed in directly (already decoded above).
// ---------------------------------------------------------------------------
module FA_FI_v2 (
    output wire S, Cout,
    input  wire A, B, Cin,
    // Per-internal-wire fault enables (one-hot decoded, registered)
    input  wire fi_en_w1, fi_en_w2, fi_en_w3,
    input  wire stuck           // shared stuck value (SA0 or SA1)
);
    wire w1_raw, w2_raw, w3_raw;
    wire w1_f,   w2_f,   w3_f;

    xor  g1 (w1_raw, A,    B);
    FI_LUT2 fi_w1 (.in(w1_raw), .fi_en(fi_en_w1), .stuck(stuck), .out(w1_f));

    xor  g2 (S,      w1_f, Cin);

    nand g3 (w2_raw, A,    B);
    FI_LUT2 fi_w2 (.in(w2_raw), .fi_en(fi_en_w2), .stuck(stuck), .out(w2_f));

    nand g4 (w3_raw, w1_f, Cin);    // depends on faulty w1
    FI_LUT2 fi_w3 (.in(w3_raw), .fi_en(fi_en_w3), .stuck(stuck), .out(w3_f));

    nand g5 (Cout,   w2_f, w3_f);
endmodule

// =============================================================================
//  MUL8_FAULT_INJECT_v2 — Top wrapper (optimized)
// =============================================================================
module CUT (
    output signed [15:0] M,
    input  signed [7:0]  A,
    input  signed [7:0]  B,
    input                fault_en,
    input         [9:0]  fault_id     // 0..769
);

    // ------------------------------------------------------------------
    //  OPTIMIZATION 1: Registered one-hot decode
    //  fi_en is a REG, not a combinational function of fault_id.
    //  It is updated ONCE when fault_id/fault_en changes, and is then
    //  stable throughout the entire simulation of that fault.
    //  The simulator's event-driven engine will only wake up FI_LUT2
    //  cells whose enable bit actually transitions — typically only 2
    //  cells fire (the old active site deactivates, the new one activates).
    // ------------------------------------------------------------------
    reg  [384:0] fi_en;
    wire [8:0]   sig_idx   = fault_id[9:1];
    wire         stuck_val = fault_id[0];

    always @(fault_en or fault_id) begin
        if (!fault_en)
            fi_en = 385'b0;
        else begin
            fi_en        = 385'b0;      // clear all
            fi_en[sig_idx] = 1'b1;      // set exactly one bit
        end
    end
    // NOTE: In a real BIST chip, fi_en would be a shift register or
    // ROM-decoded address — never a barrel shifter in silicon.

    // ------------------------------------------------------------------
    //  Primary input fault injection (signal index 0..15)
    // ------------------------------------------------------------------
    wire [7:0] A_f, B_f;
    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : GEN_AB
            FI_LUT2 fi_A (.in(A[gi]),   .fi_en(fi_en[gi]),    .stuck(stuck_val), .out(A_f[gi]));
            FI_LUT2 fi_B (.in(B[gi]),   .fi_en(fi_en[gi+8]),  .stuck(stuck_val), .out(B_f[gi]));
        end
    endgenerate

    // ------------------------------------------------------------------
    //  Partial products (signal index 32..95)
    // ------------------------------------------------------------------
    wire [7:0] pp_f [7:0];
    genvar pi, pj;
    generate
        for (pi = 0; pi < 8; pi = pi + 1) begin : PP_ROW
            for (pj = 0; pj < 8; pj = pj + 1) begin : PP_COL
                wire pp_raw_w;
                and (pp_raw_w, A_f[pj], B_f[pi]);
                FI_LUT2 fi_pp (
                    .in    (pp_raw_w),
                    .fi_en (fi_en[32 + pi*8 + pj]),
                    .stuck (stuck_val),
                    .out   (pp_f[pi][pj])
                );
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    //  Sum/carry/inv wires with FI (signal index 96..237)
    // ------------------------------------------------------------------
    wire [63:0] s_raw, c_raw;
    wire [13:0] inv_raw;
    wire [63:0] s_f, c_f;
    wire [13:0] inv_f;

    genvar si, ci, ii;
    generate
        for (si = 0; si < 64; si = si + 1) begin : GEN_S_FI
            FI_LUT2 fi_s (.in(s_raw[si]), .fi_en(fi_en[96+si]),  .stuck(stuck_val), .out(s_f[si]));
        end
        for (ci = 0; ci < 64; ci = ci + 1) begin : GEN_C_FI
            FI_LUT2 fi_c (.in(c_raw[ci]), .fi_en(fi_en[160+ci]), .stuck(stuck_val), .out(c_f[ci]));
        end
        for (ii = 0; ii < 14; ii = ii + 1) begin : GEN_INV_FI
            FI_LUT2 fi_inv (.in(inv_raw[ii]), .fi_en(fi_en[224+ii]), .stuck(stuck_val), .out(inv_f[ii]));
        end
    endgenerate

    // ------------------------------------------------------------------
    //  HA instances
    // ------------------------------------------------------------------
    HA h1  (s_raw[0],  c_raw[0],  pp_f[1][0], pp_f[0][1]);
    HA h2  (s_raw[1],  c_raw[1],  pp_f[0][2], pp_f[1][1]);
    HA h3  (s_raw[2],  c_raw[2],  pp_f[1][2], pp_f[0][3]);
    HA h4  (s_raw[3],  c_raw[3],  pp_f[1][3], pp_f[0][4]);
    HA h5  (s_raw[4],  c_raw[4],  pp_f[1][4], pp_f[0][5]);
    HA h6  (s_raw[5],  c_raw[5],  pp_f[0][6], pp_f[1][5]);

    not n1 (inv_raw[0], pp_f[0][7]);
    HA h7  (s_raw[6],  c_raw[6],  inv_f[0],  pp_f[1][6]);

    not n2 (inv_raw[1], pp_f[1][7]);
    HA h8  (s_raw[7],  c_raw[7],  inv_f[1],  1'b1);

    // ------------------------------------------------------------------
    //  FA instances using FA_FI_v2 (internal FI_LUT2s inside)
    //  FA k uses fi_en bits: 238+k*3, 239+k*3, 240+k*3
    // ------------------------------------------------------------------

    // 2nd Row (FA index 0..6)
    FA_FI_v2 f1  (s_raw[8],  c_raw[8],  pp_f[2][0], s_f[1],  c_f[0],  fi_en[238], fi_en[239], fi_en[240], stuck_val);
    FA_FI_v2 f2  (s_raw[9],  c_raw[9],  pp_f[2][1], s_f[2],  c_f[1],  fi_en[241], fi_en[242], fi_en[243], stuck_val);
    FA_FI_v2 f3  (s_raw[10], c_raw[10], pp_f[2][2], s_f[3],  c_f[2],  fi_en[244], fi_en[245], fi_en[246], stuck_val);
    FA_FI_v2 f4  (s_raw[11], c_raw[11], pp_f[2][3], s_f[4],  c_f[3],  fi_en[247], fi_en[248], fi_en[249], stuck_val);
    FA_FI_v2 f5  (s_raw[12], c_raw[12], pp_f[2][4], s_f[5],  c_f[4],  fi_en[250], fi_en[251], fi_en[252], stuck_val);
    FA_FI_v2 f6  (s_raw[13], c_raw[13], pp_f[2][5], s_f[6],  c_f[5],  fi_en[253], fi_en[254], fi_en[255], stuck_val);
    FA_FI_v2 f7  (s_raw[14], c_raw[14], pp_f[2][6], s_f[7],  c_f[6],  fi_en[256], fi_en[257], fi_en[258], stuck_val);

    not n3 (inv_raw[2], pp_f[2][7]);
    HA h9  (s_raw[15], c_raw[15], inv_f[2], c_f[7]);

    // 3rd Row (FA index 7..13)
    FA_FI_v2 f8  (s_raw[16], c_raw[16], pp_f[3][0], s_f[9],  c_f[8],  fi_en[259], fi_en[260], fi_en[261], stuck_val);
    FA_FI_v2 f9  (s_raw[17], c_raw[17], pp_f[3][1], s_f[10], c_f[9],  fi_en[262], fi_en[263], fi_en[264], stuck_val);
    FA_FI_v2 f10 (s_raw[18], c_raw[18], pp_f[3][2], s_f[11], c_f[10], fi_en[265], fi_en[266], fi_en[267], stuck_val);
    FA_FI_v2 f11 (s_raw[19], c_raw[19], pp_f[3][3], s_f[12], c_f[11], fi_en[268], fi_en[269], fi_en[270], stuck_val);
    FA_FI_v2 f12 (s_raw[20], c_raw[20], pp_f[3][4], s_f[13], c_f[12], fi_en[271], fi_en[272], fi_en[273], stuck_val);
    FA_FI_v2 f13 (s_raw[21], c_raw[21], pp_f[3][5], s_f[14], c_f[13], fi_en[274], fi_en[275], fi_en[276], stuck_val);
    FA_FI_v2 f14 (s_raw[22], c_raw[22], pp_f[3][6], s_f[15], c_f[14], fi_en[277], fi_en[278], fi_en[279], stuck_val);

    not n4 (inv_raw[3], pp_f[3][7]);
    HA h10 (s_raw[23], c_raw[23], inv_f[3], c_f[15]);

    // 4th Row (FA index 14..20)
    FA_FI_v2 f15 (s_raw[24], c_raw[24], pp_f[4][0], s_f[17], c_f[16], fi_en[280], fi_en[281], fi_en[282], stuck_val);
    FA_FI_v2 f16 (s_raw[25], c_raw[25], pp_f[4][1], s_f[18], c_f[17], fi_en[283], fi_en[284], fi_en[285], stuck_val);
    FA_FI_v2 f17 (s_raw[26], c_raw[26], pp_f[4][2], s_f[19], c_f[18], fi_en[286], fi_en[287], fi_en[288], stuck_val);
    FA_FI_v2 f18 (s_raw[27], c_raw[27], pp_f[4][3], s_f[20], c_f[19], fi_en[289], fi_en[290], fi_en[291], stuck_val);
    FA_FI_v2 f19 (s_raw[28], c_raw[28], pp_f[4][4], s_f[21], c_f[20], fi_en[292], fi_en[293], fi_en[294], stuck_val);
    FA_FI_v2 f20 (s_raw[29], c_raw[29], pp_f[4][5], s_f[22], c_f[21], fi_en[295], fi_en[296], fi_en[297], stuck_val);
    FA_FI_v2 f21 (s_raw[30], c_raw[30], pp_f[4][6], s_f[23], c_f[22], fi_en[298], fi_en[299], fi_en[300], stuck_val);

    not n5 (inv_raw[4], pp_f[4][7]);
    HA h11 (s_raw[31], c_raw[31], inv_f[4], c_f[23]);

    // 5th Row (FA index 21..27)
    FA_FI_v2 f22 (s_raw[32], c_raw[32], pp_f[5][0], s_f[25], c_f[24], fi_en[301], fi_en[302], fi_en[303], stuck_val);
    FA_FI_v2 f23 (s_raw[33], c_raw[33], pp_f[5][1], s_f[26], c_f[25], fi_en[304], fi_en[305], fi_en[306], stuck_val);
    FA_FI_v2 f24 (s_raw[34], c_raw[34], pp_f[5][2], s_f[27], c_f[26], fi_en[307], fi_en[308], fi_en[309], stuck_val);
    FA_FI_v2 f25 (s_raw[35], c_raw[35], pp_f[5][3], s_f[28], c_f[27], fi_en[310], fi_en[311], fi_en[312], stuck_val);
    FA_FI_v2 f26 (s_raw[36], c_raw[36], pp_f[5][4], s_f[29], c_f[28], fi_en[313], fi_en[314], fi_en[315], stuck_val);
    FA_FI_v2 f27 (s_raw[37], c_raw[37], pp_f[5][5], s_f[30], c_f[29], fi_en[316], fi_en[317], fi_en[318], stuck_val);
    FA_FI_v2 f28 (s_raw[38], c_raw[38], pp_f[5][6], s_f[31], c_f[30], fi_en[319], fi_en[320], fi_en[321], stuck_val);

    not n6 (inv_raw[5], pp_f[5][7]);
    HA h12 (s_raw[39], c_raw[39], inv_f[5], c_f[31]);

    // 6th Row (FA index 28..34)
    FA_FI_v2 f29 (s_raw[40], c_raw[40], pp_f[6][0], s_f[33], c_f[32], fi_en[322], fi_en[323], fi_en[324], stuck_val);
    FA_FI_v2 f30 (s_raw[41], c_raw[41], pp_f[6][1], s_f[34], c_f[33], fi_en[325], fi_en[326], fi_en[327], stuck_val);
    FA_FI_v2 f31 (s_raw[42], c_raw[42], pp_f[6][2], s_f[35], c_f[34], fi_en[328], fi_en[329], fi_en[330], stuck_val);
    FA_FI_v2 f32 (s_raw[43], c_raw[43], pp_f[6][3], s_f[36], c_f[35], fi_en[331], fi_en[332], fi_en[333], stuck_val);
    FA_FI_v2 f33 (s_raw[44], c_raw[44], pp_f[6][4], s_f[37], c_f[36], fi_en[334], fi_en[335], fi_en[336], stuck_val);
    FA_FI_v2 f34 (s_raw[45], c_raw[45], pp_f[6][5], s_f[38], c_f[37], fi_en[337], fi_en[338], fi_en[339], stuck_val);
    FA_FI_v2 f35 (s_raw[46], c_raw[46], pp_f[6][6], s_f[39], c_f[38], fi_en[340], fi_en[341], fi_en[342], stuck_val);

    not n7 (inv_raw[6], pp_f[6][7]);
    HA h13 (s_raw[47], c_raw[47], inv_f[6], c_f[39]);

    // 7th Row (FA index 35..41)
    not n8  (inv_raw[7],  pp_f[7][0]);
    FA_FI_v2 f36 (s_raw[48], c_raw[48], inv_f[7],  s_f[41], c_f[40], fi_en[343], fi_en[344], fi_en[345], stuck_val);

    not n9  (inv_raw[8],  pp_f[7][1]);
    FA_FI_v2 f37 (s_raw[49], c_raw[49], inv_f[8],  s_f[42], c_f[41], fi_en[346], fi_en[347], fi_en[348], stuck_val);

    not n10 (inv_raw[9],  pp_f[7][2]);
    FA_FI_v2 f38 (s_raw[50], c_raw[50], inv_f[9],  s_f[43], c_f[42], fi_en[349], fi_en[350], fi_en[351], stuck_val);

    not n11 (inv_raw[10], pp_f[7][3]);
    FA_FI_v2 f39 (s_raw[51], c_raw[51], inv_f[10], s_f[44], c_f[43], fi_en[352], fi_en[353], fi_en[354], stuck_val);

    not n12 (inv_raw[11], pp_f[7][4]);
    FA_FI_v2 f40 (s_raw[52], c_raw[52], inv_f[11], s_f[45], c_f[44], fi_en[355], fi_en[356], fi_en[357], stuck_val);

    not n13 (inv_raw[12], pp_f[7][5]);
    FA_FI_v2 f41 (s_raw[53], c_raw[53], inv_f[12], s_f[46], c_f[45], fi_en[358], fi_en[359], fi_en[360], stuck_val);

    not n14 (inv_raw[13], pp_f[7][6]);
    FA_FI_v2 f42 (s_raw[54], c_raw[54], inv_f[13], s_f[47], c_f[46], fi_en[361], fi_en[362], fi_en[363], stuck_val);

    HA h14 (s_raw[55], c_raw[55], pp_f[7][7], c_f[47]);

    // 8th Row (FA index 42..48)
    HA h15 (s_raw[56], c_raw[56], s_f[49], c_f[48]);

    FA_FI_v2 f43 (s_raw[57], c_raw[57], s_f[50], c_f[49], c_f[56], fi_en[364], fi_en[365], fi_en[366], stuck_val);
    FA_FI_v2 f44 (s_raw[58], c_raw[58], s_f[51], c_f[50], c_f[57], fi_en[367], fi_en[368], fi_en[369], stuck_val);
    FA_FI_v2 f45 (s_raw[59], c_raw[59], s_f[52], c_f[51], c_f[58], fi_en[370], fi_en[371], fi_en[372], stuck_val);
    FA_FI_v2 f46 (s_raw[60], c_raw[60], s_f[53], c_f[52], c_f[59], fi_en[373], fi_en[374], fi_en[375], stuck_val);
    FA_FI_v2 f47 (s_raw[61], c_raw[61], s_f[54], c_f[53], c_f[60], fi_en[376], fi_en[377], fi_en[378], stuck_val);
    FA_FI_v2 f48 (s_raw[62], c_raw[62], s_f[55], c_f[54], c_f[61], fi_en[379], fi_en[380], fi_en[381], stuck_val);
    FA_FI_v2 f49 (s_raw[63], c_raw[63], 1'b1,    c_f[55], c_f[62], fi_en[382], fi_en[383], fi_en[384], stuck_val);

    // ------------------------------------------------------------------
    //  Primary output fault injection (signal index 16..31)
    // ------------------------------------------------------------------
    wire [15:0] M_raw;
    assign M_raw[0]  = pp_f[0][0];
    assign M_raw[1]  = s_f[0];
    assign M_raw[2]  = s_f[8];
    assign M_raw[3]  = s_f[16];
    assign M_raw[4]  = s_f[24];
    assign M_raw[5]  = s_f[32];
    assign M_raw[6]  = s_f[40];
    assign M_raw[7]  = s_f[48];
    assign M_raw[8]  = s_f[56];
    assign M_raw[9]  = s_f[57];
    assign M_raw[10] = s_f[58];
    assign M_raw[11] = s_f[59];
    assign M_raw[12] = s_f[60];
    assign M_raw[13] = s_f[61];
    assign M_raw[14] = s_f[62];
    assign M_raw[15] = s_f[63];

    genvar oi;
    generate
        for (oi = 0; oi < 16; oi = oi + 1) begin : GEN_M_FI
            FI_LUT2 fi_m (
                .in    (M_raw[oi]),
                .fi_en (fi_en[16+oi]),
                .stuck (stuck_val),
                .out   (M[oi])
            );
        end
    endgenerate

endmodule