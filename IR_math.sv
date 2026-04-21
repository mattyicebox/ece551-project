//Dataflow verilog that handles gyro drift correction

module IR_math #(
    parameter NOM_IR = 12'h900
)(clk, lft_opn, rght_opn, lft_IR, rght_IR, IR_Dtrm, en_fusion, dsrd_hdng, dsrd_hdng_adj);

input clk;
input lft_opn, rght_opn;
input [11:0] lft_IR, rght_IR;
input signed [8:0] IR_Dtrm;
input en_fusion;
input signed [11:0] dsrd_hdng;
  
output signed [11:0] dsrd_hdng_adj;

logic signed [12:0] IR_diff;
logic signed [12:0] IRL_min_NOM;
logic signed [12:0] NOM_min_IRR;

logic signed [12:0] mux1_LorNom, mux2_LorR;
logic signed [12:0] mux3_const_orLorR;
logic signed [11:0] IR_diff_adj;
logic signed [12:0] IR_Dtrm_x4;
logic signed [12:0] IR_correction_prelim;
logic signed [11:0] IR_Dtrm_adj;
logic signed [11:0] IR_Dtrm_final;

// calculate the difference between the left and right readings
// Use bits [12:1] (divide by 2) per diagram before further scaling
assign IR_diff = ($signed({1'b0, lft_IR}) - $signed({1'b0, rght_IR})) >>> 1;

// calculate the difference between left reading and nominal
assign IRL_min_NOM = $signed({1'b0, lft_IR}) - $signed({1'b0, NOM_IR});

// calculate the difference between nominal and right reading
assign NOM_min_IRR = $signed({1'b0, NOM_IR}) - $signed({1'b0, rght_IR});

// make a multiplexer, and feed in IR_diff and IRL_min_NOM, with rght_opn as select sig
assign mux1_LorNom = rght_opn ? IRL_min_NOM : IR_diff;

//feed into a mux with lft_opn as select sig
assign mux2_LorR = lft_opn ? NOM_min_IRR : mux1_LorNom;

// mux to choose between constant 0 and previous mux output, with selector being lft_opn AND rght_opn
assign mux3_const_orLorR = (lft_opn && rght_opn) ? 13'sh000 : mux2_LorR;

//divide mux result by 32 (take bits [11:5] with sign extension)
assign IR_diff_adj = mux3_const_orLorR >>> 5;

assign IR_Dtrm_x4 = $signed({{4{IR_Dtrm[8]}}, IR_Dtrm}) <<< 2;

//add IR_Dtrm_x4 
assign IR_correction_prelim = $signed(IR_diff_adj) + IR_Dtrm_x4;

//divide the result by 2
assign IR_Dtrm_adj = IR_correction_prelim >>> 1;

assign IR_Dtrm_final = IR_Dtrm_adj + dsrd_hdng;

assign dsrd_hdng_adj = en_fusion ? IR_Dtrm_final : dsrd_hdng;

endmodule