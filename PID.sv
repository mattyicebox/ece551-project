module PID(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        moving,
    input  logic [11:0] dsrd_hdng,
    input  logic [11:0] actl_hdng,
    input  logic        hdng_vld,
    input  logic [10:0] frwrd_spd,
    output logic        at_hdng,
    output logic [11:0] lft_spd,
    output logic [11:0] rght_spd
);

    // Signals
    logic signed [11:0] hdng_diff;
    logic signed [12:0] hdng_diff_ext;
    logic signed [9:0]  err_sat;
    localparam signed [3:0] P_COEFF = 4'h3;
    localparam signed [4:0] D_COEFF = 5'h0E;

    logic signed [13:0] P_term;
    logic signed [11:0] I_term;
    logic signed [12:0] D_term;

    logic signed [14:0] P_term_ext, I_term_ext, D_term_ext;
    logic signed [15:0] err_sat_ext;
    logic signed [15:0] integrator;
    logic signed [15:0] integrator_bf_mux1;
    logic signed [15:0] integrator_bf_mux2;
    logic signed [15:0] nxt_integrator;
    logic               ov;
    logic               ov_after_and;

    logic signed [9:0]  ff1, prev_err;
    logic signed [9:0]  ff1_din, prev_err_din;
    logic signed [10:0] D_diff;
    logic signed [7:0]  D_diff_sat;
    logic signed [15:0] PID_sum;
    logic signed [12:0] PID_sum_div8;
    logic signed [12:0] frwrd_spd_ext;
    logic signed [12:0] lft_spd_pre_mux, rght_spd_pre_mux;

    // Calculate heading difference with wrap-around
    always_comb begin
        hdng_diff_ext = $signed({1'b0, actl_hdng}) - $signed({1'b0, dsrd_hdng});
        if (hdng_diff_ext > 13'sd2047)
            hdng_diff_ext = hdng_diff_ext - 13'sd4096;
        else if (hdng_diff_ext < -13'sd2048)
            hdng_diff_ext = hdng_diff_ext + 13'sd4096;
        hdng_diff = hdng_diff_ext[11:0];  // Now this is signed
    end

    // Saturate hdng_diff to signed 10-bit value
    assign err_sat = (hdng_diff[11] & ~(&hdng_diff[10:9])) ? 10'sb1000000000 :
                     (~hdng_diff[11] & (|hdng_diff[10:9])) ? 10'sb0111111111 :
                     hdng_diff[9:0];

    assign P_term = err_sat * P_COEFF;

    // I-term integration
    always_comb begin
        err_sat_ext = { {6{err_sat[9]}}, err_sat };
    end
    
    assign integrator_bf_mux1 = err_sat_ext + integrator;
    assign ov = (err_sat_ext[15] == integrator[15]) && (integrator_bf_mux1[15] != integrator[15]);
    assign ov_after_and = ~ov & hdng_vld;
    assign integrator_bf_mux2 = ov_after_and ? integrator_bf_mux1 : integrator;
    assign nxt_integrator = moving ? integrator_bf_mux2 : 16'sh0000;

    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            integrator <= 16'sh0000;
        else
            integrator <= nxt_integrator;
    end

    assign I_term = integrator[15:4];

    // D-term with proper saturation
    always_comb begin
        ff1_din      = hdng_vld ? err_sat : ff1;
        prev_err_din = hdng_vld ? ff1 : prev_err;
    end

    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) ff1 <= '0;
        else        ff1 <= ff1_din;
    end

    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) prev_err <= '0;
        else        prev_err <= prev_err_din;
    end

    always_comb begin
        D_diff = err_sat - prev_err;
        // Saturate to signed 8-bit range (-128 to +127)
        if (D_diff > 11'sd127)
            D_diff_sat = 8'sd127;
        else if (D_diff < -11'sd128)
            D_diff_sat = -8'sd128;
        else
            D_diff_sat = D_diff[7:0];
        D_term = D_diff_sat * D_COEFF;
    end

    // Sum PID terms
    assign P_term_ext = { {1{P_term[13]}}, P_term };
    assign I_term_ext = { {3{I_term[11]}}, I_term };
    assign D_term_ext = { {2{D_term[12]}}, D_term };
    assign PID_sum = P_term_ext + I_term_ext + D_term_ext;
    assign PID_sum_div8 = PID_sum >>> 3;

    // Motor speed calculation
    assign frwrd_spd_ext = $signed({1'b0, frwrd_spd});
    assign rght_spd_pre_mux = frwrd_spd_ext - PID_sum_div8;
    assign lft_spd_pre_mux  = frwrd_spd_ext + PID_sum_div8;

    assign rght_spd = moving ? rght_spd_pre_mux[11:0] : 12'h000;
    assign lft_spd  = moving ? lft_spd_pre_mux[11:0]  : 12'h000;

    // At heading detection
    assign at_hdng = (err_sat < 10'sd30) && (err_sat > -10'sd30);

endmodule