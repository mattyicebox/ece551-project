module dur_cntr #(parameter FAST_SIM = 1'b0) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        clr,
    input  logic [24:0] dur,
    output logic        dur_done
);

logic [24:0] count;
logic [24:0] dur_step;

generate
    if (FAST_SIM) begin
        assign dur_step = 25'd16;
    end else begin
        assign dur_step = 25'd1;
    end
endgenerate

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        count <= 25'd0;
    end else if (clr || dur_done) begin
        count <= 25'd0;
    end else begin
        count <= count + dur_step;
    end
end

assign dur_done = (count >= dur);

endmodule