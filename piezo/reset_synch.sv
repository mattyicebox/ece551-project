module reset_synch (
    input logic RST_n,
    input logic clk,
    output logic rst_n
);

//double flop the async reset input
logic rst_n_ff1;
logic rst_n_ff2;


always @ (posedge clk or negedge RST_n) begin
    if (!RST_n) begin
        rst_n_ff1 <= 1'b0;
        rst_n_ff2 <= 1'b0;
        rst_n     <= 1'b0;
    end else begin
        rst_n_ff1 <= 1'b1;
        rst_n_ff2 <= rst_n_ff1;
        rst_n     <= rst_n_ff2;
    end
end

endmodule