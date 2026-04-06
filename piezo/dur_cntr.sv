module dur_cntr(clk, dur, dur_done);

input logic clk, [24:0] dur;
output logic dur_done;
logic [24:0] count;

always_ff @(posedge clk) begin
        count <= count + 1;
end

always_comb begin
    if (count == dur) begin
        dur_done = 1;
    end else begin
        dur_done = 0;
    end
end

endmodule