module piezo_drv_test(clk, RST_n, Batt_low, fanfare, piezo, piezo_n);
    input logic clk;
    input logic RST_n;
    input logic Batt_low;
    input logic fanfare;
    output logic piezo;
    output logic piezo_n;

    logic rst_n;

    reset_synch iRSTn_sync (
        .clk(clk),
        .rst_n(RST_n)
    );

    piezo_drv #(.FAST_SIM(1'b0)) iDUT (
        .clk(clk),
        .rst_n(rst_n),
        .batt_low(Batt_low),
        .fanfare(fanfare),
        .piezo(piezo),
        .piezo_n(piezo_n)
    );



endmodule

