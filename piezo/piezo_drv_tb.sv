`timescale 1ns/1ps

module piezo_drv_tb;

logic clk;
logic rst_n;
logic fanfare;
logic piezo;
logic piezo_n;
logic A2D_MISO;

logic IR_lft_en;
logic IR_cntr_en;
logic IR_rght_en;
logic [11:0] lft_IR;
logic [11:0] rght_IR;
logic [8:0] IR_Dtrm;
logic [11:0] vbatt;
logic lft_opn;
logic frwrd_opn;
logic rght_opn;
logic batt_low;
logic A2D_SS_n;
logic A2D_SCLK;
logic A2D_MOSI;
logic [6:0] LED;

logic [11:0] adc_batt;
logic [11:0] adc_ir_lft;
logic [11:0] adc_ir_cntr;
logic [11:0] adc_ir_rght;

int unsigned error_count;

localparam logic [11:0] BATT_GOOD = 12'hE20;
localparam logic [11:0] BATT_LOW_VAL = 12'hC80;

sensor_intf #(.FAST_SIM(1'b1)) iSENS (
    .clk(clk),
    .rst_n(rst_n),
    .IR_lft_en(IR_lft_en),
    .IR_cntr_en(IR_cntr_en),
    .IR_rght_en(IR_rght_en),
    .lft_IR(lft_IR),
    .rght_IR(rght_IR),
    .IR_Dtrm(IR_Dtrm),
    .vbatt(vbatt),
    .lft_opn(lft_opn),
    .rght_opn(rght_opn),
    .strt_cal(1'b0),
    .frwrd_opn(frwrd_opn),
    .batt_low(batt_low),
    .A2D_SS_n(A2D_SS_n),
    .A2D_SCLK(A2D_SCLK),
    .A2D_MOSI(A2D_MOSI),
    .A2D_MISO(A2D_MISO),
    .LED(LED)
);

ADC128S_FC iADC (
    .clk(clk),
    .rst_n(rst_n),
    .SS_n(A2D_SS_n),
    .SCLK(A2D_SCLK),
    .MISO(A2D_MISO),
    .MOSI(A2D_MOSI),
    .IR_lft(adc_ir_lft),
    .IR_cntr(adc_ir_cntr),
    .IR_rght(adc_ir_rght),
    .batt(adc_batt)
);

piezo_drv #(.FAST_SIM(1'b1)) dut (
    .clk(clk),
    .rst_n(rst_n),
    .batt_low(batt_low),
    .fanfare(fanfare),
    .piezo(piezo),
    .piezo_n(piezo_n)
);

always #10 clk = ~clk; // 50 MHz

// Differential outputs should always be complements.
always @(posedge clk) begin
    if (rst_n && (piezo_n !== ~piezo)) begin
        $error("piezo_n is not complement of piezo at t=%0t", $time);
        error_count = error_count + 1;
    end
end

task automatic check_stable(input int unsigned cycles, input string tag);
    logic p0;
    logic pn0;
    int unsigned i;
    begin
        p0 = piezo;
        pn0 = piezo_n;
        for (i = 0; i < cycles; i = i + 1) begin
            @(posedge clk);
            if (piezo !== p0 || piezo_n !== pn0) begin
                $error("%s: outputs toggled unexpectedly at t=%0t", tag, $time);
                error_count = error_count + 1;
                disable check_stable;
            end
        end
    end
endtask

task automatic wait_for_first_toggle(input int unsigned max_cycles, input string tag);
    logic prev;
    int unsigned i;
    begin
        prev = piezo;
        for (i = 0; i < max_cycles; i = i + 1) begin
            @(posedge clk);
            if (piezo !== prev) begin
                return;
            end
        end
        $error("%s: no toggle seen within %0d cycles", tag, max_cycles);
        error_count = error_count + 1;
    end
endtask

task automatic check_min_toggles(
    input int unsigned window_cycles,
    input int unsigned min_toggles,
    input string tag
);
    logic prev;
    int unsigned i;
    int unsigned toggles;
    begin
        prev = piezo;
        toggles = 0;
        for (i = 0; i < window_cycles; i = i + 1) begin
            @(posedge clk);
            if (piezo !== prev) begin
                toggles = toggles + 1;
                prev = piezo;
            end
        end

        if (toggles < min_toggles) begin
            $error("%s: expected at least %0d toggles, saw %0d", tag, min_toggles, toggles);
            error_count = error_count + 1;
        end
    end
endtask

task automatic wait_until_quiet(
    input int unsigned max_wait_cycles,
    input int unsigned quiet_cycles,
    input string tag
);
    logic prev;
    int unsigned wait_i;
    int unsigned quiet_i;
    bit became_quiet;
    begin
        became_quiet = 0;
        for (wait_i = 0; wait_i < max_wait_cycles; wait_i = wait_i + 1) begin
            prev = piezo;
            @(posedge clk);
            if (piezo === prev) begin
                quiet_i = 1;
                while (quiet_i < quiet_cycles) begin
                    prev = piezo;
                    @(posedge clk);
                    if (piezo !== prev)
                        break;
                    quiet_i = quiet_i + 1;
                end
                if (quiet_i == quiet_cycles) begin
                    became_quiet = 1;
                    break;
                end
            end
        end

        if (!became_quiet) begin
            $error("%s: never became quiet", tag);
            error_count = error_count + 1;
        end
    end
endtask

task automatic wait_for_batt_low(
    input bit expected,
    input int unsigned max_cycles,
    input string tag
);
    int unsigned i;
    begin
        for (i = 0; i < max_cycles; i = i + 1) begin
            @(posedge clk);
            if (batt_low === expected)
                return;
        end

        $error("%s: batt_low did not become %0d within %0d cycles", tag, expected, max_cycles);
        error_count = error_count + 1;
    end
endtask

initial begin
    clk = 1'b0;
    rst_n = 1'b0;
    fanfare = 1'b0;
    adc_batt = BATT_GOOD;
    adc_ir_lft = 12'h980;
    adc_ir_cntr = 12'h980;
    adc_ir_rght = 12'h980;
    error_count = 0;

    repeat (6) @(posedge clk);
    rst_n = 1'b1;

    // Let sensor_intf settle and report healthy battery first.
    wait_for_batt_low(1'b0, 800000, "BATT_GOOD_DETECT");
    wait_until_quiet(1200000, 40000, "QUIET_AFTER_BATT_GOOD");

    // 1) In integrated idle mode (good battery), outputs should stay quiet.
    check_stable(5000, "IDLE_QUIET_PRE");

    // 2) One-cycle fanfare request should start playback.
    fanfare = 1'b1;
    @(posedge clk);
    fanfare = 1'b0;

    wait_for_first_toggle(25000, "FANFARE_STARTS");
    check_min_toggles(200000, 8, "FANFARE_ACTIVE");

    // 3) Drive battery below threshold through sensor_intf path.
    // batt_low going active must not preempt an already-active fanfare.
    adc_batt = BATT_LOW_VAL;
    wait_for_batt_low(1'b1, 1200000, "BATT_LOW_ASSERTS");
    check_min_toggles(120000, 5, "FANFARE_NOT_PREEMPTED");

    // 4) Fanfare eventually ends; with batt_low still asserted, low-battery tone should continue.
    wait_for_first_toggle(50000, "LOW_BATT_FOLLOWS_FANFARE");
    check_min_toggles(250000, 10, "LOW_BATT_CONTINUOUS");

    // 5) Recover battery and verify sound stops.
    adc_batt = BATT_GOOD;
    wait_for_batt_low(1'b0, 1200000, "BATT_RECOVERY");
    wait_until_quiet(400000, 40000, "LOW_BATT_STOPS");
    check_stable(40000, "IDLE_QUIET_POST");

    // 6) With both requests active in IDLE, batt_low path has priority over fanfare.
    adc_batt = BATT_LOW_VAL;
    wait_for_batt_low(1'b1, 1200000, "BATT_LOW_REASSERT");
    fanfare = 1'b1;
    @(posedge clk);
    fanfare = 1'b0;

    wait_for_first_toggle(25000, "LOW_BATT_PRIORITY_STARTS");
    check_min_toggles(250000, 10, "LOW_BATT_PRIORITY_CONTINUOUS");

    // Return to nominal battery and verify final quiet behavior.
    adc_batt = BATT_GOOD;
    wait_for_batt_low(1'b0, 1200000, "FINAL_BATT_RECOVERY");
    wait_until_quiet(1200000, 40000, "FINAL_QUIET");
    check_stable(40000, "FINAL_IDLE_QUIET");

    if (error_count == 0) begin
        $display("PASS: piezo_drv_tb completed with no errors.");
    end else begin
        $fatal(1, "FAIL: piezo_drv_tb found %0d errors.", error_count);
    end

    $finish;
end

endmodule
