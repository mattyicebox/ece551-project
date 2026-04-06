`timescale 1ns/1ps

module piezo_drv_tb;

logic clk;
logic rst_n;
logic batt_low;
logic fanfare;
logic piezo;
logic piezo_n;

int unsigned error_count;

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

initial begin
    clk = 1'b0;
    rst_n = 1'b0;
    batt_low = 1'b0;
    fanfare = 1'b0;
    error_count = 0;

    repeat (6) @(posedge clk);
    rst_n = 1'b1;

    // 1) Idle should be quiet.
    check_stable(5000, "IDLE_QUIET_PRE");

    // 2) One-cycle fanfare request should start playback.
    fanfare = 1'b1;
    @(posedge clk);
    fanfare = 1'b0;

    wait_for_first_toggle(25000, "FANFARE_STARTS");
    check_min_toggles(200000, 8, "FANFARE_ACTIVE");

    // Battery-low assertion during fanfare should not stop fanfare immediately.
    batt_low = 1'b1;
    @(posedge clk);
    batt_low = 1'b0;
    check_min_toggles(120000, 5, "FANFARE_NOT_PREEMPTED");

    // 3) Fanfare eventually ends and outputs become quiet.
    wait_until_quiet(6000000, 40000, "FANFARE_ENDS");
    check_stable(40000, "IDLE_QUIET_POST");

    // 4) batt_low has priority over fanfare when both asserted in IDLE.
    batt_low = 1'b1;
    fanfare = 1'b1;
    @(posedge clk);
    fanfare = 1'b0;

    wait_for_first_toggle(25000, "LOW_BATT_STARTS");
    check_min_toggles(250000, 10, "LOW_BATT_CONTINUOUS");

    // Deassert batt_low and verify sound stops.
    batt_low = 1'b0;
    wait_until_quiet(200000, 40000, "LOW_BATT_STOPS");
    check_stable(40000, "FINAL_IDLE_QUIET");

    if (error_count == 0) begin
        $display("PASS: piezo_drv_tb completed with no errors.");
    end else begin
        $fatal(1, "FAIL: piezo_drv_tb found %0d errors.", error_count);
    end

    $finish;
end

endmodule
