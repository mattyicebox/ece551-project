`timescale 1ns/1ps

module MtrDrv_tb;

    localparam integer CLK_PER = 10;
    localparam integer PWM_SAMPLES = 4096;
    localparam integer DUTY_TOL = 20;   // tenths of a percent

    logic clk;
    logic rst_n;
    logic [11:0] lft_spd;
    logic [11:0] rght_spd;
    logic [11:0] vbatt;
    logic lftPWM1, lftPWM2, rghtPWM1, rghtPWM2;

    integer errors;

    MtrDrv iDUT(
        .clk(clk),
        .rst_n(rst_n),
        .lft_spd(lft_spd),
        .rght_spd(rght_spd),
        .vbatt(vbatt),
        .lftPWM1(lftPWM1),
        .lftPWM2(lftPWM2),
        .rghtPWM1(rghtPWM1),
        .rghtPWM2(rghtPWM2)
    );

    initial clk = 1'b0;
    always #(CLK_PER/2) clk = ~clk;

    task measure_counts(
        output integer l1_count,
        output integer l2_count,
        output integer r1_count,
        output integer r2_count
    );
        integer i;
        begin
            l1_count = 0;
            l2_count = 0;
            r1_count = 0;
            r2_count = 0;

            for (i = 0; i < PWM_SAMPLES; i = i + 1) begin
                @(negedge clk);
                if (lftPWM1)
                    l1_count = l1_count + 1;
                if (lftPWM2)
                    l2_count = l2_count + 1;
                if (rghtPWM1)
                    r1_count = r1_count + 1;
                if (rghtPWM2)
                    r2_count = r2_count + 1;
            end
        end
    endtask

    task check_duty(
        input [8*20-1:0] name,
        input integer high_count,
        input integer exp_tenths
    );
        integer act_tenths;
        integer diff;
        begin
            act_tenths = (high_count * 1000 + (PWM_SAMPLES/2)) / PWM_SAMPLES;
            diff = act_tenths - exp_tenths;
            if (diff < 0)
                diff = -diff;

            if (diff > DUTY_TOL) begin
                errors = errors + 1;
                $display("ERROR: %0s expected about %0d.%0d%%, got %0d.%0d%%",
                         name,
                         exp_tenths / 10, exp_tenths % 10,
                         act_tenths / 10, act_tenths % 10);
            end
        end
    endtask

    task run_case(
        input [8*24-1:0] test_name,
        input [11:0] lft_cmd,
        input [11:0] rght_cmd,
        input [11:0] batt_code,
        input integer exp_l1,
        input integer exp_l2,
        input integer exp_r1,
        input integer exp_r2
    );
        integer l1_count, l2_count, r1_count, r2_count;
        begin
            lft_spd = lft_cmd;
            rght_spd = rght_cmd;
            vbatt = batt_code;

            repeat (PWM_SAMPLES + 16) @(posedge clk);
            measure_counts(l1_count, l2_count, r1_count, r2_count);

            $display("Test %0s complete", test_name);

            check_duty("lftPWM1",  l1_count, exp_l1);
            check_duty("lftPWM2",  l2_count, exp_l2);
            check_duty("rghtPWM1", r1_count, exp_r1);
            check_duty("rghtPWM2", r2_count, exp_r2);
        end
    endtask

    initial begin
        errors = 0;
        rst_n = 1'b0;
        lft_spd = 12'h000;
        rght_spd = 12'h000;
        vbatt = 12'hC00;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        // Zero command should be about 50% on both outputs regardless of battery.
        run_case("zero @ C0", 12'h000, 12'h000, 12'hC00, 500, 500, 500, 500);
        run_case("zero @ FF", 12'h000, 12'h000, 12'hFF0, 500, 500, 500, 500);

        // Check left and right motors independently against the slide values.
        run_case("left +3FF @ DB", 12'h3FF, 12'h000, 12'hDB0, 750, 250, 500, 500);
        run_case("right +3FF @ DB", 12'h000, 12'h3FF, 12'hDB0, 500, 500, 250, 750);

        run_case("left +3FF @ D0", 12'h3FF, 12'h000, 12'hD00, 762, 230, 500, 500);
        run_case("right +3FF @ D0", 12'h000, 12'h3FF, 12'hD00, 500, 500, 230, 762);

        run_case("left C00 @ FF", 12'hC00, 12'h000, 12'hFF0, 285, 714, 500, 500);
        run_case("right C00 @ FF", 12'h000, 12'hC00, 12'hFF0, 500, 500, 714, 285);

        if (errors == 0)
            $display("PASS: MtrDrv_tb completed with no errors.");
        else
            $fatal(1, "FAIL: MtrDrv_tb found %0d errors.", errors);

        $finish;
    end

endmodule
