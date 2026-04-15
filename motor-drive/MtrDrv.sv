
module MtrDrv (clk, rst_n, lft_spd, rght_spd, vbatt, lftPWM1, lftPWM2, rghtPWM1, rghtPWM2);

    input clk, rst_n;
    input signed [11:0] lft_spd, rght_spd;
    input [11:0] vbatt;		// battery voltage level
    output lftPWM1, lftPWM2, rghtPWM1, rghtPWM2;
    
    logic signed [12:0] lft_scaled, rght_scaled;
    logic [12:0] scale_factor;
    logic signed [24:0] lft_prod, rght_prod;
    logic [11:0] lft_duty, rght_duty;

    DutyScaleROM scale_rom(.clk(clk), .batt_level(vbatt[9:4]), .scale(scale_factor));

    // Generate PWM signals based on the scaled speeds
    PWM12 lft_pwm(.clk(clk), .rst_n(rst_n), .duty(lft_duty), .PWM1(lftPWM1), .PWM2(lftPWM2));
    PWM12 rght_pwm(.clk(clk), .rst_n(rst_n), .duty(rght_duty), .PWM1(rghtPWM1), .PWM2(rghtPWM2));

    always_comb begin
        // Multiply signed speed by the positive battery scale factor.
        lft_prod = lft_spd * $signed({1'b0, scale_factor});
        rght_prod = rght_spd * $signed({1'b0, scale_factor});
        
        // Divide by 2048, keeping the sign.
        lft_scaled = lft_prod >>> 11;
        rght_scaled = rght_prod >>> 11;

        // Clamp to the signed PWM input range before offsetting to unsigned duty.
        if (lft_scaled > 13'sd2047)
            lft_scaled = 13'sd2047;
        else if (lft_scaled < -13'sd2048)
            lft_scaled = -13'sd2048;

        if (rght_scaled > 13'sd2047)
            rght_scaled = 13'sd2047;
        else if (rght_scaled < -13'sd2048)
            rght_scaled = -13'sd2048;

        lft_duty = lft_scaled + 13'sd2048;
        rght_duty = 13'sd2048 - rght_scaled;
    end

endmodule
