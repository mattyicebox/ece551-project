module PWM12 (clk, rst_n, duty, PWM1, PWM2);
    input clk, rst_n;
    input [11:0] duty;
    output logic PWM1, PWM2;

    localparam NONOVERLAP = 12'h02C;

    logic [11:0] counter;
    logic pwm1_set, pwm1_reset;
    logic pwm2_set, pwm2_reset;

    assign pwm1_set = (counter >= NONOVERLAP);
    assign pwm1_reset = (counter >= duty);

    assign pwm2_set = (counter >= (duty + NONOVERLAP));
    assign pwm2_reset = &counter;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PWM2 <= 1'b0;
        end else if (pwm2_reset) begin
            PWM2 <= 1'b0;
        end else if (pwm2_set) begin
            PWM2 <= 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PWM1 <= 1'b0;
        end else if (pwm1_reset) begin
            PWM1 <= 1'b0;
        end else if (pwm1_set) begin
            PWM1 <= 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 12'd0;
        end else begin
            counter <= counter + 12'd1;
        end
    end

endmodule