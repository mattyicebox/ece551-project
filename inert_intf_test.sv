module inert_intf_test (
    input logic clk,
    input logic RST_n,
    output logic [7:0] LED,
    input logic MISO,
    output logic MOSI,
    output logic SCLK,
    output logic SS_n,
    input logic INT
);



typedef enum logic [1:0] {
    IDLE, CAL, DISP
} state_t;

state_t state, next_state;

// 17-bit free-running timer
logic [16:0] timer;
logic timer_full;

// Calibration control
logic strt_cal;
logic cal_done;
logic [15:0] heading;

// LED output register
logic [7:0] led_reg;
assign LED = led_reg;

// Instantiate reset synchronizer
logic rst_n;
reset_synch rst_sync (
    .RST_n(RST_n),
    .clk(clk),
    .rst_n(rst_n)
);

// Instantiate inertial interface
inert_intf intf (
    .clk(clk),
    .rst_n(rst_n),
    .MISO(MISO),
    .MOSI(MOSI),
    .SCLK(SCLK),
    .SS_n(SS_n),
    .INT(INT),
    .strt_cal(strt_cal),
    .cal_done(cal_done),
    .heading(heading)
);

// 17-bit free-running timer
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        timer <= 17'd0;
    else if (state == IDLE)
        timer <= timer + 1'b1;
    else
        timer <= 17'd0;
end

assign timer_full = &timer; // All bits are 1

// State machine sequential logic
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= IDLE;
    else
        state <= next_state;
end

// State machine combinational logic
always_comb begin
    next_state = state;
    strt_cal = 1'b0;
    case (state)
        IDLE: begin
            if (timer_full) begin
                next_state = CAL;
            end
        end
        CAL: begin
            strt_cal = 1'b1;
            if (cal_done) begin
                next_state = DISP;
            end
        end
        DISP: begin
            // Remain in DISP until reset
            next_state = DISP;
        end
        default: next_state = IDLE;
    endcase
end

// LED output logic
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        led_reg <= 8'h00;
    end else begin
        case (state)
            IDLE: led_reg <= 8'h00;
            CAL:  led_reg <= 8'hA5;
            DISP: led_reg <= heading[11:4];
            default: led_reg <= 8'h00;
        endcase
    end
end

endmodule