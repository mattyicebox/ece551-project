module piezo_drv(clk, rst_n, batt_low, fanfare, piezo, piezo_n);

input clk, rst_n, batt_low, fanfare;
output logic piezo, piezo_n;

logic [24:0] note_duration_counter;
logic [15:0] note_frequency_counter;
logic note_on;

// State encoding
typedef enum logic [1:0] {
    IDLE,
    LOW_BATTERY,
    FANFARE
} state_t;

state_t state, next_state;

// State transition logic
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
    end else begin
        state <= next_state;
    end
end

// State Combinational Logic
always_comb begin
    next_state = state;
    case (state)
        IDLE: begin
            if (batt_low) begin
                next_state = LOW_BATTERY;
            end else if (fanfare) begin
                next_state = FANFARE;
            end
        end
        LOW_BATTERY: begin
            if (!batt_low)
                next_state = IDLE;
            else
                // Start playing low battery tone G6, C7, E7
                note_on = 1;
                
        end
        FANFARE: begin
            if (!fanfare) begin
                next_state = IDLE;
            end
        end
    endcase
end

// Note duration counter
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        note_duration_counter <= 0;
    else if (note_on)
        note_duration_counter <= note_duration_counter + 1;
    else
        note_duration_counter <= 0;
end

// Note frequency counter
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        note_frequency_counter <= 0;
    else
        note_frequency_counter <= note_frequency_counter + 1;
end

endmodule