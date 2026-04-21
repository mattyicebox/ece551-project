module piezo_drv #(parameter FAST_SIM = 1'b0) (
    input  logic clk,
    input  logic rst_n,
    input  logic batt_low,
    input  logic fanfare,
    output logic piezo,
    output logic piezo_n
);

localparam logic [24:0] DUR_2P22       = 25'd4194304;
localparam logic [24:0] DUR_2P23       = 25'd8388608;
localparam logic [24:0] DUR_2P24       = 25'd16777216;
localparam logic [24:0] DUR_2P23_P_2P22 = 25'd12582912;

localparam logic [15:0] G6_HALF_PERIOD = 16'd15944;
localparam logic [15:0] C7_HALF_PERIOD = 16'd11945;
localparam logic [15:0] E7_HALF_PERIOD = 16'd9480;
localparam logic [15:0] G7_HALF_PERIOD = 16'd7972;

// State encoding
typedef enum logic [1:0] {
    IDLE,
    LOW_BATTERY,
    FANFARE
} state_t;

state_t state, next_state;

typedef enum logic [1:0] {
    NOTE_G6,
    NOTE_C7,
    NOTE_E7,
    NOTE_G7
} note_t;

note_t note_sel;
logic [2:0] note_idx;
logic [24:0] note_dur;
logic [15:0] half_period;
logic [15:0] note_frequency_counter;
logic tone_en;
logic dur_done;
logic dur_clr;
logic fanfare_done;

dur_cntr #(.FAST_SIM(FAST_SIM)) i_dur_cntr (
    .clk(clk),
    .rst_n(rst_n),
    .clr(dur_clr),
    .dur(note_dur),
    .dur_done(dur_done)
);

// State transition logic
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
    end else begin
        state <= next_state;
    end
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        note_idx <= 3'd0;
    end else if (state != next_state) begin
        note_idx <= 3'd0;
    end else if ((state == LOW_BATTERY) && dur_done) begin
        if (note_idx == 3'd2)
            note_idx <= 3'd0;
        else
            note_idx <= note_idx + 3'd1;
    end else if ((state == FANFARE) && dur_done) begin
        if (note_idx != 3'd5)
            note_idx <= note_idx + 3'd1;
    end
end

// State Combinational Logic
always_comb begin
    next_state = state;
    fanfare_done = (state == FANFARE) && (note_idx == 3'd5) && dur_done;

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
        end
        FANFARE: begin
            if (fanfare_done) begin
                next_state = IDLE;
            end
        end
    endcase
end

always_comb begin
    tone_en = (state != IDLE);
    dur_clr = (state == IDLE);
    note_sel = NOTE_G6;
    note_dur = DUR_2P23;

    case (state)
        LOW_BATTERY: begin
            case (note_idx)
                3'd0: begin
                    note_sel = NOTE_G6;
                    note_dur = DUR_2P23;
                end
                3'd1: begin
                    note_sel = NOTE_C7;
                    note_dur = DUR_2P23;
                end
                default: begin
                    note_sel = NOTE_E7;
                    note_dur = DUR_2P23;
                end
            endcase
        end

        FANFARE: begin
            case (note_idx)
                3'd0: begin
                    note_sel = NOTE_G6;
                    note_dur = DUR_2P23;
                end
                3'd1: begin
                    note_sel = NOTE_C7;
                    note_dur = DUR_2P23;
                end
                3'd2: begin
                    note_sel = NOTE_E7;
                    note_dur = DUR_2P23;
                end
                3'd3: begin
                    note_sel = NOTE_G7;
                    note_dur = DUR_2P23_P_2P22;
                end
                3'd4: begin
                    note_sel = NOTE_E7;
                    note_dur = DUR_2P22;
                end
                default: begin
                    note_sel = NOTE_G7;
                    note_dur = DUR_2P24;
                end
            endcase
        end

        default: begin
            note_sel = NOTE_G6;
            note_dur = DUR_2P23;
        end
    endcase

    case (note_sel)
        NOTE_G6: half_period = G6_HALF_PERIOD;
        NOTE_C7: half_period = C7_HALF_PERIOD;
        NOTE_E7: half_period = E7_HALF_PERIOD;
        default: half_period = G7_HALF_PERIOD;
    endcase
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        note_frequency_counter <= 16'd0;
        piezo <= 1'b0;
        piezo_n <= 1'b1;
    end else if (!tone_en) begin
        note_frequency_counter <= 16'd0;
        piezo <= 1'b0;
        piezo_n <= 1'b1;
    end else if (note_frequency_counter >= (half_period - 16'd1)) begin
        note_frequency_counter <= 16'd0;
        piezo <= ~piezo;
        piezo_n <= ~piezo_n;
    end else begin
        note_frequency_counter <= note_frequency_counter + 16'd1;
    end
end

endmodule