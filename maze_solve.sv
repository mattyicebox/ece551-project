module maze_solve(cmd_md, cmd0, lft_opn, rght_opn, mv_cmplt, sol_cmplt, clk, rst_n, strt_hdng, dsrd_hdng, strt_mv, stp_lft, stp_rght);

//////////////////////
// Inputs / Outputs //
//////////////////////

input logic cmd_md, cmd0;
input logic lft_opn, rght_opn;
input logic mv_cmplt, sol_cmplt;
input logic clk;
input logic rst_n;

output logic        strt_hdng;
output logic [11:0] dsrd_hdng;
output logic strt_mv, stp_lft, stp_rght;


////////////////
// Parameters //
////////////////
localparam NORTH = 12'h000;
localparam WEST = 12'h3FF;
localparam SOUTH = 12'h7FF;
localparam EAST = 12'hC00;

////////////////////
// Intermediates  //
////////////////////
logic [11:0] next_hdng;
assign stp_lft = cmd0;
assign stp_rght = ~cmd0;

////////////////////////////////////////////////////////////////////////////////////////
// This algorithm will solve a maze by using left affinity or right affinity movement //
////////////////////////////////////////////////////////////////////////////////////////
typedef enum logic [3:0] {IDLE, HOME, MOVING, TURNING, DONE} state_t;
state_t state, next_state;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        state <= IDLE;
     else 
        state <= next_state;
    
end

// Start with north but can be any direction
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        dsrd_hdng <= 12'h000; 
    else if (state == TURNING)
        dsrd_hdng <= next_hdng;
end


 always_comb begin
        // defaults
        next_state = state;
        strt_mv    = 0;
        strt_hdng  = 0;
        next_hdng  = dsrd_hdng;

        case (state)


            IDLE: begin
                if (!cmd_md)
                    next_state = HOME;
            end


            HOME: begin
                // start first move
                strt_mv = 1;
                next_state = MOVING;
            end

            MOVING: begin
                if (sol_cmplt)
                    next_state = DONE;
                else if (mv_cmplt)
                    next_state = TURNING;
            end


            TURNING: begin
                strt_hdng = 1;

                // LEFT affinity
                if (cmd0) begin
                    if (lft_opn) begin
                        case (dsrd_hdng)
                            12'h000: next_hdng = 12'h3FF;
                            12'h3FF: next_hdng = 12'h7FF;
                            12'h7FF: next_hdng = 12'hC00;
                            12'hC00: next_hdng = 12'h000;
                            default: next_hdng = 12'h000;
                        endcase
                    end
                    else if (rght_opn) begin
                        case (dsrd_hdng)
                            12'h000: next_hdng = 12'hC00;
                            12'hC00: next_hdng = 12'h7FF;
                            12'h7FF: next_hdng = 12'h3FF;
                            12'h3FF: next_hdng = 12'h000;
                            default: next_hdng = 12'h000;
                        endcase
                    end
                    else begin
                        case (dsrd_hdng)
                            12'h000: next_hdng = 12'h7FF;
                            12'h3FF: next_hdng = 12'hC00;
                            12'h7FF: next_hdng = 12'h000;
                            12'hC00: next_hdng = 12'h3FF;
                            default: next_hdng = 12'h000;
                        endcase
                    end
                end

                // RIGHT affinity
                else begin
                    if (rght_opn) begin
                        case (dsrd_hdng)
                            12'h000: next_hdng = 12'hC00;
                            12'hC00: next_hdng = 12'h7FF;
                            12'h7FF: next_hdng = 12'h3FF;
                            12'h3FF: next_hdng = 12'h000;
                            default: next_hdng = 12'h000;
                        endcase
                    end
                    else if (lft_opn) begin
                        case (dsrd_hdng)
                            12'h000: next_hdng = 12'h3FF;
                            12'h3FF: next_hdng = 12'h7FF;
                            12'h7FF: next_hdng = 12'hC00;
                            12'hC00: next_hdng = 12'h000;
                            default: next_hdng = 12'h000;
                        endcase
                    end
                    else begin
                        case (dsrd_hdng)
                            12'h000: next_hdng = 12'h7FF;
                            12'h3FF: next_hdng = 12'hC00;
                            12'h7FF: next_hdng = 12'h000;
                            12'hC00: next_hdng = 12'h3FF;
                            default: next_hdng = 12'h000;
                        endcase
                    end
                end
                if (mv_cmplt)
                    next_state = HOME; 
            end

            
            DONE: begin
              next_state = IDLE;  
            end

            default:
                next_state = IDLE;
        endcase
    end

endmodule