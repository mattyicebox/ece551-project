module cmd_proc(clk, rst_n, cmd, cmd_rdy, clr_cmd_rdy, send_resp,
                strt_cal, cal_done, in_cal,
                strt_hdng, strt_mv, stp_lft, stp_rght,
                dsrd_hdng, mv_cmplt, sol_cmplt, cmd_md);

  input         clk, rst_n;
  input  [15:0] cmd;
  input         cmd_rdy;
  output reg    clr_cmd_rdy;
  output reg    send_resp;

  // Inertial interface
  output reg    strt_cal;
  input         cal_done;
  output reg    in_cal;

  // Navigate interface
  output reg    strt_hdng;
  output reg    strt_mv;
  output reg    stp_lft;
  output reg    stp_rght;
  output reg [11:0] dsrd_hdng;
  input         mv_cmplt;

  // Solve interface
  input         sol_cmplt;
  output reg    cmd_md;

  // Opcode definitions
  localparam OP_CAL   = 3'b000;
  localparam OP_HDNG  = 3'b001;
  localparam OP_MOVE  = 3'b010;
  localparam OP_SOLVE = 3'b011;

  // State encoding
  typedef enum reg [2:0] {IDLE, CAL, HDNG, MOVE, SOLVE} state_t;
  state_t state, nxt_state;

  // Decode opcode from command
  wire [2:0] opcode = cmd[15:13];

  // State register
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      state <= IDLE;
    else
      state <= nxt_state;

  // dsrd_hdng register
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      dsrd_hdng <= 12'h000;
    else if (cmd_rdy && opcode == OP_HDNG)
      dsrd_hdng <= cmd[11:0];

  // stp_lft / stp_rght registers
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      stp_lft  <= 1'b0;
      stp_rght <= 1'b0;
    end else if (cmd_rdy && opcode == OP_MOVE) begin
      stp_lft  <= cmd[1];
      stp_rght <= cmd[0];
    end

  // cmd_md register (high by default, deasserted for solve)
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      cmd_md <= 1'b1;
    else if (cmd_rdy && opcode == OP_SOLVE)
      cmd_md <= 1'b0;

  // FSM combinational logic
  always_comb begin
    // Defaults
    nxt_state    = state;
    clr_cmd_rdy  = 1'b0;
    send_resp    = 1'b0;
    strt_cal     = 1'b0;
    in_cal       = 1'b0;
    strt_hdng    = 1'b0;
    strt_mv      = 1'b0;

    case (state)
      IDLE:
        if (cmd_rdy) begin
          clr_cmd_rdy = 1'b1;
          case (opcode)
            OP_CAL: begin
              strt_cal  = 1'b1;
              nxt_state = CAL;
            end
            OP_HDNG: begin
              strt_hdng = 1'b1;
              nxt_state = HDNG;
            end
            OP_MOVE: begin
              strt_mv   = 1'b1;
              nxt_state = MOVE;
            end
            OP_SOLVE: begin
              nxt_state = SOLVE;
            end
            default: ; // reserved opcodes - do nothing
          endcase
        end

      CAL: begin
        in_cal = 1'b1;
        if (cal_done) begin
          send_resp = 1'b1;
          nxt_state = IDLE;
        end
      end

      HDNG:
        if (mv_cmplt) begin
          send_resp = 1'b1;
          nxt_state = IDLE;
        end

      MOVE:
        if (mv_cmplt) begin
          send_resp = 1'b1;
          nxt_state = IDLE;
        end

      SOLVE:
        if (sol_cmplt) begin
          send_resp = 1'b1;
          nxt_state = IDLE;
        end

      default: nxt_state = IDLE;
    endcase
  end

endmodule
