// UART_wrapper: receives two UART bytes and assembles a 16-bit command.
// High byte arrives first, then low byte.
// Transmit side passes trmt/resp directly to UART_tx.

module UART_wrapper(clk, rst_n, RX, TX, cmd_rdy, clr_cmd_rdy, cmd,
                    trmt, resp, tx_done);

  input         clk, rst_n;
  input         RX;
  output        TX;
  output reg    cmd_rdy;
  input         clr_cmd_rdy;
  output [15:0] cmd;
  input         trmt;
  input  [7:0]  resp;
  output        tx_done;

  wire       rx_rdy;
  wire [7:0] rx_data;

  // Self-clearing: clr_rx_rdy = rx_rdy ensures rx_rdy pulses for exactly 1 cycle.
  assign clr_rx_rdy = rx_rdy;

  UART iUART (
    .clk(clk), .rst_n(rst_n),
    .RX(RX),          .TX(TX),
    .rx_rdy(rx_rdy),  .clr_rx_rdy(clr_rx_rdy),
    .rx_data(rx_data),
    .trmt(trmt),      .tx_data(resp),
    .tx_done(tx_done)
  );

  // 2-state FSM: waiting for high byte, then low byte
  typedef enum logic {WAIT_HI, WAIT_LO} state_t;
  state_t state, nxt_state;

  // Datapath registers
  reg [7:0]  hi_byte;
  reg [15:0] cmd_reg;

  // Control signals produced by the SM
  logic capture_hi;   // latch rx_data into hi_byte
  logic capture_lo;   // latch {hi_byte, rx_data} into cmd_reg
  logic set_cmd_rdy;  // assert cmd_rdy

  assign cmd = cmd_reg;

  // -----------------------------------------------------------------
  // Block 1: state register
  // -----------------------------------------------------------------
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) state <= WAIT_HI;
    else        state <= nxt_state;

  // -----------------------------------------------------------------
  // Block 2: next-state and SM output (control signal) logic
  // -----------------------------------------------------------------
  always_comb begin
    nxt_state   = state;
    capture_hi  = 1'b0;
    capture_lo  = 1'b0;
    set_cmd_rdy = 1'b0;
    case (state)
      WAIT_HI:
        if (rx_rdy) begin
          capture_hi = 1'b1;
          nxt_state  = WAIT_LO;
        end
      WAIT_LO:
        if (rx_rdy) begin
          capture_lo  = 1'b1;
          set_cmd_rdy = 1'b1;
          nxt_state   = WAIT_HI;
        end
    endcase
  end

  // -----------------------------------------------------------------
  // Block 3: datapath flops driven by SM control signals
  // -----------------------------------------------------------------
  // high-byte holding register
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)           hi_byte <= '0;
    else if (capture_hi)  hi_byte <= rx_data;

  // 16-bit command register
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)           cmd_reg <= '0;
    else if (capture_lo)  cmd_reg <= {hi_byte, rx_data};

  // cmd_rdy flag: set by SM when low byte arrives, cleared by clr_cmd_rdy
  // (set has priority so a newly-assembled command is not lost).
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)              cmd_rdy <= 1'b0;
    else if (set_cmd_rdy)    cmd_rdy <= 1'b1;
    else if (clr_cmd_rdy)    cmd_rdy <= 1'b0;

endmodule
