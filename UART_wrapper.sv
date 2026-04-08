// UART_wrapper: receives two UART bytes and assembles a 16-bit command.
// High byte arrives first, then low byte.
// Transmit side passes trmt/resp directly to UART_tx.

module UART_wrapper(clk, rst_n, RX, TX, cmd_rdy, clr_cmd_rdy, cmd,
                    trmt, resp, tx_done);

  input        clk, rst_n;
  input        RX;
  output       TX;
  output reg   cmd_rdy;
  input        clr_cmd_rdy;
  output reg [15:0] cmd;
  input        trmt;
  input  [7:0] resp;
  output       tx_done;

  wire       rx_rdy;
  wire [7:0] rx_data;

  // Self-clearing: clr_rx_rdy = rx_rdy ensures rx_rdy pulses for exactly 1 cycle.
  // At the posedge rx_rdy goes high, UART_rx samples clr_rdy=1 and clears rdy
  // on the following posedge, giving the FSM one cycle to capture data.
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
  typedef enum reg {WAIT_HI, WAIT_LO} state_t;
  state_t state;

  reg [7:0] hi_byte; // holding register for high byte

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      state   <= WAIT_HI;
      hi_byte <= '0;
      cmd     <= '0;
      cmd_rdy <= 1'b0;
    end else begin
      if (clr_cmd_rdy)
        cmd_rdy <= 1'b0;

      case (state)
        WAIT_HI:
          if (rx_rdy) begin
            hi_byte <= rx_data;
            state   <= WAIT_LO;
          end
        WAIT_LO:
          if (rx_rdy) begin
            cmd     <= {hi_byte, rx_data};
            cmd_rdy <= 1'b1;
            state   <= WAIT_HI;
          end
      endcase
    end

endmodule
