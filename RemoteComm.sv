// RemoteComm: BLE module emulator.
// Splits a 16-bit cmd into two 8-bit UART transmissions (high byte first).
// Receives 1-byte ack response from UART_wrapper.

module RemoteComm(clk, rst_n, TX, RX, cmd, send_cmd, cmd_sent, resp_rdy, resp);

  input        clk, rst_n;
  output       TX;
  input        RX;
  input  [15:0] cmd;
  input         send_cmd;
  output        cmd_sent;
  output        resp_rdy;
  output [7:0]  resp;

  // Internal UART signals
  reg        trmt;
  reg  [7:0] tx_data;
  wire       tx_done;
  wire       rx_rdy;
  wire [7:0] rx_data;

  // Auto-clear rx_rdy: resp_rdy pulses for exactly 1 clock cycle
  assign resp_rdy = rx_rdy;

  UART iUART (
    .clk(clk), .rst_n(rst_n),
    .RX(RX),          .TX(TX),
    .rx_rdy(rx_rdy),  .clr_rx_rdy(rx_rdy),
    .rx_data(rx_data),
    .trmt(trmt),      .tx_data(tx_data),
    .tx_done(tx_done)
  );

  assign resp = rx_data;

  // Control FSM
  typedef enum reg [1:0] {IDLE, SEND_HI, SEND_LO} state_t;
  state_t state;

  reg [15:0] cmd_reg;     // holds cmd while transmitting
  reg        cmd_sent_reg; // SR flip-flop output

  assign cmd_sent = cmd_sent_reg;

  // Registered FSM + datapath
  // trmt and tx_data are registered outputs so tx_data is stable when UART_tx latches trmt.
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      state       <= IDLE;
      cmd_reg     <= '0;
      trmt        <= 1'b0;
      tx_data     <= '0;
      cmd_sent_reg <= 1'b0;
    end else begin
      trmt <= 1'b0; // default deassert

      case (state)
        IDLE:
          if (send_cmd) begin
            cmd_reg <= cmd;
            tx_data <= cmd[15:8];
            trmt    <= 1'b1;
            state   <= SEND_HI;
          end

        SEND_HI:
          if (tx_done) begin
            tx_data <= cmd_reg[7:0];
            trmt    <= 1'b1;
            state   <= SEND_LO;
          end

        SEND_LO:
          if (tx_done) begin
            cmd_sent_reg <= 1'b1;
            state       <= IDLE;
          end
      endcase

      // Reset cmd_sent on new command (higher priority than set)
      if (send_cmd)
        cmd_sent_reg <= 1'b0;
    end

endmodule