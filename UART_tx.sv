module UART_tx(clk, rst_n, TX, trmt, tx_data, tx_done);

  input        clk, rst_n;
  input        trmt;
  input  [7:0] tx_data;
  output       TX;
  output       tx_done;

  localparam BAUD_DIV = 2604; // 50MHz / 19200 baud

  reg        transmitting;
  reg [11:0] baud_cnt;
  reg  [3:0] bit_cnt;
  reg  [9:0] tx_shift; // {stop(1), data[7:0], start(0)}

  wire baud_pulse = (baud_cnt == BAUD_DIV - 1);

  assign TX      = transmitting ? tx_shift[0] : 1'b1;
  assign tx_done = (bit_cnt == 4'd9) && baud_pulse;

  // transmitting flag
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)                      transmitting <= 1'b0;
    else if (trmt && !transmitting)  transmitting <= 1'b1;
    else if (tx_done)                transmitting <= 1'b0;

  // baud counter
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)          baud_cnt <= '0;
    else if (!transmitting) baud_cnt <= '0;
    else if (baud_pulse) baud_cnt <= '0;
    else                 baud_cnt <= baud_cnt + 1;

  // bit counter (0=start, 1-8=data, 9=stop)
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)          bit_cnt <= '0;
    else if (!transmitting) bit_cnt <= '0;
    else if (baud_pulse) bit_cnt <= bit_cnt + 1;

  // shift register
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)                          tx_shift <= 10'h3FF;
    else if (trmt && !transmitting)      tx_shift <= {1'b1, tx_data, 1'b0};
    else if (baud_pulse && transmitting) tx_shift <= {1'b1, tx_shift[9:1]};

endmodule
