module UART_rx(clk, rst_n, RX, rdy, clr_rdy, rx_data);

  input        clk, rst_n;
  input        RX;
  input        clr_rdy;
  output reg   rdy;
  output [7:0] rx_data;

  localparam BAUD_DIV = 2604; // 50MHz / 19200 baud
  localparam HALF_DIV = 1302; // BAUD_DIV / 2

  // Double-flop synchronizer for RX input
  reg RX1, RXff;
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) {RXff, RX1} <= 2'b11;
    else        {RXff, RX1} <= {RX1, RX};

  typedef enum reg [1:0] {IDLE, START, DATA, STOP} state_t;
  state_t state;

  reg [11:0] baud_cnt;
  reg  [3:0] bit_cnt;
  reg  [7:0] rx_shift;

  wire baud_pulse = (baud_cnt == BAUD_DIV - 1);
  wire half_pulse = (baud_cnt == HALF_DIV - 1);

  assign rx_data = rx_shift;

  // State machine
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) state <= IDLE;
    else case (state)
      IDLE:  if (!RXff)                         state <= START;
      START: if (half_pulse)                    state <= DATA;
      DATA:  if (baud_pulse && bit_cnt == 4'd7) state <= STOP;
      STOP:  if (baud_pulse)                    state <= IDLE;
    endcase

  // Baud counter: resets in IDLE, resets at half_pulse in START, resets at baud_pulse otherwise
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)                           baud_cnt <= '0;
    else if (state == IDLE)               baud_cnt <= '0;
    else if (state == START && half_pulse) baud_cnt <= '0;
    else if (baud_pulse)                  baud_cnt <= '0;
    else                                  baud_cnt <= baud_cnt + 1;

  // Bit counter
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)                          bit_cnt <= '0;
    else if (state == IDLE || state == START) bit_cnt <= '0;
    else if (baud_pulse)                 bit_cnt <= bit_cnt + 1;

  // Shift register: sample at baud_pulse during DATA (LSB first)
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)                          rx_shift <= '0;
    else if (baud_pulse && state == DATA) rx_shift <= {RXff, rx_shift[7:1]};

  // Ready flag
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)                           rdy <= 1'b0;
    else if (clr_rdy || (!RXff && state == IDLE)) rdy <= 1'b0;
    else if (baud_pulse && state == STOP) rdy <= 1'b1;

endmodule
