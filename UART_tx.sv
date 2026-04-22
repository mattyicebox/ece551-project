module UART_tx(clk, rst_n, TX, trmt, tx_data, tx_done);

  input        clk, rst_n;
  input        trmt;
  input  [7:0] tx_data;
  output       TX;
  output       tx_done;

  localparam BAUD_DIV = 2604; // 50MHz / 19200 baud

  typedef enum logic {IDLE, TRANSMIT} state_t;
  state_t state, nxt_state;

  reg [11:0] baud_cnt;
  reg  [3:0] bit_cnt;
  reg  [8:0] tx_shift; // {stop(1), data[7:0]} on load; MSB shifts in 1's
  reg        tx_done_ff;

  // Control signals driven by the state machine
  logic load;          // load shift register, clear counters
  logic transmitting;  // enable baud counter (in TRANSMIT state)
  logic set_done;      // completion pulse

  wire baud_pulse = (baud_cnt == BAUD_DIV - 1);
  wire shift      = baud_pulse & transmitting;

  // TX is simply the LSB of the shift register (start bit is loaded at [0],
  // stop/idle '1's get shifted in from the MSB).
  assign TX      = tx_shift[0];
  assign tx_done = tx_done_ff;

  // State flop
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) state <= IDLE;
    else        state <= nxt_state;

  // Next-state + control signal decode
  always_comb begin
    load         = 1'b0;
    transmitting = 1'b0;
    set_done     = 1'b0;
    nxt_state    = state;
    case (state)
      IDLE: begin
        if (trmt) begin
          load      = 1'b1;
          nxt_state = TRANSMIT;
        end
      end
      TRANSMIT: begin
        transmitting = 1'b1;
        if ((bit_cnt == 4'd9) && baud_pulse) begin
          set_done  = 1'b1;
          nxt_state = IDLE;
        end
      end
    endcase
  end

  // 9-bit shift register: loaded with {tx_data, start-bit=0}; shifts in 1's.
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)      tx_shift <= 9'h1FF;
    else if (load)   tx_shift <= {tx_data, 1'b0};
    else if (shift)  tx_shift <= {1'b1, tx_shift[8:1]};

  // Baud counter
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)             baud_cnt <= '0;
    else if (load)          baud_cnt <= '0;
    else if (!transmitting) baud_cnt <= '0;
    else if (baud_pulse)    baud_cnt <= '0;
    else                    baud_cnt <= baud_cnt + 1;

  // Bit counter (0 = start, 1..8 = data, 9 = stop)
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)     bit_cnt <= '0;
    else if (load)  bit_cnt <= '0;
    else if (shift) bit_cnt <= bit_cnt + 1;

  // tx_done: set by SM on completion, cleared when a new transmission loads
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)         tx_done_ff <= 1'b0;
    else if (load)      tx_done_ff <= 1'b0;
    else if (set_done)  tx_done_ff <= 1'b1;

endmodule
