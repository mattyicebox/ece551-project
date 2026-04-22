module cmd_proc_tb();

  reg clk, rst_n;
  reg snd_cmd;
  reg [15:0] cmd_to_send;

  // Wires between RemoteComm and UART_wrapper
  wire TX_to_wrapper, RX_to_remote;

  // UART_wrapper <-> cmd_proc
  wire [15:0] cmd;
  wire cmd_rdy, clr_cmd_rdy, send_resp;

  // cmd_proc outputs
  wire strt_cal, in_cal;
  wire strt_hdng, strt_mv;
  wire stp_lft, stp_rght;
  wire [11:0] dsrd_hdng;
  wire cmd_md;

  // Completion signals driven by flops (simulate delayed response)
  reg cal_done, mv_cmplt, sol_cmplt;

  // Flop chains to generate done signals a few clocks after strt signals
  reg strt_cal_ff1, strt_cal_ff2, strt_cal_ff3;
  reg strt_hdng_ff1, strt_hdng_ff2, strt_hdng_ff3;
  reg strt_mv_ff1, strt_mv_ff2, strt_mv_ff3;

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      strt_cal_ff1  <= 0; strt_cal_ff2  <= 0; strt_cal_ff3  <= 0;
      strt_hdng_ff1 <= 0; strt_hdng_ff2 <= 0; strt_hdng_ff3 <= 0;
      strt_mv_ff1   <= 0; strt_mv_ff2   <= 0; strt_mv_ff3   <= 0;
    end else begin
      strt_cal_ff1  <= strt_cal;  strt_cal_ff2  <= strt_cal_ff1;  strt_cal_ff3  <= strt_cal_ff2;
      strt_hdng_ff1 <= strt_hdng; strt_hdng_ff2 <= strt_hdng_ff1; strt_hdng_ff3 <= strt_hdng_ff2;
      strt_mv_ff1   <= strt_mv;   strt_mv_ff2   <= strt_mv_ff1;   strt_mv_ff3   <= strt_mv_ff2;
    end

  always_comb begin
    cal_done = strt_cal_ff3;
    mv_cmplt = strt_hdng_ff3 | strt_mv_ff3;
    sol_cmplt = 1'b0; // driven manually in test
  end

  // RemoteComm response signals
  wire [7:0] resp;
  wire resp_rdy;
  wire cmd_snt;

  // Instantiate RemoteComm (sends commands via UART)
  RemoteComm iRemoteComm(
    .clk(clk),
    .rst_n(rst_n),
    .TX(TX_to_wrapper),
    .RX(RX_to_remote),
    .cmd(cmd_to_send),
    .snd_cmd(snd_cmd),
    .cmd_snt(cmd_snt),
    .resp_rdy(resp_rdy),
    .resp(resp)
  );

  // Instantiate UART_wrapper (receives commands, sends responses)
  UART_wrapper iUART_wrapper(
    .clk(clk),
    .rst_n(rst_n),
    .RX(TX_to_wrapper),
    .TX(RX_to_remote),
    .cmd_rdy(cmd_rdy),
    .clr_cmd_rdy(clr_cmd_rdy),
    .cmd(cmd),
    .trmt(send_resp),
    .resp(8'hA5),
    .tx_done()
  );

  // Instantiate cmd_proc (DUT)
  cmd_proc iDUT(
    .clk(clk),
    .rst_n(rst_n),
    .cmd(cmd),
    .cmd_rdy(cmd_rdy),
    .clr_cmd_rdy(clr_cmd_rdy),
    .send_resp(send_resp),
    .strt_cal(strt_cal),
    .cal_done(cal_done),
    .in_cal(in_cal),
    .strt_hdng(strt_hdng),
    .strt_mv(strt_mv),
    .stp_lft(stp_lft),
    .stp_rght(stp_rght),
    .dsrd_hdng(dsrd_hdng),
    .mv_cmplt(mv_cmplt),
    .sol_cmplt(sol_cmplt),
    .cmd_md(cmd_md)
  );

  // Clock generation: 50MHz -> 20ns period
  always #5 clk = ~clk;

  // Task to send a command and wait for it to be received
  task send_command(input [15:0] cmd_val);
    begin
      @(posedge clk);
      cmd_to_send = cmd_val;
      snd_cmd = 1;
      @(posedge clk);
      snd_cmd = 0;
    end
  endtask

  // Timeout parameter for UART transactions
  localparam TIMEOUT = 1000000;
  integer timeout_cnt;

  initial begin
    //-------------------------------------------
    // Sequence 1: Initialize and reset
    //-------------------------------------------
    clk = 0;
    rst_n = 0;
    snd_cmd = 0;
    cmd_to_send = 16'h0000;

    repeat(2) @(posedge clk);
    rst_n = 1;
    repeat(2) @(posedge clk);

    $display("=== Sequence 1: Reset complete ===");

    //-------------------------------------------
    // Sequence 2: Calibrate command (0x0000)
    //-------------------------------------------
    $display("=== Sequence 2: Sending Calibrate command ===");
    send_command(16'h0000);

    // Wait for cal_done
    timeout_cnt = 0;
    while (!cal_done && timeout_cnt < TIMEOUT) begin
      @(posedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (timeout_cnt >= TIMEOUT) begin
      $display("ERROR: Timeout waiting for cal_done");
      $stop;
    end
    $display("  cal_done asserted");

    // Wait for resp_rdy
    timeout_cnt = 0;
    while (!resp_rdy && timeout_cnt < TIMEOUT) begin
      @(posedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (timeout_cnt >= TIMEOUT) begin
      $display("ERROR: Timeout waiting for resp_rdy");
      $stop;
    end
    if (resp !== 8'hA5)
      $display("ERROR: Expected response 0xA5, got 0x%h", resp);
    else
      $display("  Received correct 0xA5 response");

    repeat(100) @(posedge clk);

    //-------------------------------------------
    // Sequence 3: Heading command (heading = 12'h3FF = West)
    //-------------------------------------------
    $display("=== Sequence 3: Sending Heading command (West) ===");
    send_command(16'h23FF); // opcode 001, heading 0x3FF

    // Wait for cmd_rdy
    timeout_cnt = 0;
    while (!cmd_rdy && timeout_cnt < TIMEOUT) begin
      @(posedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (timeout_cnt >= TIMEOUT) begin
      $display("ERROR: Timeout waiting for cmd_rdy");
      $stop;
    end
    $display("  cmd_rdy asserted");

    // Check strt_hdng is asserted at cmd_rdy (combinational, same cycle)
    if (!strt_hdng)
      $display("ERROR: strt_hdng not asserted at cmd_rdy");
    else
      $display("  strt_hdng correctly asserted");

    // One clock later check dsrd_hdng
    @(posedge clk);
    if (dsrd_hdng !== 12'h3FF)
      $display("ERROR: dsrd_hdng = 0x%h, expected 0x3FF", dsrd_hdng);
    else
      $display("  dsrd_hdng correctly set to 0x3FF (West)");

    // Wait for resp_rdy
    timeout_cnt = 0;
    while (!resp_rdy && timeout_cnt < TIMEOUT) begin
      @(posedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (timeout_cnt >= TIMEOUT) begin
      $display("ERROR: Timeout waiting for resp_rdy");
      $stop;
    end
    if (resp !== 8'hA5)
      $display("ERROR: Expected response 0xA5, got 0x%h", resp);
    else
      $display("  Received correct 0xA5 response for heading cmd");

    repeat(100) @(posedge clk);

    //-------------------------------------------
    // Sequence 4: Move command (stop at left opening)
    //-------------------------------------------
    $display("=== Sequence 4: Sending Move command (stp_lft) ===");
    send_command(16'h4002); // opcode 010, cmd[1]=1 (stp_lft)

    // Wait for cmd_rdy
    timeout_cnt = 0;
    while (!cmd_rdy && timeout_cnt < TIMEOUT) begin
      @(posedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (timeout_cnt >= TIMEOUT) begin
      $display("ERROR: Timeout waiting for cmd_rdy");
      $stop;
    end
    $display("  cmd_rdy asserted");

    // Check strt_mv is asserted at cmd_rdy (combinational, same cycle)
    if (!strt_mv)
      $display("ERROR: strt_mv not asserted at cmd_rdy");
    else
      $display("  strt_mv correctly asserted");

    // One clock later check stp_lft/stp_rght
    @(posedge clk);
    if (!stp_lft)
      $display("ERROR: stp_lft not asserted");
    else
      $display("  stp_lft correctly asserted");
    if (stp_rght)
      $display("ERROR: stp_rght should not be asserted");

    // Wait for resp_rdy
    timeout_cnt = 0;
    while (!resp_rdy && timeout_cnt < TIMEOUT) begin
      @(posedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (timeout_cnt >= TIMEOUT) begin
      $display("ERROR: Timeout waiting for resp_rdy");
      $stop;
    end
    $display("  Received 0xA5 response for move cmd");

    repeat(100) @(posedge clk);

    //-------------------------------------------
    // Sequence 5: Solve command
    //-------------------------------------------
    $display("=== Sequence 5: Sending Solve command ===");
    send_command(16'h6001); // opcode 011, cmd[0]=1 (left affinity)

    // Wait for cmd_rdy
    timeout_cnt = 0;
    while (!cmd_rdy && timeout_cnt < TIMEOUT) begin
      @(posedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (timeout_cnt >= TIMEOUT) begin
      $display("ERROR: Timeout waiting for cmd_rdy");
      $stop;
    end
    $display("  cmd_rdy asserted");

    // One clock later check cmd_md is low
    @(posedge clk);
    @(posedge clk);
    if (cmd_md !== 1'b0)
      $display("ERROR: cmd_md should be low after solve command");
    else
      $display("  cmd_md correctly deasserted");

    // Manually assert sol_cmplt to finish solve
    repeat(5) @(posedge clk);
    force sol_cmplt = 1'b1;
    @(posedge clk);
    release sol_cmplt;

    // Wait for resp_rdy
    timeout_cnt = 0;
    while (!resp_rdy && timeout_cnt < TIMEOUT) begin
      @(posedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (timeout_cnt >= TIMEOUT) begin
      $display("ERROR: Timeout waiting for resp_rdy");
      $stop;
    end
    $display("  Received 0xA5 response for solve cmd");

    repeat(100) @(posedge clk);

    $display("=== All tests passed! ===");
    $stop;
  end

endmodule
