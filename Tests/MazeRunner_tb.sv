module MazeRunner_tb();

  localparam int unsigned RESET_CYCLES         = 8;
  localparam int unsigned CMD_TIMEOUT_CYCLES   = 300_000;
  localparam int unsigned CAL_TIMEOUT_CYCLES   = 600_000;
  localparam int unsigned HDNG_TIMEOUT_CYCLES  = 1_000_000;
  localparam int unsigned HDNG_TOLERANCE       = 40;
  localparam logic [15:0] CMD_CALIBRATE        = 16'h0000;
  localparam logic [15:0] CMD_HEADING_WEST     = 16'h23FF;
  localparam logic signed [11:0] WEST_HEADING  = 12'sh3FF;
  localparam logic [7:0] POS_ACK               = 8'hA5;

  logic clk, RST_n;
  logic send_cmd;
  logic [15:0] cmd;
  logic [11:0] batt;

  logic cmd_sent;
  logic resp_rdy;
  logic [7:0] resp;
  logic hall_n;

  logic TX_RX, RX_TX;
  logic INRT_SS_n, INRT_SCLK, INRT_MOSI, INRT_MISO, INRT_INT;
  logic lftPWM1, lftPWM2, rghtPWM1, rghtPWM2;
  logic A2D_SS_n, A2D_SCLK, A2D_MOSI, A2D_MISO;
  logic IR_lft_en, IR_cntr_en, IR_rght_en;
  logic piezo;

  MazeRunner iDUT(
    .clk(clk),
    .RST_n(RST_n),
    .INRT_SS_n(INRT_SS_n),
    .INRT_SCLK(INRT_SCLK),
    .INRT_MOSI(INRT_MOSI),
    .INRT_MISO(INRT_MISO),
    .INRT_INT(INRT_INT),
    .A2D_SS_n(A2D_SS_n),
    .A2D_SCLK(A2D_SCLK),
    .A2D_MOSI(A2D_MOSI),
    .A2D_MISO(A2D_MISO),
    .lftPWM1(lftPWM1),
    .lftPWM2(lftPWM2),
    .rghtPWM1(rghtPWM1),
    .rghtPWM2(rghtPWM2),
    .RX(RX_TX),
    .TX(TX_RX),
    .hall_n(hall_n),
    .piezo(piezo),
    .piezo_n(),
    .IR_lft_en(IR_lft_en),
    .IR_rght_en(IR_rght_en),
    .IR_cntr_en(IR_cntr_en),
    .LED()
  );

  RemoteComm iCMD(
    .clk(clk),
    .rst_n(RST_n),
    .RX(TX_RX),
    .TX(RX_TX),
    .cmd(cmd),
    .send_cmd(send_cmd),
    .cmd_sent(cmd_sent),
    .resp_rdy(resp_rdy),
    .resp(resp)
  );

  RunnerPhysics iPHYS(
    .clk(clk),
    .RST_n(RST_n),
    .SS_n(INRT_SS_n),
    .SCLK(INRT_SCLK),
    .MISO(INRT_MISO),
    .MOSI(INRT_MOSI),
    .INT(INRT_INT),
    .lftPWM1(lftPWM1),
    .lftPWM2(lftPWM2),
    .rghtPWM1(rghtPWM1),
    .rghtPWM2(rghtPWM2),
    .IR_lft_en(IR_lft_en),
    .IR_cntr_en(IR_cntr_en),
    .IR_rght_en(IR_rght_en),
    .A2D_SS_n(A2D_SS_n),
    .A2D_SCLK(A2D_SCLK),
    .A2D_MOSI(A2D_MOSI),
    .A2D_MISO(A2D_MISO),
    .hall_n(hall_n),
    .batt(batt)
  );

  // Internal probes that match the project slide and make later scenarios easy to add.
  wire dut_strt_cal                 = iDUT.strt_cal;
  wire dut_in_cal                   = iDUT.iCMD.in_cal;
  wire dut_cal_done                 = iDUT.cal_done;
  wire dut_strt_hdng                = iDUT.strt_hdng;
  wire dut_mv_cmplt                 = iDUT.mv_cmplt;
  wire signed [11:0] dut_dsrd_hdng  = iDUT.dsrd_hdng;
  wire signed [11:0] dut_actl_hdng  = iDUT.actl_hdng;
  wire signed [15:0] phys_omega_lft = iPHYS.omega_lft;
  wire signed [15:0] phys_omega_rgt = iPHYS.omega_rght;
  wire signed [15:0] phys_heading_v = iPHYS.heading_v;
  wire signed [11:0] phys_heading   = iPHYS.heading_robot[19:8];

  function automatic int signed heading_error(
    input logic signed [11:0] actual,
    input logic signed [11:0] desired
  );
    int signed diff;
    begin
      diff = $signed(actual) - $signed(desired);
      if (diff > 2047)
        diff = diff - 4096;
      else if (diff < -2048)
        diff = diff + 4096;
      heading_error = diff;
    end
  endfunction

  function automatic int unsigned abs_int(input int signed value);
    begin
      abs_int = (value < 0) ? -value : value;
    end
  endfunction

  task automatic fail(input string msg);
    begin
      $fatal(1, "[%0t] %s", $time, msg);
    end
  endtask

  task automatic check(input bit condition, input string msg);
    begin
      if (!condition)
        fail(msg);
    end
  endtask

  task automatic wait_clks(input int unsigned num_clks);
    begin
      repeat (num_clks)
        @(posedge clk);
    end
  endtask

  task automatic apply_reset();
    begin
      clk = 1'b0;
      RST_n = 1'b0;
      send_cmd = 1'b0;
      cmd = 16'h0000;
      batt = 12'hD80;
      wait_clks(RESET_CYCLES);
      RST_n = 1'b1;
      wait_clks(RESET_CYCLES);
    end
  endtask

  task automatic issue_command(
    input logic [15:0] cmd_word,
    input string cmd_name
  );
    int unsigned cycles;
    begin
      $display("[%0t] Sending %s command: 0x%04h", $time, cmd_name, cmd_word);
      @(posedge clk);
      cmd <= cmd_word;
      send_cmd <= 1'b1;
      @(posedge clk);
      send_cmd <= 1'b0;

      cycles = 0;
      while (!cmd_sent && (cycles < CMD_TIMEOUT_CYCLES)) begin
        @(posedge clk);
        cycles++;
      end
      if (!cmd_sent)
        fail($sformatf("Timed out waiting for RemoteComm to finish sending %s", cmd_name));
    end
  endtask

  task automatic wait_for_positive_ack(input string ack_context);
    int unsigned cycles;
    begin
      cycles = 0;
      while (!resp_rdy && (cycles < CMD_TIMEOUT_CYCLES)) begin
        @(posedge clk);
        cycles++;
      end

      if (!resp_rdy)
        fail($sformatf("Timed out waiting for positive acknowledgement after %s", ack_context));

      check(resp === POS_ACK,
            $sformatf("Expected acknowledgement 0x%02h after %s, observed 0x%02h",
                      POS_ACK, ack_context, resp));

      $display("[%0t] Positive acknowledgement received for %s", $time, ack_context);
    end
  endtask

  task automatic exercise_calibration();
    int unsigned cycles;
    begin
      issue_command(CMD_CALIBRATE, "calibrate");

      cycles = 0;
      while (!dut_strt_cal && (cycles < CMD_TIMEOUT_CYCLES)) begin
        @(posedge clk);
        cycles++;
      end
      if (!dut_strt_cal)
        fail("strt_cal was never asserted after the calibration command");
      $display("[%0t] strt_cal asserted", $time);

      cycles = 0;
      while (!dut_in_cal && (cycles < CMD_TIMEOUT_CYCLES)) begin
        @(posedge clk);
        cycles++;
        if (resp_rdy)
          fail("Calibration acknowledgement arrived before in_cal asserted");
      end
      if (!dut_in_cal)
        fail("in_cal never asserted after calibration started");
      $display("[%0t] in_cal asserted and calibration is active", $time);

      cycles = 0;
      while (!dut_cal_done && (cycles < CAL_TIMEOUT_CYCLES)) begin
        @(posedge clk);
        cycles++;
        if (!dut_in_cal)
          fail("in_cal deasserted before cal_done");
        if (!dut_strt_cal)
          fail("strt_cal deasserted before cal_done");
        if (resp_rdy)
          fail("Calibration acknowledgement arrived before cal_done");
      end
      if (!dut_cal_done)
        fail("Timed out waiting for cal_done");
      $display("[%0t] cal_done asserted after %0d cycles in calibration", $time, cycles);

      wait_for_positive_ack("calibration");

      @(posedge clk);
      check(!dut_in_cal, "in_cal should deassert after the calibration sequence completes");
    end
  endtask

  task automatic exercise_heading_change(
    input logic [15:0] heading_cmd,
    input logic signed [11:0] expected_heading,
    input bit expect_ccw_turn,
    input string heading_name
  );
    int unsigned cycles;
    int unsigned prev_heading_v_mag;
    int unsigned curr_heading_v_mag;
    int unsigned prev_heading_err_mag;
    bit saw_expected_wheel_motion;
    bit saw_expected_yaw_direction;
    bit saw_heading_v_rise;
    bit saw_heading_v_fall;
    bit saw_heading_progress;
    begin
      issue_command(heading_cmd, {"heading ", heading_name});

      cycles = 0;
      while (!dut_strt_hdng && (cycles < CMD_TIMEOUT_CYCLES)) begin
        @(posedge clk);
        cycles++;
      end
      if (!dut_strt_hdng)
        fail($sformatf("strt_hdng was never asserted for the %s heading command", heading_name));
      $display("[%0t] strt_hdng asserted for heading %s", $time, heading_name);

      cycles = 0;
      while ((dut_dsrd_hdng !== expected_heading) && (cycles < CMD_TIMEOUT_CYCLES)) begin
        @(posedge clk);
        cycles++;
        if (resp_rdy)
          fail("Heading acknowledgement arrived before dsrd_hdng updated");
      end
      if (dut_dsrd_hdng !== expected_heading)
        fail($sformatf("dsrd_hdng never loaded 0x%03h for heading %s",
                       expected_heading[11:0], heading_name));
      $display("[%0t] dsrd_hdng loaded to 0x%03h", $time, dut_dsrd_hdng[11:0]);

      saw_expected_wheel_motion = 1'b0;
      saw_expected_yaw_direction = 1'b0;
      saw_heading_v_rise = 1'b0;
      saw_heading_v_fall = 1'b0;
      saw_heading_progress = 1'b0;
      prev_heading_v_mag = abs_int($signed(phys_heading_v));
      prev_heading_err_mag = abs_int(heading_error(dut_actl_hdng, expected_heading));

      cycles = 0;
      while (!dut_mv_cmplt && (cycles < HDNG_TIMEOUT_CYCLES)) begin
        @(posedge clk);
        cycles++;

        if (resp_rdy)
          fail("Heading acknowledgement arrived before mv_cmplt");

        if (expect_ccw_turn) begin
          if (($signed(phys_omega_lft) < 0) && ($signed(phys_omega_rgt) > 0))
            saw_expected_wheel_motion = 1'b1;
          if ($signed(phys_heading_v) > 0)
            saw_expected_yaw_direction = 1'b1;
        end else begin
          if (($signed(phys_omega_lft) > 0) && ($signed(phys_omega_rgt) < 0))
            saw_expected_wheel_motion = 1'b1;
          if ($signed(phys_heading_v) < 0)
            saw_expected_yaw_direction = 1'b1;
        end

        curr_heading_v_mag = abs_int($signed(phys_heading_v));
        if (curr_heading_v_mag > prev_heading_v_mag)
          saw_heading_v_rise = 1'b1;
        if (saw_heading_v_rise && (curr_heading_v_mag < prev_heading_v_mag))
          saw_heading_v_fall = 1'b1;
        prev_heading_v_mag = curr_heading_v_mag;

        if (abs_int(heading_error(dut_actl_hdng, expected_heading)) < prev_heading_err_mag)
          saw_heading_progress = 1'b1;
        prev_heading_err_mag = abs_int(heading_error(dut_actl_hdng, expected_heading));
      end

      if (!dut_mv_cmplt)
        fail($sformatf("Timed out waiting for the %s heading move to complete", heading_name));

      check(saw_expected_wheel_motion,
            $sformatf("Wheel velocities never matched the expected %s turn direction",
                      heading_name));
      check(saw_expected_yaw_direction,
            $sformatf("heading_v never moved in the expected direction for heading %s",
                      heading_name));
      check(saw_heading_v_rise,
            $sformatf("heading_v never increased in magnitude during heading %s", heading_name));
      check(saw_heading_v_fall,
            $sformatf("heading_v never decreased after ramping during heading %s", heading_name));
      check(saw_heading_progress,
            $sformatf("actl_hdng never moved closer to dsrd_hdng during heading %s", heading_name));
      check(abs_int(heading_error(dut_actl_hdng, expected_heading)) <= HDNG_TOLERANCE,
            $sformatf("actl_hdng did not converge near dsrd_hdng for heading %s. actl=0x%03h dsrd=0x%03h",
                      heading_name, dut_actl_hdng[11:0], expected_heading[11:0]));

      $display("[%0t] Heading %s converged. actl_hdng=0x%03h dsrd_hdng=0x%03h phys_heading=0x%03h",
               $time, heading_name, dut_actl_hdng[11:0], dut_dsrd_hdng[11:0], phys_heading[11:0]);

      wait_for_positive_ack({"heading change to ", heading_name});
    end
  endtask

  task automatic run_slide_example();
    begin
      $display("============================================================");
      $display("Slide scenario: calibrate, then change heading to west");
      $display("============================================================");
      exercise_calibration();
      exercise_heading_change(CMD_HEADING_WEST, WEST_HEADING, 1'b1, "west");
    end
  endtask

  initial begin
    apply_reset();
    run_slide_example();
    $display("[%0t] MazeRunner full-system slide scenario PASSED", $time);
    $finish;
  end

  always
    #5 clk = ~clk;

endmodule
