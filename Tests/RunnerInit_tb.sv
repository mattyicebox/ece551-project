module MazeRunner_tb();
  
  reg clk,RST_n;
  reg send_cmd;					// assert to send command to MazeRunner_tb
  reg [15:0] cmd;				// 16-bit command to send
  reg [11:0] batt;				// battery voltage 0xD80 is nominal
  
  logic cmd_sent;				
  logic resp_rdy;				// MazeRunner has sent a pos acknowledge
  logic [7:0] resp;				// resp byte from MazeRunner (hopefully 0xA5)
  logic hall_n;					// magnet found?
  
  /////////////////////////////////////////////////////////////////////////
  // Signals interconnecting MazeRunner to RunnerPhysics and RemoteComm //
  ///////////////////////////////////////////////////////////////////////
  wire TX_RX,RX_TX;
  wire INRT_SS_n,INRT_SCLK,INRT_MOSI,INRT_MISO,INRT_INT;
  wire lftPWM1,lftPWM2,rghtPWM1,rghtPWM2;
  wire A2D_SS_n,A2D_SCLK,A2D_MOSI,A2D_MISO;
  wire IR_lft_en,IR_cntr_en,IR_rght_en;  
  wire piezo,piezo_n;
  wire [7:0] LED;

  integer tests_run;
  integer tests_failed;
  reg clk_run;

  ///// Internal registers for testing purposes??? /////////

  
  //////////////////////
  // Instantiate DUT //
  ////////////////////
  MazeRunner iDUT(.clk(clk),.RST_n(RST_n),.INRT_SS_n(INRT_SS_n),.INRT_SCLK(INRT_SCLK),
                  .INRT_MOSI(INRT_MOSI),.INRT_MISO(INRT_MISO),.INRT_INT(INRT_INT),
				  .A2D_SS_n(A2D_SS_n),.A2D_SCLK(A2D_SCLK),.A2D_MOSI(A2D_MOSI),
				  .A2D_MISO(A2D_MISO),.lftPWM1(lftPWM1),.lftPWM2(lftPWM2),
				  .rghtPWM1(rghtPWM1),.rghtPWM2(rghtPWM2),.RX(RX_TX),.TX(TX_RX),
          .hall_n(hall_n),.piezo(piezo),.piezo_n(piezo_n),.IR_lft_en(IR_lft_en),
          .IR_rght_en(IR_rght_en),.IR_cntr_en(IR_cntr_en),.LED(LED));
	
  ///////////////////////////////////////////////////////////////////////////////////////
  // Instantiate RemoteComm which models bluetooth module receiving & forwarding cmds //
  /////////////////////////////////////////////////////////////////////////////////////
  RemoteComm iCMD(.clk(clk), .rst_n(RST_n), .RX(TX_RX), .TX(RX_TX), .cmd(cmd), .send_cmd(send_cmd),
               .cmd_sent(cmd_sent), .resp_rdy(resp_rdy), .resp(resp));
			   
  ///////////////////////////////////////////////////
  // Instantiate physical model of robot and maze //
  /////////////////////////////////////////////////
  RunnerPhysics iPHYS(.clk(clk),.RST_n(RST_n),.SS_n(INRT_SS_n),.SCLK(INRT_SCLK),.MISO(INRT_MISO),
                      .MOSI(INRT_MOSI),.INT(INRT_INT),.lftPWM1(lftPWM1),.lftPWM2(lftPWM2),
					  .rghtPWM1(rghtPWM1),.rghtPWM2(rghtPWM2),
                     .IR_lft_en(IR_lft_en),.IR_cntr_en(IR_cntr_en),.IR_rght_en(IR_rght_en),
					 .A2D_SS_n(A2D_SS_n),.A2D_SCLK(A2D_SCLK),.A2D_MOSI(A2D_MOSI),
					 .A2D_MISO(A2D_MISO),.hall_n(hall_n),.batt(batt));


					 
  task automatic report_test;
    input bit pass;
    input [1023:0] name;
    begin
      tests_run = tests_run + 1;
      if (pass)
        $display("TEST PASS [%0d] %0s", tests_run, name);
      else begin
        tests_failed = tests_failed + 1;
        $display("TEST FAIL [%0d] %0s", tests_run, name);
      end
    end
  endtask

  task automatic check_post_reset_midrail;
    input [1023:0] name;
    input integer settle_cycles;
    reg pass;
    begin
      repeat (settle_cycles) @(posedge clk);
      pass = (iDUT.iMTR.lft_duty === 12'h800) &&
             (iDUT.iMTR.rght_duty === 12'h800) &&
             (lftPWM1 !== 1'bx) && (lftPWM2 !== 1'bx) &&
             (rghtPWM1 !== 1'bx) && (rghtPWM2 !== 1'bx);
      report_test(pass, name);
    end
  endtask

  task automatic check_in_reset_safe;
    input [1023:0] name;
    reg pass;
    begin
      #1;
      pass = (lftPWM1 === 1'b0) && (lftPWM2 === 1'b0) &&
             (rghtPWM1 === 1'b0) && (rghtPWM2 === 1'b0);
      report_test(pass, name);
    end
  endtask

  initial begin
  integer tgl_count;
  integer cyc;
  integer en_sum;
  reg prev_pwm;
  reg pass;
  reg saw_high;
  reg saw_yawL;
  reg saw_yawH;
  reg [11:0] snap_lft_duty;
  reg [11:0] snap_rght_duty;
  reg [2:0] snap_nav_state;
  reg saw_strt_cal;
  reg saw_cal_state;
  reg saw_smpl_adv;
  reg saw_cal_done;
  reg [11:0] prev_smpl_cntr;

    ////////////////////////////
    // Basic initialization   //
    ////////////////////////////
  clk = 1'b0;
  clk_run = 1'b1;
  RST_n = 1'b0;
  send_cmd = 1'b0;
  cmd = 16'h0000;
  batt = 12'hD80;
  tests_run = 0;
  tests_failed = 0;

  ////////////////////////////////////////////////////
  // 1) Cold boot with clean reset                  //
  //////////////////////////////////////////////////
  repeat (8) @(posedge clk);
  RST_n = 1'b1;
  check_post_reset_midrail("1) Cold boot with clean reset", 20);

  ////////////////////////////////////////////////////
  // 2) Minimum-width reset pulse                    //
  //////////////////////////////////////////////////
  @(negedge clk);
  RST_n = 1'b0;
  @(posedge clk);
  RST_n = 1'b1;
  check_post_reset_midrail("2) Minimum-width reset pulse", 20);

  ////////////////////////////////////////////////////
  // 3) Long reset pulse                             //
  //////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (200) @(posedge clk);
  RST_n = 1'b1;
  check_post_reset_midrail("3) Long reset pulse", 20);

  ////////////////////////////////////////////////////
  // 4) Reset release on clock edge                  //
  //////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (6) @(posedge clk);
  @(posedge clk);
  RST_n = 1'b1;
  check_post_reset_midrail("4) Reset release on clock edge", 20);

  ////////////////////////////////////////////////////
  // 5) Reset release off clock edge                 //
  //////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (6) @(posedge clk);
  #3 RST_n = 1'b1;
  check_post_reset_midrail("5) Reset release off clock edge", 20);

  ////////////////////////////////////////////////////
  // 6) Multiple reset pulses before startup done    //
  //////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (2) @(posedge clk);
  RST_n = 1'b1;
  repeat (3) @(posedge clk);
  RST_n = 1'b0;
  repeat (2) @(posedge clk);
  RST_n = 1'b1;
  repeat (4) @(posedge clk);
  RST_n = 1'b0;
  repeat (2) @(posedge clk);
  RST_n = 1'b1;
  check_post_reset_midrail("6) Multiple reset pulses before startup completion", 25);

  ////////////////////////////////////////////////////
  // 7) Reset during active operation                //
  //////////////////////////////////////////////////
  tgl_count = 0;
  prev_pwm = lftPWM1;
  repeat (2000) begin
    @(posedge clk);
    if (lftPWM1 !== prev_pwm)
      tgl_count = tgl_count + 1;
    prev_pwm = lftPWM1;
  end
  RST_n = 1'b0;
  #1;
  RST_n = 1'b1;
  repeat (20) @(posedge clk);
  report_test((tgl_count > 0) &&
              (iDUT.iMTR.lft_duty === 12'h800) &&
              (iDUT.iMTR.rght_duty === 12'h800),
              "7) Reset during active operation");

  ////////////////////////////////////////////////////
  // 8) Reset glitch immunity/recovery               //
  //////////////////////////////////////////////////
  repeat (30) @(posedge clk);
  RST_n = 1'b0;
  #2 RST_n = 1'b1;
  check_post_reset_midrail("8) Reset glitch recovery", 25);

  ////////////////////////////////////////////////////
  // 9) Clock starts before reset release            //
  //////////////////////////////////////////////////
  clk_run = 1'b1;
  clk = 1'b0;
  RST_n = 1'b0;
  repeat (10) @(posedge clk);
  RST_n = 1'b1;
  check_post_reset_midrail("9) Clock starts before reset release", 20);

  ////////////////////////////////////////////////////
  // 10) Clock starts after reset asserted           //
  //////////////////////////////////////////////////
  clk_run = 1'b0;
  clk = 1'b0;
  RST_n = 1'b1;
  #20;
  RST_n = 1'b0;
  check_in_reset_safe("10) Reset asserted while clock is stopped");
  #20;
  clk_run = 1'b1;
  repeat (6) @(posedge clk);
  RST_n = 1'b1;
  check_post_reset_midrail("10) Clock starts after reset asserted", 20);

  ////////////////////////////////////////////////////
  // 11) Initial output safety                       //
  //////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (lftPWM1 === 1'b0) && (lftPWM2 === 1'b0) &&
         (rghtPWM1 === 1'b0) && (rghtPWM2 === 1'b0) &&
         (IR_lft_en === 1'b0) && (IR_cntr_en === 1'b0) && (IR_rght_en === 1'b0) &&
         (piezo === 1'b0) && (piezo_n === 1'b1);
  report_test(pass, "11) Initial output safety while in reset");

  ////////////////////////////////////////////////////
  // 12) No motor drive during reset                 //
  //////////////////////////////////////////////////
  pass = 1'b1;
  repeat (80) begin
    @(posedge clk);
    if ((lftPWM1 !== 1'b0) || (lftPWM2 !== 1'b0) ||
        (rghtPWM1 !== 1'b0) || (rghtPWM2 !== 1'b0))
      pass = 1'b0;
  end
  report_test(pass, "12) No motor drive while reset asserted");
  RST_n = 1'b1;
  repeat (20) @(posedge clk);

  ////////////////////////////////////////////////////
  // 13) PWM startup phase correctness               //
  //////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b1;
  saw_high = 1'b0;
  repeat (30) begin
    @(posedge clk);
    if (lftPWM1 !== 1'b0)
      pass = 1'b0;
  end
  repeat (120) begin
    @(posedge clk);
    if (lftPWM1 === 1'b1)
      saw_high = 1'b1;
  end
  report_test(pass && saw_high, "13) PWM startup phase has expected delayed first-high");

  ////////////////////////////////////////////////////
  // 14) PWM duty at neutral command                 //
  //////////////////////////////////////////////////
  repeat (20) @(posedge clk);
  report_test((iDUT.iMTR.lft_duty === 12'h800) &&
              (iDUT.iMTR.rght_duty === 12'h800),
              "14) PWM duty is midrail at neutral command");

  ////////////////////////////////////////////////////
  // 15) PWM complement/non-overlap sanity           //
  //////////////////////////////////////////////////
  pass = 1'b1;
  repeat (5000) begin
    @(posedge clk);
    if ((lftPWM1 === 1'b1 && lftPWM2 === 1'b1) ||
        (rghtPWM1 === 1'b1 && rghtPWM2 === 1'b1))
      pass = 1'b0;
  end
  report_test(pass, "15) Complementary PWM outputs never overlap high");

  ////////////////////////////////////////////////////
  // 16) No spurious pulse at reset deassertion      //
  //////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b1;
  repeat (20) begin
    @(posedge clk);
    if ((lftPWM1 !== 1'b0) || (lftPWM2 !== 1'b0) ||
        (rghtPWM1 !== 1'b0) || (rghtPWM2 !== 1'b0))
      pass = 1'b0;
  end
  report_test(pass, "16) No spurious immediate motor pulse after reset release");

  ////////////////////////////////////////////////////
  // 17) LED/status startup defaults                 //
  //////////////////////////////////////////////////
  repeat (40) @(posedge clk);
  pass = (LED[6:1] === 6'h00) && (LED[0] === 1'b0) && (^LED[6:0] !== 1'bx);
  report_test(pass, "17) LED status defaults after initialization");

  ////////////////////////////////////////////////////
  // 18) Piezo startup muted/complementary           //
  //////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (piezo === 1'b0) && (piezo_n === 1'b1);
  RST_n = 1'b1;
  repeat (300) begin
    @(posedge clk);
    if ((piezo !== 1'b0) || (piezo_n !== 1'b1))
      pass = 1'b0;
  end
  report_test(pass, "18) Piezo starts muted with complementary idle levels");

  ////////////////////////////////////////////////////
  // 19) Sensor enable default behavior              //
  //////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (IR_lft_en === 1'b0) && (IR_cntr_en === 1'b0) && (IR_rght_en === 1'b0);
  RST_n = 1'b1;
  repeat (1200) begin
    @(posedge clk);
    if ((IR_lft_en === 1'bx) || (IR_cntr_en === 1'bx) || (IR_rght_en === 1'bx))
      pass = 1'b0;
    en_sum = (IR_lft_en === 1'b1) + (IR_cntr_en === 1'b1) + (IR_rght_en === 1'b1);
    if (en_sum > 1)
      pass = 1'b0;
  end
  report_test(pass, "19) Sensor enables default low and stay one-hot when active");

  ////////////////////////////////////////////////////
  // 20) UART TX idle after reset                    //
  //////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (TX_RX === 1'b1);
  RST_n = 1'b1;
  repeat (3000) begin
    @(posedge clk);
    if (TX_RX !== 1'b1)
      pass = 1'b0;
  end
  report_test(pass, "20) UART TX remains idle high after reset");

  ////////////////////////////////////////////////////
  // 21) UART RX ignores garbage until framed command //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  repeat (20) @(posedge clk);
  force RX_TX = 1'b0;
  repeat (1200) @(posedge clk);
  release RX_TX;
  repeat (50) @(posedge clk);
  pass = (iDUT.iWRAP.cmd_rdy === 1'b0);
  report_test(pass, "21) UART RX path ignores garbage activity");

  ////////////////////////////////////////////////////
  // 22) UART cmd_rdy starts low                     //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.iWRAP.cmd_rdy === 1'b0);
  report_test(pass, "22) UART cmd_rdy defaults low on reset");
  RST_n = 1'b1;
  repeat (10) @(posedge clk);

  ////////////////////////////////////////////////////
  // 23) UART resp_rdy starts low                    //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (resp_rdy === 1'b0);
  report_test(pass, "23) UART resp_rdy defaults low on reset");
  RST_n = 1'b1;
  repeat (10) @(posedge clk);

  ////////////////////////////////////////////////////
  // 24) SPI chip-select idle polarity               //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (INRT_SS_n === 1'b1) && (A2D_SS_n === 1'b1);
  report_test(pass, "24) SPI SS_n idles high during reset");
  RST_n = 1'b1;
  repeat (40) @(posedge clk);

  ////////////////////////////////////////////////////
  // 25) SPI SCLK idle polarity and first activity   //
  ////////////////////////////////////////////////////
  pass = (INRT_SCLK === 1'b1) && (A2D_SCLK === 1'b1);
  report_test(pass, "25) SPI SCLK idles high before transactions");

  ////////////////////////////////////////////////////
  // 26) SPI MOSI idle value sanity                  //
  ////////////////////////////////////////////////////
  pass = (INRT_MOSI !== 1'bx) && (A2D_MOSI !== 1'bx);
  report_test(pass, "26) SPI MOSI has known idle value");

  ////////////////////////////////////////////////////
  // 27) No inertial SPI traffic before init_done    //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b1;
  repeat (2000) begin
    @(posedge clk);
    if (!iDUT.iNEMO.init_done && (INRT_SS_n === 1'b0))
      pass = 1'b0;
  end
  report_test(pass, "27) No inertial SPI command before init_done");

  ////////////////////////////////////////////////////
  // 28) Inertial power-up timer starts at zero      //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.iNEMO.init_timer === 16'h0000);
  report_test(pass, "28) Inertial init_timer resets to zero");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 29) Inertial FSM reaches INIT1                  //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b0;
  for (cyc = 0; cyc < 80000; cyc = cyc + 1) begin
    @(posedge clk);
    if (iDUT.iNEMO.state == iDUT.iNEMO.INIT1)
      pass = 1'b1;
  end
  report_test(pass, "29) Inertial FSM eventually enters INIT1");

  ////////////////////////////////////////////////////
  // 30) Inertial register write #1 command          //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b0;
  for (cyc = 0; cyc < 120000; cyc = cyc + 1) begin
    @(posedge clk);
    if ((iDUT.iNEMO.state == iDUT.iNEMO.INIT1) && (iDUT.iNEMO.cmd === 16'h0D02))
      pass = 1'b1;
  end
  report_test(pass, "30) Inertial init writes 0D<=02");

  ////////////////////////////////////////////////////
  // 31) Inertial register write #2 command          //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b0;
  for (cyc = 0; cyc < 120000; cyc = cyc + 1) begin
    @(posedge clk);
    if ((iDUT.iNEMO.state == iDUT.iNEMO.INIT2) && (iDUT.iNEMO.cmd === 16'h1160))
      pass = 1'b1;
  end
  report_test(pass, "31) Inertial init writes 11<=60");

  ////////////////////////////////////////////////////
  // 32) Inertial register write #3 command          //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b0;
  for (cyc = 0; cyc < 120000; cyc = cyc + 1) begin
    @(posedge clk);
    if ((iDUT.iNEMO.state == iDUT.iNEMO.INIT3) && (iDUT.iNEMO.cmd === 16'h1440))
      pass = 1'b1;
  end
  report_test(pass, "32) Inertial init writes 14<=40");

  ////////////////////////////////////////////////////
  // 33) Inertial FSM reaches WAIT_INT               //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b0;
  for (cyc = 0; cyc < 120000; cyc = cyc + 1) begin
    @(posedge clk);
    if (iDUT.iNEMO.state == iDUT.iNEMO.WAIT_INT)
      pass = 1'b1;
  end
  report_test(pass, "33) Inertial FSM reaches WAIT_INT");

  ////////////////////////////////////////////////////
  // 34) NEMO_setup eventually asserts               //
  ////////////////////////////////////////////////////
  pass = 1'b0;
  for (cyc = 0; cyc < 400000; cyc = cyc + 1) begin
    @(posedge clk);
    if (iPHYS.iNEMO.NEMO_setup === 1'b1)
      pass = 1'b1;
  end
  report_test(pass, "34) RunnerPhysics iNEMO NEMO_setup eventually high");

  ////////////////////////////////////////////////////
  // 35) INT synchronizer flops start known          //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.iNEMO.INT_ff1 === 1'b0) &&
         (iDUT.iNEMO.INT_ff2 === 1'b0) &&
         (iDUT.iNEMO.INT_ff3 === 1'b0);
  report_test(pass, "35) Inertial INT synchronizer flops reset cleanly");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 36) No integrator vld before WAIT_INT reached   //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b1;
  for (cyc = 0; cyc < 70000; cyc = cyc + 1) begin
    @(posedge clk);
    if ((iDUT.iNEMO.state != iDUT.iNEMO.WAIT_INT) && (iDUT.iNEMO.vld === 1'b1))
      pass = 1'b0;
  end
  report_test(pass, "36) No vld pulse before inertial init completes");

  ////////////////////////////////////////////////////
  // 37) A2D interface idle correctness              //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (A2D_SS_n === 1'b1) && (A2D_SCLK === 1'b1) && (A2D_MOSI !== 1'bx);
  report_test(pass, "37) A2D SPI bus idles correctly on reset");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 38) A2D round-robin scan starts                 //
  ////////////////////////////////////////////////////
  pass = 1'b0;
  for (cyc = 0; cyc < 200000; cyc = cyc + 1) begin
    @(posedge clk);
    if (A2D_SS_n === 1'b0)
      pass = 1'b1;
  end
  report_test(pass, "38) A2D SPI transactions begin after startup");

  ////////////////////////////////////////////////////
  // 39) Initial battery reading becomes valid       //
  ////////////////////////////////////////////////////
  pass = 1'b0;
  for (cyc = 0; cyc < 240000; cyc = cyc + 1) begin
    @(posedge clk);
    if ((iDUT.vbatt !== 12'hxxx) && (iDUT.vbatt != 12'h000))
      pass = 1'b1;
  end
  report_test(pass, "39) Battery reading path produces non-zero valid value");

  ////////////////////////////////////////////////////
  // 40) IR readings become known after startup      //
  ////////////////////////////////////////////////////
  pass = 1'b1;
  for (cyc = 0; cyc < 120000; cyc = cyc + 1) begin
    @(posedge clk);
    if ((iDUT.lft_IR === 12'hxxx) || (iDUT.rght_IR === 12'hxxx))
      pass = 1'b0;
  end
  report_test(pass, "40) IR readings are known (non-X) after startup");

  ////////////////////////////////////////////////////
  // 41) strt_cal default low on reset               //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.strt_cal === 1'b0);
  report_test(pass, "41) strt_cal defaults low on reset");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 42) cal_done default low on reset               //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.cal_done === 1'b0);
  report_test(pass, "42) cal_done defaults low on reset");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 44) hdng_rdy default low on reset               //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.hdng_rdy === 1'b0);
  report_test(pass, "44) hdng_rdy defaults low on reset");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 45) actl_hdng starts near zero                  //
  ////////////////////////////////////////////////////
  repeat (30) @(posedge clk);
  pass = (iDUT.actl_hdng === 12'h000) || (iDUT.actl_hdng === 12'hFFF) || (iDUT.actl_hdng === 12'h001);
  report_test(pass, "45) actl_hdng starts near zero at bring-up");

  ////////////////////////////////////////////////////
  // 46) PID integrator clears on reset              //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.iCNTRL.integrator === 16'sh0000);
  report_test(pass, "46) PID integrator clears to zero on reset");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 47) PID derivative history clears on reset      //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.iCNTRL.ff1 === 10'sh000) && (iDUT.iCNTRL.prev_err === 10'sh000);
  report_test(pass, "47) PID derivative history registers clear on reset");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 48) frwrd_spd default zero                      //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.iNAV.frwrd_spd === 11'h000);
  report_test(pass, "48) navigate frwrd_spd defaults to zero");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 49) navigate FSM initializes to IDLE            //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.iNAV.state == iDUT.iNAV.IDLE);
  report_test(pass, "49) navigate FSM resets to IDLE");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 50) cmd_proc FSM initializes to IDLE            //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.iCMD.state == iDUT.iCMD.IDLE);
  report_test(pass, "50) cmd_proc FSM resets to IDLE");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 51) maze_solve FSM initializes to IDLE          //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.iSLV.state == iDUT.iSLV.IDLE);
  report_test(pass, "51) maze_solve FSM resets to IDLE");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 52) cmd_md defaults high                        //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.cmd_md === 1'b1);
  report_test(pass, "52) cmd_md defaults high on reset");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 53) control handshake defaults are low          //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.cmd_rdy === 1'b0) && (iDUT.send_resp === 1'b0) &&
         (iDUT.mv_cmplt === 1'b0) && (iDUT.strt_mv === 1'b0) && (iDUT.strt_hdng === 1'b0);
  report_test(pass, "53) Key control handshakes default low on reset");
  RST_n = 1'b1;

  ////////////////////////////////////////////////////
  // 54) Top-level outputs stay known (no X/Z)      //
  ////////////////////////////////////////////////////
  pass = 1'b1;
  repeat (8000) begin
    @(posedge clk);
    if ((lftPWM1 === 1'bx) || (lftPWM2 === 1'bx) || (rghtPWM1 === 1'bx) || (rghtPWM2 === 1'bx) ||
        (INRT_SS_n === 1'bx) || (A2D_SS_n === 1'bx) || (TX_RX === 1'bx) ||
        (lftPWM1 === 1'bz) || (lftPWM2 === 1'bz) || (rghtPWM1 === 1'bz) || (rghtPWM2 === 1'bz))
      pass = 1'b0;
  end
  report_test(pass, "54) Critical top-level outputs remain known (no X/Z)");

  ////////////////////////////////////////////////////
  // 55) Critical internals stay known              //
  ////////////////////////////////////////////////////
  pass = 1'b1;
  repeat (5000) begin
    @(posedge clk);
    if ((iDUT.vbatt === 12'hxxx) || (iDUT.actl_hdng === 12'hxxx) || (iDUT.iNAV.frwrd_spd === 11'hxxx))
      pass = 1'b0;
  end
  report_test(pass, "55) Critical internal bring-up signals remain known");

  ////////////////////////////////////////////////////
  // 56) WAIT_INT reached within bounded time        //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b0;
  for (cyc = 0; cyc < 120000; cyc = cyc + 1) begin
    @(posedge clk);
    if (iDUT.iNEMO.state == iDUT.iNEMO.WAIT_INT)
      pass = 1'b1;
  end
  report_test(pass, "56) Inertial startup reaches WAIT_INT within timeout");

  ////////////////////////////////////////////////////
  // 57) Deterministic reset snapshot               //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (6) @(posedge clk);
  RST_n = 1'b1;
  repeat (60) @(posedge clk);
  snap_lft_duty = iDUT.iMTR.lft_duty;
  snap_rght_duty = iDUT.iMTR.rght_duty;
  snap_nav_state = iDUT.iNAV.state;
  RST_n = 1'b0;
  repeat (6) @(posedge clk);
  RST_n = 1'b1;
  repeat (60) @(posedge clk);
  pass = (iDUT.iMTR.lft_duty == snap_lft_duty) &&
         (iDUT.iMTR.rght_duty == snap_rght_duty) &&
         (iDUT.iNAV.state == snap_nav_state);
  report_test(pass, "57) Reset startup snapshot is deterministic");

  ////////////////////////////////////////////////////
  // 58) Low-battery startup flag behavior          //
  ////////////////////////////////////////////////////
  batt = 12'hC80;
  RST_n = 1'b0;
  repeat (6) @(posedge clk);
  RST_n = 1'b1;
  repeat (80000) @(posedge clk);
  pass = (iDUT.batt_low === 1'b1);
  report_test(pass, "58) Low battery input leads to batt_low assertion");

  ////////////////////////////////////////////////////
  // 59) High-battery startup flag behavior         //
  ////////////////////////////////////////////////////
  batt = 12'hE20;
  RST_n = 1'b0;
  repeat (6) @(posedge clk);
  RST_n = 1'b1;
  repeat (80000) @(posedge clk);
  pass = (iDUT.batt_low === 1'b0);
  report_test(pass, "59) High battery input keeps batt_low deasserted");

  ////////////////////////////////////////////////////
  // 60) End-to-end startup readiness               //
  ////////////////////////////////////////////////////
  pass = 1'b0;
  for (cyc = 0; cyc < 200000; cyc = cyc + 1) begin
    @(posedge clk);
    if ((iDUT.iNEMO.state == iDUT.iNEMO.WAIT_INT) &&
        (A2D_SS_n == 1'b0 || iDUT.iIR.cnv_cmplt == 1'b1) &&
        (iDUT.iMTR.lft_duty === 12'h800) && (iDUT.iMTR.rght_duty === 12'h800))
      pass = 1'b1;
  end
  report_test(pass, "60) Startup reaches inertial+A2D+motor-ready condition");

  ////////////////////////////////////////////////////
  // 61) Synced reset signal behaves correctly      //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  #1;
  pass = (iDUT.rst_n === 1'b0);
  RST_n = 1'b1;
  repeat (20) @(posedge clk);
  pass = pass && (iDUT.rst_n === 1'b1);
  report_test(pass, "61) reset_synch drives rst_n low/high correctly");

  ////////////////////////////////////////////////////
  // 62) No spurious send_resp without commands     //
  ////////////////////////////////////////////////////
  send_cmd = 1'b0;
  pass = 1'b1;
  repeat (200000) begin
    @(posedge clk);
    if (iDUT.send_resp === 1'b1)
      pass = 1'b0;
  end
  report_test(pass, "62) No unsolicited send_resp when no command is sent");

  ////////////////////////////////////////////////////
  // 63) hall_n inactive => no solve complete       //
  ////////////////////////////////////////////////////
  force hall_n = 1'b1;
  repeat (20) @(posedge clk);
  pass = (iDUT.sol_cmplt === 1'b0) && (LED[7] === 1'b0);
  release hall_n;
  report_test(pass, "63) hall_n high keeps sol_cmplt and LED[7] deasserted");

  ////////////////////////////////////////////////////
  // 64) hall_n active => solve complete            //
  ////////////////////////////////////////////////////
  force hall_n = 1'b0;
  repeat (20) @(posedge clk);
  pass = (iDUT.sol_cmplt === 1'b1) && (LED[7] === 1'b1);
  release hall_n;
  report_test(pass, "64) hall_n low asserts sol_cmplt and LED[7]");


  ////////////////////////////////////////////////////
  // 66) INIT command sequence order is preserved   //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (6) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b0;
  snap_nav_state = 3'h0;
  for (cyc = 0; cyc < 200000; cyc = cyc + 1) begin
    @(posedge clk);
    if ((iDUT.iNEMO.state == iDUT.iNEMO.INIT1) && (iDUT.iNEMO.cmd == 16'h0D02))
      snap_nav_state[0] = 1'b1;
    if ((iDUT.iNEMO.state == iDUT.iNEMO.INIT2) && (iDUT.iNEMO.cmd == 16'h1160) && snap_nav_state[0])
      snap_nav_state[1] = 1'b1;
    if ((iDUT.iNEMO.state == iDUT.iNEMO.INIT3) && (iDUT.iNEMO.cmd == 16'h1440) && snap_nav_state[1])
      snap_nav_state[2] = 1'b1;
  end
  pass = &snap_nav_state;
  report_test(pass, "66) Inertial init commands appear in expected order");

  ////////////////////////////////////////////////////
  // 67) FSM state legality during startup window   //
  ////////////////////////////////////////////////////
  pass = 1'b1;
  repeat (120000) begin
    @(posedge clk);
    if ((iDUT.iNEMO.state > iDUT.iNEMO.READ_YAWH_WAIT) ||
        (iDUT.iCMD.state > iDUT.iCMD.SOLVE) ||
        (iDUT.iNAV.state > iDUT.iNAV.DEC_FAST))
      pass = 1'b0;
  end
  report_test(pass, "67) Major FSM states remain in legal encoded ranges");

  ////////////////////////////////////////////////////
  // 68) init_timer monotonic while counting        //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (4) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b1;
  snap_lft_duty = 12'h000;
  for (cyc = 0; cyc < 20000; cyc = cyc + 1) begin
    @(posedge clk);
    if (iDUT.iNEMO.init_timer < snap_lft_duty)
      pass = 1'b0;
    snap_lft_duty = iDUT.iNEMO.init_timer;
  end
  report_test(pass, "68) Inertial init_timer increments monotonically");

  ////////////////////////////////////////////////////
  // 69) No command accepted before being sent      //
  ////////////////////////////////////////////////////
  send_cmd = 1'b0;
  pass = 1'b1;
  repeat (100000) begin
    @(posedge clk);
    if (iDUT.iCMD.state != iDUT.iCMD.IDLE)
      pass = 1'b0;
  end
  report_test(pass, "69) cmd_proc remains IDLE when no commands are issued");

  ////////////////////////////////////////////////////
  // 70) Final recovery reset after stress          //
  ////////////////////////////////////////////////////
  RST_n = 1'b0;
  repeat (6) @(posedge clk);
  RST_n = 1'b1;
  pass = 1'b0;
  for (cyc = 0; cyc < 180000; cyc = cyc + 1) begin
    @(posedge clk);
    if ((iDUT.iNEMO.state == iDUT.iNEMO.WAIT_INT) &&
        (iDUT.iMTR.lft_duty === 12'h800) &&
        (iDUT.iMTR.rght_duty === 12'h800) &&
        (iDUT.iNAV.state == iDUT.iNAV.IDLE))
      pass = 1'b1;
  end
  report_test(pass, "70) Post-stress reset recovers to startup-ready state");

  $display("Initialization suite complete: tests_run=%0d tests_failed=%0d", tests_run, tests_failed);
  if (tests_failed == 0)
    $display("ALL FIRST 70 INITIALIZATION TESTS PASSED");
  else
    $display("ONE OR MORE INITIALIZATION TESTS FAILED");

    $stop();
	
  end
  
  initial begin
    forever begin
      if (clk_run)
        #5 clk = ~clk;
      else
        #1;
    end
  end
	
endmodule
