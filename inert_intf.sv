//////////////////////////////////////////////////////
// Interfaces with ST 6-axis inertial sensor.  In  //
// this application we only use Z-axis gyro for   //
// heading of mazeRunner.  Fusion correction     //
// comes from IR_Dtrm when en_fusion is high.   //
/////////////////////////////////////////////////
module inert_intf(clk,rst_n,strt_cal,cal_done,heading,rdy,IR_Dtrm,
                  SS_n,SCLK,MOSI,MISO,INT,moving,en_fusion);

  parameter FAST_SIM = 1;	// used to speed up simulation
  
  input clk, rst_n;
  input MISO;							// SPI input from inertial sensor
  input INT;							// goes high when measurement ready
  input strt_cal;						// initiate claibration of yaw readings
  input moving;							// Only integrate yaw when going
  input en_fusion;						// do fusion corr only when forward at decent clip
  input [8:0] IR_Dtrm;					// derivative term of IR sensors (used for fusion)
  
  output cal_done;				// pulses high for 1 clock when calibration done
  output signed [11:0] heading;	// heading of robot.  000 = Orig dir 3FF = 90 CCW 7FF = 180 CCW
  output rdy;					// goes high for 1 clock when new outputs ready (from inertial_integrator)
  output SS_n,SCLK,MOSI;		// SPI outputs
 

  ////////////////////////////////////////////
  // Declare any needed internal registers //
  //////////////////////////////////////////
  logic [7:0] yawL, yawH;                  // low/high gyro yaw bytes
  logic [15:0] cmd;                        // SPI command word
  logic [15:0] init_timer;                 // power-up delay timer
  logic INT_ff1, INT_ff2, INT_ff3;         // 2FF synchronizer + edge detect history
  
  //////////////////////////////////////
  // Outputs of SM are of type logic //
  ////////////////////////////////////
  logic wrt;
  logic vld;
  logic ld_yawL;
  logic ld_yawH;
  logic clr_tmr;
  logic inc_tmr;

  //////////////////////////////////////////////////////////////
  // Declare any needed internal signals that connect blocks //
  ////////////////////////////////////////////////////////////
  wire done;
  wire [15:0] inert_data;		// Data back from inertial sensor (only lower 8-bits used)
  wire signed [15:0] yaw_rt;
  wire init_done;
  wire int_rise;
  
  
  ///////////////////////////////////////
  // Create enumerated type for state //
  /////////////////////////////////////
  typedef enum logic [3:0] {
    PWRUP_WAIT,
    INIT1, INIT1_WAIT,
    INIT2, INIT2_WAIT,
    INIT3, INIT3_WAIT,
    WAIT_INT,
    READ_YAWL, READ_YAWL_WAIT,
    READ_YAWH, READ_YAWH_WAIT
  } state_t;

  state_t state, nxt_state;
  
  ////////////////////////////////////////////////////////////
  // Instantiate SPI monarch for Inertial Sensor interface //
  //////////////////////////////////////////////////////////
  SPI_main iSPI(.clk(clk),.rst_n(rst_n),.SS_n(SS_n),.SCLK(SCLK),
                .MISO(MISO),.MOSI(MOSI),.wrt(wrt),.done(done),
				.rd_data(inert_data),.wt_data(cmd));
				  
  ////////////////////////////////////////////////////////////////////
  // Instantiate Angle Engine that takes in angular rate readings  //
  // and gaurdrail info and produces a heading reading            //
  /////////////////////////////////////////////////////////////////
  inertial_integrator #(FAST_SIM) iINT(.clk(clk), .rst_n(rst_n), .strt_cal(strt_cal),
                        .vld(vld),.rdy(rdy),.cal_done(cal_done), .yaw_rt(yaw_rt),.moving(moving),
						.en_fusion(en_fusion),.IR_Dtrm(IR_Dtrm),.heading(heading));
	
  assign yaw_rt = {yawH,yawL};
  assign init_done = &init_timer;
  assign int_rise = INT_ff2 & ~INT_ff3;

  //////////////////////////
  // Infer needed flops   //
  //////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      state <= PWRUP_WAIT;
    else
      state <= nxt_state;

  // Synchronize INT and detect rising edges in clk domain.
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      INT_ff1 <= 1'b0;
      INT_ff2 <= 1'b0;
      INT_ff3 <= 1'b0;
    end else begin
      INT_ff1 <= INT;
      INT_ff2 <= INT_ff1;
      INT_ff3 <= INT_ff2;
    end

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      init_timer <= 16'h0000;
    else if (clr_tmr)
      init_timer <= 16'h0000;
    else if (inc_tmr)
      init_timer <= init_timer + 16'h0001;

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      yawL <= 8'h00;
    else if (ld_yawL)
      yawL <= inert_data[7:0];

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      yawH <= 8'h00;
    else if (ld_yawH)
      yawH <= inert_data[7:0];

  //////////////////////////////////////
  // State transition/output function //
  //////////////////////////////////////
  always_comb begin
    // defaults
    nxt_state = state;
    cmd = 16'h0000;
    wrt = 1'b0;
    vld = 1'b0;
    ld_yawL = 1'b0;
    ld_yawH = 1'b0;
    clr_tmr = 1'b0;
    inc_tmr = 1'b0;

    case (state)
      PWRUP_WAIT: begin
        inc_tmr = 1'b1;
        if (init_done)
          nxt_state = INIT1;
      end

      INIT1: begin
        cmd = 16'h0D02;
        wrt = 1'b1;
        nxt_state = INIT1_WAIT;
      end
      INIT1_WAIT: begin
        if (done)
          nxt_state = INIT2;
      end

      INIT2: begin
        cmd = 16'h1160;
        wrt = 1'b1;
        nxt_state = INIT2_WAIT;
      end
      INIT2_WAIT: begin
        if (done)
          nxt_state = INIT3;
      end

      INIT3: begin
        cmd = 16'h1440;
        wrt = 1'b1;
        nxt_state = INIT3_WAIT;
      end
      INIT3_WAIT: begin
        if (done)
          nxt_state = WAIT_INT;
      end

      WAIT_INT: begin
        if (int_rise)
          nxt_state = READ_YAWL;
      end

      READ_YAWL: begin
        cmd = 16'hA600;
        wrt = 1'b1;
        nxt_state = READ_YAWL_WAIT;
      end
      READ_YAWL_WAIT: begin
        if (done) begin
          ld_yawL = 1'b1;
          nxt_state = READ_YAWH;
        end
      end

      READ_YAWH: begin
        cmd = 16'hA700;
        wrt = 1'b1;
        nxt_state = READ_YAWH_WAIT;
      end
      default: begin // READ_YAWH_WAIT
        if (done) begin
          ld_yawH = 1'b1;
          vld = 1'b1;
          nxt_state = WAIT_INT;
        end
      end
    endcase
  end
  
  
 
endmodule
	  