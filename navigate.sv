module navigate(clk,rst_n,strt_hdng,strt_mv,stp_lft,stp_rght,mv_cmplt,hdng_rdy,moving,
                en_fusion,at_hdng,lft_opn,rght_opn,frwrd_opn,frwrd_spd);
				
  parameter FAST_SIM = 1;		// speeds up incrementing of frwrd register for faster simulation
				
  input clk,rst_n;					// 50MHz clock and asynch active low reset
  input strt_hdng;					// indicates should start a new heading
  input strt_mv;					// indicates should start a new forward move
  input stp_lft;					// indicates should stop at first left opening
  input stp_rght;					// indicates should stop at first right opening
  input hdng_rdy;					// new heading reading ready....used to pace frwrd_spd increments
  output logic mv_cmplt;			// asserted when heading or forward move complete
  output logic moving;				// enables integration in PID and in inertial_integrator
  output en_fusion;					// Only enable fusion (IR reading affect on nav) when moving forward at decent speed.
  input at_hdng;					// from PID, indicates heading close enough to consider heading complete.
  input lft_opn,rght_opn,frwrd_opn;	// from IR sensors, indicates available direction.
  output reg [10:0] frwrd_spd;		// unsigned forward speed setting to PID

  ///////////////////////////
  // State encoding        //
  ///////////////////////////
  typedef enum logic [2:0] {
    IDLE      = 3'b000,
    CHNG_HDG  = 3'b001,
    ACCEL     = 3'b010,
    DEC_NORM  = 3'b011,
    DEC_FAST  = 3'b100
  } state_t;

  state_t state, nxt_state;

  ///////////////////////////
  // SM output signals     //
  ///////////////////////////
  logic init_frwrd;
  logic inc_frwrd;
  logic dec_frwrd;
  logic dec_frwrd_fast;

  localparam MAX_FRWRD = 11'h2A0;		// max forward speed
  localparam MIN_FRWRD = 11'h0D0;		// minimum duty at which wheels will turn

  ///////////////////////////
  // frwrd_inc via generate//
  ///////////////////////////
  wire [5:0] frwrd_inc;
  generate
    if (FAST_SIM)
      assign frwrd_inc = 6'h18;
    else
      assign frwrd_inc = 6'h02;
  endgenerate

  ////////////////////////////////
  // Now form forward register  //
  ////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  frwrd_spd <= 11'h000;
	else if (init_frwrd)
	  frwrd_spd <= MIN_FRWRD;
	else if (hdng_rdy && inc_frwrd && (frwrd_spd<MAX_FRWRD))
	  frwrd_spd <= frwrd_spd + {5'h00,frwrd_inc};
	else if (hdng_rdy && (frwrd_spd>11'h000) && (dec_frwrd | dec_frwrd_fast))
	  frwrd_spd <= ((dec_frwrd_fast) && (frwrd_spd>{2'h0,frwrd_inc,3'b000})) ? frwrd_spd - {2'h0,frwrd_inc,3'b000} :
                    (dec_frwrd_fast) ? 11'h000 :
	                (frwrd_spd>{4'h0,frwrd_inc,1'b0}) ? frwrd_spd - {4'h0,frwrd_inc,1'b0} :
					11'h000;

  ///////////////////////////
  // Rising edge detectors //
  ///////////////////////////
  logic lft_opn_ff, rght_opn_ff;
  logic lft_opn_rise, rght_opn_rise;

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      lft_opn_ff  <= 1'b0;
      rght_opn_ff <= 1'b0;
    end else begin
      lft_opn_ff  <= lft_opn;
      rght_opn_ff <= rght_opn;
    end
  end

  assign lft_opn_rise  = lft_opn  & ~lft_opn_ff;
  assign rght_opn_rise = rght_opn & ~rght_opn_ff;

  ///////////////////////////
  // en_fusion             //
  ///////////////////////////
  // Assert en_fusion when frwrd_spd > 1/2 MAX_FRWRD
  assign en_fusion = (frwrd_spd > (MAX_FRWRD >> 1));

  ///////////////////////////
  // State register        //
  ///////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      state <= IDLE;
    else
      state <= nxt_state;

  ///////////////////////////////////////////
  // State transition and output logic     //
  ///////////////////////////////////////////
  always_comb begin
    // Default outputs
    nxt_state      = state;
    moving         = 1'b0;
    mv_cmplt       = 1'b0;
    init_frwrd     = 1'b0;
    inc_frwrd      = 1'b0;
    dec_frwrd      = 1'b0;
    dec_frwrd_fast = 1'b0;

    case (state)
      IDLE: begin
        if (strt_hdng)
          nxt_state = CHNG_HDG;
        else if (strt_mv) begin
          // Mealy: assert init_frwrd NOW so frwrd_spd loads MIN_FRWRD
          // on the same posedge that we enter ACCEL
          init_frwrd = 1'b1;
          nxt_state  = ACCEL;
        end
      end

      CHNG_HDG: begin
        moving = 1'b1;
        if (at_hdng) begin
          mv_cmplt  = 1'b1;
          nxt_state = IDLE;
        end
      end

      ACCEL: begin
        moving    = 1'b1;
        inc_frwrd = 1'b1;
        if (!frwrd_opn)
          nxt_state = DEC_FAST;
        else if ((stp_lft  && lft_opn_rise) ||
                 (stp_rght && rght_opn_rise))
          nxt_state = DEC_NORM;
      end

      DEC_NORM: begin
        moving    = 1'b1;
        dec_frwrd = 1'b1;
        // frwrd_spd==0 means the register already decremented to zero
        // last cycle; assert mv_cmplt and return to IDLE
        if (frwrd_spd == 11'h000) begin
          dec_frwrd = 1'b0;
          mv_cmplt  = 1'b1;
          nxt_state = IDLE;
        end
      end

      DEC_FAST: begin
        moving         = 1'b1;
        dec_frwrd_fast = 1'b1;
        if (frwrd_spd == 11'h000) begin
          dec_frwrd_fast = 1'b0;
          mv_cmplt  = 1'b1;
          nxt_state = IDLE;
        end
      end

      default: nxt_state = IDLE;
    endcase
  end

endmodule