`timescale 1ns/1ps

module maze_solve_tb;

    ///////////////////////
    // DUT Connections   //
    ///////////////////////
    logic cmd_md, cmd0;
    logic lft_opn, rght_opn;
    logic mv_cmplt, sol_cmplt;
    logic clk, rst_n;

    logic strt_hdng;
    logic [11:0] dsrd_hdng;
    logic strt_mv, stp_lft, stp_rght;

    localparam NORTH   = 12'h000;
    localparam WEST    = 12'h3FF;
    localparam SOUTH   = 12'h7FF;
    localparam EAST    = 12'hC00;
    localparam TIMEOUT = 25;

    ///////////////////////
    // Instantiate DUT   //
    ///////////////////////
    maze_solve iDUT(
        .cmd_md(cmd_md),
        .cmd0(cmd0),
        .lft_opn(lft_opn),
        .rght_opn(rght_opn),
        .mv_cmplt(mv_cmplt),
        .sol_cmplt(sol_cmplt),
        .clk(clk),
        .rst_n(rst_n),
        .strt_hdng(strt_hdng),
        .dsrd_hdng(dsrd_hdng),
        .strt_mv(strt_mv),
        .stp_lft(stp_lft),
        .stp_rght(stp_rght)
    );

    ///////////////////////
    // Clock Generation  //
    ///////////////////////
    initial clk = 0;
    always #5 clk = ~clk; // 100MHz

    /////////////////////////////////////////////////
    // Generate clean 3-cycle completion pulses    //
    /////////////////////////////////////////////////
    logic strt_mv_d, strt_hdng_d;
    logic [2:0] mv_pipe, hdng_pipe;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            strt_mv_d   <= 1'b0;
            strt_hdng_d <= 1'b0;
            mv_pipe     <= 3'b000;
            hdng_pipe   <= 3'b000;
        end
        else begin
            strt_mv_d   <= strt_mv;
            strt_hdng_d <= strt_hdng;
            mv_pipe     <= {mv_pipe[1:0],   strt_mv   & ~strt_mv_d};
            hdng_pipe   <= {hdng_pipe[1:0], strt_hdng & ~strt_hdng_d};
        end
    end

    assign mv_cmplt = mv_pipe[2] | hdng_pipe[2];

    ///////////////////////
    // Helper Routines   //
    ///////////////////////
    function automatic bit orthogonal_heading(input logic [11:0] hdng);
        return (hdng == NORTH) || (hdng == WEST) || (hdng == SOUTH) || (hdng == EAST);
    endfunction

    function automatic logic [11:0] turn_left(input logic [11:0] hdng);
        case (hdng)
            NORTH:   turn_left = WEST;
            WEST:    turn_left = SOUTH;
            SOUTH:   turn_left = EAST;
            EAST:    turn_left = NORTH;
            default: turn_left = NORTH;
        endcase
    endfunction

    function automatic logic [11:0] turn_right(input logic [11:0] hdng);
        case (hdng)
            NORTH:   turn_right = EAST;
            EAST:    turn_right = SOUTH;
            SOUTH:   turn_right = WEST;
            WEST:    turn_right = NORTH;
            default: turn_right = NORTH;
        endcase
    endfunction

    function automatic logic [11:0] turn_around(input logic [11:0] hdng);
        case (hdng)
            NORTH:   turn_around = SOUTH;
            WEST:    turn_around = EAST;
            SOUTH:   turn_around = NORTH;
            EAST:    turn_around = WEST;
            default: turn_around = NORTH;
        endcase
    endfunction

    task automatic fail(input string msg);
        begin
            $display("ERROR @ %0t: %s", $time, msg);
            $stop;
        end
    endtask

    task automatic check_heading(input logic [11:0] expected, input string phase);
        begin
            if (!orthogonal_heading(dsrd_hdng))
                fail($sformatf("%s: dsrd_hdng 0x%03h is not on an orthogonal axis",
                               phase, dsrd_hdng));

            if (dsrd_hdng !== expected)
                fail($sformatf("%s: dsrd_hdng 0x%03h, expected 0x%03h",
                               phase, dsrd_hdng, expected));
        end
    endtask

    function automatic logic [11:0] expected_turn(
        input bit          left_affinity,
        input logic        next_lft_opn,
        input logic        next_rght_opn,
        input logic [11:0] current_hdng
    );
        begin
            if (left_affinity) begin
                if (next_lft_opn)
                    expected_turn = turn_left(current_hdng);
                else if (next_rght_opn)
                    expected_turn = turn_right(current_hdng);
                else
                    expected_turn = turn_around(current_hdng);
            end
            else begin
                if (next_rght_opn)
                    expected_turn = turn_right(current_hdng);
                else if (next_lft_opn)
                    expected_turn = turn_left(current_hdng);
                else
                    expected_turn = turn_around(current_hdng);
            end
        end
    endfunction

    task automatic check_affinity(input bit left_affinity, input string phase);
        begin
            if ((stp_lft !== left_affinity) || (stp_rght !== ~left_affinity))
                fail($sformatf("%s: expected affinity controls stp_lft=%0b stp_rght=%0b, got stp_lft=%0b stp_rght=%0b",
                               phase, left_affinity, ~left_affinity, stp_lft, stp_rght));
        end
    endtask

    task automatic check_idle(input logic [11:0] expected, input bit left_affinity, input string phase);
        begin
            if (strt_mv || strt_hdng)
                fail($sformatf("%s: solver unexpectedly asserted strt_mv=%0b strt_hdng=%0b",
                               phase, strt_mv, strt_hdng));
            check_affinity(left_affinity, phase);
            check_heading(expected, phase);
        end
    endtask

    task automatic wait_for_move_start(
        input bit          left_affinity,
        input logic [11:0] expected,
        input string       phase
    );
        int cycles;
        bit seen;

        begin
            seen = 1'b0;
            for (cycles = 0; cycles < TIMEOUT; cycles++) begin
                @(negedge clk);
                if (strt_mv) begin
                    seen = 1'b1;
                    check_affinity(left_affinity, phase);
                    check_heading(expected, phase);
                    break;
                end
            end

            if (!seen)
                fail($sformatf("%s: timeout waiting for strt_mv", phase));
        end
    endtask

    task automatic wait_for_turn_start(
        input bit          left_affinity,
        input logic [11:0] expected,
        input string       phase
    );
        int cycles;
        bit seen;

        begin
            seen = 1'b0;
            for (cycles = 0; cycles < TIMEOUT; cycles++) begin
                @(negedge clk);
                if (strt_hdng) begin
                    seen = 1'b1;
                    check_affinity(left_affinity, phase);
                    check_heading(expected, phase);
                    break;
                end
            end

            if (!seen)
                fail($sformatf("%s: timeout waiting for strt_hdng", phase));
        end
    endtask

    task automatic complete_branch(
        input bit         left_affinity,
        input logic       next_lft_opn,
        input logic       next_rght_opn,
        input logic [11:0] current_hdng,
        input string      phase
    );
        logic [11:0] next_hdng;

        begin
            next_hdng = expected_turn(left_affinity, next_lft_opn, next_rght_opn, current_hdng);
            lft_opn  = next_lft_opn;
            rght_opn = next_rght_opn;

            repeat (2) begin
                @(negedge clk);
                check_affinity(left_affinity, {phase, " while moving"});
                check_heading(current_hdng, {phase, " while moving"});
            end

            wait_for_turn_start(left_affinity, current_hdng, {phase, " turn request"});

            @(negedge clk);
            check_affinity(left_affinity, {phase, " heading update"});
            check_heading(next_hdng, {phase, " heading update"});

            wait_for_move_start(left_affinity, next_hdng, {phase, " next move"});
        end
    endtask

    task automatic start_solution(
        input bit    left_affinity,
        input logic  initial_lft_opn,
        input logic  initial_rght_opn,
        input string mode_name
    );
        begin
            cmd_md    = 1'b1;
            cmd0      = left_affinity;
            lft_opn   = 1'b0;
            rght_opn  = 1'b0;
            sol_cmplt = 1'b0;
            rst_n     = 1'b0;

            repeat (2) @(posedge clk);
            rst_n = 1'b1;
            @(negedge clk);

            check_affinity(left_affinity, {mode_name, " after reset"});
            check_heading(NORTH, {mode_name, " after reset"});

            repeat (2) begin
                @(negedge clk);
                check_idle(NORTH, left_affinity, {mode_name, " idle before solve"});
            end

            cmd_md   = 1'b0;
            lft_opn  = initial_lft_opn;
            rght_opn = initial_rght_opn;

            wait_for_move_start(left_affinity, NORTH, {mode_name, " initial northward move"});
        end
    endtask

    task automatic finish_solution(
        input bit          left_affinity,
        input logic [11:0] expected_final_hdng,
        input string       mode_name
    );
        begin
            sol_cmplt = 1'b1;

            @(negedge clk);
            check_idle(expected_final_hdng, left_affinity, {mode_name, " solution complete branch"});

            @(negedge clk);
            check_idle(expected_final_hdng, left_affinity, {mode_name, " done-to-idle handoff"});

            cmd_md    = 1'b1;
            sol_cmplt = 1'b0;

            repeat (4) begin
                @(negedge clk);
                check_idle(expected_final_hdng, left_affinity, {mode_name, " post-completion hold"});
            end
        end
    endtask

    ///////////////////////
    // Stimulus          //
    ///////////////////////
    initial begin
        start_solution(1'b1, 1'b1, 1'b1, "left-affinity");
        complete_branch(1'b1, 1'b1, 1'b1, NORTH, "left-affinity prefers left when both open");
        complete_branch(1'b1, 1'b1, 1'b0, WEST,  "left-affinity left-only from west");
        complete_branch(1'b1, 1'b0, 1'b1, SOUTH, "left-affinity right fallback from south");
        complete_branch(1'b1, 1'b0, 1'b0, WEST,  "left-affinity 180 from west");
        complete_branch(1'b1, 1'b1, 1'b1, EAST,  "left-affinity wrap to north when both open");
        finish_solution(1'b1, NORTH, "left-affinity");

        start_solution(1'b0, 1'b1, 1'b1, "right-affinity");
        complete_branch(1'b0, 1'b1, 1'b1, NORTH, "right-affinity prefers right when both open");
        complete_branch(1'b0, 1'b0, 1'b1, EAST,  "right-affinity right-only from east");
        complete_branch(1'b0, 1'b1, 1'b0, SOUTH, "right-affinity left fallback from south");
        complete_branch(1'b0, 1'b0, 1'b0, EAST,  "right-affinity 180 from east");
        complete_branch(1'b0, 1'b1, 1'b1, WEST,  "right-affinity wrap to north when both open");
        finish_solution(1'b0, NORTH, "right-affinity");

        $display("YAHOO TEST PASSED for left and right affinity!!!!!");
        $stop;
    end

endmodule
