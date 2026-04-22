module CommTB();

    //--------------------------------------------------
    // Clock & reset
    //--------------------------------------------------
    reg clk, rst_n;
    always #10 clk = ~clk;   // 50 MHz

    //--------------------------------------------------
    // Timeouts
    //--------------------------------------------------
    localparam int CMD_TIMEOUT_CLKS  = 300000;
    localparam int RESP_TIMEOUT_CLKS = 300000;

    //--------------------------------------------------
    // Wires between DUT and RemoteComm
    //--------------------------------------------------
    wire TX_RX;   // RemoteComm TX -> UART_wrapper RX
    wire RX_TX;   // UART_wrapper TX -> RemoteComm RX

    //--------------------------------------------------
    // RemoteComm interface
    //--------------------------------------------------
    reg  [15:0] cmd;
    reg         snd_cmd;
    wire        cmd_snt;
    wire [7:0]  resp;
    wire        resp_rdy;

    //--------------------------------------------------
    // UART_wrapper interface
    //--------------------------------------------------
    wire        cmd_rdy;
    wire [15:0] cmd_out;
    reg         clr_cmd_rdy;
    reg         trmt;
    reg  [7:0]  resp_byte;
    wire        tx_done;

    //--------------------------------------------------
    // Test tracking
    //--------------------------------------------------
    int pass_count = 0;
    int fail_count = 0;
    int tx_count   = 0;

    //--------------------------------------------------
    // DUT instantiations
    //--------------------------------------------------
    RemoteComm iRC(
        .clk     (clk),
        .rst_n   (rst_n),
        .TX      (TX_RX),    // RemoteComm transmits -> wrapper's RX
        .RX      (RX_TX),    // RemoteComm receives on wrapper's TX
        .cmd     (cmd),
        .send_cmd(snd_cmd),
        .cmd_sent(cmd_snt),
        .resp    (resp),
        .resp_rdy(resp_rdy)
    );

    UART_wrapper iDUT(
        .clk        (clk),
        .rst_n      (rst_n),
        .RX         (TX_RX),   // wrapper receives RemoteComm's TX
        .TX         (RX_TX),   // wrapper transmits -> RemoteComm's RX
        .clr_cmd_rdy(clr_cmd_rdy),
        .trmt       (trmt),
        .resp       (resp_byte),
        .tx_done    (tx_done),
        .cmd_rdy    (cmd_rdy),
        .cmd        (cmd_out)
    );

    task automatic wait_for_cmd_rdy(output bit got_cmd);
        int i;
        got_cmd = 1'b0;
        for (i = 0; i < CMD_TIMEOUT_CLKS && !got_cmd; i++) begin
            @(posedge clk);
            if (cmd_rdy)
                got_cmd = 1'b1;
        end
    endtask

    task automatic wait_for_resp_rdy(output bit got_resp);
        int i;
        got_resp = 1'b0;
        for (i = 0; i < RESP_TIMEOUT_CLKS && !got_resp; i++) begin
            @(posedge clk);
            if (resp_rdy)
                got_resp = 1'b1;
        end
    endtask

    //--------------------------------------------------
    // Task: send command, verify cmd, send ack, verify resp
    //--------------------------------------------------
    task automatic send_and_check(
        input [15:0] test_cmd,
        input [7:0]  ack_byte,
        input string test_name
    );
        bit got_cmd, got_resp;
        tx_count = tx_count + 1;

        @(negedge clk);
        cmd       = test_cmd;
        resp_byte = ack_byte;
        snd_cmd   = 1'b1;
        @(negedge clk);
        snd_cmd   = 1'b0;

        wait_for_cmd_rdy(got_cmd);

        if (!got_cmd) begin
            $display("FAIL [%s]: TIMEOUT waiting for cmd_rdy", test_name);
            fail_count++;
        end else if (cmd_out !== test_cmd) begin
            $display("FAIL [%s]: cmd=0x%04h, expected 0x%04h", test_name, cmd_out, test_cmd);
            fail_count++;
        end else begin
            $display("PASS [%s]: cmd_rdy asserted, cmd=0x%04h", test_name, cmd_out);
            pass_count++;
        end

        if (got_cmd) begin
            @(negedge clk);
            clr_cmd_rdy = 1'b1;
            @(negedge clk);
            clr_cmd_rdy = 1'b0;
            // Self-check: cmd_rdy must be deasserted by clr_cmd_rdy
            if (cmd_rdy !== 1'b0) begin
                $display("FAIL [%s]: cmd_rdy still high after clr_cmd_rdy pulse", test_name);
                fail_count++;
            end else begin
                $display("PASS [%s]: cmd_rdy cleared by clr_cmd_rdy", test_name);
                pass_count++;
            end
        end

        @(negedge clk);
        trmt = 1'b1;
        @(negedge clk);
        trmt = 1'b0;

        wait_for_resp_rdy(got_resp);

        if (!got_resp) begin
            $display("FAIL [%s]: TIMEOUT waiting for resp_rdy", test_name);
            fail_count++;
        end else if (resp !== ack_byte) begin
            $display("FAIL [%s]: resp=0x%02h, expected 0x%02h", test_name, resp, ack_byte);
            fail_count++;
        end else begin
            $display("PASS [%s]: resp=0x%02h received correctly", test_name, resp);
            pass_count++;
        end

        repeat(20) @(posedge clk);
    endtask

    //--------------------------------------------------
    // Main test sequence
    //--------------------------------------------------
    initial begin
        clk         = 0;
        rst_n       = 0;
        snd_cmd     = 0;
        clr_cmd_rdy = 0;
        trmt        = 0;
        cmd         = 16'h0000;
        resp_byte   = 8'h00;

        repeat(5) @(negedge clk);
        rst_n = 1;
        repeat(5) @(negedge clk);

        // Original 6 transmissions (12 checks total)
        send_and_check(16'hABCD, 8'h5A, "Basic_ABCD");
        send_and_check(16'h0000, 8'h00, "AllZeros");
        send_and_check(16'hFFFF, 8'hFF, "AllOnes");
        send_and_check(16'hBE00, 8'hAA, "HighByteOnly");
        send_and_check(16'h00EF, 8'h01, "LowByteOnly");
        send_and_check(16'hA5A5, 8'h5A, "Alternating");

        // 20 additional edge-case transmissions
        send_and_check(16'h0001, 8'h10, "Edge_0001");
        send_and_check(16'h0002, 8'h20, "Edge_0002");
        send_and_check(16'h0004, 8'h40, "Edge_0004");
        send_and_check(16'h0008, 8'h80, "Edge_0008");
        send_and_check(16'h0010, 8'h01, "Edge_0010");
        send_and_check(16'h0020, 8'h02, "Edge_0020");
        send_and_check(16'h0040, 8'h04, "Edge_0040");
        send_and_check(16'h0080, 8'h08, "Edge_0080");
        send_and_check(16'h0100, 8'h11, "Edge_0100");
        send_and_check(16'h0200, 8'h22, "Edge_0200");
        send_and_check(16'h0400, 8'h44, "Edge_0400");
        send_and_check(16'h0800, 8'h88, "Edge_0800");
        send_and_check(16'h1000, 8'hFE, "Edge_1000");
        send_and_check(16'h2000, 8'hEF, "Edge_2000");
        send_and_check(16'h4000, 8'h7E, "Edge_4000");
        send_and_check(16'h8000, 8'hE7, "Edge_8000");
        send_and_check(16'h00FF, 8'h3C, "Edge_00FF");
        send_and_check(16'hFF00, 8'hC3, "Edge_FF00");
        send_and_check(16'h7FFF, 8'h7F, "Edge_7FFF");
        send_and_check(16'h8001, 8'h81, "Edge_8001");

        $display("\n=============================");
        $display("  TEST SUMMARY");
        $display("  TRANSMISSIONS: %0d", tx_count);
        $display("  PASSED CHECKS: %0d", pass_count);
        $display("  FAILED CHECKS: %0d", fail_count);
        $display("=============================\n");

        if (fail_count == 0)
            $display(">>> ALL TESTS PASSED <<<");
        else
            $display(">>> %0d CHECK(S) FAILED <<<", fail_count);

        $stop;
    end

endmodule
