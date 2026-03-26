// Testbench for inert_intf_test
`timescale 1ns/1ps

module inert_intf_test_tb;
    // Clock and reset
    logic clk;
    logic RST_n;
    
    // DUT I/O
    logic [7:0] LED;
    logic MISO;
    logic MOSI;
    logic SCLK;
    logic SS_n;
    logic INT;

    // Instantiate DUT
    inert_intf_test dut (
        .clk(clk),
        .RST_n(RST_n),
        .LED(LED),
        .MISO(MISO),
        .MOSI(MOSI),
        .SCLK(SCLK),
        .SS_n(SS_n),
        .INT(INT)
    );

    // Clock generation: 100 MHz -> 10 ns period
    initial clk = 0;
    always #5 clk = ~clk;

    // Reset sequence
    initial begin
        RST_n = 0;
        INT   = 0;
        MISO  = 0;
        #50;
        RST_n = 1;
    end

    // Stimulus
    initial begin
        // Wait for reset deassertion
        @(posedge RST_n);
        repeat (20) @(posedge clk);

        fork
            // INT pulse generator.
            // The inert_intf FSM takes two sequential SPI reads per INT
            // (READ_YAWL + READ_YAWH), each 16 bits x 32 sys-clocks = 512
            // clocks = 5120 ns.  Two reads = ~10240 ns plus FSM overhead.
            // Pulse INT every 12000 ns (1200 clocks) -- slow enough that
            // the FSM always finishes both SPI transactions before the next
            // rising edge reaches the synchronizer.
            forever begin
                #12000;
                INT = 1;
                #50;        // hold high for 5 clocks (enough for 2-FF sync)
                INT = 0;
            end

            // MISO stimulus: randomize between SPI clocks.
            // SCLK idles high and toggles at ~3.125 MHz (320 ns period),
            // so changing MISO every 160 ns keeps it stable across edges.
            forever begin
                @(negedge SCLK);    // change on falling SCLK -- safe setup time
                MISO = $random;
            end
        join_none

        // Runtime budget:
        //   17-bit timer overflow : 2^17 clocks x 10 ns  =  1.31 ms
        //   3 SPI init transactions                       ~  0.05 ms
        //   Calibration: 2048 INTs x 12 us/INT            ~ 24.58 ms
        //   Margin                                         ~  4.06 ms
        //                                              total = 30 ms
        #30_000_000;

        $display("Simulation complete. Final LED=%h heading=%0d",
                 LED, $signed(dut.intf.iINT.heading));
        $finish;
    end

    // Monitor: print only when outputs change to keep log manageable
    initial begin
        $monitor("%0t ns | LED=%h SS_n=%b SCLK=%b MOSI=%b heading=%0d",
                 $time, LED, SS_n, SCLK, MOSI,
                 $signed(dut.intf.iINT.heading));
    end

endmodule