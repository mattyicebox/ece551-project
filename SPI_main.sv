module SPI_main(clk, rst_n, SS_n, SCLK, MOSI, MISO, wrt, cmd, done, rspns);
    input logic clk, rst_n, MISO, wrt;
    input logic [15:0] cmd;
    output logic [15:0] rspns;
    output logic SS_n, SCLK, MOSI;
    output logic done;

    // Transaction control/FSM state collapsed to a single active flag.
    logic        active;

    // SCLK generation: divide 50MHz clock by 32 using a 5-bit free-running divider.
    // The divider runs only while active; SCLK is divider MSB.
    logic [4:0]  SCLK_div;

    // Bit counter: counts sampled MISO bits (0..16) to track 16-bit packet progress.
    logic [4:0]  smpl_cnt;

    // Decoder pulses from divider to align with SPI mode timing.
    logic        smpl;
    logic        shft_imm;

    // Shared full-duplex SPI shift register (MOSI out from MSB, MISO in at LSB).
    logic [15:0] shft_reg;
    logic        MISO_smpl;
    logic [15:0] shft_next;

    // Transaction-done condition after the back-porch final shift.
    logic        finish_now;

    // Decode divider values used by the slides for sample and shift timing.
    assign smpl      = (SCLK_div == 5'b01111);
    assign shft_imm  = (SCLK_div == 5'b11111);
    assign finish_now = active && shft_imm && (smpl_cnt == 5'd16);

    // SPI pins: idle bus is SS_n=1 and SCLK=1 (CPOL=1).
    assign SS_n = ~active;
    assign SCLK = active ? SCLK_div[4] : 1'b1;
    assign MOSI = shft_reg[15];
    assign shft_next = {shft_reg[14:0], MISO_smpl};

    // Datapath + control sequencing on system clock (never clock FFs from SCLK).
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            active     <= 1'b0;
            SCLK_div   <= 5'b11111;
            smpl_cnt   <= 5'd0;
            shft_reg   <= 16'h0000;
            MISO_smpl  <= 1'b0;
            rspns      <= 16'h0000;
            done       <= 1'b0;
        end else begin
            // Start of transaction: load command and create front-porch delay.
            if (wrt && !active) begin
                active     <= 1'b1;
                SCLK_div   <= 5'b10111;
                smpl_cnt   <= 5'd0;
                shft_reg   <= cmd;
                MISO_smpl  <= 1'b0;
                done       <= 1'b0;
            end else if (active) begin
                // Sample MISO before the upcoming shift edge.
                if (smpl)
                    MISO_smpl <= MISO;

                // End transaction in back porch with one final shift into rd_data.
                if (finish_now) begin
                    shft_reg <= shft_next;
                    rspns    <= shft_next;
                    active   <= 1'b0;
                    done     <= 1'b1;
                end else begin
                    SCLK_div <= SCLK_div + 5'd1;

                    if (smpl)
                        smpl_cnt <= smpl_cnt + 5'd1;

                    // Skip first shft_imm to preserve front porch; shift remaining bits.
                    if (shft_imm && (smpl_cnt != 5'd0) && (smpl_cnt < 5'd16))
                        shft_reg <= shft_next;
                end
            end
        end
    end

endmodule