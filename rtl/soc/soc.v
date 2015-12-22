/*
 * Copyright (c) 2014, Aleksander Osman
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

module soc(
    input               CLOCK_50,

    //SDRAM
    output      [12:0]  DRAM_ADDR,
    output      [1:0]   DRAM_BA,
    output              DRAM_CAS_N,
    output              DRAM_CKE,
    output              DRAM_CLK,
    output              DRAM_CS_N,
    inout       [31:0]  DRAM_DQ,
    output      [3:0]   DRAM_DQM,
    output              DRAM_RAS_N,
    output              DRAM_WE_N,

    //PS2 KEYBOARD
    inout               PS2_CLK,
    inout               PS2_DAT,
    //PS2 MOUSE
    inout               PS2_CLK2,
    inout               PS2_DAT2,

    //KEYS
    input       [3:0]   KEY,

    //SD
    output              SD_CLK,
    inout               SD_CMD,
    inout       [3:0]   SD_DAT,
    input               SD_WP_N,

    //VGA
    output              VGA_CLK,
    output              VGA_SYNC_N,
    output              VGA_BLANK_N,
    output              VGA_HS,
    output              VGA_VS,

    output      [7:0]   VGA_R,
    output      [7:0]   VGA_G,
    output      [7:0]   VGA_B,

    //SOUND
    output              I2C_SCLK,
    inout               I2C_SDAT,
    output              AUD_XCK,
    output              AUD_BCLK,
    output              AUD_DACDAT,
    output              AUD_DACLRCK,

    //SWITCHES
    input   [17:0]  SW,

    //debug - ethernet
    output          ENET0_GTX_CLK,
    output          ENET0_RST_N,
    input           ENET0_RX_CLK,
    input   [3:0]   ENET0_RX_DATA,
    input           ENET0_RX_DV,
    output  [3:0]   ENET0_TX_DATA,
    output          ENET0_TX_EN
);

//------------------------------------------------------------------------------

assign DRAM_CLK = clk_sys;

//------------------------------------------------------------------------------

wire clk_sys;
wire clk_vga;
wire clk_sound;

wire rst_n;

pll pll_inst(
    .inclk0     (CLOCK_50),
    .c0         (clk_sys),
    .c1         (clk_vga),
    .c2         (clk_sound),
    .locked     (rst_n)
);

//------------------------------------------------------------------------------

wire [7:0] pio_output;

wire ps2_a20_enable;
wire ps2_reset_n;

//------------------------------------------------------------------------------

system u0(
    .clk_sys_clk                       (clk_sys),
    .reset_sys_reset_n                 (rst_n),

    .clk_vga_clk                       (clk_vga),
    .reset_vga_reset_n                 (rst_n),

    .clk_sound_clk                     (clk_sound),
    .reset_sound_reset_n               (rst_n),

    .export_vga_clock                  (VGA_CLK),
    .export_vga_sync_n                 (VGA_SYNC_N),
    .export_vga_blank_n                (VGA_BLANK_N),
    .export_vga_horiz_sync             (VGA_HS),
    .export_vga_vert_sync              (VGA_VS),
    .export_vga_r                      (VGA_R),
    .export_vga_g                      (VGA_G),
    .export_vga_b                      (VGA_B),

    .sdram_conduit_end_addr            (DRAM_ADDR),
    .sdram_conduit_end_ba              (DRAM_BA),
    .sdram_conduit_end_cas_n           (DRAM_CAS_N),
    .sdram_conduit_end_cke             (DRAM_CKE),
    .sdram_conduit_end_cs_n            (DRAM_CS_N),
    .sdram_conduit_end_dq              (DRAM_DQ),
    .sdram_conduit_end_dqm             (DRAM_DQM),
    .sdram_conduit_end_ras_n           (DRAM_RAS_N),
    .sdram_conduit_end_we_n            (DRAM_WE_N),

    .export_sound_sclk                 (I2C_SCLK),
    .export_sound_sdat                 (I2C_SDAT),
    .export_sound_xclk                 (AUD_XCK),
    .export_sound_bclk                 (AUD_BCLK),
    .export_sound_dat                  (AUD_DACDAT),
    .export_sound_lr                   (AUD_DACLRCK),

    .sd_clk_export                     (SD_CLK),
    .sd_dat_export                     (SD_DAT),
    .sd_cmd_export                     (SD_CMD),

    .export_ps2_out_port_a20_enable    (ps2_a20_enable),
    .export_ps2_out_port_reset_n       (ps2_reset_n),

    .export_ps2_kbclk                  (PS2_CLK),
    .export_ps2_kbdat                  (PS2_DAT),
    .export_ps2_mouseclk               (PS2_CLK2),
    .export_ps2_mousedat               (PS2_DAT2),

    .pio_input_export                  ({ 2'd0, ps2_reset_n, ps2_a20_enable, KEY }),
    .reset_only_ao486_reset            (pio_output[0] || ~(ps2_reset_n)),
    .pio_output_export                 (pio_output),

    .ao486_conduit_SW                (SW),
    .ao486_conduit_tb_finish_instr   (dbg_tb_finish_instr),

    .ao486_conduit_dbg_io_address    (dbg_io_address),
    .ao486_conduit_dbg_io_byteenable (dbg_io_byteenable),
    .ao486_conduit_dbg_io_write      (dbg_io_write),
    .ao486_conduit_dbg_io_read       (dbg_io_read),
    .ao486_conduit_dbg_io_data       (dbg_io_data),

    .ao486_conduit_dbg_int_vector    (dbg_int_vector),
    .ao486_conduit_dbg_int           (dbg_int),

    .ao486_conduit_dbg_exc_vector    (dbg_exc_vector),
    .ao486_conduit_dbg_exc           (dbg_exc),

    .ao486_conduit_dbg_mem_address   (dbg_mem_address),
    .ao486_conduit_dbg_mem_byteenable(dbg_mem_byteenable),
    .ao486_conduit_dbg_mem_write     (dbg_mem_write),
    .ao486_conduit_dbg_mem_read      (dbg_mem_read),
    .ao486_conduit_dbg_mem_data      (dbg_mem_data)

);

//------------------------------------------------------------------------------

wire        dbg_tb_finish_instr;

wire [15:0] dbg_io_address;
wire [3:0]  dbg_io_byteenable;
wire        dbg_io_write;
wire        dbg_io_read;
wire [31:0] dbg_io_data;

wire [7:0]  dbg_int_vector;
wire        dbg_int;
wire [7:0]  dbg_exc_vector;
wire        dbg_exc;

wire [31:0] dbg_mem_address;
wire [3:0]  dbg_mem_byteenable;
wire        dbg_mem_write;
wire        dbg_mem_read;
wire [31:0] dbg_mem_data;

//------------------------------------------------------------------------------ ethernet debug

wire       enet_rst_n;
wire [5:0] enet_clk;

altpll   pll_eth_inst (
    .inclk  ( {1'b0, CLOCK_50} ),
    .clk    (enet_clk),
    .locked (enet_rst_n)
);
defparam
    pll_eth_inst.clk0_divide_by         = 2,
    pll_eth_inst.clk0_duty_cycle        = 50,
    pll_eth_inst.clk0_multiply_by       = 5,
    pll_eth_inst.clk0_phase_shift       = "0",
    pll_eth_inst.clk1_divide_by         = 2,
    pll_eth_inst.clk1_duty_cycle        = 50,
    pll_eth_inst.clk1_multiply_by       = 5,
    pll_eth_inst.clk1_phase_shift       = "2000",
    pll_eth_inst.compensate_clock       = "CLK0",
    pll_eth_inst.inclk0_input_frequency = 20000,
    pll_eth_inst.operation_mode         = "NO_COMPENSATION";

wire enet_clk_data  = enet_clk[0];
wire enet_clk_clock = enet_clk[1];

assign ENET0_RST_N  = enet_rst_cnt == 20'hFFFFF;

reg [19:0] enet_rst_cnt;
always @(posedge enet_clk_data or negedge enet_rst_n) begin
    if(enet_rst_n == 1'b0)              enet_rst_cnt <= 20'd0;
    else if(enet_rst_cnt < 20'hFFFFF)   enet_rst_cnt <= enet_rst_cnt + 20'd1;
end

//------------------------------------------------------------------------------

reg [39:0] dbg_tb_finish_instr_reg;
always @(posedge clk_sys or negedge rst_n) begin
    if(rst_n == 1'b0)               dbg_tb_finish_instr_reg <= 40'd0;
    else if(dbg_tb_finish_instr)    dbg_tb_finish_instr_reg <= dbg_tb_finish_instr_reg + 40'd1;
end

//------------------------------------------------------------------------------

reg [7:0] dbg_int_vector_reg;
reg       dbg_int_reg;
always @(posedge clk_sys or negedge rst_n) begin
    if(rst_n == 1'b0)               dbg_int_vector_reg <= 8'd0;
    else if(dbg_int_reg == 1'b0)    dbg_int_vector_reg <= dbg_int_vector;
end
wire dbg_int_reg_lower = dbg_int_reg && dbg_mem_read_reg == 1'b0 && dbg_mem_write_reg == 1'b0 && dbg_io_read_reg == 1'b0 && dbg_io_write_reg == 1'b0;
always @(posedge clk_sys or negedge rst_n) begin
    if(rst_n == 1'b0)               dbg_int_reg <= 1'd0;
    else if(dbg_int_reg == 1'b0)    dbg_int_reg <= dbg_int;
    else if(dbg_int_reg_lower)      dbg_int_reg <= 1'b0;
end

reg [7:0] dbg_exc_vector_reg;
reg       dbg_exc_reg;
always @(posedge clk_sys or negedge rst_n) begin
    if(rst_n == 1'b0)               dbg_exc_vector_reg <= 8'd0;
    else if(dbg_exc_reg == 1'b0)    dbg_exc_vector_reg <= dbg_exc_vector;
end
wire dbg_exc_reg_lower = dbg_exc_reg && dbg_mem_read_reg == 1'b0 && dbg_mem_write_reg == 1'b0 && dbg_io_read_reg == 1'b0 && dbg_io_write_reg == 1'b0 && dbg_int_reg == 1'b0;
always @(posedge clk_sys or negedge rst_n) begin
    if(rst_n == 1'b0)               dbg_exc_reg <= 1'd0;
    else if(dbg_exc_reg == 1'b0)    dbg_exc_reg <= dbg_exc;
    else if(dbg_exc_reg_lower)      dbg_exc_reg <= 1'b0;
end

reg [15:0] dbg_io_address_reg;
reg [3:0]  dbg_io_byteenable_reg;
reg        dbg_io_write_reg;
reg        dbg_io_read_reg;
reg [31:0] dbg_io_data_reg;
always @(posedge clk_sys or negedge rst_n) begin if(rst_n == 1'b0) dbg_io_address_reg    <= 16'd0; else dbg_io_address_reg    <= dbg_io_address;    end
always @(posedge clk_sys or negedge rst_n) begin if(rst_n == 1'b0) dbg_io_byteenable_reg <= 4'd0;  else dbg_io_byteenable_reg <= dbg_io_byteenable; end
always @(posedge clk_sys or negedge rst_n) begin if(rst_n == 1'b0) dbg_io_write_reg      <= 1'd0;  else dbg_io_write_reg      <= dbg_io_write;      end
always @(posedge clk_sys or negedge rst_n) begin if(rst_n == 1'b0) dbg_io_read_reg       <= 1'd0;  else dbg_io_read_reg       <= dbg_io_read;       end
always @(posedge clk_sys or negedge rst_n) begin if(rst_n == 1'b0) dbg_io_data_reg       <= 32'd0; else dbg_io_data_reg       <= dbg_io_data;       end

reg [31:0] dbg_mem_address_reg;
reg [3:0]  dbg_mem_byteenable_reg;
reg        dbg_mem_write_reg;
reg        dbg_mem_read_reg;
reg [31:0] dbg_mem_data_reg;
always @(posedge clk_sys or negedge rst_n) begin if(rst_n == 1'b0) dbg_mem_address_reg    <= 32'd0; else dbg_mem_address_reg    <= dbg_mem_address;    end
always @(posedge clk_sys or negedge rst_n) begin if(rst_n == 1'b0) dbg_mem_byteenable_reg <= 4'd0;  else dbg_mem_byteenable_reg <= dbg_mem_byteenable; end
always @(posedge clk_sys or negedge rst_n) begin if(rst_n == 1'b0) dbg_mem_write_reg      <= 1'd0;  else dbg_mem_write_reg      <= dbg_mem_write;      end
always @(posedge clk_sys or negedge rst_n) begin if(rst_n == 1'b0) dbg_mem_read_reg       <= 1'd0;  else dbg_mem_read_reg       <= dbg_mem_read;       end
always @(posedge clk_sys or negedge rst_n) begin if(rst_n == 1'b0) dbg_mem_data_reg       <= 32'd0; else dbg_mem_data_reg       <= dbg_mem_data;       end

//------------------------------------------------------------------------------

wire [71:0] dbg_step1_q;
wire        dbg_step1_empty;
wire [9:0]  dbg_step1_usedw;

wire [71:0] dbg_a_next =
    (dbg_mem_read_reg || dbg_mem_write_reg)?    { 2'd1, dbg_mem_address_reg,       dbg_mem_data_reg, dbg_mem_byteenable_reg, dbg_mem_write_reg, dbg_mem_read_reg } :
    (dbg_io_read_reg  || dbg_io_write_reg)?     { 2'd2, 16'd0, dbg_io_address_reg, dbg_io_data_reg,  dbg_io_byteenable_reg,  dbg_io_write_reg,  dbg_io_read_reg } :
    (dbg_int_reg_lower)?                        { 2'd3, 22'd0, dbg_tb_finish_instr_reg, dbg_int_vector_reg } :
                                                { 2'd3, 1'd1, 21'd0, dbg_tb_finish_instr_reg, dbg_exc_vector_reg };

scfifo scfifo_a_inst(
    .clock          (clk_sys),

    .wrreq          (dbg_mem_read_reg || dbg_mem_write_reg || dbg_io_read_reg || dbg_io_write_reg || dbg_int_reg_lower || dbg_exc_reg_lower),
    .data           (dbg_a_next),

    .rdreq          (dbg_a_state_enable && dbg_a_state == 2'd2),
    .q              (dbg_step1_q),
    .empty          (dbg_step1_empty),
    .usedw          (dbg_step1_usedw)
);
defparam
    scfifo_a_inst.lpm_width       = 72,
    scfifo_a_inst.lpm_widthu      = 10,
    scfifo_a_inst.lpm_numwords    = 1024,
    scfifo_a_inst.lpm_showahead   = "ON";

wire dbg_a_state_start = dbg_step1_step >= 9'd12 && dbg_step1_step <= 9'd376 && dbg_a_state == 2'd0 && dbg_step1_empty == 1'b0;
wire dbg_a_state_enable= dbg_step1_step >= 9'd12 && dbg_step1_step <= 9'd376 && (dbg_a_state == 2'd1 || dbg_a_state == 2'd2);

reg [1:0] dbg_a_state;
always @(posedge clk_sys or negedge rst_n) begin
    if(rst_n == 1'b0)                                   dbg_a_state <= 2'd0;
    else if(dbg_a_state_start)                          dbg_a_state <= 2'd1;
    else if(dbg_a_state_enable && dbg_a_state == 2'd1)  dbg_a_state <= 2'd2;
    else if(dbg_a_state_enable && dbg_a_state == 2'd2)  dbg_a_state <= 2'd0;
end

//------------------------------------------------------------------------------

// d4 3d 7e 00 e9 c4 - 00 11 22 33 44 55 - 08 00 -
// 45 00 05 d2 00 00 40 00 40 11 - 1e 51 - 0a 01 01 65 0a 01 01 64 //1490 = 365*4+ 20 + 8 + 2
// cc dd cc de 05 be 00 00

wire [31:0] dbg_step1_part =
    (dbg_a_state == 2'd0)?  dbg_step1_q[71:40] :
    (dbg_a_state == 2'd1)?  dbg_step1_q[39:8] :
                            { dbg_step1_q[7:0], 24'd0 };

reg [19:0] dbg_step1_counter;
always @(posedge clk_sys or negedge rst_n) begin
    if(rst_n == 1'b0)               dbg_step1_counter <= 20'd0;
    else if(dbg_step1_step_last)    dbg_step1_counter <= 20'd0;
    else if(dbg_step1_empty)        dbg_step1_counter <= 20'd0;
    else if(dbg_step1_step == 9'd0) dbg_step1_counter <= dbg_step1_counter + 20'd1;
end

wire dbg_step1_start =
    dbg_step1_empty == 1'b0 && (dbg_step1_usedw == 10'd0 || dbg_step1_usedw > 10'd121 || dbg_step1_counter == 20'hFFFFF);

reg [8:0] dbg_step1_step;
always @(posedge clk_sys or negedge rst_n) begin
    if(rst_n == 1'b0)                                   dbg_step1_step <= 9'd0;
    else if(dbg_step1_step == 9'd0 && dbg_step1_start)  dbg_step1_step <= 9'd1;
    else if(dbg_step1_step_last)                        dbg_step1_step <= 9'd0;
    else if(dbg_step1_step > 9'd0)                      dbg_step1_step <= dbg_step1_step + 9'd1;
end

wire dbg_step1_step_last = dbg_step1_step == 9'd376;

reg [15:0] dbg_io_counter;
always @(posedge clk_sys or negedge rst_n) begin
    if(rst_n == 1'b0)                   dbg_io_counter <= 16'd0;
    else if(dbg_step1_step == 9'd11)    dbg_io_counter <= dbg_io_counter + 16'd1;
end

wire [31:0] dbg_step1_next =
    (dbg_step1_step == 9'd1)?   32'hD43D7E00 :
    (dbg_step1_step == 9'd2)?   32'hE9C40011 :
    (dbg_step1_step == 9'd3)?   32'h22334455 :
    (dbg_step1_step == 9'd4)?   32'h08004500 :
    (dbg_step1_step == 9'd5)?   32'h05D20000 :
    (dbg_step1_step == 9'd6)?   32'h40004011 :
    (dbg_step1_step == 9'd7)?   32'h1E510A01 :
    (dbg_step1_step == 9'd8)?   32'h01650A01 :
    (dbg_step1_step == 9'd9)?   32'h0164CCDD :
    (dbg_step1_step == 9'd10)?  32'hCCDE05BE :
    (dbg_step1_step == 9'd11)?  { 16'h0000, dbg_io_counter } :
    (dbg_step1_empty)?          32'h00000000 :
                                dbg_step1_part;

//------------------------------------------------------------------------------

wire [34:0] out_dc_q;
wire        out_dc_empty;
wire [9:0]  out_dc_rdusedw;

dcfifo dcfifo_inst(
    .wrclk          (clk_sys),
    .wrreq          (dbg_step1_step != 9'd0),
    .data           ({ dbg_step1_step_last, 2'b0, dbg_step1_next }),

    .rdclk          (enet_clk_data),
    .rdreq          (out_dc_rdreq),
    .q              (out_dc_q),
    .rdempty        (out_dc_empty),
    .rdusedw        (out_dc_rdusedw)
);
defparam
    dcfifo_inst.lpm_width       = 35,
    dcfifo_inst.lpm_widthu      = 10,
    dcfifo_inst.lpm_numwords    = 1024;

wire out_dc_rdreq;
mac_output mac_output_inst(
    .clk_data       (enet_clk_data),
    .clk_clock      (enet_clk_clock),
    .rst_n          (enet_rst_n),

    .fifo_empty     (out_dc_empty || out_dc_rdusedw < 10'd375),
    .fifo_rdreq     (out_dc_rdreq),
    // 34: end, 33,32: not valid
    .fifo_q         (out_dc_q),

    .phy_txc        (ENET0_GTX_CLK),
    .phy_ddio_td    (ENET0_TX_DATA),
    .phy_ddio_tx_ctl(ENET0_TX_EN),

    .slow           (1'b0)
);

//------------------------------------------------------------------------------

endmodule

//------------------------------------------------------------------------------ ethernet output mac

module mac_output(
    input           clk_data,
    input           clk_clock,
    input           rst_n,

    input           fifo_empty,
    output          fifo_rdreq,
    // 34: end, 33,32: not valid
    input   [34:0]  fifo_q,

    output          phy_txc,
    output  [3:0]   phy_ddio_td,
    output          phy_ddio_tx_ctl,

    input           slow
);

// *************************************************** TXC clock

altddio_out ddio_clk_inst(
    .outclock       (clk_clock),

    .datain_h       (1'b1),
    .datain_l       (1'b0),

    .dataout        (phy_txc)
);
defparam ddio_clk_inst.width = 1;


// *************************************************** DDIO

wire [7:0] phy_data_final;
reg phy_valid;

altddio_out ddio_td_inst(
    .outclock       (clk_data),

    .datain_h       (phy_data_final[3:0]),
    .datain_l       (phy_data_final[7:4]),

    .dataout        (phy_ddio_td)
);
defparam ddio_td_inst.width = 4;

altddio_out ddio_tx_ctl_inst(
    .outclock       (clk_data),

    .datain_h       (phy_valid),
    .datain_l       (phy_valid),

    .dataout        (phy_ddio_tx_ctl)
);
defparam ddio_tx_ctl_inst.width = 1;

wire idle;

// *************************************************** speed
reg slow_reg_1;
always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)   slow_reg_1 <= 1'b0;
    else                slow_reg_1 <= slow;
end

reg slow_reg_2;
always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)   slow_reg_2 <= 1'b0;
    else                slow_reg_2 <= slow_reg_1;
end

reg slow_reg;
always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)                           slow_reg <= 1'b0;
    else if(slow_reg_1 == slow_reg_2 && idle)   slow_reg <= slow_reg_2;
end

reg toggle;
always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)   toggle <= 1'b0;
    else if(slow_reg)   toggle <= ~toggle;
    else                toggle <= 1'b0;
end

wire enabled = slow_reg == 1'b0 || toggle;

// *************************************************** data path

wire [31:0] crc_output;
wire        crc_load;

wire start_preamble;
reg [2:0] preamble_cnt;
wire phy_valid_value;


reg fifo_rdreq_reg;
always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)   fifo_rdreq_reg <= 1'b0;
    else                fifo_rdreq_reg <= fifo_rdreq;
end

reg [7:0] phy_data_1;
always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)       phy_data_1 <= 8'd0;
    else if(enabled) begin
        if(fifo_rdreq_reg)  phy_data_1 <= fifo_q[7:0];
    end
end

reg [7:0] phy_data_2;
always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)           phy_data_2 <= 8'd0;
    else if(enabled) begin
        if(crc_load)            phy_data_2 <= crc_output[7:0];
        else if(fifo_rdreq_reg) phy_data_2 <= fifo_q[15:8];
        else                    phy_data_2 <= phy_data_1;
    end
end

reg [7:0] phy_data_3;
always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)           phy_data_3 <= 8'd0;
    else if(enabled) begin
        if(crc_load)            phy_data_3 <= crc_output[15:8];
        else if(fifo_rdreq_reg) phy_data_3 <= fifo_q[23:16];
        else                    phy_data_3 <= phy_data_2;
    end
end

reg [7:0] phy_data_4;
always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)           phy_data_4 <= 8'd0;
    else if(enabled) begin
        if(crc_load)            phy_data_4 <= crc_output[23:16];
        else if(fifo_rdreq_reg) phy_data_4 <= fifo_q[31:24];
        else                    phy_data_4 <= phy_data_3;
    end
end

reg [7:0] phy_data_5;
always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)                                   phy_data_5 <= 8'd0;
    else if(enabled) begin
        if(crc_load)                                    phy_data_5 <= crc_output[31:24];
        else if(preamble_cnt == 3'd7)                   phy_data_5 <= 8'hD5;
        else if(start_preamble || preamble_cnt != 3'd0) phy_data_5 <= 8'h55;
        else if(phy_valid_value == 1'b0)                phy_data_5 <= 8'h00;
        else                                            phy_data_5 <= phy_data_4;
    end
end

//-
assign phy_data_final = (slow == 1'b0)? phy_data_5 : (enabled)? { phy_data_5[7:4], phy_data_5[7:4] } : { phy_data_5[3:0], phy_data_5[3:0] };

always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)   phy_valid <= 1'b0;
    else if(enabled) begin
        phy_valid <= phy_valid_value;
    end
end

// *************************************************** counters

reg [2:0] left_cnt;
reg [4:0] gap_cnt;
reg [31:0] crc;

assign idle             = preamble_cnt == 4'd0 && left_cnt == 3'd0 && gap_cnt == 5'd0;
assign start_preamble     = idle && fifo_empty == 1'b0;

assign phy_valid_value    = start_preamble || preamble_cnt != 4'd0 || left_cnt != 3'd0 || gap_cnt >= 5'd13;
wire crc_count          = left_cnt != 3'd0 || gap_cnt >= 5'd17;
assign crc_load         = gap_cnt == 5'd16;

assign fifo_rdreq       = (slow_reg == 1'b0 && (preamble_cnt == 4'd6 || left_cnt == 3'd2)) || (slow_reg && enabled == 1'b0 && (preamble_cnt == 4'd7 || left_cnt == 3'd1));


always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)                               preamble_cnt <= 3'd0;
    else if(enabled) begin
        if(start_preamble || preamble_cnt != 3'd0)  preamble_cnt <= preamble_cnt + 3'd1;
    end
end

always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)                               left_cnt <= 3'd0;
    else if(enabled) begin
        if(fifo_rdreq_reg && fifo_q[34] == 1'b0)    left_cnt <= 3'd4;
        else if(fifo_rdreq_reg)                     left_cnt <= 3'd0;
        else if(left_cnt > 3'd0)                    left_cnt <= left_cnt - 3'd1;
    end
end


always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)                               gap_cnt <= 5'd0;
    else if(enabled) begin
        if(fifo_rdreq_reg && fifo_q[34] == 1'b1)    gap_cnt <= 5'd20 - { 3'b0, fifo_q[33:32] };
        else if(gap_cnt > 5'd0)                     gap_cnt <= gap_cnt - 5'd1;
    end
end


// *************************************************** crc

wire [31:0] crc_rev = {
    crc[24],crc[25],crc[26],crc[27],crc[28],crc[29],crc[30],crc[31],
    crc[16],crc[17],crc[18],crc[19],crc[20],crc[21],crc[22],crc[23],
    crc[8], crc[9], crc[10],crc[11],crc[12],crc[13],crc[14],crc[15],
    crc[0], crc[1], crc[2], crc[3], crc[4], crc[5], crc[6], crc[7]
};

assign crc_output  = ~crc_rev;

wire [31:0] crc_next = {
    phy_data_4[2] ^ crc[23] ^ crc[29],
    phy_data_4[0] ^ phy_data_4[3] ^ crc[22] ^ crc[28] ^ crc[31],
    phy_data_4[0] ^ phy_data_4[1] ^ phy_data_4[4] ^ crc[21] ^ crc[27] ^ crc[30] ^ crc[31],
    phy_data_4[1] ^ phy_data_4[2] ^ phy_data_4[5] ^ crc[20] ^ crc[26] ^ crc[29] ^ crc[30],
    phy_data_4[0] ^ phy_data_4[2] ^ phy_data_4[3] ^ phy_data_4[6] ^ crc[19] ^ crc[25] ^ crc[28] ^ crc[29] ^ crc[31],
    phy_data_4[1] ^ phy_data_4[3] ^ phy_data_4[4] ^ phy_data_4[7] ^ crc[18] ^ crc[24] ^ crc[27] ^ crc[28] ^ crc[30],
    phy_data_4[4] ^ phy_data_4[5] ^ crc[17] ^ crc[26] ^ crc[27],
    phy_data_4[0] ^ phy_data_4[5] ^ phy_data_4[6] ^ crc[16] ^ crc[25] ^ crc[26] ^ crc[31],
    phy_data_4[1] ^ phy_data_4[6] ^ phy_data_4[7] ^ crc[15] ^ crc[24] ^ crc[25] ^ crc[30],
    phy_data_4[7] ^ crc[14] ^ crc[24],
    phy_data_4[2] ^ crc[13] ^ crc[29],
    phy_data_4[3] ^ crc[12] ^ crc[28],
    phy_data_4[0] ^ phy_data_4[4] ^ crc[11] ^ crc[27] ^ crc[31],
    phy_data_4[0] ^ phy_data_4[1] ^ phy_data_4[5] ^ crc[10] ^ crc[26] ^ crc[30] ^ crc[31],
    phy_data_4[1] ^ phy_data_4[2] ^ phy_data_4[6] ^ crc[9] ^ crc[25] ^ crc[29] ^ crc[30],
    phy_data_4[2] ^ phy_data_4[3] ^ phy_data_4[7] ^ crc[8] ^ crc[24] ^ crc[28] ^ crc[29],
    phy_data_4[0] ^ phy_data_4[2] ^ phy_data_4[3] ^ phy_data_4[4] ^ crc[7] ^ crc[27] ^ crc[28] ^ crc[29] ^ crc[31],
    phy_data_4[0] ^ phy_data_4[1] ^ phy_data_4[3] ^ phy_data_4[4] ^ phy_data_4[5] ^ crc[6] ^ crc[26] ^ crc[27] ^ crc[28] ^ crc[30] ^ crc[31],
    phy_data_4[0] ^ phy_data_4[1] ^ phy_data_4[2] ^ phy_data_4[4] ^ phy_data_4[5] ^ phy_data_4[6] ^ crc[5] ^ crc[25] ^ crc[26] ^ crc[27] ^ crc[29] ^ crc[30] ^ crc[31],
    phy_data_4[1] ^ phy_data_4[2] ^ phy_data_4[3] ^ phy_data_4[5] ^ phy_data_4[6] ^ phy_data_4[7] ^ crc[4] ^ crc[24] ^ crc[25] ^ crc[26] ^ crc[28] ^ crc[29] ^ crc[30],
    phy_data_4[3] ^ phy_data_4[4] ^ phy_data_4[6] ^ phy_data_4[7] ^ crc[3] ^ crc[24] ^ crc[25] ^ crc[27] ^ crc[28],
    phy_data_4[2] ^ phy_data_4[4] ^ phy_data_4[5] ^ phy_data_4[7] ^ crc[2] ^ crc[24] ^ crc[26] ^ crc[27] ^ crc[29],
    phy_data_4[2] ^ phy_data_4[3] ^ phy_data_4[5] ^ phy_data_4[6] ^ crc[1] ^ crc[25] ^ crc[26] ^ crc[28] ^ crc[29],
    phy_data_4[3] ^ phy_data_4[4] ^ phy_data_4[6] ^ phy_data_4[7] ^ crc[0] ^ crc[24] ^ crc[25] ^ crc[27] ^ crc[28],
    phy_data_4[0] ^ phy_data_4[2] ^ phy_data_4[4] ^ phy_data_4[5] ^ phy_data_4[7] ^ crc[24] ^ crc[26] ^ crc[27] ^ crc[29] ^ crc[31],
    phy_data_4[0] ^ phy_data_4[1] ^ phy_data_4[2] ^ phy_data_4[3] ^ phy_data_4[5] ^ phy_data_4[6] ^ crc[25] ^ crc[26] ^ crc[28] ^ crc[29] ^ crc[30] ^ crc[31],
    phy_data_4[0] ^ phy_data_4[1] ^ phy_data_4[2] ^ phy_data_4[3] ^ phy_data_4[4] ^ phy_data_4[6] ^ phy_data_4[7] ^ crc[24] ^ crc[25] ^ crc[27] ^ crc[28] ^ crc[29] ^ crc[30] ^ crc[31],
    phy_data_4[1] ^ phy_data_4[3] ^ phy_data_4[4] ^ phy_data_4[5] ^ phy_data_4[7] ^ crc[24] ^ crc[26] ^ crc[27] ^ crc[28] ^ crc[30],
    phy_data_4[0] ^ phy_data_4[4] ^ phy_data_4[5] ^ phy_data_4[6] ^ crc[25] ^ crc[26] ^ crc[27] ^ crc[31],
    phy_data_4[0] ^ phy_data_4[1] ^ phy_data_4[5] ^ phy_data_4[6] ^ phy_data_4[7] ^ crc[24] ^ crc[25] ^ crc[26] ^ crc[30] ^ crc[31],
    phy_data_4[0] ^ phy_data_4[1] ^ phy_data_4[6] ^ phy_data_4[7] ^ crc[24] ^ crc[25] ^ crc[30] ^ crc[31],
    phy_data_4[1] ^ phy_data_4[7] ^ crc[24] ^ crc[30]
};

always @(posedge clk_data or negedge rst_n) begin
    if(rst_n == 1'b0)       crc <= 32'hFFFFFFFF;
    else if(enabled) begin
        if(crc_count)       crc <= crc_next;
        else if(crc_load)   crc <= 32'hFFFFFFFF;
    end
end

endmodule
