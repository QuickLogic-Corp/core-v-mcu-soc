// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.


`define REG_INFO        7'b0000000 //BASEADDR+0x00 CONTAINS NUMBER OF CORES [31:16] AND CLUSTERS [15:0]
`define REG_FCBOOT      7'b0000001 //BASEADDR+0x04 not used at the moment
`define REG_FCFETCH     7'b0000010 //BASEADDR+0x08 not used at the moment

`define REG_PADFUN0     7'b0000100 //BASEADDR+0x10 sets the mux for pins  0 (bits [1:0]) to 15 (bits [31:30])
`define REG_PADFUN1     7'b0000101 //BASEADDR+0x14 sets the mux for pins 16 (bits [1:0]) to 31 (bits [31:30])
`define REG_PADFUN2     7'b0000110 //BASEADDR+0x18 sets the mux for pins 32 (bits [1:0]) to 47 (bits [31:30])
`define REG_PADFUN3     7'b0000111 //BASEADDR+0x1C sets the mux for pins 48 (bits [1:0]) to 63 (bits [31:30])

//PADs configuration is made of 8bits out of which only the first 6 are used
//bit0    enable pull UP
//bit1    enable pull DOWN
//bit2    enable ST
//bit3    enable SlewRate Limit
//bit4..5 Driving Strength
//bit6..7 not used
`define REG_PADCFG0     7'b0001000 //BASEADDR+0x20 sets config for pin  0(bits [7:0]) to pin  3(bits [31:24])
`define REG_PADCFG1     7'b0001001 //BASEADDR+0x24 sets config for pin  4(bits [7:0]) to pin  7(bits [31:24])
`define REG_PADCFG2     7'b0001010 //BASEADDR+0x28 sets config for pin  8(bits [7:0]) to pin 11(bits [31:24])
`define REG_PADCFG3     7'b0001011 //BASEADDR+0x2C sets config for pin 12(bits [7:0]) to pin 15(bits [31:24])
`define REG_PADCFG4     7'b0001100 //BASEADDR+0x30 sets config for pin 16(bits [7:0]) to pin 19(bits [31:24])
`define REG_PADCFG5     7'b0001101 //BASEADDR+0x34 sets config for pin 20(bits [7:0]) to pin 23(bits [31:24])
`define REG_PADCFG6     7'b0001110 //BASEADDR+0x38 sets config for pin 24(bits [7:0]) to pin 27(bits [31:24])
`define REG_PADCFG7     7'b0001111 //BASEADDR+0x3C sets config for pin 28(bits [7:0]) to pin 31(bits [31:24])
`define REG_PADCFG8     7'b0010000 //BASEADDR+0x40 sets config for pin 32(bits [7:0]) to pin 35(bits [31:24])
`define REG_PADCFG9     7'b0010001 //BASEADDR+0x44 sets config for pin 36(bits [7:0]) to pin 39(bits [31:24])
`define REG_PADCFG10    7'b0010010 //BASEADDR+0x48 sets config for pin 40(bits [7:0]) to pin 43(bits [31:24])
`define REG_PADCFG11    7'b0010011 //BASEADDR+0x4C sets config for pin 44(bits [7:0]) to pin 47(bits [31:24])
`define REG_PADCFG12    7'b0010100 //BASEADDR+0x50 sets config for pin 48(bits [7:0]) to pin 51(bits [31:24])
`define REG_PADCFG13    7'b0010101 //BASEADDR+0x54 sets config for pin 52(bits [7:0]) to pin 55(bits [31:24])
`define REG_PADCFG14    7'b0010110 //BASEADDR+0x58 sets config for pin 56(bits [7:0]) to pin 59(bits [31:24])
`define REG_PADCFG15    7'b0010111 //BASEADDR+0x5C sets config for pin 60(bits [7:0]) to pin 63(bits [31:24])

`define REG_JTAGREG     7'b0011101 //BASEADDR+0x74 JTAG REG

`define REG_CORESTATUS  7'b0101000 //BASEADDR+0xA0 32bit GP register to be used during testing to return EOC(bit[31]) and status(bit[30:0])
`define REG_CS_RO       7'b0110000 //BASEADDR+0xC0 32bit GP register to be used during testing to return EOC(bit[31]) and status(bit[30:0]) Read Only mirror
`define REG_BOOTSEL     7'b0110001 //BASEADDR+0xC4 bootsel
`define REG_CLKSEL      7'b0110010 //BASEADDR+0xC8 clocksel

`define REG_CLK_DIV_CLU           7'b0110110 //BASEADDR+0xD8
`define SEL_CLK_DC_FIFO_EFPGA     7'b0111000 //BASEADDR+0xE0
`define CLK_GATING_DC_FIFO_EFPGA  7'b0111001 //BASEADDR+0xE4
`define RESET_TYPE1_EFPGA         7'b0111010 //BASEADDR+0xE8
`define ENABLE_IN_OUT_EFPGA       7'b0111011 //BASEADDR+0xEC

`define REG_CLUSTER_CTRL 7'b0011100 //BASEADDR+0x70 CLUSTER Ctrl
`define REG_CTRL_PER     7'b0011110
`define REG_CLUSTER_IRQ  7'b0011111

`define REG_CLUSTER_BOOT_ADDR0 7'b0100000
`define REG_CLUSTER_BOOT_ADDR1 7'b0100001
// NOTE: safe regs will be mapped starting from BASEADDR+0x100

//`define MSG_VERBOSE

module apb_soc_ctrl #(
    parameter int unsigned APB_ADDR_WIDTH = 12,  // APB slaves are 4KB by default
    parameter int unsigned NB_CLUSTERS    = 0,   // N_CLUSTERS
    parameter int unsigned NB_CORES       = 4,   // N_CORES
    parameter int unsigned JTAG_REG_SIZE  = 8,
    parameter int unsigned NBIT_PADCFG    = 6
) (
    input  logic                      HCLK,
    input  logic                      HRESETn,
    input  logic [APB_ADDR_WIDTH-1:0] PADDR,
    input  logic               [31:0] PWDATA,
    input  logic                      PWRITE,
    input  logic                      PSEL,
    input  logic                      PENABLE,
    output logic               [31:0] PRDATA,
    output logic                      PREADY,
    output logic                      PSLVERR,

    input  logic                      sel_fll_clk_i,
    input  logic                      boot_l2_i,
    input  logic                      bootsel_i,
    input  logic                      fc_fetch_en_valid_i,
    input  logic                      fc_fetch_en_i,

    output logic         [63:0] [NBIT_PADCFG-1:0] pad_cfg_o,
    output logic         [63:0] [1:0] pad_mux_o,

    input  logic                [JTAG_REG_SIZE-1:0] soc_jtag_reg_i,
    output logic                [JTAG_REG_SIZE-1:0] soc_jtag_reg_o,

    output logic               [31:0] fc_bootaddr_o,

    output logic                [2:0] sel_clk_dc_fifo_efpga_o,
    output logic                      clk_gating_dc_fifo_o,
    output logic                [3:0] reset_type1_efpga_o,
    output logic                      enable_udma_efpga_o,
    output logic                      enable_events_efpga_o,
    output logic                      enable_apb_efpga_o,
    output logic                      enable_tcdm3_efpga_o,
    output logic                      enable_tcdm2_efpga_o,
    output logic                      enable_tcdm1_efpga_o,
    output logic                      enable_tcdm0_efpga_o,                                                          
    output logic                      fc_fetchen_o,
    output logic                      sel_hyper_axi_o,
    output logic                      cluster_pow_o, // power cluster
    output logic                      cluster_byp_o, // bypass cluster
    output logic               [63:0] cluster_boot_addr_o,
    output logic                      cluster_fetch_enable_o,
    output logic                      cluster_rstn_o,
    output logic                      cluster_irq_o
    );

   logic     [31:0] r_pwr_reg;
   logic     [31:0] r_corestatus;


   logic      [6:0] s_apb_addr;

   logic     [15:0] n_cores;
   logic     [15:0] n_clusters;

   logic     [63:0] r_pad_fun0;
   logic     [63:0] r_pad_fun1;

   logic     [63:0] r_cluster_boot;
   logic            r_cluster_fetch_enable;
   logic            r_cluster_rstn;

   logic      [JTAG_REG_SIZE-1:0] r_jtag_rego;
   logic      [JTAG_REG_SIZE-1:0] r_jtag_regi_sync[1:0];

   logic            r_cluster_byp;
   logic            r_cluster_pow;
   logic     [31:0] r_bootaddr;
   logic            r_fetchen;

   logic            r_cluster_irq;

   logic            r_sel_hyper_axi;
   logic      [1:0] r_bootsel;
   
   logic [7:0]      r_clk_div_cluster;
   logic            s_div_cluster_valid, s_div_cluster_sel;

   logic [5:0]      r_sel_clk_dc_fifo_onehot;
   logic            r_clk_gating_dc_fifo;
   logic [3:0]      r_reset_type1_efpga;
   logic [5:0]      r_enable_inout_efpga;
                                          
   logic s_apb_write;

   // sanity check on NBIT_PADCFG
   if (NBIT_PADCFG > 8 || NBIT_PADCFG < 3)
       $error("apb_soc_ctrl NBIT_PADCFG out of range. This won't work.");

   assign soc_jtag_reg_o = r_jtag_rego;

   assign fc_bootaddr_o = r_bootaddr;
   assign fc_fetchen_o  = r_fetchen;

   assign cluster_pow_o = r_cluster_pow;
   assign sel_hyper_axi_o = r_sel_hyper_axi;

   assign s_apb_write = PSEL && PENABLE && PWRITE;

   assign cluster_rstn_o = r_cluster_rstn;

   assign cluster_boot_addr_o = r_cluster_boot;
   assign cluster_fetch_enable_o = r_cluster_fetch_enable;
   assign cluster_byp_o = r_cluster_byp;
   assign cluster_irq_o = r_cluster_irq;

    assign clk_div_cluster_data_o = r_clk_div_cluster;

   edge_propagator_tx i_edgeprop_clu
   (
     .clk_i(HCLK),
     .rstn_i(HRESETn),
     .valid_i(s_div_cluster_valid),
     .ack_i(clk_div_cluster_ack_i),
     .valid_o(clk_div_cluster_valid_o)
   );

   assign s_div_cluster_valid = s_apb_write & s_div_cluster_sel;
   assign s_div_cluster_sel   = s_apb_addr == `REG_CLK_DIV_CLU;

  always_comb begin : proc_id
    sel_clk_dc_fifo_efpga_o = '0;
    for(int unsigned i=0;i<6;i++)
    begin
      if (r_sel_clk_dc_fifo_onehot[i])
        sel_clk_dc_fifo_efpga_o = i;
    end
  end

  assign clk_gating_dc_fifo_o = r_clk_gating_dc_fifo;

  assign reset_type1_efpga_o  = r_reset_type1_efpga;

  assign {enable_udma_efpga_o, enable_events_efpga_o, enable_apb_efpga_o, enable_tcdm3_efpga_o, enable_tcdm2_efpga_o, enable_tcdm1_efpga_o, enable_tcdm0_efpga_o} = r_enable_inout_efpga[5:0];
                                                  
   always_comb begin
     for (int i=0;i<64;i++)
     begin
       pad_mux_o[i][0]  = r_pad_fun0[i];
       pad_mux_o[i][1]  = r_pad_fun1[i];
     end
   end


   assign s_apb_addr = PADDR[8:2];

    always_ff @(posedge HCLK, negedge HRESETn)
    begin
      if(~HRESETn) begin
        r_corestatus           <= '0;
        r_pwr_reg              <= '0;
        r_pad_fun0             <= '0;
        r_pad_fun1             <= '0;
        r_jtag_regi_sync[0]    <= 'h0;
        r_jtag_regi_sync[1]    <= 'h0;
        r_jtag_rego            <= 'h0;
        r_bootaddr             <= 32'h1A000080;
        r_fetchen              <= 1'h0; // on reset, fc doesn't do anything
        r_cluster_pow          <= 1'b0;
        r_cluster_byp          <= 1'b1;
        //pad_cfg                <= '{default: 6'b111111};
        pad_cfg_o              <= '1;
        r_sel_hyper_axi        <= 1'b0;
        r_cluster_fetch_enable <= 1'b0;
        r_cluster_boot         <= '0;
        r_cluster_rstn         <= 1'b1;
        r_cluster_irq          <= 1'b0;
        r_clk_div_cluster        <= 'h0;
        r_sel_clk_dc_fifo_onehot <= '0;
        r_clk_gating_dc_fifo     <= 1'b1;
        r_reset_type1_efpga      <= '0;
        r_enable_inout_efpga     <= '0;                                
      end
      else
      begin
        r_jtag_regi_sync[1] <= soc_jtag_reg_i;
        r_jtag_regi_sync[0] <= r_jtag_regi_sync[1];

        // allow fc fetch enable to be controlled through a signal
        if (fc_fetch_en_valid_i)
            r_fetchen <= fc_fetch_en_i;

        if (PSEL && PENABLE && PWRITE)
        begin
          case (s_apb_addr)
                `REG_FCBOOT:
                 begin
                   r_bootaddr <= PWDATA;
                 end
                `REG_FCFETCH:
                 begin
                   // allow fc fetch enable to be controlled through JTAG
                   r_fetchen <= PWDATA[0];
                 end
                `REG_PADFUN0:
                for (int i=0;i<16;i++)
                begin
                  r_pad_fun0[i] <= PWDATA[i*2];
                  r_pad_fun1[i] <= PWDATA[i*2+1];
                end
               `REG_PADFUN1:
                for (int i=0;i<16;i++)
                begin
                  r_pad_fun0[16+i] <= PWDATA[i*2];
                  r_pad_fun1[16+i] <= PWDATA[i*2+1];
                end
                `REG_PADFUN2:
                for (int i=0;i<16;i++)
                begin
                  r_pad_fun0[32+i] <= PWDATA[i*2];
                  r_pad_fun1[32+i] <= PWDATA[i*2+1];
                end
                `REG_PADFUN3:
                for (int i=0;i<16;i++)
                begin
                  r_pad_fun0[48+i] <= PWDATA[i*2];
                  r_pad_fun1[48+i] <= PWDATA[i*2+1];
                end
                `REG_PADCFG0:
                begin
                  pad_cfg_o[0]         <= PWDATA[0  +: NBIT_PADCFG-1]; //pad_rf_miso
                  pad_cfg_o[1]         <= PWDATA[8  +: NBIT_PADCFG-1]; //pad_rf_mosi
                  pad_cfg_o[2]         <= PWDATA[16 +: NBIT_PADCFG-1]; //pad_rf_cs
                  pad_cfg_o[3]         <= PWDATA[24 +: NBIT_PADCFG-1]; //pad_rf_sck
                end
                `REG_PADCFG1:
                begin
                  pad_cfg_o[4]         <= PWDATA[0  +: NBIT_PADCFG-1]; //pad_rf_pactrl0
                  pad_cfg_o[5]         <= PWDATA[8  +: NBIT_PADCFG-1]; //pad_rf_pactrl1
                  pad_cfg_o[6]         <= PWDATA[16 +: NBIT_PADCFG-1]; //pad_rf_pactrl2
                  pad_cfg_o[7]         <= PWDATA[24 +: NBIT_PADCFG-1]; //pad_rf_pactrl3
                end
                `REG_PADCFG2:
                begin
                  pad_cfg_o[8]         <= PWDATA[0  +: NBIT_PADCFG-1]; //pad_cam_pclk
                  pad_cfg_o[9]         <= PWDATA[8  +: NBIT_PADCFG-1]; //pad_cam_valid
                  pad_cfg_o[10]        <= PWDATA[16 +: NBIT_PADCFG-1]; //pad_cam_data0
                  pad_cfg_o[11]        <= PWDATA[24 +: NBIT_PADCFG-1]; //pad_cam_data1
                end
                `REG_PADCFG3:
                begin
                  pad_cfg_o[12]        <= PWDATA[0  +: NBIT_PADCFG-1]; //pad_cam_data2
                  pad_cfg_o[13]        <= PWDATA[8  +: NBIT_PADCFG-1]; //pad_cam_data3
                  pad_cfg_o[14]        <= PWDATA[16 +: NBIT_PADCFG-1]; //pad_cam_data4
                  pad_cfg_o[15]        <= PWDATA[24 +: NBIT_PADCFG-1]; //pad_cam_data5
                end
                `REG_PADCFG4:
                begin
                  pad_cfg_o[16]        <= PWDATA[0  +: NBIT_PADCFG-1]; //pad_cam_data6
                  pad_cfg_o[17]        <= PWDATA[8  +: NBIT_PADCFG-1]; //pad_cam_data7
                  pad_cfg_o[18]        <= PWDATA[16 +: NBIT_PADCFG-1]; //pad_cam_hsync
                  pad_cfg_o[19]        <= PWDATA[24 +: NBIT_PADCFG-1]; //pad_cam_vsync
                end
                `REG_PADCFG5:
                begin
                  pad_cfg_o[20]        <= PWDATA[0  +: NBIT_PADCFG-1]; //pad_cam_miso
                  pad_cfg_o[21]        <= PWDATA[8  +: NBIT_PADCFG-1]; //pad_cam_mosi
                  pad_cfg_o[22]        <= PWDATA[16 +: NBIT_PADCFG-1]; //pad_cam_cs
                  pad_cfg_o[23]        <= PWDATA[24 +: NBIT_PADCFG-1]; //pad_cam_sck
                end
                `REG_PADCFG6:
                begin
                  pad_cfg_o[24]        <= PWDATA[0  +: NBIT_PADCFG-1]; //pad_i2c0_sda
                  pad_cfg_o[25]        <= PWDATA[8  +: NBIT_PADCFG-1]; //pad_i2c0_scl
                  pad_cfg_o[26]        <= PWDATA[16 +: NBIT_PADCFG-1]; //pad_i2c1_sda
                  pad_cfg_o[27]        <= PWDATA[24 +: NBIT_PADCFG-1]; //pad_i2c1_scl
                end
                `REG_PADCFG7:
                begin
                  pad_cfg_o[28]        <= PWDATA[0  +: NBIT_PADCFG-1]; //pad_timer0_ch0
                  pad_cfg_o[29]        <= PWDATA[8  +: NBIT_PADCFG-1]; //pad_timer0_ch1
                  pad_cfg_o[30]        <= PWDATA[16 +: NBIT_PADCFG-1]; //pad_timer0_ch2
                  pad_cfg_o[31]        <= PWDATA[24 +: NBIT_PADCFG-1]; //pad_timer0_ch3
                end
                `REG_PADCFG8:
                begin
                  pad_cfg_o[32]        <= PWDATA[0  +: NBIT_PADCFG-1]; //pad_i2s0_sck
                  pad_cfg_o[33]        <= PWDATA[8  +: NBIT_PADCFG-1]; //pad_i2s0_ws
                  pad_cfg_o[34]        <= PWDATA[16 +: NBIT_PADCFG-1]; //pad_i2s0_sdi
                  pad_cfg_o[35]        <= PWDATA[24 +: NBIT_PADCFG-1]; //pad_i2s1_sck
                end
                `REG_PADCFG9:
                begin
                  pad_cfg_o[36]        <= PWDATA[0  +: NBIT_PADCFG-1]; //pad_i2s1_ws
                  pad_cfg_o[37]        <= PWDATA[8  +: NBIT_PADCFG-1]; //pad_i2s1_sdi
                  pad_cfg_o[38]        <= PWDATA[16 +: NBIT_PADCFG-1]; //pad_uart_rx
                  pad_cfg_o[39]        <= PWDATA[24 +: NBIT_PADCFG-1]; //pad_uart_tx
                end
                `REG_PADCFG10:
                begin
                  pad_cfg_o[40]        <= PWDATA[0   +: NBIT_PADCFG-1]; //pad_spim_sdio0
                  pad_cfg_o[41]        <= PWDATA[8   +: NBIT_PADCFG-1]; //pad_spim_sdio1
                  pad_cfg_o[42]        <= PWDATA[16  +: NBIT_PADCFG-1]; //pad_spim_sdio2
                  pad_cfg_o[43]        <= PWDATA[24  +: NBIT_PADCFG-1]; //pad_spim_sdio3
                end
                `REG_PADCFG11:
                begin
                  pad_cfg_o[44]        <= PWDATA[0  +: NBIT_PADCFG-1]; //pad_spim_csn0
                  pad_cfg_o[45]        <= PWDATA[8  +: NBIT_PADCFG-1]; //pad_spim_csn1
                  pad_cfg_o[46]        <= PWDATA[16 +: NBIT_PADCFG-1]; //pad_spim_sck
                  pad_cfg_o[47]        <= PWDATA[24 +: NBIT_PADCFG-1]; //
                end
                `REG_PADCFG12:
                begin
                  pad_cfg_o[48]        <= PWDATA[0  +: NBIT_PADCFG-1]; //
                  pad_cfg_o[49]        <= PWDATA[8  +: NBIT_PADCFG-1]; //
                  pad_cfg_o[50]        <= PWDATA[16 +: NBIT_PADCFG-1]; //
                  pad_cfg_o[51]        <= PWDATA[24 +: NBIT_PADCFG-1]; //
                end
                `REG_PADCFG13:
                begin
                  pad_cfg_o[52]        <= PWDATA[0   +: NBIT_PADCFG-1]; //
                  pad_cfg_o[53]        <= PWDATA[8   +: NBIT_PADCFG-1]; //
                  pad_cfg_o[54]        <= PWDATA[16  +: NBIT_PADCFG-1]; //
                  pad_cfg_o[55]        <= PWDATA[24  +: NBIT_PADCFG-1]; //
                end
                `REG_PADCFG14:
                begin
                  pad_cfg_o[56]        <= PWDATA[0  +: NBIT_PADCFG-1]; //
                  pad_cfg_o[57]        <= PWDATA[8  +: NBIT_PADCFG-1]; //
                  pad_cfg_o[58]        <= PWDATA[16 +: NBIT_PADCFG-1]; //
                  pad_cfg_o[59]        <= PWDATA[24 +: NBIT_PADCFG-1]; //
                end
                `REG_PADCFG15:
                begin
                  pad_cfg_o[60]        <= PWDATA[0  +: NBIT_PADCFG-1]; //
                  pad_cfg_o[61]        <= PWDATA[8  +: NBIT_PADCFG-1]; //
                  pad_cfg_o[62]        <= PWDATA[16 +: NBIT_PADCFG-1]; //
                  pad_cfg_o[63]        <= PWDATA[24 +: NBIT_PADCFG-1]; //
                end
                `REG_JTAGREG:
                begin
                  r_jtag_rego   <= PWDATA[JTAG_REG_SIZE-1:0];
                end
                `REG_CORESTATUS:
                begin
                  r_corestatus  <= PWDATA[31:0];
                end
                `REG_CLUSTER_CTRL:
                begin
                  r_cluster_byp          <= PWDATA[0];
                  r_cluster_pow          <= PWDATA[1];
                  r_cluster_fetch_enable <= PWDATA[2];
                  r_cluster_rstn         <= PWDATA[3];
                end
                `REG_CTRL_PER: begin
`ifdef HYPER_RAM
                    r_sel_hyper_axi <= PWDATA[0];
`endif
                end
                `REG_CLUSTER_IRQ:
                    r_cluster_irq <= PWDATA[0];
                `REG_CLUSTER_BOOT_ADDR0:
                    r_cluster_boot[31:0] <= PWDATA;
                `REG_CLUSTER_BOOT_ADDR1:
                    r_cluster_boot[63:32] <= PWDATA;
                `REG_CLK_DIV_CLU:
                    r_clk_div_cluster  <= PWDATA[7:0];
                `SEL_CLK_DC_FIFO_EFPGA:
                    r_sel_clk_dc_fifo_onehot <= PWDATA[5:0];
                `CLK_GATING_DC_FIFO_EFPGA:
                    r_clk_gating_dc_fifo     <= PWDATA[0];
                `RESET_TYPE1_EFPGA:
                    r_reset_type1_efpga      <= PWDATA[3:0];
                `ENABLE_IN_OUT_EFPGA:
                    //0: TCDM0
                    //1: TCDM1
                    //2: TCDM2
                    //3: TCDM3:
                    //4: APB
                    //5: EVENTS
                    //6: uDMA
                    r_enable_inout_efpga     <= PWDATA[5:0];
                
                default: begin
                `ifndef SYNTHESIS
                  `ifdef MSG_VERBOSE
                  $display("[APB SOC CTRL] INVALID WRITE ACCESS to %x at time %t\n",PADDR, $time);
                  `endif
                `endif
                end
          endcase
        end
      end
    end

    // read data
    always_comb
    begin
        PRDATA = '0;
        case (s_apb_addr)
          `REG_PADFUN0:
              for (int i=0;i<16;i++)
              begin
                PRDATA[i*2]   = r_pad_fun0[i];
                PRDATA[i*2+1] = r_pad_fun1[i];
              end
          `REG_PADFUN1:
              for (int i=0;i<16;i++)
              begin
                PRDATA[i*2]   = r_pad_fun0[16+i];
                PRDATA[i*2+1] = r_pad_fun1[16+i];
              end
          `REG_PADFUN2:
              for (int i=0;i<16;i++)
              begin
                PRDATA[i*2]   = r_pad_fun0[32+i];
                PRDATA[i*2+1] = r_pad_fun1[32+i];
              end
          `REG_PADFUN3:
              for (int i=0;i<16;i++)
              begin
                PRDATA[i*2]   = r_pad_fun0[48+i];
                PRDATA[i*2+1] = r_pad_fun1[48+i];
              end
          `REG_PADCFG0:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i];
              end
          `REG_PADCFG1:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+4];
              end
          `REG_PADCFG2:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+8];
              end
          `REG_PADCFG3:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+12];
              end
          `REG_PADCFG4:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+16];
              end
          `REG_PADCFG5:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+20];
              end
          `REG_PADCFG6:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+24];
              end
          `REG_PADCFG7:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+28];
              end
          `REG_PADCFG8:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+32];
              end
          `REG_PADCFG9:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+36];
              end
          `REG_PADCFG10:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+40];
              end
          `REG_PADCFG11:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+44];
              end
          `REG_PADCFG12:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+48];
              end
          `REG_PADCFG13:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+52];
              end
          `REG_PADCFG14:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+56];
              end
          `REG_PADCFG15:
              for (int i=0; i<4; i++) begin
                  PRDATA[i*8 +: NBIT_PADCFG-1] = pad_cfg_o[i+60];
              end
          `REG_FCBOOT:
            PRDATA = r_bootaddr;
          `REG_INFO:
            PRDATA = {n_cores,n_clusters};
          `REG_CORESTATUS:
            PRDATA = r_corestatus;
          `REG_CS_RO:
            PRDATA = r_corestatus;
          `REG_BOOTSEL:
            PRDATA =  {30'h0, r_bootsel};
          `REG_CLKSEL:
            PRDATA = {31'h0,sel_fll_clk_i};
          `REG_CLUSTER_CTRL:
            PRDATA = {
              29'h0,
              r_cluster_rstn,
              r_cluster_fetch_enable,
              r_cluster_pow,
              r_cluster_byp };
          `REG_JTAGREG:
            PRDATA = {16'h0,r_jtag_regi_sync[0],r_jtag_rego};
          `REG_CTRL_PER:
            PRDATA = {31'b0, r_sel_hyper_axi};
          `REG_CLUSTER_IRQ:
            PRDATA = {31'b0, r_cluster_irq};
          `REG_CLUSTER_BOOT_ADDR0:
            PRDATA = r_cluster_boot[31:0];
          `REG_CLUSTER_BOOT_ADDR1:
            PRDATA = r_cluster_boot[63:32];
          `REG_CLK_DIV_CLU:
            PRDATA = {24'h0,r_clk_div_cluster};
          `SEL_CLK_DC_FIFO_EFPGA:
            PRDATA = {26'h0, r_sel_clk_dc_fifo_onehot};
          `CLK_GATING_DC_FIFO_EFPGA:
            PRDATA = {31'b0, r_clk_gating_dc_fifo};
          `RESET_TYPE1_EFPGA:
            PRDATA = {28'b0, r_reset_type1_efpga};
          `ENABLE_IN_OUT_EFPGA:
            PRDATA = {26'b0, r_enable_inout_efpga};                 
          default:
            begin
            PRDATA = 'h0;
            `ifndef SYNTHESIS
              `ifdef MSG_VERBOSE
              $display("[APB SOC CTRL] INVALID READ ACCESS to %x at time %t\n",PADDR, $time);
              `endif
            `endif
            end
        endcase
    end

   always_ff @(posedge HCLK, negedge HRESETn)
    begin
      if(~HRESETn) begin
        r_bootsel <= 2'b00;
      end
      else
      begin
        r_bootsel <= {r_bootsel[0],bootsel_i};
      end
    end


   assign n_cores    = NB_CORES;
   assign n_clusters = NB_CLUSTERS;

   assign PREADY     = 1'b1;
   assign PSLVERR    = 1'b0;

endmodule
