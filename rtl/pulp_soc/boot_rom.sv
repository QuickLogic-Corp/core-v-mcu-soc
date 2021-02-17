// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.


`include "soc_mem_map.svh"

module boot_rom #(
    parameter ROM_ADDR_WIDTH = 13
    )
    (
     input logic             clk_i,
     input logic             rst_ni,
     input logic             init_ni,
     XBAR_TCDM_BUS.Slave     mem_slave,
     input logic             test_mode_i
    );

    //Perform TCDM handshaking for constant 1 cycle latency
    assign mem_slave.gnt     = mem_slave.req;
    always_ff @(posedge clk_i, negedge rst_ni) begin
        if (!rst_ni) begin
            mem_slave.r_valid <= 1'b0;
        end else begin
            mem_slave.r_valid <= mem_slave.req;
        end
    end

    //Remove address offset
    logic [31:0] address;
    assign address = mem_slave.add - `SOC_MEM_MAP_BOOT_ROM_START_ADDR;

    `ifndef PULP_FPGA_EMUL

        //generic_rom #(
        //    .ADDR_WIDTH(ROM_ADDR_WIDTH-2), //The ROM uses 32-bit word addressing while the bus addresses bytes
        //    .DATA_WIDTH(32)
        // ) rom_mem_i (
        //    .CLK            (  clk_i                ),
        //    .CEN            (  ~mem_slave.req        ),
        //    .A              (  address[ROM_ADDR_WIDTH-1:2]  ), //Cutoff insignificant address bits. The
        //                                                             //interconnect makes sure we only receive addresses in the bootrom address space
        //    .Q              (  mem_slave.r_rdata      )
        //);
        
        IN22FDX_ROMI_FRG_W02048B032M32C064_boot_code
        `ifndef SYNTHESIS
        #(
        .ROMDATA_FILE_NAME  ("./boot/boot_code.cde" ),
        .BINARY_FILE        ( 1                     )
         )
        `endif
        rom_mem_i (
            .CLK            (  clk_i                ),
            .CEN            (  ~mem_slave.req       ),
            .POWERGATE      (  1'b0                 ),
            .AS             (  address[7]     		),
            .AW             (  address[ROM_ADDR_WIDTH-1:8]  ),
            .AC             (  address[6:2] 	  	),
            .T_BIST         ( 1'b0                  ),
            .T_LOGIC        ( 1'b0                  ),
            .T_SCAN         ( 1'b0                  ),
            .T_SI           ( 1'b0                  ),
            .T_CEN          ( 1'b1                  ),
            .T_AS           ( 1'b0                  ),
            .T_AW           ( '0                    ),
            .T_AC           ( '0                    ),
            .T_POWERGATE    ( 1'b0                  ),
            .T_WBT          ( 1'b0                  ),
            .MA_SAWL        ( '0                    ),
            .MA_WL          ( 1'b0                  ),
            .Q              (  mem_slave.r_rdata     ),
            .T_SO           (                       )
        );

        // assign mem_slave.add[31:ROM_ADDR_WIDTH] = '0;

    `else // !`ifndef PULP_FPGA_EMUL

    fpga_bootrom #(
                   .ADDR_WIDTH(ROM_ADDR_WIDTH-2), //The ROM uses 32-bit word addressing while the bus addresses bytes
                   .DATA_WIDTH(32)
                   ) rom_mem_i (
                            .CLK(clk_i),
                            .CEN(~mem_slave.req),
                            .A(address[ROM_ADDR_WIDTH-1:2]), //Cutoff insignificant address bits. The interconnect
                                                                   //makes sure we only receive addresses in the bootrom address space
                            .Q(mem_slave.r_rdata)
                            );

    `endif

endmodule
