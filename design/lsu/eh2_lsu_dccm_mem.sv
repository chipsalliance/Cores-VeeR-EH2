// SPDX-License-Identifier: Apache-2.0
// Copyright 2020 Western Digital Corporation or its affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//********************************************************************************
// $Id$
//
//
// Owner:
// Function: DCCM for LSU pipe
// Comments: Single ported memory
//
//
// DC1 -> DC2 -> DC3 -> DC4 (Commit)
//
// //********************************************************************************


`define EH2_LOCAL_DCCM_RAM_TEST_PORTS    .TEST1(dccm_ext_in_pkt[i].TEST1),                      \
                                     .RME(dccm_ext_in_pkt[i].RME),                      \
                                     .RM(dccm_ext_in_pkt[i].RM),                        \
                                     .LS(dccm_ext_in_pkt[i].LS),                        \
                                     .DS(dccm_ext_in_pkt[i].DS),                        \
                                     .SD(dccm_ext_in_pkt[i].SD),                        \
                                     .TEST_RNM(dccm_ext_in_pkt[i].TEST_RNM),            \
                                     .BC1(dccm_ext_in_pkt[i].BC1),                      \
                                     .BC2(dccm_ext_in_pkt[i].BC2),                      \



module eh2_lsu_dccm_mem
import eh2_pkg::*;
#(
`include "eh2_param.vh"
 )(
   input logic         clk,                                             // clock
   input logic         active_clk,                                        // clock
   input logic         rst_l,
   input logic         clk_override,                                    // clock override

   input logic         dccm_wren,                                       // write enable
   input logic         dccm_rden,                                       // read enable
   input logic [pt.DCCM_BITS-1:0]  dccm_wr_addr_lo,                     // write address
   input logic [pt.DCCM_BITS-1:0]  dccm_wr_addr_hi,                     // write address
   input logic [pt.DCCM_BITS-1:0]  dccm_rd_addr_lo,                     // read address
   input logic [pt.DCCM_BITS-1:0]  dccm_rd_addr_hi,                     // read address for the upper bank in case of a misaligned access
   input logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_wr_data_lo,              // write data
   input logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_wr_data_hi,              // write data
   input eh2_dccm_ext_in_pkt_t  [pt.DCCM_NUM_BANKS-1:0] dccm_ext_in_pkt,    // the dccm packet from the soc

   output logic [pt.DCCM_FDATA_WIDTH-1:0] dccm_rd_data_lo,              // read data from the lo bank
   output logic [pt.DCCM_FDATA_WIDTH-1:0] dccm_rd_data_hi,              // read data from the hi bank

   input  logic         scan_mode
);

   localparam DCCM_INDEX_BITS = (pt.DCCM_BITS - pt.DCCM_BANK_BITS - pt.DCCM_ADDR_OFF);
   localparam DCCM_INDEX_DEPTH = (((pt.DCCM_SIZE)*1024)>>($clog2(pt.DCCM_BYTE_WIDTH)))>>$clog2((pt.DCCM_NUM_BANKS));  // Depth of memory bank


   logic [pt.DCCM_NUM_BANKS-1:0]                                        wren_bank;
   logic [pt.DCCM_NUM_BANKS-1:0]                                        rden_bank;
   logic [pt.DCCM_NUM_BANKS-1:0] [pt.DCCM_BITS-1:(pt.DCCM_BANK_BITS+2)] addr_bank;
   logic [pt.DCCM_BITS-1:(pt.DCCM_BANK_BITS+pt.DCCM_ADDR_OFF)]           rd_addr_even, rd_addr_odd;
   logic                                                                rd_unaligned, wr_unaligned;
   logic [pt.DCCM_NUM_BANKS-1:0] [pt.DCCM_FDATA_WIDTH-1:0]              dccm_bank_dout;
   logic [pt.DCCM_FDATA_WIDTH-1:0]                                      wrdata;

   logic [pt.DCCM_NUM_BANKS-1:0][pt.DCCM_FDATA_WIDTH-1:0]               wr_data_bank;

   logic [pt.DCCM_NUM_BANKS-1:0]                                        dccm_rd_data_bank_hi, dccm_rd_data_bank_lo;

   logic [(pt.DCCM_ADDR_OFF+pt.DCCM_BANK_BITS-1):pt.DCCM_ADDR_OFF]        dccm_rd_addr_lo_q;
   logic [(pt.DCCM_ADDR_OFF+pt.DCCM_BANK_BITS-1):pt.DCCM_ADDR_OFF]        dccm_rd_addr_hi_q;

   logic [pt.DCCM_NUM_BANKS-1:0]                                        dccm_clken;

   logic                                                                dccm_rd_addr_lo_q2;
   logic                                                                dccm_rd_addr_hi_q2;


   assign rd_unaligned = (dccm_rd_addr_lo[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS] != dccm_rd_addr_hi[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]);
   assign wr_unaligned = (dccm_wr_addr_lo[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS] != dccm_wr_addr_hi[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]);


   // 8 Banks, 16KB each (2048 x 72)
   for (genvar i=0; i<pt.DCCM_NUM_BANKS; i++) begin: mem_bank
      assign  wren_bank[i]        = dccm_wren & ((dccm_wr_addr_hi[2+:pt.DCCM_BANK_BITS] == i) | (dccm_wr_addr_lo[2+:pt.DCCM_BANK_BITS] == i));
      assign  rden_bank[i]        = dccm_rden & ((dccm_rd_addr_hi[2+:pt.DCCM_BANK_BITS] == i) | (dccm_rd_addr_lo[2+:pt.DCCM_BANK_BITS] == i));
      assign  addr_bank[i][(pt.DCCM_BANK_BITS+pt.DCCM_ADDR_OFF)+:DCCM_INDEX_BITS] = rden_bank[i] ? (((dccm_rd_addr_hi[2+:pt.DCCM_BANK_BITS] == i) & rd_unaligned) ?
                                                                                                        dccm_rd_addr_hi[(pt.DCCM_BANK_BITS+pt.DCCM_ADDR_OFF)+:DCCM_INDEX_BITS] :
                                                                                                        dccm_rd_addr_lo[(pt.DCCM_BANK_BITS+pt.DCCM_ADDR_OFF)+:DCCM_INDEX_BITS]) :
                                                                                                  (((dccm_wr_addr_hi[2+:pt.DCCM_BANK_BITS] == i) & wr_unaligned) ?
                                                                                                        dccm_wr_addr_hi[(pt.DCCM_BANK_BITS+pt.DCCM_ADDR_OFF)+:DCCM_INDEX_BITS] :
                                                                                                        dccm_wr_addr_lo[(pt.DCCM_BANK_BITS+pt.DCCM_ADDR_OFF)+:DCCM_INDEX_BITS]);


      assign wr_data_bank[i]     = ((dccm_wr_addr_hi[2+:pt.DCCM_BANK_BITS] == i) & wr_unaligned) ? dccm_wr_data_hi[pt.DCCM_FDATA_WIDTH-1:0] : dccm_wr_data_lo[pt.DCCM_FDATA_WIDTH-1:0];

      // clock gating section
      assign  dccm_clken[i] = (wren_bank[i] | rden_bank[i] | clk_override) ;
      // end clock gating section

`ifdef VERILATOR
        eh2_ram #(DCCM_INDEX_DEPTH,pt.DCCM_FDATA_WIDTH)  ram (
                                  // Primary ports
                                  .ME(dccm_clken[i]),
                                  .CLK(clk),
                                  .WE(wren_bank[i]),
                                  .ADR(addr_bank[i]),
                                  .D(wr_data_bank[i][pt.DCCM_FDATA_WIDTH-1:0]),
                                  .Q(dccm_bank_dout[i][pt.DCCM_FDATA_WIDTH-1:0]),
                                  .ROP ( ),
                                  // These are used by SoC
                                  `EH2_LOCAL_DCCM_RAM_TEST_PORTS
                                  .*
                                  );

`else
   `define DCCM_BANK (depth, width)                      \
      ram_``depth``x``width  dccm_bank (                 \
         // Primary ports                                \
         .ME(dccm_clken[i]),                             \
         .CLK(clk),                                      \
         .WE(wren_bank[i]),                              \
         .ADR(addr_bank[i]),                             \
         .D(wr_data_bank[i][pt.DCCM_FDATA_WIDTH-1:0]),   \
         .Q(dccm_bank_dout[i][pt.DCCM_FDATA_WIDTH-1:0]), \
         .ROP ( ),                                       \
         // These are used by SoC                        \
         `EH2_LOCAL_DCCM_RAM_TEST_PORTS                  \
         .*                                              \
         );

         `DCCM_BANK(DCCM_INDEX_DEPTH, `RV_DCCM_FDATA_WIDTH)
`endif // VERILATOR
   end : mem_bank

   // Flops
   rvdffs  #(pt.DCCM_BANK_BITS) rd_addr_lo_ff (.*, .din(dccm_rd_addr_lo[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]), .dout(dccm_rd_addr_lo_q[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]), .en(1'b1), .clk(active_clk));
   rvdffs  #(pt.DCCM_BANK_BITS) rd_addr_hi_ff (.*, .din(dccm_rd_addr_hi[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]), .dout(dccm_rd_addr_hi_q[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]), .en(1'b1), .clk(active_clk));

   // For Plus1 --> Read data comes out 2 cycle after dccm_rden since we need to flop the bank data and then mux between the banks
   if (pt.LOAD_TO_USE_PLUS1 == 1) begin: GenL2U_1
      logic                                                          dccm_rden_q;
      logic [pt.DCCM_NUM_BANKS-1:0] [pt.DCCM_FDATA_WIDTH-1:0]        dccm_bank_dout_q;
      logic [(pt.DCCM_ADDR_OFF+pt.DCCM_BANK_BITS-1):pt.DCCM_ADDR_OFF]  dccm_rd_addr_lo_q2;
      logic [(pt.DCCM_ADDR_OFF+pt.DCCM_BANK_BITS-1):pt.DCCM_ADDR_OFF]  dccm_rd_addr_hi_q2;

      // Mux out the read data
      assign dccm_rd_data_lo[pt.DCCM_FDATA_WIDTH-1:0]  = dccm_bank_dout_q[dccm_rd_addr_lo_q2[pt.pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]][pt.DCCM_FDATA_WIDTH-1:0];
      assign dccm_rd_data_hi[pt.DCCM_FDATA_WIDTH-1:0]  = dccm_bank_dout_q[dccm_rd_addr_hi_q2[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]][pt.DCCM_FDATA_WIDTH-1:0];

      for (genvar i=0; i<pt.DCCM_NUM_BANKS; i++) begin: GenBanks
         rvdffe #(pt.DCCM_FDATA_WIDTH) dccm_bank_dout_ff(.*, .din(dccm_bank_dout[i]), .dout(dccm_bank_dout_q[i]), .en(dccm_rden_q | clk_override));
      end

      rvdff  #(1)                  dccm_rden_ff  (.*, .din(dccm_rden), .dout(dccm_rden_q), .clk(active_clk));
      rvdffs  #(pt.DCCM_BANK_BITS) rd_addr_lo_ff (.*, .din(dccm_rd_addr_lo_q[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]), .dout(dccm_rd_addr_lo_q2[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]), .en(1'b1), .clk(active_clk));
      rvdffs  #(pt.DCCM_BANK_BITS) rd_addr_hi_ff (.*, .din(dccm_rd_addr_hi_q[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]), .dout(dccm_rd_addr_hi_q2[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]), .en(1'b1), .clk(active_clk));
   end else begin
      // mux out the read data
      assign dccm_rd_data_lo[pt.DCCM_FDATA_WIDTH-1:0]  = dccm_bank_dout[dccm_rd_addr_lo_q[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]][pt.DCCM_FDATA_WIDTH-1:0];
      assign dccm_rd_data_hi[pt.DCCM_FDATA_WIDTH-1:0]  = dccm_bank_dout[dccm_rd_addr_hi_q[pt.DCCM_ADDR_OFF+:pt.DCCM_BANK_BITS]][pt.DCCM_FDATA_WIDTH-1:0];

      assign dccm_rd_addr_lo_q2 = '0;
      assign dccm_rd_addr_hi_q2 = '0;
   end
`undef EH2_LOCAL_DCCM_RAM_TEST_PORTS

endmodule // eh2_lsu_dccm_mem
