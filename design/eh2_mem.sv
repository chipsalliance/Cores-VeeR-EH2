//********************************************************************************
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

module eh2_mem
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
(
   input logic         clk,
   input logic         rst_l,
   input logic         dccm_clk_override,
   input logic         icm_clk_override,
   input logic         dec_tlu_core_ecc_disable,
   input logic         btb_clk_override,

   //DCCM ports
   input logic         dccm_wren,
   input logic         dccm_rden,
   input logic [pt.DCCM_BITS-1:0]  dccm_wr_addr_lo,
   input logic [pt.DCCM_BITS-1:0]  dccm_wr_addr_hi,
   input logic [pt.DCCM_BITS-1:0]  dccm_rd_addr_lo,
   input logic [pt.DCCM_BITS-1:0]  dccm_rd_addr_hi,
   input logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_wr_data_lo,
   input logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_wr_data_hi,

   output logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_rd_data_lo,
   output logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_rd_data_hi,

   input eh2_dccm_ext_in_pkt_t  [pt.DCCM_NUM_BANKS-1:0] dccm_ext_in_pkt,

   //ICCM ports
   input eh2_ccm_ext_in_pkt_t   [pt.ICCM_NUM_BANKS/4-1:0][1:0][1:0]  iccm_ext_in_pkt,

   input logic [pt.ICCM_BITS-1:1]  iccm_rw_addr,
   input logic [pt.NUM_THREADS-1:0]iccm_buf_correct_ecc_thr,            // ICCM is doing a single bit error correct cycle
   input logic                     iccm_correction_state,               // We are under a correction - This is needed to guard replacements when hit
   input logic                     iccm_stop_fetch,                     // Squash any lru updates on the red hits as we have fetched ahead
   input logic                     iccm_corr_scnd_fetch,                // dont match on middle bank when under correction


   input logic         ifc_select_tid_f1,
   input logic         iccm_wren,
   input logic         iccm_rden,
   input logic [2:0]   iccm_wr_size,
   input logic [77:0]  iccm_wr_data,

   output logic [63:0]  iccm_rd_data,
   output logic [116:0] iccm_rd_data_ecc,
   // Icache and Itag Ports
   input  logic [31:1]  ic_rw_addr,
   input  logic [pt.ICACHE_NUM_WAYS-1:0]   ic_tag_valid,
   input  logic [pt.ICACHE_NUM_WAYS-1:0]          ic_wr_en  ,         // Which way to write
   input  logic         ic_rd_en,
   input  logic [63:0]  ic_premux_data,     // Premux data to be muxed with each way of the Icache.
   input  logic         ic_sel_premux_data, // Premux data sel

   input eh2_ic_data_ext_in_pkt_t   [pt.ICACHE_NUM_WAYS-1:0][pt.ICACHE_BANKS_WAY-1:0]         ic_data_ext_in_pkt,
   input eh2_ic_tag_ext_in_pkt_t    [pt.ICACHE_NUM_WAYS-1:0]              ic_tag_ext_in_pkt,

   input logic [pt.ICACHE_BANKS_WAY-1:0] [70:0]               ic_wr_data,           // Data to fill to the Icache. With ECC
   output logic [63:0]               ic_rd_data ,          // Data read from Icache. 2x64bits + parity bits. F2 stage. With ECC
   output logic [70:0]               ic_debug_rd_data ,    // Data read from Icache. 2x64bits + parity bits. F2 stage. With ECC
   output logic [25:0]               ictag_debug_rd_data,  // Debug icache tag.
   input  logic [70:0]               ic_debug_wr_data,     // Debug wr cache.


   input logic [pt.ICACHE_INDEX_HI:3]           ic_debug_addr,      // Read/Write addresss to the Icache.
   input  logic                                 ic_debug_rd_en,     // Icache debug rd
   input  logic                                 ic_debug_wr_en,     // Icache debug wr
   input  logic                                 ic_debug_tag_array, // Debug tag array
   input  logic [pt.ICACHE_NUM_WAYS-1:0]        ic_debug_way,       // Debug way. Rd or Wr.


   output  logic [pt.ICACHE_BANKS_WAY-1:0]       ic_eccerr,
   output  logic [pt.ICACHE_BANKS_WAY-1:0]       ic_parerr,


   output logic [pt.ICACHE_NUM_WAYS-1:0]   ic_rd_hit,
   output logic         ic_tag_perr,        // Icache Tag parity error

   // BTB ports
 input eh2_ccm_ext_in_pkt_t   [1:0] btb_ext_in_pkt,

 input logic                         btb_wren,
 input logic                         btb_rden,
 input logic [1:0] [pt.BTB_ADDR_HI:1] btb_rw_addr,  // per bank
 input logic [1:0] [pt.BTB_ADDR_HI:1] btb_rw_addr_f1,  // per bank
 input logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]         btb_sram_wr_data,
 input logic [1:0] [pt.BTB_BTAG_SIZE-1:0] btb_sram_rd_tag_f1,

 output eh2_btb_sram_pkt btb_sram_pkt,

 output logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank0_rd_data_f1,
 output logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank1_rd_data_f1,
 output logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank2_rd_data_f1,
 output logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank3_rd_data_f1,

 input  logic         scan_mode
);

   logic  active_clk;
   rvoclkhdr active_cg   ( .en(1'b1),         .l1clk(active_clk), .* );

   // DCCM Instantiation
   if (pt.DCCM_ENABLE == 1) begin: Gen_dccm_enable
      eh2_lsu_dccm_mem #(.pt(pt)) dccm (
         .clk_override(dccm_clk_override),
         .*
      );
   end else begin: Gen_dccm_disable
      assign dccm_rd_data_lo = '0;
      assign dccm_rd_data_hi = '0;
   end

if (pt.ICACHE_ENABLE == 1) begin : icache
   eh2_ifu_ic_mem #(.pt(pt)) icm  (
      .clk_override(icm_clk_override),
      .*
   );
end
else begin
   assign   ic_rd_hit[3:0] = '0;
   assign   ic_tag_perr    = '0 ;
   assign   ic_rd_data  = '0 ;
   assign   ictag_debug_rd_data  = '0 ;
end

if (pt.ICCM_ENABLE == 1) begin : iccm
   eh2_ifu_iccm_mem  #(.pt(pt)) iccm (.*,
                  .clk_override(icm_clk_override),
                  .iccm_rw_addr(iccm_rw_addr[pt.ICCM_BITS-1:1]),
                  .iccm_rd_data(iccm_rd_data[63:0])
                   );
end
else  begin
   assign iccm_rd_data     = '0 ;
   assign iccm_rd_data_ecc = '0 ;
end

// BTB sram
if (pt.BTB_USE_SRAM == 1) begin : btb
   eh2_ifu_btb_mem #(.pt(pt)) btb  (
      .clk_override(btb_clk_override),
      .*
   );
end


endmodule
