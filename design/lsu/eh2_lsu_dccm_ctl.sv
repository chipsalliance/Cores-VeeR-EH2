// SPDX-License-Identifier: Apache-2.0
// Copyright 2020 Western Digital Corporation or it's affiliates.
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

module eh2_lsu_dccm_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (

   input logic                             clk,
   input logic                             lsu_free_c2_clk,
   input logic                             lsu_dccm_c1_dc3_clk,
   input logic                             lsu_c1_dc4_clk,
   input logic                             lsu_c1_dc5_clk,
   input logic                             lsu_c2_dc2_clk,
   input logic                             lsu_c2_dc3_clk,
   input logic                             lsu_pic_c1_dc3_clk,

   input logic                             rst_l,

   input                                   eh2_lsu_pkt_t lsu_pkt_dc5,     // lsu packets
   input                                   eh2_lsu_pkt_t lsu_pkt_dc4,     // lsu packets
   input                                   eh2_lsu_pkt_t lsu_pkt_dc3,     // lsu packets
   input                                   eh2_lsu_pkt_t lsu_pkt_dc2,     // lsu packets
   input                                   eh2_lsu_pkt_t lsu_pkt_dc1,
   input                                   eh2_lsu_pkt_t lsu_pkt_dc1_pre,
   input logic                             addr_in_dccm_region_dc1,     // address in dccm region
   input logic                             addr_in_dccm_dc1,          // address maps to dccm
   input logic                             addr_in_pic_dc1,           // address maps to pic
   input logic                             addr_in_pic_dc3,           // address maps to pic
   input logic                             addr_in_dccm_dc2, addr_in_dccm_dc3, addr_in_dccm_dc4, addr_in_dccm_dc5,
   input logic                             addr_in_pic_dc5,
   input logic                             lsu_raw_fwd_lo_dc5, lsu_raw_fwd_hi_dc5,

   input logic                             dma_pic_wen,
   input logic                             dma_dccm_wen,
   input logic                             dma_dccm_spec_wen,
   input logic                             dma_dccm_spec_req,
   input logic                             dma_mem_write,
   input logic [31:0]                      dma_mem_addr,
   input logic [63:0]                      dma_mem_wdata,
   input logic [2:0]                       dma_mem_tag_dc3,

   input logic [31:0]                      lsu_addr_dc1,              // starting byte address for loads
   input logic [31:0]                      lsu_addr_dc2,              // starting byte address for loads
   input logic [31:0]                      lsu_addr_dc3,              // starting byte address for loads
   input logic [31:0]                      lsu_addr_dc4,              // starting byte address for loads
   input logic [31:0]                      lsu_addr_dc5,              // starting byte address for loads

   input logic [31:0]                      end_addr_dc1,
   input logic [31:0]                      end_addr_dc2,
   input logic [31:0]                      end_addr_dc3,
   input logic [31:0]                      end_addr_dc4,
   input logic [31:0]                      end_addr_dc5,

   input logic                             stbuf_reqvld_any,          // write enable
   input logic [pt.LSU_SB_BITS-1:0]        stbuf_addr_any,            // stbuf address (aligned)

   input logic [pt.DCCM_DATA_WIDTH-1:0]   stbuf_data_any,            // the read out from stbuf
   input logic [pt.DCCM_DATA_WIDTH-1:0]   stbuf_fwddata_hi_dc3,      // stbuf fowarding to load
   input logic [pt.DCCM_DATA_WIDTH-1:0]   stbuf_fwddata_lo_dc3,      // stbuf fowarding to load
   input logic [pt.DCCM_BYTE_WIDTH-1:0]   stbuf_fwdbyteen_hi_dc3,    // stbuf fowarding to load
   input logic [pt.DCCM_BYTE_WIDTH-1:0]   stbuf_fwdbyteen_lo_dc3,    // stbuf fowarding to load
   input logic                            picm_fwd_en_dc2,
   input logic [31:0]                     picm_fwd_data_dc2,

   input logic                            lsu_commit_dc5,
   input logic                            lsu_sc_success_dc5,        // the store condition result
   input logic                            lsu_double_ecc_error_dc3,  // lsu has a DED
   input logic                            lsu_double_ecc_error_dc5,  // lsu has a DED
   input logic                            single_ecc_error_hi_dc3,   // Single bit ECC error on hi data
   input logic                            single_ecc_error_lo_dc3,   // Single bit ECC error on lo data
   input logic                            single_ecc_error_hi_dc4,   // Single bit ECC error on hi data
   input logic                            single_ecc_error_lo_dc4,   // Single bit ECC error on lo data
   input logic                            single_ecc_error_hi_dc5,   // Single bit ECC error on hi data
   input logic                            single_ecc_error_lo_dc5,   // Single bit ECC error on lo data
   input logic [pt.DCCM_DATA_WIDTH-1:0]   sec_data_hi_dc3,
   input logic [pt.DCCM_DATA_WIDTH-1:0]   sec_data_lo_dc3,

   input logic [pt.DCCM_DATA_WIDTH-1:0]   store_ecc_data_hi_dc3,   // store data
   input logic [pt.DCCM_DATA_WIDTH-1:0]   store_ecc_data_lo_dc3,   // store data
   input logic [31:0]                     amo_data_dc3,
   output logic [pt.DCCM_DATA_WIDTH-1:0]  dccm_data_hi_dc3,          // data from the dccm
   output logic [pt.DCCM_DATA_WIDTH-1:0]  dccm_data_lo_dc3,          // data from the dccm
   output logic [pt.DCCM_DATA_WIDTH-1:0]  dccm_datafn_hi_dc5,        // data from the dccm
   output logic [pt.DCCM_DATA_WIDTH-1:0]  dccm_datafn_lo_dc5,        // data from the dccm
   output logic [pt.DCCM_ECC_WIDTH-1:0]   dccm_data_ecc_hi_dc3,      // data from the dccm + ecc
   output logic [pt.DCCM_ECC_WIDTH-1:0]   dccm_data_ecc_lo_dc3,
   output logic [63:0]                    store_data_ext_dc3, store_data_ext_dc4, store_data_ext_dc5,   // goes to the stbuf/busbuf for load-store fwdding
   output logic                           disable_ecc_check_lo_dc3,
   output logic                           disable_ecc_check_hi_dc3,
   output logic                           ld_single_ecc_error_dc3,
   output logic                           ld_single_ecc_error_dc5,
   output logic                           ld_single_ecc_error_dc5_ff,
   output logic                           ld_single_ecc_error_lo_dc5_ff,
   output logic                           ld_single_ecc_error_hi_dc5_ff,

   output logic [pt.DCCM_DATA_WIDTH-1:0]  sec_data_hi_dc5,          // load single error corrected hi data
   output logic [pt.DCCM_DATA_WIDTH-1:0]  sec_data_lo_dc5,          // load single error corrected lo data

   output logic [pt.DCCM_DATA_WIDTH-1:0]  lsu_dccm_data_dc3,        // right justified, ie load byte will have data at 7:0
   output logic [pt.DCCM_DATA_WIDTH-1:0]  lsu_dccm_data_corr_dc3,   // right justified, ie load byte will have data at 7:0
   output logic [31:0]                    picm_mask_data_dc3,        // pic data to stbuf
   output logic [31:0]                    picm_rd_data_dc3,          // pic read data in dc3
   output logic                           lsu_stbuf_commit_any,      // stbuf wins the dccm port or is to pic
   output logic                           lsu_dccm_rden_dc3,         // dccm read

   output logic                           dccm_dma_rvalid,           // dccm serviving the dma load
   output logic                           dccm_dma_ecc_error,        // DMA load had ecc error
   output logic [2:0]                     dccm_dma_rtag,             // DCCM return tag
   output logic [63:0]                    dccm_dma_rdata,            // dccm data to dma request

   // DCCM ports
   output logic                           dccm_wren,                // dccm interface -- write
   output logic                           dccm_rden,                // dccm interface -- write
   output logic [pt.DCCM_BITS-1:0]        dccm_wr_addr_lo,          // dccm interface -- wr addr for lo bankd
   output logic [pt.DCCM_BITS-1:0]        dccm_wr_addr_hi,          // dccm interface -- wr addr for hi bankd
   output logic [pt.DCCM_BITS-1:0]        dccm_rd_addr_lo,          // dccm interface -- read address for lo bank
   output logic [pt.DCCM_BITS-1:0]        dccm_rd_addr_hi,          // dccm interface -- read address for hi bank

   input logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_rd_data_lo,          // dccm read data back from the dccm
   input logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_rd_data_hi,          // dccm read data back from the dccm

   // PIC ports
   output logic                            picm_wren,          // write to pic
   output logic                            picm_rden,          // read to pick
   output logic                            picm_mken,          // write to pic need a mask
   output logic                            picm_rd_thr,      // PICM read thread
   output logic [31:0]                     picm_rdaddr,        // address for pic access - shared between reads and write
   output logic [31:0]                     picm_wraddr,        // address for pic access - shared between reads and write
   output logic [31:0]                     picm_wr_data,       // write data
   input logic [31:0]                      picm_rd_data,       // read data

   input logic                             scan_mode           // scan mode
);

   localparam DCCM_WIDTH_BITS = $clog2(pt.DCCM_BYTE_WIDTH);

   logic                        lsu_dccm_rden_dc1, lsu_dccm_rden_dc2, disable_ecc_check_lo_dc2, disable_ecc_check_hi_dc2;
   logic                        lsu_dccm_wren_dc1, lsu_dccm_wren_spec_dc1;
   logic [pt.DCCM_DATA_WIDTH-1:0]  store_data_hi_dc4, store_data_lo_dc4, dccm_data_lo_dc4_in, dccm_data_hi_dc4_in, dccm_data_lo_dc5_in, dccm_data_hi_dc5_in, store_data_lo_dc5, store_data_hi_dc5;
   logic [63:0]  dccm_dout_dc3, dccm_corr_dout_dc3;
   logic [63:0]  stbuf_fwddata_dc3;
   logic [7:0]   stbuf_fwdbyteen_dc3;
   logic [63:0]  lsu_rdata_dc3, lsu_rdata_corr_dc3;
   logic [31:0]  picm_rd_data_dc2;
   logic [31:0]  picm_rd_dataQ;
   logic [63:32] lsu_dccm_data_dc3_nc, lsu_dccm_data_corr_dc3_nc;

   logic         dccm_wr_bypass_c1_c2_hi, dccm_wr_bypass_c1_c3_hi, dccm_wr_bypass_c1_c4_hi, dccm_wr_bypass_c1_c5_hi;
   logic         dccm_wr_bypass_c1_c2_lo, dccm_wr_bypass_c1_c3_lo, dccm_wr_bypass_c1_c4_lo, dccm_wr_bypass_c1_c5_lo;
   logic         ld_single_ecc_error_lo_dc5, ld_single_ecc_error_hi_dc5;
   logic         ld_single_ecc_error_lo_dc5_ns, ld_single_ecc_error_hi_dc5_ns;
   logic         ld_single_ecc_error_dc4;
   logic         lsu_double_ecc_error_dc5_ff;
   logic         lsu_stbuf_ecc_block;
   logic [pt.DCCM_BITS-1:0] ld_sec_addr_lo_dc5_ff, ld_sec_addr_hi_dc5_ff;

   logic [7:0]   ldst_byteen_dc2, ldst_byteen_dc3, ldst_byteen_dc4, ldst_byteen_dc5;
   logic [7:0]   ldst_byteen_ext_dc2, ldst_byteen_ext_dc3, ldst_byteen_ext_dc4, ldst_byteen_ext_dc5;
   logic [31:0]  store_data_hi_dc3, store_data_lo_dc3;

   logic         kill_ecc_corr_lo_dc5;
   logic         kill_ecc_corr_hi_dc5;

   //------------------------------------------------------------------------------------------------------------
   //----------------------------------------Logic starts here---------------------------------------------------
   //------------------------------------------------------------------------------------------------------------

   assign dccm_dma_rvalid      = lsu_pkt_dc3.valid & lsu_pkt_dc3.load & lsu_pkt_dc3.dma;
   assign dccm_dma_ecc_error   = lsu_double_ecc_error_dc3;
   assign dccm_dma_rtag[2:0]   = dma_mem_tag_dc3[2:0];
   assign dccm_dma_rdata[63:0] = addr_in_pic_dc3 ? {2{picm_rd_data_dc3[31:0]}} : lsu_rdata_corr_dc3[63:0];

   assign {lsu_dccm_data_dc3_nc[63:32], lsu_dccm_data_dc3[31:0]} = lsu_rdata_dc3[63:0] >> 8*lsu_addr_dc3[1:0];
   assign {lsu_dccm_data_corr_dc3_nc[63:32], lsu_dccm_data_corr_dc3[31:0]} = lsu_rdata_corr_dc3[63:0] >> 8*lsu_addr_dc3[1:0];

   assign dccm_dout_dc3[63:0]      = {dccm_data_hi_dc3[pt.DCCM_DATA_WIDTH-1:0], dccm_data_lo_dc3[pt.DCCM_DATA_WIDTH-1:0]};
   assign dccm_corr_dout_dc3[63:0] = {sec_data_hi_dc3[pt.DCCM_DATA_WIDTH-1:0], sec_data_lo_dc3[pt.DCCM_DATA_WIDTH-1:0]};
   assign stbuf_fwddata_dc3[63:0]  = {stbuf_fwddata_hi_dc3[pt.DCCM_DATA_WIDTH-1:0], stbuf_fwddata_lo_dc3[pt.DCCM_DATA_WIDTH-1:0]};
   assign stbuf_fwdbyteen_dc3[7:0] = {stbuf_fwdbyteen_hi_dc3[pt.DCCM_BYTE_WIDTH-1:0], stbuf_fwdbyteen_lo_dc3[pt.DCCM_BYTE_WIDTH-1:0]};

   for (genvar i=0; i<8; i++) begin: GenLoop
      assign lsu_rdata_dc3[(8*i)+7:8*i] = stbuf_fwdbyteen_dc3[i] ? stbuf_fwddata_dc3[(8*i)+7:8*i] : dccm_dout_dc3[(8*i)+7:8*i];
      assign lsu_rdata_corr_dc3[(8*i)+7:8*i] = stbuf_fwdbyteen_dc3[i] ? stbuf_fwddata_dc3[(8*i)+7:8*i] : dccm_corr_dout_dc3[(8*i)+7:8*i];
   end

   assign kill_ecc_corr_lo_dc5 = (((lsu_addr_dc1[pt.DCCM_BITS-1:2] == lsu_addr_dc5[pt.DCCM_BITS-1:2]) | (end_addr_dc1[pt.DCCM_BITS-1:2] == lsu_addr_dc5[pt.DCCM_BITS-1:2])) & lsu_pkt_dc1.valid & lsu_pkt_dc1.store & lsu_pkt_dc1.dma & addr_in_dccm_dc1) |
                                 (((lsu_addr_dc2[pt.DCCM_BITS-1:2] == lsu_addr_dc5[pt.DCCM_BITS-1:2]) | (end_addr_dc2[pt.DCCM_BITS-1:2] == lsu_addr_dc5[pt.DCCM_BITS-1:2])) & lsu_pkt_dc2.valid & lsu_pkt_dc2.store & lsu_pkt_dc2.dma & addr_in_dccm_dc2) |
                                 (((lsu_addr_dc3[pt.DCCM_BITS-1:2] == lsu_addr_dc5[pt.DCCM_BITS-1:2]) | (end_addr_dc3[pt.DCCM_BITS-1:2] == lsu_addr_dc5[pt.DCCM_BITS-1:2])) & lsu_pkt_dc3.valid & lsu_pkt_dc3.store & lsu_pkt_dc3.dma & addr_in_dccm_dc3) |
                                 (((lsu_addr_dc4[pt.DCCM_BITS-1:2] == lsu_addr_dc5[pt.DCCM_BITS-1:2]) | (end_addr_dc4[pt.DCCM_BITS-1:2] == lsu_addr_dc5[pt.DCCM_BITS-1:2])) & lsu_pkt_dc4.valid & lsu_pkt_dc4.store & lsu_pkt_dc4.dma & addr_in_dccm_dc4);

   assign kill_ecc_corr_hi_dc5 = (((lsu_addr_dc1[pt.DCCM_BITS-1:2] == end_addr_dc5[pt.DCCM_BITS-1:2]) | (end_addr_dc1[pt.DCCM_BITS-1:2] == end_addr_dc5[pt.DCCM_BITS-1:2])) & lsu_pkt_dc1.valid & lsu_pkt_dc1.store & lsu_pkt_dc1.dma & addr_in_dccm_dc1) |
                                 (((lsu_addr_dc2[pt.DCCM_BITS-1:2] == end_addr_dc5[pt.DCCM_BITS-1:2]) | (end_addr_dc2[pt.DCCM_BITS-1:2] == end_addr_dc5[pt.DCCM_BITS-1:2])) & lsu_pkt_dc2.valid & lsu_pkt_dc2.store & lsu_pkt_dc2.dma & addr_in_dccm_dc2) |
                                 (((lsu_addr_dc3[pt.DCCM_BITS-1:2] == end_addr_dc5[pt.DCCM_BITS-1:2]) | (end_addr_dc3[pt.DCCM_BITS-1:2] == end_addr_dc5[pt.DCCM_BITS-1:2])) & lsu_pkt_dc3.valid & lsu_pkt_dc3.store & lsu_pkt_dc3.dma & addr_in_dccm_dc3) |
                                 (((lsu_addr_dc4[pt.DCCM_BITS-1:2] == end_addr_dc5[pt.DCCM_BITS-1:2]) | (end_addr_dc4[pt.DCCM_BITS-1:2] == end_addr_dc5[pt.DCCM_BITS-1:2])) & lsu_pkt_dc4.valid & lsu_pkt_dc4.store & lsu_pkt_dc4.dma & addr_in_dccm_dc4);

   assign ld_single_ecc_error_lo_dc5 = (lsu_commit_dc5 | lsu_pkt_dc5.dma) & (lsu_pkt_dc5.load | lsu_pkt_dc5.lr) & single_ecc_error_lo_dc5 & ~lsu_raw_fwd_lo_dc5;
   assign ld_single_ecc_error_hi_dc5 = (lsu_commit_dc5 | lsu_pkt_dc5.dma) & (lsu_pkt_dc5.load | lsu_pkt_dc5.lr) & single_ecc_error_hi_dc5 & ~lsu_raw_fwd_hi_dc5;
   assign ld_single_ecc_error_dc3    = (lsu_pkt_dc3.load | lsu_pkt_dc3.lr) & (single_ecc_error_lo_dc3 | single_ecc_error_hi_dc3);  // This is for blocking fast interrupt at decode-1
   assign ld_single_ecc_error_dc4    = (lsu_pkt_dc4.load | lsu_pkt_dc4.lr) & (single_ecc_error_lo_dc4 | single_ecc_error_hi_dc4);  // This is for blocking load/store/dma at decode
   assign ld_single_ecc_error_dc5    = (ld_single_ecc_error_lo_dc5 | ld_single_ecc_error_hi_dc5) & ~lsu_double_ecc_error_dc5;    // This doesn't have kill_ecc due to spyglass
   assign ld_single_ecc_error_lo_dc5_ns = ld_single_ecc_error_lo_dc5 & ~kill_ecc_corr_lo_dc5;
   assign ld_single_ecc_error_hi_dc5_ns = ld_single_ecc_error_hi_dc5 & ~kill_ecc_corr_hi_dc5;

   assign ld_single_ecc_error_dc5_ff = (ld_single_ecc_error_lo_dc5_ff | ld_single_ecc_error_hi_dc5_ff) & ~lsu_double_ecc_error_dc5_ff;

   assign sec_data_hi_dc5[pt.DCCM_DATA_WIDTH-1:0] = store_data_hi_dc5[pt.DCCM_DATA_WIDTH-1:0];
   assign sec_data_lo_dc5[pt.DCCM_DATA_WIDTH-1:0] = store_data_lo_dc5[pt.DCCM_DATA_WIDTH-1:0];

   // This is needed to avoid losing store on false sharing within a word
   assign lsu_stbuf_ecc_block = ld_single_ecc_error_dc3 | ld_single_ecc_error_dc4 | ld_single_ecc_error_dc5;
   assign lsu_stbuf_commit_any = stbuf_reqvld_any & ~lsu_stbuf_ecc_block &
                                 ((~(lsu_dccm_rden_dc1 | lsu_dccm_wren_spec_dc1 | ld_single_ecc_error_dc5_ff)) |
                                  (lsu_dccm_rden_dc1 & (~((stbuf_addr_any[DCCM_WIDTH_BITS+:pt.DCCM_BANK_BITS] == lsu_addr_dc1[DCCM_WIDTH_BITS+:pt.DCCM_BANK_BITS]) |
                                                              (stbuf_addr_any[DCCM_WIDTH_BITS+:pt.DCCM_BANK_BITS] == end_addr_dc1[DCCM_WIDTH_BITS+:pt.DCCM_BANK_BITS])))));

   // No need to read for aligned word/dword stores since ECC will come by new data completely
   // read enable is speculative for timing reasons
   assign lsu_dccm_rden_dc1 = (lsu_pkt_dc1_pre.valid & (lsu_pkt_dc1_pre.load | lsu_pkt_dc1_pre.atomic | (lsu_pkt_dc1_pre.store & (~(lsu_pkt_dc1_pre.word | lsu_pkt_dc1_pre.dword) | (lsu_addr_dc1[1:0] != 2'b0)))) & addr_in_dccm_region_dc1) |
                              (dma_dccm_spec_req & ~dma_mem_write);   // Read based on speculation is fine

   // DMA will read/write in decode stage
   assign lsu_dccm_wren_dc1 = dma_dccm_wen;
   assign lsu_dccm_wren_spec_dc1 = dma_dccm_spec_wen;

   // DCCM inputs
   assign dccm_wren                             = lsu_stbuf_commit_any | ld_single_ecc_error_dc5_ff | lsu_dccm_wren_dc1;
   assign dccm_rden                             = lsu_dccm_rden_dc1;
   assign dccm_wr_addr_lo[pt.DCCM_BITS-1:0]     = lsu_dccm_wren_spec_dc1 ? lsu_addr_dc1[pt.DCCM_BITS-1:0] :
                                                  (ld_single_ecc_error_dc5_ff ? (ld_single_ecc_error_lo_dc5_ff ? ld_sec_addr_lo_dc5_ff[pt.DCCM_BITS-1:0] : ld_sec_addr_hi_dc5_ff[pt.DCCM_BITS-1:0]) : stbuf_addr_any[pt.DCCM_BITS-1:0]);
   assign dccm_wr_addr_hi[pt.DCCM_BITS-1:0]     = lsu_dccm_wren_spec_dc1 ? end_addr_dc1[pt.DCCM_BITS-1:0] :
                                                  (ld_single_ecc_error_dc5_ff ? (ld_single_ecc_error_hi_dc5_ff ? ld_sec_addr_hi_dc5_ff[pt.DCCM_BITS-1:0] : ld_sec_addr_lo_dc5_ff[pt.DCCM_BITS-1:0]) : stbuf_addr_any[pt.DCCM_BITS-1:0]);
   assign dccm_rd_addr_lo[pt.DCCM_BITS-1:0]     = lsu_addr_dc1[pt.DCCM_BITS-1:0];
   assign dccm_rd_addr_hi[pt.DCCM_BITS-1:0]     = end_addr_dc1[pt.DCCM_BITS-1:0];

   // DCCM outputs
    assign ldst_byteen_dc2[7:0] = ({8{lsu_pkt_dc2.by}}    & 8'b0000_0001) |
                                  ({8{lsu_pkt_dc2.half}}  & 8'b0000_0011) |
                                  ({8{lsu_pkt_dc2.word}}  & 8'b0000_1111) |
                                  ({8{lsu_pkt_dc2.dword}} & 8'b1111_1111);

   assign ldst_byteen_dc3[7:0] = ({8{lsu_pkt_dc3.by}}    & 8'b0000_0001) |
                                 ({8{lsu_pkt_dc3.half}}  & 8'b0000_0011) |
                                 ({8{lsu_pkt_dc3.word}}  & 8'b0000_1111) |
                                 ({8{lsu_pkt_dc3.dword}} & 8'b1111_1111);

  assign ldst_byteen_dc4[7:0] =  ({8{lsu_pkt_dc4.by}}    & 8'b0000_0001) |
                                 ({8{lsu_pkt_dc4.half}}  & 8'b0000_0011) |
                                 ({8{lsu_pkt_dc4.word}}  & 8'b0000_1111) |
                                 ({8{lsu_pkt_dc4.dword}} & 8'b1111_1111);

  assign ldst_byteen_dc5[7:0] =  ({8{lsu_pkt_dc5.by}}    & 8'b0000_0001) |
                                 ({8{lsu_pkt_dc5.half}}  & 8'b0000_0011) |
                                 ({8{lsu_pkt_dc5.word}}  & 8'b0000_1111) |
                                 ({8{lsu_pkt_dc5.dword}} & 8'b1111_1111);

   assign ldst_byteen_ext_dc2[7:0] = ldst_byteen_dc2[7:0] << lsu_addr_dc2[1:0];      // The packet in dc2
   assign ldst_byteen_ext_dc3[7:0] = ldst_byteen_dc3[7:0] << lsu_addr_dc3[1:0];
   assign ldst_byteen_ext_dc4[7:0] = ldst_byteen_dc4[7:0] << lsu_addr_dc4[1:0];
   assign ldst_byteen_ext_dc5[7:0] = ldst_byteen_dc5[7:0] << lsu_addr_dc5[1:0];

   assign dccm_wr_bypass_c1_c2_lo   = (stbuf_addr_any[pt.DCCM_BITS-1:2] == lsu_addr_dc2[pt.DCCM_BITS-1:2]) & addr_in_dccm_dc2;
   assign dccm_wr_bypass_c1_c2_hi   = (stbuf_addr_any[pt.DCCM_BITS-1:2] == end_addr_dc2[pt.DCCM_BITS-1:2]) & addr_in_dccm_dc2 & ~lsu_pkt_dc2.sc;   // SC always aligned and upper 32 bits are used for ECC corrected data

   assign dccm_wr_bypass_c1_c3_lo   = (stbuf_addr_any[pt.DCCM_BITS-1:2] == lsu_addr_dc3[pt.DCCM_BITS-1:2]) & addr_in_dccm_dc3;
   assign dccm_wr_bypass_c1_c3_hi   = (stbuf_addr_any[pt.DCCM_BITS-1:2] == end_addr_dc3[pt.DCCM_BITS-1:2]) & addr_in_dccm_dc3 & ~lsu_pkt_dc3.sc;   // SC always aligned and upper 32 bits are used for ECC corrected data

   assign dccm_wr_bypass_c1_c4_lo   = (stbuf_addr_any[pt.DCCM_BITS-1:2] == lsu_addr_dc4[pt.DCCM_BITS-1:2]) & addr_in_dccm_dc4;
   assign dccm_wr_bypass_c1_c4_hi   = (stbuf_addr_any[pt.DCCM_BITS-1:2] == end_addr_dc4[pt.DCCM_BITS-1:2]) & addr_in_dccm_dc4 & ~lsu_pkt_dc4.sc;   // SC always aligned and upper 32 bits are used for ECC corrected data

   assign dccm_wr_bypass_c1_c5_lo   = (stbuf_addr_any[pt.DCCM_BITS-1:2] == lsu_addr_dc5[pt.DCCM_BITS-1:2]) & addr_in_dccm_dc5;
   assign dccm_wr_bypass_c1_c5_hi   = (stbuf_addr_any[pt.DCCM_BITS-1:2] == end_addr_dc5[pt.DCCM_BITS-1:2]) & addr_in_dccm_dc5 & ~lsu_pkt_dc5.sc;   // SC always aligned and upper 32 bits are used for ECC corrected data

   // For SC conditional, hi data is used for ecc corrected data since in case of sc fail we just need to write corrected data
   assign store_data_lo_dc3[31:0]= (lsu_pkt_dc3.atomic & ~lsu_pkt_dc3.lr & ~lsu_pkt_dc3.sc) ? amo_data_dc3[31:0] : store_ecc_data_lo_dc3[31:0];
   assign store_data_hi_dc3[31:0]= (lsu_pkt_dc3.atomic & ~lsu_pkt_dc3.lr & ~lsu_pkt_dc3.sc) ? amo_data_dc3[31:0] : (lsu_pkt_dc3.atomic & lsu_pkt_dc3.sc) ? sec_data_lo_dc3[31:0] : store_ecc_data_hi_dc3[31:0];

   // Timing fix for EH2_plus1. DCCM read data comes out in 2 cycles
   if (pt.LOAD_TO_USE_PLUS1 == 1) begin: GenL2U_1
      logic lsu_stbuf_commit_any_Q;
      logic dccm_wr_bypass_c1_c2_lo_Q, dccm_wr_bypass_c1_c2_hi_Q;
      logic [31:0] stbuf_data_any_Q;

      for (genvar i=0; i<4; i++) begin: Gen_dccm_data
         assign dccm_data_lo_dc3[(8*i)+7:(8*i)]     = dccm_rd_data_lo[(8*i)+7:(8*i)];
         assign dccm_data_hi_dc3[(8*i)+7:(8*i)]     = dccm_rd_data_hi[(8*i)+7:(8*i)];

         assign dccm_data_lo_dc4_in[(8*i)+7:(8*i)]  = (lsu_stbuf_commit_any &  dccm_wr_bypass_c1_c3_lo & ~ldst_byteen_ext_dc3[i]) ? stbuf_data_any[(8*i)+7:(8*i)] :
                                                                          (lsu_stbuf_commit_any_Q & dccm_wr_bypass_c1_c2_lo_Q & ~ldst_byteen_ext_dc3[i]) ? stbuf_data_any_Q[(8*i)+7:(8*i)] : store_data_lo_dc3[(8*i)+7:(8*i)];
         assign dccm_data_hi_dc4_in[(8*i)+7:(8*i)]  = (lsu_stbuf_commit_any &  dccm_wr_bypass_c1_c3_hi & ~ldst_byteen_ext_dc3[i+4]) ? stbuf_data_any[(8*i)+7:(8*i)] :
                                                                          (lsu_stbuf_commit_any_Q & dccm_wr_bypass_c1_c2_hi_Q & ~ldst_byteen_ext_dc3[i+4]) ? stbuf_data_any_Q[(8*i)+7:(8*i)] : store_data_hi_dc3[(8*i)+7:(8*i)];
      end

      assign dccm_data_ecc_lo_dc3[pt.DCCM_ECC_WIDTH-1:0] = dccm_rd_data_lo[pt.DCCM_FDATA_WIDTH-1:pt.DCCM_DATA_WIDTH];
      assign dccm_data_ecc_hi_dc3[pt.DCCM_ECC_WIDTH-1:0] = dccm_rd_data_hi[pt.DCCM_FDATA_WIDTH-1:pt.DCCM_DATA_WIDTH];

      rvdff #(1) stbuf_commit_ff (.din(lsu_stbuf_commit_any), .dout(lsu_stbuf_commit_any_Q), .clk(lsu_c2_dc3_clk), .*);
      rvdff #(1) dccm_wr_bypass_c1_c2_loff (.din(dccm_wr_bypass_c1_c2_lo), .dout(dccm_wr_bypass_c1_c2_lo_Q), .clk(lsu_c2_dc3_clk), .*);
      rvdff #(1) dccm_wr_bypass_c1_c2_hiff (.din(dccm_wr_bypass_c1_c2_hi), .dout(dccm_wr_bypass_c1_c2_hi_Q), .clk(lsu_c2_dc3_clk), .*);
      rvdffe #(32) stbuf_data_anyff (.din(stbuf_data_any[31:0]), .dout(stbuf_data_any_Q[31:0]), .en(lsu_stbuf_commit_any), .*);

   end else begin: GenL2U_0
      logic [pt.DCCM_DATA_WIDTH-1:0]  dccm_data_hi_dc2, dccm_data_lo_dc2;
      logic [pt.DCCM_ECC_WIDTH-1:0]   dccm_data_ecc_hi_dc2, dccm_data_ecc_lo_dc2;

      for (genvar i=0; i<4; i++) begin: Gen_dccm_data
         assign dccm_data_lo_dc2[(8*i)+7:(8*i)]     = (lsu_stbuf_commit_any &  lsu_pkt_dc2.store & dccm_wr_bypass_c1_c2_lo & ~ldst_byteen_ext_dc2[i])   ? stbuf_data_any[(8*i)+7:(8*i)] : dccm_rd_data_lo[(8*i)+7:(8*i)]; // for ld choose dccm_out
         assign dccm_data_hi_dc2[(8*i)+7:(8*i)]     = (lsu_stbuf_commit_any &  lsu_pkt_dc2.store & dccm_wr_bypass_c1_c2_hi & ~ldst_byteen_ext_dc2[i+4]) ? stbuf_data_any[(8*i)+7:(8*i)] : dccm_rd_data_hi[(8*i)+7:(8*i)]; // for ld this is used for ecc

         assign dccm_data_lo_dc4_in[(8*i)+7:(8*i)]  = (lsu_stbuf_commit_any &  dccm_wr_bypass_c1_c3_lo & ~ldst_byteen_ext_dc3[i])   ? stbuf_data_any[(8*i)+7:(8*i)] : store_data_lo_dc3[(8*i)+7:(8*i)];
         assign dccm_data_hi_dc4_in[(8*i)+7:(8*i)]  = (lsu_stbuf_commit_any &  dccm_wr_bypass_c1_c3_hi & ~ldst_byteen_ext_dc3[i+4]) ? stbuf_data_any[(8*i)+7:(8*i)] : store_data_hi_dc3[(8*i)+7:(8*i)];
      end

      assign dccm_data_ecc_lo_dc2[pt.DCCM_ECC_WIDTH-1:0] = dccm_rd_data_lo[pt.DCCM_FDATA_WIDTH-1:pt.DCCM_DATA_WIDTH];
      assign dccm_data_ecc_hi_dc2[pt.DCCM_ECC_WIDTH-1:0] = dccm_rd_data_hi[pt.DCCM_FDATA_WIDTH-1:pt.DCCM_DATA_WIDTH];

      rvdff #(pt.DCCM_DATA_WIDTH) dccm_data_hi_dc3ff (.*, .din(dccm_data_hi_dc2[pt.DCCM_DATA_WIDTH-1:0]),    .dout(dccm_data_hi_dc3[pt.DCCM_DATA_WIDTH-1:0]),    .clk(lsu_dccm_c1_dc3_clk));
      rvdff #(pt.DCCM_DATA_WIDTH) dccm_data_lo_dc3ff (.*, .din(dccm_data_lo_dc2[pt.DCCM_DATA_WIDTH-1:0]),    .dout(dccm_data_lo_dc3[pt.DCCM_DATA_WIDTH-1:0]),    .clk(lsu_dccm_c1_dc3_clk));

      rvdff #(pt.DCCM_ECC_WIDTH) dccm_data_ecc_hi_ff (.*, .din(dccm_data_ecc_hi_dc2[pt.DCCM_ECC_WIDTH-1:0]), .dout(dccm_data_ecc_hi_dc3[pt.DCCM_ECC_WIDTH-1:0]), .clk(lsu_dccm_c1_dc3_clk));
      rvdff #(pt.DCCM_ECC_WIDTH) dccm_data_ecc_lo_ff (.*, .din(dccm_data_ecc_lo_dc2[pt.DCCM_ECC_WIDTH-1:0]), .dout(dccm_data_ecc_lo_dc3[pt.DCCM_ECC_WIDTH-1:0]), .clk(lsu_dccm_c1_dc3_clk));

   end


   for (genvar i=0; i<4; i++) begin: Gen_dccm_data_dc4_dc5
      assign dccm_data_lo_dc5_in[(8*i)+7:(8*i)]  = (lsu_stbuf_commit_any &  dccm_wr_bypass_c1_c4_lo & ~ldst_byteen_ext_dc4[i])   ? stbuf_data_any[(8*i)+7:(8*i)] : store_data_lo_dc4[(8*i)+7:(8*i)];
      assign dccm_data_hi_dc5_in[(8*i)+7:(8*i)]  = (lsu_stbuf_commit_any &  dccm_wr_bypass_c1_c4_hi & ~ldst_byteen_ext_dc4[i+4]) ? stbuf_data_any[(8*i)+7:(8*i)] : store_data_hi_dc4[(8*i)+7:(8*i)];

      // This is store buffer write data
      assign dccm_datafn_lo_dc5[(8*i)+7:(8*i)]   = (lsu_stbuf_commit_any &  dccm_wr_bypass_c1_c5_lo & ~ldst_byteen_ext_dc5[i]) ? stbuf_data_any[(8*i)+7:(8*i)] :
                                                                                 (lsu_pkt_dc5.atomic & lsu_pkt_dc5.sc & ~lsu_sc_success_dc5) ? store_data_hi_dc5[(8*i)+7:(8*i)] : store_data_lo_dc5[(8*i)+7:(8*i)];
      assign dccm_datafn_hi_dc5[(8*i)+7:(8*i)]   = (lsu_stbuf_commit_any &  dccm_wr_bypass_c1_c5_hi & ~ldst_byteen_ext_dc5[i+4]) ? stbuf_data_any[(8*i)+7:(8*i)] : store_data_hi_dc5[(8*i)+7:(8*i)];
   end // for (genvar i=0; i<BYTE_WIDTH; i++)

   // Need to disable ecc correction since data is being forwarded for store (ECC is from RAM but data from forwarding path so they are out of sync).
   assign disable_ecc_check_lo_dc2 = lsu_stbuf_commit_any & lsu_pkt_dc2.store & dccm_wr_bypass_c1_c2_lo;
   assign disable_ecc_check_hi_dc2 = lsu_stbuf_commit_any & lsu_pkt_dc2.store & dccm_wr_bypass_c1_c2_hi;

   // PIC signals. PIC ignores the lower 2 bits of address since PIC memory registers are 32-bits
   assign picm_wren          = (lsu_pkt_dc5.valid & lsu_pkt_dc5.store & addr_in_pic_dc5 & lsu_commit_dc5) | dma_pic_wen;
   assign picm_rden          = lsu_pkt_dc1.valid & lsu_pkt_dc1.load  & addr_in_pic_dc1;
   assign picm_mken          = lsu_pkt_dc1.valid & lsu_pkt_dc1.store & addr_in_pic_dc1;  // Get the mask for stores
   assign picm_rd_thr        = lsu_pkt_dc1.tid;
   assign picm_rdaddr[31:0]  = lsu_addr_dc1[31:0];
   assign picm_wraddr[31:0]  = dma_pic_wen ? dma_mem_addr[31:0] : lsu_addr_dc5[31:0];
   assign picm_wr_data[31:0] = dma_pic_wen ? dma_mem_wdata[31:0] : store_data_lo_dc5[31:0];

   // getting raw store data back for bus
   assign store_data_ext_dc3[63:0] = {store_ecc_data_hi_dc3[31:0], store_ecc_data_lo_dc3[31:0]};   // We don't need AMO here since this is used for fwding and there can't be a load behind AMO
   assign store_data_ext_dc4[63:0] = {store_data_hi_dc4[31:0], store_data_lo_dc4[31:0]};
   assign store_data_ext_dc5[63:0] = {store_data_hi_dc5[31:0], store_data_lo_dc5[31:0]};

   // Flops
   assign picm_mask_data_dc3[31:0] = picm_rd_dataQ[31:0];   // Can't use picm_rd_data_dc3 since we don't need forward data here (this is mask)
   assign picm_rd_data_dc2 = picm_fwd_en_dc2 ? picm_fwd_data_dc2[31:0] : picm_rd_data[31:0];
   rvdff #(32) picm_data_ff    (.*, .din(picm_rd_data_dc2[31:0]), .dout(picm_rd_data_dc3[31:0]), .clk(lsu_pic_c1_dc3_clk));
   rvdff #(32) picm_rd_data_ff (.*, .din(picm_rd_data[31:0]),     .dout(picm_rd_dataQ[31:0]),    .clk(lsu_pic_c1_dc3_clk));

   rvdff #(pt.DCCM_DATA_WIDTH) dccm_data_hi_dc4ff (.*, .din(dccm_data_hi_dc4_in[pt.DCCM_DATA_WIDTH-1:0]), .dout(store_data_hi_dc4[pt.DCCM_DATA_WIDTH-1:0]),    .clk(lsu_c1_dc4_clk));
   rvdff #(pt.DCCM_DATA_WIDTH) dccm_data_lo_dc4ff (.*, .din(dccm_data_lo_dc4_in[pt.DCCM_DATA_WIDTH-1:0]), .dout(store_data_lo_dc4[pt.DCCM_DATA_WIDTH-1:0]),    .clk(lsu_c1_dc4_clk));

   rvdff #(pt.DCCM_DATA_WIDTH) dccm_data_hi_dc5ff (.*, .din(dccm_data_hi_dc5_in[pt.DCCM_DATA_WIDTH-1:0]), .dout(store_data_hi_dc5[pt.DCCM_DATA_WIDTH-1:0]),    .clk(lsu_c1_dc5_clk));
   rvdff #(pt.DCCM_DATA_WIDTH) dccm_data_lo_dc5ff (.*, .din(dccm_data_lo_dc5_in[pt.DCCM_DATA_WIDTH-1:0]), .dout(store_data_lo_dc5[pt.DCCM_DATA_WIDTH-1:0]),    .clk(lsu_c1_dc5_clk));

   if (pt.DCCM_ENABLE == 1) begin: Gen_dccm_enable
      rvdff #(1) dccm_rden_dc2ff (.*, .din(lsu_dccm_rden_dc1), .dout(lsu_dccm_rden_dc2), .clk(lsu_c2_dc2_clk));
      rvdff #(1) dccm_rden_dc3ff (.*, .din(lsu_dccm_rden_dc2), .dout(lsu_dccm_rden_dc3), .clk(lsu_c2_dc3_clk));

      rvdff #(1) ecc_disable_hi_dc3ff (.*, .din(disable_ecc_check_hi_dc2),    .dout(disable_ecc_check_hi_dc3),    .clk(lsu_dccm_c1_dc3_clk));
      rvdff #(1) ecc_disable_lo_dc3ff (.*, .din(disable_ecc_check_lo_dc2),    .dout(disable_ecc_check_lo_dc3),    .clk(lsu_dccm_c1_dc3_clk));

      // ECC correction flops since dccm write happens next cycle
      // We are writing to dccm in dc5+1 for ecc correction since fast_int needs to be blocked in decode - 2.
      rvdff #(1) lsu_double_ecc_error_dc5ff     (.*, .din(lsu_double_ecc_error_dc5),   .dout(lsu_double_ecc_error_dc5_ff),   .clk(lsu_free_c2_clk));
      rvdff #(1) ld_single_ecc_error_hi_dc5ff   (.*, .din(ld_single_ecc_error_hi_dc5_ns), .dout(ld_single_ecc_error_hi_dc5_ff), .clk(lsu_free_c2_clk));
      rvdff #(1) ld_single_ecc_error_lo_dc5ff   (.*, .din(ld_single_ecc_error_lo_dc5_ns), .dout(ld_single_ecc_error_lo_dc5_ff), .clk(lsu_free_c2_clk));
      rvdffe #(pt.DCCM_BITS) ld_sec_addr_hi_rff (.*, .din(end_addr_dc5[pt.DCCM_BITS-1:0]), .dout(ld_sec_addr_hi_dc5_ff[pt.DCCM_BITS-1:0]), .en(ld_single_ecc_error_dc5), .clk(clk));
      rvdffe #(pt.DCCM_BITS) ld_sec_addr_lo_rff (.*, .din(lsu_addr_dc5[pt.DCCM_BITS-1:0]), .dout(ld_sec_addr_lo_dc5_ff[pt.DCCM_BITS-1:0]), .en(ld_single_ecc_error_dc5), .clk(clk));

   end else begin: Gen_dccm_disable
      assign lsu_dccm_rden_dc2 = '0;
      assign lsu_dccm_rden_dc3 = '0;
      assign disable_ecc_check_lo_dc3 = 1'b1;
      assign disable_ecc_check_hi_dc3 = 1'b1;

      assign lsu_double_ecc_error_dc5_ff = '0;
      assign ld_single_ecc_error_lo_dc5_ff = '0;
      assign ld_single_ecc_error_hi_dc5_ff = '0;
      assign ld_sec_addr_lo_dc5_ff[pt.DCCM_BITS-1:0] = '0;
      assign ld_sec_addr_hi_dc5_ff[pt.DCCM_BITS-1:0] = '0;
   end

endmodule
