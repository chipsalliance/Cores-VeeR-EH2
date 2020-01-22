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
// Function: lsu interface with interface queue
// Comments:
//
//********************************************************************************
module eh2_lsu_bus_intf
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)(
   input logic                          clk,
   input logic                          rst_l,
   input logic                          scan_mode,
   input logic                          dec_tlu_external_ldfwd_disable,     // disable load to load forwarding for externals
   input logic                          dec_tlu_wb_coalescing_disable,      // disable write buffer coalescing
   input logic                          dec_tlu_sideeffect_posted_disable,  // disable posted writes to sideeffect addr to the bus

   // various clocks needed for the bus reads and writes
   input logic                          lsu_c1_dc2_clk,
   input logic                          lsu_c1_dc3_clk,
   input logic                          lsu_c1_dc4_clk,
   input logic                          lsu_c1_dc5_clk,
   input logic                          lsu_c2_dc2_clk,
   input logic                          lsu_c2_dc3_clk,
   input logic                          lsu_c2_dc4_clk,
   input logic                          lsu_c2_dc5_clk,

   input logic [pt.NUM_THREADS-1:0]     lsu_bus_ibuf_c1_clk,
   input logic [pt.NUM_THREADS-1:0]     lsu_bus_obuf_c1_clk,
   input logic [pt.NUM_THREADS-1:0]     lsu_bus_buf_c1_clk,
   input logic                          lsu_free_c2_clk,
   input logic                          free_clk,
   input logic                          lsu_busm_clk,

   input logic                          lsu_busreq_dc1,                   // bus request is in dc2

   input                                eh2_lsu_pkt_t lsu_pkt_dc1_pre,        // lsu packet flowing down the pipe
   input                                eh2_lsu_pkt_t lsu_pkt_dc2,            // lsu packet flowing down the pipe
   input                                eh2_lsu_pkt_t lsu_pkt_dc3,            // lsu packet flowing down the pipe
   input                                eh2_lsu_pkt_t lsu_pkt_dc4,            // lsu packet flowing down the pipe
   input                                eh2_lsu_pkt_t lsu_pkt_dc5,            // lsu packet flowing down the pipe

   input logic [31:0]                   lsu_addr_dc1,                     // lsu address flowing down the pipe
   input logic [31:0]                   lsu_addr_dc2,                     // lsu address flowing down the pipe
   input logic [31:0]                   lsu_addr_dc3,                     // lsu address flowing down the pipe
   input logic [31:0]                   lsu_addr_dc4,                     // lsu address flowing down the pipe
   input logic [31:0]                   lsu_addr_dc5,                     // lsu address flowing down the pipe

   input logic [31:0]                   end_addr_dc1,                     // lsu address flowing down the pipe
   input logic [31:0]                   end_addr_dc2,                     // lsu address flowing down the pipe
   input logic [31:0]                   end_addr_dc3,                     // lsu address flowing down the pipe
   input logic [31:0]                   end_addr_dc4,                     // lsu address flowing down the pipe
   input logic [31:0]                   end_addr_dc5,                     // lsu address flowing down the pipe

   input logic [63:0]                   store_data_ext_dc3,               // store data flowing down the pipe
   input logic [63:0]                   store_data_ext_dc4,               // store data flowing down the pipe
   input logic [63:0]                   store_data_ext_dc5,               // store data flowing down the pipe
   input logic [pt.NUM_THREADS-1:0]     dec_tlu_force_halt,

   input logic                          core_ldst_dual_dc1,               // core ld/st is dual
   input logic                          lsu_commit_dc5,                   // lsu instruction in dc5 commits
   input logic                          is_sideeffects_dc2,               // lsu attribute is side_effects
   input logic                          is_sideeffects_dc3,               // lsu attribute is side_effects
   input logic [pt.NUM_THREADS-1:0]     flush_dc2_up,                     // flush
   input logic [pt.NUM_THREADS-1:0]     flush_dc3,                        // flush
   input logic [pt.NUM_THREADS-1:0]     flush_dc4,                        // flush

   output logic                         lsu_busreq_dc5,                   // bus request is in dc5
   output logic [pt.NUM_THREADS-1:0]    lsu_bus_idle_any,                 // No pending responses from the bus
   output logic [pt.NUM_THREADS-1:0]    lsu_bus_buffer_pend_any,          // bus buffer has a pending bus entry
   output logic [pt.NUM_THREADS-1:0]    lsu_bus_buffer_full_any,          // write buffer is full
   output logic [pt.NUM_THREADS-1:0]    lsu_bus_buffer_empty_any,         // write buffer is empty
   output logic [31:0]                  bus_read_data_dc3,                   // the bus return data

   output logic [pt.NUM_THREADS-1:0]         lsu_imprecise_error_load_any,     // imprecise load bus error
   output logic [pt.NUM_THREADS-1:0]         lsu_imprecise_error_store_any,    // imprecise store bus error
   output logic [pt.NUM_THREADS-1:0][31:0]   lsu_imprecise_error_addr_any,     // address of the imprecise error

   // Non-blocking loads
   output logic                               lsu_nonblock_load_valid_dc1,     // there is an external load -> put in the cam
   output logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0] lsu_nonblock_load_tag_dc1,       // the tag of the external non block load
   output logic                               lsu_nonblock_load_inv_dc2,       // Invalidate the non-block load bcoz of memory forwarding
   output logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0] lsu_nonblock_load_inv_tag_dc2,
   output logic                               lsu_nonblock_load_inv_dc5,       // invalidate signal for the cam entry for non block loads
   output logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0] lsu_nonblock_load_inv_tag_dc5,   // tag of the enrty which needs to be invalidated
   output logic                               lsu_nonblock_load_data_valid,    // the non block is valid - sending information back to the cam
   output logic                               lsu_nonblock_load_data_error,    // non block load has an error
   output logic                               lsu_nonblock_load_data_tid,      // tid for nonblock load return
   output logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0] lsu_nonblock_load_data_tag,      // the tag of the non block load sending the data/error
   output logic [31:0]                        lsu_nonblock_load_data,          // Data of the non block load

   // PMU events
   output logic [pt.NUM_THREADS-1:0]    lsu_pmu_bus_trxn,
   output logic [pt.NUM_THREADS-1:0]    lsu_pmu_bus_misaligned,
   output logic [pt.NUM_THREADS-1:0]    lsu_pmu_bus_error,
   output logic [pt.NUM_THREADS-1:0]    lsu_pmu_bus_busy,

   //-------------------------- LSU AXI signals--------------------------
   // AXI Write Channels
   output logic                         lsu_axi_awvalid,
   input  logic                         lsu_axi_awready,
   output logic [pt.LSU_BUS_TAG-1:0]    lsu_axi_awid,
   output logic [31:0]                  lsu_axi_awaddr,
   output logic [3:0]                   lsu_axi_awregion,
   output logic [7:0]                   lsu_axi_awlen,
   output logic [2:0]                   lsu_axi_awsize,
   output logic [1:0]                   lsu_axi_awburst,
   output logic                         lsu_axi_awlock,
   output logic [3:0]                   lsu_axi_awcache,
   output logic [2:0]                   lsu_axi_awprot,
   output logic [3:0]                   lsu_axi_awqos,

   output logic                         lsu_axi_wvalid,
   input  logic                         lsu_axi_wready,
   output logic [63:0]                  lsu_axi_wdata,
   output logic [7:0]                   lsu_axi_wstrb,
   output logic                         lsu_axi_wlast,

   input  logic                         lsu_axi_bvalid,
   output logic                         lsu_axi_bready,
   input  logic [1:0]                   lsu_axi_bresp,
   input  logic [pt.LSU_BUS_TAG-1:0]    lsu_axi_bid,

   // AXI Read Channels
   output logic                         lsu_axi_arvalid,
   input  logic                         lsu_axi_arready,
   output logic [pt.LSU_BUS_TAG-1:0]    lsu_axi_arid,
   output logic [31:0]                  lsu_axi_araddr,
   output logic [3:0]                   lsu_axi_arregion,
   output logic [7:0]                   lsu_axi_arlen,
   output logic [2:0]                   lsu_axi_arsize,
   output logic [1:0]                   lsu_axi_arburst,
   output logic                         lsu_axi_arlock,
   output logic [3:0]                   lsu_axi_arcache,
   output logic [2:0]                   lsu_axi_arprot,
   output logic [3:0]                   lsu_axi_arqos,

   input  logic                         lsu_axi_rvalid,
   output logic                         lsu_axi_rready,
   input  logic [pt.LSU_BUS_TAG-1:0]    lsu_axi_rid,
   input  logic [63:0]                  lsu_axi_rdata,
   input  logic [1:0]                   lsu_axi_rresp,

   input logic                          lsu_bus_clk_en

);

   logic              lsu_bus_clk_en_q;
   logic              ldst_dual_dc1, ldst_dual_dc2, ldst_dual_dc3, ldst_dual_dc4, ldst_dual_dc5;
   logic              lsu_busreq_dc2, lsu_busreq_dc3, lsu_busreq_dc4;
   logic              ldst_samedw_dc5, is_aligned_dc5;

   logic [3:0]        ldst_byteen_dc2, ldst_byteen_dc3, ldst_byteen_dc4, ldst_byteen_dc5;
   logic [7:0]        ldst_byteen_ext_dc2, ldst_byteen_ext_dc3, ldst_byteen_ext_dc4, ldst_byteen_ext_dc5;
   logic [3:0]        ldst_byteen_hi_dc2, ldst_byteen_hi_dc3, ldst_byteen_hi_dc4, ldst_byteen_hi_dc5;
   logic [3:0]        ldst_byteen_lo_dc2, ldst_byteen_lo_dc3, ldst_byteen_lo_dc4, ldst_byteen_lo_dc5;
   logic              is_sideeffects_dc4, is_sideeffects_dc5;

   logic [31:0]       store_data_hi_dc3, store_data_hi_dc4, store_data_hi_dc5;
   logic [31:0]       store_data_lo_dc3, store_data_lo_dc4, store_data_lo_dc5;

   logic              addr_match_dw_lo_dc5_dc4, addr_match_dw_lo_dc5_dc3, addr_match_dw_lo_dc5_dc2;
   logic              addr_match_word_lo_dc5_dc4, addr_match_word_lo_dc5_dc3, addr_match_word_lo_dc5_dc2;
   logic              no_word_merge_dc5, no_dword_merge_dc5;

   logic              ld_addr_dc3hit_lo_lo, ld_addr_dc3hit_hi_lo, ld_addr_dc3hit_lo_hi, ld_addr_dc3hit_hi_hi;
   logic              ld_addr_dc4hit_lo_lo, ld_addr_dc4hit_hi_lo, ld_addr_dc4hit_lo_hi, ld_addr_dc4hit_hi_hi;
   logic              ld_addr_dc5hit_lo_lo, ld_addr_dc5hit_hi_lo, ld_addr_dc5hit_lo_hi, ld_addr_dc5hit_hi_hi;

   logic [3:0]        ld_byte_dc3hit_lo_lo, ld_byte_dc3hit_hi_lo, ld_byte_dc3hit_lo_hi, ld_byte_dc3hit_hi_hi;
   logic [3:0]        ld_byte_dc4hit_lo_lo, ld_byte_dc4hit_hi_lo, ld_byte_dc4hit_lo_hi, ld_byte_dc4hit_hi_hi;
   logic [3:0]        ld_byte_dc5hit_lo_lo, ld_byte_dc5hit_hi_lo, ld_byte_dc5hit_lo_hi, ld_byte_dc5hit_hi_hi;

   logic [3:0]        ld_byte_hit_lo, ld_byte_dc3hit_lo, ld_byte_dc4hit_lo, ld_byte_dc5hit_lo;
   logic [3:0]        ld_byte_hit_hi, ld_byte_dc3hit_hi, ld_byte_dc4hit_hi, ld_byte_dc5hit_hi;

   logic [31:0]       ld_fwddata_dc3pipe_lo, ld_fwddata_dc4pipe_lo, ld_fwddata_dc5pipe_lo;
   logic [31:0]       ld_fwddata_dc3pipe_hi, ld_fwddata_dc4pipe_hi, ld_fwddata_dc5pipe_hi;

   logic [pt.NUM_THREADS-1:0][3:0]   ld_byte_hit_buf_lo, ld_byte_hit_buf_hi;
   logic [pt.NUM_THREADS-1:0][31:0]  ld_fwddata_buf_lo, ld_fwddata_buf_hi;

   logic [31:0]       ld_fwddata_lo, ld_fwddata_hi;
   logic [31:0]       ld_fwddata_dc2, ld_fwddata_dc3;

   logic              ld_full_hit_hi_dc2, ld_full_hit_lo_dc2;
   logic              ld_full_hit_dc2;

   logic [pt.NUM_THREADS-1:0][pt.LSU_NUM_NBLOAD_WIDTH-1:0] WrPtr0_dc1, WrPtr0_dc2, WrPtr0_dc5;

   logic [63:32]     ld_fwddata_dc2_nc;

   logic             bus_tid, nxt_bus_tid, bus_tid_en;

   // Output buffer signals
   logic [pt.NUM_THREADS-1:0]                     obuf_valid;
   logic [pt.NUM_THREADS-1:0]                     obuf_nosend;
   logic [pt.NUM_THREADS-1:0]                     obuf_write;
   logic [pt.NUM_THREADS-1:0]                     obuf_sideeffect;
   logic [pt.NUM_THREADS-1:0][31:0]               obuf_addr;
   logic [pt.NUM_THREADS-1:0][63:0]               obuf_data;
   logic [pt.NUM_THREADS-1:0][1:0]                obuf_sz;
   logic [pt.NUM_THREADS-1:0][7:0]                obuf_byteen;
   logic [pt.NUM_THREADS-1:0]                     obuf_cmd_done, obuf_data_done;
   logic [pt.NUM_THREADS-1:0][pt.LSU_BUS_TAG-1:0] obuf_tag0;
   logic [pt.NUM_THREADS-1:0]                     obuf_nxtready;

   logic                                lsu_nonblock_load_valid_dc2, lsu_nonblock_load_valid_dc3, lsu_nonblock_load_valid_dc4,lsu_nonblock_load_valid_dc5;
   logic [pt.NUM_THREADS-1:0][7:0]      bus_pend_trxn, bus_pend_trxn_ns, bus_pend_trxnQ;
   logic [pt.NUM_THREADS-1:0]           lsu_bus_cntr_overflow;

   logic [pt.NUM_THREADS-1:0]           bus_addr_match_pending;
   logic                                bus_cmd_valid, bus_cmd_sent, bus_cmd_ready;
   logic                                bus_wcmd_sent, bus_wdata_sent;

   logic                                bus_rsp_read, bus_rsp_write;
   logic                                bus_rsp_tid;
   logic [pt.LSU_BUS_TAG-1:0]           bus_rsp_read_tag, bus_rsp_write_tag;
   logic                                bus_rsp_read_tid, bus_rsp_write_tid;
   logic                                bus_rsp_read_error, bus_rsp_write_error;
   logic [63:0]                         bus_rsp_rdata;

   logic                                bus_rsp_valid_q, bus_rsp_ready_q, bus_rsp_write_q, bus_rsp_error_q;
   logic                                bus_rsp_write_tid_q;
   logic [63:0]                         bus_rsp_rdata_q;

   logic [pt.NUM_THREADS-1:0] tid_bus_buffer_pend_any;

   logic [pt.NUM_THREADS-1:0][31:0] tid_imprecise_error_addr_any;     // address of the imprecise error

   // Non-blocking loads
   logic [pt.NUM_THREADS-1:0]                              tid_nonblock_load_data_ready;
   logic [pt.NUM_THREADS-1:0]                              tid_nonblock_load_data_valid;
   logic [pt.NUM_THREADS-1:0]                              tid_nonblock_load_data_error;
   logic [pt.NUM_THREADS-1:0][pt.LSU_NUM_NBLOAD_WIDTH-1:0] tid_nonblock_load_data_tag;
   logic [pt.NUM_THREADS-1:0][31:0]                        tid_nonblock_load_data;

   logic                   lsu_axi_awvalid_q, lsu_axi_awready_q;
   logic                   lsu_axi_wvalid_q, lsu_axi_wready_q;
   logic                   lsu_axi_arvalid_q, lsu_axi_arready_q;
   logic                   lsu_axi_bvalid_q, lsu_axi_bready_q;
   logic                   lsu_axi_rvalid_q, lsu_axi_rready_q;
   logic [pt.LSU_BUS_TAG-1:0] lsu_axi_bid_q, lsu_axi_rid_q;
   logic [1:0]             lsu_axi_bresp_q, lsu_axi_rresp_q;
   logic [63:0]            lsu_axi_rdata_q;

   logic                   bus_coalescing_disable;

   //------------------------------------------------------------------------------------------------------------
   //----------------------------------------Logic starts here---------------------------------------------------
   //------------------------------------------------------------------------------------------------------------

   assign bus_coalescing_disable = dec_tlu_wb_coalescing_disable | pt.BUILD_AHB_LITE;  // No coalescing for ahb

   assign ldst_byteen_dc2[3:0] = ({4{lsu_pkt_dc2.by}}   & 4'b0001) |
                                 ({4{lsu_pkt_dc2.half}} & 4'b0011) |
                                 ({4{lsu_pkt_dc2.word}} & 4'b1111);
   assign ldst_dual_dc1   = core_ldst_dual_dc1;
   assign ldst_samedw_dc5 = (lsu_addr_dc5[3] == end_addr_dc5[3]);
   assign is_aligned_dc5    = (lsu_pkt_dc5.word & (lsu_addr_dc5[1:0] == 2'b0)) |
                              (lsu_pkt_dc5.half & (lsu_addr_dc5[0] == 1'b0)) |
                              lsu_pkt_dc5.by;

   // Logic to determine if dc5 store can be coalesced or not with younger stores. Bypass ibuf if cannot colaesced
   assign addr_match_dw_lo_dc5_dc4 = (lsu_addr_dc5[31:3] == lsu_addr_dc4[31:3]);
   assign addr_match_dw_lo_dc5_dc3 = (lsu_addr_dc5[31:3] == lsu_addr_dc3[31:3]);
   assign addr_match_dw_lo_dc5_dc2 = (lsu_addr_dc5[31:3] == lsu_addr_dc2[31:3]);

   assign addr_match_word_lo_dc5_dc4 = addr_match_dw_lo_dc5_dc4 & ~(lsu_addr_dc5[2]^lsu_addr_dc4[2]);
   assign addr_match_word_lo_dc5_dc3 = addr_match_dw_lo_dc5_dc3 & ~(lsu_addr_dc5[2]^lsu_addr_dc3[2]);
   assign addr_match_word_lo_dc5_dc2 = addr_match_dw_lo_dc5_dc2 & ~(lsu_addr_dc5[2]^lsu_addr_dc2[2]);

   assign no_word_merge_dc5  = lsu_busreq_dc5 & ~ldst_dual_dc5 &
                               ((lsu_busreq_dc4 & (lsu_pkt_dc4.tid ~^ lsu_pkt_dc5.tid) & (lsu_pkt_dc4.load | ~addr_match_word_lo_dc5_dc4)) |
                                (lsu_busreq_dc3 & (lsu_pkt_dc3.tid ~^ lsu_pkt_dc5.tid) & ~(lsu_busreq_dc4 & (lsu_pkt_dc4.tid ~^ lsu_pkt_dc5.tid)) & (lsu_pkt_dc3.load | ~addr_match_word_lo_dc5_dc3)) |
                                (lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc5.tid) & ~(lsu_busreq_dc3 & (lsu_pkt_dc3.tid ~^ lsu_pkt_dc5.tid)) & ~(lsu_busreq_dc4 & (lsu_pkt_dc4.tid ~^ lsu_pkt_dc5.tid)) & (lsu_pkt_dc2.load | ~addr_match_word_lo_dc5_dc2)));

   assign no_dword_merge_dc5  = lsu_busreq_dc5 & ~ldst_dual_dc5 &
                                ((lsu_busreq_dc4 & (lsu_pkt_dc4.tid ~^ lsu_pkt_dc5.tid) & (lsu_pkt_dc4.load | ~addr_match_dw_lo_dc5_dc4)) |
                                 (lsu_busreq_dc3 & (lsu_pkt_dc3.tid ~^ lsu_pkt_dc5.tid) & ~(lsu_busreq_dc4 & (lsu_pkt_dc4.tid ~^ lsu_pkt_dc5.tid)) & (lsu_pkt_dc3.load | ~addr_match_dw_lo_dc5_dc3)) |
                                 (lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc5.tid) & ~(lsu_busreq_dc3 & (lsu_pkt_dc3.tid ~^ lsu_pkt_dc5.tid))  & ~(lsu_busreq_dc4 & (lsu_pkt_dc4.tid ~^ lsu_pkt_dc5.tid))  & (lsu_pkt_dc2.load | ~addr_match_dw_lo_dc5_dc2)));

   // Create Hi/Lo signals
   assign ldst_byteen_ext_dc2[7:0] = {4'b0,ldst_byteen_dc2[3:0]} << lsu_addr_dc2[1:0];
   assign ldst_byteen_ext_dc3[7:0] = {4'b0,ldst_byteen_dc3[3:0]} << lsu_addr_dc3[1:0];
   assign ldst_byteen_ext_dc4[7:0] = {4'b0,ldst_byteen_dc4[3:0]} << lsu_addr_dc4[1:0];
   assign ldst_byteen_ext_dc5[7:0] = {4'b0,ldst_byteen_dc5[3:0]} << lsu_addr_dc5[1:0];

   assign ldst_byteen_hi_dc2[3:0]   = ldst_byteen_ext_dc2[7:4];
   assign ldst_byteen_lo_dc2[3:0]   = ldst_byteen_ext_dc2[3:0];
   assign ldst_byteen_hi_dc3[3:0]   = ldst_byteen_ext_dc3[7:4];
   assign ldst_byteen_lo_dc3[3:0]   = ldst_byteen_ext_dc3[3:0];
   assign ldst_byteen_hi_dc4[3:0]   = ldst_byteen_ext_dc4[7:4];
   assign ldst_byteen_lo_dc4[3:0]   = ldst_byteen_ext_dc4[3:0];
   assign ldst_byteen_hi_dc5[3:0]   = ldst_byteen_ext_dc5[7:4];
   assign ldst_byteen_lo_dc5[3:0]   = ldst_byteen_ext_dc5[3:0];

   assign store_data_hi_dc3[31:0]   = store_data_ext_dc3[63:32];
   assign store_data_lo_dc3[31:0]   = store_data_ext_dc3[31:0];
   assign store_data_hi_dc4[31:0]   = store_data_ext_dc4[63:32];
   assign store_data_lo_dc4[31:0]   = store_data_ext_dc4[31:0];
   assign store_data_hi_dc5[31:0]   = store_data_ext_dc5[63:32];
   assign store_data_lo_dc5[31:0]   = store_data_ext_dc5[31:0];

   assign ld_addr_dc3hit_lo_lo = (lsu_addr_dc2[31:2] == lsu_addr_dc3[31:2]) & lsu_pkt_dc3.valid & lsu_pkt_dc3.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc3.tid);
   assign ld_addr_dc3hit_lo_hi = (end_addr_dc2[31:2] == lsu_addr_dc3[31:2]) & lsu_pkt_dc3.valid & lsu_pkt_dc3.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc3.tid);
   assign ld_addr_dc3hit_hi_lo = (lsu_addr_dc2[31:2] == end_addr_dc3[31:2]) & lsu_pkt_dc3.valid & lsu_pkt_dc3.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc3.tid);
   assign ld_addr_dc3hit_hi_hi = (end_addr_dc2[31:2] == end_addr_dc3[31:2]) & lsu_pkt_dc3.valid & lsu_pkt_dc3.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc3.tid);

   assign ld_addr_dc4hit_lo_lo = (lsu_addr_dc2[31:2] == lsu_addr_dc4[31:2]) & lsu_pkt_dc4.valid & lsu_pkt_dc4.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc4.tid);
   assign ld_addr_dc4hit_lo_hi = (end_addr_dc2[31:2] == lsu_addr_dc4[31:2]) & lsu_pkt_dc4.valid & lsu_pkt_dc4.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc4.tid);
   assign ld_addr_dc4hit_hi_lo = (lsu_addr_dc2[31:2] == end_addr_dc4[31:2]) & lsu_pkt_dc4.valid & lsu_pkt_dc4.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc4.tid);
   assign ld_addr_dc4hit_hi_hi = (end_addr_dc2[31:2] == end_addr_dc4[31:2]) & lsu_pkt_dc4.valid & lsu_pkt_dc4.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc4.tid);

   assign ld_addr_dc5hit_lo_lo = (lsu_addr_dc2[31:2] == lsu_addr_dc5[31:2]) & lsu_pkt_dc5.valid & lsu_pkt_dc5.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc5.tid);
   assign ld_addr_dc5hit_lo_hi = (end_addr_dc2[31:2] == lsu_addr_dc5[31:2]) & lsu_pkt_dc5.valid & lsu_pkt_dc5.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc5.tid);
   assign ld_addr_dc5hit_hi_lo = (lsu_addr_dc2[31:2] == end_addr_dc5[31:2]) & lsu_pkt_dc5.valid & lsu_pkt_dc5.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc5.tid);
   assign ld_addr_dc5hit_hi_hi = (end_addr_dc2[31:2] == end_addr_dc5[31:2]) & lsu_pkt_dc5.valid & lsu_pkt_dc5.store & lsu_busreq_dc2 & (lsu_pkt_dc2.tid ~^ lsu_pkt_dc5.tid);

   for (genvar i=0; i<4; i++) begin
      assign ld_byte_dc3hit_lo_lo[i] = ld_addr_dc3hit_lo_lo & ldst_byteen_lo_dc3[i] & ldst_byteen_lo_dc2[i];
      assign ld_byte_dc3hit_lo_hi[i] = ld_addr_dc3hit_lo_hi & ldst_byteen_lo_dc3[i] & ldst_byteen_hi_dc2[i];
      assign ld_byte_dc3hit_hi_lo[i] = ld_addr_dc3hit_hi_lo & ldst_byteen_hi_dc3[i] & ldst_byteen_lo_dc2[i];
      assign ld_byte_dc3hit_hi_hi[i] = ld_addr_dc3hit_hi_hi & ldst_byteen_hi_dc3[i] & ldst_byteen_hi_dc2[i];

      assign ld_byte_dc4hit_lo_lo[i] = ld_addr_dc4hit_lo_lo & ldst_byteen_lo_dc4[i] & ldst_byteen_lo_dc2[i];
      assign ld_byte_dc4hit_lo_hi[i] = ld_addr_dc4hit_lo_hi & ldst_byteen_lo_dc4[i] & ldst_byteen_hi_dc2[i];
      assign ld_byte_dc4hit_hi_lo[i] = ld_addr_dc4hit_hi_lo & ldst_byteen_hi_dc4[i] & ldst_byteen_lo_dc2[i];
      assign ld_byte_dc4hit_hi_hi[i] = ld_addr_dc4hit_hi_hi & ldst_byteen_hi_dc4[i] & ldst_byteen_hi_dc2[i];

      assign ld_byte_dc5hit_lo_lo[i] = ld_addr_dc5hit_lo_lo & ldst_byteen_lo_dc5[i] & ldst_byteen_lo_dc2[i];
      assign ld_byte_dc5hit_lo_hi[i] = ld_addr_dc5hit_lo_hi & ldst_byteen_lo_dc5[i] & ldst_byteen_hi_dc2[i];
      assign ld_byte_dc5hit_hi_lo[i] = ld_addr_dc5hit_hi_lo & ldst_byteen_hi_dc5[i] & ldst_byteen_lo_dc2[i];
      assign ld_byte_dc5hit_hi_hi[i] = ld_addr_dc5hit_hi_hi & ldst_byteen_hi_dc5[i] & ldst_byteen_hi_dc2[i];

      assign ld_byte_hit_lo[i] = ld_byte_dc3hit_lo_lo[i] | ld_byte_dc3hit_hi_lo[i] |
                                 ld_byte_dc4hit_lo_lo[i] | ld_byte_dc4hit_hi_lo[i] |
                                 ld_byte_dc5hit_lo_lo[i] | ld_byte_dc5hit_hi_lo[i] |
                                 ld_byte_hit_buf_lo[lsu_pkt_dc2.tid][i];
      assign ld_byte_hit_hi[i] = ld_byte_dc3hit_lo_hi[i] | ld_byte_dc3hit_hi_hi[i] |
                                 ld_byte_dc4hit_lo_hi[i] | ld_byte_dc4hit_hi_hi[i] |
                                 ld_byte_dc5hit_lo_hi[i] | ld_byte_dc5hit_hi_hi[i] |
                                 ld_byte_hit_buf_hi[lsu_pkt_dc2.tid][i];

      assign ld_byte_dc3hit_lo[i] = ld_byte_dc3hit_lo_lo[i] | ld_byte_dc3hit_hi_lo[i];
      assign ld_byte_dc4hit_lo[i] = ld_byte_dc4hit_lo_lo[i] | ld_byte_dc4hit_hi_lo[i];
      assign ld_byte_dc5hit_lo[i] = ld_byte_dc5hit_lo_lo[i] | ld_byte_dc5hit_hi_lo[i];

      assign ld_byte_dc3hit_hi[i] = ld_byte_dc3hit_lo_hi[i] | ld_byte_dc3hit_hi_hi[i];
      assign ld_byte_dc4hit_hi[i] = ld_byte_dc4hit_lo_hi[i] | ld_byte_dc4hit_hi_hi[i];
      assign ld_byte_dc5hit_hi[i] = ld_byte_dc5hit_lo_hi[i] | ld_byte_dc5hit_hi_hi[i];

      assign ld_fwddata_dc3pipe_lo[(8*i)+7:(8*i)] = ({8{ld_byte_dc3hit_lo_lo[i]}} & store_data_lo_dc3[(8*i)+7:(8*i)]) |
                                                    ({8{ld_byte_dc3hit_hi_lo[i]}} & store_data_hi_dc3[(8*i)+7:(8*i)]);
      assign ld_fwddata_dc4pipe_lo[(8*i)+7:(8*i)] = ({8{ld_byte_dc4hit_lo_lo[i]}} & store_data_lo_dc4[(8*i)+7:(8*i)]) |
                                                    ({8{ld_byte_dc4hit_hi_lo[i]}} & store_data_hi_dc4[(8*i)+7:(8*i)]);
      assign ld_fwddata_dc5pipe_lo[(8*i)+7:(8*i)] = ({8{ld_byte_dc5hit_lo_lo[i]}} & store_data_lo_dc5[(8*i)+7:(8*i)]) |
                                                    ({8{ld_byte_dc5hit_hi_lo[i]}} & store_data_hi_dc5[(8*i)+7:(8*i)]);

      assign ld_fwddata_dc3pipe_hi[(8*i)+7:(8*i)] = ({8{ld_byte_dc3hit_lo_hi[i]}} & store_data_lo_dc3[(8*i)+7:(8*i)]) |
                                                    ({8{ld_byte_dc3hit_hi_hi[i]}} & store_data_hi_dc3[(8*i)+7:(8*i)]);
      assign ld_fwddata_dc4pipe_hi[(8*i)+7:(8*i)] = ({8{ld_byte_dc4hit_lo_hi[i]}} & store_data_lo_dc4[(8*i)+7:(8*i)]) |
                                                    ({8{ld_byte_dc4hit_hi_hi[i]}} & store_data_hi_dc4[(8*i)+7:(8*i)]);
      assign ld_fwddata_dc5pipe_hi[(8*i)+7:(8*i)] = ({8{ld_byte_dc5hit_lo_hi[i]}} & store_data_lo_dc5[(8*i)+7:(8*i)]) |
                                                    ({8{ld_byte_dc5hit_hi_hi[i]}} & store_data_hi_dc5[(8*i)+7:(8*i)]);

      // Final muxing between dc3/dc4/dc5
      assign ld_fwddata_lo[(8*i)+7:(8*i)] = ld_byte_dc3hit_lo[i]    ? ld_fwddata_dc3pipe_lo[(8*i)+7:(8*i)] :
                                            ld_byte_dc4hit_lo[i]    ? ld_fwddata_dc4pipe_lo[(8*i)+7:(8*i)] :
                                            ld_byte_dc5hit_lo[i]    ? ld_fwddata_dc5pipe_lo[(8*i)+7:(8*i)] :
                                                                      ld_fwddata_buf_lo[lsu_pkt_dc2.tid][(8*i)+7:(8*i)];

      assign ld_fwddata_hi[(8*i)+7:(8*i)] = ld_byte_dc3hit_hi[i]    ? ld_fwddata_dc3pipe_hi[(8*i)+7:(8*i)] :
                                            ld_byte_dc4hit_hi[i]    ? ld_fwddata_dc4pipe_hi[(8*i)+7:(8*i)] :
                                            ld_byte_dc5hit_hi[i]    ? ld_fwddata_dc5pipe_hi[(8*i)+7:(8*i)] :
                                                                      ld_fwddata_buf_hi[lsu_pkt_dc2.tid][(8*i)+7:(8*i)];

   end

   always_comb begin
      ld_full_hit_lo_dc2 = 1'b1;
      ld_full_hit_hi_dc2 = 1'b1;
      for (int i=0; i<4; i++) begin
         ld_full_hit_lo_dc2 &= (ld_byte_hit_lo[i] | ~ldst_byteen_lo_dc2[i]);
         ld_full_hit_hi_dc2 &= (ld_byte_hit_hi[i] | ~ldst_byteen_hi_dc2[i]);
      end
   end

   // This will be high if all the bytes of load hit the stores in pipe/write buffer (dc3/dc4/dc5/wrbuf)
   assign ld_full_hit_dc2 = ld_full_hit_lo_dc2 & ld_full_hit_hi_dc2 & lsu_busreq_dc2 & lsu_pkt_dc2.load & ~is_sideeffects_dc2;
   assign {ld_fwddata_dc2_nc[63:32], ld_fwddata_dc2[31:0]} = {ld_fwddata_hi[31:0], ld_fwddata_lo[31:0]} >> (8*lsu_addr_dc2[1:0]);
   assign bus_read_data_dc3[31:0]                          = ld_fwddata_dc3[31:0];

   // Non blocking ports
   assign lsu_nonblock_load_valid_dc1 = lsu_busreq_dc1 & lsu_pkt_dc1_pre.valid & lsu_pkt_dc1_pre.load & ~flush_dc2_up[lsu_pkt_dc1_pre.tid];
   assign lsu_nonblock_load_tag_dc1[pt.LSU_NUM_NBLOAD_WIDTH-1:0] = WrPtr0_dc1[lsu_pkt_dc1_pre.tid][pt.LSU_NUM_NBLOAD_WIDTH-1:0];
   assign lsu_nonblock_load_inv_dc2 = lsu_nonblock_load_valid_dc2 & ld_full_hit_dc2;
   assign lsu_nonblock_load_inv_tag_dc2[pt.LSU_NUM_NBLOAD_WIDTH-1:0] = WrPtr0_dc2[lsu_pkt_dc2.tid][pt.LSU_NUM_NBLOAD_WIDTH-1:0];
   assign lsu_nonblock_load_inv_dc5 = lsu_nonblock_load_valid_dc5 & ~lsu_commit_dc5;
   assign lsu_nonblock_load_inv_tag_dc5[pt.LSU_NUM_NBLOAD_WIDTH-1:0] = WrPtr0_dc5[lsu_pkt_dc5.tid][pt.LSU_NUM_NBLOAD_WIDTH-1:0];      // dc5 tag needs to be accurate even if there is no invalidate

   // Generic bus signals
   assign bus_cmd_ready                      = obuf_write[bus_tid] ? ((obuf_cmd_done[bus_tid] | obuf_data_done[bus_tid]) ? (obuf_cmd_done[bus_tid] ? lsu_axi_wready : lsu_axi_awready) : (lsu_axi_awready & lsu_axi_wready)) : lsu_axi_arready;
   assign bus_cmd_valid                      = lsu_axi_awvalid | lsu_axi_wvalid | lsu_axi_arvalid;
   assign bus_wcmd_sent                      = lsu_axi_awvalid & lsu_axi_awready;
   assign bus_wdata_sent                     = lsu_axi_wvalid & lsu_axi_wready;
   assign bus_cmd_sent                       = ((obuf_cmd_done[bus_tid] | bus_wcmd_sent) & (obuf_data_done[bus_tid] | bus_wdata_sent)) | (lsu_axi_arvalid & lsu_axi_arready);

   assign bus_rsp_read                       = lsu_axi_rvalid & lsu_axi_rready;
   assign bus_rsp_write                      = lsu_axi_bvalid & lsu_axi_bready;
   assign bus_rsp_read_tag[pt.LSU_BUS_TAG-1:0]  = lsu_axi_rid[pt.LSU_BUS_TAG-2:0];
   assign bus_rsp_write_tag[pt.LSU_BUS_TAG-1:0] = lsu_axi_bid[pt.LSU_BUS_TAG-2:0];
   assign bus_rsp_write_error                = bus_rsp_write & (lsu_axi_bresp[1:0] != 2'b0);
   assign bus_rsp_read_error                 = bus_rsp_read  & (lsu_axi_rresp[1:0] != 2'b0);
   assign bus_rsp_rdata[63:0]                = lsu_axi_rdata[63:0];
   assign bus_rsp_read_tid                   = lsu_axi_rid[pt.LSU_BUS_TAG-1];
   assign bus_rsp_write_tid                  = lsu_axi_bid[pt.LSU_BUS_TAG-1];

   assign bus_rsp_write_tid_q                = lsu_axi_bid_q[pt.LSU_BUS_TAG-1];

   // AXI command signals
   assign lsu_axi_awvalid               = obuf_valid[bus_tid] & obuf_write[bus_tid] & ~obuf_cmd_done[bus_tid] & ~bus_addr_match_pending[bus_tid];
   assign lsu_axi_awid[pt.LSU_BUS_TAG-1:0] = (pt.LSU_BUS_TAG)'({bus_tid,obuf_tag0[bus_tid][pt.LSU_BUS_TAG-2:0]});
   assign lsu_axi_awaddr[31:0]          = obuf_sideeffect[bus_tid] ? obuf_addr[bus_tid][31:0] : {obuf_addr[bus_tid][31:3],3'b0};
   assign lsu_axi_awsize[2:0]           = obuf_sideeffect[bus_tid] ? {1'b0, obuf_sz[bus_tid][1:0]} : 3'b011;
   assign lsu_axi_awprot[2:0]           = '0;
   assign lsu_axi_awcache[3:0]          = obuf_sideeffect[bus_tid]? 4'b0 : 4'b1111;
   assign lsu_axi_awregion[3:0]         = obuf_addr[bus_tid][31:28];
   assign lsu_axi_awlen[7:0]            = '0;
   assign lsu_axi_awburst[1:0]          = 2'b01;
   assign lsu_axi_awqos[3:0]            = '0;
   assign lsu_axi_awlock                = '0;

   assign lsu_axi_wvalid                = obuf_valid[bus_tid] & obuf_write[bus_tid] & ~obuf_data_done[bus_tid] & ~bus_addr_match_pending[bus_tid];
   assign lsu_axi_wstrb[7:0]            = obuf_byteen[bus_tid][7:0] & {8{obuf_write[bus_tid]}};
   assign lsu_axi_wdata[63:0]           = obuf_data[bus_tid][63:0];
   assign lsu_axi_wlast                 = '1;

   assign lsu_axi_arvalid               = obuf_valid[bus_tid] & ~obuf_nosend[bus_tid] & ~obuf_write[bus_tid] & ~bus_addr_match_pending[bus_tid];
   assign lsu_axi_arid[pt.LSU_BUS_TAG-1:0] = (pt.LSU_BUS_TAG)'({bus_tid,obuf_tag0[bus_tid][pt.LSU_BUS_TAG-2:0]});
   assign lsu_axi_araddr[31:0]          = obuf_sideeffect[bus_tid] ? obuf_addr[bus_tid][31:0] : {obuf_addr[bus_tid][31:3],3'b0};
   assign lsu_axi_arsize[2:0]           = obuf_sideeffect[bus_tid] ? {1'b0, obuf_sz[bus_tid][1:0]} : 3'b011;
   assign lsu_axi_arprot[2:0]           = '0;
   assign lsu_axi_arcache[3:0]          = obuf_sideeffect[bus_tid] ? 4'b0 : 4'b1111;
   assign lsu_axi_arregion[3:0]         = obuf_addr[bus_tid][31:28];
   assign lsu_axi_arlen[7:0]            = '0;
   assign lsu_axi_arburst[1:0]          = 2'b01;
   assign lsu_axi_arqos[3:0]            = '0;
   assign lsu_axi_arlock                = '0;

   assign lsu_axi_bready = 1;
   assign lsu_axi_rready = 1;

   // Count the number of pending trxns for fence
   assign bus_pend_trxnQ[pt.NUM_THREADS-1:0]    = '0;
   assign bus_pend_trxn[pt.NUM_THREADS-1:0]     = '0;
   assign lsu_bus_cntr_overflow[pt.NUM_THREADS-1:0] = '0;
   assign lsu_bus_idle_any[pt.NUM_THREADS-1:0]  = {pt.NUM_THREADS{1'b1}};

   // PMU signals
   for (genvar i=0; i<pt.NUM_THREADS; i++) begin: GenPMU
      assign lsu_pmu_bus_trxn[i]       = ((lsu_axi_awvalid & lsu_axi_awready) | (lsu_axi_wvalid & lsu_axi_wready) | (lsu_axi_arvalid & lsu_axi_arready)) & (i == bus_tid);
      assign lsu_pmu_bus_misaligned[i] = lsu_busreq_dc5 & ldst_dual_dc5 & lsu_commit_dc5 & (i == lsu_pkt_dc5.tid);
      assign lsu_pmu_bus_error[i]      = lsu_imprecise_error_load_any[i] | lsu_imprecise_error_store_any[i];
      assign lsu_pmu_bus_busy[i]       = ((lsu_axi_awvalid & ~lsu_axi_awready) | (lsu_axi_wvalid & ~lsu_axi_wready) | (lsu_axi_arvalid & ~lsu_axi_arready)) & (i == bus_tid);
   end

   rvdff #(.WIDTH(1))               lsu_axi_awvalid_ff (.din(lsu_axi_awvalid),                .dout(lsu_axi_awvalid_q),                .clk(lsu_busm_clk), .*);
   rvdff #(.WIDTH(1))               lsu_axi_awready_ff (.din(lsu_axi_awready),                .dout(lsu_axi_awready_q),                .clk(lsu_busm_clk), .*);
   rvdff #(.WIDTH(1))               lsu_axi_wvalid_ff  (.din(lsu_axi_wvalid),                 .dout(lsu_axi_wvalid_q),                 .clk(lsu_busm_clk), .*);
   rvdff #(.WIDTH(1))               lsu_axi_wready_ff  (.din(lsu_axi_wready),                 .dout(lsu_axi_wready_q),                 .clk(lsu_busm_clk), .*);
   rvdff #(.WIDTH(1))               lsu_axi_arvalid_ff (.din(lsu_axi_arvalid),                .dout(lsu_axi_arvalid_q),                .clk(lsu_busm_clk), .*);
   rvdff #(.WIDTH(1))               lsu_axi_arready_ff (.din(lsu_axi_arready),                .dout(lsu_axi_arready_q),                .clk(lsu_busm_clk), .*);

   rvdff  #(.WIDTH(1))              lsu_axi_bvalid_ff  (.din(lsu_axi_bvalid),                 .dout(lsu_axi_bvalid_q),                 .clk(lsu_busm_clk), .*);
   rvdff  #(.WIDTH(1))              lsu_axi_bready_ff  (.din(lsu_axi_bready),                 .dout(lsu_axi_bready_q),                 .clk(lsu_busm_clk), .*);
   rvdff  #(.WIDTH(2))              lsu_axi_bresp_ff   (.din(lsu_axi_bresp[1:0]),             .dout(lsu_axi_bresp_q[1:0]),             .clk(lsu_busm_clk), .*);
   rvdff  #(.WIDTH(pt.LSU_BUS_TAG)) lsu_axi_bid_ff     (.din(lsu_axi_bid[pt.LSU_BUS_TAG-1:0]),.dout(lsu_axi_bid_q[pt.LSU_BUS_TAG-1:0]),.clk(lsu_busm_clk), .*);
   rvdffe #(.WIDTH(64))             lsu_axi_rdata_ff   (.din(lsu_axi_rdata[63:0]),            .dout(lsu_axi_rdata_q[63:0]),            .en(lsu_axi_rvalid & lsu_bus_clk_en), .*);

   rvdff  #(.WIDTH(1))              lsu_axi_rvalid_ff  (.din(lsu_axi_rvalid),                 .dout(lsu_axi_rvalid_q),                 .clk(lsu_busm_clk), .*);
   rvdff  #(.WIDTH(1))              lsu_axi_rready_ff  (.din(lsu_axi_rready),                 .dout(lsu_axi_rready_q),                 .clk(lsu_busm_clk), .*);
   rvdff  #(.WIDTH(2))              lsu_axi_rresp_ff   (.din(lsu_axi_rresp[1:0]),             .dout(lsu_axi_rresp_q[1:0]),             .clk(lsu_busm_clk), .*);
   rvdff  #(.WIDTH(pt.LSU_BUS_TAG)) lsu_axi_rid_ff     (.din(lsu_axi_rid[pt.LSU_BUS_TAG-1:0]),.dout(lsu_axi_rid_q[pt.LSU_BUS_TAG-1:0]),.clk(lsu_busm_clk), .*);

   // Per thread bus buffer
   for (genvar i=0; i<pt.NUM_THREADS; i++) begin: GenThreadLoop
      // Read/Write Buffer
      eh2_lsu_bus_buffer #(.pt(pt)) bus_buffer (
         .tid(1'(i)),
         .lsu_bus_ibuf_c1_clk(lsu_bus_ibuf_c1_clk[i]),
         .lsu_bus_buf_c1_clk(lsu_bus_buf_c1_clk[i]),
         .lsu_bus_obuf_c1_clk(lsu_bus_obuf_c1_clk[i]),
         .dec_tlu_force_halt(dec_tlu_force_halt[i]),
         .lsu_bus_cntr_overflow(lsu_bus_cntr_overflow[i]),
         .lsu_bus_idle_any(lsu_bus_idle_any[i]),

         .bus_addr_match_pending(bus_addr_match_pending[i]),
         .lsu_bus_buffer_pend_any(lsu_bus_buffer_pend_any[i]),
         .lsu_bus_buffer_full_any(lsu_bus_buffer_full_any[i]),
         .lsu_bus_buffer_empty_any(lsu_bus_buffer_empty_any[i]),

         .ld_byte_hit_buf_lo(ld_byte_hit_buf_lo[i]),
         .ld_byte_hit_buf_hi(ld_byte_hit_buf_hi[i]),
         .ld_fwddata_buf_lo(ld_fwddata_buf_lo[i]),
         .ld_fwddata_buf_hi(ld_fwddata_buf_hi[i]),

         .lsu_imprecise_error_load_any(lsu_imprecise_error_load_any[i]),
         .lsu_imprecise_error_store_any(lsu_imprecise_error_store_any[i]),
         .lsu_imprecise_error_addr_any(lsu_imprecise_error_addr_any[i]),

         .WrPtr0_dc1(WrPtr0_dc1[i]),
         .WrPtr0_dc2(WrPtr0_dc2[i]),
         .WrPtr0_dc5(WrPtr0_dc5[i]),

         .obuf_valid(obuf_valid[i]),
         .obuf_nosend(obuf_nosend[i]),
         .obuf_write(obuf_write[i]),
         .obuf_sideeffect(obuf_sideeffect[i]),
         .obuf_addr(obuf_addr[i]),
         .obuf_data(obuf_data[i]),
         .obuf_sz(obuf_sz[i]),
         .obuf_byteen(obuf_byteen[i]),
         .obuf_cmd_done(obuf_cmd_done[i]),
         .obuf_data_done(obuf_data_done[i]),
         .obuf_tag0(obuf_tag0[i]),
         .obuf_nxtready(obuf_nxtready[i]),

         .lsu_nonblock_load_data_ready(tid_nonblock_load_data_ready[i]),
         .lsu_nonblock_load_data_valid(tid_nonblock_load_data_valid[i]),
         .lsu_nonblock_load_data_error(tid_nonblock_load_data_error[i]),
         .lsu_nonblock_load_data_tag(tid_nonblock_load_data_tag[i]),
         .lsu_nonblock_load_data(tid_nonblock_load_data[i]),

         .*
      );
   end

   // // Only one thread will take imprecise error at a time
   // always_comb begin
   //    lsu_imprecise_error_addr_any  = '0;
   //    for (int i=0; i<pt.NUM_THREADS; i++) begin
   //       lsu_imprecise_error_addr_any[31:0] |= {32{(lsu_imprecise_error_load_any[i] | lsu_imprecise_error_store_any[i])}} & tid_imprecise_error_addr_any[i];
   //    end
   // end

   always_comb begin
      lsu_nonblock_load_data_valid = '0;
      lsu_nonblock_load_data_error = '0;
      lsu_nonblock_load_data_tag   = '0;
      lsu_nonblock_load_data       = '0;
      for (int i=0; i<pt.NUM_THREADS; i++) begin
         lsu_nonblock_load_data_valid |= (tid_nonblock_load_data_valid[i] & (lsu_nonblock_load_data_tid == i));
         lsu_nonblock_load_data_error |= (tid_nonblock_load_data_error[i] & (lsu_nonblock_load_data_tid == i));
         lsu_nonblock_load_data_tag   |= {(pt.LSU_NUM_NBLOAD_WIDTH){lsu_nonblock_load_data_tid == i}} & tid_nonblock_load_data_tag[i];
         lsu_nonblock_load_data       |= {32{lsu_nonblock_load_data_tid == i}} & tid_nonblock_load_data[i];
      end
   end

   // Thread arbitration logic for bus tid
   if (pt.NUM_THREADS == 2) begin: GenMT
      assign nxt_bus_tid = bus_tid ? (~(obuf_nxtready[0] | obuf_valid[0]) & obuf_nxtready[1]) :
                                     (~obuf_nxtready[0] | (obuf_nxtready[1] | obuf_valid[1]));
      assign bus_tid_en  = bus_cmd_sent | (~bus_cmd_valid & (obuf_nxtready[0] | obuf_nxtready[1]) & ~obuf_nxtready[bus_tid]) | (~obuf_valid[bus_tid] & obuf_valid[~bus_tid]);
      rvdffs #(.WIDTH(1)) bus_tidff (.din(nxt_bus_tid), .dout(bus_tid), .en(bus_tid_en), .clk(lsu_busm_clk), .*);
   end else begin: GenST
      assign bus_tid = 1'b0;
   end

   // Thread arbitration logic for nonblock tid
   if (pt.NUM_THREADS == 2) begin: GenNBTID_MT
      rvarbiter2 nbtid_arbiter (
         .ready(tid_nonblock_load_data_ready[1:0]),
         .shift(lsu_nonblock_load_data_valid | lsu_nonblock_load_data_tid),
         .tid  (lsu_nonblock_load_data_tid),   //output
         .*
      );
   end else begin: GenNBTID_ST
      assign lsu_nonblock_load_data_tid = '0;
   end

   // Fifo flops
   rvdff #(.WIDTH(32)) lsu_fwddata_dc3ff (.din(ld_fwddata_dc2[31:0]), .dout(ld_fwddata_dc3[31:0]), .clk(lsu_c1_dc3_clk), .*);

   rvdff #(.WIDTH(1)) clken_ff (.din(lsu_bus_clk_en), .dout(lsu_bus_clk_en_q), .clk(free_clk), .*);

   rvdff #(.WIDTH(1)) ldst_dual_dc2ff (.din(ldst_dual_dc1), .dout(ldst_dual_dc2), .clk(lsu_c1_dc2_clk), .*);
   rvdff #(.WIDTH(1)) ldst_dual_dc3ff (.din(ldst_dual_dc2), .dout(ldst_dual_dc3), .clk(lsu_c1_dc3_clk),  .*);
   rvdff #(.WIDTH(1)) ldst_dual_dc4ff (.din(ldst_dual_dc3), .dout(ldst_dual_dc4), .clk(lsu_c1_dc4_clk), .*);
   rvdff #(.WIDTH(1)) ldst_dual_dc5ff (.din(ldst_dual_dc4), .dout(ldst_dual_dc5), .clk(lsu_c1_dc5_clk), .*);
   rvdff #(.WIDTH(1)) is_sideeffects_dc4ff (.din(is_sideeffects_dc3), .dout(is_sideeffects_dc4), .clk(lsu_c1_dc4_clk), .*);
   rvdff #(.WIDTH(1)) is_sideeffects_dc5ff (.din(is_sideeffects_dc4), .dout(is_sideeffects_dc5), .clk(lsu_c1_dc5_clk), .*);

   rvdff #(4) lsu_byten_dc3ff (.*, .din(ldst_byteen_dc2[3:0]), .dout(ldst_byteen_dc3[3:0]), .clk(lsu_c1_dc3_clk));
   rvdff #(4) lsu_byten_dc4ff (.*, .din(ldst_byteen_dc3[3:0]), .dout(ldst_byteen_dc4[3:0]), .clk(lsu_c1_dc4_clk));
   rvdff #(4) lsu_byten_dc5ff (.*, .din(ldst_byteen_dc4[3:0]), .dout(ldst_byteen_dc5[3:0]), .clk(lsu_c1_dc5_clk));

   rvdff #(.WIDTH(1)) lsu_busreq_dc2ff (.din(lsu_busreq_dc1), .dout(lsu_busreq_dc2), .clk(lsu_c2_dc2_clk), .*);  // Don't want dc2 to dc3 propagation during freeze.
   rvdff #(.WIDTH(1)) lsu_busreq_dc3ff (.din(lsu_busreq_dc2 & ~ld_full_hit_dc2), .dout(lsu_busreq_dc3), .clk(lsu_c2_dc3_clk), .*);  // Don't want dc2 to dc3 propagation during freeze.
   rvdff #(.WIDTH(1)) lsu_busreq_dc4ff (.din(lsu_busreq_dc3 & ~flush_dc3[lsu_pkt_dc3.tid]),      .dout(lsu_busreq_dc4), .clk(lsu_c2_dc4_clk), .*);
   rvdff #(.WIDTH(1)) lsu_busreq_dc5ff (.din(lsu_busreq_dc4 & ~flush_dc4[lsu_pkt_dc4.tid]),      .dout(lsu_busreq_dc5), .clk(lsu_c2_dc5_clk), .*);

   rvdff #(.WIDTH(1)) lsu_nonblock_load_valid_dc2ff  (.din(lsu_nonblock_load_valid_dc1),  .dout(lsu_nonblock_load_valid_dc2), .clk(lsu_c2_dc2_clk), .*);
   rvdff #(.WIDTH(1)) lsu_nonblock_load_valid_dc3ff  (.din(lsu_nonblock_load_valid_dc2),  .dout(lsu_nonblock_load_valid_dc3), .clk(lsu_c2_dc3_clk), .*);
   rvdff #(.WIDTH(1)) lsu_nonblock_load_valid_dc4ff  (.din(lsu_nonblock_load_valid_dc3),  .dout(lsu_nonblock_load_valid_dc4), .clk(lsu_c2_dc4_clk), .*);
   rvdff #(.WIDTH(1)) lsu_nonblock_load_valid_dc5ff  (.din(lsu_nonblock_load_valid_dc4),  .dout(lsu_nonblock_load_valid_dc5), .clk(lsu_c2_dc5_clk), .*);

`ifdef ASSERT_ON

   // Assertion to check AXI write address is aligned to size
   property lsu_axi_awaddr_aligned;
     @(posedge lsu_busm_clk) disable iff(~rst_l) lsu_axi_awvalid |-> ((lsu_axi_awsize[2:0] == 3'h0)                                   |
                                                                      ((lsu_axi_awsize[2:0] == 3'h1) & (lsu_axi_awaddr[0] == 1'b0))   |
                                                                      ((lsu_axi_awsize[2:0] == 3'h2) & (lsu_axi_awaddr[1:0] == 2'b0)) |
                                                                      ((lsu_axi_awsize[2:0] == 3'h3) & (lsu_axi_awaddr[2:0] == 3'b0)));
   endproperty
   assert_lsu_axi_awaddr_aligned: assert property (lsu_axi_awaddr_aligned) else
     $display("Assertion lsu_axi_awaddr_aligned failed: lsu_axi_awvalid=1'b%b, lsu_axi_awsize=3'h%h, lsu_axi_awaddr=32'h%h",lsu_axi_awvalid, lsu_axi_awsize[2:0], lsu_axi_awaddr[31:0]);
   // Assertion to check awvalid stays stable during entire bus clock

   // Assertion to check AXI read address is aligned to size
   property lsu_axi_araddr_aligned;
     @(posedge lsu_busm_clk) disable iff(~rst_l) lsu_axi_awvalid |-> ((lsu_axi_arsize[2:0] == 3'h0)                                   |
                                                                      ((lsu_axi_arsize[2:0] == 3'h1) & (lsu_axi_araddr[0] == 1'b0))   |
                                                                      ((lsu_axi_arsize[2:0] == 3'h2) & (lsu_axi_araddr[1:0] == 2'b0)) |
                                                                      ((lsu_axi_arsize[2:0] == 3'h3) & (lsu_axi_araddr[2:0] == 3'b0)));
   endproperty
   assert_lsu_axi_araddr_aligned: assert property (lsu_axi_araddr_aligned) else
     $display("Assertion lsu_axi_araddr_aligned failed: lsu_axi_awvalid=1'b%b, lsu_axi_awsize=3'h%h, lsu_axi_araddr=32'h%h",lsu_axi_awvalid, lsu_axi_awsize[2:0], lsu_axi_araddr[31:0]);

   // Assertion to check awvalid stays stable during entire bus clock
  property lsu_axi_awvalid_stable;
      @(posedge clk) disable iff(~rst_l)  (lsu_axi_awvalid != $past(lsu_axi_awvalid)) |-> $past(lsu_bus_clk_en);
   endproperty
   assert_lsu_axi_awvalid_stable: assert property (lsu_axi_awvalid_stable) else
      $display("LSU AXI awvalid changed in middle of bus clock");

   // Assertion to check awid stays stable during entire bus clock
   property lsu_axi_awid_stable;
      @(posedge clk) disable iff(~rst_l)  (lsu_axi_awvalid & (lsu_axi_awid[pt.LSU_BUS_TAG-1:0] != $past(lsu_axi_awid[pt.LSU_BUS_TAG-1:0]))) |-> $past(lsu_bus_clk_en);
   endproperty
   assert_lsu_axi_awid_stable: assert property (lsu_axi_awid_stable) else
      $display("LSU AXI awid changed in middle of bus clock");

   // Assertion to check awaddr stays stable during entire bus clock
   property lsu_axi_awaddr_stable;
      @(posedge clk) disable iff(~rst_l)  (lsu_axi_awvalid & (lsu_axi_awaddr[31:0] != $past(lsu_axi_awaddr[31:0]))) |-> $past(lsu_bus_clk_en);
   endproperty
   assert_lsu_axi_awaddr_stable: assert property (lsu_axi_awaddr_stable) else
      $display("LSU AXI awaddr changed in middle of bus clock");

   // Assertion to check awsize stays stable during entire bus clock
   property lsu_axi_awsize_stable;
      @(posedge clk) disable iff(~rst_l)  (lsu_axi_awvalid & (lsu_axi_awsize[2:0] != $past(lsu_axi_awsize[2:0]))) |-> $past(lsu_bus_clk_en);
   endproperty
   assert_lsu_axi_awsize_stable: assert property (lsu_axi_awsize_stable) else
      $display("LSU AXI awsize changed in middle of bus clock");

   // Assertion to check wstrb stays stable during entire bus clock
   property lsu_axi_wstrb_stable;
      @(posedge clk) disable iff(~rst_l)  (lsu_axi_wvalid & (lsu_axi_wstrb[7:0] != $past(lsu_axi_wstrb[7:0]))) |-> $past(lsu_bus_clk_en);
   endproperty
   assert_lsu_axi_wstrb_stable: assert property (lsu_axi_wstrb_stable) else
      $display("LSU AXI wstrb changed in middle of bus clock");

   // Assertion to check wdata stays stable during entire bus clock
   property lsu_axi_wdata_stable;
      @(posedge clk) disable iff(~rst_l)  (lsu_axi_wvalid & (lsu_axi_wdata[63:0] != $past(lsu_axi_wdata[63:0]))) |-> $past(lsu_bus_clk_en);
   endproperty
   assert_lsu_axi_wdata_stable: assert property (lsu_axi_wdata_stable) else
      $display("LSU AXI command wdata changed in middle of bus clock");

   // Assertion to check awvalid stays stable during entire bus clock
   property lsu_axi_arvalid_stable;
      @(posedge clk) disable iff(~rst_l)  (lsu_axi_arvalid != $past(lsu_axi_arvalid)) |-> $past(lsu_bus_clk_en);
   endproperty
   assert_lsu_axi_arvalid_stable: assert property (lsu_axi_arvalid_stable) else
      $display("LSU AXI awvalid changed in middle of bus clock");

   // Assertion to check awid stays stable during entire bus clock
   property lsu_axi_arid_stable;
      @(posedge clk) disable iff(~rst_l)  (lsu_axi_arvalid & (lsu_axi_arid[pt.LSU_BUS_TAG-1:0] != $past(lsu_axi_arid[pt.LSU_BUS_TAG-1:0]))) |-> $past(lsu_bus_clk_en);
   endproperty
   assert_lsu_axi_arid_stable: assert property (lsu_axi_arid_stable) else
      $display("LSU AXI awid changed in middle of bus clock");

   // Assertion to check awaddr stays stable during entire bus clock
   property lsu_axi_araddr_stable;
      @(posedge clk) disable iff(~rst_l)  (lsu_axi_arvalid & (lsu_axi_araddr[31:0] != $past(lsu_axi_araddr[31:0]))) |-> $past(lsu_bus_clk_en);
   endproperty
   assert_lsu_axi_araddr_stable: assert property (lsu_axi_araddr_stable) else
      $display("LSU AXI awaddr changed in middle of bus clock");

   // Assertion to check awsize stays stable during entire bus clock
   property lsu_axi_arsize_stable;
      @(posedge clk) disable iff(~rst_l)  (lsu_axi_arvalid & (lsu_axi_arsize[2:0] != $past(lsu_axi_arsize[2:0]))) |-> $past(lsu_bus_clk_en);
   endproperty
   assert_lsu_axi_arsize_stable: assert property (lsu_axi_arsize_stable) else
      $display("LSU AXI awsize changed in middle of bus clock");

`endif

endmodule // lsu_bus_intf
