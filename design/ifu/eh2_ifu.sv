//********************************************************************************
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
//********************************************************************************
// Function: Top level file for Icache, Fetch, Branch prediction & Aligner
// BFF -> F1 -> F2 -> A
//********************************************************************************

module eh2_ifu
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (
   input logic free_clk,
   input logic active_clk,
   input logic clk,
   input logic clk_override,
   input logic rst_l,

   input logic [pt.NUM_THREADS-1:0]        dec_i1_cancel_e1,

   input logic [pt.NUM_THREADS-1:0]        dec_ib3_valid_d,           // ib3 buffer valid
   input logic [pt.NUM_THREADS-1:0]        dec_ib2_valid_d,           // ib2 buffer valid

   input logic dec_i0_tid_e4, // needed to maintain RS in BP
   input logic dec_i1_tid_e4,

   input logic        exu_i0_br_ret_e4,  // i0 branch commit is a ret
   input logic        exu_i1_br_ret_e4,  // i1 branch commit is a ret
   input logic        exu_i0_br_call_e4, // i0 branch commit is a call
   input logic        exu_i1_br_call_e4, // i1 branch commit is a call

   input logic [pt.NUM_THREADS-1:0][31:1] exu_flush_path_final, // flush fetch address
   input logic [31:0]  dec_tlu_mrac_ff ,// Side_effect , cacheable for each region

   input logic                         dec_tlu_bpred_disable, // disable all branch prediction
   input logic                         dec_tlu_core_ecc_disable,  // disable ecc checking and flagging

// Threaded signals

   input logic [pt.NUM_THREADS-1:0]        exu_flush_final,
   input logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_err_wb , // flush due to parity error.
   input logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_noredir_wb, // don't fetch, validated with exu_flush_final
   input logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_lower_wb, //
   input logic [pt.NUM_THREADS-1:0]        dec_tlu_fence_i_wb, //
   input logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_leak_one_wb, // ignore bp for leak one fetches
   input logic [pt.NUM_THREADS-1:0]        dec_tlu_force_halt , // force halt.

  //-------------------------- IFU AXI signals--------------------------
   // AXI Write Channels
   output logic                            ifu_axi_awvalid,
   output logic [pt.IFU_BUS_TAG-1:0]       ifu_axi_awid,
   output logic [31:0]                     ifu_axi_awaddr,
   output logic [3:0]                      ifu_axi_awregion,
   output logic [7:0]                      ifu_axi_awlen,
   output logic [2:0]                      ifu_axi_awsize,
   output logic [1:0]                      ifu_axi_awburst,
   output logic                            ifu_axi_awlock,
   output logic [3:0]                      ifu_axi_awcache,
   output logic [2:0]                      ifu_axi_awprot,
   output logic [3:0]                      ifu_axi_awqos,

   output logic                            ifu_axi_wvalid,
   output logic [63:0]                     ifu_axi_wdata,
   output logic [7:0]                      ifu_axi_wstrb,
   output logic                            ifu_axi_wlast,

   output logic                            ifu_axi_bready,

   // AXI Read Channels
   output logic                            ifu_axi_arvalid,
   input  logic                            ifu_axi_arready,
   output logic [pt.IFU_BUS_TAG-1:0]       ifu_axi_arid,
   output logic [31:0]                     ifu_axi_araddr,
   output logic [3:0]                      ifu_axi_arregion,
   output logic [7:0]                      ifu_axi_arlen,
   output logic [2:0]                      ifu_axi_arsize,
   output logic [1:0]                      ifu_axi_arburst,
   output logic                            ifu_axi_arlock,
   output logic [3:0]                      ifu_axi_arcache,
   output logic [2:0]                      ifu_axi_arprot,
   output logic [3:0]                      ifu_axi_arqos,

   input  logic                            ifu_axi_rvalid,
   output logic                            ifu_axi_rready,
   input  logic [pt.IFU_BUS_TAG-1:0]       ifu_axi_rid,
   input  logic [63:0]                     ifu_axi_rdata,
   input  logic [1:0]                      ifu_axi_rresp,


   input  logic                         ifu_bus_clk_en,

   input  logic                      dma_iccm_req,
   input  logic [2:0]                dma_mem_tag,
   input  logic [31:0]               dma_mem_addr,
   input  logic [2:0]                dma_mem_sz,
   input  logic                      dma_mem_write,
   input  logic [63:0]               dma_mem_wdata,
   input  logic                      dma_iccm_stall_any,


   output logic                      iccm_dma_ecc_error,
   output logic                      iccm_dma_rvalid,
   output logic [2:0]                iccm_dma_rtag,
   output logic [63:0]               iccm_dma_rdata,
   output logic                      iccm_ready,

   output logic [pt.NUM_THREADS-1:0][1:0] ifu_pmu_instr_aligned,
   output logic [pt.NUM_THREADS-1:0]      ifu_pmu_align_stall,

   output logic [pt.NUM_THREADS-1:0] ifu_pmu_fetch_stall,

//   I$ & ITAG Ports
   output logic [31:1]               ic_rw_addr,         // Read/Write addresss to the Icache.
   output logic [pt.ICACHE_NUM_WAYS-1:0]                ic_wr_en,           // Icache write enable, when filling the Icache.
   output logic                      ic_rd_en,           // Icache read  enable.

   output logic [pt.ICACHE_BANKS_WAY-1:0] [70:0]               ic_wr_data,           // Data to fill to the Icache. With ECC
   input  logic [63:0]               ic_rd_data ,          // Data read from Icache. 2x64bits + parity bits. F2 stage. With ECC
   input  logic [70:0]               ic_debug_rd_data ,    // Data read from Icache. 2x64bits + parity bits. F2 stage. With ECC
   input  logic [25:0]               ictag_debug_rd_data,  // Debug icache tag.
   output logic [70:0]               ic_debug_wr_data,     // Debug wr cache.
   output logic [70:0]               ifu_ic_debug_rd_data, // debug data read

   input  logic [pt.ICACHE_BANKS_WAY-1:0] ic_eccerr,    //
   input  logic [pt.ICACHE_BANKS_WAY-1:0] ic_parerr,



   output logic [63:0]               ic_premux_data,     // Premux data to be muxed with each way of the Icache.
   output logic                      ic_sel_premux_data, // Select the premux data.

   output logic [pt.ICACHE_INDEX_HI:3]  ic_debug_addr,      // Read/Write addresss to the Icache.
   output logic                         ic_debug_rd_en,     // Icache debug rd
   output logic                         ic_debug_wr_en,     // Icache debug wr
   output logic                         ic_debug_tag_array, // Debug tag array
   output logic [pt.ICACHE_NUM_WAYS-1:0]ic_debug_way,       // Debug way. Rd or Wr.


   output logic [pt.ICACHE_NUM_WAYS-1:0]                ic_tag_valid,       // Valid bits when accessing the Icache. One valid bit per way. F2 stage

   input  logic [pt.ICACHE_NUM_WAYS-1:0]                ic_rd_hit,          // Compare hits from Icache tags. Per way.  F2 stage
   input  logic                      ic_tag_perr,        // Icache Tag parity error


   // ICCM ports
   output logic [pt.ICCM_BITS-1:1]   iccm_rw_addr,                        // ICCM read/write address.
   output logic [pt.NUM_THREADS-1:0] iccm_buf_correct_ecc_thr,            // ICCM is doing a single bit error correct cycle
   output logic                      iccm_stop_fetch,                     // We have fetched 4 bytes. Dont consider any hits for lru replacements
   output logic                      iccm_correction_state,               // We are under a correction - This is needed to guard replacements when hit
   output logic                      iccm_corr_scnd_fetch,                // dont match on middle bank when under correction

   output logic                      ifc_select_tid_f1,  // Thread reading ICCM. Used for error redundancy logic
   output logic                      iccm_wren,          // ICCM write enable (through the DMA)
   output logic                      iccm_rden,          // ICCM read enable.
   output logic [77:0]               iccm_wr_data,       // ICCM write data.
   output logic [2:0]                iccm_wr_size,       // ICCM write location within DW.

   input  logic [63:0]               iccm_rd_data,       // Data read from ICCM.
   input  logic [116:0]              iccm_rd_data_ecc,   // Data read from ICCM.

   output logic [pt.NUM_THREADS-1:0] ifu_pmu_ic_miss,               // IC miss event
   output logic [pt.NUM_THREADS-1:0] ifu_pmu_ic_hit,                // IC hit event
   output logic [pt.NUM_THREADS-1:0] ifu_pmu_bus_error,             // Bus error event
   output logic [pt.NUM_THREADS-1:0] ifu_pmu_bus_busy,              // Bus busy event
   output logic [pt.NUM_THREADS-1:0] ifu_pmu_bus_trxn,              // Bus transaction

   output logic  [pt.NUM_THREADS-1:0] ifu_i0_valid,        // Instruction 0 valid. From Aligner to Decode
   output logic  [pt.NUM_THREADS-1:0] ifu_i1_valid,        // Instruction 1 valid. From Aligner to Decode
   output logic  [pt.NUM_THREADS-1:0] ifu_i0_icaf,         // Instruction 0 access fault. From Aligner to Decode

   output logic  [pt.NUM_THREADS-1:0] [1:0]  ifu_i0_icaf_type, // Instruction 0 access fault type

   output logic  [pt.NUM_THREADS-1:0] ifu_i0_icaf_f1,      // Instruction 0 has access fault on second fetch group
   output logic  [pt.NUM_THREADS-1:0] ifu_i0_dbecc,        // Instruction 0 has double bit ecc error
   output logic                     iccm_dma_sb_error,   // Single Bit ECC error from a DMA access
   output logic  [pt.NUM_THREADS-1:0] [31:0] ifu_i0_instr,   // Instruction 0 . From Aligner to Decode
   output logic  [pt.NUM_THREADS-1:0] [31:0] ifu_i1_instr,   // Instruction 1 . From Aligner to Decode
   output logic  [pt.NUM_THREADS-1:0] [31:1] ifu_i0_pc,      // Instruction 0 pc. From Aligner to Decode
   output logic  [pt.NUM_THREADS-1:0] [31:1] ifu_i1_pc,      // Instruction 1 pc. From Aligner to Decode
   output logic  [pt.NUM_THREADS-1:0] ifu_i0_pc4,           // Instruction 0 is 4 byte. From Aligner to Decode
   output logic  [pt.NUM_THREADS-1:0] ifu_i1_pc4,           // Instruction 1 is 4 byte. From Aligner to Decode
   output eh2_predecode_pkt_t  [pt.NUM_THREADS-1:0] ifu_i0_predecode,
   output eh2_predecode_pkt_t  [pt.NUM_THREADS-1:0] ifu_i1_predecode,


   output eh2_br_pkt_t [pt.NUM_THREADS-1:0] i0_brp,           // Instruction 0 branch packet. From Aligner to Decode
   output eh2_br_pkt_t [pt.NUM_THREADS-1:0] i1_brp,           // Instruction 1 branch packet. From Aligner to Decode
   output logic [pt.NUM_THREADS-1:0] [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ifu_i0_bp_index, // BP index
   output logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]           ifu_i0_bp_fghr, // BP FGHR
   output logic [pt.NUM_THREADS-1:0] [pt.BTB_BTAG_SIZE-1:0]          ifu_i0_bp_btag, // BP tag
   output logic [pt.NUM_THREADS-1:0] [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ifu_i1_bp_index, // BP index
   output logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]           ifu_i1_bp_fghr, // BP FGHR
   output logic [pt.NUM_THREADS-1:0] [pt.BTB_BTAG_SIZE-1:0]          ifu_i1_bp_btag, // BP tag

   input eh2_predict_pkt_t [pt.NUM_THREADS-1:0]                    exu_mp_pkt, // mispredict packet
   input logic [pt.NUM_THREADS-1:0][pt.BHT_GHR_SIZE-1:0]            exu_mp_eghr, // execute ghr
   input logic [pt.NUM_THREADS-1:0][pt.BHT_GHR_SIZE-1:0]            exu_mp_fghr,                    // Mispredict fghr
   input logic [pt.NUM_THREADS-1:0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]  exu_mp_index,         // Mispredict index
   input logic [pt.NUM_THREADS-1:0][pt.BTB_BTAG_SIZE-1:0]           exu_mp_btag,                   // Mispredict btag

   input eh2_br_tlu_pkt_t                     dec_tlu_br0_wb_pkt, // slot0 update/error pkt
   input eh2_br_tlu_pkt_t                     dec_tlu_br1_wb_pkt, // slot1 update/error pkt
   input logic [pt.BHT_GHR_SIZE-1:0]           dec_tlu_br0_fghr_wb, // fghr to bp
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br0_index_wb, // bp index
   input logic [pt.BHT_GHR_SIZE-1:0]           dec_tlu_br1_fghr_wb, // fghr to bp
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br1_index_wb, // bp index

   input [pt.NUM_THREADS-1:0] dec_tlu_i0_commit_cmt,


   output logic [pt.NUM_THREADS-1:0] [15:0] ifu_i0_cinst,
   output logic [pt.NUM_THREADS-1:0] [15:0] ifu_i1_cinst,


/// Icache debug
   input  eh2_cache_debug_pkt_t        dec_tlu_ic_diag_pkt ,
   output logic                    ifu_ic_debug_rd_data_valid,


// Icache/ICCM errors
   output logic [pt.NUM_THREADS-1:0]  ifu_miss_state_idle,          // I-side miss buffer empty
   output logic [pt.NUM_THREADS-1:0]  ifu_ic_error_start,           // IC single bit error
   output logic [pt.NUM_THREADS-1:0]  ifu_iccm_rd_ecc_single_err,   // ICCM single bit error

   input logic scan_mode
   );

   localparam TAGWIDTH = 2 ;
   localparam IDWIDTH  = 2 ;

   logic                   ifc_fetch_uncacheable_f1;

   logic [3:0]   ifu_fetch_val;  // valids on a 2B boundary, left justified [7] implies valid fetch
   logic [31:1]  ifu_fetch_pc;   // starting pc of fetch

   logic [31:1] ifc_fetch_addr_f1, ifc_fetch_addr_f2;

   logic [pt.NUM_THREADS-1:0]   ic_write_stall_thr;
   logic        ic_dma_active;
   logic        ifc_dma_access_ok;
   logic        ifc_iccm_access_f1;
   logic        ifc_region_acc_fault_f1;
   logic        ic_access_fault_f2;
   logic  [1:0] ic_access_fault_type_f2;// Access fault type
   logic [pt.NUM_THREADS-1:0]                                             ifu_ic_mb_empty_thr;
   logic [pt.NUM_THREADS-1:0]                                             ic_crit_wd_rdy_thr;
   logic [3:0]   ic_fetch_val_f2;
   logic [63:0]  ic_data_f2;
   logic [63:0]  ifu_fetch_data;
   logic         ifc_fetch_req_f1_raw, ifc_fetch_req_f1, ifc_fetch_req_f2;
   logic         iccm_rd_ecc_single_err;  // This fetch has an iccm single error.
   logic         iccm_rd_ecc_double_err;  // This fetch has an iccm double error.
   logic [pt.NUM_THREADS-1:0]         ifu_async_error_start;

   logic ifu_fetch_tid;
   logic ic_hit_f2;


   // fetch control
   logic [pt.NUM_THREADS-1:0] [31:1] fetch_addr_f1; // fetch address
   logic [pt.NUM_THREADS-1:0] fetch_uncacheable_f1, fetch_req_f1, fetch_req_f1_raw, fetch_req_f2,
                              iccm_access_f1, region_acc_fault_f1, dma_access_ok,
                              ifc_ready;

   logic [pt.NUM_THREADS-1:0] fb_consume1;                                   // Consumed one buffer. To fetch control fetch for buffer mass balance
   logic [pt.NUM_THREADS-1:0] fb_consume2;                                   // Consumed two buffers.To fetch control fetch for buffer mass balance

logic [pt.NUM_THREADS-1:0] dec_tlu_i0_commit_cmt_thr;
logic  fetch_tid_f1 ;
   logic [pt.NUM_THREADS-1:0] i0_valid;                                      // Instruction 0 is valid
   logic [pt.NUM_THREADS-1:0] i1_valid;                                      // Instruction 1 is valid
   logic [pt.NUM_THREADS-1:0] i0_icaf;                                       // Instruction 0 has access fault
   logic [pt.NUM_THREADS-1:0] [1:0]  i0_icaf_type;                           // Instruction 0 access fault type
   logic [pt.NUM_THREADS-1:0] i0_icaf_f1;                                    // Instruction 0 has access fault on second fetch group
   logic [pt.NUM_THREADS-1:0] i0_dbecc;                                      // Instruction 0 has double bit ecc error
   logic [pt.NUM_THREADS-1:0] [31:0] i0_instr;                               // Instruction 0
   logic [pt.NUM_THREADS-1:0] [31:0] i1_instr;                               // Instruction 1
   logic [pt.NUM_THREADS-1:0] [31:1] i0_pc;                                  // Instruction 0 PC
   logic [pt.NUM_THREADS-1:0] [31:1] i1_pc;                                  // Instruction 1 PC
   logic [pt.NUM_THREADS-1:0] i0_pc4;
   logic [pt.NUM_THREADS-1:0] i1_pc4;
   eh2_predecode_pkt_t [pt.NUM_THREADS-1:0] i0_predecode;
   eh2_predecode_pkt_t [pt.NUM_THREADS-1:0] i1_predecode;
   eh2_br_pkt_t [pt.NUM_THREADS-1:0] i0_br_p;                                    // Branch packet for I0.
   eh2_br_pkt_t [pt.NUM_THREADS-1:0] i1_br_p;                                    // Branch packet for I1.
   logic [pt.NUM_THREADS-1:0] [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]  i0_bp_index;  // BP index
   logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]            i0_bp_fghr;   // BP FGHR
   logic [pt.NUM_THREADS-1:0] [pt.BTB_BTAG_SIZE-1:0]           i0_bp_btag;   // BP tag
   logic [pt.NUM_THREADS-1:0] [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]  i1_bp_index;  // BP index
   logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]            i1_bp_fghr;   // BP FGHR
   logic [pt.NUM_THREADS-1:0] [pt.BTB_BTAG_SIZE-1:0]           i1_bp_btag;   // BP tag
   logic [pt.NUM_THREADS-1:0] [1:0] pmu_instr_aligned;                       // number of inst aligned this cycle
   logic [pt.NUM_THREADS-1:0]       pmu_align_stall;                         // aligner stalled this cycle
   logic [pt.NUM_THREADS-1:0] [15:0] i0_cinst;                               // 16b compress inst for i0
   logic [pt.NUM_THREADS-1:0] [15:0] i1_cinst;                               // 16b compress inst for i1
   logic [3:0]  ifu_bp_way_f2; // way indication; right justified
   logic  ifu_bp_kill_next_f2; // kill next fetch; taken target found
   logic [31:1] ifu_bp_btb_target_f2; //  predicted target PC
   logic [3:1]  ifu_bp_inst_mask_f2; // tell ic which valids to kill because of a taken branch; right justified

   logic [3:0]  ifu_bp_hist1_f2; // history counters for all 4 potential branches; right justified
   logic [3:0]  ifu_bp_hist0_f2; // history counters for all 4 potential branches; right justified
   logic [11:0] ifu_bp_poffset_f2; // predicted target
   logic [3:0]  ifu_bp_ret_f2; // predicted ret ; right justified
   logic [3:0]  ifu_bp_pc4_f2; // pc4 indication; right justified
   logic [3:0]  ifu_bp_valid_f2; // branch valid, right justified
   logic [pt.BHT_GHR_SIZE-1:0] ifu_bp_fghr_f2;

     for (genvar i=0; i<pt.NUM_THREADS; i++) begin : ifc

        eh2_ifu_ifc_ctl #(.pt(pt)) ifc (.tid               (1'(i)),
                                         .ic_write_stall(ic_write_stall_thr[i]),
                                         .ifu_ic_mb_empty(ifu_ic_mb_empty_thr[i]),
                                         .ic_crit_wd_rdy(ic_crit_wd_rdy_thr[i]),
                                         .ifu_fb_consume1(fb_consume1[i]),
                                         .ifu_fb_consume2(fb_consume2[i]),
                                         .dec_tlu_flush_noredir_wb(dec_tlu_flush_noredir_wb[i]),
                                         .exu_flush_final(exu_flush_final[i]),
                                         .exu_flush_path_final(exu_flush_path_final[i]),

                                         .fetch_uncacheable_f1(fetch_uncacheable_f1[i]),
                                         .fetch_addr_f1(fetch_addr_f1[i]),
                                         .fetch_req_f1(fetch_req_f1[i]),
                                         .fetch_req_f1_raw(fetch_req_f1_raw[i]),
                                         .fetch_req_f2(fetch_req_f2[i]),
                                         .pmu_fetch_stall(ifu_pmu_fetch_stall[i]),
                                         .iccm_access_f1(iccm_access_f1[i]),
                                         .region_acc_fault_f1(region_acc_fault_f1[i]),
                                         .dma_access_ok(dma_access_ok[i]),
                                         .ready(ifc_ready[i]),
                                         .*
                                         );

     end // block: ifc

   if (pt.NUM_THREADS == 2) begin: genmtifc

      rvarbiter2 ifc_arbiter (
                             .ready(ifc_ready[1:0]),
                             .tid  (ifc_select_tid_f1),
                             .shift(ifc_fetch_req_f1),
                           .*
                             );
   end
   else begin
      assign ifc_select_tid_f1 = 1'b0;
   end


   assign ifc_fetch_uncacheable_f1 = fetch_uncacheable_f1[ifc_select_tid_f1];
   assign ifc_fetch_addr_f1[31:1]  = fetch_addr_f1[ifc_select_tid_f1];
   assign ifc_fetch_req_f1 = fetch_req_f1[ifc_select_tid_f1];
   assign ifc_fetch_req_f1_raw = fetch_req_f1_raw[ifc_select_tid_f1];
   assign ifc_iccm_access_f1 = iccm_access_f1[ifc_select_tid_f1];
   assign ifc_region_acc_fault_f1 = region_acc_fault_f1[ifc_select_tid_f1];

   assign ifc_fetch_req_f2 = fetch_req_f2[ifu_fetch_tid];

   assign ifc_dma_access_ok = &dma_access_ok[pt.NUM_THREADS-1:0];



   // branch predictor
   eh2_ifu_bp_ctl #(.pt(pt)) bp (.*);



   assign ifu_fetch_data[63:0]  = ic_data_f2[63:0];
   assign ifu_fetch_val[3:0]    = ic_fetch_val_f2[3:0];
   assign ifu_fetch_pc[31:1]    = ifc_fetch_addr_f2[31:1];

   // aligner

   // multithreaded signals



  for (genvar i=0; i<pt.NUM_THREADS; i++) begin : aln

     eh2_ifu_aln_ctl #(.pt(pt)) aln (.tid               (1'(i)),
                                      .dec_i1_cancel_e1  (dec_i1_cancel_e1[i]),
                                      .dec_ib3_valid_d   (dec_ib3_valid_d[i]),
                                      .dec_ib2_valid_d   (dec_ib2_valid_d[i]),
                                      .exu_flush_final   (exu_flush_final[i]),
                                      .ifu_async_error_start (ifu_async_error_start[i]),
                                      .i0_valid          (i0_valid[i]),
                                      .i1_valid          (i1_valid[i]),
                                      .i0_icaf           (i0_icaf[i]),
                                      .i0_icaf_type      (i0_icaf_type[i]),
                                      .i0_icaf_f1        (i0_icaf_f1[i]),
                                      .i0_dbecc          (i0_dbecc[i]),
                                      .i0_instr          (i0_instr[i]),
                                      .i1_instr          (i1_instr[i]),
                                      .i0_pc             (i0_pc[i]),
                                      .i1_pc             (i1_pc[i]),
                                      .i0_pc4            (i0_pc4[i]),
                                      .i1_pc4            (i1_pc4[i]),
                                      .i0_predecode      (i0_predecode[i]),
                                      .i1_predecode      (i1_predecode[i]),
                                      .fb_consume1       (fb_consume1[i]),
                                      .fb_consume2       (fb_consume2[i]),
                                      .i0_br_p           (i0_br_p[i]),
                                      .i1_br_p           (i1_br_p[i]),
                                      .i0_bp_index       (i0_bp_index[i]),
                                      .i0_bp_fghr        (i0_bp_fghr[i]),
                                      .i0_bp_btag        (i0_bp_btag[i]),
                                      .i1_bp_index       (i1_bp_index[i]),
                                      .i1_bp_fghr        (i1_bp_fghr[i]),
                                      .i1_bp_btag        (i1_bp_btag[i]),
                                      .pmu_instr_aligned (pmu_instr_aligned[i]),
                                      .pmu_align_stall   (pmu_align_stall[i]),
                                      .i0_cinst          (i0_cinst[i]),
                                      .i1_cinst          (i1_cinst[i]),
                                      .*);
  end






      assign dec_tlu_i0_commit_cmt_thr[pt.NUM_THREADS-1:0] =   dec_tlu_i0_commit_cmt[pt.NUM_THREADS-1:0] ;

      assign ifu_i0_valid [pt.NUM_THREADS-1:0] =     i0_valid[pt.NUM_THREADS-1:0];
      assign ifu_i1_valid [pt.NUM_THREADS-1:0] =     i1_valid[pt.NUM_THREADS-1:0];
      assign ifu_i0_icaf  [pt.NUM_THREADS-1:0] =     i0_icaf[pt.NUM_THREADS-1:0];
      assign ifu_i0_icaf_type [pt.NUM_THREADS-1:0] = i0_icaf_type[pt.NUM_THREADS-1:0];
      assign ifu_i0_icaf_f1   [pt.NUM_THREADS-1:0] = i0_icaf_f1[pt.NUM_THREADS-1:0];
      assign ifu_i0_dbecc [pt.NUM_THREADS-1:0] =     i0_dbecc[pt.NUM_THREADS-1:0];
      assign ifu_i0_instr [pt.NUM_THREADS-1:0] =     i0_instr[pt.NUM_THREADS-1:0];
      assign ifu_i1_instr [pt.NUM_THREADS-1:0] =     i1_instr[pt.NUM_THREADS-1:0];
      assign ifu_i0_pc    [pt.NUM_THREADS-1:0] =     i0_pc[pt.NUM_THREADS-1:0];
      assign ifu_i1_pc    [pt.NUM_THREADS-1:0] =     i1_pc[pt.NUM_THREADS-1:0];
      assign ifu_i0_pc4   [pt.NUM_THREADS-1:0] =     i0_pc4[pt.NUM_THREADS-1:0];
      assign ifu_i1_pc4   [pt.NUM_THREADS-1:0] =     i1_pc4[pt.NUM_THREADS-1:0];
      assign ifu_i0_predecode [pt.NUM_THREADS-1:0] = i0_predecode[pt.NUM_THREADS-1:0];
      assign ifu_i1_predecode [pt.NUM_THREADS-1:0] = i1_predecode[pt.NUM_THREADS-1:0];
      assign i0_brp [pt.NUM_THREADS-1:0] =           i0_br_p[pt.NUM_THREADS-1:0];
      assign i1_brp [pt.NUM_THREADS-1:0] =           i1_br_p[pt.NUM_THREADS-1:0];
      assign ifu_i0_bp_index [pt.NUM_THREADS-1:0] =  i0_bp_index[pt.NUM_THREADS-1:0];
      assign ifu_i0_bp_fghr  [pt.NUM_THREADS-1:0] =  i0_bp_fghr[pt.NUM_THREADS-1:0];
      assign ifu_i0_bp_btag  [pt.NUM_THREADS-1:0] =  i0_bp_btag[pt.NUM_THREADS-1:0];
      assign ifu_i1_bp_index [pt.NUM_THREADS-1:0] =  i1_bp_index[pt.NUM_THREADS-1:0];
      assign ifu_i1_bp_fghr  [pt.NUM_THREADS-1:0] =  i1_bp_fghr[pt.NUM_THREADS-1:0];
      assign ifu_i1_bp_btag  [pt.NUM_THREADS-1:0] =  i1_bp_btag[pt.NUM_THREADS-1:0];
      assign ifu_i0_cinst [pt.NUM_THREADS-1:0] =     i0_cinst[pt.NUM_THREADS-1:0];
      assign ifu_i1_cinst [pt.NUM_THREADS-1:0] =     i1_cinst[pt.NUM_THREADS-1:0];



   assign ifu_pmu_instr_aligned[pt.NUM_THREADS-1:0] = pmu_instr_aligned[pt.NUM_THREADS-1:0];

   assign ifu_pmu_align_stall[pt.NUM_THREADS-1:0] = pmu_align_stall[pt.NUM_THREADS-1:0];

   assign fetch_tid_f1 = ifc_select_tid_f1;


   // icache
   eh2_ifu_mem_ctl #(.pt(pt)) mem_ctl
     (.*,
      .fetch_addr_f1         (ifc_fetch_addr_f1),
      .fetch_tid_f2          (ifu_fetch_tid),
      .dec_tlu_i0_commit_cmt (dec_tlu_i0_commit_cmt_thr)
      );



   // Performance debug info
   //
   //
`ifdef DUMP_BTB_ON

 `define DEC `CPU_TOP.dec
 `define EXU `CPU_TOP.exu

   logic exu_mp_valid; // conditional branch mispredict
   logic exu_mp_way; // conditional branch mispredict
   logic exu_mp_ataken; // direction is actual taken
   logic exu_mp_boffset; // branch offsett
   logic exu_mp_pc4; // branch is a 4B inst
   logic exu_mp_call; // branch is a call inst
   logic exu_mp_ret; // branch is a ret inst
   logic exu_mp_ja; // branch is a jump always
   logic exu_mp_bank; // write bank; based on branch PC[3:2]
   logic [1:0] exu_mp_hist; // new history
   logic [11:0] exu_mp_tgt; // target offset
   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] exu_mp_addr; // BTB/BHT address
   logic [3:0] ic_rd_hit_f2;
   logic [1:0] tmp_bnk;
   logic [31:0] mppc_ns0, mppc0;
   logic [31:0] mppc_ns1, mppc1;
   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] btb_rd_addr_f2, btb_rd_addr_p1_f2;
   logic [pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] bht_rd_addr_f2, bht_rd_addr_p1_f2;
   logic                                 i;

   eh2_btb_addr_hash #(.pt(pt)) f2hash(.pc(ifc_fetch_addr_f2[pt.BTB_INDEX3_HI:pt.BTB_INDEX1_LO]), .hash(btb_rd_addr_f2[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]));
   logic use_p1;
   assign use_p1 = (bp.fetch_start_f2[1] & bp.vwayhit_f2[3]) | (bp.fetch_start_f2[2] & |bp.vwayhit_f2[3:2]) | (bp.fetch_start_f2[3] & |bp.vwayhit_f2[3:1]) ;

   assign mppc_ns0[0] = 1'b0;
   assign mppc_ns1[0] = 1'b0;

   rvdff #(36)  mdseal_ff (.*, .din({mppc_ns0[31:0], mem_ctl.ic_rd_hit[3:0]}), .dout({mppc0[31:0],ic_rd_hit_f2[3:0]}));
   rvdff #(32)  mdseal1_ff (.*, .din({mppc_ns1[31:0]}), .dout({mppc1[31:0]}));
   logic [31:0] i0_pc_wb, i1_pc_wb;
   assign i0_pc_wb[0] = 1'b0;
   assign i1_pc_wb[0] = 1'b0;

   rvdff #(62)  e4pc (.*, .din({`DEC.dec_tlu_i0_pc_e4[31:1],`DEC.dec_tlu_i1_pc_e4[31:1]}), .dout({i0_pc_wb[31:1], i1_pc_wb[31:1]}));
   rvdff #(2*(pt.BHT_ADDR_HI-pt.BHT_ADDR_LO+1))  bhtff (.*, .din({bp.bht_rd_addr_f1, bp.bht_rd_addr_p1_f1}), .dout({bht_rd_addr_f2, bht_rd_addr_p1_f2}));

   assign tmp_bnk[1:0] = encode4_2(bp.btb_sel_f2[3:0]);
   logic [31:1] flush_path_i0_wb,flush_path_i1_wb;
   assign flush_path_i0_wb[31:1] = exu_flush_path_final[`DEC.tlu.i0tid_wb][31:1];
   assign flush_path_i1_wb[31:1] = exu_flush_path_final[`DEC.tlu.i1tid_wb][31:1];

   always @(negedge clk) begin
      if(`DEC.tlu.tlumt[0].tlu.mcyclel[31:0] == 32'h0000_0010) begin
         $display("BTB_CONFIG: %d",pt.BTB_ARRAY_DEPTH*4);
         `ifndef BP_NOGSHARE
         $display("BHT_CONFIG: %d gshare: 1",pt.BHT_ARRAY_DEPTH*4);
         `else
         $display("BHT_CONFIG: %d gshare: 0",pt.BHT_ARRAY_DEPTH*4);
         `endif
         $display("RS_CONFIG: %d", pt.RET_STACK_SIZE);
      end


      mppc_ns0[31:1] = `EXU.i0_flush_upper_e1[0] ? `DEC.decode.i0_pc_e1[31:1] :
                      (`EXU.i1_flush_upper_e1[0] ? `DEC.decode.i1_pc_e1[31:1] :
                       (`EXU.exu_i0_flush_lower_e4[0] ?  `DEC.decode.i0_pc_e4[31:1] :  `DEC.decode.i1_pc_e4[31:1]));


      if(exu_flush_final[0] & ~(dec_tlu_br0_wb_pkt.br_error | dec_tlu_br0_wb_pkt.br_start_error | dec_tlu_br1_wb_pkt.br_error | dec_tlu_br1_wb_pkt.br_start_error) & (exu_mp_pkt[0].misp | exu_mp_pkt[0].ataken))
        $display("%7d BTB_MP[T0]  : index: %0h bank: %0h call: %b ret: %b ataken: %b hist: %h valid: %b tag: %h targ: %h eghr: %b pred: %b ghr_index: %h brpc: %h way: %h",
                 `DEC.tlu.tlumt[0].tlu.mcyclel[31:0]+32'ha, exu_mp_index[0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO], exu_mp_pkt[0].bank, exu_mp_pkt[0].pcall, exu_mp_pkt[0].pret,
                 exu_mp_pkt[0].ataken, exu_mp_pkt[0].hist[1:0],
                 exu_mp_pkt[0].misp, exu_mp_btag[0][pt.BTB_BTAG_SIZE-1:0], {exu_flush_path_final[0][31:1], 1'b0}, exu_mp_eghr[0][pt.BHT_GHR_SIZE-1:0], exu_mp_pkt[0].misp,
                 bp.mp_hashed[0], mppc0[31:0], exu_mp_pkt[0].way);
      for(int i = 0; i < 4; i++) begin
         if(ifu_bp_valid_f2[i] & ifc_fetch_req_f2)
           $display("%7d BTB_HIT[T%b] : index: %0h bank: %0h call: %b ret: %b taken: %b strength: %b tag: %h targ: %0h ghr: %4b ghr_index: %h way: %h",
                    `DEC.tlu.tlumt[0].tlu.mcyclel[31:0]+32'ha, bp.ifc_select_tid_f2, btb_rd_addr_f2[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO],encode4_2(bp.btb_sel_f2[3:0]), bp.btb_rd_call_f2, bp.btb_rd_ret_f2,
                    ifu_bp_hist1_f2[tmp_bnk], ifu_bp_hist0_f2[tmp_bnk], bp.fetch_rd_tag_f2[pt.BTB_BTAG_SIZE-1:0], {ifu_bp_btb_target_f2[31:1], 1'b0},
                    bp.fghr[0][pt.BHT_GHR_SIZE-1:0], use_p1 ? bht_rd_addr_p1_f2 : bht_rd_addr_f2, ifu_bp_way_f2[tmp_bnk]);
      end


         mppc_ns1[31:1] = `EXU.i0_flush_upper_e1[1] ? `DEC.decode.i0_pc_e1[31:1] :
                         (`EXU.i1_flush_upper_e1[1] ? `DEC.decode.i1_pc_e1[31:1] :
                          (`EXU.exu_i0_flush_lower_e4[1] ?  `DEC.decode.i0_pc_e4[31:1] :  `DEC.decode.i1_pc_e4[31:1]));


         if(exu_flush_final[1] & ~(dec_tlu_br0_wb_pkt.br_error | dec_tlu_br0_wb_pkt.br_start_error | dec_tlu_br1_wb_pkt.br_error | dec_tlu_br1_wb_pkt.br_start_error) & (exu_mp_pkt[1].misp | exu_mp_pkt[1].ataken))
           $display("%7d BTB_MP[T1]  : index: %0h bank: %0h call: %b ret: %b ataken: %b hist: %h valid: %b tag: %h targ: %h eghr: %b pred: %b ghr_index: %h brpc: %h way: %h",
                    `DEC.tlu.tlumt[0].tlu.mcyclel[31:0]+32'ha, exu_mp_index[1][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO], exu_mp_pkt[1].bank, exu_mp_pkt[1].pcall, exu_mp_pkt[1].pret,
                    exu_mp_pkt[1].ataken, exu_mp_pkt[1].hist[1:0],
                    exu_mp_pkt[1].misp, exu_mp_btag[1][pt.BTB_BTAG_SIZE-1:0], {exu_flush_path_final[1][31:1], 1'b0}, exu_mp_eghr[1][pt.BHT_GHR_SIZE-1:0], exu_mp_pkt[1].misp,
                    bp.mp_hashed[1], mppc1[31:0], exu_mp_pkt[1].way);

      if(dec_tlu_br0_wb_pkt.valid & ~(dec_tlu_br0_wb_pkt.br_error | dec_tlu_br0_wb_pkt.br_start_error))
        $display("%7d BTB_UPD0[T%b]: ghr_index: %0h bank: %0h hist: %h  way: %h brpc: %h",
                 `DEC.tlu.tlumt[0].tlu.mcyclel[31:0]+32'ha,`DEC.tlu.i0tid_wb, bp.br0_hashed_wb[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO],{dec_tlu_br0_wb_pkt.bank,dec_tlu_br0_wb_pkt.middle},
                 dec_tlu_br0_wb_pkt.hist, dec_tlu_br0_wb_pkt.way, i0_pc_wb);
      if(dec_tlu_br1_wb_pkt.valid & ~(dec_tlu_br1_wb_pkt.br_error | dec_tlu_br1_wb_pkt.br_start_error))
        $display("%7d BTB_UPD1[T%b]: ghr_index: %0h bank: %0h hist: %h  way: %h brpc: %h",
                 `DEC.tlu.tlumt[0].tlu.mcyclel[31:0]+32'ha,`DEC.tlu.i1tid_wb,bp.br1_hashed_wb[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO],{dec_tlu_br1_wb_pkt.bank,dec_tlu_br1_wb_pkt.middle},
                 dec_tlu_br1_wb_pkt.hist, dec_tlu_br1_wb_pkt.way, i1_pc_wb);
      if(dec_tlu_br0_wb_pkt.br_error | dec_tlu_br0_wb_pkt.br_start_error)
        $display("%7d BTB_ERR0[T%b]: index: %0h bank: %0h start: %b rfpc: %h way: %h",
                 `DEC.tlu.tlumt[0].tlu.mcyclel[31:0]+32'ha,`DEC.tlu.i0tid_wb,dec_tlu_br0_index_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO],dec_tlu_br0_wb_pkt.bank, dec_tlu_br0_wb_pkt.br_start_error,
                 {flush_path_i0_wb, 1'b0}, dec_tlu_br0_wb_pkt.way);
      if(dec_tlu_br1_wb_pkt.br_error | dec_tlu_br1_wb_pkt.br_start_error)
        $display("%7d BTB_ERR1[T%b]: index: %0h bank: %0h start: %b rfpc: %h way: %h",
                 `DEC.tlu.tlumt[0].tlu.mcyclel[31:0]+32'ha,`DEC.tlu.i1tid_wb,dec_tlu_br1_index_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO],dec_tlu_br1_wb_pkt.bank, dec_tlu_br1_wb_pkt.br_start_error,
                 {flush_path_i1_wb, 1'b0}, dec_tlu_br1_wb_pkt.way);
   end // always @ (negedge clk)
      function [1:0] encode4_2;
      input [3:0] in;

      encode4_2[1] = in[3] | in[2];
      encode4_2[0] = in[3] | in[1];

   endfunction
`endif
endmodule // ifu
