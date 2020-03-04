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
// Function: Top level file for load store unit
// Comments:
//
//
// DC1 -> DC2 -> DC3 -> DC4 (Commit)
//
//********************************************************************************

module eh2_lsu
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)(

   input logic [31:0]                      i0_result_e4_eff, // I0 e4 result for e4 -> dc3 store forwarding
   input logic [31:0]                      i1_result_e4_eff, // I1 e4 result for e4 -> dc3 store forwarding
   input logic [31:0]                      i0_result_e2,     // I0 e2 result for e2 -> dc2 store forwarding

   input logic [pt.NUM_THREADS-1:0]        flush_final_e3,            // I0/I1 flush in e3
   input logic [pt.NUM_THREADS-1:0]        i0_flush_final_e3,         // I0 flush in e3
   input logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_lower_wb,    // I0/I1 writeback flush. This is used to flush the old packets only
   input logic                             dec_tlu_i0_kill_writeb_wb, // I0 is flushed, don't writeback any results to arch state
   input logic                             dec_tlu_i1_kill_writeb_wb, // I1 is flushed, don't writeback any results to arch state
   input logic [pt.NUM_THREADS-1:0]        dec_tlu_lr_reset_wb,
   input logic [pt.NUM_THREADS-1:0]        dec_tlu_force_halt,

   // chicken signals
   input logic                             dec_tlu_external_ldfwd_disable,     // disable load to load forwarding for externals
   input logic                             dec_tlu_wb_coalescing_disable,      // disable the write buffer coalesce
   input logic                             dec_tlu_sideeffect_posted_disable,  // disable posted writes to sideeffect addr to the bus
   input logic                             dec_tlu_core_ecc_disable,           // disable the generation of the ecc

   input logic [31:0]                      exu_lsu_rs1_d,      // address rs operand
   input logic [31:0]                      exu_lsu_rs2_d,      // store data
   input logic [11:0]                      dec_lsu_offset_d,   // address offset operand

   input                                   eh2_lsu_pkt_t lsu_p,     // lsu control packet
   input logic [31:0]                      dec_tlu_mrac_ff,     // CSR for memory region control

   output logic [31:0]                     lsu_result_dc3,      // lsu load data
   output logic [31:0]                     lsu_result_corr_dc4, // This is the ECC corrected data going to RF
   output logic                            lsu_fastint_stall_any, // Stall fast interrupts at decode-1
   output logic                            lsu_sc_success_dc5,  // the store condition result ( 1 :

   output logic [pt.NUM_THREADS-1:0]       lsu_store_stall_any, // This is for blocking stores in the decode
   output logic [pt.NUM_THREADS-1:0]       lsu_load_stall_any,  // This is for blocking loads in the decode
   output logic [pt.NUM_THREADS-1:0]       lsu_amo_stall_any,   // This is for blocking amo in the decode
   output logic [pt.NUM_THREADS-1:0]       lsu_idle_any,        // This is used to enter halt mode. Exclude DMA

   output logic [31:1]                     lsu_fir_addr,        // fast interrupt address
   output logic [1:0]                      lsu_fir_error,       // Error during fast interrupt lookup

   output eh2_lsu_error_pkt_t             lsu_error_pkt_dc3,             // lsu exception packet
   output logic                            lsu_single_ecc_error_incr,     // Increment the ecc error counter
   output logic [pt.NUM_THREADS-1:0]       lsu_imprecise_error_load_any,  // bus load imprecise error
   output logic [pt.NUM_THREADS-1:0]       lsu_imprecise_error_store_any, // bus store imprecise error
   output logic [pt.NUM_THREADS-1:0][31:0] lsu_imprecise_error_addr_any,  // bus store imprecise error address

   // Non-blocking loads
   output logic                                lsu_nonblock_load_valid_dc1,    // there is an external load -> put in the cam
   output logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_tag_dc1,      // the tag of the external non block load
   output logic                                lsu_nonblock_load_inv_dc2,      // Invalidate the non-block load bcoz of memory forwarding
   output logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_inv_tag_dc2,
   output logic                                lsu_nonblock_load_inv_dc5,      // invalidate signal for the cam entry for non block loads
   output logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_inv_tag_dc5,  // tag of the enrty which needs to be invalidated
   output logic                                lsu_nonblock_load_data_valid,   // the non block is valid - sending information back to the cam
   output logic                                lsu_nonblock_load_data_error,   // non block load has an error
   output logic                               lsu_nonblock_load_data_tid,      // tid for nonblock load return
   output logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_data_tag,     // the tag of the non block load sending the data/error
   output logic [31:0]                         lsu_nonblock_load_data,         // Data of the non block load

   output logic [pt.NUM_THREADS-1:0]       lsu_pmu_load_external_dc3,      // PMU : Load to the bus
   output logic [pt.NUM_THREADS-1:0]       lsu_pmu_store_external_dc3,     // PMU : Load to the bus
   output logic [pt.NUM_THREADS-1:0]       lsu_pmu_misaligned_dc3,         // PMU : misaligned
   output logic [pt.NUM_THREADS-1:0]       lsu_pmu_bus_trxn,               // PMU : bus transaction
   output logic [pt.NUM_THREADS-1:0]       lsu_pmu_bus_misaligned,         // PMU : misaligned access going to the bus
   output logic [pt.NUM_THREADS-1:0]       lsu_pmu_bus_error,              // PMU : bus sending error back
   output logic [pt.NUM_THREADS-1:0]       lsu_pmu_bus_busy,               // PMU : bus is not ready

   output logic [31:0]                     lsu_rs1_dc1,

   // Trigger signals
   input eh2_trigger_pkt_t [pt.NUM_THREADS-1:0][3:0] trigger_pkt_any, // Trigger info from the decode
   output logic [3:0]                      lsu_trigger_match_dc4,                    // lsu trigger hit (one bit per trigger)

   // DCCM ports
   output logic                            dccm_wren,       // DCCM write enable
   output logic                            dccm_rden,       // DCCM read enable
   output logic [pt.DCCM_BITS-1:0]         dccm_wr_addr_lo, // DCCM write address low bankd
   output logic [pt.DCCM_BITS-1:0]         dccm_wr_addr_hi, // DCCM write address low bankd
   output logic [pt.DCCM_BITS-1:0]         dccm_rd_addr_lo, // DCCM read address low bank
   output logic [pt.DCCM_BITS-1:0]         dccm_rd_addr_hi, // DCCM read address hi bank (hi and low same if aligned read)
   output logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_wr_data_lo, // DCCM write data for hi bank
   output logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_wr_data_hi, // DCCM write data for hi bank

   input logic [pt.DCCM_FDATA_WIDTH-1:0]   dccm_rd_data_lo, // DCCM read data low bank
   input logic [pt.DCCM_FDATA_WIDTH-1:0]   dccm_rd_data_hi, // DCCM read data hi bank

   // PIC ports
   output logic                            picm_wren,        // PIC memory write enable
   output logic                            picm_rden,        // PIC memory read enable
   output logic                            picm_mken,        // Need to read the mask for stores to determine which bits to write/forward
   output logic                            picm_rd_thr,      // PICM read thread
   output logic [31:0]                     picm_rdaddr,      // PIC memory address
   output logic [31:0]                     picm_wraddr,      // PIC memory address
   output logic [31:0]                     picm_wr_data,     // PIC memory write data
   input logic [31:0]                      picm_rd_data,     // PIC memory read/mask data

   //-------------------------- LSU AXI signals--------------------------
   // AXI Write Channels
   output logic                            lsu_axi_awvalid,
   input  logic                            lsu_axi_awready,
   output logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_awid,
   output logic [31:0]                     lsu_axi_awaddr,
   output logic [3:0]                      lsu_axi_awregion,
   output logic [7:0]                      lsu_axi_awlen,
   output logic [2:0]                      lsu_axi_awsize,
   output logic [1:0]                      lsu_axi_awburst,
   output logic                            lsu_axi_awlock,
   output logic [3:0]                      lsu_axi_awcache,
   output logic [2:0]                      lsu_axi_awprot,
   output logic [3:0]                      lsu_axi_awqos,

   output logic                            lsu_axi_wvalid,
   input  logic                            lsu_axi_wready,
   output logic [63:0]                     lsu_axi_wdata,
   output logic [7:0]                      lsu_axi_wstrb,
   output logic                            lsu_axi_wlast,

   input  logic                            lsu_axi_bvalid,
   output logic                            lsu_axi_bready,
   input  logic [1:0]                      lsu_axi_bresp,
   input  logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_bid,

   // AXI Read Channels
   output logic                            lsu_axi_arvalid,
   input  logic                            lsu_axi_arready,
   output logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_arid,
   output logic [31:0]                     lsu_axi_araddr,
   output logic [3:0]                      lsu_axi_arregion,
   output logic [7:0]                      lsu_axi_arlen,
   output logic [2:0]                      lsu_axi_arsize,
   output logic [1:0]                      lsu_axi_arburst,
   output logic                            lsu_axi_arlock,
   output logic [3:0]                      lsu_axi_arcache,
   output logic [2:0]                      lsu_axi_arprot,
   output logic [3:0]                      lsu_axi_arqos,

   input  logic                            lsu_axi_rvalid,
   output logic                            lsu_axi_rready,
   input  logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_rid,
   input  logic [63:0]                     lsu_axi_rdata,
   input  logic [1:0]                      lsu_axi_rresp,
   input  logic                            lsu_axi_rlast,

   input logic                             lsu_bus_clk_en,    // external drives a clock_en to control bus ratio

   // DMA slave
   input logic                             dma_dccm_req,       // DMA read/write to dccm
   input logic                             dma_dccm_spec_req,  // DMA spec_read/write to dccm
   input logic                             dma_mem_addr_in_dccm,   // DMA address is in dccm
   input logic [2:0]                       dma_mem_tag,        // DMA request tag
   input logic [31:0]                      dma_mem_addr,       // DMA address
   input logic [2:0]                       dma_mem_sz,         // DMA access size
   input logic                             dma_mem_write,      // DMA access is a write
   input logic [63:0]                      dma_mem_wdata,      // DMA write data

   output logic                            dccm_dma_rvalid,     // lsu data valid for DMA dccm read
   output logic                            dccm_dma_ecc_error,  // DMA load had ecc error
   output logic [2:0]                      dccm_dma_rtag,       // DMA return tag
   output logic [63:0]                     dccm_dma_rdata,      // lsu data for DMA dccm read
   output logic                            dccm_ready,          // lsu ready for DMA access

   input logic                             clk_override,        // Disable clock gating
   input logic                             scan_mode,           // scan
   input logic                             clk,
   input logic                             free_clk,
   input logic                             rst_l

   );

   logic [31:0] lsu_addr_dc1;
   logic        lsu_dccm_rden_dc3;
   logic [31:0] store_data_dc3;
   logic [31:0] store_data_pre_dc3;
   logic [31:0] store_ecc_data_hi_dc3;          // final store data either from store_data or SEC DCCM readout - not STBUF FWD
   logic [31:0] store_ecc_data_lo_dc3;
   logic [pt.DCCM_DATA_WIDTH-1:0] sec_data_hi_dc3;
   logic [pt.DCCM_DATA_WIDTH-1:0] sec_data_lo_dc3;
   logic        disable_ecc_check_lo_dc3;
   logic        disable_ecc_check_hi_dc3;

   logic [pt.DCCM_DATA_WIDTH-1:0] sec_data_hi_dc5, sec_data_lo_dc5;

   logic        ld_single_ecc_error_dc3, ld_single_ecc_error_dc5, ld_single_ecc_error_dc5_ff;
   logic        ld_single_ecc_error_lo_dc5_ff, ld_single_ecc_error_hi_dc5_ff;
   logic        single_ecc_error_hi_dc3, single_ecc_error_lo_dc3;
   logic        single_ecc_error_hi_dc4, single_ecc_error_lo_dc4;
   logic        single_ecc_error_hi_dc5, single_ecc_error_lo_dc5;
   logic        lsu_single_ecc_error_dc3, lsu_single_ecc_error_dc5;
   logic        lsu_double_ecc_error_dc3, lsu_double_ecc_error_dc5;
   logic        access_fault_dc3;
   logic        misaligned_fault_dc3;

   logic [31:0] dccm_data_hi_dc3;
   logic [31:0] dccm_data_lo_dc3;
   logic [31:0] dccm_datafn_hi_dc5;
   logic [31:0] dccm_datafn_lo_dc5;
   logic [6:0]  dccm_data_ecc_hi_dc3;
   logic [6:0]  dccm_data_ecc_lo_dc3;
   logic [63:0] store_data_ext_dc3, store_data_ext_dc4, store_data_ext_dc5;

   logic [31:0] lsu_dccm_data_dc3;
   logic [31:0] lsu_dccm_data_corr_dc3;
   logic [31:0] picm_mask_data_dc3;
   logic [31:0] picm_rd_data_dc3;

   logic [31:0] lsu_addr_dc2, lsu_addr_dc3, lsu_addr_dc4, lsu_addr_dc5;
   logic [31:0] end_addr_dc1, end_addr_dc2, end_addr_dc3, end_addr_dc4, end_addr_dc5;
   logic        core_ldst_dual_dc1;


   eh2_lsu_pkt_t  lsu_pkt_dc1_pre, lsu_pkt_dc1, lsu_pkt_dc2, lsu_pkt_dc3, lsu_pkt_dc4, lsu_pkt_dc5;

   // Store Buffer signals
   logic        store_stbuf_reqvld_dc5;
   logic        lsu_commit_dc5;

   logic        addr_in_dccm_region_dc1;     // address in dccm region
   logic        addr_in_dccm_dc1, addr_in_dccm_dc2, addr_in_dccm_dc3, addr_in_dccm_dc4, addr_in_dccm_dc5;
   logic        addr_in_pic_dc1, addr_in_pic_dc2, addr_in_pic_dc3, addr_in_pic_dc4, addr_in_pic_dc5;
   logic        addr_external_dc1, addr_external_dc3;

   logic                          stbuf_reqvld_any;
   logic                          stbuf_reqvld_flushed_any;
   logic [pt.LSU_SB_BITS-1:0]     stbuf_addr_any;
   logic [pt.DCCM_DATA_WIDTH-1:0] stbuf_data_any;

   logic                          lsu_cmpen_dc2;
   logic [pt.DCCM_DATA_WIDTH-1:0] stbuf_fwddata_hi_dc3;
   logic [pt.DCCM_DATA_WIDTH-1:0] stbuf_fwddata_lo_dc3;
   logic [pt.DCCM_BYTE_WIDTH-1:0] stbuf_fwdbyteen_hi_dc3;
   logic [pt.DCCM_BYTE_WIDTH-1:0] stbuf_fwdbyteen_lo_dc3;

   logic                          picm_fwd_en_dc2;
   logic [31:0]                   picm_fwd_data_dc2;

   logic                       lsu_stbuf_commit_any;
   logic [pt.NUM_THREADS-1:0]  lsu_stbuf_empty_any;   // This is for blocking loads
   logic [pt.NUM_THREADS-1:0]  lsu_stbuf_full_any;

    // Bus signals
   logic        lsu_busreq_dc5;
   logic        lsu_busreq_dc1;
   logic [pt.NUM_THREADS-1:0]  lsu_bus_idle_any;
   logic [pt.NUM_THREADS-1:0]  lsu_bus_buffer_pend_any;
   logic [pt.NUM_THREADS-1:0]  lsu_bus_buffer_empty_any;
   logic [pt.NUM_THREADS-1:0]  lsu_bus_buffer_full_any;
   logic [31:0] bus_read_data_dc3;

   logic [pt.NUM_THREADS-1:0]  flush_dc2_up, flush_dc3, flush_dc4, flush_dc5;// flush_prior_dc5;
   logic        is_sideeffects_dc2, is_sideeffects_dc3;
   logic        ldst_nodma_dc2todc5;
   logic        dma_dccm_wen, dma_dccm_spec_wen, dma_pic_wen;
   logic [2:0]  dma_mem_tag_dc1, dma_mem_tag_dc2, dma_mem_tag_dc3;
   logic [31:0] dma_start_addr_dc1, dma_end_addr_dc1;
   logic [31:0] dma_dccm_wdata_hi, dma_dccm_wdata_lo;

   // Clocks
   logic        lsu_c1_dc1_clk, lsu_c1_dc2_clk, lsu_c1_dc3_clk, lsu_c1_dc4_clk, lsu_c1_dc5_clk;
   logic        lsu_c2_dc1_clk, lsu_c2_dc2_clk, lsu_c2_dc3_clk, lsu_c2_dc4_clk, lsu_c2_dc5_clk;

   logic        lsu_store_c1_dc1_clk, lsu_store_c1_dc2_clk, lsu_store_c1_dc3_clk;
   logic        lsu_dccm_c1_dc3_clk, lsu_pic_c1_dc3_clk;
   logic        lsu_stbuf_c1_clk;
   logic        lsu_free_c2_clk;

   logic [pt.NUM_THREADS-1:0] lsu_bus_ibuf_c1_clk, lsu_bus_obuf_c1_clk, lsu_bus_buf_c1_clk;
   logic                      lsu_busm_clk;

   logic [31:0]                 amo_data_dc3;
   logic [pt.NUM_THREADS-1:0]   lr_vld;            // needed for clk gating

   logic        lsu_raw_fwd_lo_dc3, lsu_raw_fwd_hi_dc3;
   logic        lsu_raw_fwd_lo_dc4, lsu_raw_fwd_hi_dc4;
   logic        lsu_raw_fwd_lo_dc5, lsu_raw_fwd_hi_dc5;

   eh2_lsu_lsc_ctl #(.pt(pt)) lsu_lsc_ctl(.*);

   // Ready to accept dma trxns
   // There can't be any inpipe forwarding from non-dma packet to dma packet since they can be flushed so we can't have ld/st in dc3-dc5 when dma is in dc2
   assign ldst_nodma_dc2todc5 = (lsu_pkt_dc2.valid & ~lsu_pkt_dc2.dma & (addr_in_dccm_dc2 | addr_in_pic_dc2) & lsu_pkt_dc2.store) |
                                (lsu_pkt_dc3.valid & ~lsu_pkt_dc3.dma & (addr_in_dccm_dc3 | addr_in_pic_dc3) & lsu_pkt_dc3.store) |
                                (lsu_pkt_dc4.valid & ~lsu_pkt_dc4.dma & (addr_in_dccm_dc4 | addr_in_pic_dc4) & lsu_pkt_dc4.store);
   assign dccm_ready = ~(lsu_pkt_dc1_pre.valid | ldst_nodma_dc2todc5 | ld_single_ecc_error_dc5_ff);
   assign dma_mem_tag_dc1[2:0] = dma_mem_tag[2:0];

   assign dma_pic_wen  = dma_dccm_req & dma_mem_write & ~dma_mem_addr_in_dccm;
   assign dma_dccm_wen = dma_dccm_req & dma_mem_write & dma_mem_addr_in_dccm;
   assign dma_dccm_spec_wen = dma_dccm_spec_req & dma_mem_write;
   assign dma_start_addr_dc1[31:0] = dma_mem_addr[31:0];
   assign dma_end_addr_dc1[31:3]   = dma_mem_addr[31:3];
   assign dma_end_addr_dc1[2:0]    = (dma_mem_sz[2:0] == 3'b11) ? 3'b100 : dma_mem_addr[2:0];
    assign {dma_dccm_wdata_hi[31:0], dma_dccm_wdata_lo[31:0]} = dma_mem_wdata[63:0] >> {dma_mem_addr[2:0], 3'b000};   // Shift the dma data to lower bits to make it consistent to lsu stores

   // Generate per cycle flush signals
   for (genvar i=0; i<pt.NUM_THREADS; i++) begin: GenFlushLoop
      assign flush_dc2_up[i] = flush_final_e3[i] | dec_tlu_flush_lower_wb[i];
      assign flush_dc3[i]    = (flush_final_e3[i] & i0_flush_final_e3[i]) | dec_tlu_flush_lower_wb[i];
      assign flush_dc4[i]    = dec_tlu_flush_lower_wb[i];
      assign flush_dc5[i]    = ((dec_tlu_i0_kill_writeb_wb & ~lsu_pkt_dc5.pipe) | (dec_tlu_i1_kill_writeb_wb & lsu_pkt_dc5.pipe)) & (lsu_pkt_dc5.tid == i);
   end

   assign lsu_fastint_stall_any = ld_single_ecc_error_dc3;

   for (genvar i=0; i<pt.NUM_THREADS; i++) begin: GenThreadLoop
      // block stores in decode  - for either bus or stbuf reasons
      // block for sc/amo since stores does read modify write so they are similar to load
      assign lsu_store_stall_any[i] = (lsu_pkt_dc1.valid & (lsu_pkt_dc1.sc | (lsu_pkt_dc1.atomic & lsu_pkt_dc1.store))) |
                                      (lsu_pkt_dc2.valid & (lsu_pkt_dc2.sc | (lsu_pkt_dc2.atomic & lsu_pkt_dc2.store))) |
                                      (lsu_pkt_dc3.valid & (lsu_pkt_dc3.sc | (lsu_pkt_dc3.atomic & lsu_pkt_dc3.store))) |
                                      lsu_stbuf_full_any[i] | lsu_bus_buffer_full_any[i] | ld_single_ecc_error_dc5;
      // block the atomic (including lr/sc). We need to block lr/sc as well for ECC case (Store on T0 followed by lr/sr on T1 with ECC error)
      assign lsu_amo_stall_any[i]   = (lsu_pkt_dc1.valid & lsu_pkt_dc1.store & (lsu_pkt_dc1.tid != i)) |
                                      (lsu_pkt_dc2.valid & lsu_pkt_dc2.store & (lsu_pkt_dc2.tid != i)) |
                                      (lsu_pkt_dc3.valid & lsu_pkt_dc3.store & (lsu_pkt_dc3.tid != i));
      assign lsu_load_stall_any[i]  = (lsu_pkt_dc1.valid & (lsu_pkt_dc1.sc | (lsu_pkt_dc1.atomic & lsu_pkt_dc1.store))) |
                                      (lsu_pkt_dc2.valid & (lsu_pkt_dc2.sc | (lsu_pkt_dc2.atomic & lsu_pkt_dc2.store))) |
                                      (lsu_pkt_dc3.valid & (lsu_pkt_dc3.sc | (lsu_pkt_dc3.atomic & lsu_pkt_dc3.store))) |
                                      lsu_bus_buffer_full_any[i] | ld_single_ecc_error_dc5;

      // lsu halt idle. This is used for entering the halt mode
      // Indicates non-idle if there is a instruction valid in dc1-dc5 or read/write buffers are non-empty since they can come with error
      // We don't need store buffer here since it's commit state
      assign lsu_idle_any[i] = ~((lsu_pkt_dc1.valid & ~lsu_pkt_dc1.dma & (lsu_pkt_dc1.tid == 1'(i))) |
                                 (lsu_pkt_dc2.valid & ~lsu_pkt_dc2.dma & (lsu_pkt_dc2.tid == 1'(i))) |
                                 (lsu_pkt_dc3.valid & ~lsu_pkt_dc3.dma & (lsu_pkt_dc3.tid == 1'(i))) |
                                 (lsu_pkt_dc4.valid & ~lsu_pkt_dc4.dma & (lsu_pkt_dc4.tid == 1'(i))) |
                                 (lsu_pkt_dc5.valid & ~lsu_pkt_dc5.dma & (lsu_pkt_dc5.tid == 1'(i)))) &
                                 lsu_bus_idle_any[i] & lsu_bus_buffer_empty_any[i];
   end

   assign       lsu_raw_fwd_lo_dc3 = (|stbuf_fwdbyteen_lo_dc3[pt.DCCM_BYTE_WIDTH-1:0]);
   assign       lsu_raw_fwd_hi_dc3 = (|stbuf_fwdbyteen_hi_dc3[pt.DCCM_BYTE_WIDTH-1:0]);

   // Instantiate the store buffer
   assign store_stbuf_reqvld_dc5 = lsu_pkt_dc5.valid & (lsu_pkt_dc5.store | (lsu_pkt_dc5.atomic & ~lsu_pkt_dc5.lr)) &
                                   (~lsu_pkt_dc5.sc | lsu_sc_success_dc5 | (lsu_single_ecc_error_dc5 & ~lsu_raw_fwd_lo_dc5)) & addr_in_dccm_dc5 & lsu_commit_dc5;

   // Disable Forwarding for now
   assign lsu_cmpen_dc2 = lsu_pkt_dc2.valid & (lsu_pkt_dc2.load | lsu_pkt_dc2.store | lsu_pkt_dc1.atomic) & (addr_in_dccm_dc2 | addr_in_pic_dc2);

   // Bus signals
   assign lsu_busreq_dc1 = lsu_pkt_dc1_pre.valid & ((lsu_pkt_dc1_pre.load | lsu_pkt_dc1_pre.store) & addr_external_dc1) & ~flush_dc2_up[lsu_pkt_dc1_pre.tid] & ~lsu_pkt_dc1_pre.fast_int;

   // PMU signals
   for (genvar i=0; i<pt.NUM_THREADS; i++) begin: GenPMU
      assign lsu_pmu_misaligned_dc3[i]     = lsu_pkt_dc3.valid & ~lsu_pkt_dc3.dma & ((lsu_pkt_dc3.half & lsu_addr_dc3[0]) | (lsu_pkt_dc3.word & (|lsu_addr_dc3[1:0]))) & (i == lsu_pkt_dc3.tid);
      assign lsu_pmu_load_external_dc3[i]  = lsu_pkt_dc3.valid & ~lsu_pkt_dc3.dma & lsu_pkt_dc3.load & addr_external_dc3 & (i == lsu_pkt_dc3.tid);
      assign lsu_pmu_store_external_dc3[i] = lsu_pkt_dc3.valid & ~lsu_pkt_dc3.dma & lsu_pkt_dc3.store & addr_external_dc3 & (i == lsu_pkt_dc3.tid);
   end

   eh2_lsu_amo #(.pt(pt))  lsu_amo (.*);

   eh2_lsu_dccm_ctl #(.pt(pt)) dccm_ctl (
      .lsu_addr_dc1(lsu_addr_dc1[31:0]),
      .end_addr_dc1(end_addr_dc1[31:0]),
      .lsu_addr_dc3(lsu_addr_dc3[31:0]),
      .lsu_addr_dc4(lsu_addr_dc4[31:0]),
      .lsu_addr_dc5(lsu_addr_dc5[31:0]),

      .end_addr_dc2(end_addr_dc2[31:0]),
      .end_addr_dc3(end_addr_dc3[31:0]),
      .end_addr_dc4(end_addr_dc4[31:0]),
      .end_addr_dc5(end_addr_dc5[31:0]),
      .*
   );

   eh2_lsu_stbuf #(.pt(pt)) stbuf(
      .lsu_addr_dc1(lsu_addr_dc1[pt.LSU_SB_BITS-1:0]),
      .end_addr_dc1(end_addr_dc1[pt.LSU_SB_BITS-1:0]),

      .*

   );

   eh2_lsu_ecc #(.pt(pt)) ecc (
      .lsu_addr_dc3(lsu_addr_dc3[pt.DCCM_BITS-1:0]),
      .end_addr_dc3(end_addr_dc3[pt.DCCM_BITS-1:0]),
      .*
   );

   eh2_lsu_trigger #(.pt(pt)) trigger (
      .store_data_dc3(store_data_dc3[31:0]),
      .*
   );

   // Clk domain
   eh2_lsu_clkdomain #(.pt(pt)) clkdomain (.*);

   // Bus interface
   eh2_lsu_bus_intf #(.pt(pt)) bus_intf (.*);

   //Flops
   rvdff #(1) single_ecc_err_hidc4  (.*, .din(single_ecc_error_hi_dc3),     .dout(single_ecc_error_hi_dc4), .clk(lsu_c2_dc4_clk));
   rvdff #(1) single_ecc_err_hidc5  (.*, .din(single_ecc_error_hi_dc4),     .dout(single_ecc_error_hi_dc5), .clk(lsu_c2_dc5_clk));
   rvdff #(1) single_ecc_err_lodc4  (.*, .din(single_ecc_error_lo_dc3),     .dout(single_ecc_error_lo_dc4), .clk(lsu_c2_dc4_clk));
   rvdff #(1) single_ecc_err_lodc5  (.*, .din(single_ecc_error_lo_dc4),     .dout(single_ecc_error_lo_dc5), .clk(lsu_c2_dc5_clk));

   rvdff #(3) dma_mem_tag_dc2ff    (.*, .din(dma_mem_tag_dc1[2:0]),         .dout(dma_mem_tag_dc2[2:0]),     .clk(lsu_c2_dc2_clk));
   rvdff #(3) dma_mem_tag_dc3ff    (.*, .din(dma_mem_tag_dc2[2:0]),         .dout(dma_mem_tag_dc3[2:0]),     .clk(lsu_c2_dc3_clk));

   rvdff #(2) lsu_raw_fwd_dc4_ff    (.*, .din({lsu_raw_fwd_hi_dc3, lsu_raw_fwd_lo_dc3}),     .dout({lsu_raw_fwd_hi_dc4, lsu_raw_fwd_lo_dc4}),     .clk(lsu_c2_dc4_clk));
   rvdff #(2) lsu_raw_fwd_dc5_ff    (.*, .din({lsu_raw_fwd_hi_dc4, lsu_raw_fwd_lo_dc4}),     .dout({lsu_raw_fwd_hi_dc5, lsu_raw_fwd_lo_dc5}),     .clk(lsu_c2_dc5_clk));

`ifdef ASSERT_ON
   logic [8:0] store_data_bypass_sel;
   assign store_data_bypass_sel[8:0] =  {lsu_p.store_data_bypass_c1,
                                         lsu_p.store_data_bypass_c2,
                                         lsu_p.store_data_bypass_i0_e2_c2,
                                         lsu_p.store_data_bypass_e4_c1[1:0],
                                         lsu_p.store_data_bypass_e4_c2[1:0],
                                         lsu_p.store_data_bypass_e4_c3[1:0]} & {9{lsu_p.valid}};
   assert_store_data_bypass_onehot: assert #0 ($onehot0(store_data_bypass_sel[8:0]));

   //assert_no_exceptions: assert #0 (lsu_exc_pkt_dc3.exc_valid == 1'b0);
   property exception_no_lsu_flush;
      logic    tid;
      @(posedge clk)  disable iff(~rst_l) (lsu_error_pkt_dc3.exc_valid, tid = lsu_pkt_dc3.tid) |-> ##[1:2] (flush_dc4[tid] | flush_dc5[tid]);
   endproperty
   assert_exception_no_lsu_flush: assert property (exception_no_lsu_flush) else
      $display("No flush within 2 cycles of exception");

   // offset should be zero for fast interrupt
   property offset_0_fastint;
      @(posedge clk) disable iff(~rst_l) (lsu_p.valid & lsu_p.fast_int) |-> (dec_lsu_offset_d[11:0] == 12'b0);
   endproperty
   assert_offset_0_fastint: assert property (offset_0_fastint) else
      $display("dec_tlu_offset_d not zero for fast interrupt redirect");

   // fastint_stall should cause load/store stall next cycle
   property fastint_stall_imply_loadstore_stall;
      @(posedge clk) disable iff(~rst_l) lsu_fastint_stall_any |-> ##2 (~ld_single_ecc_error_dc5 | (|(lsu_load_stall_any[pt.NUM_THREADS-1:0] | lsu_store_stall_any[pt.NUM_THREADS-1:0])));
   endproperty
   assert_fastint_stall_imply_loadstore_stall: assert property (fastint_stall_imply_loadstore_stall) else
      $display("fastint_stall should be followed by lsu_load/store_stall_any");

  // Atomic needs to preserve memory ordering(aq/rl attributes are assumed) so lsu_idle needs to be high when atomics are dispatched to lsu
  property atomic_notidle;
     logic     tid;
     @(posedge clk) disable iff (~rst_l) ((lsu_p.valid & lsu_p.atomic), tid = lsu_p.tid) |-> lsu_idle_any[tid];
  endproperty
  assert_atomic_notidle: assert property (atomic_notidle) else
     $display("LSU not idle but Atomic instruction (AMO/LR/SC) in decode");

`endif

endmodule : eh2_lsu
