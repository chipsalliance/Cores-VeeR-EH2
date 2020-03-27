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
// Function: LSU control
// Comments:
//
//
// DC1 -> DC2 -> DC3 -> DC4 (Commit)
//
//********************************************************************************
module eh2_lsu_lsc_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)(
   input  logic                  scan_mode,
   input logic                   rst_l,

   input logic                   clk,
   input logic                   lsu_c1_dc1_clk,
   input logic                   lsu_c1_dc2_clk,
   input logic                   lsu_c1_dc3_clk,
   input logic                   lsu_c1_dc4_clk,
   input logic                   lsu_c1_dc5_clk,

   input logic                   lsu_c2_dc1_clk,       // clock
   input logic                   lsu_c2_dc2_clk,       // clock
   input logic                   lsu_c2_dc3_clk,
   input logic                   lsu_c2_dc4_clk,
   input logic                   lsu_c2_dc5_clk,
   input logic                   lsu_free_c2_clk,

   input logic                   lsu_store_c1_dc1_clk,
   input logic                   lsu_store_c1_dc2_clk,
   input logic                   lsu_store_c1_dc3_clk,

   input logic [31:0]            i0_result_e4_eff,
   input logic [31:0]            i1_result_e4_eff,
   input logic [31:0]            i0_result_e2,
   input logic [31:0]            exu_lsu_rs1_d, // address
   input logic [31:0]            exu_lsu_rs2_d, // store data
   input logic [11:0]            dec_lsu_offset_d,
   input logic [31:0]            dec_tlu_mrac_ff,           // CSR read

   input eh2_lsu_pkt_t          lsu_p,                     // lsu control packet
   input logic                   lsu_single_ecc_error_dc3,
   input logic                   lsu_double_ecc_error_dc3,
   output logic                  access_fault_dc3,
   output logic                  misaligned_fault_dc3,
   output logic                  lsu_single_ecc_error_dc5,
   output logic                  lsu_double_ecc_error_dc5,

   input logic [pt.NUM_THREADS-1:0] flush_dc2_up,
   input logic [pt.NUM_THREADS-1:0] flush_dc3,
   input logic [pt.NUM_THREADS-1:0] flush_dc4,
   input logic [pt.NUM_THREADS-1:0] flush_dc5,
   input logic [pt.NUM_THREADS-1:0] dec_tlu_lr_reset_wb,

   input  logic [31:0]           lsu_dccm_data_dc3,
   input  logic [31:0]           lsu_dccm_data_corr_dc3,
   input  logic [31:0]           picm_rd_data_dc3,
   input  logic [31:0]           bus_read_data_dc3,
   output logic [31:0]           lsu_result_dc3,
   output logic [31:0]           lsu_result_corr_dc4,   // This is the ECC corrected data going to RF
   output logic [31:0]           lsu_rs1_dc1,

   // lsu address down the pipe
   output logic [31:0]           lsu_addr_dc1,
   output logic [31:0]           lsu_addr_dc2,
   output logic [31:0]           lsu_addr_dc3,
   output logic [31:0]           lsu_addr_dc4,
   output logic [31:0]           lsu_addr_dc5,

   output logic [31:0]           end_addr_dc1,
   output logic [31:0]           end_addr_dc2,
   output logic [31:0]           end_addr_dc3,
   output logic [31:0]           end_addr_dc4,
   output logic [31:0]           end_addr_dc5,

   // store data down the pipe
   output logic [31:0]           store_data_pre_dc3,

   output logic                  lsu_single_ecc_error_incr,
   output eh2_lsu_error_pkt_t   lsu_error_pkt_dc3,

   output logic [31:1]           lsu_fir_addr,        // fast interrupt address
   output logic [1:0]            lsu_fir_error,       // Error during fast interrupt lookup

   output logic                  core_ldst_dual_dc1,
   output logic                  is_sideeffects_dc2,
   output logic                  is_sideeffects_dc3,
   output logic                  lsu_commit_dc5,

   // address in dccm/pic/external per pipe stage
   output logic                  addr_in_dccm_region_dc1,     // address in dccm region
   output logic                  addr_in_dccm_dc1,
   output logic                  addr_in_dccm_dc2,
   output logic                  addr_in_dccm_dc3,
   output logic                  addr_in_dccm_dc4,
   output logic                  addr_in_dccm_dc5,

   output logic                  addr_in_pic_dc1,
   output logic                  addr_in_pic_dc2,
   output logic                  addr_in_pic_dc3,
   output logic                  addr_in_pic_dc4,
   output logic                  addr_in_pic_dc5,

   output eh2_lsu_pkt_t         lsu_pkt_dc1_pre,
   output eh2_lsu_pkt_t         lsu_pkt_dc1,
   output eh2_lsu_pkt_t         lsu_pkt_dc2,
   output eh2_lsu_pkt_t         lsu_pkt_dc3,
   output eh2_lsu_pkt_t         lsu_pkt_dc4,
   output eh2_lsu_pkt_t         lsu_pkt_dc5,

   output logic                  addr_external_dc1,
   output logic                  addr_external_dc3,
   output logic                  lsu_sc_success_dc5,
   output logic [pt.NUM_THREADS-1:0]            lr_vld,   // needed for clk gating

   // DMA slave
   input logic                   dma_dccm_req,
   input logic [31:0]            dma_start_addr_dc1,
   input logic [31:0]            dma_end_addr_dc1,
   input logic [31:0]            dma_dccm_wdata_lo,   // This has both dccm/pic data
   input logic [2:0]             dma_mem_sz,
   input logic                   dma_mem_write,
   input logic                   dma_mem_addr_in_dccm
);

   localparam THREADS          = pt.NUM_THREADS;

   logic [31:0]        core_start_addr_dc1;
   logic [31:0]        core_end_addr_dc1;
   logic [31:0]        lsu_rs1_d;
   logic [11:0]        lsu_offset_d;
   logic [31:0]        rs1_dc1;
   logic [11:0]        offset_dc1;
   logic [11:0]        lsu_offset_dc1;
   logic [12:0]        end_addr_offset_dc1;
   logic [31:0]        lsu_ld_datafn_dc3;
   logic [31:0]        lsu_ld_datafn_corr_dc3;
   logic [2:0]         addr_offset_dc1;

   logic               core_addr_in_dccm_dc1, core_addr_in_pic_dc1, core_addr_external_dc1;
   logic               addr_external_dc2;
   logic               access_fault_dc2, misaligned_fault_dc2;
   logic [3:0]         exc_mscause_dc1, exc_mscause_dc2, exc_mscause_dc3;
   logic               lsu_single_ecc_error_dc4;
   logic               lsu_double_ecc_error_dc4;

   logic [31:0]        store_data_d;
   logic [31:0]        store_data_dc1;
   logic [31:0]        store_data_pre_dc2;
   logic [31:0]        store_data_dc2_in;
   logic [31:0]        store_data_dc2;
   logic [31:0]        rs1_dc1_raw;
   logic [31:0]        offset32_dc1;

   eh2_lsu_pkt_t      dma_pkt_dc1;
   eh2_lsu_pkt_t      lsu_pkt_dc1_in, lsu_pkt_dc2_in, lsu_pkt_dc3_in, lsu_pkt_dc4_in, lsu_pkt_dc5_in;

   logic               fir_dccm_access_error_dc2, fir_nondccm_access_error_dc2;
   logic               fir_dccm_access_error_dc3, fir_nondccm_access_error_dc3;
   logic [1:0]         lsu_fir_error_dc3;

   logic [31:0]        lsu_result_corr_dc3;

   logic [THREADS-1:0] [31:2] lr_addr;   // Per Thread LR stations
   logic [THREADS-1:0]        lr_wr_en, lr_reset;   // set and reset logic
   logic                      tid_dc5;
   logic [THREADS-1:0]        lsu_sc_success_vec_dc5;


   //------------------------------------------------------------------------------------------------------------
   //----------------------------------------Logic starts here---------------------------------------------------
   //------------------------------------------------------------------------------------------------------------

   if (pt.LOAD_TO_USE_PLUS1 == 1) begin: GenL2U_1
      assign lsu_rs1_d[31:0] = lsu_pkt_dc1_in.load_ldst_bypass_c1 ? lsu_result_dc3[31:0] :  exu_lsu_rs1_d[31:0];
      assign rs1_dc1[31:0]   = rs1_dc1_raw[31:0];
   end else begin: GenL2U_0
      assign lsu_rs1_d[31:0] = exu_lsu_rs1_d[31:0];
      assign rs1_dc1[31:0]   = (lsu_pkt_dc1_pre.load_ldst_bypass_c1) ? lsu_result_dc3[31:0] : rs1_dc1_raw[31:0];
   end

   assign lsu_rs1_dc1[31:0] = rs1_dc1[31:0];

   // Premux the rs1/offset for dma
   assign lsu_offset_dc1[11:0] = offset_dc1[11:0] & ~{12{lsu_pkt_dc1_pre.atomic}};

   rvdff #(32) rs1ff    (.*, .din(lsu_rs1_d[31:0]),    .dout(rs1_dc1_raw[31:0]), .clk(lsu_c1_dc1_clk));
   rvdff #(12) offsetff (.*, .din(dec_lsu_offset_d[11:0]), .dout(offset_dc1[11:0]),  .clk(lsu_c1_dc1_clk));

    assign offset32_dc1[31:0] =  { {20{lsu_offset_dc1[11]}},lsu_offset_dc1[11:0]};

   assign core_start_addr_dc1[31:0] =  rs1_dc1[31:0] + offset32_dc1[31:0];
   assign core_end_addr_dc1[31:0]   = rs1_dc1[31:0] + {{19{end_addr_offset_dc1[12]}},end_addr_offset_dc1[12:0]};


   // Module to generate the memory map of the address
   eh2_lsu_addrcheck #(.pt(pt)) addrcheck (
                  .start_addr_dc1(core_start_addr_dc1[31:0]),
                  .end_addr_dc1(core_end_addr_dc1[31:0]),
                  .start_addr_dc2(lsu_addr_dc2[31:0]),
                  .end_addr_dc2(end_addr_dc2[31:0]),
                  .addr_in_dccm_dc1(core_addr_in_dccm_dc1),
                  .addr_in_pic_dc1(core_addr_in_pic_dc1),
                  .addr_external_dc1(core_addr_external_dc1),
                  .*
  );

   // Calculate start/end address for load/store
   assign addr_offset_dc1[2:0]      = ({3{lsu_pkt_dc1_pre.half}} & 3'b01) | ({3{lsu_pkt_dc1_pre.word}} & 3'b11) | ({3{lsu_pkt_dc1_pre.dword}} & 3'b111);
   assign end_addr_offset_dc1[12:0] = {lsu_offset_dc1[11],lsu_offset_dc1[11:0]} + {9'b0,addr_offset_dc1[2:0]};
   assign end_addr_dc1[31:0]        = lsu_pkt_dc1_pre.valid ? core_end_addr_dc1[31:0] : dma_end_addr_dc1[31:0];
   assign lsu_addr_dc1[31:0]        = lsu_pkt_dc1_pre.valid ? core_start_addr_dc1[31:0] : dma_start_addr_dc1[31:0];   // absence load/store all 0's

   assign addr_in_dccm_dc1 = lsu_pkt_dc1_pre.valid ? core_addr_in_dccm_dc1 : dma_mem_addr_in_dccm;
   assign addr_in_pic_dc1  = lsu_pkt_dc1_pre.valid ? core_addr_in_pic_dc1 : ~dma_mem_addr_in_dccm;
   assign addr_external_dc1 = lsu_pkt_dc1_pre.valid & core_addr_external_dc1;
   assign core_ldst_dual_dc1 = core_start_addr_dc1[2] != core_end_addr_dc1[2];

   // Goes to TLU to increment the ECC error counter
   assign lsu_single_ecc_error_incr = (lsu_single_ecc_error_dc5 & ~lsu_double_ecc_error_dc5) & (lsu_commit_dc5 | lsu_pkt_dc5.dma) & lsu_pkt_dc5.valid;

   // Generate exception packet
   assign lsu_error_pkt_dc3.exc_valid = (access_fault_dc3 | misaligned_fault_dc3 | lsu_double_ecc_error_dc3) & lsu_pkt_dc3.valid & ~lsu_pkt_dc3.dma & ~flush_dc3[lsu_pkt_dc3.tid] & ~lsu_pkt_dc3.fast_int;
   assign lsu_error_pkt_dc3.single_ecc_error = lsu_single_ecc_error_dc3 & ~lsu_error_pkt_dc3.exc_valid & ~lsu_pkt_dc3.dma & ~lsu_pkt_dc3.fast_int;   // This is used for rfnpc. Suppress single bit error if there is a fault/dma/fastint
   assign lsu_error_pkt_dc3.inst_type = lsu_pkt_dc3.store;   // AMO should be store
   assign lsu_error_pkt_dc3.amo_valid = lsu_pkt_dc3.atomic & ~(lsu_pkt_dc3.lr | lsu_pkt_dc3.sc);
   assign lsu_error_pkt_dc3.exc_type  = ~misaligned_fault_dc3;
   assign lsu_error_pkt_dc3.mscause[3:0] = (lsu_double_ecc_error_dc3 & ~misaligned_fault_dc3 & ~access_fault_dc3) ? 4'h1 : exc_mscause_dc3[3:0];
   assign lsu_error_pkt_dc3.addr[31:0] = lsu_addr_dc3[31:0];

   //Create DMA packet
   always_comb begin
      dma_pkt_dc1 = '0;
      dma_pkt_dc1.valid   = dma_dccm_req;
      dma_pkt_dc1.dma     = 1'b1;
      dma_pkt_dc1.store   = dma_mem_write;
      dma_pkt_dc1.load    = ~dma_mem_write;
      dma_pkt_dc1.by      = (dma_mem_sz[2:0] == 3'b0);
      dma_pkt_dc1.half    = (dma_mem_sz[2:0] == 3'b1);
      dma_pkt_dc1.word    = (dma_mem_sz[2:0] == 3'b10);
      dma_pkt_dc1.dword   = (dma_mem_sz[2:0] == 3'b11);
   end

   always_comb begin
      lsu_pkt_dc1_in = lsu_p;
      lsu_pkt_dc1    = dma_dccm_req ? dma_pkt_dc1 : lsu_pkt_dc1_pre;
      lsu_pkt_dc2_in = lsu_pkt_dc1;
      lsu_pkt_dc3_in = lsu_pkt_dc2;
      lsu_pkt_dc4_in = lsu_pkt_dc3;
      lsu_pkt_dc5_in = lsu_pkt_dc4;

      lsu_pkt_dc1_in.valid = lsu_p.valid & ~(flush_dc2_up[lsu_p.tid] & ~lsu_p.fast_int);
      lsu_pkt_dc2_in.valid = (lsu_pkt_dc1.valid & ~flush_dc2_up[lsu_pkt_dc1.tid]) | dma_dccm_req;
      lsu_pkt_dc3_in.valid = lsu_pkt_dc2.valid & ~(flush_dc2_up[lsu_pkt_dc2.tid] & ~lsu_pkt_dc2.dma);
      lsu_pkt_dc4_in.valid = lsu_pkt_dc3.valid & ~(flush_dc3[lsu_pkt_dc3.tid] & ~lsu_pkt_dc3.dma);
      lsu_pkt_dc5_in.valid = lsu_pkt_dc4.valid & ~(flush_dc4[lsu_pkt_dc4.tid] & ~lsu_pkt_dc4.dma);
   end

   assign lsu_ld_datafn_dc3[31:0] = ({32{addr_external_dc3}} & bus_read_data_dc3) |
                                    ({32{addr_in_pic_dc3}}   & picm_rd_data_dc3)  |
                                    ({32{addr_in_dccm_dc3}}  & lsu_dccm_data_dc3);

   assign lsu_ld_datafn_corr_dc3[31:0] = ({32{addr_external_dc3}} & bus_read_data_dc3) |
                                         ({32{addr_in_pic_dc3}}   & picm_rd_data_dc3)  |
                                         ({32{addr_in_dccm_dc3}}  & lsu_dccm_data_corr_dc3);

   // this result must look at prior stores and merge them in
   assign lsu_result_dc3[31:0] = ({32{ lsu_pkt_dc3.unsign & lsu_pkt_dc3.by  }} & {24'b0,lsu_ld_datafn_dc3[7:0]}) |
                                 ({32{ lsu_pkt_dc3.unsign & lsu_pkt_dc3.half}} & {16'b0,lsu_ld_datafn_dc3[15:0]}) |
                                 ({32{~lsu_pkt_dc3.unsign & lsu_pkt_dc3.by  }} & {{24{  lsu_ld_datafn_dc3[7]}}, lsu_ld_datafn_dc3[7:0]}) |
                                 ({32{~lsu_pkt_dc3.unsign & lsu_pkt_dc3.half}} & {{16{  lsu_ld_datafn_dc3[15]}},lsu_ld_datafn_dc3[15:0]}) |
                                 ({32{lsu_pkt_dc3.word}} &                       lsu_ld_datafn_dc3[31:0]);

   assign lsu_result_corr_dc3[31:0] = ({32{ lsu_pkt_dc3.unsign & lsu_pkt_dc3.by  }} & {24'b0,lsu_ld_datafn_corr_dc3[7:0]}) |
                                      ({32{ lsu_pkt_dc3.unsign & lsu_pkt_dc3.half}} & {16'b0,lsu_ld_datafn_corr_dc3[15:0]}) |
                                      ({32{~lsu_pkt_dc3.unsign & lsu_pkt_dc3.by  }} & {{24{  lsu_ld_datafn_corr_dc3[7]}}, lsu_ld_datafn_corr_dc3[7:0]}) |
                                      ({32{~lsu_pkt_dc3.unsign & lsu_pkt_dc3.half}} & {{16{  lsu_ld_datafn_corr_dc3[15]}},lsu_ld_datafn_corr_dc3[15:0]}) |
                                      ({32{lsu_pkt_dc3.word}} &                       lsu_ld_datafn_corr_dc3[31:0]);

   assign lsu_fir_addr[31:1]     = lsu_result_corr_dc4[31:1];
   assign lsu_fir_error_dc3[1:0] = fir_nondccm_access_error_dc3 ? 2'b11 : (fir_dccm_access_error_dc3 ? 2'b10 : ((lsu_pkt_dc3.fast_int & lsu_double_ecc_error_dc3) ? 2'b01 : 2'b00));

   // Interrupt as a flush source allows the WB to occur
   assign lsu_commit_dc5 = lsu_pkt_dc5.valid & (lsu_pkt_dc5.store | lsu_pkt_dc5.load | lsu_pkt_dc5.atomic) & ~flush_dc5[lsu_pkt_dc5.tid] & ~lsu_pkt_dc5.dma;

   assign store_data_d[31:0] = exu_lsu_rs2_d[31:0];

   //assign store_data_dc2_in[63:32] = store_data_dc1[63:32];
   assign store_data_dc2_in[31:0] = dma_dccm_req ? dma_dccm_wdata_lo[31:0] :                      // PIC writes from DMA still happens in dc5 since we need to read mask
                                    (lsu_pkt_dc1.store_data_bypass_c1) ? lsu_result_dc3[31:0] :
                                    (lsu_pkt_dc1.store_data_bypass_e4_c1[1]) ? i1_result_e4_eff[31:0] :
                                    (lsu_pkt_dc1.store_data_bypass_e4_c1[0]) ? i0_result_e4_eff[31:0] : store_data_dc1[31:0];

   //assign store_data_dc2[63:32] = store_data_pre_dc2[63:32];
   assign store_data_dc2[31:0] = (lsu_pkt_dc2.store_data_bypass_i0_e2_c2) ? i0_result_e2[31:0]     :
                                 (lsu_pkt_dc2.store_data_bypass_c2)       ? lsu_result_dc3[31:0]   :
                                 (lsu_pkt_dc2.store_data_bypass_e4_c2[1]) ? i1_result_e4_eff[31:0] :
                                 (lsu_pkt_dc2.store_data_bypass_e4_c2[0]) ? i0_result_e4_eff[31:0] : store_data_pre_dc2[31:0];


   // Flops
   rvdff #(32) lsu_result_corr_dc4ff (.*, .din(lsu_result_corr_dc3[31:0]), .dout(lsu_result_corr_dc4[31:0]), .clk(lsu_c1_dc4_clk));

   // C2 clock for valid and C1 for other bits of packet
   rvdff #(1) lsu_pkt_vlddc1ff (.*, .din(lsu_pkt_dc1_in.valid), .dout(lsu_pkt_dc1_pre.valid), .clk(lsu_c2_dc1_clk));
   rvdff #(1) lsu_pkt_vlddc2ff (.*, .din(lsu_pkt_dc2_in.valid), .dout(lsu_pkt_dc2.valid), .clk(lsu_c2_dc2_clk));
   rvdff #(1) lsu_pkt_vlddc3ff (.*, .din(lsu_pkt_dc3_in.valid), .dout(lsu_pkt_dc3.valid), .clk(lsu_c2_dc3_clk));
   rvdff #(1) lsu_pkt_vlddc4ff (.*, .din(lsu_pkt_dc4_in.valid), .dout(lsu_pkt_dc4.valid), .clk(lsu_c2_dc4_clk));
   rvdff #(1) lsu_pkt_vlddc5ff (.*, .din(lsu_pkt_dc5_in.valid), .dout(lsu_pkt_dc5.valid), .clk(lsu_c2_dc5_clk));

   rvdff #($bits(eh2_lsu_pkt_t)-1) lsu_pkt_dc1ff (.*, .din(lsu_pkt_dc1_in[$bits(eh2_lsu_pkt_t)-1:1]), .dout(lsu_pkt_dc1_pre[$bits(eh2_lsu_pkt_t)-1:1]), .clk(lsu_c1_dc1_clk));
   rvdff #($bits(eh2_lsu_pkt_t)-1) lsu_pkt_dc2ff (.*, .din(lsu_pkt_dc2_in[$bits(eh2_lsu_pkt_t)-1:1]), .dout(lsu_pkt_dc2[$bits(eh2_lsu_pkt_t)-1:1]), .clk(lsu_c1_dc2_clk));
   rvdff #($bits(eh2_lsu_pkt_t)-1) lsu_pkt_dc3ff (.*, .din(lsu_pkt_dc3_in[$bits(eh2_lsu_pkt_t)-1:1]), .dout(lsu_pkt_dc3[$bits(eh2_lsu_pkt_t)-1:1]), .clk(lsu_c1_dc3_clk));
   rvdff #($bits(eh2_lsu_pkt_t)-1) lsu_pkt_dc4ff (.*, .din(lsu_pkt_dc4_in[$bits(eh2_lsu_pkt_t)-1:1]), .dout(lsu_pkt_dc4[$bits(eh2_lsu_pkt_t)-1:1]), .clk(lsu_c1_dc4_clk));
   rvdff #($bits(eh2_lsu_pkt_t)-1) lsu_pkt_dc5ff (.*, .din(lsu_pkt_dc5_in[$bits(eh2_lsu_pkt_t)-1:1]), .dout(lsu_pkt_dc5[$bits(eh2_lsu_pkt_t)-1:1]), .clk(lsu_c1_dc5_clk));

   rvdff #(32) sddc1ff (.*, .din(store_data_d[31:0]),      .dout(store_data_dc1[31:0]),     .clk(lsu_store_c1_dc1_clk));
   rvdff #(32) sddc2ff (.*, .din(store_data_dc2_in[31:0]), .dout(store_data_pre_dc2[31:0]), .clk(lsu_store_c1_dc2_clk));
   rvdff #(32) sddc3ff (.*, .din(store_data_dc2[31:0]),    .dout(store_data_pre_dc3[31:0]), .clk(lsu_store_c1_dc3_clk));

   rvdff #(32) sadc2ff  (.*, .din(lsu_addr_dc1[31:0]),      .dout(lsu_addr_dc2[31:0]),       .clk(lsu_c1_dc2_clk));
   rvdff #(32) sadc3ff  (.*, .din(lsu_addr_dc2[31:0]),      .dout(lsu_addr_dc3[31:0]),       .clk(lsu_c1_dc3_clk));
   rvdff #(32) sadc4ff  (.*, .din(lsu_addr_dc3[31:0]),      .dout(lsu_addr_dc4[31:0]),       .clk(lsu_c1_dc4_clk));
   rvdff #(32) sadc5ff  (.*, .din(lsu_addr_dc4[31:0]),      .dout(lsu_addr_dc5[31:0]),       .clk(lsu_c1_dc5_clk));

   rvdff #(32) end_addr_dc2ff (.*, .din(end_addr_dc1[31:0]),    .dout(end_addr_dc2[31:0]), .clk(lsu_c1_dc2_clk));
   rvdff #(32) end_addr_dc3ff (.*, .din(end_addr_dc2[31:0]),    .dout(end_addr_dc3[31:0]), .clk(lsu_c1_dc3_clk));
   rvdff #(32) end_addr_dc4ff (.*, .din(end_addr_dc3[31:0]),    .dout(end_addr_dc4[31:0]), .clk(lsu_c1_dc4_clk));
   rvdff #(32) end_addr_dc5ff (.*, .din(end_addr_dc4[31:0]),    .dout(end_addr_dc5[31:0]), .clk(lsu_c1_dc5_clk));

   rvdff #(1) addr_in_dccm_dc2ff(.din(addr_in_dccm_dc1), .dout(addr_in_dccm_dc2), .clk(lsu_c2_dc2_clk), .*);
   rvdff #(1) addr_in_dccm_dc3ff(.din(addr_in_dccm_dc2), .dout(addr_in_dccm_dc3), .clk(lsu_c2_dc3_clk), .*);
   rvdff #(1) addr_in_dccm_dc4ff(.din(addr_in_dccm_dc3), .dout(addr_in_dccm_dc4), .clk(lsu_c2_dc4_clk), .*);
   rvdff #(1) addr_in_dccm_dc5ff(.din(addr_in_dccm_dc4), .dout(addr_in_dccm_dc5), .clk(lsu_c2_dc5_clk), .*);

   rvdff #(1) addr_in_pic_dc2ff(.din(addr_in_pic_dc1), .dout(addr_in_pic_dc2), .clk(lsu_c2_dc2_clk), .*);
   rvdff #(1) addr_in_pic_dc3ff(.din(addr_in_pic_dc2), .dout(addr_in_pic_dc3), .clk(lsu_c2_dc3_clk), .*);
   rvdff #(1) addr_in_pic_dc4ff(.din(addr_in_pic_dc3), .dout(addr_in_pic_dc4), .clk(lsu_c2_dc4_clk), .*);
   rvdff #(1) addr_in_pic_dc5ff(.din(addr_in_pic_dc4), .dout(addr_in_pic_dc5), .clk(lsu_c2_dc5_clk), .*);

   rvdff #(1) addr_external_dc3ff(.din(addr_external_dc2), .dout(addr_external_dc3), .clk(lsu_c2_dc3_clk), .*);

   rvdff #(1) access_fault_dc3ff     (.din(access_fault_dc2),     .dout(access_fault_dc3),     .clk(lsu_c2_dc3_clk), .*);
   rvdff #(1) misaligned_fault_dc3ff (.din(misaligned_fault_dc2), .dout(misaligned_fault_dc3), .clk(lsu_c2_dc3_clk), .*);
   rvdff #(4) exc_mscause_dc3ff      (.din(exc_mscause_dc2[3:0]), .dout(exc_mscause_dc3[3:0]), .clk(lsu_c2_dc3_clk), .*);

   rvdff #(1) lsu_single_ecc_error_dc4ff (.*, .din(lsu_single_ecc_error_dc3), .dout(lsu_single_ecc_error_dc4), .clk(lsu_c2_dc4_clk));
   rvdff #(1) lsu_single_ecc_error_dc5ff (.*, .din(lsu_single_ecc_error_dc4), .dout(lsu_single_ecc_error_dc5), .clk(lsu_c2_dc5_clk));
   rvdff #(1) lsu_double_ecc_error_dc4ff (.*, .din(lsu_double_ecc_error_dc3), .dout(lsu_double_ecc_error_dc4), .clk(lsu_c2_dc4_clk));
   rvdff #(1) lsu_double_ecc_error_dc5ff (.*, .din(lsu_double_ecc_error_dc4), .dout(lsu_double_ecc_error_dc5), .clk(lsu_c2_dc5_clk));

   rvdff #(1) fir_dccm_access_error_dc3ff    (.din(fir_dccm_access_error_dc2),    .dout(fir_dccm_access_error_dc3),    .clk(lsu_c2_dc3_clk), .*);
   rvdff #(1) fir_nondccm_access_error_dc3ff (.din(fir_nondccm_access_error_dc2), .dout(fir_nondccm_access_error_dc3), .clk(lsu_c2_dc3_clk), .*);
   rvdff #(2) fir_error_dc4ff                (.din(lsu_fir_error_dc3[1:0]),       .dout(lsu_fir_error[1:0]),           .clk(lsu_c2_dc4_clk), .*);

   // Load Reservation
   // when the LR commits - it will set a valid and its address [31:2] for its own thread's LR
   // the Reset conditions are :
   // Same Thread : 1) Any Store Conditional - match or not is not relevant
   //               2) Entering Debug,
   //               3) Leaving Debug,
   //               4) Mret, Interrup or Exception
   //
   // Other Thread :1) Store or AMO to this location ( 31:2 match )
   assign tid_dc5 = lsu_pkt_dc5.tid;
   always_comb  begin : store_cond
     lsu_sc_success_vec_dc5[THREADS-1:0] = '0;
     for (int i=0; i < THREADS; i++ ) begin
        lsu_sc_success_vec_dc5[i] = (i == tid_dc5) & (lsu_addr_dc5[31:2] == lr_addr[i][31:2]) & lsu_pkt_dc5.valid & lsu_pkt_dc5.sc & lr_vld[i];
     end
   end : store_cond

   assign lsu_sc_success_dc5 = |lsu_sc_success_vec_dc5[THREADS-1:0];

   for (genvar i=0; i<THREADS; i++) begin
      assign lr_wr_en[i] =  ( i == tid_dc5 )  & lsu_commit_dc5 & lsu_pkt_dc5.lr;
      assign lr_reset[i] =  (( i == tid_dc5 ) & (lsu_commit_dc5 & lsu_pkt_dc5.sc))                                                                                            |        // same thread cases. One signal from tlu covers the non-lsu cases
                            (i != tid_dc5     & (lsu_commit_dc5 & lsu_pkt_dc5.store & (~lsu_pkt_dc5.sc | lsu_sc_success_dc5) &
                             ((lsu_addr_dc5[31:2] == lr_addr[i][31:2]) | (end_addr_dc5[31:2] == lr_addr[i][31:2]))))                                                          |        // other thread case - any update to this location
                            dec_tlu_lr_reset_wb[i]                                                                                                                            |        // Reset from dec
                            (lsu_pkt_dc5.dma  & lsu_pkt_dc5.store & (lsu_addr_dc5[31:3] == lr_addr[i][31:3]) & (lsu_pkt_dc5.dword | (lsu_addr_dc5[2] == lr_addr[i][2])));              // DMA store case
      rvdffsc #(.WIDTH(1))  lr_vldff   (.din(1'b1),               .dout(lr_vld[i]),  .en(lr_wr_en[i]), .clear(lr_reset[i]), .clk(lsu_free_c2_clk), .*);
      rvdffs  #(.WIDTH(30)) lr_address (.din(lsu_addr_dc5[31:2]), .dout(lr_addr[i]), .en(lr_wr_en[i]),                      .clk(lsu_free_c2_clk), .*);
   end


endmodule
