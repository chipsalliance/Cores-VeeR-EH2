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


module eh2_dec_decode_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (
   input dec_i1_debug_valid_d,

   input logic dec_i0_csr_global_d,

   input eh2_predecode_pkt_t dec_i0_predecode,
   input eh2_predecode_pkt_t dec_i1_predecode,

   input logic [pt.NUM_THREADS-1:0] dec_tlu_force_halt, // invalidate nonblock load cam on a force halt event

   input logic [pt.NUM_THREADS-1:0] dec_tlu_debug_stall, // stall decode while waiting on core to empty

   input logic [pt.NUM_THREADS-1:0] dec_tlu_flush_extint,

   input logic dec_i0_tid_d,
   input logic dec_i1_tid_d,

   output logic dec_div_cancel,       // cancel divide operation

   output logic dec_extint_stall,


   input logic [15:0] dec_i0_cinst_d,         // 16b compressed instruction
   input logic [15:0] dec_i1_cinst_d,

   output logic [31:0] dec_i0_inst_wb1,       // 32b instruction at wb+1 for trace encoder
   output logic [31:0] dec_i1_inst_wb1,

   output logic [31:1] dec_i0_pc_wb1,         // 31b pc at wb+1 for trace encoder
   output logic [31:1] dec_i1_pc_wb1,


   output logic [pt.NUM_THREADS-1:0] dec_i1_cancel_e1,

   input logic [31:0] lsu_rs1_dc1,

   input logic                                lsu_nonblock_load_valid_dc1,     // valid nonblock load at dc3
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_tag_dc1,       // -> corresponding tag
   input logic                                lsu_nonblock_load_inv_dc2,       // invalidate request for nonblock load dc2
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_inv_tag_dc2,   // -> corresponding tag
   input logic                                lsu_nonblock_load_inv_dc5,       // invalidate request for nonblock load dc5
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_inv_tag_dc5,   // -> corresponding tag
   input logic                                lsu_nonblock_load_data_valid,    // valid nonblock load data back
   input logic                                lsu_nonblock_load_data_error,    // nonblock load bus error
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_data_tag,      // -> corresponding tag
   input logic                                lsu_nonblock_load_data_tid,


   input logic [31:0]                         lsu_nonblock_load_data,          // nonblock load data

   input logic [3:0] dec_i0_trigger_match_d,          // i0 decode trigger matches
   input logic [3:0] dec_i1_trigger_match_d,          // i1 decode trigger matches

   input logic [pt.NUM_THREADS-1:0]           dec_tlu_wr_pause_wb,                   // pause instruction at wb

   input logic dec_tlu_pipelining_disable,            // pipeline disable - presync, i0 decode only
   input logic dec_tlu_dual_issue_disable,            // i0 decode only

   input logic [3:0]  lsu_trigger_match_dc4,          // lsu trigger matches

   input logic[pt.NUM_THREADS-1:0] lsu_pmu_misaligned_dc3,                // perf mon: load/store misalign

   input logic [pt.NUM_THREADS-1:0] dec_tlu_flush_leak_one_wb,             // leak1 instruction

   input logic dec_debug_fence_d,                     // debug fence instruction

   input logic [1:0] dbg_cmd_wrdata,                  // disambiguate fence, fence_i

   input logic dec_i0_icaf_d,                         // icache access fault
   input logic dec_i1_icaf_d,
   input logic dec_i0_icaf_f1_d,                      // i0 instruction access fault at decode for f1 fetch group
   input logic dec_i1_icaf_f1_d,
   input logic [1:0] dec_i0_icaf_type_d,              // i0 instruction access fault type
   input logic [1:0] dec_i1_icaf_type_d,

   input logic dec_i0_dbecc_d,                        // icache/iccm double-bit error
   input logic dec_i1_dbecc_d,

   input eh2_br_pkt_t dec_i0_brp,                         // branch packet
   input eh2_br_pkt_t dec_i1_brp,
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_i0_bp_index,            // i0 branch index
   input logic [pt.BHT_GHR_SIZE-1:0] dec_i0_bp_fghr, // BP FGHR
   input logic [pt.BTB_BTAG_SIZE-1:0] dec_i0_bp_btag, // BP tag
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_i1_bp_index,            // i0 branch index
   input logic [pt.BHT_GHR_SIZE-1:0] dec_i1_bp_fghr, // BP FGHR
   input logic [pt.BTB_BTAG_SIZE-1:0] dec_i1_bp_btag, // BP tag

   input logic [pt.NUM_THREADS-1:0]  lsu_idle_any,                          // lsu idle: if fence instr & ~lsu_idle then stall decode
   input logic [pt.NUM_THREADS-1:0]  lsu_load_stall_any,                    // stall any load  at decode
   input logic [pt.NUM_THREADS-1:0]  lsu_store_stall_any,                   // stall any store at decode
   input logic [pt.NUM_THREADS-1:0]  lsu_amo_stall_any,         // This is for blocking amo

   input logic dma_dccm_stall_any,                    // stall any load/store at decode

   input logic exu_div_wren,                          // div finish this cycle

   input logic dec_tlu_i0_kill_writeb_wb,    // I0 is flushed, don't writeback any results to arch state
   input logic dec_tlu_i1_kill_writeb_wb,    // I1 is flushed, don't writeback any results to arch state

   input logic [pt.NUM_THREADS-1:0] dec_tlu_flush_lower_wb,          // trap lower flush

   input logic [pt.NUM_THREADS-1:0] dec_tlu_flush_pause_wb,          // don't clear pause state on initial lower flush

   input logic [pt.NUM_THREADS-1:0] dec_tlu_presync_d,               // CSR read needs to be presync'd
   input logic [pt.NUM_THREADS-1:0] dec_tlu_postsync_d,              // CSR ops that need to be postsync'd

   input logic [31:0] exu_mul_result_e3,        // multiply result

   input logic dec_i0_pc4_d,               // inst is 4B inst else 2B
   input logic dec_i1_pc4_d,



   input logic [31:0] lsu_result_dc3,      // load result
   input logic [31:0] lsu_result_corr_dc4, // load result - corrected data for writing gprs; not for bypassing

   input logic        lsu_sc_success_dc5,   // store conditional matched ( 1 = success, which means the GPR should write 0 )

   input logic [pt.NUM_THREADS-1:0] exu_i0_flush_final,         // lower flush or i0 flush at e2
   input logic [pt.NUM_THREADS-1:0] exu_i1_flush_final,         // lower flush or i1 flush at e2


   input logic [31:1] exu_i0_pc_e1,        // pcs at e1
   input logic [31:1] exu_i1_pc_e1,

   input logic [31:0] dec_i0_instr_d,      // inst at decode
   input logic [31:0] dec_i1_instr_d,

   input logic  dec_ib0_valid_d,          // inst valid at decode
   input logic  dec_ib1_valid_d,

   input logic [31:0] exu_i0_result_e1,    // from primary alu's
   input logic [31:0] exu_i1_result_e1,

   input logic [31:0] exu_i0_result_e4,    // from secondary alu's
   input logic [31:0] exu_i1_result_e4,

   input logic  clk,                       // for rvdffe's
   input logic  active_clk,                // clk except for halt / pause
   input logic  free_clk,                  // free running clock

   input logic  clk_override,              // test stuff
   input logic  rst_l,


   output logic         dec_i0_rs1_en_d,   // rs1 enable at decode
   output logic         dec_i0_rs2_en_d,

   output logic [4:0] dec_i0_rs1_d,        // rs1 logical source
   output logic [4:0] dec_i0_rs2_d,

   output logic dec_i0_tid_e4, // needed to maintain RS in BP
   output logic dec_i1_tid_e4,

   output logic [31:0] dec_i0_immed_d,     // 32b immediate data decode

   output logic          dec_i1_rs1_en_d,
   output logic          dec_i1_rs2_en_d,

   output logic [4:0]  dec_i1_rs1_d,
   output logic [4:0]  dec_i1_rs2_d,



   output logic [31:0] dec_i1_immed_d,

   output logic [12:1] dec_i0_br_immed_d,    // 12b branch immediate
   output logic [12:1] dec_i1_br_immed_d,

   output eh2_alu_pkt_t i0_ap,                   // alu packets
   output eh2_alu_pkt_t i1_ap,

   output logic          dec_i0_decode_d,    // i0 decode
   output logic          dec_i1_decode_d,

   output logic          dec_i0_alu_decode_d,   // decode to primary alu's
   output logic          dec_i1_alu_decode_d,


   output logic [31:0] i0_rs1_bypass_data_d,    // i0 rs1 bypass data
   output logic [31:0] i0_rs2_bypass_data_d,    // i0 rs2 bypass data
   output logic [31:0] i1_rs1_bypass_data_d,
   output logic [31:0] i1_rs2_bypass_data_d,


   output logic [4:0]  dec_i0_waddr_wb,         // i0 logical source to write to gpr's
   output logic          dec_i0_wen_wb,         // i0 write enable
   output logic          dec_i0_tid_wb,         // i0 write tid
   output logic [31:0] dec_i0_wdata_wb,         // i0 write data

   output logic [4:0]  dec_i1_waddr_wb,
   output logic          dec_i1_wen_wb,
   output logic          dec_i1_tid_wb,
   output logic [31:0] dec_i1_wdata_wb,

   output logic          dec_i0_select_pc_d,    // i0 select pc for rs1 - branches
   output logic          dec_i1_select_pc_d,

   output logic dec_i0_rs1_bypass_en_d,         // i0 rs1 bypass enable
   output logic dec_i0_rs2_bypass_en_d,         // i0 rs2 bypass enable
   output logic dec_i1_rs1_bypass_en_d,
   output logic dec_i1_rs2_bypass_en_d,

   output eh2_lsu_pkt_t    lsu_p,                   // load/store packet

   output eh2_mul_pkt_t    mul_p,                   // multiply packet

   output eh2_div_pkt_t    div_p,                   // divide packet
   output logic             div_tid_wb,              // DIV write tid     to GPR
   output logic [4:0]       div_waddr_wb,            // DIV write address to GPR

   output logic [11:0] dec_lsu_offset_d,
   output logic        dec_i0_lsu_d,        // chose which gpr value to use
   output logic        dec_i1_lsu_d,
   output logic        dec_i0_mul_d,        // chose which gpr value to use
   output logic        dec_i1_mul_d,

   output logic        dec_i0_div_d,        // chose which gpr value to use

   output logic [pt.NUM_THREADS-1:0]       flush_final_e3,      // flush final at e3: i0  or i1
   output logic [pt.NUM_THREADS-1:0]       i0_flush_final_e3,   // i0 flush final at e3

// CSR interface
   input logic [31:0]  dec_i0_csr_rddata_d,    // csr read data at wb
   input logic         dec_i0_csr_legal_d,            // csr indicates legal operation
   input logic [31:0]  exu_i0_csr_rs1_e1,      // rs1 for csr instr


   output logic        dec_i0_csr_ren_d,       // valid csr decode
   output logic        dec_i0_csr_wen_unq_d,       // valid csr with write - for csr legal
   output logic        dec_i0_csr_any_unq_d,       // valid csr - for csr legal
   output logic        dec_i0_csr_wen_wb,      // csr write enable at wb
   output logic [11:0] dec_i0_csr_rdaddr_d,      // read address for csr
   output logic [11:0] dec_i0_csr_wraddr_wb,     // write address for csr
   output logic [31:0] dec_i0_csr_wrdata_wb,   // csr write data at wb
   output logic        dec_i0_csr_is_mcpc_e4,     // csr address is to MCPC

   output logic [pt.NUM_THREADS-1:0] dec_csr_stall_int_ff, // csr is mie/mstatus

   output logic dec_csr_nmideleg_e4, // csr is mnmipdel

// end CSR interface

   output              dec_tlu_i0_valid_e4,  // i0 valid inst at e4
   output              dec_tlu_i1_valid_e4,

   output              eh2_trap_pkt_t dec_tlu_packet_e4,   // trap packet

   output logic [31:1] dec_tlu_i0_pc_e4,  // i0 trap pc
   output logic [31:1] dec_tlu_i1_pc_e4,


   output logic [pt.NUM_THREADS-1:0][31:0] dec_illegal_inst,

   output logic        dec_i1_valid_e1,         // i1 valid e1

   output logic [pt.NUM_THREADS-1:0][31:1] pred_correct_npc_e2, // npc e2 if the prediction is correct

   output logic        dec_i0_rs1_bypass_en_e3, // i0 rs1 bypass enables e3
   output logic        dec_i0_rs2_bypass_en_e3, // i1 rs1 bypass enables e3
   output logic        dec_i1_rs1_bypass_en_e3,
   output logic        dec_i1_rs2_bypass_en_e3,
   output logic [31:0] i0_rs1_bypass_data_e3,   // i0 rs1 bypass data e3
   output logic [31:0] i0_rs2_bypass_data_e3,   // i1 rs1 bypass data e3
   output logic [31:0] i1_rs1_bypass_data_e3,
   output logic [31:0] i1_rs2_bypass_data_e3,
   output logic        dec_i0_sec_decode_e3,    // i0 secondary alu e3
   output logic        dec_i1_sec_decode_e3,    // i1 secondary alu e3
   output logic [31:1] dec_i0_pc_e3,            // i0 pc e3
   output logic [31:1] dec_i1_pc_e3,            // i1 pc e3

   output logic        dec_i0_rs1_bypass_en_e2, // i0 rs1 bypass enable e2
   output logic        dec_i0_rs2_bypass_en_e2, // i0 rs2 bypass enable e2
   output logic        dec_i1_rs1_bypass_en_e2,
   output logic        dec_i1_rs2_bypass_en_e2,
   output logic [31:0] i0_rs1_bypass_data_e2,   // i0 rs1 bypass data e2
   output logic [31:0] i0_rs2_bypass_data_e2,   // i0 rs2 bypass data e2
   output logic [31:0] i1_rs1_bypass_data_e2,
   output logic [31:0] i1_rs2_bypass_data_e2,

   output eh2_predict_pkt_t  i0_predict_p_d,        // i0 predict packet decode
   output eh2_predict_pkt_t  i1_predict_p_d,
   output logic [pt.BHT_GHR_SIZE-1:0] i0_predict_fghr_d, // i0 predict fghr
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] i0_predict_index_d, // i0 predict index
   output logic [pt.BTB_BTAG_SIZE-1:0] i0_predict_btag_d, // i0_predict branch tag

   output logic [pt.BHT_GHR_SIZE-1:0] i1_predict_fghr_d, // i1 predict fghr
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] i1_predict_index_d, // i1 predict index
   output logic [pt.BTB_BTAG_SIZE-1:0] i1_predict_btag_d, // i1_predict branch tag

   output logic [31:0] i0_result_e4_eff,        // i0 e4 result
   output logic [31:0] i1_result_e4_eff,
   output logic [31:0] i0_result_e2,            // i0 result e2

   output logic [4:2] dec_i0_data_en,           // clock-gating logic
   output logic [4:1] dec_i0_ctl_en,
   output logic [4:2] dec_i1_data_en,
   output logic [4:1] dec_i1_ctl_en,

   output logic [pt.NUM_THREADS-1:0][1:0] dec_pmu_instr_decoded,    // number of instructions decode this cycle encoded

   output logic [pt.NUM_THREADS-1:0]   dec_pmu_decode_stall,     // decode is stalled

   output logic [pt.NUM_THREADS-1:0]      dec_pmu_presync_stall,    // decode has presync stall
   output logic [pt.NUM_THREADS-1:0]      dec_pmu_postsync_stall,   // decode has postsync stall

   output logic [pt.NUM_THREADS-1:0]      dec_nonblock_load_wen,        // write enable for nonblock load
   output logic [pt.NUM_THREADS-1:0][4:0] dec_nonblock_load_waddr,      // logical write addr for nonblock load


   output logic [pt.NUM_THREADS-1:0]      dec_pause_state,              // core in pause state
   output logic       dec_pause_state_cg,           // core in pause state for clock-gating

   output logic [pt.NUM_THREADS-1:0]      dec_thread_stall_in,       // thread is known to stall next cycle - eg pause

   output logic        dec_div_active,     // non-block divide is active
   output logic        dec_div_tid,        // non-block divide tid
   input  logic        scan_mode
   );




   eh2_dec_pkt_t i0_dp_raw, i0_dp;
   eh2_dec_pkt_t i1_dp_raw, i1_dp;



   logic [31:0]        i0, i1;
   logic               i0_valid_d, i1_valid_d;

   logic [31:0]        i0_result_e1, i1_result_e1;
   logic [31:0]                      i1_result_e2;
   logic [31:0]        i0_result_e3, i1_result_e3;
   logic [31:0]        i0_result_e4, i1_result_e4;
   logic [31:0]        i0_result_wb, i1_result_wb;

   logic [31:1]        i0_pc_e1, i1_pc_e1;
   logic [31:1]        i0_pc_e2, i1_pc_e2;
   logic [31:1]        i0_pc_e3, i1_pc_e3;
   logic [31:1]        i0_pc_e4, i1_pc_e4;

   logic [9:0]         i0_rs1bypass, i0_rs2bypass;
   logic [9:0]         i1_rs1bypass, i1_rs2bypass;

   logic               i0_jalimm20, i1_jalimm20;
   logic               i0_uiimm20, i1_uiimm20;

   logic               lsu_decode_d;
   logic [31:0]        i0_immed_d;
   logic [31:0]        i1_immed_d;
   logic               i0_presync;
   logic               i0_postsync;

   logic [pt.NUM_THREADS-1:0]    presync_stall;
   logic [pt.NUM_THREADS-1:0]    postsync_stall_in, postsync_stall;
   logic [pt.NUM_THREADS-1:0]    base_postsync_stall_in, base_postsync_stall;
   logic [pt.NUM_THREADS-1:0]    jal_postsync_stall_in, jal_postsync_stall;
   logic [pt.NUM_THREADS-1:0]    prior_inflight, prior_inflight_e1e3, prior_inflight_e1e4, prior_inflight_wb;
   logic [pt.NUM_THREADS-1:0]    prior_csr_write, prior_csr_write_e1e4;
   logic                         prior_any_csr_write_any_thread, prior_any_csr_write_any_thread_e1e4;

   logic     i0_csr_clr_d, i0_csr_set_d, i0_csr_write_d;

   logic        i0_csr_clr_e1,i0_csr_set_e1,i0_csr_write_e1,i0_csr_imm_e1;

   logic [31:0] i0_csr_mask_e1;
   logic [31:0] i0_write_csr_data_e1;

   logic [pt.NUM_THREADS-1:0][31:0] write_csr_data_in;
   logic [pt.NUM_THREADS-1:0][31:0] write_csr_data;
   logic [pt.NUM_THREADS-1:0]       csr_data_wen;

   logic [4:0]         i0_csrimm_e1;
   logic [31:0]        i0_csr_rddata_e1;

   logic               i1_load_block_d;
   logic               i1_mul_block_d, i1_mul_block_thread_1cycle_d;
   logic               i1_load2_block_d;
   logic               i1_mul2_block_d;
   logic               mul_decode_d;

   logic               i0_legal, i1_legal;

   logic [pt.NUM_THREADS-1:0]         shift_illegal;
   logic [pt.NUM_THREADS-1:0]         illegal_inst_en;
   logic [pt.NUM_THREADS-1:0]         illegal_lockout_in, illegal_lockout;

   logic               i0_legal_decode_d, i1_legal_decode_d;

   logic [31:0]        i0_result_e3_final, i1_result_e3_final;
   logic [31:0]        i0_result_wb_raw,   i1_result_wb_raw;

   logic [pt.NUM_THREADS-1:0][12:1]        last_br_immed_d, last_br_immed_e1, last_br_immed_e2;
   logic [pt.NUM_THREADS-1:0][31:1]        last_pc_e2;

   logic        i1_depend_i0_d;
   logic        i0_rs1_depend_i0_e1, i0_rs1_depend_i0_e2, i0_rs1_depend_i0_e3, i0_rs1_depend_i0_e4, i0_rs1_depend_i0_wb;
   logic        i0_rs1_depend_i1_e1, i0_rs1_depend_i1_e2, i0_rs1_depend_i1_e3, i0_rs1_depend_i1_e4, i0_rs1_depend_i1_wb;
   logic        i0_rs2_depend_i0_e1, i0_rs2_depend_i0_e2, i0_rs2_depend_i0_e3, i0_rs2_depend_i0_e4, i0_rs2_depend_i0_wb;
   logic        i0_rs2_depend_i1_e1, i0_rs2_depend_i1_e2, i0_rs2_depend_i1_e3, i0_rs2_depend_i1_e4, i0_rs2_depend_i1_wb;
   logic        i1_rs1_depend_i0_e1, i1_rs1_depend_i0_e2, i1_rs1_depend_i0_e3, i1_rs1_depend_i0_e4, i1_rs1_depend_i0_wb;
   logic        i1_rs1_depend_i1_e1, i1_rs1_depend_i1_e2, i1_rs1_depend_i1_e3, i1_rs1_depend_i1_e4, i1_rs1_depend_i1_wb;
   logic        i1_rs2_depend_i0_e1, i1_rs2_depend_i0_e2, i1_rs2_depend_i0_e3, i1_rs2_depend_i0_e4, i1_rs2_depend_i0_wb;
   logic        i1_rs2_depend_i1_e1, i1_rs2_depend_i1_e2, i1_rs2_depend_i1_e3, i1_rs2_depend_i1_e4, i1_rs2_depend_i1_wb;
   logic        i1_rs1_depend_i0_d, i1_rs2_depend_i0_d;

   logic        i0_secondary_d, i1_secondary_d;
   logic        i0_secondary_block_d, i1_secondary_block_d;
   logic        non_block_case_d;
   logic        i0_div_decode_d;
   logic [31:0] i0_result_e4_final, i1_result_e4_final;
   logic        i0_load_block_d;
   logic        i0_mul_block_d, i0_mul_block_thread_1cycle_d;
   logic [3:0]  i0_rs1_depth_d, i0_rs2_depth_d;
   logic [3:0]  i1_rs1_depth_d, i1_rs2_depth_d;

   logic        i0_rs1_match_e1_e2, i0_rs1_match_e1_e3;
   logic        i0_rs2_match_e1_e2, i0_rs2_match_e1_e3;
   logic        i1_rs1_match_e1_e2, i1_rs1_match_e1_e3;
   logic        i1_rs2_match_e1_e2, i1_rs2_match_e1_e3;

   logic        i0_amo_stall_d;
   logic        i0_load_stall_d,  i1_load_stall_d;
   logic        i0_store_stall_d, i1_store_stall_d;

   logic        i0_predict_nt, i0_predict_t;
   logic        i1_predict_nt, i1_predict_t;

   logic        i0_notbr_error, i0_br_toffset_error;
   logic        i1_notbr_error, i1_br_toffset_error;
   logic        i0_ret_error,   i1_ret_error;
   logic        i0_br_error, i1_br_error;
   logic        i0_br_error_all, i1_br_error_all;
   logic [11:0] i0_br_offset, i1_br_offset;

   logic [20:1] i0_pcall_imm, i1_pcall_imm;    // predicted jal's
   logic        i0_pcall_12b_offset, i1_pcall_12b_offset;
   logic        i0_pcall_raw,   i1_pcall_raw;
   logic        i0_pcall_case,  i1_pcall_case;
   logic        i0_pcall,  i1_pcall;

   logic        i0_pja_raw,   i1_pja_raw;
   logic        i0_pja_case,  i1_pja_case;
   logic        i0_pja,  i1_pja;

   logic        i0_pret_case, i1_pret_case;
   logic        i0_pret_raw, i0_pret;
   logic        i1_pret_raw, i1_pret;

   logic        i0_jal, i1_jal;  // jal's that are not predicted


   logic        i0_predict_br, i1_predict_br;

   logic [31:0] i1_result_wb_eff, i0_result_wb_eff;
   logic [2:0]  i1rs1_intra, i1rs2_intra;
   logic        i1_rs1_intra_bypass, i1_rs2_intra_bypass;
   logic        store_data_bypass_c1, store_data_bypass_c2;
   logic [1:0]  store_data_bypass_e4_c1, store_data_bypass_e4_c2, store_data_bypass_e4_c3;
   logic        store_data_bypass_i0_e2_c2;

   eh2_class_pkt_t i0_rs1_class_d, i0_rs2_class_d;
   eh2_class_pkt_t i1_rs1_class_d, i1_rs2_class_d;

   eh2_class_pkt_t i0_dc, i0_e1c, i0_e2c, i0_e3c, i0_e4c, i0_wbc;
   eh2_class_pkt_t i1_dc, i1_e1c, i1_e2c, i1_e3c, i1_e4c, i1_wbc;


   logic i0_rs1_match_e1, i0_rs1_match_e2, i0_rs1_match_e3;
   logic i1_rs1_match_e1, i1_rs1_match_e2, i1_rs1_match_e3;
   logic i0_rs2_match_e1, i0_rs2_match_e2, i0_rs2_match_e3;
   logic i1_rs2_match_e1, i1_rs2_match_e2, i1_rs2_match_e3;

   logic       i0_secondary_stall_d;

   logic       i0_ap_pc2, i0_ap_pc4;
   logic       i1_ap_pc2, i1_ap_pc4;

   logic        i0_rd_en_d;
   logic        i1_rd_en_d;

   logic        load_ldst_bypass_c1;
   logic        load_mul_rs1_bypass_e1;
   logic        load_mul_rs2_bypass_e1;

   logic [pt.NUM_THREADS-1:0] leak1_i0_stall_in, leak1_i0_stall;
   logic [pt.NUM_THREADS-1:0] leak1_i1_stall_in, leak1_i1_stall;
   logic [pt.NUM_THREADS-1:0] leak1_mode;

   logic        i0_csr_write_only_d;

   logic        i0_any_csr_d;


   logic [5:0] i0_pipe_en;
   logic       i0_e1_ctl_en, i0_e2_ctl_en, i0_e3_ctl_en, i0_e4_ctl_en, i0_wb_ctl_en;
   logic       i0_e1_data_en, i0_e2_data_en, i0_e3_data_en, i0_e4_data_en, i0_wb_data_en, i0_wb1_data_en;

   logic [5:0] i1_pipe_en;
   logic       i1_e1_ctl_en, i1_e2_ctl_en, i1_e3_ctl_en, i1_e4_ctl_en, i1_wb_ctl_en;
   logic       i1_e1_data_en, i1_e2_data_en, i1_e3_data_en, i1_e4_data_en, i1_wb_data_en, i1_wb1_data_en;

   logic debug_fence_i;
   logic debug_fence;

   logic i0_csr_write;

   logic i0_instr_error;
   logic i0_icaf_d;
   logic i1_icaf_d;

   logic i0_not_alu_eff, i1_not_alu_eff;

   logic [pt.NUM_THREADS-1:0]   clear_pause;
   logic [pt.NUM_THREADS-1:0]   pause_state_in, pause_state;
   logic [pt.NUM_THREADS-1:0]   pause_stall;

   logic [31:1] i1_pc_wb;

   logic        i0_brp_valid;

   logic [pt.NUM_THREADS-1:0]   lsu_idle;
   logic        i0_csr_read_e1;
   logic        i0_block_d;
   logic        i1_block_d;


   eh2_inst_pkt_t                  i0_itype, i1_itype;

   logic                            i0_br_unpred, i1_br_unpred;

   logic [pt.NUM_THREADS-1:0]       flush_final_lower, flush_final_upper_e2;

   eh2_reg_pkt_t                   i0r, i1r;
   logic                            i1_cancel_d, i1_cancel_e1;

   logic [4:0]                      nonblock_load_rd;
   logic                            nonblock_load_tid_dc1;
   logic                            i1_wen_wb, i0_wen_wb;

   logic [pt.NUM_THREADS-1:0] [4:0] cam_nonblock_load_waddr;
   logic [pt.NUM_THREADS-1:0]       cam_nonblock_load_wen;
   logic [pt.NUM_THREADS-1:0]       cam_i0_nonblock_load_stall;
   logic [pt.NUM_THREADS-1:0]       cam_i1_nonblock_load_stall;
   logic [pt.NUM_THREADS-1:0]       cam_i0_load_kill_wen;
   logic [pt.NUM_THREADS-1:0]       cam_i1_load_kill_wen;

   logic [0:0]                      tlu_wr_pause_wb1;  // leave stranded so it is clear this is thread 0 only
   logic [0:0]                      tlu_wr_pause_wb2;  // leave stranded so it is clear this is thread 0 only

   logic                            debug_fence_raw;
   eh2_trap_pkt_t                  dt, e1t_in, e1t, e2t_in, e2t, e3t_in, e3t, e4t_ff, e4t;


   logic [31:0]        i0_inst_d, i1_inst_d;
   logic [31:0]        i0_inst_e1, i1_inst_e1;
   logic [31:0]        i0_inst_e2, i1_inst_e2;
   logic [31:0]        i0_inst_e3, i1_inst_e3;
   logic [31:0]        i0_inst_e4, i1_inst_e4;
   logic [31:0]        i0_inst_wb, i1_inst_wb;
   logic [31:0]        i0_inst_wb1,i1_inst_wb1;

   eh2_dest_pkt_t     dd, e1d, e2d, e3d, e4d, wbd;
   eh2_class_pkt_t    i0_e4c_in, i1_e4c_in;
   eh2_dest_pkt_t     e1d_in, e2d_in, e3d_in, e4d_in;

   logic [31:1] i0_pc_wb, i0_pc_wb1;
   logic [31:1]           i1_pc_wb1;

   logic [pt.NUM_THREADS-1:0][31:0] illegal_inst;

   // new flush logic
   logic [pt.NUM_THREADS-1:0] i1_flush_final_e3;
   logic [pt.NUM_THREADS-1:0] i0_flush_final_e4;

   logic i1_block_same_thread_d;

   logic [pt.NUM_THREADS-1:0] flush_lower_wb;

   logic [pt.NUM_THREADS-1:0] flush_extint;

   logic i0_csr_update_e1;

   logic [pt.NUM_THREADS-1:0]       csr_update_e1;
   logic [pt.NUM_THREADS-1:0][31:0] write_csr_data_e1;
   logic [pt.NUM_THREADS-1:0][31:0] write_csr_data_wb;

   logic i0_csr_legal_d;

   logic lsu_tid_e3;

   logic div_stall;
   logic div_tid;
   logic div_active, div_active_in;
   logic div_valid;
   logic [4:0] div_rd;
   logic i0_nonblock_div_stall, i1_nonblock_div_stall;
   logic div_e1_to_wb;
   logic div_flush;
   logic nonblock_div_cancel;

   logic i0_div_prior_div_stall;

   logic i1_secondary_block_thread_1cycle_d, i0_secondary_block_thread_1cycle_d;
   logic i1_secondary_block_thread_2cycle_d, i0_secondary_block_thread_2cycle_d;
   logic i0_secondary_stall_1cycle_d, i0_secondary_stall_2cycle_d;
   logic i0_secondary_stall_thread_1cycle_d, i0_secondary_stall_thread_2cycle_d;

   logic i1_br_error_fast, i0_br_error_fast;


   logic i0_legal_except_csr;

   logic [pt.NUM_THREADS-1:0] flush_all;
   logic [pt.NUM_THREADS-1:0] smt_secondary_stall_in, smt_secondary_stall, smt_secondary_stall_raw;
   logic [pt.NUM_THREADS-1:0] set_smt_presync_stall;
   logic [pt.NUM_THREADS-1:0] smt_presync_stall_in, smt_presync_stall, smt_presync_stall_raw;
   logic [pt.NUM_THREADS-1:0] set_smt_csr_write_stall;
   logic [pt.NUM_THREADS-1:0] smt_csr_write_stall_in, smt_csr_write_stall, smt_csr_write_stall_raw;

   logic [pt.NUM_THREADS-1:0] set_smt_atomic_stall;
   logic [pt.NUM_THREADS-1:0] smt_atomic_stall_in, smt_atomic_stall, smt_atomic_stall_raw;

   logic [pt.NUM_THREADS-1:0] set_smt_div_stall;
   logic [pt.NUM_THREADS-1:0] smt_div_stall_in, smt_div_stall, smt_div_stall_raw;

   logic [pt.NUM_THREADS-1:0] set_smt_nonblock_load_stall;
   logic [pt.NUM_THREADS-1:0] smt_nonblock_load_stall_in, smt_nonblock_load_stall, smt_nonblock_load_stall_raw;
   logic [pt.NUM_THREADS-1:0] cam_nonblock_load_stall;

   logic nonblock_load_tid_dc2, nonblock_load_tid_dc5, i0_rs1_nonblock_load_bypass_en_d, i0_rs2_nonblock_load_bypass_en_d, i1_rs1_nonblock_load_bypass_en_d, i1_rs2_nonblock_load_bypass_en_d;

   typedef struct packed {
                          logic csr_read_stall;
                          logic extint_stall;
                          logic i1_cancel_e1_stall;
                          logic pause_stall;
                          logic leak1_stall;
                          logic debug_stall;
                          logic postsync_stall;
                          logic presync_stall;
                          logic wait_lsu_idle_stall;
                          logic nonblock_load_stall;
                          logic nonblock_div_stall;
                          logic prior_div_stall;
                          logic load_stall;
                          logic store_stall;
                          logic amo_stall;
                          logic load_block;
                          logic mul_block;
                          logic secondary_block;
                          logic secondary_stall;
                          } i0_block_pkt_t;

    typedef struct packed {
                           logic debug_valid_stall;
                           logic nonblock_load_stall;
                           logic wait_lsu_idle_stall;
                           logic extint_stall;
                           logic i1_cancel_e1_stall;
                           logic pause_stall;
                           logic debug_stall;
                           logic postsync_stall;
                           logic presync_stall;
                           logic nonblock_div_stall;
                           logic load_stall;
                           logic store_stall;
                           logic load_block;
                           logic mul_block;
                           logic load2_block;
                           logic mul2_block;
                           logic secondary_block;
                           logic leak1_stall;
                           logic i0_only_block;
                           logic icaf_block;
                           logic barrier_block;
                           logic block_same_thread;
                           } i1_block_pkt_t;


   i0_block_pkt_t i0blockp;
   i1_block_pkt_t i1blockp;

   logic i1_depend_i0_case_d;


// branch prediction

   // in leak1_mode, ignore any predictions for i0, treat branch as if we haven't seen it before
   // in leak1 mode, also ignore branch errors for i0
   assign i0_brp_valid = dec_i0_brp.valid & ~leak1_mode[dd.i0tid];

   assign      i0_predict_p_d.misp    =  '0;
   assign      i0_predict_p_d.ataken  =  '0;
   assign      i0_predict_p_d.boffset =  '0;

   assign      i0_predict_p_d.pcall  =  i0_pcall;  // dont mark as pcall if branch error
   assign      i0_predict_p_d.pja    =  i0_pja;
   assign      i0_predict_p_d.pret   =  i0_pret;
   assign      i0_predict_p_d.prett[31:1] = dec_i0_brp.prett[31:1];
   assign      i0_predict_p_d.pc4 = dec_i0_pc4_d;
   assign      i0_predict_p_d.hist[1:0] = dec_i0_brp.hist[1:0];
   assign      i0_predict_p_d.valid = i0_brp_valid & i0_legal_decode_d;
   assign      i0_notbr_error = i0_brp_valid & ~(i0_dp_raw.condbr | i0_pcall_raw | i0_pja_raw | i0_pret_raw);

   // no toffset error for a pret
   assign      i0_br_toffset_error = i0_brp_valid & dec_i0_brp.hist[1] & (dec_i0_brp.toffset[11:0] != i0_br_offset[11:0]) & !i0_pret_raw;
   assign      i0_ret_error = i0_brp_valid & dec_i0_brp.ret & ~i0_pret_raw;
   assign      i0_br_error =  dec_i0_brp.br_error | i0_notbr_error | i0_br_toffset_error | i0_ret_error;
   assign      i0_predict_p_d.br_error = i0_br_error & i0_legal_decode_d & ~leak1_mode[dd.i0tid];
   assign      i0_predict_p_d.br_start_error = dec_i0_brp.br_start_error & i0_legal_decode_d & ~leak1_mode[dd.i0tid];

   assign      i0_predict_p_d.bank = dec_i0_brp.bank;

   assign      i0_br_error_all = (i0_br_error | dec_i0_brp.br_start_error) & ~leak1_mode[dd.i0tid];

   // timing limits what can be done
   assign      i0_br_error_fast = (dec_i0_brp.br_error | dec_i0_brp.br_start_error) & ~leak1_mode[dd.i0tid];

   assign      i0_predict_p_d.toffset[11:0] = i0_br_offset[11:0];

   assign      i0_predict_p_d.way = dec_i0_brp.way;
   assign      i0_predict_index_d[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] =  dec_i0_bp_index;
   assign      i0_predict_btag_d[pt.BTB_BTAG_SIZE-1:0]            =  dec_i0_bp_btag[pt.BTB_BTAG_SIZE-1:0];
   assign      i0_predict_fghr_d[pt.BHT_GHR_SIZE-1:0]                =  dec_i0_bp_fghr[pt.BHT_GHR_SIZE-1:0];


   assign      i1_predict_p_d.misp    =  '0;
   assign      i1_predict_p_d.ataken  =  '0;
   assign      i1_predict_p_d.boffset =  '0;

   assign      i1_predict_p_d.pcall  =  i1_pcall;
   assign      i1_predict_p_d.pja    =  i1_pja;
   assign      i1_predict_p_d.pret   =  i1_pret;
   assign      i1_predict_p_d.prett[31:1] = dec_i1_brp.prett[31:1];
   assign      i1_predict_p_d.pc4 = dec_i1_pc4_d;
   assign      i1_predict_p_d.hist[1:0] = dec_i1_brp.hist[1:0];
   assign      i1_predict_p_d.valid = dec_i1_brp.valid & i1_legal_decode_d;
   assign      i1_notbr_error = dec_i1_brp.valid & ~(i1_dp_raw.condbr | i1_pcall_raw | i1_pja_raw | i1_pret_raw);


   assign      i1_br_toffset_error = dec_i1_brp.valid & dec_i1_brp.hist[1] & (dec_i1_brp.toffset[11:0] != i1_br_offset[11:0]) & !i1_pret_raw;
   assign      i1_ret_error = dec_i1_brp.valid & dec_i1_brp.ret & ~i1_pret_raw;
   assign      i1_br_error = dec_i1_brp.br_error | i1_notbr_error | i1_br_toffset_error | i1_ret_error;
   assign      i1_predict_p_d.br_error = i1_br_error & i1_legal_decode_d;
   assign      i1_predict_p_d.br_start_error = dec_i1_brp.br_start_error & i1_legal_decode_d;
   assign      i1_predict_p_d.bank = dec_i1_brp.bank;

   assign      i1_br_error_all = (i1_br_error | dec_i1_brp.br_start_error);

   assign      i1_br_error_fast = (dec_i1_brp.br_error | dec_i1_brp.br_start_error);


   assign      i1_predict_p_d.toffset[11:0] = i1_br_offset[11:0];
   assign      i1_predict_p_d.way = dec_i1_brp.way;
   assign      i1_predict_index_d[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] =  dec_i1_bp_index;
   assign      i1_predict_btag_d[pt.BTB_BTAG_SIZE-1:0]           =  dec_i1_bp_btag[pt.BTB_BTAG_SIZE-1:0];
   assign      i1_predict_fghr_d[pt.BHT_GHR_SIZE-1:0]            =  dec_i1_bp_fghr[pt.BHT_GHR_SIZE-1:0];

   //   end

   // on br error turn anything into a nop
   // on i0 instruction fetch access fault turn anything into a nop
   // nop =>   alu rs1 imm12 rd lor

   assign i0_icaf_d = dec_i0_icaf_d | dec_i0_dbecc_d;
   assign i1_icaf_d = dec_i1_icaf_d | dec_i1_dbecc_d;


   assign i0_instr_error = i0_icaf_d;

   always_comb begin
      i0_dp = i0_dp_raw;

      if (i0_br_error_fast | i0_instr_error) begin
      i0_dp = '0;
      i0_dp.alu = 1'b1;
      i0_dp.rs1 = 1'b1;
      i0_dp.rs2 = 1'b1;
      i0_dp.lor = 1'b1;
      i0_dp.legal = 1'b1;
      i0_dp.postsync = 1'b1;
      i0_dp.i0_only = 1'b1;
      end

      i1_dp = i1_dp_raw;

      if (i1_br_error_fast) begin
         i1_dp = '0;
         i1_dp.alu = 1'b1;
         i1_dp.rs1 = 1'b1;
         i1_dp.rs2 = 1'b1;
         i1_dp.lor = 1'b1;
         i1_dp.legal = 1'b1;
         i1_dp.postsync = 1'b1;
         i1_dp.i0_only = 1'b1;
      end

   end


   assign flush_lower_wb[pt.NUM_THREADS-1:0] = dec_tlu_flush_lower_wb[pt.NUM_THREADS-1:0];


   assign i0[31:0] = dec_i0_instr_d[31:0];

   assign i1[31:0] = dec_i1_instr_d[31:0];

   assign dec_i0_select_pc_d = i0_dp.pc;
   assign dec_i1_select_pc_d = i1_dp.pc;

   // branches that can be predicted

   assign i0_predict_br =  i0_dp.condbr | i0_pcall | i0_pja | i0_pret;
   assign i1_predict_br =  i1_dp.condbr | i1_pcall | i1_pja | i1_pret;

   assign i0_predict_nt = ~(dec_i0_brp.hist[1] & i0_brp_valid) & i0_predict_br;
   assign i0_predict_t  =  (dec_i0_brp.hist[1] & i0_brp_valid) & i0_predict_br;

   assign i0_ap.add =    i0_dp.add;
   assign i0_ap.sub =    i0_dp.sub;
   assign i0_ap.land =   i0_dp.land;
   assign i0_ap.lor =    i0_dp.lor;
   assign i0_ap.lxor =   i0_dp.lxor;
   assign i0_ap.sll =    i0_dp.sll;
   assign i0_ap.srl =    i0_dp.srl;
   assign i0_ap.sra =    i0_dp.sra;
   assign i0_ap.slt =    i0_dp.slt;
   assign i0_ap.unsign = i0_dp.unsign;
   assign i0_ap.beq =    i0_dp.beq;
   assign i0_ap.bne =    i0_dp.bne;
   assign i0_ap.blt =    i0_dp.blt;
   assign i0_ap.bge =    i0_dp.bge;


   assign i0_ap.csr_write = i0_csr_write_only_d;
   assign i0_ap.csr_imm = i0_dp.csr_imm;


   assign i0_ap.jal    =  i0_jal;


   assign i0_ap_pc2 = ~dec_i0_pc4_d;
   assign i0_ap_pc4 =  dec_i0_pc4_d;

   assign i0_ap.predict_nt = i0_predict_nt;
   assign i0_ap.predict_t  = i0_predict_t;

   assign i0_ap.tid = dd.i0tid;

   assign i1_predict_nt = ~(dec_i1_brp.hist[1] & dec_i1_brp.valid) & i1_predict_br;
   assign i1_predict_t  =  (dec_i1_brp.hist[1] & dec_i1_brp.valid) & i1_predict_br;

   assign i1_ap.add =    i1_dp.add;
   assign i1_ap.sub =    i1_dp.sub;
   assign i1_ap.land =   i1_dp.land;
   assign i1_ap.lor =    i1_dp.lor;
   assign i1_ap.lxor =   i1_dp.lxor;
   assign i1_ap.sll =    i1_dp.sll;
   assign i1_ap.srl =    i1_dp.srl;
   assign i1_ap.sra =    i1_dp.sra;
   assign i1_ap.slt =    i1_dp.slt;
   assign i1_ap.unsign = i1_dp.unsign;
   assign i1_ap.beq =    i1_dp.beq;
   assign i1_ap.bne =    i1_dp.bne;
   assign i1_ap.blt =    i1_dp.blt;
   assign i1_ap.bge =    i1_dp.bge;


   assign i1_ap.csr_write = 1'b0;
   assign i1_ap.csr_imm   = 1'b0;

   assign i1_ap.jal    =    i1_jal;

   assign i1_ap_pc2 = ~dec_i1_pc4_d;
   assign i1_ap_pc4 =  dec_i1_pc4_d;

   assign i1_ap.predict_nt = i1_predict_nt;
   assign i1_ap.predict_t  = i1_predict_t;
   assign i1_cancel_d = i0_dp.load & i1_depend_i0_d & i1_legal_decode_d & ~i1_br_error_all;  // no decode if flush

   assign i1_ap.tid = dd.i1tid;


   rvdff #(1) cancel_ff (.*, .clk(active_clk), .din(i1_cancel_d),  .dout(i1_cancel_e1) );


   // i1_cancel_e1 == 1 iff i0tid == i1tid

   always_comb begin

      dec_i1_cancel_e1 = '0;

      dec_i1_cancel_e1[e1d.i1tid] = ~(lsu_rs1_dc1[31:28]==pt.DCCM_REGION | lsu_rs1_dc1[31:28]==pt.PIC_REGION) & i1_cancel_e1 &
                                    ~flush_final_e3[e1d.i1tid] &
                                    ~flush_lower_wb[e1d.i1tid];
   end

// START: non block load cam logic

   assign nonblock_load_rd[4:0] = (e1d.i0load) ? e1d.i0rd[4:0] : e1d.i1rd[4:0];  // rd data
   // threaded
   assign nonblock_load_tid_dc1 = e1d.lsu_tid;
   assign nonblock_load_tid_dc2 = e2d.lsu_tid;
   assign nonblock_load_tid_dc5 = wbd.lsu_tid;


   for (genvar i=0; i<pt.NUM_THREADS; i++) begin : cam

      eh2_dec_cam #(.pt(pt)) cam (
                        .tid                     (1'(i)),
                        .flush                   (flush_all[i]),
                        .dec_tlu_force_halt      (dec_tlu_force_halt[i]),
                        .nonblock_load_waddr     (cam_nonblock_load_waddr[i]   ),
                        .nonblock_load_wen       (cam_nonblock_load_wen[i]     ),
                        .i0_nonblock_load_stall  (cam_i0_nonblock_load_stall[i]),
                        .i1_nonblock_load_stall  (cam_i1_nonblock_load_stall[i]),
                        .i0_load_kill_wen        (cam_i0_load_kill_wen[i]      ),
                        .i1_load_kill_wen        (cam_i1_load_kill_wen[i]      ),
                        .nonblock_load_stall     (cam_nonblock_load_stall[i]   ),
                        .*
                        );

   end


   assign dec_nonblock_load_waddr[pt.NUM_THREADS-1:0]  = cam_nonblock_load_waddr[pt.NUM_THREADS-1:0];
   assign dec_nonblock_load_wen[pt.NUM_THREADS-1:0]    = cam_nonblock_load_wen[pt.NUM_THREADS-1:0];

// END non block load cam logic

// pmu start

   assign i0_br_unpred = (i0_dp.condbr | i0_dp.jal) & ~i0_predict_br;
   assign i1_br_unpred = (i1_dp.condbr | i1_dp.jal) & ~i1_predict_br;

   // the classes must be mutually exclusive with one another

   always_comb begin
      i0_itype = NULL;
      i1_itype = NULL;

      if (i0_legal_decode_d & ~i0_br_error_all) begin
         if (i0_dp.mul)                  i0_itype = MUL;
         if (i0_dp.load)                 i0_itype = LOAD;
         if (i0_dp.store)                i0_itype = STORE;
         if (i0_dp.pm_alu)               i0_itype = ALU;
         if (i0_dp.atomic & ~(i0_dp.lr | i0_dp.sc))
                                         i0_itype = ATOMIC;
         if (i0_dp.lr)                   i0_itype = LR;
         if (i0_dp.sc)                   i0_itype = SC;
         if ( dec_i0_csr_ren_d & ~dec_i0_csr_wen_unq_d)     i0_itype = CSRREAD;
         if (~dec_i0_csr_ren_d &  dec_i0_csr_wen_unq_d)     i0_itype = CSRWRITE;
         if ( dec_i0_csr_ren_d &  dec_i0_csr_wen_unq_d)     i0_itype = CSRRW;
         if (i0_dp.ebreak)               i0_itype = EBREAK;
         if (i0_dp.ecall)                i0_itype = ECALL;
         if (i0_dp.fence)                i0_itype = FENCE;
         if (i0_dp.fence_i)              i0_itype = FENCEI;  // fencei will set this even with fence attribute
         if (i0_dp.mret)                 i0_itype = MRET;
         if (i0_dp.condbr)               i0_itype = CONDBR;
         if (i0_dp.jal)                  i0_itype = JAL;
      end

      if (i1_legal_decode_d & ~i1_br_error_all) begin
         if (i1_dp.ebreak)               i1_itype = EBREAK;   // this is based on doing SMT
         if (i1_dp.ecall)                i1_itype = ECALL;
         if (i1_dp.mret)                 i1_itype = MRET;


         if (i1_dp.mul)                  i1_itype = MUL;
         if (i1_dp.load)                 i1_itype = LOAD;
         if (i1_dp.store)                i1_itype = STORE;
         if (i1_dp.pm_alu)               i1_itype = ALU;
         if (i1_dp.condbr)               i1_itype = CONDBR;
         if (i1_dp.jal)                  i1_itype = JAL;
         if (i1_dp.atomic & ~(i1_dp.lr | i1_dp.sc))
                                         i1_itype = ATOMIC;
         if (i1_dp.lr)                   i1_itype = LR;
         if (i1_dp.sc)                   i1_itype = SC;
      end
   end


// end pmu

   eh2_dec_dec_ctl i0_dec (.inst(i0[31:0]),.predecode(dec_i0_predecode),.out(i0_dp_raw));

   eh2_dec_dec_ctl i1_dec (.inst(i1[31:0]),.predecode(dec_i1_predecode),.out(i1_dp_raw));

   for (genvar i=0; i<pt.NUM_THREADS; i++) begin

      rvdff #(1) lsu_idle_ff (.*, .clk(free_clk), .din(lsu_idle_any[i]), .dout(lsu_idle[i]));

   end

// thread the leak1 logic
// leak1 needed for debug single-step

   for (genvar i=0; i<pt.NUM_THREADS; i++) begin

      assign leak1_i1_stall_in[i] = (dec_tlu_flush_leak_one_wb[i] | (leak1_i1_stall[i] & ~flush_lower_wb[i]));

      rvdff #(1) leak1_i1_stall_ff (.*, .clk(free_clk), .din(leak1_i1_stall_in[i]), .dout(leak1_i1_stall[i]));

      assign leak1_mode[i] = leak1_i1_stall[i];

      assign leak1_i0_stall_in[i] = ((dec_i0_decode_d & (dd.i0tid == i) & leak1_i1_stall[i]) | (leak1_i0_stall[i] & ~flush_lower_wb[i]));

      rvdff #(1) leak1_i0_stall_ff (.*, .clk(free_clk), .din(leak1_i0_stall_in[i]), .dout(leak1_i0_stall[i]));

   end


   // 12b jal's can be predicted - these are calls

   assign i0_pcall_imm[20:1] = {i0[31],i0[19:12],i0[20],i0[30:21]};

   assign i0_pcall_12b_offset = (i0_pcall_imm[12]) ? (i0_pcall_imm[20:13] == 8'hff) : (i0_pcall_imm[20:13] == 8'h0);

   assign i0_pcall_case  = i0_pcall_12b_offset & i0_dp_raw.imm20 &  (i0r.rd[4:0] == 5'd1 | i0r.rd[4:0] == 5'd5);
   assign i0_pja_case    = i0_pcall_12b_offset & i0_dp_raw.imm20 & ~(i0r.rd[4:0] == 5'd1 | i0r.rd[4:0] == 5'd5);

   assign i1_pcall_imm[20:1] = {i1[31],i1[19:12],i1[20],i1[30:21]};

   assign i1_pcall_12b_offset = (i1_pcall_imm[12]) ? (i1_pcall_imm[20:13] == 8'hff) : (i1_pcall_imm[20:13] == 8'h0);

   assign i1_pcall_case  = i1_pcall_12b_offset & i1_dp_raw.imm20 &  (i1r.rd[4:0] == 5'd1 | i1r.rd[4:0] == 5'd5);
   assign i1_pja_case    = i1_pcall_12b_offset & i1_dp_raw.imm20 & ~(i1r.rd[4:0] == 5'd1 | i1r.rd[4:0] == 5'd5);


   assign i0_pcall_raw = i0_dp_raw.jal &   i0_pcall_case;   // this includes ja
   assign i0_pcall     = i0_dp.jal     &   i0_pcall_case;

   assign i1_pcall_raw = i1_dp_raw.jal &   i1_pcall_case;
   assign i1_pcall     = i1_dp.jal     &   i1_pcall_case;

   assign i0_pja_raw = i0_dp_raw.jal &   i0_pja_case;
   assign i0_pja     = i0_dp.jal     &   i0_pja_case;

   assign i1_pja_raw = i1_dp_raw.jal &   i1_pja_case;
   assign i1_pja     = i1_dp.jal     &   i1_pja_case;



   assign i0_br_offset[11:0] = (i0_pcall_raw | i0_pja_raw) ? i0_pcall_imm[12:1] : {i0[31],i0[7],i0[30:25],i0[11:8]};

   assign i1_br_offset[11:0] = (i1_pcall_raw | i1_pja_raw) ? i1_pcall_imm[12:1] : {i1[31],i1[7],i1[30:25],i1[11:8]};


   assign i0_pret_case = (i0_dp_raw.jal & i0_dp_raw.imm12 & (i0r.rd[4:0] == 5'b0) & (i0r.rs1[4:0] == 5'd1 | i0r.rs1[4:0] == 5'd5));  // jalr with rd==0, rs1==1 or rs1==5 is a ret
   assign i1_pret_case = (i1_dp_raw.jal & i1_dp_raw.imm12 & (i1r.rd[4:0] == 5'b0) & (i1r.rs1[4:0] == 5'd1 | i1r.rs1[4:0] == 5'd5));  // jalr with rd==0, rs1==1 or rs1==5 is a ret

   assign i0_pret_raw = i0_dp_raw.jal &   i0_pret_case;
   assign i0_pret    = i0_dp.jal     &   i0_pret_case;

   assign i1_pret_raw = i1_dp_raw.jal &   i1_pret_case;
   assign i1_pret     = i1_dp.jal     &   i1_pret_case;

   assign i0_jal    = i0_dp.jal  &  ~i0_pcall_case & ~i0_pja_case & ~i0_pret_case;
   assign i1_jal    = i1_dp.jal  &  ~i1_pcall_case & ~i1_pja_case & ~i1_pret_case;

   // lsu stuff
   // load/store mutually exclusive
   assign dec_lsu_offset_d[11:0] =
                                   ({12{~dec_extint_stall &  i0_dp.lsu & i0_dp.load}} &               i0[31:20]) |
                                   ({12{~dec_extint_stall & ~i0_dp.lsu & i1_dp.lsu & i1_dp.load}} &   i1[31:20]) |
                                   ({12{~dec_extint_stall &  i0_dp.lsu & i0_dp.store}} &             {i0[31:25],i0[11:7]}) |
                                   ({12{~dec_extint_stall & ~i0_dp.lsu & i1_dp.lsu & i1_dp.store}} & {i1[31:25],i1[11:7]});



   assign dec_i0_lsu_d = i0_dp.lsu;
   assign dec_i1_lsu_d = i1_dp.lsu;

   assign dec_i0_mul_d = i0_dp.mul;
   assign dec_i1_mul_d = i1_dp.mul;

   assign dec_i0_div_d = i0_dp.div;


   assign div_p.valid  =  i0_div_decode_d;
   assign div_p.unsign =  i0_dp.unsign;
   assign div_p.rem    =  i0_dp.rem;
   assign div_p.tid    =  dd.i0tid;


   assign mul_p.valid = mul_decode_d;

   assign mul_p.rs1_sign =   (i0_dp.mul) ? i0_dp.rs1_sign :   i1_dp.rs1_sign;
   assign mul_p.rs2_sign =   (i0_dp.mul) ? i0_dp.rs2_sign :   i1_dp.rs2_sign;
   assign mul_p.low      =   (i0_dp.mul) ? i0_dp.low      :   i1_dp.low;

   assign mul_p.load_mul_rs1_bypass_e1 = load_mul_rs1_bypass_e1;
   assign mul_p.load_mul_rs2_bypass_e1 = load_mul_rs2_bypass_e1;

   // manage 64b state for loads / stores

   for (genvar i=0; i<pt.NUM_THREADS; i++) begin

      assign flush_final_lower[i] = flush_lower_wb[i];

      assign flush_final_upper_e2[i] = (exu_i0_flush_final[i] | exu_i1_flush_final[i]) & ~flush_lower_wb[i];  // the exu*_flush_final is e2 stage and has the dec_tlu flush_lower_wb in it

   end

   rvdff #(pt.NUM_THREADS) extint_stall_ff (.*, .clk(free_clk), .din(dec_tlu_flush_extint[pt.NUM_THREADS-1:0]), .dout(flush_extint[pt.NUM_THREADS-1:0]));

   assign dec_extint_stall = |flush_extint[pt.NUM_THREADS-1:0];

`ifdef ASSERT_ON
   assert_dec_flush_extint_onehot:          assert #0 ($onehot0(dec_tlu_flush_extint[pt.NUM_THREADS-1:0]));
`endif

   always_comb  begin
      lsu_p = '0;

      if (dec_extint_stall) begin
         lsu_p.load = 1'b1;
         lsu_p.word = 1'b1;
         lsu_p.fast_int = 1'b1;
         lsu_p.valid = 1'b1;
         lsu_p.tid = ~flush_extint[0];
      end
      else begin

         lsu_p.atomic             = (i0_dp.lsu) ? i0_dp.atomic  :   i1_dp.atomic;
         lsu_p.atomic_instr[4:0]  = (i0_dp.lsu) ? i0[31:27]     :   i1[31:27];
         lsu_p.lr                 = (i0_dp.lsu) ? i0_dp.lr      :   i1_dp.lr;
         lsu_p.sc                 = (i0_dp.lsu) ? i0_dp.sc      :   i1_dp.sc;

         lsu_p.tid = (i0_dp.lsu) ?  dd.i0tid : dd.i1tid;

         lsu_p.pipe = ~i0_dp.lsu;

         lsu_p.load =    (i0_dp.lsu) ? i0_dp.load :   i1_dp.load;

         lsu_p.store =   (i0_dp.lsu) ? i0_dp.store :  i1_dp.store;
         lsu_p.by =      (i0_dp.lsu) ? i0_dp.by :     i1_dp.by;
         lsu_p.half =    (i0_dp.lsu) ? i0_dp.half :   i1_dp.half;

         lsu_p.word =    (i0_dp.lsu) ? i0_dp.word :   i1_dp.word;

         lsu_p.store_data_bypass_i0_e2_c2   = store_data_bypass_i0_e2_c2;  // has priority over all else
         lsu_p.load_ldst_bypass_c1          = load_ldst_bypass_c1       ;
         lsu_p.store_data_bypass_c1         = store_data_bypass_c1 & ~store_data_bypass_i0_e2_c2;
         lsu_p.store_data_bypass_c2         = store_data_bypass_c2 & ~store_data_bypass_i0_e2_c2;
         lsu_p.store_data_bypass_e4_c1[1:0] = store_data_bypass_e4_c1[1:0] & ~{2{store_data_bypass_i0_e2_c2}};
         lsu_p.store_data_bypass_e4_c2[1:0] = store_data_bypass_e4_c2[1:0] & ~{2{store_data_bypass_i0_e2_c2}};
         lsu_p.store_data_bypass_e4_c3[1:0] = store_data_bypass_e4_c3[1:0] & ~{2{store_data_bypass_i0_e2_c2}};

         lsu_p.unsign = (i0_dp.lsu) ? i0_dp.unsign : i1_dp.unsign;

         lsu_p.valid = lsu_decode_d;
      end

   end




   // defined register packet

   assign i0r.rs1[4:0] = i0[19:15];
   assign i0r.rs2[4:0] = i0[24:20];
   assign i0r.rd[4:0] = i0[11:7];

   assign i1r.rs1[4:0] = i1[19:15];
   assign i1r.rs2[4:0] = i1[24:20];
   assign i1r.rd[4:0] = i1[11:7];


   assign dec_i0_rs1_en_d = i0_dp.rs1 & (i0r.rs1[4:0] != 5'd0) & i0_valid_d;  // if rs1_en=0 then read will be all 0's
   assign dec_i0_rs2_en_d = i0_dp.rs2 & (i0r.rs2[4:0] != 5'd0) & i0_valid_d;
   assign i0_rd_en_d      =  i0_dp.rd & (i0r.rd[4:0] != 5'd0)  & i0_valid_d;

   assign dec_i0_rs1_d[4:0] = i0r.rs1[4:0];
   assign dec_i0_rs2_d[4:0] = i0r.rs2[4:0];


   assign i0_jalimm20 = i0_dp.jal & i0_dp.imm20;   // jal
   assign i1_jalimm20 = i1_dp.jal & i1_dp.imm20;


   assign i0_uiimm20 = ~i0_dp.jal & i0_dp.imm20;
   assign i1_uiimm20 = ~i1_dp.jal & i1_dp.imm20;


   // csr logic

   assign dec_i0_csr_ren_d = i0_dp.csr_read & i0_legal_decode_d & ~i0_br_error_all;

   assign i0_csr_clr_d =   i0_dp.csr_clr   & i0_legal_decode_d & ~i0_br_error_all;
   assign i0_csr_set_d   = i0_dp.csr_set   & i0_legal_decode_d & ~i0_br_error_all;
   assign i0_csr_write_d = i0_csr_write    & i0_legal_decode_d & ~i0_br_error_all;

   assign i0_csr_write_only_d = i0_csr_write & ~i0_dp.csr_read;

   assign dec_i0_csr_wen_unq_d = (i0_dp.csr_clr | i0_dp.csr_set | i0_csr_write);   // for csr legal, can't write read-only csr

   assign dec_i0_csr_any_unq_d = i0_any_csr_d;


   assign dec_i0_csr_rdaddr_d[11:0] = i0[31:20];
   assign dec_i0_csr_wraddr_wb[11:0] = wbd.i0csrwaddr[11:0];

   assign dec_i0_csr_is_mcpc_e4 = (e4d.i0csrwaddr[11:0] == 12'h7c2);

   // make sure csr doesn't write same cycle as flush_lower_wb
   // also use valid so it's flushable
   assign dec_i0_csr_wen_wb = wbd.i0csrwen & wbd.i0valid & ~dec_tlu_i0_kill_writeb_wb;

   // If we are writing MIE or MSTATUS, hold off the external interrupt for a cycle on the write.
   // this assumes CSRs only done in i0 pipe
   // Note: WB kill only applies if E4 has the same tid
   for (genvar i=0; i<pt.NUM_THREADS; i++) begin

      assign dec_csr_stall_int_ff[i] = (i==e4d.i0tid) & ((e4d.i0csrwaddr[11:0] == 12'h300) | (e4d.i0csrwaddr[11:0] == 12'h304)) & e4d.i0csrwen & e4d.i0valid & (~dec_tlu_i0_kill_writeb_wb | (e4d.i0tid != wbd.i0tid));

   end

   // tell tlu when the nmipdelegate csr is being written
   assign dec_csr_nmideleg_e4 = (e4d.i0csrwaddr[11:0] == 12'h7fe) & e4d.i0csrwen & e4d.i0valid & ~dec_tlu_i0_kill_writeb_wb & ~flush_lower_wb[e4d.i0tid];

   // perform the update operation if any for i0 pipe

   rvdff #(5) i0_csrmiscff (.*,
                        .clk(active_clk),
                        .din({ dec_i0_csr_ren_d,  i0_csr_clr_d,  i0_csr_set_d,  i0_csr_write_d,  i0_dp.csr_imm}),
                        .dout({i0_csr_read_e1,    i0_csr_clr_e1, i0_csr_set_e1, i0_csr_write_e1, i0_csr_imm_e1})
                       );

   rvdffe #(37) i0_csr_data_e1ff (.*, .en(i0_e1_data_en), .din( {i0[19:15],dec_i0_csr_rddata_d[31:0]}), .dout({i0_csrimm_e1[4:0],i0_csr_rddata_e1[31:0]}));


   assign i0_csr_mask_e1[31:0] = ({32{ i0_csr_imm_e1}} & {27'b0,i0_csrimm_e1[4:0]}) |
                              ({32{~i0_csr_imm_e1}} &  exu_i0_csr_rs1_e1[31:0]);


   assign i0_write_csr_data_e1[31:0] = ({32{i0_csr_clr_e1}}   & (i0_csr_rddata_e1[31:0] & ~i0_csr_mask_e1[31:0])) |
                                       ({32{i0_csr_set_e1}}   & (i0_csr_rddata_e1[31:0] |  i0_csr_mask_e1[31:0])) |
                                       ({32{i0_csr_write_e1}} & (                          i0_csr_mask_e1[31:0]));


   // clock-gating for pause state
   if (pt.NUM_THREADS==1) begin

      rvdff #(2) pause_state_r_ff (.*, .clk(free_clk), .din({dec_tlu_wr_pause_wb[0],tlu_wr_pause_wb1[0]}), .dout({tlu_wr_pause_wb1[0],tlu_wr_pause_wb2[0]}));

      assign dec_pause_state_cg = pause_state[0] & ~tlu_wr_pause_wb1[0] & ~tlu_wr_pause_wb2[0];

   end
   else begin  // dont clockgate pause if NUM_THREADS==2

      assign dec_pause_state_cg = 1'b0;

   end


   assign dec_pause_state[pt.NUM_THREADS-1:0] = pause_state[pt.NUM_THREADS-1:0];



// csr complex logic - only 1 csr read/write can go down the pipe per thread and each with be presync/postsync


   assign i0_csr_update_e1 = (i0_csr_clr_e1  | i0_csr_set_e1 | i0_csr_write_e1) & i0_csr_read_e1;


   // it is illegal for 2 csr read/writes for same thread in same cycle
   for (genvar i=0; i<pt.NUM_THREADS; i++) begin

      assign csr_update_e1[i] = (e1d.i0tid==i) & i0_csr_update_e1;


      assign write_csr_data_e1[i] = i0_write_csr_data_e1[31:0];


      // need to qualify this more - i0 pause

      assign write_csr_data_wb[i] = dec_i0_csr_wrdata_wb[31:0];

      assign clear_pause[i] = (flush_lower_wb[i] & ~dec_tlu_flush_pause_wb[i]) |
                              (pause_state[i] & (write_csr_data[i][31:1] == 31'b0));        // if 0 or 1 then exit pause state - 1 cycle pause

      assign pause_state_in[i] = (dec_tlu_wr_pause_wb[i] | pause_state[i]) & ~clear_pause[i];

      rvdff #(1) pause_state_f (.*, .clk(free_clk), .din(pause_state_in[i]), .dout(pause_state[i]));

      assign csr_data_wen[i] = csr_update_e1[i] | dec_tlu_wr_pause_wb[i] | pause_state[i];

      assign write_csr_data_in[i][31:0] = (pause_state[i])               ? (write_csr_data[i][31:0] - 32'b1) :
                                          (dec_tlu_wr_pause_wb[i]) ?  write_csr_data_wb[i][31:0] : write_csr_data_e1[i][31:0];

      // will hold until write-back at which time the CSR will be updated while GPR is possibly written with prior CSR
      rvdffe #(32) write_csr_ff (.*, .en(csr_data_wen[i]), .din(write_csr_data_in[i][31:0]), .dout(write_csr_data[i][31:0]));

      assign pause_stall[i] = pause_state[i];

   end


   // for csr write only data is produced by the alu
   assign dec_i0_csr_wrdata_wb[31:0] = (wbd.i0csrwonly) ? i0_result_wb[31:0] : write_csr_data[wbd.i0tid][31:0];


// read the csr value through rs2 immed port
   assign dec_i0_immed_d[31:0] = ({32{ i0_dp.csr_read}} & dec_i0_csr_rddata_d[31:0]) |
                                 ({32{~i0_dp.csr_read}} & i0_immed_d[31:0]);

// end csr stuff

   assign     i0_immed_d[31:0] = ({32{i0_dp.imm12}} &   { {20{i0[31]}},i0[31:20] }) |  // jalr
                                 ({32{i0_dp.shimm5}} &    {27'b0, i0[24:20]}) |
                                 ({32{i0_jalimm20}} &   { {12{i0[31]}},i0[19:12],i0[20],i0[30:21],1'b0}) |
                                 ({32{i0_uiimm20}}  &     {i0[31:12],12'b0 }) |
                                 ({32{i0_csr_write_only_d & i0_dp.csr_imm}} & {27'b0,i0[19:15]});  // for csr's that only write csr, dont read csr


   // all conditional branches are currently predict_nt
   // change this to generate the sequential address for all other cases for NPC requirements at commit
   assign dec_i0_br_immed_d[12:1] = (i0_ap.predict_nt & ~i0_dp.jal) ? i0_br_offset[11:0] : {10'b0,i0_ap_pc4,i0_ap_pc2};


   assign dec_i1_rs1_en_d = i1_dp.rs1 & (i1r.rs1[4:0] != 5'd0) & i1_valid_d;
   assign dec_i1_rs2_en_d = i1_dp.rs2 & (i1r.rs2[4:0] != 5'd0) & i1_valid_d;
   assign i1_rd_en_d      = i1_dp.rd  & (i1r.rd[4:0] != 5'd0)  & i1_valid_d;

   assign dec_i1_rs1_d[4:0] = i1r.rs1[4:0];
   assign dec_i1_rs2_d[4:0] = i1r.rs2[4:0];


   assign i1_immed_d[31:0] = ({32{i1_dp.imm12}} &   { {20{i1[31]}},i1[31:20] }) |
                             ({32{i1_dp.shimm5}} &    {27'b0, i1[24:20]}) |
                             ({32{i1_jalimm20}} &   { {12{i1[31]}},i1[19:12],i1[20],i1[30:21],1'b0}) |
                             ({32{i1_uiimm20}}  &     {i1[31:12],12'b0 });


   assign dec_i1_immed_d[31:0] = i1_immed_d[31:0];


   // jal is always +2 or +4
   assign dec_i1_br_immed_d[12:1] = (i1_ap.predict_nt & ~i1_dp.jal) ? i1_br_offset[11:0] : {10'b0,i1_ap_pc4,i1_ap_pc2};




   assign i0_valid_d = dec_ib0_valid_d;
   assign i1_valid_d = dec_ib1_valid_d;

   // load_stall includes bus_barrier

   assign i0_amo_stall_d =   i0_dp.atomic & lsu_amo_stall_any[dd.i0tid];

   assign i0_load_stall_d = i0_dp.load & (lsu_load_stall_any[dd.i0tid] | dma_dccm_stall_any);

   assign i1_load_stall_d = i1_dp.load & (lsu_load_stall_any[dd.i1tid] | dma_dccm_stall_any);

   assign i0_store_stall_d =  i0_dp.store & (lsu_store_stall_any[dd.i0tid] | dma_dccm_stall_any);
   assign i1_store_stall_d =  i1_dp.store & (lsu_store_stall_any[dd.i1tid] | dma_dccm_stall_any);

   assign i1_depend_i0_d = ((dec_i1_rs1_en_d & i0_dp.rd & (i1r.rs1[4:0] == i0r.rd[4:0])) |
                            (dec_i1_rs2_en_d & i0_dp.rd & (i1r.rs2[4:0] == i0r.rd[4:0])))  & (dd.i0tid == dd.i1tid);



   assign i1_load2_block_d = i1_dp.lsu & i0_dp.lsu;



// some CSR reads need to be presync'd
   assign i0_presync = i0_dp.presync | dec_tlu_presync_d[dd.i0tid] | debug_fence_i | debug_fence_raw | dec_tlu_pipelining_disable;  // both fence's presync

// some CSR writes need to be postsync'd
   assign i0_postsync = i0_dp.postsync | dec_tlu_postsync_d[dd.i0tid] | debug_fence_i | // only fence_i postsync
                        (i0_csr_write_only_d & (i0[31:20] == 12'h7c2));   // wr_pause must postsync

   assign i1_mul2_block_d  = i1_dp.mul & i0_dp.mul;


// debug fence csr

   assign debug_fence_i     = dec_debug_fence_d & dbg_cmd_wrdata[0];
   assign debug_fence_raw   = dec_debug_fence_d & dbg_cmd_wrdata[1];

   assign debug_fence = debug_fence_raw | debug_fence_i;    // fence_i causes a fence


   assign i0_csr_write = i0_dp.csr_write & ~dec_debug_fence_d;


// end debug




always_comb begin
   i0blockp.csr_read_stall      = (i0_dp.csr_read & (dec_i0_csr_global_d ? prior_any_csr_write_any_thread : prior_csr_write[dd.i0tid])); // no csr bypass
   i0blockp.extint_stall        = dec_extint_stall & i0_dp.lsu;                            // 1 external interrupt per cycle, block both threads
   i0blockp.i1_cancel_e1_stall  = dec_i1_cancel_e1[dd.i0tid];                              // block i0 if same tid as i1_cancel_e1
   i0blockp.pause_stall         = pause_stall[dd.i0tid];
   i0blockp.leak1_stall         = leak1_i0_stall[dd.i0tid];                                // need 1 inst for debug single step
   i0blockp.debug_stall         = dec_tlu_debug_stall[dd.i0tid];                           // stop decode for db-halt request
   i0blockp.postsync_stall      = postsync_stall[dd.i0tid];
   i0blockp.presync_stall       = presync_stall[dd.i0tid];
   i0blockp.wait_lsu_idle_stall = ((i0_dp.fence | debug_fence | i0_dp.atomic) & ~lsu_idle[dd.i0tid]);   // fences only go out as i0 - presync'd
   i0blockp.nonblock_load_stall = cam_i0_nonblock_load_stall[dd.i0tid];
   i0blockp.nonblock_div_stall  = i0_nonblock_div_stall;
   i0blockp.prior_div_stall     = i0_div_prior_div_stall;
   i0blockp.load_stall          = i0_load_stall_d ;
   i0blockp.store_stall         = i0_store_stall_d;
   i0blockp.amo_stall           = i0_amo_stall_d  ;
   i0blockp.load_block          = i0_load_block_d ;
   i0blockp.mul_block           = i0_mul_block_d      ;
   i0blockp.secondary_block     = i0_secondary_block_d;
   i0blockp.secondary_stall     = i0_secondary_stall_d;

   i1blockp.debug_valid_stall    = dec_i1_debug_valid_d;                     // debug insts must go out as i0
   i1blockp.nonblock_load_stall  = cam_i1_nonblock_load_stall[dd.i1tid];
   i1blockp.wait_lsu_idle_stall  = i1_dp.atomic & ~lsu_idle[dd.i1tid];       // allow atomics to go out as i1 unless lsu is not idle
   i1blockp.extint_stall         = dec_extint_stall & i1_dp.lsu;             // 1 external interrupt per cycle, block both threads
   i1blockp.i1_cancel_e1_stall   = dec_i1_cancel_e1[dd.i1tid];               // block i1 if same tid as i1_cancel_e1
   i1blockp.pause_stall          = pause_stall[dd.i1tid];
   i1blockp.debug_stall          = dec_tlu_debug_stall[dd.i1tid];
   i1blockp.postsync_stall       = postsync_stall[dd.i1tid];
   i1blockp.presync_stall        = presync_stall[dd.i1tid];
   i1blockp.nonblock_div_stall   = i1_nonblock_div_stall;
   i1blockp.load_stall           = i1_load_stall_d;
   i1blockp.store_stall          = i1_store_stall_d;
   i1blockp.load_block           = i1_load_block_d;
   i1blockp.mul_block            = i1_mul_block_d;
   i1blockp.load2_block          = i1_load2_block_d;                         // thread independent; back-to-back load's at decode
   i1blockp.mul2_block           = i1_mul2_block_d;                          // thread independent
   i1blockp.secondary_block      = i1_secondary_block_d;                     // threaded; secondary alu data not ready and op is not alu
   i1blockp.leak1_stall          = leak1_i1_stall[dd.i1tid];
   i1blockp.i0_only_block        = i1_dp.i0_only;
   i1blockp.icaf_block           = i1_icaf_d;
   i1blockp.barrier_block        = 1'b0;
   i1blockp.block_same_thread    = i1_block_same_thread_d & (dd.i0tid == dd.i1tid);


end

   assign i0_div_prior_div_stall = i0_dp.div & div_stall;

   assign i0_block_d = |i0blockp;

   assign i1_block_d = |i1blockp;

   assign i1_depend_i0_case_d = (i1_depend_i0_d & ~non_block_case_d & ~store_data_bypass_i0_e2_c2);


   assign i1_block_same_thread_d =  i0_jal |               // all the i1 block cases for ST - none of these valid for MT

                                    i0_presync |
                                    i0_postsync |

                                    i0_dp.csr_read  |      // thread independent
                                    i0_dp.csr_write |

                                    dec_tlu_dual_issue_disable |

                                    i1_depend_i0_case_d;





   for (genvar i=0; i<pt.NUM_THREADS; i++) begin
      assign dec_thread_stall_in[i] =
                                     // exact 1 cycle stall
                                      (i0_valid_d & i0_mul_block_thread_1cycle_d & (dd.i0tid==i)) |
                                      (i1_valid_d & i1_mul_block_thread_1cycle_d & (dd.i1tid==i) & (dd.i0tid!=dd.i1tid)) |

                                     // exact 1 cycle stall
                                      (i0_valid_d & i0_secondary_stall_thread_1cycle_d & (dd.i0tid==i)) |
                                      (i0_valid_d & i0_secondary_block_thread_1cycle_d & (dd.i0tid==i)) |
                                      (i1_valid_d & i1_secondary_block_thread_1cycle_d & (dd.i1tid==i) & (dd.i0tid!=dd.i1tid)) |

                                     // exact 2 cycle stall
                                      smt_secondary_stall[i]           |

                                      smt_csr_write_stall_in[i]        |

                                      smt_atomic_stall_in[i]           |

                                      smt_div_stall_in[i]              |

                                      pause_state_in[i]                |
                                      postsync_stall_in[i]             |
                                      smt_presync_stall_in[i]          |

                                      smt_nonblock_load_stall[i];  // for nonblock load stalls


   end


   for (genvar i=0; i<pt.NUM_THREADS; i++) begin

      assign flush_all[i] = flush_lower_wb[i] | flush_final_e3[i];

      // secondary smt optimizations
      assign smt_secondary_stall_in[i] = ((i0_valid_d & i0_secondary_stall_thread_2cycle_d & (dd.i0tid==i)) |
                                          (i0_valid_d & i0_secondary_block_thread_2cycle_d & (dd.i0tid==i)) |
                                          (i1_valid_d & i1_secondary_block_thread_2cycle_d & (dd.i1tid==i) & (dd.i0tid!=dd.i1tid))) & ~flush_all[i];

      rvdff #(1) secondary_stallff (.*, .clk(free_clk), .din(smt_secondary_stall_in[i]), .dout(smt_secondary_stall_raw[i]));

      // asserted for exactly 2 cycles
      assign smt_secondary_stall[i] = (smt_secondary_stall_in[i] | smt_secondary_stall_raw[i]) & ~flush_all[i];

      // presync smt optimizations
      assign set_smt_presync_stall[i] = i0_valid_d & i0_presync & prior_inflight_e1e4[i] & (dd.i0tid==i) & ~flush_all[i];

      assign smt_presync_stall_in[i] =  set_smt_presync_stall[i] | smt_presync_stall[i];


      rvdff #(1) presync_stallff (.*, .clk(free_clk), .din(smt_presync_stall_in[i]), .dout(smt_presync_stall_raw[i]));

      assign smt_presync_stall[i] = smt_presync_stall_raw[i] & prior_inflight_e1e4[i] & ~flush_all[i];

      // csr_write smt optimizations
      assign set_smt_csr_write_stall[i] = i0_valid_d & i0_dp.csr_read & ~dec_i0_csr_global_d & prior_csr_write_e1e4[i] & (dd.i0tid==i) & ~flush_all[i];

      assign smt_csr_write_stall_in[i] =  set_smt_csr_write_stall[i] | smt_csr_write_stall[i];

      rvdff #(1) csr_write_stallff (.*, .clk(free_clk), .din(smt_csr_write_stall_in[i]), .dout(smt_csr_write_stall_raw[i]));

      assign smt_csr_write_stall[i] = smt_csr_write_stall_raw[i] & prior_csr_write_e1e4[i] & ~flush_all[i];

      // atomic smt optimizations
      assign set_smt_atomic_stall[i] = i0_valid_d & (i0_dp.fence | i0_dp.atomic) & ~lsu_idle[i] & (dd.i0tid==i) & ~flush_all[i];

      assign smt_atomic_stall_in[i] =  set_smt_atomic_stall[i] | smt_atomic_stall[i];

      rvdff #(1) atomic_stallff (.*, .clk(free_clk), .din(smt_atomic_stall_in[i]), .dout(smt_atomic_stall_raw[i]));

      assign smt_atomic_stall[i] = smt_atomic_stall_raw[i] & ~lsu_idle[i] & ~flush_all[i];

      // div smt optimizations
      assign set_smt_div_stall[i] =
                                    ((i0_valid_d & i0_dp.div & div_valid   & (dd.i0tid==i) & (dd.i0tid==div_tid)) |
                                     (i1_valid_d & i1_dp.div & div_valid   & (dd.i1tid==i) & (dd.i1tid==div_tid) & (dd.i0tid!=dd.i1tid)) |
                                     (i0_valid_d  & i0_nonblock_div_stall  & (dd.i0tid==i))  |
                                     (i1_valid_d  & i1_nonblock_div_stall  & (dd.i1tid==i) & (dd.i0tid!=dd.i1tid))) & ~flush_all[i];

      assign smt_div_stall_in[i] =  set_smt_div_stall[i] | smt_div_stall[i];

      rvdff #(1) div_stallff (.*, .clk(free_clk), .din(smt_div_stall_in[i]), .dout(smt_div_stall_raw[i]));

      assign smt_div_stall[i] = smt_div_stall_raw[i] & div_valid & ~flush_all[i];


      // nonblock load optimizations
      assign set_smt_nonblock_load_stall[i] = i0_valid_d & cam_i0_nonblock_load_stall[i] & (dd.i0tid==i) & ~flush_all[i];

      assign smt_nonblock_load_stall_in[i] =  set_smt_nonblock_load_stall[i] | smt_nonblock_load_stall[i];

      rvdff #(1) nonblock_load_stallff (.*, .clk(free_clk), .din(smt_nonblock_load_stall_in[i]), .dout(smt_nonblock_load_stall_raw[i]));

      assign smt_nonblock_load_stall[i] = smt_nonblock_load_stall_raw[i] & cam_nonblock_load_stall[i] & ~flush_all[i];

   end



   // all legals go here


   assign i0_any_csr_d = i0_dp.csr_read | i0_csr_write;

   assign i0_csr_legal_d = dec_i0_csr_legal_d;


   assign i0_legal = i0_dp.legal & (~i0_any_csr_d | i0_csr_legal_d);

   assign i0_legal_except_csr = i0_dp.legal;

   assign i1_legal = i1_dp.legal;


   // illegal inst handling

   assign i0_inst_d[31:0] = (dec_i0_pc4_d) ? i0[31:0] : {16'b0, dec_i0_cinst_d[15:0] };

   for (genvar i=0; i<pt.NUM_THREADS; i++) begin : illegal

      assign shift_illegal[i] = dec_i0_decode_d & ~i0_legal & (i == dd.i0tid);

      assign illegal_inst_en[i] = shift_illegal[i] & ~illegal_lockout[i];

      assign illegal_lockout_in[i] = (shift_illegal[i] | illegal_lockout[i]) & ~flush_final_e3[i];


      rvdffe #(32) illegal_any_ff  (.*,
                                    .en(illegal_inst_en[i]),
                                    .din(i0_inst_d[31:0]),
                                    .dout(illegal_inst[i][31:0]));

      rvdff #(1) illegal_lockout_any_ff (.*, .clk(active_clk), .din(illegal_lockout_in[i]), .dout(illegal_lockout[i]));

   end

   assign dec_illegal_inst[pt.NUM_THREADS-1:0] = illegal_inst[pt.NUM_THREADS-1:0];





   // allow illegals to flow down the pipe
   assign dec_i0_decode_d = i0_valid_d & ~i0_block_d & ~flush_lower_wb[dd.i0tid] & ~flush_final_e3[dd.i0tid];

   // define i0 legal decode
   assign i0_legal_decode_d = dec_i0_decode_d & i0_legal;

   // for timing only consider i0_legal wout csr considerations - also does not include all br error cases

   assign dec_i1_decode_d = (dd.i0tid==dd.i1tid) ? (dec_i0_decode_d & i0_legal_except_csr & i1_valid_d & i1_legal & ~i1_block_d & ~flush_lower_wb[dd.i1tid] & ~flush_final_e3[dd.i1tid]) :
                                                   (                  i0_legal_except_csr & i1_valid_d & i1_legal & ~i1_block_d & ~flush_lower_wb[dd.i1tid] & ~flush_final_e3[dd.i1tid]);


   assign i1_legal_decode_d = dec_i1_decode_d & i1_legal;


   // performance monitor signals
   for (genvar i=0; i<pt.NUM_THREADS; i++) begin
      assign dec_pmu_instr_decoded[i][1:0] = { dec_i1_decode_d & (dd.i1tid==i), dec_i0_decode_d & (dd.i0tid==i) };
   end

   for (genvar i=0; i<pt.NUM_THREADS; i++) begin
      assign dec_pmu_decode_stall[i] = ((i == dd.i0tid) & i0_valid_d & ~dec_i0_decode_d) |
                                       ((i == dd.i1tid) & i1_valid_d & ~dec_i1_decode_d & (dd.i0tid!=dd.i1tid));
   end


   assign dec_pmu_postsync_stall[pt.NUM_THREADS-1:0] = postsync_stall[pt.NUM_THREADS-1:0];

   assign dec_pmu_presync_stall[pt.NUM_THREADS-1:0]  = presync_stall[pt.NUM_THREADS-1:0];

   // thread presyncs and postsyncs

   for (genvar i=0; i<pt.NUM_THREADS; i++) begin
      // lets make ebreak, ecall, mret postsync, so break sync into pre and post

      assign presync_stall[i] = i0_valid_d & i0_presync & (dd.i0tid==i) & prior_inflight[i];

      // illegals will postsync
      assign base_postsync_stall_in[i] =  (dec_i0_decode_d & (dd.i0tid == i) & (i0_postsync | ~i0_legal))  |
                                          (base_postsync_stall[i] & prior_inflight_e1e4[i]);


      rvdff #(1) base_postsync_stallff (.*, .clk(free_clk), .din(base_postsync_stall_in[i]), .dout(base_postsync_stall[i]));

      // jal's will flush, so postsync
      // can't stall more than e1e3 or else delay correct path after jal mispredict
      assign jal_postsync_stall_in[i] = (dec_i0_decode_d & (dd.i0tid == i) & i0_jal)  |
                                        (dec_i1_decode_d & (dd.i1tid == i) & i1_jal ) |
                                        (jal_postsync_stall[i] & prior_inflight_e1e3[i]);


      rvdff #(1) jal_postsync_stallff (.*, .clk(free_clk), .din(jal_postsync_stall_in[i]), .dout(jal_postsync_stall[i]));

      assign postsync_stall_in[i] = base_postsync_stall_in[i] | jal_postsync_stall_in[i];

      assign postsync_stall[i] = base_postsync_stall[i] | jal_postsync_stall[i];


      assign prior_inflight_e1e3[i] =    |{ e1d.i0valid & (e1d.i0tid == i),
                                            e2d.i0valid & (e2d.i0tid == i),
                                            e3d.i0valid & (e3d.i0tid == i),
                                            e1d.i1valid & (e1d.i1tid == i),
                                            e2d.i1valid & (e2d.i1tid == i),
                                            e3d.i1valid & (e3d.i1tid == i)
                                            };

      assign prior_inflight_e1e4[i] =    |{ prior_inflight_e1e3[i],
                                            e4d.i0valid & (e4d.i0tid == i),
                                            e4d.i1valid & (e4d.i1tid == i)
                                            };


      assign prior_inflight_wb[i] =            |{
                                                 wbd.i0valid & (wbd.i0tid == i),
                                                 wbd.i1valid & (wbd.i1tid == i)
                                                 };


      assign prior_inflight[i] = prior_inflight_e1e4[i] | prior_inflight_wb[i];


      // block reads if there is a prior csr write in the pipeline

      assign prior_csr_write_e1e4[i] = (e1d.i0csrwonly & (e1d.i0tid==i)) |
                                       (e2d.i0csrwonly & (e2d.i0tid==i)) |
                                       (e3d.i0csrwonly & (e3d.i0tid==i)) |
                                       (e4d.i0csrwonly & (e4d.i0tid==i));

      assign prior_csr_write[i] = prior_csr_write_e1e4[i] |
                                  (wbd.i0csrwonly & (wbd.i0tid==i));



   end

   assign prior_any_csr_write_any_thread_e1e4 = (e1d.i0csrwen) |
                                                (e2d.i0csrwen) |
                                                (e3d.i0csrwen) |
                                                (e4d.i0csrwen);

   assign prior_any_csr_write_any_thread = prior_any_csr_write_any_thread_e1e4 |
                                           (wbd.i0csrwen);


   assign dec_i0_alu_decode_d = i0_legal_decode_d & i0_dp.alu & ~i0_secondary_d & ~i0_br_error_all;
   assign dec_i1_alu_decode_d = i1_legal_decode_d & i1_dp.alu & ~i1_secondary_d & ~i1_br_error_all;

   assign lsu_decode_d = (i0_legal_decode_d & i0_dp.lsu & ~i0_br_error_all) |
                         (i1_legal_decode_d & i1_dp.lsu & ~i1_br_error_all);

   assign mul_decode_d = (i0_legal_decode_d & i0_dp.mul & ~i0_br_error_all) |
                         (i1_legal_decode_d & i1_dp.mul & ~i1_br_error_all);


   for (genvar i=0; i<pt.NUM_THREADS; i++) begin

      rvdff #(1) flushff  (.*, .clk(free_clk), .din(exu_i0_flush_final[i]), .dout(i0_flush_final_e3[i]));

      rvdff #(1) flushff1 (.*, .clk(free_clk), .din(exu_i1_flush_final[i]), .dout(i1_flush_final_e3[i]));

      rvdff #(1) flushff2 (.*, .clk(free_clk), .din( i0_flush_final_e3[i]),   .dout( i0_flush_final_e4[i]));

      assign flush_final_e3[i] = i0_flush_final_e3[i] | i1_flush_final_e3[i];

   end


// scheduling logic for primary and secondary alu's

   assign i0_rs1_depend_i0_e1 = dec_i0_rs1_en_d & e1d.i0v & (e1d.i0rd[4:0] == i0r.rs1[4:0]) & (e1d.i0tid == dd.i0tid);
   assign i0_rs1_depend_i0_e2 = dec_i0_rs1_en_d & e2d.i0v & (e2d.i0rd[4:0] == i0r.rs1[4:0]) & (e2d.i0tid == dd.i0tid);
   assign i0_rs1_depend_i0_e3 = dec_i0_rs1_en_d & e3d.i0v & (e3d.i0rd[4:0] == i0r.rs1[4:0]) & (e3d.i0tid == dd.i0tid);
   assign i0_rs1_depend_i0_e4 = dec_i0_rs1_en_d & e4d.i0v & (e4d.i0rd[4:0] == i0r.rs1[4:0]) & (e4d.i0tid == dd.i0tid);
   assign i0_rs1_depend_i0_wb = dec_i0_rs1_en_d & wbd.i0v & (wbd.i0rd[4:0] == i0r.rs1[4:0]) & (wbd.i0tid == dd.i0tid);

   assign i0_rs1_depend_i1_e1 = dec_i0_rs1_en_d & e1d.i1v & (e1d.i1rd[4:0] == i0r.rs1[4:0]) & (e1d.i1tid == dd.i0tid);
   assign i0_rs1_depend_i1_e2 = dec_i0_rs1_en_d & e2d.i1v & (e2d.i1rd[4:0] == i0r.rs1[4:0]) & (e2d.i1tid == dd.i0tid);
   assign i0_rs1_depend_i1_e3 = dec_i0_rs1_en_d & e3d.i1v & (e3d.i1rd[4:0] == i0r.rs1[4:0]) & (e3d.i1tid == dd.i0tid);
   assign i0_rs1_depend_i1_e4 = dec_i0_rs1_en_d & e4d.i1v & (e4d.i1rd[4:0] == i0r.rs1[4:0]) & (e4d.i1tid == dd.i0tid);
   assign i0_rs1_depend_i1_wb = dec_i0_rs1_en_d & wbd.i1v & (wbd.i1rd[4:0] == i0r.rs1[4:0]) & (wbd.i1tid == dd.i0tid);

   assign i0_rs2_depend_i0_e1 = dec_i0_rs2_en_d & e1d.i0v & (e1d.i0rd[4:0] == i0r.rs2[4:0]) & (e1d.i0tid == dd.i0tid);
   assign i0_rs2_depend_i0_e2 = dec_i0_rs2_en_d & e2d.i0v & (e2d.i0rd[4:0] == i0r.rs2[4:0]) & (e2d.i0tid == dd.i0tid);
   assign i0_rs2_depend_i0_e3 = dec_i0_rs2_en_d & e3d.i0v & (e3d.i0rd[4:0] == i0r.rs2[4:0]) & (e3d.i0tid == dd.i0tid);
   assign i0_rs2_depend_i0_e4 = dec_i0_rs2_en_d & e4d.i0v & (e4d.i0rd[4:0] == i0r.rs2[4:0]) & (e4d.i0tid == dd.i0tid);
   assign i0_rs2_depend_i0_wb = dec_i0_rs2_en_d & wbd.i0v & (wbd.i0rd[4:0] == i0r.rs2[4:0]) & (wbd.i0tid == dd.i0tid);

   assign i0_rs2_depend_i1_e1 = dec_i0_rs2_en_d & e1d.i1v & (e1d.i1rd[4:0] == i0r.rs2[4:0]) & (e1d.i1tid == dd.i0tid);
   assign i0_rs2_depend_i1_e2 = dec_i0_rs2_en_d & e2d.i1v & (e2d.i1rd[4:0] == i0r.rs2[4:0]) & (e2d.i1tid == dd.i0tid);
   assign i0_rs2_depend_i1_e3 = dec_i0_rs2_en_d & e3d.i1v & (e3d.i1rd[4:0] == i0r.rs2[4:0]) & (e3d.i1tid == dd.i0tid);
   assign i0_rs2_depend_i1_e4 = dec_i0_rs2_en_d & e4d.i1v & (e4d.i1rd[4:0] == i0r.rs2[4:0]) & (e4d.i1tid == dd.i0tid);
   assign i0_rs2_depend_i1_wb = dec_i0_rs2_en_d & wbd.i1v & (wbd.i1rd[4:0] == i0r.rs2[4:0]) & (wbd.i1tid == dd.i0tid);


   assign i1_rs1_depend_i0_e1 = dec_i1_rs1_en_d & e1d.i0v & (e1d.i0rd[4:0] == i1r.rs1[4:0]) & (e1d.i0tid == dd.i1tid);
   assign i1_rs1_depend_i0_e2 = dec_i1_rs1_en_d & e2d.i0v & (e2d.i0rd[4:0] == i1r.rs1[4:0]) & (e2d.i0tid == dd.i1tid);
   assign i1_rs1_depend_i0_e3 = dec_i1_rs1_en_d & e3d.i0v & (e3d.i0rd[4:0] == i1r.rs1[4:0]) & (e3d.i0tid == dd.i1tid);
   assign i1_rs1_depend_i0_e4 = dec_i1_rs1_en_d & e4d.i0v & (e4d.i0rd[4:0] == i1r.rs1[4:0]) & (e4d.i0tid == dd.i1tid);
   assign i1_rs1_depend_i0_wb = dec_i1_rs1_en_d & wbd.i0v & (wbd.i0rd[4:0] == i1r.rs1[4:0]) & (wbd.i0tid == dd.i1tid);

   assign i1_rs1_depend_i1_e1 = dec_i1_rs1_en_d & e1d.i1v & (e1d.i1rd[4:0] == i1r.rs1[4:0]) & (e1d.i1tid == dd.i1tid);
   assign i1_rs1_depend_i1_e2 = dec_i1_rs1_en_d & e2d.i1v & (e2d.i1rd[4:0] == i1r.rs1[4:0]) & (e2d.i1tid == dd.i1tid);
   assign i1_rs1_depend_i1_e3 = dec_i1_rs1_en_d & e3d.i1v & (e3d.i1rd[4:0] == i1r.rs1[4:0]) & (e3d.i1tid == dd.i1tid);
   assign i1_rs1_depend_i1_e4 = dec_i1_rs1_en_d & e4d.i1v & (e4d.i1rd[4:0] == i1r.rs1[4:0]) & (e4d.i1tid == dd.i1tid);
   assign i1_rs1_depend_i1_wb = dec_i1_rs1_en_d & wbd.i1v & (wbd.i1rd[4:0] == i1r.rs1[4:0]) & (wbd.i1tid == dd.i1tid);

   assign i1_rs2_depend_i0_e1 = dec_i1_rs2_en_d & e1d.i0v & (e1d.i0rd[4:0] == i1r.rs2[4:0]) & (e1d.i0tid == dd.i1tid);
   assign i1_rs2_depend_i0_e2 = dec_i1_rs2_en_d & e2d.i0v & (e2d.i0rd[4:0] == i1r.rs2[4:0]) & (e2d.i0tid == dd.i1tid);
   assign i1_rs2_depend_i0_e3 = dec_i1_rs2_en_d & e3d.i0v & (e3d.i0rd[4:0] == i1r.rs2[4:0]) & (e3d.i0tid == dd.i1tid);
   assign i1_rs2_depend_i0_e4 = dec_i1_rs2_en_d & e4d.i0v & (e4d.i0rd[4:0] == i1r.rs2[4:0]) & (e4d.i0tid == dd.i1tid);
   assign i1_rs2_depend_i0_wb = dec_i1_rs2_en_d & wbd.i0v & (wbd.i0rd[4:0] == i1r.rs2[4:0]) & (wbd.i0tid == dd.i1tid);

   assign i1_rs2_depend_i1_e1 = dec_i1_rs2_en_d & e1d.i1v & (e1d.i1rd[4:0] == i1r.rs2[4:0]) & (e1d.i1tid == dd.i1tid);
   assign i1_rs2_depend_i1_e2 = dec_i1_rs2_en_d & e2d.i1v & (e2d.i1rd[4:0] == i1r.rs2[4:0]) & (e2d.i1tid == dd.i1tid);
   assign i1_rs2_depend_i1_e3 = dec_i1_rs2_en_d & e3d.i1v & (e3d.i1rd[4:0] == i1r.rs2[4:0]) & (e3d.i1tid == dd.i1tid);
   assign i1_rs2_depend_i1_e4 = dec_i1_rs2_en_d & e4d.i1v & (e4d.i1rd[4:0] == i1r.rs2[4:0]) & (e4d.i1tid == dd.i1tid);
   assign i1_rs2_depend_i1_wb = dec_i1_rs2_en_d & wbd.i1v & (wbd.i1rd[4:0] == i1r.rs2[4:0]) & (wbd.i1tid == dd.i1tid);

// define bypasses for e2 stage - 1 is youngest

   assign dd.i0rs1bype2[1:0] = {  i0_dp.alu & i0_rs1_depth_d[3:0] == 4'd5 & i0_rs1_class_d.sec,
                                  i0_dp.alu & i0_rs1_depth_d[3:0] == 4'd6 & i0_rs1_class_d.sec };

   assign dd.i0rs2bype2[1:0] = {  i0_dp.alu & i0_rs2_depth_d[3:0] == 4'd5 & i0_rs2_class_d.sec,
                                  i0_dp.alu & i0_rs2_depth_d[3:0] == 4'd6 & i0_rs2_class_d.sec };

   assign dd.i1rs1bype2[1:0] = {  i1_dp.alu & i1_rs1_depth_d[3:0] == 4'd5 & i1_rs1_class_d.sec,
                                  i1_dp.alu & i1_rs1_depth_d[3:0] == 4'd6 & i1_rs1_class_d.sec };

   assign dd.i1rs2bype2[1:0] = {  i1_dp.alu & i1_rs2_depth_d[3:0] == 4'd5 & i1_rs2_class_d.sec,
                                  i1_dp.alu & i1_rs2_depth_d[3:0] == 4'd6 & i1_rs2_class_d.sec };


   assign i1_result_wb_eff[31:0] = i1_result_wb[31:0];

   assign i0_result_wb_eff[31:0] = i0_result_wb[31:0];


   assign i0_rs1_bypass_data_e2[31:0] = ({32{e2d.i0rs1bype2[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e2d.i0rs1bype2[0]}} & i0_result_wb_eff[31:0]);

   assign i0_rs2_bypass_data_e2[31:0] = ({32{e2d.i0rs2bype2[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e2d.i0rs2bype2[0]}} & i0_result_wb_eff[31:0]);

   assign i1_rs1_bypass_data_e2[31:0] = ({32{e2d.i1rs1bype2[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e2d.i1rs1bype2[0]}} & i0_result_wb_eff[31:0]);

   assign i1_rs2_bypass_data_e2[31:0] = ({32{e2d.i1rs2bype2[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e2d.i1rs2bype2[0]}} & i0_result_wb_eff[31:0]);


   assign dec_i0_rs1_bypass_en_e2 = |e2d.i0rs1bype2[1:0];
   assign dec_i0_rs2_bypass_en_e2 = |e2d.i0rs2bype2[1:0];
   assign dec_i1_rs1_bypass_en_e2 = |e2d.i1rs1bype2[1:0];
   assign dec_i1_rs2_bypass_en_e2 = |e2d.i1rs2bype2[1:0];


// define bypasses for e3 stage before secondary alu's


   assign i1_rs1_depend_i0_d = dec_i1_rs1_en_d & i0_dp.rd & (i1r.rs1[4:0] == i0r.rd[4:0]) & (dd.i1tid == dd.i0tid);
   assign i1_rs2_depend_i0_d = dec_i1_rs2_en_d & i0_dp.rd & (i1r.rs2[4:0] == i0r.rd[4:0]) & (dd.i1tid == dd.i0tid);


// i0
   assign dd.i0rs1bype3[3:0] = { i0_dp.alu & i0_rs1_depth_d[3:0]==4'd1 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                 i0_dp.alu & i0_rs1_depth_d[3:0]==4'd2 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                 i0_dp.alu & i0_rs1_depth_d[3:0]==4'd3 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                 i0_dp.alu & i0_rs1_depth_d[3:0]==4'd4 & (i0_rs1_class_d.sec | i0_rs1_class_d.load | i0_rs1_class_d.mul) };

   assign dd.i0rs2bype3[3:0] = { i0_dp.alu & i0_rs2_depth_d[3:0]==4'd1 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                 i0_dp.alu & i0_rs2_depth_d[3:0]==4'd2 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                 i0_dp.alu & i0_rs2_depth_d[3:0]==4'd3 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                 i0_dp.alu & i0_rs2_depth_d[3:0]==4'd4 & (i0_rs2_class_d.sec | i0_rs2_class_d.load | i0_rs2_class_d.mul) };

// i1

   assign i1rs1_intra[2:0] = {   i1_dp.alu & i0_dp.alu  & i1_rs1_depend_i0_d,
                                 i1_dp.alu & i0_dp.mul  & i1_rs1_depend_i0_d,
                                 i1_dp.alu & i0_dp.load & i1_rs1_depend_i0_d
                                 };

   assign i1rs2_intra[2:0] = {   i1_dp.alu & i0_dp.alu  & i1_rs2_depend_i0_d,
                                 i1_dp.alu & i0_dp.mul  & i1_rs2_depend_i0_d,
                                 i1_dp.alu & i0_dp.load & i1_rs2_depend_i0_d
                                 };

   assign i1_rs1_intra_bypass = |i1rs1_intra[2:0];

   assign i1_rs2_intra_bypass = |i1rs2_intra[2:0];


   assign dd.i1rs1bype3[6:0] = { i1rs1_intra[2:0],
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd1 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass,
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd2 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass,
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd3 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass,
                                 i1_dp.alu & i1_rs1_depth_d[3:0]==4'd4 & (i1_rs1_class_d.sec | i1_rs1_class_d.load | i1_rs1_class_d.mul) & ~i1_rs1_intra_bypass };

   assign dd.i1rs2bype3[6:0] = { i1rs2_intra[2:0],
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd1 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass,
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd2 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass,
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd3 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass,
                                 i1_dp.alu & i1_rs2_depth_d[3:0]==4'd4 & (i1_rs2_class_d.sec | i1_rs2_class_d.load | i1_rs2_class_d.mul) & ~i1_rs2_intra_bypass };




   assign dec_i0_rs1_bypass_en_e3 = |e3d.i0rs1bype3[3:0];
   assign dec_i0_rs2_bypass_en_e3 = |e3d.i0rs2bype3[3:0];
   assign dec_i1_rs1_bypass_en_e3 = |e3d.i1rs1bype3[6:0];
   assign dec_i1_rs2_bypass_en_e3 = |e3d.i1rs2bype3[6:0];



   assign i1_result_e4_eff[31:0] = i1_result_e4_final[31:0];

   assign i0_result_e4_eff[31:0] = i0_result_e4_final[31:0];


   assign i0_rs1_bypass_data_e3[31:0] = ({32{e3d.i0rs1bype3[3]}} & i1_result_e4_eff[31:0]) |
                                        ({32{e3d.i0rs1bype3[2]}} & i0_result_e4_eff[31:0]) |
                                        ({32{e3d.i0rs1bype3[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e3d.i0rs1bype3[0]}} & i0_result_wb_eff[31:0]);

   assign i0_rs2_bypass_data_e3[31:0] = ({32{e3d.i0rs2bype3[3]}} & i1_result_e4_eff[31:0]) |
                                        ({32{e3d.i0rs2bype3[2]}} & i0_result_e4_eff[31:0]) |
                                        ({32{e3d.i0rs2bype3[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e3d.i0rs2bype3[0]}} & i0_result_wb_eff[31:0]);

   assign i1_rs1_bypass_data_e3[31:0] = ({32{e3d.i1rs1bype3[6]}} & i0_result_e3[31:0]) |
                                        ({32{e3d.i1rs1bype3[5]}} & exu_mul_result_e3[31:0]) |
                                        ({32{e3d.i1rs1bype3[4]}} & lsu_result_dc3[31:0]) |
                                        ({32{e3d.i1rs1bype3[3]}} & i1_result_e4_eff[31:0]) |
                                        ({32{e3d.i1rs1bype3[2]}} & i0_result_e4_eff[31:0]) |
                                        ({32{e3d.i1rs1bype3[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e3d.i1rs1bype3[0]}} & i0_result_wb_eff[31:0]);


   assign i1_rs2_bypass_data_e3[31:0] = ({32{e3d.i1rs2bype3[6]}} & i0_result_e3[31:0]) |
                                        ({32{e3d.i1rs2bype3[5]}} & exu_mul_result_e3[31:0]) |
                                        ({32{e3d.i1rs2bype3[4]}} & lsu_result_dc3[31:0]) |
                                        ({32{e3d.i1rs2bype3[3]}} & i1_result_e4_eff[31:0]) |
                                        ({32{e3d.i1rs2bype3[2]}} & i0_result_e4_eff[31:0]) |
                                        ({32{e3d.i1rs2bype3[1]}} & i1_result_wb_eff[31:0]) |
                                        ({32{e3d.i1rs2bype3[0]}} & i0_result_wb_eff[31:0]);



// order the producers as follows:  i1_e1 - 1, i0_e1 - 2, i1_e2 - 3, ..., i1_wb - 9, i0_wb - 10


   assign {i0_rs1_class_d, i0_rs1_depth_d[3:0]} =
                                                  (i0_rs1_depend_i1_e1) ? { i1_e1c, 4'd1 } :
                                                  (i0_rs1_depend_i0_e1) ? { i0_e1c, 4'd2 } :
                                                  (i0_rs1_depend_i1_e2) ? { i1_e2c, 4'd3 } :
                                                  (i0_rs1_depend_i0_e2) ? { i0_e2c, 4'd4 } :
                                                  (i0_rs1_depend_i1_e3) ? { i1_e3c, 4'd5 } :
                                                  (i0_rs1_depend_i0_e3) ? { i0_e3c, 4'd6 } :
                                                  (i0_rs1_depend_i1_e4) ? { i1_e4c, 4'd7 } :
                                                  (i0_rs1_depend_i0_e4) ? { i0_e4c, 4'd8 } :
                                                  (i0_rs1_depend_i1_wb) ? { i1_wbc, 4'd9 } :
                                                  (i0_rs1_depend_i0_wb) ? { i0_wbc, 4'd10 } : '0;

   assign {i0_rs2_class_d, i0_rs2_depth_d[3:0]} =
                                                  (i0_rs2_depend_i1_e1) ? { i1_e1c, 4'd1 } :
                                                  (i0_rs2_depend_i0_e1) ? { i0_e1c, 4'd2 } :
                                                  (i0_rs2_depend_i1_e2) ? { i1_e2c, 4'd3 } :
                                                  (i0_rs2_depend_i0_e2) ? { i0_e2c, 4'd4 } :
                                                  (i0_rs2_depend_i1_e3) ? { i1_e3c, 4'd5 } :
                                                  (i0_rs2_depend_i0_e3) ? { i0_e3c, 4'd6 } :
                                                  (i0_rs2_depend_i1_e4) ? { i1_e4c, 4'd7 } :
                                                  (i0_rs2_depend_i0_e4) ? { i0_e4c, 4'd8 } :
                                                  (i0_rs2_depend_i1_wb) ? { i1_wbc, 4'd9 } :
                                                  (i0_rs2_depend_i0_wb) ? { i0_wbc, 4'd10 } : '0;

   assign {i1_rs1_class_d, i1_rs1_depth_d[3:0]} =
                                                  (i1_rs1_depend_i1_e1) ? { i1_e1c, 4'd1 } :
                                                  (i1_rs1_depend_i0_e1) ? { i0_e1c, 4'd2 } :
                                                  (i1_rs1_depend_i1_e2) ? { i1_e2c, 4'd3 } :
                                                  (i1_rs1_depend_i0_e2) ? { i0_e2c, 4'd4 } :
                                                  (i1_rs1_depend_i1_e3) ? { i1_e3c, 4'd5 } :
                                                  (i1_rs1_depend_i0_e3) ? { i0_e3c, 4'd6 } :
                                                  (i1_rs1_depend_i1_e4) ? { i1_e4c, 4'd7 } :
                                                  (i1_rs1_depend_i0_e4) ? { i0_e4c, 4'd8 } :
                                                  (i1_rs1_depend_i1_wb) ? { i1_wbc, 4'd9 } :
                                                  (i1_rs1_depend_i0_wb) ? { i0_wbc, 4'd10 } : '0;

   assign {i1_rs2_class_d, i1_rs2_depth_d[3:0]} =
                                                  (i1_rs2_depend_i1_e1) ? { i1_e1c, 4'd1 } :
                                                  (i1_rs2_depend_i0_e1) ? { i0_e1c, 4'd2 } :
                                                  (i1_rs2_depend_i1_e2) ? { i1_e2c, 4'd3 } :
                                                  (i1_rs2_depend_i0_e2) ? { i0_e2c, 4'd4 } :
                                                  (i1_rs2_depend_i1_e3) ? { i1_e3c, 4'd5 } :
                                                  (i1_rs2_depend_i0_e3) ? { i0_e3c, 4'd6 } :
                                                  (i1_rs2_depend_i1_e4) ? { i1_e4c, 4'd7 } :
                                                  (i1_rs2_depend_i0_e4) ? { i0_e4c, 4'd8 } :
                                                  (i1_rs2_depend_i1_wb) ? { i1_wbc, 4'd9 } :
                                                  (i1_rs2_depend_i0_wb) ? { i0_wbc, 4'd10 } : '0;


   assign i0_rs1_match_e1 = (i0_rs1_depth_d[3:0] == 4'd1 |
                             i0_rs1_depth_d[3:0] == 4'd2);

   assign i0_rs1_match_e2 = (i0_rs1_depth_d[3:0] == 4'd3 |
                             i0_rs1_depth_d[3:0] == 4'd4);

   assign i0_rs1_match_e3 = (i0_rs1_depth_d[3:0] == 4'd5 |
                             i0_rs1_depth_d[3:0] == 4'd6);

   assign i0_rs2_match_e1 = (i0_rs2_depth_d[3:0] == 4'd1 |
                             i0_rs2_depth_d[3:0] == 4'd2);

   assign i0_rs2_match_e2 = (i0_rs2_depth_d[3:0] == 4'd3 |
                             i0_rs2_depth_d[3:0] == 4'd4);

   assign i0_rs2_match_e3 = (i0_rs2_depth_d[3:0] == 4'd5 |
                             i0_rs2_depth_d[3:0] == 4'd6);

   assign i0_rs1_match_e1_e2 = i0_rs1_match_e1 | i0_rs1_match_e2;
   assign i0_rs1_match_e1_e3 = i0_rs1_match_e1 | i0_rs1_match_e2 | i0_rs1_match_e3;

   assign i0_rs2_match_e1_e2 = i0_rs2_match_e1 | i0_rs2_match_e2;
   assign i0_rs2_match_e1_e3 = i0_rs2_match_e1 | i0_rs2_match_e2 | i0_rs2_match_e3;


   // smt optimization stalls

   assign i0_secondary_block_thread_1cycle_d = (~i0_dp.alu & i0_rs1_class_d.sec & i0_rs1_match_e2) |
                                               (~i0_dp.alu & i0_rs2_class_d.sec & i0_rs2_match_e2 & ~i0_dp.store);

   assign i1_secondary_block_thread_1cycle_d = (~i1_dp.alu & i1_rs1_class_d.sec & i1_rs1_match_e2) |
                                               (~i1_dp.alu & i1_rs2_class_d.sec & i1_rs2_match_e2 & ~i1_dp.store);

   assign i0_secondary_block_thread_2cycle_d = (~i0_dp.alu & i0_rs1_class_d.sec & i0_rs1_match_e1) |
                                               (~i0_dp.alu & i0_rs2_class_d.sec & i0_rs2_match_e1 & ~i0_dp.store);

   assign i1_secondary_block_thread_2cycle_d = (~i1_dp.alu & i1_rs1_class_d.sec & i1_rs1_match_e1) |
                                               (~i1_dp.alu & i1_rs2_class_d.sec & i1_rs2_match_e1 & ~i1_dp.store);

   assign i0_secondary_stall_1cycle_d = (i0_dp.alu & (i0_rs1_class_d.load | i0_rs1_class_d.mul) & i0_rs1_match_e1) |
                                        (i0_dp.alu & (i0_rs2_class_d.load | i0_rs2_class_d.mul) & i0_rs2_match_e1) |
                                        (i0_dp.alu & i0_rs1_class_d.sec & i0_rs1_match_e2) |
                                        (i0_dp.alu & i0_rs2_class_d.sec & i0_rs2_match_e2);

   assign i0_secondary_stall_2cycle_d = (i0_dp.alu & i0_rs1_class_d.sec & i0_rs1_match_e1) |
                                        (i0_dp.alu & i0_rs2_class_d.sec & i0_rs2_match_e1);

   assign i0_secondary_stall_thread_1cycle_d = (i0_dp.alu & i1_rs1_depend_i0_d & ~i1_dp.alu & i0_secondary_stall_1cycle_d) |
                                               (i0_dp.alu & i1_rs2_depend_i0_d & ~i1_dp.alu & ~i1_dp.store & i0_secondary_stall_1cycle_d);

   assign i0_secondary_stall_thread_2cycle_d = (i0_dp.alu & i1_rs1_depend_i0_d & ~i1_dp.alu & i0_secondary_stall_2cycle_d) |
                                               (i0_dp.alu & i1_rs2_depend_i0_d & ~i1_dp.alu & ~i1_dp.store & i0_secondary_stall_2cycle_d);
   // end

   assign i0_secondary_d = (i0_dp.alu & (i0_rs1_class_d.load | i0_rs1_class_d.mul) & i0_rs1_match_e1_e2) |
                           (i0_dp.alu & (i0_rs2_class_d.load | i0_rs2_class_d.mul) & i0_rs2_match_e1_e2) |
                           (i0_dp.alu & i0_rs1_class_d.sec & i0_rs1_match_e1_e3) |
                           (i0_dp.alu & i0_rs2_class_d.sec & i0_rs2_match_e1_e3);

  // stall i0 until it's not a secondary for performance
   assign i0_secondary_stall_d = (i0_dp.alu & i1_rs1_depend_i0_d & ~i1_dp.alu & i0_secondary_d) |
                                 (i0_dp.alu & i1_rs2_depend_i0_d & ~i1_dp.alu & ~i1_dp.store & i0_secondary_d);

   assign i1_rs1_match_e1 = (i1_rs1_depth_d[3:0] == 4'd1 |
                             i1_rs1_depth_d[3:0] == 4'd2);

   assign i1_rs1_match_e2 = (i1_rs1_depth_d[3:0] == 4'd3 |
                             i1_rs1_depth_d[3:0] == 4'd4);

   assign i1_rs1_match_e3 = (i1_rs1_depth_d[3:0] == 4'd5 |
                             i1_rs1_depth_d[3:0] == 4'd6);

   assign i1_rs2_match_e1 = (i1_rs2_depth_d[3:0] == 4'd1 |
                             i1_rs2_depth_d[3:0] == 4'd2);

   assign i1_rs2_match_e2 = (i1_rs2_depth_d[3:0] == 4'd3 |
                             i1_rs2_depth_d[3:0] == 4'd4);

   assign i1_rs2_match_e3 = (i1_rs2_depth_d[3:0] == 4'd5 |
                             i1_rs2_depth_d[3:0] == 4'd6);

   assign i1_rs1_match_e1_e2 = i1_rs1_match_e1 | i1_rs1_match_e2;
   assign i1_rs1_match_e1_e3 = i1_rs1_match_e1 | i1_rs1_match_e2 | i1_rs1_match_e3;

   assign i1_rs2_match_e1_e2 = i1_rs2_match_e1 | i1_rs2_match_e2;
   assign i1_rs2_match_e1_e3 = i1_rs2_match_e1 | i1_rs2_match_e2 | i1_rs2_match_e3;




   assign i1_secondary_d = (i1_dp.alu & (i1_rs1_class_d.load | i1_rs1_class_d.mul) & i1_rs1_match_e1_e2) |
                           (i1_dp.alu & (i1_rs2_class_d.load | i1_rs2_class_d.mul) & i1_rs2_match_e1_e2) |
                           (i1_dp.alu & (i1_rs1_class_d.sec) & i1_rs1_match_e1_e3) |
                           (i1_dp.alu & (i1_rs2_class_d.sec) & i1_rs2_match_e1_e3) |
                           (non_block_case_d & i1_depend_i0_d);



   assign store_data_bypass_i0_e2_c2 = i0_dp.alu & ~i0_secondary_d & i1_rs2_depend_i0_d & ~i1_rs1_depend_i0_d & i1_dp.store;

   assign non_block_case_d = (i1_dp.alu & i0_dp.load) |
                             (i1_dp.alu & i0_dp.mul);

   assign store_data_bypass_c2        =  (             i0_dp.store & (i0_rs2_depth_d[3:0] == 4'd1) & i0_rs2_class_d.load) |
                                         (             i0_dp.store & (i0_rs2_depth_d[3:0] == 4'd2) & i0_rs2_class_d.load) |
                                         (~i0_dp.lsu & i1_dp.store & (i1_rs2_depth_d[3:0] == 4'd1) & i1_rs2_class_d.load) |
                                         (~i0_dp.lsu & i1_dp.store & (i1_rs2_depth_d[3:0] == 4'd2) & i1_rs2_class_d.load);

   assign store_data_bypass_c1        =  (             i0_dp.store & (i0_rs2_depth_d[3:0] == 4'd3) & i0_rs2_class_d.load) |
                                         (             i0_dp.store & (i0_rs2_depth_d[3:0] == 4'd4) & i0_rs2_class_d.load) |
                                         (~i0_dp.lsu & i1_dp.store & (i1_rs2_depth_d[3:0] == 4'd3) & i1_rs2_class_d.load) |
                                         (~i0_dp.lsu & i1_dp.store & (i1_rs2_depth_d[3:0] == 4'd4) & i1_rs2_class_d.load);

if (pt.LOAD_TO_USE_PLUS1 == 1)
 begin
   assign load_ldst_bypass_c1        =  (             (i0_dp.load | i0_dp.store) & (i0_rs1_depth_d[3:0] == 4'd5) & i0_rs1_class_d.load) |
                                        (             (i0_dp.load | i0_dp.store) & (i0_rs1_depth_d[3:0] == 4'd6) & i0_rs1_class_d.load) |
                                        (~i0_dp.lsu & (i1_dp.load | i1_dp.store) & (i1_rs1_depth_d[3:0] == 4'd5) & i1_rs1_class_d.load) |
                                        (~i0_dp.lsu & (i1_dp.load | i1_dp.store) & (i1_rs1_depth_d[3:0] == 4'd6) & i1_rs1_class_d.load);
 end
else
 begin
   assign load_ldst_bypass_c1        =  (             (i0_dp.load | i0_dp.store) & (i0_rs1_depth_d[3:0] == 4'd3) & i0_rs1_class_d.load) |
                                        (             (i0_dp.load | i0_dp.store) & (i0_rs1_depth_d[3:0] == 4'd4) & i0_rs1_class_d.load) |
                                        (~i0_dp.lsu & (i1_dp.load | i1_dp.store) & (i1_rs1_depth_d[3:0] == 4'd3) & i1_rs1_class_d.load) |
                                        (~i0_dp.lsu & (i1_dp.load | i1_dp.store) & (i1_rs1_depth_d[3:0] == 4'd4) & i1_rs1_class_d.load);
 end

   assign load_mul_rs1_bypass_e1     =  (             (i0_dp.mul) & (i0_rs1_depth_d[3:0] == 4'd3) & i0_rs1_class_d.load) |
                                        (             (i0_dp.mul) & (i0_rs1_depth_d[3:0] == 4'd4) & i0_rs1_class_d.load) |
                                        (~i0_dp.mul & (i1_dp.mul) & (i1_rs1_depth_d[3:0] == 4'd3) & i1_rs1_class_d.load) |
                                        (~i0_dp.mul & (i1_dp.mul) & (i1_rs1_depth_d[3:0] == 4'd4) & i1_rs1_class_d.load);

   assign load_mul_rs2_bypass_e1     =  (             (i0_dp.mul) & (i0_rs2_depth_d[3:0] == 4'd3) & i0_rs2_class_d.load) |
                                        (             (i0_dp.mul) & (i0_rs2_depth_d[3:0] == 4'd4) & i0_rs2_class_d.load) |
                                        (~i0_dp.mul & (i1_dp.mul) & (i1_rs2_depth_d[3:0] == 4'd3) & i1_rs2_class_d.load) |
                                        (~i0_dp.mul & (i1_dp.mul) & (i1_rs2_depth_d[3:0] == 4'd4) & i1_rs2_class_d.load);


   assign store_data_bypass_e4_c3[1] = ( ~i0_dp.lsu & i1_dp.store & (i1_rs2_depth_d[3:0] == 4'd1) & i1_rs2_class_d.sec ) |
                                       (              i0_dp.store & (i0_rs2_depth_d[3:0] == 4'd1) & i0_rs2_class_d.sec );

   assign store_data_bypass_e4_c3[0] = ( ~i0_dp.lsu & i1_dp.store & (i1_rs2_depth_d[3:0] == 4'd2) & i1_rs2_class_d.sec ) |
                                       (              i0_dp.store & (i0_rs2_depth_d[3:0] == 4'd2) & i0_rs2_class_d.sec );

   assign store_data_bypass_e4_c2[1] = ( ~i0_dp.lsu & i1_dp.store & (i1_rs2_depth_d[3:0] == 4'd3) & i1_rs2_class_d.sec ) |
                                       (              i0_dp.store & (i0_rs2_depth_d[3:0] == 4'd3) & i0_rs2_class_d.sec );

   assign store_data_bypass_e4_c2[0] = ( ~i0_dp.lsu & i1_dp.store & (i1_rs2_depth_d[3:0] == 4'd4) & i1_rs2_class_d.sec ) |
                                       (              i0_dp.store & (i0_rs2_depth_d[3:0] == 4'd4) & i0_rs2_class_d.sec );


   assign store_data_bypass_e4_c1[1] = ( ~i0_dp.lsu & i1_dp.store & (i1_rs2_depth_d[3:0] == 4'd5) & i1_rs2_class_d.sec ) |
                                       (              i0_dp.store & (i0_rs2_depth_d[3:0] == 4'd5) & i0_rs2_class_d.sec );

   assign store_data_bypass_e4_c1[0] = ( ~i0_dp.lsu & i1_dp.store & (i1_rs2_depth_d[3:0] == 4'd6) & i1_rs2_class_d.sec ) |
                                       (              i0_dp.store & (i0_rs2_depth_d[3:0] == 4'd6) & i0_rs2_class_d.sec );



   assign i0_not_alu_eff = ~i0_dp.alu;
   assign i1_not_alu_eff = ~i1_dp.alu;

// stores will bypass load data in the lsu pipe
if (pt.LOAD_TO_USE_PLUS1 == 1)
 begin
   assign i0_load_block_d = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e1                            ) |
                            (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e2 & ~i0_dp.mul               ) | // can bypass load to address of load/store
                            (i0_not_alu_eff & i0_rs2_class_d.load & i0_rs2_match_e1 & ~i0_dp.store             ) |
                            (i0_not_alu_eff & i0_rs2_class_d.load & i0_rs2_match_e2 & ~i0_dp.store & ~i0_dp.mul);

   assign i1_load_block_d = (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e1                            ) |
                            (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e2 & ~i1_dp.mul               ) |
                            (i1_not_alu_eff & i1_rs2_class_d.load & i1_rs2_match_e1 & ~i1_dp.store             ) |
                            (i1_not_alu_eff & i1_rs2_class_d.load & i1_rs2_match_e2 & ~i1_dp.store & ~i1_dp.mul);
 end
else
 begin
   assign i0_load_block_d = (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e1                                          ) |
                            (i0_not_alu_eff & i0_rs1_class_d.load & i0_rs1_match_e2 & ~i0_dp.load & ~i0_dp.store & ~i0_dp.mul) | // can bypass load to address of load/store
                            (i0_not_alu_eff & i0_rs2_class_d.load & i0_rs2_match_e1 &               ~i0_dp.store             ) |
                            (i0_not_alu_eff & i0_rs2_class_d.load & i0_rs2_match_e2 &               ~i0_dp.store & ~i0_dp.mul);

   assign i1_load_block_d = (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e1                                          ) |
                            (i1_not_alu_eff & i1_rs1_class_d.load & i1_rs1_match_e2 & ~i1_dp.load & ~i1_dp.store & ~i1_dp.mul) |
                            (i1_not_alu_eff & i1_rs2_class_d.load & i1_rs2_match_e1 &               ~i1_dp.store             ) |
                            (i1_not_alu_eff & i1_rs2_class_d.load & i1_rs2_match_e2 &               ~i1_dp.store & ~i1_dp.mul);
 end

   assign i0_mul_block_thread_1cycle_d        = (i0_not_alu_eff & i0_rs1_class_d.mul & i0_rs1_match_e1) |
                                                (i0_not_alu_eff & i0_rs2_class_d.mul & i0_rs2_match_e1);

   assign i0_mul_block_d        = (i0_not_alu_eff & i0_rs1_class_d.mul & i0_rs1_match_e1_e2) |
                                  (i0_not_alu_eff & i0_rs2_class_d.mul & i0_rs2_match_e1_e2);

   assign i1_mul_block_thread_1cycle_d        = (i1_not_alu_eff & i1_rs1_class_d.mul & i1_rs1_match_e1) |
                                                (i1_not_alu_eff & i1_rs2_class_d.mul & i1_rs2_match_e1);

   assign i1_mul_block_d       = (i1_not_alu_eff & i1_rs1_class_d.mul & i1_rs1_match_e1_e2) |
                                 (i1_not_alu_eff & i1_rs2_class_d.mul & i1_rs2_match_e1_e2);


   assign i0_secondary_block_d = (~i0_dp.alu & i0_rs1_class_d.sec & i0_rs1_match_e1_e3) |
                                 (~i0_dp.alu & i0_rs2_class_d.sec & i0_rs2_match_e1_e3 & ~i0_dp.store);

   assign i1_secondary_block_d = (~i1_dp.alu & i1_rs1_class_d.sec & i1_rs1_match_e1_e3) |
                                 (~i1_dp.alu & i1_rs2_class_d.sec & i1_rs2_match_e1_e3 & ~i1_dp.store);

   assign dec_tlu_i0_valid_e4 =  e4d.i0valid & ~flush_lower_wb[e4d.i0tid];
   assign dec_tlu_i1_valid_e4 =  e4d.i1valid & ~flush_lower_wb[e4d.i1tid];



   assign dt.i0legal               =  i0_legal_decode_d;
   assign dt.i0icaf                =  i0_icaf_d & i0_legal_decode_d;            // dbecc is icaf exception
   assign dt.i0icaf_type[1:0]      =  dec_i0_icaf_type_d[1:0];
   assign dt.i0icaf_f1             =  dec_i0_icaf_f1_d & i0_legal_decode_d;     // this includes icaf and dbecc
   assign dt.i0fence_i             = (i0_dp.fence_i | debug_fence_i) & i0_legal_decode_d & ~i0_br_error_all;


   assign dt.i0tid = dd.i0tid;
   assign dt.i1tid = dd.i1tid;

   assign dt.pmu_i0_itype = i0_itype;
   assign dt.pmu_i1_itype = i1_itype;
   assign dt.pmu_i0_br_unpred = i0_br_unpred;
   assign dt.pmu_i1_br_unpred = i1_br_unpred;

   assign dt.lsu_pipe0 = i0_legal_decode_d & ~lsu_p.pipe & ~i0_br_error_all;

   assign dt.pmu_divide = i0_dp.div;

   // written later in the pipe
   assign dt.pmu_lsu_misaligned = 1'b0;

   assign dt.i0trigger[3:0] = dec_i0_trigger_match_d[3:0] & {4{dec_i0_decode_d}};
   assign dt.i1trigger[3:0] = dec_i1_trigger_match_d[3:0] & {4{i1_legal_decode_d}};

   rvdffe #( $bits(eh2_trap_pkt_t) ) trap_e1ff (.*, .en(i0_e1_ctl_en | i1_e1_ctl_en), .din( dt),  .dout(e1t));

  always_comb begin
      e1t_in = e1t;
      e1t_in.i0trigger[3:0] = e1t.i0trigger & ~{4{flush_final_e3[e1t.i0tid]}};
      e1t_in.i1trigger[3:0] = e1t.i1trigger & ~{4{flush_final_e3[e1t.i1tid] | dec_i1_cancel_e1[e1t.i1tid]}};
   end

   rvdffe #( $bits(eh2_trap_pkt_t) ) trap_e2ff (.*, .en(i0_e2_ctl_en | i1_e2_ctl_en), .din(e1t_in),  .dout(e2t));

   always_comb begin
      e2t_in = e2t;
      e2t_in.i0trigger[3:0] = e2t.i0trigger & ~{4{flush_final_e3[e2t.i0tid] | flush_lower_wb[e2t.i0tid]}};
      e2t_in.i1trigger[3:0] = e2t.i1trigger & ~{4{flush_final_e3[e2t.i1tid] | flush_lower_wb[e2t.i1tid]}};
   end

   rvdffe  #($bits(eh2_trap_pkt_t) ) trap_e3ff (.*, .en(i0_e3_ctl_en | i1_e3_ctl_en), .din(e2t_in),  .dout(e3t));

   assign lsu_tid_e3 = e3t.lsu_pipe0 ? e3t.i0tid : e3t.i1tid;

    always_comb begin
      e3t_in = e3t;

       e3t_in.pmu_lsu_misaligned = lsu_pmu_misaligned_dc3[lsu_tid_e3];   // only valid if a load/store is valid in e3 stage

       if (flush_lower_wb[e3t.i0tid]) begin
          e3t_in.i0legal = '0;
          e3t_in.i0icaf = '0;
          e3t_in.i0icaf_type = '0;
          e3t_in.i0icaf_f1 = '0;
          e3t_in.i0fence_i = '0;
          e3t_in.i0trigger = '0;
          e3t_in.pmu_i0_br_unpred = '0;
          e3t_in.pmu_i0_itype = eh2_inst_pkt_t'(0);
       end

       if (flush_lower_wb[e3t.i1tid]) begin
          e3t_in.i1trigger = '0;
          e3t_in.pmu_i1_br_unpred = '0;
          e3t_in.pmu_i1_itype = eh2_inst_pkt_t'(0);
       end


   end


   rvdffe #( $bits(eh2_trap_pkt_t) ) trap_e4ff (.*, .en(i0_e4_ctl_en | i1_e4_ctl_en), .din(e3t_in),  .dout(e4t_ff));

    always_comb begin
       e4t = e4t_ff;

       e4t.i0trigger[3:0] = ({4{ (e4d.i0load | e4d.i0store)}} & lsu_trigger_match_dc4[3:0]) | e4t.i0trigger[3:0];

       e4t.i1trigger[3:0] = ~{4{(e4t.i0tid==e4t.i1tid) & i0_flush_final_e4[e4t.i0tid]}} & (({4{~(e4d.i0load | e4d.i0store)}} & lsu_trigger_match_dc4[3:0]) | e4t.i1trigger[3:0]);


       if (flush_lower_wb[e4t.i0tid]) begin
          e4t.i0legal = '0;
          e4t.i0icaf = '0;
          e4t.i0icaf_type = '0;
          e4t.i0icaf_f1 = '0;
          e4t.i0fence_i = '0;
          e4t.i0trigger = '0;
          e4t.pmu_i0_br_unpred = '0;
          e4t.pmu_i0_itype = eh2_inst_pkt_t'(0);
       end

       if (flush_lower_wb[e4t.i1tid]) begin
          e4t.i1trigger = '0;
          e4t.pmu_i1_br_unpred = '0;
          e4t.pmu_i1_itype = eh2_inst_pkt_t'(0);
       end


    end


   always_comb begin

      dec_tlu_packet_e4 = e4t;

   end
   assign dec_i0_tid_e4 = e4t.i0tid;
   assign dec_i1_tid_e4 = e4t.i1tid;



// end tlu stuff
   assign i0_dc.mul   = i0_dp.mul  & i0_legal_decode_d & ~i0_br_error_all;
   assign i0_dc.load  = i0_dp.load & i0_legal_decode_d & ~i0_br_error_all;
   assign i0_dc.sec   = i0_dp.alu  &  i0_secondary_d   & i0_legal_decode_d & ~i0_br_error_all;
   assign i0_dc.alu   = i0_dp.alu  & ~i0_secondary_d   & i0_legal_decode_d & ~i0_br_error_all;

   rvdffs #( $bits(eh2_class_pkt_t) ) i0_e1c_ff (.*, .en(i0_e1_ctl_en), .clk(active_clk), .din(i0_dc),   .dout(i0_e1c));
   rvdffs #( $bits(eh2_class_pkt_t) ) i0_e2c_ff (.*, .en(i0_e2_ctl_en), .clk(active_clk), .din(i0_e1c),  .dout(i0_e2c));
   rvdffs #( $bits(eh2_class_pkt_t) ) i0_e3c_ff (.*, .en(i0_e3_ctl_en), .clk(active_clk), .din(i0_e2c),  .dout(i0_e3c));

   assign i0_e4c_in = i0_e3c;

   rvdffs  #( $bits(eh2_class_pkt_t) ) i0_e4c_ff (.*, .en(i0_e4_ctl_en),              .clk(active_clk), .din(i0_e4c_in), .dout(i0_e4c));

   rvdffs  #( $bits(eh2_class_pkt_t) ) i0_wbc_ff (.*, .en(i0_wb_ctl_en),              .clk(active_clk), .din(i0_e4c),    .dout(i0_wbc));


   assign i1_dc.mul   = i1_dp.mul  & i1_legal_decode_d & ~i1_br_error_all;
   assign i1_dc.load  = i1_dp.load & i1_legal_decode_d & ~i1_br_error_all;
   assign i1_dc.sec   = i1_dp.alu  &  i1_secondary_d   & i1_legal_decode_d & ~i1_br_error_all;
   assign i1_dc.alu   = i1_dp.alu  & ~i1_secondary_d   & i1_legal_decode_d & ~i1_br_error_all;

   rvdffs #( $bits(eh2_class_pkt_t) ) i1_e1c_ff (.*, .en(i1_e1_ctl_en), .clk(active_clk), .din(i1_dc),   .dout(i1_e1c));
   rvdffs #( $bits(eh2_class_pkt_t) ) i1_e2c_ff (.*, .en(i1_e2_ctl_en), .clk(active_clk), .din(i1_e1c),  .dout(i1_e2c));
   rvdffs #( $bits(eh2_class_pkt_t) ) i1_e3c_ff (.*, .en(i1_e3_ctl_en), .clk(active_clk), .din(i1_e2c),  .dout(i1_e3c));

   assign i1_e4c_in = i1_e3c;

   rvdffs #( $bits(eh2_class_pkt_t) ) i1_e4c_ff (.*, .en(i1_e4_ctl_en), .clk(active_clk), .din(i1_e4c_in), .dout(i1_e4c));

   rvdffs #( $bits(eh2_class_pkt_t) ) i1_wbc_ff (.*, .en(i1_wb_ctl_en), .clk(active_clk), .din(i1_e4c),    .dout(i1_wbc));


   assign dd.i0rd[4:0] = i0r.rd[4:0];
   assign dd.i0v = i0_rd_en_d & i0_legal_decode_d & ~i0_br_error_all;
   assign dd.i0valid =  dec_i0_decode_d;  // has final flush in it
   assign dd.i0tid   =  dec_i0_tid_d;

   assign dd.i0mul  = i0_dp.mul    & i0_legal_decode_d & ~i0_br_error_all;
   assign dd.i0load  = i0_dp.load  & i0_legal_decode_d & ~i0_br_error_all;
   assign dd.i0store = i0_dp.store & i0_legal_decode_d & ~i0_br_error_all;
   assign dd.i0sc    = i0_dp.sc    & i0_legal_decode_d & ~i0_br_error_all;
   assign dd.i0div = i0_div_decode_d;
   assign dd.i0secondary = i0_secondary_d & i0_legal_decode_d & ~i0_br_error_all;

   assign dd.lsu_tid = (i0_dp.lsu) ? dd.i0tid : dd.i1tid;


   assign dd.i1rd[4:0]   = i1r.rd[4:0];
   assign dd.i1v         = i1_rd_en_d & i1_legal_decode_d & ~i1_br_error_all;
   assign dd.i1valid     = i1_legal_decode_d;
   assign dd.i1tid       = dec_i1_tid_d;

   assign dd.i1mul       = i1_dp.mul;
   assign dd.i1load      = i1_dp.load;
   assign dd.i1store     = i1_dp.store;
   assign dd.i1sc        = i1_dp.sc;
   assign dd.i1secondary = i1_secondary_d & i1_legal_decode_d & ~i1_br_error_all;

   assign dd.i0csrwen = dec_i0_csr_wen_unq_d & i0_legal_decode_d & ~i0_br_error_all;

   assign dd.i0csrwonly = i0_csr_write_only_d & i0_legal_decode_d & ~i0_br_error_all;
   assign dd.i0csrwaddr[11:0] = i0[31:20];    // csr write address for rd==0 case

   assign i0_pipe_en[5] = dec_i0_decode_d;

   rvdff  #(3) i0cg0ff (.*, .clk(active_clk), .din(i0_pipe_en[5:3]), .dout(i0_pipe_en[4:2]));
   rvdff  #(2) i0cg1ff (.*, .clk(active_clk), .din(i0_pipe_en[2:1]), .dout(i0_pipe_en[1:0]));


   assign i0_e1_ctl_en = (|i0_pipe_en[5:4] | clk_override);
   assign i0_e2_ctl_en = (|i0_pipe_en[4:3] | clk_override);
   assign i0_e3_ctl_en = (|i0_pipe_en[3:2] | clk_override);
   assign i0_e4_ctl_en = (|i0_pipe_en[2:1] | clk_override);
   assign i0_wb_ctl_en = (|i0_pipe_en[1:0] | clk_override);

   assign i0_e1_data_en = (i0_pipe_en[5] | clk_override);
   assign i0_e2_data_en = (i0_pipe_en[4] | clk_override);
   assign i0_e3_data_en = (i0_pipe_en[3] | clk_override);
   assign i0_e4_data_en = (i0_pipe_en[2] | clk_override);
   assign i0_wb_data_en = (i0_pipe_en[1] | clk_override);
   assign i0_wb1_data_en = (i0_pipe_en[0] | clk_override);

   assign dec_i0_data_en[4:2] = {i0_e1_data_en, i0_e2_data_en, i0_e3_data_en};
   assign dec_i0_ctl_en[4:1]  = {i0_e1_ctl_en, i0_e2_ctl_en, i0_e3_ctl_en, i0_e4_ctl_en};


   assign i1_pipe_en[5] = dec_i1_decode_d;

   rvdff  #(3) i1cg0ff (.*, .clk(free_clk), .din(i1_pipe_en[5:3]), .dout(i1_pipe_en[4:2]));
   rvdff  #(2) i1cg1ff (.*, .clk(free_clk), .din(i1_pipe_en[2:1]), .dout(i1_pipe_en[1:0]));


   assign i1_e1_ctl_en = (|i1_pipe_en[5:4] | clk_override);
   assign i1_e2_ctl_en = (|i1_pipe_en[4:3] | clk_override);
   assign i1_e3_ctl_en = (|i1_pipe_en[3:2] | clk_override);
   assign i1_e4_ctl_en = (|i1_pipe_en[2:1] | clk_override);
   assign i1_wb_ctl_en = (|i1_pipe_en[1:0] | clk_override);

   assign i1_e1_data_en = (i1_pipe_en[5] | clk_override);
   assign i1_e2_data_en = (i1_pipe_en[4] | clk_override);
   assign i1_e3_data_en = (i1_pipe_en[3] | clk_override);
   assign i1_e4_data_en = (i1_pipe_en[2] | clk_override);
   assign i1_wb_data_en = (i1_pipe_en[1] | clk_override);
   assign i1_wb1_data_en = (i1_pipe_en[0] | clk_override);

   assign dec_i1_data_en[4:2] = {i1_e1_data_en, i1_e2_data_en, i1_e3_data_en};
   assign dec_i1_ctl_en[4:1]  = {i1_e1_ctl_en, i1_e2_ctl_en, i1_e3_ctl_en, i1_e4_ctl_en};

   rvdffe #( $bits(eh2_dest_pkt_t) ) e1ff (.*, .en(i0_e1_ctl_en | i1_e1_ctl_en), .din(dd),  .dout(e1d));

   always_comb begin
      e1d_in = e1d;

      e1d_in.i0div =        e1d.i0div       & ~div_flush;

      e1d_in.i0v =          e1d.i0v         & ~flush_final_e3[e1d.i0tid];
      e1d_in.i1v =          e1d.i1v         & ~flush_final_e3[e1d.i1tid] & ~dec_i1_cancel_e1[e1d.i1tid];
      e1d_in.i0valid =      e1d.i0valid     & ~flush_final_e3[e1d.i0tid];
      e1d_in.i1valid =      e1d.i1valid     & ~flush_final_e3[e1d.i1tid] & ~dec_i1_cancel_e1[e1d.i1tid];
      e1d_in.i0secondary =  e1d.i0secondary & ~flush_final_e3[e1d.i0tid];
      e1d_in.i1secondary =  e1d.i1secondary & ~flush_final_e3[e1d.i1tid] & ~dec_i1_cancel_e1[e1d.i1tid];
   end

   assign dec_i1_valid_e1 = e1d.i1valid & ~dec_i1_cancel_e1[e1d.i1tid];


   rvdffe #( $bits(eh2_dest_pkt_t) ) e2ff (.*, .en(i0_e2_ctl_en | i1_e2_ctl_en), .din(e1d_in), .dout(e2d));

   always_comb begin
      e2d_in = e2d;

      e2d_in.i0div =       e2d.i0div       & ~div_flush;

      e2d_in.i0v =         e2d.i0v         & ~flush_final_e3[e2d.i0tid] & ~flush_lower_wb[e2d.i0tid];
      e2d_in.i1v =         e2d.i1v         & ~flush_final_e3[e2d.i1tid] & ~flush_lower_wb[e2d.i1tid];
      e2d_in.i0valid =     e2d.i0valid     & ~flush_final_e3[e2d.i0tid] & ~flush_lower_wb[e2d.i0tid];
      e2d_in.i1valid =     e2d.i1valid     & ~flush_final_e3[e2d.i1tid] & ~flush_lower_wb[e2d.i1tid];
      e2d_in.i0secondary = e2d.i0secondary & ~flush_final_e3[e2d.i0tid] & ~flush_lower_wb[e2d.i0tid];
      e2d_in.i1secondary = e2d.i1secondary & ~flush_final_e3[e2d.i1tid] & ~flush_lower_wb[e2d.i1tid];

   end


   rvdffe #( $bits(eh2_dest_pkt_t) ) e3ff (.*, .en(i0_e3_ctl_en | i1_e3_ctl_en), .din(e2d_in), .dout(e3d));

   always_comb begin
      e3d_in = e3d;

      e3d_in.i0div =       e3d.i0div       & ~div_flush;

      e3d_in.i0v = e3d.i0v                              & ~flush_lower_wb[e3d.i0tid];
      e3d_in.i0valid = e3d.i0valid                      & ~flush_lower_wb[e3d.i0tid];

      e3d_in.i0secondary = e3d.i0secondary              & ~flush_lower_wb[e3d.i0tid];

      e3d_in.i1v = e3d.i1v         & ~((e3d.i0tid==e3d.i1tid) & i0_flush_final_e3[e3d.i1tid]) & ~flush_lower_wb[e3d.i1tid];
      e3d_in.i1valid = e3d.i1valid & ~((e3d.i0tid==e3d.i1tid) & i0_flush_final_e3[e3d.i1tid]) & ~flush_lower_wb[e3d.i1tid];

      e3d_in.i1secondary = e3d.i1secondary & ~((e3d.i0tid==e3d.i1tid) & i0_flush_final_e3[e3d.i1tid]) & ~flush_lower_wb[e3d.i1tid];
   end


   assign dec_i0_sec_decode_e3 = e3d.i0secondary & ~flush_lower_wb[e3d.i0tid];
   assign dec_i1_sec_decode_e3 = e3d.i1secondary & ~((e3d.i0tid==e3d.i1tid) & i0_flush_final_e3[e3d.i1tid]) & ~flush_lower_wb[e3d.i1tid];

   rvdffe #( $bits(eh2_dest_pkt_t) ) e4ff (.*, .en(i0_e4_ctl_en | i1_e4_ctl_en), .din(e3d_in), .dout(e4d));

   always_comb begin
      e4d_in = e4d;


      e4d_in.i0div =       e4d.i0div       & ~div_flush;

      e4d_in.i0v =     (e4d.i0v                  & ~flush_lower_wb[e4d.i0tid]);

      e4d_in.i0valid = (e4d.i0valid              & ~flush_lower_wb[e4d.i0tid]);

      e4d_in.i0secondary = e4d.i0secondary & ~flush_lower_wb[e4d.i0tid];

      e4d_in.i1v = e4d.i1v                 & ~flush_lower_wb[e4d.i1tid];
      e4d_in.i1valid = e4d.i1valid         & ~flush_lower_wb[e4d.i1tid];
      e4d_in.i1secondary = e3d.i1secondary & ~flush_lower_wb[e4d.i1tid];
   end

   rvdffe #( $bits(eh2_dest_pkt_t) ) wbff (.*, .en(i0_wb_ctl_en | i1_wb_ctl_en), .din(e4d_in), .dout(wbd));

   assign dec_i0_waddr_wb[4:0] = wbd.i0rd[4:0];

   // squash same write, take last write assuming we don't kill the I1 write for some reason.
   // threaded
   assign     i0_wen_wb = wbd.i0v & ~(~dec_tlu_i1_kill_writeb_wb & ~cam_i1_load_kill_wen[wbd.i1tid] & wbd.i0v & wbd.i1v & (wbd.i0rd[4:0] == wbd.i1rd[4:0]) & (wbd.i0tid == wbd.i1tid)) & ~dec_tlu_i0_kill_writeb_wb;

   assign dec_i0_wen_wb = i0_wen_wb & ~wbd.i0div & ~cam_i0_load_kill_wen[wbd.i0tid];  // don't write a nonblock load 1st time down the pipe

   assign dec_i0_wdata_wb[31:0] = i0_result_wb[31:0];

   assign dec_i0_tid_wb = wbd.i0tid;

   assign dec_i1_waddr_wb[4:0] = wbd.i1rd[4:0];

   assign     i1_wen_wb = wbd.i1v & ~dec_tlu_i1_kill_writeb_wb;
   assign dec_i1_wen_wb = i1_wen_wb & ~cam_i1_load_kill_wen[wbd.i1tid];

   assign dec_i1_wdata_wb[31:0] = i1_result_wb[31:0];

   assign dec_i1_tid_wb = wbd.i1tid;


// divides are i0 only; flush_upper cases

   assign div_flush = (e1d.i0div & e1d.i0valid & e1d.i0rd[4:0]==5'b0) |
                      (e1d.i0div & e1d.i0valid & (flush_lower_wb[e1d.i0tid] | flush_final_e3[e1d.i0tid])) |
                      (e2d.i0div & e2d.i0valid & (flush_lower_wb[e2d.i0tid] | flush_final_e3[e2d.i0tid])) |
                      (e3d.i0div & e3d.i0valid &  flush_lower_wb[e3d.i0tid]) |
                      (e4d.i0div & e4d.i0valid &  flush_lower_wb[e4d.i0tid]) |
                      (wbd.i0div & wbd.i0valid & dec_tlu_i0_kill_writeb_wb);

// divide stuff
   assign div_e1_to_wb = (e1d.i0div & e1d.i0valid) |
                         (e2d.i0div & e2d.i0valid) |
                         (e3d.i0div & e3d.i0valid) |
                         (e4d.i0div & e4d.i0valid) |
                         (wbd.i0div & wbd.i0valid);

   assign div_active_in = i0_div_decode_d | (div_active & ~exu_div_wren & ~nonblock_div_cancel);

   rvdff  #(1) divactiveff   (.*, .clk(free_clk), .din(div_active_in), .dout(div_active));

   assign dec_div_active = div_active;
   assign dec_div_tid = div_tid;

   assign div_stall = div_active;
   assign div_valid = div_active;

// nonblocking div scheme

// divides must go down as i0; i1 will not go same cycle if dependent on i0 and same tid as i0 div

// after div reaches wb if any inst writes to same dest on subsequent cycles and same tid as div then div is canceled

   assign i0_nonblock_div_stall  = (dec_i0_rs1_en_d & (dd.i0tid == div_tid) & div_valid & (div_rd[4:0] == i0r.rs1[4:0])) |
                                   (dec_i0_rs2_en_d & (dd.i0tid == div_tid) & div_valid & (div_rd[4:0] == i0r.rs2[4:0]));

   assign i1_nonblock_div_stall  = (dec_i1_rs1_en_d & (dd.i1tid == div_tid) & div_valid & (div_rd[4:0] == i1r.rs1[4:0])) |
                                   (dec_i1_rs2_en_d & (dd.i1tid == div_tid) & div_valid & (div_rd[4:0] == i1r.rs2[4:0]));

// cancel if any younger inst committing this cycle to same dest as nonblock divide
   assign nonblock_div_cancel = (div_valid & div_flush) |
                                (div_valid & ~div_e1_to_wb & (wbd.i0rd[4:0] == div_rd[4:0]) & (wbd.i0tid == div_tid) & i0_wen_wb) |
                                (div_valid & ~div_e1_to_wb & (wbd.i1rd[4:0] == div_rd[4:0]) & (wbd.i1tid == div_tid) & i1_wen_wb) |
                                (div_valid & wbd.i0div & wbd.i0valid & (wbd.i0rd[4:0] == wbd.i1rd[4:0]) & (wbd.i0tid == wbd.i1tid) & i1_wen_wb);


   assign dec_div_cancel = nonblock_div_cancel;

   assign i0_div_decode_d = i0_legal_decode_d & i0_dp.div & ~i0_br_error_all;

   rvdffs #(5) divwbaddrff (.*, .en(i0_div_decode_d), .clk(active_clk), .din(i0r.rd[4:0]), .dout(div_rd[4:0]));
   rvdffs #(1) divtidff (.*, .en(i0_div_decode_d), .clk(active_clk), .din(dd.i0tid), .dout(div_tid));

   assign div_waddr_wb[4:0] = div_rd[4:0];
   assign div_tid_wb        = div_tid;

   assign i0_result_e1[31:0] = exu_i0_result_e1[31:0];
   assign i1_result_e1[31:0] = exu_i1_result_e1[31:0];

   // pipe the results down the pipe
   rvdffe #(32) i0e2resultff (.*, .en(i0_e2_data_en), .din(i0_result_e1[31:0]), .dout(i0_result_e2[31:0]));
   rvdffe #(32) i1e2resultff (.*, .en(i1_e2_data_en), .din(i1_result_e1[31:0]), .dout(i1_result_e2[31:0]));

   rvdffe #(32) i0e3resultff (.*, .en(i0_e3_data_en), .din(i0_result_e2[31:0]), .dout(i0_result_e3[31:0]));
   rvdffe #(32) i1e3resultff (.*, .en(i1_e3_data_en), .din(i1_result_e2[31:0]), .dout(i1_result_e3[31:0]));

   assign i0_result_e3_final[31:0] = (e3d.i0v & e3d.i0load) ? lsu_result_dc3[31:0] : (e3d.i0v & e3d.i0mul) ? exu_mul_result_e3[31:0] : i0_result_e3[31:0];

   assign i1_result_e3_final[31:0] = (e3d.i1v & e3d.i1load) ? lsu_result_dc3[31:0] : (e3d.i1v & e3d.i1mul) ? exu_mul_result_e3[31:0] : i1_result_e3[31:0];

   rvdffe #(32) i0e4resultff (.*, .en(i0_e4_data_en), .din(i0_result_e3_final[31:0]), .dout(i0_result_e4[31:0]));
   rvdffe #(32) i1e4resultff (.*, .en(i1_e4_data_en), .din(i1_result_e3_final[31:0]), .dout(i1_result_e4[31:0]));

   assign i0_result_e4_final[31:0] =
                                     (          e4d.i0secondary) ? exu_i0_result_e4[31:0] : (e4d.i0v & e4d.i0load) ? lsu_result_corr_dc4[31:0] : i0_result_e4[31:0];

   assign i1_result_e4_final[31:0] =
                                     (e4d.i1v & e4d.i1secondary) ? exu_i1_result_e4[31:0] : (e4d.i1v & e4d.i1load) ? lsu_result_corr_dc4[31:0] : i1_result_e4[31:0];

   rvdffe #(32) i0wbresultff (.*, .en(i0_wb_data_en), .din(i0_result_e4_final[31:0]), .dout(i0_result_wb_raw[31:0]));
   rvdffe #(32) i1wbresultff (.*, .en(i1_wb_data_en), .din(i1_result_e4_final[31:0]), .dout(i1_result_wb_raw[31:0]));

   assign i0_result_wb[31:0] = (wbd.i0sc) ? {31'b0, ~lsu_sc_success_dc5} : i0_result_wb_raw[31:0];

   assign i1_result_wb[31:0] = (wbd.i1sc) ? {31'b0, ~lsu_sc_success_dc5} : i1_result_wb_raw[31:0];

   rvdffe #(32) i0e1instff  (.*, .en(i0_e1_data_en),  .din(i0_inst_d[31:0] ), .dout(i0_inst_e1[31:0]));
   rvdffe #(32) i0e2instff  (.*, .en(i0_e2_data_en),  .din(i0_inst_e1[31:0]), .dout(i0_inst_e2[31:0]));
   rvdffe #(32) i0e3instff  (.*, .en(i0_e3_data_en),  .din(i0_inst_e2[31:0]), .dout(i0_inst_e3[31:0]));
   rvdffe #(32) i0e4instff  (.*, .en(i0_e4_data_en),  .din(i0_inst_e3[31:0]), .dout(i0_inst_e4[31:0]));
   rvdffe #(32) i0wbinstff  (.*, .en(i0_wb_data_en),  .din(i0_inst_e4[31:0]), .dout(i0_inst_wb[31:0] ));
   rvdffe #(32) i0wb1instff (.*, .en(i0_wb1_data_en), .din(i0_inst_wb[31:0]), .dout(i0_inst_wb1[31:0]));

   assign i1_inst_d[31:0] = (dec_i1_pc4_d) ? i1[31:0] : {16'b0, dec_i1_cinst_d[15:0] };

   rvdffe #(32) i1e1instff  (.*, .en(i1_e1_data_en), .din(i1_inst_d[31:0]),  .dout(i1_inst_e1[31:0]));
   rvdffe #(32) i1e2instff  (.*, .en(i1_e2_data_en), .din(i1_inst_e1[31:0]), .dout(i1_inst_e2[31:0]));
   rvdffe #(32) i1e3instff  (.*, .en(i1_e3_data_en), .din(i1_inst_e2[31:0]), .dout(i1_inst_e3[31:0]));
   rvdffe #(32) i1e4instff  (.*, .en(i1_e4_data_en), .din(i1_inst_e3[31:0]), .dout(i1_inst_e4[31:0]));
   rvdffe #(32) i1wbinstff  (.*, .en(i1_wb_data_en), .din(i1_inst_e4[31:0]), .dout(i1_inst_wb[31:0]));
   rvdffe #(32) i1wb1instff (.*, .en(i1_wb1_data_en),.din(i1_inst_wb[31:0]), .dout(i1_inst_wb1[31:0]));

   assign dec_i0_inst_wb1[31:0] = i0_inst_wb1[31:0];
   assign dec_i1_inst_wb1[31:0] = i1_inst_wb1[31:0];

   rvdffe #(31) i0wbpcff  (.*, .en(i0_wb_data_en ), .din(dec_tlu_i0_pc_e4[31:1]), .dout(i0_pc_wb[31:1]));
   rvdffe #(31) i0wb1pcff (.*, .en(i0_wb1_data_en), .din(i0_pc_wb[31:1]),         .dout(i0_pc_wb1[31:1]));

   rvdffe #(31) i1wb1pcff (.*, .en(i1_wb1_data_en),.din(i1_pc_wb[31:1]),         .dout(i1_pc_wb1[31:1]));

   assign dec_i0_pc_wb1[31:1] = i0_pc_wb1[31:1];
   assign dec_i1_pc_wb1[31:1] = i1_pc_wb1[31:1];

   // pipe the pc's down the pipe
   assign i0_pc_e1[31:1] = exu_i0_pc_e1[31:1];
   assign i1_pc_e1[31:1] = exu_i1_pc_e1[31:1];

   rvdffe #(31) i0e2pcff (.*, .en(i0_e2_data_en), .din(i0_pc_e1[31:1]), .dout(i0_pc_e2[31:1]));
   rvdffe #(31) i0e3pcff (.*, .en(i0_e3_data_en), .din(i0_pc_e2[31:1]), .dout(i0_pc_e3[31:1]));
   rvdffe #(31) i0e4pcff (.*, .en(i0_e4_data_en), .din(i0_pc_e3[31:1]), .dout(i0_pc_e4[31:1]));
   rvdffe #(31) i1e2pcff (.*, .en(i1_e2_data_en), .din(i1_pc_e1[31:1]), .dout(i1_pc_e2[31:1]));
   rvdffe #(31) i1e3pcff (.*, .en(i1_e3_data_en), .din(i1_pc_e2[31:1]), .dout(i1_pc_e3[31:1]));
   rvdffe #(31) i1e4pcff (.*, .en(i1_e4_data_en), .din(i1_pc_e3[31:1]), .dout(i1_pc_e4[31:1]));

   assign dec_i0_pc_e3[31:1] = i0_pc_e3[31:1];
   assign dec_i1_pc_e3[31:1] = i1_pc_e3[31:1];


   assign dec_tlu_i0_pc_e4[31:1] = i0_pc_e4[31:1];
   assign dec_tlu_i1_pc_e4[31:1] = i1_pc_e4[31:1];

   // generate the correct npc for correct br predictions

   for (genvar i=0; i<pt.NUM_THREADS; i++) begin

      assign last_br_immed_d[i][12:1] = (i1_legal_decode_d & (dd.i1tid==i)) ?
                                        ((i1_ap.predict_nt & (dd.i1tid==i)) ? {10'b0,i1_ap_pc4,i1_ap_pc2} : i1_br_offset[11:0] ) :
                                        ((i0_ap.predict_nt & (dd.i0tid==i)) ? {10'b0,i0_ap_pc4,i0_ap_pc2} : i0_br_offset[11:0] );

      rvdffe #(12) e1brpcff (.*, .en(i0_e1_data_en | i1_e1_data_en), .din(last_br_immed_d[i][12:1] ), .dout(last_br_immed_e1[i][12:1]));
      rvdffe #(12) e2brpcff (.*, .en(i0_e2_data_en | i1_e2_data_en), .din(last_br_immed_e1[i][12:1]), .dout(last_br_immed_e2[i][12:1]));


      assign last_pc_e2[i][31:1] = (e2d.i1valid & (e2d.i1tid==i)) ? i1_pc_e2[31:1] : i0_pc_e2[31:1];

      rvbradder ibradder_correct (
                                  .pc(last_pc_e2[i][31:1]),
                                  .offset(last_br_immed_e2[i][12:1]),
                                  .dout(pred_correct_npc_e2[i][31:1])
                                  );


   end


   // needed for debug triggers
   rvdffe #(31) i1wbpcff (.*, .en(i1_wb_data_en), .din(dec_tlu_i1_pc_e4[31:1]), .dout(i1_pc_wb[31:1]));

   assign i0_rs1_nonblock_load_bypass_en_d  = dec_i0_rs1_en_d & dec_nonblock_load_wen[dd.i0tid] & (dec_nonblock_load_waddr[dd.i0tid][4:0] == i0r.rs1[4:0]);
   assign i0_rs2_nonblock_load_bypass_en_d  = dec_i0_rs2_en_d & dec_nonblock_load_wen[dd.i0tid] & (dec_nonblock_load_waddr[dd.i0tid][4:0] == i0r.rs2[4:0]);
   assign i1_rs1_nonblock_load_bypass_en_d  = dec_i1_rs1_en_d & dec_nonblock_load_wen[dd.i1tid] & (dec_nonblock_load_waddr[dd.i1tid][4:0] == i1r.rs1[4:0]);
   assign i1_rs2_nonblock_load_bypass_en_d  = dec_i1_rs2_en_d & dec_nonblock_load_wen[dd.i1tid] & (dec_nonblock_load_waddr[dd.i1tid][4:0] == i1r.rs2[4:0]);

   // bit 9 is priority match, bit 0 lowest priority, i1_e1, i0_e1, i1_e2, ... i1_wb, i0_wb

   assign i0_rs1bypass[9:0] = {   i0_rs1_depth_d[3:0] == 4'd1  &  i0_rs1_class_d.alu,
                                  i0_rs1_depth_d[3:0] == 4'd2  &  i0_rs1_class_d.alu,
                                  i0_rs1_depth_d[3:0] == 4'd3  &  i0_rs1_class_d.alu,
                                  i0_rs1_depth_d[3:0] == 4'd4  &  i0_rs1_class_d.alu,
                                  i0_rs1_depth_d[3:0] == 4'd5  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                  i0_rs1_depth_d[3:0] == 4'd6  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul),
                                  i0_rs1_depth_d[3:0] == 4'd7  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec),
                                  i0_rs1_depth_d[3:0] == 4'd8  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec),
                                  i0_rs1_depth_d[3:0] == 4'd9  & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec),
                                  i0_rs1_depth_d[3:0] == 4'd10 & (i0_rs1_class_d.alu | i0_rs1_class_d.load | i0_rs1_class_d.mul | i0_rs1_class_d.sec) };


   assign i0_rs2bypass[9:0] = {   i0_rs2_depth_d[3:0] == 4'd1  &  i0_rs2_class_d.alu,
                                  i0_rs2_depth_d[3:0] == 4'd2  &  i0_rs2_class_d.alu,
                                  i0_rs2_depth_d[3:0] == 4'd3  &  i0_rs2_class_d.alu,
                                  i0_rs2_depth_d[3:0] == 4'd4  &  i0_rs2_class_d.alu,
                                  i0_rs2_depth_d[3:0] == 4'd5  & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                  i0_rs2_depth_d[3:0] == 4'd6  & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul),
                                  i0_rs2_depth_d[3:0] == 4'd7  & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul | i0_rs2_class_d.sec),
                                  i0_rs2_depth_d[3:0] == 4'd8  & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul | i0_rs2_class_d.sec),
                                  i0_rs2_depth_d[3:0] == 4'd9  & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul | i0_rs2_class_d.sec),
                                  i0_rs2_depth_d[3:0] == 4'd10 & (i0_rs2_class_d.alu | i0_rs2_class_d.load | i0_rs2_class_d.mul | i0_rs2_class_d.sec) };


   assign i1_rs1bypass[9:0] = {   i1_rs1_depth_d[3:0] == 4'd1  &  i1_rs1_class_d.alu,
                                  i1_rs1_depth_d[3:0] == 4'd2  &  i1_rs1_class_d.alu,
                                  i1_rs1_depth_d[3:0] == 4'd3  &  i1_rs1_class_d.alu,
                                  i1_rs1_depth_d[3:0] == 4'd4  &  i1_rs1_class_d.alu,
                                  i1_rs1_depth_d[3:0] == 4'd5  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul),
                                  i1_rs1_depth_d[3:0] == 4'd6  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul),
                                  i1_rs1_depth_d[3:0] == 4'd7  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec),
                                  i1_rs1_depth_d[3:0] == 4'd8  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec),
                                  i1_rs1_depth_d[3:0] == 4'd9  & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec),
                                  i1_rs1_depth_d[3:0] == 4'd10 & (i1_rs1_class_d.alu | i1_rs1_class_d.load | i1_rs1_class_d.mul | i1_rs1_class_d.sec) };


   assign i1_rs2bypass[9:0] = {   i1_rs2_depth_d[3:0] == 4'd1  &  i1_rs2_class_d.alu,
                                  i1_rs2_depth_d[3:0] == 4'd2  &  i1_rs2_class_d.alu,
                                  i1_rs2_depth_d[3:0] == 4'd3  &  i1_rs2_class_d.alu,
                                  i1_rs2_depth_d[3:0] == 4'd4  &  i1_rs2_class_d.alu,
                                  i1_rs2_depth_d[3:0] == 4'd5  & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul),
                                  i1_rs2_depth_d[3:0] == 4'd6  & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul),
                                  i1_rs2_depth_d[3:0] == 4'd7  & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul | i1_rs2_class_d.sec),
                                  i1_rs2_depth_d[3:0] == 4'd8  & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul | i1_rs2_class_d.sec),
                                  i1_rs2_depth_d[3:0] == 4'd9  & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul | i1_rs2_class_d.sec),
                                  i1_rs2_depth_d[3:0] == 4'd10 & (i1_rs2_class_d.alu | i1_rs2_class_d.load | i1_rs2_class_d.mul | i1_rs2_class_d.sec) };

   assign dec_i0_rs1_bypass_en_d = (|i0_rs1bypass[9:0]) | i0_rs1_nonblock_load_bypass_en_d;
   assign dec_i0_rs2_bypass_en_d = (|i0_rs2bypass[9:0]) | i0_rs2_nonblock_load_bypass_en_d;
   assign dec_i1_rs1_bypass_en_d = (|i1_rs1bypass[9:0]) | i1_rs1_nonblock_load_bypass_en_d;
   assign dec_i1_rs2_bypass_en_d = (|i1_rs2bypass[9:0]) | i1_rs2_nonblock_load_bypass_en_d;

   assign i0_rs1_bypass_data_d[31:0] = ({32{i0_rs1bypass[9]}} & i1_result_e1[31:0]) |
                                       ({32{i0_rs1bypass[8]}} & i0_result_e1[31:0]) |
                                       ({32{i0_rs1bypass[7]}} & i1_result_e2[31:0]) |
                                       ({32{i0_rs1bypass[6]}} & i0_result_e2[31:0]) |
                                       ({32{i0_rs1bypass[5]}} & i1_result_e3_final[31:0]) |
                                       ({32{i0_rs1bypass[4]}} & i0_result_e3_final[31:0]) |
                                       ({32{i0_rs1bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i0_rs1bypass[2]}} & i0_result_e4_final[31:0]) |
                                       ({32{i0_rs1bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i0_rs1bypass[0]}} & i0_result_wb[31:0]) |
                                       ({32{~(|i0_rs1bypass[9:0])}} & lsu_nonblock_load_data[31:0]);


   assign i0_rs2_bypass_data_d[31:0] = ({32{i0_rs2bypass[9]}} & i1_result_e1[31:0]) |
                                       ({32{i0_rs2bypass[8]}} & i0_result_e1[31:0]) |
                                       ({32{i0_rs2bypass[7]}} & i1_result_e2[31:0]) |
                                       ({32{i0_rs2bypass[6]}} & i0_result_e2[31:0]) |
                                       ({32{i0_rs2bypass[5]}} & i1_result_e3_final[31:0]) |
                                       ({32{i0_rs2bypass[4]}} & i0_result_e3_final[31:0]) |
                                       ({32{i0_rs2bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i0_rs2bypass[2]}} & i0_result_e4_final[31:0]) |
                                       ({32{i0_rs2bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i0_rs2bypass[0]}} & i0_result_wb[31:0]) |
                                       ({32{~(|i0_rs2bypass[9:0])}} & lsu_nonblock_load_data[31:0]);

   assign i1_rs1_bypass_data_d[31:0] = ({32{i1_rs1bypass[9]}} & i1_result_e1[31:0]) |
                                       ({32{i1_rs1bypass[8]}} & i0_result_e1[31:0]) |
                                       ({32{i1_rs1bypass[7]}} & i1_result_e2[31:0]) |
                                       ({32{i1_rs1bypass[6]}} & i0_result_e2[31:0]) |
                                       ({32{i1_rs1bypass[5]}} & i1_result_e3_final[31:0]) |
                                       ({32{i1_rs1bypass[4]}} & i0_result_e3_final[31:0]) |
                                       ({32{i1_rs1bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i1_rs1bypass[2]}} & i0_result_e4_final[31:0]) |
                                       ({32{i1_rs1bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i1_rs1bypass[0]}} & i0_result_wb[31:0]) |
                                       ({32{~(|i1_rs1bypass[9:0])}} & lsu_nonblock_load_data[31:0]);


   assign i1_rs2_bypass_data_d[31:0] = ({32{i1_rs2bypass[9]}} & i1_result_e1[31:0]) |
                                       ({32{i1_rs2bypass[8]}} & i0_result_e1[31:0]) |
                                       ({32{i1_rs2bypass[7]}} & i1_result_e2[31:0]) |
                                       ({32{i1_rs2bypass[6]}} & i0_result_e2[31:0]) |
                                       ({32{i1_rs2bypass[5]}} & i1_result_e3_final[31:0]) |
                                       ({32{i1_rs2bypass[4]}} & i0_result_e3_final[31:0]) |
                                       ({32{i1_rs2bypass[3]}} & i1_result_e4_final[31:0]) |
                                       ({32{i1_rs2bypass[2]}} & i0_result_e4_final[31:0]) |
                                       ({32{i1_rs2bypass[1]}} & i1_result_wb[31:0]) |
                                       ({32{i1_rs2bypass[0]}} & i0_result_wb[31:0]) |
                                       ({32{~(|i1_rs2bypass[9:0])}} & lsu_nonblock_load_data[31:0]);




endmodule

module eh2_dec_cam
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)  (
   input logic  clk,
   input logic  scan_mode,
   input logic  rst_l,

   input logic  free_clk,
   input logic  active_clk,

   input logic flush,
   input logic  tid,

   input logic dec_tlu_i0_kill_writeb_wb,
   input logic dec_tlu_i1_kill_writeb_wb,

   input logic dec_tlu_force_halt,

   input logic                                lsu_nonblock_load_data_tid,

   input eh2_dest_pkt_t dd,
   input eh2_dest_pkt_t wbd,
   input eh2_reg_pkt_t i0r,
   input eh2_reg_pkt_t i1r,

   input logic                                lsu_nonblock_load_valid_dc1,     // valid nonblock load at dc3
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_tag_dc1,       // -> corresponding tag

   input logic                                lsu_nonblock_load_inv_dc2,       // invalidate request for nonblock load dc2
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_inv_tag_dc2,   // -> corresponding tag

   input logic                                lsu_nonblock_load_inv_dc5,       // invalidate request for nonblock load dc5
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_inv_tag_dc5,   // -> corresponding tag

   input logic                                lsu_nonblock_load_data_valid,    // valid nonblock load data back
   input logic                                lsu_nonblock_load_data_error,    // nonblock load bus error
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]  lsu_nonblock_load_data_tag,      // -> corresponding tag


   input logic [4:0] nonblock_load_rd,
   input logic       nonblock_load_tid_dc1,
   input logic       nonblock_load_tid_dc2,
   input logic       nonblock_load_tid_dc5,

   input logic       dec_i0_rs1_en_d,
   input logic       dec_i0_rs2_en_d,
   input logic       dec_i1_rs1_en_d,
   input logic       dec_i1_rs2_en_d,

   input logic       i1_wen_wb,
   input logic       i0_wen_wb,

   output logic [4:0] nonblock_load_waddr,
   output logic       nonblock_load_wen,

   output logic       i0_nonblock_load_stall,
   output logic       i1_nonblock_load_stall,
   output logic       i0_load_kill_wen,
   output logic       i1_load_kill_wen,
   output logic      nonblock_load_stall
   );

   localparam NBLOAD_SIZE     = pt.LSU_NUM_NBLOAD;
   localparam NBLOAD_SIZE_MSB = int'(pt.LSU_NUM_NBLOAD)-1;
   localparam NBLOAD_TAG_MSB  = pt.LSU_NUM_NBLOAD_WIDTH-1;

   logic                     cam_write,     cam_inv_dc2_reset,     cam_inv_dc5_reset,     cam_data_reset;
   logic [NBLOAD_TAG_MSB:0]  cam_write_tag, cam_inv_dc2_reset_tag, cam_inv_dc5_reset_tag, cam_data_reset_tag;
   logic [NBLOAD_SIZE_MSB:0] cam_wen;

   logic [NBLOAD_TAG_MSB:0]  load_data_tag;
   logic [NBLOAD_SIZE_MSB:0] nonblock_load_write;
   logic i1_nonblock_boundary_stall, i0_nonblock_boundary_stall;

   logic nonblock_load_valid_dc2_raw, nonblock_load_valid_dc2, nonblock_load_valid_dc3, nonblock_load_valid_dc4;

   logic found;
   logic cam_reset_same_dest_wb;
   logic nonblock_load_valid_wb;
   logic i0_nonblock_load_match;
   logic [NBLOAD_SIZE_MSB:0] cam_inv_dc2_reset_val, cam_inv_dc5_reset_val, cam_data_reset_val;
   logic                     nonblock_load_cancel;

   eh2_load_cam_pkt_t [NBLOAD_SIZE_MSB:0] cam;
   eh2_load_cam_pkt_t [NBLOAD_SIZE_MSB:0] cam_in;
   eh2_load_cam_pkt_t [NBLOAD_SIZE_MSB:0] cam_raw;


   always_comb begin
      found = 0;
      cam_wen[NBLOAD_SIZE_MSB:0] = '0;
      for (int i=0; i<NBLOAD_SIZE; i++) begin
         if (~found) begin
            if (~cam[i].valid) begin
               cam_wen[i] = cam_write;  // cam_write is threaded
               found = 1'b1;
            end
         end
      end
   end


   // threaded
   assign cam_reset_same_dest_wb = wbd.i0v & wbd.i1v & (wbd.i0rd[4:0] == wbd.i1rd[4:0]) & (wbd.i0tid == tid) & (wbd.i1tid == tid) &
                                   wbd.i0load & nonblock_load_valid_wb & ~dec_tlu_i0_kill_writeb_wb & ~dec_tlu_i1_kill_writeb_wb;

   // threaded
   assign cam_write          = lsu_nonblock_load_valid_dc1 & (nonblock_load_tid_dc1 == tid);

   assign cam_write_tag[NBLOAD_TAG_MSB:0] = lsu_nonblock_load_tag_dc1[NBLOAD_TAG_MSB:0];

   // threaded
   assign cam_inv_dc2_reset                       = lsu_nonblock_load_inv_dc2 & (nonblock_load_tid_dc2 == tid);

   assign cam_inv_dc2_reset_tag[NBLOAD_TAG_MSB:0] = lsu_nonblock_load_inv_tag_dc2[NBLOAD_TAG_MSB:0];

   // threaded
   assign cam_inv_dc5_reset                       = (lsu_nonblock_load_inv_dc5 & (nonblock_load_tid_dc5 == tid)) |
                                                     cam_reset_same_dest_wb;

   assign cam_inv_dc5_reset_tag[NBLOAD_TAG_MSB:0] = lsu_nonblock_load_inv_tag_dc5[NBLOAD_TAG_MSB:0];

   // threaded
   assign cam_data_reset          = (lsu_nonblock_load_data_valid | lsu_nonblock_load_data_error) & (lsu_nonblock_load_data_tid == tid);

   assign cam_data_reset_tag[NBLOAD_TAG_MSB:0] = lsu_nonblock_load_data_tag[NBLOAD_TAG_MSB:0];

   // checks

`ifdef ASSERT_ON
   assert_dec_data_valid_data_error_onehot: assert #0 ($onehot0({lsu_nonblock_load_data_valid,lsu_nonblock_load_data_error}));
   assert_dec_cam_dc2_inv_reset_onehot:     assert #0 ($onehot0(cam_inv_dc2_reset_val[NBLOAD_SIZE_MSB:0]));
   assert_dec_cam_dc5_inv_reset_onehot:     assert #0 ($onehot0(cam_inv_dc5_reset_val[NBLOAD_SIZE_MSB:0]));
   assert_dec_cam_data_reset_onehot:        assert #0 ($onehot0(cam_data_reset_val[NBLOAD_SIZE_MSB:0]));
`endif

   // all these signals are threaded

    // case of multiple loads to same dest ie. x1 ... you have to invalidate the older one

   for (genvar i=0; i<NBLOAD_SIZE; i++) begin : cam_array

      assign cam_inv_dc2_reset_val[i] = cam_inv_dc2_reset   & (cam_inv_dc2_reset_tag[NBLOAD_TAG_MSB:0]  == cam[i].tag[NBLOAD_TAG_MSB:0]) & cam[i].valid;

      assign cam_inv_dc5_reset_val[i] = cam_inv_dc5_reset   & (cam_inv_dc5_reset_tag[NBLOAD_TAG_MSB:0]  == cam[i].tag[NBLOAD_TAG_MSB:0]) & cam[i].valid;

      assign cam_data_reset_val[i] = cam_data_reset & (cam_data_reset_tag[NBLOAD_TAG_MSB:0] == cam_raw[i].tag[NBLOAD_TAG_MSB:0]) & cam_raw[i].valid;

      always_comb begin

         cam[i] = cam_raw[i];

         if (pt.LOAD_TO_USE_BUS_PLUS1==0 & cam_data_reset_val[i])
           cam[i].valid = 1'b0;

         cam_in[i] = cam[i];

         if (cam_wen[i]) begin
            cam_in[i].valid    = 1'b1;
            cam_in[i].stall    = 1'b0;
            cam_in[i].wb       = 1'b0;
            cam_in[i].tag[NBLOAD_TAG_MSB:0] = cam_write_tag[NBLOAD_TAG_MSB:0];
            cam_in[i].rd[4:0]  = nonblock_load_rd[4:0];
         end
         else if ( (cam_inv_dc2_reset_val[i]) |
                   (cam_inv_dc5_reset_val[i]) |
                   (pt.LOAD_TO_USE_BUS_PLUS1==1 & cam_data_reset_val[i]) |
                   (i0_wen_wb & (wbd.i0rd[4:0] == cam[i].rd[4:0]) & (wbd.i0tid == tid) & cam[i].wb) |
                   (i1_wen_wb & (wbd.i1rd[4:0] == cam[i].rd[4:0]) & (wbd.i1tid == tid) & cam[i].wb) )
           cam_in[i].valid = 1'b0;

         // nonblock_load_valid_wb is threaded
         if (nonblock_load_valid_wb & (lsu_nonblock_load_inv_tag_dc5[NBLOAD_TAG_MSB:0]==cam[i].tag[NBLOAD_TAG_MSB:0]) & cam[i].valid)
           cam_in[i].wb = 1'b1;

         // force debug halt forces cam valids to 0; highest priority
         if (dec_tlu_force_halt)
           cam_in[i].valid = 1'b0;

         // smt optimization
         if (flush)
           cam_in[i].stall = 1'b0;
         else if ((dec_i0_rs1_en_d & (dd.i0tid == tid) & cam[i].valid & (cam[i].rd[4:0] == i0r.rs1[4:0])) |
                  (dec_i0_rs2_en_d & (dd.i0tid == tid) & cam[i].valid & (cam[i].rd[4:0] == i0r.rs2[4:0])))
           cam_in[i].stall = 1'b1;

      end // always_comb begin

      rvdff #( $bits(eh2_load_cam_pkt_t) ) cam_ff (.*, .clk(free_clk), .din(cam_in[i]), .dout(cam_raw[i]));

      // not threaded
      assign nonblock_load_write[i] = (load_data_tag[NBLOAD_TAG_MSB:0] == cam_raw[i].tag[NBLOAD_TAG_MSB:0]) & cam_raw[i].valid;

   end : cam_array

   assign load_data_tag[NBLOAD_TAG_MSB:0] = lsu_nonblock_load_data_tag[NBLOAD_TAG_MSB:0];

`ifdef ASSERT_ON
   assert_dec_cam_nonblock_load_write_onehot:   assert #0 ($onehot0(nonblock_load_write[NBLOAD_SIZE_MSB:0]));
`endif

   assign nonblock_load_cancel = ((wbd.i0rd[4:0] == nonblock_load_waddr[4:0]) & (wbd.i0tid == tid) & (wbd.i0tid == lsu_nonblock_load_data_tid) & i0_wen_wb) |    // cancel if any younger inst (including another nonblock) committing this cycle
                                 ((wbd.i1rd[4:0] == nonblock_load_waddr[4:0]) & (wbd.i1tid == tid) & (wbd.i1tid == lsu_nonblock_load_data_tid) & i1_wen_wb);

   // threaded
   assign nonblock_load_wen = lsu_nonblock_load_data_valid & (lsu_nonblock_load_data_tid == tid) & |nonblock_load_write[NBLOAD_SIZE_MSB:0] & ~nonblock_load_cancel;

   always_comb begin
      nonblock_load_waddr[4:0] = '0;

      nonblock_load_stall = '0;

      i0_nonblock_load_stall = i0_nonblock_boundary_stall;
      i1_nonblock_load_stall = i1_nonblock_boundary_stall;

      for (int i=0; i<NBLOAD_SIZE; i++) begin
         nonblock_load_waddr[4:0] |= ({5{nonblock_load_write[i] & (lsu_nonblock_load_data_tid == tid)}} & cam[i].rd[4:0]);

         // threaded
         i0_nonblock_load_stall |= dec_i0_rs1_en_d & (dd.i0tid == tid) & cam[i].valid & (cam[i].rd[4:0] == i0r.rs1[4:0]);
         i0_nonblock_load_stall |= dec_i0_rs2_en_d & (dd.i0tid == tid) & cam[i].valid & (cam[i].rd[4:0] == i0r.rs2[4:0]);

         i1_nonblock_load_stall |= dec_i1_rs1_en_d & (dd.i1tid == tid) & cam[i].valid & (cam[i].rd[4:0] == i1r.rs1[4:0]);
         i1_nonblock_load_stall |= dec_i1_rs2_en_d & (dd.i1tid == tid) & cam[i].valid & (cam[i].rd[4:0] == i1r.rs2[4:0]);

         nonblock_load_stall |= (cam_in[i].valid & cam[i].stall);
      end
   end

   // cam_write is threaded
   assign i0_nonblock_boundary_stall = ((nonblock_load_rd[4:0]==i0r.rs1[4:0]) & (dd.i0tid == tid) & cam_write & dec_i0_rs1_en_d) |
                                       ((nonblock_load_rd[4:0]==i0r.rs2[4:0]) & (dd.i0tid == tid) & cam_write & dec_i0_rs2_en_d);

   assign i1_nonblock_boundary_stall = ((nonblock_load_rd[4:0]==i1r.rs1[4:0]) & (dd.i1tid == tid) & cam_write & dec_i1_rs1_en_d) |
                                       ((nonblock_load_rd[4:0]==i1r.rs2[4:0]) & (dd.i1tid == tid) & cam_write & dec_i1_rs2_en_d);

// don't writeback a nonblock load

   // cam write is threaded
   rvdff #(1) e2nbloadff (.*, .clk(active_clk), .din(cam_write),  .dout(nonblock_load_valid_dc2_raw) );

   // cam_inv_dc2_reset is threaded
   assign nonblock_load_valid_dc2 = nonblock_load_valid_dc2_raw & ~cam_inv_dc2_reset;

   rvdff #(1) e3nbloadff (.*, .clk(active_clk), .din(    nonblock_load_valid_dc2),  .dout(nonblock_load_valid_dc3) );
   rvdff #(1) e4nbloadff (.*, .clk(active_clk), .din(    nonblock_load_valid_dc3),  .dout(nonblock_load_valid_dc4) );
   rvdff #(1) wbnbloadff (.*, .clk(active_clk), .din(    nonblock_load_valid_dc4),  .dout(nonblock_load_valid_wb) );

   // illegal for i0load and i1load same time - even with threads
   assign i0_load_kill_wen = nonblock_load_valid_wb &  wbd.i0load;
   assign i1_load_kill_wen = nonblock_load_valid_wb &  wbd.i1load;

endmodule


// file "decode" is human readable file that has all of the instruction decodes defined and is part of git repo
// modify this file as needed

// to generate all the equations below from "decode" except legal equation:

// 1) coredecode -in decode > coredecode.e

// 2) espresso -Dso -oeqntott coredecode.e | addassign -pre out.  > equations

// to generate the legal (32b instruction is legal) equation below:

// 1) coredecode -in decode -legal > legal.e

// 2) espresso -Dso -oeqntott legal.e | addassign -pre out. > legal_equation

module eh2_dec_dec_ctl
import eh2_pkg::*;
  (
   input logic [31:0] inst,
   input eh2_predecode_pkt_t predecode,

   output eh2_dec_pkt_t out
   );

   logic [31:0] i;


assign i[31:0] = inst[31:0];


assign out.alu = (!i[5]&i[2]) | (!i[3]&i[2]) | (i[6]) | (!i[25]&i[4]) | (!i[5]&i[4]);

assign out.rs1 = (!i[6]&i[5]&i[3]) | (!i[14]&!i[13]&!i[2]) | (!i[13]&i[11]&!i[2]) | (
    i[19]&i[13]&!i[2]) | (!i[13]&i[10]&!i[2]) | (i[18]&i[13]&!i[2]) | (
    !i[13]&i[9]&!i[2]) | (i[17]&i[13]&!i[2]) | (!i[13]&i[8]&!i[2]) | (
    i[16]&i[13]&!i[2]) | (!i[13]&i[7]&!i[2]) | (i[15]&i[13]&!i[2]) | (
    !i[4]&!i[3]) | (!i[6]&!i[2]);

assign out.rs2 = (i[27]&!i[6]&i[5]&i[3]) | (!i[28]&!i[6]&i[5]&i[3]) | (i[5]&!i[4]
    &!i[2]) | (!i[6]&i[5]&!i[2]);

assign out.imm12 = (!i[4]&!i[3]&i[2]) | (i[13]&!i[5]&i[4]&!i[2]) | (!i[13]&!i[12]
    &i[6]&i[4]) | (!i[12]&!i[5]&i[4]&!i[2]);

assign out.rd = (!i[5]&!i[2]) | (i[5]&i[2]) | (i[4]);

assign out.shimm5 = (!i[13]&i[12]&!i[5]&i[4]&!i[2]);

assign out.imm20 = (i[6]&i[3]) | (i[4]&i[2]);

assign out.pc = (!i[5]&!i[3]&i[2]) | (i[6]&i[3]);

assign out.load = (!i[28]&!i[6]&i[5]&i[3]) | (!i[27]&!i[6]&i[5]&i[3]) | (!i[5]&!i[4]
    &!i[2]);

assign out.store = (i[27]&!i[6]&i[5]&i[3]) | (!i[28]&!i[6]&i[5]&i[3]) | (!i[6]&i[5]
    &!i[4]&!i[2]);

assign out.lsu = (!i[6]&!i[4]&!i[2]) | (!i[6]&i[5]&!i[4]);

assign out.add = (!i[14]&!i[13]&!i[12]&!i[5]&i[4]) | (!i[5]&!i[3]&i[2]) | (!i[30]
    &!i[25]&!i[14]&!i[13]&!i[12]&!i[6]&i[4]&!i[2]);

assign out.sub = (i[30]&!i[12]&!i[6]&i[5]&i[4]&!i[2]) | (!i[25]&!i[14]&i[13]&!i[6]
    &i[4]&!i[2]) | (!i[14]&i[13]&!i[5]&i[4]&!i[2]) | (i[6]&!i[4]&!i[2]);

assign out.land = (i[14]&i[13]&i[12]&!i[5]&!i[2]) | (!i[25]&i[14]&i[13]&i[12]&!i[6]
    &!i[2]);

assign out.lor = (!i[5]&i[3]) | (!i[25]&i[14]&i[13]&!i[12]&i[4]&!i[2]) | (i[5]&i[4]
    &i[2]) | (!i[12]&i[6]&i[4]) | (i[13]&i[6]&i[4]) | (i[14]&i[13]&!i[12]
    &!i[5]&!i[2]) | (i[7]&i[6]&i[4]) | (i[8]&i[6]&i[4]) | (i[9]&i[6]&i[4]) | (
    i[10]&i[6]&i[4]) | (i[11]&i[6]&i[4]);

assign out.lxor = (!i[25]&i[14]&!i[13]&!i[12]&i[4]&!i[2]) | (i[14]&!i[13]&!i[12]
    &!i[5]&i[4]&!i[2]);

assign out.sll = (!i[25]&!i[14]&!i[13]&i[12]&!i[6]&i[4]&!i[2]);

assign out.sra = (i[30]&!i[13]&i[12]&!i[6]&i[4]&!i[2]);

assign out.srl = (!i[30]&!i[25]&i[14]&!i[13]&i[12]&!i[6]&i[4]&!i[2]);

assign out.slt = (!i[25]&!i[14]&i[13]&!i[6]&i[4]&!i[2]) | (!i[14]&i[13]&!i[5]&i[4]
    &!i[2]);

assign out.unsign = (i[31]&i[30]&!i[6]&i[3]) | (!i[14]&i[13]&i[12]&!i[5]&!i[2]) | (
    i[14]&!i[5]&!i[4]) | (i[13]&i[6]&!i[4]&!i[2]) | (i[25]&i[14]&i[12]
    &!i[6]&i[5]&!i[2]) | (!i[25]&!i[14]&i[13]&i[12]&!i[6]&!i[2]);

assign out.condbr = (i[6]&!i[4]&!i[2]);

assign out.beq = (!i[14]&!i[12]&i[6]&!i[4]&!i[2]);

assign out.bne = (!i[14]&i[12]&i[6]&!i[4]&!i[2]);

assign out.bge = (i[14]&i[12]&i[5]&!i[4]&!i[2]);

assign out.blt = (i[14]&!i[12]&i[5]&!i[4]&!i[2]);

assign out.jal = (i[6]&i[2]);

assign out.by = (!i[13]&!i[12]&!i[6]&!i[4]&!i[2]);

assign out.half = (i[12]&!i[6]&!i[4]&!i[2]);

assign out.word = (i[13]&!i[6]&!i[4]);

assign out.csr_read = (i[13]&i[6]&i[4]) | (i[7]&i[6]&i[4]) | (i[8]&i[6]&i[4]) | (
    i[9]&i[6]&i[4]) | (i[10]&i[6]&i[4]) | (i[11]&i[6]&i[4]);

assign out.csr_clr = (i[15]&i[13]&i[12]&i[6]&i[4]) | (i[16]&i[13]&i[12]&i[6]&i[4]) | (
    i[17]&i[13]&i[12]&i[6]&i[4]) | (i[18]&i[13]&i[12]&i[6]&i[4]) | (
    i[19]&i[13]&i[12]&i[6]&i[4]);

assign out.csr_set = (i[15]&!i[12]&i[6]&i[4]) | (i[16]&!i[12]&i[6]&i[4]) | (i[17]
    &!i[12]&i[6]&i[4]) | (i[18]&!i[12]&i[6]&i[4]) | (i[19]&!i[12]&i[6]
    &i[4]);

assign out.csr_write = (!i[13]&i[12]&i[6]&i[4]);

assign out.csr_imm = (i[14]&!i[13]&i[6]&i[4]) | (i[15]&i[14]&i[6]&i[4]) | (i[16]
    &i[14]&i[6]&i[4]) | (i[17]&i[14]&i[6]&i[4]) | (i[18]&i[14]&i[6]&i[4]) | (
    i[19]&i[14]&i[6]&i[4]);

assign out.presync = (!i[6]&i[3]) | (!i[13]&i[7]&i[6]&i[4]) | (!i[13]&i[8]&i[6]&i[4]) | (
    !i[13]&i[9]&i[6]&i[4]) | (!i[13]&i[10]&i[6]&i[4]) | (!i[13]&i[11]
    &i[6]&i[4]) | (i[15]&i[13]&i[6]&i[4]) | (i[16]&i[13]&i[6]&i[4]) | (
    i[17]&i[13]&i[6]&i[4]) | (i[18]&i[13]&i[6]&i[4]) | (i[19]&i[13]&i[6]
    &i[4]);

assign out.postsync = (i[12]&!i[5]&i[3]) | (!i[22]&!i[13]&!i[12]&i[6]&i[4]) | (
    i[28]&i[27]&!i[6]&i[3]) | (!i[13]&i[7]&i[6]&i[4]) | (!i[13]&i[8]&i[6]
    &i[4]) | (!i[13]&i[9]&i[6]&i[4]) | (!i[13]&i[10]&i[6]&i[4]) | (!i[13]
    &i[11]&i[6]&i[4]) | (i[15]&i[13]&i[6]&i[4]) | (i[16]&i[13]&i[6]&i[4]) | (
    i[17]&i[13]&i[6]&i[4]) | (i[18]&i[13]&i[6]&i[4]) | (i[19]&i[13]&i[6]
    &i[4]);

assign out.ebreak = (!i[22]&i[20]&!i[13]&!i[12]&i[6]&i[4]);

assign out.ecall = (!i[21]&!i[20]&!i[13]&!i[12]&i[6]&i[4]);

assign out.mret = (i[29]&!i[13]&!i[12]&i[6]&i[4]);

assign out.mul = (i[25]&!i[14]&!i[6]&i[5]&i[4]&!i[2]);

assign out.rs1_sign = (i[25]&!i[14]&i[13]&!i[12]&!i[6]&i[5]&i[4]&!i[2]) | (i[25]
    &!i[14]&!i[13]&i[12]&!i[6]&i[4]&!i[2]);

assign out.rs2_sign = (i[25]&!i[14]&!i[13]&i[12]&!i[6]&i[4]&!i[2]);

assign out.low = (i[25]&!i[14]&!i[13]&!i[12]&i[5]&i[4]&!i[2]);

assign out.div = (i[25]&i[14]&!i[6]&i[5]&!i[2]);

assign out.rem = (i[25]&i[14]&i[13]&!i[6]&i[5]&!i[2]);

assign out.fence = (!i[5]&i[3]);

assign out.fence_i = (i[12]&!i[5]&i[3]);

assign out.pm_alu = (i[28]&i[22]&!i[13]&!i[12]&i[4]) | (i[4]&i[2]) | (!i[25]&!i[6]
    &i[4]) | (!i[5]&i[4]);

assign out.atomic = (!i[6]&i[5]&i[3]);

assign out.lr = (i[28]&!i[27]&!i[6]&i[3]);

assign out.sc = (i[28]&i[27]&!i[6]&i[3]);



assign out.i0_only = predecode.i0_only;

assign out.legal = predecode.legal1 | predecode.legal2 | predecode.legal3 | predecode.legal4;


endmodule
