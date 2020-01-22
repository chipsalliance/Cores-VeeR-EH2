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

// dec: decode unit - decode, bypassing, ARF, interrupts
//
//********************************************************************************
//
// Function: Decode
// Comments: Decode, dependency scoreboard, ARF
//
// A -> D -> EX1 ... WB
//
//********************************************************************************

module eh2_dec
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (
   input logic clk,
   input logic free_clk,
   input logic active_clk,

   output logic dec_div_cancel,       // cancel divide operation

   output logic [pt.NUM_THREADS-1:0]         dec_i1_cancel_e1,

// fast interrupt
   output logic dec_extint_stall,
   input logic lsu_fastint_stall_any,

   input logic [31:0] lsu_rs1_dc1,

   output logic dec_pause_state_cg,             // to top for active state clock gating

   input logic rst_l,                        // reset, active low
   input logic [31:1] rst_vec,               // reset vector, from core pins

   input logic        nmi_int,               // NMI pin
   input logic [31:1] nmi_vec,               // NMI vector, from pins

   input logic  [pt.NUM_THREADS-1:0] i_cpu_halt_req,              // Asynchronous Halt request to CPU
   input logic  [pt.NUM_THREADS-1:0] i_cpu_run_req,               // Asynchronous Restart request to CPU

   output logic [pt.NUM_THREADS-1:0] dec_tlu_mhartstart, // thread 1 hartstart
   output logic [pt.NUM_THREADS-1:0] o_cpu_halt_status, // PMU interface, halted
   output logic [pt.NUM_THREADS-1:0] o_cpu_halt_ack,              // Halt request ack
   output logic [pt.NUM_THREADS-1:0] o_cpu_run_ack,               // Run request ack
   output logic [pt.NUM_THREADS-1:0] o_debug_mode_status,         // Core to the PMU that core is in debug mode. When core is in debug mode, the PMU should refrain from sendng a halt or run request

   output logic [pt.NUM_THREADS-1:0] dec_tlu_force_halt,

   input logic [31:4]     core_id, // Core ID


   // external MPC halt/run interface
   input logic  [pt.NUM_THREADS-1:0] mpc_debug_halt_req, // Async halt request
   input logic  [pt.NUM_THREADS-1:0] mpc_debug_run_req, // Async run request
   input logic  [pt.NUM_THREADS-1:0] mpc_reset_run_req, // Run/halt after reset
   output logic [pt.NUM_THREADS-1:0] mpc_debug_halt_ack, // Halt ack
   output logic [pt.NUM_THREADS-1:0] mpc_debug_run_ack, // Run ack
   output logic [pt.NUM_THREADS-1:0] debug_brkpt_status, // debug breakpoint

   input logic       exu_pmu_i0_br_misp,     // slot 0 branch misp
   input logic       exu_pmu_i0_br_ataken,   // slot 0 branch actual taken
   input logic       exu_pmu_i0_pc4,         // slot 0 4 byte branch
   input logic       exu_pmu_i1_br_misp,     // slot 1 branch misp
   input logic       exu_pmu_i1_br_ataken,   // slot 1 branch actual taken
   input logic       exu_pmu_i1_pc4,         // slot 1 4 byte branch


   input logic                                 lsu_nonblock_load_valid_dc1,      // valid nonblock load at dc3
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]   lsu_nonblock_load_tag_dc1,        // -> corresponding tag
   input logic                                 lsu_nonblock_load_inv_dc2,       // invalidate request for nonblock load dc2
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]   lsu_nonblock_load_inv_tag_dc2,   // -> corresponding tag
   input logic                                 lsu_nonblock_load_inv_dc5,        // invalidate request for nonblock load dc5
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]   lsu_nonblock_load_inv_tag_dc5,    // -> corresponding tag
   input logic                                 lsu_nonblock_load_data_valid,     // valid nonblock load data back
   input logic                                 lsu_nonblock_load_data_tid,
   input logic                                 lsu_nonblock_load_data_error,     // nonblock load bus error
   input logic [pt.LSU_NUM_NBLOAD_WIDTH-1:0]   lsu_nonblock_load_data_tag,       // -> corresponding tag
   input logic [31:0]                          lsu_nonblock_load_data,           // nonblock load data

   input logic [pt.NUM_THREADS-1:0] lsu_pmu_load_external_dc3,
   input logic [pt.NUM_THREADS-1:0] lsu_pmu_store_external_dc3,
   input logic [pt.NUM_THREADS-1:0] lsu_pmu_misaligned_dc3,
   input logic [pt.NUM_THREADS-1:0] lsu_pmu_bus_trxn,
   input logic [pt.NUM_THREADS-1:0] lsu_pmu_bus_busy,
   input logic [pt.NUM_THREADS-1:0] lsu_pmu_bus_misaligned,
   input logic [pt.NUM_THREADS-1:0] lsu_pmu_bus_error,


   input logic       dma_pmu_dccm_read,          // DMA DCCM read
   input logic       dma_pmu_dccm_write,         // DMA DCCM write
   input logic       dma_pmu_any_read,           // DMA read
   input logic       dma_pmu_any_write,          // DMA write

   input logic [pt.NUM_THREADS-1:0][1:0] ifu_pmu_instr_aligned,
   input logic [pt.NUM_THREADS-1:0]      ifu_pmu_align_stall,

   input logic [pt.NUM_THREADS-1:0] ifu_pmu_fetch_stall,

   input logic [pt.NUM_THREADS-1:0] ifu_pmu_ic_miss,
   input logic [pt.NUM_THREADS-1:0] ifu_pmu_ic_hit,
   input logic [pt.NUM_THREADS-1:0] ifu_pmu_bus_error,
   input logic [pt.NUM_THREADS-1:0] ifu_pmu_bus_busy,
   input logic [pt.NUM_THREADS-1:0] ifu_pmu_bus_trxn,

   input logic [3:0]  lsu_trigger_match_dc4,
   input logic        dbg_cmd_valid,   // debugger abstract command valid
   input logic        dbg_cmd_tid,     // thread for debug register read
   input logic        dbg_cmd_write,   // command is a write
   input logic  [1:0] dbg_cmd_type,    // command type
   input logic [31:0] dbg_cmd_addr,    // command address
   input logic  [1:0] dbg_cmd_wrdata,  // command write data, for fence/fence_i


   input logic [pt.NUM_THREADS-1:0] [1:0]  ifu_i0_icaf_type,                           // Instruction 0 access fault type
   input logic [pt.NUM_THREADS-1:0]      ifu_i0_icaf,          // icache access fault
   input logic [pt.NUM_THREADS-1:0]        ifu_i0_icaf_f1,       // i0 has access fault on second fetch group
   input logic [pt.NUM_THREADS-1:0]      ifu_i0_dbecc,         // icache/iccm double-bit error


   input logic [pt.NUM_THREADS-1:0]  lsu_idle_any,                          // lsu idle: if fence instr & ~lsu_idle then stall decode
   input logic [pt.NUM_THREADS-1:0]  lsu_load_stall_any,                    // stall any load  at decode
   input logic [pt.NUM_THREADS-1:0]  lsu_store_stall_any,                   // stall any store at decode
   input logic [pt.NUM_THREADS-1:0]  lsu_amo_stall_any,         // This is for blocking amo

   input eh2_br_pkt_t [pt.NUM_THREADS-1:0] i0_brp,              // branch packet
   input eh2_br_pkt_t [pt.NUM_THREADS-1:0] i1_brp,
   input logic [pt.NUM_THREADS-1:0] [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ifu_i0_bp_index, // BP index
   input logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]           ifu_i0_bp_fghr, // BP FGHR
   input logic [pt.NUM_THREADS-1:0] [pt.BTB_BTAG_SIZE-1:0]          ifu_i0_bp_btag, // BP tag
   input logic [pt.NUM_THREADS-1:0] [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ifu_i1_bp_index, // BP index
   input logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]           ifu_i1_bp_fghr, // BP FGHR
   input logic [pt.NUM_THREADS-1:0] [pt.BTB_BTAG_SIZE-1:0]          ifu_i1_bp_btag, // BP tag

   input    eh2_lsu_error_pkt_t lsu_error_pkt_dc3, // LSU exception/error packet
   input logic         lsu_single_ecc_error_incr,     // Increment the ecc error counter

   input logic [pt.NUM_THREADS-1:0] lsu_imprecise_error_store_any,
   input logic [pt.NUM_THREADS-1:0] lsu_imprecise_error_load_any,
   input logic [pt.NUM_THREADS-1:0][31:0]  lsu_imprecise_error_addr_any,   // LSU imprecise bus error address


   input logic [pt.NUM_THREADS-1:0]      exu_flush_final,            // Pipe is being flushed this cycle
   input logic [pt.NUM_THREADS-1:0]      exu_i0_flush_final,         // I0 flush to DEC
   input logic [pt.NUM_THREADS-1:0]      exu_i1_flush_final,         // I1 flush to DEC
   input logic [pt.NUM_THREADS-1:0]      exu_i0_flush_lower_e4,        // to TLU - lower branch flush
   input logic [pt.NUM_THREADS-1:0]      exu_i1_flush_lower_e4,        // to TLU - lower branch flush

   input logic [31:1] exu_i0_flush_path_e4, // pipe 0 correct path for mp, merge with lower path
   input logic [31:1] exu_i1_flush_path_e4, // pipe 1 correct path for mp, merge with lower path

   input logic         exu_div_wren,        // final div write enable to GPR
   input logic [31:0]  exu_div_result,      // final div result

   input logic [31:0] exu_mul_result_e3,    // 32b mul result

   input logic [31:0] exu_i0_csr_rs1_e1,       // rs1 for csr instruction

   input logic [31:0] lsu_result_dc3,       // load result
   input logic [31:0] lsu_result_corr_dc4, // load result - corrected data for writing gprs; not for bypassing

   input logic        lsu_sc_success_dc5,   // store conditional matched ( 1 = success, which means the GPR should write 0 )
   input logic        dma_dccm_stall_any,   // stall any load/store at decode, pmu event
   input logic        dma_iccm_stall_any,   // iccm stalled, pmu event

   input logic [31:1] lsu_fir_addr, // Fast int address
   input logic [1:0]  lsu_fir_error, // Fast int lookup error

   input logic       iccm_dma_sb_error,     // ICCM DMA single bit error

   input logic [pt.NUM_THREADS-1:0][31:1] exu_npc_e4,           // next PC

   input logic [31:0] exu_i0_result_e1,     // alu result e1
   input logic [31:0] exu_i1_result_e1,

   input logic [31:0] exu_i0_result_e4,     // alu result e4
   input logic [31:0] exu_i1_result_e4,


   input logic [pt.NUM_THREADS-1:0]       ifu_i0_valid, ifu_i1_valid,    // fetch valids to instruction buffer
   input logic [pt.NUM_THREADS-1:0] [31:0]  ifu_i0_instr, ifu_i1_instr,    // fetch inst's to instruction buffer
   input logic [pt.NUM_THREADS-1:0] [31:1]  ifu_i0_pc, ifu_i1_pc,          // pc's for instruction buffer
   input logic [pt.NUM_THREADS-1:0]         ifu_i0_pc4, ifu_i1_pc4,        // indication of 4B or 2B for corresponding inst

   input eh2_predecode_pkt_t  [pt.NUM_THREADS-1:0] ifu_i0_predecode,
   input eh2_predecode_pkt_t  [pt.NUM_THREADS-1:0] ifu_i1_predecode,

   input logic  [31:1] exu_i0_pc_e1,                  // pc's for e1 from the alu's
   input logic  [31:1] exu_i1_pc_e1,

   input logic [pt.NUM_THREADS-1:0] timer_int,                             // Timer interrupt pending (from pin)
   input logic [pt.NUM_THREADS-1:0] soft_int,                             // Software interrupt pending (from pin)

   input logic [pt.NUM_THREADS-1:0]       mexintpend,                      // External interrupt pending
   input logic [pt.NUM_THREADS-1:0] [7:0] pic_claimid,                     // PIC claimid
   input logic [pt.NUM_THREADS-1:0] [3:0] pic_pl,                          // PIC priv level
   input logic [pt.NUM_THREADS-1:0]       mhwakeup,                        // High priority wakeup

   output logic [pt.NUM_THREADS-1:0][3:0] dec_tlu_meicurpl,               // to PIC, Current priv level
   output logic [pt.NUM_THREADS-1:0][3:0] dec_tlu_meipt,                  // to PIC
   output logic [31:2] dec_tlu_meihap, // Fast ext int base

   input logic [70:0] ifu_ic_debug_rd_data,           // diagnostic icache read data
   input logic ifu_ic_debug_rd_data_valid,            // diagnostic icache read data valid
   output eh2_cache_debug_pkt_t dec_tlu_ic_diag_pkt,      // packet of DICAWICS, DICAD0/1, DICAGO info for icache diagnostics


// Debug start
   input logic [pt.NUM_THREADS-1:0] dbg_halt_req,                 // DM requests a halt
   input logic [pt.NUM_THREADS-1:0] dbg_resume_req,               // DM requests a resume

   input logic [pt.NUM_THREADS-1:0] ifu_miss_state_idle,
   input logic [pt.NUM_THREADS-1:0] ifu_ic_error_start,
   input logic [pt.NUM_THREADS-1:0] ifu_iccm_rd_ecc_single_err,

   output logic [pt.NUM_THREADS-1:0] dec_tlu_dbg_halted,          // Core is halted and ready for debug command
   output logic [pt.NUM_THREADS-1:0] dec_tlu_debug_mode,          // Core is in debug mode
   output logic [pt.NUM_THREADS-1:0] dec_tlu_resume_ack,          // Resume acknowledge
   output logic [pt.NUM_THREADS-1:0] dec_tlu_mpc_halted_only,     // Core is halted only due to MPC

   output logic dec_debug_wdata_rs1_d,       // insert debug write data into rs1 at decode

   output logic [31:0] dec_dbg_rddata,       // debug command read data

   output logic dec_dbg_cmd_done,            // abstract command is done
   output logic dec_dbg_cmd_fail,            // abstract command failed (illegal reg address)
   output logic dec_dbg_cmd_tid,             // Tid for debug abstract command response

   output eh2_trigger_pkt_t  [pt.NUM_THREADS-1:0][3:0] trigger_pkt_any, // info needed by debug trigger blocks
// Debug end

   // branch info from pipe0 for errors or counter updates
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] exu_i0_br_index_e4,   // index
   input logic [1:0]  exu_i0_br_hist_e4,                             // history
   input logic        exu_i0_br_bank_e4,                             // bank
   input logic        exu_i0_br_error_e4,                            // error
   input logic        exu_i0_br_start_error_e4,                      // start error
   input logic        exu_i0_br_valid_e4,                            // valid
   input logic        exu_i0_br_mp_e4,                               // mispredict
   input logic        exu_i0_br_middle_e4,                           // middle of bank
   input logic [pt.BHT_GHR_SIZE-1:0] exu_i0_br_fghr_e4,              // FGHR when predicted

   // branch info from pipe1 for errors or counter updates
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] exu_i1_br_index_e4,   // index
   input logic [1:0]  exu_i1_br_hist_e4,                             // history
   input logic        exu_i1_br_bank_e4,                             // bank
   input logic        exu_i1_br_error_e4,                            // error
   input logic        exu_i1_br_start_error_e4,                      // start error
   input logic        exu_i1_br_valid_e4,                            // valid
   input logic        exu_i1_br_mp_e4,                               // mispredict
   input logic        exu_i1_br_middle_e4,                           // middle of bank
   input logic [pt.BHT_GHR_SIZE-1:0] exu_i1_br_fghr_e4,              // FGHR when predicted


   input logic        exu_i1_br_way_e4,             // way hit or repl
   input logic        exu_i0_br_way_e4,             // way hit or repl

   output logic [31:0] gpr_i0_rs1_d,               // gpr rs1 data
   output logic [31:0] gpr_i0_rs2_d,               // gpr rs2 data
   output logic [31:0] gpr_i1_rs1_d,
   output logic [31:0] gpr_i1_rs2_d,

   output logic [31:0] dec_i0_immed_d,              // immediate data
   output logic [31:0] dec_i1_immed_d,

   output logic [12:1] dec_i0_br_immed_d,           // br immediate data
   output logic [12:1] dec_i1_br_immed_d,

   output        eh2_alu_pkt_t i0_ap,                   // alu packet
   output        eh2_alu_pkt_t i1_ap,

   output logic          dec_i0_alu_decode_d,       // alu schedule on primary alu
   output logic          dec_i1_alu_decode_d,

   output logic          dec_i0_select_pc_d,        // select pc onto rs1 for jal's
   output logic          dec_i1_select_pc_d,

   output logic [31:1] dec_i0_pc_d, dec_i1_pc_d,    // pc's at decode
   output logic         dec_i0_rs1_bypass_en_d,     // rs1 bypass enable
   output logic         dec_i0_rs2_bypass_en_d,     // rs2 bypass enable
   output logic         dec_i1_rs1_bypass_en_d,
   output logic         dec_i1_rs2_bypass_en_d,

   output logic [31:0] i0_rs1_bypass_data_d,       // rs1 bypass data
   output logic [31:0] i0_rs2_bypass_data_d,       // rs2 bypass data
   output logic [31:0] i1_rs1_bypass_data_d,
   output logic [31:0] i1_rs2_bypass_data_d,
   output logic [pt.NUM_THREADS-1:0]        dec_ib3_valid_d,           // ib3 buffer valid
   output logic [pt.NUM_THREADS-1:0]        dec_ib2_valid_d,           // ib2 buffer valid

   output eh2_lsu_pkt_t    lsu_p,                      // lsu packet
   output eh2_mul_pkt_t    mul_p,                      // mul packet
   output eh2_div_pkt_t    div_p,                      // div packet

   output logic [11:0] dec_lsu_offset_d,           // 12b offset for load/store addresses
   output logic        dec_i0_lsu_d,               // is load/store
   output logic        dec_i1_lsu_d,

   output logic [pt.NUM_THREADS-1:0]       flush_final_e3,             // final flush
   output logic [pt.NUM_THREADS-1:0]       i0_flush_final_e3,          // final flush from i0

   output logic        dec_i0_csr_ren_d,              // csr read enable

   output logic        dec_tlu_i0_kill_writeb_wb,  // I0 is flushed, don't writeback any results to arch state
   output logic        dec_tlu_i1_kill_writeb_wb,  // I1 is flushed, don't writeback any results to arch state

   output logic        dec_i0_mul_d,               // chose which gpr value to use
   output logic        dec_i1_mul_d,
   output logic        dec_i0_div_d,               // chose which gpr value to use

   output logic        dec_i1_valid_e1,            // i1 valid at e1 stage

   output logic [pt.NUM_THREADS-1:0][31:1] pred_correct_npc_e2, // npc e2 if the prediction is correct

   output logic        dec_i0_rs1_bypass_en_e3,    // rs1 bypass enable e3
   output logic        dec_i0_rs2_bypass_en_e3,    // rs2 bypass enable e3
   output logic        dec_i1_rs1_bypass_en_e3,
   output logic        dec_i1_rs2_bypass_en_e3,
   output logic [31:0] i0_rs1_bypass_data_e3,      // rs1 bypass data e3
   output logic [31:0] i0_rs2_bypass_data_e3,      // rs2 bypass data e3
   output logic [31:0] i1_rs1_bypass_data_e3,
   output logic [31:0] i1_rs2_bypass_data_e3,
   output logic        dec_i0_sec_decode_e3,       // secondary decode e3
   output logic        dec_i1_sec_decode_e3,
   output logic [31:1] dec_i0_pc_e3,               // pc at e3
   output logic [31:1] dec_i1_pc_e3,

   output logic        dec_i0_rs1_bypass_en_e2,    // rs1 bypass enable e2
   output logic        dec_i0_rs2_bypass_en_e2,    // rs2 bypass enable e2
   output logic        dec_i1_rs1_bypass_en_e2,
   output logic        dec_i1_rs2_bypass_en_e2,
   output logic [31:0] i0_rs1_bypass_data_e2,      // rs1 bypass data e2
   output logic [31:0] i0_rs2_bypass_data_e2,      // rs2 bypass data e2
   output logic [31:0] i1_rs1_bypass_data_e2,
   output logic [31:0] i1_rs2_bypass_data_e2,

   output eh2_br_tlu_pkt_t dec_tlu_br0_wb_pkt,         // slot 0 branch predictor update packet
   output eh2_br_tlu_pkt_t dec_tlu_br1_wb_pkt,         // slot 1 branch predictor update packet
   output logic [pt.BHT_GHR_SIZE-1:0] dec_tlu_br0_fghr_wb, // fghr to bp
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br0_index_wb, // bp index
   output logic [pt.BHT_GHR_SIZE-1:0] dec_tlu_br1_fghr_wb, // fghr to bp
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br1_index_wb, // bp index

   output logic [pt.NUM_THREADS-1:0] [1:0] dec_tlu_perfcnt0, // toggles when pipe0 perf counter 0 has an event inc
   output logic [pt.NUM_THREADS-1:0] [1:0] dec_tlu_perfcnt1, // toggles when pipe0 perf counter 1 has an event inc
   output logic [pt.NUM_THREADS-1:0] [1:0] dec_tlu_perfcnt2, // toggles when pipe0 perf counter 2 has an event inc
   output logic [pt.NUM_THREADS-1:0] [1:0] dec_tlu_perfcnt3, // toggles when pipe0 perf counter 3 has an event inc

   output eh2_predict_pkt_t  i0_predict_p_d,           // prediction packet to alus
   output eh2_predict_pkt_t  i1_predict_p_d,
   output logic [pt.BHT_GHR_SIZE-1:0] i0_predict_fghr_d,                // DEC predict fghr
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] i0_predict_index_d,     // DEC predict index
   output logic [pt.BTB_BTAG_SIZE-1:0] i0_predict_btag_d,               // DEC predict branch tgt
   output logic [pt.BHT_GHR_SIZE-1:0] i1_predict_fghr_d,                // DEC predict fghr
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] i1_predict_index_d,     // DEC predict index
   output logic [pt.BTB_BTAG_SIZE-1:0] i1_predict_btag_d,               // DEC predict branch tgt


   output logic [31:0] i0_result_e4_eff,           // alu result e4
   output logic [31:0] i1_result_e4_eff,

   output   logic dec_tlu_i0_valid_e4,             // slot 0 instruction is valid at e4
   output   logic dec_tlu_i1_valid_e4,             // slot 1 instruction is valid at e4, implies i0_valid_e4

   output logic [31:0] i0_result_e2,               // i0 result data e2
   output logic [31:0] dec_tlu_mrac_ff,            // CSR for memory region control

   output logic [4:2] dec_i0_data_en,              // clock-gate control logic
   output logic [4:1] dec_i0_ctl_en,
   output logic [4:2] dec_i1_data_en,
   output logic [4:1] dec_i1_ctl_en,

   output logic [pt.NUM_THREADS-1:0] dec_tlu_lr_reset_wb, // Reset the reservation on certain events

   input logic [pt.NUM_THREADS-1:0] [15:0] ifu_i0_cinst,                  // 16b compressed instruction for trace
   input logic [pt.NUM_THREADS-1:0] [15:0] ifu_i1_cinst,

   output eh2_trace_pkt_t  [pt.NUM_THREADS-1:0] rv_trace_pkt,             // trace packet for trace

   // feature disable from mfdc
   output logic  dec_tlu_external_ldfwd_disable, // disable external load forwarding
   output logic  dec_tlu_sideeffect_posted_disable, // disable posted writes to side-effect address
   output logic  dec_tlu_core_ecc_disable,           // disable core ECC
   output logic  dec_tlu_bpred_disable,              // disable branch prediction
   output logic  dec_tlu_wb_coalescing_disable,      // disable writebuffer coalescing
   output logic [2:0]  dec_tlu_dma_qos_prty,         // DMA QoS priority coming from MFDC [18:16]
   output logic [pt.NUM_THREADS-1:0]      dec_tlu_i0_commit_cmt,        // goes to IFU for commit 1 instruction in the FSM
   // clock gating overrides from mcgc
   output logic  dec_tlu_misc_clk_override,          // override misc clock domain gating
   output logic  dec_tlu_exu_clk_override,           // override exu clock domain gating
   output logic  dec_tlu_ifu_clk_override,           // override fetch clock domain gating
   output logic  dec_tlu_lsu_clk_override,           // override load/store clock domain gating
   output logic  dec_tlu_bus_clk_override,           // override bus clock domain gating
   output logic  dec_tlu_pic_clk_override,           // override PIC clock domain gating
   output logic  dec_tlu_dccm_clk_override,          // override DCCM clock domain gating
   output logic  dec_tlu_icm_clk_override,           // override ICCM clock domain gating

   output logic dec_i0_tid_e4, // needed to maintain RS in BP
   output logic dec_i1_tid_e4,

   output logic [pt.NUM_THREADS-1:0] [31:1] dec_tlu_flush_path_wb,  // flush pc
   output logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_lower_wb, // commit has a flush (exception, int, mispredict at e4)
   output logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_noredir_wb , // Tell fetch to idle on this flush
   output logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_leak_one_wb, // single step
   output logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_err_wb, // iside perr/ecc rfpc
   output logic [pt.NUM_THREADS-1:0]        dec_tlu_fence_i_wb,     // flush is a fence_i rfnpc, flush icache
   //
   input  logic        scan_mode

   );

   localparam GPR_BANKS = 1;
   localparam GPR_BANKS_LOG2 = (GPR_BANKS == 1) ? 1 : $clog2(GPR_BANKS);

   logic [pt.NUM_THREADS-1:0] dec_tlu_flush_pause_wb;
   logic [pt.NUM_THREADS-1:0] dec_tlu_wr_pause_wb;


   logic  dec_tlu_dec_clk_override; // to and from dec blocks
   logic  clk_override;

   logic               dec_ib1_valid_d;
   logic               dec_ib0_valid_d;


   logic [pt.NUM_THREADS-1:0][1:0] dec_pmu_instr_decoded;
   logic [pt.NUM_THREADS-1:0]      dec_pmu_decode_stall;
   logic [pt.NUM_THREADS-1:0]      dec_pmu_presync_stall;
   logic [pt.NUM_THREADS-1:0]      dec_pmu_postsync_stall;

   logic        dec_i0_rs1_en_d;
   logic        dec_i0_rs2_en_d;

   logic [4:0]  dec_i0_rs1_d;
   logic [4:0]  dec_i0_rs2_d;


   logic        dec_i1_rs1_en_d;
   logic        dec_i1_rs2_en_d;

   logic [4:0]  dec_i1_rs1_d;
   logic [4:0]  dec_i1_rs2_d;


   logic [31:0] dec_i0_instr_d, dec_i1_instr_d;

   logic  dec_tlu_pipelining_disable;
   logic  dec_tlu_dual_issue_disable;


   logic [4:0]  dec_i0_waddr_wb;
   logic        dec_i0_wen_wb;
   logic        dec_i0_tid_wb;
   logic [31:0] dec_i0_wdata_wb;

   logic [4:0]  dec_i1_waddr_wb;
   logic        dec_i1_wen_wb;
   logic        dec_i1_tid_wb;
   logic [31:0] dec_i1_wdata_wb;

   logic        dec_i0_csr_wen_wb;      // csr write enable at wb
   logic [11:0] dec_i0_csr_rdaddr_d;      // read address for csr
   logic [11:0] dec_i0_csr_wraddr_wb;      // write address for csryes
   logic        dec_i0_csr_is_mcpc_e4;

   logic [31:0] dec_i0_csr_wrdata_wb;    // csr write data at wb

   logic [31:0] dec_i0_csr_rddata_d;    // csr read data at wb
   logic        dec_i0_csr_legal_d;            // csr indicates legal operation
   logic        dec_i0_csr_global_d;

   logic        dec_i0_csr_wen_unq_d;       // valid csr with write - for csr legal
   logic        dec_i0_csr_any_unq_d;       // valid csr - for csr legal


   logic [pt.NUM_THREADS-1:0] dec_csr_stall_int_ff; // csr is mie/mstatus
   logic                      dec_csr_nmideleg_e4;  // csr is mnmipdel


   eh2_trap_pkt_t dec_tlu_packet_e4;

   logic                        dec_i1_debug_valid_d;

   logic                        dec_i0_pc4_d, dec_i1_pc4_d;
   logic [pt.NUM_THREADS-1:0]   dec_tlu_presync_d;
   logic [pt.NUM_THREADS-1:0]   dec_tlu_postsync_d;
   logic [pt.NUM_THREADS-1:0]   dec_tlu_debug_stall;
 // stall decode while waiting on core to empty

   logic [pt.NUM_THREADS-1:0][31:0] dec_illegal_inst;


   logic                      dec_i0_icaf_d;
   logic [1:0]                dec_i0_icaf_type_d;
   logic                      dec_i0_icaf_f1_d;

   logic                      dec_i1_icaf_d;
   logic [1:0]                dec_i1_icaf_type_d;
   logic                      dec_i1_icaf_f1_d;

   logic                      dec_i0_dbecc_d;
   logic                      dec_i1_dbecc_d;

   logic                      dec_i0_decode_d;
   logic                      dec_i1_decode_d;

   logic [3:0]                dec_i0_trigger_match_d;
   logic [3:0]                dec_i1_trigger_match_d;


   logic                      dec_debug_fence_d;

   logic [pt.NUM_THREADS-1:0]                 dec_nonblock_load_wen;
   logic [pt.NUM_THREADS-1:0][4:0]            dec_nonblock_load_waddr;

   eh2_br_pkt_t dec_i0_brp;
   eh2_br_pkt_t dec_i1_brp;

   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_i0_bp_index;
   logic [pt.BHT_GHR_SIZE-1:0] dec_i0_bp_fghr;
   logic [pt.BTB_BTAG_SIZE-1:0] dec_i0_bp_btag;
   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_i1_bp_index;
   logic [pt.BHT_GHR_SIZE-1:0] dec_i1_bp_fghr;
   logic [pt.BTB_BTAG_SIZE-1:0] dec_i1_bp_btag;

   logic [pt.NUM_THREADS-1:0] dec_pause_state;          // core in pause state

   logic [15:0] dec_i0_cinst_d;
   logic [15:0] dec_i1_cinst_d;

   eh2_predecode_pkt_t dec_i0_predecode;
   eh2_predecode_pkt_t dec_i1_predecode;

   logic [31:0]               dec_i0_inst_wb1;
   logic [31:0]               dec_i1_inst_wb1;
   logic [31:1]               dec_i0_pc_wb1;
   logic [31:1]               dec_i1_pc_wb1;
   logic [pt.NUM_THREADS-1:0] dec_tlu_i1_valid_wb1, dec_tlu_i0_valid_wb1,  dec_tlu_int_valid_wb1;
   logic [pt.NUM_THREADS-1:0] [4:0] dec_tlu_exc_cause_wb1;
   logic [pt.NUM_THREADS-1:0] [31:0] dec_tlu_mtval_wb1;
   logic [pt.NUM_THREADS-1:0]   dec_tlu_i0_exc_valid_wb1, dec_tlu_i1_exc_valid_wb1;

   logic dec_i0_tid_d;
   logic dec_i1_tid_d;


   logic [1:0] [31:0] gpr_i0rs1_d;               // gpr rs1 data
   logic [1:0] [31:0] gpr_i0rs2_d;               // gpr rs2 data
   logic [1:0] [31:0] gpr_i1rs1_d;
   logic [1:0] [31:0] gpr_i1rs2_d;

   logic [31:1] dec_tlu_i0_pc_e4;                // pc e4
   logic [31:1] dec_tlu_i1_pc_e4;



   assign clk_override = dec_tlu_dec_clk_override;


   assign dec_dbg_rddata[31:0] = dec_i0_wdata_wb[31:0];

// multithreaded signals

   logic [pt.NUM_THREADS-1:0] ib3_valid_d;               // ib3 valid
   logic [pt.NUM_THREADS-1:0] ib2_valid_d;               // ib2 valid
   logic [pt.NUM_THREADS-1:0] ib1_valid_d;               // ib1 valid
   logic [pt.NUM_THREADS-1:0] ib0_valid_d;               // ib0 valid
   logic [pt.NUM_THREADS-1:0] ib0_valid_in;              // ib0 valid cycle before decode
   logic [pt.NUM_THREADS-1:0] ib0_lsu_in;              // ib0 lsu cycle before decode
   logic [pt.NUM_THREADS-1:0] ib0_mul_in;              // ib0 mul cycle before decode
   logic [pt.NUM_THREADS-1:0] ib0_i0_only_in;          // ib0 i0_only cycle before decode

   logic [pt.NUM_THREADS-1:0] [31:0] i0_instr_d;         // i0 inst at decode
   logic [pt.NUM_THREADS-1:0] [31:0] i1_instr_d;         // i1 inst at decode
   logic [pt.NUM_THREADS-1:0] [31:1] i0_pc_d;            // i0 pc at decode
   logic [pt.NUM_THREADS-1:0] [31:1] i1_pc_d;
   logic [pt.NUM_THREADS-1:0] i0_pc4_d;                  // i0 is 4B inst else 2B
   logic [pt.NUM_THREADS-1:0] i1_pc4_d;
   logic [pt.NUM_THREADS-1:0] [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] i0_bp_index;            // i0 branch index
   logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]           i0_bp_fghr; // BP FGHR
   logic [pt.NUM_THREADS-1:0] [pt.BTB_BTAG_SIZE-1:0]          i0_bp_btag; // BP tag
   logic [pt.NUM_THREADS-1:0] [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] i1_bp_index;            // i0 branch index
   logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]           i1_bp_fghr; // BP FGHR
   logic [pt.NUM_THREADS-1:0] [pt.BTB_BTAG_SIZE-1:0]          i1_bp_btag; // BP tag
   logic [pt.NUM_THREADS-1:0] i0_icaf_d;                 // i0 instruction access fault at decode
   logic [pt.NUM_THREADS-1:0] i1_icaf_d;
   logic [pt.NUM_THREADS-1:0] i0_icaf_f1_d;              // i0 instruction access fault at decode for f1 fetch group
   logic [pt.NUM_THREADS-1:0] i1_icaf_f1_d;              // i0 instruction access fault at decode for f1 fetch group
   logic [pt.NUM_THREADS-1:0] i0_dbecc_d;                // i0 double-bit error at decode
   logic [pt.NUM_THREADS-1:0] i1_dbecc_d;
   logic [pt.NUM_THREADS-1:0] debug_wdata_rs1_d;         // put debug write data onto rs1 source: machine is halted
   logic [pt.NUM_THREADS-1:0] debug_fence_d;             // debug fence inst
   logic [pt.NUM_THREADS-1:0] i0_debug_valid_d;             // debug fence inst
   logic [pt.NUM_THREADS-1:0] [15:0] i0_cinst_d;         // 16b compress inst at decode
   logic [pt.NUM_THREADS-1:0] [15:0] i1_cinst_d;
   logic [pt.NUM_THREADS-1:0] [1:0] i0_icaf_type_d;
   logic [pt.NUM_THREADS-1:0] [1:0] i1_icaf_type_d;

   eh2_br_pkt_t [pt.NUM_THREADS-1:0] i0_br_p;                 // i0 branch packet at decode
   eh2_br_pkt_t [pt.NUM_THREADS-1:0] i1_br_p;

   eh2_predecode_pkt_t [pt.NUM_THREADS-1:0] i0_predecode_p;                 // i0 branch packet at decode
   eh2_predecode_pkt_t [pt.NUM_THREADS-1:0] i1_predecode_p;

   logic [pt.NUM_THREADS-1:0]         ready_in,ready;
   logic [pt.NUM_THREADS-1:0]         lsu_in, mul_in, i0_only_in;
   logic [pt.NUM_THREADS-1:0]         dec_thread_stall_in;
   logic [pt.NUM_THREADS-1:0]         dec_tlu_flush_extint;

   logic [4:0] div_waddr_wb;
   logic       div_tid_wb;

   logic       dec_div_active;    // non-block divide is active
   logic       dec_div_tid;       // non-block divide tid


   // dec_i1_cancel_e1 needs to be threaded

  for (genvar i=0; i<pt.NUM_THREADS; i++) begin : ib


     eh2_dec_ib_ctl #(.pt(pt)) instbuff (.tid               (1'(i)            ),
                                          .ifu_i0_valid      (ifu_i0_valid[i]),
                                          .ifu_i1_valid      (ifu_i1_valid[i]),
                                          .ifu_i0_icaf       (ifu_i0_icaf[i]),
                                          .ifu_i0_icaf_type  (ifu_i0_icaf_type[i]),
                                          .ifu_i0_icaf_f1    (ifu_i0_icaf_f1[i]),
                                          .ifu_i0_dbecc      (ifu_i0_dbecc[i]),
                                          .ifu_i0_instr      (ifu_i0_instr[i]),
                                          .ifu_i1_instr      (ifu_i1_instr[i]),
                                          .ifu_i0_pc         (ifu_i0_pc[i]),
                                          .ifu_i1_pc         (ifu_i1_pc[i]),
                                          .ifu_i0_pc4        (ifu_i0_pc4[i]),
                                          .ifu_i1_pc4        (ifu_i1_pc4[i]),
                                          .ifu_i0_predecode  (ifu_i0_predecode[i]),
                                          .ifu_i1_predecode  (ifu_i1_predecode[i]),
                                          .i0_brp            (i0_brp[i]),
                                          .i1_brp            (i1_brp[i]),
                                          .ifu_i0_bp_index   (ifu_i0_bp_index[i]),
                                          .ifu_i0_bp_fghr    (ifu_i0_bp_fghr[i]),
                                          .ifu_i0_bp_btag    (ifu_i0_bp_btag[i]),
                                          .ifu_i1_bp_index   (ifu_i1_bp_index[i]),
                                          .ifu_i1_bp_fghr    (ifu_i1_bp_fghr[i]),
                                          .ifu_i1_bp_btag    (ifu_i1_bp_btag[i]),
                                          .ifu_i0_cinst      (ifu_i0_cinst[i]),
                                          .ifu_i1_cinst      (ifu_i1_cinst[i]),

                                          .dec_i1_cancel_e1  (dec_i1_cancel_e1[i] ),
                                          .exu_flush_final   (exu_flush_final[i] ),
                                          .ib3_valid_d       (ib3_valid_d[i]   ),
                                          .ib2_valid_d       (ib2_valid_d[i]   ),
                                          .ib1_valid_d       (ib1_valid_d[i]   ),
                                          .ib0_valid_d       (ib0_valid_d[i]   ),
                                          .ib0_valid_in      (ib0_valid_in[i]   ),
                                          .ib0_lsu_in        (ib0_lsu_in[i]   ),
                                          .ib0_mul_in        (ib0_mul_in[i]   ),
                                          .ib0_i0_only_in    (ib0_i0_only_in[i]   ),
                                          .i0_instr_d        (i0_instr_d[i]    ),
                                          .i1_instr_d        (i1_instr_d[i]    ),
                                          .i0_debug_valid_d  (i0_debug_valid_d[i] ),
                                          .i0_pc_d           (i0_pc_d[i]       ),
                                          .i1_pc_d           (i1_pc_d[i]       ),
                                          .i0_pc4_d          (i0_pc4_d[i]      ),
                                          .i1_pc4_d          (i1_pc4_d[i]      ),
                                          .i0_bp_index       (i0_bp_index[i]   ),
                                          .i0_bp_fghr        (i0_bp_fghr[i]    ),
                                          .i0_bp_btag        (i0_bp_btag[i]    ),
                                          .i1_bp_index       (i1_bp_index[i]   ),
                                          .i1_bp_fghr        (i1_bp_fghr[i]    ),
                                          .i1_bp_btag        (i1_bp_btag[i]    ),
                                          .i0_icaf_d         (i0_icaf_d[i]     ),
                                          .i1_icaf_d         (i1_icaf_d[i]     ),
                                          .i0_icaf_f1_d      (i0_icaf_f1_d[i]  ),
                                          .i1_icaf_f1_d      (i1_icaf_f1_d[i]  ),
                                          .i0_dbecc_d        (i0_dbecc_d[i]    ),
                                          .i1_dbecc_d        (i1_dbecc_d[i]    ),
                                          .debug_wdata_rs1_d (debug_wdata_rs1_d[i]),
                                          .debug_fence_d     (debug_fence_d[i] ),
                                          .i0_cinst_d        (i0_cinst_d[i]    ),
                                          .i1_cinst_d        (i1_cinst_d[i]    ),
                                          .i0_icaf_type_d    (i0_icaf_type_d[i]),
                                          .i1_icaf_type_d    (i1_icaf_type_d[i]),
                                          .i0_br_p           (i0_br_p[i]       ),
                                          .i1_br_p           (i1_br_p[i]       ),
                                          .i0_predecode      (i0_predecode_p[i]       ),
                                          .i1_predecode      (i1_predecode_p[i]       ),
                                          .*);


  end // block: ib


   for (genvar i=0; i<pt.NUM_THREADS; i++) begin : arf

      eh2_dec_gpr_ctl #(.pt(pt)) arf (.*,
                                       .tid (1'(i)),

                                       .rtid0(dec_i0_tid_d),
                                       .rtid1(dec_i0_tid_d),
                                       .rtid2(dec_i1_tid_d),
                                       .rtid3(dec_i1_tid_d),

                                       // inputs
                                       .raddr0(dec_i0_rs1_d[4:0]), .rden0(dec_i0_rs1_en_d),
                                       .raddr1(dec_i0_rs2_d[4:0]), .rden1(dec_i0_rs2_en_d),
                                       .raddr2(dec_i1_rs1_d[4:0]), .rden2(dec_i1_rs1_en_d),
                                       .raddr3(dec_i1_rs2_d[4:0]), .rden3(dec_i1_rs2_en_d),

                                       .wtid0(dec_i0_tid_wb),              .waddr0(dec_i0_waddr_wb[4:0]),            .wen0(dec_i0_wen_wb),            .wd0(dec_i0_wdata_wb[31:0]),
                                       .wtid1(dec_i1_tid_wb),              .waddr1(dec_i1_waddr_wb[4:0]),            .wen1(dec_i1_wen_wb),            .wd1(dec_i1_wdata_wb[31:0]),
                                       .wtid2(lsu_nonblock_load_data_tid), .waddr2(dec_nonblock_load_waddr[i][4:0]), .wen2(dec_nonblock_load_wen[i]), .wd2(lsu_nonblock_load_data[31:0]),
                                       .wtid3(div_tid_wb),                 .waddr3(div_waddr_wb[4:0]),               .wen3(exu_div_wren),             .wd3(exu_div_result[31:0]),

                                       // outputs
                                       .rd0(gpr_i0rs1_d[i]), .rd1(gpr_i0rs2_d[i]),
                                       .rd2(gpr_i1rs1_d[i]), .rd3(gpr_i1rs2_d[i])
                                       );


   end // block: arf




   assign ready_in[pt.NUM_THREADS-1:0] = ib0_valid_in[pt.NUM_THREADS-1:0];
   assign lsu_in[pt.NUM_THREADS-1:0] = ib0_lsu_in[pt.NUM_THREADS-1:0];
   assign mul_in[pt.NUM_THREADS-1:0] = ib0_mul_in[pt.NUM_THREADS-1:0];
   assign i0_only_in[pt.NUM_THREADS-1:0] = ib0_i0_only_in[pt.NUM_THREADS-1:0];

   logic i0_sel_i0_t1_d;
   logic [1:0] i1_sel_i0_d, i1_sel_i1_d;


   if (pt.NUM_THREADS == 1) begin: genst
      assign gpr_i0_rs1_d[31:0] = gpr_i0rs1_d[0];
      assign gpr_i0_rs2_d[31:0] = gpr_i0rs2_d[0];
      assign gpr_i1_rs1_d[31:0] = gpr_i1rs1_d[0];
      assign gpr_i1_rs2_d[31:0] = gpr_i1rs2_d[0];

      assign dec_i0_tid_d = 1'b0;
      assign dec_i1_tid_d = 1'b0;

      assign ready[0] = 1'b1;

      assign i0_sel_i0_t1_d = 1'b0;
      assign i1_sel_i0_d[1:0] = 2'b00;
      assign i1_sel_i1_d[1:0] = 2'b01;

   end

   else begin: genmt

      assign gpr_i0_rs1_d[31:0] = gpr_i0rs1_d[1] | gpr_i0rs1_d[0];
      assign gpr_i0_rs2_d[31:0] = gpr_i0rs2_d[1] | gpr_i0rs2_d[0];
      assign gpr_i1_rs1_d[31:0] = gpr_i1rs1_d[1] | gpr_i1rs1_d[0];
      assign gpr_i1_rs2_d[31:0] = gpr_i1rs2_d[1] | gpr_i1rs2_d[0];



      rvarbiter2_smt dec_arbiter (
                                  .flush(exu_flush_final[1:0]),
                                  .shift(dec_i0_decode_d),
                                  .ready_in(ready_in[1:0]),
                                  .lsu_in(lsu_in[1:0]),
                                  .mul_in(mul_in[1:0]),
                                  .i0_only_in(i0_only_in[1:0]),
                                  .thread_stall_in(dec_thread_stall_in[1:0]),
                                  .ready(ready[1:0]),
                                  .i0_sel_i0_t1(i0_sel_i0_t1_d),
                                  .i1_sel_i0(i1_sel_i0_d[1:0]),
                                  .i1_sel_i1(i1_sel_i1_d[1:0]),
                                  .*
                                  );

      assign dec_i0_tid_d = i0_sel_i0_t1_d;

      assign dec_i1_tid_d = i1_sel_i1_d[1] | i1_sel_i0_d[1];
   end

//   end // block: genmt


   // send to aligner
   assign dec_ib3_valid_d[pt.NUM_THREADS-1:0]       = ib3_valid_d[pt.NUM_THREADS-1:0];
   assign dec_ib2_valid_d[pt.NUM_THREADS-1:0]       = ib2_valid_d[pt.NUM_THREADS-1:0];

   assign dec_ib0_valid_d       = ib0_valid_d[dec_i0_tid_d] & ready[dec_i0_tid_d]     ;
   assign dec_i0_instr_d        = i0_instr_d[dec_i0_tid_d]        ;
   assign dec_i0_pc_d           = i0_pc_d[dec_i0_tid_d]           ;
   assign dec_i0_pc4_d          = i0_pc4_d[dec_i0_tid_d]          ;
   assign dec_i0_bp_index       = i0_bp_index[dec_i0_tid_d]       ;
   assign dec_i0_bp_fghr        = i0_bp_fghr[dec_i0_tid_d]        ;
   assign dec_i0_bp_btag        = i0_bp_btag[dec_i0_tid_d]        ;
   assign dec_i0_icaf_d         = i0_icaf_d[dec_i0_tid_d]         ;
   assign dec_i0_icaf_f1_d      = i0_icaf_f1_d[dec_i0_tid_d]      ;
   assign dec_i0_dbecc_d        = i0_dbecc_d[dec_i0_tid_d]        ;
   assign dec_i0_cinst_d        = i0_cinst_d[dec_i0_tid_d]        ;
   assign dec_i0_icaf_type_d    = i0_icaf_type_d[dec_i0_tid_d]    ;
   assign dec_i0_brp            = i0_br_p[dec_i0_tid_d]           ;
   assign dec_i0_predecode      = i0_predecode_p[dec_i0_tid_d]           ;


   assign dec_debug_wdata_rs1_d = debug_wdata_rs1_d[dec_i0_tid_d] ;
   assign dec_debug_fence_d     = debug_fence_d[dec_i0_tid_d]     ;

   // only SMT is supported for threading
   if (pt.NUM_THREADS==2 )  begin

      // pipe is flushed; should not need ready[]
      assign dec_i1_debug_valid_d  = (i1_sel_i0_d[0] & i0_debug_valid_d[0]) |
                                     (i1_sel_i0_d[1] & i0_debug_valid_d[1]);

      assign dec_ib1_valid_d       = (i1_sel_i0_d[0] & ib0_valid_d[0] & ready[0]) |
                                     (i1_sel_i1_d[0] & ib1_valid_d[0] & ready[0]) |
                                     (i1_sel_i0_d[1] & ib0_valid_d[1] & ready[1]) |
                                     (i1_sel_i1_d[1] & ib1_valid_d[1] & ready[1]);


      assign dec_i1_instr_d        = ({32{i1_sel_i0_d[0]}} & i0_instr_d[0]) |
                                     ({32{i1_sel_i1_d[0]}} & i1_instr_d[0]) |
                                     ({32{i1_sel_i0_d[1]}} & i0_instr_d[1]) |
                                     ({32{i1_sel_i1_d[1]}} & i1_instr_d[1]);

      assign dec_i1_pc_d           = ({31{i1_sel_i0_d[0]}} & i0_pc_d[0]) |
                                     ({31{i1_sel_i1_d[0]}} & i1_pc_d[0]) |
                                     ({31{i1_sel_i0_d[1]}} & i0_pc_d[1]) |
                                     ({31{i1_sel_i1_d[1]}} & i1_pc_d[1]);

      assign dec_i1_pc4_d          = (i1_sel_i0_d[0] & i0_pc4_d[0]) |
                                     (i1_sel_i1_d[0] & i1_pc4_d[0]) |
                                     (i1_sel_i0_d[1] & i0_pc4_d[1]) |
                                     (i1_sel_i1_d[1] & i1_pc4_d[1]);


      assign dec_i1_bp_index           = ({pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1{i1_sel_i0_d[0]}} & i0_bp_index[0]) |
                                         ({pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1{i1_sel_i1_d[0]}} & i1_bp_index[0]) |
                                         ({pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1{i1_sel_i0_d[1]}} & i0_bp_index[1]) |
                                         ({pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1{i1_sel_i1_d[1]}} & i1_bp_index[1]);

      assign dec_i1_bp_fghr            = ({pt.BHT_GHR_SIZE{i1_sel_i0_d[0]}} & i0_bp_fghr[0]) |
                                         ({pt.BHT_GHR_SIZE{i1_sel_i1_d[0]}} & i1_bp_fghr[0]) |
                                         ({pt.BHT_GHR_SIZE{i1_sel_i0_d[1]}} & i0_bp_fghr[1]) |
                                         ({pt.BHT_GHR_SIZE{i1_sel_i1_d[1]}} & i1_bp_fghr[1]);

      assign dec_i1_bp_btag            = ({pt.BTB_BTAG_SIZE{i1_sel_i0_d[0]}} & i0_bp_btag[0]) |
                                         ({pt.BTB_BTAG_SIZE{i1_sel_i1_d[0]}} & i1_bp_btag[0]) |
                                         ({pt.BTB_BTAG_SIZE{i1_sel_i0_d[1]}} & i0_bp_btag[1]) |
                                         ({pt.BTB_BTAG_SIZE{i1_sel_i1_d[1]}} & i1_bp_btag[1]);

      assign dec_i1_icaf_d          = (i1_sel_i0_d[0] & i0_icaf_d[0]) |
                                      (i1_sel_i1_d[0] & i1_icaf_d[0]) |
                                      (i1_sel_i0_d[1] & i0_icaf_d[1]) |
                                      (i1_sel_i1_d[1] & i1_icaf_d[1]);

      assign dec_i1_icaf_f1_d          = (i1_sel_i0_d[0] & i0_icaf_f1_d[0]) |
                                         (i1_sel_i1_d[0] & i1_icaf_f1_d[0]) |
                                         (i1_sel_i0_d[1] & i0_icaf_f1_d[1]) |
                                         (i1_sel_i1_d[1] & i1_icaf_f1_d[1]);

      assign dec_i1_dbecc_d          = (i1_sel_i0_d[0] & i0_dbecc_d[0]) |
                                       (i1_sel_i1_d[0] & i1_dbecc_d[0]) |
                                       (i1_sel_i0_d[1] & i0_dbecc_d[1]) |
                                       (i1_sel_i1_d[1] & i1_dbecc_d[1]);

      assign dec_i1_cinst_d         = ({16{i1_sel_i0_d[0]}} & i0_cinst_d[0]) |
                                      ({16{i1_sel_i1_d[0]}} & i1_cinst_d[0]) |
                                      ({16{i1_sel_i0_d[1]}} & i0_cinst_d[1]) |
                                      ({16{i1_sel_i1_d[1]}} & i1_cinst_d[1]);

      assign dec_i1_icaf_type_d         = ({2{i1_sel_i0_d[0]}} & i0_icaf_type_d[0]) |
                                          ({2{i1_sel_i1_d[0]}} & i1_icaf_type_d[0]) |
                                          ({2{i1_sel_i0_d[1]}} & i0_icaf_type_d[1]) |
                                          ({2{i1_sel_i1_d[1]}} & i1_icaf_type_d[1]);


      assign dec_i1_brp                 = ({$bits(eh2_br_pkt_t){i1_sel_i0_d[0]}} & i0_br_p[0]) |
                                          ({$bits(eh2_br_pkt_t){i1_sel_i1_d[0]}} & i1_br_p[0]) |
                                          ({$bits(eh2_br_pkt_t){i1_sel_i0_d[1]}} & i0_br_p[1]) |
                                          ({$bits(eh2_br_pkt_t){i1_sel_i1_d[1]}} & i1_br_p[1]);

      assign dec_i1_predecode                 = ({$bits(eh2_predecode_pkt_t){i1_sel_i0_d[0]}} & i0_predecode_p[0]) |
                                                ({$bits(eh2_predecode_pkt_t){i1_sel_i1_d[0]}} & i1_predecode_p[0]) |
                                                ({$bits(eh2_predecode_pkt_t){i1_sel_i0_d[1]}} & i0_predecode_p[1]) |
                                                ({$bits(eh2_predecode_pkt_t){i1_sel_i1_d[1]}} & i1_predecode_p[1]);

   end
   else begin
      assign dec_i1_debug_valid_d  = '0;   // for 1 thread cannot have debug commands in i1

      assign dec_ib1_valid_d       = ib1_valid_d[dec_i1_tid_d] & ready[dec_i1_tid_d]      ;
      assign dec_i1_instr_d        = i1_instr_d[dec_i1_tid_d]        ;
      assign dec_i1_pc_d           = i1_pc_d[dec_i1_tid_d]           ;
      assign dec_i1_pc4_d          = i1_pc4_d[dec_i1_tid_d]          ;
      assign dec_i1_bp_index       = i1_bp_index[dec_i1_tid_d]       ;
      assign dec_i1_bp_fghr        = i1_bp_fghr[dec_i1_tid_d]        ;
      assign dec_i1_bp_btag        = i1_bp_btag[dec_i1_tid_d]        ;
      assign dec_i1_icaf_d         = i1_icaf_d[dec_i1_tid_d]         ;
      assign dec_i1_icaf_f1_d      = i1_icaf_f1_d[dec_i1_tid_d]      ;
      assign dec_i1_dbecc_d        = i1_dbecc_d[dec_i1_tid_d]        ;
      assign dec_i1_cinst_d        = i1_cinst_d[dec_i1_tid_d]        ;
      assign dec_i1_icaf_type_d    = i1_icaf_type_d[dec_i1_tid_d]    ;
      assign dec_i1_brp            = i1_br_p[dec_i1_tid_d]           ;
      assign dec_i1_predecode      = i1_predecode_p[dec_i1_tid_d]           ;


   end


   eh2_dec_decode_ctl #(.pt(pt)) decode (.*);

   eh2_dec_tlu_top #(.pt(pt)) tlu (.*);


// Trigger

   eh2_dec_trigger #(.pt(pt)) dec_trigger (.*);




// trace
   // also need retires_p==3
     for (genvar i=0; i<pt.NUM_THREADS; i++) begin : tracep

        assign rv_trace_pkt[i].rv_i_insn_ip    = { dec_i1_inst_wb1[31:0],     dec_i0_inst_wb1[31:0] };
        assign rv_trace_pkt[i].rv_i_address_ip = { dec_i1_pc_wb1[31:1], 1'b0, dec_i0_pc_wb1[31:1], 1'b0 };

        assign rv_trace_pkt[i].rv_i_valid_ip =     {dec_tlu_int_valid_wb1[i],   // always int
                                                         dec_tlu_i1_valid_wb1[i] | dec_tlu_i1_exc_valid_wb1[i],  // not interrupts
                                                         dec_tlu_i0_valid_wb1[i] | dec_tlu_i0_exc_valid_wb1[i]
                                                         };
        assign rv_trace_pkt[i].rv_i_exception_ip = {dec_tlu_int_valid_wb1[i], dec_tlu_i1_exc_valid_wb1[i], dec_tlu_i0_exc_valid_wb1[i]};
        assign rv_trace_pkt[i].rv_i_ecause_ip =     dec_tlu_exc_cause_wb1[i][4:0];  // replicate across ports
        assign rv_trace_pkt[i].rv_i_interrupt_ip = {dec_tlu_int_valid_wb1[i],2'b0};
        assign rv_trace_pkt[i].rv_i_tval_ip =    dec_tlu_mtval_wb1[i][31:0];        // replicate across ports
     end


// end trace

endmodule // dec

