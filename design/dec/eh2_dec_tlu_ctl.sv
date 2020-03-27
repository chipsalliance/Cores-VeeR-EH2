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
// eh2_dec_tlu_ctl.sv
//
//
// Function: CSRs, flushing, exceptions, interrupts
// Comments:
//
//********************************************************************************

module eh2_dec_tlu_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (
   input logic clk,
   input logic active_clk,
   input logic free_clk,
   input logic rst_l,
   input logic scan_mode,
   input logic mytid, // tid of this instance

   input logic tlu_select_tid, // selected tid for fast int

   input logic dec_tlu_dec_clk_override,

   input logic [31:1] rst_vec, // reset vector, from core pins
   input logic        nmi_int_sync, // nmi pin
   input logic [31:1] nmi_vec, // nmi vector
   input logic  i_cpu_halt_req,    // Asynchronous Halt request to CPU
   input logic  i_cpu_run_req,     // Asynchronous Restart request to CPU
   input logic mhartstart_csr,    // Start valid

   input logic lsu_fastint_stall_any,   // needed by lsu for 2nd pass of dma with ecc correction, stall next cycle

   // perf counter inputs
   input logic [1:0] ifu_pmu_instr_aligned,   // aligned instructions
   input logic       ifu_pmu_align_stall,  // aligner stalled
   input logic       ifu_pmu_fetch_stall, // fetch unit stalled
   input logic       ifu_pmu_ic_miss, // icache miss
   input logic       ifu_pmu_ic_hit, // icache hit
   input logic       ifu_pmu_bus_error, // Instruction side bus error
   input logic       ifu_pmu_bus_busy, // Instruction side bus busy
   input logic       ifu_pmu_bus_trxn, // Instruction side bus transaction
   input logic [1:0] dec_pmu_instr_decoded, // decoded instructions
   input logic       dec_pmu_decode_stall, // decode stall
   input logic       dec_pmu_presync_stall, // decode stall due to presync'd inst
   input logic       dec_pmu_postsync_stall,// decode stall due to postsync'd inst
   input logic       lsu_store_stall_any,    // SB or WB is full, stall decode
   input logic       dma_dccm_stall_any,     // DMA stall of lsu
   input logic       dma_iccm_stall_any,     // DMA stall of ifu
   input logic       exu_pmu_i0_br_misp,     // pipe 0 branch misp
   input logic       exu_pmu_i0_br_ataken,   // pipe 0 branch actual taken
   input logic       exu_pmu_i0_pc4,         // pipe 0 4 byte branch
   input logic       exu_pmu_i1_br_misp,     // pipe 1 branch misp
   input logic       exu_pmu_i1_br_ataken,   // pipe 1 branch actual taken
   input logic       exu_pmu_i1_pc4,         // pipe 1 4 byte branch
   input logic       lsu_pmu_bus_trxn,       // D side bus transaction
   input logic       lsu_pmu_bus_misaligned, // D side bus misaligned
   input logic       lsu_pmu_bus_error,      // D side bus error
   input logic       lsu_pmu_bus_busy,       // D side bus busy
   input logic       lsu_pmu_load_external_dc3, // D side bus load
   input logic       lsu_pmu_store_external_dc3, // D side bus store
   input logic       dma_pmu_dccm_read,          // DMA DCCM read
   input logic       dma_pmu_dccm_write,         // DMA DCCM write
   input logic       dma_pmu_any_read,           // DMA read
   input logic       dma_pmu_any_write,          // DMA write

   input logic dec_tlu_br0_error_e4,
   input logic dec_tlu_br0_start_error_e4,
   input logic dec_tlu_br1_error_e4,
   input logic dec_tlu_br1_start_error_e4,

   input logic [31:1] lsu_fir_addr, // Fast int address
   input logic [1:0]  lsu_fir_error, // Fast int lookup error

   input logic mice_ce_req,
   input logic miccme_ce_req,
   input logic mdccme_ce_req,

   input logic [5:0] mfdht, // halt timeout threshold

   input    eh2_lsu_error_pkt_t lsu_error_pkt_dc3, // lsu precise exception/error packet

   input logic dec_pause_state, // Pause counter not zero
   input logic         lsu_imprecise_error_store_any,      // store bus error
   input logic         lsu_imprecise_error_load_any,      // store bus error
   input logic [31:0]  lsu_imprecise_error_addr_any, // store bus error address

   input logic        dec_i0_csr_wen_unq_d,       // valid csr with write - for csr legal
   input logic        dec_i0_csr_any_unq_d,       // valid csr - for csr legal
   input logic        dec_i0_csr_wen_wb,      // csr write enable at wb
   input logic [11:0] dec_i0_csr_rdaddr_d,      // read address for csr
   input logic [11:0] dec_i0_csr_wraddr_wb,      // write address for csr
   input logic [31:0] dec_i0_csr_wrdata_wb,   // csr write data at wb
   input logic        dec_i0_csr_is_mcpc_e4,     // csr address is to MCPC

   input logic        dec_csr_stall_int_ff, // csr is mie/mstatus

   input eh2_csr_tlu_pkt_t tlu_i0_csr_pkt_d, // csr decodes for i0

   input logic dec_tlu_i0_valid_e4, // pipe 0 op at e4 is valid
   input logic dec_tlu_i1_valid_e4, // pipe 1 op at e4 is valid

   input logic [31:1] exu_npc_e4, // for NPC tracking
   input logic exu_i0_flush_lower_e4,       // pipe 0 branch mp flush
   input logic exu_i1_flush_lower_e4,       // pipe 1 branch mp flush
   input logic [31:1] exu_i0_flush_path_e4, // pipe 0 correct path for mp, merge with lower path
   input logic [31:1] exu_i1_flush_path_e4, // pipe 1 correct path for mp, merge with lower path

   input logic [31:1] dec_tlu_i0_pc_e4, // for PC/NPC tracking
   input logic [31:1] dec_tlu_i1_pc_e4, // for PC/NPC tracking

   input eh2_trap_pkt_t dec_tlu_packet_e4, // exceptions known at decode

   input logic [31:0] dec_illegal_inst, // For mtval
   input logic        dec_i0_decode_d,  // decode valid, used for clean icache diagnostics

   // branch info from pipe0 for errors or counter updates
   input logic        exu_i0_br_error_e4, // error
   input logic        exu_i0_br_start_error_e4, // start error
   input logic        exu_i0_br_mp_e4, // mispredict
   // branch info from pipe1 for errors or counter updates
   input logic        exu_i1_br_error_e4, // error
   input logic        exu_i1_br_start_error_e4, // start error

   input  logic dbg_halt_req, // DM requests a halt
   input  logic dbg_resume_req, // DM requests a resume
   input  logic ifu_miss_state_idle, // I-side miss buffer empty
   input  logic lsu_idle_any, // lsu is idle
   input  logic dec_div_active, // oop divide is active

   input logic  ifu_ic_error_start,     // IC single bit error
   input logic  ifu_iccm_rd_ecc_single_err, // ICCM single bit error

   input logic [70:0] ifu_ic_debug_rd_data, // diagnostic icache read data
   input logic ifu_ic_debug_rd_data_valid, // diagnostic icache read data valid

   input logic [7:0] pic_claimid, // pic claimid for csr
   input logic [3:0] pic_pl, // pic priv level for csr
   input logic       mhwakeup, // high priority external int, wakeup if halted

   input logic mexintpend, // external interrupt pending
   input logic timer_int, // timer interrupt pending
   input logic soft_int, // software interrupt pending

   input logic [31:4]     core_id, // Core ID

   input logic mpc_debug_halt_req, // Async halt request
   input logic mpc_debug_run_req, // Async run request
   input logic mpc_reset_run_req, // Run/halt after reset

   // Debug start
   output logic dec_dbg_cmd_done, // abstract command done
   output logic dec_dbg_cmd_fail, // abstract command failed
   output logic dec_tlu_flush_noredir_wb , // Tell fetch to idle on this flush
   output logic dec_tlu_dbg_halted, // Core is halted and ready for debug command
   output logic dec_tlu_debug_mode, // Core is in debug mode
   output logic dec_tlu_resume_ack, // Resume acknowledge
   output logic dec_tlu_debug_stall, // stall decode while waiting on core to empty
   output logic dec_tlu_flush_leak_one_wb, // single step
   output logic dec_tlu_flush_err_wb, // iside perr/ecc rfpc
   output logic tlu_mpc_halted_only, // Core is halted only due to MPC
   output eh2_trigger_pkt_t  [3:0] tlu_trigger_pkt_any, // trigger info for trigger blocks
   output eh2_cache_debug_pkt_t dec_tlu_ic_diag_pkt, // packet of DICAWICS, DICAD0/1, DICAGO info for icache diagnostics

   output logic tlu_fast_ext_int_ready, // ready for fast int arb

   output logic dec_tlu_force_halt, // halt has been forced
   // Debug end

   output logic dec_tlu_flush_extint, // fast ext int started
   output logic [31:2] dec_tlu_meihap, // meihap for fast int

   output logic o_cpu_halt_status, // PMU interface, halted
   output logic o_cpu_halt_ack, // halt req ack
   output logic o_cpu_run_ack, // run req ack
   output logic o_debug_mode_status, // Core to the PMU that core is in debug mode. When core is in debug mode, the PMU should refrain from sendng a halt or run request

   // external MPC halt/run interface
   output logic mpc_debug_halt_ack, // Halt ack
   output logic mpc_debug_run_ack, // Run ack
   output logic debug_brkpt_status, // debug breakpoint

   output logic [3:0] tlu_meicurpl, // to PIC
   output logic [3:0] tlu_meipt, // to PIC

   output logic [31:0] csr_rddata_d,      // csr read data at wb

   output logic tlu_i0_kill_writeb_wb,    // I0 is flushed, don't writeback any results to arch state
   output logic tlu_i1_kill_writeb_wb,    // I1 is flushed, don't writeback any results to arch state

   output logic dec_tlu_flush_lower_wb,       // commit has a flush (exception, int, mispredict at e4)
   output logic [31:1] dec_tlu_flush_path_wb, // flush pc
   output logic dec_tlu_fence_i_wb,           // flush is a fence_i rfnpc, flush icache

   output logic tlu_i0_commit_cmt,        // goes to IFU for commit 1 instruction in the FSM

   output logic tlu_wr_pause_wb,           // CSR write to pause reg is at WB.
   output logic tlu_flush_pause_wb,        // Flush is due to pause

   output logic tlu_lr_reset_wb, // Reset the reservation on certain events


   output logic tlu_i0_valid_wb1,  // pipe 0 valid
   output logic tlu_i1_valid_wb1,  // pipe 1 valid
   output logic tlu_i0_exc_valid_wb1, // pipe 0 exception valid
   output logic tlu_i1_exc_valid_wb1, // pipe 1 exception valid
   output logic tlu_int_valid_wb1, // pipe 2 int valid

   output logic [4:0] tlu_exc_cause_wb1, // exception or int cause
   output logic [31:0] tlu_mtval_wb1, // MTVAL value

   output logic [1:0] tlu_perfcnt0, // toggles when pipe0 perf counter 0 has an event inc
   output logic [1:0] tlu_perfcnt1, // toggles when pipe0 perf counter 1 has an event inc
   output logic [1:0] tlu_perfcnt2, // toggles when pipe0 perf counter 2 has an event inc
   output logic [1:0] tlu_perfcnt3, // toggles when pipe0 perf counter 3 has an event inc

   output logic ic_perr_wb,
   output logic iccm_sbecc_wb,
   output logic allow_dbg_halt_csr_write,
   output logic dec_i0_csr_wen_wb_mod // don't write csr if trigger kills it
   )
;

   logic         clk_override, e4e5_int_clk, nmi_lsu_load_type, nmi_lsu_store_type, nmi_int_detected_f, nmi_lsu_load_type_f,
                 nmi_lsu_store_type_f, dbg_cmd_done_ns, i_cpu_run_req_d1_raw, debug_mode_status,
                 i0_mp_e4, i1_mp_e4, sel_npc_e4, sel_npc_wb, ce_int, mtval_capture_lsu_wb, wr_mdeau_wb,
                 nmi_in_debug_mode, dpc_capture_npc, dpc_capture_pc, tdata_load, tdata_opcode, tdata_action, perfcnt_halted,
                 tlu_i0_valid_e4, tlu_i1_valid_e4;

   eh2_trap_pkt_t  tlu_packet_e4;
   eh2_lsu_error_pkt_t lsu_error_pkt_e4, lsu_error_pkt_dc4;
   eh2_csr_tlu_pkt_t csr_rd;

   logic reset_delayed, reset_detect, reset_detected, reset_allowed, reset_delayed_f;
   logic wr_mstatus_wb, wr_mtvec_wb, wr_mie_wb, wr_mcyclel_wb, wr_mcycleh_wb,
         wr_minstretl_wb, wr_minstreth_wb, wr_mscratch_wb, wr_mepc_wb, wr_mcause_wb, wr_mscause_wb, wr_mtval_wb,
         wr_meihap_wb, wr_meicurpl_wb, wr_meipt_wb, wr_dcsr_wb, wr_mfdhs_wb,
         wr_dpc_wb, wr_meicidpl_wb, wr_meivt_wb, wr_meicpct_wb,
         wr_mhpme3_wb, wr_mhpme4_wb, wr_mhpme5_wb, wr_mhpme6_wb;
   logic wr_mpmc_wb;
   logic [1:1] mpmc_b_ns, mpmc, mpmc_b;
   logic [1:0] mfdhs_ns, mfdhs;
   logic [31:0] force_halt_ctr, force_halt_ctr_f;
   logic        force_halt;
   logic set_mie_pmu_fw_halt;
   logic wr_mcountinhibit_wb;
   logic [6:0] mcountinhibit;
   logic wr_mtsel_wb, wr_mtdata1_t0_wb, wr_mtdata1_t1_wb, wr_mtdata1_t2_wb, wr_mtdata1_t3_wb, wr_mtdata2_t0_wb, wr_mtdata2_t1_wb, wr_mtdata2_t2_wb, wr_mtdata2_t3_wb;
   logic [31:0] mtdata2_t0, mtdata2_t1, mtdata2_t2, mtdata2_t3, mtdata2_tsel_out, mtdata1_tsel_out;
   logic [9:0]  mtdata1_t0_ns, mtdata1_t0, mtdata1_t1_ns, mtdata1_t1, mtdata1_t2_ns, mtdata1_t2, mtdata1_t3_ns, mtdata1_t3;
   logic [9:0] tdata_wrdata_wb;
   logic [1:0] mtsel_ns, mtsel;
   logic tlu_i0_kill_writeb_e4, tlu_i1_kill_writeb_e4;
   logic [1:0]  mstatus_ns, mstatus;
   logic mstatus_mie_ns;
   logic [30:0] mtvec_ns, mtvec;
   logic [15:2] dcsr_ns, dcsr;
   logic [5:0] mip_ns, mip;
   logic [5:0] mie_ns, mie;
   logic [31:0] mcyclel_ns, mcyclel;
   logic [31:0] mcycleh_ns, mcycleh;
   logic [31:0] minstretl_ns, minstretl;
   logic [31:0] minstreth_ns, minstreth;
   logic [31:0] mscratch;
   logic [31:0] mhpmc3, mhpmc3_ns, mhpmc4, mhpmc4_ns, mhpmc5, mhpmc5_ns, mhpmc6, mhpmc6_ns;
   logic [31:0] mhpmc3h, mhpmc3h_ns, mhpmc4h, mhpmc4h_ns, mhpmc5h, mhpmc5h_ns, mhpmc6h, mhpmc6h_ns;
   logic [9:0]  mhpme3, mhpme4, mhpme5, mhpme6;
   logic [9:2] meihap;
   logic [31:10] meivt;
   logic [3:0] meicurpl_ns, meicurpl;
   logic [3:0] meicidpl_ns, meicidpl;
   logic [3:0] meipt_ns, meipt;
   logic [31:0] mdseac;
   logic mdseac_locked_ns, mdseac_locked_f, mdseac_en, nmi_lsu_detected;
   logic        wr_dicawics_wb, wr_dicad0_wb, wr_dicad0h_wb, wr_dicad1_wb;
   logic [31:0] dicad0_ns, dicad0, dicad0h_ns, dicad0h, dicad1;
   logic [31:1] mepc_ns, mepc;
   logic [31:1] dpc_ns, dpc;
   logic [31:0] mcause_ns, mcause;
   logic [3:0] mscause_ns, mscause, mscause_type, ifu_mscause;
   logic [31:0] mtval_ns, mtval;
   logic       mret_wb;
   logic dec_pause_state_f, tlu_wr_pause_wb_f, pause_expired_e4, pause_expired_wb;
   logic       tlu_flush_lower_e4, tlu_flush_lower_wb;
   logic [31:1] tlu_flush_path_e4, tlu_flush_path_wb;
   logic i0_valid_wb, i1_valid_wb;
   logic [5:1] vectored_cause;
   logic [31:2] vectored_path;
   logic [31:1] interrupt_path;
   logic [16:0] dicawics_ns, dicawics;
   logic [6:0]  dicad1_ns, dicad1_raw;

   logic        ebreak_e4, ebreak_to_debug_mode_e4, ecall_e4, illegal_e4, illegal_e4_qual, mret_e4, inst_acc_e4, fence_i_e4,
                ic_perr_e4, iccm_sbecc_e4, ebreak_to_debug_mode_wb, kill_ebreak_count_wb, inst_acc_second_e4;
   logic        ebreak_wb, ecall_wb, illegal_wb,  illegal_raw_wb, inst_acc_wb, inst_acc_second_wb, fence_i_wb;
   logic ce_int_ready, ext_int_ready, timer_int_ready, soft_int_ready, int_timer0_int_ready, int_timer1_int_ready, mhwakeup_ready,
         take_ext_int, take_ce_int, take_timer_int, take_soft_int, take_int_timer0_int, take_int_timer1_int, take_nmi, take_nmi_wb, int_timer0_int_possible, int_timer1_int_possible;
   logic i0_exception_valid_e4, interrupt_valid, i0_exception_valid_wb, interrupt_valid_wb, exc_or_int_valid, exc_or_int_valid_wb;
   logic synchronous_flush_e4;
   logic [4:0] exc_cause_e4, exc_cause_wb;
   logic [1:0] lsu_fir_error_d1;
   logic        mcyclel_cout, mcyclel_cout_f;
   logic [31:0] mcyclel_inc;
   logic        mcycleh_cout_nc;
   logic [31:0] mcycleh_inc;
   logic        minstretl_cout, minstretl_cout_f, minstret_enable;
   logic [31:0] minstretl_inc, minstretl_read;
   logic        minstreth_cout_nc;
   logic [31:0] minstreth_inc, minstreth_read;
   logic [31:1] pc_e4, pc_wb, npc_e4, npc_wb;
   logic        mtval_capture_pc_wb, mtval_capture_inst_wb, mtval_clear_wb, mtval_capture_pc_plus2_wb;
   logic rfpc_i0_e4, rfpc_i1_e4;
   logic lsu_i0_rfnpc_dc4, lsu_i1_rfnpc_dc4;
   logic lsu_i0_exc_dc4, lsu_i1_exc_dc4, lsu_i0_exc_dc4_raw, lsu_i1_exc_dc4_raw, lsu_exc_ma_dc4, lsu_exc_acc_dc4, lsu_exc_st_dc4,
         lsu_exc_valid_e4, lsu_exc_valid_e4_raw, lsu_exc_valid_wb, lsu_i0_exc_wb,
         block_interrupts ;
   logic tlu_i1_commit_cmt;

   logic request_debug_mode_e4, request_debug_mode_wb, request_debug_mode_done, request_debug_mode_done_f;

    logic take_halt, halt_taken, halt_taken_f, internal_dbg_halt_mode, dbg_tlu_halted_f, take_reset,
         dbg_tlu_halted, core_empty, lsu_idle_any_f, ifu_miss_state_idle_f, resume_ack_ns,
         debug_halt_req_f, debug_resume_req_f, enter_debug_halt_req, dcsr_single_step_done, dcsr_single_step_done_f,
         debug_halt_req_d1, debug_halt_req_ns, dcsr_single_step_running, dcsr_single_step_running_f, internal_dbg_halt_timers;

 logic [3:0] i0_trigger_e4, i1_trigger_e4, trigger_action, trigger_enabled,
               i0_trigger_chain_masked_e4, i1_trigger_chain_masked_e4;
   logic [2:0] trigger_chain;
   logic       i0_trigger_hit_e4, i0_trigger_hit_raw_e4, i0_trigger_action_e4,
               trigger_hit_e4, trigger_hit_wb, i0_trigger_hit_wb,
               mepc_trigger_hit_sel_pc_e4,
               mepc_trigger_hit_sel_pc_wb;
   logic       i1_trigger_hit_e4, i1_trigger_hit_raw_e4, i1_trigger_action_e4;
   logic [3:0] update_hit_bit_e4, update_hit_bit_wb, i0_iside_trigger_has_pri_e4, i1_iside_trigger_has_pri_e4,
               i0_lsu_trigger_has_pri_e4, i1_lsu_trigger_has_pri_e4;
   logic cpu_halt_status, cpu_halt_ack, cpu_run_ack, ext_halt_pulse, i_cpu_halt_req_d1, i_cpu_run_req_d1;

   logic inst_acc_e4_raw, trigger_hit_dmode_e4, trigger_hit_dmode_wb, trigger_hit_for_dscr_cause_wb;
   logic i_cpu_halt_req_sync_qual, i_cpu_run_req_sync_qual, pmu_fw_halt_req_ns, pmu_fw_halt_req_f, int_timer_stalled,
         fw_halt_req, enter_pmu_fw_halt_req, pmu_fw_tlu_halted, pmu_fw_tlu_halted_f, internal_pmu_fw_halt_mode,
         internal_pmu_fw_halt_mode_f, int_timer0_int_hold, int_timer1_int_hold, int_timer0_int_hold_f, int_timer1_int_hold_f;
   logic nmi_int_delayed, nmi_int_detected;
   logic [3:0] trigger_execute, trigger_data, trigger_store;

   // internal timer, isolated for size reasons
   logic [31:0] dec_timer_rddata_d;
   logic  dec_timer_read_d;
   logic       dec_timer_t0_pulse, dec_timer_t1_pulse;
   logic dec_tlu_pmu_fw_halted;

   logic mpc_run_state_ns, debug_brkpt_status_ns, mpc_debug_halt_ack_ns, mpc_debug_run_ack_ns, dbg_halt_state_ns, dbg_run_state_ns,
         dbg_halt_state_f, mpc_debug_halt_req_sync_f, mpc_debug_run_req_sync_f, mpc_halt_state_f, mpc_halt_state_ns, mpc_run_state_f, debug_brkpt_status_f,
         mpc_debug_halt_ack_f, mpc_debug_run_ack_f, dbg_run_state_f, mpc_debug_halt_req_sync_pulse,
         mpc_debug_run_req_sync_pulse, debug_brkpt_valid, debug_halt_req, debug_resume_req, dec_tlu_mpc_halted_only_ns;
   logic take_ext_int_start, ext_int_freeze, take_ext_int_start_d1, take_ext_int_start_d2, ignore_ext_int_due_to_lsu_stall,
         take_ext_int_start_d3, take_ext_int_start_d4, take_ext_int_start_d5, take_ext_int_start_d6, ext_int_freeze_d1;
   logic mcause_sel_nmi_store, mcause_sel_nmi_load, mcause_sel_nmi_ext;
   logic [1:0] mcause_fir_error_type;
   logic dbg_halt_req_held_ns, dbg_halt_req_held, dbg_halt_req_final;
   logic i0tid_wb, iccm_repair_state_ns, iccm_repair_state_d1, iccm_repair_state_rfnpc;
   logic [3:0][1:0] mhpmc_inc_e4, mhpmc_inc_wb;
   logic [3:0][9:0] mhpme_vec;
   logic            mhpmc3_wr_en0, mhpmc3_wr_en1, mhpmc3_wr_en;
   logic            mhpmc4_wr_en0, mhpmc4_wr_en1, mhpmc4_wr_en;
   logic            mhpmc5_wr_en0, mhpmc5_wr_en1, mhpmc5_wr_en;
   logic            mhpmc6_wr_en0, mhpmc6_wr_en1, mhpmc6_wr_en;
   logic            mhpmc3h_wr_en0, mhpmc3h_wr_en;
   logic            mhpmc4h_wr_en0, mhpmc4h_wr_en;
   logic            mhpmc5h_wr_en0, mhpmc5h_wr_en;
   logic            mhpmc6h_wr_en0, mhpmc6h_wr_en;
   logic            tlu_commit_lsu_op_e4;
   logic [63:0]     mhpmc3_incr, mhpmc4_incr, mhpmc5_incr, mhpmc6_incr;
   logic trace_tclk;
   logic [9:0] event_saturate_wb;
   logic [3:0] perfcnt_during_sleep;

   logic icache_rd_valid, icache_wr_valid, icache_rd_valid_f, icache_wr_valid_f;

if (pt.ICACHE_ECC == 1) begin
   logic [3:0] dicad1_raw, dicad1_ns;
end
else begin
   logic [6:0] dicad1_raw, dicad1_ns;
end
   logic enter_debug_halt_req_le, dcsr_cause_upgradeable;
   logic [8:6] dcsr_cause;
   logic pc0_valid_e4, pc1_valid_e4;
   logic sel_exu_npc_e4, sel_flush_npc_e4, sel_i0_npc_e4;
   logic minstret_enable_f;
   logic i0_valid_no_ebreak_ecall_wb;
   logic mcyclel_cout_in;
   logic [3:0] lsu_error_mscause_wb;
   logic [1:0] icaf_type_wb;
   logic [31:0] lsu_error_pkt_addr_dc4, lsu_error_pkt_addr_wb;
   logic        iside_oop_rfpc;
   logic i0_problem_kills_i1_trigger;
   logic lsu_pmu_load_external_dc4, lsu_pmu_store_external_dc4;
   logic e4_valid, e5_valid, e4e5_valid, internal_dbg_halt_mode_f, internal_dbg_halt_mode_f2, internal_dbg_halt_mode_f3;
   logic lsu_e3_e4_clk, lsu_e4_e5_clk;
   logic csr_wr_clk;
   logic timer_int_sync, soft_int_sync, i_cpu_halt_req_sync, i_cpu_run_req_sync, mpc_debug_halt_req_sync, mpc_debug_run_req_sync, mpc_debug_halt_req_sync_raw;
   logic take_halt_f, ifu_ic_error_start_d1, ifu_iccm_rd_ecc_single_err_d1, sel_fir_addr, sel_hold_npc_e4, tlu_dcsr_ss;

     eh2_dec_timer_ctl #(.pt(pt)) int_timers(.*);
   // end of internal timers

   assign clk_override           = dec_tlu_dec_clk_override;
   // Async inputs to the core have to be sync'd to the core clock.
   rvsyncss #(6) syncro_ff(.*,
                           .clk(free_clk),
                           .din ({timer_int,      soft_int,      i_cpu_halt_req,      i_cpu_run_req,      mpc_debug_halt_req,          mpc_debug_run_req}),
                           .dout({timer_int_sync, soft_int_sync, i_cpu_halt_req_sync, i_cpu_run_req_sync, mpc_debug_halt_req_sync_raw, mpc_debug_run_req_sync}));


   always_comb begin
      tlu_packet_e4 = dec_tlu_packet_e4;
      lsu_error_pkt_e4 = lsu_error_pkt_dc4;
      tlu_i0_valid_e4 = dec_tlu_i0_valid_e4;
      tlu_i1_valid_e4 = dec_tlu_i1_valid_e4;

      if(dec_tlu_packet_e4.i0tid != mytid) begin
         tlu_packet_e4.i0legal = 'b0;
         tlu_packet_e4.i0icaf = 'b0;
         tlu_packet_e4.i0icaf_type = 'b0;
         tlu_packet_e4.i0icaf_f1 = 'b0;
         tlu_packet_e4.i0fence_i = 'b0;
         tlu_packet_e4.i0trigger = 'b0;
         tlu_packet_e4.pmu_i0_br_unpred = '0;
         tlu_packet_e4.pmu_i0_itype = NULL;
         tlu_packet_e4.pmu_divide = 'b0;
         tlu_i0_valid_e4 = 'b0;
      end
      if(dec_tlu_packet_e4.i1tid != mytid) begin
         tlu_packet_e4.i1trigger = 'b0;
         tlu_packet_e4.pmu_i1_br_unpred = '0;
         tlu_packet_e4.pmu_i1_itype = NULL;
         tlu_i1_valid_e4 = 'b0;
      end

      // lsu is in pipe0, and the tids match
      if( ( dec_tlu_packet_e4.lsu_pipe0 & (dec_tlu_packet_e4.i0tid == mytid)) |
          (~dec_tlu_packet_e4.lsu_pipe0 & (dec_tlu_packet_e4.i1tid == mytid)) ) begin

         tlu_packet_e4.pmu_lsu_misaligned = dec_tlu_packet_e4.pmu_lsu_misaligned;
         lsu_error_pkt_e4.exc_valid = lsu_error_pkt_dc4.exc_valid;
         lsu_error_pkt_e4.single_ecc_error = lsu_error_pkt_dc4.single_ecc_error;

      end
      else begin
         tlu_packet_e4.pmu_lsu_misaligned = 'b0;
         lsu_error_pkt_e4.exc_valid = 'b0;
         lsu_error_pkt_e4.single_ecc_error = 'b0;
      end

   end


   // for CSRs that have inpipe writes only

   rvoclkhdr csrwr_wb_cgc ( .en(dec_i0_csr_wen_wb_mod | clk_override), .l1clk(csr_wr_clk), .* );
   rvoclkhdr lsu_e3_e4_cgc ( .en(lsu_error_pkt_dc3.exc_valid | lsu_error_pkt_dc4.exc_valid | lsu_error_pkt_dc3.single_ecc_error |
                                lsu_error_pkt_dc4.single_ecc_error | clk_override), .l1clk(lsu_e3_e4_clk), .* );
   rvoclkhdr lsu_e4_e5_cgc ( .en(lsu_error_pkt_dc4.exc_valid | lsu_exc_valid_wb | clk_override), .l1clk(lsu_e4_e5_clk), .* );

   assign e4_valid = tlu_i0_valid_e4 | tlu_i1_valid_e4;
   assign e4e5_valid = e4_valid | e5_valid;
   rvoclkhdr e4e5_int_cgc ( .en(e4e5_valid | internal_dbg_halt_mode_f | i_cpu_run_req_d1 | interrupt_valid | interrupt_valid_wb |
                               reset_allowed | pause_expired_e4 | pause_expired_wb | iccm_sbecc_e4 | iccm_sbecc_wb | ic_perr_e4 |
                               ic_perr_wb |clk_override), .l1clk(e4e5_int_clk), .* );

   rvdff #(12)  freeff (.*,   .clk(free_clk),
                       .din ({internal_dbg_halt_mode_f2,internal_dbg_halt_mode_f, force_halt,
                              dec_tlu_packet_e4.i0tid, iccm_repair_state_ns, e4_valid, internal_dbg_halt_mode,
                              tlu_flush_lower_e4, tlu_i0_kill_writeb_e4, tlu_i1_kill_writeb_e4,
                              lsu_pmu_load_external_dc3, lsu_pmu_store_external_dc3}),
                       .dout({internal_dbg_halt_mode_f3, internal_dbg_halt_mode_f2, dec_tlu_force_halt,
                              i0tid_wb, iccm_repair_state_d1, e5_valid, internal_dbg_halt_mode_f,
                              tlu_flush_lower_wb, tlu_i0_kill_writeb_wb, tlu_i1_kill_writeb_wb,
                              lsu_pmu_load_external_dc4, lsu_pmu_store_external_dc4}));


   rvdff #(3) reset_ff (.*, .clk(free_clk), .din({1'b1, reset_detect, reset_delayed}), .dout({reset_detect, reset_detected, reset_delayed_f}));
   assign reset_delayed = (reset_detect ^ reset_detected) | (reset_delayed_f & ~dec_tlu_flush_lower_wb);
   assign reset_allowed = reset_delayed & mhartstart_csr;

   rvdff #(4) nmi_ff (.*, .clk(free_clk), .din({nmi_int_sync, nmi_int_detected, nmi_lsu_load_type, nmi_lsu_store_type}),
                                         .dout({nmi_int_delayed, nmi_int_detected_f, nmi_lsu_load_type_f, nmi_lsu_store_type_f}));

   // Filter subsequent bus errors after the first, until the lock on MDSEAC is cleared
   assign nmi_lsu_detected = ~mdseac_locked_f & (lsu_imprecise_error_load_any | lsu_imprecise_error_store_any);

   assign nmi_int_detected = (nmi_int_sync & ~nmi_int_delayed) | nmi_lsu_detected | (nmi_int_detected_f & ~take_nmi_wb) | (take_ext_int_start_d6 & |lsu_fir_error[1:0]);
   // if the first nmi is a lsu type, note it. If there's already an nmi pending, ignore
   assign nmi_lsu_load_type = (nmi_lsu_detected & lsu_imprecise_error_load_any & ~(nmi_int_detected_f & ~take_nmi_wb)) | (nmi_lsu_load_type_f & ~take_nmi_wb);
   assign nmi_lsu_store_type = (nmi_lsu_detected & lsu_imprecise_error_store_any & ~(nmi_int_detected_f & ~take_nmi_wb)) | (nmi_lsu_store_type_f & ~take_nmi_wb);

`define MSTATUS_MIE 0
`define MIP_MCEIP 5
`define MIP_MITIP0 4
`define MIP_MITIP1 3
`define MIP_MEIP 2
`define MIP_MTIP 1
`define MIP_MSIP 0

`define MIE_MCEIE 5
`define MIE_MITIE0 4
`define MIE_MITIE1 3
`define MIE_MEIE 2
`define MIE_MTIE 1
`define MIE_MSIE 0

`define DCSR_EBREAKM 15
`define DCSR_STEPIE 11
`define DCSR_STOPC 10
`define DCSR_STEP 2
   // ----------------------------------------------------------------------
   // MPC halt
   // - can interact with debugger halt and v-v

   // fast ints in progress have priority
   assign mpc_debug_halt_req_sync = mpc_debug_halt_req_sync_raw & ~ext_int_freeze_d1;

    rvdff #(10)  mpvhalt_ff (.*, .clk(free_clk),
                                 .din({mpc_debug_halt_req_sync, mpc_debug_run_req_sync & debug_mode_status,
                                       mpc_halt_state_ns, mpc_run_state_ns, debug_brkpt_status_ns,
                                       mpc_debug_halt_ack_ns, mpc_debug_run_ack_ns,
                                       dbg_halt_state_ns, dbg_run_state_ns,
                                       dec_tlu_mpc_halted_only_ns}),
                                .dout({mpc_debug_halt_req_sync_f, mpc_debug_run_req_sync_f,
                                       mpc_halt_state_f, mpc_run_state_f, debug_brkpt_status_f,
                                       mpc_debug_halt_ack_f, mpc_debug_run_ack_f,
                                       dbg_halt_state_f, dbg_run_state_f,
                                       tlu_mpc_halted_only}));

   // turn level sensitive requests into pulses
   assign mpc_debug_halt_req_sync_pulse = mpc_debug_halt_req_sync & ~mpc_debug_halt_req_sync_f;
   assign mpc_debug_run_req_sync_pulse = mpc_debug_run_req_sync & ~mpc_debug_run_req_sync_f;

   // states
   assign mpc_halt_state_ns = (mpc_halt_state_f | mpc_debug_halt_req_sync_pulse | (reset_allowed & ~mpc_reset_run_req)) & ~mpc_debug_run_req_sync;
   assign mpc_run_state_ns = (mpc_run_state_f | (mpc_debug_run_req_sync_pulse & ~mpc_debug_run_ack_f)) & (internal_dbg_halt_mode_f & ~dcsr_single_step_running_f);

   assign dbg_halt_state_ns = (dbg_halt_state_f | (dbg_halt_req_final | dcsr_single_step_done_f | trigger_hit_dmode_wb | ebreak_to_debug_mode_wb)) & ~dbg_resume_req;
   assign dbg_run_state_ns = (dbg_run_state_f | dbg_resume_req) & (internal_dbg_halt_mode_f & ~dcsr_single_step_running_f);

   // tell dbg we are only MPC halted
   assign dec_tlu_mpc_halted_only_ns = ~dbg_halt_state_f & mpc_halt_state_f;

   // this asserts from detection of bkpt until after we leave debug mode
   assign debug_brkpt_valid = ebreak_to_debug_mode_wb | trigger_hit_dmode_wb;
   assign debug_brkpt_status_ns = (debug_brkpt_valid | debug_brkpt_status_f) & (internal_dbg_halt_mode & ~dcsr_single_step_running_f);

   // acks back to interface
   assign mpc_debug_halt_ack_ns = mpc_halt_state_f & internal_dbg_halt_mode_f & mpc_debug_halt_req_sync & core_empty;
   assign mpc_debug_run_ack_ns = (mpc_debug_run_req_sync & ~dbg_halt_state_ns & ~mpc_debug_halt_req_sync) | (mpc_debug_run_ack_f & mpc_debug_run_req_sync) ;

   // Pins
   assign mpc_debug_halt_ack = mpc_debug_halt_ack_f;
   assign mpc_debug_run_ack = mpc_debug_run_ack_f;
   assign debug_brkpt_status = debug_brkpt_status_f;

   // DBG halt req is a pulse, fast ext int in progress has priority
   assign dbg_halt_req_held_ns = (dbg_halt_req | dbg_halt_req_held) & ext_int_freeze_d1;
   assign dbg_halt_req_final = (dbg_halt_req | dbg_halt_req_held) & ~ext_int_freeze_d1;

   // combine MPC and DBG halt requests
   assign debug_halt_req = (dbg_halt_req_final | mpc_debug_halt_req_sync | (reset_allowed & ~mpc_reset_run_req)) & ~internal_dbg_halt_mode_f & ~ext_int_freeze_d1;

   assign debug_resume_req = ~debug_resume_req_f &  // squash back to back resumes
                             ((mpc_run_state_ns & ~dbg_halt_state_ns) |  // MPC run req
                              (dbg_run_state_ns & ~mpc_halt_state_ns)); // dbg request is a pulse


   // HALT

   // dbg/pmu/fw requests halt, service as soon as lsu is not blocking interrupts
   assign take_halt = (debug_halt_req_f | pmu_fw_halt_req_f) & ~synchronous_flush_e4 & ~mret_e4 & ~halt_taken_f & ~dec_tlu_flush_noredir_wb & ~take_reset;

   // hold after we take a halt, so we don't keep taking halts
   assign halt_taken = (dec_tlu_flush_noredir_wb & ~tlu_flush_pause_wb & ~take_ext_int_start_d1) | (halt_taken_f & ~dbg_tlu_halted_f & ~pmu_fw_tlu_halted_f & ~interrupt_valid_wb);

   // After doing halt flush (RFNPC) wait until core is idle before asserting a particular halt mode
   // It takes a cycle for mb_empty to assert after a fetch, take_halt covers that cycle
   assign core_empty = force_halt |
                       (lsu_idle_any & lsu_idle_any_f & ifu_miss_state_idle & ifu_miss_state_idle_f & ~debug_halt_req & ~debug_halt_req_d1 & ~dec_div_active);

//--------------------------------------------------------------------------------
// Debug start
//

   assign enter_debug_halt_req = (~internal_dbg_halt_mode_f & debug_halt_req) | dcsr_single_step_done_f | trigger_hit_dmode_wb | ebreak_to_debug_mode_wb;

   // dbg halt state active from request until non-step resume
   assign internal_dbg_halt_mode = debug_halt_req_ns | (internal_dbg_halt_mode_f & ~(debug_resume_req_f & ~dcsr[`DCSR_STEP]));
   // dbg halt can access csrs as long as we are not stepping
   assign allow_dbg_halt_csr_write = internal_dbg_halt_mode_f & ~dcsr_single_step_running_f;


   // hold debug_halt_req_ns high until we enter debug halt
   assign debug_halt_req_ns = enter_debug_halt_req | (debug_halt_req_f & ~dbg_tlu_halted);

   assign dbg_tlu_halted = (debug_halt_req_f & core_empty & halt_taken) | (dbg_tlu_halted_f & ~debug_resume_req_f);

   assign resume_ack_ns = (debug_resume_req_f & dbg_tlu_halted_f & dbg_run_state_ns);

   assign dcsr_single_step_done = tlu_i0_valid_e4 & ~dec_tlu_dbg_halted & dcsr[`DCSR_STEP] & ~rfpc_i0_e4;

   assign dcsr_single_step_running = (debug_resume_req_f & dcsr[`DCSR_STEP]) | (dcsr_single_step_running_f & ~dcsr_single_step_done_f);

   assign dbg_cmd_done_ns = tlu_i0_valid_e4 & dec_tlu_dbg_halted;

   // used to hold off commits after an in-pipe debug mode request (triggers, DCSR)
   assign request_debug_mode_e4 = (trigger_hit_dmode_e4 | ebreak_to_debug_mode_e4) | (request_debug_mode_wb & ~dec_tlu_flush_lower_wb);

   assign request_debug_mode_done = (request_debug_mode_wb | request_debug_mode_done_f) & ~dbg_tlu_halted_f;

    rvdff #(22)  halt_ff (.*, .clk(free_clk), .din({halt_taken, take_halt, lsu_idle_any, ifu_miss_state_idle, dbg_tlu_halted,
                                  resume_ack_ns, dbg_cmd_done_ns, debug_halt_req_ns, debug_resume_req, trigger_hit_dmode_e4,
                                  dcsr_single_step_done, debug_halt_req,  update_hit_bit_e4[3:0], tlu_wr_pause_wb, dec_pause_state,
                                  request_debug_mode_e4, request_debug_mode_done, dcsr_single_step_running, dbg_halt_req_held_ns}),
                           .dout({halt_taken_f, take_halt_f, lsu_idle_any_f, ifu_miss_state_idle_f, dbg_tlu_halted_f,
                                  dec_tlu_resume_ack, dec_dbg_cmd_done, debug_halt_req_f, debug_resume_req_f, trigger_hit_dmode_wb,
                                  dcsr_single_step_done_f, debug_halt_req_d1, update_hit_bit_wb[3:0], tlu_wr_pause_wb_f, dec_pause_state_f,
                                  request_debug_mode_wb, request_debug_mode_done_f, dcsr_single_step_running_f, dbg_halt_req_held}));

   assign dec_tlu_debug_stall = debug_halt_req_f;
   assign dec_tlu_dbg_halted = dbg_tlu_halted_f;
   assign dec_tlu_debug_mode = internal_dbg_halt_mode_f;
   assign dec_tlu_pmu_fw_halted = pmu_fw_tlu_halted_f;

   // kill fetch redirection on flush if going to halt, or if there's a fence during db-halt
   assign dec_tlu_flush_noredir_wb = take_halt_f | (fence_i_wb & internal_dbg_halt_mode_f) | tlu_flush_pause_wb | (trigger_hit_wb & trigger_hit_dmode_wb) | take_ext_int_start_d1;
   assign dec_tlu_flush_extint = take_ext_int_start_d1;

   // 1 cycle after writing the PAUSE counter, flush with noredir to idle F1-D.
   assign tlu_flush_pause_wb = tlu_wr_pause_wb_f & ~interrupt_valid_wb;

   // detect end of pause counter and rfpc
   assign pause_expired_e4 = ~dec_pause_state & dec_pause_state_f &
                             ~(ext_int_ready | ce_int_ready | timer_int_ready | soft_int_ready | int_timer0_int_hold_f | int_timer1_int_hold_f | nmi_int_detected | ext_int_freeze_d1) &
                             ~interrupt_valid_wb & ~debug_halt_req_f & ~pmu_fw_halt_req_f & ~halt_taken_f;

   assign dec_tlu_flush_leak_one_wb = dec_tlu_flush_lower_wb & ~dec_tlu_flush_noredir_wb & ( (dcsr[`DCSR_STEP] & (dec_tlu_resume_ack | dcsr_single_step_running)) |
                                                                                             iccm_sbecc_wb);
   assign dec_tlu_flush_err_wb = dec_tlu_flush_lower_wb & (ic_perr_wb | iccm_sbecc_wb);

   // If DM attempts to access an illegal CSR, send cmd_fail back
   assign dec_dbg_cmd_fail = illegal_raw_wb & dec_dbg_cmd_done;


   //--------------------------------------------------------------------------------
   //--------------------------------------------------------------------------------
   // Triggers
   //
`define MTDATA1_DMODE 9
`define MTDATA1_SEL 7
`define MTDATA1_ACTION 6
`define MTDATA1_CHAIN 5
`define MTDATA1_MATCH 4
`define MTDATA1_M_ENABLED 3
`define MTDATA1_EXE 2
`define MTDATA1_ST 1
`define MTDATA1_LD 0

   // Prioritize trigger hits with other exceptions.
   //
   // Trigger should have highest priority except:
   // - trigger is an execute-data and there is an inst_access exception (lsu triggers won't fire, inst. is nop'd by decode)
   // - trigger is a store-data and there is a lsu_acc_exc or lsu_ma_exc.
   assign trigger_execute[3:0] = {mtdata1_t3[`MTDATA1_EXE], mtdata1_t2[`MTDATA1_EXE], mtdata1_t1[`MTDATA1_EXE], mtdata1_t0[`MTDATA1_EXE]};
   assign trigger_data[3:0] = {mtdata1_t3[`MTDATA1_SEL], mtdata1_t2[`MTDATA1_SEL], mtdata1_t1[`MTDATA1_SEL], mtdata1_t0[`MTDATA1_SEL]};
   assign trigger_store[3:0] = {mtdata1_t3[`MTDATA1_ST], mtdata1_t2[`MTDATA1_ST], mtdata1_t1[`MTDATA1_ST], mtdata1_t0[`MTDATA1_ST]};

   // MSTATUS[MIE] needs to be on to take triggers unless the action is trigger to debug mode.
   assign trigger_enabled[3:0] = {(mtdata1_t3[`MTDATA1_ACTION] | mstatus[`MSTATUS_MIE]) & mtdata1_t3[`MTDATA1_M_ENABLED],
                                  (mtdata1_t2[`MTDATA1_ACTION] | mstatus[`MSTATUS_MIE]) & mtdata1_t2[`MTDATA1_M_ENABLED],
                                  (mtdata1_t1[`MTDATA1_ACTION] | mstatus[`MSTATUS_MIE]) & mtdata1_t1[`MTDATA1_M_ENABLED],
                                  (mtdata1_t0[`MTDATA1_ACTION] | mstatus[`MSTATUS_MIE]) & mtdata1_t0[`MTDATA1_M_ENABLED]};

   // iside exceptions are always in i0
   assign i0_iside_trigger_has_pri_e4[3:0] = ~( (trigger_execute[3:0] & trigger_data[3:0] & {4{inst_acc_e4_raw}}) | // exe-data with inst_acc
                                                ({4{exu_i0_br_error_e4 | exu_i0_br_start_error_e4 | ic_perr_e4 | iccm_sbecc_e4}}));              // branch error in i0

   assign i1_iside_trigger_has_pri_e4[3:0] = ~( ({4{exu_i1_br_error_e4 | exu_i1_br_start_error_e4 | ic_perr_e4 | iccm_sbecc_e4}}) ); // branch error in i1

   // lsu excs have to line up with their respective triggers since the lsu op can be in either i0 or i1 but not both
   assign i0_lsu_trigger_has_pri_e4[3:0] = ~(trigger_store[3:0] & trigger_data[3:0] & {4{lsu_i0_exc_dc4_raw}});
   assign i1_lsu_trigger_has_pri_e4[3:0] = ~(trigger_store[3:0] & trigger_data[3:0] & {4{lsu_i1_exc_dc4_raw}});

   assign i0_trigger_e4[3:0] = {4{tlu_i0_valid_e4}} & tlu_packet_e4.i0trigger[3:0] & i0_iside_trigger_has_pri_e4[3:0] & i0_lsu_trigger_has_pri_e4[3:0] & trigger_enabled[3:0];
   assign i1_trigger_e4[3:0] = {4{tlu_i1_valid_e4}} & tlu_packet_e4.i1trigger[3:0] & i1_iside_trigger_has_pri_e4[3:0] & i1_lsu_trigger_has_pri_e4[3:0] & trigger_enabled[3:0];


   assign trigger_chain[2:0] = {mtdata1_t2[`MTDATA1_CHAIN], mtdata1_t1[`MTDATA1_CHAIN], mtdata1_t0[`MTDATA1_CHAIN]};

   // chaining can mask raw trigger info
   assign i0_trigger_chain_masked_e4[3:0] = {i0_trigger_e4[3] & (~trigger_chain[2] | i0_trigger_e4[2]),
                                             i0_trigger_e4[2] & (~trigger_chain[2] | i0_trigger_e4[3]),
                                             i0_trigger_e4[1] & (~trigger_chain[0] | i0_trigger_e4[0]),
                                             i0_trigger_e4[0] & (~trigger_chain[0] | i0_trigger_e4[1])};

   assign i1_trigger_chain_masked_e4[3:0] = {i1_trigger_e4[3] & (~trigger_chain[2] | i1_trigger_e4[2]),
                                             i1_trigger_e4[2] & (~trigger_chain[2] | i1_trigger_e4[3]),
                                             i1_trigger_e4[1] & (~trigger_chain[0] | i1_trigger_e4[0]),
                                             i1_trigger_e4[0] & (~trigger_chain[0] | i1_trigger_e4[1])};

   // This is the highest priority by this point.
   assign i0_trigger_hit_raw_e4 = |i0_trigger_chain_masked_e4[3:0];
   assign i1_trigger_hit_raw_e4 = |i1_trigger_chain_masked_e4[3:0];

   assign i0_problem_kills_i1_trigger = (~tlu_i0_commit_cmt | exu_i0_br_mp_e4 | lsu_i0_rfnpc_dc4) & tlu_i0_valid_e4;
   // Qual trigger hits
   assign i0_trigger_hit_e4 = ~(dec_tlu_flush_lower_wb | dec_tlu_dbg_halted) & i0_trigger_hit_raw_e4;
   assign i1_trigger_hit_e4 = ~(dec_tlu_flush_lower_wb | dec_tlu_dbg_halted | i0_problem_kills_i1_trigger) & i1_trigger_hit_raw_e4;

   // Actions include breakpoint, or dmode. Dmode is only possible if the DMODE bit is set.
   // Otherwise, take a breakpoint.
   assign trigger_action[3:0] = {mtdata1_t3[`MTDATA1_ACTION] & mtdata1_t3[`MTDATA1_DMODE],
                                 mtdata1_t2[`MTDATA1_ACTION] & mtdata1_t2[`MTDATA1_DMODE],
                                 mtdata1_t1[`MTDATA1_ACTION] & mtdata1_t1[`MTDATA1_DMODE],
                                 mtdata1_t0[`MTDATA1_ACTION] & mtdata1_t0[`MTDATA1_DMODE]};

   // this is needed to set the HIT bit in the triggers
   assign update_hit_bit_e4[3:0] = ({4{i0_trigger_hit_e4                     }} & i0_trigger_chain_masked_e4[3:0]) |
                                   ({4{i1_trigger_hit_e4 & ~i0_trigger_hit_e4}} & i1_trigger_chain_masked_e4[3:0]);

   // action, 1 means dmode. Simultaneous triggers with at least 1 set for dmode force entire action to dmode.
   assign i0_trigger_action_e4 = |(i0_trigger_chain_masked_e4[3:0] & trigger_action[3:0]);
   assign i1_trigger_action_e4 = |(i1_trigger_chain_masked_e4[3:0] & trigger_action[3:0]);

   assign trigger_hit_e4 = i0_trigger_hit_e4 | i1_trigger_hit_e4;
   assign trigger_hit_dmode_e4 = (i0_trigger_hit_e4 & i0_trigger_action_e4) | (i1_trigger_hit_e4 & ~i0_trigger_hit_e4 & i1_trigger_action_e4);

   assign mepc_trigger_hit_sel_pc_e4 = trigger_hit_e4 & ~trigger_hit_dmode_e4;


//
// Debug end
//--------------------------------------------------------------------------------

   //----------------------------------------------------------------------
   //
   // Commit
   //
   //----------------------------------------------------------------------



   //--------------------------------------------------------------------------------
   // External halt (not debug halt)
   // - Fully interlocked handshake
   // i_cpu_halt_req  ____|--------------|_______________
   // core_empty      ---------------|___________
   // o_cpu_halt_ack  _________________|----|__________
   // o_cpu_halt_status _______________|---------------------|_________
   // i_cpu_run_req                              ______|----------|____
   // o_cpu_run_ack                              ____________|------|________
   //


   // debug mode has priority, ignore PMU/FW halt/run while in debug mode
   assign i_cpu_halt_req_sync_qual = i_cpu_halt_req_sync & ~dec_tlu_debug_mode & ~ext_int_freeze_d1;
   assign i_cpu_run_req_sync_qual = i_cpu_run_req_sync & ~dec_tlu_debug_mode & pmu_fw_tlu_halted_f & ~ext_int_freeze_d1;

   rvdff #(10) exthaltff (.*, .clk(free_clk), .din({i_cpu_halt_req_sync_qual, i_cpu_run_req_sync_qual,   cpu_halt_status,
                                                   cpu_halt_ack,   cpu_run_ack, internal_pmu_fw_halt_mode,
                                                   pmu_fw_halt_req_ns, pmu_fw_tlu_halted,
                                                   int_timer0_int_hold, int_timer1_int_hold}),
                                            .dout({i_cpu_halt_req_d1,        i_cpu_run_req_d1_raw,      o_cpu_halt_status,
                                                   o_cpu_halt_ack, o_cpu_run_ack, internal_pmu_fw_halt_mode_f,
                                                   pmu_fw_halt_req_f, pmu_fw_tlu_halted_f,
                                                   int_timer0_int_hold_f, int_timer1_int_hold_f}));

   // only happens if we aren't in dgb_halt
   assign ext_halt_pulse = i_cpu_halt_req_sync_qual & ~i_cpu_halt_req_d1;

   assign enter_pmu_fw_halt_req =  ext_halt_pulse | fw_halt_req;

   assign pmu_fw_halt_req_ns = (enter_pmu_fw_halt_req | (pmu_fw_halt_req_f & ~pmu_fw_tlu_halted)) & ~debug_halt_req_f;

   assign internal_pmu_fw_halt_mode = pmu_fw_halt_req_ns | (internal_pmu_fw_halt_mode_f & ~i_cpu_run_req_d1 & ~debug_halt_req_f);

   // debug halt has priority
   assign pmu_fw_tlu_halted = ((pmu_fw_halt_req_f & core_empty & halt_taken) | (pmu_fw_tlu_halted_f & ~i_cpu_run_req_d1)) & ~debug_halt_req_f;

   assign cpu_halt_ack = i_cpu_halt_req_d1 & pmu_fw_tlu_halted_f;
   assign cpu_halt_status = ((pmu_fw_tlu_halted_f & ~i_cpu_run_req_d1) | (o_cpu_halt_status & ~i_cpu_run_req_d1)) & ~internal_dbg_halt_mode_f;
   assign cpu_run_ack = (o_cpu_halt_status & i_cpu_run_req_sync_qual) | (o_cpu_run_ack & i_cpu_run_req_sync_qual);
   assign debug_mode_status = internal_dbg_halt_mode_f;
   assign o_debug_mode_status = debug_mode_status;

`ifdef ASSERT_ON
  assert_commit_while_halted: assert #0 (~((tlu_i0_commit_cmt | tlu_i1_commit_cmt) & o_cpu_halt_status)) else $display("ERROR: Commiting while cpu_halt_status asserted!");
  assert_flush_while_fastint: assert #0 (~((take_ext_int_start_d1 | take_ext_int_start_d2| take_ext_int_start_d3| take_ext_int_start_d4| take_ext_int_start_d5) & tlu_flush_lower_e4)) else $display("ERROR: TLU Flushing inside fast interrupt procedure!");
`endif

   // high priority interrupts can wakeup from external halt, so can unmasked timer interrupts
   assign i_cpu_run_req_d1 = i_cpu_run_req_d1_raw | ((nmi_int_detected | timer_int_ready | soft_int_ready | int_timer0_int_hold_f | int_timer1_int_hold_f | (mhwakeup & mhwakeup_ready)) & o_cpu_halt_status & ~i_cpu_halt_req_d1);

   //--------------------------------------------------------------------------------
   //--------------------------------------------------------------------------------

   // LSU exceptions (LSU responsible for prioritizing simultaneous cases)

   rvdff #( $bits(eh2_lsu_error_pkt_t) ) lsu_error_dc4ff (.*, .clk(lsu_e3_e4_clk), .din(lsu_error_pkt_dc3),  .dout(lsu_error_pkt_dc4));

   rvdff #(3) lsu_dccm_errorff (.*, .clk(free_clk), .din({ifu_ic_error_start, ifu_iccm_rd_ecc_single_err,
                                                          mdseac_locked_ns}),
                                                   .dout({ifu_ic_error_start_d1, ifu_iccm_rd_ecc_single_err_d1,
                                                          mdseac_locked_f}));

   assign lsu_error_pkt_addr_dc4[31:0] = lsu_error_pkt_e4.addr[31:0];
   rvdff #(38) lsu_error_wbff (.*, .clk(lsu_e4_e5_clk), .din({lsu_error_pkt_addr_dc4[31:0], lsu_exc_valid_e4, lsu_i0_exc_dc4, lsu_error_pkt_e4.mscause[3:0]}),
                                                       .dout({lsu_error_pkt_addr_wb[31:0], lsu_exc_valid_wb, lsu_i0_exc_wb, lsu_error_mscause_wb[3:0]}));


   // lsu exception is valid unless it's in pipe1 and there was a rfpc_i0_e4, brmp, or an iside exception in pipe0.
   assign lsu_exc_valid_e4_raw = lsu_error_pkt_e4.exc_valid & ~(~tlu_packet_e4.lsu_pipe0 & (rfpc_i0_e4 | i0_exception_valid_e4 | exu_i0_br_mp_e4)) & ~dec_tlu_flush_lower_wb;

   assign lsu_i0_exc_dc4_raw =  lsu_error_pkt_e4.exc_valid & tlu_packet_e4.lsu_pipe0;
   assign lsu_i1_exc_dc4_raw = lsu_error_pkt_e4.exc_valid &  ~tlu_packet_e4.lsu_pipe0;
   assign lsu_i0_exc_dc4 = lsu_i0_exc_dc4_raw & lsu_exc_valid_e4_raw & ~i0_trigger_hit_e4 & ~iside_oop_rfpc;
   assign lsu_i1_exc_dc4 = lsu_i1_exc_dc4_raw & lsu_exc_valid_e4_raw & ~trigger_hit_e4 & ~iside_oop_rfpc;
   assign lsu_exc_valid_e4 = lsu_i0_exc_dc4 | lsu_i1_exc_dc4;

   assign lsu_exc_ma_dc4 = (lsu_i0_exc_dc4 | lsu_i1_exc_dc4) & ~lsu_error_pkt_e4.exc_type;
   assign lsu_exc_acc_dc4 = (lsu_i0_exc_dc4 | lsu_i1_exc_dc4) & lsu_error_pkt_e4.exc_type;
   assign lsu_exc_st_dc4 = (lsu_i0_exc_dc4 | lsu_i1_exc_dc4) & lsu_error_pkt_e4.inst_type;


   // Single bit ECC errors on loads are RFNPC corrected, with the corrected data written to the GPR.
   // LSU turns the load into a store and patches the data in the DCCM
   assign lsu_i0_rfnpc_dc4 = tlu_i0_valid_e4 & tlu_packet_e4.lsu_pipe0 & (~lsu_error_pkt_e4.inst_type | lsu_error_pkt_e4.amo_valid) &
                             lsu_error_pkt_e4.single_ecc_error & ~i0_trigger_hit_e4;
   assign lsu_i1_rfnpc_dc4 = tlu_i1_valid_e4 &  ~tlu_packet_e4.lsu_pipe0 & (~lsu_error_pkt_e4.inst_type | lsu_error_pkt_e4.amo_valid) &
                             lsu_error_pkt_e4.single_ecc_error & ~i0_trigger_hit_e4 & ~i1_trigger_hit_e4;



   //  Final commit valids
   assign tlu_i0_commit_cmt = tlu_i0_valid_e4 &
                              ~rfpc_i0_e4 &
                              ~lsu_i0_exc_dc4 &
                              ~inst_acc_e4 &
                              ~dec_tlu_dbg_halted &
                              ~request_debug_mode_wb &
                              ~i0_trigger_hit_e4;

   assign tlu_i1_commit_cmt = tlu_i1_valid_e4 &
                              ~rfpc_i0_e4 & ~rfpc_i1_e4 &
                              ~exu_i0_br_mp_e4 &
                              ~lsu_i0_exc_dc4  & ~lsu_i1_exc_dc4 &
                              ~lsu_i0_rfnpc_dc4 &
                              ~inst_acc_e4 &
                              ~request_debug_mode_wb &
                              ~i0_trigger_hit_e4 & ~i1_trigger_hit_e4;

   // unified place to manage the killing of arch state writebacks
   assign tlu_i0_kill_writeb_e4 = (rfpc_i0_e4 | lsu_i0_exc_dc4 | inst_acc_e4 | (illegal_e4 & dec_tlu_dbg_halted) | i0_trigger_hit_e4) & tlu_i0_valid_e4;
   assign tlu_i1_kill_writeb_e4 = (rfpc_i1_e4 | lsu_i1_exc_dc4 | i1_trigger_hit_e4 | ((ic_perr_e4 | iccm_sbecc_e4) & ~ext_int_freeze_d1) |
                                   ((rfpc_i0_e4 | exu_i0_br_mp_e4 | i0_trigger_hit_e4 |
                                     lsu_i0_rfnpc_dc4 | lsu_i0_exc_dc4 | inst_acc_e4 |
                                     (illegal_e4 & dec_tlu_dbg_halted)) & tlu_i0_valid_e4)) & tlu_i1_valid_e4;

   // refetch PC, microarch flush
   // ic errors only in pipe0
   assign rfpc_i0_e4 = ((tlu_i0_valid_e4 & ~tlu_flush_lower_wb & (exu_i0_br_error_e4 | exu_i0_br_start_error_e4)) |
                       ((ic_perr_e4 | iccm_sbecc_e4) & ~ext_int_freeze_d1)) &
                       ~i0_trigger_hit_e4 &
                       ~lsu_i0_rfnpc_dc4;
   assign rfpc_i1_e4 = tlu_i1_valid_e4 & ~tlu_flush_lower_wb & ~i0_exception_valid_e4 & ~exu_i0_br_mp_e4 & ~lsu_i0_exc_dc4 & ~lsu_i0_rfnpc_dc4 &
                       ~(exu_i0_br_error_e4 | exu_i0_br_start_error_e4 | ic_perr_e4 | iccm_sbecc_e4) &
                       (exu_i1_br_error_e4 | exu_i1_br_start_error_e4) &
                       ~trigger_hit_e4;

   // From the indication of a iccm single bit error until the first commit or flush, maintain a repair state. In the repair state, rfnpc i0 commits.
   assign iccm_repair_state_ns = iccm_sbecc_wb | (iccm_repair_state_d1 & ~dec_tlu_flush_lower_wb);

   // this is a flush of last resort, meaning only assert it if there is no other flush happening.
   assign iccm_repair_state_rfnpc = ((tlu_i0_commit_cmt &
                                    ~(ebreak_e4 | ecall_e4 | mret_e4 | take_reset | illegal_e4 | dec_i0_csr_is_mcpc_e4)) | tlu_i1_commit_cmt) & iccm_repair_state_d1;

   assign iside_oop_rfpc = (ifu_ic_error_start_d1 | ifu_iccm_rd_ecc_single_err_d1) & (~internal_dbg_halt_mode_f | dcsr_single_step_running) & ~internal_pmu_fw_halt_mode_f;

   // only expect these in pipe 0
   assign       ebreak_e4    =  (tlu_packet_e4.pmu_i0_itype == EBREAK)  & tlu_i0_valid_e4 & ~i0_trigger_hit_e4 & ~dcsr[`DCSR_EBREAKM] & ~iside_oop_rfpc;
   assign       ecall_e4     =  (tlu_packet_e4.pmu_i0_itype == ECALL)   & tlu_i0_valid_e4 & ~i0_trigger_hit_e4 & ~iside_oop_rfpc;
   assign       illegal_e4   =  ~tlu_packet_e4.i0legal   & tlu_i0_valid_e4 & ~i0_trigger_hit_e4 & ~iside_oop_rfpc;
   assign       mret_e4      =  (tlu_packet_e4.pmu_i0_itype == MRET)    & tlu_i0_valid_e4 & ~i0_trigger_hit_e4 & ~iside_oop_rfpc;
   // fence_i includes debug only fence_i's
   assign       fence_i_e4   =  tlu_packet_e4.i0fence_i & tlu_i0_valid_e4 & ~i0_trigger_hit_e4 & ~iside_oop_rfpc;
   assign       ic_perr_e4    =  ifu_ic_error_start_d1 & ~ext_int_freeze_d1 & ~dec_tlu_flush_lower_wb & (~internal_dbg_halt_mode_f | dcsr_single_step_running) & ~internal_pmu_fw_halt_mode_f;
   assign       iccm_sbecc_e4 =  ifu_iccm_rd_ecc_single_err_d1 & ~ext_int_freeze_d1 & ~dec_tlu_flush_lower_wb & (~internal_dbg_halt_mode_f | dcsr_single_step_running) & ~internal_pmu_fw_halt_mode_f;
   assign       inst_acc_e4_raw  =  tlu_packet_e4.i0icaf & tlu_i0_valid_e4;
   assign       inst_acc_e4 = inst_acc_e4_raw & ~rfpc_i0_e4 & ~i0_trigger_hit_e4;
   assign       inst_acc_second_e4 = tlu_packet_e4.i0icaf_f1;

   assign       ebreak_to_debug_mode_e4 = (tlu_packet_e4.pmu_i0_itype == EBREAK)  & tlu_i0_valid_e4 & ~i0_trigger_hit_e4 & dcsr[`DCSR_EBREAKM] & ~iside_oop_rfpc;

   assign illegal_e4_qual = illegal_e4 & ~dec_tlu_dbg_halted;

   rvdff #(11)  exctype_wb_ff (.*, .clk(e4e5_int_clk),
                                .din({ebreak_e4, ebreak_to_debug_mode_e4, illegal_e4,  ecall_e4,
                                      illegal_e4_qual,  inst_acc_e4, inst_acc_second_e4, fence_i_e4, mret_e4,
                                      tlu_packet_e4.i0icaf_type[1:0]}),
                               .dout({ebreak_wb, ebreak_to_debug_mode_wb, illegal_raw_wb, ecall_wb,
                                      illegal_wb,       inst_acc_wb, inst_acc_second_wb, fence_i_wb, mret_wb,
                                      icaf_type_wb[1:0]}));

   assign dec_tlu_fence_i_wb = fence_i_wb;

   // Reset the reservation in LSU for mret, exceptions, ints, halts, resumes.
   assign tlu_lr_reset_wb = mret_wb | exc_or_int_valid_wb | take_halt_f | debug_resume_req_f | (i_cpu_run_req_d1 & pmu_fw_tlu_halted_f);

   //
   // Exceptions
   //
   // - MEPC <- PC
   // - PC <- MTVEC, assert flush_lower
   // - MCAUSE <- cause
   // - MSCAUSE <- secondary cause
   // - MTVAL <-
   // - MPIE <- MIE
   // - MIE <- 0
   //
   assign i0_exception_valid_e4 = (ebreak_e4 | ecall_e4 | illegal_e4 | inst_acc_e4) & ~rfpc_i0_e4 & ~dec_tlu_dbg_halted;

   // Cause:
   //
   // 0x2 : illegal
   // 0x3 : breakpoint
   // 0xb : Environment call M-mode


   assign exc_cause_e4[4:0] = ( ({5{take_ext_int}}        & 5'h0b) |
                                ({5{take_timer_int}}      & 5'h07) |
                                ({5{take_soft_int}}       & 5'h03) |
                                ({5{take_int_timer0_int}} & 5'h1d) |
                                ({5{take_int_timer1_int}} & 5'h1c) |
                                ({5{take_ce_int}}         & 5'h1e) |
                                ({5{illegal_e4}}          & 5'h02) |
                                ({5{ecall_e4}}            & 5'h0b) |
                                ({5{inst_acc_e4}}         & 5'h01) |
                                ({5{ebreak_e4 | trigger_hit_e4}}        & 5'h03) |
                                ({5{lsu_exc_ma_dc4 & ~lsu_exc_st_dc4}}  & 5'h04) |
                                ({5{lsu_exc_acc_dc4 & ~lsu_exc_st_dc4}} & 5'h05) |
                                ({5{lsu_exc_ma_dc4 & lsu_exc_st_dc4}}   & 5'h06) |
                                ({5{lsu_exc_acc_dc4 & lsu_exc_st_dc4}}  & 5'h07)
                                ) & ~{5{take_nmi}};

   //
   // Interrupts
   //
   // For above purposes, exceptions that are committed have already happened and will cause an int at E4 to wait a cycle
   // or more if MSTATUS[MIE] is cleared.
   //
   // -in priority order, highest to lowest
   // -single cycle window where a csr write to MIE/MSTATUS is at E4 when the other conditions for externals are met.
   //  Hold off externals for a cycle to make sure we are consistent with what was just written
   assign mhwakeup_ready =  ~dec_csr_stall_int_ff & mstatus_mie_ns & mip[`MIP_MEIP]   & mie_ns[`MIE_MEIE];
   assign ext_int_ready   = ~dec_csr_stall_int_ff & mstatus_mie_ns & mip[`MIP_MEIP]   & mie_ns[`MIE_MEIE] & ~ignore_ext_int_due_to_lsu_stall;
   assign ce_int_ready    = ~dec_csr_stall_int_ff & mstatus_mie_ns & mip[`MIP_MCEIP]  & mie_ns[`MIE_MCEIE];
   assign soft_int_ready  = ~dec_csr_stall_int_ff & mstatus_mie_ns & mip[`MIP_MSIP]   & mie_ns[`MIE_MSIE];
   assign timer_int_ready = ~dec_csr_stall_int_ff & mstatus_mie_ns & mip[`MIP_MTIP]   & mie_ns[`MIE_MTIE];

   // MIP for internal timers pulses for 1 clock, resets the timer counter. Mip won't hold past the various stall conditions.
   assign int_timer0_int_possible = mstatus_mie_ns & mie_ns[`MIE_MITIE0];
   assign int_timer0_int_ready = mip[`MIP_MITIP0] & int_timer0_int_possible;
   assign int_timer1_int_possible = mstatus_mie_ns & mie_ns[`MIE_MITIE1];
   assign int_timer1_int_ready = mip[`MIP_MITIP1] & int_timer1_int_possible;

   // Internal timers pulse and reset. If core is PMU/FW halted, the pulse will cause an exit from halt, but won't stick around
   // Make it sticky, also for 1 cycle stall conditions.
   assign int_timer_stalled = dec_csr_stall_int_ff | synchronous_flush_e4 | exc_or_int_valid_wb | mret_wb | mret_e4;

   assign int_timer0_int_hold = (int_timer0_int_ready & (pmu_fw_tlu_halted_f | int_timer_stalled)) | (int_timer0_int_possible & int_timer0_int_hold_f & ~interrupt_valid & ~take_ext_int_start & ~internal_dbg_halt_mode_f);
   assign int_timer1_int_hold = (int_timer1_int_ready & (pmu_fw_tlu_halted_f | int_timer_stalled)) | (int_timer1_int_possible & int_timer1_int_hold_f & ~interrupt_valid & ~take_ext_int_start & ~internal_dbg_halt_mode_f);


   // mispredicts
   assign i0_mp_e4 = exu_i0_flush_lower_e4 & ~(exu_i0_br_error_e4 | exu_i0_br_start_error_e4 | ic_perr_e4 | iccm_sbecc_e4) & ~i0_trigger_hit_e4;
   assign i1_mp_e4 = exu_i1_flush_lower_e4 & ~(exu_i1_br_error_e4 | exu_i1_br_start_error_e4 | ic_perr_e4 | iccm_sbecc_e4) & ~trigger_hit_e4 & ~lsu_i0_rfnpc_dc4;

   assign internal_dbg_halt_timers = ~mhartstart_csr | (internal_dbg_halt_mode_f & ~dcsr_single_step_running);

   // Prioritize externals
   assign block_interrupts = ( (internal_dbg_halt_mode & (~dcsr_single_step_running | tlu_i0_valid_e4)) | // No ints in db-halt unless we are single stepping
                               internal_pmu_fw_halt_mode | i_cpu_halt_req_d1 |// No ints in PMU/FW halt. First we exit halt
                               take_nmi | // NMI is top priority
                               ebreak_to_debug_mode_e4 | // Heading to debug mode, hold off ints
                               synchronous_flush_e4 | // exception flush this cycle
                               exc_or_int_valid_wb | // ext/int past cycle (need time for MIE to update)
                               mret_wb | // mret (need time for MIE to update)
                               mret_e4 | // mret in progress, for cases were ISR enables ints before mret
                               ext_int_freeze_d1 // fast interrupt in progress
                               );

if (pt.FAST_INTERRUPT_REDIRECT) begin

      rvdff #(9)  fastint_ff (.*, .clk(free_clk),
                                .din({take_ext_int_start,    take_ext_int_start_d1, take_ext_int_start_d2, take_ext_int_start_d3,
                                      take_ext_int_start_d4, take_ext_int_start_d5, ext_int_freeze, lsu_fir_error[1:0]}),
                               .dout({take_ext_int_start_d1, take_ext_int_start_d2, take_ext_int_start_d3, take_ext_int_start_d4,
                                      take_ext_int_start_d5, take_ext_int_start_d6, ext_int_freeze_d1, lsu_fir_error_d1[1:0]}));

   assign tlu_fast_ext_int_ready = ext_int_ready & ~block_interrupts;
   assign take_ext_int_start = tlu_fast_ext_int_ready & (tlu_select_tid == mytid);

   assign ext_int_freeze = take_ext_int_start | take_ext_int_start_d1 | take_ext_int_start_d2 | take_ext_int_start_d3 |
                            take_ext_int_start_d4 | take_ext_int_start_d5 | take_ext_int_start_d6;
   assign take_ext_int = take_ext_int_start_d6 & ~|lsu_fir_error[1:0];

   assign ignore_ext_int_due_to_lsu_stall = lsu_fastint_stall_any;

end
else begin
   assign take_ext_int_start = 1'b0;
   assign ext_int_freeze = 1'b0;
   assign ext_int_freeze_d1 = 1'b0;
   assign take_ext_int_start_d1 = 1'b0;
   assign take_ext_int_start_d2 = 1'b0;
   assign take_ext_int_start_d3 = 1'b0;
   assign take_ext_int_start_d4 = 1'b0;
   assign take_ext_int_start_d5 = 1'b0;
   assign take_ext_int_start_d6 = 1'b0;
   assign lsu_fir_error_d1[1:0] = 2'b0;
   assign ignore_ext_int_due_to_lsu_stall = 1'b0;
   assign tlu_fast_ext_int_ready = 1'b0;

   assign take_ext_int = ext_int_ready & ~block_interrupts;
end



   assign take_ce_int  = ce_int_ready & ~ext_int_ready & ~block_interrupts;
   assign take_soft_int = soft_int_ready & ~ext_int_ready & ~ce_int_ready & ~block_interrupts;
   assign take_timer_int = timer_int_ready & ~soft_int_ready & ~ext_int_ready & ~ce_int_ready & ~block_interrupts;
   assign take_int_timer0_int = (int_timer0_int_ready | int_timer0_int_hold_f) & int_timer0_int_possible &
                                ~dec_csr_stall_int_ff & ~timer_int_ready & ~soft_int_ready & ~ext_int_ready & ~ce_int_ready & ~block_interrupts;
   assign take_int_timer1_int = (int_timer1_int_ready | int_timer1_int_hold_f) & int_timer1_int_possible &
                                ~dec_csr_stall_int_ff & ~(int_timer0_int_ready | int_timer0_int_hold_f) & ~timer_int_ready &
                                ~soft_int_ready & ~ext_int_ready & ~ce_int_ready & ~block_interrupts;

   assign take_reset = reset_allowed & mpc_reset_run_req;
   assign take_nmi = nmi_int_detected & ~internal_pmu_fw_halt_mode &
                     (~internal_dbg_halt_mode | (dcsr_single_step_running_f & dcsr[`DCSR_STEPIE] & ~tlu_i0_valid_e4 & ~dcsr_single_step_done_f)) &
                     ~synchronous_flush_e4 & ~mret_e4 & ~take_reset & ~ebreak_to_debug_mode_e4 & (~ext_int_freeze_d1 | (take_ext_int_start_d6 & |lsu_fir_error[1:0]));

   assign interrupt_valid = take_ext_int | take_timer_int | take_soft_int | take_nmi | take_ce_int | take_int_timer0_int | take_int_timer1_int;


   // Compute interrupt path:
   // If vectored async is set in mtvec, flush path for interrupts is MTVEC + (4 * CAUSE);
   assign vectored_cause[5:1]  = exc_cause_e4[4:0];
   assign vectored_path[31:2]  = mtvec[30:1] + {25'b0, vectored_cause[5:1]};
   assign interrupt_path[31:1] = take_nmi ? nmi_vec[31:1] : ((mtvec[0] == 1'b1) ? {vectored_path[31:2], 1'b0} : {mtvec[30:1], 1'b0});

   assign sel_npc_e4 = lsu_i0_rfnpc_dc4 | (lsu_i1_rfnpc_dc4 & tlu_i1_commit_cmt) | fence_i_e4 | iccm_repair_state_rfnpc | (i_cpu_run_req_d1 & ~interrupt_valid) | (rfpc_i0_e4 & ~tlu_i0_valid_e4);
   assign sel_npc_wb = (i_cpu_run_req_d1 & pmu_fw_tlu_halted_f) | pause_expired_e4;

   assign sel_fir_addr = take_ext_int_start_d6 & ~|lsu_fir_error[1:0];

   assign synchronous_flush_e4 = i0_exception_valid_e4 | // exception
                                 i0_mp_e4 | i1_mp_e4 |  // mispredict
                                 rfpc_i0_e4 | rfpc_i1_e4 | // rfpc
                                 lsu_exc_valid_e4 |  // lsu exception in either pipe 0 or pipe 1
                                 fence_i_e4 |  // fence, a rfnpc
                                 lsu_i0_rfnpc_dc4 | lsu_i1_rfnpc_dc4 |
                                 iccm_repair_state_rfnpc | // Iccm sb ecc
                                 debug_resume_req_f | // resume from debug halt, fetch the dpc
                                 sel_npc_wb |  // resume from pmu/fw halt, or from pause and fetch the NPC
                                 tlu_wr_pause_wb | // flush at start of pause
                                 trigger_hit_e4; // trigger hit, ebreak or goto debug mode

   assign tlu_flush_lower_e4 = interrupt_valid | mret_e4 | synchronous_flush_e4 | take_halt | take_reset | take_ext_int_start;

   assign tlu_flush_path_e4[31:1] = take_reset ? rst_vec[31:1] :

                                     (({31{sel_fir_addr}} & lsu_fir_addr[31:1]) |
                                      ({31{~take_nmi & i0_mp_e4}} & exu_i0_flush_path_e4[31:1]) |
                                      ({31{~take_nmi & ~i0_mp_e4 & i1_mp_e4 & ~rfpc_i0_e4 & ~lsu_i0_exc_dc4}} & exu_i1_flush_path_e4[31:1]) |
                                      ({31{~take_nmi & sel_npc_e4}} & npc_e4[31:1]) |
                                      ({31{~take_nmi & rfpc_i0_e4 & tlu_i0_valid_e4 & ~sel_npc_e4}} & dec_tlu_i0_pc_e4[31:1]) |
                                      ({31{~take_nmi & rfpc_i1_e4}} & dec_tlu_i1_pc_e4[31:1]) |
                                      ({31{interrupt_valid & ~sel_fir_addr}} & interrupt_path[31:1]) |
                                      ({31{(i0_exception_valid_e4 | lsu_exc_valid_e4 |
                                            (trigger_hit_e4 & ~trigger_hit_dmode_e4)) & ~interrupt_valid & ~sel_fir_addr}} & {mtvec[30:1],1'b0}) |
                                      ({31{~take_nmi & mret_e4 & ~wr_mepc_wb}} & mepc[31:1]) |
                                      ({31{~take_nmi & debug_resume_req_f}} & dpc[31:1]) |
                                      ({31{~take_nmi & sel_npc_wb}} & npc_wb[31:1]) |
                                      ({31{~take_nmi & mret_e4 & wr_mepc_wb}} & dec_i0_csr_wrdata_wb[31:1]) );

   rvdff #(31)  flush_lower_ff (.*, .clk(e4e5_int_clk),
                                  .din({tlu_flush_path_e4[31:1]}),
                                 .dout({tlu_flush_path_wb[31:1]}));

   assign dec_tlu_flush_lower_wb = tlu_flush_lower_wb ;
   assign dec_tlu_flush_path_wb[31:1] = tlu_flush_path_wb[31:1];


   // this is used to capture mepc, etc.
   assign exc_or_int_valid = lsu_exc_valid_e4 | i0_exception_valid_e4 | interrupt_valid | (trigger_hit_e4 & ~trigger_hit_dmode_e4);

   rvdff #(17)  excinfo_wb_ff (.*, .clk(e4e5_int_clk),
                                .din({ic_perr_e4, iccm_sbecc_e4, interrupt_valid, i0_exception_valid_e4, exc_or_int_valid,
                                      exc_cause_e4[4:0], tlu_i0_commit_cmt & ~illegal_e4, tlu_i1_commit_cmt,
                                       mepc_trigger_hit_sel_pc_e4, trigger_hit_e4, i0_trigger_hit_e4,
                                      take_nmi, pause_expired_e4 }),
                               .dout({ic_perr_wb, iccm_sbecc_wb, interrupt_valid_wb, i0_exception_valid_wb, exc_or_int_valid_wb,
                                      exc_cause_wb[4:0], i0_valid_wb, i1_valid_wb,
                                       mepc_trigger_hit_sel_pc_wb, trigger_hit_wb, i0_trigger_hit_wb,
                                      take_nmi_wb, pause_expired_wb}));

   //----------------------------------------------------------------------
   //
   // CSRs
   //
   //----------------------------------------------------------------------


   // ----------------------------------------------------------------------
   // MISA (RO)
   //  [31:30] XLEN - implementation width, 2'b01 - 32 bits
   //  [12]    M    - integer mul/div
   //  [8]     I    - RV32I
   //  [2]     C    - Compressed extension
   //  [0]     A    - Atomic extension
   `define MISA 12'h301

   // MVENDORID, MARCHID, MIMPID, MHARTID
   `define MVENDORID 12'hf11
   `define MARCHID 12'hf12
   `define MIMPID 12'hf13
   `define MHARTID 12'hf14


   // ----------------------------------------------------------------------
   // MSTATUS (RW)
   // [12:11] MPP  : Prior priv level, always 2'b11, not flopped
   // [7]     MPIE : Int enable previous [1]
   // [3]     MIE  : Int enable          [0]
   `define MSTATUS 12'h300


   //When executing a MRET instruction, supposing MPP holds the value 3, MIE
   //is set to MPIE; the privilege mode is changed to 3; MPIE is set to 1; and MPP is set to 3
   assign dec_i0_csr_wen_wb_mod = dec_i0_csr_wen_wb & ~trigger_hit_wb & (mytid == i0tid_wb);
   assign wr_mstatus_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MSTATUS);

    // set this even if we don't go to fwhalt due to debug halt. We committed the inst, so ...
   assign set_mie_pmu_fw_halt = ~mpmc_b_ns[1] & wr_mpmc_wb & dec_i0_csr_wrdata_wb[0] & ~internal_dbg_halt_mode_f3;

   assign mstatus_ns[1:0] = ( ({2{~wr_mstatus_wb & exc_or_int_valid_wb}} & {(mstatus[`MSTATUS_MIE] | set_mie_pmu_fw_halt), 1'b0}) |
                              ({2{ wr_mstatus_wb & exc_or_int_valid_wb}} & {dec_i0_csr_wrdata_wb[3], 1'b0}) |
                              ({2{mret_wb & ~exc_or_int_valid_wb}} & {1'b1, mstatus[1]}) |
                              ({2{set_mie_pmu_fw_halt & ~exc_or_int_valid_wb}} & {mstatus[1], 1'b1}) |
                              ({2{wr_mstatus_wb & ~exc_or_int_valid_wb}} & {dec_i0_csr_wrdata_wb[7], dec_i0_csr_wrdata_wb[3]}) |
                              ({2{~wr_mstatus_wb & ~exc_or_int_valid_wb & ~mret_wb & ~set_mie_pmu_fw_halt}} & mstatus[1:0]) );

   // gate MIE if we are single stepping and DCSR[STEPIE] is off
   assign mstatus_mie_ns = mstatus_ns[`MSTATUS_MIE] & (~dcsr_single_step_running_f | dcsr[`DCSR_STEPIE]);
   rvdff #(2)  mstatus_ff (.*, .clk(free_clk), .din(mstatus_ns[1:0]), .dout(mstatus[1:0]));

   // ----------------------------------------------------------------------
   // MTVEC (RW)
   // [31:2] BASE : Trap vector base address
   // [1] - Reserved, not implemented, reads zero
   // [0]  MODE : 0 = Direct, 1 = Asyncs are vectored to BASE + (4 * CAUSE)
   `define MTVEC 12'h305

   assign wr_mtvec_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MTVEC);
   assign mtvec_ns[30:0] = {dec_i0_csr_wrdata_wb[31:2], dec_i0_csr_wrdata_wb[0]} ;
   rvdffe #(31)  mtvec_ff (.*, .en(wr_mtvec_wb), .din(mtvec_ns[30:0]), .dout(mtvec[30:0]));

   // ----------------------------------------------------------------------
   // MIP (RW)
   //
   // [30] MCEIP  : (RO) M-Mode Correctable Error interrupt pending
   // [29] MITIP0 : (RO) M-Mode Internal Timer0 interrupt pending
   // [28] MITIP1 : (RO) M-Mode Internal Timer1 interrupt pending
   // [11] MEIP   : (RO) M-Mode external interrupt pending
   // [7]  MTIP   : (RO) M-Mode timer interrupt pending
   // [3]  MSIP   : (RO) M-Mode software interrupt pending
   `define MIP 12'h344

   assign ce_int = (mdccme_ce_req | miccme_ce_req | mice_ce_req);

   assign mip_ns[5:0] = {ce_int, dec_timer_t0_pulse, dec_timer_t1_pulse, mexintpend, timer_int_sync, soft_int_sync};
   rvdff #(6)  mip_ff (.*, .clk(free_clk), .din(mip_ns[5:0]), .dout(mip[5:0]));

   // ----------------------------------------------------------------------
   // MIE (RW)
   // [30] MCEIE  : (RO) M-Mode Correctable Error interrupt enable
   // [29] MITIE0 : (RO) M-Mode Internal Timer0 interrupt enable
   // [28] MITIE1 : (RO) M-Mode Internal Timer1 interrupt enable
   // [11] MEIE   : (RW) M-Mode external interrupt enable
   // [7]  MTIE   : (RW) M-Mode timer interrupt enable
   // [3]  MSIE   : (RW) M-Mode software interrupt enable
   `define MIE 12'h304

   assign wr_mie_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MIE);
   assign mie_ns[5:0] = wr_mie_wb ? {dec_i0_csr_wrdata_wb[30:28], dec_i0_csr_wrdata_wb[11], dec_i0_csr_wrdata_wb[7], dec_i0_csr_wrdata_wb[3]} : mie[5:0];
   rvdff #(6)  mie_ff (.*, .clk(csr_wr_clk), .din(mie_ns[5:0]), .dout(mie[5:0]));


   // ----------------------------------------------------------------------
   // MCYCLEL (RW)
   // [31:0] : Lower Cycle count

   `define MCYCLEL 12'hb00

   assign kill_ebreak_count_wb = ebreak_to_debug_mode_wb & dcsr[`DCSR_STOPC];

   assign wr_mcyclel_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MCYCLEL);

   assign mcyclel_cout_in = ~(kill_ebreak_count_wb | (dec_tlu_dbg_halted & dcsr[`DCSR_STOPC]) |
                              dec_tlu_pmu_fw_halted | mcountinhibit[0] | ~mhartstart_csr);

   assign {mcyclel_cout, mcyclel_inc[31:0]} = mcyclel[31:0] + {31'b0, mcyclel_cout_in};
   assign mcyclel_ns[31:0] = wr_mcyclel_wb ? dec_i0_csr_wrdata_wb[31:0] : mcyclel_inc[31:0];

   rvdffe #(32) mcyclel_ff      (.*, .en(wr_mcyclel_wb | mcyclel_cout_in), .din(mcyclel_ns[31:0]), .dout(mcyclel[31:0]));
   rvdff   #(1) mcyclef_cout_ff (.*, .clk(free_clk), .din(mcyclel_cout & ~wr_mcycleh_wb), .dout(mcyclel_cout_f));
   // ----------------------------------------------------------------------
   // MCYCLEH (RW)
   // [63:32] : Higher Cycle count
   // Chained with mcyclel. Note: mcyclel overflow due to a mcycleh write gets ignored.

   `define MCYCLEH 12'hb80

   assign wr_mcycleh_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MCYCLEH);

   assign {mcycleh_cout_nc, mcycleh_inc[31:0]} = mcycleh[31:0] + {31'b0, mcyclel_cout_f};
   assign mcycleh_ns[31:0] = wr_mcycleh_wb ? dec_i0_csr_wrdata_wb[31:0] : mcycleh_inc[31:0];

   rvdffe #(32)  mcycleh_ff (.*, .en(wr_mcycleh_wb | mcyclel_cout_f), .din(mcycleh_ns[31:0]), .dout(mcycleh[31:0]));

   // ----------------------------------------------------------------------
   // MINSTRETL (RW)
   // [31:0] : Lower Instruction retired count
   // From the spec "Some CSRs, such as the instructions retired counter, instret, may be modified as side effects
   // of instruction execution. In these cases, if a CSR access instruction reads a CSR, it reads the
   // value prior to the execution of the instruction. If a CSR access instruction writes a CSR, the
   // update occurs after the execution of the instruction. In particular, a value written to instret by
   // one instruction will be the value read by the following instruction (i.e., the increment of instret
   // caused by the first instruction retiring happens before the write of the new value)."
   `define MINSTRETL 12'hb02

   assign i0_valid_no_ebreak_ecall_wb = i0_valid_wb & ~(ebreak_wb | ecall_wb | ebreak_to_debug_mode_wb) & ~mcountinhibit[2];

   assign wr_minstretl_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MINSTRETL);

   assign {minstretl_cout, minstretl_inc[31:0]} = minstretl[31:0] + {31'b0,i0_valid_no_ebreak_ecall_wb} + {31'b0,i1_valid_wb & ~mcountinhibit[2]};

   assign minstret_enable = (i0_valid_no_ebreak_ecall_wb | i1_valid_wb);

   assign minstretl_ns[31:0] = wr_minstretl_wb ? dec_i0_csr_wrdata_wb[31:0] : minstretl_inc[31:0];
   rvdffe #(32)  minstretl_ff (.*, .en(minstret_enable | wr_minstretl_wb), .din(minstretl_ns[31:0]), .dout(minstretl[31:0]));
   rvdff #(2) minstretf_cout_ff (.*, .clk(free_clk), .din({minstret_enable, minstretl_cout & ~wr_minstreth_wb}), .dout({minstret_enable_f, minstretl_cout_f}));

   assign minstretl_read[31:0] = minstretl[31:0];
   // ----------------------------------------------------------------------
   // MINSTRETH (RW)
   // [63:32] : Higher Instret count
   // Chained with minstretl. Note: minstretl overflow due to a minstreth write gets ignored.

   `define MINSTRETH 12'hb82

   assign wr_minstreth_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MINSTRETH);

   assign {minstreth_cout_nc, minstreth_inc[31:0]} = minstreth[31:0] + {31'b0, minstretl_cout_f};
   assign minstreth_ns[31:0] = wr_minstreth_wb ? dec_i0_csr_wrdata_wb[31:0] : minstreth_inc[31:0];
   rvdffe #(32)  minstreth_ff (.*, .en(minstret_enable_f | wr_minstreth_wb), .din(minstreth_ns[31:0]), .dout(minstreth[31:0]));

   assign minstreth_read[31:0] = minstreth_inc[31:0];

   // ----------------------------------------------------------------------
   // MSCRATCH (RW)
   // [31:0] : Scratch register
   `define MSCRATCH 12'h340

   assign wr_mscratch_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MSCRATCH);

   rvdffe #(32)  mscratch_ff (.*, .en(wr_mscratch_wb), .din(dec_i0_csr_wrdata_wb[31:0]), .dout(mscratch[31:0]));

   // ----------------------------------------------------------------------
   // MEPC (RW)
   // [31:1] : Exception PC
   `define MEPC 12'h341

   // NPC

   assign sel_exu_npc_e4 = ~dec_tlu_dbg_halted & ~tlu_flush_lower_wb & (tlu_i0_valid_e4 | tlu_i1_valid_e4) & ~(tlu_i1_valid_e4 & (lsu_i0_rfnpc_dc4 | iccm_sbecc_e4 | ic_perr_e4));
   assign sel_i0_npc_e4 = ~dec_tlu_dbg_halted & ~tlu_flush_lower_wb & tlu_i0_valid_e4 & lsu_i0_rfnpc_dc4 & tlu_i1_valid_e4;
   assign sel_flush_npc_e4 = ~dec_tlu_dbg_halted & tlu_flush_lower_wb & ~dec_tlu_flush_noredir_wb;
   assign sel_hold_npc_e4 = ~sel_exu_npc_e4 & ~sel_flush_npc_e4 & ~sel_i0_npc_e4;


   assign npc_e4[31:1] = ( ({31{sel_exu_npc_e4}} & exu_npc_e4[31:1]) |
                           ({31{sel_i0_npc_e4}} & dec_tlu_i1_pc_e4[31:1]) |
                           ({31{~mpc_reset_run_req & reset_allowed}} & rst_vec[31:1]) | // init to reset vector for mpc halt on reset case
                           ({31{(sel_flush_npc_e4)}} & tlu_flush_path_wb[31:1]) |
                           ({31{(sel_hold_npc_e4)}} & npc_wb[31:1]) );

   rvdffe #(31)  npwbc_ff (.*, .en(sel_exu_npc_e4 | sel_flush_npc_e4 | reset_allowed), .din(npc_e4[31:1]), .dout(npc_wb[31:1]));

   // PC has to be captured for exceptions and interrupts. For MRET, we could execute it and then take an
   // interrupt before the next instruction.
   assign pc0_valid_e4 = ~dec_tlu_dbg_halted & tlu_i0_valid_e4;
   assign pc1_valid_e4 = ~dec_tlu_dbg_halted & tlu_i1_valid_e4 & ~lsu_i0_exc_dc4 & ~rfpc_i0_e4 & ~inst_acc_e4 & ~i0_trigger_hit_e4;

   assign pc_e4[31:1] = ( ({31{ pc0_valid_e4 & ~pc1_valid_e4}} & dec_tlu_i0_pc_e4[31:1]) |
                          ({31{ pc1_valid_e4}} & dec_tlu_i1_pc_e4[31:1]) |
                          ({31{~pc0_valid_e4 & ~pc1_valid_e4}} & pc_wb[31:1]));

   rvdffe #(31)  pwbc_ff (.*, .en(pc0_valid_e4 | pc1_valid_e4), .din(pc_e4[31:1]), .dout(pc_wb[31:1]));

   assign wr_mepc_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MEPC);

   assign mepc_ns[31:1] = ( ({31{i0_exception_valid_wb | lsu_exc_valid_wb | mepc_trigger_hit_sel_pc_wb}} & pc_wb[31:1]) |
                            ({31{interrupt_valid_wb}} & npc_wb[31:1]) |
                            ({31{wr_mepc_wb & ~exc_or_int_valid_wb}} & dec_i0_csr_wrdata_wb[31:1]) |
                            ({31{~wr_mepc_wb & ~exc_or_int_valid_wb}} & mepc[31:1]) );


   rvdff #(31)  mepc_ff (.*, .clk(e4e5_int_clk), .din(mepc_ns[31:1]), .dout(mepc[31:1]));

   // ----------------------------------------------------------------------
   // MCAUSE (RW)
   // [31:0] : Exception Cause
   `define MCAUSE 12'h342

   assign wr_mcause_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MCAUSE);
   assign mcause_sel_nmi_store = exc_or_int_valid_wb & take_nmi_wb & nmi_lsu_store_type_f;
   assign mcause_sel_nmi_load = exc_or_int_valid_wb & take_nmi_wb & nmi_lsu_load_type_f;
   assign mcause_sel_nmi_ext = exc_or_int_valid_wb & take_nmi_wb & |lsu_fir_error_d1[1:0];


   assign mcause_fir_error_type[1:0] = {&lsu_fir_error_d1[1:0], lsu_fir_error_d1[1] & ~lsu_fir_error_d1[0]};

   assign mcause_ns[31:0] = ( ({32{mcause_sel_nmi_store}} & {32'hf000_0000}) |
                              ({32{mcause_sel_nmi_load}} & {32'hf000_0001}) |
                              ({32{mcause_sel_nmi_ext}} & {28'hf000_100, 2'b0, mcause_fir_error_type[1:0]}) |
                              ({32{exc_or_int_valid_wb & ~take_nmi_wb}} & {interrupt_valid_wb, 26'b0, exc_cause_wb[4:0]}) |
                              ({32{wr_mcause_wb & ~exc_or_int_valid_wb}} & dec_i0_csr_wrdata_wb[31:0]) |
                              ({32{~wr_mcause_wb & ~exc_or_int_valid_wb}} & mcause[31:0]) );

   rvdff #(32)  mcause_ff (.*, .clk(e4e5_int_clk), .din(mcause_ns[31:0]), .dout(mcause[31:0]));
   // ----------------------------------------------------------------------
   // MSCAUSE (RW)
   // [2:0] : Secondary exception Cause
   `define MSCAUSE 12'h7ff

   assign wr_mscause_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MSCAUSE);
   assign ifu_mscause[3:0]  =  (icaf_type_wb[1:0] == 2'b00) ? 4'b1001 :
                               {2'b00 , icaf_type_wb[1:0]} ;


   assign mscause_type[3:0] = ( ({4{lsu_exc_valid_wb}} & lsu_error_mscause_wb[3:0]) |
                                ({4{trigger_hit_wb}} & 4'b0001) |
                                ({4{ebreak_wb}} & 4'b0010) |
                                ({4{inst_acc_wb}} & ifu_mscause[3:0])
                                );


   assign mscause_ns[3:0] = ( ({4{exc_or_int_valid_wb}} & mscause_type[3:0]) |
                              ({4{ wr_mscause_wb & ~exc_or_int_valid_wb}} & dec_i0_csr_wrdata_wb[3:0]) |
                              ({4{~wr_mscause_wb & ~exc_or_int_valid_wb}} & mscause[3:0])
                             );

   rvdff #(4)  mscause_ff (.*, .clk(e4e5_int_clk), .din(mscause_ns[3:0]), .dout(mscause[3:0]));

   // ----------------------------------------------------------------------
   // MTVAL (RW)
   // [31:0] : Exception address if relevant
   `define MTVAL 12'h343

   assign wr_mtval_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MTVAL);
   assign mtval_capture_pc_wb = exc_or_int_valid_wb & (ebreak_wb | (inst_acc_wb & ~inst_acc_second_wb) | mepc_trigger_hit_sel_pc_wb) & ~take_nmi_wb;
   assign mtval_capture_pc_plus2_wb = exc_or_int_valid_wb & (inst_acc_wb & inst_acc_second_wb) & ~take_nmi_wb;
   assign mtval_capture_inst_wb = exc_or_int_valid_wb & illegal_wb & ~take_nmi_wb;
   assign mtval_capture_lsu_wb = exc_or_int_valid_wb & lsu_exc_valid_wb & ~take_nmi_wb;
   assign mtval_clear_wb = exc_or_int_valid_wb & ~mtval_capture_pc_wb & ~mtval_capture_inst_wb & ~mtval_capture_lsu_wb & ~mepc_trigger_hit_sel_pc_wb;


   assign mtval_ns[31:0] = (({32{mtval_capture_pc_wb}} & {pc_wb[31:1], 1'b0}) |
                            ({32{mtval_capture_pc_plus2_wb}} & {pc_wb[31:1] + 31'b1, 1'b0}) |
                            ({32{mtval_capture_inst_wb}} & dec_illegal_inst[31:0]) |
                            ({32{mtval_capture_lsu_wb}} & lsu_error_pkt_addr_wb[31:0]) |
                            ({32{wr_mtval_wb & ~interrupt_valid_wb}} & dec_i0_csr_wrdata_wb[31:0]) |
                            ({32{~take_nmi_wb & ~wr_mtval_wb & ~mtval_capture_pc_wb & ~mtval_capture_inst_wb & ~mtval_clear_wb & ~mtval_capture_lsu_wb}} & mtval[31:0]) );


   rvdff #(32)  mtval_ff (.*, .clk(e4e5_int_clk), .din(mtval_ns[31:0]), .dout(mtval[31:0]));

   // ----------------------------------------------------------------------
   // MCPC (RW) Pause counter
   // [31:0] : Reads 0x0, decs in the wb register in decode_ctl

   `define MCPC 12'h7c2
   assign tlu_wr_pause_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MCPC) & ~interrupt_valid_wb & ~ext_int_freeze_d1;
   // ----------------------------------------------------------------------
   // MDEAU (WAR0)
   // [31:0] : Dbus Error Address Unlock register
   //
   `define MDEAU 12'hbc0

   assign wr_mdeau_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MDEAU);


   // ----------------------------------------------------------------------
   // MDSEAC (R)
   // [31:0] : Dbus Store Error Address Capture register
   //
   `define MDSEAC 12'hfc0

   // only capture error bus if the MDSEAC reg is not locked
   assign mdseac_locked_ns = mdseac_en | (mdseac_locked_f & ~wr_mdeau_wb);

   assign mdseac_en = (lsu_imprecise_error_store_any | lsu_imprecise_error_load_any) & ~nmi_int_detected_f & ~mdseac_locked_f;

   rvdffe #(32)  mdseac_ff (.*, .en(mdseac_en), .din(lsu_imprecise_error_addr_any[31:0]), .dout(mdseac[31:0]));


   // ----------------------------------------------------------------------
   // MPMC (R0W1)
   // [0:0] : FW halt
   //
   `define MPMC 12'h7c6
   assign wr_mpmc_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MPMC);

   // allow the cycle of the dbg halt flush that contains the wr_mpmc_wb to
   // set the mstatus bit potentially, use delayed version of internal dbg halt.
   // Kill the req when we commit the fwhalt csr write and take an int
   assign fw_halt_req = wr_mpmc_wb & dec_i0_csr_wrdata_wb[0] & ~internal_dbg_halt_mode_f3 & ~ext_int_freeze_d1 & ~interrupt_valid_wb;

   assign mpmc_b_ns[1] = wr_mpmc_wb ? ~dec_i0_csr_wrdata_wb[1] : ~mpmc[1];
   rvdff #(1)  mpmc_ff (.*, .clk(csr_wr_clk), .din(mpmc_b_ns[1]), .dout(mpmc_b[1]));
   assign mpmc[1] = ~mpmc_b[1];

   // ----------------------------------------------------------------------
   // MEIVT (External Interrupt Vector Table (R/W))
   // [31:10]: Base address (R/W)
   // [9:0]  : Reserved, reads 0x0
   `define MEIVT 12'hbc8

   assign wr_meivt_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MEIVT);

   rvdffe #(22)  meivt_ff (.*, .en(wr_meivt_wb), .din(dec_i0_csr_wrdata_wb[31:10]), .dout(meivt[31:10]));

   // ----------------------------------------------------------------------
   // MEIHAP (External Interrupt Handler Access Pointer (R))
   // [31:10]: Base address (R/W)
   // [9:2]  : ClaimID (R)
   // [1:0]  : Reserved, 0x0
   `define MEIHAP 12'hfc8

   assign wr_meihap_wb = wr_meicpct_wb;

   rvdffe #(8)  meihap_ff (.*, .en(wr_meihap_wb), .din(pic_claimid[7:0]), .dout(meihap[9:2]));
   assign dec_tlu_meihap[31:2] = {meivt[31:10], meihap[9:2]};

   // ----------------------------------------------------------------------
   // MEICURPL (R/W)
   // [31:4] : Reserved (read 0x0)
   // [3:0]  : CURRPRI - Priority level of current interrupt service routine (R/W)
   `define MEICURPL 12'hbcc

   assign wr_meicurpl_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MEICURPL);
   assign meicurpl_ns[3:0] = wr_meicurpl_wb ? dec_i0_csr_wrdata_wb[3:0] : meicurpl[3:0];

   rvdff #(4)  meicurpl_ff (.*, .clk(csr_wr_clk), .din(meicurpl_ns[3:0]), .dout(meicurpl[3:0]));

   // PIC needs this reg
   assign tlu_meicurpl[3:0] = meicurpl[3:0];


   // ----------------------------------------------------------------------
   // MEICIDPL (R/W)
   // [31:4] : Reserved (read 0x0)
   // [3:0]  : External Interrupt Claim ID's Priority Level Register
   `define MEICIDPL 12'hbcb

   assign wr_meicidpl_wb = (dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MEICIDPL)) | take_ext_int_start;

   assign meicidpl_ns[3:0] = wr_meicpct_wb ? pic_pl[3:0] : (wr_meicidpl_wb ? dec_i0_csr_wrdata_wb[3:0] : meicidpl[3:0]);

   rvdff #(4)  meicidpl_ff (.*, .clk(free_clk), .din(meicidpl_ns[3:0]), .dout(meicidpl[3:0]));

   // ----------------------------------------------------------------------
   // MEICPCT (Capture CLAIMID in MEIHAP and PL in MEICIDPL
   // [31:1] : Reserved (read 0x0)
   // [0]    : Capture (W1, Read 0)
   `define MEICPCT 12'hbca

   assign wr_meicpct_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MEICPCT) | take_ext_int_start;

   // ----------------------------------------------------------------------
   // MEIPT (External Interrupt Priority Threshold)
   // [31:4] : Reserved (read 0x0)
   // [3:0]  : PRITHRESH
   `define MEIPT 12'hbc9

   assign wr_meipt_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MEIPT);
   assign meipt_ns[3:0] = wr_meipt_wb ? dec_i0_csr_wrdata_wb[3:0] : meipt[3:0];

   rvdff #(4)  meipt_ff (.*, .clk(active_clk), .din(meipt_ns[3:0]), .dout(meipt[3:0]));

   // to PIC
   assign tlu_meipt[3:0] = meipt[3:0];
   // ----------------------------------------------------------------------
   // DCSR (R/W) (Only accessible in debug mode)
   // [31:28] : xdebugver (hard coded to 0x4) RO
   // [27:16] : 0x0, reserved
   // [15]    : ebreakm
   // [14]    : 0x0, reserved
   // [13]    : ebreaks (0x0 for this core)
   // [12]    : ebreaku (0x0 for this core)
   // [11]    : stepie
   // [10]    : stopcount
   // [9]     : 0x0 //stoptime
   // [8:6]   : cause (RO)
   // [5:4]   : 0x0, reserved
   // [3]     : nmip
   // [2]     : step
   // [1:0]   : prv (0x3 for this core)
   //
   `define DCSR 12'h7b0

   // RV has clarified that 'priority 4' in the spec means top priority.
   // 4. single step. 3. Debugger request. 2. Ebreak. 1. Trigger.

   // RV debug spec indicates a cause priority change for trigger hits during single step.
   assign trigger_hit_for_dscr_cause_wb = trigger_hit_dmode_wb | (trigger_hit_wb & dcsr_single_step_done_f);

   assign dcsr_cause[8:6] = ( ({3{dcsr_single_step_done_f & ~ebreak_to_debug_mode_wb & ~trigger_hit_for_dscr_cause_wb & ~debug_halt_req}} & 3'b100) |
                              ({3{debug_halt_req & ~ebreak_to_debug_mode_wb & ~trigger_hit_for_dscr_cause_wb}} &  3'b011) |
                              ({3{ebreak_to_debug_mode_wb & ~trigger_hit_for_dscr_cause_wb}} &  3'b001) |
                              ({3{trigger_hit_for_dscr_cause_wb}} & 3'b010));

   assign wr_dcsr_wb = allow_dbg_halt_csr_write & dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `DCSR);



  // Multiple halt enter requests can happen before we are halted.
  // We have to continue to upgrade based on dcsr_cause priority but we can't downgrade.
   assign dcsr_cause_upgradeable = internal_dbg_halt_mode_f & (dcsr[8:6] == 3'b011);
   assign enter_debug_halt_req_le = enter_debug_halt_req & (~dbg_tlu_halted | dcsr_cause_upgradeable);

   assign nmi_in_debug_mode = nmi_int_detected_f & internal_dbg_halt_mode_f;
   assign dcsr_ns[15:2] = enter_debug_halt_req_le ? {dcsr[15:9], dcsr_cause[8:6], dcsr[5:2]} :
                          (wr_dcsr_wb ? {dec_i0_csr_wrdata_wb[15], 3'b0, dec_i0_csr_wrdata_wb[11:10], 1'b0, dcsr[8:6], 2'b00, nmi_in_debug_mode | dcsr[3], dec_i0_csr_wrdata_wb[2]} :
                           {dcsr[15:4], nmi_in_debug_mode, dcsr[2]});

   rvdffe #(14)  dcsr_ff (.*, .en(enter_debug_halt_req_le | wr_dcsr_wb | internal_dbg_halt_mode | take_nmi_wb), .din(dcsr_ns[15:2]), .dout(dcsr[15:2]));

   assign tlu_dcsr_ss = dcsr[2];

   // ----------------------------------------------------------------------
   // DPC (R/W) (Only accessible in debug mode)
   // [31:0] : Debug PC
   `define DPC 12'h7b1

   assign wr_dpc_wb = allow_dbg_halt_csr_write & dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `DPC);
   assign dpc_capture_npc = dbg_tlu_halted & ~dbg_tlu_halted_f & ~request_debug_mode_done_f;
   assign dpc_capture_pc = request_debug_mode_wb;

   assign dpc_ns[31:1] = ( ({31{~dpc_capture_pc & ~dpc_capture_npc & wr_dpc_wb}} & dec_i0_csr_wrdata_wb[31:1]) |
                           ({31{dpc_capture_pc}} & pc_wb[31:1]) |
                           ({31{~dpc_capture_pc & dpc_capture_npc}} & npc_wb[31:1]) );

   rvdffe #(31)  dpc_ff (.*, .en(wr_dpc_wb | dpc_capture_pc | dpc_capture_npc), .din(dpc_ns[31:1]), .dout(dpc[31:1]));


    // ----------------------------------------------------------------------
   // DICAWICS (R/W) (Only accessible in debug mode)
   // [31:25] : Reserved
   // [24]    : Array select, 0 is data, 1 is tag
   // [23:22] : Reserved
   // [21:20] : Way select
   // [19:17] : Reserved
   // [16:3]  : Index
   // [2:0]   : Reserved
   `define DICAWICS 12'h7c8

   assign dicawics_ns[16:0] = {dec_i0_csr_wrdata_wb[24], dec_i0_csr_wrdata_wb[21:20], dec_i0_csr_wrdata_wb[16:3]};
   assign wr_dicawics_wb = allow_dbg_halt_csr_write & dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `DICAWICS);

   rvdffe #(17)  dicawics_ff (.*, .en(wr_dicawics_wb), .din(dicawics_ns[16:0]), .dout(dicawics[16:0]));

   // ----------------------------------------------------------------------
   // DICAD0 (R/W) (Only accessible in debug mode)
   //
   // If dicawics[array] is 0
   // [31:0]  : inst data
   //
   // If dicawics[array] is 1
   // [31:16] : Tag
   // [15:7]  : Reserved
   // [6:4]   : LRU
   // [3:1]   : Reserved
   // [0]     : Valid
   `define DICAD0 12'h7c9

   assign dicad0_ns[31:0] = wr_dicad0_wb ? dec_i0_csr_wrdata_wb[31:0] : ifu_ic_debug_rd_data[31:0];

   assign wr_dicad0_wb = allow_dbg_halt_csr_write & dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `DICAD0);

   rvdffe #(32)  dicad0_ff (.*, .en(wr_dicad0_wb | ifu_ic_debug_rd_data_valid), .din(dicad0_ns[31:0]), .dout(dicad0[31:0]));

   // ----------------------------------------------------------------------
   // DICAD0H (R/W) (Only accessible in debug mode)
   //
   // If dicawics[array] is 0
   // [63:32]  : inst data
   //
   `define DICAD0H 12'h7cc

   assign dicad0h_ns[31:0] = wr_dicad0h_wb ? dec_i0_csr_wrdata_wb[31:0] : ifu_ic_debug_rd_data[63:32];

   assign wr_dicad0h_wb = allow_dbg_halt_csr_write & dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `DICAD0H);

   rvdffe #(32)  dicad0h_ff (.*, .en(wr_dicad0h_wb | ifu_ic_debug_rd_data_valid), .din(dicad0h_ns[31:0]), .dout(dicad0h[31:0]));



if (pt.ICACHE_ECC == 1) begin
   // ----------------------------------------------------------------------
   // DICAD1 (R/W) (Only accessible in debug mode)
   // [6:0]     : ECC
   `define DICAD1 12'h7ca

   assign dicad1_ns[6:0] = wr_dicad1_wb ? dec_i0_csr_wrdata_wb[6:0] : ifu_ic_debug_rd_data[70:64];

   assign wr_dicad1_wb = allow_dbg_halt_csr_write & dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `DICAD1);

   rvdffs #(7)  dicad1_ff (.*, .clk(active_clk), .en(wr_dicad1_wb | ifu_ic_debug_rd_data_valid), .din(dicad1_ns[6:0]), .dout(dicad1_raw[6:0]));

   assign dicad1[31:0] = {25'b0, dicad1_raw[6:0]};
end
else begin
   // ----------------------------------------------------------------------
   // DICAD1 (R/W) (Only accessible in debug mode)
   // [3:0]     : Parity
   `define DICAD1 12'h7ca

   assign dicad1_ns[3:0] = wr_dicad1_wb ? dec_i0_csr_wrdata_wb[3:0] : ifu_ic_debug_rd_data[67:64];

   assign wr_dicad1_wb = allow_dbg_halt_csr_write & dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `DICAD1);

   rvdffs #(4)  dicad1_ff (.*, .clk(active_clk), .en(wr_dicad1_wb | ifu_ic_debug_rd_data_valid), .din(dicad1_ns[3:0]), .dout(dicad1_raw[3:0]));

   assign dicad1[31:0] = {28'b0, dicad1_raw[3:0]};
end
   // ----------------------------------------------------------------------
   // DICAGO (R/W) (Only accessible in debug mode)
   // [0]     : Go
   `define DICAGO 12'h7cb

if (pt.ICACHE_ECC == 1) begin
   assign dec_tlu_ic_diag_pkt.icache_wrdata[70:0] = {dicad1[6:0], dicad0h[31:0], dicad0[31:0]};
end
else begin
   assign dec_tlu_ic_diag_pkt.icache_wrdata[67:0] = {dicad1[3:0], dicad0h[31:0], dicad0[31:0]};
end
   assign dec_tlu_ic_diag_pkt.icache_dicawics[16:0] = dicawics[16:0];

   assign icache_rd_valid = allow_dbg_halt_csr_write & dec_i0_csr_any_unq_d & dec_i0_decode_d & ~dec_i0_csr_wen_unq_d & (dec_i0_csr_rdaddr_d[11:0] == `DICAGO);
   assign icache_wr_valid = allow_dbg_halt_csr_write & dec_i0_csr_any_unq_d & dec_i0_decode_d & dec_i0_csr_wen_unq_d & (dec_i0_csr_rdaddr_d[11:0] == `DICAGO);

   rvdff #(2)  dicgo_ff (.*, .clk(active_clk), .din({icache_rd_valid, icache_wr_valid}), .dout({icache_rd_valid_f, icache_wr_valid_f}));

   assign dec_tlu_ic_diag_pkt.icache_rd_valid = icache_rd_valid_f;
   assign dec_tlu_ic_diag_pkt.icache_wr_valid = icache_wr_valid_f;

   // ----------------------------------------------------------------------
   // MTSEL (R/W)
   // [1:0] : Trigger select : 00, 01, 10 are data/address triggers. 11 is inst count
   `define MTSEL 12'h7a0

   assign wr_mtsel_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MTSEL);
   assign mtsel_ns[1:0] = wr_mtsel_wb ? {dec_i0_csr_wrdata_wb[1:0]} : mtsel[1:0];

   rvdff #(2)  mtsel_ff (.*, .clk(csr_wr_clk), .din(mtsel_ns[1:0]), .dout(mtsel[1:0]));

   // ----------------------------------------------------------------------
   // MTDATA1 (R/W)
   // [31:0] : Trigger Data 1
   `define MTDATA1 12'h7a1

   // for triggers 0, 1, 2 and 3 aka Match Control
   // [31:28] : type, hard coded to 0x2
   // [27]    : dmode
   // [26:21] : hard coded to 0x1f
   // [20]    : hit
   // [19]    : select (0 - address, 1 - data)
   // [18]    : timing, always 'before', reads 0x0
   // [17:12] : action, bits  [17:13] not implemented and reads 0x0
   // [11]    : chain
   // [10:7]  : match, bits [10:8] not implemented and reads 0x0
   // [6]     : M
   // [5:3]   : not implemented, reads 0x0
   // [2]     : execute
   // [1]     : store
   // [0]     : load
   //
   // decoder ring
   // [27]    : => 9
   // [20]    : => 8
   // [19]    : => 7
   // [12]    : => 6
   // [11]    : => 5
   // [7]     : => 4
   // [6]     : => 3
   // [2]     : => 2
   // [1]     : => 1
   // [0]     : => 0


   // don't allow setting load-data.
   assign tdata_load = dec_i0_csr_wrdata_wb[0] & ~dec_i0_csr_wrdata_wb[19];
   // don't allow setting execute-data.
   assign tdata_opcode = dec_i0_csr_wrdata_wb[2] & ~dec_i0_csr_wrdata_wb[19];
   // don't allow clearing DMODE and action=1
   assign tdata_action = (dec_i0_csr_wrdata_wb[27] & dbg_tlu_halted_f) & dec_i0_csr_wrdata_wb[12];

   assign tdata_wrdata_wb[9:0]  = {dec_i0_csr_wrdata_wb[27] & dbg_tlu_halted_f,
                                   dec_i0_csr_wrdata_wb[20:19],
                                   tdata_action,
                                   dec_i0_csr_wrdata_wb[11],
                                   dec_i0_csr_wrdata_wb[7:6],
                                   tdata_opcode,
                                   dec_i0_csr_wrdata_wb[1],
                                   tdata_load};

   // If the DMODE bit is set, tdata1 can only be updated in debug_mode
   assign wr_mtdata1_t0_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MTDATA1) & (mtsel[1:0] == 2'b0) & (~mtdata1_t0[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign mtdata1_t0_ns[9:0] = wr_mtdata1_t0_wb ? tdata_wrdata_wb[9:0] :
                                {mtdata1_t0[9], update_hit_bit_wb[0] | mtdata1_t0[8], mtdata1_t0[7:0]};

   assign wr_mtdata1_t1_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MTDATA1) & (mtsel[1:0] == 2'b01) & (~mtdata1_t1[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign mtdata1_t1_ns[9:0] = wr_mtdata1_t1_wb ? tdata_wrdata_wb[9:0] :
                                {mtdata1_t1[9], update_hit_bit_wb[1] | mtdata1_t1[8], mtdata1_t1[7:0]};

   assign wr_mtdata1_t2_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MTDATA1) & (mtsel[1:0] == 2'b10) & (~mtdata1_t2[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign mtdata1_t2_ns[9:0] = wr_mtdata1_t2_wb ? tdata_wrdata_wb[9:0] :
                                {mtdata1_t2[9], update_hit_bit_wb[2] | mtdata1_t2[8], mtdata1_t2[7:0]};

   assign wr_mtdata1_t3_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MTDATA1) & (mtsel[1:0] == 2'b11) & (~mtdata1_t3[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign mtdata1_t3_ns[9:0] = wr_mtdata1_t3_wb ? tdata_wrdata_wb[9:0] :
                                {mtdata1_t3[9], update_hit_bit_wb[3] | mtdata1_t3[8], mtdata1_t3[7:0]};


   rvdff #(10)  mtdata1_t0_ff (.*, .clk(active_clk), .din(mtdata1_t0_ns[9:0]), .dout(mtdata1_t0[9:0]));
   rvdff #(10)  mtdata1_t1_ff (.*, .clk(active_clk), .din(mtdata1_t1_ns[9:0]), .dout(mtdata1_t1[9:0]));
   rvdff #(10)  mtdata1_t2_ff (.*, .clk(active_clk), .din(mtdata1_t2_ns[9:0]), .dout(mtdata1_t2[9:0]));
   rvdff #(10)  mtdata1_t3_ff (.*, .clk(active_clk), .din(mtdata1_t3_ns[9:0]), .dout(mtdata1_t3[9:0]));

   assign mtdata1_tsel_out[31:0] = ( ({32{(mtsel[1:0] == 2'b00)}} & {4'h2, mtdata1_t0[9], 6'b011111, mtdata1_t0[8:7], 6'b0, mtdata1_t0[6:5], 3'b0, mtdata1_t0[4:3], 3'b0, mtdata1_t0[2:0]}) |
                                     ({32{(mtsel[1:0] == 2'b01)}} & {4'h2, mtdata1_t1[9], 6'b011111, mtdata1_t1[8:7], 6'b0, mtdata1_t1[6:5], 3'b0, mtdata1_t1[4:3], 3'b0, mtdata1_t1[2:0]}) |
                                     ({32{(mtsel[1:0] == 2'b10)}} & {4'h2, mtdata1_t2[9], 6'b011111, mtdata1_t2[8:7], 6'b0, mtdata1_t2[6:5], 3'b0, mtdata1_t2[4:3], 3'b0, mtdata1_t2[2:0]}) |
                                     ({32{(mtsel[1:0] == 2'b11)}} & {4'h2, mtdata1_t3[9], 6'b011111, mtdata1_t3[8:7], 6'b0, mtdata1_t3[6:5], 3'b0, mtdata1_t3[4:3], 3'b0, mtdata1_t3[2:0]}));

   assign tlu_trigger_pkt_any[0].select = mtdata1_t0[`MTDATA1_SEL];
   assign tlu_trigger_pkt_any[0].match = mtdata1_t0[`MTDATA1_MATCH];
   assign tlu_trigger_pkt_any[0].store = mtdata1_t0[`MTDATA1_ST];
   assign tlu_trigger_pkt_any[0].load = mtdata1_t0[`MTDATA1_LD];
   assign tlu_trigger_pkt_any[0].execute = mtdata1_t0[`MTDATA1_EXE];
   assign tlu_trigger_pkt_any[0].m = mtdata1_t0[`MTDATA1_M_ENABLED];

   assign tlu_trigger_pkt_any[1].select = mtdata1_t1[`MTDATA1_SEL];
   assign tlu_trigger_pkt_any[1].match = mtdata1_t1[`MTDATA1_MATCH];
   assign tlu_trigger_pkt_any[1].store = mtdata1_t1[`MTDATA1_ST];
   assign tlu_trigger_pkt_any[1].load = mtdata1_t1[`MTDATA1_LD];
   assign tlu_trigger_pkt_any[1].execute = mtdata1_t1[`MTDATA1_EXE];
   assign tlu_trigger_pkt_any[1].m = mtdata1_t1[`MTDATA1_M_ENABLED];

   assign tlu_trigger_pkt_any[2].select = mtdata1_t2[`MTDATA1_SEL];
   assign tlu_trigger_pkt_any[2].match = mtdata1_t2[`MTDATA1_MATCH];
   assign tlu_trigger_pkt_any[2].store = mtdata1_t2[`MTDATA1_ST];
   assign tlu_trigger_pkt_any[2].load = mtdata1_t2[`MTDATA1_LD];
   assign tlu_trigger_pkt_any[2].execute = mtdata1_t2[`MTDATA1_EXE];
   assign tlu_trigger_pkt_any[2].m = mtdata1_t2[`MTDATA1_M_ENABLED];

   assign tlu_trigger_pkt_any[3].select = mtdata1_t3[`MTDATA1_SEL];
   assign tlu_trigger_pkt_any[3].match = mtdata1_t3[`MTDATA1_MATCH];
   assign tlu_trigger_pkt_any[3].store = mtdata1_t3[`MTDATA1_ST];
   assign tlu_trigger_pkt_any[3].load = mtdata1_t3[`MTDATA1_LD];
   assign tlu_trigger_pkt_any[3].execute = mtdata1_t3[`MTDATA1_EXE];
   assign tlu_trigger_pkt_any[3].m = mtdata1_t3[`MTDATA1_M_ENABLED];





   // ----------------------------------------------------------------------
   // MTDATA2 (R/W)
   // [31:0] : Trigger Data 2
   `define MTDATA2 12'h7a2

   // If the DMODE bit is set, tdata2 can only be updated in debug_mode
   assign wr_mtdata2_t0_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MTDATA2) & (mtsel[1:0] == 2'b0)  & (~mtdata1_t0[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign wr_mtdata2_t1_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MTDATA2) & (mtsel[1:0] == 2'b01) & (~mtdata1_t1[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign wr_mtdata2_t2_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MTDATA2) & (mtsel[1:0] == 2'b10) & (~mtdata1_t2[`MTDATA1_DMODE] | dbg_tlu_halted_f);
   assign wr_mtdata2_t3_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MTDATA2) & (mtsel[1:0] == 2'b11) & (~mtdata1_t3[`MTDATA1_DMODE] | dbg_tlu_halted_f);

   rvdffe #(32)  mtdata2_t0_ff (.*, .en(wr_mtdata2_t0_wb), .din(dec_i0_csr_wrdata_wb[31:0]), .dout(mtdata2_t0[31:0]));
   rvdffe #(32)  mtdata2_t1_ff (.*, .en(wr_mtdata2_t1_wb), .din(dec_i0_csr_wrdata_wb[31:0]), .dout(mtdata2_t1[31:0]));
   rvdffe #(32)  mtdata2_t2_ff (.*, .en(wr_mtdata2_t2_wb), .din(dec_i0_csr_wrdata_wb[31:0]), .dout(mtdata2_t2[31:0]));
   rvdffe #(32)  mtdata2_t3_ff (.*, .en(wr_mtdata2_t3_wb), .din(dec_i0_csr_wrdata_wb[31:0]), .dout(mtdata2_t3[31:0]));

   assign mtdata2_tsel_out[31:0] = ( ({32{(mtsel[1:0] == 2'b00)}} & mtdata2_t0[31:0]) |
                                     ({32{(mtsel[1:0] == 2'b01)}} & mtdata2_t1[31:0]) |
                                     ({32{(mtsel[1:0] == 2'b10)}} & mtdata2_t2[31:0]) |
                                     ({32{(mtsel[1:0] == 2'b11)}} & mtdata2_t3[31:0]));

   assign tlu_trigger_pkt_any[0].tdata2[31:0] = mtdata2_t0[31:0];
   assign tlu_trigger_pkt_any[1].tdata2[31:0] = mtdata2_t1[31:0];
   assign tlu_trigger_pkt_any[2].tdata2[31:0] = mtdata2_t2[31:0];
   assign tlu_trigger_pkt_any[3].tdata2[31:0] = mtdata2_t3[31:0];


   //----------------------------------------------------------------------
   // Performance Monitor Counters section starts
   //----------------------------------------------------------------------
   `define MHPME_NOEVENT         10'd0
   `define MHPME_CLK_ACTIVE      10'd1 // OOP - out of pipe
   `define MHPME_ICACHE_HIT      10'd2 // OOP
   `define MHPME_ICACHE_MISS     10'd3 // OOP
   `define MHPME_INST_COMMIT     10'd4
   `define MHPME_INST_COMMIT_16B 10'd5
   `define MHPME_INST_COMMIT_32B 10'd6
   `define MHPME_INST_ALIGNED    10'd7 // OOP
   `define MHPME_INST_DECODED    10'd8 // OOP
   `define MHPME_INST_MUL        10'd9
   `define MHPME_INST_DIV        10'd10
   `define MHPME_INST_LOAD       10'd11
   `define MHPME_INST_STORE      10'd12
   `define MHPME_INST_MALOAD     10'd13
   `define MHPME_INST_MASTORE    10'd14
   `define MHPME_INST_ALU        10'd15
   `define MHPME_INST_CSRREAD    10'd16
   `define MHPME_INST_CSRRW      10'd17
   `define MHPME_INST_CSRWRITE   10'd18
   `define MHPME_INST_EBREAK     10'd19
   `define MHPME_INST_ECALL      10'd20
   `define MHPME_INST_FENCE      10'd21
   `define MHPME_INST_FENCEI     10'd22
   `define MHPME_INST_MRET       10'd23
   `define MHPME_INST_BRANCH     10'd24
   `define MHPME_BRANCH_MP       10'd25
   `define MHPME_BRANCH_TAKEN    10'd26
   `define MHPME_BRANCH_NOTP     10'd27
   `define MHPME_FETCH_STALL     10'd28 // OOP
   `define MHPME_ALGNR_STALL     10'd29 // OOP
   `define MHPME_DECODE_STALL    10'd30 // OOP
   `define MHPME_POSTSYNC_STALL  10'd31 // OOP
   `define MHPME_PRESYNC_STALL   10'd32 // OOP
   `define MHPME_LSU_SB_WB_STALL 10'd34 // OOP
   `define MHPME_DMA_DCCM_STALL  10'd35 // OOP
   `define MHPME_DMA_ICCM_STALL  10'd36 // OOP
   `define MHPME_EXC_TAKEN       10'd37
   `define MHPME_TIMER_INT_TAKEN 10'd38
   `define MHPME_EXT_INT_TAKEN   10'd39
   `define MHPME_FLUSH_LOWER     10'd40
   `define MHPME_BR_ERROR        10'd41
   `define MHPME_IBUS_TRANS      10'd42 // OOP
   `define MHPME_DBUS_TRANS      10'd43 // OOP
   `define MHPME_DBUS_MA_TRANS   10'd44 // OOP
   `define MHPME_IBUS_ERROR      10'd45 // OOP
   `define MHPME_DBUS_ERROR      10'd46 // OOP
   `define MHPME_IBUS_STALL      10'd47 // OOP
   `define MHPME_DBUS_STALL      10'd48 // OOP
   `define MHPME_INT_DISABLED    10'd49 // OOP
   `define MHPME_INT_STALLED     10'd50 // OOP
   `define MHPME_INST_AMO        10'd51
   `define MHPME_INST_LR         10'd52
   `define MHPME_INST_SC         10'd53
   `define MHPME_INST_BITMANIP     10'd54
   `define MHPME_DBUS_LOAD       10'd55
   `define MHPME_DBUS_STORE      10'd56
   // Counts even during sleep state
   `define MHPME_SLEEP_CYC       10'd512 // OOP
   `define MHPME_DMA_READ_ALL    10'd513 // OOP
   `define MHPME_DMA_WRITE_ALL   10'd514 // OOP
   `define MHPME_DMA_READ_DCCM   10'd515 // OOP
   `define MHPME_DMA_WRITE_DCCM  10'd516 // OOP

   // Pack the event selects into a vector for genvar
   assign mhpme_vec[0][9:0] = mhpme3[9:0];
   assign mhpme_vec[1][9:0] = mhpme4[9:0];
   assign mhpme_vec[2][9:0] = mhpme5[9:0];
   assign mhpme_vec[3][9:0] = mhpme6[9:0];

   assign tlu_commit_lsu_op_e4 = (tlu_i0_commit_cmt &  tlu_packet_e4.lsu_pipe0) |
                                 (tlu_i1_commit_cmt & ~tlu_packet_e4.lsu_pipe0) ;
   // only consider committed itypes
   eh2_inst_pkt_t pmu_i0_itype_qual ;
   eh2_inst_pkt_t pmu_i1_itype_qual ;
   assign pmu_i0_itype_qual[4:0] = tlu_packet_e4.pmu_i0_itype[4:0] & {5{tlu_i0_commit_cmt}};
   assign pmu_i1_itype_qual[4:0] = tlu_packet_e4.pmu_i1_itype[4:0] & {5{tlu_i1_commit_cmt}};

   // Generate the muxed incs for all counters based on event type
   for (genvar i=0 ; i < 4; i++) begin
      assign mhpmc_inc_e4[i][1:0] =  {2{~mcountinhibit[i+3]}} &
           (
             ({2{(mhpme_vec[i][9:0] == `MHPME_CLK_ACTIVE      )}} & 2'b01) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_ICACHE_HIT      )}} & {1'b0, ifu_pmu_ic_hit}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_ICACHE_MISS     )}} & {1'b0, ifu_pmu_ic_miss}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_COMMIT     )}} & {tlu_i1_commit_cmt, tlu_i0_commit_cmt & ~illegal_e4}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_COMMIT_16B )}} & {tlu_i1_commit_cmt & ~exu_pmu_i1_pc4,
                                                                     tlu_i0_commit_cmt & ~exu_pmu_i0_pc4 & ~illegal_e4}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_COMMIT_32B )}} & {tlu_i1_commit_cmt &  exu_pmu_i1_pc4,
                                                                     tlu_i0_commit_cmt &  exu_pmu_i0_pc4 & ~illegal_e4}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_ALIGNED    )}} & ifu_pmu_instr_aligned[1:0])  |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_DECODED    )}} & dec_pmu_instr_decoded[1:0])  |
             ({2{(mhpme_vec[i][9:0] == `MHPME_ALGNR_STALL     )}} & {1'b0,ifu_pmu_align_stall})  |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DECODE_STALL    )}} & {1'b0,dec_pmu_decode_stall}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_MUL        )}} & {(pmu_i1_itype_qual == MUL),     (pmu_i0_itype_qual == MUL)})     |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_DIV        )}} & {1'b0, tlu_packet_e4.pmu_divide & tlu_i0_commit_cmt})     |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_LOAD       )}} & {(pmu_i1_itype_qual == LOAD),    (pmu_i0_itype_qual == LOAD)})    |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_STORE      )}} & {(pmu_i1_itype_qual == STORE),   (pmu_i0_itype_qual == STORE)})   |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_MALOAD     )}} & {(pmu_i1_itype_qual == LOAD),    (pmu_i0_itype_qual == LOAD)} &
                                                                      {2{tlu_packet_e4.pmu_lsu_misaligned}})    |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_MASTORE    )}} & {(pmu_i1_itype_qual == STORE),   (pmu_i0_itype_qual == STORE)} &
                                                                      {2{tlu_packet_e4.pmu_lsu_misaligned}})    |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_ALU        )}} & {(pmu_i1_itype_qual == ALU),     (pmu_i0_itype_qual == ALU)})     |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_CSRREAD    )}} & {1'b0, (pmu_i0_itype_qual == CSRREAD)}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_CSRWRITE   )}} & {1'b0, (pmu_i0_itype_qual == CSRWRITE)})|
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_CSRRW      )}} & {1'b0, (pmu_i0_itype_qual == CSRRW)})   |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_EBREAK     )}} & {1'b0, (pmu_i0_itype_qual == EBREAK)})  |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_ECALL      )}} & {1'b0, (pmu_i0_itype_qual == ECALL)})   |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_FENCE      )}} & {1'b0, (pmu_i0_itype_qual == FENCE)})   |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_FENCEI     )}} & {1'b0, (pmu_i0_itype_qual == FENCEI)})  |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_MRET       )}} & {1'b0, (pmu_i0_itype_qual == MRET)})    |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_BRANCH     )}} & {((pmu_i1_itype_qual == CONDBR) | (pmu_i1_itype_qual == JAL)),
                                                                     ((pmu_i0_itype_qual == CONDBR) | (pmu_i0_itype_qual == JAL))})   |
             ({2{(mhpme_vec[i][9:0] == `MHPME_BRANCH_MP       )}} & {exu_pmu_i1_br_misp & tlu_i1_commit_cmt,
                                                                     exu_pmu_i0_br_misp & tlu_i0_commit_cmt}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_BRANCH_TAKEN    )}} & {exu_pmu_i1_br_ataken & tlu_i1_commit_cmt,
                                                                     exu_pmu_i0_br_ataken & tlu_i0_commit_cmt}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_BRANCH_NOTP     )}} & {tlu_packet_e4.pmu_i1_br_unpred & tlu_i1_commit_cmt,
                                                                     tlu_packet_e4.pmu_i0_br_unpred & tlu_i0_commit_cmt}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_FETCH_STALL     )}} & {1'b0, ifu_pmu_fetch_stall}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_ALGNR_STALL     )}} & {1'b0, ifu_pmu_align_stall}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DECODE_STALL    )}} & {1'b0, dec_pmu_decode_stall}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_POSTSYNC_STALL  )}} & {1'b0,dec_pmu_postsync_stall}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_PRESYNC_STALL   )}} & {1'b0,dec_pmu_presync_stall}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_LSU_SB_WB_STALL )}} & {1'b0, lsu_store_stall_any}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DMA_DCCM_STALL  )}} & {1'b0, dma_dccm_stall_any}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DMA_ICCM_STALL  )}} & {1'b0, dma_iccm_stall_any}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_EXC_TAKEN       )}} & {1'b0, (i0_exception_valid_e4 | trigger_hit_e4 | lsu_exc_valid_e4)}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_TIMER_INT_TAKEN )}} & {1'b0, take_timer_int | take_int_timer0_int | take_int_timer1_int}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_EXT_INT_TAKEN   )}} & {1'b0, take_ext_int}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_FLUSH_LOWER     )}} & {1'b0, tlu_flush_lower_e4}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_BR_ERROR        )}} & {(dec_tlu_br1_error_e4 | dec_tlu_br1_start_error_e4) & rfpc_i1_e4,
                                                                     (dec_tlu_br0_error_e4 | dec_tlu_br0_start_error_e4) & rfpc_i0_e4}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_IBUS_TRANS      )}} & {1'b0, ifu_pmu_bus_trxn}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DBUS_TRANS      )}} & {1'b0, lsu_pmu_bus_trxn}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DBUS_MA_TRANS   )}} & {1'b0, lsu_pmu_bus_misaligned}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_IBUS_ERROR      )}} & {1'b0, ifu_pmu_bus_error}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DBUS_ERROR      )}} & {1'b0, lsu_pmu_bus_error}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_IBUS_STALL      )}} & {1'b0, ifu_pmu_bus_busy}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DBUS_STALL      )}} & {1'b0, lsu_pmu_bus_busy}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INT_DISABLED    )}} & {1'b0, ~mstatus[`MSTATUS_MIE]}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INT_STALLED     )}} & {1'b0, ~mstatus[`MSTATUS_MIE] & |(mip[5:0] & mie[5:0])}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_AMO        )}} & {(pmu_i1_itype_qual == ATOMIC),    (pmu_i0_itype_qual == ATOMIC)}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_LR         )}} & {(pmu_i1_itype_qual == LR),    (pmu_i0_itype_qual == LR)}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_SC         )}} & {(pmu_i1_itype_qual == SC),    (pmu_i0_itype_qual == SC)}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_INST_BITMANIP     )}} & {(pmu_i1_itype_qual == BITMANIPU),    (pmu_i0_itype_qual == BITMANIPU)}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DBUS_LOAD       )}} & {1'b0, tlu_commit_lsu_op_e4 & lsu_pmu_load_external_dc4}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DBUS_STORE      )}} & {1'b0, tlu_commit_lsu_op_e4 & lsu_pmu_store_external_dc4}) |
             // These count even during sleep
             ({2{(mhpme_vec[i][9:0] == `MHPME_SLEEP_CYC       )}} & {1'b0, dec_tlu_pmu_fw_halted}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DMA_READ_ALL    )}} & {1'b0, dma_pmu_any_read}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DMA_WRITE_ALL   )}} & {1'b0, dma_pmu_any_write}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DMA_READ_DCCM   )}} & {1'b0, dma_pmu_dccm_read}) |
             ({2{(mhpme_vec[i][9:0] == `MHPME_DMA_WRITE_DCCM  )}} & {1'b0, dma_pmu_dccm_write})
             );
   end
   rvdff #(2) pmu0inc_ff (.*, .clk(free_clk), .din(mhpmc_inc_e4[0][1:0]), .dout(mhpmc_inc_wb[0][1:0]));
   rvdff #(2) pmu1inc_ff (.*, .clk(free_clk), .din(mhpmc_inc_e4[1][1:0]), .dout(mhpmc_inc_wb[1][1:0]));
   rvdff #(2) pmu2inc_ff (.*, .clk(free_clk), .din(mhpmc_inc_e4[2][1:0]), .dout(mhpmc_inc_wb[2][1:0]));
   rvdff #(2) pmu3inc_ff (.*, .clk(free_clk), .din(mhpmc_inc_e4[3][1:0]), .dout(mhpmc_inc_wb[3][1:0]));

   assign perfcnt_halted = ((dec_tlu_dbg_halted & dcsr[`DCSR_STOPC]) | dec_tlu_pmu_fw_halted);
   assign perfcnt_during_sleep[3:0] = {4{~(dec_tlu_dbg_halted & dcsr[`DCSR_STOPC])}} &
                                      {mhpme_vec[3][9],mhpme_vec[2][9],mhpme_vec[1][9],mhpme_vec[0][9]};



   assign tlu_perfcnt0[1:0] = mhpmc_inc_wb[0][1:0] & ~{2{perfcnt_halted & ~perfcnt_during_sleep[0]}};
   assign tlu_perfcnt1[1:0] = mhpmc_inc_wb[1][1:0] & ~{2{perfcnt_halted & ~perfcnt_during_sleep[1]}};
   assign tlu_perfcnt2[1:0] = mhpmc_inc_wb[2][1:0] & ~{2{perfcnt_halted & ~perfcnt_during_sleep[2]}};
   assign tlu_perfcnt3[1:0] = mhpmc_inc_wb[3][1:0] & ~{2{perfcnt_halted & ~perfcnt_during_sleep[3]}};

   // ----------------------------------------------------------------------
   // MHPMC3H(RW), MHPMC3(RW)
   // [63:32][31:0] : Hardware Performance Monitor Counter 3
   `define MHPMC3 12'hB03
   `define MHPMC3H 12'hB83

   assign mhpmc3_wr_en0 = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPMC3);
   assign mhpmc3_wr_en1 = (~perfcnt_halted | perfcnt_during_sleep[0]) & (|(mhpmc_inc_wb[0][1:0]));
   assign mhpmc3_wr_en  = mhpmc3_wr_en0 | mhpmc3_wr_en1;
   assign mhpmc3_incr[63:0] = {mhpmc3h[31:0],mhpmc3[31:0]} + {63'b0,mhpmc_inc_wb[0][1]} + {63'b0,mhpmc_inc_wb[0][0]};
   assign mhpmc3_ns[31:0] = mhpmc3_wr_en0 ? dec_i0_csr_wrdata_wb[31:0] : mhpmc3_incr[31:0];
   rvdffe #(32)  mhpmc3_ff (.*, .en(mhpmc3_wr_en), .din(mhpmc3_ns[31:0]), .dout(mhpmc3[31:0]));

   assign mhpmc3h_wr_en0 = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPMC3H);
   assign mhpmc3h_wr_en  = mhpmc3h_wr_en0 | mhpmc3_wr_en1;
   assign mhpmc3h_ns[31:0] = mhpmc3h_wr_en0 ? dec_i0_csr_wrdata_wb[31:0] : mhpmc3_incr[63:32];
   rvdffe #(32)  mhpmc3h_ff (.*, .en(mhpmc3h_wr_en), .din(mhpmc3h_ns[31:0]), .dout(mhpmc3h[31:0]));

   // ----------------------------------------------------------------------
   // MHPMC4H(RW), MHPMC4(RW)
   // [63:32][31:0] : Hardware Performance Monitor Counter 4
   `define MHPMC4 12'hB04
   `define MHPMC4H 12'hB84

   assign mhpmc4_wr_en0 = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPMC4);
   assign mhpmc4_wr_en1 = (~perfcnt_halted | perfcnt_during_sleep[1]) & (|(mhpmc_inc_wb[1][1:0]));
   assign mhpmc4_wr_en  = mhpmc4_wr_en0 | mhpmc4_wr_en1;
   assign mhpmc4_incr[63:0] = {mhpmc4h[31:0],mhpmc4[31:0]} + {63'b0,mhpmc_inc_wb[1][1]} + {63'b0,mhpmc_inc_wb[1][0]};
   assign mhpmc4_ns[31:0] = mhpmc4_wr_en0 ? dec_i0_csr_wrdata_wb[31:0] : mhpmc4_incr[31:0];
   rvdffe #(32)  mhpmc4_ff (.*, .en(mhpmc4_wr_en), .din(mhpmc4_ns[31:0]), .dout(mhpmc4[31:0]));

   assign mhpmc4h_wr_en0 = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPMC4H);
   assign mhpmc4h_wr_en  = mhpmc4h_wr_en0 | mhpmc4_wr_en1;
   assign mhpmc4h_ns[31:0] = mhpmc4h_wr_en0 ? dec_i0_csr_wrdata_wb[31:0] : mhpmc4_incr[63:32];
   rvdffe #(32)  mhpmc4h_ff (.*, .en(mhpmc4h_wr_en), .din(mhpmc4h_ns[31:0]), .dout(mhpmc4h[31:0]));

   // ----------------------------------------------------------------------
   // MHPMC5H(RW), MHPMC5(RW)
   // [63:32][31:0] : Hardware Performance Monitor Counter 5
   `define MHPMC5 12'hB05
   `define MHPMC5H 12'hB85

   assign mhpmc5_wr_en0 = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPMC5);
   assign mhpmc5_wr_en1 = (~perfcnt_halted | perfcnt_during_sleep[2]) & (|(mhpmc_inc_wb[2][1:0]));
   assign mhpmc5_wr_en  = mhpmc5_wr_en0 | mhpmc5_wr_en1;
   assign mhpmc5_incr[63:0] = {mhpmc5h[31:0],mhpmc5[31:0]} + {63'b0,mhpmc_inc_wb[2][1]} + {63'b0,mhpmc_inc_wb[2][0]};
   assign mhpmc5_ns[31:0] = mhpmc5_wr_en0 ? dec_i0_csr_wrdata_wb[31:0] : mhpmc5_incr[31:0];
   rvdffe #(32)  mhpmc5_ff (.*, .en(mhpmc5_wr_en), .din(mhpmc5_ns[31:0]), .dout(mhpmc5[31:0]));

   assign mhpmc5h_wr_en0 = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPMC5H);
   assign mhpmc5h_wr_en  = mhpmc5h_wr_en0 | mhpmc5_wr_en1;
   assign mhpmc5h_ns[31:0] = mhpmc5h_wr_en0 ? dec_i0_csr_wrdata_wb[31:0] : mhpmc5_incr[63:32];
   rvdffe #(32)  mhpmc5h_ff (.*, .en(mhpmc5h_wr_en), .din(mhpmc5h_ns[31:0]), .dout(mhpmc5h[31:0]));

   // ----------------------------------------------------------------------
   // MHPMC6H(RW), MHPMC6(RW)
   // [63:32][31:0] : Hardware Performance Monitor Counter 6
   `define MHPMC6 12'hB06
   `define MHPMC6H 12'hB86

   assign mhpmc6_wr_en0 = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPMC6);
   assign mhpmc6_wr_en1 = (~perfcnt_halted | perfcnt_during_sleep[3]) & (|(mhpmc_inc_wb[3][1:0]));
   assign mhpmc6_wr_en  = mhpmc6_wr_en0 | mhpmc6_wr_en1;
   assign mhpmc6_incr[63:0] = {mhpmc6h[31:0],mhpmc6[31:0]} + {63'b0,mhpmc_inc_wb[3][1]} + {63'b0,mhpmc_inc_wb[3][0]};
   assign mhpmc6_ns[31:0] = mhpmc6_wr_en0 ? dec_i0_csr_wrdata_wb[31:0] : mhpmc6_incr[31:0];
   rvdffe #(32)  mhpmc6_ff (.*, .en(mhpmc6_wr_en), .din(mhpmc6_ns[31:0]), .dout(mhpmc6[31:0]));

   assign mhpmc6h_wr_en0 = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPMC6H);
   assign mhpmc6h_wr_en  = mhpmc6h_wr_en0 | mhpmc6_wr_en1;
   assign mhpmc6h_ns[31:0] = mhpmc6h_wr_en0 ? dec_i0_csr_wrdata_wb[31:0] : mhpmc6_incr[63:32];
   rvdffe #(32)  mhpmc6h_ff (.*, .en(mhpmc6h_wr_en), .din(mhpmc6h_ns[31:0]), .dout(mhpmc6h[31:0]));

   // ----------------------------------------------------------------------
   // MHPME3(RW)
   // [9:0] : Hardware Performance Monitor Event 3
   `define MHPME3 12'h323

   // we only have events 0-56, 512-516, HPME* are WARL so saturate otherwise
   assign event_saturate_wb[9:0] = ((dec_i0_csr_wrdata_wb[9:0] > 10'd516) | (|dec_i0_csr_wrdata_wb[31:10])) ? 10'd516 : dec_i0_csr_wrdata_wb[9:0];

   assign wr_mhpme3_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPME3);
   rvdffs #(10)  mhpme3_ff (.*, .clk(active_clk), .en(wr_mhpme3_wb), .din(event_saturate_wb[9:0]), .dout(mhpme3[9:0]));
   // ----------------------------------------------------------------------
   // MHPME4(RW)
   // [9:0] : Hardware Performance Monitor Event 4
   `define MHPME4 12'h324

   assign wr_mhpme4_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPME4);
   rvdffs #(10)  mhpme4_ff (.*, .clk(active_clk), .en(wr_mhpme4_wb), .din(event_saturate_wb[9:0]), .dout(mhpme4[9:0]));
   // ----------------------------------------------------------------------
   // MHPME5(RW)
   // [9:0] : Hardware Performance Monitor Event 5
   `define MHPME5 12'h325

   assign wr_mhpme5_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPME5);
   rvdffs #(10)  mhpme5_ff (.*, .clk(active_clk), .en(wr_mhpme5_wb), .din(event_saturate_wb[9:0]), .dout(mhpme5[9:0]));
   // ----------------------------------------------------------------------
   // MHPME6(RW)
   // [9:0] : Hardware Performance Monitor Event 6
   `define MHPME6 12'h326

   assign wr_mhpme6_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MHPME6);
   rvdffs #(10)  mhpme6_ff (.*, .clk(active_clk), .en(wr_mhpme6_wb), .din(event_saturate_wb[9:0]), .dout(mhpme6[9:0]));

   // MCOUNTINHIBIT(RW)
   // [31:7] : Reserved, read 0x0
   // [6]    : HPM6 disable
   // [5]    : HPM5 disable
   // [4]    : HPM4 disable
   // [3]    : HPM3 disable
   // [2]    : MINSTRET disable
   // [1]    : reserved, read 0x0
   // [0]    : MCYCLE disable

   `define MCOUNTINHIBIT 12'h320

   assign wr_mcountinhibit_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MCOUNTINHIBIT);

   rvdffs #(6)  mcountinhibit_ff (.*, .clk(active_clk), .en(wr_mcountinhibit_wb), .din({dec_i0_csr_wrdata_wb[6:2], dec_i0_csr_wrdata_wb[0]}), .dout({mcountinhibit[6:2], mcountinhibit[0]}));
   assign mcountinhibit[1] = 1'b0;

   //----------------------------------------------------------------------
   // Performance Monitor Counters section ends
   //----------------------------------------------------------------------
   // ----------------------------------------------------------------------


   // ----------------------------------------------------------------------
   // MFDHS(RW)
   // [1] : LSU operation pending when debug halt threshold reached
   // [0] : IFU operation pending when debug halt threshold reached

   `define MFDHS 12'h7cf

   assign wr_mfdhs_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MFDHS);

   assign mfdhs_ns[1:0] = wr_mfdhs_wb ? dec_i0_csr_wrdata_wb[1:0] : ((dbg_tlu_halted & ~dbg_tlu_halted_f) ? {~lsu_idle_any_f, ~ifu_miss_state_idle_f} : mfdhs[1:0]);

   rvdffs #(2)  mfdhs_ff (.*, .clk(active_clk), .en(wr_mfdhs_wb | dbg_tlu_halted), .din(mfdhs_ns[1:0]), .dout(mfdhs[1:0]));

   assign force_halt_ctr[31:0] = debug_halt_req_f ? (force_halt_ctr_f[31:0] + 32'b1) : (dbg_tlu_halted_f ? 32'b0 : force_halt_ctr_f[31:0]);

   rvdffs #(32)  forcehaltctr_ff (.*, .clk(active_clk), .en(mfdht[0]), .din(force_halt_ctr[31:0]), .dout(force_halt_ctr_f[31:0]));

   assign force_halt = mfdht[0] & |(force_halt_ctr_f[31:0] & (32'hffffffff << mfdht[5:1]));



   //--------------------------------------------------------------------------------
   // trace
   //--------------------------------------------------------------------------------

   rvoclkhdr trace_cgc ( .en(i0_valid_wb | i1_valid_wb | exc_or_int_valid_wb | interrupt_valid_wb | tlu_i0_valid_wb1 | tlu_i1_valid_wb1 |
                                tlu_i0_exc_valid_wb1 | tlu_i1_exc_valid_wb1 | tlu_int_valid_wb1 | clk_override), .l1clk(trace_tclk), .* );
   rvdff #(10)  traceff (.*,   .clk(trace_tclk),
                        .din ({i0_valid_wb, i1_valid_wb,
                               i0_exception_valid_wb | lsu_i0_exc_wb | (i0_trigger_hit_wb & ~trigger_hit_dmode_wb),
                               ~(i0_exception_valid_wb | lsu_i0_exc_wb | i0_trigger_hit_wb) & exc_or_int_valid_wb & ~interrupt_valid_wb,
                               exc_cause_wb[4:0],
                               interrupt_valid_wb}),
                        .dout({tlu_i0_valid_wb1, tlu_i1_valid_wb1,
                               tlu_i0_exc_valid_wb1, tlu_i1_exc_valid_wb1,
                               tlu_exc_cause_wb1[4:0],
                               tlu_int_valid_wb1}));

   assign tlu_mtval_wb1  = mtval[31:0];

   // end trace
   //--------------------------------------------------------------------------------


   // ----------------------------------------------------------------------
   // CSR read mux
   // ----------------------------------------------------------------------
   assign csr_rd = tlu_i0_csr_pkt_d;

   assign csr_rddata_d[31:0] = (  ({32{csr_rd.csr_mhartid}}   & {core_id[31:4], 3'b0, mytid}) |
                                  ({32{csr_rd.csr_mstatus}}   & {19'b0, 2'b11, 3'b0, mstatus[1], 3'b0, mstatus[0], 3'b0}) |
                                  ({32{csr_rd.csr_mtvec}}     & {mtvec[30:1], 1'b0, mtvec[0]}) |
                                  ({32{csr_rd.csr_mip}}       & {1'b0, mip[5:3], 16'b0, mip[2], 3'b0, mip[1], 3'b0, mip[0], 3'b0}) |
                                  ({32{csr_rd.csr_mie}}       & {1'b0, mie[5:3], 16'b0, mie[2], 3'b0, mie[1], 3'b0, mie[0], 3'b0}) |
                                  ({32{csr_rd.csr_mcyclel}}   & mcyclel[31:0]) |
                                  ({32{csr_rd.csr_mcycleh}}   & mcycleh_inc[31:0]) |
                                  ({32{csr_rd.csr_minstretl}} & minstretl_read[31:0]) |
                                  ({32{csr_rd.csr_minstreth}} & minstreth_read[31:0]) |
                                  ({32{csr_rd.csr_mscratch}}  & mscratch[31:0]) |
                                  ({32{csr_rd.csr_mepc}}      & {mepc[31:1], 1'b0}) |
                                  ({32{csr_rd.csr_mcause}}    & mcause[31:0]) |
                                  ({32{csr_rd.csr_mscause}}   & {28'b0, mscause[3:0]}) |
                                  ({32{csr_rd.csr_mtval}}     & mtval[31:0]) |
                                  ({32{csr_rd.csr_mdseac}}    & mdseac[31:0]) |
                                  ({32{csr_rd.csr_meivt}}     & {meivt[31:10], 10'b0}) |
                                  ({32{csr_rd.csr_meihap}}    & {meivt[31:10], meihap[9:2], 2'b0}) |
                                  ({32{csr_rd.csr_meicurpl}}  & {28'b0, meicurpl[3:0]}) |
                                  ({32{csr_rd.csr_meicidpl}}  & {28'b0, meicidpl[3:0]}) |
                                  ({32{csr_rd.csr_meipt}}     & {28'b0, meipt[3:0]}) |
                                  ({32{csr_rd.csr_dcsr}}      & {16'h4000, dcsr[15:2], 2'b11}) |
                                  ({32{csr_rd.csr_dpc}}       & {dpc[31:1], 1'b0}) |
                                  ({32{csr_rd.csr_mtsel}}     & {30'b0, mtsel[1:0]}) |
                                  ({32{csr_rd.csr_mtdata1}}   & {mtdata1_tsel_out[31:0]}) |
                                  ({32{csr_rd.csr_mtdata2}}   & {mtdata2_tsel_out[31:0]}) |
                                  ({32{csr_rd.csr_mhpmc3}}    & mhpmc3[31:0]) |
                                  ({32{csr_rd.csr_mhpmc4}}    & mhpmc4[31:0]) |
                                  ({32{csr_rd.csr_mhpmc5}}    & mhpmc5[31:0]) |
                                  ({32{csr_rd.csr_mhpmc6}}    & mhpmc6[31:0]) |
                                  ({32{csr_rd.csr_mhpmc3h}}   & mhpmc3h[31:0]) |
                                  ({32{csr_rd.csr_mhpmc4h}}   & mhpmc4h[31:0]) |
                                  ({32{csr_rd.csr_mhpmc5h}}   & mhpmc5h[31:0]) |
                                  ({32{csr_rd.csr_mhpmc6h}}   & mhpmc6h[31:0]) |
                                  ({32{csr_rd.csr_mhpme3}}    & {22'b0,mhpme3[9:0]}) |
                                  ({32{csr_rd.csr_mhpme4}}    & {22'b0,mhpme4[9:0]}) |
                                  ({32{csr_rd.csr_mhpme5}}    & {22'b0,mhpme5[9:0]}) |
                                  ({32{csr_rd.csr_mhpme6}}    & {22'b0,mhpme6[9:0]}) |
                                  ({32{csr_rd.csr_mcountinhibit}} & {25'b0, mcountinhibit[6:0]}) |
                                  ({32{csr_rd.csr_mpmc}}      & {30'b0, mpmc[1], 1'b0}) |
                                  ({32{csr_rd.csr_dicad0}}    & dicad0[31:0]) |
                                  ({32{csr_rd.csr_dicad0h}}   & dicad0h[31:0]) |
                                  ({32{csr_rd.csr_dicad1}}    & dicad1[31:0]) |
                                  ({32{csr_rd.csr_dicawics}}  & {7'b0, dicawics[16], 2'b0, dicawics[15:14], 3'b0, dicawics[13:0], 3'b0}) |
                                  ({32{csr_rd.csr_mfdhs}}     & {30'b0, mfdhs[1:0]}) |
                                  ({32{dec_timer_read_d}} & dec_timer_rddata_d[31:0])
                                  );
//   end // block: CSR_rd_mux

endmodule // eh2_dec_tlu_ctl

module eh2_dec_timer_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (
   input logic clk,
   input logic free_clk,
   input logic rst_l,
   input logic        dec_i0_csr_wen_wb_mod,      // csr write enable at wb

   input logic [11:0] dec_i0_csr_wraddr_wb,      // write address for csr
   input logic [31:0] dec_i0_csr_wrdata_wb,   // csr write data at wb

   input eh2_csr_tlu_pkt_t csr_rd, // csr decodes

   input logic dec_pause_state, // Paused
   input logic dec_tlu_pmu_fw_halted, // pmu/fw halted
   input logic internal_dbg_halt_timers, // debug halted

   output logic [31:0] dec_timer_rddata_d, // timer CSR read data
   output logic   dec_timer_read_d, // timer CSR address match
   output logic        dec_timer_t0_pulse, // timer0 int
   output logic        dec_timer_t1_pulse, // timer1 int

   input  logic        scan_mode
   );
   `define MITCTL_ENABLE 0
   `define MITCTL_ENABLE_HALTED 1
   `define MITCTL_ENABLE_PAUSED 2

   logic [31:0] mitcnt0_ns, mitcnt0, mitcnt1_ns, mitcnt1, mitb0, mitb1, mitb0_b, mitb1_b, mitcnt0_inc, mitcnt1_inc;
   logic [2:0] mitctl0_ns, mitctl0;
   logic [3:0] mitctl1_ns, mitctl1;
   logic wr_mitcnt0_wb, wr_mitcnt1_wb, wr_mitb0_wb, wr_mitb1_wb, wr_mitctl0_wb, wr_mitctl1_wb;
   logic mitcnt0_inc_ok, mitcnt1_inc_ok, mitcnt0_cout_nc, mitcnt1_cout_nc;

 logic mit0_match_ns;
 logic mit1_match_ns;
 logic mitctl0_0_b_ns;
 logic mitctl0_0_b;
 logic mitctl1_0_b_ns;
 logic mitctl1_0_b;
   logic mit0_match_d1;

   if(pt.TIMER_LEGAL_EN) begin : internal_timers

   assign mit0_match_ns = (mitcnt0[31:0] >= mitb0[31:0]);
   assign mit1_match_ns = (mitcnt1[31:0] >= mitb1[31:0]);

   assign dec_timer_t0_pulse = mit0_match_ns;
   assign dec_timer_t1_pulse = mit1_match_ns;
   // ----------------------------------------------------------------------
   // MITCNT0 (RW)
   // [31:0] : Internal Timer Counter 0

   `define MITCNT0 12'h7d2

   assign wr_mitcnt0_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MITCNT0);

   assign mitcnt0_inc_ok = mitctl0[`MITCTL_ENABLE] & (~dec_pause_state | mitctl0[`MITCTL_ENABLE_PAUSED]) & (~dec_tlu_pmu_fw_halted | mitctl0[`MITCTL_ENABLE_HALTED]) & ~internal_dbg_halt_timers;

   assign {mitcnt0_cout_nc, mitcnt0_inc[31:0]} = mitcnt0[31:0] + {31'b0, 1'b1};
   assign mitcnt0_ns[31:0] = mit0_match_ns ? 'b0 : wr_mitcnt0_wb ? dec_i0_csr_wrdata_wb[31:0] : mitcnt0_inc[31:0];

   rvdffe #(32) mitcnt0_ff      (.*, .en(wr_mitcnt0_wb | mitcnt0_inc_ok | mit0_match_ns), .din(mitcnt0_ns[31:0]), .dout(mitcnt0[31:0]));

   // ----------------------------------------------------------------------
   // MITCNT1 (RW)
   // [31:0] : Internal Timer Counter 0

   `define MITCNT1 12'h7d5

   assign wr_mitcnt1_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MITCNT1);

   assign mitcnt1_inc_ok = mitctl1[`MITCTL_ENABLE] & (~dec_pause_state | mitctl1[`MITCTL_ENABLE_PAUSED]) & (~dec_tlu_pmu_fw_halted | mitctl1[`MITCTL_ENABLE_HALTED]) & ~internal_dbg_halt_timers;

   // only inc MITCNT1 if not cascaded with 0, or if 0 overflows
   assign {mitcnt1_cout_nc, mitcnt1_inc[31:0]} = mitcnt1[31:0] + {31'b0, (~mitctl1[3] | mit0_match_d1)};
   assign mitcnt1_ns[31:0] = mit1_match_ns ? 'b0 :  wr_mitcnt1_wb ? dec_i0_csr_wrdata_wb[31:0] : mitcnt1_inc[31:0];

   rvdffe #(32) mitcnt1_ff      (.*, .en(wr_mitcnt1_wb | mitcnt1_inc_ok | mit1_match_ns), .din(mitcnt1_ns[31:0]), .dout(mitcnt1[31:0]));

   // ----------------------------------------------------------------------
   // MITB0 (RW)
   // [31:0] : Internal Timer Bound 0

   `define MITB0 12'h7d3

   assign wr_mitb0_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MITB0);

   rvdffe #(32) mitb0_ff      (.*, .en(wr_mitb0_wb), .din(~dec_i0_csr_wrdata_wb[31:0]), .dout(mitb0_b[31:0]));
   assign mitb0[31:0] = ~mitb0_b[31:0];

   // ----------------------------------------------------------------------
   // MITB1 (RW)
   // [31:0] : Internal Timer Bound 1

   `define MITB1 12'h7d6

   assign wr_mitb1_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MITB1);

   rvdffe #(32) mitb1_ff      (.*, .en(wr_mitb1_wb), .din(~dec_i0_csr_wrdata_wb[31:0]), .dout(mitb1_b[31:0]));
   assign mitb1[31:0] = ~mitb1_b[31:0];

   // ----------------------------------------------------------------------
   // MITCTL0 (RW) Internal Timer Ctl 0
   // [31:3] : Reserved, reads 0x0
   // [2]    : Enable while PAUSEd
   // [1]    : Enable while HALTed
   // [0]    : Enable (resets to 0x1)

   `define MITCTL0 12'h7d4

   assign wr_mitctl0_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MITCTL0);
   assign mitctl0_ns[2:0] = wr_mitctl0_wb ? {dec_i0_csr_wrdata_wb[2:0]} : {mitctl0[2:0]};

   assign mitctl0_0_b_ns = ~mitctl0_ns[0];
   rvdff #(3) mitctl0_ff      (.*, .clk(free_clk), .din({mitctl0_ns[2:1], mitctl0_0_b_ns}), .dout({mitctl0[2:1], mitctl0_0_b}));
   assign mitctl0[0] = ~mitctl0_0_b;

   // ----------------------------------------------------------------------
   // MITCTL1 (RW) Internal Timer Ctl 1
   // [31:4] : Reserved, reads 0x0
   // [3]    : Cascade
   // [2]    : Enable while PAUSEd
   // [1]    : Enable while HALTed
   // [0]    : Enable (resets to 0x1)

   `define MITCTL1 12'h7d7

   assign wr_mitctl1_wb = dec_i0_csr_wen_wb_mod & (dec_i0_csr_wraddr_wb[11:0] == `MITCTL1);
   assign mitctl1_ns[3:0] = wr_mitctl1_wb ? {dec_i0_csr_wrdata_wb[3:0]} : {mitctl1[3:0]};

   assign mitctl1_0_b_ns = ~mitctl1_ns[0];
   rvdff #(5) mitctl1_ff      (.*, .clk(free_clk), .din({mitctl1_ns[3:1], mitctl1_0_b_ns, mit0_match_ns}), .dout({mitctl1[3:1], mitctl1_0_b, mit0_match_d1}));
   assign mitctl1[0] = ~mitctl1_0_b;

   assign dec_timer_read_d = csr_rd.csr_mitcnt1 |
                             csr_rd.csr_mitcnt0 |
                             csr_rd.csr_mitb1 |
                             csr_rd.csr_mitb0 |
                             csr_rd.csr_mitctl0 |
                             csr_rd.csr_mitctl1;

   assign dec_timer_rddata_d[31:0] = ( ({32{csr_rd.csr_mitcnt0}}      & mitcnt0[31:0]) |
                                       ({32{csr_rd.csr_mitcnt1}}      & mitcnt1[31:0]) |
                                       ({32{csr_rd.csr_mitb0}}        & mitb0[31:0]) |
                                       ({32{csr_rd.csr_mitb1}}        & mitb1[31:0]) |
                                       ({32{csr_rd.csr_mitctl0}}      & {29'b0, mitctl0[2:0]}) |
                                       ({32{csr_rd.csr_mitctl1}}      & {28'b0, mitctl1[3:0]})
                                       );
   end // block: internal_timers
   else begin
      assign dec_timer_rddata_d[31:0] = 32'b0;
      assign dec_timer_read_d = 1'b0;
      assign dec_timer_t0_pulse = 1'b0;
      assign dec_timer_t1_pulse = 1'b0;
   end // else: !if(pt.TIMER_LEGAL_EN)

endmodule // dec_timer_ctl
