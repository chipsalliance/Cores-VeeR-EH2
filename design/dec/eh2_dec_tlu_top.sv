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
// eh2_dec_tlu_top.sv
//
//
// Function: Global CSRs, Commit/WB, thread management
// Comments:
//
//********************************************************************************

module eh2_dec_tlu_top
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

   input logic [31:1] rst_vec, // reset vector, from core pins
   input logic        nmi_int, // nmi pin
   input logic [31:1] nmi_vec, // nmi vector
   input logic  [pt.NUM_THREADS-1:0] i_cpu_halt_req,    // Asynchronous Halt request to CPU
   input logic  [pt.NUM_THREADS-1:0] i_cpu_run_req,     // Asynchronous Restart request to CPU

   input logic lsu_fastint_stall_any,   // needed by lsu for 2nd pass of dma with ecc correction, stall next cycle

   // perf counter inputs
   input logic [pt.NUM_THREADS-1:0][1:0] dec_pmu_instr_decoded,  // perf mon - decoded inst count
   input logic [pt.NUM_THREADS-1:0]    dec_pmu_decode_stall,     // perf mon - decode stall count
   input logic [pt.NUM_THREADS-1:0]    dec_pmu_presync_stall,    // perf mon - presync stall count
   input logic [pt.NUM_THREADS-1:0]    dec_pmu_postsync_stall,   // perf mon - postsync stall count
   input logic [pt.NUM_THREADS-1:0][1:0] ifu_pmu_instr_aligned,  // perf mon - inst aligned count
   input logic [pt.NUM_THREADS-1:0]      ifu_pmu_align_stall,    // perf mon - aligner stall count
   input logic [pt.NUM_THREADS-1:0]  lsu_pmu_load_external_dc3,  // perf mon - load count
   input logic [pt.NUM_THREADS-1:0]  lsu_pmu_store_external_dc3, // perf mon - store count
   input logic [pt.NUM_THREADS-1:0]  lsu_pmu_bus_trxn,           // perf mon - bus transaction count
   input logic [pt.NUM_THREADS-1:0]  lsu_pmu_bus_busy,           // perf mon - bus busy count
   input logic [pt.NUM_THREADS-1:0]  lsu_pmu_bus_misaligned,     // perf mon - bus misalign count
   input logic [pt.NUM_THREADS-1:0]  lsu_pmu_bus_error,          // perf mon - bus error count
   input logic [pt.NUM_THREADS-1:0] ifu_pmu_ic_miss,               // IC miss event
   input logic [pt.NUM_THREADS-1:0] ifu_pmu_ic_hit,                // IC hit event
   input logic [pt.NUM_THREADS-1:0] ifu_pmu_bus_error,             // Bus error event
   input logic [pt.NUM_THREADS-1:0] ifu_pmu_bus_busy,              // Bus busy event
   input logic [pt.NUM_THREADS-1:0] ifu_pmu_bus_trxn,              // Bus transaction
   input logic [pt.NUM_THREADS-1:0] ifu_pmu_fetch_stall, // perf mon - fetch stall count
   input logic       exu_pmu_i0_br_misp,     // pipe 0 branch misp
   input logic       exu_pmu_i0_br_ataken,   // pipe 0 branch actual taken
   input logic       exu_pmu_i0_pc4,         // pipe 0 4 byte branch
   input logic       exu_pmu_i1_br_misp,     // pipe 1 branch misp
   input logic       exu_pmu_i1_br_ataken,   // pipe 1 branch actual taken
   input logic       exu_pmu_i1_pc4,         // pipe 1 4 byte branch
   input logic       [pt.NUM_THREADS-1:0] lsu_store_stall_any,    // SB or WB is full, stall decode
   input logic       dma_dccm_stall_any,     // DMA stall of lsu
   input logic       dma_iccm_stall_any,     // DMA stall of ifu
   input logic       dma_pmu_dccm_read,          // DMA DCCM read
   input logic       dma_pmu_dccm_write,         // DMA DCCM write
   input logic       dma_pmu_any_read,           // DMA read
   input logic       dma_pmu_any_write,          // DMA write


   input logic [31:1] lsu_fir_addr, // Fast int address
   input logic [1:0]  lsu_fir_error, // Fast int lookup error

   input logic       iccm_dma_sb_error,      // I side dma single bit error
   input logic       lsu_single_ecc_error_incr,     // Increment the ecc error counter

   input eh2_trap_pkt_t dec_tlu_packet_e4, // exceptions known at decode (contains info for both pipes)
   input eh2_lsu_error_pkt_t lsu_error_pkt_dc3, // lsu precise exception/error packet

   input logic [pt.NUM_THREADS-1:0] dec_pause_state, // Pause counter not zero
   input logic [pt.NUM_THREADS-1:0] lsu_imprecise_error_store_any,      // store bus error
   input logic [pt.NUM_THREADS-1:0] lsu_imprecise_error_load_any,      // store bus error
   input logic [pt.NUM_THREADS-1:0][31:0]  lsu_imprecise_error_addr_any,   // LSU imprecise bus error address

   input logic dec_i0_tid_d, // pipe0 tid at decode

   input logic        dec_i0_csr_wen_unq_d,       // valid csr with write - for csr legal
   input logic        dec_i0_csr_any_unq_d,       // valid csr - for csr legal
   input logic        dec_i0_csr_wen_wb,      // csr write enable at wb
   input logic [11:0] dec_i0_csr_rdaddr_d,      // read address for csr
   input logic [11:0] dec_i0_csr_wraddr_wb,      // write address for csr
   input logic [31:0] dec_i0_csr_wrdata_wb,   // csr write data at wb
   input logic        dec_i0_csr_is_mcpc_e4,     // csr address is to MCPC

   input logic [pt.NUM_THREADS-1:0] dec_csr_stall_int_ff, // csr is mie/mstatus
   input logic dec_csr_nmideleg_e4, // csr is mnmipdel

   input logic dec_tlu_i0_valid_e4, // pipe 0 op at e4 is valid
   input logic dec_tlu_i1_valid_e4, // pipe 1 op at e4 is valid

   input logic [pt.NUM_THREADS-1:0] [31:1] exu_npc_e4, // for NPC tracking

   input logic [pt.NUM_THREADS-1:0] exu_i0_flush_lower_e4,       // pipe 0 branch mp flush
   input logic [pt.NUM_THREADS-1:0] exu_i1_flush_lower_e4,       // pipe 1 branch mp flush
   input logic [31:1] exu_i0_flush_path_e4, // pipe 0 correct path for mp, merge with lower path
   input logic [31:1] exu_i1_flush_path_e4, // pipe 1 correct path for mp, merge with lower path

   input logic [31:1] dec_tlu_i0_pc_e4, // for PC/NPC tracking
   input logic [31:1] dec_tlu_i1_pc_e4, // for PC/NPC tracking


   input logic [pt.NUM_THREADS-1:0] [31:0] dec_illegal_inst, // For mtval
   input logic        dec_i0_decode_d,  // decode valid, used for clean icache diagnostics

   // branch info from pipe0 for errors or counter updates
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] exu_i0_br_index_e4, // index
   input logic [1:0]  exu_i0_br_hist_e4, // history
   input logic        exu_i0_br_bank_e4, // bank
   input logic        exu_i0_br_error_e4, // error
   input logic        exu_i0_br_start_error_e4, // start error
   input logic        exu_i0_br_valid_e4, // valid
   input logic        exu_i0_br_mp_e4, // mispredict
   input logic        exu_i0_br_middle_e4, // middle of bank
   input logic [pt.BHT_GHR_SIZE-1:0] exu_i0_br_fghr_e4, // FGHR when predicted

   // branch info from pipe1 for errors or counter updates
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] exu_i1_br_index_e4, // index
   input logic [1:0]  exu_i1_br_hist_e4, // history
   input logic        exu_i1_br_bank_e4, // bank
   input logic        exu_i1_br_error_e4, // error
   input logic        exu_i1_br_start_error_e4, // start error
   input logic        exu_i1_br_valid_e4, // valid
   input logic        exu_i1_br_mp_e4, // mispredict
   input logic        exu_i1_br_middle_e4, // middle of bank
   input logic [pt.BHT_GHR_SIZE-1:0]  exu_i1_br_fghr_e4, // FGHR when predicted

   input logic        exu_i1_br_way_e4, // way hit or repl
   input logic        exu_i0_br_way_e4, // way hit or repl

   input  logic [pt.NUM_THREADS-1:0] dbg_halt_req, // DM requests a halt
   input  logic [pt.NUM_THREADS-1:0] dbg_resume_req, // DM requests a resume
   input  logic [pt.NUM_THREADS-1:0] ifu_miss_state_idle, // I-side miss buffer empty
   input  logic [pt.NUM_THREADS-1:0] lsu_idle_any, // lsu is idle
   input  logic                      dec_div_active, // oop divide is active
   input  logic                      dec_div_tid,    // oop divide tid

   input logic  [pt.NUM_THREADS-1:0] ifu_ic_error_start,     // IC single bit error
   input logic  [pt.NUM_THREADS-1:0] ifu_iccm_rd_ecc_single_err, // ICCM single bit error

   input logic [70:0] ifu_ic_debug_rd_data, // diagnostic icache read data
   input logic ifu_ic_debug_rd_data_valid, // diagnostic icache read data valid

   input logic [pt.NUM_THREADS-1:0] [7:0] pic_claimid, // pic claimid for csr
   input logic [pt.NUM_THREADS-1:0] [3:0] pic_pl, // pic priv level for csr
   input logic [pt.NUM_THREADS-1:0]       mhwakeup, // high priority external int, wakeup if halted

   input logic [pt.NUM_THREADS-1:0] mexintpend, // external interrupt pending
   input logic [pt.NUM_THREADS-1:0] timer_int, // timer interrupt pending
   input logic [pt.NUM_THREADS-1:0] soft_int,                             // Software interrupt pending (from pin)


   input logic [31:4]     core_id, // Core ID

   // external MPC halt/run interface
   input logic [pt.NUM_THREADS-1:0] mpc_debug_halt_req, // Async halt request
   input logic [pt.NUM_THREADS-1:0] mpc_debug_run_req, // Async run request
   input logic [pt.NUM_THREADS-1:0] mpc_reset_run_req, // Run/halt after reset

   output logic [pt.NUM_THREADS-1:0] dec_tlu_dbg_halted, // Core is halted and ready for debug command
   output logic [pt.NUM_THREADS-1:0] dec_tlu_debug_mode, // Core is in debug mode
   output logic dec_dbg_cmd_done, // abstract command done
   output logic dec_dbg_cmd_fail, // abstract command failed
   output logic dec_dbg_cmd_tid,  // Tid for debug abstract command response
   output logic [pt.NUM_THREADS-1:0] dec_tlu_resume_ack, // Resume acknowledge
   output logic [pt.NUM_THREADS-1:0] dec_tlu_debug_stall, // stall decode while waiting on core to empty
   output logic [pt.NUM_THREADS-1:0] dec_tlu_mpc_halted_only, // Core is halted only due to MPC
   output eh2_trigger_pkt_t [pt.NUM_THREADS-1:0] [3:0] trigger_pkt_any, // trigger info for trigger blocks

   output logic [pt.NUM_THREADS-1:0] dec_tlu_mhartstart, // thread 1 hartstart
   output logic [pt.NUM_THREADS-1:0] o_cpu_halt_status, // PMU interface, halted
   output logic [pt.NUM_THREADS-1:0] o_cpu_halt_ack, // halt req ack
   output logic [pt.NUM_THREADS-1:0] o_cpu_run_ack, // run req ack
   output logic [pt.NUM_THREADS-1:0] o_debug_mode_status, // Core to the PMU that core is in debug mode. When core is in debug mode, the PMU should refrain from sendng a halt or run request
   output logic [pt.NUM_THREADS-1:0] dec_tlu_force_halt, // forcing debug halt

   output eh2_cache_debug_pkt_t dec_tlu_ic_diag_pkt, // packet of DICAWICS, DICAD0/1, DICAGO info for icache diagnostics

   output logic [31:2] dec_tlu_meihap, // meihap for fast int

   // external MPC halt/run interface
   output logic [pt.NUM_THREADS-1:0] mpc_debug_halt_ack, // Halt ack
   output logic [pt.NUM_THREADS-1:0] mpc_debug_run_ack, // Run ack
   output logic [pt.NUM_THREADS-1:0] debug_brkpt_status, // debug breakpoint

   output logic [pt.NUM_THREADS-1:0] [3:0] dec_tlu_meicurpl, // to PIC
   output logic [pt.NUM_THREADS-1:0] [3:0] dec_tlu_meipt, // to PIC

   output eh2_br_tlu_pkt_t dec_tlu_br0_wb_pkt, // branch pkt to bp
   output eh2_br_tlu_pkt_t dec_tlu_br1_wb_pkt, // branch pkt to bp
   output logic [pt.BHT_GHR_SIZE-1:0] dec_tlu_br0_fghr_wb, // fghr to bp
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br0_index_wb, // bp index
   output logic [pt.BHT_GHR_SIZE-1:0] dec_tlu_br1_fghr_wb, // fghr to bp
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br1_index_wb, // bp index

   output logic [31:0] dec_i0_csr_rddata_d,      // csr read data at d
   output logic dec_i0_csr_legal_d,              // csr indicates legal operation
   output logic dec_i0_csr_global_d,             // global csr

   output logic dec_tlu_i0_kill_writeb_wb,    // I0 is flushed, don't writeback any results to arch state
   output logic dec_tlu_i1_kill_writeb_wb,    // I1 is flushed, don't writeback any results to arch state

   output logic [pt.NUM_THREADS-1:0] [31:1] dec_tlu_flush_path_wb,  // flush pc
   output logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_lower_wb, // commit has a flush (exception, int, mispredict at e4)
   output logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_noredir_wb , // Tell fetch to idle on this flush
   output logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_leak_one_wb, // single step
   output logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_err_wb, // iside perr/ecc rfpc
   output logic [pt.NUM_THREADS-1:0]        dec_tlu_flush_extint, // fast ext int started
   output logic [pt.NUM_THREADS-1:0]        dec_tlu_fence_i_wb,     // flush is a fence_i rfnpc, flush icache

   output logic [pt.NUM_THREADS-1:0] dec_tlu_presync_d,            // CSR read needs to be presync'd
   output logic [pt.NUM_THREADS-1:0] dec_tlu_postsync_d,           // CSR needs to be presync'd
   output logic [pt.NUM_THREADS-1:0] dec_tlu_i0_commit_cmt,        // goes to IFU for commit 1 instruction in the FSM
   output logic [31:0] dec_tlu_mrac_ff,        // CSR for memory region control

   output logic [pt.NUM_THREADS-1:0] dec_tlu_wr_pause_wb,           // CSR write to pause reg is at WB.
   output logic [pt.NUM_THREADS-1:0] dec_tlu_flush_pause_wb,        // Flush is due to pause
   output logic [pt.NUM_THREADS-1:0] dec_tlu_lr_reset_wb, // Reset the reservation on certain events

   // trace interface.
   output logic [pt.NUM_THREADS-1:0] dec_tlu_i0_valid_wb1,  // pipe 0 valid
   output logic [pt.NUM_THREADS-1:0] dec_tlu_i1_valid_wb1,  // pipe 1 valid
   output logic [pt.NUM_THREADS-1:0] dec_tlu_i0_exc_valid_wb1, // pipe 0 exception valid
   output logic [pt.NUM_THREADS-1:0] dec_tlu_i1_exc_valid_wb1, // pipe 1 exception valid
   output logic [pt.NUM_THREADS-1:0] dec_tlu_int_valid_wb1, // pipe 2 int valid
   output logic [pt.NUM_THREADS-1:0] [4:0] dec_tlu_exc_cause_wb1, // exception or int cause
   output logic [pt.NUM_THREADS-1:0] [31:0] dec_tlu_mtval_wb1, // MTVAL value

   output logic [pt.NUM_THREADS-1:0] [1:0] dec_tlu_perfcnt0, // toggles when pipe0 perf counter 0 has an event inc
   output logic [pt.NUM_THREADS-1:0] [1:0] dec_tlu_perfcnt1, // toggles when pipe0 perf counter 1 has an event inc
   output logic [pt.NUM_THREADS-1:0] [1:0] dec_tlu_perfcnt2, // toggles when pipe0 perf counter 2 has an event inc
   output logic [pt.NUM_THREADS-1:0] [1:0] dec_tlu_perfcnt3, // toggles when pipe0 perf counter 3 has an event inc

   // feature disable from mfdc
   output logic  dec_tlu_external_ldfwd_disable, // disable external load forwarding
   output logic  dec_tlu_sideeffect_posted_disable, // disable posted writes to side-effect address
   output logic  dec_tlu_dual_issue_disable, // disable dual issue
   output logic  dec_tlu_core_ecc_disable, // disable core ECC
   output logic  dec_tlu_bpred_disable,           // disable branch prediction
   output logic  dec_tlu_wb_coalescing_disable,   // disable writebuffer coalescing
   output logic  dec_tlu_pipelining_disable,      // disable pipelining
   output logic [2:0]  dec_tlu_dma_qos_prty,    // DMA QoS priority coming from MFDC [18:16]

   // clock gating overrides from mcgc
   output logic  dec_tlu_misc_clk_override, // override misc clock domain gating
   output logic  dec_tlu_dec_clk_override,  // override decode clock domain gating
   output logic  dec_tlu_exu_clk_override,  // override exu clock domain gating
   output logic  dec_tlu_ifu_clk_override,  // override fetch clock domain gating
   output logic  dec_tlu_lsu_clk_override,  // override load/store clock domain gating
   output logic  dec_tlu_bus_clk_override,  // override bus clock domain gating
   output logic  dec_tlu_pic_clk_override,  // override PIC clock domain gating
   output logic  dec_tlu_dccm_clk_override, // override DCCM clock domain gating
   output logic  dec_tlu_icm_clk_override   // override ICCM clock domain gating

   );


   eh2_cache_debug_pkt_t [pt.NUM_THREADS-1:0] dec_tlu_ic_diag_pkt_thr;
   logic nmi_int_sync, nmi_int_sync_raw;
   logic                      tlu_select_tid, tlu_select_tid_f, tlu_select_tid_f2, i0tid_wb, i1tid_wb, dec_i0_csr_tid_halted;
   logic [pt.NUM_THREADS-1:0] tlu_i0_valid_wb1, tlu_i1_valid_wb1, tlu_i0_exc_valid_wb1, tlu_i1_exc_valid_wb1, tlu_int_valid_wb1,
                              debug_brkpt_status_thr, mpc_debug_halt_ack_thr, mpc_debug_run_ack_thr, o_cpu_run_ack_thr,
                              o_cpu_halt_ack_thr, o_debug_mode_status_thr, br0_error_e4_thr,
                              br1_error_e4_thr, br0_start_error_e4_thr, br1_start_error_e4_thr, br0_mp_e4_thr,
                              pmu_i0_br_misp_thr, pmu_i0_br_ataken_thr, pmu_i0_pc4_thr, pmu_i1_br_misp_thr,
                              pmu_i1_br_ataken_thr, pmu_i1_pc4_thr, tlu_i0_kill_writeb_wb_thr, tlu_i1_kill_writeb_wb_thr,
                              dec_i0_csr_wen_wb_mod_thr, allow_dbg_halt_csr_write_thr, ic_perr_wb_thr, iccm_sbecc_wb_thr,
                              dec_tlu_dbg_halted_thr, dec_tlu_br0_error_e4_thr,
                              dec_tlu_br1_error_e4_thr, dec_tlu_br0_start_error_e4_thr, dec_tlu_br1_start_error_e4_thr,
                              tlu_i0_commit_cmt_thr, tlu_mpc_halted_only_thr, tlu_debug_stall_thr, dec_dbg_cmd_done_thr,
                              dec_dbg_cmd_fail_thr, dec_tlu_debug_mode_thr, dec_tlu_resume_ack_thr, tlu_fast_ext_int_ready;
   logic dec_tlu_br0_error_e4, dec_tlu_br0_start_error_e4, dec_tlu_br0_v_e4;
   logic dec_tlu_br1_error_e4, dec_tlu_br1_start_error_e4, dec_tlu_br1_v_e4;
   logic [pt.NUM_THREADS-1:0] [4:0] tlu_exc_cause_wb1;
   logic [pt.NUM_THREADS-1:0] [31:0] tlu_mtval_wb1, csr_rddata_d;
   logic [pt.NUM_THREADS-1:0] [31:2] dec_tlu_meihap_thr;

   logic        wr_mcgc_wb, wr_mfdc_wb, wr_mrac_wb, wr_mfdht_wb,
                wr_micect_wb, wr_miccmect_wb, miccmect_cout_nc,
                micect_cout_nc, wr_mdccmect_wb, mdccmect_cout_nc, wr_mhartstart_wb, wr_mnmipdel_wb,
                ignore_mnmipdel_wr, mnmipdel0_b, ic_perr_wb_all, iccm_sbecc_wb_all, dec_i0_tid_d_f;
   logic [5:0] mfdht, mfdht_ns;
   logic [8:0]  mcgc;
   logic [10:0] mfdc_ns, mfdc_int;
   logic [18:0] mfdc;
   logic [31:0] mrac_in, mrac;
   logic [31:0] micect_ns, micect, miccmect_ns,
                miccmect, mdccmect_ns, mdccmect, thread_csr_data_d;
   logic [26:0] miccmect_inc, micect_inc, mdccmect_inc;
   logic        mice_ce_req, miccme_ce_req, mdccme_ce_req;
   logic [1:1]  mhartstart_ns;
   logic [1:0]  mnmipdel_ns, mnmipdel, mhartstart;
   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br0_addr_e4, dec_tlu_br1_addr_e4;
   logic        dec_tlu_br0_bank_e4, dec_tlu_br1_bank_e4;
   logic         lsu_single_ecc_error_wb_ns;
   logic [31:27] csr_sat;
   logic [1:0] mhartnums;
   logic       tlu_i0_presync_d, tlu_i0_postsync_d, lsu_single_ecc_error_wb;

   assign dec_tlu_debug_mode[pt.NUM_THREADS-1:0] = dec_tlu_debug_mode_thr[pt.NUM_THREADS-1:0];
   assign dec_tlu_dbg_halted[pt.NUM_THREADS-1:0] = dec_tlu_dbg_halted_thr[pt.NUM_THREADS-1:0];
   assign dec_tlu_mpc_halted_only[pt.NUM_THREADS-1:0] = tlu_mpc_halted_only_thr[pt.NUM_THREADS-1:0];
   assign dec_tlu_resume_ack[pt.NUM_THREADS-1:0] = dec_tlu_resume_ack_thr[pt.NUM_THREADS-1:0];

   eh2_csr_tlu_pkt_t tlu_i0_csr_pkt_d;


   assign dec_i0_csr_tid_halted = dec_tlu_dbg_halted_thr[dec_i0_tid_d];

   eh2_dec_csr i0_csr_decoder(.dec_csr_rdaddr_d(dec_i0_csr_rdaddr_d[11:0]),
                               .dec_csr_any_unq_d(dec_i0_csr_any_unq_d),
                               .dec_csr_wen_unq_d(dec_i0_csr_wen_unq_d),
                               .dec_tlu_dbg_halted(dec_i0_csr_tid_halted),
                               // outputs
                               .tlu_csr_pkt_d(tlu_i0_csr_pkt_d),
                               .dec_csr_legal_d(dec_i0_csr_legal_d),
                               .tlu_presync_d(tlu_i0_presync_d),
                               .tlu_postsync_d(tlu_i0_postsync_d)
                               );

   assign dec_tlu_mhartstart[0] = mhartstart[0];

   // convert pipe signals to thread signals. SMT-ready
if(pt.NUM_THREADS > 1) begin : pipe2thr
   assign br0_error_e4_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i0tid, exu_i0_br_error_e4);
   assign br1_error_e4_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i1tid, exu_i1_br_error_e4);
   assign br0_mp_e4_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i0tid, exu_i0_br_mp_e4);
   assign br0_start_error_e4_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i0tid, exu_i0_br_start_error_e4);
   assign br1_start_error_e4_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i1tid, exu_i1_br_start_error_e4);
   assign pmu_i0_br_misp_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i0tid, exu_pmu_i0_br_misp);
   assign pmu_i0_br_ataken_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i0tid, exu_pmu_i0_br_ataken);
   assign pmu_i0_pc4_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i0tid, exu_pmu_i0_pc4);
   assign pmu_i1_br_misp_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i1tid, exu_pmu_i1_br_misp);
   assign pmu_i1_br_ataken_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i1tid, exu_pmu_i1_br_ataken);
   assign pmu_i1_pc4_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i1tid, exu_pmu_i1_pc4);
   assign dec_tlu_br0_error_e4_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i1tid, dec_tlu_br0_error_e4);
   assign dec_tlu_br1_error_e4_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i1tid, dec_tlu_br1_error_e4);
   assign dec_tlu_br0_start_error_e4_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i1tid, dec_tlu_br0_start_error_e4);
   assign dec_tlu_br1_start_error_e4_thr[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_tlu_packet_e4.i1tid, dec_tlu_br1_start_error_e4);

   assign dec_tlu_presync_d[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_i0_tid_d, tlu_i0_presync_d);
   assign dec_tlu_postsync_d[pt.NUM_THREADS-1:0] = pipe_to_thr(dec_i0_tid_d, tlu_i0_postsync_d);

   assign dec_tlu_mhartstart[1] = mhartstart[1];

   if(pt.FAST_INTERRUPT_REDIRECT)
     rvarbiter2 fastint_arbiter (
                                     .ready(tlu_fast_ext_int_ready[1:0]),
                                     .tid  (tlu_select_tid),
                                     .shift(&tlu_fast_ext_int_ready[1:0]),
                                     .*
                                     );
   else
     assign tlu_select_tid = 1'b0;


end
else begin
   assign tlu_select_tid = 1'b0;
   assign br0_error_e4_thr[pt.NUM_THREADS-1:0] = exu_i0_br_error_e4;
   assign br1_error_e4_thr[pt.NUM_THREADS-1:0] = exu_i1_br_error_e4;
   assign br0_mp_e4_thr[pt.NUM_THREADS-1:0] = exu_i0_br_mp_e4;
   assign br0_start_error_e4_thr[pt.NUM_THREADS-1:0] = exu_i0_br_start_error_e4;
   assign br1_start_error_e4_thr[pt.NUM_THREADS-1:0] = exu_i1_br_start_error_e4;
   assign pmu_i0_br_misp_thr[pt.NUM_THREADS-1:0] = exu_pmu_i0_br_misp;
   assign pmu_i0_br_ataken_thr[pt.NUM_THREADS-1:0] = exu_pmu_i0_br_ataken;
   assign pmu_i0_pc4_thr[pt.NUM_THREADS-1:0] = exu_pmu_i0_pc4;
   assign pmu_i1_br_misp_thr[pt.NUM_THREADS-1:0] = exu_pmu_i1_br_misp;
   assign pmu_i1_br_ataken_thr[pt.NUM_THREADS-1:0] = exu_pmu_i1_br_ataken;
   assign pmu_i1_pc4_thr[pt.NUM_THREADS-1:0] = exu_pmu_i1_pc4;
   assign dec_tlu_br0_error_e4_thr[pt.NUM_THREADS-1:0] = dec_tlu_br0_error_e4;
   assign dec_tlu_br1_error_e4_thr[pt.NUM_THREADS-1:0] = dec_tlu_br1_error_e4;
   assign dec_tlu_br0_start_error_e4_thr[pt.NUM_THREADS-1:0] = dec_tlu_br0_start_error_e4;
   assign dec_tlu_br1_start_error_e4_thr[pt.NUM_THREADS-1:0] = dec_tlu_br1_start_error_e4;

   assign dec_tlu_presync_d[pt.NUM_THREADS-1:0] = tlu_i0_presync_d;
   assign dec_tlu_postsync_d[pt.NUM_THREADS-1:0] = tlu_i0_postsync_d;
end // else: !if(pt.NUM_THREADS > 1)


   function [1:0] pipe_to_thr;
      input tid;
      input signal;
      begin
         pipe_to_thr[0] = signal & ~tid;
         pipe_to_thr[1] = signal & tid;
      end
   endfunction //

   rvsyncss #(1) syncro_ff(.*,
                           .clk(free_clk),
                           .din ({nmi_int    }),
                           .dout({nmi_int_sync_raw}));

   // If SW is writing the nmipdel register, hold off nmis for a cycle
   assign nmi_int_sync = nmi_int_sync_raw & ~dec_csr_nmideleg_e4;

   // ================================================================================
   // TID CSRs, Int, PC/NPC, Flush, FW HALT, Pause, Internal timers
   // ================================================================================
     for (genvar i=0; i<pt.NUM_THREADS; i++) begin : tlumt
        eh2_dec_tlu_ctl #(.pt(pt)) tlu (//inputs
                                         .mytid               (1'(i)),
                                         .dec_div_active(dec_div_active & (dec_div_tid == i)),
                                         .i_cpu_run_req(i_cpu_run_req[i] & mhartstart[i]),
                                         .i_cpu_halt_req(i_cpu_halt_req[i] & mhartstart[i]),
                                         .mpc_debug_halt_req(mpc_debug_halt_req[i] & mhartstart[i]),
                                         .mpc_debug_run_req(mpc_debug_run_req[i] & mhartstart[i]),
                                         .mpc_reset_run_req(mpc_reset_run_req[i]),
                                         .dbg_halt_req(dbg_halt_req[i] & mhartstart[i]),
                                         .dbg_resume_req(dbg_resume_req[i] & mhartstart[i]),
                                         .exu_npc_e4(exu_npc_e4[i]),
                                         .lsu_store_stall_any(lsu_store_stall_any[i]),
                                         .dec_tlu_br0_error_e4(dec_tlu_br0_error_e4_thr[i]),
                                         .dec_tlu_br1_error_e4(dec_tlu_br1_error_e4_thr[i]),
                                         .dec_tlu_br0_start_error_e4(dec_tlu_br0_start_error_e4_thr[i]),
                                         .dec_tlu_br1_start_error_e4(dec_tlu_br1_start_error_e4_thr[i]),
                                         .ifu_pmu_fetch_stall(ifu_pmu_fetch_stall[i]),
                                         .timer_int(timer_int[i]),
                                         .soft_int(soft_int[i]),
                                         .mexintpend(mexintpend[i]),
                                         .mhartstart_csr(mhartstart[i]),
                                         .ifu_miss_state_idle(ifu_miss_state_idle[i]),
                                         .dec_illegal_inst(dec_illegal_inst[i]),
                                         .lsu_imprecise_error_store_any(lsu_imprecise_error_store_any[i]),
                                         .lsu_imprecise_error_load_any(lsu_imprecise_error_load_any[i]),
                                         .lsu_imprecise_error_addr_any(lsu_imprecise_error_addr_any[i]),
                                         .dec_pause_state(dec_pause_state[i]),
                                         .nmi_int_sync(nmi_int_sync & mnmipdel_ns[i]),
                                         .exu_i0_flush_lower_e4(exu_i0_flush_lower_e4[i]),
                                         .exu_i1_flush_lower_e4(exu_i1_flush_lower_e4[i]),
                                         .lsu_idle_any(lsu_idle_any[i]),
                                         .ifu_ic_error_start(ifu_ic_error_start[i]),
                                         .ifu_iccm_rd_ecc_single_err(ifu_iccm_rd_ecc_single_err[i]),
                                         .lsu_pmu_load_external_dc3(lsu_pmu_load_external_dc3[i]),
                                         .lsu_pmu_store_external_dc3(lsu_pmu_store_external_dc3[i]),
                                         .lsu_pmu_bus_trxn(lsu_pmu_bus_trxn[i]),
                                         .lsu_pmu_bus_busy(lsu_pmu_bus_busy[i]),
                                         .lsu_pmu_bus_misaligned(lsu_pmu_bus_misaligned[i]),
                                         .lsu_pmu_bus_error(lsu_pmu_bus_error[i]),
                                         .dec_pmu_instr_decoded(dec_pmu_instr_decoded[i]),
                                         .dec_pmu_decode_stall(dec_pmu_decode_stall[i]),
                                         .dec_pmu_presync_stall(dec_pmu_presync_stall[i]),
                                         .dec_pmu_postsync_stall(dec_pmu_postsync_stall[i]),
                                         .ifu_pmu_instr_aligned(ifu_pmu_instr_aligned[i]),
                                         .ifu_pmu_align_stall(ifu_pmu_align_stall[i]),
                                         .ifu_pmu_ic_miss(ifu_pmu_ic_miss[i]),
                                         .ifu_pmu_ic_hit(ifu_pmu_ic_hit[i]),
                                         .ifu_pmu_bus_error(ifu_pmu_bus_error[i]),
                                         .ifu_pmu_bus_busy(ifu_pmu_bus_busy[i]),
                                         .ifu_pmu_bus_trxn(ifu_pmu_bus_trxn[i]),
                                         .dec_csr_stall_int_ff(dec_csr_stall_int_ff[i]),
                                         .pic_claimid(pic_claimid[i]),
                                         .pic_pl(pic_pl[i]),
                                         .mhwakeup(mhwakeup[i]),
                                         .exu_i0_br_start_error_e4(br0_start_error_e4_thr[i]),
                                         .exu_i1_br_start_error_e4(br1_start_error_e4_thr[i]),
                                         .exu_i0_br_error_e4(br0_error_e4_thr[i]),
                                         .exu_i1_br_error_e4(br1_error_e4_thr[i]),
                                         .exu_i0_br_mp_e4(br0_mp_e4_thr[i]),
                                         .exu_pmu_i0_br_misp(pmu_i0_br_misp_thr[i]),
                                         .exu_pmu_i0_br_ataken(pmu_i0_br_ataken_thr[i]),
                                         .exu_pmu_i0_pc4(pmu_i0_pc4_thr[i]),
                                         .exu_pmu_i1_br_misp(pmu_i1_br_misp_thr[i]),
                                         .exu_pmu_i1_br_ataken(pmu_i1_br_ataken_thr[i]),
                                         .exu_pmu_i1_pc4(pmu_i1_pc4_thr[i]),
                                         //outputs
                                         .tlu_perfcnt0(dec_tlu_perfcnt0[i]),
                                         .tlu_perfcnt1(dec_tlu_perfcnt1[i]),
                                         .tlu_perfcnt2(dec_tlu_perfcnt2[i]),
                                         .tlu_perfcnt3(dec_tlu_perfcnt3[i]),
                                         .dec_tlu_force_halt(dec_tlu_force_halt[i]),
                                         .dec_tlu_ic_diag_pkt(dec_tlu_ic_diag_pkt_thr[i]),
                                         .tlu_fast_ext_int_ready(tlu_fast_ext_int_ready[i]),
                                         .tlu_i0_commit_cmt(tlu_i0_commit_cmt_thr[i]),
                                         .tlu_i0_valid_wb1(dec_tlu_i0_valid_wb1[i]),
                                         .tlu_i1_valid_wb1(dec_tlu_i1_valid_wb1[i]),
                                         .tlu_i0_exc_valid_wb1(dec_tlu_i0_exc_valid_wb1[i]),
                                         .tlu_i1_exc_valid_wb1(dec_tlu_i1_exc_valid_wb1[i]),
                                         .tlu_int_valid_wb1(dec_tlu_int_valid_wb1[i]),
                                         .tlu_exc_cause_wb1(dec_tlu_exc_cause_wb1[i]),
                                         .tlu_mtval_wb1(dec_tlu_mtval_wb1[i]),
                                         .tlu_wr_pause_wb(dec_tlu_wr_pause_wb[i]),
                                         .tlu_flush_pause_wb(dec_tlu_flush_pause_wb[i]),
                                         .tlu_lr_reset_wb(dec_tlu_lr_reset_wb[i]),
                                         .tlu_meicurpl(dec_tlu_meicurpl[i]),
                                         .debug_brkpt_status(debug_brkpt_status[i]),
                                         .mpc_debug_halt_ack(mpc_debug_halt_ack[i]),
                                         .mpc_debug_run_ack(mpc_debug_run_ack[i]),
                                         .o_cpu_halt_status(o_cpu_halt_status[i]),
                                         .o_cpu_halt_ack(o_cpu_halt_ack[i]),
                                         .o_cpu_run_ack(o_cpu_run_ack[i]),
                                         .o_debug_mode_status(o_debug_mode_status[i]),
                                         .tlu_trigger_pkt_any(trigger_pkt_any[i]),
                                         .csr_rddata_d(csr_rddata_d[i]),
                                         .dec_tlu_meihap(dec_tlu_meihap_thr[i]),
                                         .tlu_meipt(dec_tlu_meipt[i]),
                                         .tlu_i0_kill_writeb_wb(tlu_i0_kill_writeb_wb_thr[i]),
                                         .tlu_i1_kill_writeb_wb(tlu_i1_kill_writeb_wb_thr[i]),
                                         .dec_i0_csr_wen_wb_mod(dec_i0_csr_wen_wb_mod_thr[i]),
                                         .allow_dbg_halt_csr_write(allow_dbg_halt_csr_write_thr[i]),
                                         .ic_perr_wb(ic_perr_wb_thr[i]),
                                         .iccm_sbecc_wb(iccm_sbecc_wb_thr[i]),
                                         .dec_tlu_debug_stall(dec_tlu_debug_stall[i]),
                                         .tlu_mpc_halted_only(tlu_mpc_halted_only_thr[i]),
                                         .dec_dbg_cmd_done(dec_dbg_cmd_done_thr[i]),
                                         .dec_dbg_cmd_fail(dec_dbg_cmd_fail_thr[i]),
                                         .dec_tlu_debug_mode(dec_tlu_debug_mode_thr[i]),
                                         .dec_tlu_resume_ack(dec_tlu_resume_ack_thr[i]),
                                         .dec_tlu_flush_path_wb(dec_tlu_flush_path_wb[i]),
                                         .dec_tlu_flush_lower_wb(dec_tlu_flush_lower_wb[i]),
                                         .dec_tlu_flush_noredir_wb(dec_tlu_flush_noredir_wb[i]),
                                         .dec_tlu_flush_leak_one_wb(dec_tlu_flush_leak_one_wb[i]),
                                         .dec_tlu_flush_err_wb(dec_tlu_flush_err_wb[i]),
                                         .dec_tlu_flush_extint(dec_tlu_flush_extint[i]),
                                         .dec_tlu_fence_i_wb(dec_tlu_fence_i_wb[i]),
                                         .dec_tlu_dbg_halted(dec_tlu_dbg_halted_thr[i]),
                                         .*);
     end

   assign dec_tlu_meihap = dec_tlu_meihap_thr[tlu_select_tid_f2];

   assign dec_tlu_ic_diag_pkt = dec_tlu_ic_diag_pkt_thr[dec_i0_tid_d_f];

   // tid specific signals to pipe specific conversion
   assign dec_tlu_i0_kill_writeb_wb = |tlu_i0_kill_writeb_wb_thr[pt.NUM_THREADS-1:0];
   assign dec_tlu_i1_kill_writeb_wb = |tlu_i1_kill_writeb_wb_thr[pt.NUM_THREADS-1:0];
   assign dec_tlu_i0_commit_cmt[pt.NUM_THREADS-1:0] = tlu_i0_commit_cmt_thr[pt.NUM_THREADS-1:0];

   assign dec_dbg_cmd_tid = ~dec_dbg_cmd_done_thr[0];
   assign dec_dbg_cmd_done = |dec_dbg_cmd_done_thr[pt.NUM_THREADS-1:0];
   assign dec_dbg_cmd_fail = |dec_dbg_cmd_fail_thr[pt.NUM_THREADS-1:0];
   assign ic_perr_wb_all = |ic_perr_wb_thr[pt.NUM_THREADS-1:0];
   assign iccm_sbecc_wb_all = |iccm_sbecc_wb_thr[pt.NUM_THREADS-1:0];

   // ================================================================================
   // Commit
   // ================================================================================
   // Branch prediction updating
   assign dec_tlu_br0_addr_e4[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] = exu_i0_br_index_e4[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];
   assign dec_tlu_br0_bank_e4 = exu_i0_br_bank_e4;
   assign dec_tlu_br1_addr_e4[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] = exu_i1_br_index_e4[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];
   assign dec_tlu_br1_bank_e4 = exu_i1_br_bank_e4;
     rvdff #(pt.BHT_GHR_SIZE*2)   bp_wb_ghrff (.*,  .clk(active_clk),
                                               .din({exu_i0_br_fghr_e4[pt.BHT_GHR_SIZE-1:0],
                                                     exu_i1_br_fghr_e4[pt.BHT_GHR_SIZE-1:0]
                                                     }),
                                              .dout({dec_tlu_br0_fghr_wb[pt.BHT_GHR_SIZE-1:0],
                                                     dec_tlu_br1_fghr_wb[pt.BHT_GHR_SIZE-1:0]
                                                     }));

   rvdff #(2*$bits(dec_tlu_br0_addr_e4[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]))
        bp_wb_index_ff (.*,  .clk(active_clk),
                            .din({dec_tlu_br0_addr_e4[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO],
                                  dec_tlu_br1_addr_e4[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]}),
                           .dout({dec_tlu_br0_index_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO],
                                  dec_tlu_br1_index_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]}));

   // go ahead and repair the branch error on other flushes, doesn't have to be the rfpc flush
   assign dec_tlu_br0_error_e4 = exu_i0_br_error_e4 & dec_tlu_i0_valid_e4 & ~dec_tlu_flush_lower_wb[i0tid_wb];
   assign dec_tlu_br0_start_error_e4 = exu_i0_br_start_error_e4 & dec_tlu_i0_valid_e4 & ~dec_tlu_flush_lower_wb[i0tid_wb];
   assign dec_tlu_br0_v_e4 = exu_i0_br_valid_e4 & dec_tlu_i0_valid_e4 & ~dec_tlu_flush_lower_wb[i0tid_wb] & ~exu_i0_br_mp_e4;

   assign dec_tlu_br1_error_e4 = exu_i1_br_error_e4 & dec_tlu_i1_valid_e4 & ~dec_tlu_flush_lower_wb[i1tid_wb] & ~exu_i0_br_mp_e4;
   assign dec_tlu_br1_start_error_e4 = exu_i1_br_start_error_e4 & dec_tlu_i1_valid_e4 & ~dec_tlu_flush_lower_wb[i1tid_wb] & ~exu_i0_br_mp_e4;
   assign dec_tlu_br1_v_e4 = exu_i1_br_valid_e4 & ~dec_tlu_flush_lower_wb[i1tid_wb] & dec_tlu_i1_valid_e4 & ~exu_i0_br_mp_e4 & ~exu_i1_br_mp_e4;

   rvdff #(21) bp_wb_ff (.*, .clk(active_clk),
                            .din({tlu_select_tid,
                                  tlu_select_tid_f,
                                  dec_tlu_packet_e4.i0tid,
                                  dec_tlu_packet_e4.i1tid,
                                  dec_i0_tid_d,
                                  exu_i0_br_hist_e4[1:0],
                                  dec_tlu_br0_error_e4,
                                  dec_tlu_br0_start_error_e4,
                                  dec_tlu_br0_v_e4,
                                  exu_i1_br_hist_e4[1:0],
                                  dec_tlu_br1_error_e4,
                                  dec_tlu_br1_start_error_e4,
                                  dec_tlu_br1_v_e4,
                                  dec_tlu_br0_bank_e4,
                                  dec_tlu_br1_bank_e4,
                                  exu_i0_br_way_e4,
                                  exu_i1_br_way_e4,
                                  exu_i0_br_middle_e4,
                                  exu_i1_br_middle_e4
                                  }),
                           .dout({tlu_select_tid_f,
                                  tlu_select_tid_f2,
                                  i0tid_wb,
                                  i1tid_wb,
                                  dec_i0_tid_d_f,
                                  dec_tlu_br0_wb_pkt.hist[1:0],
                                  dec_tlu_br0_wb_pkt.br_error,
                                  dec_tlu_br0_wb_pkt.br_start_error,
                                  dec_tlu_br0_wb_pkt.valid,
                                  dec_tlu_br1_wb_pkt.hist[1:0],
                                  dec_tlu_br1_wb_pkt.br_error,
                                  dec_tlu_br1_wb_pkt.br_start_error,
                                  dec_tlu_br1_wb_pkt.valid,
                                  dec_tlu_br0_wb_pkt.bank,
                                  dec_tlu_br1_wb_pkt.bank,
                                  dec_tlu_br0_wb_pkt.way,
                                  dec_tlu_br1_wb_pkt.way,
                                  dec_tlu_br0_wb_pkt.middle,
                                  dec_tlu_br1_wb_pkt.middle
                                  }));
   // ================================================================================
   // Global core CSRs
   // ================================================================================

   // ----------------------------------------------------------------------
   // MCGC (RW) Clock gating control
   // [31:9] : Reserved, reads 0x0
   // [8]    : misc_clk_override
   // [7]    : dec_clk_override
   // [6]    : exu_clk_override
   // [5]    : ifu_clk_override
   // [4]    : lsu_clk_override
   // [3]    : bus_clk_override
   // [2]    : pic_clk_override
   // [1]    : dccm_clk_override
   // [0]    : icm_clk_override
   //
   `define MCGC 12'h7f8
   assign wr_mcgc_wb = dec_i0_csr_wen_wb_mod_thr[i0tid_wb] & (dec_i0_csr_wraddr_wb[11:0] == `MCGC);

   rvdffe #(9)  mcgc_ff (.*, .en(wr_mcgc_wb), .din(dec_i0_csr_wrdata_wb[8:0]), .dout(mcgc[8:0]));

   assign dec_tlu_misc_clk_override = mcgc[8];
   assign dec_tlu_dec_clk_override  = mcgc[7];
   assign dec_tlu_exu_clk_override  = mcgc[6];
   assign dec_tlu_ifu_clk_override  = mcgc[5];
   assign dec_tlu_lsu_clk_override  = mcgc[4];
   assign dec_tlu_bus_clk_override  = mcgc[3];
   assign dec_tlu_pic_clk_override  = mcgc[2];
   assign dec_tlu_dccm_clk_override = mcgc[1];
   assign dec_tlu_icm_clk_override  = mcgc[0];

   // ----------------------------------------------------------------------
   // MFDC (RW) Feature Disable Control
   // [31:19] : Reserved, reads 0x0
   // [18:16] : DMA QoS Prty
   // [15:12] : Reserved, reads 0x0
   // [11]   : Disable external load forwarding
   // [10]   : Disable dual issue
   // [9]    : Unused, reads 0x0
   // [8]    : Disable core ecc
   // [7]    : Unused, reads 0x0
   // [6]    : Disable side effect posting
   // [5:4]  : Unused, reads 0x0
   // [3]    : Disable branch prediction and return stack
   // [2]    : Disable write buffer coalescing
   // [1]    : Unused, reads 0x0
   // [0]    : Disable pipelining - Enable single instruction execution
   //
   `define MFDC 12'h7f9

   assign wr_mfdc_wb = dec_i0_csr_wen_wb_mod_thr[i0tid_wb] & (dec_i0_csr_wraddr_wb[11:0] == `MFDC);

   rvdffe #(11)  mfdc_ff (.*, .en(wr_mfdc_wb), .din(mfdc_ns[10:0]), .dout(mfdc_int[10:0]));


if (pt.BUILD_AXI4 == 1) begin
   // flip poweron value of bit 6 for AXI build
   assign mfdc_ns[10:0] = {~dec_i0_csr_wrdata_wb[18:16],dec_i0_csr_wrdata_wb[11:8], ~dec_i0_csr_wrdata_wb[6], dec_i0_csr_wrdata_wb[3:2], dec_i0_csr_wrdata_wb[0]};
   assign mfdc[18:0] = {~mfdc_int[10:8], 4'b0, mfdc_int[7:4], 1'b0, ~mfdc_int[3], 2'b0, mfdc_int[2:1], 1'b0, mfdc_int[0]};
end
else begin
   assign mfdc_ns[10:0] = {~dec_i0_csr_wrdata_wb[18:16],dec_i0_csr_wrdata_wb[11:8], dec_i0_csr_wrdata_wb[6], dec_i0_csr_wrdata_wb[3:2], dec_i0_csr_wrdata_wb[0]};
   assign mfdc[18:0] = {~mfdc_int[10:8], 4'b0, mfdc_int[7:4], 1'b0, mfdc_int[3], 2'b0, mfdc_int[2:1], 1'b0, mfdc_int[0]};
end

   assign dec_tlu_dma_qos_prty[2:0] = mfdc[18:16];
   assign dec_tlu_external_ldfwd_disable = mfdc[11];
   assign dec_tlu_dual_issue_disable = mfdc[10];
   assign dec_tlu_core_ecc_disable = mfdc[8];
   assign dec_tlu_sideeffect_posted_disable = mfdc[6];
   assign dec_tlu_bpred_disable = mfdc[3];
   assign dec_tlu_wb_coalescing_disable = mfdc[2];
   assign dec_tlu_pipelining_disable = mfdc[0];

   // ----------------------------------------------------------------------
   // MRAC (RW)
   // [31:0] : Region Access Control Register, 16 regions, {side_effect, cachable} pairs
   `define MRAC 12'h7c0

   assign wr_mrac_wb = dec_i0_csr_wen_wb_mod_thr[i0tid_wb] & (dec_i0_csr_wraddr_wb[11:0] == `MRAC);

   // prevent pairs of 0x11, side_effect and cacheable
   assign mrac_in[31:0] = {dec_i0_csr_wrdata_wb[31], dec_i0_csr_wrdata_wb[30] & ~dec_i0_csr_wrdata_wb[31],
                           dec_i0_csr_wrdata_wb[29], dec_i0_csr_wrdata_wb[28] & ~dec_i0_csr_wrdata_wb[29],
                           dec_i0_csr_wrdata_wb[27], dec_i0_csr_wrdata_wb[26] & ~dec_i0_csr_wrdata_wb[27],
                           dec_i0_csr_wrdata_wb[25], dec_i0_csr_wrdata_wb[24] & ~dec_i0_csr_wrdata_wb[25],
                           dec_i0_csr_wrdata_wb[23], dec_i0_csr_wrdata_wb[22] & ~dec_i0_csr_wrdata_wb[23],
                           dec_i0_csr_wrdata_wb[21], dec_i0_csr_wrdata_wb[20] & ~dec_i0_csr_wrdata_wb[21],
                           dec_i0_csr_wrdata_wb[19], dec_i0_csr_wrdata_wb[18] & ~dec_i0_csr_wrdata_wb[19],
                           dec_i0_csr_wrdata_wb[17], dec_i0_csr_wrdata_wb[16] & ~dec_i0_csr_wrdata_wb[17],
                           dec_i0_csr_wrdata_wb[15], dec_i0_csr_wrdata_wb[14] & ~dec_i0_csr_wrdata_wb[15],
                           dec_i0_csr_wrdata_wb[13], dec_i0_csr_wrdata_wb[12] & ~dec_i0_csr_wrdata_wb[13],
                           dec_i0_csr_wrdata_wb[11], dec_i0_csr_wrdata_wb[10] & ~dec_i0_csr_wrdata_wb[11],
                           dec_i0_csr_wrdata_wb[9], dec_i0_csr_wrdata_wb[8] & ~dec_i0_csr_wrdata_wb[9],
                           dec_i0_csr_wrdata_wb[7], dec_i0_csr_wrdata_wb[6] & ~dec_i0_csr_wrdata_wb[7],
                           dec_i0_csr_wrdata_wb[5], dec_i0_csr_wrdata_wb[4] & ~dec_i0_csr_wrdata_wb[5],
                           dec_i0_csr_wrdata_wb[3], dec_i0_csr_wrdata_wb[2] & ~dec_i0_csr_wrdata_wb[3],
                           dec_i0_csr_wrdata_wb[1], dec_i0_csr_wrdata_wb[0] & ~dec_i0_csr_wrdata_wb[1]};

   rvdffe #(32)  mrac_ff (.*, .en(wr_mrac_wb), .din(mrac_in[31:0]), .dout(mrac[31:0]));

   // drive to LSU/IFU
   assign dec_tlu_mrac_ff[31:0] = mrac[31:0];
   // ----------------------------------------------------------------------
   // MICECT (I-Cache error counter/threshold)
   // [31:27] : Icache parity error threshold
   // [26:0]  : Icache parity error count
   `define MICECT 12'h7f0

   assign csr_sat[31:27] = (dec_i0_csr_wrdata_wb[31:27] > 5'd26) ? 5'd26 : dec_i0_csr_wrdata_wb[31:27];

   assign wr_micect_wb = dec_i0_csr_wen_wb_mod_thr[i0tid_wb] & (dec_i0_csr_wraddr_wb[11:0] == `MICECT);
   assign {micect_cout_nc, micect_inc[26:0]} = micect[26:0] + {26'b0, ic_perr_wb_all};
   assign micect_ns =  wr_micect_wb ? {csr_sat[31:27], dec_i0_csr_wrdata_wb[26:0]} : {micect[31:27], micect_inc[26:0]};

   rvdffe #(32)  micect_ff (.*, .en(wr_micect_wb | ic_perr_wb_all), .din(micect_ns[31:0]), .dout(micect[31:0]));

   assign mice_ce_req = |({32'hffffffff << micect[31:27]} & {5'b0, micect[26:0]});

   // ----------------------------------------------------------------------
   // MICCMECT (ICCM error counter/threshold)
   // [31:27] : ICCM parity error threshold
   // [26:0]  : ICCM parity error count
   `define MICCMECT 12'h7f1

   assign wr_miccmect_wb = dec_i0_csr_wen_wb_mod_thr[i0tid_wb] & (dec_i0_csr_wraddr_wb[11:0] == `MICCMECT);
   assign {miccmect_cout_nc, miccmect_inc[26:0]} = miccmect[26:0] + {26'b0, iccm_sbecc_wb_all | iccm_dma_sb_error};
   assign miccmect_ns =  wr_miccmect_wb ? {csr_sat[31:27], dec_i0_csr_wrdata_wb[26:0]} : {miccmect[31:27], miccmect_inc[26:0]};

   rvdffe #(32)  miccmect_ff (.*, .en(wr_miccmect_wb | iccm_sbecc_wb_all | iccm_dma_sb_error), .din(miccmect_ns[31:0]), .dout(miccmect[31:0]));

   assign miccme_ce_req = |({32'hffffffff << miccmect[31:27]} & {5'b0, miccmect[26:0]});

   // ----------------------------------------------------------------------
   // MDCCMECT (DCCM error counter/threshold)
   // [31:27] : DCCM parity error threshold
   // [26:0]  : DCCM parity error count
   `define MDCCMECT 12'h7f2

   assign lsu_single_ecc_error_wb_ns = lsu_single_ecc_error_incr;
   rvdff #(1) lsu_dccm_errorff (.*, .clk(free_clk), .din({lsu_single_ecc_error_wb_ns}),
                                                   .dout({lsu_single_ecc_error_wb}));

   assign wr_mdccmect_wb = dec_i0_csr_wen_wb_mod_thr[i0tid_wb] & (dec_i0_csr_wraddr_wb[11:0] == `MDCCMECT);
   assign {mdccmect_cout_nc, mdccmect_inc[26:0]} = mdccmect[26:0] + {26'b0, lsu_single_ecc_error_wb};
   assign mdccmect_ns =  wr_mdccmect_wb ? {csr_sat[31:27], dec_i0_csr_wrdata_wb[26:0]} : {mdccmect[31:27], mdccmect_inc[26:0]};

   rvdffe #(32)  mdccmect_ff (.*, .en(wr_mdccmect_wb | lsu_single_ecc_error_wb), .din(mdccmect_ns[31:0]), .dout(mdccmect[31:0]));

   assign mdccme_ce_req = |({32'hffffffff << mdccmect[31:27]} & {5'b0, mdccmect[26:0]});

   // ----------------------------------------------------------------------
   // MFDHT (Force Debug Halt Threshold)
   // [5:1] : Halt timeout threshold (power of 2)
   //   [0] : Halt timeout enabled
   `define MFDHT 12'h7ce

   assign wr_mfdht_wb = dec_i0_csr_wen_wb_mod_thr[i0tid_wb] & (dec_i0_csr_wraddr_wb[11:0] == `MFDHT);

   assign mfdht_ns[5:0] = wr_mfdht_wb ? dec_i0_csr_wrdata_wb[5:0] : mfdht[5:0];

   rvdff #(6)  mfdht_ff (.*, .clk(active_clk), .din(mfdht_ns[5:0]), .dout(mfdht[5:0]));


   // ----------------------------------------------------------------------
   // MHARTSTART (Write 1 only)
   // [31:2] : Reserved
   // [1]    : Start thread 1
   // [0]    : Start thread 0 (Resets to 0x1)
   `define MHARTSTART 12'h7fc

   assign wr_mhartstart_wb = dec_i0_csr_wen_wb_mod_thr[i0tid_wb] & (dec_i0_csr_wraddr_wb[11:0] == `MHARTSTART);

   if (pt.NUM_THREADS > 1)
     assign mhartstart_ns[1] =  wr_mhartstart_wb ? (dec_i0_csr_wrdata_wb[1] | mhartstart[1]) : mhartstart[1];
   else
     assign mhartstart_ns[1] =  'b0;

   rvdff #(1)  mhartstart_ff (.*, .clk(active_clk), .din(mhartstart_ns[1]), .dout(mhartstart[1]));
   assign mhartstart[0] = 1'b1;

   // ----------------------------------------------------------------------
   // MNMIPDEL (Legal values: 01, 10, 11.
   // [31:2] : Reserved
   // [1]    : Delegate NMI pin to thread 1
   // [0]    : Delegate NMI pin to thread 0 (Resets to 0x1)
   `define MNMIPDEL 12'h7fe

   assign wr_mnmipdel_wb = dec_i0_csr_wen_wb_mod_thr[i0tid_wb] & (dec_i0_csr_wraddr_wb[11:0] == `MNMIPDEL);

   if(pt.NUM_THREADS == 1)
     assign ignore_mnmipdel_wr = 1'b1;
   else
     assign ignore_mnmipdel_wr = &(~dec_i0_csr_wrdata_wb[1:0]);

   assign mnmipdel_ns[1:0] =  (wr_mnmipdel_wb & ~ignore_mnmipdel_wr) ? dec_i0_csr_wrdata_wb[1:0] : mnmipdel[1:0];

   rvdff #(2)  mnmipdel_ff (.*, .clk(active_clk), .din({mnmipdel_ns[1], ~mnmipdel_ns[0]}), .dout({mnmipdel[1], mnmipdel0_b}));
   assign mnmipdel[0] = ~mnmipdel0_b;


   // Thread mux, if required
   if (pt.NUM_THREADS > 1) begin: tlutop
      assign thread_csr_data_d[31:0] = ( ({32{~dec_i0_tid_d}} & csr_rddata_d[0]) |
                                         ({32{ dec_i0_tid_d}} & csr_rddata_d[1]) );
      assign mhartnums[1:0] = 2'b10;
   end
   else begin
      assign thread_csr_data_d[31:0] =  csr_rddata_d[dec_i0_tid_d];
      assign mhartnums[1:0] = 2'b01;
   end
   assign dec_i0_csr_global_d = tlu_i0_csr_pkt_d.glob;

   // Final CSR mux
   assign dec_i0_csr_rddata_d[31:0] = ( // global csrs
                                     ({32{tlu_i0_csr_pkt_d.csr_misa}}       & 32'h40001105) |
                                     ({32{tlu_i0_csr_pkt_d.csr_mvendorid}}  & 32'h00000045) |
                                     ({32{tlu_i0_csr_pkt_d.csr_marchid}}    & 32'h00000011) |
                                     ({32{tlu_i0_csr_pkt_d.csr_mimpid}}     & 32'h2) |
                                     ({32{tlu_i0_csr_pkt_d.csr_mhartnum}}   & {30'h0, mhartnums[1:0]}) |
                                     ({32{tlu_i0_csr_pkt_d.csr_mrac}}       & mrac[31:0]) |
                                     ({32{tlu_i0_csr_pkt_d.csr_mcgc}}       & {23'b0, mcgc[8:0]}) |
                                     ({32{tlu_i0_csr_pkt_d.csr_mfdc}}       & {13'b0, mfdc[18:0]}) |
                                     ({32{tlu_i0_csr_pkt_d.csr_micect}}     & {micect[31:0]}) |
                                     ({32{tlu_i0_csr_pkt_d.csr_miccmect}}   & {miccmect[31:0]}) |
                                     ({32{tlu_i0_csr_pkt_d.csr_mdccmect}}   & {mdccmect[31:0]}) |
                                     ({32{tlu_i0_csr_pkt_d.csr_mfdht  }}    & {26'b0, mfdht[5:0]}) |
                                     ({32{tlu_i0_csr_pkt_d.csr_mhartstart}} & {30'b0, mhartstart[1:0]}) |
                                     ({32{tlu_i0_csr_pkt_d.csr_mnmipdel}}   & {30'b0, mnmipdel[1:0]}) |
                                     // threaded csrs
                                     ({32{~tlu_i0_csr_pkt_d.glob}} & thread_csr_data_d[31:0])
                                     );

endmodule
