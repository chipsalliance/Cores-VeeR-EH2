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


module eh2_exu
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (

   input logic                                   clk,                          // Top level clock
   input logic                                   active_clk,                   // Level 1 active clock
   input logic                                   clk_override,                 // Override multiply clock enables
   input logic                                   rst_l,                        // Reset
   input logic                                   scan_mode,                    // Scan control

   input logic                                   dec_extint_stall,             // External interrupt mux select
   input logic                      [31:2]       dec_tlu_meihap,               // External interrupt mux data

   input logic [4:2]                             dec_i0_data_en,               // Slot I0 clock enable {e1, e2, e3    }, one cycle pulse
   input logic [4:1]                             dec_i0_ctl_en,                // Slot I0 clock enable {e1, e2, e3, e4}, two cycle pulse
   input logic [4:2]                             dec_i1_data_en,               // Slot I1 clock enable {e1, e2, e3    }, one cycle pulse
   input logic [4:1]                             dec_i1_ctl_en,                // Slot I1 clock enable {e1, e2, e3, e4}, two cycle pulse

   input logic                                   dec_debug_wdata_rs1_d,        // Debug select to primary I0 RS1

   input logic [31:0]                            dbg_cmd_wrdata,               // Debug data   to primary I0 RS1

   input logic [31:0]                            lsu_result_dc3,               // Load result

   input eh2_predict_pkt_t                      i0_predict_p_d,               // DEC branch predict packet
   input eh2_predict_pkt_t                      i1_predict_p_d,               // DEC branch predict packet
   input logic [pt.BHT_GHR_SIZE-1:0]             i0_predict_fghr_d,            // DEC predict fghr
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]   i0_predict_index_d,           // DEC predict index
   input logic [pt.BTB_BTAG_SIZE-1:0]            i0_predict_btag_d,            // DEC predict branch tag
   input logic [pt.BHT_GHR_SIZE-1:0]             i1_predict_fghr_d,            // DEC predict fghr
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]   i1_predict_index_d,           // DEC predict index
   input logic [pt.BTB_BTAG_SIZE-1:0]            i1_predict_btag_d,            // DEC predict branch tag

   input logic                                   dec_i0_rs1_bypass_en_e2,      // DEC bypass bus select for E2 stage
   input logic                                   dec_i0_rs2_bypass_en_e2,      // DEC bypass bus select for E2 stage
   input logic                                   dec_i1_rs1_bypass_en_e2,      // DEC bypass bus select for E2 stage
   input logic                                   dec_i1_rs2_bypass_en_e2,      // DEC bypass bus select for E2 stage
   input logic [31:0]                            i0_rs1_bypass_data_e2,        // DEC bypass bus
   input logic [31:0]                            i0_rs2_bypass_data_e2,        // DEC bypass bus
   input logic [31:0]                            i1_rs1_bypass_data_e2,        // DEC bypass bus
   input logic [31:0]                            i1_rs2_bypass_data_e2,        // DEC bypass bus

   input logic                                   dec_i0_rs1_bypass_en_e3,      // DEC bypass bus select for E3 stage
   input logic                                   dec_i0_rs2_bypass_en_e3,      // DEC bypass bus select for E3 stage
   input logic                                   dec_i1_rs1_bypass_en_e3,      // DEC bypass bus select for E3 stage
   input logic                                   dec_i1_rs2_bypass_en_e3,      // DEC bypass bus select for E3 stage
   input logic [31:0]                            i0_rs1_bypass_data_e3,        // DEC bypass bus
   input logic [31:0]                            i0_rs2_bypass_data_e3,        // DEC bypass bus
   input logic [31:0]                            i1_rs1_bypass_data_e3,        // DEC bypass bus
   input logic [31:0]                            i1_rs2_bypass_data_e3,        // DEC bypass bus

   input logic                                   dec_i0_sec_decode_e3,         // Secondary ALU valid
   input logic                                   dec_i1_sec_decode_e3,         // Secondary ALU valid
   input logic [31:1]                            dec_i0_pc_e3,                 // Secondary ALU PC
   input logic [31:1]                            dec_i1_pc_e3,                 // Secondary ALU PC

   input logic [pt.NUM_THREADS-1:0][31:1]        pred_correct_npc_e2,          // npc e2 if the prediction is correct

   input logic                                   dec_i1_valid_e1,              // I1 valid E1

   input logic                                   dec_i0_mul_d,                 // Select for Multiply GPR value
   input logic                                   dec_i1_mul_d,                 // Select for Multiply GPR value

   input logic                                   dec_i0_div_d,                 // Select for Divide GPR value
   input logic                                   dec_div_cancel,               // Cancel divide operation due to write-after-write

   input logic [31:0]                            gpr_i0_rs1_d,                 // DEC data gpr
   input logic [31:0]                            gpr_i0_rs2_d,                 // DEC data gpr
   input logic [31:0]                            dec_i0_immed_d,               // DEC data immediate

   input logic [31:0]                            gpr_i1_rs1_d,                 // DEC data gpr
   input logic [31:0]                            gpr_i1_rs2_d,                 // DEC data gpr
   input logic [31:0]                            dec_i1_immed_d,               // DEC data immediate

   input logic [31:0]                            i0_rs1_bypass_data_d,         // DEC bypass data
   input logic [31:0]                            i0_rs2_bypass_data_d,         // DEC bypass data
   input logic [31:0]                            i1_rs1_bypass_data_d,         // DEC bypass data
   input logic [31:0]                            i1_rs2_bypass_data_d,         // DEC bypass data

   input logic [12:1]                            dec_i0_br_immed_d,            // Branch immediate
   input logic [12:1]                            dec_i1_br_immed_d,            // Branch immediate

   input logic                                   dec_i0_lsu_d,                 // Bypass control for LSU operand bus
   input logic                                   dec_i1_lsu_d,                 // Bypass control for LSU operand bus

   input logic                                   dec_i0_csr_ren_d,             // Clear I0 RS1 primary

   input eh2_alu_pkt_t                          i0_ap,                        // DEC alu {valid,predecodes}
   input eh2_alu_pkt_t                          i1_ap,                        // DEC alu {valid,predecodes}

   input eh2_mul_pkt_t                          mul_p,                        // DEC {valid, operand signs, low, operand bypass}
   input eh2_div_pkt_t                          div_p,                        // DEC {valid, unsigned, rem}

   input logic                                   dec_i0_alu_decode_d,          // Valid to Primary ALU
   input logic                                   dec_i1_alu_decode_d,          // Valid to Primary ALU

   input logic                                   dec_i0_select_pc_d,           // PC select to RS1
   input logic                                   dec_i1_select_pc_d,           // PC select to RS1

   input logic [31:1]                            dec_i0_pc_d,
   input logic [31:1]                            dec_i1_pc_d,                  // Instruction PC

   input logic                                   dec_i0_rs1_bypass_en_d,       // DEC bypass select
   input logic                                   dec_i0_rs2_bypass_en_d,       // DEC bypass select
   input logic                                   dec_i1_rs1_bypass_en_d,       // DEC bypass select
   input logic                                   dec_i1_rs2_bypass_en_d,       // DEC bypass select

   input logic [pt.NUM_THREADS-1:0]              dec_tlu_flush_lower_wb,       // Flush divide and secondary ALUs
   input logic [pt.NUM_THREADS-1:0] [31:1]       dec_tlu_flush_path_wb,        // Redirect target

   input logic                                   dec_tlu_i0_valid_e4,          // Valid for GHR
   input logic                                   dec_tlu_i1_valid_e4,          // Valid for GHR



   output logic [31:0]                           exu_i0_result_e1,             // Primary ALU result to DEC
   output logic [31:0]                           exu_i1_result_e1,             // Primary ALU result to DEC
   output logic [31:1]                           exu_i0_pc_e1,                 // Primary PC  result to DEC
   output logic [31:1]                           exu_i1_pc_e1,                 // Primary PC  result to DEC

   output logic [31:0]                           exu_i0_result_e4,             // Secondary ALU result
   output logic [31:0]                           exu_i1_result_e4,             // Secondary ALU result

   output logic [31:0]                           exu_lsu_rs1_d,                // LSU operand
   output logic [31:0]                           exu_lsu_rs2_d,                // LSU operand

   output logic [31:0]                           exu_i0_csr_rs1_e1,            // RS1 source for a CSR instruction

   output logic [pt.NUM_THREADS-1:0]             exu_flush_final,              // Pipe is being flushed this cycle
   output logic [pt.NUM_THREADS-1:0]             exu_i0_flush_final,           // I0 flush to DEC
   output logic [pt.NUM_THREADS-1:0]             exu_i1_flush_final,           // I1 flush to DEC


   output logic [pt.NUM_THREADS-1:0][31:1]       exu_flush_path_final,         // Target for the oldest flush source

   output logic [31:0]                           exu_mul_result_e3,            // Multiply result

   output logic [31:0]                           exu_div_result,               // Divide result
   output logic                                  exu_div_wren,                 // Divide write enable to GPR
   output logic [pt.NUM_THREADS-1:0] [31:1]      exu_npc_e4,                   // Divide NPC

   output logic [pt.NUM_THREADS-1:0]             exu_i0_flush_lower_e4,        // to TLU - lower branch flush
   output logic [pt.NUM_THREADS-1:0]             exu_i1_flush_lower_e4,        // to TLU - lower branch flush

   output logic [31:1]                           exu_i0_flush_path_e4,         // to TLU - lower branch flush path
   output logic [31:1]                           exu_i1_flush_path_e4,         // to TLU - lower branch flush path



   output eh2_predict_pkt_t [pt.NUM_THREADS-1:0]                    exu_mp_pkt,   // to IFU_DP - final mispredict
   output logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]           exu_mp_eghr,  // to IFU_DP - for bht write
   output logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]           exu_mp_fghr,  // to IFU_DP - fghr repair value
   output logic [pt.NUM_THREADS-1:0] [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] exu_mp_index, // to IFU_DP - misprecict index
   output logic [pt.NUM_THREADS-1:0] [pt.BTB_BTAG_SIZE-1:0]          exu_mp_btag,  // to IFU_DP - mispredict tag



   output logic [1:0]                            exu_i0_br_hist_e4,            // to DEC  I0 branch history
   output logic                                  exu_i0_br_bank_e4,            // to DEC  I0 branch bank
   output logic                                  exu_i0_br_error_e4,           // to DEC  I0 branch error
   output logic                                  exu_i0_br_start_error_e4,     // to DEC  I0 branch start error
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]  exu_i0_br_index_e4,           // to DEC  I0 branch index
   output logic                                  exu_i0_br_valid_e4,           // to DEC  I0 branch valid
   output logic                                  exu_i0_br_mp_e4,              // to DEC  I0 branch mispredict
   output logic                                  exu_i0_br_way_e4,             // to DEC  I0 branch way
   output logic                                  exu_i0_br_middle_e4,          // to DEC  I0 branch middle
   output logic [pt.BHT_GHR_SIZE-1:0]            exu_i0_br_fghr_e4,            // to DEC  I0 branch fghr
   output logic                                  exu_i0_br_ret_e4,             // to DEC  I0 branch return
   output logic                                  exu_i0_br_call_e4,            // to DEC  I0 branch call

   output logic [1:0]                            exu_i1_br_hist_e4,            // to DEC  I1 branch history
   output logic                                  exu_i1_br_bank_e4,            // to DEC  I1 branch bank
   output logic                                  exu_i1_br_error_e4,           // to DEC  I1 branch error
   output logic                                  exu_i1_br_start_error_e4,     // to DEC  I1 branch start error
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]  exu_i1_br_index_e4,           // to DEC  I1 branch index
   output logic                                  exu_i1_br_valid_e4,           // to DEC  I1 branch valid
   output logic                                  exu_i1_br_mp_e4,              // to DEC  I1 branch mispredict
   output logic                                  exu_i1_br_way_e4,             // to DEC  I1 branch way
   output logic                                  exu_i1_br_middle_e4,          // to DEC  I1 branch middle
   output logic [pt.BHT_GHR_SIZE-1:0]            exu_i1_br_fghr_e4,            // to DEC  I1 branch fghr
   output logic                                  exu_i1_br_ret_e4,             // to DEC  I1 branch return
   output logic                                  exu_i1_br_call_e4,            // to DEC  I1 branch call

   output logic                                  exu_pmu_i0_br_misp,           // to PMU - I0 E4 branch mispredict
   output logic                                  exu_pmu_i0_br_ataken,         // to PMU - I0 E4 taken
   output logic                                  exu_pmu_i0_pc4,               // to PMU - I0 E4 PC
   output logic                                  exu_pmu_i1_br_misp,           // to PMU - I1 E4 branch mispredict
   output logic                                  exu_pmu_i1_br_ataken,         // to PMU - I1 E4 taken
   output logic                                  exu_pmu_i1_pc4                // to PMU - I1 E4 PC

   );


   logic [31:0]                      i0_rs1_d,i0_rs2_d,i1_rs1_d,i1_rs2_d;

   logic [pt.NUM_THREADS-1:0]        i0_flush_upper_e1, i1_flush_upper_e1;

   logic [31:1]                      i0_flush_path_e1;
   logic [31:1]                      i1_flush_path_e1;

   logic [31:0]                      i0_rs1_final_d;

   logic [31:0]                      mul_rs1_d, mul_rs2_d;

   logic [31:0]                      div_rs1_d, div_rs2_d;

   logic                             i1_valid_e2;

   logic [31:0]                      i0_rs1_e1, i0_rs2_e1;
   logic [31:0]                      i0_rs1_e2, i0_rs2_e2;
   logic [31:0]                      i0_rs1_e3, i0_rs2_e3;
   logic [12:1]                      i0_br_immed_e1, i0_br_immed_e2, i0_br_immed_e3;

   logic [31:0]                      i1_rs1_e1, i1_rs2_e1;
   logic [31:0]                      i1_rs1_e2, i1_rs2_e2;
   logic [31:0]                      i1_rs1_e3, i1_rs2_e3;

   logic [12:1]                      i1_br_immed_e1, i1_br_immed_e2, i1_br_immed_e3;

   logic [31:0]                      i0_rs1_e2_final, i0_rs2_e2_final;
   logic [31:0]                      i1_rs1_e2_final, i1_rs2_e2_final;
   logic [31:0]                      i0_rs1_e3_final, i0_rs2_e3_final;
   logic [31:0]                      i1_rs1_e3_final, i1_rs2_e3_final;
   logic [31:1]                      i0_alu_pc_unused, i1_alu_pc_unused;
   logic [pt.NUM_THREADS-1:0]        i0_flush_upper_e2, i1_flush_upper_e2;
   logic                             i1_valid_e3, i1_valid_e4;
   logic [pt.NUM_THREADS-1:0] [31:1] pred_correct_npc_e3, pred_correct_npc_e4;
   logic [pt.NUM_THREADS-1:0]        i0_flush_upper_e3;
   logic [pt.NUM_THREADS-1:0]        i0_flush_upper_e4;
   logic                             i1_pred_correct_upper_e1, i0_pred_correct_upper_e1;
   logic                             i1_pred_correct_upper_e2, i0_pred_correct_upper_e2;
   logic                             i1_pred_correct_upper_e3, i0_pred_correct_upper_e3;
   logic                             i1_pred_correct_upper_e4, i0_pred_correct_upper_e4;
   logic                             i1_pred_correct_lower_e4, i0_pred_correct_lower_e4;

   logic [pt.NUM_THREADS-1:0]        i1_valid_e4_eff;
   logic                             i1_sec_decode_e4, i0_sec_decode_e4;
   logic                             i1_pred_correct_e4_eff, i0_pred_correct_e4_eff;
   logic [31:1]                      i1_flush_path_e4_eff, i0_flush_path_e4_eff;
   logic [31:0]                      i0_csr_rs1_in_d;
   logic [31:1]                      i1_flush_path_upper_e2, i0_flush_path_upper_e2;
   logic [31:1]                      i1_flush_path_upper_e3, i0_flush_path_upper_e3;
   logic [31:1]                      i1_flush_path_upper_e4, i0_flush_path_upper_e4;

   eh2_alu_pkt_t                    i0_ap_e1, i0_ap_e2, i0_ap_e3, i0_ap_e4;
   eh2_alu_pkt_t                    i1_ap_e1, i1_ap_e2, i1_ap_e3, i1_ap_e4;

   logic                             i0_e1_data_en, i0_e2_data_en, i0_e3_data_en;
   logic                             i0_e1_ctl_en,  i0_e2_ctl_en,  i0_e3_ctl_en,  i0_e4_ctl_en;

   logic                             i1_e1_data_en, i1_e2_data_en, i1_e3_data_en;
   logic                             i1_e1_ctl_en,  i1_e2_ctl_en,  i1_e3_ctl_en,  i1_e4_ctl_en;

   localparam PREDPIPESIZE = pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1+pt.BHT_GHR_SIZE+pt.BTB_BTAG_SIZE;
   logic [PREDPIPESIZE-1:0]          i0_predpipe_d, i0_predpipe_e1, i0_predpipe_e2, i0_predpipe_e3, i0_predpipe_e4;
   logic [PREDPIPESIZE-1:0]          i1_predpipe_d, i1_predpipe_e1, i1_predpipe_e2, i1_predpipe_e3, i1_predpipe_e4;

   logic                             i0_taken_e1, i1_taken_e1, dec_i0_alu_decode_e1, dec_i1_alu_decode_e1;
   logic [pt.NUM_THREADS-1:0]        flush_final_f;

   eh2_predict_pkt_t                i0_predict_p_e1, i0_predict_p_e4;
   eh2_predict_pkt_t                i1_predict_p_e1, i1_predict_p_e4;

   eh2_predict_pkt_t                i0_pp_e2, i0_pp_e3, i0_pp_e4_in;
   eh2_predict_pkt_t                i1_pp_e2, i1_pp_e3, i1_pp_e4_in;
   eh2_predict_pkt_t                i0_predict_newp_d, i1_predict_newp_d;


   logic [pt.NUM_THREADS-1:0]                        i0_valid_e1, i1_valid_e1;
   logic [pt.NUM_THREADS-1:0]                        i0_valid_e4, i1_pred_valid_e4;
   logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]  ghr_e1_ns, ghr_e1;
   logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]  ghr_e4_ns, ghr_e4;
   logic [pt.NUM_THREADS-1:0]                        fp_enable, fp_enable_ff;
   logic [pt.NUM_THREADS-1:0] [pt.BHT_GHR_SIZE-1:0]  after_flush_eghr;
   logic [pt.NUM_THREADS-1:0] [PREDPIPESIZE-1:0]     final_predpipe_mp, final_predpipe_mp_ff;
   eh2_predict_pkt_t [pt.NUM_THREADS-1:0]           final_predict_mp;
   logic [pt.NUM_THREADS-1:0] [31:1]                 flush_path_e2;




   assign i0_rs1_d[31:0]       = ({32{~dec_i0_rs1_bypass_en_d}} & ((dec_debug_wdata_rs1_d) ? dbg_cmd_wrdata[31:0] : gpr_i0_rs1_d[31:0])) |
                                 ({32{~dec_i0_rs1_bypass_en_d   & dec_i0_select_pc_d}} & { dec_i0_pc_d[31:1], 1'b0}) |    // for jal's
                                 ({32{ dec_i0_rs1_bypass_en_d}} & i0_rs1_bypass_data_d[31:0]);


   assign i0_rs1_final_d[31:0] =  {32{~dec_i0_csr_ren_d}}       & i0_rs1_d[31:0];

   assign i0_rs2_d[31:0]       = ({32{~dec_i0_rs2_bypass_en_d}} & gpr_i0_rs2_d[31:0]        ) |
                                 ({32{~dec_i0_rs2_bypass_en_d}} & dec_i0_immed_d[31:0]      ) |
                                 ({32{ dec_i0_rs2_bypass_en_d}} & i0_rs2_bypass_data_d[31:0]);

   assign i1_rs1_d[31:0]       = ({32{~dec_i1_rs1_bypass_en_d}} & gpr_i1_rs1_d[31:0]) |
                                 ({32{~dec_i1_rs1_bypass_en_d   & dec_i1_select_pc_d}} & { dec_i1_pc_d[31:1], 1'b0}) |  // pc orthogonal with rs1
                                 ({32{ dec_i1_rs1_bypass_en_d}} & i1_rs1_bypass_data_d[31:0]);


   assign i1_rs2_d[31:0]       = ({32{~dec_i1_rs2_bypass_en_d}} & gpr_i1_rs2_d[31:0]        ) |
                                 ({32{~dec_i1_rs2_bypass_en_d}} & dec_i1_immed_d[31:0]      ) |
                                 ({32{ dec_i1_rs2_bypass_en_d}} & i1_rs2_bypass_data_d[31:0]);


   assign exu_lsu_rs1_d[31:0]  = ({32{ ~dec_i0_rs1_bypass_en_d &  dec_i0_lsu_d & ~dec_extint_stall               }} & gpr_i0_rs1_d[31:0]        ) |
                                 ({32{ ~dec_i1_rs1_bypass_en_d & ~dec_i0_lsu_d & ~dec_extint_stall & dec_i1_lsu_d}} & gpr_i1_rs1_d[31:0]        ) |
                                 ({32{  dec_i0_rs1_bypass_en_d &  dec_i0_lsu_d & ~dec_extint_stall               }} & i0_rs1_bypass_data_d[31:0]) |
                                 ({32{  dec_i1_rs1_bypass_en_d & ~dec_i0_lsu_d & ~dec_extint_stall & dec_i1_lsu_d}} & i1_rs1_bypass_data_d[31:0]) |
                                 ({32{                                            dec_extint_stall               }} & {dec_tlu_meihap[31:2],2'b0});

   assign exu_lsu_rs2_d[31:0]  = ({32{ ~dec_i0_rs2_bypass_en_d &  dec_i0_lsu_d & ~dec_extint_stall               }} & gpr_i0_rs2_d[31:0]        ) |
                                 ({32{ ~dec_i1_rs2_bypass_en_d & ~dec_i0_lsu_d & ~dec_extint_stall & dec_i1_lsu_d}} & gpr_i1_rs2_d[31:0]        ) |
                                 ({32{  dec_i0_rs2_bypass_en_d &  dec_i0_lsu_d & ~dec_extint_stall               }} & i0_rs2_bypass_data_d[31:0]) |
                                 ({32{  dec_i1_rs2_bypass_en_d & ~dec_i0_lsu_d & ~dec_extint_stall & dec_i1_lsu_d}} & i1_rs2_bypass_data_d[31:0]);


   assign mul_rs1_d[31:0]      = ({32{ ~dec_i0_rs1_bypass_en_d &  dec_i0_mul_d               }} & gpr_i0_rs1_d[31:0]        ) |
                                 ({32{ ~dec_i1_rs1_bypass_en_d & ~dec_i0_mul_d & dec_i1_mul_d}} & gpr_i1_rs1_d[31:0]        ) |
                                 ({32{  dec_i0_rs1_bypass_en_d &  dec_i0_mul_d               }} & i0_rs1_bypass_data_d[31:0]) |
                                 ({32{  dec_i1_rs1_bypass_en_d & ~dec_i0_mul_d & dec_i1_mul_d}} & i1_rs1_bypass_data_d[31:0]);

   assign mul_rs2_d[31:0]      = ({32{ ~dec_i0_rs2_bypass_en_d &  dec_i0_mul_d               }} & gpr_i0_rs2_d[31:0]        ) |
                                 ({32{ ~dec_i1_rs2_bypass_en_d & ~dec_i0_mul_d & dec_i1_mul_d}} & gpr_i1_rs2_d[31:0]        ) |
                                 ({32{  dec_i0_rs2_bypass_en_d &  dec_i0_mul_d               }} & i0_rs2_bypass_data_d[31:0]) |
                                 ({32{  dec_i1_rs2_bypass_en_d & ~dec_i0_mul_d & dec_i1_mul_d}} & i1_rs2_bypass_data_d[31:0]);



   assign div_rs1_d[31:0]      = ({32{ ~dec_i0_rs1_bypass_en_d &  dec_i0_div_d               }} & gpr_i0_rs1_d[31:0]) |
                                 ({32{  dec_i0_rs1_bypass_en_d &  dec_i0_div_d               }} & i0_rs1_bypass_data_d[31:0]);

   assign div_rs2_d[31:0]      = ({32{ ~dec_i0_rs2_bypass_en_d &  dec_i0_div_d               }} & gpr_i0_rs2_d[31:0]) |
                                 ({32{  dec_i0_rs2_bypass_en_d &  dec_i0_div_d               }} & i0_rs2_bypass_data_d[31:0]);



   assign i0_csr_rs1_in_d[31:0] = (dec_i0_csr_ren_d) ? i0_rs1_d[31:0] : exu_i0_csr_rs1_e1[31:0];

   assign {i0_e1_data_en, i0_e2_data_en, i0_e3_data_en }                = dec_i0_data_en[4:2];
   assign {i0_e1_ctl_en,  i0_e2_ctl_en,  i0_e3_ctl_en,  i0_e4_ctl_en }  = dec_i0_ctl_en[4:1];

   assign {i1_e1_data_en, i1_e2_data_en, i1_e3_data_en}                = dec_i1_data_en[4:2];
   assign {i1_e1_ctl_en,  i1_e2_ctl_en,  i1_e3_ctl_en,  i1_e4_ctl_en}  = dec_i1_ctl_en[4:1];




   rvdffe #(32) i0_csr_rs1_ff (.*, .en(i0_e1_data_en), .din(i0_csr_rs1_in_d[31:0]), .dout(exu_i0_csr_rs1_e1[31:0]));


   eh2_exu_mul_ctl #(.pt(pt)) mul_e1    (.*,
                          .clk_override  ( clk_override                ),   // I
                          .mp            ( mul_p                       ),   // I
                          .a             ( mul_rs1_d[31:0]             ),   // I
                          .b             ( mul_rs2_d[31:0]             ),   // I
                          .out           ( exu_mul_result_e3[31:0]     ));  // O


   eh2_exu_div_ctl #(.pt(pt)) div_e1    (.*,
                          .cancel        ( dec_div_cancel              ),   // I
                          .dp            ( div_p                       ),   // I
                          .dividend      ( div_rs1_d[31:0]             ),   // I
                          .divisor       ( div_rs2_d[31:0]             ),   // I
                          .finish_dly    ( exu_div_wren                ),   // O
                          .out           ( exu_div_result[31:0]        ));  // O


   always_comb begin
      i0_predict_newp_d         = i0_predict_p_d;
      i0_predict_newp_d.boffset = dec_i0_pc_d[1];  // from the start of inst
      i0_predict_newp_d.bank    = i0_predict_p_d.bank;

      i1_predict_newp_d         = i1_predict_p_d;
      i1_predict_newp_d.boffset = dec_i1_pc_d[1];
      i1_predict_newp_d.bank    = i1_predict_p_d.bank;

   end




   eh2_exu_alu_ctl #(.pt(pt)) i0_alu_e1 (.*,
                          .enable        ( i0_e1_ctl_en                ),   // I
                          .predict_p     ( i0_predict_newp_d           ),   // I
                          .valid         ( dec_i0_alu_decode_d         ),   // I
                          .flush         ( exu_flush_final             ),   // I
                          .a             ( i0_rs1_final_d[31:0]        ),   // I
                          .b             ( i0_rs2_d[31:0]              ),   // I
                          .pc            ( dec_i0_pc_d[31:1]           ),   // I
                          .brimm         ( dec_i0_br_immed_d[12:1]     ),   // I
                          .ap_in_tid     ( i0_ap.tid                   ),   // I
                          .ap            ( i0_ap_e1                    ),   // I
                          .out           ( exu_i0_result_e1[31:0]      ),   // O
                          .flush_upper   ( i0_flush_upper_e1           ),   // O
                          .flush_path    ( i0_flush_path_e1[31:1]      ),   // O
                          .predict_p_ff  ( i0_predict_p_e1             ),   // O
                          .pc_ff         ( exu_i0_pc_e1[31:1]          ),   // O
                          .pred_correct  ( i0_pred_correct_upper_e1    )    // O
                          );


   eh2_exu_alu_ctl #(.pt(pt)) i1_alu_e1 (.*,
                          .enable        ( i1_e1_ctl_en                ),   // I
                          .predict_p     ( i1_predict_newp_d           ),   // I
                          .valid         ( dec_i1_alu_decode_d         ),   // I
                          .flush         ( exu_flush_final             ),   // I
                          .a             ( i1_rs1_d[31:0]              ),   // I
                          .b             ( i1_rs2_d[31:0]              ),   // I
                          .pc            ( dec_i1_pc_d[31:1]           ),   // I
                          .brimm         ( dec_i1_br_immed_d[12:1]     ),   // I
                          .ap_in_tid     ( i1_ap.tid                   ),   // I
                          .ap            ( i1_ap_e1                    ),   // I
                          .out           ( exu_i1_result_e1[31:0]      ),   // O
                          .flush_upper   ( i1_flush_upper_e1           ),   // O
                          .flush_path    ( i1_flush_path_e1[31:1]      ),   // O
                          .predict_p_ff  ( i1_predict_p_e1             ),   // O
                          .pc_ff         ( exu_i1_pc_e1[31:1]          ),   // O
                          .pred_correct  ( i1_pred_correct_upper_e1    )    // O
                          );



   assign i0_predpipe_d[PREDPIPESIZE-1:0] = {i0_predict_fghr_d, i0_predict_index_d, i0_predict_btag_d};
   assign i1_predpipe_d[PREDPIPESIZE-1:0] = {i1_predict_fghr_d, i1_predict_index_d, i1_predict_btag_d};


   rvdffe #($bits(eh2_predict_pkt_t))  i0_pp_e2_ff            (.*, .en ( i0_e2_ctl_en      ),  .din ( i0_predict_p_e1                  ),  .dout ( i0_pp_e2                      ) );
   rvdffe #($bits(eh2_predict_pkt_t))  i0_pp_e3_ff            (.*, .en ( i0_e3_ctl_en      ),  .din ( i0_pp_e2                         ),  .dout ( i0_pp_e3                      ) );

   rvdffe #($bits(eh2_predict_pkt_t))  i1_pp_e2_ff            (.*, .en ( i1_e2_ctl_en      ),  .din( i1_predict_p_e1                   ),  .dout( i1_pp_e2                       ) );
   rvdffe #($bits(eh2_predict_pkt_t))  i1_pp_e3_ff            (.*, .en ( i1_e3_ctl_en      ),  .din( i1_pp_e2                          ),  .dout( i1_pp_e3                       ) );


   rvdffe #(PREDPIPESIZE)               i0_predpipe_e1_ff      (.*, .en ( i0_e1_data_en     ),  .din( i0_predpipe_d                     ),  .dout( i0_predpipe_e1                 ) );
   rvdffe #(PREDPIPESIZE)               i0_predpipe_e2_ff      (.*, .en ( i0_e2_data_en     ),  .din( i0_predpipe_e1                    ),  .dout( i0_predpipe_e2                 ) );
   rvdffe #(PREDPIPESIZE)               i0_predpipe_e3_ff      (.*, .en ( i0_e3_data_en     ),  .din( i0_predpipe_e2                    ),  .dout( i0_predpipe_e3                 ) );
   rvdffe #(PREDPIPESIZE)               i0_predpipe_e4_ff      (.*, .en ( i0_e4_ctl_en      ),  .din( i0_predpipe_e3                    ),  .dout( i0_predpipe_e4                 ) );

   rvdffe #(PREDPIPESIZE)               i1_predpipe_e1_ff      (.*, .en ( i1_e1_data_en     ),  .din( i1_predpipe_d                     ),  .dout( i1_predpipe_e1                 ) );
   rvdffe #(PREDPIPESIZE)               i1_predpipe_e2_ff      (.*, .en ( i1_e2_data_en     ),  .din( i1_predpipe_e1                    ),  .dout( i1_predpipe_e2                 ) );
   rvdffe #(PREDPIPESIZE)               i1_predpipe_e3_ff      (.*, .en ( i1_e3_data_en     ),  .din( i1_predpipe_e2                    ),  .dout( i1_predpipe_e3                 ) );
   rvdffe #(PREDPIPESIZE)               i1_predpipe_e4_ff      (.*, .en ( i1_e4_ctl_en      ),  .din( i1_predpipe_e3                    ),  .dout( i1_predpipe_e4                 ) );


   assign exu_pmu_i0_br_misp   = i0_predict_p_e4.misp;
   assign exu_pmu_i0_br_ataken = i0_predict_p_e4.ataken;
   assign exu_pmu_i0_pc4       = i0_predict_p_e4.pc4;
   assign exu_pmu_i1_br_misp   = i1_predict_p_e4.misp;
   assign exu_pmu_i1_br_ataken = i1_predict_p_e4.ataken;
   assign exu_pmu_i1_pc4       = i1_predict_p_e4.pc4;



   assign i0_pp_e4_in = i0_pp_e3;
   assign i1_pp_e4_in = i1_pp_e3;

   rvdffe #($bits(eh2_alu_pkt_t)) i0_ap_e1_ff (.*,  .en(i0_e1_ctl_en), .din(i0_ap),   .dout(i0_ap_e1) );
   rvdffe #($bits(eh2_alu_pkt_t)) i0_ap_e2_ff (.*,  .en(i0_e2_ctl_en), .din(i0_ap_e1),.dout(i0_ap_e2) );
   rvdffe #($bits(eh2_alu_pkt_t)) i0_ap_e3_ff (.*,  .en(i0_e3_ctl_en), .din(i0_ap_e2),.dout(i0_ap_e3) );
   rvdffe #($bits(eh2_alu_pkt_t)) i0_ap_e4_ff (.*,  .en(i0_e4_ctl_en), .din(i0_ap_e3),.dout(i0_ap_e4) );


   rvdffe #($bits(eh2_alu_pkt_t)) i1_ap_e1_ff (.*,  .en(i1_e1_ctl_en), .din(i1_ap),   .dout(i1_ap_e1) );
   rvdffe #($bits(eh2_alu_pkt_t)) i1_ap_e2_ff (.*,  .en(i1_e2_ctl_en), .din(i1_ap_e1),.dout(i1_ap_e2) );
   rvdffe #($bits(eh2_alu_pkt_t)) i1_ap_e3_ff (.*,  .en(i1_e3_ctl_en), .din(i1_ap_e2),.dout(i1_ap_e3) );
   rvdffe #($bits(eh2_alu_pkt_t)) i1_ap_e4_ff (.*,  .en(i1_e4_ctl_en), .din(i1_ap_e3),.dout(i1_ap_e4) );



   rvdffe #(76) i0_src_e1_ff (.*,
                            .en  (i0_e1_data_en),
                            .din ({i0_rs1_d [31:0], i0_rs2_d [31:0], dec_i0_br_immed_d [12:1]}),
                            .dout({i0_rs1_e1[31:0], i0_rs2_e1[31:0],     i0_br_immed_e1[12:1]}));

   rvdffe #(76) i0_src_e2_ff (.*,
                            .en  (i0_e2_data_en),
                            .din( {i0_rs1_e1[31:0], i0_rs2_e1[31:0], i0_br_immed_e1[12:1]}),
                            .dout({i0_rs1_e2[31:0], i0_rs2_e2[31:0], i0_br_immed_e2[12:1]}));

   rvdffe #(76) i0_src_e3_ff (.*,
                            .en  (i0_e3_data_en),
                            .din( {i0_rs1_e2_final[31:0], i0_rs2_e2_final[31:0], i0_br_immed_e2[12:1]}),
                            .dout({i0_rs1_e3[31:0],       i0_rs2_e3[31:0],       i0_br_immed_e3[12:1]}));



   rvdffe #(76) i1_src_e1_ff (.*,
                            .en  (i1_e1_data_en),
                            .din ({i1_rs1_d [31:0], i1_rs2_d [31:0], dec_i1_br_immed_d [12:1]}),
                            .dout({i1_rs1_e1[31:0], i1_rs2_e1[31:0],     i1_br_immed_e1[12:1]}));

   rvdffe #(76) i1_src_e2_ff (.*,
                            .en  (i1_e2_data_en),
                            .din ({i1_rs1_e1[31:0], i1_rs2_e1[31:0], i1_br_immed_e1[12:1]}),
                            .dout({i1_rs1_e2[31:0], i1_rs2_e2[31:0], i1_br_immed_e2[12:1]}));

   rvdffe #(76) i1_src_e3_ff (.*,
                            .en  (i1_e3_data_en),
                            .din ({i1_rs1_e2_final[31:0], i1_rs2_e2_final[31:0], i1_br_immed_e2[12:1]}),
                            .dout({i1_rs1_e3[31:0],       i1_rs2_e3[31:0],       i1_br_immed_e3[12:1]}));




   assign i0_rs1_e2_final[31:0] = (dec_i0_rs1_bypass_en_e2) ? i0_rs1_bypass_data_e2[31:0] : i0_rs1_e2[31:0];
   assign i0_rs2_e2_final[31:0] = (dec_i0_rs2_bypass_en_e2) ? i0_rs2_bypass_data_e2[31:0] : i0_rs2_e2[31:0];
   assign i1_rs1_e2_final[31:0] = (dec_i1_rs1_bypass_en_e2) ? i1_rs1_bypass_data_e2[31:0] : i1_rs1_e2[31:0];
   assign i1_rs2_e2_final[31:0] = (dec_i1_rs2_bypass_en_e2) ? i1_rs2_bypass_data_e2[31:0] : i1_rs2_e2[31:0];


   assign i0_rs1_e3_final[31:0] = (dec_i0_rs1_bypass_en_e3) ? i0_rs1_bypass_data_e3[31:0] : i0_rs1_e3[31:0];
   assign i0_rs2_e3_final[31:0] = (dec_i0_rs2_bypass_en_e3) ? i0_rs2_bypass_data_e3[31:0] : i0_rs2_e3[31:0];
   assign i1_rs1_e3_final[31:0] = (dec_i1_rs1_bypass_en_e3) ? i1_rs1_bypass_data_e3[31:0] : i1_rs1_e3[31:0];
   assign i1_rs2_e3_final[31:0] = (dec_i1_rs2_bypass_en_e3) ? i1_rs2_bypass_data_e3[31:0] : i1_rs2_e3[31:0];



   assign i0_taken_e1  = (i0_predict_p_e1.ataken & dec_i0_alu_decode_e1) | (i0_predict_p_e1.hist[1] & ~dec_i0_alu_decode_e1);
   assign i1_taken_e1  = (i1_predict_p_e1.ataken & dec_i1_alu_decode_e1) | (i1_predict_p_e1.hist[1] & ~dec_i1_alu_decode_e1);

   rvdff #(2) e1ghrdecff                  (.*, .clk(active_clk), .din({dec_i0_alu_decode_d, dec_i1_alu_decode_d}), .dout({dec_i0_alu_decode_e1, dec_i1_alu_decode_e1}));




   eh2_exu_alu_ctl #(.pt(pt)) i0_alu_e4 (.*,
                          .enable        ( i0_e4_ctl_en                ),   // I
                          .predict_p     ( i0_pp_e4_in                 ),   // I
                          .valid         ( dec_i0_sec_decode_e3        ),   // I
                          .flush         ( dec_tlu_flush_lower_wb      ),   // I
                          .a             ( i0_rs1_e3_final[31:0]       ),   // I
                          .b             ( i0_rs2_e3_final[31:0]       ),   // I
                          .pc            ( dec_i0_pc_e3[31:1]          ),   // I
                          .brimm         ( i0_br_immed_e3[12:1]        ),   // I
                          .ap_in_tid     ( i0_ap_e3.tid                ),   // I
                          .ap            ( i0_ap_e4                    ),   // I
                          .out           ( exu_i0_result_e4[31:0]      ),   // O
                          .flush_upper   ( exu_i0_flush_lower_e4       ),   // O
                          .flush_path    ( exu_i0_flush_path_e4[31:1]  ),   // O
                          .predict_p_ff  ( i0_predict_p_e4             ),   // O
                          .pc_ff         ( i0_alu_pc_unused[31:1]      ),   // O
                          .pred_correct  ( i0_pred_correct_lower_e4    )    // O
                          );


   eh2_exu_alu_ctl #(.pt(pt)) i1_alu_e4 (.*,
                          .enable        ( i1_e4_ctl_en                ),   // I
                          .predict_p     ( i1_pp_e4_in                 ),   // I
                          .valid         ( dec_i1_sec_decode_e3        ),   // I
                          .flush         ( dec_tlu_flush_lower_wb      ),   // I
                          .a             ( i1_rs1_e3_final[31:0]       ),   // I
                          .b             ( i1_rs2_e3_final[31:0]       ),   // I
                          .pc            ( dec_i1_pc_e3[31:1]          ),   // I
                          .brimm         ( i1_br_immed_e3[12:1]        ),   // I
                          .ap_in_tid     ( i1_ap_e3.tid                ),   // I
                          .ap            ( i1_ap_e4                    ),   // I
                          .out           ( exu_i1_result_e4[31:0]      ),   // O
                          .flush_upper   ( exu_i1_flush_lower_e4       ),   // O
                          .flush_path    ( exu_i1_flush_path_e4[31:1]  ),   // O
                          .predict_p_ff  ( i1_predict_p_e4             ),   // O
                          .pc_ff         ( i1_alu_pc_unused[31:1]      ),   // O
                          .pred_correct  ( i1_pred_correct_lower_e4    )    // O
                          );


   assign exu_i0_br_hist_e4[1:0]               =  i0_predict_p_e4.hist[1:0];
   assign exu_i0_br_bank_e4                    =  i0_predict_p_e4.bank;
   assign exu_i0_br_error_e4                   =  i0_predict_p_e4.br_error;
   assign exu_i0_br_middle_e4                  =  i0_predict_p_e4.pc4 ^ i0_predict_p_e4.boffset;
   assign exu_i0_br_start_error_e4             =  i0_predict_p_e4.br_start_error;

   assign exu_i0_br_valid_e4                   =  i0_predict_p_e4.valid;
   assign exu_i0_br_mp_e4                      =  i0_predict_p_e4.misp; // needed to squash i1 error
   assign exu_i0_br_ret_e4                     =  i0_predict_p_e4.pret;
   assign exu_i0_br_call_e4                    =  i0_predict_p_e4.pcall;
   assign exu_i0_br_way_e4                     =  i0_predict_p_e4.way;

   assign {exu_i0_br_fghr_e4[pt.BHT_GHR_SIZE-1:0],
           exu_i0_br_index_e4[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]} =  i0_predpipe_e4[PREDPIPESIZE-1:pt.BTB_BTAG_SIZE];

   assign exu_i1_br_hist_e4[1:0]               =  i1_predict_p_e4.hist[1:0];
   assign exu_i1_br_bank_e4                    =  i1_predict_p_e4.bank;
   assign exu_i1_br_middle_e4                  =  i1_predict_p_e4.pc4 ^ i1_predict_p_e4.boffset;
   assign exu_i1_br_error_e4                   =  i1_predict_p_e4.br_error;

   assign exu_i1_br_start_error_e4             =  i1_predict_p_e4.br_start_error;
   assign exu_i1_br_valid_e4                   =  i1_predict_p_e4.valid;
   assign exu_i1_br_mp_e4                      =  i1_predict_p_e4.misp;
   assign exu_i1_br_way_e4                     =  i1_predict_p_e4.way;
   assign exu_i1_br_ret_e4                     =  i1_predict_p_e4.pret;
   assign exu_i1_br_call_e4                    =  i1_predict_p_e4.pcall;

   assign {exu_i1_br_fghr_e4[pt.BHT_GHR_SIZE-1:0],
           exu_i1_br_index_e4[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]} =  i1_predpipe_e4[PREDPIPESIZE-1:pt.BTB_BTAG_SIZE];



   for (genvar i=0; i<pt.NUM_THREADS; i++) begin

      assign fp_enable[i]                             = (exu_i0_flush_lower_e4[i]) | (exu_i1_flush_lower_e4[i]) |
                                                        (i0_flush_upper_e1[i])     | (i1_flush_upper_e1[i]);

      assign final_predict_mp[i]                      = (exu_i0_flush_lower_e4[i])  ?  i0_predict_p_e4 :
                                                        (exu_i1_flush_lower_e4[i])  ?  i1_predict_p_e4 :
                                                        (i0_flush_upper_e1[i])      ?  i0_predict_p_e1 :
                                                        (i1_flush_upper_e1[i])      ?  i1_predict_p_e1 : '0;

      assign final_predpipe_mp[i][PREDPIPESIZE-1:0]   = (exu_i0_flush_lower_e4[i])  ?  i0_predpipe_e4  :
                                                        (exu_i1_flush_lower_e4[i])  ?  i1_predpipe_e4  :
                                                        (i0_flush_upper_e1[i])      ?  i0_predpipe_e1  :
                                                        (i1_flush_upper_e1[i])      ?  i1_predpipe_e1  : '0;


      assign after_flush_eghr[i][pt.BHT_GHR_SIZE-1:0] = (i0_flush_upper_e2[i] | i1_flush_upper_e2[i] & ~dec_tlu_flush_lower_wb[i]) ? ghr_e1[i][pt.BHT_GHR_SIZE-1:0] : ghr_e4[i][pt.BHT_GHR_SIZE-1:0];

      assign exu_mp_fghr[i][pt.BHT_GHR_SIZE-1:0]      =  after_flush_eghr[i][pt.BHT_GHR_SIZE-1:0];     // fghr repair value

      assign {exu_mp_index[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO],
              exu_mp_btag[i][pt.BTB_BTAG_SIZE-1:0]}   =  final_predpipe_mp_ff[i][PREDPIPESIZE-pt.BHT_GHR_SIZE-1:0];
      assign  exu_mp_eghr[i][pt.BHT_GHR_SIZE-1:0]     =  final_predpipe_mp_ff[i][PREDPIPESIZE-1:pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+pt.BTB_BTAG_SIZE+1]; // mp ghr for bht write


      // E1 GHR - fill in the ptaken for secondary branches.
      assign i0_valid_e1[i]  = ~exu_flush_final[i] & ~flush_final_f[i] & (i0_predict_p_e1.valid | i0_predict_p_e1.misp);
      assign i1_valid_e1[i]  = ~exu_flush_final[i] & ~flush_final_f[i] & (i1_predict_p_e1.valid | i1_predict_p_e1.misp) & ~(i0_flush_upper_e1[i]);

      assign ghr_e1_ns[i][pt.BHT_GHR_SIZE-1:0]  = ({pt.BHT_GHR_SIZE{ dec_tlu_flush_lower_wb[i]}}                                                                  &  ghr_e4[i][pt.BHT_GHR_SIZE-1:0]) |
                                                  ({pt.BHT_GHR_SIZE{~dec_tlu_flush_lower_wb[i] & ~i0_valid_e1[i] &  ~i1_valid_e1[i]}}                             &  ghr_e1[i][pt.BHT_GHR_SIZE-1:0]) |
                                                  ({pt.BHT_GHR_SIZE{~dec_tlu_flush_lower_wb[i] & ~i0_valid_e1[i] &   i1_valid_e1[i] & ~i0_predict_p_e1.br_error}} & {ghr_e1[i][pt.BHT_GHR_SIZE-2:0], i1_taken_e1}) |
                                                  ({pt.BHT_GHR_SIZE{~dec_tlu_flush_lower_wb[i] &  i0_valid_e1[i] & (~i1_valid_e1[i] |  i0_predict_p_e1.misp   )}} & {ghr_e1[i][pt.BHT_GHR_SIZE-2:0], i0_taken_e1}) |
                                                  ({pt.BHT_GHR_SIZE{~dec_tlu_flush_lower_wb[i] &  i0_valid_e1[i] &   i1_valid_e1[i] & ~i0_predict_p_e1.misp    }} & {ghr_e1[i][pt.BHT_GHR_SIZE-3:0], i0_taken_e1, i1_taken_e1});


      // E4 GHR - the ataken is filled in by e1 stage if e1 stage executes the branch, otherwise by e4 stage.
      assign i0_valid_e4[i]                     =  dec_tlu_i0_valid_e4 & (i0_ap_e4.tid==i) & ((i0_predict_p_e4.valid) | i0_predict_p_e4.misp);
      assign i1_pred_valid_e4[i]                =  dec_tlu_i1_valid_e4 & (i1_ap_e4.tid==i) & ((i1_predict_p_e4.valid) | i1_predict_p_e4.misp) & ~i0_flush_upper_e4[i];
      assign ghr_e4_ns[i][pt.BHT_GHR_SIZE-1:0]  = ({pt.BHT_GHR_SIZE{ i0_valid_e4[i] & (i0_predict_p_e4.misp |     ~i1_pred_valid_e4[i])}} & {ghr_e4[i][pt.BHT_GHR_SIZE-2:0], i0_predict_p_e4.ataken}) |
                                                  ({pt.BHT_GHR_SIZE{ i0_valid_e4[i] & ~i0_predict_p_e4.misp &      i1_pred_valid_e4[i]}}  & {ghr_e4[i][pt.BHT_GHR_SIZE-3:0], i0_predict_p_e4.ataken, i1_predict_p_e4.ataken}) |
                                                  ({pt.BHT_GHR_SIZE{~i0_valid_e4[i] & ~i0_predict_p_e4.br_error &  i1_pred_valid_e4[i]}}  & {ghr_e4[i][pt.BHT_GHR_SIZE-2:0], i1_predict_p_e4.ataken}) |
                                                  ({pt.BHT_GHR_SIZE{~i0_valid_e4[i] &                             ~i1_pred_valid_e4[i]}}  &  ghr_e4[i][pt.BHT_GHR_SIZE-1:0]);


      rvdff  #(1)                         e4ghrflushff      (.*, .clk(active_clk),                    .din (exu_flush_final[i]),                 .dout(flush_final_f[i]));
      rvdff  #(1)                         final_predict_ff  (.*, .clk(active_clk),                    .din(fp_enable[i]),                        .dout(fp_enable_ff[i]));
      rvdffe #($bits(eh2_predict_pkt_t)) predict_mp_ff     (.*, .en(fp_enable[i] | fp_enable_ff[i]), .din(final_predict_mp [i]),                .dout(exu_mp_pkt[i]));
      rvdffe #(PREDPIPESIZE)              predictpipe_mp_ff (.*, .en(fp_enable[i] | fp_enable_ff[i]), .din(final_predpipe_mp[i]),                .dout(final_predpipe_mp_ff[i]));
      rvdff #(pt.BHT_GHR_SIZE)            e1ghrff           (.*, .clk(active_clk),                    .din (ghr_e1_ns[i][pt.BHT_GHR_SIZE-1:0]),  .dout(ghr_e1[i][pt.BHT_GHR_SIZE-1:0]));
      rvdff #(pt.BHT_GHR_SIZE)            e4ghrff           (.*, .clk(active_clk),                    .din (ghr_e4_ns[i][pt.BHT_GHR_SIZE-1:0]),  .dout(ghr_e4[i][pt.BHT_GHR_SIZE-1:0]));

   end






   rvdffe #(31+pt.NUM_THREADS) i0_upper_flush_e2_ff (.*,
                                    .en  ( i0_e2_ctl_en),
                                    .din ({i0_flush_path_e1[31:1],
                                           i0_flush_upper_e1[pt.NUM_THREADS-1:0]}),
                                    .dout({i0_flush_path_upper_e2[31:1],
                                           i0_flush_upper_e2[pt.NUM_THREADS-1:0]}));

   rvdffe #(32+pt.NUM_THREADS) i1_upper_flush_e2_ff (.*,
                                    .en  ( i1_e2_ctl_en),
                                    .din ({dec_i1_valid_e1,
                                           i1_flush_path_e1[31:1],
                                           i1_flush_upper_e1[pt.NUM_THREADS-1:0]}),
                                    .dout({i1_valid_e2,
                                           i1_flush_path_upper_e2[31:1],
                                           i1_flush_upper_e2[pt.NUM_THREADS-1:0]}));



   for (genvar i=0; i<pt.NUM_THREADS; i++) begin

      assign flush_path_e2[i][31:1]           = (i0_flush_upper_e2[i])       ?  i0_flush_path_upper_e2[31:1]    :  i1_flush_path_upper_e2[31:1];
      assign exu_flush_path_final[i][31:1]    = (dec_tlu_flush_lower_wb[i])  ?  dec_tlu_flush_path_wb[i][31:1]  :  flush_path_e2[i][31:1];



      assign exu_i0_flush_final[i]         =    dec_tlu_flush_lower_wb[i] | i0_flush_upper_e2[i];
      assign exu_i1_flush_final[i]         =    dec_tlu_flush_lower_wb[i] | i1_flush_upper_e2[i];
      assign exu_flush_final[i]            =    dec_tlu_flush_lower_wb[i] | i0_flush_upper_e2[i]  | i1_flush_upper_e2[i];

   end


   rvdffe #(31+pt.NUM_THREADS*32) i0_upper_flush_e3_ff (.*,
                                    .en  ( i0_e3_ctl_en | i1_e3_ctl_en),
                                    .din ({i0_flush_path_upper_e2[31:1],
                                           pred_correct_npc_e2,
                                           i0_flush_upper_e2}),
                                    .dout({i0_flush_path_upper_e3[31:1],
                                           pred_correct_npc_e3,
                                           i0_flush_upper_e3}));

   rvdffe #(32) i1_upper_flush_e3_ff (.*,
                                    .en  ( i1_e3_ctl_en),
                                    .din ({i1_valid_e2,
                                           i1_flush_path_upper_e2[31:1]}),
                                    .dout({i1_valid_e3,
                                           i1_flush_path_upper_e3[31:1]}));

   rvdffe #(31+pt.NUM_THREADS*32) i0_upper_flush_e4_ff (.*,
                                    .en  ( i0_e4_ctl_en | i1_e4_ctl_en),
                                    .din ({i0_flush_path_upper_e3[31:1],
                                           pred_correct_npc_e3,
                                           i0_flush_upper_e3}),
                                    .dout({i0_flush_path_upper_e4[31:1],
                                           pred_correct_npc_e4,
                                           i0_flush_upper_e4}));

   rvdffe #(32) i1_upper_flush_e4_ff (.*,
                                    .en  ( i1_e4_ctl_en),
                                    .din ({i1_valid_e3,
                                           i1_flush_path_upper_e3[31:1]}),
                                    .dout({i1_valid_e4,
                                           i1_flush_path_upper_e4[31:1]}));


   // npc for commit

   rvdff #(2) pred_correct_upper_e2_ff  (.*,
                                         .clk ( active_clk),
                                         .din ({i1_pred_correct_upper_e1,i0_pred_correct_upper_e1}),
                                         .dout({i1_pred_correct_upper_e2,i0_pred_correct_upper_e2}));

   rvdff #(2) pred_correct_upper_e3_ff  (.*,
                                         .clk ( active_clk),
                                         .din ({i1_pred_correct_upper_e2,i0_pred_correct_upper_e2}),
                                         .dout({i1_pred_correct_upper_e3,i0_pred_correct_upper_e3}));

   rvdff #(2) pred_correct_upper_e4_ff  (.*,
                                         .clk ( active_clk),
                                         .din ({i1_pred_correct_upper_e3,i0_pred_correct_upper_e3}),
                                         .dout({i1_pred_correct_upper_e4,i0_pred_correct_upper_e4}));

   rvdff #(2) sec_decode_e4_ff          (.*,
                                         .clk ( active_clk),
                                         .din ({dec_i0_sec_decode_e3,dec_i1_sec_decode_e3}),
                                         .dout({i0_sec_decode_e4,i1_sec_decode_e4}));



   assign i1_pred_correct_e4_eff     = (i1_sec_decode_e4) ? i1_pred_correct_lower_e4 : i1_pred_correct_upper_e4;
   assign i0_pred_correct_e4_eff     = (i0_sec_decode_e4) ? i0_pred_correct_lower_e4 : i0_pred_correct_upper_e4;

   assign i1_flush_path_e4_eff[31:1] = (i1_sec_decode_e4) ? exu_i1_flush_path_e4[31:1] : i1_flush_path_upper_e4[31:1];
   assign i0_flush_path_e4_eff[31:1] = (i0_sec_decode_e4) ? exu_i0_flush_path_e4[31:1] : i0_flush_path_upper_e4[31:1];


   for (genvar i=0; i<pt.NUM_THREADS; i++) begin
     assign i1_valid_e4_eff[i]  =  i1_valid_e4 & (i1_ap_e4.tid==i) & ~((i0_sec_decode_e4 & (i0_ap_e4.tid==i)) ?  exu_i0_flush_lower_e4[i]  :  i0_flush_upper_e4[i]);

     assign exu_npc_e4[i][31:1] = (i1_valid_e4_eff[i]) ? ((i1_pred_correct_e4_eff & (i1_ap_e4.tid==i)) ? pred_correct_npc_e4[i][31:1] : i1_flush_path_e4_eff[31:1]) :
                                                         ((i0_pred_correct_e4_eff & (i0_ap_e4.tid==i)) ? pred_correct_npc_e4[i][31:1] : i0_flush_path_e4_eff[31:1]);
   end


endmodule // exu
