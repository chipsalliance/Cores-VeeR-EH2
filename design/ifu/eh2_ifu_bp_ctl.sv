//********************************************************************************
// SPDX-License-Identifier: Apache-2.0
// Copyright 2020 Western Digital Corporation or its affiliates.
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
// Function: Branch predictor
// Comments:
//
//
//********************************************************************************

module eh2_ifu_bp_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (

   input logic clk,
   input logic active_clk,
   input logic rst_l,

   input logic ifc_select_tid_f1, // TID at F1
   input logic ic_hit_f2,      // Icache hit, enables F2 address capture

   input logic [31:1] ifc_fetch_addr_bf, // look up btb address
   input [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ifc_fetch_btb_rd_addr_f1, // btb read hash
   input [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ifc_fetch_btb_rd_addr_p1_f1, // btb read hash
   input logic [31:1] ifc_fetch_addr_f1, // look up btb address
   input logic ifc_fetch_req_f1,  // F1 valid
   input logic ifc_fetch_req_f2,  // F2 valid

   input eh2_br_tlu_pkt_t dec_tlu_br0_wb_pkt, // BP commit update packet, includes errors
   input eh2_br_tlu_pkt_t dec_tlu_br1_wb_pkt, // BP commit update packet, includes errors
   input logic [pt.BHT_GHR_SIZE-1:0] dec_tlu_br0_fghr_wb, // fghr to bp
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br0_index_wb, // bp index
   input logic [pt.BHT_GHR_SIZE-1:0] dec_tlu_br1_fghr_wb, // fghr to bp
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br1_index_wb, // bp index

   input logic [$clog2(pt.BTB_SIZE)-1:0] dec_fa_error_index, // Fully associt btb error index

   input logic [pt.NUM_THREADS-1:0] dec_tlu_flush_lower_wb, // used to move EX4 RS to EX1 and F
   input logic [pt.NUM_THREADS-1:0] dec_tlu_flush_leak_one_wb, // don't hit for leak one fetches

   input logic dec_tlu_bpred_disable, // disable all branch prediction

   input logic [pt.NUM_THREADS-1:0] dec_tlu_btb_write_kill, // Kill writes while working on forward progress after a branch error

   input logic        exu_i0_br_ret_e4, // EX4 ret stack update
   input logic        exu_i1_br_ret_e4, // EX4 ret stack update
   input logic        exu_i0_br_call_e4, // EX4 ret stack update
   input logic        exu_i1_br_call_e4, // EX4 ret stack update
   input logic dec_i0_tid_e4, // needed to maintain RS in BP
   input logic dec_i1_tid_e4,

   input logic [pt.NUM_THREADS-1:0][31:1] exu_flush_path_final, // flush fetch address

   input eh2_predict_pkt_t [pt.NUM_THREADS-1:0] exu_mp_pkt, // mispredict packet(s)
   input logic [pt.NUM_THREADS-1:0][pt.BTB_TOFFSET_SIZE-1:0] exu_mp_toffset, // target offset

   input logic [pt.NUM_THREADS-1:0][pt.BHT_GHR_SIZE-1:0] exu_mp_eghr, // execute ghr (for patching fghr)
   input logic [pt.NUM_THREADS-1:0][pt.BHT_GHR_SIZE-1:0] exu_mp_fghr,                    // Mispredict fghr
   input logic [pt.NUM_THREADS-1:0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] exu_mp_index,         // Mispredict index
   input logic [pt.NUM_THREADS-1:0][pt.BTB_BTAG_SIZE-1:0] exu_mp_btag,                   // Mispredict btag

   input logic [pt.NUM_THREADS-1:0] exu_flush_final, // all flushes

   // For sram btb
   output logic                                              btb_sram_rw,
   output logic [1:0] [pt.BTB_ADDR_HI:1]                     btb_sram_rw_addr,
   output logic [1:0] [pt.BTB_ADDR_HI:1]                     btb_sram_rw_addr_f1,
   output logic [1:0] [pt.BTB_BTAG_SIZE-1:0]                 btb_sram_rd_tag_f1,
   output logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0] btb_sram_wr_data,

   output logic btb_wr_stall, // simul MPs, stall fetch for 2 cycles
   input eh2_btb_sram_pkt btb_sram_pkt,

   input logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank0_rd_data_f1,
   input logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank1_rd_data_f1,
   input logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank2_rd_data_f1,
   input logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank3_rd_data_f1,


   // end sram btb ports

   output logic ifu_bp_kill_next_f2, // kill next fetch, taken target found
   output logic [31:1] ifu_bp_btb_target_f2, //  predicted target PC
   output logic [3:1] ifu_bp_inst_mask_f2, // tell ic which valids to kill because of a taken branch, right justified

   output logic [pt.BHT_GHR_SIZE-1:0] ifu_bp_fghr_f2, // fetch ghr

   output logic [3:0] ifu_bp_way_f2, // way
   output logic [3:0] ifu_bp_ret_f2, // predicted ret
   output logic [3:0] ifu_bp_hist1_f2, // history counters for all 4 potential branches, bit 1, right justified
   output logic [3:0] ifu_bp_hist0_f2, // history counters for all 4 potential branches, bit 0, right justified
   output logic [pt.BTB_TOFFSET_SIZE-1:0] ifu_bp_poffset_f2, // predicted target
   output logic [3:0] ifu_bp_pc4_f2, // pc4 indication, right justified
   output logic [3:0] ifu_bp_valid_f2, // branch valid, right justified

   output logic [3:0] [$clog2(pt.BTB_SIZE)-1:0]    ifu_bp_fa_index_f2, // predicted branch index (fully associative option)

   input  logic       scan_mode
   );

   localparam  BTB_DWIDTH =  pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5;
   localparam  BTB_DWIDTH_TOP =  int'(pt.BTB_TOFFSET_SIZE)+int'(pt.BTB_BTAG_SIZE)+4;
`define RV_TAG BTB_DWIDTH-1:BTB_DWIDTH-pt.BTB_BTAG_SIZE

   localparam BTB_FA_INDEX = $clog2(pt.BTB_SIZE)-1;
   localparam FA_CMP_LOWER = $clog2(pt.ICACHE_LN_SZ);
   localparam FA_TAG_END_UPPER= 5+int'(pt.BTB_TOFFSET_SIZE)+int'(FA_CMP_LOWER)-1; // must cast to int or vcs build fails
   localparam FA_TAG_START_LOWER = 3+int'(pt.BTB_TOFFSET_SIZE)+int'(FA_CMP_LOWER);
   localparam FA_TAG_END_LOWER = 5+int'(pt.BTB_TOFFSET_SIZE);
   localparam PC4=4;
   localparam BOFF=3;
   localparam CALL=2;
   localparam RET=1;
   localparam BV=0;

   localparam LRU_SIZE=pt.BTB_ARRAY_DEPTH;
   localparam NUM_BHT_LOOP = (pt.BHT_ARRAY_DEPTH > 16 ) ? 16 : pt.BHT_ARRAY_DEPTH;
   localparam NUM_BHT_LOOP_INNER_HI =  (pt.BHT_ARRAY_DEPTH > 16 ) ? pt.BHT_ADDR_LO+3 : pt.BHT_ADDR_HI;
   localparam NUM_BHT_LOOP_OUTER_LO =  (pt.BHT_ARRAY_DEPTH > 16 ) ? pt.BHT_ADDR_LO+4 : pt.BHT_ADDR_LO;
   localparam BHT_NO_ADDR_MATCH     =  (pt.BHT_ARRAY_DEPTH <= 16 );

   logic [31:1]       ifc_fetch_addr_f2; // to tgt calc

   logic [pt.NUM_THREADS-1:0] exu_mp_valid_write, middle_of_bank;
   logic [pt.NUM_THREADS-1:0] exu_mp_ataken;
   logic [pt.NUM_THREADS-1:0] exu_mp_valid; // conditional branch mispredict
   logic [pt.NUM_THREADS-1:0] exu_mp_boffset; // branch offsett
   logic [pt.NUM_THREADS-1:0] exu_mp_pc4; // branch is a 4B inst
   logic [pt.NUM_THREADS-1:0] exu_mp_call; // branch is a call inst
   logic [pt.NUM_THREADS-1:0] exu_mp_ret; // branch is a ret inst
   logic [pt.NUM_THREADS-1:0] exu_mp_ja; // branch is a jump always
   logic [pt.NUM_THREADS-1:0] exu_mp_bank; // write bank; based on branch PC[3:2]
   logic [pt.NUM_THREADS-1:0] [1:0] exu_mp_hist; // new history
   logic [pt.NUM_THREADS-1:0] [pt.BTB_TOFFSET_SIZE-1:0] exu_mp_tgt; // target offset
   logic [pt.NUM_THREADS-1:0] [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] exu_mp_addr; // BTB/BHT address

   logic [1:0] [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0] btb_wr_data;

   logic                                   dec_tlu_br0_v_wb; // WB stage history update
   logic [1:0]                             dec_tlu_br0_hist_wb; // new history
   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br0_addr_wb; // addr
   logic                                   dec_tlu_br0_bank_wb; // write bank; based on branch PC[3:2]
   logic                                   dec_tlu_br0_error_wb; // error; invalidate bank
   logic                                   dec_tlu_br0_start_error_wb; // error; invalidate all 4 banks in fg

   logic                                   dec_tlu_br1_v_wb; // WB stage history update
   logic [1:0]                             dec_tlu_br1_hist_wb; // new history
   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br1_addr_wb; // addr
   logic                                   dec_tlu_br1_bank_wb; // write bank; based on branch PC[3:2]
   logic                                   dec_tlu_br1_error_wb; // error
   logic                                   dec_tlu_br1_start_error_wb; // error; invalidate all 4 banks in fg

   logic [1:0]        use_mp_way, use_mp_way_p1;
   logic [pt.NUM_THREADS-1:0] [pt.RET_STACK_SIZE-1:0][31:0] rets_out, rets_in;
   logic [pt.NUM_THREADS-1:0] [pt.RET_STACK_SIZE-1:0]   rsenable;
   logic                                ifc_select_tid_f2;

   logic [pt.NUM_THREADS-1:0][pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] mp_hashed;
   logic [pt.BTB_TOFFSET_SIZE-1:0]       btb_rd_tgt_f2;
   logic              btb_rd_pc4_f2,  btb_rd_call_f2, btb_rd_ret_f2;
   logic [2:1]        bp_total_branch_offset_f2;

   logic [31:1]       bp_btb_target_adder_f2;
   logic [31:1]       bp_rs_call_target_f2;
   logic [pt.NUM_THREADS-1:0]         rs_push, rs_pop, rs_hold, rs_push_mp, rs_pop_mp, fetch_mp_collision_f1, fetch_mp_collision_f2,fetch_mp_collision_p1_f1, fetch_mp_collision_p1_f2;
   logic [pt.NUM_THREADS-1:0][pt.BTB_BTAG_SIZE-1:0] btb_wr_tag;
   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] btb_rd_addr_f1, btb_rd_addr_p1_f1, btb_rd_addr_f2, btb_rd_addr_p1_f2;
   logic [pt.BTB_BTAG_SIZE-1:0] fetch_rd_tag_f1, fetch_rd_tag_p1_f1, fetch_rd_tag_f2, fetch_rd_tag_p1_f2;
   logic [1:0]         btb_wr_en_error_way0, btb_wr_en_error_way1;

   logic [pt.BTB_BTAG_SIZE-1:0] fetch_rd_tag_bf, fetch_rd_tag_p1_bf;
   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] btb_rd_addr_bf, btb_rd_addr_p1_bf;
   logic btb_sram_wr_t0, btb_sram_wr_t1;
   logic [pt.BTB_ADDR_HI:1] btb_sram_wr_addr, btb_sram_wr_addr_f1;
   logic [1:0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] btb_sram_rd_index, btb_sram_rd_index_f1;

   logic [pt.NUM_THREADS-1:0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] btb_wr_addr;


   logic               dec_tlu_error_wb, dec_tlu_all_banks_error_wb, dec_tlu_br0_middle_wb, dec_tlu_br1_middle_wb;
   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]        btb_error_addr_wb;
   logic               dec_tlu_error_bank_wb;
   logic branch_error_collision_f1, branch_error_collision_p1_f1;

   logic [2:0] fgmask_f2;
   logic [1:0] branch_error_bank_conflict_f1, branch_error_bank_conflict_f2;
   logic [pt.BHT_GHR_SIZE-1:0] merged_ghr;
   logic [pt.NUM_THREADS-1:0][pt.BHT_GHR_SIZE-1:0] fghr_ns, fghr;
   logic [2:0] num_valids;
   logic [LRU_SIZE-1:0] btb_lru_b0_f, btb_lru_b0_hold, btb_lru_b0_ns, btb_lru_b1_f, btb_lru_b1_hold, btb_lru_b1_ns,
                        fetch_wrindex_dec, fetch_wrindex_p1_dec, fetch_wrlru_b0, fetch_wrlru_b1, fetch_wrlru_p1_b0,
                        fetch_wrlru_p1_b1, wr0_b0w0, wr0_b0w1, wr0_b1w0, wr0_b1w1, wr1_b0w0, wr1_b0w1, wr1_b1w0, wr1_b1w1;

   logic [pt.NUM_THREADS-1:0][LRU_SIZE-1:0] mp_wrindex_dec, mp_wrlru_b0, mp_wrlru_b1;
   logic [1:0]          btb_lru_rd_f2, btb_lru_rd_p1_f2,lru_update_valid_f2, lru_update_valid_p1_f2;

   logic [1:0] tag_match_way0_f2, tag_match_way1_f2;
   logic [3:0] way_raw, bht_dir_f2, btb_sel_f2, wayhit_f2, vwayhit_f2, wayhit_p1_f2;
   logic [3:0] btb_sel_mask_f2, bht_valid_f2, bht_force_taken_f2;

   logic [pt.NUM_THREADS-1:0] leak_one_f1, leak_one_f2, exu_mp_way, exu_mp_way_f;
   logic ifc_fetch_req_f2_raw;

   logic [LRU_SIZE-1:0][BTB_DWIDTH-1:0]  btb_bank0_rd_data_way0_out ;
   logic [LRU_SIZE-1:0][BTB_DWIDTH-1:0]  btb_bank1_rd_data_way0_out ;

   logic [LRU_SIZE-1:0][BTB_DWIDTH-1:0]  btb_bank0_rd_data_way1_out ;
   logic [LRU_SIZE-1:0][BTB_DWIDTH-1:0]  btb_bank1_rd_data_way1_out ;

   logic                [BTB_DWIDTH-1:0] btb_bank0_rd_data_way0_f2_in ;
   logic                [BTB_DWIDTH-1:0] btb_bank1_rd_data_way0_f2_in ;
   logic                [BTB_DWIDTH-1:0] btb_bank0_rd_data_way1_f2_in ;
   logic                [BTB_DWIDTH-1:0] btb_bank1_rd_data_way1_f2_in ;

   logic                [BTB_DWIDTH-1:0] btb_bank0_rd_data_way0_p1_f2_in ;
   logic                [BTB_DWIDTH-1:0] btb_bank1_rd_data_way0_p1_f2_in ;
   logic                [BTB_DWIDTH-1:0] btb_bank0_rd_data_way1_p1_f2_in ;
   logic                [BTB_DWIDTH-1:0] btb_bank1_rd_data_way1_p1_f2_in ;


   logic                [BTB_DWIDTH-1:0] btb_bank0_rd_data_way0_f2, btb_bank0_rd_data_way0_p1_f2;
   logic                [BTB_DWIDTH-1:0] btb_bank1_rd_data_way0_f2, btb_bank1_rd_data_way0_p1_f2;

   logic                [BTB_DWIDTH-1:0] btb_bank0_rd_data_way1_f2, btb_bank0_rd_data_way1_p1_f2;
   logic                [BTB_DWIDTH-1:0] btb_bank1_rd_data_way1_f2, btb_bank1_rd_data_way1_p1_f2;
   logic                [BTB_DWIDTH-1:0] btb_vbank0_rd_data_f2, btb_vbank1_rd_data_f2, btb_vbank2_rd_data_f2, btb_vbank3_rd_data_f2;

   logic                                         final_h;
   logic                                         btb_fg_crossing_f2;


   logic [1:0]                                   bht_vbank0_rd_data_f2, bht_vbank1_rd_data_f2, bht_vbank2_rd_data_f2, bht_vbank3_rd_data_f2,
                                                 branch_error_bank_conflict_p1_f1, branch_error_bank_conflict_p1_f2, tag_match_way0_p1_f2, tag_match_way1_p1_f2;

   logic [3:0]                                   btb_vlru_rd_f2, fetch_start_f2, tag_match_vway1_expanded_f2, tag_match_way0_expanded_p1_f2, tag_match_way1_expanded_p1_f2;
   logic [31:3] fetch_addr_p1_f1, fetch_addr_p1_f2;

   logic dec_tlu_br0_way_wb, dec_tlu_br1_way_wb, dec_tlu_way_wb, dec_tlu_way_wb_f;

   logic                [BTB_DWIDTH-1:0] btb_bank0e_rd_data_f2, btb_bank0e_rd_data_p1_f2;
   logic                [BTB_DWIDTH-1:0] btb_bank1e_rd_data_f2, btb_bank1e_rd_data_p1_f2;

   logic                [BTB_DWIDTH-1:0] btb_bank0o_rd_data_f2, btb_bank0o_rd_data_p1_f2;
   logic                [BTB_DWIDTH-1:0] btb_bank1o_rd_data_f2;

   logic [3:0] tag_match_way0_expanded_f2, tag_match_way1_expanded_f2;


   logic [1:0] bht_bank0_rd_data_f2 ;
   logic [1:0] bht_bank1_rd_data_f2 ;
   logic [1:0] bht_bank2_rd_data_f2 ;
   logic [1:0] bht_bank3_rd_data_f2 ;
   logic [1:0] bht_bank0_rd_data_p1_f2 ;
   logic [1:0] bht_bank1_rd_data_p1_f2 ;
   logic [1:0] bht_bank2_rd_data_p1_f2 ;
   logic [pt.NUM_THREADS-1:0][1:0] bht_wr_data0_thr, mp_bank_decoded, mp_bank_decoded_f;
   logic [1:0] bht_wr_data0, bht_wr_data1, bht_wr_data2, bht_wr_data3;
   logic [pt.NUM_THREADS-1:0][3:0] bht_wr_en0_thr;
   logic [3:0]                     bht_wr_en0, bht_wr_en1, bht_wr_en2, bht_wr_en3;
   logic [3:0] [(pt.BHT_ARRAY_DEPTH/NUM_BHT_LOOP)-1:0][NUM_BHT_LOOP-1:0][1:0]      bht_bank_wr_data ;
   logic [3:0] [pt.BHT_ARRAY_DEPTH-1:0] [1:0]                bht_bank_rd_data_out ;
   logic [1:0]                                                bht_bank0_rd_data_f2_in, bht_bank1_rd_data_f2_in, bht_bank2_rd_data_f2_in, bht_bank3_rd_data_f2_in;
   logic [1:0]                                                bht_bank0_rd_data_p1_f2_in, bht_bank1_rd_data_p1_f2_in, bht_bank2_rd_data_p1_f2_in;
   logic [3:0] [(pt.BHT_ARRAY_DEPTH/NUM_BHT_LOOP)-1:0]                 bht_bank_clken ;
   logic [3:0] [(pt.BHT_ARRAY_DEPTH/NUM_BHT_LOOP)-1:0]                 bht_bank_clk   ;
   logic [3:0] [(pt.BHT_ARRAY_DEPTH/NUM_BHT_LOOP)-1:0][NUM_BHT_LOOP-1:0]           bht_bank_sel   ;

   logic [pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] bht_rd_addr_f1, bht_rd_addr_p1_f1, bht_wr_addr0, bht_wr_addr1, bht_wr_addr2, bht_wr_addr3;

   logic [pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] br0_hashed_wb, br1_hashed_wb, bht_rd_addr_hashed_f1, bht_rd_addr_hashed_p1_f1;
   logic [pt.NUM_THREADS-1:0] rs_overpop_correct, rsoverpop_valid_ns, rsoverpop_valid_f;
   logic [pt.NUM_THREADS-1:0] [31:0] rsoverpop_ns, rsoverpop_f;
   logic [pt.NUM_THREADS-1:0] rsunderpop_valid_ns, rsunderpop_valid_f, rs_underpop_correct,
                              exu_i0_br_call_e4_thr, exu_i1_br_call_e4_thr, exu_i0_br_ret_e4_thr, exu_i1_br_ret_e4_thr;
   logic [31:3] adder_pc_in_f2;
   logic [pt.NUM_THREADS-1:0][31:3] ifc_fetch_adder_prior;
   logic [3:0] bloc_f2;
   logic use_fa_plus, btb_sram_rw_f1;
   logic [3:0] hist0_raw, hist1_raw, pc4_raw, pret_raw;
   logic [pt.BTB_TOFFSET_SIZE+4:1] btb_sel_data_f2;
   logic eoc_near;
   logic [3:1] eoc_mask;
   logic mp_collision, mp_collision_winner_tid, mp_bht_collision, mp_bht_collision_winner_tid;
   logic [pt.NUM_THREADS-1:0] fetch_req_val_f2;
   logic[3:1] btb_vmask_f2;
   logic [3:1] btb_vmask_raw_f2;


   assign dec_tlu_br0_v_wb = dec_tlu_br0_wb_pkt.valid;
   assign dec_tlu_br0_hist_wb[1:0]  = dec_tlu_br0_wb_pkt.hist[1:0];
   assign dec_tlu_br0_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] = dec_tlu_br0_index_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];
   assign dec_tlu_br0_bank_wb  = dec_tlu_br0_wb_pkt.bank;
   assign dec_tlu_br0_error_wb = dec_tlu_br0_wb_pkt.br_error;
   assign dec_tlu_br0_middle_wb = dec_tlu_br0_wb_pkt.middle;
   assign dec_tlu_br0_way_wb = dec_tlu_br0_wb_pkt.way;
   assign dec_tlu_br0_start_error_wb = dec_tlu_br0_wb_pkt.br_start_error;

   assign dec_tlu_br1_v_wb = dec_tlu_br1_wb_pkt.valid;
   assign dec_tlu_br1_hist_wb[1:0]  = dec_tlu_br1_wb_pkt.hist[1:0];
   assign dec_tlu_br1_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] = dec_tlu_br1_index_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];
   assign dec_tlu_br1_bank_wb  = dec_tlu_br1_wb_pkt.bank;
   assign dec_tlu_br1_middle_wb = dec_tlu_br1_wb_pkt.middle;
   assign dec_tlu_br1_error_wb = dec_tlu_br1_wb_pkt.br_error;
   assign dec_tlu_br1_way_wb = dec_tlu_br1_wb_pkt.way;
   assign dec_tlu_br1_start_error_wb = dec_tlu_br1_wb_pkt.br_start_error;



   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] t0_error_lockout_index_ns, t1_error_lockout_index_ns, t0_error_lockout_index, t1_error_lockout_index;
   logic error_mp_collision, dec_tlu_error_tid;


   // ----------------------------------------------------------------------
   // READ
   // ----------------------------------------------------------------------

   // hash the incoming fetch PC, first guess at hashing algorithm

   // 2way SA
   // Index is hi:3
   //
   if (pt.NUM_THREADS == 1) begin
      assign btb_wr_data[1] = '0;
   end

   logic [31:3] fetch_addr_p1_bf;
   assign fetch_addr_p1_bf[31:3] = ifc_fetch_addr_bf[31:3] + 29'b1;
   eh2_btb_addr_hash #(.pt(pt)) f1hash(.pc(ifc_fetch_addr_bf[pt.BTB_INDEX3_HI:pt.BTB_INDEX1_LO]), .hash(btb_rd_addr_bf[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]));
   eh2_btb_addr_hash #(.pt(pt)) f1hash_p1(.pc(fetch_addr_p1_bf[pt.BTB_INDEX3_HI:pt.BTB_INDEX1_LO]), .hash(btb_rd_addr_p1_bf[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]));



   assign fetch_addr_p1_f1[31:3] = ifc_fetch_addr_f1[31:3] + 29'b1;

   // Timing
   assign btb_rd_addr_f1 = ifc_fetch_btb_rd_addr_f1;
   assign btb_rd_addr_p1_f1 = ifc_fetch_btb_rd_addr_p1_f1;


   // Put the table below in a file and run espresso to generate the btb_sel_f2 and btb_vmask_raw_f2 equations
   // espresso -oeqntott -eeat <file> | addassign
   //
   // .i 4
   // .o 7
   // .ilb bht_dir_f2[3] bht_dir_f2[2] bht_dir_f2[1] bht_dir_f2[0]
   // .ob btb_sel_f2[3] btb_sel_f2[2] btb_sel_f2[1] btb_sel_f2[0] btb_vmask_raw_f2[3] btb_vmask_raw_f2[2] btb_vmask_raw_f2[1]
   // .type fr
   // ##dir[3:0] sel[3:0] mask[3:1]
   //   ---1 0001 000
   //   --10 0010 001
   //   -100 0100 010
   //   1000 1000 100
   //

assign btb_sel_f2[3] = (~bht_dir_f2[2] & ~bht_dir_f2[1] & ~bht_dir_f2[0]);

assign btb_sel_f2[2] = (bht_dir_f2[2] & ~bht_dir_f2[1] & ~bht_dir_f2[0]);

assign btb_sel_f2[1] = (bht_dir_f2[1] & ~bht_dir_f2[0]);

assign btb_sel_f2[0] = (bht_dir_f2[0]);

assign btb_vmask_raw_f2[3] = (~bht_dir_f2[2] & ~bht_dir_f2[1] & ~bht_dir_f2[0]);

assign btb_vmask_raw_f2[2] = (bht_dir_f2[2] & ~bht_dir_f2[1] & ~bht_dir_f2[0]);

assign btb_vmask_raw_f2[1] = (bht_dir_f2[1] & ~bht_dir_f2[0]);

   // vmask[0] is always 1
   assign btb_vmask_f2[3:1] = { btb_vmask_raw_f2[3],
                               |btb_vmask_raw_f2[3:2],
                               |btb_vmask_raw_f2[3:1]};


   assign fetch_start_f2[3:0] = decode2_4(ifc_fetch_addr_f2[2:1]);

   rvdff #(2) fetch_ff (.*, .clk(active_clk),
                         .din({dec_tlu_way_wb, ifc_fetch_req_f1}),
                        .dout({dec_tlu_way_wb_f, ifc_fetch_req_f2_raw}));


   eh2_predict_pkt_t exu_mp_pkt_t1_f;
   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] exu_mp_index_t1_f;
   logic [pt.BTB_BTAG_SIZE-1:0]          exu_mp_btag_t1_f;
   logic [pt.BTB_TOFFSET_SIZE-1:0]       exu_mp_toffset_t1_f;


if(pt.BTB_USE_SRAM) begin
   //--------------------------------------------------------------------------------
   // BTB SRAM support

   // MP stalls fetching to allow write. Multiple thread MPs stalls for multiple cycles
   //
   // Handle T0/T1 simul followed by T0/T1 simul

   rvdffe #($bits(eh2_btb_sram_pkt)) srampkt (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_sram_pkt),
                    .dout       ({wayhit_f2[3:0], wayhit_p1_f2[3:0],tag_match_way0_f2[1:0],tag_match_way0_p1_f2[1:0], tag_match_vway1_expanded_f2[3:0]}));
   rvdffe #(BTB_DWIDTH) btb_vbank0_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_vbank0_rd_data_f1[BTB_DWIDTH-1:0]),
                    .dout       (btb_vbank0_rd_data_f2[BTB_DWIDTH-1:0]));
   rvdffe #(BTB_DWIDTH) btb_vbank1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_vbank1_rd_data_f1[BTB_DWIDTH-1:0]),
                    .dout       (btb_vbank1_rd_data_f2[BTB_DWIDTH-1:0]));
   rvdffe #(BTB_DWIDTH) btb_vbank2_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_vbank2_rd_data_f1[BTB_DWIDTH-1:0]),
                    .dout       (btb_vbank2_rd_data_f2[BTB_DWIDTH-1:0]));
   rvdffe #(BTB_DWIDTH) btb_vbank3_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_vbank3_rd_data_f1[BTB_DWIDTH-1:0]),
                    .dout       (btb_vbank3_rd_data_f2[BTB_DWIDTH-1:0]));

   rvdff #(pt.BTB_ADDR_HI+1) stallff (.*, .clk(active_clk),
                         .din({btb_sram_wr_addr[pt.BTB_ADDR_HI:1], btb_sram_rw}),
                        .dout({btb_sram_wr_addr_f1[pt.BTB_ADDR_HI:1], btb_sram_rw_f1}));

   assign btb_wr_stall = btb_sram_rw_f1;


   assign btb_sram_wr_t0 = exu_mp_pkt[0].misp & ~exu_mp_pkt[0].valid & ~leak_one_f2[0] & ~dec_tlu_error_wb & ~dec_tlu_bpred_disable & ~error_mp_collision;


logic btb_delayed_wr_t1;
   if(pt.NUM_THREADS > 1) begin
      logic btb_delayed_wr_t1_ns, btb_delayed_wr_t1_raw;

      // flop T1s mp info if there's a simultaneous MP on both threads
      rvdffe #($bits(eh2_predict_pkt_t)+pt.BTB_ADDR_HI+1-pt.BTB_ADDR_LO+pt.BTB_BTAG_SIZE+pt.BTB_TOFFSET_SIZE) btb_mpt1_info (.*,
                             .en(btb_delayed_wr_t1_ns),
                             .din        ({exu_mp_pkt[1], exu_mp_index[1], exu_mp_btag[1], exu_mp_toffset[1]}),
                             .dout       ({exu_mp_pkt_t1_f, exu_mp_index_t1_f, exu_mp_btag_t1_f, exu_mp_toffset_t1_f}));


      // MPs from both threads simultaneously need to stall and write back to back, assuming they aren't to the same branch
      assign btb_delayed_wr_t1_ns = btb_sram_wr_t0 & btb_sram_wr_t1;

      assign btb_sram_wr_t1 = exu_mp_pkt[1].misp & ~exu_mp_pkt[1].valid & ~leak_one_f2[1] & ~dec_tlu_error_wb & ~dec_tlu_bpred_disable & ~error_mp_collision;

      // Write address mux. T0 has priority, but T1 will be written next if there's a simultaneous MP without a collision.
      // Note that mp_index bit 3 is really PC[3] used for the bank select. Not included in index hash.
      // Collisions drop T1 update.
      // Note, using delayed mp_pkt interface from flop BTB for simplicity. Could flop locally instead.
      assign btb_sram_wr_addr[pt.BTB_ADDR_HI:1] = dec_tlu_error_wb ? {btb_error_addr_wb, dec_tlu_error_bank_wb, dec_tlu_way_wb} :
                                                  btb_sram_wr_t0 ?  // T0 MP
                                                  {exu_mp_index[0], exu_mp_pkt[0].bank, exu_mp_pkt[0].way} :
                                                  btb_delayed_wr_t1 ? // T1 MP delayed because there was also a T0 MP
                                                  {exu_mp_index_t1_f, exu_mp_pkt_t1_f.bank, exu_mp_pkt_t1_f.way} :
                                                  // T1 MP
                                                  {exu_mp_index[1], exu_mp_pkt[1].bank, exu_mp_pkt[1].way} ;


      assign btb_sram_wr_data[BTB_DWIDTH-1:0] = btb_sram_wr_t0 ? btb_wr_data[0] :
                                                btb_delayed_wr_t1 ? {exu_mp_btag_t1_f[pt.BTB_BTAG_SIZE-1:0], exu_mp_toffset_t1_f[pt.BTB_TOFFSET_SIZE-1:0],
                                                                          exu_mp_pkt_t1_f.pc4, exu_mp_pkt_t1_f.boffset, exu_mp_pkt_t1_f.pcall | exu_mp_pkt_t1_f.pja,
                                                                          exu_mp_pkt_t1_f.pret | exu_mp_pkt_t1_f.pja, ~dec_tlu_error_wb} :
                                                btb_wr_data[1];

      rvdff #(1) simul_ff (.*, .clk(active_clk),
                           .din({btb_delayed_wr_t1_ns}),
                           .dout({btb_delayed_wr_t1_raw}));

      assign btb_delayed_wr_t1 = (btb_delayed_wr_t1_raw & ~mp_collision) | dec_tlu_error_wb;

   end
   else begin
      assign btb_sram_wr_t1 = '0;
      assign btb_delayed_wr_t1 = '0;
      // Note that mp_index bit 3 is really PC[3] used for the bank select. Not included in index hash.
      assign btb_sram_wr_addr[pt.BTB_ADDR_HI:1] = dec_tlu_error_wb ? {btb_error_addr_wb, dec_tlu_error_bank_wb, dec_tlu_way_wb} :
                                                  {exu_mp_index[0], exu_mp_pkt[0].bank, exu_mp_pkt[0].way};

      assign btb_sram_wr_data[BTB_DWIDTH-1:0] = btb_wr_data[0];
   end

   assign btb_sram_rw = btb_sram_wr_t0 | btb_sram_wr_t1 | btb_delayed_wr_t1 | dec_tlu_error_wb;



   // index 1 bit 3 contains the original bit 3, not the inc'd. index 1 bit 3 is only used for bank ordering in the sram muxing code.
   assign btb_sram_rd_index[0] =  ifc_fetch_addr_bf[3] ? btb_rd_addr_p1_bf[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] : btb_rd_addr_bf[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];
   assign btb_sram_rd_index[1] = ~ifc_fetch_addr_bf[3] ? {btb_rd_addr_p1_bf[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1], ifc_fetch_addr_bf[3]} : btb_rd_addr_bf[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];


   assign btb_sram_rw_addr[0] = btb_sram_rw ? {btb_sram_wr_addr[pt.BTB_ADDR_HI:1]} :
                                {btb_sram_rd_index[0], ifc_fetch_addr_bf[2:1]};
   assign btb_sram_rw_addr[1] = btb_sram_rw ? {btb_sram_wr_addr[pt.BTB_ADDR_HI:1]} :
                                {btb_sram_rd_index[1], ifc_fetch_addr_bf[2:1]};

   // for timing, don't use bf read address for valids
   assign btb_sram_rd_index_f1[0] =  ifc_fetch_addr_f1[3] ?
                                     ifc_fetch_btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] :
                                     ifc_fetch_btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];
   assign btb_sram_rd_index_f1[1] = ~ifc_fetch_addr_f1[3] ?
                                    {ifc_fetch_btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1], ifc_fetch_addr_f1[3]} :
                                    ifc_fetch_btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];

   assign btb_sram_rw_addr_f1[0] = btb_sram_rw_f1 ? {btb_sram_wr_addr_f1[pt.BTB_ADDR_HI:1]} :
                                   {btb_sram_rd_index_f1[0], ifc_fetch_addr_f1[2:1]};
   assign btb_sram_rw_addr_f1[1] = btb_sram_rw_f1 ? {btb_sram_wr_addr_f1[pt.BTB_ADDR_HI:1]} :
                                   {btb_sram_rd_index_f1[1], ifc_fetch_addr_f1[2:1]};

   assign btb_sram_rd_tag_f1[0] =  fetch_rd_tag_f1[pt.BTB_BTAG_SIZE-1:0];
   assign btb_sram_rd_tag_f1[1] =  fetch_rd_tag_p1_f1[pt.BTB_BTAG_SIZE-1:0];

   //--------------------------------------------------------------------------------
end // if (pt.BTB_USE_SRAM)

else begin
   assign btb_wr_stall = '0;
   assign btb_sram_rw = '0;
   // Errors colliding with fetches must kill the btb/bht hit.

   assign branch_error_collision_f1 = dec_tlu_error_wb & (btb_error_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]);
   assign branch_error_collision_p1_f1 = dec_tlu_error_wb & (btb_error_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]);

   assign branch_error_bank_conflict_f1[1:0] = {2{branch_error_collision_f1}} & (decode1_2(dec_tlu_error_bank_wb) | {2{dec_tlu_all_banks_error_wb}});
   assign branch_error_bank_conflict_p1_f1[1:0] = {2{branch_error_collision_p1_f1}} & (decode1_2(dec_tlu_error_bank_wb) | {2{dec_tlu_all_banks_error_wb}});

   rvdff #(4) coll_ff (.*, .clk(active_clk),
                         .din({branch_error_bank_conflict_f1[1:0], branch_error_bank_conflict_p1_f1[1:0]}),
                        .dout({branch_error_bank_conflict_f2[1:0], branch_error_bank_conflict_p1_f2[1:0]}));

   if(!pt.BTB_FULLYA) begin


   // 2 -way SA, figure out the way hit and mux accordingly
   assign tag_match_way0_f2[1:0] = {btb_bank1_rd_data_way0_f2[BV] & (btb_bank1_rd_data_way0_f2[`RV_TAG] == fetch_rd_tag_f2[pt.BTB_BTAG_SIZE-1:0]),
                                    btb_bank0_rd_data_way0_f2[BV] & (btb_bank0_rd_data_way0_f2[`RV_TAG] == fetch_rd_tag_f2[pt.BTB_BTAG_SIZE-1:0])} &
                                   ~({2{~dec_tlu_way_wb_f}} & branch_error_bank_conflict_f2[1:0]) & {2{ifc_fetch_req_f2_raw & ~leak_one_f2[ifc_select_tid_f2]}};

   assign tag_match_way1_f2[1:0] = {btb_bank1_rd_data_way1_f2[BV] & (btb_bank1_rd_data_way1_f2[`RV_TAG] == fetch_rd_tag_f2[pt.BTB_BTAG_SIZE-1:0]),
                                    btb_bank0_rd_data_way1_f2[BV] & (btb_bank0_rd_data_way1_f2[`RV_TAG] == fetch_rd_tag_f2[pt.BTB_BTAG_SIZE-1:0])} &
                                   ~({2{dec_tlu_way_wb_f}} & branch_error_bank_conflict_f2[1:0]) & {2{ifc_fetch_req_f2_raw & ~leak_one_f2[ifc_select_tid_f2]}};


   assign tag_match_way0_p1_f2[1:0] = {btb_bank1_rd_data_way0_p1_f2[BV] & (btb_bank1_rd_data_way0_p1_f2[`RV_TAG] == fetch_rd_tag_p1_f2[pt.BTB_BTAG_SIZE-1:0]),
                                       btb_bank0_rd_data_way0_p1_f2[BV] & (btb_bank0_rd_data_way0_p1_f2[`RV_TAG] == fetch_rd_tag_p1_f2[pt.BTB_BTAG_SIZE-1:0])} &
                                      ~({2{~dec_tlu_way_wb_f}} & branch_error_bank_conflict_p1_f2[1:0]) & {2{ifc_fetch_req_f2_raw & ~leak_one_f2[ifc_select_tid_f2]}};

   assign tag_match_way1_p1_f2[1:0] = {btb_bank1_rd_data_way1_p1_f2[BV] & (btb_bank1_rd_data_way1_p1_f2[`RV_TAG] == fetch_rd_tag_p1_f2[pt.BTB_BTAG_SIZE-1:0]),
                                       btb_bank0_rd_data_way1_p1_f2[BV] & (btb_bank0_rd_data_way1_p1_f2[`RV_TAG] == fetch_rd_tag_p1_f2[pt.BTB_BTAG_SIZE-1:0])} &
                                      ~({2{dec_tlu_way_wb_f}} & branch_error_bank_conflict_p1_f2[1:0]) & {2{ifc_fetch_req_f2_raw & ~leak_one_f2[ifc_select_tid_f2]}};


   // Both ways could hit, use the offset bit to reorder

   assign tag_match_way0_expanded_f2[3:0] = {tag_match_way0_f2[1] &  (btb_bank1_rd_data_way0_f2[BOFF] ^ btb_bank1_rd_data_way0_f2[PC4]),
                                             tag_match_way0_f2[1] & ~(btb_bank1_rd_data_way0_f2[BOFF] ^ btb_bank1_rd_data_way0_f2[PC4]),
                                             tag_match_way0_f2[0] &  (btb_bank0_rd_data_way0_f2[BOFF] ^ btb_bank0_rd_data_way0_f2[PC4]),
                                             tag_match_way0_f2[0] & ~(btb_bank0_rd_data_way0_f2[BOFF] ^ btb_bank0_rd_data_way0_f2[PC4])};

   assign tag_match_way1_expanded_f2[3:0] = {tag_match_way1_f2[1] &  (btb_bank1_rd_data_way1_f2[BOFF] ^ btb_bank1_rd_data_way1_f2[PC4]),
                                             tag_match_way1_f2[1] & ~(btb_bank1_rd_data_way1_f2[BOFF] ^ btb_bank1_rd_data_way1_f2[PC4]),
                                             tag_match_way1_f2[0] &  (btb_bank0_rd_data_way1_f2[BOFF] ^ btb_bank0_rd_data_way1_f2[PC4]),
                                             tag_match_way1_f2[0] & ~(btb_bank0_rd_data_way1_f2[BOFF] ^ btb_bank0_rd_data_way1_f2[PC4])};

   assign tag_match_way0_expanded_p1_f2[3:0] = {tag_match_way0_p1_f2[1] &  (btb_bank1_rd_data_way0_p1_f2[BOFF] ^ btb_bank1_rd_data_way0_p1_f2[PC4]),
                                                tag_match_way0_p1_f2[1] & ~(btb_bank1_rd_data_way0_p1_f2[BOFF] ^ btb_bank1_rd_data_way0_p1_f2[PC4]),
                                                tag_match_way0_p1_f2[0] &  (btb_bank0_rd_data_way0_p1_f2[BOFF] ^ btb_bank0_rd_data_way0_p1_f2[PC4]),
                                                tag_match_way0_p1_f2[0] & ~(btb_bank0_rd_data_way0_p1_f2[BOFF] ^ btb_bank0_rd_data_way0_p1_f2[PC4])};

   assign tag_match_way1_expanded_p1_f2[3:0] = {tag_match_way1_p1_f2[1] &  (btb_bank1_rd_data_way1_p1_f2[BOFF] ^ btb_bank1_rd_data_way1_p1_f2[PC4]),
                                                tag_match_way1_p1_f2[1] & ~(btb_bank1_rd_data_way1_p1_f2[BOFF] ^ btb_bank1_rd_data_way1_p1_f2[PC4]),
                                                tag_match_way1_p1_f2[0] &  (btb_bank0_rd_data_way1_p1_f2[BOFF] ^ btb_bank0_rd_data_way1_p1_f2[PC4]),
                                                tag_match_way1_p1_f2[0] & ~(btb_bank0_rd_data_way1_p1_f2[BOFF] ^ btb_bank0_rd_data_way1_p1_f2[PC4])};

   assign wayhit_f2[3:0] = tag_match_way0_expanded_f2[3:0] | tag_match_way1_expanded_f2[3:0];
   assign wayhit_p1_f2[3:0] = tag_match_way0_expanded_p1_f2[3:0] | tag_match_way1_expanded_p1_f2[3:0];

   assign btb_bank1o_rd_data_f2[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_f2[3]}} & btb_bank1_rd_data_way0_f2[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{tag_match_way1_expanded_f2[3]}} & btb_bank1_rd_data_way1_f2[BTB_DWIDTH-1:0]) );
   assign btb_bank1e_rd_data_f2[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_f2[2]}} & btb_bank1_rd_data_way0_f2[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{tag_match_way1_expanded_f2[2]}} & btb_bank1_rd_data_way1_f2[BTB_DWIDTH-1:0]) );

   assign btb_bank0o_rd_data_f2[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_f2[1]}} & btb_bank0_rd_data_way0_f2[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{tag_match_way1_expanded_f2[1]}} & btb_bank0_rd_data_way1_f2[BTB_DWIDTH-1:0]) );
   assign btb_bank0e_rd_data_f2[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_f2[0]}} & btb_bank0_rd_data_way0_f2[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{tag_match_way1_expanded_f2[0]}} & btb_bank0_rd_data_way1_f2[BTB_DWIDTH-1:0]) );


   assign btb_bank1e_rd_data_p1_f2[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_p1_f2[2]}} & btb_bank1_rd_data_way0_p1_f2[BTB_DWIDTH-1:0]) |
                                                        ({BTB_DWIDTH{tag_match_way1_expanded_p1_f2[2]}} & btb_bank1_rd_data_way1_p1_f2[BTB_DWIDTH-1:0]) );
   assign btb_bank0o_rd_data_p1_f2[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_p1_f2[1]}} & btb_bank0_rd_data_way0_p1_f2[BTB_DWIDTH-1:0]) |
                                                        ({BTB_DWIDTH{tag_match_way1_expanded_p1_f2[1]}} & btb_bank0_rd_data_way1_p1_f2[BTB_DWIDTH-1:0]) );
   assign btb_bank0e_rd_data_p1_f2[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_p1_f2[0]}} & btb_bank0_rd_data_way0_p1_f2[BTB_DWIDTH-1:0]) |
                                                        ({BTB_DWIDTH{tag_match_way1_expanded_p1_f2[0]}} & btb_bank0_rd_data_way1_p1_f2[BTB_DWIDTH-1:0]) );

   // virtual bank order

   assign btb_vbank0_rd_data_f2[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{fetch_start_f2[0]}} &  btb_bank0e_rd_data_f2[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f2[1]}} &  btb_bank0o_rd_data_f2[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f2[2]}} &  btb_bank1e_rd_data_f2[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f2[3]}} &  btb_bank1o_rd_data_f2[BTB_DWIDTH-1:0]) );
   assign btb_vbank1_rd_data_f2[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{fetch_start_f2[0]}} &  btb_bank0o_rd_data_f2[BTB_DWIDTH-1:0]) |
                                                            ({BTB_DWIDTH{fetch_start_f2[1]}} &  btb_bank1e_rd_data_f2[BTB_DWIDTH-1:0]) |
                                                            ({BTB_DWIDTH{fetch_start_f2[2]}} &  btb_bank1o_rd_data_f2[BTB_DWIDTH-1:0]) |
                                                            ({BTB_DWIDTH{fetch_start_f2[3]}} &  btb_bank0e_rd_data_p1_f2[BTB_DWIDTH-1:0]) );
   assign btb_vbank2_rd_data_f2[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{fetch_start_f2[0]}} &  btb_bank1e_rd_data_f2[BTB_DWIDTH-1:0]) |
                                                            ({BTB_DWIDTH{fetch_start_f2[1]}} &  btb_bank1o_rd_data_f2[BTB_DWIDTH-1:0]) |
                                                            ({BTB_DWIDTH{fetch_start_f2[2]}} &  btb_bank0e_rd_data_p1_f2[BTB_DWIDTH-1:0]) |
                                                            ({BTB_DWIDTH{fetch_start_f2[3]}} &  btb_bank0o_rd_data_p1_f2[BTB_DWIDTH-1:0]) );
   assign btb_vbank3_rd_data_f2[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{fetch_start_f2[0]}} &  btb_bank1o_rd_data_f2[BTB_DWIDTH-1:0]) |
                                                            ({BTB_DWIDTH{fetch_start_f2[1]}} &  btb_bank0e_rd_data_p1_f2[BTB_DWIDTH-1:0]) |
                                                            ({BTB_DWIDTH{fetch_start_f2[2]}} &  btb_bank0o_rd_data_p1_f2[BTB_DWIDTH-1:0]) |
                                                            ({BTB_DWIDTH{fetch_start_f2[3]}} &  btb_bank1e_rd_data_p1_f2[BTB_DWIDTH-1:0]) );

   assign tag_match_vway1_expanded_f2[3:0] = ( ({4{fetch_start_f2[0]}} & {tag_match_way1_expanded_f2[3:0]}) |
                                               ({4{fetch_start_f2[1]}} & {tag_match_way1_expanded_p1_f2[0], tag_match_way1_expanded_f2[3:1]}) |
                                               ({4{fetch_start_f2[2]}} & {tag_match_way1_expanded_p1_f2[1:0], tag_match_way1_expanded_f2[3:2]}) |
                                               ({4{fetch_start_f2[3]}} & {tag_match_way1_expanded_p1_f2[2:0], tag_match_way1_expanded_f2[3]}) );
   end // else: !if(pt.BTB_USE_SRAM)
end
   // --------------------------------------------------------------------------------
   // --------------------------------------------------------------------------------
   // update lru
   // mp

   assign fetch_req_val_f2[0] = ifc_fetch_req_f2_raw & ~leak_one_f2[0] & ~ifc_select_tid_f2;
   if(pt.NUM_THREADS > 1) begin

      assign fetch_req_val_f2[1] = ifc_fetch_req_f2_raw & ~leak_one_f2[1] &  ifc_select_tid_f2;

      // Simultaneous MPs to the same btb entry or bht entry can occur. Arb.
      assign mp_collision = exu_mp_valid[0] & exu_mp_valid[1] & ({exu_mp_addr[0], exu_mp_way[0], exu_mp_bank[0]} == {exu_mp_addr[1], exu_mp_way[1], exu_mp_bank[1]});

      rvarbiter2 mp_arbiter (
                             .clk(active_clk),
                             .ready(exu_mp_valid[1:0] & {2{mp_collision}}),
                             .tid  (mp_collision_winner_tid),
                             .shift(mp_collision),
                             .*);
      assign mp_bht_collision = exu_mp_valid[0] & exu_mp_valid[1] & (mp_hashed[0] == mp_hashed[1]);

      rvarbiter2 mp_bht_arbiter (
                             .clk(active_clk),
                             .ready(exu_mp_valid[1:0] & {2{mp_bht_collision}}),
                             .tid  (mp_bht_collision_winner_tid),
                             .shift(mp_bht_collision),
                           .*
                             );

      if(!pt.BTB_FULLYA) begin  //

         // cross thread error/mp to the same index can livelock until loop is complete, fix by ignoring btb write until other thread makes forward progress
         assign dec_tlu_error_tid = (dec_tlu_br0_error_wb | dec_tlu_br0_start_error_wb) ? dec_tlu_br0_wb_pkt.tid : dec_tlu_br1_wb_pkt.tid;

         assign t0_error_lockout_index_ns[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] = (dec_tlu_error_wb & ~dec_tlu_error_tid) ?
                                                                           btb_error_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] : t0_error_lockout_index[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];

         assign t1_error_lockout_index_ns[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] = (dec_tlu_error_wb &  dec_tlu_error_tid) ?
                                                                           btb_error_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] : t1_error_lockout_index[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];

         rvdffie #(.WIDTH(2*(pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1)), .OVERRIDE(1)) errorindx (.*, .din({t0_error_lockout_index_ns, t1_error_lockout_index_ns}),
                                                                                             .dout({t0_error_lockout_index,    t1_error_lockout_index}));

         assign error_mp_collision = ((exu_mp_index[0] == t1_error_lockout_index) & dec_tlu_btb_write_kill[1]) |
                                     ((exu_mp_index[1] == t0_error_lockout_index) & dec_tlu_btb_write_kill[0]);
         //


      assign btb_lru_b0_hold[LRU_SIZE-1:0] = ~mp_wrlru_b0[0][LRU_SIZE-1:0] & ~mp_wrlru_b0[1][LRU_SIZE-1:0] & ~fetch_wrlru_b0[LRU_SIZE-1:0] & ~fetch_wrlru_p1_b0[LRU_SIZE-1:0];
      assign btb_lru_b1_hold[LRU_SIZE-1:0] = ~mp_wrlru_b1[0][LRU_SIZE-1:0] & ~mp_wrlru_b1[1][LRU_SIZE-1:0] & ~fetch_wrlru_b1[LRU_SIZE-1:0] & ~fetch_wrlru_p1_b1[LRU_SIZE-1:0];
      assign btb_lru_b0_ns[LRU_SIZE-1:0] = ( (btb_lru_b0_hold[LRU_SIZE-1:0] & btb_lru_b0_f[LRU_SIZE-1:0]) |
                                             (mp_wrlru_b0[0][LRU_SIZE-1:0] & {LRU_SIZE{~exu_mp_way[0]}}) |
                                             (mp_wrlru_b0[1][LRU_SIZE-1:0] & {LRU_SIZE{~exu_mp_way[1]}}) |
                                             (fetch_wrlru_b0[LRU_SIZE-1:0] & {LRU_SIZE{tag_match_way0_f2[0]}}) |
                                             (fetch_wrlru_p1_b0[LRU_SIZE-1:0] & {LRU_SIZE{tag_match_way0_p1_f2[0]}}) );

      assign btb_lru_b1_ns[LRU_SIZE-1:0] = ( (btb_lru_b1_hold[LRU_SIZE-1:0] & btb_lru_b1_f[LRU_SIZE-1:0]) |
                                             (mp_wrlru_b1[0][LRU_SIZE-1:0] & {LRU_SIZE{~exu_mp_way[0]}}) |
                                             (mp_wrlru_b1[1][LRU_SIZE-1:0] & {LRU_SIZE{~exu_mp_way[1]}}) |
                                             (fetch_wrlru_b1[LRU_SIZE-1:0] & {LRU_SIZE{tag_match_way0_f2[1]}}) |
                                             (fetch_wrlru_p1_b1[LRU_SIZE-1:0] & {LRU_SIZE{tag_match_way0_p1_f2[1]}}) );

      // Forward the mp lru information to the fetch, avoids multiple way hits later
      assign use_mp_way[1:0] = ({2{fetch_mp_collision_f2[0]}} & mp_bank_decoded_f[0][1:0]) | ({2{fetch_mp_collision_f2[1]}} & mp_bank_decoded_f[1][1:0]);
      assign use_mp_way_p1[1:0] = ({2{fetch_mp_collision_p1_f2[0]}} & mp_bank_decoded_f[0][1:0]) | ({2{fetch_mp_collision_p1_f2[1]}} & mp_bank_decoded_f[1][1:0]);

      assign btb_lru_rd_f2[0] = use_mp_way[0] ? (fetch_mp_collision_f2[0] ? exu_mp_way_f[0] : exu_mp_way_f[1]) : |(fetch_wrindex_dec[LRU_SIZE-1:0] & btb_lru_b0_f[LRU_SIZE-1:0]);
      assign btb_lru_rd_f2[1] = use_mp_way[1] ? (fetch_mp_collision_f2[0] ? exu_mp_way_f[0] : exu_mp_way_f[1]) : |(fetch_wrindex_dec[LRU_SIZE-1:0] & btb_lru_b1_f[LRU_SIZE-1:0]);

      assign btb_lru_rd_p1_f2[0] = use_mp_way_p1[0] ? (fetch_mp_collision_p1_f2[0] ? exu_mp_way_f[0] : exu_mp_way_f[1]) : |(fetch_wrindex_p1_dec[LRU_SIZE-1:0] & btb_lru_b0_f[LRU_SIZE-1:0]);
      assign btb_lru_rd_p1_f2[1] = use_mp_way_p1[1] ? (fetch_mp_collision_p1_f2[0] ? exu_mp_way_f[0] : exu_mp_way_f[1]):  |(fetch_wrindex_p1_dec[LRU_SIZE-1:0] & btb_lru_b1_f[LRU_SIZE-1:0]);
      end // if (!pt.BTB_FULLYA)


      else
        assign error_mp_collision = 'b0;

   end // if (pt.NUM_THREADS > 1)

   else begin
      assign mp_collision = 'b0;
      assign mp_collision_winner_tid = 'b0;
      assign mp_bht_collision = 'b0;
      assign mp_bht_collision_winner_tid = 'b0;
      assign error_mp_collision = 'b0;

      if(!pt.BTB_FULLYA) begin  //

      assign btb_lru_b0_hold[LRU_SIZE-1:0] = ~mp_wrlru_b0[0][LRU_SIZE-1:0] & ~fetch_wrlru_b0[LRU_SIZE-1:0] & ~fetch_wrlru_p1_b0[LRU_SIZE-1:0];
      assign btb_lru_b1_hold[LRU_SIZE-1:0] = ~mp_wrlru_b1[0][LRU_SIZE-1:0] & ~fetch_wrlru_b1[LRU_SIZE-1:0] & ~fetch_wrlru_p1_b1[LRU_SIZE-1:0];
      assign btb_lru_b0_ns[LRU_SIZE-1:0] = ( (btb_lru_b0_hold[LRU_SIZE-1:0] & btb_lru_b0_f[LRU_SIZE-1:0]) |
                                             (mp_wrlru_b0[0][LRU_SIZE-1:0] & {LRU_SIZE{~exu_mp_way}}) |
                                             (fetch_wrlru_b0[LRU_SIZE-1:0] & {LRU_SIZE{tag_match_way0_f2[0]}}) |
                                             (fetch_wrlru_p1_b0[LRU_SIZE-1:0] & {LRU_SIZE{tag_match_way0_p1_f2[0]}}) );

      assign btb_lru_b1_ns[LRU_SIZE-1:0] = ( (btb_lru_b1_hold[LRU_SIZE-1:0] & btb_lru_b1_f[LRU_SIZE-1:0]) |
                                             (mp_wrlru_b1[0][LRU_SIZE-1:0] & {LRU_SIZE{~exu_mp_way}}) |
                                             (fetch_wrlru_b1[LRU_SIZE-1:0] & {LRU_SIZE{tag_match_way0_f2[1]}}) |
                                             (fetch_wrlru_p1_b1[LRU_SIZE-1:0] & {LRU_SIZE{tag_match_way0_p1_f2[1]}}) );

      assign use_mp_way[1:0] = ({2{fetch_mp_collision_f2[0]}} & mp_bank_decoded_f[0][1:0]);
      assign use_mp_way_p1[1:0] = ({2{fetch_mp_collision_p1_f2[0]}} & mp_bank_decoded_f[0][1:0]);

      assign btb_lru_rd_f2[0] = use_mp_way[0] ? exu_mp_way_f[0] : |(fetch_wrindex_dec[LRU_SIZE-1:0] & btb_lru_b0_f[LRU_SIZE-1:0]);
      assign btb_lru_rd_f2[1] = use_mp_way[1] ? exu_mp_way_f[0] : |(fetch_wrindex_dec[LRU_SIZE-1:0] & btb_lru_b1_f[LRU_SIZE-1:0]);

      assign btb_lru_rd_p1_f2[0] = use_mp_way_p1[0] ? exu_mp_way_f[0] : |(fetch_wrindex_p1_dec[LRU_SIZE-1:0] & btb_lru_b0_f[LRU_SIZE-1:0]);
      assign btb_lru_rd_p1_f2[1] = use_mp_way_p1[1] ? exu_mp_way_f[0] : |(fetch_wrindex_p1_dec[LRU_SIZE-1:0] & btb_lru_b1_f[LRU_SIZE-1:0]);
      end
   end

   genvar     j, i;

   if(!pt.BTB_FULLYA) begin;    //

   // fetch
   assign fetch_wrindex_dec[LRU_SIZE-1:0] = {{LRU_SIZE-1{1'b0}},1'b1} <<  btb_rd_addr_f2[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];
   assign fetch_wrindex_p1_dec[LRU_SIZE-1:0] = {{LRU_SIZE-1{1'b0}},1'b1} <<  btb_rd_addr_p1_f2[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];


   assign lru_update_valid_f2[1:0] = {((wayhit_f2[2] & btb_sel_mask_f2[2]) | (wayhit_f2[3] & btb_sel_mask_f2[3])) & ifc_fetch_req_f2 & ~leak_one_f2[ifc_select_tid_f2],
                                      ((wayhit_f2[0] & btb_sel_mask_f2[0]) | (wayhit_f2[1] & btb_sel_mask_f2[1])) & ifc_fetch_req_f2 & ~leak_one_f2[ifc_select_tid_f2]};

   assign lru_update_valid_p1_f2[1:0] = {((wayhit_p1_f2[2] & btb_sel_mask_f2[2]) | (wayhit_p1_f2[3] & btb_sel_mask_f2[3])) & ifc_fetch_req_f2 & ~leak_one_f2[ifc_select_tid_f2],
                                         ((wayhit_p1_f2[0] & btb_sel_mask_f2[0]) | (wayhit_p1_f2[1] & btb_sel_mask_f2[1])) & ifc_fetch_req_f2 & ~leak_one_f2[ifc_select_tid_f2]};

   assign fetch_wrlru_b0[LRU_SIZE-1:0] = fetch_wrindex_dec[LRU_SIZE-1:0] &
                                         {LRU_SIZE{lru_update_valid_f2[0]}};
   assign fetch_wrlru_b1[LRU_SIZE-1:0] = fetch_wrindex_dec[LRU_SIZE-1:0] &
                                         {LRU_SIZE{lru_update_valid_f2[1]}};

   assign fetch_wrlru_p1_b0[LRU_SIZE-1:0] = fetch_wrindex_p1_dec[LRU_SIZE-1:0] &
                                         {LRU_SIZE{lru_update_valid_p1_f2[0]}};
   assign fetch_wrlru_p1_b1[LRU_SIZE-1:0] = fetch_wrindex_p1_dec[LRU_SIZE-1:0] &
                                         {LRU_SIZE{lru_update_valid_p1_f2[1]}};

   // rotated
   assign btb_vlru_rd_f2[3:0] = ( ({4{fetch_start_f2[0]}} & {btb_lru_rd_f2[1], btb_lru_rd_f2[1], btb_lru_rd_f2[0], btb_lru_rd_f2[0]}) |
                                  ({4{fetch_start_f2[1]}} & {btb_lru_rd_p1_f2[0], btb_lru_rd_f2[1], btb_lru_rd_f2[1], btb_lru_rd_f2[0]}) |
                                  ({4{fetch_start_f2[2]}} & {btb_lru_rd_p1_f2[0], btb_lru_rd_p1_f2[0], btb_lru_rd_f2[1], btb_lru_rd_f2[1]}) |
                                  ({4{fetch_start_f2[3]}} & {btb_lru_rd_p1_f2[1], btb_lru_rd_p1_f2[0], btb_lru_rd_p1_f2[0], btb_lru_rd_f2[1]}));

   end
   // Detect end of cache line and mask as needed
   assign eoc_near = &ifc_fetch_addr_f2[pt.ICACHE_BEAT_ADDR_HI:3];
   assign eoc_mask[3:1] = {3{~eoc_near}} | {ifc_fetch_addr_f2[2:1] == 2'b0,
                                            ~ifc_fetch_addr_f2[2],
                                            |(~ifc_fetch_addr_f2[2:1])};

if(!pt.BTB_FULLYA) begin
   assign vwayhit_f2[3:0] = ( ({4{fetch_start_f2[0]}} & {wayhit_f2[3:0]}) |
                              ({4{fetch_start_f2[1]}} & {wayhit_p1_f2[0], wayhit_f2[3:1]}) |
                              ({4{fetch_start_f2[2]}} & {wayhit_p1_f2[1:0], wayhit_f2[3:2]}) |
                              ({4{fetch_start_f2[3]}} & {wayhit_p1_f2[2:0], wayhit_f2[3]}) ) & {eoc_mask[3:1], 1'b1};

   assign way_raw[3:0] =  tag_match_vway1_expanded_f2[3:0] | (~vwayhit_f2[3:0] & btb_vlru_rd_f2[3:0]);

   // a valid taken target needs to kill the next fetch as we compute the target address
   assign ifu_bp_kill_next_f2 = |(vwayhit_f2[3:0] & hist1_raw[3:0]) & ifc_fetch_req_f2 & ~leak_one_f2[ifc_select_tid_f2] & ~dec_tlu_bpred_disable;

   rvdffe #(LRU_SIZE*2) btb_lru_ff (.*, .en(ifc_fetch_req_f2 | (|exu_mp_valid[pt.NUM_THREADS-1:0])),
                                    .din({btb_lru_b0_ns[(LRU_SIZE)-1:0],
                                          btb_lru_b1_ns[(LRU_SIZE)-1:0]}),
                                   .dout({btb_lru_b0_f[(LRU_SIZE)-1:0],
                                          btb_lru_b1_f[(LRU_SIZE)-1:0]}));
end


   // --------------------------------------------------------------------------------
   // --------------------------------------------------------------------------------

   // mux out critical hit bank for pc computation
   // This is only useful for the first taken branch in the fetch group

   assign btb_rd_tgt_f2[pt.BTB_TOFFSET_SIZE-1:0] = btb_sel_data_f2[pt.BTB_TOFFSET_SIZE+4:5];
   assign btb_rd_pc4_f2       = btb_sel_data_f2[4];
   assign btb_rd_call_f2      = btb_sel_data_f2[2];
   assign btb_rd_ret_f2       = btb_sel_data_f2[1];

   assign btb_sel_data_f2[pt.BTB_TOFFSET_SIZE+4:1] = ( ({pt.BTB_TOFFSET_SIZE+4{btb_sel_f2[3]}} & btb_vbank3_rd_data_f2[pt.BTB_TOFFSET_SIZE+4:1]) |
                                                       ({pt.BTB_TOFFSET_SIZE+4{btb_sel_f2[2]}} & btb_vbank2_rd_data_f2[pt.BTB_TOFFSET_SIZE+4:1]) |
                                                       ({pt.BTB_TOFFSET_SIZE+4{btb_sel_f2[1]}} & btb_vbank1_rd_data_f2[pt.BTB_TOFFSET_SIZE+4:1]) |
                                                       ({pt.BTB_TOFFSET_SIZE+4{btb_sel_f2[0]}} & btb_vbank0_rd_data_f2[pt.BTB_TOFFSET_SIZE+4:1]) );




   // Don't put calls/rets/ja in the predictor, force the bht taken instead
   assign bht_force_taken_f2[3:0] = {(btb_vbank3_rd_data_f2[CALL] | btb_vbank3_rd_data_f2[RET]),
                                     (btb_vbank2_rd_data_f2[CALL] | btb_vbank2_rd_data_f2[RET]),
                                     (btb_vbank1_rd_data_f2[CALL] | btb_vbank1_rd_data_f2[RET]),
                                     (btb_vbank0_rd_data_f2[CALL] | btb_vbank0_rd_data_f2[RET])};


   // taken and valid, otherwise, branch errors must clear the bht
   assign bht_valid_f2[3:0] = vwayhit_f2[3:0];

   assign bht_vbank0_rd_data_f2[1:0] = ( ({2{fetch_start_f2[0]}} & bht_bank0_rd_data_f2[1:0]) |
                                         ({2{fetch_start_f2[1]}} & bht_bank1_rd_data_f2[1:0]) |
                                         ({2{fetch_start_f2[2]}} & bht_bank2_rd_data_f2[1:0]) |
                                         ({2{fetch_start_f2[3]}} & bht_bank3_rd_data_f2[1:0]) );

   assign bht_vbank1_rd_data_f2[1:0] = ( ({2{fetch_start_f2[0]}} & bht_bank1_rd_data_f2[1:0]) |
                                         ({2{fetch_start_f2[1]}} & bht_bank2_rd_data_f2[1:0]) |
                                         ({2{fetch_start_f2[2]}} & bht_bank3_rd_data_f2[1:0]) |
                                         ({2{fetch_start_f2[3]}} & bht_bank0_rd_data_p1_f2[1:0]) );

   assign bht_vbank2_rd_data_f2[1:0] = ( ({2{fetch_start_f2[0]}} & bht_bank2_rd_data_f2[1:0]) |
                                         ({2{fetch_start_f2[1]}} & bht_bank3_rd_data_f2[1:0]) |
                                         ({2{fetch_start_f2[2]}} & bht_bank0_rd_data_p1_f2[1:0]) |
                                         ({2{fetch_start_f2[3]}} & bht_bank1_rd_data_p1_f2[1:0]) );

   assign bht_vbank3_rd_data_f2[1:0] = ( ({2{fetch_start_f2[0]}} & bht_bank3_rd_data_f2[1:0]) |
                                         ({2{fetch_start_f2[1]}} & bht_bank0_rd_data_p1_f2[1:0]) |
                                         ({2{fetch_start_f2[2]}} & bht_bank1_rd_data_p1_f2[1:0]) |
                                         ({2{fetch_start_f2[3]}} & bht_bank2_rd_data_p1_f2[1:0]) );


   assign bht_dir_f2[3:0] = {(bht_force_taken_f2[3] | bht_vbank3_rd_data_f2[1]) & bht_valid_f2[3],
                             (bht_force_taken_f2[2] | bht_vbank2_rd_data_f2[1]) & bht_valid_f2[2],
                             (bht_force_taken_f2[1] | bht_vbank1_rd_data_f2[1]) & bht_valid_f2[1],
                             (bht_force_taken_f2[0] | bht_vbank0_rd_data_f2[1]) & bht_valid_f2[0]};

   assign ifu_bp_inst_mask_f2[3:1] = ( ({3{ ifu_bp_kill_next_f2}} & btb_vmask_f2[3:1]) |
                                       ({3{~ifu_bp_kill_next_f2}} & 3'b111) );



   // Branch prediction info is sent with the 2byte lane associated with the end of the branch.
   // Cases
   //       BANK1         BANK0
   // -------------------------------
   // |      :       |      :       |
   // -------------------------------
   //         <------------>                   : PC4 branch, offset, should be in B1 (indicated on [2])
   //                <------------>            : PC4 branch, no offset, indicate PC4, VALID, HIST on [1]
   //                       <------------>     : PC4 branch, offset, indicate PC4, VALID, HIST on [0]
   //                <------>                  : PC2 branch, offset, indicate VALID, HIST on [1]
   //                       <------>           : PC2 branch, no offset, indicate VALID, HIST on [0]
   //



   assign hist1_raw[3:0] = bht_force_taken_f2[3:0] | {bht_vbank3_rd_data_f2[1],
                                                      bht_vbank2_rd_data_f2[1],
                                                      bht_vbank1_rd_data_f2[1],
                                                      bht_vbank0_rd_data_f2[1]};

   assign hist0_raw[3:0] = {bht_vbank3_rd_data_f2[0],
                            bht_vbank2_rd_data_f2[0],
                            bht_vbank1_rd_data_f2[0],
                            bht_vbank0_rd_data_f2[0]};


   assign pc4_raw[3:0] = {vwayhit_f2[3] & btb_vbank3_rd_data_f2[PC4],
                          vwayhit_f2[2] & btb_vbank2_rd_data_f2[PC4],
                          vwayhit_f2[1] & btb_vbank1_rd_data_f2[PC4],
                          vwayhit_f2[0] & btb_vbank0_rd_data_f2[PC4]};

   assign pret_raw[3:0] = {vwayhit_f2[3] & ~btb_vbank3_rd_data_f2[CALL] & btb_vbank3_rd_data_f2[RET],
                           vwayhit_f2[2] & ~btb_vbank2_rd_data_f2[CALL] & btb_vbank2_rd_data_f2[RET],
                           vwayhit_f2[1] & ~btb_vbank1_rd_data_f2[CALL] & btb_vbank1_rd_data_f2[RET],
                           vwayhit_f2[0] & ~btb_vbank0_rd_data_f2[CALL] & btb_vbank0_rd_data_f2[RET]};

   // GHR

   // Figure out how many valid branches are in the fetch group
   assign fgmask_f2[2] = (~ifc_fetch_addr_f2[1]) | (~ifc_fetch_addr_f2[2]);
   assign fgmask_f2[1] = (~ifc_fetch_addr_f2[2]);
   assign fgmask_f2[0] = (~ifc_fetch_addr_f2[2] & ~ifc_fetch_addr_f2[1]);

   assign btb_sel_mask_f2[3:0] = {btb_sel_f2[3],
                                  |btb_sel_f2[3:2] & fgmask_f2[2],
                                  |btb_sel_f2[3:1] & fgmask_f2[1],
                                  |btb_sel_f2[3:0] & fgmask_f2[0]};

  // count the valids with masking based on first taken
   assign num_valids[2:0] = countones(bht_valid_f2[3:0]);

   // Note that the following property holds
   // P: prior ghr, H: history bit of last valid branch in line (could be 1 or 0)
   // Num valid branches   What lowest bits of new GHR must be
   // 4                    000H
   // 3                    P00H
   // 2                    PP0H
   // 1                    PPPH
   // 0                    PPPP

   assign final_h = |(btb_sel_f2[3:0] & bht_dir_f2[3:0]);

   if(pt.BHT_GHR_SIZE==3) begin : fghr_shift
      assign merged_ghr[pt.BHT_GHR_SIZE-1:0] = ( ({pt.BHT_GHR_SIZE{num_valids[2:0] >= 3'h3}} & {2'b0, final_h}) | // 00H
                                                 ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h2}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-3:0], 1'b0, final_h}) | // P0H
                                                 ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h1}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-2:0], final_h}) | // PPH
                                                 ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h0}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-1:0]}) ); // PPP
   end
   else if(pt.BHT_GHR_SIZE==4) begin
      assign merged_ghr[pt.BHT_GHR_SIZE-1:0] = ( ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h4}} & {3'b0, final_h}) | // 000H
                                                 ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h3}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-4:0], 2'b0, final_h}) | // P00H
                                                 ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h2}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-3:0], 1'b0, final_h}) | // PP0H
                                                 ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h1}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-2:0], final_h}) | // PPPH
                                                 ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h0}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-1:0]}) ); // PPPP
      end
   else begin
      assign merged_ghr[pt.BHT_GHR_SIZE-1:0] = ( ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h4}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-5:0], 3'b0, final_h}) | // 000H
                                                 ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h3}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-4:0], 2'b0, final_h}) | // P00H
                                                 ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h2}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-3:0], 1'b0, final_h}) | // PP0H
                                                 ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h1}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-2:0], final_h}) | // PPPH
                                                 ({pt.BHT_GHR_SIZE{num_valids[2:0] == 3'h0}} & {fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-1:0]}) ); // PPPP
   end


   for (i=0; i<pt.NUM_THREADS; i++) begin : fghrmaint

      assign exu_mp_valid[i] = exu_mp_pkt[i].misp & ~leak_one_f2[ifc_select_tid_f2] & ~dec_tlu_error_wb; // conditional branch mispredict, unless there was a simul error
      assign exu_mp_boffset[i] = exu_mp_pkt[i].boffset;  // branch offset
      assign exu_mp_pc4[i] = exu_mp_pkt[i].pc4;  // branch is a 4B inst
      assign exu_mp_call[i] = exu_mp_pkt[i].pcall;  // branch is a call inst
      assign exu_mp_ret[i] = exu_mp_pkt[i].pret;  // branch is a ret inst
      assign exu_mp_ja[i] = exu_mp_pkt[i].pja;  // branch is a jump always
      assign exu_mp_way[i] = exu_mp_pkt[i].way;  // repl way
      assign exu_mp_hist[i][1:0] = exu_mp_pkt[i].hist[1:0];  // new history
      assign exu_mp_tgt[i][pt.BTB_TOFFSET_SIZE-1:0]  = exu_mp_toffset[i][pt.BTB_TOFFSET_SIZE-1:0] ;  // target offset
      assign exu_mp_bank[i]  = exu_mp_pkt[i].bank ;  // write bank = exu_mp_pkt[i].;  based on branch PC[3:2]
      assign exu_mp_ataken[i] = exu_mp_pkt[i].ataken;
      assign exu_mp_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]  = exu_mp_index[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ;  // BTB/BHT address

      assign btb_wr_tag[i][pt.BTB_BTAG_SIZE-1:0] = exu_mp_btag[i][pt.BTB_BTAG_SIZE-1:0];

      assign exu_mp_valid_write[i] = exu_mp_valid[i] & ~exu_mp_pkt[i].valid & exu_mp_ataken[i] & (~mp_collision | (i == mp_collision_winner_tid));
      assign btb_wr_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] = dec_tlu_error_wb ? btb_error_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] : exu_mp_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];
      assign btb_wr_data[i][BTB_DWIDTH-1:0] = {btb_wr_tag[i][pt.BTB_BTAG_SIZE-1:0], exu_mp_tgt[i][pt.BTB_TOFFSET_SIZE-1:0],
                                                exu_mp_pc4[i], exu_mp_boffset[i], exu_mp_call[i] | exu_mp_ja[i],
                                                exu_mp_ret[i] | exu_mp_ja[i], ~dec_tlu_error_wb & ~error_mp_collision} ;


      assign fghr_ns[i][pt.BHT_GHR_SIZE-1:0] = ( ({pt.BHT_GHR_SIZE{exu_flush_final[i]}} & exu_mp_fghr[i][pt.BHT_GHR_SIZE-1:0]) |
                                                 ({pt.BHT_GHR_SIZE{~exu_flush_final[i] & fetch_req_val_f2[i]}} & merged_ghr[pt.BHT_GHR_SIZE-1:0]) |
                                                 ({pt.BHT_GHR_SIZE{~exu_flush_final[i] & ~fetch_req_val_f2[i]}} & fghr[i][pt.BHT_GHR_SIZE-1:0]));

      rvdff #(pt.BHT_GHR_SIZE) fetchghr (.*, .clk(active_clk), .din(fghr_ns[i][pt.BHT_GHR_SIZE-1:0]), .dout(fghr[i][pt.BHT_GHR_SIZE-1:0]));

      eh2_btb_ghr_hash #(.pt(pt)) mpghrhs  (.hashin(exu_mp_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]),
                                             .ghr(exu_mp_eghr[i][pt.BHT_GHR_SIZE-1:0]),
                                             .hash(mp_hashed[i][pt.BHT_ADDR_HI:pt.BHT_ADDR_LO]));


      assign middle_of_bank[i] = exu_mp_pc4[i] ^ exu_mp_boffset[i];
      assign bht_wr_en0_thr[i][3:0] = {4{exu_mp_valid[i] & ~exu_mp_call[i] & ~exu_mp_ret[i] & ~exu_mp_ja[i]}} & decode2_4({exu_mp_bank[i], middle_of_bank[i]});
      // Experiments show this is the best priority scheme for same bank/index writes at the same time.
      assign bht_wr_data0_thr[i] = exu_mp_hist[i]; // lowest priority

      if(!pt.BTB_FULLYA) begin

      assign mp_bank_decoded[i][1:0] = decode1_2(exu_mp_bank[i]);
      // create a onehot lru write vector
      assign mp_wrindex_dec[i][LRU_SIZE-1:0] = {{LRU_SIZE-1{1'b0}},1'b1} <<  exu_mp_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];

      assign mp_wrlru_b0[i][LRU_SIZE-1:0] = mp_wrindex_dec[i][LRU_SIZE-1:0] & {LRU_SIZE{mp_bank_decoded[i][0] & exu_mp_valid[i] & (~mp_collision | (i == mp_collision_winner_tid))}};
      assign mp_wrlru_b1[i][LRU_SIZE-1:0] = mp_wrindex_dec[i][LRU_SIZE-1:0] & {LRU_SIZE{mp_bank_decoded[i][1] & exu_mp_valid[i] & (~mp_collision | (i == mp_collision_winner_tid))}};


      // MP/Fetch collision
      assign fetch_mp_collision_f1[i] = ( (exu_mp_btag[i][pt.BTB_BTAG_SIZE-1:0] == fetch_rd_tag_f1[pt.BTB_BTAG_SIZE-1:0]) &
                                          exu_mp_valid[i] & ifc_fetch_req_f1 & (~mp_collision | (i == mp_collision_winner_tid)) &
                                          (exu_mp_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO])
                                          );
      assign fetch_mp_collision_p1_f1[i] = ( (exu_mp_btag[i][pt.BTB_BTAG_SIZE-1:0] == fetch_rd_tag_p1_f1[pt.BTB_BTAG_SIZE-1:0]) &
                                             exu_mp_valid[i] & ifc_fetch_req_f1 & (~mp_collision | (i == mp_collision_winner_tid)) &
                                             (exu_mp_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO])
                                             );

   rvdff #(5) mpcoll_ff (.*, .clk(active_clk),
                          .din({fetch_mp_collision_f1[i], fetch_mp_collision_p1_f1[i], mp_bank_decoded[i][1:0], exu_mp_way[i]}),
                         .dout({fetch_mp_collision_f2[i], fetch_mp_collision_p1_f2[i], mp_bank_decoded_f[i][1:0], exu_mp_way_f[i]}));
         end
   end

   assign ifu_bp_fghr_f2[pt.BHT_GHR_SIZE-1:0] = fghr[ifc_select_tid_f2][pt.BHT_GHR_SIZE-1:0];


   assign ifu_bp_way_f2[3:0] = way_raw[3:0];
   assign ifu_bp_hist1_f2[3:0]    = hist1_raw[3:0];
   assign ifu_bp_hist0_f2[3:0]    = hist0_raw[3:0];
   assign ifu_bp_pc4_f2[3:0]     = pc4_raw[3:0];
   assign ifu_bp_valid_f2[3:0]   = vwayhit_f2[3:0] & ~{4{dec_tlu_bpred_disable}};
   assign ifu_bp_ret_f2[3:0]     = pret_raw[3:0];


   // compute target
   // Form the fetch group offset based on the btb hit location and the location of the branch within the 4 byte chunk

//  .i 9
//  .o 5
//  .ilb bht_dir_f2[3] bht_dir_f2[2] bht_dir_f2[1] bht_dir_f2[0] fetch_start_f2[3] fetch_start_f2[2] fetch_start_f2[1] fetch_start_f2[0] btb_rd_pc4_f2
//  .ob bloc_f2[3] bloc_f2[2] bloc_f2[1] bloc_f2[0] use_fa_plus
//  .type fr
//
//
//  ## rotdir[3:0]  fs   pc4  off fapl
//    ---1          0001 -  0001  0
//    --10          0001 -  0010  0
//    -100          0001 -  0100  0
//    1000          0001 -  1000  0
//
//    ---1          0010 -  0010  0
//    --10          0010 -  0100  0
//    -100          0010 -  1000  0
//    1000          0010 0  0001  1
//    1000          0010 1  0001  0
//
//    ---1          0100 -  0100  0
//    --10          0100 -  1000  0
//    -100          0100 0  0001  1
//    -100          0100 1  0001  0
//    1000          0100 -  0010  1
//
//    ---1          1000 -  1000  0
//    --10          1000 0  0001  1
//    --10          1000 1  0001  0
//    -100          1000 -  0010  1
//    1000          1000 -  0100  1

assign bloc_f2[3] = (!bht_dir_f2[2]&!bht_dir_f2[1]&!bht_dir_f2[0]
    &fetch_start_f2[0]) | (bht_dir_f2[2]&!bht_dir_f2[1]&!bht_dir_f2[0]
    &fetch_start_f2[1]) | (bht_dir_f2[1]&!bht_dir_f2[0]&fetch_start_f2[2]) | (
    bht_dir_f2[0]&fetch_start_f2[3]);

assign bloc_f2[2] = (bht_dir_f2[2]&!bht_dir_f2[1]&!bht_dir_f2[0]
    &fetch_start_f2[0]) | (!bht_dir_f2[2]&!bht_dir_f2[1]&!bht_dir_f2[0]
    &fetch_start_f2[3]) | (bht_dir_f2[1]&!bht_dir_f2[0]&fetch_start_f2[1]) | (
    bht_dir_f2[0]&fetch_start_f2[2]);

assign bloc_f2[1] = (!bht_dir_f2[2]&!bht_dir_f2[1]&!bht_dir_f2[0]
    &fetch_start_f2[2]) | (bht_dir_f2[2]&!bht_dir_f2[1]&!bht_dir_f2[0]
    &fetch_start_f2[3]) | (bht_dir_f2[1]&!bht_dir_f2[0]&fetch_start_f2[0]) | (
    bht_dir_f2[0]&fetch_start_f2[1]);

assign bloc_f2[0] = (!bht_dir_f2[2]&!bht_dir_f2[1]&!bht_dir_f2[0]
    &fetch_start_f2[1]) | (bht_dir_f2[2]&!bht_dir_f2[1]&!bht_dir_f2[0]
    &fetch_start_f2[2]) | (bht_dir_f2[1]&!bht_dir_f2[0]&fetch_start_f2[3]) | (
    bht_dir_f2[0]&fetch_start_f2[0]);

assign use_fa_plus = (!bht_dir_f2[2]&!bht_dir_f2[1]&!bht_dir_f2[0]
    &fetch_start_f2[1]&!btb_rd_pc4_f2) | (!bht_dir_f2[1]&!bht_dir_f2[0]
    &fetch_start_f2[2]&!btb_rd_pc4_f2) | (!bht_dir_f2[0]
    &fetch_start_f2[3]&!btb_rd_pc4_f2) | (!bht_dir_f2[2]&!bht_dir_f2[1]
    &!bht_dir_f2[0]&fetch_start_f2[3]) | (!bht_dir_f2[2]&!bht_dir_f2[1]
    &!bht_dir_f2[0]&fetch_start_f2[2]) | (bht_dir_f2[2]&!bht_dir_f2[1]
    &!bht_dir_f2[0]&fetch_start_f2[3]);



    assign btb_fg_crossing_f2 = fetch_start_f2[0] & btb_sel_f2[0] & btb_rd_pc4_f2;

   wire [1:0] btb_sel_f2_enc, btb_sel_f2_enc_shift;
   assign btb_sel_f2_enc[1:0] = encode4_2(bloc_f2[3:0]);
   assign btb_sel_f2_enc_shift[1:0] = encode4_2({bloc_f2[0],bloc_f2[3:1]});

   assign bp_total_branch_offset_f2[2:1] =  (({2{ btb_rd_pc4_f2}} &  btb_sel_f2_enc_shift[1:0]) |
                                             ({2{~btb_rd_pc4_f2}} &  btb_sel_f2_enc[1:0]) |
                                             ({2{btb_fg_crossing_f2}}));


   rvdffpcie #(31) faddrf2raw_ff (.*, .en(ifc_fetch_req_f1), .din(ifc_fetch_addr_f1[31:1]), .dout(ifc_fetch_addr_f2[31:1]));

   rvdfflie #(.WIDTH(2*(pt.BTB_ADDR_HI-2) + 29), .LEFT(20)) faddr_p1_f2ff (.*, .en(ifc_fetch_req_f1), .din({fetch_addr_p1_f1[31:3],
                                                                btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO],
                                                                btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]
                                                                }),
                                                         .dout({fetch_addr_p1_f2[31:3],
                                                                btb_rd_addr_f2[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO],
                                                                btb_rd_addr_p1_f2[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]
                                                                }));

   assign ifu_bp_poffset_f2[pt.BTB_TOFFSET_SIZE-1:0] = btb_rd_tgt_f2[pt.BTB_TOFFSET_SIZE-1:0];

   assign adder_pc_in_f2[31:3] = ( ({29{ use_fa_plus}} & fetch_addr_p1_f2[31:3]) |
                                   ({29{ btb_fg_crossing_f2}} & ifc_fetch_adder_prior[ifc_select_tid_f2][31:3]) |
                                   ({29{~btb_fg_crossing_f2 & ~use_fa_plus}} & ifc_fetch_addr_f2[31:3]));

   rvbradder predtgt_addr (.pc({adder_pc_in_f2[31:3], bp_total_branch_offset_f2[2:1]}),
                         .offset(btb_rd_tgt_f2[pt.BTB_TOFFSET_SIZE-1:0]),
                         .dout(bp_btb_target_adder_f2[31:1])
                         );
   // mux in the return stack address here for a predicted return, if it is valid
   // if no btb kill, quite the bus to 0
   assign ifu_bp_btb_target_f2[31:1] = ( ({31{btb_rd_ret_f2 & ~btb_rd_call_f2 & rets_out[ifc_select_tid_f2][0][0] & ifu_bp_kill_next_f2}} & rets_out[ifc_select_tid_f2][0][31:1]) |
                                         ({31{~(btb_rd_ret_f2 & ~btb_rd_call_f2 & rets_out[ifc_select_tid_f2][0][0]) & ifu_bp_kill_next_f2}} & bp_btb_target_adder_f2[31:1]) );



   // ----------------------------------------------------------------------
   // Return Stack
   // ----------------------------------------------------------------------

   rvbradder rs_addr (.pc({adder_pc_in_f2[31:3], bp_total_branch_offset_f2[2:1]}),
                      .offset({ {pt.BTB_TOFFSET_SIZE-2{1'b0}}, btb_rd_pc4_f2, ~btb_rd_pc4_f2}),
                      .dout(bp_rs_call_target_f2[31:1])
                      );

   // Calls/Rets are always taken, so there shouldn't be a push and pop in the same fetch group


   for (genvar tid=0; tid < pt.NUM_THREADS; tid++) begin: rs_thr

      // set on leak one, hold until next flush without leak one
      assign leak_one_f1[tid] = (dec_tlu_flush_leak_one_wb[tid] & dec_tlu_flush_lower_wb[tid]) | (leak_one_f2[tid] & ~dec_tlu_flush_lower_wb[tid]);

   rvdff #(1) leak_ff (.*, .clk(active_clk),
                         .din({leak_one_f1[tid]}),
                        .dout({leak_one_f2[tid]}));

`ifdef RS_COMMIT_EN
      assign rs_overpop_correct[tid] = rsoverpop_valid_f[tid] & exu_flush_final[tid] & ~exu_mp_ret[tid];
      assign rs_underpop_correct[tid] = rsunderpop_valid_f[tid] & exu_flush_final[tid] & ~exu_mp_call[tid];

      assign exu_i0_br_call_e4_thr[tid] = exu_i0_br_call_e4 & (tid == dec_i0_tid_e4);
      assign exu_i1_br_call_e4_thr[tid] = exu_i1_br_call_e4 & (tid == dec_i1_tid_e4);
      assign exu_i0_br_ret_e4_thr[tid] = exu_i0_br_ret_e4 & (tid == dec_i0_tid_e4);
      assign exu_i1_br_ret_e4_thr[tid] = exu_i1_br_ret_e4 & (tid == dec_i1_tid_e4);

      assign rsunderpop_valid_ns[tid] = (rs_push[tid] | (rsunderpop_valid_f[tid] & ~(exu_i0_br_call_e4_thr[tid] | exu_i1_br_call_e4_thr[tid]))) & ~exu_flush_final[tid];
      assign rsoverpop_valid_ns[tid] = (rs_pop[tid] | (rsoverpop_valid_f[tid] & ~(exu_i0_br_ret_e4_thr[tid] | exu_i1_br_ret_e4_thr[tid]))) & ~exu_flush_final[tid];
      assign rsoverpop_ns[tid][31:0] = ( ({32{rs_pop[tid]}}  & rets_out[tid][0][31:0]) |
                                         ({32{~rs_pop[tid]}} & rsoverpop_f[tid][31:0]) );

      rvdff #(34) retoverpop_ff (.*, .clk(active_clk), .din({rsunderpop_valid_ns[tid], rsoverpop_valid_ns[tid], rsoverpop_ns[tid][31:0]}), .dout({rsunderpop_valid_f[tid], rsoverpop_valid_f[tid], rsoverpop_f[tid][31:0]}));
`else
      assign rs_overpop_correct[tid] = 1'b0;
      assign rs_underpop_correct[tid] = 1'b0;
      assign rsoverpop_f[tid][31:0]  = 'b0;
`endif // !`ifdef RS_COMMIT_EN


logic [31:1] rs_push_addr;

`ifdef RS_MP_PP
      assign rs_push_addr[31:1] = rs_push_mp ? (exu_flush_path_final[tid][31:1] - {{19{exu_mp_toffset[tid][11]}}, exu_mp_toffset[tid]} + {exu_mp_pc4[tid], ~exu_mp_pc4[tid]}) : bp_rs_call_target_f2[31:1];
      assign rs_push_mp[tid] = exu_mp_valid[tid] & exu_mp_call[tid] & ~exu_mp_ret[tid];
      assign rs_pop_mp[tid] = exu_mp_valid[tid] & ~exu_mp_call[tid] & exu_mp_ret[tid] & (rets_out[tid][0][31:1] == exu_flush_path_final[tid][31:1]);
`else
      assign rs_push_addr[31:1] = bp_rs_call_target_f2[31:1];
      assign rs_push_mp[tid] = '0;
      assign rs_pop_mp[tid] = '0;
`endif

      assign rs_push[tid] = ((btb_rd_call_f2 & ~btb_rd_ret_f2 & ifu_bp_kill_next_f2 & fetch_req_val_f2[tid]) | (rs_overpop_correct[tid] & ~rs_underpop_correct[tid]) | rs_push_mp[tid]);
      assign rs_pop[tid] = ((btb_rd_ret_f2 & ~btb_rd_call_f2 & ifu_bp_kill_next_f2 & fetch_req_val_f2[tid]) | (rs_underpop_correct[tid] & ~rs_overpop_correct[tid]) | rs_pop_mp[tid]);
      assign rs_hold[tid] = ~rs_push[tid] & ~rs_pop[tid] & ~rs_overpop_correct[tid] & ~rs_underpop_correct[tid];

      // Fetch based
      assign rets_in[tid][0][31:0] = ( ({32{rs_overpop_correct[tid] & rs_underpop_correct[tid]}} & rsoverpop_f[tid][31:0]) |
                                       ({32{rs_push[tid] & rs_overpop_correct[tid]}} & rsoverpop_f[tid][31:0]) |
                                       ({32{rs_push[tid] & ~rs_overpop_correct[tid]}} & {rs_push_addr[31:1], 1'b1}) |
                                       ({32{rs_pop[tid]}}  & rets_out[tid][1][31:0]) );

      assign rsenable[tid][0] = ~rs_hold[tid];

      for (genvar i=0; i<pt.RET_STACK_SIZE; i++) begin : retstack

         // for the last entry in the stack, we don't have a pop position
         if(i==pt.RET_STACK_SIZE-1) begin
            assign rets_in[tid][i][31:0] = rets_out[tid][i-1][31:0];
            assign rsenable[tid][i] = rs_push[tid];
         end
         else if(i>0) begin
            assign rets_in[tid][i][31:0] = ( ({32{rs_push[tid]}} & rets_out[tid][i-1][31:0]) |
                                             ({32{rs_pop[tid]}}  & rets_out[tid][i+1][31:0]) );
            assign rsenable[tid][i] = rs_push[tid] | rs_pop[tid];
         end
         rvdffe #(32) rets_ff (.*, .en(rsenable[tid][i]), .din(rets_in[tid][i][31:0]), .dout(rets_out[tid][i][31:0]));

      end : retstack



   end // block: rs_thr


   // ----------------------------------------------------------------------
   // WRITE
   // ----------------------------------------------------------------------


   assign dec_tlu_error_wb = dec_tlu_br0_start_error_wb | dec_tlu_br0_error_wb | dec_tlu_br1_start_error_wb | dec_tlu_br1_error_wb;
   assign dec_tlu_all_banks_error_wb = dec_tlu_br0_start_error_wb | (~dec_tlu_br0_error_wb & dec_tlu_br1_start_error_wb);

   assign dec_tlu_error_bank_wb = (dec_tlu_br0_error_wb | dec_tlu_br0_start_error_wb) ? dec_tlu_br0_bank_wb : dec_tlu_br1_bank_wb;
   assign btb_error_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] = (dec_tlu_br0_error_wb | dec_tlu_br0_start_error_wb) ? dec_tlu_br0_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] : dec_tlu_br1_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];

   assign dec_tlu_way_wb = (dec_tlu_br0_error_wb | dec_tlu_br0_start_error_wb) ? dec_tlu_br0_way_wb : dec_tlu_br1_way_wb;

   if(!pt.BTB_FULLYA) begin


if (pt.BTB_BTAG_FOLD) begin : btbfold
   eh2_btb_tag_hash_fold #(.pt(pt)) rdtagf1(.hash(fetch_rd_tag_f1[pt.BTB_BTAG_SIZE-1:0]),
                                             .pc({ifc_fetch_addr_f1[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]}));
   eh2_btb_tag_hash_fold #(.pt(pt)) rdtagp1f1(.hash(fetch_rd_tag_p1_f1[pt.BTB_BTAG_SIZE-1:0]),
                                               .pc({fetch_addr_p1_f1[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]}));
end
   else begin
   eh2_btb_tag_hash #(.pt(pt)) rdtagf1(.hash(fetch_rd_tag_f1[pt.BTB_BTAG_SIZE-1:0]),
                                        .pc({ifc_fetch_addr_f1[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]}));
   eh2_btb_tag_hash #(.pt(pt)) rdtagp1f1(.hash(fetch_rd_tag_p1_f1[pt.BTB_BTAG_SIZE-1:0]),
                                          .pc({fetch_addr_p1_f1[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]}));
end

   rvdffe #(.WIDTH(2*pt.BTB_BTAG_SIZE+1), .OVERRIDE(1)) rdtagf (.*,
                                                                .en(ifc_fetch_req_f1),
                                                                .din({ifc_select_tid_f1, fetch_rd_tag_f1[pt.BTB_BTAG_SIZE-1:0], fetch_rd_tag_p1_f1[pt.BTB_BTAG_SIZE-1:0]}),
                                                                .dout({ifc_select_tid_f2, fetch_rd_tag_f2[pt.BTB_BTAG_SIZE-1:0], fetch_rd_tag_p1_f2[pt.BTB_BTAG_SIZE-1:0]}));

end
   else begin
   rvdff #(.WIDTH(1)) rdtagf (.*, .clk(active_clk),
                              .din({ifc_select_tid_f1}),
                              .dout({ifc_select_tid_f2}));

   end // else: !if(!pt.BTB_FULLYA)




      assign bht_wr_en0 = bht_wr_en0_thr[0];
      assign bht_wr_data0 = bht_wr_data0_thr[0];
   rvdffe #(29) faddrf2_ff (.*, .en(ifc_fetch_req_f2 & ~ifu_bp_kill_next_f2 & ic_hit_f2 & ~ifc_select_tid_f2), .din(ifc_fetch_addr_f2[31:3]), .dout(ifc_fetch_adder_prior[0][31:3]));



   if(pt.NUM_THREADS > 1) begin

      rvdffe #(29) faddrf2__t1ff (.*, .en(ifc_fetch_req_f2 & ~ifu_bp_kill_next_f2 & ic_hit_f2 & ifc_select_tid_f2), .din(ifc_fetch_addr_f2[31:3]), .dout(ifc_fetch_adder_prior[1][31:3]));
      assign bht_wr_en3 = bht_wr_en0_thr[1];
      assign bht_wr_data3 = bht_wr_data0_thr[1];
      assign bht_wr_addr3[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = mp_hashed[1][pt.BHT_ADDR_HI:pt.BHT_ADDR_LO];


   end // if (pt.NUM_THREADS > 1)
   else begin
      assign bht_wr_en3 = 'b0;
      assign bht_wr_data3 = 'b0;
      assign bht_wr_addr3[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = 'b0;
   end


   assign bht_wr_en1[3:0] = {4{dec_tlu_br1_v_wb}} & decode2_4({dec_tlu_br1_bank_wb, dec_tlu_br1_middle_wb});
   assign bht_wr_en2[3:0] = {4{dec_tlu_br0_v_wb}} & decode2_4({dec_tlu_br0_bank_wb, dec_tlu_br0_middle_wb});

   assign bht_wr_data1[1:0] = dec_tlu_br1_hist_wb[1:0];
   assign bht_wr_data2[1:0] = dec_tlu_br0_hist_wb[1:0]; // highest priority



   eh2_btb_ghr_hash #(.pt(pt)) br0ghrhs  (.hashin(dec_tlu_br0_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]),
                                           .ghr({dec_tlu_br0_fghr_wb[pt.BHT_GHR_SIZE-1:0]}),
                                           .hash(br0_hashed_wb[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO]));
   eh2_btb_ghr_hash #(.pt(pt)) br1ghrhs  (.hashin(dec_tlu_br1_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]),
                                           .ghr({dec_tlu_br1_fghr_wb[pt.BHT_GHR_SIZE-1:0]}),
                                           .hash(br1_hashed_wb[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO]));
   eh2_btb_ghr_hash #(.pt(pt)) fghrhs    (.hashin(btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]),
                                           .ghr({fghr_ns[ifc_select_tid_f1][pt.BHT_GHR_SIZE-1:0]}),
                                           .hash(bht_rd_addr_hashed_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO]));
   eh2_btb_ghr_hash #(.pt(pt)) fghrhs_p1 (.hashin(btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]),
                                           .ghr({fghr_ns[ifc_select_tid_f1][pt.BHT_GHR_SIZE-1:0]}),
                                           .hash(bht_rd_addr_hashed_p1_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO]));

   assign bht_wr_addr0[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = mp_hashed[0][pt.BHT_ADDR_HI:pt.BHT_ADDR_LO];
   assign bht_wr_addr1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = br1_hashed_wb[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO];
   assign bht_wr_addr2[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = br0_hashed_wb[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO];
   assign bht_rd_addr_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = bht_rd_addr_hashed_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO];
   assign bht_rd_addr_p1_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = bht_rd_addr_hashed_p1_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO];


if(!pt.BTB_USE_SRAM) begin

   assign btb_wr_en_error_way0[1:0] = ( ({2{~dec_tlu_way_wb & dec_tlu_error_wb & ~dec_tlu_all_banks_error_wb}} & decode1_2(dec_tlu_error_bank_wb)) |
                                        ({2{~dec_tlu_way_wb & dec_tlu_all_banks_error_wb}}));

   assign btb_wr_en_error_way1[1:0] = ( ({2{dec_tlu_way_wb & dec_tlu_error_wb & ~dec_tlu_all_banks_error_wb}} & decode1_2(dec_tlu_error_bank_wb)) |
                                        ({2{dec_tlu_way_wb & dec_tlu_all_banks_error_wb}}));


   // ----------------------------------------------------------------------
   // Structures. Using FLOPS
   // ----------------------------------------------------------------------
   // BTB
   // Entry -> tag[pt.BTB_BTAG_SIZE-1:0], toffset[pt.BTB_TOFFSET_SIZE-1:0], pc4, boffset, call, ret, valid

    for (j=0 ; j<LRU_SIZE ; j++) begin : BTB_FLOPS

      // Way 0
       assign wr0_b0w0[j] = (btb_wr_addr[0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == j) & (~mp_collision | ~mp_collision_winner_tid) &
                            ((~exu_mp_bank[0] & ~exu_mp_way[0] & exu_mp_valid_write[0]) | btb_wr_en_error_way0[0]);
       assign wr0_b0w1[j] = (btb_wr_addr[0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == j) & (~mp_collision | ~mp_collision_winner_tid) &
                            ((~exu_mp_bank[0] &  exu_mp_way[0] & exu_mp_valid_write[0]) | btb_wr_en_error_way1[0]);
       assign wr0_b1w0[j] = (btb_wr_addr[0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == j) & (~mp_collision | ~mp_collision_winner_tid) &
                            ((exu_mp_bank[0] & ~exu_mp_way[0] & exu_mp_valid_write[0]) | btb_wr_en_error_way0[1]);
       assign wr0_b1w1[j] = (btb_wr_addr[0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == j) & (~mp_collision | ~mp_collision_winner_tid) &
                            ((exu_mp_bank[0] &  exu_mp_way[0] & exu_mp_valid_write[0]) | btb_wr_en_error_way1[1]);

       if (pt.NUM_THREADS > 1) begin
       assign wr1_b0w0[j] = (btb_wr_addr[1][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == j) & (~mp_collision | mp_collision_winner_tid) &
                            ((~exu_mp_bank[1] & ~exu_mp_way[1] & exu_mp_valid_write[1]) | btb_wr_en_error_way0[0]);
       assign wr1_b0w1[j] = (btb_wr_addr[1][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == j) & (~mp_collision | mp_collision_winner_tid) &
                            ((~exu_mp_bank[1] &  exu_mp_way[1] & exu_mp_valid_write[1]) | btb_wr_en_error_way1[0]);
       assign wr1_b1w0[j] = (btb_wr_addr[1][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == j) & (~mp_collision | mp_collision_winner_tid) &
                             ((exu_mp_bank[1] & ~exu_mp_way[1] & exu_mp_valid_write[1]) | btb_wr_en_error_way0[1]);
       assign wr1_b1w1[j] = (btb_wr_addr[1][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == j) & (~mp_collision | mp_collision_winner_tid) &
                             ((exu_mp_bank[1] &  exu_mp_way[1] & exu_mp_valid_write[1]) | btb_wr_en_error_way1[1]);


       end
       else begin
          assign wr1_b0w0[j] = 'b0;
          assign wr1_b0w1[j] = 'b0;
          assign wr1_b1w0[j] = 'b0;
          assign wr1_b1w1[j] = 'b0;
       end

          rvdffe #(BTB_DWIDTH) btb_bank0_way0 (.*,
                    .en   (wr0_b0w0[j] | wr1_b0w0[j]),
                    .din  (wr0_b0w0[j] ? btb_wr_data[0][BTB_DWIDTH-1:0] : btb_wr_data[1][BTB_DWIDTH-1:0]),
                    .dout (btb_bank0_rd_data_way0_out[j]));

          rvdffe #(BTB_DWIDTH) btb_bank1_way0 (.*,
                    .en   (wr0_b1w0[j] | wr1_b1w0[j]),
                    .din  (wr0_b1w0[j] ? btb_wr_data[0][BTB_DWIDTH-1:0] : btb_wr_data[1][BTB_DWIDTH-1:0]),
                    .dout (btb_bank1_rd_data_way0_out[j]));

      // Way 1
          rvdffe #(BTB_DWIDTH) btb_bank0_way1 (.*,
                    .en   (wr0_b0w1[j] | wr1_b0w1[j]),
                    .din  (wr0_b0w1[j] ? btb_wr_data[0][BTB_DWIDTH-1:0] : btb_wr_data[1][BTB_DWIDTH-1:0]),
                    .dout (btb_bank0_rd_data_way1_out[j]));

          rvdffe #(BTB_DWIDTH) btb_bank1_way1 (.*,
                    .en   (wr0_b1w1[j] | wr1_b1w1[j]),
                    .din  (wr0_b1w1[j] ? btb_wr_data[0][BTB_DWIDTH-1:0] : btb_wr_data[1][BTB_DWIDTH-1:0]),
                    .dout (btb_bank1_rd_data_way1_out[j]));

    end

   rvdffe #(BTB_DWIDTH) btb_bank0_way0_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank0_rd_data_way0_f2_in[BTB_DWIDTH-1:0]),
                    .dout       (btb_bank0_rd_data_way0_f2   [BTB_DWIDTH-1:0]));

   rvdffe #(BTB_DWIDTH) btb_bank1_way0_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank1_rd_data_way0_f2_in[BTB_DWIDTH-1:0]),
                    .dout       (btb_bank1_rd_data_way0_f2   [BTB_DWIDTH-1:0]));

   rvdffe #(BTB_DWIDTH) btb_bank0_way1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank0_rd_data_way1_f2_in[BTB_DWIDTH-1:0]),
                    .dout       (btb_bank0_rd_data_way1_f2   [BTB_DWIDTH-1:0]));

   rvdffe #(BTB_DWIDTH) btb_bank1_way1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank1_rd_data_way1_f2_in[BTB_DWIDTH-1:0]),
                    .dout       (btb_bank1_rd_data_way1_f2   [BTB_DWIDTH-1:0]));


   rvdffe #(BTB_DWIDTH) btb_bank0_way0_p1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank0_rd_data_way0_p1_f2_in[BTB_DWIDTH-1:0]),
                    .dout       (btb_bank0_rd_data_way0_p1_f2   [BTB_DWIDTH-1:0]));

   rvdffe #(BTB_DWIDTH) btb_bank1_way0_p1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank1_rd_data_way0_p1_f2_in[BTB_DWIDTH-1:0]),
                    .dout       (btb_bank1_rd_data_way0_p1_f2   [BTB_DWIDTH-1:0]));

   rvdffe #(BTB_DWIDTH) btb_bank0_way1_p1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank0_rd_data_way1_p1_f2_in[BTB_DWIDTH-1:0]),
                    .dout       (btb_bank0_rd_data_way1_p1_f2   [BTB_DWIDTH-1:0]));

   rvdffe #(BTB_DWIDTH) btb_bank1_way1_p1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank1_rd_data_way1_p1_f2_in[BTB_DWIDTH-1:0]),
                    .dout       (btb_bank1_rd_data_way1_p1_f2   [BTB_DWIDTH-1:0]));


    always_comb begin : BTB_rd_mux
        btb_bank0_rd_data_way0_f2_in[BTB_DWIDTH-1:0] = '0 ;
        btb_bank1_rd_data_way0_f2_in[BTB_DWIDTH-1:0] = '0 ;
        btb_bank0_rd_data_way1_f2_in[BTB_DWIDTH-1:0] = '0 ;
        btb_bank1_rd_data_way1_f2_in[BTB_DWIDTH-1:0] = '0 ;
        btb_bank0_rd_data_way0_p1_f2_in[BTB_DWIDTH-1:0] = '0 ;
        btb_bank1_rd_data_way0_p1_f2_in[BTB_DWIDTH-1:0] = '0 ;
        btb_bank0_rd_data_way1_p1_f2_in[BTB_DWIDTH-1:0] = '0 ;
        btb_bank1_rd_data_way1_p1_f2_in[BTB_DWIDTH-1:0] = '0 ;

        for (int j=0; j< LRU_SIZE; j++) begin
          if (btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == (pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1)'(j)) begin

           btb_bank0_rd_data_way0_f2_in[BTB_DWIDTH-1:0] =  btb_bank0_rd_data_way0_out[j];
           btb_bank1_rd_data_way0_f2_in[BTB_DWIDTH-1:0] =  btb_bank1_rd_data_way0_out[j];

           btb_bank0_rd_data_way1_f2_in[BTB_DWIDTH-1:0] =  btb_bank0_rd_data_way1_out[j];
           btb_bank1_rd_data_way1_f2_in[BTB_DWIDTH-1:0] =  btb_bank1_rd_data_way1_out[j];

          end
        end
        for (int j=0; j< LRU_SIZE; j++) begin
          if (btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == (pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1)'(j)) begin

           btb_bank0_rd_data_way0_p1_f2_in[BTB_DWIDTH-1:0] =  btb_bank0_rd_data_way0_out[j];
           btb_bank1_rd_data_way0_p1_f2_in[BTB_DWIDTH-1:0] =  btb_bank1_rd_data_way0_out[j];

           btb_bank0_rd_data_way1_p1_f2_in[BTB_DWIDTH-1:0] =  btb_bank0_rd_data_way1_out[j];
           btb_bank1_rd_data_way1_p1_f2_in[BTB_DWIDTH-1:0] =  btb_bank1_rd_data_way1_out[j];

          end
        end
    end
end // if (!pt.BTB_USE_SRAM)

   // FULLYA - relocate after all the merging
      if(pt.BTB_FULLYA) begin : fa

         logic found1, found2, hit0, hit1, hit2, hit3, hit0_other, hit1_other, hit2_other, hit3_other, multihit;
         logic btb_used_reset, write_used;
         logic [$clog2(pt.BTB_SIZE)-1:0] btb_fa_wr_addr0, btb_fa_wr_addr1, hit0_index, hit1_index, hit2_index, hit3_index;
         logic [$clog2(pt.BTB_SIZE)-1:0] hit0_other_index, hit1_other_index, hit2_other_index, hit3_other_index;

         logic [pt.BTB_SIZE-1:0]         btb_tag_hit, btb_offset_0, btb_offset_1, btb_offset_2, btb_offset_3, btb_used_ns, btb_used,
                                         btb_offset_0_f2, btb_offset_1_f2, btb_offset_2_f2, btb_offset_3_f2, wr0_en, wr1_en, wr_cleanup,
                                         fa_fetch_mp_collision_f1, btb_upper_hit, btb_used_clr;
         logic [pt.BTB_SIZE-1:0][BTB_DWIDTH-1:0] btbdata;

         // Fully Associative tag hash uses bits 31:3. Bits 2:1 are the offset bits used for the 4 tag comp banks
         // Full tag used to speed up lookup. There is one 31:3 cmp per entry, and 4 2:1 cmps per entry.

         logic [FA_CMP_LOWER-1:1]  ifc_fetch_addr_p1_f1, ifc_fetch_addr_p2_f1, ifc_fetch_addr_p3_f1;


         assign ifc_fetch_addr_p1_f1[FA_CMP_LOWER-1:1] = ifc_fetch_addr_f1[FA_CMP_LOWER-1:1] + 1'b1;
         assign ifc_fetch_addr_p2_f1[FA_CMP_LOWER-1:1] = ifc_fetch_addr_f1[FA_CMP_LOWER-1:1] + 2'b10;
         assign ifc_fetch_addr_p3_f1[FA_CMP_LOWER-1:1] = ifc_fetch_addr_f1[FA_CMP_LOWER-1:1] + 2'b11;

      rvdffe #(4*pt.BTB_SIZE) btb_hitsf2 (.*, .clk(clk),
                                          .en  (ifc_fetch_req_f1),
                                          .din ({btb_offset_0, btb_offset_1, btb_offset_2, btb_offset_3}),
                                          .dout({btb_offset_0_f2, btb_offset_1_f2, btb_offset_2_f2, btb_offset_3_f2}));

      always_comb begin
         btb_vbank0_rd_data_f2 = '0;
         btb_vbank1_rd_data_f2 = '0;
         btb_vbank2_rd_data_f2 = '0;
         btb_vbank3_rd_data_f2 = '0;
         btb_tag_hit = '0;
         btb_upper_hit = '0;
         btb_offset_0 = '0;
         btb_offset_1 = '0;
         btb_offset_2 = '0;
         btb_offset_3 = '0;

         found1 = 1'b0;
         found2 = 1'b0;
         hit0 = 1'b0;
         hit1 = 1'b0;
         hit2 = 1'b0;
         hit3 = 1'b0;
         hit0_index = '0;
         hit1_index = '0;
         hit2_index = '0;
         hit3_index = '0;
         hit0_other = 1'b0;
         hit1_other = 1'b0;
         hit2_other = 1'b0;
         hit3_other = 1'b0;
         hit0_other_index = '0;
         hit1_other_index = '0;
         hit2_other_index = '0;
         hit3_other_index = '0;
         btb_fa_wr_addr0 = '0;
         btb_fa_wr_addr1 = '0;
         fa_fetch_mp_collision_f1 = '0;

         for(int i=0; i<pt.BTB_SIZE; i++) begin
            // Break the cmp into chunks for lower area.
            // Chunk1: FA 31:6 or 31:5 depending on icache line size
            // Chunk2: FA 5:1 or 4:1 depending on icache line size
            btb_upper_hit[i] = (btbdata[i][BTB_DWIDTH_TOP:FA_TAG_END_UPPER] == ifc_fetch_addr_f1[31:FA_CMP_LOWER]) & btbdata[i][0] & ~wr0_en[i] & ~wr1_en[i];
            btb_offset_0[i] = (btbdata[i][FA_TAG_START_LOWER:FA_TAG_END_LOWER] == ifc_fetch_addr_f1[FA_CMP_LOWER-1:1]) & btb_upper_hit[i];
            btb_offset_1[i] = (btbdata[i][FA_TAG_START_LOWER:FA_TAG_END_LOWER] == ifc_fetch_addr_p1_f1[FA_CMP_LOWER-1:1]) & btb_upper_hit[i];
            btb_offset_2[i] = (btbdata[i][FA_TAG_START_LOWER:FA_TAG_END_LOWER] == ifc_fetch_addr_p2_f1[FA_CMP_LOWER-1:1]) & btb_upper_hit[i];
            btb_offset_3[i] = (btbdata[i][FA_TAG_START_LOWER:FA_TAG_END_LOWER] == ifc_fetch_addr_p3_f1[FA_CMP_LOWER-1:1]) & btb_upper_hit[i];

            if(~hit0) begin
               if(btb_offset_0_f2[i]) begin
                  hit0_index[BTB_FA_INDEX:0] = (BTB_FA_INDEX+1)'(i);
                  // hit unless we are also writing this entry at the same time
                  hit0 = 1'b1;
               end
            end
            else if(~hit0_other) begin // find multihit
               if(btb_offset_0_f2[i]) begin
                  hit0_other_index[BTB_FA_INDEX:0] = (BTB_FA_INDEX+1)'(i);
                  hit0_other = 1'b1;
               end
            end

            if(~hit1) begin
               if(btb_offset_1_f2[i]) begin
                  hit1_index[BTB_FA_INDEX:0] = (BTB_FA_INDEX+1)'(i);
                  hit1 = 1'b1;
               end
            end
            else if(~hit1_other) begin // multihit
               if(btb_offset_1_f2[i]) begin
                  hit1_other_index[BTB_FA_INDEX:0] = (BTB_FA_INDEX+1)'(i);
                  hit1_other = 1'b1;
               end
            end

            if(~hit2) begin
               if(btb_offset_2_f2[i]) begin
                  hit2_index[BTB_FA_INDEX:0] = (BTB_FA_INDEX+1)'(i);
                  hit2 = 1'b1;
               end
            end
            else if(~hit2_other) begin // multihit
               if(btb_offset_2_f2[i]) begin
                  hit2_other_index[BTB_FA_INDEX:0] = (BTB_FA_INDEX+1)'(i);
                  hit2_other = 1'b1;
               end
            end

            if(~hit3) begin
               if(btb_offset_3_f2[i]) begin
                  hit3_index[BTB_FA_INDEX:0] = (BTB_FA_INDEX+1)'(i);
                  hit3 = 1'b1;
               end
            end
            else if(~hit3_other) begin // multihit
               if(btb_offset_3_f2[i]) begin
                  hit3_other_index[BTB_FA_INDEX:0] = (BTB_FA_INDEX+1)'(i);
                  hit3_other = 1'b1;
               end
            end

            // Mux out the 4 potential branches
            if(btb_offset_0_f2[i] == 1'b1)
              btb_vbank0_rd_data_f2[BTB_DWIDTH-1:0] = btbdata[i];
            if(btb_offset_1_f2[i] == 1'b1)
              btb_vbank1_rd_data_f2[BTB_DWIDTH-1:0] = btbdata[i];
            if(btb_offset_2_f2[i] == 1'b1)
              btb_vbank2_rd_data_f2[BTB_DWIDTH-1:0] = btbdata[i];
            if(btb_offset_3_f2[i] == 1'b1)
              btb_vbank3_rd_data_f2[BTB_DWIDTH-1:0] = btbdata[i];

            // find the first zero from bit zero in the used vector, this is the write address
            if(~found1) begin
               if(~btb_used[i]) begin
                  btb_fa_wr_addr0[BTB_FA_INDEX:0] = i[BTB_FA_INDEX:0];
                  found1 = 1'b1;
               end
            end
            // find the second zero from bit zero in the used vector, this is the write address for the
            // case where both threads MP at the same time.
            else if(~found2) begin
               if(~btb_used[i]) begin
                  btb_fa_wr_addr1[BTB_FA_INDEX:0] = i[BTB_FA_INDEX:0];
                  found2 = 1'b1;
               end
            end

         end
      end // always_comb begin


   assign vwayhit_f2[3:0] = {hit3, hit2, hit1, hit0} & {eoc_mask[3:1], 1'b1};

   // a valid taken target needs to kill the next fetch as we compute the target address
   assign ifu_bp_kill_next_f2 = |(vwayhit_f2[3:0] & hist1_raw[3:0]) & ifc_fetch_req_f2 & ~leak_one_f2[ifc_select_tid_f2] & ~dec_tlu_bpred_disable;

   // way bit is reused as the predicted bit
   assign way_raw[3:0] =  vwayhit_f2[3:0];

   for (j=0 ; j<pt.BTB_SIZE ; j++) begin : BTB_FAFLOPS

      assign wr0_en[j] = ((btb_fa_wr_addr0[BTB_FA_INDEX:0] == j) & (exu_mp_valid_write[0] & ~exu_mp_pkt[0].way)) |
                         ((dec_fa_error_index == j) & dec_tlu_error_wb);
      if(pt.NUM_THREADS>1)
        assign wr1_en[j] = ((btb_fa_wr_addr1[BTB_FA_INDEX:0] == j) & (exu_mp_valid_write[1] & ~exu_mp_pkt[1].way)) |
                           ((dec_fa_error_index == j) & dec_tlu_error_wb);
      else
        assign wr1_en[j] = '0;

      assign wr_cleanup[j] = (hit0_other & (hit0_other_index[BTB_FA_INDEX:0] == (BTB_FA_INDEX+1)'(j))) |
                             (hit1_other & (hit1_other_index[BTB_FA_INDEX:0] == (BTB_FA_INDEX+1)'(j))) |
                             (hit2_other & (hit2_other_index[BTB_FA_INDEX:0] == (BTB_FA_INDEX+1)'(j))) |
                             (hit3_other & (hit3_other_index[BTB_FA_INDEX:0] == (BTB_FA_INDEX+1)'(j))) ;

      rvdffe #(BTB_DWIDTH) btb_fa (.*, .clk(clk),
                                            .en  (wr0_en[j] | wr1_en[j] | wr_cleanup[j]),
                                            .din (wr_cleanup[j] ? '0 : wr0_en[j] ? btb_wr_data[0][BTB_DWIDTH-1:0] : btb_wr_data[1][BTB_DWIDTH-1:0]),
                                            .dout(btbdata[j]));
   end

   assign ifu_bp_fa_index_f2[3] = hit3 ? hit3_index : '0;
   assign ifu_bp_fa_index_f2[2] = hit2 ? hit2_index : '0;
   assign ifu_bp_fa_index_f2[1] = hit1 ? hit1_index : '0;
   assign ifu_bp_fa_index_f2[0] = hit0 ? hit0_index : '0;

   assign multihit = hit0_other | hit1_other | hit2_other | hit3_other;
   if(pt.NUM_THREADS>1) begin
     // need room for 2 for worst case MP on both threads at limit
      assign btb_used_reset = &btb_used[pt.BTB_SIZE-2:0];
      assign btb_used_clr[pt.BTB_SIZE-1:0] = ~(({pt.BTB_SIZE{hit0_other}} & (32'b1 << hit0_other_index[BTB_FA_INDEX:0])) |
                                               ({pt.BTB_SIZE{hit1_other}} & (32'b1 << hit1_other_index[BTB_FA_INDEX:0])) |
                                               ({pt.BTB_SIZE{hit2_other}} & (32'b1 << hit2_other_index[BTB_FA_INDEX:0])) |
                                               ({pt.BTB_SIZE{hit3_other}} & (32'b1 << hit3_other_index[BTB_FA_INDEX:0])) ) & btb_used[pt.BTB_SIZE-1:0];

      assign btb_used_ns[pt.BTB_SIZE-1:0] = ({pt.BTB_SIZE{vwayhit_f2[3]}} & (32'b1 << hit3_index[BTB_FA_INDEX:0])) |
                                            ({pt.BTB_SIZE{vwayhit_f2[2]}} & (32'b1 << hit2_index[BTB_FA_INDEX:0])) |
                                            ({pt.BTB_SIZE{vwayhit_f2[1]}} & (32'b1 << hit1_index[BTB_FA_INDEX:0])) |
                                            ({pt.BTB_SIZE{vwayhit_f2[0]}} & (32'b1 << hit0_index[BTB_FA_INDEX:0])) |
                                            ({pt.BTB_SIZE{exu_mp_valid_write[0] & ~exu_mp_pkt[0].way & ~dec_tlu_error_wb}} & (32'b1 << btb_fa_wr_addr0[BTB_FA_INDEX:0])) |
                                            ({pt.BTB_SIZE{exu_mp_valid_write[1] & ~exu_mp_pkt[1].way & ~dec_tlu_error_wb}} & (32'b1 << btb_fa_wr_addr1[BTB_FA_INDEX:0])) |
                                            ({pt.BTB_SIZE{btb_used_reset}} & {pt.BTB_SIZE{1'b0}}) |
                                            ({pt.BTB_SIZE{~btb_used_reset & dec_tlu_error_wb}} & (btb_used_clr[pt.BTB_SIZE-1:0] & ~(32'b1 << dec_fa_error_index[BTB_FA_INDEX:0]))) |
                                            (~{pt.BTB_SIZE{btb_used_reset | dec_tlu_error_wb}} & btb_used_clr[pt.BTB_SIZE-1:0]);

      assign write_used = btb_used_reset | ifu_bp_kill_next_f2 | exu_mp_valid_write[0] | exu_mp_valid_write[1] | dec_tlu_error_wb | multihit;
   end
   else begin
      assign btb_used_reset = &btb_used[pt.BTB_SIZE-1:0];
      assign btb_used_clr[pt.BTB_SIZE-1:0] = ~(({pt.BTB_SIZE{hit0_other}} & (32'b1 << hit0_other_index[BTB_FA_INDEX:0])) |
                                               ({pt.BTB_SIZE{hit1_other}} & (32'b1 << hit1_other_index[BTB_FA_INDEX:0])) |
                                               ({pt.BTB_SIZE{hit2_other}} & (32'b1 << hit2_other_index[BTB_FA_INDEX:0])) |
                                               ({pt.BTB_SIZE{hit3_other}} & (32'b1 << hit3_other_index[BTB_FA_INDEX:0])) ) & btb_used[pt.BTB_SIZE-1:0];
      assign btb_used_ns[pt.BTB_SIZE-1:0] = ({pt.BTB_SIZE{vwayhit_f2[3]}} & (32'b1 << hit3_index[BTB_FA_INDEX:0])) |
                                            ({pt.BTB_SIZE{vwayhit_f2[2]}} & (32'b1 << hit2_index[BTB_FA_INDEX:0])) |
                                            ({pt.BTB_SIZE{vwayhit_f2[1]}} & (32'b1 << hit1_index[BTB_FA_INDEX:0])) |
                                            ({pt.BTB_SIZE{vwayhit_f2[0]}} & (32'b1 << hit0_index[BTB_FA_INDEX:0])) |
                                            ({pt.BTB_SIZE{exu_mp_valid_write[0] & ~exu_mp_pkt[0].way & ~dec_tlu_error_wb}} & (32'b1 << btb_fa_wr_addr0[BTB_FA_INDEX:0])) |
                                            ({pt.BTB_SIZE{btb_used_reset}} & {pt.BTB_SIZE{1'b0}}) |
                                            ({pt.BTB_SIZE{~btb_used_reset & dec_tlu_error_wb}} & (btb_used_clr[pt.BTB_SIZE-1:0] & ~(32'b1 << dec_fa_error_index[BTB_FA_INDEX:0]))) |
                                            (~{pt.BTB_SIZE{btb_used_reset | dec_tlu_error_wb}} & btb_used_clr[pt.BTB_SIZE-1:0]);

      assign write_used = btb_used_reset | ifu_bp_kill_next_f2 | exu_mp_valid_write[0] | dec_tlu_error_wb | multihit;
   end

   rvdffe #(pt.BTB_SIZE) btb_usedf (.*, .clk(clk),
                    .en  (write_used),
                    .din (btb_used_ns[pt.BTB_SIZE-1:0]),
                    .dout(btb_used[pt.BTB_SIZE-1:0]));


end // block: fa




   //-----------------------------------------------------------------------------
   // BHT
   // 2 bit Entry -> direction, strength
   //
   //-----------------------------------------------------------------------------

   for ( i=0; i<4; i++) begin : BANKS
     for (genvar k=0 ; k < (pt.BHT_ARRAY_DEPTH)/NUM_BHT_LOOP ; k++) begin : BHT_CLK_GROUP
     assign bht_bank_clken[i][k]  = (bht_wr_en0[i] & ((bht_wr_addr0[pt.BHT_ADDR_HI: NUM_BHT_LOOP_OUTER_LO]==k) |  BHT_NO_ADDR_MATCH)) |
                                    (bht_wr_en1[i] & ((bht_wr_addr1[pt.BHT_ADDR_HI: NUM_BHT_LOOP_OUTER_LO]==k) |  BHT_NO_ADDR_MATCH)) |
                                    (bht_wr_en2[i] & ((bht_wr_addr2[pt.BHT_ADDR_HI: NUM_BHT_LOOP_OUTER_LO]==k) |  BHT_NO_ADDR_MATCH)) |
                                    (bht_wr_en3[i] & ((bht_wr_addr3[pt.BHT_ADDR_HI: NUM_BHT_LOOP_OUTER_LO]==k) |  BHT_NO_ADDR_MATCH));

 `ifndef RV_FPGA_OPTIMIZE
    rvclkhdr bht_bank_grp_cgc ( .en(bht_bank_clken[i][k]), .l1clk(bht_bank_clk[i][k]), .* );
`endif
     for (j=0 ; j<NUM_BHT_LOOP ; j++) begin : BHT_FLOPS

        wire[3:0] wr_sel = {
                             bht_wr_en3[i] && bht_wr_addr3 == (j+16*k),
                             bht_wr_en2[i] && bht_wr_addr2 == (j+16*k),
                             bht_wr_en1[i] && bht_wr_addr1 == (j+16*k),
                             bht_wr_en0[i] && bht_wr_addr0 == (j+16*k)
                            };

        wire[1:0] wdata  = wr_sel[2] ? bht_wr_data2[1:0] :
                           wr_sel[1] ? bht_wr_data1[1:0] :
                            // wr_en0 is thread 0, wr_en3 is thread 1. Consider collisions and kill t0 if t1 wins arb
                           (wr_sel[0] & (~mp_bht_collision | ~mp_bht_collision_winner_tid))
                                     ? bht_wr_data0[1:0] : bht_wr_data3[1:0];

          rvdffs_fpga #(2) bht_bank (.*,
                                     .clk        (bht_bank_clk[i][k]),
                                     .rawclk     (clk),
                                     .en         (|wr_sel),
                                     .clken      (bht_bank_clken[i][k]),
                                     .din        (wdata),
                                     .dout       (bht_bank_rd_data_out[i][(16*k)+j]));



     end // block: BHT_FLOPS
     end // block: BHT_CLK_GROUP
   end // block: BANKS

    always_comb begin : BHT_rd_mux
     bht_bank0_rd_data_f2_in[1:0] = '0 ;
     bht_bank1_rd_data_f2_in[1:0] = '0 ;
     bht_bank2_rd_data_f2_in[1:0] = '0 ;
     bht_bank3_rd_data_f2_in[1:0] = '0 ;
     bht_bank0_rd_data_p1_f2_in[1:0] = '0 ;
     bht_bank1_rd_data_p1_f2_in[1:0] = '0 ;
     bht_bank2_rd_data_p1_f2_in[1:0] = '0 ;
     for (int j=0; j< pt.BHT_ARRAY_DEPTH; j++) begin
       if (bht_rd_addr_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] == (pt.BHT_ADDR_HI-pt.BHT_ADDR_LO+1)'(j)) begin
         bht_bank0_rd_data_f2_in[1:0] = bht_bank_rd_data_out[0][j];
         bht_bank1_rd_data_f2_in[1:0] = bht_bank_rd_data_out[1][j];
         bht_bank2_rd_data_f2_in[1:0] = bht_bank_rd_data_out[2][j];
         bht_bank3_rd_data_f2_in[1:0] = bht_bank_rd_data_out[3][j];
       end
       if (bht_rd_addr_p1_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] == (pt.BHT_ADDR_HI-pt.BHT_ADDR_LO+1)'(j)) begin
         bht_bank0_rd_data_p1_f2_in[1:0] = bht_bank_rd_data_out[0][j];
         bht_bank1_rd_data_p1_f2_in[1:0] = bht_bank_rd_data_out[1][j];
         bht_bank2_rd_data_p1_f2_in[1:0] = bht_bank_rd_data_out[2][j];
       end
      end
    end // block: BHT_rd_mux



   rvdffe #(14) bht_dataoutf (.*, .en         (ifc_fetch_req_f1),
                                 .din        ({bht_bank0_rd_data_f2_in[1:0],
                                               bht_bank1_rd_data_f2_in[1:0],
                                               bht_bank2_rd_data_f2_in[1:0],
                                               bht_bank3_rd_data_f2_in[1:0],
                                               bht_bank0_rd_data_p1_f2_in[1:0],
                                               bht_bank1_rd_data_p1_f2_in[1:0],
                                               bht_bank2_rd_data_p1_f2_in[1:0]
                                               }),
                                 .dout       ({bht_bank0_rd_data_f2   [1:0],
                                               bht_bank1_rd_data_f2   [1:0],
                                               bht_bank2_rd_data_f2   [1:0],
                                               bht_bank3_rd_data_f2   [1:0],
                                               bht_bank0_rd_data_p1_f2   [1:0],
                                               bht_bank1_rd_data_p1_f2   [1:0],
                                               bht_bank2_rd_data_p1_f2   [1:0]
                                               }));




     function [2:0] encode8_3;
      input [7:0] in;

      encode8_3[2] = |in[7:4];
      encode8_3[1] = in[7] | in[6] | in[3] | in[2];
      encode8_3[0] = in[7] | in[5] | in[3] | in[1];

   endfunction
     function [1:0] encode4_2;
      input [3:0] in;

      encode4_2[1] = in[3] | in[2];
      encode4_2[0] = in[3] | in[1];

   endfunction
   function [7:0] decode3_8;
      input [2:0] in;

      decode3_8[7] =  in[2] &  in[1] &  in[0];
      decode3_8[6] =  in[2] &  in[1] & ~in[0];
      decode3_8[5] =  in[2] & ~in[1] &  in[0];
      decode3_8[4] =  in[2] & ~in[1] & ~in[0];
      decode3_8[3] = ~in[2] &  in[1] &  in[0];
      decode3_8[2] = ~in[2] &  in[1] & ~in[0];
      decode3_8[1] = ~in[2] & ~in[1] &  in[0];
      decode3_8[0] = ~in[2] & ~in[1] & ~in[0];

   endfunction
   function [3:0] decode2_4;
      input [1:0] in;

      decode2_4[3] =  in[1] &  in[0];
      decode2_4[2] =  in[1] & ~in[0];
      decode2_4[1] = ~in[1] &  in[0];
      decode2_4[0] = ~in[1] & ~in[0];

   endfunction
   function [1:0] decode1_2;
      input  in;

      decode1_2[1] = in;
      decode1_2[0] = ~in;

   endfunction

   function [2:0] countones;
      input [3:0] valid;

      begin

countones[2:0] = {2'b0, valid[3]} +
                 {2'b0, valid[2]} +
                 {2'b0, valid[1]} +
                 {2'b0, valid[0]};
      end
   endfunction
   function [2:0] newlru; // updated lru
      input [2:0] lru;// current lru
      input [1:0] used;// hit way
      begin
newlru[2] = (lru[2] & ~used[0]) | (~used[1] & ~used[0]);
newlru[1] = (~used[1] & ~used[0]) | (used[0]);
newlru[0] = (~lru[2] & lru[1] & ~used[1] & ~used[0]) | (~lru[1] & ~lru[0] & used[0]) | (
    ~lru[2] & lru[0] & used[0]) | (lru[0] & ~used[1] & ~used[0]);
      end
   endfunction //

   function [1:0] lru2way; // new repl way taking invalid ways into account
      input [2:0] lru; // current lru
      input [2:0] v; // current way valids
      begin
         lru2way[1] = (~lru[2] & lru[1] & ~lru[0] & v[1] & v[0]) | (lru[2] & lru[0] & v[1] & v[0]) | (~v[2] & v[1] & v[0]);
         lru2way[0] = (lru[2] & ~lru[0] & v[2] & v[0]) | (~v[1] & v[0]);
      end
   endfunction
`undef TAG
endmodule // eh2_ifu_bp_ctl

