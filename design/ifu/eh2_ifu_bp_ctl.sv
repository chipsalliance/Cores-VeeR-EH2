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
// Function: Branch predictor
// Comments:
//
//
//  Bank1 : Bank0
//  FA  4       0
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

   input logic [31:1] ifc_fetch_addr_f1, // look up btb address
   input logic ifc_fetch_req_f1,  // F1 valid
   input logic ifc_fetch_req_f2,  // F2 valid

   input eh2_br_tlu_pkt_t dec_tlu_br0_wb_pkt, // BP commit update packet, includes errors
   input eh2_br_tlu_pkt_t dec_tlu_br1_wb_pkt, // BP commit update packet, includes errors
   input logic [pt.BHT_GHR_SIZE-1:0] dec_tlu_br0_fghr_wb, // fghr to bp
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br0_index_wb, // bp index
   input logic [pt.BHT_GHR_SIZE-1:0] dec_tlu_br1_fghr_wb, // fghr to bp
   input logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] dec_tlu_br1_index_wb, // bp index

   input logic [pt.NUM_THREADS-1:0] dec_tlu_flush_lower_wb, // used to move EX4 RS to EX1 and F
   input logic [pt.NUM_THREADS-1:0] dec_tlu_flush_leak_one_wb, // don't hit for leak one fetches

   input logic dec_tlu_bpred_disable, // disable all branch prediction

   input logic        exu_i0_br_ret_e4, // EX4 ret stack update
   input logic        exu_i1_br_ret_e4, // EX4 ret stack update
   input logic        exu_i0_br_call_e4, // EX4 ret stack update
   input logic        exu_i1_br_call_e4, // EX4 ret stack update
   input logic dec_i0_tid_e4, // needed to maintain RS in BP
   input logic dec_i1_tid_e4,

   input eh2_predict_pkt_t [pt.NUM_THREADS-1:0] exu_mp_pkt, // mispredict packet(s)

   input logic [pt.NUM_THREADS-1:0][pt.BHT_GHR_SIZE-1:0] exu_mp_eghr, // execute ghr (for patching fghr)
   input logic [pt.NUM_THREADS-1:0][pt.BHT_GHR_SIZE-1:0] exu_mp_fghr,                    // Mispredict fghr
   input logic [pt.NUM_THREADS-1:0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] exu_mp_index,         // Mispredict index
   input logic [pt.NUM_THREADS-1:0][pt.BTB_BTAG_SIZE-1:0] exu_mp_btag,                   // Mispredict btag

   input logic [pt.NUM_THREADS-1:0] exu_flush_final, // all flushes
   output logic ifu_bp_kill_next_f2, // kill next fetch, taken target found
   output logic [31:1] ifu_bp_btb_target_f2, //  predicted target PC
   output logic [3:1] ifu_bp_inst_mask_f2, // tell ic which valids to kill because of a taken branch, right justified

   output logic [pt.BHT_GHR_SIZE-1:0] ifu_bp_fghr_f2, // fetch ghr

   output logic [3:0] ifu_bp_way_f2, // way
   output logic [3:0] ifu_bp_ret_f2, // predicted ret
   output logic [3:0] ifu_bp_hist1_f2, // history counters for all 4 potential branches, bit 1, right justified
   output logic [3:0] ifu_bp_hist0_f2, // history counters for all 4 potential branches, bit 0, right justified
   output logic [11:0] ifu_bp_poffset_f2, // predicted target
   output logic [3:0] ifu_bp_pc4_f2, // pc4 indication, right justified
   output logic [3:0] ifu_bp_valid_f2, // branch valid, right justified
   output logic [31:1] ifc_fetch_addr_f2, // to tgt calc

   input  logic       scan_mode
   );

`define TAG 16+pt.BTB_BTAG_SIZE:17

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
   logic [pt.NUM_THREADS-1:0] [11:0] exu_mp_tgt; // target offset
   logic [pt.NUM_THREADS-1:0] [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] exu_mp_addr; // BTB/BHT address

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
   logic [11:0]       btb_rd_tgt_f2;
   logic              btb_rd_pc4_f2,  btb_rd_call_f2, btb_rd_ret_f2;
   logic [2:1]        bp_total_branch_offset_f2;

   logic [31:1]       bp_btb_target_adder_f2;
   logic [31:1]       bp_rs_call_target_f2;
   logic [pt.NUM_THREADS-1:0]         rs_push, rs_pop, rs_hold, fetch_mp_collision_f1, fetch_mp_collision_f2,fetch_mp_collision_p1_f1, fetch_mp_collision_p1_f2;
   logic [pt.NUM_THREADS-1:0][pt.BTB_BTAG_SIZE-1:0] btb_wr_tag;
   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] btb_rd_addr_f1, btb_rd_addr_p1_f1, btb_rd_addr_f2, btb_rd_addr_p1_f2;
   logic [pt.BTB_BTAG_SIZE-1:0] fetch_rd_tag_f1, fetch_rd_tag_p1_f1, fetch_rd_tag_f2, fetch_rd_tag_p1_f2;
   logic [1:0]         btb_wr_en_error_way0, btb_wr_en_error_way1;

   logic [pt.NUM_THREADS-1:0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] btb_wr_addr;
   logic [1:0][16+pt.BTB_BTAG_SIZE:0]        btb_wr_data;

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

   logic [LRU_SIZE-1:0][16+pt.BTB_BTAG_SIZE:0]  btb_bank0_rd_data_way0_out ;
   logic [LRU_SIZE-1:0][16+pt.BTB_BTAG_SIZE:0]  btb_bank1_rd_data_way0_out ;

   logic [LRU_SIZE-1:0][16+pt.BTB_BTAG_SIZE:0]  btb_bank0_rd_data_way1_out ;
   logic [LRU_SIZE-1:0][16+pt.BTB_BTAG_SIZE:0]  btb_bank1_rd_data_way1_out ;

   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank0_rd_data_way0_f2_in ;
   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank1_rd_data_way0_f2_in ;
   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank0_rd_data_way1_f2_in ;
   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank1_rd_data_way1_f2_in ;

   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank0_rd_data_way0_p1_f2_in ;
   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank1_rd_data_way0_p1_f2_in ;
   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank0_rd_data_way1_p1_f2_in ;
   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank1_rd_data_way1_p1_f2_in ;


   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank0_rd_data_way0_f2, btb_bank0_rd_data_way0_p1_f2;
   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank1_rd_data_way0_f2, btb_bank1_rd_data_way0_p1_f2;

   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank0_rd_data_way1_f2, btb_bank0_rd_data_way1_p1_f2;
   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank1_rd_data_way1_f2, btb_bank1_rd_data_way1_p1_f2;
   logic                [16+pt.BTB_BTAG_SIZE:0] btb_vbank0_rd_data_f2, btb_vbank1_rd_data_f2, btb_vbank2_rd_data_f2, btb_vbank3_rd_data_f2;

   logic                                         final_h;
   logic                                         btb_fg_crossing_f2;


   logic [1:0]                                   bht_vbank0_rd_data_f2, bht_vbank1_rd_data_f2, bht_vbank2_rd_data_f2, bht_vbank3_rd_data_f2,
                                                 branch_error_bank_conflict_p1_f1, branch_error_bank_conflict_p1_f2, tag_match_way0_p1_f2, tag_match_way1_p1_f2;

   logic [3:0]                                   btb_vlru_rd_f2, fetch_start_f2, tag_match_vway1_expanded_f2, tag_match_way0_expanded_p1_f2, tag_match_way1_expanded_p1_f2;
   logic [31:3] fetch_addr_p1_f1, fetch_addr_p1_f2;

   logic dec_tlu_br0_way_wb, dec_tlu_br1_way_wb, dec_tlu_way_wb, dec_tlu_way_wb_f;

   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank0e_rd_data_f2, btb_bank0e_rd_data_p1_f2;
   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank1e_rd_data_f2, btb_bank1e_rd_data_p1_f2;

   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank0o_rd_data_f2, btb_bank0o_rd_data_p1_f2;
   logic                [16+pt.BTB_BTAG_SIZE:0] btb_bank1o_rd_data_f2;

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
   logic use_fa_plus;
   logic [3:0] hist0_raw, hist1_raw, pc4_raw, pret_raw;
   logic [16:1] btb_sel_data_f2;
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

   // ----------------------------------------------------------------------
   // READ
   // ----------------------------------------------------------------------

   // hash the incoming fetch PC, first guess at hashing algorithm
   eh2_btb_addr_hash #(.pt(pt)) f1hash(.pc(ifc_fetch_addr_f1[pt.BTB_INDEX3_HI:pt.BTB_INDEX1_LO]), .hash(btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]));

   assign fetch_addr_p1_f1[31:3] = ifc_fetch_addr_f1[31:3] + 29'b1;
   eh2_btb_addr_hash #(.pt(pt)) f1hash_p1(.pc(fetch_addr_p1_f1[pt.BTB_INDEX3_HI:pt.BTB_INDEX1_LO]), .hash(btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]));

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

   // Errors colliding with fetches must kill the btb/bht hit.

   assign branch_error_collision_f1 = dec_tlu_error_wb & (btb_error_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]);
   assign branch_error_collision_p1_f1 = dec_tlu_error_wb & (btb_error_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]);

   assign branch_error_bank_conflict_f1[1:0] = {2{branch_error_collision_f1}} & (decode1_2(dec_tlu_error_bank_wb) | {2{dec_tlu_all_banks_error_wb}});
   assign branch_error_bank_conflict_p1_f1[1:0] = {2{branch_error_collision_p1_f1}} & (decode1_2(dec_tlu_error_bank_wb) | {2{dec_tlu_all_banks_error_wb}});

   rvdff #(6) coll_ff (.*, .clk(active_clk),
                         .din({branch_error_bank_conflict_f1[1:0], branch_error_bank_conflict_p1_f1[1:0], dec_tlu_way_wb, ifc_fetch_req_f1}),
                        .dout({branch_error_bank_conflict_f2[1:0], branch_error_bank_conflict_p1_f2[1:0], dec_tlu_way_wb_f, ifc_fetch_req_f2_raw}));

   // 2 -way SA, figure out the way hit and mux accordingly
   assign tag_match_way0_f2[1:0] = {btb_bank1_rd_data_way0_f2[BV] & (btb_bank1_rd_data_way0_f2[`TAG] == fetch_rd_tag_f2[pt.BTB_BTAG_SIZE-1:0]),
                                    btb_bank0_rd_data_way0_f2[BV] & (btb_bank0_rd_data_way0_f2[`TAG] == fetch_rd_tag_f2[pt.BTB_BTAG_SIZE-1:0])} &
                                   ~({2{~dec_tlu_way_wb_f}} & branch_error_bank_conflict_f2[1:0]) & {2{ifc_fetch_req_f2_raw & ~leak_one_f2[ifc_select_tid_f2]}};

   assign tag_match_way1_f2[1:0] = {btb_bank1_rd_data_way1_f2[BV] & (btb_bank1_rd_data_way1_f2[`TAG] == fetch_rd_tag_f2[pt.BTB_BTAG_SIZE-1:0]),
                                    btb_bank0_rd_data_way1_f2[BV] & (btb_bank0_rd_data_way1_f2[`TAG] == fetch_rd_tag_f2[pt.BTB_BTAG_SIZE-1:0])} &
                                   ~({2{dec_tlu_way_wb_f}} & branch_error_bank_conflict_f2[1:0]) & {2{ifc_fetch_req_f2_raw & ~leak_one_f2[ifc_select_tid_f2]}};


   assign tag_match_way0_p1_f2[1:0] = {btb_bank1_rd_data_way0_p1_f2[BV] & (btb_bank1_rd_data_way0_p1_f2[`TAG] == fetch_rd_tag_p1_f2[pt.BTB_BTAG_SIZE-1:0]),
                                       btb_bank0_rd_data_way0_p1_f2[BV] & (btb_bank0_rd_data_way0_p1_f2[`TAG] == fetch_rd_tag_p1_f2[pt.BTB_BTAG_SIZE-1:0])} &
                                      ~({2{~dec_tlu_way_wb_f}} & branch_error_bank_conflict_p1_f2[1:0]) & {2{ifc_fetch_req_f2_raw & ~leak_one_f2[ifc_select_tid_f2]}};

   assign tag_match_way1_p1_f2[1:0] = {btb_bank1_rd_data_way1_p1_f2[BV] & (btb_bank1_rd_data_way1_p1_f2[`TAG] == fetch_rd_tag_p1_f2[pt.BTB_BTAG_SIZE-1:0]),
                                       btb_bank0_rd_data_way1_p1_f2[BV] & (btb_bank0_rd_data_way1_p1_f2[`TAG] == fetch_rd_tag_p1_f2[pt.BTB_BTAG_SIZE-1:0])} &
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

   assign btb_bank1o_rd_data_f2[16+pt.BTB_BTAG_SIZE:0] = ( ({17+pt.BTB_BTAG_SIZE{tag_match_way0_expanded_f2[3]}} & btb_bank1_rd_data_way0_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{tag_match_way1_expanded_f2[3]}} & btb_bank1_rd_data_way1_f2[16+pt.BTB_BTAG_SIZE:0]) );
   assign btb_bank1e_rd_data_f2[16+pt.BTB_BTAG_SIZE:0] = ( ({17+pt.BTB_BTAG_SIZE{tag_match_way0_expanded_f2[2]}} & btb_bank1_rd_data_way0_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{tag_match_way1_expanded_f2[2]}} & btb_bank1_rd_data_way1_f2[16+pt.BTB_BTAG_SIZE:0]) );

   assign btb_bank0o_rd_data_f2[16+pt.BTB_BTAG_SIZE:0] = ( ({17+pt.BTB_BTAG_SIZE{tag_match_way0_expanded_f2[1]}} & btb_bank0_rd_data_way0_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{tag_match_way1_expanded_f2[1]}} & btb_bank0_rd_data_way1_f2[16+pt.BTB_BTAG_SIZE:0]) );
   assign btb_bank0e_rd_data_f2[16+pt.BTB_BTAG_SIZE:0] = ( ({17+pt.BTB_BTAG_SIZE{tag_match_way0_expanded_f2[0]}} & btb_bank0_rd_data_way0_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{tag_match_way1_expanded_f2[0]}} & btb_bank0_rd_data_way1_f2[16+pt.BTB_BTAG_SIZE:0]) );

   assign btb_bank1e_rd_data_p1_f2[16+pt.BTB_BTAG_SIZE:0] = ( ({17+pt.BTB_BTAG_SIZE{tag_match_way0_expanded_p1_f2[2]}} & btb_bank1_rd_data_way0_p1_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                               ({17+pt.BTB_BTAG_SIZE{tag_match_way1_expanded_p1_f2[2]}} & btb_bank1_rd_data_way1_p1_f2[16+pt.BTB_BTAG_SIZE:0]) );

   assign btb_bank0o_rd_data_p1_f2[16+pt.BTB_BTAG_SIZE:0] = ( ({17+pt.BTB_BTAG_SIZE{tag_match_way0_expanded_p1_f2[1]}} & btb_bank0_rd_data_way0_p1_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                               ({17+pt.BTB_BTAG_SIZE{tag_match_way1_expanded_p1_f2[1]}} & btb_bank0_rd_data_way1_p1_f2[16+pt.BTB_BTAG_SIZE:0]) );
   assign btb_bank0e_rd_data_p1_f2[16+pt.BTB_BTAG_SIZE:0] = ( ({17+pt.BTB_BTAG_SIZE{tag_match_way0_expanded_p1_f2[0]}} & btb_bank0_rd_data_way0_p1_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                               ({17+pt.BTB_BTAG_SIZE{tag_match_way1_expanded_p1_f2[0]}} & btb_bank0_rd_data_way1_p1_f2[16+pt.BTB_BTAG_SIZE:0]) );


   // virtual bank order

   assign btb_vbank0_rd_data_f2[16+pt.BTB_BTAG_SIZE:0] = ( ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[0]}} &  btb_bank0e_rd_data_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[1]}} &  btb_bank0o_rd_data_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[2]}} &  btb_bank1e_rd_data_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[3]}} &  btb_bank1o_rd_data_f2[16+pt.BTB_BTAG_SIZE:0]) );
   assign btb_vbank1_rd_data_f2[16+pt.BTB_BTAG_SIZE:0] = ( ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[0]}} &  btb_bank0o_rd_data_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[1]}} &  btb_bank1e_rd_data_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[2]}} &  btb_bank1o_rd_data_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[3]}} &  btb_bank0e_rd_data_p1_f2[16+pt.BTB_BTAG_SIZE:0]) );
   assign btb_vbank2_rd_data_f2[16+pt.BTB_BTAG_SIZE:0] = ( ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[0]}} &  btb_bank1e_rd_data_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[1]}} &  btb_bank1o_rd_data_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[2]}} &  btb_bank0e_rd_data_p1_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[3]}} &  btb_bank0o_rd_data_p1_f2[16+pt.BTB_BTAG_SIZE:0]) );
   assign btb_vbank3_rd_data_f2[16+pt.BTB_BTAG_SIZE:0] = ( ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[0]}} &  btb_bank1o_rd_data_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[1]}} &  btb_bank0e_rd_data_p1_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[2]}} &  btb_bank0o_rd_data_p1_f2[16+pt.BTB_BTAG_SIZE:0]) |
                                                            ({17+pt.BTB_BTAG_SIZE{fetch_start_f2[3]}} &  btb_bank1e_rd_data_p1_f2[16+pt.BTB_BTAG_SIZE:0]) );


   // --------------------------------------------------------------------------------
   // --------------------------------------------------------------------------------
   // update lru
   // mp

   assign fetch_req_val_f2[0] = ifc_fetch_req_f2_raw & ~leak_one_f2[0] & ~ifc_select_tid_f2;
   if(pt.NUM_THREADS > 1) begin

      assign fetch_req_val_f2[1] = ifc_fetch_req_f2_raw & ~leak_one_f2[1] &  ifc_select_tid_f2;

      // Simultaneous MPs to the same btb entry or bht entry can occur. Arb.
      assign mp_collision = exu_mp_valid[0] & exu_mp_valid[1] & ({exu_mp_addr[0], exu_mp_way[0], exu_mp_bank[0]} == {exu_mp_addr[1], exu_mp_way[1], exu_mp_bank[1]});
      assign mp_bht_collision = exu_mp_valid[0] & exu_mp_valid[1] & (mp_hashed[0] == mp_hashed[1]);

      rvarbiter2 mp_arbiter (
                             .ready(exu_mp_valid[1:0] & {2{mp_collision}}),
                             .tid  (mp_collision_winner_tid),
                             .shift(mp_collision),
                           .*
                             );
      rvarbiter2 mp_bht_arbiter (
                             .ready(exu_mp_valid[1:0] & {2{mp_bht_collision}}),
                             .tid  (mp_bht_collision_winner_tid),
                             .shift(mp_bht_collision),
                           .*
                             );


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

   end
   else begin
      assign mp_collision = 'b0;
      assign mp_collision_winner_tid = 'b0;
      assign mp_bht_collision = 'b0;
      assign mp_bht_collision_winner_tid = 'b0;
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

   genvar     j, i;

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

   assign tag_match_vway1_expanded_f2[3:0] = ( ({4{fetch_start_f2[0]}} & {tag_match_way1_expanded_f2[3:0]}) |
                                               ({4{fetch_start_f2[1]}} & {tag_match_way1_expanded_p1_f2[0], tag_match_way1_expanded_f2[3:1]}) |
                                               ({4{fetch_start_f2[2]}} & {tag_match_way1_expanded_p1_f2[1:0], tag_match_way1_expanded_f2[3:2]}) |
                                               ({4{fetch_start_f2[3]}} & {tag_match_way1_expanded_p1_f2[2:0], tag_match_way1_expanded_f2[3]}) );

   // Detect end of cache line and mask as needed
   assign eoc_near = &ifc_fetch_addr_f2[pt.ICACHE_BEAT_ADDR_HI:3];
   assign eoc_mask[3:1] = {3{~eoc_near}} | {ifc_fetch_addr_f2[2:1] == 2'b0,
                                            ~ifc_fetch_addr_f2[2],
                                            |(~ifc_fetch_addr_f2[2:1])};


   assign vwayhit_f2[3:0] = ( ({4{fetch_start_f2[0]}} & {wayhit_f2[3:0]}) |
                              ({4{fetch_start_f2[1]}} & {wayhit_p1_f2[0], wayhit_f2[3:1]}) |
                              ({4{fetch_start_f2[2]}} & {wayhit_p1_f2[1:0], wayhit_f2[3:2]}) |
                              ({4{fetch_start_f2[3]}} & {wayhit_p1_f2[2:0], wayhit_f2[3]}) ) & {eoc_mask[3:1], 1'b1};



   assign way_raw[3:0] =  tag_match_vway1_expanded_f2[3:0] | (~vwayhit_f2[3:0] & btb_vlru_rd_f2[3:0]);

   rvdffe #(LRU_SIZE*2) btb_lru_ff (.*, .en(ifc_fetch_req_f2 | (|exu_mp_valid[pt.NUM_THREADS-1:0])),
                                    .din({btb_lru_b0_ns[(LRU_SIZE)-1:0],
                                          btb_lru_b1_ns[(LRU_SIZE)-1:0]}),
                                   .dout({btb_lru_b0_f[(LRU_SIZE)-1:0],
                                          btb_lru_b1_f[(LRU_SIZE)-1:0]}));


   // --------------------------------------------------------------------------------
   // --------------------------------------------------------------------------------

   // mux out critical hit bank for pc computation
   // This is only useful for the first taken branch in the fetch group

   assign btb_rd_tgt_f2[11:0] = btb_sel_data_f2[16:5];
   assign btb_rd_pc4_f2       = btb_sel_data_f2[4];
   assign btb_rd_call_f2      = btb_sel_data_f2[2];
   assign btb_rd_ret_f2       = btb_sel_data_f2[1];

   assign btb_sel_data_f2[16:1] = ( ({16{btb_sel_f2[3]}} & btb_vbank3_rd_data_f2[16:1]) |
                                    ({16{btb_sel_f2[2]}} & btb_vbank2_rd_data_f2[16:1]) |
                                    ({16{btb_sel_f2[1]}} & btb_vbank1_rd_data_f2[16:1]) |
                                    ({16{btb_sel_f2[0]}} & btb_vbank0_rd_data_f2[16:1]) );



   // a valid taken target needs to kill the next fetch as we compute the target address
   assign ifu_bp_kill_next_f2 = |(vwayhit_f2[3:0] & hist1_raw[3:0]) & ifc_fetch_req_f2 & ~leak_one_f2[ifc_select_tid_f2] & ~dec_tlu_bpred_disable;


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


   for (genvar i=0; i<pt.NUM_THREADS; i++) begin : fghrmaint

      assign exu_mp_valid[i] = exu_mp_pkt[i].misp & ~leak_one_f2[ifc_select_tid_f2] & ~dec_tlu_error_wb; // conditional branch mispredict, unless there was a simul error
      assign exu_mp_boffset[i] = exu_mp_pkt[i].boffset;  // branch offset
      assign exu_mp_pc4[i] = exu_mp_pkt[i].pc4;  // branch is a 4B inst
      assign exu_mp_call[i] = exu_mp_pkt[i].pcall;  // branch is a call inst
      assign exu_mp_ret[i] = exu_mp_pkt[i].pret;  // branch is a ret inst
      assign exu_mp_ja[i] = exu_mp_pkt[i].pja;  // branch is a jump always
      assign exu_mp_way[i] = exu_mp_pkt[i].way;  // repl way
      assign exu_mp_hist[i][1:0] = exu_mp_pkt[i].hist[1:0];  // new history
      assign exu_mp_tgt[i][11:0]  = exu_mp_pkt[i].toffset[11:0] ;  // target offset
      assign exu_mp_bank[i]  = exu_mp_pkt[i].bank ;  // write bank = exu_mp_pkt[i].;  based on branch PC[3:2]
      assign exu_mp_ataken[i] = exu_mp_pkt[i].ataken;
      assign exu_mp_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]  = exu_mp_index[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ;  // BTB/BHT address

      assign btb_wr_tag[i][pt.BTB_BTAG_SIZE-1:0] = exu_mp_btag[i][pt.BTB_BTAG_SIZE-1:0];

      assign exu_mp_valid_write[i] = exu_mp_valid[i] & exu_mp_ataken[i] & (~mp_collision | (i == mp_collision_winner_tid));
      assign btb_wr_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] = dec_tlu_error_wb ? btb_error_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] : exu_mp_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];
      assign btb_wr_data[i][16+pt.BTB_BTAG_SIZE:0] = {btb_wr_tag[i][pt.BTB_BTAG_SIZE-1:0], exu_mp_tgt[i][11:0], exu_mp_pc4[i], exu_mp_boffset[i], exu_mp_call[i] | exu_mp_ja[i],
                                                      exu_mp_ret[i] | exu_mp_ja[i], ~dec_tlu_error_wb} ;


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


   rvdffe #(31) faddrf2raw_ff (.*, .en(ifc_fetch_req_f1), .din(ifc_fetch_addr_f1[31:1]), .dout(ifc_fetch_addr_f2[31:1]));

   rvdffe #(2*(pt.BTB_ADDR_HI-2) + 29) faddr_p1_f2ff (.*, .en(ifc_fetch_req_f1), .din({fetch_addr_p1_f1[31:3],
                                                                btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO],
                                                                btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]
                                                                }),
                                                         .dout({fetch_addr_p1_f2[31:3],
                                                                btb_rd_addr_f2[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO],
                                                                btb_rd_addr_p1_f2[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]
                                                                }));

   assign ifu_bp_poffset_f2[11:0] = btb_rd_tgt_f2[11:0];

   assign adder_pc_in_f2[31:3] = ( ({29{ use_fa_plus}} & fetch_addr_p1_f2[31:3]) |
                                   ({29{ btb_fg_crossing_f2}} & ifc_fetch_adder_prior[ifc_select_tid_f2][31:3]) |
                                   ({29{~btb_fg_crossing_f2 & ~use_fa_plus}} & ifc_fetch_addr_f2[31:3]));

   rvbradder predtgt_addr (.pc({adder_pc_in_f2[31:3], bp_total_branch_offset_f2[2:1]}),
                         .offset(btb_rd_tgt_f2[11:0]),
                         .dout(bp_btb_target_adder_f2[31:1])
                         );
   // mux in the return stack address here for a predicted return, if it is valid
   assign ifu_bp_btb_target_f2[31:1] = (btb_rd_ret_f2 & ~btb_rd_call_f2 & rets_out[ifc_select_tid_f2][0][0]) ? rets_out[ifc_select_tid_f2][0][31:1] : bp_btb_target_adder_f2[31:1];


   // ----------------------------------------------------------------------
   // Return Stack
   // ----------------------------------------------------------------------

   rvbradder rs_addr (.pc({adder_pc_in_f2[31:3], bp_total_branch_offset_f2[2:1]}),
                      .offset({10'b0, btb_rd_pc4_f2, ~btb_rd_pc4_f2}),
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


      assign rs_push[tid] = ((btb_rd_call_f2 & ~btb_rd_ret_f2 & ifu_bp_kill_next_f2 & fetch_req_val_f2[tid]) | (rs_overpop_correct[tid] & ~rs_underpop_correct[tid]));
      assign rs_pop[tid] = ((btb_rd_ret_f2 & ~btb_rd_call_f2 & ifu_bp_kill_next_f2 & fetch_req_val_f2[tid]) | (rs_underpop_correct[tid] & ~rs_overpop_correct[tid]));
      assign rs_hold[tid] = ~rs_push[tid] & ~rs_pop[tid] & ~rs_overpop_correct[tid] & ~rs_underpop_correct[tid];



      // Fetch based
      assign rets_in[tid][0][31:0] = ( ({32{rs_overpop_correct[tid] & rs_underpop_correct[tid]}} & rsoverpop_f[tid][31:0]) |
                                       ({32{rs_push[tid] & rs_overpop_correct[tid]}} & rsoverpop_f[tid][31:0]) |
                                       ({32{rs_push[tid] & ~rs_overpop_correct[tid]}} & {bp_rs_call_target_f2[31:1], 1'b1}) |
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

if (pt.BTB_BTAG_FOLD) begin : btbfold
   eh2_btb_tag_hash_fold #(.pt(pt)) rdtagf1(.hash(fetch_rd_tag_f1[pt.BTB_BTAG_SIZE-1:0]), .pc({ifc_fetch_addr_f1[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]}));
   eh2_btb_tag_hash_fold #(.pt(pt)) rdtagp1f1(.hash(fetch_rd_tag_p1_f1[pt.BTB_BTAG_SIZE-1:0]), .pc({fetch_addr_p1_f1[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]}));
end
   else begin
   eh2_btb_tag_hash #(.pt(pt)) rdtagf1(.hash(fetch_rd_tag_f1[pt.BTB_BTAG_SIZE-1:0]), .pc({ifc_fetch_addr_f1[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]}));
   eh2_btb_tag_hash #(.pt(pt)) rdtagp1f1(.hash(fetch_rd_tag_p1_f1[pt.BTB_BTAG_SIZE-1:0]), .pc({fetch_addr_p1_f1[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]}));
end

   rvdff #(pt.BTB_BTAG_SIZE+1) rdtagf (.*, .clk(active_clk), .din({ifc_select_tid_f1, fetch_rd_tag_f1[pt.BTB_BTAG_SIZE-1:0]}),
                                                            .dout({ifc_select_tid_f2, fetch_rd_tag_f2[pt.BTB_BTAG_SIZE-1:0]}));

   rvdff #(pt.BTB_BTAG_SIZE) rdtagp1f (.*, .clk(active_clk), .din({fetch_rd_tag_p1_f1[pt.BTB_BTAG_SIZE-1:0]}),
                                                            .dout({fetch_rd_tag_p1_f2[pt.BTB_BTAG_SIZE-1:0]}));



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



   eh2_btb_ghr_hash #(.pt(pt)) br0ghrhs (.hashin(dec_tlu_br0_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]), .ghr({dec_tlu_br0_fghr_wb[pt.BHT_GHR_SIZE-1:0]}), .hash(br0_hashed_wb[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO]));
   eh2_btb_ghr_hash #(.pt(pt)) br1ghrhs (.hashin(dec_tlu_br1_addr_wb[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]), .ghr({dec_tlu_br1_fghr_wb[pt.BHT_GHR_SIZE-1:0]}), .hash(br1_hashed_wb[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO]));
   eh2_btb_ghr_hash #(.pt(pt)) fghrhs (.hashin(btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]), .ghr({fghr_ns[ifc_select_tid_f1][pt.BHT_GHR_SIZE-1:0]}), .hash(bht_rd_addr_hashed_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO]));
   eh2_btb_ghr_hash #(.pt(pt)) fghrhs_p1 (.hashin(btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]), .ghr({fghr_ns[ifc_select_tid_f1][pt.BHT_GHR_SIZE-1:0]}), .hash(bht_rd_addr_hashed_p1_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO]));

   assign bht_wr_addr0[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = mp_hashed[0][pt.BHT_ADDR_HI:pt.BHT_ADDR_LO];
   assign bht_wr_addr1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = br1_hashed_wb[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO];
   assign bht_wr_addr2[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = br0_hashed_wb[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO];
   assign bht_rd_addr_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = bht_rd_addr_hashed_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO];
   assign bht_rd_addr_p1_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO] = bht_rd_addr_hashed_p1_f1[pt.BHT_ADDR_HI:pt.BHT_ADDR_LO];

   assign btb_wr_en_error_way0[1:0] = ( ({2{~dec_tlu_way_wb & dec_tlu_error_wb & ~dec_tlu_all_banks_error_wb}} & decode1_2(dec_tlu_error_bank_wb)) |
                                        ({2{~dec_tlu_way_wb & dec_tlu_all_banks_error_wb}}));

   assign btb_wr_en_error_way1[1:0] = ( ({2{dec_tlu_way_wb & dec_tlu_error_wb & ~dec_tlu_all_banks_error_wb}} & decode1_2(dec_tlu_error_bank_wb)) |
                                        ({2{dec_tlu_way_wb & dec_tlu_all_banks_error_wb}}));

       if (pt.NUM_THREADS == 1) begin
          assign btb_wr_data[1] = 'b0;
       end

   // ----------------------------------------------------------------------
   // Structures. Using FLOPS
   // ----------------------------------------------------------------------
   // BTB
   // Entry -> tag[pt.BTB_BTAG_SIZE-1:0], toffset[11:0], pc4, boffset, call, ret, valid

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

          rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank0_way0 (.*,
                    .en   (wr0_b0w0[j] | wr1_b0w0[j]),
                    .din  (wr0_b0w0[j] ? btb_wr_data[0][16+pt.BTB_BTAG_SIZE:0] : btb_wr_data[1][16+pt.BTB_BTAG_SIZE:0]),
                    .dout (btb_bank0_rd_data_way0_out[j]));

          rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank1_way0 (.*,
                    .en   (wr0_b1w0[j] | wr1_b1w0[j]),
                    .din  (wr0_b1w0[j] ? btb_wr_data[0][16+pt.BTB_BTAG_SIZE:0] : btb_wr_data[1][16+pt.BTB_BTAG_SIZE:0]),
                    .dout (btb_bank1_rd_data_way0_out[j]));

      // Way 1
          rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank0_way1 (.*,
                    .en   (wr0_b0w1[j] | wr1_b0w1[j]),
                    .din  (wr0_b0w1[j] ? btb_wr_data[0][16+pt.BTB_BTAG_SIZE:0] : btb_wr_data[1][16+pt.BTB_BTAG_SIZE:0]),
                    .dout (btb_bank0_rd_data_way1_out[j]));

          rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank1_way1 (.*,
                    .en   (wr0_b1w1[j] | wr1_b1w1[j]),
                    .din  (wr0_b1w1[j] ? btb_wr_data[0][16+pt.BTB_BTAG_SIZE:0] : btb_wr_data[1][16+pt.BTB_BTAG_SIZE:0]),
                    .dout (btb_bank1_rd_data_way1_out[j]));

    end

   rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank0_way0_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank0_rd_data_way0_f2_in[16+pt.BTB_BTAG_SIZE:0]),
                    .dout       (btb_bank0_rd_data_way0_f2   [16+pt.BTB_BTAG_SIZE:0]));

   rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank1_way0_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank1_rd_data_way0_f2_in[16+pt.BTB_BTAG_SIZE:0]),
                    .dout       (btb_bank1_rd_data_way0_f2   [16+pt.BTB_BTAG_SIZE:0]));

   rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank0_way1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank0_rd_data_way1_f2_in[16+pt.BTB_BTAG_SIZE:0]),
                    .dout       (btb_bank0_rd_data_way1_f2   [16+pt.BTB_BTAG_SIZE:0]));

   rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank1_way1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank1_rd_data_way1_f2_in[16+pt.BTB_BTAG_SIZE:0]),
                    .dout       (btb_bank1_rd_data_way1_f2   [16+pt.BTB_BTAG_SIZE:0]));


   rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank0_way0_p1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank0_rd_data_way0_p1_f2_in[16+pt.BTB_BTAG_SIZE:0]),
                    .dout       (btb_bank0_rd_data_way0_p1_f2   [16+pt.BTB_BTAG_SIZE:0]));

   rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank1_way0_p1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank1_rd_data_way0_p1_f2_in[16+pt.BTB_BTAG_SIZE:0]),
                    .dout       (btb_bank1_rd_data_way0_p1_f2   [16+pt.BTB_BTAG_SIZE:0]));

   rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank0_way1_p1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank0_rd_data_way1_p1_f2_in[16+pt.BTB_BTAG_SIZE:0]),
                    .dout       (btb_bank0_rd_data_way1_p1_f2   [16+pt.BTB_BTAG_SIZE:0]));

   rvdffe #(17+pt.BTB_BTAG_SIZE) btb_bank1_way1_p1_data_out (.*,
                    .en(ifc_fetch_req_f1),
                    .din        (btb_bank1_rd_data_way1_p1_f2_in[16+pt.BTB_BTAG_SIZE:0]),
                    .dout       (btb_bank1_rd_data_way1_p1_f2   [16+pt.BTB_BTAG_SIZE:0]));


    always_comb begin : BTB_rd_mux
        btb_bank0_rd_data_way0_f2_in[16+pt.BTB_BTAG_SIZE:0] = '0 ;
        btb_bank1_rd_data_way0_f2_in[16+pt.BTB_BTAG_SIZE:0] = '0 ;
        btb_bank0_rd_data_way1_f2_in[16+pt.BTB_BTAG_SIZE:0] = '0 ;
        btb_bank1_rd_data_way1_f2_in[16+pt.BTB_BTAG_SIZE:0] = '0 ;
        btb_bank0_rd_data_way0_p1_f2_in[16+pt.BTB_BTAG_SIZE:0] = '0 ;
        btb_bank1_rd_data_way0_p1_f2_in[16+pt.BTB_BTAG_SIZE:0] = '0 ;
        btb_bank0_rd_data_way1_p1_f2_in[16+pt.BTB_BTAG_SIZE:0] = '0 ;
        btb_bank1_rd_data_way1_p1_f2_in[16+pt.BTB_BTAG_SIZE:0] = '0 ;

        for (int j=0; j< LRU_SIZE; j++) begin
          if (btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == (pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1)'(j)) begin

           btb_bank0_rd_data_way0_f2_in[16+pt.BTB_BTAG_SIZE:0] =  btb_bank0_rd_data_way0_out[j];
           btb_bank1_rd_data_way0_f2_in[16+pt.BTB_BTAG_SIZE:0] =  btb_bank1_rd_data_way0_out[j];

           btb_bank0_rd_data_way1_f2_in[16+pt.BTB_BTAG_SIZE:0] =  btb_bank0_rd_data_way1_out[j];
           btb_bank1_rd_data_way1_f2_in[16+pt.BTB_BTAG_SIZE:0] =  btb_bank1_rd_data_way1_out[j];

          end
        end
        for (int j=0; j< LRU_SIZE; j++) begin
          if (btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] == (pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1)'(j)) begin

           btb_bank0_rd_data_way0_p1_f2_in[16+pt.BTB_BTAG_SIZE:0] =  btb_bank0_rd_data_way0_out[j];
           btb_bank1_rd_data_way0_p1_f2_in[16+pt.BTB_BTAG_SIZE:0] =  btb_bank1_rd_data_way0_out[j];

           btb_bank0_rd_data_way1_p1_f2_in[16+pt.BTB_BTAG_SIZE:0] =  btb_bank0_rd_data_way1_out[j];
           btb_bank1_rd_data_way1_p1_f2_in[16+pt.BTB_BTAG_SIZE:0] =  btb_bank1_rd_data_way1_out[j];

          end
        end
    end

   //-----------------------------------------------------------------------------
   // BHT
   // 2 bit Entry -> direction, strength
   //
   //-----------------------------------------------------------------------------

   for ( genvar i=0; i<4; i++) begin : BANKS
     for (genvar k=0 ; k < (pt.BHT_ARRAY_DEPTH)/NUM_BHT_LOOP ; k++) begin : BHT_CLK_GROUP
     assign bht_bank_clken[i][k]  = (bht_wr_en0[i] & ((bht_wr_addr0[pt.BHT_ADDR_HI: NUM_BHT_LOOP_OUTER_LO]==k) |  BHT_NO_ADDR_MATCH)) |
                                    (bht_wr_en1[i] & ((bht_wr_addr1[pt.BHT_ADDR_HI: NUM_BHT_LOOP_OUTER_LO]==k) |  BHT_NO_ADDR_MATCH)) |
                                    (bht_wr_en2[i] & ((bht_wr_addr2[pt.BHT_ADDR_HI: NUM_BHT_LOOP_OUTER_LO]==k) |  BHT_NO_ADDR_MATCH)) |
                                    (bht_wr_en3[i] & ((bht_wr_addr3[pt.BHT_ADDR_HI: NUM_BHT_LOOP_OUTER_LO]==k) |  BHT_NO_ADDR_MATCH));

     rvclkhdr bht_bank_grp_cgc ( .en(bht_bank_clken[i][k]), .l1clk(bht_bank_clk[i][k]), .* );

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

          rvdffs #(2) bht_bank (.*,
                    .clk        (bht_bank_clk[i][k]),
                    .en         (|wr_sel),
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

endmodule // eh2_ifu_bp_ctl

