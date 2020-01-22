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
// Function: Instruction aligner
//********************************************************************************
module eh2_ifu_aln_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (

   input logic        active_clk,

   input logic        tid,                                         // hardwired tid for align

   input logic        dec_i1_cancel_e1,

   input logic        ifu_async_error_start,                       // ecc/parity related errors with current fetch - not sent down the pipe

   input logic        iccm_rd_ecc_double_err,                      // This fetch has a double ICCM ecc  error.

   input logic        ic_access_fault_f2,                          // Instruction access fault for the current fetch.
   input logic [1:0]  ic_access_fault_type_f2,                     // Instruction access fault types

   input logic [pt.BHT_GHR_SIZE-1:0]  ifu_bp_fghr_f2,              // fetch GHR
   input logic [31:1] ifu_bp_btb_target_f2,                        //  predicted RET target
   input logic [11:0] ifu_bp_poffset_f2,                           // predicted target offset

   input logic [3:0]  ifu_bp_hist0_f2,                             // history counters for all 4 potential branches, bit 1, right justified
   input logic [3:0]  ifu_bp_hist1_f2,                             // history counters for all 4 potential branches, bit 1, right justified
   input logic [3:0]  ifu_bp_pc4_f2,                               // pc4 indication, right justified

   input logic [3:0]  ifu_bp_way_f2,                               // way indication, right justified

   input logic [3:0]  ifu_bp_valid_f2,                             // branch valid, right justified
   input logic [3:0]  ifu_bp_ret_f2,                               // predicted ret indication, right justified

   input logic exu_flush_final,                                    // Flush from the pipeline.

   input logic dec_ib3_valid_d,                                    // valids for top 2 instruction buffers at decode
   input logic dec_ib2_valid_d,


   input logic [63:0] ifu_fetch_data,                              // fetch data in memory format - not right justified

   input logic [3:0]   ifu_fetch_val,                              // valids on a 2B boundary, right justified
   input logic [31:1]  ifu_fetch_pc,                               // starting pc of fetch
   input logic         ifu_fetch_tid,                              // tid


   input logic   rst_l,
   input logic   clk,

   output logic i0_valid,                                      // Instruction 0 is valid
   output logic i1_valid,                                      // Instruction 1 is valid
   output logic i0_icaf,                                       // Instruction 0 has access fault
   output logic [1:0]  i0_icaf_type,                           // Instruction 0 access fault type
   output logic i0_icaf_f1,                                    // Instruction 0 has access fault on second fetch group
   output logic i0_dbecc,                                      // Instruction 0 has double bit ecc error
   output logic [31:0] i0_instr,                               // Instruction 0
   output logic [31:0] i1_instr,                               // Instruction 1
   output logic [31:1] i0_pc,                                  // Instruction 0 PC
   output logic [31:1] i1_pc,                                  // Instruction 1 PC
   output logic i0_pc4,
   output logic i1_pc4,
   output eh2_predecode_pkt_t i0_predecode,
   output eh2_predecode_pkt_t i1_predecode,

   output logic fb_consume1,                                   // Consumed one buffer. To fetch control fetch for buffer mass balance
   output logic fb_consume2,                                   // Consumed two buffers.To fetch control fetch for buffer mass balance

   output eh2_br_pkt_t i0_br_p,                                    // Branch packet for I0.
   output eh2_br_pkt_t i1_br_p,                                    // Branch packet for I1.
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]  i0_bp_index,  // BP index
   output logic [pt.BHT_GHR_SIZE-1:0]            i0_bp_fghr,   // BP FGHR
   output logic [pt.BTB_BTAG_SIZE-1:0]           i0_bp_btag,   // BP tag
   output logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]  i1_bp_index,  // BP index
   output logic [pt.BHT_GHR_SIZE-1:0]            i1_bp_fghr,   // BP FGHR
   output logic [pt.BTB_BTAG_SIZE-1:0]           i1_bp_btag,   // BP tag

   output logic [1:0] pmu_instr_aligned,                       // number of inst aligned this cycle
   output logic       pmu_align_stall,                         // aligner stalled this cycle

   output logic [15:0] i0_cinst,                               // 16b compress inst for i0
   output logic [15:0] i1_cinst,                               // 16b compress inst for i1

   input  logic    scan_mode


   );

   logic         ifvalid;
   logic         shift_f1_f0, shift_f2_f0, shift_f2_f1;
   logic         fetch_to_f0, fetch_to_f1, fetch_to_f2;

   logic [3:0]   f3val_in, f3val;
   logic [3:0]   f2val_in, f2val;
   logic [3:0]   f1val_in, f1val;
   logic [3:0]   f0val_in, f0val;

   logic [3:0]   sf1val, sf0val;

   logic [63:0]  f3data_in, f3data;
   logic [63:0]  f2data_in, f2data;
   logic [63:0]  f1data_in, f1data, sf1data;
   logic [63:0]  f0data_in, f0data, sf0data;

   logic [31:1]  f3pc_in, f3pc;
   logic [31:1]  f2pc_in, f2pc;
   logic [31:1]  f1pc_in, f1pc;
   logic [31:1]  f0pc_in, f0pc;
   logic [31:1]  sf1pc, sf0pc;

   logic [63:0]  aligndata;
   logic         first4B, first2B;
   logic         second4B, second2B;

   logic         third4B, third2B;
   logic [31:0]  uncompress0, uncompress1, uncompress2;
   logic         ibuffer_room1_more;
   logic         ibuffer_room2_more;
   logic         i1_shift;
   logic         shift_2B, shift_4B, shift_6B, shift_8B;
   logic         f1_shift_2B, f1_shift_4B, f1_shift_6B;
   logic         f2_valid, sf1_valid, sf0_valid;

   logic [31:0]  ifirst, isecond, ithird;
   logic [31:1]  f0pc_plus1, f0pc_plus2, f0pc_plus3, f0pc_plus4;
   logic [31:1]  f1pc_plus1, f1pc_plus2, f1pc_plus3;
   logic [3:0]   alignval;
   logic [31:1]  firstpc, secondpc, thirdpc, fourthpc;

   logic [11:0]  f1poffset;
   logic [11:0]  f0poffset;
   logic [pt.BHT_GHR_SIZE-1:0]  f1fghr;
   logic [pt.BHT_GHR_SIZE-1:0]  f0fghr;
   logic [3:0]   f1hist1;
   logic [3:0]   f0hist1;
   logic [3:0]   f1hist0;
   logic [3:0]   f0hist0;

   logic [1:0]                                    f1ictype;
   logic [1:0]                                    f0ictype;

   logic [3:0]   f1pc4;
   logic [3:0]   f0pc4;

   logic [3:0]   f1ret;
   logic [3:0]   f0ret;
   logic [3:0]   f1way;
   logic [3:0]   f0way;


   logic [3:0]   f1brend;
   logic [3:0]   f0brend;

   logic [3:0]   alignbrend;
   logic [3:0]   alignpc4;

   logic [3:0]   alignret;
   logic [3:0]   alignway;
   logic [3:0]   alignhist1;

   logic [3:0]   alignhist0;
   logic [3:1]   alignfromf1;
   logic         i0_ends_f1, i1_ends_f1;
   logic         i0_br_start_error, i1_br_start_error;

   logic [31:1]  f1prett;
   logic [31:1]  f0prett;
   logic         f1dbecc;
   logic         f0dbecc;
   logic         f1icaf;
   logic         f0icaf;

   logic [3:0]   aligndbecc;
   logic [3:0]   alignicaf;
   logic         i0_brp_pc4, i1_brp_pc4;

   logic [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] firstpc_hash, secondpc_hash, thirdpc_hash, fourthpc_hash;

   logic f2_wr_en;
   logic f0_shift_wr_en;
   logic f1_shift_wr_en;

   logic [1:0] wrptr, wrptr_in;
   logic [1:0] rdptr, rdptr_in;
   logic [3:0] qwen;
   logic [1:0]   first_offset, second_offset;
   logic [1:0]   q3off_eff, q3off_in, q3off;
   logic [1:0]   q2off_eff, q2off_in, q2off;
   logic [1:0]   q1off_eff, q1off_in, q1off;
   logic [1:0]   q0off_eff, q0off_in, q0off;
   logic         f0_shift_2B, f0_shift_4B, f0_shift_6B;

   logic [1:0]   q0ptr;
   logic [3:0]   q0sel;

   logic [1:0]   q1ptr;
   logic [3:0]   q1sel;

   logic [3:0]   qren;

   logic         consume_fb1, consume_fb0;
   logic [3:1]   icaf_eff;


   logic [3:0]   fetch_val;

   logic         error_stall_in, error_stall;
   localparam MHI   = 46+pt.BHT_GHR_SIZE;
   localparam MSIZE = 47+pt.BHT_GHR_SIZE;

   logic [MHI:0] misc_data_in, misc3, misc2, misc1, misc0;
   logic [MHI:0] misc1eff, misc0eff;
   localparam BRDATA_SIZE=24;
   localparam BRDATA_WIDTH = 6;

   logic [BRDATA_SIZE-1:0] brdata_in, brdata3, brdata2, brdata1, brdata0;
   logic [BRDATA_SIZE-1:0] brdata1eff, brdata0eff;
   logic [BRDATA_SIZE-1:0] brdata1final, brdata0final;
   logic  i1_icaf;
   logic  i1_dbecc;
   logic [31:0] i0,i1;
   logic [pt.BTB_BTAG_SIZE-1:0] firstbrtag_hash, secondbrtag_hash, thirdbrtag_hash, fourthbrtag_hash;

   logic                        shift_f3_f2, shift_f3_f1;
   logic                        fetch_to_f3;
   logic                        f3_wr_en;
   logic                        f3_valid;

   logic                        i0_shift;

   assign f3_wr_en = fetch_to_f3;

   assign f2_wr_en = fetch_to_f2 | shift_f3_f2;

   assign f0_shift_wr_en = (fetch_to_f0 | shift_f2_f0 | shift_f1_f0 | shift_2B | shift_4B | shift_6B | shift_8B);

   assign f1_shift_wr_en = (fetch_to_f1 | shift_f3_f1 | shift_f2_f1 | f1_shift_2B | f1_shift_4B | f1_shift_6B);

   assign fetch_val[3:0] = ifu_fetch_val[3:0]  & {4{tid == ifu_fetch_tid}};

   assign error_stall_in = (error_stall | (ifu_async_error_start & (tid == ifu_fetch_tid))) & ~exu_flush_final;

   rvdff  #(1)            error_stallff   (.*, .clk(active_clk),    .din(error_stall_in),         .dout(error_stall));



   // new queue control logic

   assign wrptr_in[1:0] =  (({2{wrptr[1:0]==2'b00 & ifvalid}} & 2'b01) |
                            ({2{wrptr[1:0]==2'b01 & ifvalid}} & 2'b10) |
                            ({2{wrptr[1:0]==2'b10 & ifvalid}} & 2'b11) |
                            ({2{wrptr[1:0]==2'b11 & ifvalid}} & 2'b00) |
                            ({2{~ifvalid}} & wrptr[1:0])) & ~{2{exu_flush_final}};

   rvdff #(2) wrpff (.*, .clk(active_clk), .din(wrptr_in[1:0]), .dout(wrptr[1:0]));

   assign rdptr_in[1:0] =  (({2{rdptr[1:0]==2'b00 & fb_consume1}} & 2'b01) |
                            ({2{rdptr[1:0]==2'b01 & fb_consume1}} & 2'b10) |
                            ({2{rdptr[1:0]==2'b10 & fb_consume1}} & 2'b11) |
                            ({2{rdptr[1:0]==2'b11 & fb_consume1}} & 2'b00) |
                            ({2{rdptr[1:0]==2'b00 & fb_consume2}} & 2'b10) |
                            ({2{rdptr[1:0]==2'b01 & fb_consume2}} & 2'b11) |
                            ({2{rdptr[1:0]==2'b10 & fb_consume2}} & 2'b00) |
                            ({2{rdptr[1:0]==2'b11 & fb_consume2}} & 2'b01) |
                            ({2{~fb_consume1&~fb_consume2}} & rdptr[1:0])) & ~{2{exu_flush_final}};

   rvdff #(2) rdpff (.*, .clk(active_clk), .din(rdptr_in[1:0]), .dout(rdptr[1:0]));

   assign qren[3:0] = { rdptr[1:0]==2'b11,
                        rdptr[1:0]==2'b10,
                        rdptr[1:0]==2'b01,
                        rdptr[1:0]==2'b00
                        };

   assign qwen[3:0] = { wrptr[1:0]==2'b11 & ifvalid,
                        wrptr[1:0]==2'b10 & ifvalid,
                        wrptr[1:0]==2'b01 & ifvalid,
                        wrptr[1:0]==2'b00 & ifvalid
                        };


   assign first_offset[1:0]  = {f0_shift_6B|f0_shift_4B,  f0_shift_6B|f0_shift_2B };

   assign second_offset[1:0] = {f1_shift_6B|f1_shift_4B,  f1_shift_6B|f1_shift_2B };


   assign q3off_eff[1:0] = (rdptr[1:0]==2'd3) ? (q3off[1:0] + first_offset[1:0])  :
                           (rdptr[1:0]==2'd2) ? (q3off[1:0] + second_offset[1:0]) :
                                                 q3off[1:0];

   assign q3off_in[1:0] = (qwen[3]) ? {2'b0} : q3off_eff[1:0];

   rvdff #(2) q3offsetff (.*, .clk(active_clk), .din(q3off_in[1:0]), .dout(q3off[1:0]));

   assign q2off_eff[1:0] = (rdptr[1:0]==2'd2) ? (q2off[1:0] + first_offset[1:0])  :
                           (rdptr[1:0]==2'd1) ? (q2off[1:0] + second_offset[1:0]) :
                                                 q2off[1:0];

   assign q2off_in[1:0] = (qwen[2]) ? {2'b0} : q2off_eff[1:0];

   rvdff #(2) q2offsetff (.*, .clk(active_clk), .din(q2off_in[1:0]), .dout(q2off[1:0]));

   assign q1off_eff[1:0] = (rdptr[1:0]==2'd1) ? (q1off[1:0] + first_offset[1:0])  :
                           (rdptr[1:0]==2'd0) ? (q1off[1:0] + second_offset[1:0]) :
                                                 q1off[1:0];


   assign q1off_in[1:0] = (qwen[1]) ? {2'b0} : q1off_eff[1:0];

   rvdff #(2) q1offsetff (.*, .clk(active_clk), .din(q1off_in[1:0]), .dout(q1off[1:0]));


   assign q0off_eff[1:0] = (rdptr[1:0]==2'd0) ? (q0off[1:0] + first_offset[1:0])  :
                           (rdptr[1:0]==2'd3) ? (q0off[1:0] + second_offset[1:0]) :
                                                 q0off[1:0];


   assign q0off_in[1:0] = (qwen[0]) ? {2'b0} : q0off_eff[1:0];


   rvdff #(2) q0offsetff (.*, .clk(active_clk), .din(q0off_in[1:0]), .dout(q0off[1:0]));


   assign q0ptr[1:0] = (({2{rdptr[1:0]==2'b00}} & q0off[1:0]) |
                        ({2{rdptr[1:0]==2'b01}} & q1off[1:0]) |
                        ({2{rdptr[1:0]==2'b10}} & q2off[1:0]) |
                        ({2{rdptr[1:0]==2'b11}} & q3off[1:0]));

   assign q1ptr[1:0] = (({2{rdptr[1:0]==2'b00}} & q1off[1:0]) |
                        ({2{rdptr[1:0]==2'b01}} & q2off[1:0]) |
                        ({2{rdptr[1:0]==2'b10}} & q3off[1:0]) |
                        ({2{rdptr[1:0]==2'b11}} & q0off[1:0]));

   assign q0sel[3:0] = {
                         q0ptr[1:0]==2'b11,
                         q0ptr[1:0]==2'b10,
                         q0ptr[1:0]==2'b01,
                         q0ptr[1:0]==2'b00
                         };

   assign q1sel[3:0] = {
                         q1ptr[1:0]==2'b11,
                         q1ptr[1:0]==2'b10,
                         q1ptr[1:0]==2'b01,
                         q1ptr[1:0]==2'b00
                         };

   // end new queue control logic


   // misc data that is associated with each fetch buffer


   assign misc_data_in[MHI:0] = { iccm_rd_ecc_double_err,
                                  ic_access_fault_f2,
                                  ic_access_fault_type_f2[1:0],
                                  ifu_bp_btb_target_f2[31:1],
                                  ifu_bp_poffset_f2[11:0],
                                  ifu_bp_fghr_f2[pt.BHT_GHR_SIZE-1:0]
                                  };

   rvdffe #(MSIZE) misc3ff (.*, .en(qwen[3]), .din(misc_data_in[MHI:0]), .dout(misc3[MHI:0]));
   rvdffe #(MSIZE) misc2ff (.*, .en(qwen[2]), .din(misc_data_in[MHI:0]), .dout(misc2[MHI:0]));
   rvdffe #(MSIZE) misc1ff (.*, .en(qwen[1]), .din(misc_data_in[MHI:0]), .dout(misc1[MHI:0]));
   rvdffe #(MSIZE) misc0ff (.*, .en(qwen[0]), .din(misc_data_in[MHI:0]), .dout(misc0[MHI:0]));


   assign {misc1eff[MHI:0],misc0eff[MHI:0]} = (({MSIZE*2{qren[0]}} & {misc1[MHI:0],misc0[MHI:0]}) |
                                               ({MSIZE*2{qren[1]}} & {misc2[MHI:0],misc1[MHI:0]}) |
                                               ({MSIZE*2{qren[2]}} & {misc3[MHI:0],misc2[MHI:0]}) |
                                               ({MSIZE*2{qren[3]}} & {misc0[MHI:0],misc3[MHI:0]}));

   assign { f1dbecc,
            f1icaf,
            f1ictype[1:0],
            f1prett[31:1],
            f1poffset[11:0],
            f1fghr[pt.BHT_GHR_SIZE-1:0]
            } = misc1eff[MHI:0];

   assign { f0dbecc,
            f0icaf,
            f0ictype[1:0],
            f0prett[31:1],
            f0poffset[11:0],
            f0fghr[pt.BHT_GHR_SIZE-1:0]
            } = misc0eff[MHI:0];



   assign brdata_in[BRDATA_SIZE-1:0] = {
                              ifu_bp_hist1_f2[3],ifu_bp_hist0_f2[3],ifu_bp_pc4_f2[3],ifu_bp_way_f2[3],ifu_bp_valid_f2[3],ifu_bp_ret_f2[3],
                              ifu_bp_hist1_f2[2],ifu_bp_hist0_f2[2],ifu_bp_pc4_f2[2],ifu_bp_way_f2[2],ifu_bp_valid_f2[2],ifu_bp_ret_f2[2],
                              ifu_bp_hist1_f2[1],ifu_bp_hist0_f2[1],ifu_bp_pc4_f2[1],ifu_bp_way_f2[1],ifu_bp_valid_f2[1],ifu_bp_ret_f2[1],
                              ifu_bp_hist1_f2[0],ifu_bp_hist0_f2[0],ifu_bp_pc4_f2[0],ifu_bp_way_f2[0],ifu_bp_valid_f2[0],ifu_bp_ret_f2[0]
                              };
   rvdffe #(BRDATA_SIZE) brdata3ff (.*, .en(qwen[3]), .din(brdata_in[BRDATA_SIZE-1:0]), .dout(brdata3[BRDATA_SIZE-1:0]));
   rvdffe #(BRDATA_SIZE) brdata2ff (.*, .en(qwen[2]), .din(brdata_in[BRDATA_SIZE-1:0]), .dout(brdata2[BRDATA_SIZE-1:0]));
   rvdffe #(BRDATA_SIZE) brdata1ff (.*, .en(qwen[1]), .din(brdata_in[BRDATA_SIZE-1:0]), .dout(brdata1[BRDATA_SIZE-1:0]));
   rvdffe #(BRDATA_SIZE) brdata0ff (.*, .en(qwen[0]), .din(brdata_in[BRDATA_SIZE-1:0]), .dout(brdata0[BRDATA_SIZE-1:0]));


   assign {brdata1eff[BRDATA_SIZE-1:0],brdata0eff[BRDATA_SIZE-1:0]} = (({BRDATA_SIZE*2{qren[0]}} & {brdata1[BRDATA_SIZE-1:0],brdata0[BRDATA_SIZE-1:0]}) |
                                                                       ({BRDATA_SIZE*2{qren[1]}} & {brdata2[BRDATA_SIZE-1:0],brdata1[BRDATA_SIZE-1:0]}) |
                                                                       ({BRDATA_SIZE*2{qren[2]}} & {brdata3[BRDATA_SIZE-1:0],brdata2[BRDATA_SIZE-1:0]}) |
                                                                       ({BRDATA_SIZE*2{qren[3]}} & {brdata0[BRDATA_SIZE-1:0],brdata3[BRDATA_SIZE-1:0]}));


   assign brdata0final[BRDATA_SIZE-1:0] = (({BRDATA_SIZE{q0sel[0]}} & {                       brdata0eff[BRDATA_SIZE-1:0*BRDATA_WIDTH]}) |
                                           ({BRDATA_SIZE{q0sel[1]}} & {{1*BRDATA_WIDTH{1'b0}},brdata0eff[BRDATA_SIZE-1:1*BRDATA_WIDTH]}) |
                                           ({BRDATA_SIZE{q0sel[2]}} & {{2*BRDATA_WIDTH{1'b0}},brdata0eff[BRDATA_SIZE-1:2*BRDATA_WIDTH]}) |
                                           ({BRDATA_SIZE{q0sel[3]}} & {{3*BRDATA_WIDTH{1'b0}},brdata0eff[BRDATA_SIZE-1:3*BRDATA_WIDTH]}));


   assign brdata1final[BRDATA_SIZE-1:0] = (({BRDATA_SIZE{q1sel[0]}} & {                       brdata1eff[BRDATA_SIZE-1:0*BRDATA_WIDTH]}) |
                                           ({BRDATA_SIZE{q1sel[1]}} & {{1*BRDATA_WIDTH{1'b0}},brdata1eff[BRDATA_SIZE-1:1*BRDATA_WIDTH]}) |
                                           ({BRDATA_SIZE{q1sel[2]}} & {{2*BRDATA_WIDTH{1'b0}},brdata1eff[BRDATA_SIZE-1:2*BRDATA_WIDTH]}) |
                                           ({BRDATA_SIZE{q1sel[3]}} & {{3*BRDATA_WIDTH{1'b0}},brdata1eff[BRDATA_SIZE-1:3*BRDATA_WIDTH]}));


   assign {
            f0hist1[3],f0hist0[3],f0pc4[3],f0way[3],f0brend[3],f0ret[3],
            f0hist1[2],f0hist0[2],f0pc4[2],f0way[2],f0brend[2],f0ret[2],
            f0hist1[1],f0hist0[1],f0pc4[1],f0way[1],f0brend[1],f0ret[1],
            f0hist1[0],f0hist0[0],f0pc4[0],f0way[0],f0brend[0],f0ret[0]
            } = brdata0final[BRDATA_SIZE-1:0];

   assign {
            f1hist1[3],f1hist0[3],f1pc4[3],f1way[3],f1brend[3],f1ret[3],
            f1hist1[2],f1hist0[2],f1pc4[2],f1way[2],f1brend[2],f1ret[2],
            f1hist1[1],f1hist0[1],f1pc4[1],f1way[1],f1brend[1],f1ret[1],
            f1hist1[0],f1hist0[0],f1pc4[0],f1way[0],f1brend[0],f1ret[0]
            } = brdata1final[BRDATA_SIZE-1:0];


   // possible states of { sf0_valid, sf1_valid, f2_valid, f3_valid }

   // 0001 illegal
   // 1010 illegal
   // 1011 illegal
   // 1001 illegal
   // 0101 illegal
   // 1101 illegal

   // 0000 if->f0

   // 1000 if->f1

   // 0100 f1->f0, if->f1

   // 0110 f1->f0, f2->f1, if->f2

   // 0111 f1->f0, f2->f1, f3->f2, if->f3

   // 1100 if->f2

   // 1110 if->f3

   // 0010 if->f1, f2->f0

   // 0011 if->f2, f2->f0, f3->f1

   // 1111 !if, no shift

   assign shift_f1_f0 =  ~sf0_valid & sf1_valid;

   assign shift_f2_f0 =  ~sf0_valid & ~sf1_valid & f2_valid;

   assign shift_f3_f1 =  ~sf0_valid & ~sf1_valid & f2_valid & f3_valid;

   assign shift_f2_f1 =  ~sf0_valid & sf1_valid & f2_valid;

   assign shift_f3_f2 =  ~sf0_valid & sf1_valid & f2_valid & f3_valid;

   assign fetch_to_f0 =  ~sf0_valid & ~sf1_valid & ~f2_valid & ~f3_valid & ifvalid;

   assign fetch_to_f1 =  (~sf0_valid & ~sf1_valid &  f2_valid & ~f3_valid & ifvalid)  |
                         (~sf0_valid &  sf1_valid & ~f2_valid & ~f3_valid & ifvalid)  |
                         ( sf0_valid & ~sf1_valid & ~f2_valid & ~f3_valid & ifvalid);

   assign fetch_to_f2 =  (~sf0_valid &  sf1_valid &  f2_valid & ~f3_valid & ifvalid)  |
                         ( sf0_valid &  sf1_valid & ~f2_valid & ~f3_valid & ifvalid)  |
                         (~sf0_valid & ~sf1_valid &  f2_valid &  f3_valid & ifvalid);

   assign fetch_to_f3 =  (~sf0_valid &  sf1_valid &  f2_valid &  f3_valid & ifvalid) |
                         ( sf0_valid &  sf1_valid &  f2_valid & ~f3_valid & ifvalid);
   // valids

   assign f3_valid = f3val[0];

   assign f2_valid = f2val[0];

   assign sf1_valid = sf1val[0];

   assign sf0_valid = sf0val[0];

   // interface to fetch

   assign consume_fb0 = ~sf0val[0] & f0val[0];

   assign consume_fb1 = ~sf1val[0] & f1val[0];

   assign fb_consume1 = consume_fb0 & ~consume_fb1 & ~exu_flush_final;

   assign fb_consume2 = consume_fb0 &  consume_fb1 & ~exu_flush_final;

   assign ifvalid = fetch_val[0];



   // f0 valid states
   //
   // 11111111
   // 11111110
   // 11111100
   // 11111000
   // 11110000

   // 11100000
   // 11000000
   // 10000000
   // 00000000



   // make this two incrementors with some logic on the lower bits

   assign f0pc_plus1[31:1] = f0pc[31:1] + 31'd1;
   assign f0pc_plus2[31:1] = f0pc[31:1] + 31'd2;
   assign f0pc_plus3[31:1] = f0pc[31:1] + 31'd3;
   assign f0pc_plus4[31:1] = f0pc[31:1] + 31'd4;

   assign f1pc_plus1[31:1] = f1pc[31:1] + 31'd1;
   assign f1pc_plus2[31:1] = f1pc[31:1] + 31'd2;
   assign f1pc_plus3[31:1] = f1pc[31:1] + 31'd3;

   assign f3pc_in[31:1] = ifu_fetch_pc[31:1];

   rvdffe #(31) f3pcff (.*, .en(f3_wr_en), .din(f3pc_in[31:1]), .dout(f3pc[31:1]));

   assign f2pc_in[31:1] = ({31{fetch_to_f2}} & ifu_fetch_pc[31:1]) |
                          ({31{shift_f3_f2}} & f3pc[31:1]) |
                          ({31{~fetch_to_f2&~shift_f3_f2}} & f2pc[31:1]);

   rvdffe #(31) f2pcff (.*, .en(f2_wr_en), .din(f2pc_in[31:1]), .dout(f2pc[31:1]));

   assign sf1pc[31:1] = ({31{f1_shift_2B}} & (f1pc_plus1[31:1])) |
                        ({31{f1_shift_4B}} & (f1pc_plus2[31:1])) |
                        ({31{f1_shift_6B}} & (f1pc_plus3[31:1])) |
                        ({31{~f1_shift_2B&~f1_shift_4B&~f1_shift_6B}} & f1pc[31:1]);

   assign f1pc_in[31:1] = ({31{fetch_to_f1}} & ifu_fetch_pc[31:1]) |
                          ({31{shift_f2_f1}} & f2pc[31:1]) |
                          ({31{shift_f3_f1}} & f3pc[31:1]) |
                          ({31{~fetch_to_f1&~shift_f2_f1&~shift_f3_f1}} & sf1pc[31:1]);

   rvdffe #(31) f1pcff (.*, .en(f1_shift_wr_en), .din(f1pc_in[31:1]), .dout(f1pc[31:1]));

   assign sf0pc[31:1] = ({31{shift_2B}} & (f0pc_plus1[31:1])) |
                        ({31{shift_4B}} & (f0pc_plus2[31:1])) |
                        ({31{shift_6B}} & (f0pc_plus3[31:1])) |
                        ({31{shift_8B}} & (f0pc_plus4[31:1]));

   assign f0pc_in[31:1] = ({31{fetch_to_f0}} & ifu_fetch_pc[31:1]) |
                          ({31{shift_f2_f0}} & f2pc[31:1]) |
                          ({31{shift_f1_f0}} & sf1pc[31:1]) |
                          ({31{~fetch_to_f0&~shift_f2_f0&~shift_f1_f0}} & sf0pc[31:1]);

   rvdffe #(31) f0pcff (.*, .en(f0_shift_wr_en), .din(f0pc_in[31:1]), .dout(f0pc[31:1]));

   // on flush_final all valids go to 0

   // no clock-gating on the valids

   assign f3val_in[3:0] = (({4{fetch_to_f3}} & fetch_val[3:0]) |
                           ({4{~fetch_to_f3&~shift_f3_f1&~shift_f3_f2}} & f3val[3:0])) & ~{4{exu_flush_final}};

   rvdff #(4) f3valff (.*, .clk(active_clk), .din(f3val_in[3:0]), .dout(f3val[3:0]));

   assign f2val_in[3:0] = (({4{fetch_to_f2}} & fetch_val[3:0]) |
                           ({4{shift_f3_f2}} & f3val[3:0]) |
                           ({4{~fetch_to_f2&~shift_f3_f2&~shift_f2_f1&~shift_f2_f0}} & f2val[3:0])) & ~{4{exu_flush_final}};

   rvdff #(4) f2valff (.*, .clk(active_clk), .din(f2val_in[3:0]), .dout(f2val[3:0]));

   assign sf1val[3:0] = ({4{f1_shift_2B}} & {1'b0,f1val[3:1]}) |
                        ({4{f1_shift_4B}} & {2'b0,f1val[3:2]}) |
                        ({4{f1_shift_6B}} & {3'b0,f1val[3:3]}) |
                        ({4{~f1_shift_2B&~f1_shift_4B&~f1_shift_6B}} & f1val[3:0]);

   assign f1val_in[3:0] = (({4{fetch_to_f1}} & fetch_val[3:0]) |
                           ({4{shift_f3_f1}} & f3val[3:0]) |
                           ({4{shift_f2_f1}} & f2val[3:0]) |
                           ({4{~fetch_to_f1&~shift_f3_f1&~shift_f2_f1&~shift_f1_f0}} & sf1val[3:0])) & ~{4{exu_flush_final}};

   rvdff #(4) f1valff (.*, .clk(active_clk), .din(f1val_in[3:0]), .dout(f1val[3:0]));


   assign sf0val[3:0] = ({4{shift_2B}} & {1'b0,f0val[3:1]}) |
                        ({4{shift_4B}} & {2'b0,f0val[3:2]}) |
                        ({4{shift_6B}} & {3'b0,f0val[3:3]}) |
                        ({4{~shift_2B&~shift_4B&~shift_6B&~shift_8B}} & f0val[3:0]);

   assign f0val_in[3:0] = (({4{fetch_to_f0}} & fetch_val[3:0]) |
                           ({4{shift_f2_f0}} & f2val[3:0]) |
                           ({4{shift_f1_f0}} & sf1val[3:0]) |
                           ({4{~fetch_to_f0&~shift_f2_f0&~shift_f1_f0}} & sf0val[3:0])) & ~{4{exu_flush_final}};

   rvdff #(4) f0valff (.*, .clk(active_clk), .din(f0val_in[3:0]), .dout(f0val[3:0]));


   // fifo implementation of the fetch data for timing of predecodes

   assign f3data_in[63:0] = ifu_fetch_data[63:0];

   rvdffe #(64) f3dataff (.*, .en(f3_wr_en), .din(f3data_in[63:0]), .dout(f3data[63:0]));

   assign f2data_in[63:0] = (fetch_to_f2) ? ifu_fetch_data[63:0] : f3data[63:0];

   rvdffe #(64) f2dataff (.*, .en(f2_wr_en), .din(f2data_in[63:0]), .dout(f2data[63:0]));

   assign sf1data[63:0] = ({64{f1_shift_2B}} & {16'b0,f1data[63:16]}) |
                          ({64{f1_shift_4B}} & {32'b0,f1data[63:32]}) |
                          ({64{f1_shift_6B}} & {48'b0,f1data[63:48]}) |
                          ({64{~f1_shift_2B & ~f1_shift_4B & ~f1_shift_6B}} & f1data[63:0]);

   assign f1data_in[63:0] = (fetch_to_f1) ? ifu_fetch_data[63:0] : (shift_f2_f1) ? f2data[63:0] : (shift_f3_f1) ? f3data[63:0] : sf1data[63:0];

   rvdffe #(64) f1dataff (.*, .en(f1_shift_wr_en), .din(f1data_in[63:0]), .dout(f1data[63:0]));

   assign sf0data[63:0] = ({64{shift_2B}} & {16'b0,f0data[63:16]}) |
                          ({64{shift_4B}} & {32'b0,f0data[63:32]}) |
                          ({64{shift_6B}} & {48'b0,f0data[63:48]});


   assign f0data_in[63:0] = (fetch_to_f0) ? ifu_fetch_data[63:0] : (shift_f1_f0) ? sf1data[63:0] : (shift_f2_f0) ? f2data[63:0] : sf0data[63:0];

   rvdffe #(64) f0dataff (.*, .en(fetch_to_f0 | shift_f2_f0 | shift_f1_f0 | shift_2B | shift_4B | shift_6B), .din(f0data_in[63:0]), .dout(f0data[63:0]));


   assign aligndata[63:0] = ({64{(f0val[3])}} &                  {f0data[4*16-1:0]}) |
                            ({64{(f0val[2]&~f0val[3])}} &        {f1data[1*16-1:0],f0data[3*16-1:0]}) |
                            ({64{(f0val[1]&~f0val[2])}} &        {f1data[2*16-1:0],f0data[2*16-1:0]}) |
                            ({64{(f0val[0]&~f0val[1])}} &        {f1data[3*16-1:0],f0data[1*16-1:0]});

   assign alignval[3:0] =   ({4{(f0val[3])}} &                   4'b1111) |
                            ({4{(f0val[2]&~f0val[3])}} &        {f1val[0],3'b111}) |
                            ({4{(f0val[1]&~f0val[2])}} &        {f1val[1:0],2'b11}) |
                            ({4{(f0val[0]&~f0val[1])}} &        {f1val[2:0],1'b1});

   assign alignicaf[3:0] =   ({4{(f0val[3])}} &                  {4{f0icaf}}) |
                             ({4{(f0val[2]&~f0val[3])}} &        {{1{f1icaf}},{3{f0icaf}}}) |
                             ({4{(f0val[1]&~f0val[2])}} &        {{2{f1icaf}},{2{f0icaf}}}) |
                             ({4{(f0val[0]&~f0val[1])}} &        {{3{f1icaf}},{1{f0icaf}}});

   assign aligndbecc[3:0] =   ({4{(f0val[3])}} &                  {4{f0dbecc}}) |
                              ({4{(f0val[2]&~f0val[3])}} &        {{1{f1dbecc}},{3{f0dbecc}}}) |
                              ({4{(f0val[1]&~f0val[2])}} &        {{2{f1dbecc}},{2{f0dbecc}}}) |
                              ({4{(f0val[0]&~f0val[1])}} &        {{3{f1dbecc}},{1{f0dbecc}}});

   // for branch prediction
   assign alignbrend[3:0] =   ({4{(f0val[3])}} &                   f0brend[3:0]) |
                              ({4{(f0val[2]&~f0val[3])}} &        {f1brend[0],f0brend[2:0]}) |
                              ({4{(f0val[1]&~f0val[2])}} &        {f1brend[1:0],f0brend[1:0]}) |
                              ({4{(f0val[0]&~f0val[1])}} &        {f1brend[2:0],f0brend[0]});

   assign alignpc4[3:0] =   ({4{(f0val[3])}} &                   f0pc4[3:0]) |
                            ({4{(f0val[2]&~f0val[3])}} &        {f1pc4[0],f0pc4[2:0]}) |
                            ({4{(f0val[1]&~f0val[2])}} &        {f1pc4[1:0],f0pc4[1:0]}) |
                            ({4{(f0val[0]&~f0val[1])}} &        {f1pc4[2:0],f0pc4[0]});


   assign alignret[3:0] =   ({4{(f0val[3])}} &                   f0ret[3:0]) |
                            ({4{(f0val[2]&~f0val[3])}} &        {f1ret[0],f0ret[2:0]}) |
                            ({4{(f0val[1]&~f0val[2])}} &        {f1ret[1:0],f0ret[1:0]}) |
                            ({4{(f0val[0]&~f0val[1])}} &        {f1ret[2:0],f0ret[0]});

   assign alignway[3:0] =   ({4{(f0val[3])}} &                   f0way[3:0]) |
                            ({4{(f0val[2]&~f0val[3])}} &        {f1way[0],f0way[2:0]}) |
                            ({4{(f0val[1]&~f0val[2])}} &        {f1way[1:0],f0way[1:0]}) |
                            ({4{(f0val[0]&~f0val[1])}} &        {f1way[2:0],f0way[0]});

   assign alignhist1[3:0] =   ({4{(f0val[3])}} &                   f0hist1[3:0]) |
                              ({4{(f0val[2]&~f0val[3])}} &        {f1hist1[0],f0hist1[2:0]}) |
                              ({4{(f0val[1]&~f0val[2])}} &        {f1hist1[1:0],f0hist1[1:0]}) |
                              ({4{(f0val[0]&~f0val[1])}} &        {f1hist1[2:0],f0hist1[0]});

   assign alignhist0[3:0] =   ({4{(f0val[3])}} &                   f0hist0[3:0]) |
                              ({4{(f0val[2]&~f0val[3])}} &        {f1hist0[0],f0hist0[2:0]}) |
                              ({4{(f0val[1]&~f0val[2])}} &        {f1hist0[1:0],f0hist0[1:0]}) |
                              ({4{(f0val[0]&~f0val[1])}} &        {f1hist0[2:0],f0hist0[0]});

   assign alignfromf1[3:1] =     ({3{(f0val[3])}} &                   3'b0) |
                                 ({3{(f0val[2]&~f0val[3])}} &        {1'b1,2'b0}) |
                                 ({3{(f0val[1]&~f0val[2])}} &        {2'b11,1'b0}) |
                                 ({3{(f0val[0]&~f0val[1])}} &        {3'b111});


   assign { secondpc[31:1],
            thirdpc[31:1],
            fourthpc[31:1] } =   ({3*31{(f0val[3])}}           & {f0pc_plus1[31:1], f0pc_plus2[31:1], f0pc_plus3[31:1]}) |
                                 ({3*31{(f0val[2]&~f0val[3])}} & {f0pc_plus1[31:1], f0pc_plus2[31:1], f1pc[31:1]}) |
                                 ({3*31{(f0val[1]&~f0val[2])}} & {f0pc_plus1[31:1], f1pc[31:1],       f1pc_plus1[31:1]})   |
                                 ({3*31{(f0val[0]&~f0val[1])}} & {f1pc[31:1],       f1pc_plus1[31:1], f1pc_plus2[31:1]});


   assign i0_pc[31:1] = f0pc[31:1];

   assign firstpc[31:1] = f0pc[31:1];

   assign i1_pc[31:1] = (first2B) ? secondpc[31:1] : thirdpc[31:1];


   assign i0_pc4 = first4B;

   assign i1_pc4 = (first2B & second4B) |
                   (first4B & third4B);



   //logic for trace
   assign i0_cinst[15:0] = aligndata[15:0];
   assign i1_cinst[15:0] = (first4B) ? aligndata[47:32] : aligndata[31:16];
   // end trace


   // check on 16B boundaries
   //
   assign first4B = aligndata[16*0+1:16*0] == 2'b11;
   assign first2B = ~first4B;

   assign second4B = aligndata[16*1+1:16*1] == 2'b11;
   assign second2B = ~second4B;

   assign third4B = aligndata[16*2+1:16*2] == 2'b11;
   assign third2B = ~third4B;


   assign i0_valid = ((first4B & alignval[1]) |
                      (first2B & alignval[0])) & ~exu_flush_final;


   assign i1_valid = ((first4B & third4B & alignval[3])  |
                      (first4B & third2B & alignval[2])  |
                      (first2B & second4B & alignval[2]) |
                      (first2B & second2B & alignval[1])) & ~exu_flush_final & ~i1_icaf & ~i1_dbecc;

   // inst access fault on any byte of inst results in access fault for the inst
   assign i0_icaf = ((first4B & (|alignicaf[1:0])) |
                     (first2B &   alignicaf[0])) & ~exu_flush_final;

   // restrict icaf and dbecc to be i0 only
   assign i0_icaf_type[1:0] = (first4B & ~f0val[1] & f0val[0] & ~alignicaf[0] & ~aligndbecc[0]) ? f1ictype[1:0] : f0ictype[1:0];

   assign icaf_eff[3:1] = alignicaf[3:1] | aligndbecc[3:1];

   assign i0_icaf_f1 = first4B & icaf_eff[1] & alignfromf1[1];


   assign i1_icaf = ((first4B & third4B &  (|alignicaf[3:2])) |
                     (first4B & third2B &    alignicaf[2])    |
                     (first2B & second4B & (|alignicaf[2:1])) |
                     (first2B & second2B &   alignicaf[1])) & ~exu_flush_final;


   assign i0_dbecc = ((first4B & (|aligndbecc[1:0])) |
                      (first2B &   aligndbecc[0])) & ~exu_flush_final;

   assign i1_dbecc = ((first4B & third4B &  (|aligndbecc[3:2])) |
                      (first4B & third2B &    aligndbecc[2])    |
                      (first2B & second4B & (|aligndbecc[2:1])) |
                      (first2B & second2B &   aligndbecc[1])) & ~exu_flush_final;



   // big endian 4B instructions

   assign ifirst[31:0] =  aligndata[2*16-1:0*16];

   assign isecond[31:0] = aligndata[3*16-1:1*16];

   assign ithird[31:0] =  aligndata[4*16-1:2*16];



   assign i0_instr[31:0] = ({32{first4B}} & ifirst[31:0]) |
                           ({32{first2B}} & uncompress0[31:0]);


   assign i1_instr[31:0] = ({32{first4B & third4B}} & ithird[31:0]) |
                           ({32{first4B & third2B}} & uncompress2[31:0]) |
                           ({32{first2B & second4B}} & isecond[31:0]) |
                           ({32{first2B & second2B}} & uncompress1[31:0]);


// file "decode" is human readable file that has all of the instruction decodes defined and is part of git repo
// modify this file as needed

// to generate the equations below from "decode"

// 1) coredecode -in decode > coredecode.e

// 2) change '.type fr' to '.type fd'   ---> type fd is a full decode among off/on set

// 3) espresso -Dso -oeqntott coredecode.e | addassign  > equations



   assign i0[31:0] = i0_instr[31:0];
   assign i1[31:0] = i1_instr[31:0];

   eh2_ifu_predecode_ctl i0_pred (.inst(i0[31:0]),.predecode(i0_predecode));
   eh2_ifu_predecode_ctl i1_pred (.inst(i1[31:0]),.predecode(i1_predecode));



   // if you detect br does not start on instruction boundary

   eh2_btb_addr_hash #(.pt(pt)) firsthash(.pc(firstpc[pt.BTB_INDEX3_HI:pt.BTB_INDEX1_LO]), .hash(firstpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]));
   eh2_btb_addr_hash #(.pt(pt)) secondhash(.pc(secondpc[pt.BTB_INDEX3_HI:pt.BTB_INDEX1_LO]), .hash(secondpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]));
   eh2_btb_addr_hash #(.pt(pt)) thirdhash(.pc(thirdpc[pt.BTB_INDEX3_HI:pt.BTB_INDEX1_LO]), .hash(thirdpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]));
   eh2_btb_addr_hash #(.pt(pt)) fourthhash(.pc(fourthpc[pt.BTB_INDEX3_HI:pt.BTB_INDEX1_LO]), .hash(fourthpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]));


if(pt.BTB_BTAG_FOLD) begin : btbfold
   eh2_btb_tag_hash_fold #(.pt(pt)) first_brhash (.pc(firstpc [pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]), .hash(firstbrtag_hash[pt.BTB_BTAG_SIZE-1:0]));
   eh2_btb_tag_hash_fold #(.pt(pt)) second_brhash(.pc(secondpc[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]), .hash(secondbrtag_hash[pt.BTB_BTAG_SIZE-1:0]));
   eh2_btb_tag_hash_fold #(.pt(pt)) third_brhash (.pc(thirdpc [pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]), .hash(thirdbrtag_hash[pt.BTB_BTAG_SIZE-1:0]));
   eh2_btb_tag_hash_fold #(.pt(pt)) fourth_brhash(.pc(fourthpc[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]), .hash(fourthbrtag_hash[pt.BTB_BTAG_SIZE-1:0]));
end
   else begin
   eh2_btb_tag_hash #(.pt(pt)) first_brhash (.pc(firstpc [pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]), .hash(firstbrtag_hash[pt.BTB_BTAG_SIZE-1:0]));
   eh2_btb_tag_hash #(.pt(pt)) second_brhash(.pc(secondpc[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]), .hash(secondbrtag_hash[pt.BTB_BTAG_SIZE-1:0]));
   eh2_btb_tag_hash #(.pt(pt)) third_brhash (.pc(thirdpc [pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]), .hash(thirdbrtag_hash[pt.BTB_BTAG_SIZE-1:0]));
   eh2_btb_tag_hash #(.pt(pt)) fourth_brhash(.pc(fourthpc[pt.BTB_ADDR_HI+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE+pt.BTB_BTAG_SIZE:pt.BTB_ADDR_HI+1]), .hash(fourthbrtag_hash[pt.BTB_BTAG_SIZE-1:0]));
end

   // start_indexing - you want pc to be based on where the end of branch is prediction
   // normal indexing pc based that's incorrect now for pc4 cases it's pc4 + 2

   always_comb begin

      i0_br_p = '0;

      i0_br_start_error = (first4B & alignval[1] & alignbrend[0]);

      i0_br_p.valid = (first2B & alignbrend[0]) |
                      (first4B & alignbrend[1]) |
                      i0_br_start_error;

      i0_brp_pc4 = (first2B & alignpc4[0]) |
                   (first4B & alignpc4[1]);

      i0_br_p.ret = (first2B & alignret[0]) |
                    (first4B & alignret[1]);


      i0_br_p.way = (first2B | alignbrend[0]) ? alignway[0] : alignway[1];

      i0_br_p.hist[1] = (first2B & alignhist1[0]) |
                        (first4B & alignhist1[1]);

      i0_br_p.hist[0] = (first2B & alignhist0[0]) |
                        (first4B & alignhist0[1]);

      i0_ends_f1 = (first4B & alignfromf1[1]);

      i0_br_p.toffset[11:0] = (i0_ends_f1) ? f1poffset[11:0] : f0poffset[11:0];

      i0_br_p.prett[31:1] = (i0_ends_f1) ? f1prett[31:1] : f0prett[31:1];

      i0_br_p.br_start_error = i0_br_start_error;


      i0_br_p.bank = (first2B | alignbrend[0]) ? firstpc[2] :
                                               secondpc[2];


      i0_br_p.br_error = (i0_br_p.valid &  i0_brp_pc4 &  first2B) |
                         (i0_br_p.valid & ~i0_brp_pc4 &  first4B);

      i1_br_p = '0;

      i1_br_start_error = (first2B & second4B & alignval[2] & alignbrend[1]) |
                          (first4B & third4B  & alignval[3] & alignbrend[2]);

      i1_br_p.valid = (first4B & third2B & alignbrend[2]) |
                      (first4B & third4B & alignbrend[3]) |
                      (first2B & second2B & alignbrend[1]) |
                      (first2B & second4B & alignbrend[2]) |
                      i1_br_start_error;

      i1_brp_pc4 = (first4B & third2B & alignpc4[2]) |
                   (first4B & third4B & alignpc4[3]) |
                   (first2B & second2B & alignpc4[1]) |
                   (first2B & second4B & alignpc4[2]);

      i1_br_p.ret = (first4B & third2B & alignret[2]) |
                    (first4B & third4B & alignret[3]) |
                    (first2B & second2B & alignret[1]) |
                    (first2B & second4B & alignret[2]);

      i1_br_p.way = (first4B & third2B                   & alignway[2] ) |
                    (first4B & third4B &  alignbrend[2]  & alignway[2] ) |
                    (first4B & third4B & ~alignbrend[2]  & alignway[3] ) |
                    (first2B & second2B                  & alignway[1] ) |
                    (first2B & second4B &  alignbrend[1] & alignway[1] ) |
                    (first2B & second4B & ~alignbrend[1] & alignway[2] );

      i1_br_p.hist[1] = (first4B & third2B & alignhist1[2]) |
                        (first4B & third4B & alignhist1[3]) |
                        (first2B & second2B & alignhist1[1]) |
                        (first2B & second4B & alignhist1[2]);

      i1_br_p.hist[0] = (first4B & third2B & alignhist0[2]) |
                        (first4B & third4B & alignhist0[3]) |
                        (first2B & second2B & alignhist0[1]) |
                        (first2B & second4B & alignhist0[2]);

      i1_ends_f1 = (first4B & third2B & alignfromf1[2]) |
                   (first4B & third4B & alignfromf1[3]) |
                   (first2B & second2B & alignfromf1[1]) |
                   (first2B & second4B & alignfromf1[2]);

      i1_br_p.toffset[11:0] = (i1_ends_f1) ? f1poffset[11:0] : f0poffset[11:0];

      i1_br_p.prett[31:1] = (i1_ends_f1) ? f1prett[31:1] : f0prett[31:1];

      i1_br_p.br_start_error = i1_br_start_error;


      i1_br_p.bank = ({{first4B & third2B }}                  & thirdpc[2] ) |
                     ({{first4B & third4B &  alignbrend[2] }} & thirdpc[2] ) |
                     ({{first4B & third4B & ~alignbrend[2] }} & fourthpc[2] ) |
                     ({{first2B & second2B}}                  & secondpc[2] ) |
                     ({{first2B & second4B &  alignbrend[1]}} & secondpc[2] ) |
                     ({{first2B & second4B & ~alignbrend[1]}} & thirdpc[2] );

      i1_br_p.br_error = (i1_br_p.valid &  i1_brp_pc4 & first4B & third2B ) |
                         (i1_br_p.valid & ~i1_brp_pc4 & first4B & third4B ) |
                         (i1_br_p.valid &  i1_brp_pc4 & first2B & second2B) |
                         (i1_br_p.valid & ~i1_brp_pc4 & first2B & second4B);
   end

   assign i0_bp_fghr[pt.BHT_GHR_SIZE-1:0] = (i0_ends_f1) ? f1fghr[pt.BHT_GHR_SIZE-1:0] : f0fghr[pt.BHT_GHR_SIZE-1:0];

   assign i0_bp_index[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] = (first2B | alignbrend[0]) ? firstpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]:
                                                                                  secondpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO];

   assign i0_bp_btag[pt.BTB_BTAG_SIZE-1:0] = (first2B | alignbrend[0]) ? firstbrtag_hash[pt.BTB_BTAG_SIZE-1:0]:
                                                                       secondbrtag_hash[pt.BTB_BTAG_SIZE-1:0];

   assign i1_bp_fghr[pt.BHT_GHR_SIZE-1:0] = (i1_ends_f1) ? f1fghr[pt.BHT_GHR_SIZE-1:0] : f0fghr[pt.BHT_GHR_SIZE-1:0];

   assign i1_bp_index[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] = ({pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1{first4B & third2B }}                  & thirdpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ) |
                                                    ({pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1{first4B & third4B &  alignbrend[2] }} & thirdpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ) |
                                                    ({pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1{first4B & third4B & ~alignbrend[2] }} & fourthpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ) |
                                                    ({pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1{first2B & second2B}}                  & secondpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ) |
                                                    ({pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1{first2B & second4B &  alignbrend[1]}} & secondpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] ) |
                                                    ({pt.BTB_ADDR_HI-pt.BTB_ADDR_LO+1{first2B & second4B & ~alignbrend[1]}} & thirdpc_hash[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] );

   assign i1_bp_btag[pt.BTB_BTAG_SIZE-1:0] = ({pt.BTB_BTAG_SIZE{first4B & third2B }}                  &  thirdbrtag_hash[pt.BTB_BTAG_SIZE-1:0] ) |
                                           ({pt.BTB_BTAG_SIZE{first4B & third4B &  alignbrend[2] }} &  thirdbrtag_hash[pt.BTB_BTAG_SIZE-1:0] ) |
                                           ({pt.BTB_BTAG_SIZE{first4B & third4B & ~alignbrend[2] }} & fourthbrtag_hash[pt.BTB_BTAG_SIZE-1:0] ) |
                                           ({pt.BTB_BTAG_SIZE{first2B & second2B}}                  & secondbrtag_hash[pt.BTB_BTAG_SIZE-1:0] ) |
                                           ({pt.BTB_BTAG_SIZE{first2B & second4B &  alignbrend[1]}} & secondbrtag_hash[pt.BTB_BTAG_SIZE-1:0] ) |
                                           ({pt.BTB_BTAG_SIZE{first2B & second4B & ~alignbrend[1]}} &  thirdbrtag_hash[pt.BTB_BTAG_SIZE-1:0] );


   // decompress

   eh2_ifu_compress_ctl compress0 (.din(aligndata[16*1-1:0*16]), .dout(uncompress0[31:0]) );

   eh2_ifu_compress_ctl compress1 (.din(aligndata[16*2-1:1*16]), .dout(uncompress1[31:0]) );

   eh2_ifu_compress_ctl compress2 (.din(aligndata[16*3-1:2*16]), .dout(uncompress2[31:0]) );


`ifdef ASSERT_ON
   assert_valid_consistency: assert #0 (~( i1_valid & ~i0_valid ) );
   assert_shift_consistency: assert #0 (~( i1_shift & ~i0_shift ) );
`endif

   assign i0_shift = i0_valid & ibuffer_room1_more & ~error_stall;  // & (ifu_aln_tid == tid);

   assign i1_shift = i1_valid & ibuffer_room2_more & ~error_stall;  // & (ifu_aln_tid == tid);

   assign ibuffer_room1_more = ~dec_ib3_valid_d & ~dec_i1_cancel_e1;
   assign ibuffer_room2_more = ~dec_ib2_valid_d & ~dec_i1_cancel_e1;

   assign pmu_instr_aligned[1:0] = { i1_shift, i0_shift };

   assign pmu_align_stall = i0_valid & ~ibuffer_room1_more;

   // compute how many bytes are being shifted from f0

   assign shift_2B = i0_shift & ~i1_shift & first2B;


   assign shift_4B = (i0_shift & ~i1_shift & first4B) |
                     (i0_shift &  i1_shift & first2B & second2B);

   assign shift_6B = (i0_shift &  i1_shift & first2B & second4B) |
                     (i0_shift &  i1_shift & first4B & third2B);

   assign shift_8B = i0_shift &  i1_shift & first4B & third4B;

   // exact equations for the queue logic
   assign f0_shift_2B = (shift_2B & f0val[0]) |
                        ((shift_4B | shift_6B | shift_8B) & f0val[0] & ~f0val[1]);

   assign f0_shift_4B = (shift_4B & f0val[1]) |
                        ((shift_6B & shift_8B) & f0val[1] & ~f0val[2]);


   assign f0_shift_6B = (shift_6B & f0val[2]) |
                        (shift_8B & f0val[2] & ~f0val[3]);

   //assign f0_shift_8B =  shift_8B & f0val[3];



   // f0 valid states
   //
   // 11111111
   // 11111110
   // 11111100
   // 11111000
   // 11110000

   // 11100000
   // 11000000
   // 10000000
   // 00000000

   // assign f1_shift_0B = shift_0B;

   assign f1_shift_2B = (f0val[2] & ~f0val[3] & shift_8B) |
                        (f0val[1] & ~f0val[2] & shift_6B) |
                        (f0val[0] & ~f0val[1] & shift_4B);

   assign f1_shift_4B = (f0val[1] & ~f0val[2] & shift_8B) |
                        (f0val[0] & ~f0val[1] & shift_6B);

   assign f1_shift_6B = (f0val[0] & ~f0val[1] & shift_8B);



endmodule

module eh2_ifu_predecode_ctl
import eh2_pkg::*;
  (
   input logic [31:0] inst,

   output eh2_predecode_pkt_t predecode
   );

   logic [31:0] i;

   assign i[31:0] = inst[31:0];



// full decode
assign predecode.lsu = (!i[31]&!i[30]&!i[29]&!i[24]&!i[23]&!i[22]&!i[21]&!i[20]&!i[14]
    &i[13]&!i[12]&!i[6]&i[5]&!i[4]&i[3]&i[2]&i[1]&i[0]) | (!i[31]&!i[30]
    &!i[29]&i[27]&!i[14]&i[13]&!i[12]&!i[6]&i[5]&!i[4]&i[3]&i[2]&i[1]
    &i[0]) | (!i[28]&!i[27]&!i[14]&i[13]&!i[12]&!i[6]&i[5]&!i[4]&i[3]
    &i[2]&i[1]&i[0]) | (!i[13]&!i[6]&!i[5]&!i[4]&!i[3]&!i[2]&i[1]&i[0]) | (
    !i[14]&!i[13]&!i[6]&!i[4]&!i[3]&!i[2]&i[1]&i[0]) | (!i[14]&!i[12]
    &!i[6]&!i[4]&!i[3]&!i[2]&i[1]&i[0]);

assign predecode.mul = (!i[31]&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&i[25]&!i[14]&!i[6]&i[5]
    &i[4]&!i[3]&!i[2]&i[1]&i[0]);

// split the legal equation in 4 more or less equal parts
assign predecode.legal1 = (!i[31]&!i[30]&i[29]&i[28]&!i[27]&!i[26]&!i[25]&!i[24]&!i[23]
    &!i[22]&i[21]&!i[20]&!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[11]
    &!i[10]&!i[9]&!i[8]&!i[7]&i[6]&i[5]&i[4]&!i[3]&!i[2]&i[1]&i[0]) | (
    !i[31]&!i[30]&!i[29]&i[28]&!i[27]&!i[26]&!i[25]&!i[24]&!i[23]&i[22]
    &!i[21]&i[20]&!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[11]&!i[10]
    &!i[9]&!i[8]&!i[7]&i[6]&i[5]&i[4]&!i[3]&!i[2]&i[1]&i[0]) | (!i[31]
    &!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&!i[24]&!i[23]&!i[22]&!i[21]
    &!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[11]&!i[10]&!i[9]&!i[8]
    &!i[7]&i[5]&i[4]&!i[3]&!i[2]&i[1]&i[0]);

assign predecode.legal2 = (!i[31]&!i[30]&!i[29]&!i[28]
    &!i[27]&!i[26]&!i[25]&!i[6]&i[4]&!i[3]&i[1]&i[0]) | (!i[31]&!i[29]
    &!i[28]&!i[27]&!i[26]&!i[25]&!i[14]&!i[13]&!i[12]&!i[6]&!i[3]&!i[2]
    &i[1]&i[0]) | (!i[31]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&i[14]&!i[13]
    &i[12]&!i[6]&i[4]&!i[3]&i[1]&i[0]) | (!i[31]&!i[30]&!i[29]&!i[28]
    &!i[27]&!i[26]&!i[6]&i[5]&i[4]&!i[3]&i[1]&i[0]) | (!i[14]&!i[13]
    &!i[12]&i[6]&i[5]&!i[4]&!i[3]&i[1]&i[0]) | (i[14]&i[6]&i[5]&!i[4]
    &!i[3]&!i[2]&i[1]&i[0]);

assign predecode.legal3 =  (!i[12]&!i[6]&!i[5]&i[4]&!i[3]&i[1]&i[0]) | (
    !i[14]&!i[13]&i[5]&!i[4]&!i[3]&!i[2]&i[1]&i[0]) | (i[12]&i[6]&i[5]
    &i[4]&!i[3]&!i[2]&i[1]&i[0]) | (!i[31]&!i[30]&!i[29]&!i[28]&!i[27]
    &!i[26]&!i[25]&!i[24]&!i[23]&!i[22]&!i[21]&!i[20]&!i[19]&!i[18]&!i[17]
    &!i[16]&!i[15]&!i[14]&!i[13]&!i[11]&!i[10]&!i[9]&!i[8]&!i[7]&!i[6]
    &!i[5]&!i[4]&i[3]&i[2]&i[1]&i[0]) | (!i[31]&!i[30]&!i[29]&!i[28]
    &!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[13]&!i[12]&!i[11]&!i[10]
    &!i[9]&!i[8]&!i[7]&!i[6]&!i[5]&!i[4]&i[3]&i[2]&i[1]&i[0]);

assign predecode.legal4 =  (!i[31]
    &!i[30]&!i[29]&!i[24]&!i[23]&!i[22]&!i[21]&!i[20]&!i[14]&i[13]&!i[12]
    &i[5]&!i[4]&i[3]&i[2]&i[1]&i[0]) | (!i[28]&!i[27]&!i[14]&i[13]&!i[12]
    &i[5]&!i[4]&i[3]&i[2]&i[1]&i[0]) | (!i[31]&!i[30]&!i[29]&i[27]&!i[14]
    &i[13]&!i[12]&i[5]&!i[4]&i[3]&i[2]&i[1]&i[0]) | (i[13]&i[6]&i[5]&i[4]
    &!i[3]&!i[2]&i[1]&i[0]) | (!i[13]&!i[6]&!i[5]&!i[4]&!i[3]&!i[2]&i[1]
    &i[0]) | (i[13]&!i[6]&!i[5]&i[4]&!i[3]&i[1]&i[0]) | (!i[14]&!i[12]
    &!i[6]&!i[4]&!i[3]&!i[2]&i[1]&i[0]) | (i[6]&i[5]&!i[4]&i[3]&i[2]&i[1]
    &i[0]) | (!i[6]&i[4]&!i[3]&i[2]&i[1]&i[0]);


assign predecode.i0_only = (!i[31]&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&!i[24]&!i[23]
    &!i[22]&!i[21]&!i[20]&!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[13]
    &!i[11]&!i[10]&!i[9]&!i[8]&!i[7]&!i[6]&!i[5]&!i[4]&i[3]&i[2]&i[1]
    &i[0]) | (!i[31]&!i[30]&i[29]&i[28]&!i[27]&!i[26]&!i[25]&!i[24]&!i[23]
    &!i[22]&i[21]&!i[20]&!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[11]
    &!i[10]&!i[9]&!i[8]&!i[7]&i[6]&i[5]&i[4]&!i[3]&!i[2]&i[1]&i[0]) | (
    !i[31]&!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&!i[25]&!i[24]&!i[23]&!i[22]
    &!i[21]&!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[11]&!i[10]&!i[9]
    &!i[8]&!i[7]&i[6]&i[5]&i[4]&!i[3]&!i[2]&i[1]&i[0]) | (!i[31]&!i[30]
    &!i[29]&!i[28]&!i[19]&!i[18]&!i[17]&!i[16]&!i[15]&!i[14]&!i[13]&!i[12]
    &!i[11]&!i[10]&!i[9]&!i[8]&!i[7]&!i[6]&!i[5]&!i[4]&i[3]&i[2]&i[1]
    &i[0]) | (!i[31]&!i[30]&!i[29]&!i[24]&!i[23]&!i[22]&!i[21]&!i[20]
    &!i[14]&i[13]&!i[12]&!i[6]&i[5]&!i[4]&i[3]&i[2]&i[1]&i[0]) | (!i[31]
    &!i[30]&!i[29]&!i[28]&!i[27]&!i[26]&i[25]&i[14]&!i[6]&i[5]&i[4]&!i[3]
    &!i[2]&i[1]&i[0]) | (!i[28]&!i[27]&!i[14]&i[13]&!i[12]&!i[6]&i[5]
    &!i[4]&i[3]&i[2]&i[1]&i[0]) | (!i[31]&!i[30]&!i[29]&i[27]&!i[14]
    &i[13]&!i[12]&!i[6]&i[5]&!i[4]&i[3]&i[2]&i[1]&i[0]) | (i[12]&i[6]
    &i[5]&i[4]&!i[3]&!i[2]&i[1]&i[0]) | (i[13]&i[6]&i[5]&i[4]&!i[3]&!i[2]
    &i[1]&i[0]);


endmodule