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
// $Id$
//
//
// Owner:
// Function: Checks the memory map for the address
// Comments:
//
//********************************************************************************
module eh2_lsu_amo
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)(

   input eh2_lsu_pkt_t          lsu_pkt_dc3,                 // packet in dc3
   input logic                  addr_in_pic_dc3,
   input logic  [pt.XLEN-1:0]   lsu_dccm_data_corr_dc3,      // Operand 1 for the ALU
   input logic  [pt.XLEN-1:0]   store_data_dc3,              // Store_Data Operand

   output logic [pt.XLEN-1:0]   amo_data_dc3                 // Final AMO result to go down the store path

);

// this section does the decode of the type of AMO in dc3.

   logic               amo_sc_dc3;


   logic                amo_add_dc3;
   logic                amo_max_dc3;
   logic                amo_maxu_dc3;
   logic                amo_min_dc3;
   logic                amo_minu_dc3;
   logic                amo_minmax_sel_dc3;
   logic [pt.XLEN-1:0]  amo_minmax_dc3;
   logic                amo_xor_dc3;
   logic                amo_or_dc3;
   logic                amo_and_dc3;
   logic                amo_swap_dc3;

   logic                logic_sel;
   logic [pt.XLEN-1:0]  logical_out;
   logic [pt.XLEN-1:0]  sum_out;

   logic [pt.XLEN-1:0]  store_datafn_dc3;

   logic [pt.XLEN-1:0]  amo_operand1, amo_operand2;
   logic [pt.XLEN-1:0]  amo_operand2_inv;

   //------------------------------------------------------------------------------------------------------------
   //----------------------------------------Logic starts here---------------------------------------------------
   //------------------------------------------------------------------------------------------------------------

   // decode the instruction type
   assign amo_sc_dc3     = lsu_pkt_dc3.valid & (lsu_pkt_dc3.atomic | lsu_pkt_dc3.atomic64) & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd3);

   assign amo_add_dc3    = lsu_pkt_dc3.valid & (lsu_pkt_dc3.atomic | lsu_pkt_dc3.atomic64) & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd0);
   assign amo_max_dc3    = lsu_pkt_dc3.valid & (lsu_pkt_dc3.atomic | lsu_pkt_dc3.atomic64) & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd20);
   assign amo_maxu_dc3   = lsu_pkt_dc3.valid & (lsu_pkt_dc3.atomic | lsu_pkt_dc3.atomic64) & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd28);
   assign amo_min_dc3    = lsu_pkt_dc3.valid & (lsu_pkt_dc3.atomic | lsu_pkt_dc3.atomic64) & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd16);
   assign amo_minu_dc3   = lsu_pkt_dc3.valid & (lsu_pkt_dc3.atomic | lsu_pkt_dc3.atomic64) & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd24);
   assign amo_xor_dc3    = lsu_pkt_dc3.valid & (lsu_pkt_dc3.atomic | lsu_pkt_dc3.atomic64) & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd4);
   assign amo_or_dc3     = lsu_pkt_dc3.valid & (lsu_pkt_dc3.atomic | lsu_pkt_dc3.atomic64) & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd8);
   assign amo_and_dc3    = lsu_pkt_dc3.valid & (lsu_pkt_dc3.atomic | lsu_pkt_dc3.atomic64) & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd12);
   assign amo_swap_dc3   = lsu_pkt_dc3.valid & (lsu_pkt_dc3.atomic | lsu_pkt_dc3.atomic64) & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd1);

   assign amo_minmax_sel_dc3 =  amo_max_dc3 | amo_maxu_dc3 | amo_min_dc3 | amo_minu_dc3;
   assign logic_sel          =  amo_and_dc3 | amo_or_dc3   | amo_xor_dc3;


   if (pt.XLEN == 32) begin
      assign amo_operand1[31:0] = {32{lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic}} & lsu_dccm_data_corr_dc3[31:0];
      assign amo_operand2[31:0] = {32{lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic}} & store_data_dc3[31:0];
   end else if (pt.XLEN == 64) begin
      assign amo_operand1[63:0] = ({{32{1'b0}}, {32{lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic}}   & lsu_dccm_data_corr_dc3[31:0]}) |
                                  (             {64{lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic64}} & lsu_dccm_data_corr_dc3[63:0]);
      assign amo_operand2[63:0] = ({{32{1'b0}}, {32{lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic}}   & store_data_dc3[31:0]}) |
                                  (             {64{lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic64}} & store_data_dc3[63:0]);
   end


   // logical
   if (pt.XLEN == 32) begin
      assign logical_out[31:0] = ( {32{amo_and_dc3}} & (amo_operand1[31:0] & amo_operand2[31:0]) ) |
                                 ( {32{amo_or_dc3}}  & (amo_operand1[31:0] | amo_operand2[31:0]) ) |
                                 ( {32{amo_xor_dc3}} & (amo_operand1[31:0] ^ amo_operand2[31:0]) );
   end else if (pt.XLEN == 64) begin
      logic [63:0] amo_and_result, amo_or_result, amo_xor_result;

      assign amo_and_result    = amo_operand1 & amo_operand2;
      assign amo_or_result     = amo_operand1 | amo_operand2;
      assign amo_xor_result    = amo_operand1 ^ amo_operand2;
      assign logical_out[63:0] = ( {64{amo_and_dc3}} & (lsu_pkt_dc3.atomic ? ({{32{amo_and_result[31]}}, amo_and_result[31:0]}) : amo_and_result[63:0]) ) |
                                 ( {64{amo_or_dc3}}  & (lsu_pkt_dc3.atomic ? ({{32{amo_or_result[31]}},  amo_or_result[31:0]})  : amo_or_result[63:0])  ) |
                                 ( {64{amo_xor_dc3}} & (lsu_pkt_dc3.atomic ? ({{32{amo_xor_result[31]}}, amo_xor_result[31:0]}) : amo_xor_result[63:0]) );
   end

   // adder
   logic         lsu_result_lt_storedata;
   logic         cout;

   // ADD
   assign amo_operand2_inv[pt.XLEN-1:0]  =  amo_add_dc3 ? amo_operand2[pt.XLEN-1:0] : ~amo_operand2[pt.XLEN-1:0];
   if (pt.XLEN == 32) begin
      assign {cout, sum_out[31:0]} = ({1'b0, amo_operand1[31:0]} + {1'b0, amo_operand2_inv[31:0]} + {{32{1'b0}}, ~amo_add_dc3});
   end else if (pt.XLEN == 64) begin
      logic [64:0] amo_add_result;

      assign amo_add_result = {1'b0, amo_operand1[63:0]} + {1'b0, amo_operand2_inv[63:0]} + {{64{1'b0}}, ~amo_add_dc3};
      assign {cout, sum_out[63:0]} = lsu_pkt_dc3.atomic ? {amo_add_result[32], {32{amo_add_result[31]}}, amo_add_result[31:0]} : amo_add_result;
   end


   // Min/Max/Minu/Maxu
   if (pt.XLEN == 32) begin
      assign lsu_result_lt_storedata = (~cout & (lsu_pkt_dc3.unsign | ~(amo_operand1[31] ^amo_operand2[31]))) |    // either doing unsigned math or signed with same polarity
                                       (amo_operand1[31] & ~amo_operand2[31] & ~lsu_pkt_dc3.unsign);

      assign amo_minmax_dc3[31:0] = ({32{(amo_max_dc3 | amo_maxu_dc3) &  lsu_result_lt_storedata}}  & amo_operand2[31:0]) |  // MAX if store_data >  result
                                    ({32{(amo_max_dc3 | amo_maxu_dc3) & ~lsu_result_lt_storedata}}  & amo_operand1[31:0]) |  // MAX if store_data <= result
                                    ({32{(amo_min_dc3 | amo_minu_dc3) & ~lsu_result_lt_storedata}}  & amo_operand2[31:0]) |  // MIN if store_data >  result
                                    ({32{(amo_min_dc3 | amo_minu_dc3) &  lsu_result_lt_storedata}}  & amo_operand1[31:0]);   // MIN if store_data <= result
   end else if (pt.XLEN == 64) begin
      logic lsu_result_lt_storedata32, lsu_result_lt_storedata64;
      assign lsu_result_lt_storedata32 = (~cout & (lsu_pkt_dc3.unsign | ~(amo_operand1[31] ^amo_operand2[31]))) |    // either doing unsigned math or signed with same polarity
                                         (amo_operand1[31] & ~amo_operand2[31] & ~lsu_pkt_dc3.unsign);
      assign lsu_result_lt_storedata64 = (~cout & (lsu_pkt_dc3.unsign | ~(amo_operand1[63] ^amo_operand2[63]))) |    // either doing unsigned math or signed with same polarity
                                         (amo_operand1[63] & ~amo_operand2[63] & ~lsu_pkt_dc3.unsign);
      assign lsu_result_lt_storedata = lsu_pkt_dc3.atomic ? lsu_result_lt_storedata32 : lsu_result_lt_storedata64;

      assign amo_minmax_dc3[63:0] = ({64{(amo_max_dc3 | amo_maxu_dc3) &  lsu_result_lt_storedata}}  & amo_operand2[63:0]) |  // MAX if store_data >  result
                                    ({64{(amo_max_dc3 | amo_maxu_dc3) & ~lsu_result_lt_storedata}}  & amo_operand1[63:0]) |  // MAX if store_data <= result
                                    ({64{(amo_min_dc3 | amo_minu_dc3) & ~lsu_result_lt_storedata}}  & amo_operand2[63:0]) |  // MIN if store_data >  result
                                    ({64{(amo_min_dc3 | amo_minu_dc3) &  lsu_result_lt_storedata}}  & amo_operand1[63:0]);   // MIN if store_data <= result
   end

  // final result
   assign amo_data_dc3[pt.XLEN-1:0] = ({pt.XLEN{logic_sel}}                 & logical_out[pt.XLEN-1:0])    |  // for the AND/OR/XOR
                                      ({pt.XLEN{amo_add_dc3}}               & sum_out[pt.XLEN-1:0])        |  // for ADD
                                      ({pt.XLEN{amo_minmax_sel_dc3}}        & amo_minmax_dc3[pt.XLEN-1:0]) |  // for Min/Max/Minu/Maxu
                                      ({pt.XLEN{amo_swap_dc3 | amo_sc_dc3}} & amo_operand2[pt.XLEN-1:0]);     // for SWAP need to store the store data value to the location

endmodule // lsu_amo
