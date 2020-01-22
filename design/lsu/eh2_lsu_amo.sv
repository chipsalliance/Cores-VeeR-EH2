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
// Function: Checks the memory map for the address
// Comments:
//
//********************************************************************************
module eh2_lsu_amo
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)(

   input eh2_lsu_pkt_t     lsu_pkt_dc3,                 // packet in dc3
   input logic  [31:0]  i0_result_e4_eff,
   input logic  [31:0]  i1_result_e4_eff,
   input logic          addr_in_pic_dc3,
   input logic  [31:0]  picm_mask_data_dc3,
   input logic  [31:0]  store_data_pre_dc3,
   input logic  [31:0]  lsu_dccm_data_corr_dc3,              // Operand 1 for the ALU

   output logic [31:0]  store_data_dc3,              // Store_Data Operand
   output logic [31:0]  amo_data_dc3                 // Final AMO result to go down the store path

);

// this section does the decode of the type of AMO in dc3.

   logic               amo_sc_dc3;


   logic         amo_add_dc3;
   logic         amo_max_dc3;
   logic         amo_maxu_dc3;
   logic         amo_min_dc3;
   logic         amo_minu_dc3;
   logic         amo_minmax_sel_dc3;
   logic [31:0]  amo_minmax_dc3;
   logic         amo_xor_dc3;
   logic         amo_or_dc3;
   logic         amo_and_dc3;
   logic         amo_swap_dc3;

   logic         logic_sel;
   logic [31:0]  logical_out;
   logic [31:0]  sum_out;

   logic [31:0]  store_datafn_dc3;

   //------------------------------------------------------------------------------------------------------------
   //----------------------------------------Logic starts here---------------------------------------------------
   //------------------------------------------------------------------------------------------------------------

   // decode the instruction type
   assign amo_sc_dc3     = lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd3);

   assign amo_add_dc3    = lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd0);
   assign amo_max_dc3    = lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd20);
   assign amo_maxu_dc3   = lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd28);
   assign amo_min_dc3    = lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd16);
   assign amo_minu_dc3   = lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd24);
   assign amo_xor_dc3    = lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd4);
   assign amo_or_dc3     = lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd8);
   assign amo_and_dc3    = lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd12);
   assign amo_swap_dc3   = lsu_pkt_dc3.valid & lsu_pkt_dc3.atomic & (lsu_pkt_dc3.atomic_instr[4:0] == 5'd1);

   assign amo_minmax_sel_dc3 =  amo_max_dc3 | amo_maxu_dc3 | amo_min_dc3 | amo_minu_dc3;
   assign logic_sel          =  amo_and_dc3 | amo_or_dc3   | amo_xor_dc3;

   assign store_data_dc3[31:0] = (picm_mask_data_dc3[31:0] | {32{~addr_in_pic_dc3}}) &
                                 ((lsu_pkt_dc3.store_data_bypass_e4_c3[1]) ? i1_result_e4_eff[31:0] :
                                  (lsu_pkt_dc3.store_data_bypass_e4_c3[0]) ? i0_result_e4_eff[31:0] : store_data_pre_dc3[31:0]);


   // logical
   assign logical_out[31:0] =  ( {32{amo_and_dc3}} & (lsu_dccm_data_corr_dc3[31:0] & store_data_dc3[31:0]) ) |
                               ( {32{amo_or_dc3}}  & (lsu_dccm_data_corr_dc3[31:0] | store_data_dc3[31:0]) ) |
                               ( {32{amo_xor_dc3}} & (lsu_dccm_data_corr_dc3[31:0] ^ store_data_dc3[31:0]) );
   // adder

   logic         lsu_result_lt_storedata;
   logic         cout;


   // ADD
   assign store_datafn_dc3[31:0]  =  amo_add_dc3 ? store_data_dc3[31:0] : ~store_data_dc3[31:0];
   assign {cout, sum_out[31:0]}   = {1'b0, lsu_dccm_data_corr_dc3[31:0]} + {1'b0, store_datafn_dc3[31:0]} + {32'b0, ~amo_add_dc3};


   // Min/Max/Minu/Maxu
   assign lsu_result_lt_storedata = (~cout & (lsu_pkt_dc3.unsign | ~(lsu_dccm_data_corr_dc3[31] ^ store_data_dc3[31]))) |    // either doing unsigned math or signed with same polarity
                                    (lsu_dccm_data_corr_dc3[31] & ~store_data_dc3[31] & ~lsu_pkt_dc3.unsign);

   assign amo_minmax_dc3[31:0]    = ({32{(amo_max_dc3 | amo_maxu_dc3) &  lsu_result_lt_storedata}}  & store_data_dc3[31:0]        ) |  // MAX if store_data >  result
                                    ({32{(amo_max_dc3 | amo_maxu_dc3) & ~lsu_result_lt_storedata}}  & lsu_dccm_data_corr_dc3[31:0]) |  // MAX if store_data <= result
                                    ({32{(amo_min_dc3 | amo_minu_dc3) & ~lsu_result_lt_storedata}}  & store_data_dc3[31:0]        ) |  // MIN if store_data >  result
                                    ({32{(amo_min_dc3 | amo_minu_dc3) &  lsu_result_lt_storedata}}  & lsu_dccm_data_corr_dc3[31:0]);   // MIN if store_data <= result


  // final result
   assign amo_data_dc3[31:0]      = ({32{logic_sel}}                 & logical_out[31:0])    |  // for the AND/OR/XOR
                                    ({32{amo_add_dc3}}               & sum_out[31:0])        |  // for ADD
                                    ({32{amo_minmax_sel_dc3}}        & amo_minmax_dc3[31:0]) |  // for Min/Max/Minu/Maxu
                                    ({32{amo_swap_dc3 | amo_sc_dc3}} & store_data_dc3[31:0]);   // for SWAP need to store the store data value to the location

endmodule // lsu_amo
