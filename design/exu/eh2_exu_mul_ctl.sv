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


module eh2_exu_mul_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (
   input logic          clk,              // Top level clock
   input logic          clk_override,     // Override clock enables
   input logic          rst_l,            // Reset
   input logic          scan_mode,        // Scan mode

   input logic [31:0]   a,                // A operand
   input logic [31:0]   b,                // B operand

   input logic [31:0]   lsu_result_dc3,   // Load result used in E1 bypass

   input eh2_mul_pkt_t mp,               // valid, rs1_sign, rs2_sign, low, load_mul_rs1_bypass_e1, load_mul_rs2_bypass_e1, bitmanip controls


   output logic [31:0]  out               // Result

   );


   eh2_mul_pkt_t       mp_e1, mp_e2;
   logic                valid_e1, valid_e2;
   logic                mul_c1_e1_clken,   mul_c1_e2_clken,   mul_c1_e3_clken;
   logic                exu_mul_c1_e1_clk, exu_mul_c1_e2_clk, exu_mul_c1_e3_clk;

   logic        [31:0]  a_ff_e1, a_e1;
   logic        [31:0]  b_ff_e1, b_e1;
   logic                rs1_sign_e1, rs1_neg_e1;
   logic                rs2_sign_e1, rs2_neg_e1;
   logic signed [32:0]  a_ff_e2, b_ff_e2;
   logic        [63:0]  prod_e3;
   logic                low_e2, low_e3;


   // *** Start - BitManip ***

   logic                bitmanip_sel_e2;
   logic                bitmanip_sel_e3;
   logic        [31:0]  bitmanip_e2;
   logic        [31:0]  bitmanip_e3;


   // ZBE
   logic                ap_bcompress_e2;
   logic                ap_bdecompress_e2;

   // ZBC
   logic                ap_clmul_e2;
   logic                ap_clmulh_e2;
   logic                ap_clmulr_e2;

   // ZBP
   logic                ap_grev_e2;
   logic                ap_gorc_e2;
   logic                ap_shfl_e2;
   logic                ap_unshfl_e2;
   logic                ap_xperm_n_e2;
   logic                ap_xperm_b_e2;
   logic                ap_xperm_h_e2;

   // ZBR
   logic                ap_crc32_b_e2;
   logic                ap_crc32_h_e2;
   logic                ap_crc32_w_e2;
   logic                ap_crc32c_b_e2;
   logic                ap_crc32c_h_e2;
   logic                ap_crc32c_w_e2;

   // ZBF
   logic                ap_bfp_e2;


   if (pt.BITMANIP_ZBE == 1)
     begin
       assign ap_bcompress_e2   =  mp_e2.bcompress;
       assign ap_bdecompress_e2 =  mp_e2.bdecompress;
     end
   else
     begin
       assign ap_bcompress_e2   =  1'b0;
       assign ap_bdecompress_e2 =  1'b0;
     end

   if (pt.BITMANIP_ZBC == 1)
     begin
       assign ap_clmul_e2     =  mp_e2.clmul;
       assign ap_clmulh_e2    =  mp_e2.clmulh;
       assign ap_clmulr_e2    =  mp_e2.clmulr;
     end
   else
     begin
       assign ap_clmul_e2     =  1'b0;
       assign ap_clmulh_e2    =  1'b0;
       assign ap_clmulr_e2    =  1'b0;
     end

   if (pt.BITMANIP_ZBP == 1)
     begin
       assign ap_grev_e2      =  mp_e2.grev;
       assign ap_gorc_e2      =  mp_e2.gorc;
       assign ap_shfl_e2      =  mp_e2.shfl;
       assign ap_unshfl_e2    =  mp_e2.unshfl;
       assign ap_xperm_n_e2   =  mp_e2.xperm_n;
       assign ap_xperm_b_e2   =  mp_e2.xperm_b;
       assign ap_xperm_h_e2   =  mp_e2.xperm_h;
     end
   else
     begin
       assign ap_grev_e2      =  1'b0;
       assign ap_gorc_e2      =  1'b0;
       assign ap_shfl_e2      =  1'b0;
       assign ap_unshfl_e2    =  1'b0;
       assign ap_xperm_n_e2   =  1'b0;
       assign ap_xperm_b_e2   =  1'b0;
       assign ap_xperm_h_e2   =  1'b0;
     end

   if (pt.BITMANIP_ZBR == 1)
     begin
       assign ap_crc32_b_e2   =  mp_e2.crc32_b;
       assign ap_crc32_h_e2   =  mp_e2.crc32_h;
       assign ap_crc32_w_e2   =  mp_e2.crc32_w;
       assign ap_crc32c_b_e2  =  mp_e2.crc32c_b;
       assign ap_crc32c_h_e2  =  mp_e2.crc32c_h;
       assign ap_crc32c_w_e2  =  mp_e2.crc32c_w;
     end
   else
     begin
       assign ap_crc32_b_e2   =  1'b0;
       assign ap_crc32_h_e2   =  1'b0;
       assign ap_crc32_w_e2   =  1'b0;
       assign ap_crc32c_b_e2  =  1'b0;
       assign ap_crc32c_h_e2  =  1'b0;
       assign ap_crc32c_w_e2  =  1'b0;
     end

   if (pt.BITMANIP_ZBF == 1)
     begin
       assign ap_bfp_e2       =  mp_e2.bfp;
     end
   else
     begin
       assign ap_bfp_e2       =  1'b0;
     end


   // *** End   - BitManip ***



   // --------------------------- Clock gating   ----------------------------------

   // C1 clock enables
   assign mul_c1_e1_clken        = (mp.valid | clk_override);
   assign mul_c1_e2_clken        = (valid_e1 | clk_override);
   assign mul_c1_e3_clken        = (valid_e2 | clk_override);

   // C1 - 1 clock pulse for data
   rvoclkhdr exu_mul_c1e1_cgc    (.*, .en(mul_c1_e1_clken),   .l1clk(exu_mul_c1_e1_clk));
   rvoclkhdr exu_mul_c1e2_cgc    (.*, .en(mul_c1_e2_clken),   .l1clk(exu_mul_c1_e2_clk));
   rvoclkhdr exu_mul_c1e3_cgc    (.*, .en(mul_c1_e3_clken),   .l1clk(exu_mul_c1_e3_clk));


   // --------------------------- Input flops    ----------------------------------

   rvdffie #(2,1)                   valid_ff      (.*, .din({mp.valid,valid_e1}),       .dout({valid_e1,valid_e2}),  .clk(clk));
   rvdff  #($bits(eh2_mul_pkt_t))  mp_e1_ff      (.*, .din(mp),                        .dout(mp_e1),                .clk(exu_mul_c1_e1_clk));
   rvdff  #(32)                     a_e1_ff       (.*, .din(a[31:0]),                   .dout(a_ff_e1[31:0]),        .clk(exu_mul_c1_e1_clk));
   rvdff  #(32)                     b_e1_ff       (.*, .din(b[31:0]),                   .dout(b_ff_e1[31:0]),        .clk(exu_mul_c1_e1_clk));


   // --------------------------- E1 Logic Stage ----------------------------------

   assign rs1_sign_e1            =  mp_e1.rs1_sign;
   assign rs2_sign_e1            =  mp_e1.rs2_sign;

   assign a_e1[31:0]             = (mp_e1.load_mul_rs1_bypass_e1)  ?  lsu_result_dc3[31:0]  :  a_ff_e1[31:0];
   assign b_e1[31:0]             = (mp_e1.load_mul_rs2_bypass_e1)  ?  lsu_result_dc3[31:0]  :  b_ff_e1[31:0];

   assign rs1_neg_e1             =  rs1_sign_e1 & a_e1[31];
   assign rs2_neg_e1             =  rs2_sign_e1 & b_e1[31];


   rvdff  #($bits(eh2_mul_pkt_t))  mp_e2_ff      (.*, .din(mp_e1),                     .dout(mp_e2),             .clk(exu_mul_c1_e2_clk));

   rvdff  #(33)                     a_e2_ff       (.*, .din({rs1_neg_e1, a_e1[31:0]}),  .dout(a_ff_e2[32:0]),     .clk(exu_mul_c1_e2_clk));
   rvdff  #(33)                     b_e2_ff       (.*, .din({rs2_neg_e1, b_e1[31:0]}),  .dout(b_ff_e2[32:0]),     .clk(exu_mul_c1_e2_clk));


   logic signed [65:0]  prod_e2;

   assign prod_e2[65:0]          =  a_ff_e2  *  b_ff_e2;


   rvdff  #(1)                      low_e3_ff     (.*, .din(mp_e2.low),                 .dout(low_e3),            .clk(exu_mul_c1_e3_clk));
   rvdff  #(64)                     prod_e3_ff    (.*, .din(prod_e2[63:0]),             .dout(prod_e3[63:0]),     .clk(exu_mul_c1_e3_clk));



   // * * * * * * * * * * * * * * * * * *  BitManip  :  BCOMPRESS, BDECOMPRESS * * * * * * * * * * * * *


   // *** BCOMPRESS == "gather"  ***

   logic        [31:0]    bcompress_e2;
   logic                  bcompress_test_bit_e2;
   integer                bcompress_i, bcompress_j;


   always_comb
     begin

       bcompress_j                      =      0;
       bcompress_test_bit_e2            =   1'b0;
       bcompress_e2[31:0]               =  32'b0;

       for (bcompress_i=0; bcompress_i<32; bcompress_i++)
         begin
             bcompress_test_bit_e2      =  b_ff_e2[bcompress_i];
             if (bcompress_test_bit_e2)
               begin
                  bcompress_e2[bcompress_j]  =  a_ff_e2[bcompress_i];
                  bcompress_j           =  bcompress_j + 1;
               end  // IF  bcompress_test_bit
         end        // FOR bcompress_i
     end            // ALWAYS_COMB



   // *** BDECOMPRESS == "scatter" ***

   logic        [31:0]    bdecompress_e2;
   logic                  bdecompress_test_bit_e2;
   integer                bdecompress_i, bdecompress_j;


   always_comb
     begin

       bdecompress_j                      =      0;
       bdecompress_test_bit_e2            =   1'b0;
       bdecompress_e2[31:0]               =  32'b0;

       for (bdecompress_i=0; bdecompress_i<32; bdecompress_i++)
         begin
             bdecompress_test_bit_e2      =  b_ff_e2[bdecompress_i];
             if (bdecompress_test_bit_e2)
               begin
                  bdecompress_e2[bdecompress_i]  =  a_ff_e2[bdecompress_j];
                  bdecompress_j           =  bdecompress_j + 1;
               end  // IF  bdecompress_test_bit
         end        // FOR bdecompress_i
     end            // ALWAYS_COMB




   // * * * * * * * * * * * * * * * * * *  BitManip  :  CLMUL, CLMULH, CLMULR  * * * * * * * * * * * * *

   logic        [62:0]    clmul_raw_e2;


   assign clmul_raw_e2[62:0]     = ( {63{b_ff_e2[00]}} & {31'b0,a_ff_e2[31:0]      } ) ^
                                   ( {63{b_ff_e2[01]}} & {30'b0,a_ff_e2[31:0], 1'b0} ) ^
                                   ( {63{b_ff_e2[02]}} & {29'b0,a_ff_e2[31:0], 2'b0} ) ^
                                   ( {63{b_ff_e2[03]}} & {28'b0,a_ff_e2[31:0], 3'b0} ) ^
                                   ( {63{b_ff_e2[04]}} & {27'b0,a_ff_e2[31:0], 4'b0} ) ^
                                   ( {63{b_ff_e2[05]}} & {26'b0,a_ff_e2[31:0], 5'b0} ) ^
                                   ( {63{b_ff_e2[06]}} & {25'b0,a_ff_e2[31:0], 6'b0} ) ^
                                   ( {63{b_ff_e2[07]}} & {24'b0,a_ff_e2[31:0], 7'b0} ) ^
                                   ( {63{b_ff_e2[08]}} & {23'b0,a_ff_e2[31:0], 8'b0} ) ^
                                   ( {63{b_ff_e2[09]}} & {22'b0,a_ff_e2[31:0], 9'b0} ) ^
                                   ( {63{b_ff_e2[10]}} & {21'b0,a_ff_e2[31:0],10'b0} ) ^
                                   ( {63{b_ff_e2[11]}} & {20'b0,a_ff_e2[31:0],11'b0} ) ^
                                   ( {63{b_ff_e2[12]}} & {19'b0,a_ff_e2[31:0],12'b0} ) ^
                                   ( {63{b_ff_e2[13]}} & {18'b0,a_ff_e2[31:0],13'b0} ) ^
                                   ( {63{b_ff_e2[14]}} & {17'b0,a_ff_e2[31:0],14'b0} ) ^
                                   ( {63{b_ff_e2[15]}} & {16'b0,a_ff_e2[31:0],15'b0} ) ^
                                   ( {63{b_ff_e2[16]}} & {15'b0,a_ff_e2[31:0],16'b0} ) ^
                                   ( {63{b_ff_e2[17]}} & {14'b0,a_ff_e2[31:0],17'b0} ) ^
                                   ( {63{b_ff_e2[18]}} & {13'b0,a_ff_e2[31:0],18'b0} ) ^
                                   ( {63{b_ff_e2[19]}} & {12'b0,a_ff_e2[31:0],19'b0} ) ^
                                   ( {63{b_ff_e2[20]}} & {11'b0,a_ff_e2[31:0],20'b0} ) ^
                                   ( {63{b_ff_e2[21]}} & {10'b0,a_ff_e2[31:0],21'b0} ) ^
                                   ( {63{b_ff_e2[22]}} & { 9'b0,a_ff_e2[31:0],22'b0} ) ^
                                   ( {63{b_ff_e2[23]}} & { 8'b0,a_ff_e2[31:0],23'b0} ) ^
                                   ( {63{b_ff_e2[24]}} & { 7'b0,a_ff_e2[31:0],24'b0} ) ^
                                   ( {63{b_ff_e2[25]}} & { 6'b0,a_ff_e2[31:0],25'b0} ) ^
                                   ( {63{b_ff_e2[26]}} & { 5'b0,a_ff_e2[31:0],26'b0} ) ^
                                   ( {63{b_ff_e2[27]}} & { 4'b0,a_ff_e2[31:0],27'b0} ) ^
                                   ( {63{b_ff_e2[28]}} & { 3'b0,a_ff_e2[31:0],28'b0} ) ^
                                   ( {63{b_ff_e2[29]}} & { 2'b0,a_ff_e2[31:0],29'b0} ) ^
                                   ( {63{b_ff_e2[30]}} & { 1'b0,a_ff_e2[31:0],30'b0} ) ^
                                   ( {63{b_ff_e2[31]}} & {      a_ff_e2[31:0],31'b0} );




   // * * * * * * * * * * * * * * * * * *  BitManip  :  GREV         * * * * * * * * * * * * * * * * * *

   // uint32_t grev32(uint32_t rs1, uint32_t rs2)
   // {
   //     uint32_t x = rs1;
   //     int shamt = rs2 & 31;
   //
   //     if (shamt &  1)  x = ( (x & 0x55555555) <<  1) | ( (x & 0xAAAAAAAA) >>  1);
   //     if (shamt &  2)  x = ( (x & 0x33333333) <<  2) | ( (x & 0xCCCCCCCC) >>  2);
   //     if (shamt &  4)  x = ( (x & 0x0F0F0F0F) <<  4) | ( (x & 0xF0F0F0F0) >>  4);
   //     if (shamt &  8)  x = ( (x & 0x00FF00FF) <<  8) | ( (x & 0xFF00FF00) >>  8);
   //     if (shamt & 16)  x = ( (x & 0x0000FFFF) << 16) | ( (x & 0xFFFF0000) >> 16);
   //
   //     return x;
   //  }


   logic        [31:0]    grev1_e2;
   logic        [31:0]    grev2_e2;
   logic        [31:0]    grev4_e2;
   logic        [31:0]    grev8_e2;
   logic        [31:0]    grev_e2;


   assign grev1_e2[31:0]      = (b_ff_e2[0])  ?  {a_ff_e2[30],a_ff_e2[31],a_ff_e2[28],a_ff_e2[29],a_ff_e2[26],a_ff_e2[27],a_ff_e2[24],a_ff_e2[25],
                                                  a_ff_e2[22],a_ff_e2[23],a_ff_e2[20],a_ff_e2[21],a_ff_e2[18],a_ff_e2[19],a_ff_e2[16],a_ff_e2[17],
                                                  a_ff_e2[14],a_ff_e2[15],a_ff_e2[12],a_ff_e2[13],a_ff_e2[10],a_ff_e2[11],a_ff_e2[08],a_ff_e2[09],
                                                  a_ff_e2[06],a_ff_e2[07],a_ff_e2[04],a_ff_e2[05],a_ff_e2[02],a_ff_e2[03],a_ff_e2[00],a_ff_e2[01]}  :  a_ff_e2[31:0];

   assign grev2_e2[31:0]      = (b_ff_e2[1])  ?  {grev1_e2[29:28],grev1_e2[31:30],grev1_e2[25:24],grev1_e2[27:26],
                                                  grev1_e2[21:20],grev1_e2[23:22],grev1_e2[17:16],grev1_e2[19:18],
                                                  grev1_e2[13:12],grev1_e2[15:14],grev1_e2[09:08],grev1_e2[11:10],
                                                  grev1_e2[05:04],grev1_e2[07:06],grev1_e2[01:00],grev1_e2[03:02]}  :  grev1_e2[31:0];

   assign grev4_e2[31:0]      = (b_ff_e2[2])  ?  {grev2_e2[27:24],grev2_e2[31:28],grev2_e2[19:16],grev2_e2[23:20],
                                                  grev2_e2[11:08],grev2_e2[15:12],grev2_e2[03:00],grev2_e2[07:04]}  :  grev2_e2[31:0];

   assign grev8_e2[31:0]      = (b_ff_e2[3])  ?  {grev4_e2[23:16],grev4_e2[31:24],grev4_e2[07:00],grev4_e2[15:08]}  :  grev4_e2[31:0];

   assign grev_e2[31:0]       = (b_ff_e2[4])  ?  {grev8_e2[15:00],grev8_e2[31:16]}  :  grev8_e2[31:0];




   // * * * * * * * * * * * * * * * * * *  BitManip  :  GORC         * * * * * * * * * * * * * * * * * *

   // uint32_t gorc32(uint32_t rs1, uint32_t rs2)
   // {
   //     uint32_t x = rs1;
   //     int shamt = rs2 & 31;
   //
   //     if (shamt &  1)  x |= ( (x & 0x55555555) <<  1) | ( (x & 0xAAAAAAAA) >>  1);
   //     if (shamt &  2)  x |= ( (x & 0x33333333) <<  2) | ( (x & 0xCCCCCCCC) >>  2);
   //     if (shamt &  4)  x |= ( (x & 0x0F0F0F0F) <<  4) | ( (x & 0xF0F0F0F0) >>  4);
   //     if (shamt &  8)  x |= ( (x & 0x00FF00FF) <<  8) | ( (x & 0xFF00FF00) >>  8);
   //     if (shamt & 16)  x |= ( (x & 0x0000FFFF) << 16) | ( (x & 0xFFFF0000) >> 16);
   //
   //     return x;
   //  }


   logic        [31:0]    gorc1_e2;
   logic        [31:0]    gorc2_e2;
   logic        [31:0]    gorc4_e2;
   logic        [31:0]    gorc8_e2;
   logic        [31:0]    gorc_e2;


   assign gorc1_e2[31:0]      = ( {32{b_ff_e2[0]}} & {a_ff_e2[30],a_ff_e2[31],a_ff_e2[28],a_ff_e2[29],a_ff_e2[26],a_ff_e2[27],a_ff_e2[24],a_ff_e2[25],
                                                      a_ff_e2[22],a_ff_e2[23],a_ff_e2[20],a_ff_e2[21],a_ff_e2[18],a_ff_e2[19],a_ff_e2[16],a_ff_e2[17],
                                                      a_ff_e2[14],a_ff_e2[15],a_ff_e2[12],a_ff_e2[13],a_ff_e2[10],a_ff_e2[11],a_ff_e2[08],a_ff_e2[09],
                                                      a_ff_e2[06],a_ff_e2[07],a_ff_e2[04],a_ff_e2[05],a_ff_e2[02],a_ff_e2[03],a_ff_e2[00],a_ff_e2[01]} ) | a_ff_e2[31:0];

   assign gorc2_e2[31:0]      = ( {32{b_ff_e2[1]}} & {gorc1_e2[29:28],gorc1_e2[31:30],gorc1_e2[25:24],gorc1_e2[27:26],
                                                      gorc1_e2[21:20],gorc1_e2[23:22],gorc1_e2[17:16],gorc1_e2[19:18],
                                                      gorc1_e2[13:12],gorc1_e2[15:14],gorc1_e2[09:08],gorc1_e2[11:10],
                                                      gorc1_e2[05:04],gorc1_e2[07:06],gorc1_e2[01:00],gorc1_e2[03:02]} ) | gorc1_e2[31:0];

   assign gorc4_e2[31:0]      = ( {32{b_ff_e2[2]}} & {gorc2_e2[27:24],gorc2_e2[31:28],gorc2_e2[19:16],gorc2_e2[23:20],
                                                      gorc2_e2[11:08],gorc2_e2[15:12],gorc2_e2[03:00],gorc2_e2[07:04]} ) | gorc2_e2[31:0];

   assign gorc8_e2[31:0]      = ( {32{b_ff_e2[3]}} & {gorc4_e2[23:16],gorc4_e2[31:24],gorc4_e2[07:00],gorc4_e2[15:08]} ) | gorc4_e2[31:0];

   assign gorc_e2[31:0]       = ( {32{b_ff_e2[4]}} & {gorc8_e2[15:00],gorc8_e2[31:16]} ) | gorc8_e2[31:0];




   // * * * * * * * * * * * * * * * * * *  BitManip  :  SHFL, UNSHLF * * * * * * * * * * * * * * * * * *

   // uint32_t shuffle32_stage (uint32_t src, uint32_t maskL, uint32_t maskR, int N)
   // {
   //     uint32_t x  = src & ~(maskL | maskR);
   //     x          |= ((src << N) & maskL) | ((src >> N) & maskR);
   //     return x;
   // }
   //
   //
   //
   // uint32_t shfl32(uint32_t rs1, uint32_t rs2)
   // {
   //     uint32_t x = rs1;
   //     int shamt = rs2 & 15
   //
   //     if (shamt & 8)  x = shuffle32_stage(x, 0x00ff0000, 0x0000ff00, 8);
   //     if (shamt & 4)  x = shuffle32_stage(x, 0x0f000f00, 0x00f000f0, 4);
   //     if (shamt & 2)  x = shuffle32_stage(x, 0x30303030, 0xc0c0c0c0, 2);
   //     if (shamt & 1)  x = shuffle32_stage(x, 0x44444444, 0x22222222, 1);
   //
   //     return x;
   // }


   logic        [31:0]    shfl8_e2;
   logic        [31:0]    shfl4_e2;
   logic        [31:0]    shfl2_e2;
   logic        [31:0]    shfl_e2;



   assign shfl8_e2[31:0]      = (b_ff_e2[3])  ?  {a_ff_e2[31:24],a_ff_e2[15:08],a_ff_e2[23:16],a_ff_e2[07:00]}      :  a_ff_e2[31:0];

   assign shfl4_e2[31:0]      = (b_ff_e2[2])  ?  {shfl8_e2[31:28],shfl8_e2[23:20],shfl8_e2[27:24],shfl8_e2[19:16],
                                                  shfl8_e2[15:12],shfl8_e2[07:04],shfl8_e2[11:08],shfl8_e2[03:00]}  :  shfl8_e2[31:0];

   assign shfl2_e2[31:0]      = (b_ff_e2[1])  ?  {shfl4_e2[31:30],shfl4_e2[27:26],shfl4_e2[29:28],shfl4_e2[25:24],
                                                  shfl4_e2[23:22],shfl4_e2[19:18],shfl4_e2[21:20],shfl4_e2[17:16],
                                                  shfl4_e2[15:14],shfl4_e2[11:10],shfl4_e2[13:12],shfl4_e2[09:08],
                                                  shfl4_e2[07:06],shfl4_e2[03:02],shfl4_e2[05:04],shfl4_e2[01:00]}  :  shfl4_e2[31:0];

   assign shfl_e2[31:0]       = (b_ff_e2[0])  ?  {shfl2_e2[31],shfl2_e2[29],shfl2_e2[30],shfl2_e2[28],shfl2_e2[27],shfl2_e2[25],shfl2_e2[26],shfl2_e2[24],
                                                  shfl2_e2[23],shfl2_e2[21],shfl2_e2[22],shfl2_e2[20],shfl2_e2[19],shfl2_e2[17],shfl2_e2[18],shfl2_e2[16],
                                                  shfl2_e2[15],shfl2_e2[13],shfl2_e2[14],shfl2_e2[12],shfl2_e2[11],shfl2_e2[09],shfl2_e2[10],shfl2_e2[08],
                                                  shfl2_e2[07],shfl2_e2[05],shfl2_e2[06],shfl2_e2[04],shfl2_e2[03],shfl2_e2[01],shfl2_e2[02],shfl2_e2[00]}  :  shfl2_e2[31:0];




   // uint32_t unshfl32(uint32_t rs1, uint32_t rs2)
   // {
   //     uint32_t x = rs1;
   //     int shamt = rs2 & 15
   //
   //     if (shamt & 1)  x = shuffle32_stage(x, 0x44444444, 0x22222222, 1);
   //     if (shamt & 2)  x = shuffle32_stage(x, 0x30303030, 0xc0c0c0c0, 2);
   //     if (shamt & 4)  x = shuffle32_stage(x, 0x0f000f00, 0x00f000f0, 4);
   //     if (shamt & 8)  x = shuffle32_stage(x, 0x00ff0000, 0x0000ff00, 8);
   //
   //     return x;
   // }


   logic        [31:0]    unshfl1_e2;
   logic        [31:0]    unshfl2_e2;
   logic        [31:0]    unshfl4_e2;
   logic        [31:0]    unshfl_e2;


   assign unshfl1_e2[31:0]    = (b_ff_e2[0])  ?  {a_ff_e2[31],a_ff_e2[29],a_ff_e2[30],a_ff_e2[28],a_ff_e2[27],a_ff_e2[25],a_ff_e2[26],a_ff_e2[24],
                                                  a_ff_e2[23],a_ff_e2[21],a_ff_e2[22],a_ff_e2[20],a_ff_e2[19],a_ff_e2[17],a_ff_e2[18],a_ff_e2[16],
                                                  a_ff_e2[15],a_ff_e2[13],a_ff_e2[14],a_ff_e2[12],a_ff_e2[11],a_ff_e2[09],a_ff_e2[10],a_ff_e2[08],
                                                  a_ff_e2[07],a_ff_e2[05],a_ff_e2[06],a_ff_e2[04],a_ff_e2[03],a_ff_e2[01],a_ff_e2[02],a_ff_e2[00]}  :  a_ff_e2[31:0];

   assign unshfl2_e2[31:0]    = (b_ff_e2[1])  ?  {unshfl1_e2[31:30],unshfl1_e2[27:26],unshfl1_e2[29:28],unshfl1_e2[25:24],
                                                  unshfl1_e2[23:22],unshfl1_e2[19:18],unshfl1_e2[21:20],unshfl1_e2[17:16],
                                                  unshfl1_e2[15:14],unshfl1_e2[11:10],unshfl1_e2[13:12],unshfl1_e2[09:08],
                                                  unshfl1_e2[07:06],unshfl1_e2[03:02],unshfl1_e2[05:04],unshfl1_e2[01:00]}  :  unshfl1_e2[31:0];

   assign unshfl4_e2[31:0]    = (b_ff_e2[2])  ?  {unshfl2_e2[31:28],unshfl2_e2[23:20],unshfl2_e2[27:24],unshfl2_e2[19:16],
                                                  unshfl2_e2[15:12],unshfl2_e2[07:04],unshfl2_e2[11:08],unshfl2_e2[03:00]}  :  unshfl2_e2[31:0];

   assign unshfl_e2[31:0]     = (b_ff_e2[3])  ?  {unshfl4_e2[31:24],unshfl4_e2[15:08],unshfl4_e2[23:16],unshfl4_e2[07:00]}  :  unshfl4_e2[31:0];




   // * * * * * * * * * * * * * * * * * *  BitManip  :  CRC32, CRC32c  * * * * * * * * * * * * * * * * *

   // ***  computed from   https: //crccalc.com  ***
   //
   // "a" is 8'h61 = 8'b0110_0001    (8'h61 ^ 8'hff = 8'h9e)
   //
   // Input must first be XORed with 32'hffff_ffff
   //
   //
   // CRC32
   //
   // Input    Output        Input      Output
   // -----   --------      --------   --------
   // "a"     e8b7be43      ffffff9e   174841bc
   // "aa"    078a19d7      ffff9e9e   f875e628
   // "aaaa"  ad98e545      9e9e9e9e   5267a1ba
   //
   //
   //
   // CRC32c
   //
   // Input    Output        Input      Output
   // -----   --------      --------   --------
   // "a"     c1d04330      ffffff9e   3e2fbccf
   // "aa"    f1f2dac2      ffff9e9e   0e0d253d
   // "aaaa"  6a52eeb0      9e9e9e9e   95ad114f


   logic                  crc32_all_e2;
   logic        [31:0]    crc32_poly_rev;
   logic        [31:0]    crc32c_poly_rev;
   integer                crc32_bi,   crc32_hi,   crc32_wi,   crc32c_bi,   crc32c_hi,   crc32c_wi;
   logic        [31:0]    crc32_b_e2, crc32_h_e2, crc32_w_e2, crc32c_b_e2, crc32c_h_e2, crc32c_w_e2;


   assign crc32_all_e2           =  ap_crc32_b_e2  | ap_crc32_h_e2  | ap_crc32_w_e2 | ap_crc32c_b_e2 | ap_crc32c_h_e2 | ap_crc32c_w_e2;

   assign crc32_poly_rev[31:0]   =  32'hEDB88320;    // bit reverse of 32'h04C11DB7
   assign crc32c_poly_rev[31:0]  =  32'h82F63B78;    // bit reverse of 32'h1EDC6F41


   always_comb
     begin
       crc32_b_e2[31:0]          =  a_ff_e2[31:0];

       for (crc32_bi=0; crc32_bi<8; crc32_bi++)
         begin
            crc32_b_e2[31:0]     = (crc32_b_e2[31:0] >> 1) ^ (crc32_poly_rev[31:0] & {32{crc32_b_e2[0]}});
         end      // FOR    crc32_bi
     end          // ALWAYS_COMB


   always_comb
     begin
       crc32_h_e2[31:0]          =  a_ff_e2[31:0];

       for (crc32_hi=0; crc32_hi<16; crc32_hi++)
         begin
            crc32_h_e2[31:0]     = (crc32_h_e2[31:0] >> 1) ^ (crc32_poly_rev[31:0] & {32{crc32_h_e2[0]}});
         end      // FOR    crc32_hi
     end          // ALWAYS_COMB


   always_comb
     begin
       crc32_w_e2[31:0]          =  a_ff_e2[31:0];

       for (crc32_wi=0; crc32_wi<32; crc32_wi++)
         begin
            crc32_w_e2[31:0]     = (crc32_w_e2[31:0] >> 1) ^ (crc32_poly_rev[31:0] & {32{crc32_w_e2[0]}});
         end      // FOR    crc32_wi
     end          // ALWAYS_COMB




   always_comb
     begin
       crc32c_b_e2[31:0]         =  a_ff_e2[31:0];

       for (crc32c_bi=0; crc32c_bi<8; crc32c_bi++)
         begin
            crc32c_b_e2[31:0]    = (crc32c_b_e2[31:0] >> 1) ^ (crc32c_poly_rev[31:0] & {32{crc32c_b_e2[0]}});
         end      // FOR    crc32c_bi
     end          // ALWAYS_COMB


   always_comb
     begin
       crc32c_h_e2[31:0]         =  a_ff_e2[31:0];

       for (crc32c_hi=0; crc32c_hi<16; crc32c_hi++)
         begin
            crc32c_h_e2[31:0]    = (crc32c_h_e2[31:0] >> 1) ^ (crc32c_poly_rev[31:0] & {32{crc32c_h_e2[0]}});
         end      // FOR    crc32c_hi
     end          // ALWAYS_COMB


   always_comb
     begin
       crc32c_w_e2[31:0]         =  a_ff_e2[31:0];

       for (crc32c_wi=0; crc32c_wi<32; crc32c_wi++)
         begin
            crc32c_w_e2[31:0]    = (crc32c_w_e2[31:0] >> 1) ^ (crc32c_poly_rev[31:0] & {32{crc32c_w_e2[0]}});
         end      // FOR    crc32c_wi
     end          // ALWAYS_COMB





   // * * * * * * * * * * * * * * * * * *  BitManip  :  BFP          * * * * * * * * * * * * * * * * * *

   // uint_xlen_t bfp(uint_xlen_t rs1, uint_xlen_t rs2)
   // {
   //    uint_xlen_t cfg = rs2 >> (XLEN/2);
   //    if ((cfg >> 30) == 2) cfg = cfg >> 16;
   //    int len          = (cfg >> 8) & (XLEN/2-1);
   //    int off          = cfg & (XLEN-1);
   //    len              = len ? len : XLEN/2;
   //    uint_xlen_t mask = slo(0, len) << off;
   //    uint_xlen_t data = rs2 << off;
   //    return (data & mask) | (rs1 & ~mask);

   logic        [4:0]     bfp_len_e2;
   logic        [4:0]     bfp_off_e2;
   logic        [31:0]    bfp_len_mask_e2_;
   logic        [31:0]    bfp_off_mask_e2_;
   logic        [15:0]    bfp_preshift_data_e2;
   logic        [31:0]    bfp_shift_data_e2;
   logic        [31:0]    bfp_shift_mask_e2;
   logic        [31:0]    bfp_result_e2;


   assign bfp_len_e2[3:0]            =  b_ff_e2[27:24];
   assign bfp_len_e2[4]              = (bfp_len_e2[3:0] == 4'b0);   // If LEN field is zero, then LEN=16
   assign bfp_off_e2[4:0]            =  b_ff_e2[20:16];

   assign bfp_len_mask_e2_[31:0]     =  32'hffff_ffff  <<  bfp_len_e2[4:0];
   assign bfp_off_mask_e2_[31:0]     =  32'hffff_ffff  <<  bfp_off_e2[4:0];
   assign bfp_preshift_data_e2[15:0] =  b_ff_e2[15:0] & ~bfp_len_mask_e2_[15:0];

   assign bfp_shift_data_e2[31:0]    = {16'b0,bfp_preshift_data_e2[15:0]}  <<  bfp_off_e2[4:0];
   assign bfp_shift_mask_e2[31:0]    = (bfp_len_mask_e2_[31:0]             <<  bfp_off_e2[4:0]) | ~bfp_off_mask_e2_[31:0];

   assign bfp_result_e2[31:0]        = bfp_shift_data_e2[31:0] | (a_ff_e2[31:0] & bfp_shift_mask_e2[31:0]);




   // * * * * * * * * * * * * * * * * * *  BitManip  :  XPERM          * * * * * * * * * * * * * * * * *

// These instructions operate on nibbles/bytes/half-words/words.
// rs1 is a vector of data words and rs2 is a vector of indices into rs1.
// The result of the instruction is the vector rs2 with each element replaced by the corresponding data word from rs1,
// or zero then the index in rs2 is out of bounds.
//
//   uint_xlen_t xperm(uint_xlen_t rs1, uint_xlen_t rs2, int sz_log2)
//   {
//       uint_xlen_t r = 0;
//       uint_xlen_t sz = 1LL << sz_log2;
//       uint_xlen_t mask = (1LL << sz) - 1;
//       for (int i = 0; i < XLEN; i += sz)
//           { uint_xlen_t pos = ((rs2 >> i) & mask) << sz_log2;
//             if (pos < XLEN)
//                 r |= ((rs1 >> pos) & mask) << i;
//           }
//       return r;
//   }
//
// uint_xlen_t xperm_n (uint_xlen_t rs1, uint_xlen_t rs2) { return xperm(rs1, rs2, 2); }
// uint_xlen_t xperm_b (uint_xlen_t rs1, uint_xlen_t rs2) { return xperm(rs1, rs2, 3); }
// uint_xlen_t xperm_h (uint_xlen_t rs1, uint_xlen_t rs2) { return xperm(rs1, rs2, 4); }
// uint_xlen_t xperm_w (uint_xlen_t rs1, uint_xlen_t rs2) { return xperm(rs1, rs2, 5); }   Not part of RV32
//
// The xperm.[nbhw] instructions can be implemented with an XLEN/4-lane nibble-wide crossbarswitch.


   logic        [31:0]    xperm_n_e2;
   logic        [31:0]    xperm_b_e2;
   logic        [31:0]    xperm_h_e2;

   assign xperm_n_e2[03:00]      =  { 4{    ~b_ff_e2[03]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[02:00],2'b0}) &     4'hf );   // This is a 8:1 mux with qualified selects
   assign xperm_n_e2[07:04]      =  { 4{    ~b_ff_e2[07]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[06:04],2'b0}) &     4'hf );
   assign xperm_n_e2[11:08]      =  { 4{    ~b_ff_e2[11]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[10:08],2'b0}) &     4'hf );
   assign xperm_n_e2[15:12]      =  { 4{    ~b_ff_e2[15]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[14:12],2'b0}) &     4'hf );
   assign xperm_n_e2[19:16]      =  { 4{    ~b_ff_e2[19]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[18:16],2'b0}) &     4'hf );
   assign xperm_n_e2[23:20]      =  { 4{    ~b_ff_e2[23]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[22:20],2'b0}) &     4'hf );
   assign xperm_n_e2[27:24]      =  { 4{    ~b_ff_e2[27]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[26:24],2'b0}) &     4'hf );
   assign xperm_n_e2[31:28]      =  { 4{    ~b_ff_e2[31]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[30:28],2'b0}) &     4'hf );

   assign xperm_b_e2[07:00]      =  { 8{ ~(| b_ff_e2[07:02]) }} & ( (a_ff_e2[31:0] >> {b_ff_e2[01:00],3'b0}) &    8'hff );   // This is a 4:1 mux with qualified selects
   assign xperm_b_e2[15:08]      =  { 8{ ~(| b_ff_e2[15:10]) }} & ( (a_ff_e2[31:0] >> {b_ff_e2[09:08],3'b0}) &    8'hff );
   assign xperm_b_e2[23:16]      =  { 8{ ~(| b_ff_e2[23:18]) }} & ( (a_ff_e2[31:0] >> {b_ff_e2[17:16],3'b0}) &    8'hff );
   assign xperm_b_e2[31:24]      =  { 8{ ~(| b_ff_e2[31:26]) }} & ( (a_ff_e2[31:0] >> {b_ff_e2[25:24],3'b0}) &    8'hff );

   assign xperm_h_e2[15:00]      =  {16{ ~(| b_ff_e2[15:01]) }} & ( (a_ff_e2[31:0] >> {b_ff_e2[00]   ,4'b0}) & 16'hffff );   // This is a 2:1 mux with qualified selects
   assign xperm_h_e2[31:16]      =  {16{ ~(| b_ff_e2[31:17]) }} & ( (a_ff_e2[31:0] >> {b_ff_e2[16]   ,4'b0}) & 16'hffff );





   // * * * * * * * * * * * * * * * * * *  BitManip  :  Common logic * * * * * * * * * * * * * * * * * *


   assign bitmanip_sel_e2        =  ap_bcompress_e2 | ap_bdecompress_e2 | ap_clmul_e2 | ap_clmulh_e2 | ap_clmulr_e2 | ap_grev_e2 | ap_gorc_e2 | ap_shfl_e2 | ap_unshfl_e2 | crc32_all_e2 | ap_bfp_e2 | ap_xperm_n_e2 | ap_xperm_b_e2 | ap_xperm_h_e2;

   assign bitmanip_e2[31:0]      = ( {32{ap_bcompress_e2}}   &       bcompress_e2[31:0]   ) |
                                   ( {32{ap_bdecompress_e2}} &       bdecompress_e2[31:0] ) |
                                   ( {32{ap_clmul_e2}}       &       clmul_raw_e2[31:0]   ) |
                                   ( {32{ap_clmulh_e2}}      & {1'b0,clmul_raw_e2[62:32]} ) |
                                   ( {32{ap_clmulr_e2}}      &       clmul_raw_e2[62:31]  ) |
                                   ( {32{ap_grev_e2}}        &       grev_e2[31:0]        ) |
                                   ( {32{ap_gorc_e2}}        &       gorc_e2[31:0]        ) |
                                   ( {32{ap_shfl_e2}}        &       shfl_e2[31:0]        ) |
                                   ( {32{ap_unshfl_e2}}      &       unshfl_e2[31:0]      ) |
                                   ( {32{ap_crc32_b_e2}}     &       crc32_b_e2[31:0]     ) |
                                   ( {32{ap_crc32_h_e2}}     &       crc32_h_e2[31:0]     ) |
                                   ( {32{ap_crc32_w_e2}}     &       crc32_w_e2[31:0]     ) |
                                   ( {32{ap_crc32c_b_e2}}    &       crc32c_b_e2[31:0]    ) |
                                   ( {32{ap_crc32c_h_e2}}    &       crc32c_h_e2[31:0]    ) |
                                   ( {32{ap_crc32c_w_e2}}    &       crc32c_w_e2[31:0]    ) |
                                   ( {32{ap_bfp_e2}}         &       bfp_result_e2[31:0]  ) |
                                   ( {32{ap_xperm_n_e2}}     &       xperm_n_e2[31:0]     ) |
                                   ( {32{ap_xperm_b_e2}}     &       xperm_b_e2[31:0]     ) |
                                   ( {32{ap_xperm_h_e2}}     &       xperm_h_e2[31:0]     );



   rvdff  #(33)                     i_bitmanip_ff (.*, .din({bitmanip_sel_e2,bitmanip_e2[31:0]}),   .dout({bitmanip_sel_e3,bitmanip_e3[31:0]}),   .clk(exu_mul_c1_e3_clk));




   assign out[31:0]              =  ( {32{~bitmanip_sel_e3 & ~low_e3}} & prod_e3[63:32]    ) |
                                    ( {32{~bitmanip_sel_e3 &  low_e3}} & prod_e3[31:0]     ) |
                                                                         bitmanip_e3[31:0];




endmodule // eh2_exu_mul_ctl
