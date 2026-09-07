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

   // ZBC
   logic                ap_clmul_e2;
   logic                ap_clmulh_e2;
   logic                ap_clmulr_e2;

   // ZBB
   logic                ap_gorc_e2;

   // ZBKB
   logic                ap_grev_e2;
   logic                ap_zip_e2;
   logic                ap_unzip_e2;

   // ZBKX
   logic                ap_xperm4_e2;
   logic                ap_xperm8_e2;

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

    if (pt.BITMANIP_ZBB == 1)
       assign ap_gorc_e2     =  mp_e2.gorc;
    else
      assign ap_gorc_e2      =  1'b0;


   if (pt.BITMANIP_ZBKX == 1)
     begin
       assign ap_xperm4_e2    =  mp_e2.xperm4;
       assign ap_xperm8_e2    =  mp_e2.xperm8;
     end
   else
     begin
       assign ap_xperm4_e2    =  1'b0;
       assign ap_xperm8_e2    =  1'b0;
     end

    if (pt.BITMANIP_ZBKB == 1)
     begin
       assign ap_grev_e2     =  mp_e2.grev;

       assign ap_zip_e2      =  mp_e2.zip;
       assign ap_unzip_e2    =  mp_e2.unzip;
     end
   else
     begin
       assign ap_grev_e2      =  1'b0;

       assign ap_zip_e2      =  1'b0;
       assign ap_unzip_e2    =  1'b0;
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


    // * * * * * * * * * * * * * * * * * *  BitManip  :  ZIP, UNZIP  * * * * * * * * * * * * * * * * * *
    // ZIP effectively implements the old shfli instruction (shfli rd, rs1, imm) with a hardwired shamt of 15:
    // zip -> shfli rd, rs1, 15
   logic        [31:0]    zip8_e2;
   logic        [31:0]    zip4_e2;
   logic        [31:0]    zip2_e2;
   logic        [31:0]    zip_e2;



   assign zip8_e2[31:0]  = {a_ff_e2[31:24],a_ff_e2[15:08],a_ff_e2[23:16],a_ff_e2[07:00]};

   assign zip4_e2[31:0]  = {zip8_e2[31:28],zip8_e2[23:20],zip8_e2[27:24],zip8_e2[19:16],
                            zip8_e2[15:12],zip8_e2[07:04],zip8_e2[11:08],zip8_e2[03:00]};

   assign zip2_e2[31:0]  = {zip4_e2[31:30],zip4_e2[27:26],zip4_e2[29:28],zip4_e2[25:24],
                            zip4_e2[23:22],zip4_e2[19:18],zip4_e2[21:20],zip4_e2[17:16],
                            zip4_e2[15:14],zip4_e2[11:10],zip4_e2[13:12],zip4_e2[09:08],
                            zip4_e2[07:06],zip4_e2[03:02],zip4_e2[05:04],zip4_e2[01:00]};

   assign zip_e2[31:0]   = {zip2_e2[31],zip2_e2[29],zip2_e2[30],zip2_e2[28],zip2_e2[27],zip2_e2[25],zip2_e2[26],zip2_e2[24],
                            zip2_e2[23],zip2_e2[21],zip2_e2[22],zip2_e2[20],zip2_e2[19],zip2_e2[17],zip2_e2[18],zip2_e2[16],
                            zip2_e2[15],zip2_e2[13],zip2_e2[14],zip2_e2[12],zip2_e2[11],zip2_e2[09],zip2_e2[10],zip2_e2[08],
                            zip2_e2[07],zip2_e2[05],zip2_e2[06],zip2_e2[04],zip2_e2[03],zip2_e2[01],zip2_e2[02],zip2_e2[00]};

   // UNZIP effectively implements the old unshfli instruction (unshfli rd, rs1, imm) with a hardwired shamt of 15:
   // unzip -> unshfli rd, rs1, 15

   logic        [31:0]    unzip1_e2;
   logic        [31:0]    unzip2_e2;
   logic        [31:0]    unzip4_e2;
   logic        [31:0]    unzip_e2;


   assign unzip1_e2[31:0] =  {a_ff_e2[31],a_ff_e2[29],a_ff_e2[30],a_ff_e2[28],a_ff_e2[27],a_ff_e2[25],a_ff_e2[26],a_ff_e2[24],
                              a_ff_e2[23],a_ff_e2[21],a_ff_e2[22],a_ff_e2[20],a_ff_e2[19],a_ff_e2[17],a_ff_e2[18],a_ff_e2[16],
                              a_ff_e2[15],a_ff_e2[13],a_ff_e2[14],a_ff_e2[12],a_ff_e2[11],a_ff_e2[09],a_ff_e2[10],a_ff_e2[08],
                              a_ff_e2[07],a_ff_e2[05],a_ff_e2[06],a_ff_e2[04],a_ff_e2[03],a_ff_e2[01],a_ff_e2[02],a_ff_e2[00]};

   assign unzip2_e2[31:0] =  {unzip1_e2[31:30],unzip1_e2[27:26],unzip1_e2[29:28],unzip1_e2[25:24],
                              unzip1_e2[23:22],unzip1_e2[19:18],unzip1_e2[21:20],unzip1_e2[17:16],
                              unzip1_e2[15:14],unzip1_e2[11:10],unzip1_e2[13:12],unzip1_e2[09:08],
                              unzip1_e2[07:06],unzip1_e2[03:02],unzip1_e2[05:04],unzip1_e2[01:00]};

   assign unzip4_e2[31:0] = {unzip2_e2[31:28],unzip2_e2[23:20],unzip2_e2[27:24],unzip2_e2[19:16],
                             unzip2_e2[15:12],unzip2_e2[07:04],unzip2_e2[11:08],unzip2_e2[03:00]};

   assign unzip_e2[31:0]  = {unzip4_e2[31:24],unzip4_e2[15:08],unzip4_e2[23:16],unzip4_e2[07:00]};


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
// uint_xlen_t xperm4 (uint_xlen_t rs1, uint_xlen_t rs2) { return xperm(rs1, rs2, 2); }
// uint_xlen_t xperm8 (uint_xlen_t rs1, uint_xlen_t rs2) { return xperm(rs1, rs2, 3); }

   logic        [31:0]    xperm4_e2;
   logic        [31:0]    xperm8_e2;

   assign xperm4_e2[03:00]      =  { 4{    ~b_ff_e2[03]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[02:00],2'b0}) &     4'hf );   // This is a 8:1 mux with qualified selects
   assign xperm4_e2[07:04]      =  { 4{    ~b_ff_e2[07]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[06:04],2'b0}) &     4'hf );
   assign xperm4_e2[11:08]      =  { 4{    ~b_ff_e2[11]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[10:08],2'b0}) &     4'hf );
   assign xperm4_e2[15:12]      =  { 4{    ~b_ff_e2[15]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[14:12],2'b0}) &     4'hf );
   assign xperm4_e2[19:16]      =  { 4{    ~b_ff_e2[19]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[18:16],2'b0}) &     4'hf );
   assign xperm4_e2[23:20]      =  { 4{    ~b_ff_e2[23]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[22:20],2'b0}) &     4'hf );
   assign xperm4_e2[27:24]      =  { 4{    ~b_ff_e2[27]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[26:24],2'b0}) &     4'hf );
   assign xperm4_e2[31:28]      =  { 4{    ~b_ff_e2[31]     }} & ( (a_ff_e2[31:0] >> {b_ff_e2[30:28],2'b0}) &     4'hf );

   assign xperm8_e2[07:00]      =  { 8{ ~(| b_ff_e2[07:02]) }} & ( (a_ff_e2[31:0] >> {b_ff_e2[01:00],3'b0}) &    8'hff );   // This is a 4:1 mux with qualified selects
   assign xperm8_e2[15:08]      =  { 8{ ~(| b_ff_e2[15:10]) }} & ( (a_ff_e2[31:0] >> {b_ff_e2[09:08],3'b0}) &    8'hff );
   assign xperm8_e2[23:16]      =  { 8{ ~(| b_ff_e2[23:18]) }} & ( (a_ff_e2[31:0] >> {b_ff_e2[17:16],3'b0}) &    8'hff );
   assign xperm8_e2[31:24]      =  { 8{ ~(| b_ff_e2[31:26]) }} & ( (a_ff_e2[31:0] >> {b_ff_e2[25:24],3'b0}) &    8'hff );





   // * * * * * * * * * * * * * * * * * *  BitManip  :  Common logic * * * * * * * * * * * * * * * * * *


   assign bitmanip_sel_e2        =  ap_clmul_e2 | ap_clmulh_e2 | ap_clmulr_e2 | ap_grev_e2 | ap_gorc_e2 | ap_zip_e2 | ap_unzip_e2 | ap_xperm4_e2 | ap_xperm8_e2;

   assign bitmanip_e2[31:0]      = ( {32{ap_clmul_e2}}      &       clmul_raw_e2[31:0]   ) |
                                   ( {32{ap_clmulh_e2}}     & {1'b0,clmul_raw_e2[62:32]} ) |
                                   ( {32{ap_clmulr_e2}}     &       clmul_raw_e2[62:31]  ) |
                                   ( {32{ap_grev_e2}}       &       grev_e2[31:0]        ) |
                                   ( {32{ap_gorc_e2}}       &       gorc_e2[31:0]        ) |
                                   ( {32{ap_zip_e2}}        &       zip_e2[31:0]         ) |
                                   ( {32{ap_unzip_e2}}      &       unzip_e2[31:0]       ) |
                                   ( {32{ap_xperm4_e2}}     &       xperm4_e2[31:0]      ) |
                                   ( {32{ap_xperm8_e2}}     &       xperm8_e2[31:0]      );



   rvdff  #(33)                     i_bitmanip_ff (.*, .din({bitmanip_sel_e2,bitmanip_e2[31:0]}),   .dout({bitmanip_sel_e3,bitmanip_e3[31:0]}),   .clk(exu_mul_c1_e3_clk));




   assign out[31:0]              =  ( {32{~bitmanip_sel_e3 & ~low_e3}} & prod_e3[63:32]    ) |
                                    ( {32{~bitmanip_sel_e3 &  low_e3}} & prod_e3[31:0]     ) |
                                                                         bitmanip_e3[31:0];




endmodule // eh2_exu_mul_ctl
