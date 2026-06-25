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


module eh2_exu_alu_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (
   input  logic                          clk,               // Top level clock
   input  logic                          rst_l,             // Reset
   input  logic                          scan_mode,         // Scan control

   input  logic [pt.NUM_THREADS-1:0]     flush,             // Flush pipeline
   input  logic                          b_enable,          // Clock enable - branch
   input  logic                          c_enable,          // Clock enable - control
   input  logic                          d_enable,          // Clock enable - data
   input  logic                          valid,             // Valid
   input  logic                          ap_in_tid,         // predecodes
   input  eh2_alu_pkt_t                  ap,                // predecodes
   input  logic [pt.XLEN-1:0]            a,                 // A operand
   input  logic [pt.XLEN-1:0]            b,                 // B operand
   input  logic [pt.XLEN-1:1]            pc,                // for pc=pc+2,4 calculations
   input  eh2_predict_pkt_t              predict_p,         // Predicted branch structure
   input  logic [pt.BTB_TOFFSET_SIZE:1]  brimm,             // Branch offset


   output logic [pt.XLEN-1:0]            out,               // final result
   output logic [pt.NUM_THREADS-1:0]     flush_upper,       // Branch flush
   output logic [pt.XLEN-1:1]            flush_path,        // Branch flush PC
   output logic [pt.XLEN-1:1]            pc_ff,             // flopped PC
   output logic                          pred_correct,      // NPC control
   output eh2_predict_pkt_t              predict_p_ff       // Predicted branch structure
  );


   logic        [pt.XLEN-1:0]            zba_a_ff;
   logic        [pt.XLEN-1:0]            aout;
   logic                                 cout,ov,neg;
   logic        [pt.XLEN-1:0]            lout;
   logic        [pt.XLEN-1:0]            sout;
   logic                                 sel_shift,sel_adder;
   logic                                 slt_one;
   logic                                 actual_taken;
   logic signed [pt.XLEN-1:0]            a_ff;
   logic        [pt.XLEN-1:0]            b_ff;
   logic        [pt.BTB_TOFFSET_SIZE:1]  brimm_ff;
   logic        [pt.XLEN-1:1]            pcout;
   logic                                 valid_ff;
   logic                                 cond_mispredict;
   logic                                 target_mispredict;
   logic                                 eq, ne, lt, ge;
   eh2_predict_pkt_t                     pp_ff;
   logic                                 any_jal;
   logic        [1:0]                    newhist;
   logic                                 sel_pc;
   logic        [pt.XLEN-1:0]            csr_write_data;




   // *** Start - BitManip ***

   // Zbb
   logic                  ap_clz;
   logic                  ap_ctz;
   logic                  ap_cpop;
   logic                  ap_sext_b;
   logic                  ap_sext_h;
   logic                  ap_zext_h;
   logic                  ap_min;
   logic                  ap_max;
   logic                  ap_orc_b;
   logic                  ap_zbb;

   // Zbb/Zbkb
   logic                  ap_rol;
   logic                  ap_ror;
   logic                  ap_rev8;

   // Zbs
   logic                  ap_bset;
   logic                  ap_bclr;
   logic                  ap_binv;
   logic                  ap_bext;

   // Zbkb
   logic                  ap_pack;
   logic                  ap_packh;

   // Zba
   logic                  ap_sh1add;
   logic                  ap_sh2add;
   logic                  ap_sh3add;
   logic                  ap_zba;

   if (pt.BITMANIP_ZBB == 1)
     begin
       assign ap_clz          =  ap.clz;
       assign ap_ctz          =  ap.ctz;
       assign ap_cpop         =  ap.cpop;
       assign ap_sext_b       =  ap.sext_b;
       assign ap_sext_h       =  ap.sext_h;
       assign ap_zext_h       =  ap.zext_h;
       assign ap_min          =  ap.min;
       assign ap_max          =  ap.max;
     end
   else
     begin
       assign ap_clz          =  1'b0;
       assign ap_ctz          =  1'b0;
       assign ap_cpop         =  1'b0;
       assign ap_sext_b       =  1'b0;
       assign ap_zext_h       =  1'b0;
       assign ap_min          =  1'b0;
       assign ap_max          =  1'b0;
     end


   if ( (pt.BITMANIP_ZBB == 1) | (pt.BITMANIP_ZBKB == 1) )
     begin
       assign ap_rol          =  ap.rol;
       assign ap_ror          =  ap.ror;
       assign ap_orc_b        =  ap.gorc & (b_ff[4:0] == 5'b00111);
       assign ap_zbb          =  ap.zbb;
       assign ap_rev8         =  ap.grev & (b_ff[4:0] == 5'b11000);
     end
   else
     begin
       assign ap_rol          =  1'b0;
       assign ap_ror          =  1'b0;
       assign ap_orc_b        =  1'b0;
       assign ap_zbb          =  1'b0;
       assign ap_rev8         =  1'b0;
     end


   if (pt.BITMANIP_ZBS == 1)
     begin
       assign ap_bset         =  ap.bset;
       assign ap_bclr         =  ap.bclr;
       assign ap_binv         =  ap.binv;
       assign ap_bext         =  ap.bext;
     end
   else
     begin
       assign ap_bset         =  1'b0;
       assign ap_bclr         =  1'b0;
       assign ap_binv         =  1'b0;
       assign ap_bext         =  1'b0;
     end


   if ( (pt.BITMANIP_ZBKB == 1) )
     begin
       assign ap_pack         =  ap.pack;
       assign ap_packh        =  ap.packh;
     end
   else
     begin
       assign ap_pack         =  1'b0;
       assign ap_packh        =  1'b0;
     end


   if (pt.BITMANIP_ZBA == 1)
     begin
       assign ap_sh1add       =  ap.sh1add;
       assign ap_sh2add       =  ap.sh2add;
       assign ap_sh3add       =  ap.sh3add;
       assign ap_zba          =  ap.zba;
     end
   else
     begin
       assign ap_sh1add       =  1'b0;
       assign ap_sh2add       =  1'b0;
       assign ap_sh3add       =  1'b0;
       assign ap_zba          =  1'b0;
     end




   // *** End   - BitManip ***



   rvdffie  #(1,1)               validff         (.*, .clk(clk),                               .din(valid & ~flush[ap_in_tid]),    .dout(valid_ff));
   rvdffe #(pt.XLEN)             aff             (.*, .clk(clk),        .en(d_enable & valid), .din(a[pt.XLEN-1:0]),               .dout(a_ff[pt.XLEN-1:0]));
   rvdffe #(pt.XLEN)             bff             (.*, .clk(clk),        .en(d_enable & valid), .din(b[pt.XLEN-1:0]),               .dout(b_ff[pt.XLEN-1:0]));
   rvdffpcie #(pt.XLEN-1)        pcff            (.*, .clk(clk),        .en(d_enable),         .din(pc[pt.XLEN-1:1]),              .dout(pc_ff[pt.XLEN-1:1]));   // all PCs run through here
   rvdffe #(pt.BTB_TOFFSET_SIZE) brimmff         (.*, .clk(clk),        .en(d_enable),         .din(brimm[pt.BTB_TOFFSET_SIZE:1]), .dout(brimm_ff[pt.BTB_TOFFSET_SIZE:1]));
   rvdffppie #(.WIDTH($bits(eh2_predict_pkt_t)),.LEFT(19),.RIGHT(9)) predictpacketff (.*, .clk(clk), .en(c_enable), .den(b_enable & d_enable),  .din(predict_p),  .dout(pp_ff));


   // immediates are just muxed into rs2

   // add    =>  add=1;
   // sub    =>  add=1; sub=1;

   // slt    =>  slt

   // lui    =>  lctl=2; or x0, imm20 previously << 12
   // auipc  =>  add;   add pc, imm20 previously << 12

   // beq    =>  bctl=4; add; add x0, pc, sext(offset[12:1])
   // bne    =>  bctl=3; add; add x0, pc, sext(offset[12:1])
   // blt    =>  bctl=2; add; add x0, pc, sext(offset[12:1])
   // bge    =>  bctl=1; add; add x0, pc, sext(offset[12:1])

   // jal    =>  rs1=pc {pc[pt.XLEN-1:1],1'b0},  rs2=sext(offset20:1]);   rd=pc+[2,4]
   // jalr   =>  rs1=rs1,                 rs2=sext(offset20:1]);   rd=pc+[2,4]



   assign zba_a_ff[pt.XLEN-1:0] = ( {pt.XLEN{ ap_sh1add}} & {a_ff[pt.XLEN-2:0],1'b0} ) |
                                  ( {pt.XLEN{ ap_sh2add}} & {a_ff[pt.XLEN-3:0],2'b0} ) |
                                  ( {pt.XLEN{ ap_sh3add}} & {a_ff[pt.XLEN-4:0],3'b0} ) |
                                  ( {pt.XLEN{~ap_zba   }} &  a_ff[pt.XLEN-1:0]       );


   logic        [pt.XLEN-1:0]    bm;

   assign bm[pt.XLEN-1:0]     = ( ap.sub )  ?  ~b_ff[pt.XLEN-1:0]  :  b_ff[pt.XLEN-1:0];

   assign {cout, aout[pt.XLEN-1:0]}  = {1'b0, zba_a_ff[pt.XLEN-1:0]} + {1'b0, bm[pt.XLEN-1:0]} + {{pt.XLEN{1'b0}}, ap.sub};

   assign ov                  = (~a_ff[pt.XLEN-1] & ~bm[pt.XLEN-1] &  aout[pt.XLEN-1]) |
                                ( a_ff[pt.XLEN-1] &  bm[pt.XLEN-1] & ~aout[pt.XLEN-1] );

   assign lt                  = (~ap.unsign & (neg ^ ov)) |
                                ( ap.unsign & ~cout);

   assign eq                  = (a_ff[pt.XLEN-1:0] == b_ff[pt.XLEN-1:0]);
   assign ne                  = ~eq;
   assign neg                 =  aout[pt.XLEN-1];
   assign ge                  = ~lt;

   assign lout[pt.XLEN-1:0]   =  ( {pt.XLEN{ap.land & ~ap_zbb}} &  a_ff[pt.XLEN-1:0] &  b_ff[pt.XLEN-1:0]  ) |
                                 ( {pt.XLEN{ap.lor  & ~ap_zbb}} & (a_ff[pt.XLEN-1:0] |  b_ff[pt.XLEN-1:0]) ) |
                                 ( {pt.XLEN{ap.lxor & ~ap_zbb}} & (a_ff[pt.XLEN-1:0] ^  b_ff[pt.XLEN-1:0]) ) |
                                 ( {pt.XLEN{ap.land &  ap_zbb}} &  a_ff[pt.XLEN-1:0] & ~b_ff[pt.XLEN-1:0]  ) |
                                 ( {pt.XLEN{ap.lor  &  ap_zbb}} & (a_ff[pt.XLEN-1:0] | ~b_ff[pt.XLEN-1:0]) ) |
                                 ( {pt.XLEN{ap.lxor &  ap_zbb}} & (a_ff[pt.XLEN-1:0] ^ ~b_ff[pt.XLEN-1:0]) );




   // * * * * * * * * * * * * * * * * * *  BitManip  :  ROL,ROR      * * * * * * * * * * * * * * * * * *
   // * * * * * * * * * * * * * * * * * *  BitManip  :  ZBEXT        * * * * * * * * * * * * * * * * * *

   logic        [pt.XLENW:0]      shift_amount;
   logic        [pt.XLEN-1:0]     shift_mask;
   logic        [2*(pt.XLEN-1):0] shift_extend;
   logic        [2*(pt.XLEN-1):0] shift_long;


   assign shift_amount[pt.XLENW:0]     = ( { pt.XLENW+1{ap.sll}}   & ({pt.XLENW{pt.XLEN}} - {1'b0,b_ff[pt.XLENW-1:0]}) ) |   // [pt.XLENW] unused
                                         ( { pt.XLENW+1{ap.srl}}   &                        {1'b0,b_ff[pt.XLENW-1:0]}  ) |
                                         ( { pt.XLENW+1{ap.sra}}   &                        {1'b0,b_ff[pt.XLENW-1:0]}  ) |
                                         ( { pt.XLENW+1{ap_rol}}   & ({pt.XLENW{pt.XLEN}} - {1'b0,b_ff[pt.XLENW-1:0]}) ) |
                                         ( { pt.XLENW+1{ap_ror}}   &                        {1'b0,b_ff[pt.XLENW-1:0]}  ) |
                                         ( { pt.XLENW+1{ap_bext}}  &                        {1'b0,b_ff[pt.XLENW-1:0]}  );


   assign shift_mask[pt.XLEN-1:0]      = ( {pt.XLEN{1'b1}} << ({pt.XLENW{ap.sll}} & b_ff[pt.XLENW-1:0]) );


   assign shift_extend[pt.XLEN-1:0]    =  a_ff[pt.XLEN-1:0];

   assign shift_extend[2*(pt.XLEN-1):pt.XLEN] = ( {pt.XLEN-1{ap.sra}} & {pt.XLEN-1{a_ff[pt.XLEN-1]}} ) |
                                                ( {pt.XLEN-1{ap.sll}} &            a_ff[pt.XLEN-2:0] ) |
                                                ( {pt.XLEN-1{ap_rol}} &            a_ff[pt.XLEN-2:0] ) |
                                                ( {pt.XLEN-1{ap_ror}} &            a_ff[pt.XLEN-2:0] );


   assign shift_long[2*(pt.XLEN-1):0] = ( shift_extend[2*(pt.XLEN-1):0] >> shift_amount[pt.XLENW-1:0] );   // 2*(pt.XLEN-1)-pt.XLEN unused

   assign sout[pt.XLEN-1:0]           =   shift_long[pt.XLEN-1:0] & shift_mask[pt.XLEN-1:0];




   // * * * * * * * * * * * * * * * * * *  BitManip  :  CLZ,CTZ      * * * * * * * * * * * * * * * * * *

   logic                  bitmanip_clz_ctz_sel;
   logic        [pt.XLEN-1:0]    bitmanip_a_reverse_ff;
   logic        [pt.XLEN-1:0]    bitmanip_lzd_ff;
   logic        [pt.XLENW:0]     bitmanip_dw_lzd_enc;
   logic        [pt.XLENW:0]     bitmanip_clz_ctz_result;

   assign bitmanip_clz_ctz_sel         =  ap_clz | ap_ctz;

   for (genvar i = 0; i < pt.XLEN; i++) begin
     assign bitmanip_a_reverse_ff[i] = a_ff[pt.XLEN-i-1];
   end

   assign bitmanip_lzd_ff[pt.XLEN-1:0] = ( {pt.XLEN{ap_clz}} & a_ff[pt.XLEN-1:0]                 ) |
                                         ( {pt.XLEN{ap_ctz}} & bitmanip_a_reverse_ff[pt.XLEN-1:0]);

   logic    [pt.XLEN-1:0] bitmanip_lzd_os;
   integer                bitmanip_clzctz_i;
   logic                  found;

   always_comb
     begin
        bitmanip_lzd_os[pt.XLEN-1:0]   =  bitmanip_lzd_ff[pt.XLEN-1:0];
        bitmanip_dw_lzd_enc[pt.XLENW:0]=  {pt.XLEN+1{1'b0}};
        found = 1'b0;

        for (int bitmanip_clzctz_i=0; bitmanip_clzctz_i<pt.XLEN && found==0; bitmanip_clzctz_i++) begin
           if (bitmanip_lzd_os[pt.XLEN-1] == 1'b0) begin
              bitmanip_dw_lzd_enc[pt.XLENW:0] =  bitmanip_dw_lzd_enc[pt.XLENW:0] + {{pt.XLENW{1'b0}}, 1'b1};
              bitmanip_lzd_os[pt.XLEN-1:0]    =  bitmanip_lzd_os[pt.XLEN-1:0] << 1;
           end
           else
              found=1'b1;
        end
     end


   assign bitmanip_clz_ctz_result[pt.XLENW:0] = {pt.XLENW+1{bitmanip_clz_ctz_sel}} & {bitmanip_dw_lzd_enc[pt.XLENW],( {pt.XLENW{~bitmanip_dw_lzd_enc[pt.XLENW]}} & bitmanip_dw_lzd_enc[pt.XLENW-1:0] )};




   // * * * * * * * * * * * * * * * * * *  BitManip  :  CPOP         * * * * * * * * * * * * * * * * * *

   logic        [pt.XLENW:0]     bitmanip_cpop;
   logic        [pt.XLENW:0]     bitmanip_cpop_result;


   integer                bitmanip_cpop_i;

   always_comb
     begin
       bitmanip_cpop[pt.XLENW:0]               =  {pt.XLENW{1'b0}};

       for (bitmanip_cpop_i=0; bitmanip_cpop_i<pt.XLEN; bitmanip_cpop_i++)
         begin
            bitmanip_cpop[pt.XLENW:0]          =  bitmanip_cpop[pt.XLENW:0] + {{pt.XLENW{1'b0}},a_ff[bitmanip_cpop_i]};
         end      // FOR    bitmanip_cpop_i
     end          // ALWAYS_COMB


   assign bitmanip_cpop_result[pt.XLENW:0]    =  {pt.XLENW+1{ap_cpop}} & bitmanip_cpop[pt.XLENW:0];




   // * * * * * * * * * * * * * * * * * *  BitManip  :  SEXT_B,SEXT_H  * * * * * * * * * * * * * * * * *

   logic       [pt.XLEN-1:0]     bitmanip_sext_result;

   assign bitmanip_sext_result[pt.XLEN-1:0]   = ( {pt.XLEN{ap_sext_b}} & { {pt.XLEN-8{a_ff[7]}},  a_ff[7:0]  } ) |
                                                ( {pt.XLEN{ap_sext_h}} & { {pt.XLEN-16{a_ff[15]}},a_ff[15:0] } );


  // * * * * * * * * * * * * * * * * * *  BitManip  :  ZEXT_H  * * * * * * * * * * * * * * * * *

   logic       [31:0]     bitmanip_zexth_result;

   assign bitmanip_zexth_result[31:0]  = {32{ap_zext_h}} & {16'b0, a_ff[15:0]};


   // * * * * * * * * * * * * * * * * * *  BitManip  :  MIN,MAX,MINU,MAXU  * * * * * * * * * * * * * * *

   logic                  bitmanip_minmax_sel;
   logic [pt.XLEN-1:0]    bitmanip_minmax_result;

   assign bitmanip_minmax_sel          =  ap_min | ap_max;

   logic                  bitmanip_minmax_sel_a;

   assign bitmanip_minmax_sel_a        =  ge  ^ ap_min;

   assign bitmanip_minmax_result[pt.XLEN-1:0] = ({pt.XLEN{bitmanip_minmax_sel &  bitmanip_minmax_sel_a}}  &  a_ff[pt.XLEN-1:0]) |
                                                ({pt.XLEN{bitmanip_minmax_sel & ~bitmanip_minmax_sel_a}}  &  b_ff[pt.XLEN-1:0]);



   // * * * * * * * * * * * * * * * * * *  BitManip  :  PACK, PACKU, PACKH * * * * * * * * * * * * * * *
   logic        [pt.XLEN-1:0]    bitmanip_pack_result;
   logic        [pt.XLEN-1:0]    bitmanip_packh_result;

   assign bitmanip_pack_result[pt.XLEN-1:0]   = {pt.XLEN{ap_pack}}  & {b_ff[(pt.XLEN/2)-1:0], a_ff[(pt.XLEN/2)-1:0]};
   assign bitmanip_packh_result[pt.XLEN-1:0]  = {pt.XLEN{ap_packh}} & {{pt.XLEN-16{1'b0}},b_ff[7:0],a_ff[7:0]};


   // * * * * * * * * * * * * * * * * * *  BitManip  :  REV8   * * * * * * * * * * * * * * * * * * * * *

   logic        [pt.XLEN-1:0]    bitmanip_rev8_result;
   logic        [pt.XLEN-1:0]    bitmanip_orc_b_result;

   if (pt.XLEN == 32) begin
     assign bitmanip_rev8_result[31:0] = {32{ap_rev8}}  & {a_ff[7:0],a_ff[15:8],a_ff[23:16],a_ff[31:24]};
   end else if (pt.XLEN == 64) begin
     assign bitmanip_rev8_result[63:0] = {64{ap_rev8}}  & {a_ff[7:0],  a_ff[15:8], a_ff[23:16],a_ff[31:24],
                                                           a_ff[39:32],a_ff[47:40],a_ff[55:48],a_ff[63:56]};
   end


// uint32_t gorc32(uint32_t rs1, uint32_t rs2)
// {
//      uint32_t x = rs1;
//      int shamt = rs2 & 31;                                                        ORC.B
//      if (shamt &  1) x |= ((x & 0x55555555) <<  1) | ((x & 0xAAAAAAAA) >>  1);      1
//      if (shamt &  2) x |= ((x & 0x33333333) <<  2) | ((x & 0xCCCCCCCC) >>  2);      1
//      if (shamt &  4) x |= ((x & 0x0F0F0F0F) <<  4) | ((x & 0xF0F0F0F0) >>  4);      1
//      if (shamt &  8) x |= ((x & 0x00FF00FF) <<  8) | ((x & 0xFF00FF00) >>  8);      0
//      if (shamt & 16) x |= ((x & 0x0000FFFF) << 16) | ((x & 0xFFFF0000) >> 16);      0
//      return x;
// }


// BEFORE              31  ,   30  ,   29  ,   28  ,    27  ,   26,     25,     24
// shamt[0]  b =    a31|a30,a31|a30,a29|a28,a29|a28, a27|a26,a27|a26,a25|a24,a25|a24
// shamt[1]  c =    b31|b29,b30|b28,b31|b29,b30|b28, b27|b25,b26|b24,b27|b25,b26|b24
// shamt[2]  d =    c31|c27,c30|c26,c29|c25,c28|c24, c31|c27,c30|c26,c29|c25,c28|c24
//
// Expand d31 =        c31         |         c27;
//            =   b31   |   b29    |    b27   |   b25;
//            = a31|a30 | a29|a28  |  a27|a26 | a25|a24

   if (pt.XLEN == 32) begin
     assign bitmanip_orc_b_result[31:0]  = {32{ap_orc_b}} & { {8{| a_ff[31:24]}}, {8{| a_ff[23:16]}}, {8{| a_ff[15:8]}}, {8{| a_ff[7:0]}} };
   end else if (pt.XLEN == 64) begin
     assign bitmanip_orc_b_result[63:0]  = {64{ap_orc_b}} & { {8{| a_ff[63:56]}}, {8{| a_ff[55:48]}}, {8{| a_ff[47:40]}}, {8{| a_ff[39:32]}},
                                                              {8{| a_ff[31:24]}}, {8{| a_ff[23:16]}}, {8{| a_ff[15:8]}},  {8{| a_ff[7:0]}} };
   end




   // * * * * * * * * * * * * * * * * * *  BitManip  :  ZBSET, ZBCLR, ZBINV  * * * * * * * * * * * * * *

   logic        [pt.XLEN-1:0]    bitmanip_sb_1hot;
   logic        [pt.XLEN-1:0]    bitmanip_sb_data;

   assign bitmanip_sb_1hot[pt.XLEN-1:0] = ( {{pt.XLEN-1{1'b0}}, 1'b1} << b_ff[pt.XLENW-1:0] );

   assign bitmanip_sb_data[pt.XLEN-1:0] = ( {pt.XLEN{ap_bset}} & ( a_ff[pt.XLEN-1:0] |  bitmanip_sb_1hot[pt.XLEN-1:0]) ) |
                                          ( {pt.XLEN{ap_bclr}} & ( a_ff[pt.XLEN-1:0] & ~bitmanip_sb_1hot[pt.XLEN-1:0]) ) |
                                          ( {pt.XLEN{ap_binv}} & ( a_ff[pt.XLEN-1:0] ^  bitmanip_sb_1hot[pt.XLEN-1:0]) );









   assign sel_shift           =  ap.sll  | ap.srl | ap.sra | ap_rol | ap_ror;
   assign sel_adder           = (ap.add  | ap.sub | ap_zba) & ~ap.slt & ~ap_min & ~ap_max;
   assign sel_pc              =  ap.jal  | pp_ff.pcall | pp_ff.pja | pp_ff.pret;
   assign csr_write_data[pt.XLEN-1:0]= (ap.csr_imm)  ?  b_ff[pt.XLEN-1:0]  :  a_ff[pt.XLEN-1:0];

   assign slt_one             =  ap.slt & lt;



   assign out[pt.XLEN-1:0]    =                              lout[pt.XLEN-1:0]             |
                                ({pt.XLEN{sel_shift}}    &   sout[pt.XLEN-1:0]           ) |
                                ({pt.XLEN{sel_adder}}    &   aout[pt.XLEN-1:0]           ) |
                                ({pt.XLEN{sel_pc}}       &  {pcout[pt.XLEN-1:1],1'b0}    ) |
                                ({pt.XLEN{ap.csr_write}} &   csr_write_data[pt.XLEN-1:0] ) |
                                                            {{pt.XLEN-1{1'b0}}, slt_one}   |
                                ({pt.XLEN{ap_bext}}      &  {{pt.XLEN-1{1'b0}}, sout[0]} ) |
                                                            {{pt.XLEN+1-pt.XLENW{1'b0}}, bitmanip_clz_ctz_result[pt.XLENW:0]} |
                                                            {{pt.XLEN+1-pt.XLENW{1'b0}}, bitmanip_cpop_result[pt.XLENW:0]}    |
                                                             bitmanip_sext_result[pt.XLEN-1:0]    |
                                                             bitmanip_zexth_result[31:0]          |
                                                             bitmanip_minmax_result[pt.XLEN-1:0]  |
                                                             bitmanip_pack_result[pt.XLEN-1:0]    |
                                                             bitmanip_packh_result[pt.XLEN-1:0]   |
                                                             bitmanip_rev8_result[pt.XLEN-1:0]    |
                                                             bitmanip_orc_b_result[pt.XLEN-1:0]   |
                                                             bitmanip_sb_data[pt.XLEN-1:0];



   // *** branch handling ***

   assign any_jal             =  ap.jal      |
                                 pp_ff.pcall |
                                 pp_ff.pja   |
                                 pp_ff.pret;

   assign actual_taken        = (ap.beq & eq) |
                                (ap.bne & ne) |
                                (ap.blt & lt) |
                                (ap.bge & ge) |
                                 any_jal;

   // for a conditional br pcout[] will be the opposite of the branch prediction
   // for jal or pcall, it will be the link address pc+2 or pc+4

   rvbradder ibradder (
                     .pc     ( pc_ff[pt.XLEN-1:1]    ),
                     .offset ( brimm_ff[pt.BTB_TOFFSET_SIZE:1] ),
                     .dout   ( pcout[pt.XLEN-1:1]    ));


   // pred_correct is for the npc logic
   // pred_correct indicates not to use the flush_path
   // for any_jal pred_correct==0

   assign pred_correct        = (ap.predict_nt & ~actual_taken & ~any_jal) |
                                (ap.predict_t  &  actual_taken & ~any_jal);


   // for any_jal adder output is the flush path
   assign flush_path[pt.XLEN-1:1]    = (any_jal) ? aout[pt.XLEN-1:1] : pcout[pt.XLEN-1:1];


   // pcall and pret are included here
   assign cond_mispredict     = (ap.predict_t  & ~actual_taken) |
                                (ap.predict_nt &  actual_taken);


   // target mispredicts on ret's

   assign target_mispredict   =  pp_ff.pret & (pp_ff.prett[pt.XLEN-1:1] != aout[pt.XLEN-1:1]);

   for (genvar i=0; i<pt.NUM_THREADS; i++) begin
     assign flush_upper[i]    = ( ap.jal | cond_mispredict | target_mispredict) & valid_ff & (i == ap.tid) & ~flush[i];
   end


   // .i 3
   // .o 2
   // .ilb hist[1] hist[0] taken
   // .ob newhist[1] newhist[0]
   // .type fd
   //
   // 00 0 01
   // 01 0 01
   // 10 0 00
   // 11 0 10
   // 00 1 10
   // 01 1 00
   // 10 1 11
   // 11 1 11

   assign newhist[1]          = ( pp_ff.hist[1] &  pp_ff.hist[0]) | (~pp_ff.hist[0] & actual_taken);
   assign newhist[0]          = (~pp_ff.hist[1] & ~actual_taken)  | ( pp_ff.hist[1] & actual_taken);

   always_comb begin
      predict_p_ff            =  pp_ff;

      predict_p_ff.misp       = ( valid_ff )  ? ( (cond_mispredict | target_mispredict) & ~flush[ap.tid] )  :  pp_ff.misp;
      predict_p_ff.ataken     = ( valid_ff )  ?  actual_taken  :  pp_ff.ataken;
      predict_p_ff.hist[1]    = ( valid_ff )  ?  newhist[1]    :  pp_ff.hist[1];
      predict_p_ff.hist[0]    = ( valid_ff )  ?  newhist[0]    :  pp_ff.hist[0];

   end



endmodule // eh2_exu_alu_ctl
