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
   input  eh2_alu_pkt_t                 ap,                // predecodes
   input  logic [31:0]                   a,                 // A operand
   input  logic [31:0]                   b,                 // B operand
   input  logic [31:1]                   pc,                // for pc=pc+2,4 calculations
   input  eh2_predict_pkt_t             predict_p,         // Predicted branch structure
   input  logic [pt.BTB_TOFFSET_SIZE:1]  brimm,             // Branch offset


   output logic [31:0]                   out,               // final result
   output logic [pt.NUM_THREADS-1:0]     flush_upper,       // Branch flush
   output logic [31:1]                   flush_path,        // Branch flush PC
   output logic [31:1]                   pc_ff,             // flopped PC
   output logic                          pred_correct,      // NPC control
   output eh2_predict_pkt_t             predict_p_ff       // Predicted branch structure
  );


   logic        [31:0]                   zba_a_ff;
   logic        [31:0]                   aout;
   logic                                 cout,ov,neg;
   logic        [31:0]                   lout;
   logic        [31:0]                   sout;
   logic                                 sel_shift,sel_adder;
   logic                                 slt_one;
   logic                                 actual_taken;
   logic signed [31:0]                   a_ff;
   logic        [31:0]                   b_ff;
   logic        [pt.BTB_TOFFSET_SIZE:1]  brimm_ff;
   logic        [31:1]                   pcout;
   logic                                 valid_ff;
   logic                                 cond_mispredict;
   logic                                 target_mispredict;
   logic                                 eq, ne, lt, ge;
   eh2_predict_pkt_t                    pp_ff;
   logic                                 any_jal;
   logic        [1:0]                    newhist;
   logic                                 sel_pc;
   logic        [31:0]                   csr_write_data;




   // *** Start - BitManip ***

   // Zbb
   logic                  ap_clz;
   logic                  ap_ctz;
   logic                  ap_cpop;
   logic                  ap_sext_b;
   logic                  ap_sext_h;
   logic                  ap_min;
   logic                  ap_max;
   logic                  ap_rol;
   logic                  ap_ror;
   logic                  ap_rev8;
   logic                  ap_orc_b;
   logic                  ap_zbb;

   // Zbs
   logic                  ap_bset;
   logic                  ap_bclr;
   logic                  ap_binv;
   logic                  ap_bext;

   // Zbp
   logic                  ap_pack;
   logic                  ap_packu;
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
       assign ap_min          =  ap.min;
       assign ap_max          =  ap.max;
     end
   else
     begin
       assign ap_clz          =  1'b0;
       assign ap_ctz          =  1'b0;
       assign ap_cpop         =  1'b0;
       assign ap_sext_b       =  1'b0;
       assign ap_sext_h       =  1'b0;
       assign ap_min          =  1'b0;
       assign ap_max          =  1'b0;
     end


   if ( (pt.BITMANIP_ZBB == 1) | (pt.BITMANIP_ZBP == 1) )
     begin
       assign ap_rol          =  ap.rol;
       assign ap_ror          =  ap.ror;
       assign ap_rev8         =  ap.grev & (b_ff[4:0] == 5'b11000);
       assign ap_orc_b        =  ap.gorc & (b_ff[4:0] == 5'b00111);
       assign ap_zbb          =  ap.zbb;
     end
   else
     begin
       assign ap_rol          =  1'b0;
       assign ap_ror          =  1'b0;
       assign ap_rev8         =  1'b0;
       assign ap_orc_b        =  1'b0;
       assign ap_zbb          =  1'b0;
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


   if (pt.BITMANIP_ZBP == 1)
     begin
       assign ap_packu        =  ap.packu;
     end
   else
     begin
       assign ap_packu        =  1'b0;
     end


   if ( (pt.BITMANIP_ZBB == 1) | (pt.BITMANIP_ZBP == 1) | (pt.BITMANIP_ZBE == 1) | (pt.BITMANIP_ZBF == 1) )
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



   rvdffie  #(1,1)                     validff         (.*, .clk(clk),                               .din(valid & ~flush[ap_in_tid]),    .dout(valid_ff));
   rvdffe #(32)                        aff             (.*, .clk(clk),        .en(d_enable & valid), .din(a[31:0]),                      .dout(a_ff[31:0]));
   rvdffe #(32)                        bff             (.*, .clk(clk),        .en(d_enable & valid), .din(b[31:0]),                      .dout(b_ff[31:0]));
   rvdffpcie #(31)                     pcff            (.*, .clk(clk),        .en(d_enable),         .din(pc[31:1]),                     .dout(pc_ff[31:1]));   // all PCs run through here
   rvdffe #(pt.BTB_TOFFSET_SIZE)       brimmff         (.*, .clk(clk),        .en(d_enable),         .din(brimm[pt.BTB_TOFFSET_SIZE:1]), .dout(brimm_ff[pt.BTB_TOFFSET_SIZE:1]));
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

   // jal    =>  rs1=pc {pc[31:1],1'b0},  rs2=sext(offset20:1]);   rd=pc+[2,4]
   // jalr   =>  rs1=rs1,                 rs2=sext(offset20:1]);   rd=pc+[2,4]



   assign zba_a_ff[31:0]      = ( {32{ ap_sh1add}} & {a_ff[30:0],1'b0} ) |
                                ( {32{ ap_sh2add}} & {a_ff[29:0],2'b0} ) |
                                ( {32{ ap_sh3add}} & {a_ff[28:0],3'b0} ) |
                                ( {32{~ap_zba   }} &  a_ff[31:0]       );


   logic        [31:0]    bm;

   assign bm[31:0]            = ( ap.sub )  ?  ~b_ff[31:0]  :  b_ff[31:0];

   assign {cout, aout[31:0]}  = {1'b0, zba_a_ff[31:0]} + {1'b0, bm[31:0]} + {32'b0, ap.sub};

   assign ov                  = (~a_ff[31] & ~bm[31] &  aout[31]) |
                                ( a_ff[31] &  bm[31] & ~aout[31] );

   assign lt                  = (~ap.unsign & (neg ^ ov)) |
                                ( ap.unsign & ~cout);

   assign eq                  = (a_ff[31:0] == b_ff[31:0]);
   assign ne                  = ~eq;
   assign neg                 =  aout[31];
   assign ge                  = ~lt;

   assign lout[31:0]          =  ( {32{ap.land & ~ap_zbb}} &  a_ff[31:0] &  b_ff[31:0]  ) |
                                 ( {32{ap.lor  & ~ap_zbb}} & (a_ff[31:0] |  b_ff[31:0]) ) |
                                 ( {32{ap.lxor & ~ap_zbb}} & (a_ff[31:0] ^  b_ff[31:0]) ) |
                                 ( {32{ap.land &  ap_zbb}} &  a_ff[31:0] & ~b_ff[31:0]  ) |
                                 ( {32{ap.lor  &  ap_zbb}} & (a_ff[31:0] | ~b_ff[31:0]) ) |
                                 ( {32{ap.lxor &  ap_zbb}} & (a_ff[31:0] ^ ~b_ff[31:0]) );




   // * * * * * * * * * * * * * * * * * *  BitManip  :  ROL,ROR      * * * * * * * * * * * * * * * * * *
   // * * * * * * * * * * * * * * * * * *  BitManip  :  ZBEXT        * * * * * * * * * * * * * * * * * *

   logic        [5:0]     shift_amount;
   logic        [31:0]    shift_mask;
   logic        [62:0]    shift_extend;
   logic        [62:0]    shift_long;


   assign shift_amount[5:0]            = ( { 6{ap.sll}}   & (6'd32 - {1'b0,b_ff[4:0]}) ) |   // [5] unused
                                         ( { 6{ap.srl}}   &          {1'b0,b_ff[4:0]}  ) |
                                         ( { 6{ap.sra}}   &          {1'b0,b_ff[4:0]}  ) |
                                         ( { 6{ap_rol}}   & (6'd32 - {1'b0,b_ff[4:0]}) ) |
                                         ( { 6{ap_ror}}   &          {1'b0,b_ff[4:0]}  ) |
                                         ( { 6{ap_bext}}  &          {1'b0,b_ff[4:0]}  );


   assign shift_mask[31:0]             = ( 32'hffffffff << ({5{ap.sll}} & b_ff[4:0]) );


   assign shift_extend[31:0]           =  a_ff[31:0];

   assign shift_extend[62:32]          = ( {31{ap.sra}} & {31{a_ff[31]}} ) |
                                         ( {31{ap.sll}} &     a_ff[30:0] ) |
                                         ( {31{ap_rol}} &     a_ff[30:0] ) |
                                         ( {31{ap_ror}} &     a_ff[30:0] );


   assign shift_long[62:0]    = ( shift_extend[62:0] >> shift_amount[4:0] );   // 62-32 unused

   assign sout[31:0]          =   shift_long[31:0] & shift_mask[31:0];




   // * * * * * * * * * * * * * * * * * *  BitManip  :  CLZ,CTZ      * * * * * * * * * * * * * * * * * *

   logic                  bitmanip_clz_ctz_sel;
   logic        [31:0]    bitmanip_a_reverse_ff;
   logic        [31:0]    bitmanip_lzd_ff;
   logic        [5:0]     bitmanip_dw_lzd_enc;
   logic        [5:0]     bitmanip_clz_ctz_result;

   assign bitmanip_clz_ctz_sel         =  ap_clz | ap_ctz;

   assign bitmanip_a_reverse_ff[31:0]  = {a_ff[0],  a_ff[1],  a_ff[2],  a_ff[3],  a_ff[4],  a_ff[5],  a_ff[6],  a_ff[7],
                                          a_ff[8],  a_ff[9],  a_ff[10], a_ff[11], a_ff[12], a_ff[13], a_ff[14], a_ff[15],
                                          a_ff[16], a_ff[17], a_ff[18], a_ff[19], a_ff[20], a_ff[21], a_ff[22], a_ff[23],
                                          a_ff[24], a_ff[25], a_ff[26], a_ff[27], a_ff[28], a_ff[29], a_ff[30], a_ff[31]};

   assign bitmanip_lzd_ff[31:0]        = ( {32{ap_clz}} & a_ff[31:0]                 ) |
                                         ( {32{ap_ctz}} & bitmanip_a_reverse_ff[31:0]);

   logic        [31:0]    bitmanip_lzd_os;
   integer                bitmanip_clzctz_i;
   logic                  found;

   always_comb
     begin
        bitmanip_lzd_os[31:0]   =  bitmanip_lzd_ff[31:0];
        bitmanip_dw_lzd_enc[5:0]=  6'b0;
        found = 1'b0;

        for (int bitmanip_clzctz_i=0; bitmanip_clzctz_i<32 && found==0; bitmanip_clzctz_i++) begin
           if (bitmanip_lzd_os[31] == 1'b0) begin
              bitmanip_dw_lzd_enc[5:0]=  bitmanip_dw_lzd_enc[5:0] + 6'b00_0001;
              bitmanip_lzd_os[31:0]   =  bitmanip_lzd_os[31:0] << 1;
           end
           else
              found=1'b1;
        end
     end


   assign bitmanip_clz_ctz_result[5:0] = {6{bitmanip_clz_ctz_sel}} & {bitmanip_dw_lzd_enc[5],( {5{~bitmanip_dw_lzd_enc[5]}} & bitmanip_dw_lzd_enc[4:0] )};




   // * * * * * * * * * * * * * * * * * *  BitManip  :  CPOP         * * * * * * * * * * * * * * * * * *

   logic        [5:0]     bitmanip_cpop;
   logic        [5:0]     bitmanip_cpop_result;


   integer                bitmanip_cpop_i;

   always_comb
     begin
       bitmanip_cpop[5:0]               =  6'b0;

       for (bitmanip_cpop_i=0; bitmanip_cpop_i<32; bitmanip_cpop_i++)
         begin
            bitmanip_cpop[5:0]          =  bitmanip_cpop[5:0] + {5'b0,a_ff[bitmanip_cpop_i]};
         end      // FOR    bitmanip_cpop_i
     end          // ALWAYS_COMB


   assign bitmanip_cpop_result[5:0]    =  {6{ap_cpop}} & bitmanip_cpop[5:0];




   // * * * * * * * * * * * * * * * * * *  BitManip  :  SEXT_B,SEXT_H  * * * * * * * * * * * * * * * * *

   logic       [31:0]     bitmanip_sext_result;

   assign bitmanip_sext_result[31:0]   = ( {32{ap_sext_b}} & { {24{a_ff[7]}} ,a_ff[7:0]  } ) |
                                         ( {32{ap_sext_h}} & { {16{a_ff[15]}},a_ff[15:0] } );




   // * * * * * * * * * * * * * * * * * *  BitManip  :  MIN,MAX,MINU,MAXU  * * * * * * * * * * * * * * *

   logic                  bitmanip_minmax_sel;
   logic        [31:0]    bitmanip_minmax_result;

   assign bitmanip_minmax_sel          =  ap_min | ap_max;

   logic                  bitmanip_minmax_sel_a;

   assign bitmanip_minmax_sel_a        =  ge  ^ ap_min;

   assign bitmanip_minmax_result[31:0] = ({32{bitmanip_minmax_sel &  bitmanip_minmax_sel_a}}  &  a_ff[31:0]) |
                                         ({32{bitmanip_minmax_sel & ~bitmanip_minmax_sel_a}}  &  b_ff[31:0]);



   // * * * * * * * * * * * * * * * * * *  BitManip  :  PACK, PACKU, PACKH * * * * * * * * * * * * * * *

   logic        [31:0]    bitmanip_pack_result;
   logic        [31:0]    bitmanip_packu_result;
   logic        [31:0]    bitmanip_packh_result;

   assign bitmanip_pack_result[31:0]   = {32{ap_pack}}  & {b_ff[15:0], a_ff[15:0]};
   assign bitmanip_packu_result[31:0]  = {32{ap_packu}} & {b_ff[31:16],a_ff[31:16]};
   assign bitmanip_packh_result[31:0]  = {32{ap_packh}} & {16'b0,b_ff[7:0],a_ff[7:0]};




   // * * * * * * * * * * * * * * * * * *  BitManip  :  REV8   * * * * * * * * * * * * * * * * * * * * *

   logic        [31:0]    bitmanip_rev8_result;
   logic        [31:0]    bitmanip_orc_b_result;

   assign bitmanip_rev8_result[31:0]   = {32{ap_rev8}}  & {a_ff[7:0],a_ff[15:8],a_ff[23:16],a_ff[31:24]};


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

   assign bitmanip_orc_b_result[31:0]  = {32{ap_orc_b}} & { {8{| a_ff[31:24]}}, {8{| a_ff[23:16]}}, {8{| a_ff[15:8]}}, {8{| a_ff[7:0]}} };




   // * * * * * * * * * * * * * * * * * *  BitManip  :  ZBSET, ZBCLR, ZBINV  * * * * * * * * * * * * * *

   logic        [31:0]    bitmanip_sb_1hot;
   logic        [31:0]    bitmanip_sb_data;

   assign bitmanip_sb_1hot[31:0]       = ( 32'h00000001 << b_ff[4:0] );

   assign bitmanip_sb_data[31:0]       = ( {32{ap_bset}} & ( a_ff[31:0] |  bitmanip_sb_1hot[31:0]) ) |
                                         ( {32{ap_bclr}} & ( a_ff[31:0] & ~bitmanip_sb_1hot[31:0]) ) |
                                         ( {32{ap_binv}} & ( a_ff[31:0] ^  bitmanip_sb_1hot[31:0]) );









   assign sel_shift           =  ap.sll  | ap.srl | ap.sra | ap_rol | ap_ror;
   assign sel_adder           = (ap.add  | ap.sub | ap_zba) & ~ap.slt & ~ap_min & ~ap_max;
   assign sel_pc              =  ap.jal  | pp_ff.pcall | pp_ff.pja | pp_ff.pret;
   assign csr_write_data[31:0]= (ap.csr_imm)  ?  b_ff[31:0]  :  a_ff[31:0];

   assign slt_one             =  ap.slt & lt;



   assign out[31:0]           =                        lout[31:0]             |
                                ({32{sel_shift}}    &  sout[31:0]           ) |
                                ({32{sel_adder}}    &  aout[31:0]           ) |
                                ({32{sel_pc}}       & {pcout[31:1],1'b0}    ) |
                                ({32{ap.csr_write}} &  csr_write_data[31:0] ) |
                                                      {31'b0, slt_one}        |
                                ({32{ap_bext}}      & {31'b0, sout[0]}      ) |
                                                      {26'b0, bitmanip_clz_ctz_result[5:0]} |
                                                      {26'b0, bitmanip_cpop_result[5:0]}    |
                                                       bitmanip_sext_result[31:0]    |
                                                       bitmanip_minmax_result[31:0]  |
                                                       bitmanip_pack_result[31:0]    |
                                                       bitmanip_packu_result[31:0]   |
                                                       bitmanip_packh_result[31:0]   |
                                                       bitmanip_rev8_result[31:0]    |
                                                       bitmanip_orc_b_result[31:0]   |
                                                       bitmanip_sb_data[31:0];



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
                     .pc     ( pc_ff[31:1]    ),
                     .offset ( brimm_ff[pt.BTB_TOFFSET_SIZE:1] ),
                     .dout   ( pcout[31:1]    ));


   // pred_correct is for the npc logic
   // pred_correct indicates not to use the flush_path
   // for any_jal pred_correct==0

   assign pred_correct        = (ap.predict_nt & ~actual_taken & ~any_jal) |
                                (ap.predict_t  &  actual_taken & ~any_jal);


   // for any_jal adder output is the flush path
   assign flush_path[31:1]    = (any_jal) ? aout[31:1] : pcout[31:1];


   // pcall and pret are included here
   assign cond_mispredict     = (ap.predict_t  & ~actual_taken) |
                                (ap.predict_nt &  actual_taken);


   // target mispredicts on ret's

   assign target_mispredict   =  pp_ff.pret & (pp_ff.prett[31:1] != aout[31:1]);

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
