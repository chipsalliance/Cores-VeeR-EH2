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


module eh2_exu_div_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (
   input logic           clk,                       // Top level clock
   input logic           rst_l,                     // Reset
   input logic           scan_mode,                 // Scan mode

   input eh2_div_pkt_t  dp,                        // valid, sign, rem
   input logic  [pt.XLEN-1:0]   dividend,                  // Numerator
   input logic  [pt.XLEN-1:0]   divisor,                   // Denominator

   input logic           cancel,                    // Cancel divide


   output logic          finish_dly,                // Finish to match data
   output logic [pt.XLEN-1:0]   out                        // Result
  );


   logic [pt.XLEN-1:0]          out_raw;



   assign out[pt.XLEN-1:0] = {pt.XLEN{finish_dly}} & out_raw[pt.XLEN-1:0];     // Qualification added to quiet result bus while divide is iterating



   if (pt.DIV_NEW == 0)
      begin
        eh2_exu_div_existing_1bit_cheapshortq   i_existing_1bit_div_cheapshortq (
            .clk              ( clk                      ),   // I
            .rst_l            ( rst_l                    ),   // I
            .scan_mode        ( scan_mode                ),   // I
            .cancel           ( cancel                   ),   // I
            .valid_in         ( dp.valid                 ),   // I
            .signed_in        (~dp.unsign                ),   // I
            .rem_in           ( dp.rem                   ),   // I
            .dividend_in      ( dividend[pt.XLEN-1:0]    ),   // I
            .divisor_in       ( divisor[pt.XLEN-1:0]     ),   // I
            .valid_out        ( finish_dly               ),   // O
            .data_out         ( out_raw[pt.XLEN-1:0]     ));  // O
      end


   if ( (pt.DIV_NEW == 1) & (pt.DIV_BIT == 1) )
      begin
        eh2_exu_div_new_1bit_fullshortq         i_new_1bit_div_fullshortq  (
            .clk              ( clk                      ),   // I
            .rst_l            ( rst_l                    ),   // I
            .scan_mode        ( scan_mode                ),   // I
            .cancel           ( cancel                   ),   // I
            .valid_in         ( dp.valid                 ),   // I
            .signed_in        (~dp.unsign                ),   // I
            .rem_in           ( dp.rem                   ),   // I
            .dividend_in      ( dividend[pt.XLEN-1:0]    ),   // I
            .divisor_in       ( divisor[pt.XLEN-1:0]     ),   // I
            .valid_out        ( finish_dly               ),   // O
            .data_out         ( out_raw[pt.XLEN-1:0]     ));  // O
      end


   if ( (pt.DIV_NEW == 1) & (pt.DIV_BIT == 2) )
      begin
        eh2_exu_div_new_2bit_fullshortq         i_new_2bit_div_fullshortq  (
            .clk              ( clk                      ),   // I
            .rst_l            ( rst_l                    ),   // I
            .scan_mode        ( scan_mode                ),   // I
            .cancel           ( cancel                   ),   // I
            .valid_in         ( dp.valid                 ),   // I
            .signed_in        (~dp.unsign                ),   // I
            .rem_in           ( dp.rem                   ),   // I
            .dividend_in      ( dividend[pt.XLEN-1:0]    ),   // I
            .divisor_in       ( divisor[pt.XLEN-1:0]     ),   // I
            .valid_out        ( finish_dly               ),   // O
            .data_out         ( out_raw[pt.XLEN-1:0]     ));  // O
      end


   if ( (pt.DIV_NEW == 1) & (pt.DIV_BIT == 3) )
      begin
        eh2_exu_div_new_3bit_fullshortq         i_new_3bit_div_fullshortq  (
            .clk              ( clk                      ),   // I
            .rst_l            ( rst_l                    ),   // I
            .scan_mode        ( scan_mode                ),   // I
            .cancel           ( cancel                   ),   // I
            .valid_in         ( dp.valid                 ),   // I
            .signed_in        (~dp.unsign                ),   // I
            .rem_in           ( dp.rem                   ),   // I
            .dividend_in      ( dividend[pt.XLEN-1:0]    ),   // I
            .divisor_in       ( divisor[pt.XLEN-1:0]     ),   // I
            .valid_out        ( finish_dly               ),   // O
            .data_out         ( out_raw[pt.XLEN-1:0]     ));  // O
      end


   if ( (pt.DIV_NEW == 1) & (pt.DIV_BIT == 4) )
      begin
        eh2_exu_div_new_4bit_fullshortq         i_new_4bit_div_fullshortq  (
            .clk              ( clk                      ),   // I
            .rst_l            ( rst_l                    ),   // I
            .scan_mode        ( scan_mode                ),   // I
            .cancel           ( cancel                   ),   // I
            .valid_in         ( dp.valid                 ),   // I
            .signed_in        (~dp.unsign                ),   // I
            .rem_in           ( dp.rem                   ),   // I
            .dividend_in      ( dividend[pt.XLEN-1:0]    ),   // I
            .divisor_in       ( divisor[pt.XLEN-1:0]     ),   // I
            .valid_out        ( finish_dly               ),   // O
            .data_out         ( out_raw[pt.XLEN-1:0]     ));  // O
      end



endmodule // eh2_exu_div_ctl





// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
module eh2_exu_div_existing_1bit_cheapshortq
#(
`include "eh2_param.vh"
)
  (
   input  logic            clk,                       // Top level clock
   input  logic            rst_l,                     // Reset
   input  logic            scan_mode,                 // Scan mode

   input  logic            cancel,                    // Flush pipeline
   input  logic            valid_in,
   input  logic            signed_in,
   input  logic            rem_in,
   input  logic [pt.XLEN-1:0]     dividend_in,
   input  logic [pt.XLEN-1:0]     divisor_in,

   output logic            valid_out,
   output logic [pt.XLEN-1:0]     data_out
  );


   logic         div_clken;
   logic         run_in, run_state;
   logic [pt.XLENW:0] count_in, count;
   logic [pt.XLEN:0] m_ff;
   logic         qff_enable;
   logic         aff_enable;
   logic [pt.XLEN:0] q_in, q_ff;
   logic [pt.XLEN:0] a_in, a_ff;
   logic [pt.XLEN:0] m_eff;
   logic [pt.XLEN:0] a_shift;
   logic         dividend_neg_ff, divisor_neg_ff;
   logic [pt.XLEN-1:0] dividend_comp;
   logic [pt.XLEN-1:0] dividend_eff;
   logic [pt.XLEN-1:0] q_ff_comp;
   logic [pt.XLEN-1:0] q_ff_eff;
   logic [pt.XLEN-1:0] a_ff_comp;
   logic [pt.XLEN-1:0] a_ff_eff;
   logic         sign_ff, sign_eff;
   logic         rem_ff;
   logic         add;
   logic [pt.XLEN:0]   a_eff;
   logic [2*pt.XLEN:0] a_eff_shift;
   logic         rem_correct;
   logic         valid_ff_x;
   logic         valid_x;
   logic         finish;
   logic         finish_ff;

   logic         smallnum_case_e1, smallnum_case_e2, smallnum_case_e3, smallnum_case_e4, smallnum_case_wb;
   logic [3:0]   smallnum, smallnum_in, smallnum_ff;
   logic         m_already_comp;

   logic [4:0]   a_cls;
   logic [4:0]   b_cls;
   logic [5:0]   shortq_shift;
   logic [5:0]   shortq_shift_ff;
   logic [5:0]   shortq;
   logic         shortq_enable;
   logic         shortq_enable_ff;
   logic [pt.XLEN:0]  short_dividend;
   logic [3:0]   shortq_raw;
   logic [3:0]   shortq_shift_xx;



   rvdffe #(18) i_misc_ff         (.*, .clk(clk), .en(div_clken), .din ({valid_in & ~cancel,
                                                                         finish   & ~cancel,
                                                                         run_in,
                                                                         count_in[pt.XLENW:0],
                                                                         shortq_enable,
                                                                         shortq_shift[3:0],
                                                                         (valid_in & dividend_in[pt.XLEN-1]) | (~valid_in & dividend_neg_ff),
                                                                         (valid_in & divisor_in[pt.XLEN-1] ) | (~valid_in & divisor_neg_ff ),
                                                                         (valid_in & sign_eff       ) | (~valid_in & sign_ff        ),
                                                                         (valid_in & rem_in         ) | (~valid_in & rem_ff         )} ),
                                                                  .dout({valid_ff_x,
                                                                         finish_ff,
                                                                         run_state,
                                                                         count[pt.XLENW:0],
                                                                         shortq_enable_ff,
                                                                         shortq_shift_xx[3:0],
                                                                         dividend_neg_ff,
                                                                         divisor_neg_ff,
                                                                         sign_ff,
                                                                         rem_ff}) );

   rvdffe #(8)  smallnumff        (.*, .clk(clk), .en(div_clken), .din ({smallnum_case_e1 & ~cancel,
                                                                         smallnum_case_e2 & ~cancel,
                                                                         smallnum_case_e3 & ~cancel,
                                                                         smallnum_case_e4 & ~cancel,
                                                                         smallnum_in[3:0]}),
                                                                  .dout({smallnum_case_e2,
                                                                         smallnum_case_e3,
                                                                         smallnum_case_e4,
                                                                         smallnum_case_wb,
                                                                         smallnum_ff[3:0]}));

   rvdffe #(pt.XLEN+1) mff               (.*, .clk(clk), .en(valid_in),     .din({signed_in & divisor_in[pt.XLEN-1], divisor_in[pt.XLEN-1:0]}),   .dout(m_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN+1) qff               (.*, .clk(clk), .en(qff_enable),   .din(q_in[pt.XLEN:0]),                                       .dout(q_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN+1) aff               (.*, .clk(clk), .en(aff_enable),   .din(a_in[pt.XLEN:0]),                                       .dout(a_ff[pt.XLEN:0]));

   rvtwoscomp #(pt.XLEN) i_dividend_comp (.din(q_ff[pt.XLEN-1:0]),    .dout(dividend_comp[pt.XLEN-1:0]));
   rvtwoscomp #(pt.XLEN) i_q_ff_comp     (.din(q_ff[pt.XLEN-1:0]),    .dout(q_ff_comp[pt.XLEN-1:0]));
   rvtwoscomp #(pt.XLEN) i_a_ff_comp     (.din(a_ff[pt.XLEN-1:0]),    .dout(a_ff_comp[pt.XLEN-1:0]));


   assign valid_x                 = valid_ff_x & ~cancel;


   // START - short circuit logic for small numbers {{

   // small number divides - any 4b / 4b is done in 1 cycle (divisor != 0)
   // to generate espresso equations:
   // 1.  smalldiv > smalldiv.e
   // 2.  espresso -Dso -oeqntott smalldiv.e | addassign > smalldiv

   // smallnum case does not cover divide by 0
   assign smallnum_case_e1        = ((q_ff[pt.XLEN-1:4] == {pt.XLEN-4{1'b0}}) & (m_ff[pt.XLEN-1:4] == {pt.XLEN-4{1'b0}}) & (m_ff[pt.XLEN-1:0] != {pt.XLEN{1'b0}}) & ~rem_ff & valid_x) |
                                    ((q_ff[pt.XLEN-1:0] == {pt.XLEN{1'b0}}  ) &                                            (m_ff[pt.XLEN-1:0] != {pt.XLEN{1'b0}}) & ~rem_ff & valid_x);


   assign smallnum[3]             = ( q_ff[3] &                                  ~m_ff[3] & ~m_ff[2] & ~m_ff[1]           );


   assign smallnum[2]             = ( q_ff[3] &                                  ~m_ff[3] & ~m_ff[2] &            ~m_ff[0]) |
                                    ( q_ff[2] &                                  ~m_ff[3] & ~m_ff[2] & ~m_ff[1]           ) |
                                    ( q_ff[3] &  q_ff[2] &                       ~m_ff[3] & ~m_ff[2]                      );


   assign smallnum[1]             = ( q_ff[2] &                                  ~m_ff[3] & ~m_ff[2] &            ~m_ff[0]) |
                                    (                       q_ff[1] &            ~m_ff[3] & ~m_ff[2] & ~m_ff[1]           ) |
                                    ( q_ff[3] &                                  ~m_ff[3] &            ~m_ff[1] & ~m_ff[0]) |
                                    ( q_ff[3] & ~q_ff[2] &                       ~m_ff[3] & ~m_ff[2] &  m_ff[1] &  m_ff[0]) |
                                    (~q_ff[3] &  q_ff[2] &  q_ff[1] &            ~m_ff[3] & ~m_ff[2]                      ) |
                                    ( q_ff[3] &  q_ff[2] &                       ~m_ff[3] &                       ~m_ff[0]) |
                                    ( q_ff[3] &  q_ff[2] &                       ~m_ff[3] &  m_ff[2] & ~m_ff[1]           ) |
                                    ( q_ff[3] &             q_ff[1] & ~m_ff[3] &                       ~m_ff[1]           ) |
                                    ( q_ff[3] &  q_ff[2] &  q_ff[1] &            ~m_ff[3] &  m_ff[2]                      );


   assign smallnum[0]             = (            q_ff[2] &  q_ff[1] &  q_ff[0] & ~m_ff[3] &            ~m_ff[1]           ) |
                                    ( q_ff[3] & ~q_ff[2] &  q_ff[0] &            ~m_ff[3] &             m_ff[1] &  m_ff[0]) |
                                    (            q_ff[2] &                       ~m_ff[3] &            ~m_ff[1] & ~m_ff[0]) |
                                    (                       q_ff[1] &            ~m_ff[3] & ~m_ff[2] &            ~m_ff[0]) |
                                    (                                  q_ff[0] & ~m_ff[3] & ~m_ff[2] & ~m_ff[1]           ) |
                                    (~q_ff[3] &  q_ff[2] & ~q_ff[1] &            ~m_ff[3] & ~m_ff[2] &  m_ff[1] &  m_ff[0]) |
                                    (~q_ff[3] &  q_ff[2] &  q_ff[1] &            ~m_ff[3] &                       ~m_ff[0]) |
                                    ( q_ff[3] &                                             ~m_ff[2] & ~m_ff[1] & ~m_ff[0]) |
                                    ( q_ff[3] & ~q_ff[2] &                       ~m_ff[3] &  m_ff[2] &  m_ff[1]           ) |
                                    (~q_ff[3] &  q_ff[2] &  q_ff[1] &            ~m_ff[3] &  m_ff[2] & ~m_ff[1]           ) |
                                    (~q_ff[3] &  q_ff[2] &             q_ff[0] & ~m_ff[3] &            ~m_ff[1]           ) |
                                    ( q_ff[3] & ~q_ff[2] & ~q_ff[1] &            ~m_ff[3] &  m_ff[2] &             m_ff[0]) |
                                    (           ~q_ff[2] &  q_ff[1] &  q_ff[0] & ~m_ff[3] & ~m_ff[2]                      ) |
                                    ( q_ff[3] &  q_ff[2] &                                             ~m_ff[1] & ~m_ff[0]) |
                                    ( q_ff[3] &             q_ff[1] &                       ~m_ff[2] &            ~m_ff[0]) |
                                    (~q_ff[3] &  q_ff[2] &  q_ff[1] &  q_ff[0] & ~m_ff[3] &  m_ff[2]                      ) |
                                    ( q_ff[3] &  q_ff[2] &                        m_ff[3] & ~m_ff[2]                      ) |
                                    ( q_ff[3] &             q_ff[1] &             m_ff[3] & ~m_ff[2] & ~m_ff[1]           ) |
                                    ( q_ff[3] &                        q_ff[0] &            ~m_ff[2] & ~m_ff[1]           ) |
                                    ( q_ff[3] &            ~q_ff[1] &            ~m_ff[3] &  m_ff[2] &  m_ff[1] &  m_ff[0]) |
                                    ( q_ff[3] &  q_ff[2] &  q_ff[1] &             m_ff[3] &                       ~m_ff[0]) |
                                    ( q_ff[3] &  q_ff[2] &  q_ff[1] &             m_ff[3] &            ~m_ff[1]           ) |
                                    ( q_ff[3] &  q_ff[2] &             q_ff[0] &  m_ff[3] &            ~m_ff[1]           ) |
                                    ( q_ff[3] & ~q_ff[2] &  q_ff[1] &            ~m_ff[3] &             m_ff[1]           ) |
                                    ( q_ff[3] &             q_ff[1] &  q_ff[0] &            ~m_ff[2]                      ) |
                                    ( q_ff[3] &  q_ff[2] &  q_ff[1] &  q_ff[0] &  m_ff[3]                                 );

   assign smallnum_in[3:0]        = ({4{ smallnum_case_e1}} & smallnum[3:0]   ) |
                                    ({4{~smallnum_case_e1}} & smallnum_ff[3:0]);


   // END   - short circuit logic for small numbers }}


   // *** Start Short Q *** {{

   assign short_dividend[pt.XLEN-1:0] =  q_ff[pt.XLEN-1:0];
   assign short_dividend[pt.XLEN]     =  sign_ff & q_ff[pt.XLEN-1];


   //    A       B
   //   210     210    SH
   //   ---     ---    --
   //   1xx     000     0
   //   1xx     001     8
   //   1xx     01x    16
   //   1xx     1xx    24
   //   01x     000     8
   //   01x     001    16
   //   01x     01x    24
   //   01x     1xx    32
   //   001     000    16
   //   001     001    24
   //   001     01x    32
   //   001     1xx    32
   //   000     000    24
   //   000     001    32
   //   000     01x    32
   //   000     1xx    32

   assign a_cls[4:3]              =  2'b0;
   assign a_cls[2]                =  (~short_dividend[pt.XLEN] & (short_dividend[pt.XLEN-1:24] != {pt.XLEN-24{1'b0}})) | ( short_dividend[pt.XLEN] & (short_dividend[pt.XLEN-1:23] != {pt.XLEN-23{1'b1}}));
   assign a_cls[1]                =  (~short_dividend[pt.XLEN] & (short_dividend[23:16]        != {8{1'b0}}))          | ( short_dividend[pt.XLEN] & (short_dividend[22:15]        != {8{1'b1}}));
   assign a_cls[0]                =  (~short_dividend[pt.XLEN] & (short_dividend[15:08]        != {8{1'b0}}))          | ( short_dividend[pt.XLEN] & (short_dividend[14:07]        != {8{1'b1}}));

   assign b_cls[4:3]              =  2'b0;
   assign b_cls[2]                =  (~m_ff[pt.XLEN]           & (          m_ff[pt.XLEN-1:24] != {pt.XLEN-24{1'b0}})) | ( m_ff[pt.XLEN]           & (          m_ff[pt.XLEN-1:24] != {pt.XLEN-24{1'b1}}));
   assign b_cls[1]                =  (~m_ff[pt.XLEN]           & (          m_ff[23:16]        != {8{1'b0}}))          | ( m_ff[pt.XLEN]           & (          m_ff[23:16]        != {8{1'b1}}));
   assign b_cls[0]                =  (~m_ff[pt.XLEN]           & (          m_ff[15:08]        != {8{1'b0}}))          | ( m_ff[pt.XLEN]           & (          m_ff[15:08]        != {8{1'b1}}));

   assign shortq_raw[3]           = ( (a_cls[2:1] == 2'b01 ) & (b_cls[2]   == 1'b1  ) ) |   // Shift by 32
                                    ( (a_cls[2:0] == 3'b001) & (b_cls[2]   == 1'b1  ) ) |
                                    ( (a_cls[2:0] == 3'b000) & (b_cls[2]   == 1'b1  ) ) |
                                    ( (a_cls[2:0] == 3'b001) & (b_cls[2:1] == 2'b01 ) ) |
                                    ( (a_cls[2:0] == 3'b000) & (b_cls[2:1] == 2'b01 ) ) |
                                    ( (a_cls[2:0] == 3'b000) & (b_cls[2:0] == 3'b001) );

   assign shortq_raw[2]           = ( (a_cls[2]   == 1'b1  ) & (b_cls[2]   == 1'b1  ) ) |   // Shift by 24
                                    ( (a_cls[2:1] == 2'b01 ) & (b_cls[2:1] == 2'b01 ) ) |
                                    ( (a_cls[2:0] == 3'b001) & (b_cls[2:0] == 3'b001) ) |
                                    ( (a_cls[2:0] == 3'b000) & (b_cls[2:0] == 3'b000) );

   assign shortq_raw[1]           = ( (a_cls[2]   == 1'b1  ) & (b_cls[2:1] == 2'b01 ) ) |   // Shift by 16
                                    ( (a_cls[2:1] == 2'b01 ) & (b_cls[2:0] == 3'b001) ) |
                                    ( (a_cls[2:0] == 3'b001) & (b_cls[2:0] == 3'b000) );

   assign shortq_raw[0]           = ( (a_cls[2]   == 1'b1  ) & (b_cls[2:0] == 3'b001) ) |   // Shift by  8
                                    ( (a_cls[2:1] == 2'b01 ) & (b_cls[2:0] == 3'b000) );


   assign shortq_enable           =  valid_ff_x & (m_ff[pt.XLEN-1:0] != (pt.XLEN)'('b0)) & (shortq_raw[3:0] != 4'b0) & ~smallnum_case_e1;

   assign shortq_shift[3:0]       = ({4{shortq_enable}} & shortq_raw[3:0]);

   assign shortq[5:0]             =  6'b0;
   assign shortq_shift[5:4]       =  2'b0;
   assign shortq_shift_ff[5]      =  1'b0;

   assign shortq_shift_ff[4:0]    = ({5{shortq_shift_xx[3]}} & 5'b1_1110) |   // 31 -> 30 required for nonblocking div so finish is no faster than E4
                                    ({5{shortq_shift_xx[2]}} & 5'b1_1000) |   // 24
                                    ({5{shortq_shift_xx[1]}} & 5'b1_0000) |   // 16
                                    ({5{shortq_shift_xx[0]}} & 5'b0_1000);    //  8

   // *** End   Short Q *** }}




   assign div_clken               =  valid_in | run_state | finish | finish_ff;

   assign run_in                  = (valid_in | run_state) & ~finish & ~cancel;

   assign count_in[pt.XLENW:0]    = {pt.XLENW+1{run_state & ~finish & ~cancel & ~shortq_enable}} & (count[pt.XLENW:0] + {{pt.XLENW-4{1'b0}},shortq_shift_ff[4:0]} + {{pt.XLENW{1'b0}}, 1'd1});


   assign finish                  = (smallnum_case_e4 | ((~rem_ff) ? (count[pt.XLENW:0] == (pt.XLENW+1)'(pt.XLEN)) : (count[pt.XLENW:0] == (pt.XLENW+1)'(pt.XLEN+1))));

   assign valid_out               =  finish_ff & ~cancel;

   assign sign_eff                =  signed_in & (divisor_in[pt.XLEN-1:0] != (pt.XLEN)'('0));


   assign q_in[pt.XLEN:0]         = ({pt.XLEN+1{~run_state                                   }} &  {1'b0,dividend_in[pt.XLEN-1:0]}) |
                                    ({pt.XLEN+1{ run_state &  (valid_ff_x | shortq_enable_ff)}} &  ({dividend_eff[pt.XLEN-1:0], ~a_in[pt.XLEN]} << shortq_shift_ff[4:0])) |
                                    ({pt.XLEN+1{ run_state & ~(valid_ff_x | shortq_enable_ff)}} &  {q_ff[pt.XLEN-1:0], ~a_in[pt.XLEN]});

   assign qff_enable              =  valid_in | (run_state & ~shortq_enable);




   assign dividend_eff[pt.XLEN-1:0] = (sign_ff & dividend_neg_ff) ? dividend_comp[pt.XLEN-1:0] : q_ff[pt.XLEN-1:0];


   assign m_eff[pt.XLEN:0]        = ( add ) ? m_ff[pt.XLEN:0] : ~m_ff[pt.XLEN:0];

   assign a_eff_shift[2*pt.XLEN:0] = {{pt.XLEN+1{1'b0}}, dividend_eff[pt.XLEN-1:0]} << shortq_shift_ff[4:0];

   assign a_eff[pt.XLEN:0]        = ({pt.XLEN+1{ rem_correct                    }} &  a_ff[pt.XLEN:0]                   ) |
                                    ({pt.XLEN+1{~rem_correct & ~shortq_enable_ff}} & {a_ff[pt.XLEN-1:0], q_ff[pt.XLEN]} ) |
                                    ({pt.XLEN+1{~rem_correct &  shortq_enable_ff}} &  a_eff_shift[2*pt.XLEN:pt.XLEN]    );

   assign a_shift[pt.XLEN:0]      = {pt.XLEN+1{run_state}} & a_eff[pt.XLEN:0];

   assign a_in[pt.XLEN:0]         = {pt.XLEN+1{run_state}} & (a_shift[pt.XLEN:0] + m_eff[pt.XLEN:0] + {{pt.XLEN{1'b0}},~add});

   assign aff_enable              =  valid_in | (run_state & ~shortq_enable & (count[pt.XLENW:0]!=(pt.XLENW+1)'(pt.XLEN+1))) | rem_correct;


   assign m_already_comp          = (divisor_neg_ff & sign_ff);

   // if m already complemented, then invert operation add->sub, sub->add
   assign add                     = (a_ff[pt.XLEN] | rem_correct) ^ m_already_comp;

   assign rem_correct             = (count[pt.XLENW:0] == (pt.XLENW+1)'(pt.XLEN+1)) & rem_ff & a_ff[pt.XLEN];



   assign q_ff_eff[pt.XLEN-1:0]   = (sign_ff & (dividend_neg_ff ^ divisor_neg_ff)) ? q_ff_comp[pt.XLEN-1:0] : q_ff[pt.XLEN-1:0];

   assign a_ff_eff[pt.XLEN-1:0]   = (sign_ff &  dividend_neg_ff) ? a_ff_comp[pt.XLEN-1:0] : a_ff[pt.XLEN-1:0];

   assign data_out[pt.XLEN-1:0]   = ({pt.XLEN{ smallnum_case_wb          }} & {{pt.XLEN-4{1'b0}}, smallnum_ff[3:0]}) |
                                    ({pt.XLEN{                     rem_ff}} &  a_ff_eff[pt.XLEN-1:0]          ) |
                                    ({pt.XLEN{~smallnum_case_wb & ~rem_ff}} &  q_ff_eff[pt.XLEN-1:0]          );




endmodule // eh2_exu_div_existing_1bit_cheapshortq






// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
module eh2_exu_div_new_1bit_fullshortq
#(
`include "eh2_param.vh"
)
  (
   input  logic               clk,                       // Top level clock
   input  logic               rst_l,                     // Reset
   input  logic               scan_mode,                 // Scan mode

   input  logic               cancel,                    // Flush pipeline
   input  logic               valid_in,
   input  logic               signed_in,
   input  logic               rem_in,
   input  logic [pt.XLEN-1:0] dividend_in,
   input  logic [pt.XLEN-1:0] divisor_in,

   output logic               valid_out,
   output logic [pt.XLEN-1:0] data_out
  );


   logic                   valid_ff_in, valid_ff;
   logic                   finish_raw, finish, finish_ff;
   logic                   running_state;
   logic                   misc_enable;
   logic [2:0]             control_in, control_ff;
   logic                   dividend_sign_ff, divisor_sign_ff, rem_ff;
   logic                   count_enable;
   logic [pt.XLENW+1:0]      count_in, count_ff;

   logic                   smallnum_case;
   logic [3:0]             smallnum;

   logic                   a_enable, a_shift;
   logic [pt.XLEN-1:0]     a_in, a_ff;

   logic                   b_enable, b_twos_comp;
   logic [pt.XLEN:0]       b_in, b_ff;

   logic [pt.XLEN-1:0]     q_in, q_ff;

   logic                   rq_enable, r_sign_sel, r_restore_sel, r_adder_sel;
   logic [pt.XLEN-1:0]     r_in, r_ff;

   logic                   twos_comp_q_sel, twos_comp_b_sel;
   logic [pt.XLEN-1:0]     twos_comp_in, twos_comp_out;

   logic                   quotient_set;
   logic [pt.XLEN:0]       adder_out;

   logic [(2*pt.XLEN)-1:0] ar_shifted;
   logic [5:0]             shortq;
   logic [4:0]             shortq_shift;
   logic [4:0]             shortq_shift_ff;
   logic                   shortq_neg_or_zero;
   logic                   shortq_enable;
   logic                   shortq_enable_ff;
   logic [pt.XLEN:0]       shortq_dividend;

   logic                   by_zero_case;

   logic [4:1]             special_in;
   logic [4:1]             special_ff;



   rvdffe #(16+pt.XLENW+1) i_misc_ff  (.*, .clk(clk), .en(misc_enable),  .din ({valid_ff_in, control_in[2:0], count_in[pt.XLENW+1:0], special_in[4:1], shortq_enable,    shortq_shift[4:0],    finish   }),
                                                                         .dout({valid_ff,    control_ff[2:0], count_ff[pt.XLENW+1:0], special_ff[4:1], shortq_enable_ff, shortq_shift_ff[4:0], finish_ff}) );

   rvdffe #(pt.XLEN)     i_a_ff    (.*, .clk(clk), .en(a_enable),     .din(a_in[pt.XLEN-1:0]),    .dout(a_ff[pt.XLEN-1:0]));
   rvdffe #(pt.XLEN+1)   i_b_ff    (.*, .clk(clk), .en(b_enable),     .din(b_in[pt.XLEN:0]),      .dout(b_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN)     i_r_ff    (.*, .clk(clk), .en(rq_enable),    .din(r_in[pt.XLEN-1:0]),    .dout(r_ff[pt.XLEN-1:0]));
   rvdffe #(pt.XLEN)     i_q_ff    (.*, .clk(clk), .en(rq_enable),    .din(q_in[pt.XLEN-1:0]),    .dout(q_ff[pt.XLEN-1:0]));



   assign special_in[4:1]        = {special_ff[3] & ~cancel,
                                    special_ff[2] & ~cancel,
                                    special_ff[1] & ~cancel,
                                    (smallnum_case | by_zero_case) & ~cancel};

   assign valid_ff_in            =  valid_in  & ~cancel;

   assign control_in[2]          = (~valid_in & control_ff[2]) | (valid_in & signed_in  & dividend_in[pt.XLEN-1]);
   assign control_in[1]          = (~valid_in & control_ff[1]) | (valid_in & signed_in  &  divisor_in[pt.XLEN-1]);
   assign control_in[0]          = (~valid_in & control_ff[0]) | (valid_in & rem_in);

   assign dividend_sign_ff       =  control_ff[2];
   assign divisor_sign_ff        =  control_ff[1];
   assign rem_ff                 =  control_ff[0];


   assign by_zero_case           =  valid_ff & (b_ff[pt.XLEN-1:0] == {pt.XLEN{1'b0}});

   assign misc_enable            =  valid_in | valid_ff | cancel | running_state | finish_ff;
   assign running_state          = (| count_ff[pt.XLENW+1:0]) | shortq_enable_ff;
   assign finish_raw             =   special_ff[3] |
                                    (count_ff[pt.XLENW+1:0] == (pt.XLENW+2)'(pt.XLEN));


   assign finish                 =  finish_raw & ~cancel;
   assign count_enable           = (valid_ff | running_state) & ~finish & ~finish_ff & ~cancel & ~shortq_enable;
   assign count_in[pt.XLENW+1:0]      = {pt.XLENW+2{count_enable}} & (count_ff[pt.XLENW+1:0] + {{pt.XLENW+1{1'b0}},1'b1} + {2'b0,shortq_shift_ff[4:0]});


   assign a_enable               =  valid_in | running_state;
   assign a_shift                =  running_state & ~shortq_enable_ff;

   assign ar_shifted[(2*pt.XLEN)-1:0] = { {pt.XLEN{dividend_sign_ff}} , a_ff[pt.XLEN-1:0]} << shortq_shift_ff[4:0];

   assign a_in[pt.XLEN-1:0]      = ( {pt.XLEN{~a_shift & ~shortq_enable_ff}} &  dividend_in[pt.XLEN-1:0] ) |
                                   ( {pt.XLEN{ a_shift                    }} & {a_ff[pt.XLEN-2:0],1'b0}  ) |
                                   ( {pt.XLEN{            shortq_enable_ff}} &  ar_shifted[pt.XLEN-1:0]  );



   assign b_enable               =    valid_in | b_twos_comp;
   assign b_twos_comp            =    valid_ff & ~(dividend_sign_ff ^ divisor_sign_ff);

   assign b_in[pt.XLEN:0]        = ( {pt.XLEN+1{~b_twos_comp}} & { (signed_in & divisor_in[pt.XLEN-1]),divisor_in[pt.XLEN-1:0] } ) |
                                   ( {pt.XLEN+1{ b_twos_comp}} & {~divisor_sign_ff,twos_comp_out[pt.XLEN-1:0] } );


   assign rq_enable              = (valid_in | valid_ff | running_state) & ~(| special_ff[3:1]);
   assign r_sign_sel             =  valid_ff      &  dividend_sign_ff & ~by_zero_case;
   assign r_restore_sel          =  running_state & ~quotient_set & ~shortq_enable_ff;
   assign r_adder_sel            =  running_state &  quotient_set & ~shortq_enable_ff;


   assign r_in[pt.XLEN-1:0]      = ( {pt.XLEN{r_sign_sel      }} & {pt.XLEN{1'b1}}                      ) |
                                   ( {pt.XLEN{r_restore_sel   }} & {r_ff[pt.XLEN-2:0], a_ff[pt.XLEN-1]} ) |
                                   ( {pt.XLEN{r_adder_sel     }} &  adder_out[pt.XLEN-1:0]              ) |
                                   ( {pt.XLEN{shortq_enable_ff}} &  ar_shifted[(2*pt.XLEN)-1:pt.XLEN]   ) |
                                   ( {pt.XLEN{by_zero_case    }} &  a_ff[pt.XLEN-1:0]                   );


   assign q_in[pt.XLEN-1:0]      = ( {pt.XLEN{~valid_ff       }} & {q_ff[pt.XLEN-2:0], quotient_set}  ) |
                                   ( {pt.XLEN{ smallnum_case  }} & {{pt.XLEN-4{1'b0}}, smallnum[3:0]} ) |
                                   ( {pt.XLEN{ by_zero_case   }} & {pt.XLEN{1'b1}}                    );



   assign adder_out[pt.XLEN:0]   = {r_ff[pt.XLEN-1:0],a_ff[pt.XLEN-1]} + {b_ff[pt.XLEN:0] };


   assign quotient_set           = (~adder_out[pt.XLEN] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-2:0] == {pt.XLEN-1{1'b0}}) & (adder_out[pt.XLEN:0] == {pt.XLEN+1{1'b0}}) );



   assign twos_comp_b_sel        =  valid_ff           & ~(dividend_sign_ff ^ divisor_sign_ff);
   assign twos_comp_q_sel        = ~valid_ff & ~rem_ff &  (dividend_sign_ff ^ divisor_sign_ff) & ~special_ff[4];

   assign twos_comp_in[pt.XLEN-1:0] = ( {pt.XLEN{twos_comp_q_sel}} & q_ff[pt.XLEN-1:0] ) |
                                      ( {pt.XLEN{twos_comp_b_sel}} & b_ff[pt.XLEN-1:0] );

   rvtwoscomp #(pt.XLEN) i_twos_comp  (.din(twos_comp_in[pt.XLEN-1:0]), .dout(twos_comp_out[pt.XLEN-1:0]));



   assign valid_out              =  finish_ff & ~cancel;

   assign data_out[pt.XLEN-1:0]  = ( {pt.XLEN{~rem_ff & ~twos_comp_q_sel}} & q_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{ rem_ff                   }} & r_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{           twos_comp_q_sel}} & twos_comp_out[pt.XLEN-1:0] );




   // *** *** *** START : SMALLNUM {{

   assign smallnum_case          = ( (a_ff[pt.XLEN-1:4] == {pt.XLEN-4{1'b0}}) & (b_ff[pt.XLEN-1:4] == {pt.XLEN-4{1'b0}}) & ~by_zero_case & ~rem_ff & valid_ff & ~cancel) |
                                   ( (a_ff[pt.XLEN-1:0] == {pt.XLEN{1'b0}}  ) &                                            ~by_zero_case & ~rem_ff & valid_ff & ~cancel);

   assign smallnum[3]            = ( a_ff[3] &                                  ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           );

   assign smallnum[2]            = ( a_ff[3] &                                  ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (            a_ff[2] &                       ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] & ~b_ff[2]                      );

   assign smallnum[1]            = (            a_ff[2] &                       ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (                       a_ff[1] &            ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &                                  ~b_ff[3] &            ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] & ~a_ff[2] &                       ~b_ff[3] & ~b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] &  b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &             a_ff[1] &            ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &  b_ff[2]                      );

   assign smallnum[0]            = (            a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] &             a_ff[0] & ~b_ff[3] &             b_ff[1] &  b_ff[0]) |
                                   (            a_ff[2] &                       ~b_ff[3] &            ~b_ff[1] & ~b_ff[0]) |
                                   (                       a_ff[1] &            ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (                                  a_ff[0] & ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] & ~a_ff[1] &            ~b_ff[3] & ~b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &                                             ~b_ff[2] & ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] & ~a_ff[2] &                       ~b_ff[3] &  b_ff[2] &  b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &  b_ff[2] & ~b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] &             a_ff[0] & ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] & ~a_ff[1] &            ~b_ff[3] &  b_ff[2] &             b_ff[0]) |
                                   (           ~a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                                             ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] &             a_ff[1] &                       ~b_ff[2] &            ~b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] &  b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                        b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &             a_ff[1] &             b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &                        a_ff[0] &            ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &            ~a_ff[1] &            ~b_ff[3] &  b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &             b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &             b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &             a_ff[0] &  b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] &  a_ff[1] &            ~b_ff[3] &             b_ff[1]           ) |
                                   ( a_ff[3] &             a_ff[1] &  a_ff[0] &            ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &  a_ff[0] &  b_ff[3]                                 );

   // *** *** *** END   : SMALLNUM }}




   // *** *** *** Start : Short Q {{

   assign shortq_dividend[pt.XLEN:0] = {dividend_sign_ff,a_ff[pt.XLEN-1:0]};


   parameter shortq_a_width = pt.XLEN + 1;
   parameter shortq_b_width = pt.XLEN + 1;

   logic [5:0]  dw_a_enc;
   logic [5:0]  dw_b_enc;
   logic [6:0]  dw_shortq_raw;


   eh2_exu_div_cls i_a_cls  (
       .operand  ( shortq_dividend[pt.XLEN:0]  ),
       .cls      ( dw_a_enc[4:0]               ));

   eh2_exu_div_cls i_b_cls  (
       .operand  ( b_ff[pt.XLEN:0]        ),
       .cls      ( dw_b_enc[4:0]          ));

   assign dw_a_enc[5]             =  1'b0;
   assign dw_b_enc[5]             =  1'b0;


   assign dw_shortq_raw[6:0]      =  {1'b0,dw_b_enc[5:0]} - {1'b0,dw_a_enc[5:0]} + 7'd1;
   assign shortq_neg_or_zero      =  dw_shortq_raw[6] | (dw_shortq_raw[5:0] == 6'b0);
   assign shortq[5:0]             =  shortq_neg_or_zero  ?  6'd1  :  dw_shortq_raw[5:0];   // 1 is minimum SHORTQ otherwise WB too early

   assign shortq_enable           =  valid_ff & ~shortq[5] & ~(shortq[4:1] ==  4'b1111) & ~cancel;

   assign shortq_shift[4:0]       = ~shortq_enable     ?  5'd0  :  (5'b11111 - shortq[4:0]);

   // *** *** *** End   : Short Q }}





endmodule // eh2_exu_div_new_1bit_fullshortq






// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
module eh2_exu_div_new_2bit_fullshortq
  (
   input  logic               clk,                       // Top level clock
   input  logic               rst_l,                     // Reset
   input  logic               scan_mode,                 // Scan mode

   input  logic               cancel,                    // Flush pipeline
   input  logic               valid_in,
   input  logic               signed_in,
   input  logic               rem_in,
   input  logic [pt.XLEN-1:0] dividend_in,
   input  logic [pt.XLEN-1:0] divisor_in,

   output logic               valid_out,
   output logic [pt.XLEN-1:0] data_out
  );


   logic                   valid_ff_in, valid_ff;
   logic                   finish_raw, finish, finish_ff;
   logic                   running_state;
   logic                   misc_enable;
   logic [2:0]             control_in, control_ff;
   logic                   dividend_sign_ff, divisor_sign_ff, rem_ff;
   logic                   count_enable;
   logic [pt.XLENW+1:0]      count_in, count_ff;

   logic                   smallnum_case;
   logic [3:0]             smallnum;

   logic                   a_enable, a_shift;
   logic [pt.XLEN-1:0]     a_in, a_ff;

   logic                   b_enable, b_twos_comp;
   logic [pt.XLEN:0]       b_in;
   logic [pt.XLEN+2:0]     b_ff;

   logic [pt.XLEN-1:0]     q_in, q_ff;

   logic                   rq_enable, r_sign_sel, r_restore_sel, r_adder1_sel, r_adder2_sel, r_adder3_sel;
   logic [pt.XLEN-1:0]     r_in, r_ff;

   logic                   twos_comp_q_sel, twos_comp_b_sel;
   logic [pt.XLEN-1:0]     twos_comp_in, twos_comp_out;

   logic [3:1]             quotient_raw;
   logic [1:0]             quotient_new;
   logic [pt.XLEN:0]       adder1_out;
   logic [pt.XLEN+1:0]     adder2_out;
   logic [pt.XLEN+2:0]     adder3_out;

   logic [(2*pt.XLEN)-1:0] ar_shifted;
   logic [5:0]             shortq;
   logic [4:0]             shortq_shift;
   logic [4:0]             shortq_shift_ff;
   logic                   shortq_neg_or_zero;
   logic                   shortq_enable;
   logic                   shortq_enable_ff;
   logic [pt.XLEN:0]       shortq_dividend;

   logic                   by_zero_case;

   logic [4:1]             special_in;
   logic [4:1]             special_ff;



   rvdffe #(15+pt.XLENW+1) i_misc_ff  (.*, .clk(clk), .en(misc_enable),  .din ({valid_ff_in, control_in[2:0], count_in[pt.XLENW+1:0], special_in[4:1], shortq_enable,    shortq_shift[4:1],    finish   }),
                                                                       .dout({valid_ff,    control_ff[2:0], count_ff[pt.XLENW+1:0], special_ff[4:1], shortq_enable_ff, shortq_shift_ff[4:1], finish_ff}) );

   rvdffe #(pt.XLEN)     i_a_ff    (.*, .clk(clk), .en(a_enable),     .din(a_in[pt.XLEN-1:0]),    .dout(a_ff[pt.XLEN-1:0]));
   rvdffe #(pt.XLEN+1)   i_b_ff    (.*, .clk(clk), .en(b_enable),     .din(b_in[pt.XLEN:0]),      .dout(b_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN)     i_r_ff    (.*, .clk(clk), .en(rq_enable),    .din(r_in[pt.XLEN-1:0]),    .dout(r_ff[pt.XLEN-1:0]));
   rvdffe #(pt.XLEN)     i_q_ff    (.*, .clk(clk), .en(rq_enable),    .din(q_in[pt.XLEN-1:0]),    .dout(q_ff[pt.XLEN-1:0]));



   assign special_in[4:1]        = {special_ff[3] & ~cancel,
                                    special_ff[2] & ~cancel,
                                    special_ff[1] & ~cancel,
                                    (smallnum_case | by_zero_case) & ~cancel};

   assign valid_ff_in            =  valid_in  & ~cancel;

   assign control_in[2]          = (~valid_in & control_ff[2]) | (valid_in & signed_in  & dividend_in[pt.XLEN-1]);
   assign control_in[1]          = (~valid_in & control_ff[1]) | (valid_in & signed_in  &  divisor_in[pt.XLEN-1]);
   assign control_in[0]          = (~valid_in & control_ff[0]) | (valid_in & rem_in);

   assign dividend_sign_ff       =  control_ff[2];
   assign divisor_sign_ff        =  control_ff[1];
   assign rem_ff                 =  control_ff[0];


   assign by_zero_case           =  valid_ff & (b_ff[pt.XLEN-1:0] == {pt.XLEN{1'b0}});

   assign misc_enable            =  valid_in | valid_ff | cancel | running_state | finish_ff;
   assign running_state          = (| count_ff[pt.XLENW+1:0]) | shortq_enable_ff;
   assign finish_raw             =   special_ff[3] |
                                    (count_ff[pt.XLENW+1:0] == (pt.XLENW+2)'(pt.XLEN));


   assign finish                 =  finish_raw & ~cancel;
   assign count_enable           = (valid_ff | running_state) & ~finish & ~finish_ff & ~cancel & ~shortq_enable;
   assign count_in[pt.XLENW+1:0]   = {pt.XLENW+2{count_enable}} & (count_ff[pt.XLENW+1:0] + {{pt.XLENW{1'b0}},2'b10} + {2'b0,shortq_shift_ff[4:1], 1'b0});


   assign a_enable               =  valid_in | running_state;
   assign a_shift                =  running_state & ~shortq_enable_ff;

   assign ar_shifted[(2*pt.XLEN)-1:0] = { {pt.XLEN{dividend_sign_ff}} , a_ff[pt.XLEN-1:0]} << {shortq_shift_ff[4:1],1'b0};

   assign a_in[pt.XLEN-1:0]      = ( {pt.XLEN{~a_shift & ~shortq_enable_ff}} &  dividend_in[pt.XLEN-1:0] ) |
                                   ( {pt.XLEN{ a_shift                    }} & {a_ff[pt.XLEN-3:0],2'b0}  ) |
                                   ( {pt.XLEN{            shortq_enable_ff}} &  ar_shifted[pt.XLEN-1:0]  );



   assign b_enable               =    valid_in | b_twos_comp;
   assign b_twos_comp            =    valid_ff & ~(dividend_sign_ff ^ divisor_sign_ff);

   assign b_in[pt.XLEN:0]        = ( {pt.XLEN+1{~b_twos_comp}} & { (signed_in & divisor_in[pt.XLEN-1]),divisor_in[pt.XLEN-1:0] } ) |
                                   ( {pt.XLEN+1{ b_twos_comp}} & {~divisor_sign_ff,twos_comp_out[pt.XLEN-1:0] } );


   assign rq_enable              = (valid_in | valid_ff | running_state) & ~(| special_ff[3:1]);
   assign r_sign_sel             =  valid_ff      &  dividend_sign_ff & ~by_zero_case;
   assign r_restore_sel          =  running_state & (quotient_new[1:0] == 2'b00) & ~shortq_enable_ff;
   assign r_adder1_sel           =  running_state & (quotient_new[1:0] == 2'b01) & ~shortq_enable_ff;
   assign r_adder2_sel           =  running_state & (quotient_new[1:0] == 2'b10) & ~shortq_enable_ff;
   assign r_adder3_sel           =  running_state & (quotient_new[1:0] == 2'b11) & ~shortq_enable_ff;


   assign r_in[pt.XLEN-1:0]      = ( {pt.XLEN{r_sign_sel      }} & {pt.XLEN{1'b1}}                                ) |
                                   ( {pt.XLEN{r_restore_sel   }} & {r_ff[pt.XLEN-3:0] ,a_ff[pt.XLEN-1:pt.XLEN-2]} ) |
                                   ( {pt.XLEN{r_adder1_sel    }} &  adder1_out[pt.XLEN-1:0]                       ) |
                                   ( {pt.XLEN{r_adder2_sel    }} &  adder2_out[pt.XLEN-1:0]                       ) |
                                   ( {pt.XLEN{r_adder3_sel    }} &  adder3_out[pt.XLEN-1:0]                       ) |
                                   ( {pt.XLEN{shortq_enable_ff}} &  ar_shifted[(2*pt.XLEN)-1:pt.XLEN]             ) |
                                   ( {pt.XLEN{by_zero_case    }} &  a_ff[pt.XLEN-1:0]                             );


   assign q_in[pt.XLEN-1:0]      = ( {pt.XLEN{~valid_ff       }} & {q_ff[pt.XLEN-3:0], quotient_new[1:0]}  ) |
                                   ( {pt.XLEN{ smallnum_case  }} & {{pt.XLEN-4{1'b0}}, smallnum[3:0]}      ) |
                                   ( {pt.XLEN{ by_zero_case   }} & {pt.XLEN{1'b1}}                         );


   assign b_ff[pt.XLEN+2:pt.XLEN+1] = {b_ff[pt.XLEN],b_ff[pt.XLEN]};


   assign adder1_out[pt.XLEN:0]   = {                r_ff[pt.XLEN-2:0],a_ff[pt.XLEN-1:pt.XLEN-2]}  +  b_ff[pt.XLEN:0];
   assign adder2_out[pt.XLEN+1:0] = {                r_ff[pt.XLEN-1:0],a_ff[pt.XLEN-1:pt.XLEN-2]}  + {b_ff[pt.XLEN:0],1'b0};
   assign adder3_out[pt.XLEN+2:0] = {r_ff[pt.XLEN-1],r_ff[pt.XLEN-1:0],a_ff[pt.XLEN-1:pt.XLEN-2]}  + {b_ff[pt.XLEN+1:0],1'b0}  +  b_ff[pt.XLEN+2:0];


   assign quotient_raw[1]        = (~adder1_out[pt.XLEN]   ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder1_out[pt.XLEN:0]   == {pt.XLEN+1{1'b0}}) );
   assign quotient_raw[2]        = (~adder2_out[pt.XLEN+1] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder2_out[pt.XLEN+1:0] == {pt.XLEN+2{1'b0}}) );
   assign quotient_raw[3]        = (~adder3_out[pt.XLEN+2] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder3_out[pt.XLEN+2:0] == {pt.XLEN+3{1'b0}}) );

   assign quotient_new[1]        = quotient_raw[3] |  quotient_raw[2];
   assign quotient_new[0]        = quotient_raw[3] |(~quotient_raw[2] & quotient_raw[1]);


   assign twos_comp_b_sel        =  valid_ff           & ~(dividend_sign_ff ^ divisor_sign_ff);
   assign twos_comp_q_sel        = ~valid_ff & ~rem_ff &  (dividend_sign_ff ^ divisor_sign_ff) & ~special_ff[4];

   assign twos_comp_in[pt.XLEN-1:0] = ( {pt.XLEN{twos_comp_q_sel}} & q_ff[pt.XLEN-1:0] ) |
                                      ( {pt.XLEN{twos_comp_b_sel}} & b_ff[pt.XLEN-1:0] );

   rvtwoscomp #(pt.XLEN) i_twos_comp  (.din(twos_comp_in[pt.XLEN-1:0]), .dout(twos_comp_out[pt.XLEN-1:0]));



   assign valid_out              =  finish_ff & ~cancel;

   assign data_out[pt.XLEN-1:0]  = ( {pt.XLEN{~rem_ff & ~twos_comp_q_sel}} & q_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{ rem_ff                   }} & r_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{           twos_comp_q_sel}} & twos_comp_out[pt.XLEN-1:0] );




   // *** *** *** START : SMALLNUM {{

   assign smallnum_case          = ( (a_ff[pt.XLEN-1:4]  == {pt.XLEN-4{1'b0}}) & (b_ff[pt.XLEN-1:4] == {pt.XLEN-4{1'b0}}) & ~by_zero_case & ~rem_ff & valid_ff & ~cancel) |
                                   ( (a_ff[pt.XLEN-1:0]  == {pt.XLEN{1'b0}})   &                                            ~by_zero_case & ~rem_ff & valid_ff & ~cancel);

   assign smallnum[3]            = ( a_ff[3] &                                  ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           );

   assign smallnum[2]            = ( a_ff[3] &                                  ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (            a_ff[2] &                       ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] & ~b_ff[2]                      );

   assign smallnum[1]            = (            a_ff[2] &                       ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (                       a_ff[1] &            ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &                                  ~b_ff[3] &            ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] & ~a_ff[2] &                       ~b_ff[3] & ~b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] &  b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &             a_ff[1] &            ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &  b_ff[2]                      );

   assign smallnum[0]            = (            a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] &             a_ff[0] & ~b_ff[3] &             b_ff[1] &  b_ff[0]) |
                                   (            a_ff[2] &                       ~b_ff[3] &            ~b_ff[1] & ~b_ff[0]) |
                                   (                       a_ff[1] &            ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (                                  a_ff[0] & ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] & ~a_ff[1] &            ~b_ff[3] & ~b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &                                             ~b_ff[2] & ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] & ~a_ff[2] &                       ~b_ff[3] &  b_ff[2] &  b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &  b_ff[2] & ~b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] &             a_ff[0] & ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] & ~a_ff[1] &            ~b_ff[3] &  b_ff[2] &             b_ff[0]) |
                                   (           ~a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                                             ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] &             a_ff[1] &                       ~b_ff[2] &            ~b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] &  b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                        b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &             a_ff[1] &             b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &                        a_ff[0] &            ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &            ~a_ff[1] &            ~b_ff[3] &  b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &             b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &             b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &             a_ff[0] &  b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] &  a_ff[1] &            ~b_ff[3] &             b_ff[1]           ) |
                                   ( a_ff[3] &             a_ff[1] &  a_ff[0] &            ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &  a_ff[0] &  b_ff[3]                                 );

   // *** *** *** END   : SMALLNUM }}




   // *** *** *** Start : Short Q {{

   assign shortq_dividend[pt.XLEN:0]   = {dividend_sign_ff,a_ff[pt.XLEN-1:0]};


   parameter shortq_a_width = pt.XLEN + 1;
   parameter shortq_b_width = pt.XLEN + 1;

   logic [5:0]  dw_a_enc;
   logic [5:0]  dw_b_enc;
   logic [6:0]  dw_shortq_raw;


   eh2_exu_div_cls i_a_cls  (
       .operand  ( shortq_dividend[pt.XLEN:0]  ),
       .cls      ( dw_a_enc[4:0]               ));

   eh2_exu_div_cls i_b_cls  (
       .operand  ( b_ff[pt.XLEN:0]             ),
       .cls      ( dw_b_enc[4:0]               ));

   assign dw_a_enc[5]             =  1'b0;
   assign dw_b_enc[5]             =  1'b0;


   assign dw_shortq_raw[6:0]      =  {1'b0,dw_b_enc[5:0]} - {1'b0,dw_a_enc[5:0]} + 7'd1;
   assign shortq_neg_or_zero      =  dw_shortq_raw[6] | (dw_shortq_raw[5:1] == 5'b0);      // Also includes 1
   assign shortq[5:0]             =  shortq_neg_or_zero  ?  6'd2  :  dw_shortq_raw[5:0];   // 2 is minimum SHORTQ otherwise WB too early

   assign shortq_enable           =  valid_ff & ~shortq[5] & ~(shortq[4:1] ==  4'b1111) & ~cancel;

   assign shortq_shift[4:0]       = ~shortq_enable     ?  5'd0  :  (5'b11111 - shortq[4:0]);   // [0] is unused

   // *** *** *** End   : Short Q }}





endmodule // eh2_exu_div_new_2bit_fullshortq






// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
module eh2_exu_div_new_3bit_fullshortq
#(
`include "eh2_param.vh"
)
  (
   input  logic               clk,                       // Top level clock
   input  logic               rst_l,                     // Reset
   input  logic               scan_mode,                 // Scan mode

   input  logic               cancel,                    // Flush pipeline
   input  logic               valid_in,
   input  logic               signed_in,
   input  logic               rem_in,
   input  logic [pt.XLEN-1:0] dividend_in,
   input  logic [pt.XLEN-1:0] divisor_in,

   output logic               valid_out,
   output logic [pt.XLEN-1:0] data_out
  );


   logic                   valid_ff_in, valid_ff;
   logic                   finish_raw, finish, finish_ff;
   logic                   running_state;
   logic                   misc_enable;
   logic [2:0]             control_in, control_ff;
   logic                   dividend_sign_ff, divisor_sign_ff, rem_ff;
   logic                   count_enable;
   logic [pt.XLENW+1:0]      count_in, count_ff;

   logic                   smallnum_case;
   logic [3:0]             smallnum;

   logic                   a_enable, a_shift;
   logic [pt.XLEN:0]       a_in, a_ff;

   logic                   b_enable, b_twos_comp;
   logic [pt.XLEN:0]       b_in;
   logic [pt.XLEN+4:0]     b_ff;

   logic [pt.XLEN-1:0]     q_in, q_ff;

   logic                   rq_enable;
   logic                   r_sign_sel;
   logic                   r_restore_sel;
   logic                   r_adder1_sel, r_adder2_sel, r_adder3_sel, r_adder4_sel, r_adder5_sel, r_adder6_sel, r_adder7_sel;
   logic [pt.XLEN:0]       r_in, r_ff;

   logic                   twos_comp_q_sel, twos_comp_b_sel;
   logic [pt.XLEN-1:0]     twos_comp_in, twos_comp_out;

   logic [7:1]             quotient_raw;
   logic [2:0]             quotient_new;
   logic [pt.XLEN+1:0]     adder1_out;
   logic [pt.XLEN+2:0]     adder2_out;
   logic [pt.XLEN+3:0]     adder3_out;
   logic [pt.XLEN+4:0]     adder4_out;
   logic [pt.XLEN+4:0]     adder5_out;
   logic [pt.XLEN+4:0]     adder6_out;
   logic [pt.XLEN+4:0]     adder7_out;

   logic [(2*pt.XLEN)+1:0] ar_shifted;
   logic [5:0]             shortq;
   logic [4:0]             shortq_shift;
   logic [4:0]             shortq_decode;
   logic [4:0]             shortq_shift_ff;
   logic                   shortq_enable;
   logic                   shortq_enable_ff;
   logic [pt.XLEN:0]       shortq_dividend;

   logic                   by_zero_case;

   logic         [4:1]     special_in;
   logic         [4:1]     special_ff;



   rvdffe #(16+pt.XLENW+1) i_misc_ff  (.*, .clk(clk), .en(misc_enable),  .din ({valid_ff_in, control_in[2:0], count_in[pt.XLENW+1:0], special_in[4:1], shortq_enable,    shortq_shift[4:0],    finish   }),
                                                                       .dout({valid_ff,    control_ff[2:0], count_ff[pt.XLENW+1:0], special_ff[4:1], shortq_enable_ff, shortq_shift_ff[4:0], finish_ff}) );

   rvdffe #(pt.XLEN+1)   i_a_ff    (.*, .clk(clk), .en(a_enable),     .din(a_in[pt.XLEN:0]),    .dout(a_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN+1)   i_b_ff    (.*, .clk(clk), .en(b_enable),     .din(b_in[pt.XLEN:0]),    .dout(b_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN+1)   i_r_ff    (.*, .clk(clk), .en(rq_enable),    .din(r_in[pt.XLEN:0]),    .dout(r_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN)     i_q_ff    (.*, .clk(clk), .en(rq_enable),    .din(q_in[pt.XLEN-1:0]),  .dout(q_ff[pt.XLEN-1:0]));



   assign special_in[4:1]        = {special_ff[3] & ~cancel,
                                    special_ff[2] & ~cancel,
                                    special_ff[1] & ~cancel,
                                    (smallnum_case | by_zero_case) & ~cancel};

   assign valid_ff_in            =  valid_in  & ~cancel;

   assign control_in[2]          = (~valid_in & control_ff[2]) | (valid_in & signed_in  & dividend_in[pt.XLEN-1]);
   assign control_in[1]          = (~valid_in & control_ff[1]) | (valid_in & signed_in  &  divisor_in[pt.XLEN-1]);
   assign control_in[0]          = (~valid_in & control_ff[0]) | (valid_in & rem_in);

   assign dividend_sign_ff       =  control_ff[2];
   assign divisor_sign_ff        =  control_ff[1];
   assign rem_ff                 =  control_ff[0];


   assign by_zero_case           =  valid_ff & (b_ff[pt.XLEN-1:0] == {pt.XLEN{1'b0}});

   assign misc_enable            =  valid_in | valid_ff | cancel | running_state | finish_ff;
   assign running_state          = (| count_ff[pt.XLENW+1:0]) | shortq_enable_ff;
   assign finish_raw             =   special_ff[3] |
                                    (count_ff[pt.XLENW+1:0] == (pt.XLENW+2)'(pt.XLEN+1));


   assign finish                 =  finish_raw & ~cancel;
   assign count_enable           = (valid_ff | running_state) & ~finish & ~finish_ff & ~cancel & ~shortq_enable;
   assign count_in[pt.XLENW+1:0]   = {pt.XLENW+2{count_enable}} & (count_ff[pt.XLENW+1:0] + {{pt.XLENW{1'b0}},2'b11} + {2'b0,shortq_shift_ff[4:0]});


   assign a_enable               =  valid_in | running_state;
   assign a_shift                =  running_state & ~shortq_enable_ff;

   assign ar_shifted[(2*pt.XLEN)+1:0] = { {pt.XLEN+1{dividend_sign_ff}} , a_ff[pt.XLEN:0]} << {shortq_shift_ff[4:0]};

   assign a_in[pt.XLEN:0]        = ( {pt.XLEN+1{~a_shift & ~shortq_enable_ff}} & {signed_in & dividend_in[pt.XLEN-1], dividend_in[pt.XLEN-1:0]} ) |
                                   ( {pt.XLEN+1{ a_shift                    }} & {a_ff[pt.XLEN-3:0], 3'b0}  ) |
                                   ( {pt.XLEN+1{            shortq_enable_ff}} &  ar_shifted[pt.XLEN:0]     );



   assign b_enable               =    valid_in | b_twos_comp;
   assign b_twos_comp            =    valid_ff & ~(dividend_sign_ff ^ divisor_sign_ff);

   assign b_in[pt.XLEN:0]        = ( {pt.XLEN+1{~b_twos_comp}} & { (signed_in & divisor_in[pt.XLEN-1]),divisor_in[pt.XLEN-1:0] } ) |
                                   ( {pt.XLEN+1{ b_twos_comp}} & {~divisor_sign_ff,twos_comp_out[pt.XLEN-1:0] } );


   assign rq_enable              = (valid_in | valid_ff | running_state) & ~(| special_ff[3:1]);
   assign r_sign_sel             =  valid_ff      &  dividend_sign_ff & ~by_zero_case;
   assign r_restore_sel          =  running_state & (quotient_new[2:0] == 3'b000) & ~shortq_enable_ff;
   assign r_adder1_sel           =  running_state & (quotient_new[2:0] == 3'b001) & ~shortq_enable_ff;
   assign r_adder2_sel           =  running_state & (quotient_new[2:0] == 3'b010) & ~shortq_enable_ff;
   assign r_adder3_sel           =  running_state & (quotient_new[2:0] == 3'b011) & ~shortq_enable_ff;
   assign r_adder4_sel           =  running_state & (quotient_new[2:0] == 3'b100) & ~shortq_enable_ff;
   assign r_adder5_sel           =  running_state & (quotient_new[2:0] == 3'b101) & ~shortq_enable_ff;
   assign r_adder6_sel           =  running_state & (quotient_new[2:0] == 3'b110) & ~shortq_enable_ff;
   assign r_adder7_sel           =  running_state & (quotient_new[2:0] == 3'b111) & ~shortq_enable_ff;


   assign r_in[pt.XLEN:0]        = ( {pt.XLEN+1{r_sign_sel      }} & {pt.XLEN+1{1'b1}}                            ) |
                                   ( {pt.XLEN+1{r_restore_sel   }} & {r_ff[pt.XLEN-3:0] ,a_ff[pt.XLEN:pt.XLEN-2]} ) |
                                   ( {pt.XLEN+1{r_adder1_sel    }} &  adder1_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder2_sel    }} &  adder2_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder3_sel    }} &  adder3_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder4_sel    }} &  adder4_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder5_sel    }} &  adder5_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder6_sel    }} &  adder6_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder7_sel    }} &  adder7_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{shortq_enable_ff}} &  ar_shifted[(2*pt.XLEN)+1:pt.XLEN+1]         ) |
                                   ( {pt.XLEN+1{by_zero_case    }} & {1'b0,a_ff[pt.XLEN-1:0]}                     );


   assign q_in[pt.XLEN-1:0]      = ( {pt.XLEN{~valid_ff       }} & {q_ff[pt.XLEN-4:0], quotient_new[2:0]}  ) |
                                   ( {pt.XLEN{ smallnum_case  }} & {{pt.XLEN-4{1'b0}}, smallnum[3:0]}      ) |
                                   ( {pt.XLEN{ by_zero_case   }} & {pt.XLEN{1'b1}}                         );


   assign b_ff[pt.XLEN+4:pt.XLEN+1] = {b_ff[pt.XLEN],b_ff[pt.XLEN],b_ff[pt.XLEN],b_ff[pt.XLEN]};


   assign adder1_out[pt.XLEN+1:0]       = {              r_ff[pt.XLEN-2:0],a_ff[pt.XLEN:pt.XLEN-2]}  +   b_ff[pt.XLEN+1:0];
   assign adder2_out[pt.XLEN+2:0]       = {              r_ff[pt.XLEN-1:0],a_ff[pt.XLEN:pt.XLEN-2]}  +  {b_ff[pt.XLEN+1:0],1'b0};
   assign adder3_out[pt.XLEN+3:0]       = {              r_ff[pt.XLEN:0],  a_ff[pt.XLEN:pt.XLEN-2]}  +  {b_ff[pt.XLEN+2:0],1'b0}  +   b_ff[pt.XLEN+3:0];
   assign adder4_out[pt.XLEN+4:0]       = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN:pt.XLEN-2]}  +  {b_ff[pt.XLEN+2:0],2'b0};
   assign adder5_out[pt.XLEN+4:0]       = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN:pt.XLEN-2]}  +  {b_ff[pt.XLEN+2:0],2'b0}  +   b_ff[pt.XLEN+4:0];
   assign adder6_out[pt.XLEN+4:0]       = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN:pt.XLEN-2]}  +  {b_ff[pt.XLEN+2:0],2'b0}  +  {b_ff[pt.XLEN+3:0],1'b0};
   assign adder7_out[pt.XLEN+4:0]       = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN:pt.XLEN-2]}  +  {b_ff[pt.XLEN+2:0],2'b0}  +  {b_ff[pt.XLEN+3:0],1'b0}  +  b_ff[pt.XLEN+4:0];

   assign quotient_raw[1]        = (~adder1_out[pt.XLEN+1] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder1_out[pt.XLEN+1:0] == {pt.XLEN+2{1'b0}}) );
   assign quotient_raw[2]        = (~adder2_out[pt.XLEN+2] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder2_out[pt.XLEN+2:0] == {pt.XLEN+3{1'b0}}) );
   assign quotient_raw[3]        = (~adder3_out[pt.XLEN+3] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder3_out[pt.XLEN+3:0] == {pt.XLEN+4{1'b0}}) );
   assign quotient_raw[4]        = (~adder4_out[pt.XLEN+4] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder4_out[pt.XLEN+4:0] == {pt.XLEN+5{1'b0}}) );
   assign quotient_raw[5]        = (~adder5_out[pt.XLEN+4] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder5_out[pt.XLEN+4:0] == {pt.XLEN+5{1'b0}}) );
   assign quotient_raw[6]        = (~adder6_out[pt.XLEN+4] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder6_out[pt.XLEN+4:0] == {pt.XLEN+5{1'b0}}) );
   assign quotient_raw[7]        = (~adder7_out[pt.XLEN+4] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder7_out[pt.XLEN+4:0] == {pt.XLEN+5{1'b0}}) );

   assign quotient_new[2]        = quotient_raw[7] |   quotient_raw[6] | quotient_raw[5]  |   quotient_raw[4];
   assign quotient_new[1]        = quotient_raw[7] |   quotient_raw[6] |                    (~quotient_raw[4] & quotient_raw[3]) | (~quotient_raw[3] & quotient_raw[2]);
   assign quotient_new[0]        = quotient_raw[7] | (~quotient_raw[6] & quotient_raw[5]) | (~quotient_raw[4] & quotient_raw[3]) | (~quotient_raw[2] & quotient_raw[1]);


   assign twos_comp_b_sel        =  valid_ff           & ~(dividend_sign_ff ^ divisor_sign_ff);
   assign twos_comp_q_sel        = ~valid_ff & ~rem_ff &  (dividend_sign_ff ^ divisor_sign_ff) & ~special_ff[4];

   assign twos_comp_in[pt.XLEN-1:0] = ( {pt.XLEN{twos_comp_q_sel}} & q_ff[pt.XLEN-1:0] ) |
                                      ( {pt.XLEN{twos_comp_b_sel}} & b_ff[pt.XLEN-1:0] );

   rvtwoscomp #(pt.XLEN) i_twos_comp  (.din(twos_comp_in[pt.XLEN-1:0]), .dout(twos_comp_out[pt.XLEN-1:0]));



   assign valid_out              =  finish_ff & ~cancel;

   assign data_out[pt.XLEN-1:0]  = ( {pt.XLEN{~rem_ff & ~twos_comp_q_sel}} & q_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{ rem_ff                   }} & r_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{           twos_comp_q_sel}} & twos_comp_out[pt.XLEN-1:0] );




   // *** *** *** START : SMALLNUM {{

   assign smallnum_case          = ( (a_ff[pt.XLEN-1:4]  == {pt.XLEN-4{1'b0}}) & (b_ff[pt.XLEN-1:4] == {pt.XLEN-4{1'b0}}) & ~by_zero_case & ~rem_ff & valid_ff & ~cancel & 1'b0) |
                                   ( (a_ff[pt.XLEN-1:0]  == {pt.XLEN{1'b0}})   &                                            ~by_zero_case & ~rem_ff & valid_ff & ~cancel & 1'b0);

   assign smallnum[3]            = ( a_ff[3] &                                  ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           );

   assign smallnum[2]            = ( a_ff[3] &                                  ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (            a_ff[2] &                       ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] & ~b_ff[2]                      );

   assign smallnum[1]            = (            a_ff[2] &                       ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (                       a_ff[1] &            ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &                                  ~b_ff[3] &            ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] & ~a_ff[2] &                       ~b_ff[3] & ~b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] &  b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &             a_ff[1] &            ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &  b_ff[2]                      );

   assign smallnum[0]            = (            a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] &             a_ff[0] & ~b_ff[3] &             b_ff[1] &  b_ff[0]) |
                                   (            a_ff[2] &                       ~b_ff[3] &            ~b_ff[1] & ~b_ff[0]) |
                                   (                       a_ff[1] &            ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (                                  a_ff[0] & ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] & ~a_ff[1] &            ~b_ff[3] & ~b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &                                             ~b_ff[2] & ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] & ~a_ff[2] &                       ~b_ff[3] &  b_ff[2] &  b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &  b_ff[2] & ~b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] &             a_ff[0] & ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] & ~a_ff[1] &            ~b_ff[3] &  b_ff[2] &             b_ff[0]) |
                                   (           ~a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                                             ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] &             a_ff[1] &                       ~b_ff[2] &            ~b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] &  b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                        b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &             a_ff[1] &             b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &                        a_ff[0] &            ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &            ~a_ff[1] &            ~b_ff[3] &  b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &             b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &             b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &             a_ff[0] &  b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] &  a_ff[1] &            ~b_ff[3] &             b_ff[1]           ) |
                                   ( a_ff[3] &             a_ff[1] &  a_ff[0] &            ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &  a_ff[0] &  b_ff[3]                                 );

   // *** *** *** END   : SMALLNUM }}




   // *** *** *** Start : Short Q {{

   assign shortq_dividend[pt.XLEN:0]   = {dividend_sign_ff,a_ff[pt.XLEN-1:0]};


   parameter shortq_a_width = pt.XLEN + 1;
   parameter shortq_b_width = pt.XLEN + 1;

   logic [5:0]  dw_a_enc;
   logic [5:0]  dw_b_enc;
   logic [6:0]  dw_shortq_raw;


   eh2_exu_div_cls i_a_cls  (
       .operand  ( shortq_dividend[pt.XLEN:0]  ),
       .cls      ( dw_a_enc[4:0]               ));

   eh2_exu_div_cls i_b_cls  (
       .operand  ( b_ff[pt.XLEN:0]             ),
       .cls      ( dw_b_enc[4:0]               ));

   assign dw_a_enc[5]             =  1'b0;
   assign dw_b_enc[5]             =  1'b0;


   assign dw_shortq_raw[6:0]      =  {1'b0,dw_b_enc[5:0]} - {1'b0,dw_a_enc[5:0]} + 7'd1;
   assign shortq[5:0]             =  dw_shortq_raw[6]    ?  6'd0  :  dw_shortq_raw[5:0];

   assign shortq_enable           =  valid_ff & ~shortq[5] & ~(shortq[4:2] ==  3'b111) & ~cancel;

   assign shortq_decode[4:0]      = ( {5{shortq[4:0] == 5'd31}} & 5'd00) |
                                    ( {5{shortq[4:0] == 5'd30}} & 5'd00) |
                                    ( {5{shortq[4:0] == 5'd29}} & 5'd00) |
                                    ( {5{shortq[4:0] == 5'd28}} & 5'd00) |
                                    ( {5{shortq[4:0] == 5'd27}} & 5'd03) |
                                    ( {5{shortq[4:0] == 5'd26}} & 5'd06) |
                                    ( {5{shortq[4:0] == 5'd25}} & 5'd06) |
                                    ( {5{shortq[4:0] == 5'd24}} & 5'd06) |
                                    ( {5{shortq[4:0] == 5'd23}} & 5'd09) |
                                    ( {5{shortq[4:0] == 5'd22}} & 5'd09) |
                                    ( {5{shortq[4:0] == 5'd21}} & 5'd09) |
                                    ( {5{shortq[4:0] == 5'd20}} & 5'd12) |
                                    ( {5{shortq[4:0] == 5'd19}} & 5'd12) |
                                    ( {5{shortq[4:0] == 5'd18}} & 5'd12) |
                                    ( {5{shortq[4:0] == 5'd17}} & 5'd15) |
                                    ( {5{shortq[4:0] == 5'd16}} & 5'd15) |
                                    ( {5{shortq[4:0] == 5'd15}} & 5'd15) |
                                    ( {5{shortq[4:0] == 5'd14}} & 5'd18) |
                                    ( {5{shortq[4:0] == 5'd13}} & 5'd18) |
                                    ( {5{shortq[4:0] == 5'd12}} & 5'd18) |
                                    ( {5{shortq[4:0] == 5'd11}} & 5'd21) |
                                    ( {5{shortq[4:0] == 5'd10}} & 5'd21) |
                                    ( {5{shortq[4:0] == 5'd09}} & 5'd21) |
                                    ( {5{shortq[4:0] == 5'd08}} & 5'd24) |
                                    ( {5{shortq[4:0] == 5'd07}} & 5'd24) |
                                    ( {5{shortq[4:0] == 5'd06}} & 5'd24) |
                                    ( {5{shortq[4:0] == 5'd05}} & 5'd27) |
                                    ( {5{shortq[4:0] == 5'd04}} & 5'd27) |
                                    ( {5{shortq[4:0] == 5'd03}} & 5'd27) |
                                    ( {5{shortq[4:0] == 5'd02}} & 5'd27) |  // Using 30 will violate the minimum latency required
                                    ( {5{shortq[4:0] == 5'd01}} & 5'd27) |  // Using 30 will violate the minimum latency required
                                    ( {5{shortq[4:0] == 5'd00}} & 5'd27);   // Using 30 will violate the minimum latency required


   assign shortq_shift[4:0]       = ~shortq_enable     ?  5'd0  :  shortq_decode[4:0];

   // *** *** *** End   : Short Q }}





endmodule // eh2_exu_div_new_3bit_fullshortq






// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
module eh2_exu_div_new_4bit_fullshortq
#(
`include "eh2_param.vh"
)
  (
   input  logic               clk,                       // Top level clock
   input  logic               rst_l,                     // Reset
   input  logic               scan_mode,                 // Scan mode

   input  logic               cancel,                    // Flush pipeline
   input  logic               valid_in,
   input  logic               signed_in,
   input  logic               rem_in,
   input  logic [pt.XLEN-1:0] dividend_in,
   input  logic [pt.XLEN-1:0] divisor_in,

   output logic               valid_out,
   output logic [pt.XLEN-1:0] data_out
  );


   logic                   valid_ff_in, valid_ff;
   logic                   finish_raw, finish, finish_ff;
   logic                   running_state;
   logic                   misc_enable;
   logic [2:0]             control_in, control_ff;
   logic                   dividend_sign_ff, divisor_sign_ff, rem_ff;
   logic                   count_enable;
   logic [pt.XLENW+1:0]      count_in, count_ff;

   logic                   smallnum_case;
   logic [3:0]             smallnum;

   logic                   a_enable, a_shift;
   logic [pt.XLEN-1:0]     a_in, a_ff;

   logic                   b_enable, b_twos_comp;
   logic [pt.XLEN:0]       b_in;
   logic [pt.XLEN+5:0]     b_ff;

   logic [pt.XLEN-1:0]     q_in, q_ff;

   logic                   rq_enable;
   logic                   r_sign_sel;
   logic                   r_restore_sel;
   logic                   r_adder01_sel, r_adder02_sel, r_adder03_sel;
   logic                   r_adder04_sel, r_adder05_sel, r_adder06_sel, r_adder07_sel;
   logic                   r_adder08_sel, r_adder09_sel, r_adder10_sel, r_adder11_sel;
   logic                   r_adder12_sel, r_adder13_sel, r_adder14_sel, r_adder15_sel;
   logic [pt.XLEN:0]       r_in, r_ff;

   logic                   twos_comp_q_sel, twos_comp_b_sel;
   logic [pt.XLEN-1:0]     twos_comp_in, twos_comp_out;

   logic [15:1]            quotient_raw;
   logic [3:0]             quotient_new;
   logic [pt.XLEN+2:0]     adder01_out;
   logic [pt.XLEN+3:0]     adder02_out;
   logic [pt.XLEN+4:0]     adder03_out;
   logic [pt.XLEN+5:0]     adder04_out;
   logic [pt.XLEN+5:0]     adder05_out;
   logic [pt.XLEN+5:0]     adder06_out;
   logic [pt.XLEN+5:0]     adder07_out;
   logic [pt.XLEN+5:0]     adder08_out;
   logic [pt.XLEN+5:0]     adder09_out;
   logic [pt.XLEN+5:0]     adder10_out;
   logic [pt.XLEN+5:0]     adder11_out;
   logic [pt.XLEN+5:0]     adder12_out;
   logic [pt.XLEN+5:0]     adder13_out;
   logic [pt.XLEN+5:0]     adder14_out;
   logic [pt.XLEN+5:0]     adder15_out;

   logic [2*pt.XLEN:0]     ar_shifted;
   logic [5:0]             shortq;
   logic [4:0]             shortq_shift;
   logic [4:0]             shortq_decode;
   logic [4:0]             shortq_shift_ff;
   logic                   shortq_enable;
   logic                   shortq_enable_ff;
   logic [pt.XLEN:0]       shortq_dividend;

   logic                   by_zero_case;

   logic [4:1]             special_in;
   logic [4:1]             special_ff;



   rvdffe #(16+pt.XLENW+1) i_misc_ff  (.*, .clk(clk), .en(misc_enable),  .din ({valid_ff_in, control_in[2:0], count_in[pt.XLENW+1:0], special_in[4:1], shortq_enable,    shortq_shift[4:0],    finish   }),
                                                                       .dout({valid_ff,    control_ff[2:0], count_ff[pt.XLENW+1:0], special_ff[4:1], shortq_enable_ff, shortq_shift_ff[4:0], finish_ff}) );

   rvdffe #(pt.XLEN)     i_a_ff    (.*, .clk(clk), .en(a_enable),     .din(a_in[pt.XLEN-1:0]),    .dout(a_ff[pt.XLEN-1:0]));
   rvdffe #(pt.XLEN+1)   i_b_ff    (.*, .clk(clk), .en(b_enable),     .din(b_in[pt.XLEN:0]),      .dout(b_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN+1)   i_r_ff    (.*, .clk(clk), .en(rq_enable),    .din(r_in[pt.XLEN:0]),      .dout(r_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN)     i_q_ff    (.*, .clk(clk), .en(rq_enable),    .din(q_in[pt.XLEN-1:0]),    .dout(q_ff[pt.XLEN-1:0]));



   assign special_in[4:1]        = {special_ff[3] & ~cancel,
                                    special_ff[2] & ~cancel,
                                    special_ff[1] & ~cancel,
                                    (smallnum_case | by_zero_case) & ~cancel};

   assign valid_ff_in            =  valid_in  & ~cancel;

   assign control_in[2]          = (~valid_in & control_ff[2]) | (valid_in & signed_in  & dividend_in[pt.XLEN-1]);
   assign control_in[1]          = (~valid_in & control_ff[1]) | (valid_in & signed_in  &  divisor_in[pt.XLEN-1]);
   assign control_in[0]          = (~valid_in & control_ff[0]) | (valid_in & rem_in);

   assign dividend_sign_ff       =  control_ff[2];
   assign divisor_sign_ff        =  control_ff[1];
   assign rem_ff                 =  control_ff[0];


   assign by_zero_case           =  valid_ff & (b_ff[pt.XLEN-1:0] == {pt.XLEN{1'b0}});

   assign misc_enable            =  valid_in | valid_ff | cancel | running_state | finish_ff;
   assign running_state          = (| count_ff[pt.XLENW+1:0]) | shortq_enable_ff;
   assign finish_raw             =   special_ff[3] |
                                    (count_ff[pt.XLENW+1:0] == (pt.XLENW+2)'(pt.XLEN));


   assign finish                 =  finish_raw & ~cancel;
   assign count_enable           = (valid_ff | running_state) & ~finish & ~finish_ff & ~cancel & ~shortq_enable;
   assign count_in[pt.XLENW+1:0]   = {pt.XLENW+2{count_enable}} & (count_ff[pt.XLENW+1:0] + (pt.XLENW+2)'('d4) + {2'b0,shortq_shift_ff[4:0]});


   assign a_enable               =  valid_in | running_state;
   assign a_shift                =  running_state & ~shortq_enable_ff;

   assign ar_shifted[2*pt.XLEN:0] = { {pt.XLEN+1{dividend_sign_ff}} , a_ff[pt.XLEN-1:0]} << {shortq_shift_ff[4:0]};

   assign a_in[pt.XLEN-1:0]      = ( {pt.XLEN{~a_shift & ~shortq_enable_ff}} & dividend_in[pt.XLEN-1:0]  ) |
                                   ( {pt.XLEN{ a_shift                    }} & {a_ff[pt.XLEN-5:0], 4'b0} ) |
                                   ( {pt.XLEN{            shortq_enable_ff}} &  ar_shifted[pt.XLEN-1:0]  );



   assign b_enable               =    valid_in | b_twos_comp;
   assign b_twos_comp            =    valid_ff & ~(dividend_sign_ff ^ divisor_sign_ff);

   assign b_in[pt.XLEN:0]        = ( {pt.XLEN+1{~b_twos_comp}} & { (signed_in & divisor_in[pt.XLEN-1]),divisor_in[pt.XLEN-1:0] } ) |
                                   ( {pt.XLEN+1{ b_twos_comp}} & {~divisor_sign_ff,twos_comp_out[pt.XLEN-1:0] } );


   assign rq_enable              =  valid_in | valid_ff | running_state & ~(| special_ff[3:1]);
   assign r_sign_sel             =  valid_ff      &  dividend_sign_ff & ~by_zero_case;
   assign r_restore_sel          =  running_state & (quotient_new[3:0] == 4'd00) & ~shortq_enable_ff;
   assign r_adder01_sel          =  running_state & (quotient_new[3:0] == 4'd01) & ~shortq_enable_ff;
   assign r_adder02_sel          =  running_state & (quotient_new[3:0] == 4'd02) & ~shortq_enable_ff;
   assign r_adder03_sel          =  running_state & (quotient_new[3:0] == 4'd03) & ~shortq_enable_ff;
   assign r_adder04_sel          =  running_state & (quotient_new[3:0] == 4'd04) & ~shortq_enable_ff;
   assign r_adder05_sel          =  running_state & (quotient_new[3:0] == 4'd05) & ~shortq_enable_ff;
   assign r_adder06_sel          =  running_state & (quotient_new[3:0] == 4'd06) & ~shortq_enable_ff;
   assign r_adder07_sel          =  running_state & (quotient_new[3:0] == 4'd07) & ~shortq_enable_ff;
   assign r_adder08_sel          =  running_state & (quotient_new[3:0] == 4'd08) & ~shortq_enable_ff;
   assign r_adder09_sel          =  running_state & (quotient_new[3:0] == 4'd09) & ~shortq_enable_ff;
   assign r_adder10_sel          =  running_state & (quotient_new[3:0] == 4'd10) & ~shortq_enable_ff;
   assign r_adder11_sel          =  running_state & (quotient_new[3:0] == 4'd11) & ~shortq_enable_ff;
   assign r_adder12_sel          =  running_state & (quotient_new[3:0] == 4'd12) & ~shortq_enable_ff;
   assign r_adder13_sel          =  running_state & (quotient_new[3:0] == 4'd13) & ~shortq_enable_ff;
   assign r_adder14_sel          =  running_state & (quotient_new[3:0] == 4'd14) & ~shortq_enable_ff;
   assign r_adder15_sel          =  running_state & (quotient_new[3:0] == 4'd15) & ~shortq_enable_ff;

   assign r_in[pt.XLEN:0]        = ( {pt.XLEN+1{r_sign_sel      }} & {pt.XLEN+1{1'b1}}                             ) |
                                   ( {pt.XLEN+1{r_restore_sel   }} & {r_ff[pt.XLEN-4:0],a_ff[pt.XLEN-1:pt.XLEN-4]} ) |
                                   ( {pt.XLEN+1{r_adder01_sel   }} &  adder01_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder02_sel   }} &  adder02_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder03_sel   }} &  adder03_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder04_sel   }} &  adder04_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder05_sel   }} &  adder05_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder06_sel   }} &  adder06_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder07_sel   }} &  adder07_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder08_sel   }} &  adder08_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder09_sel   }} &  adder09_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder10_sel   }} &  adder10_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder11_sel   }} &  adder11_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder12_sel   }} &  adder12_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder13_sel   }} &  adder13_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder14_sel   }} &  adder14_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{r_adder15_sel   }} &  adder15_out[pt.XLEN:0]                       ) |
                                   ( {pt.XLEN+1{shortq_enable_ff}} &  ar_shifted[2*pt.XLEN:pt.XLEN]                ) |
                                   ( {pt.XLEN+1{by_zero_case    }} & {1'b0,a_ff[pt.XLEN-1:0]}                      );


   assign q_in[pt.XLEN-1:0]      = ( {pt.XLEN{~valid_ff       }} & {q_ff[pt.XLEN-5:0], quotient_new[3:0]}  ) |
                                   ( {pt.XLEN{ smallnum_case  }} & {{pt.XLEN-4{1'b0}}, smallnum[3:0]}      ) |
                                   ( {pt.XLEN{ by_zero_case   }} & {pt.XLEN{1'b1}}                         );


   assign b_ff[pt.XLEN+5:pt.XLEN+1] = {b_ff[pt.XLEN],b_ff[pt.XLEN],b_ff[pt.XLEN],b_ff[pt.XLEN],b_ff[pt.XLEN]};


   assign adder01_out[pt.XLEN+2:0]      = {              r_ff[pt.XLEN-2:0], a_ff[pt.XLEN-1:pt.XLEN-4]}  +                                                                                        b_ff[pt.XLEN+2:0];
   assign adder02_out[pt.XLEN+3:0]      = {              r_ff[pt.XLEN-1:0], a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],1'b0};
   assign adder03_out[pt.XLEN+4:0]      = {              r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +                                                           {b_ff[pt.XLEN+3:0],1'b0}  +  b_ff[pt.XLEN+4:0];
   assign adder04_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +                              {b_ff[pt.XLEN+3:0],2'b0};
   assign adder05_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +                              {b_ff[pt.XLEN+3:0],2'b0}  +                               b_ff[pt.XLEN+5:0];
   assign adder06_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +                              {b_ff[pt.XLEN+3:0],2'b0}  +  {b_ff[pt.XLEN+4:0],1'b0};
   assign adder07_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +                              {b_ff[pt.XLEN+3:0],2'b0}  +  {b_ff[pt.XLEN+4:0],1'b0}  +  b_ff[pt.XLEN+5:0];
   assign adder08_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0};
   assign adder09_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +                                                            b_ff[pt.XLEN+5:0];
   assign adder10_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +                               {b_ff[pt.XLEN+4:0],1'b0};
   assign adder11_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +                               {b_ff[pt.XLEN+4:0],1'b0}  +  b_ff[pt.XLEN+5:0];
   assign adder12_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +  {b_ff[pt.XLEN+3:0],2'b0};
   assign adder13_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +  {b_ff[pt.XLEN+3:0],2'b0}  +                               b_ff[pt.XLEN+5:0];
   assign adder14_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +  {b_ff[pt.XLEN+3:0],2'b0}  +  {b_ff[pt.XLEN+4:0],1'b0};
   assign adder15_out[pt.XLEN+5:0]      = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],   a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +  {b_ff[pt.XLEN+3:0],2'b0}  +  {b_ff[pt.XLEN+4:0],1'b0}  +  b_ff[pt.XLEN+5:0];

   assign quotient_raw[01]       = (~adder01_out[pt.XLEN+2] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder01_out[pt.XLEN+2:0] == {pt.XLEN+3{1'b0}}) );
   assign quotient_raw[02]       = (~adder02_out[pt.XLEN+3] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder02_out[pt.XLEN+3:0] == {pt.XLEN+4{1'b0}}) );
   assign quotient_raw[03]       = (~adder03_out[pt.XLEN+4] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder03_out[pt.XLEN+4:0] == {pt.XLEN+5{1'b0}}) );
   assign quotient_raw[04]       = (~adder04_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder04_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[05]       = (~adder05_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder05_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[06]       = (~adder06_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder06_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[07]       = (~adder07_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder07_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[08]       = (~adder08_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder08_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[09]       = (~adder09_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder09_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[10]       = (~adder10_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder10_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[11]       = (~adder11_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder11_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[12]       = (~adder12_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder12_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[13]       = (~adder13_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder13_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[14]       = (~adder14_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder14_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[15]       = (~adder15_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder15_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );


   assign quotient_new[0]        = ( quotient_raw[15:01] == 15'b000_0000_0000_0001 ) |  //  1
                                   ( quotient_raw[15:03] == 13'b000_0000_0000_01   ) |  //  3
                                   ( quotient_raw[15:05] == 11'b000_0000_0001      ) |  //  5
                                   ( quotient_raw[15:07] ==  9'b000_0000_01        ) |  //  7
                                   ( quotient_raw[15:09] ==  7'b000_0001           ) |  //  9
                                   ( quotient_raw[15:11] ==  5'b000_01             ) |  // 11
                                   ( quotient_raw[15:13] ==  3'b001                ) |  // 13
                                   ( quotient_raw[   15] ==  1'b1                  );   // 15

   assign quotient_new[1]        = ( quotient_raw[15:02] == 14'b000_0000_0000_001  ) |  //  2
                                   ( quotient_raw[15:03] == 13'b000_0000_0000_01   ) |  //  3
                                   ( quotient_raw[15:06] == 10'b000_0000_001       ) |  //  6
                                   ( quotient_raw[15:07] ==  9'b000_0000_01        ) |  //  7
                                   ( quotient_raw[15:10] ==  6'b000_001            ) |  // 10
                                   ( quotient_raw[15:11] ==  5'b000_01             ) |  // 11
                                   ( quotient_raw[15:14] ==  2'b01                 ) |  // 14
                                   ( quotient_raw[   15] ==  1'b1                  );   // 15

   assign quotient_new[2]        = ( quotient_raw[15:04] == 12'b000_0000_0000_1    ) |  //  4
                                   ( quotient_raw[15:05] == 11'b000_0000_0001      ) |  //  5
                                   ( quotient_raw[15:06] == 10'b000_0000_001       ) |  //  6
                                   ( quotient_raw[15:07] ==  9'b000_0000_01        ) |  //  7
                                   ( quotient_raw[15:12] ==  4'b000_1              ) |  // 12
                                   ( quotient_raw[15:13] ==  3'b001                ) |  // 13
                                   ( quotient_raw[15:14] ==  2'b01                 ) |  // 14
                                   ( quotient_raw[   15] ==  1'b1                  );   // 15

   assign quotient_new[3]        = ( quotient_raw[15:08] ==  8'b000_0000_1         ) |  //  8
                                   ( quotient_raw[15:09] ==  7'b000_0001           ) |  //  9
                                   ( quotient_raw[15:10] ==  6'b000_001            ) |  // 10
                                   ( quotient_raw[15:11] ==  5'b000_01             ) |  // 11
                                   ( quotient_raw[15:12] ==  4'b000_1              ) |  // 12
                                   ( quotient_raw[15:13] ==  3'b001                ) |  // 13
                                   ( quotient_raw[15:14] ==  2'b01                 ) |  // 14
                                   ( quotient_raw[   15] ==  1'b1                  );   // 15


   assign twos_comp_b_sel        =  valid_ff           & ~(dividend_sign_ff ^ divisor_sign_ff);
   assign twos_comp_q_sel        = ~valid_ff & ~rem_ff &  (dividend_sign_ff ^ divisor_sign_ff) & ~special_ff[4];

   assign twos_comp_in[pt.XLEN-1:0] = ( {pt.XLEN{twos_comp_q_sel}} & q_ff[pt.XLEN-1:0] ) |
                                      ( {pt.XLEN{twos_comp_b_sel}} & b_ff[pt.XLEN-1:0] );

   rvtwoscomp #(pt.XLEN) i_twos_comp  (.din(twos_comp_in[pt.XLEN-1:0]), .dout(twos_comp_out[pt.XLEN-1:0]));



   assign valid_out              =  finish_ff & ~cancel;

   assign data_out[pt.XLEN-1:0]  = ( {pt.XLEN{~rem_ff & ~twos_comp_q_sel}} & q_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{ rem_ff                   }} & r_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{           twos_comp_q_sel}} & twos_comp_out[pt.XLEN-1:0] );




   // *** *** *** START : SMALLNUM {{

   assign smallnum_case          = ( (a_ff[pt.XLEN-1:4]  == {pt.XLEN-4{1'b0}}) & (b_ff[pt.XLEN-1:4] == {pt.XLEN-4{1'b0}}) & ~by_zero_case & ~rem_ff & valid_ff & ~cancel & 1'b0) |
                                   ( (a_ff[pt.XLEN-1:0]  == {pt.XLEN{1'b0}})   &                                            ~by_zero_case & ~rem_ff & valid_ff & ~cancel & 1'b0);

   assign smallnum[3]            = ( a_ff[3] &                                  ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           );

   assign smallnum[2]            = ( a_ff[3] &                                  ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (            a_ff[2] &                       ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] & ~b_ff[2]                      );

   assign smallnum[1]            = (            a_ff[2] &                       ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (                       a_ff[1] &            ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &                                  ~b_ff[3] &            ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] & ~a_ff[2] &                       ~b_ff[3] & ~b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] &  b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &             a_ff[1] &            ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &  b_ff[2]                      );

   assign smallnum[0]            = (            a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] &             a_ff[0] & ~b_ff[3] &             b_ff[1] &  b_ff[0]) |
                                   (            a_ff[2] &                       ~b_ff[3] &            ~b_ff[1] & ~b_ff[0]) |
                                   (                       a_ff[1] &            ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                   (                                  a_ff[0] & ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] & ~a_ff[1] &            ~b_ff[3] & ~b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &                                             ~b_ff[2] & ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] & ~a_ff[2] &                       ~b_ff[3] &  b_ff[2] &  b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &  b_ff[2] & ~b_ff[1]           ) |
                                   (~a_ff[3] &  a_ff[2] &             a_ff[0] & ~b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] & ~a_ff[1] &            ~b_ff[3] &  b_ff[2] &             b_ff[0]) |
                                   (           ~a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                                             ~b_ff[1] & ~b_ff[0]) |
                                   ( a_ff[3] &             a_ff[1] &                       ~b_ff[2] &            ~b_ff[0]) |
                                   (~a_ff[3] &  a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] &  b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &                        b_ff[3] & ~b_ff[2]                      ) |
                                   ( a_ff[3] &             a_ff[1] &             b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &                        a_ff[0] &            ~b_ff[2] & ~b_ff[1]           ) |
                                   ( a_ff[3] &            ~a_ff[1] &            ~b_ff[3] &  b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &             b_ff[3] &                       ~b_ff[0]) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &             b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] &  a_ff[2] &             a_ff[0] &  b_ff[3] &            ~b_ff[1]           ) |
                                   ( a_ff[3] & ~a_ff[2] &  a_ff[1] &            ~b_ff[3] &             b_ff[1]           ) |
                                   ( a_ff[3] &             a_ff[1] &  a_ff[0] &            ~b_ff[2]                      ) |
                                   ( a_ff[3] &  a_ff[2] &  a_ff[1] &  a_ff[0] &  b_ff[3]                                 );

   // *** *** *** END   : SMALLNUM }}




   // *** *** *** Start : Short Q {{

   assign shortq_dividend[pt.XLEN:0]   = {dividend_sign_ff,a_ff[pt.XLEN-1:0]};


   parameter shortq_a_width = pt.XLEN + 1;
   parameter shortq_b_width = pt.XLEN + 1;

   logic [5:0]  dw_a_enc;
   logic [5:0]  dw_b_enc;
   logic [6:0]  dw_shortq_raw;


   eh2_exu_div_cls i_a_cls  (
       .operand  ( shortq_dividend[pt.XLEN:0]  ),
       .cls      ( dw_a_enc[4:0]               ));

   eh2_exu_div_cls i_b_cls  (
       .operand  ( b_ff[pt.XLEN:0]             ),
       .cls      ( dw_b_enc[4:0]               ));

   assign dw_a_enc[5]             =  1'b0;
   assign dw_b_enc[5]             =  1'b0;


   assign dw_shortq_raw[6:0]      =  {1'b0,dw_b_enc[5:0]} - {1'b0,dw_a_enc[5:0]} + 7'd1;
   assign shortq[5:0]             =  dw_shortq_raw[6]  ?  6'd0  :  dw_shortq_raw[5:0];

   assign shortq_enable           =  valid_ff & ~shortq[5] & ~(shortq[4:2] ==  3'b111) & ~cancel;

   assign shortq_decode[4:0]      = ( {5{shortq[4:0] == 5'd31}} & 5'd00) |
                                    ( {5{shortq[4:0] == 5'd30}} & 5'd00) |
                                    ( {5{shortq[4:0] == 5'd29}} & 5'd00) |
                                    ( {5{shortq[4:0] == 5'd28}} & 5'd00) |
                                    ( {5{shortq[4:0] == 5'd27}} & 5'd04) |
                                    ( {5{shortq[4:0] == 5'd26}} & 5'd04) |
                                    ( {5{shortq[4:0] == 5'd25}} & 5'd04) |
                                    ( {5{shortq[4:0] == 5'd24}} & 5'd04) |
                                    ( {5{shortq[4:0] == 5'd23}} & 5'd08) |
                                    ( {5{shortq[4:0] == 5'd22}} & 5'd08) |
                                    ( {5{shortq[4:0] == 5'd21}} & 5'd08) |
                                    ( {5{shortq[4:0] == 5'd20}} & 5'd08) |
                                    ( {5{shortq[4:0] == 5'd19}} & 5'd12) |
                                    ( {5{shortq[4:0] == 5'd18}} & 5'd12) |
                                    ( {5{shortq[4:0] == 5'd17}} & 5'd12) |
                                    ( {5{shortq[4:0] == 5'd16}} & 5'd12) |
                                    ( {5{shortq[4:0] == 5'd15}} & 5'd16) |
                                    ( {5{shortq[4:0] == 5'd14}} & 5'd16) |
                                    ( {5{shortq[4:0] == 5'd13}} & 5'd16) |
                                    ( {5{shortq[4:0] == 5'd12}} & 5'd16) |
                                    ( {5{shortq[4:0] == 5'd11}} & 5'd20) |
                                    ( {5{shortq[4:0] == 5'd10}} & 5'd20) |
                                    ( {5{shortq[4:0] == 5'd09}} & 5'd20) |
                                    ( {5{shortq[4:0] == 5'd08}} & 5'd20) |
                                    ( {5{shortq[4:0] == 5'd07}} & 5'd24) |
                                    ( {5{shortq[4:0] == 5'd06}} & 5'd24) |
                                    ( {5{shortq[4:0] == 5'd05}} & 5'd24) |
                                    ( {5{shortq[4:0] == 5'd04}} & 5'd24) |
                                    ( {5{shortq[4:0] == 5'd03}} & 5'd24) |  // Using 28 will violate the minimum latency required
                                    ( {5{shortq[4:0] == 5'd02}} & 5'd24) |  // Using 28 will violate the minimum latency required
                                    ( {5{shortq[4:0] == 5'd01}} & 5'd24) |  // Using 28 will violate the minimum latency required
                                    ( {5{shortq[4:0] == 5'd00}} & 5'd24);   // Using 28 will violate the minimum latency required


   assign shortq_shift[4:0]       = ~shortq_enable     ?  5'd0  :  shortq_decode[4:0];

   // *** *** *** End   : Short Q }}





endmodule // eh2_exu_div_new_4bit_fullshortq






// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
module eh2_exu_div_cls
  (
   input  logic [32:0] operand,

   output logic [4:0]  cls                  // Count leading sign bits - "n" format ignoring [32]
   );


   logic [4:0]   cls_zeros;
   logic [4:0]   cls_ones;


assign cls_zeros[4:0]             = ({5{operand[31]    ==  {           1'b1} }} & 5'd00) |
                                    ({5{operand[31:30] ==  {{ 1{1'b0}},1'b1} }} & 5'd01) |
                                    ({5{operand[31:29] ==  {{ 2{1'b0}},1'b1} }} & 5'd02) |
                                    ({5{operand[31:28] ==  {{ 3{1'b0}},1'b1} }} & 5'd03) |
                                    ({5{operand[31:27] ==  {{ 4{1'b0}},1'b1} }} & 5'd04) |
                                    ({5{operand[31:26] ==  {{ 5{1'b0}},1'b1} }} & 5'd05) |
                                    ({5{operand[31:25] ==  {{ 6{1'b0}},1'b1} }} & 5'd06) |
                                    ({5{operand[31:24] ==  {{ 7{1'b0}},1'b1} }} & 5'd07) |
                                    ({5{operand[31:23] ==  {{ 8{1'b0}},1'b1} }} & 5'd08) |
                                    ({5{operand[31:22] ==  {{ 9{1'b0}},1'b1} }} & 5'd09) |
                                    ({5{operand[31:21] ==  {{10{1'b0}},1'b1} }} & 5'd10) |
                                    ({5{operand[31:20] ==  {{11{1'b0}},1'b1} }} & 5'd11) |
                                    ({5{operand[31:19] ==  {{12{1'b0}},1'b1} }} & 5'd12) |
                                    ({5{operand[31:18] ==  {{13{1'b0}},1'b1} }} & 5'd13) |
                                    ({5{operand[31:17] ==  {{14{1'b0}},1'b1} }} & 5'd14) |
                                    ({5{operand[31:16] ==  {{15{1'b0}},1'b1} }} & 5'd15) |
                                    ({5{operand[31:15] ==  {{16{1'b0}},1'b1} }} & 5'd16) |
                                    ({5{operand[31:14] ==  {{17{1'b0}},1'b1} }} & 5'd17) |
                                    ({5{operand[31:13] ==  {{18{1'b0}},1'b1} }} & 5'd18) |
                                    ({5{operand[31:12] ==  {{19{1'b0}},1'b1} }} & 5'd19) |
                                    ({5{operand[31:11] ==  {{20{1'b0}},1'b1} }} & 5'd20) |
                                    ({5{operand[31:10] ==  {{21{1'b0}},1'b1} }} & 5'd21) |
                                    ({5{operand[31:09] ==  {{22{1'b0}},1'b1} }} & 5'd22) |
                                    ({5{operand[31:08] ==  {{23{1'b0}},1'b1} }} & 5'd23) |
                                    ({5{operand[31:07] ==  {{24{1'b0}},1'b1} }} & 5'd24) |
                                    ({5{operand[31:06] ==  {{25{1'b0}},1'b1} }} & 5'd25) |
                                    ({5{operand[31:05] ==  {{26{1'b0}},1'b1} }} & 5'd26) |
                                    ({5{operand[31:04] ==  {{27{1'b0}},1'b1} }} & 5'd27) |
                                    ({5{operand[31:03] ==  {{28{1'b0}},1'b1} }} & 5'd28) |
                                    ({5{operand[31:02] ==  {{29{1'b0}},1'b1} }} & 5'd29) |
                                    ({5{operand[31:01] ==  {{30{1'b0}},1'b1} }} & 5'd30) |
                                    ({5{operand[31:00] ==  {{31{1'b0}},1'b1} }} & 5'd31) |
                                    ({5{operand[31:00] ==  {{32{1'b0}}     } }} & 5'd00);    // Don't care case as it will be handled as special case


assign cls_ones[4:0]              = ({5{operand[31:30] ==  {{ 1{1'b1}},1'b0} }} & 5'd00) |
                                    ({5{operand[31:29] ==  {{ 2{1'b1}},1'b0} }} & 5'd01) |
                                    ({5{operand[31:28] ==  {{ 3{1'b1}},1'b0} }} & 5'd02) |
                                    ({5{operand[31:27] ==  {{ 4{1'b1}},1'b0} }} & 5'd03) |
                                    ({5{operand[31:26] ==  {{ 5{1'b1}},1'b0} }} & 5'd04) |
                                    ({5{operand[31:25] ==  {{ 6{1'b1}},1'b0} }} & 5'd05) |
                                    ({5{operand[31:24] ==  {{ 7{1'b1}},1'b0} }} & 5'd06) |
                                    ({5{operand[31:23] ==  {{ 8{1'b1}},1'b0} }} & 5'd07) |
                                    ({5{operand[31:22] ==  {{ 9{1'b1}},1'b0} }} & 5'd08) |
                                    ({5{operand[31:21] ==  {{10{1'b1}},1'b0} }} & 5'd09) |
                                    ({5{operand[31:20] ==  {{11{1'b1}},1'b0} }} & 5'd10) |
                                    ({5{operand[31:19] ==  {{12{1'b1}},1'b0} }} & 5'd11) |
                                    ({5{operand[31:18] ==  {{13{1'b1}},1'b0} }} & 5'd12) |
                                    ({5{operand[31:17] ==  {{14{1'b1}},1'b0} }} & 5'd13) |
                                    ({5{operand[31:16] ==  {{15{1'b1}},1'b0} }} & 5'd14) |
                                    ({5{operand[31:15] ==  {{16{1'b1}},1'b0} }} & 5'd15) |
                                    ({5{operand[31:14] ==  {{17{1'b1}},1'b0} }} & 5'd16) |
                                    ({5{operand[31:13] ==  {{18{1'b1}},1'b0} }} & 5'd17) |
                                    ({5{operand[31:12] ==  {{19{1'b1}},1'b0} }} & 5'd18) |
                                    ({5{operand[31:11] ==  {{20{1'b1}},1'b0} }} & 5'd19) |
                                    ({5{operand[31:10] ==  {{21{1'b1}},1'b0} }} & 5'd20) |
                                    ({5{operand[31:09] ==  {{22{1'b1}},1'b0} }} & 5'd21) |
                                    ({5{operand[31:08] ==  {{23{1'b1}},1'b0} }} & 5'd22) |
                                    ({5{operand[31:07] ==  {{24{1'b1}},1'b0} }} & 5'd23) |
                                    ({5{operand[31:06] ==  {{25{1'b1}},1'b0} }} & 5'd24) |
                                    ({5{operand[31:05] ==  {{26{1'b1}},1'b0} }} & 5'd25) |
                                    ({5{operand[31:04] ==  {{27{1'b1}},1'b0} }} & 5'd26) |
                                    ({5{operand[31:03] ==  {{28{1'b1}},1'b0} }} & 5'd27) |
                                    ({5{operand[31:02] ==  {{29{1'b1}},1'b0} }} & 5'd28) |
                                    ({5{operand[31:01] ==  {{30{1'b1}},1'b0} }} & 5'd29) |
                                    ({5{operand[31:00] ==  {{31{1'b1}},1'b0} }} & 5'd30) |
                                    ({5{operand[31:00] ==  {{32{1'b1}}     } }} & 5'd31);


assign cls[4:0]                   =  operand[32]  ?  cls_ones[4:0]  :  cls_zeros[4:0];

endmodule // eh2_exu_div_cls
