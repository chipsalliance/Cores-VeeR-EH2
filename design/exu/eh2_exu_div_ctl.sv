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
   input logic                   clk,                       // Top level clock
   input logic                   rst_l,                     // Reset
   input logic                   scan_mode,                 // Scan mode

   input eh2_div_pkt_t           dp,                        // valid, sign, rem
   input logic  [pt.XLEN-1:0]    dividend,                  // Numerator
   input logic  [pt.XLEN-1:0]    divisor,                   // Denominator

   input logic                   cancel,                    // Cancel divide


   output logic                  finish_dly,                // Finish to match data
   output logic [pt.XLEN-1:0]    out                        // Result
  );

   logic [pt.XLEN-1:0]           out_raw;



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

   logic                div_clken;
   logic                run_in, run_state;
   logic [6:0]          count_in, count;
   logic [pt.XLEN:0]    m_ff;
   logic                qff_enable;
   logic                aff_enable;
   logic [pt.XLEN:0]    q_in, q_ff;
   logic [pt.XLEN:0]    a_in, a_ff;
   logic [pt.XLEN:0]    m_eff;
   logic [pt.XLEN:0]    a_shift;
   logic                dividend_neg_ff, divisor_neg_ff;
   logic [pt.XLEN-1:0]  dividend_comp;
   logic [pt.XLEN-1:0]  dividend_eff;
   logic [pt.XLEN-1:0]  q_ff_comp;
   logic [pt.XLEN-1:0]  q_ff_eff;
   logic [pt.XLEN-1:0]  a_ff_comp;
   logic [pt.XLEN-1:0]  a_ff_eff;
   logic                sign_ff, sign_eff;
   logic                rem_ff;
   logic                add;
   logic [pt.XLEN:0]    a_eff;
   logic [2*pt.XLEN:0]  a_eff_shift;
   logic                rem_correct;
   logic                valid_ff_x;
   logic                valid_x;
   logic                finish;
   logic                finish_ff;

   logic                smallnum_case_e1, smallnum_case_e2, smallnum_case_e3, smallnum_case_e4, smallnum_case_wb;
   logic [3:0]          smallnum, smallnum_in, smallnum_ff;
   logic                m_already_comp;

   logic [6:0]          a_cls;
   logic [6:0]          b_cls;
   logic [pt.XLEN_BYTES-1:0] shortq_shift;
   logic [5:0]          shortq_shift_ff;
   logic                shortq_enable;
   logic                shortq_enable_ff;
   logic [pt.XLEN:0]    short_dividend;
   logic [pt.XLEN_BYTES-1:0]  shortq_raw;
   logic [pt.XLEN_BYTES-1:0]  shortq_shift_xx;

   localparam unsigned MISC_FF_WIDTH = 1 + 1 + 1 + 7 + 1 + pt.XLEN_BYTES + 1 + 1 + 1 + 1;

   rvdffe #(MISC_FF_WIDTH) i_misc_ff   (.*, .clk(clk), .en(div_clken), .din ({valid_in & ~cancel,
                                                                         finish   & ~cancel,
                                                                         run_in,
                                                                         count_in[6:0],
                                                                         shortq_enable,
                                                                         shortq_shift[pt.XLEN_BYTES-1:0],
                                                                         (valid_in & dividend_in[pt.XLEN-1]) | (~valid_in & dividend_neg_ff),
                                                                         (valid_in & divisor_in[pt.XLEN-1] ) | (~valid_in & divisor_neg_ff ),
                                                                         (valid_in & sign_eff       ) | (~valid_in & sign_ff        ),
                                                                         (valid_in & rem_in         ) | (~valid_in & rem_ff         )} ),
                                                                       .dout({valid_ff_x,
                                                                         finish_ff,
                                                                         run_state,
                                                                         count[6:0],
                                                                         shortq_enable_ff,
                                                                         shortq_shift_xx[pt.XLEN_BYTES-1:0],
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

   rvdffe #(pt.XLEN+1) mff               (.*, .clk(clk), .en(valid_in),     .din({signed_in & divisor_in[pt.XLEN-1], divisor_in[pt.XLEN-1:0]}),    .dout(m_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN+1) qff               (.*, .clk(clk), .en(qff_enable),   .din(q_in[pt.XLEN:0]),                                                 .dout(q_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN+1) aff               (.*, .clk(clk), .en(aff_enable),   .din(a_in[pt.XLEN:0]),                                                 .dout(a_ff[pt.XLEN:0]));

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
                                    ((q_ff[pt.XLEN-1:0] == {pt.XLEN{1'b0}})   &                                            (m_ff[pt.XLEN-1:0] != {pt.XLEN{1'b0}}) & ~rem_ff & valid_x);


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

   assign short_dividend[pt.XLEN-1:0]     =  q_ff[pt.XLEN-1:0];
   assign short_dividend[pt.XLEN]         =  sign_ff & q_ff[pt.XLEN-1];

   /*
    * The cheapshortq module, does not implement the SRT algorithm like the other modules below do.
    * Instead, when the 2 operands are wider than 4 bits (smallnum case), the shortq logic will skip
    * bits of the quotient that are guaranteed to be zero based on the 2 operands.
    * This is done to reduce the number of cycles required to compute the full quotient from XLEN to XLEN-k,
    * where k is the number of bits of the quotient that have been identified as zero.
    * The logic is based on 2 classifier signals (called `a_cls/b_cls`), one for each operand of the division (A/B).
    * On the original RV32 implementation, the *_cls signals are 3-bits wide and used as:
    *    a_cls/b_cls[2] = 1 when A/B has non-zero bits on byte 3 (unsigned), or when not all bits are set to 1 (signed)
    *    a_cls/b_cls[1] = 1 when A/B has non-zero bits on byte 2 (unsigned), or when not all bits are set to 1 (signed)
    *    a_cls/b_cls[0] = 1 when A/B has non-zero bits on byte 1 (unsigned), or when not all bits are set to 1 (signed)
    *    Note: There is no classifier bit for byte 0 (i.e. bits 7:0)
    * Once these values are computed, the truth tables seen below contain the cartesian product of their values, with don't care bits
    * since *_cls[2] = 1 is more important than *_cls[1:0], in order to determine the number of shift positions (3rd column of the table).
    *    Note: The shift does not apply to the two operands, i.e. to align the two numbers based on their magnitude difference, but
    *          determines how many quotient bits are detected to be zero (starting from the most important ones down to LSB), in order to
    *          skip computing zero bits on the quotient.
    * The 4-bit `shortq_raw` signal groups all the different rows of the truth table (i.e. a_cls/b_cls combinations) that result
    * in the same shift (`short_raw[3]` groups combinations for a shift of 32, `shortq_raw[2]` for shift of 24, etc).
    * The value of this signal is then decoded to get the actual number of shifts assigned on the `shortq_shift_ff` signal.
    *    Note: If the required number of shifts is 32, i.e. all quotient bits would be zero, the shift is truncated to 30,
               since no division operation is allowed to complete in less than 2 cycles.
    *
    * Example: If A and B are both unsigned numbers:
    *
    * If A has significant bits on byte 3, i.e. bits [31:24] (a_byte = 3), then A < 2^(8*(a_byte+1)) => A < 2^32
    * If B has significant bits on byte 2, i.e. bits [23:16] (b_byte = 2), then B >= 2^(8*b_byte) => B >= 2^16.
    * By dividing these numbers:
    *       Q = floor(A/B), i.e. Q < 2^32/2^16, and thus Q < 2^16 or Q <= 2^16 - 1.
    *       This value requires log2(2^16 - 1) + 1 = 16 bits to be represented.
    *       Therefore, XLEN-16 = 16 bits can be skipped, since the 16 highest bits of the quotient will be equal to zero.
    *    Note: This example explains row: (A_cls=1xx, B_cls=01x, sh=16) of the truth table below.
    *
    * In short, the values on the truth table are derived using :
    *       shift = 8 * clamp(3 + b_byte - a_byte, 0, 4), where clamp is used to limit the first argument in the range of [0, 4],
            and a_byte/b_byte is the actual byte index of the highest most significant byte of numbers A and B,
            which can be derived by adding 1 to the values of the a_cls/b_cls signals, in order to get the real byte index.
    *
    * The above also apply to RV64 accordingly, and the shift expression slightly changes to :
    *       shift = 8 * clamp(7 + b_byte - a_byte, 0, 8)
    * and the a_cls/b_cls classifier signals for the two division operands, are extended to account for the increased number of bytes.
    */
   //          XLEN = 32
   //     A       B
   //    210     210    SH
   //    ---     ---    --
   //    1xx     000     0
   //    1xx     001     8
   //    1xx     01x    16
   //    1xx     1xx    24
   //    01x     000     8
   //    01x     001    16
   //    01x     01x    24
   //    01x     1xx    32
   //    001     000    16
   //    001     001    24
   //    001     01x    32
   //    001     1xx    32
   //    000     000    24
   //    000     001    32
   //    000     01x    32
   //    000     1xx    32

   //          XLEN = 64
   //      A         B
   //   6543210   6543210    SH
   //   -------   -------    --
   //   1xxxxxx   0000000     0
   //   1xxxxxx   0000001     8
   //   1xxxxxx   000001x    16
   //   1xxxxxx   00001xx    24
   //   1xxxxxx   0001xxx    32
   //   1xxxxxx   001xxxx    40
   //   1xxxxxx   01xxxxx    48
   //   1xxxxxx   1xxxxxx    56
   //
   //   01xxxxx   0000000     8
   //   01xxxxx   0000001    16
   //   01xxxxx   000001x    24
   //   01xxxxx   00001xx    32
   //   01xxxxx   0001xxx    40
   //   01xxxxx   001xxxx    48
   //   01xxxxx   01xxxxx    56
   //   01xxxxx   1xxxxxx    64
   //
   //   001xxxx   0000000    16
   //   001xxxx   0000001    24
   //   001xxxx   000001x    32
   //   001xxxx   00001xx    40
   //   001xxxx   0001xxx    48
   //   001xxxx   001xxxx    56
   //   001xxxx   01xxxxx    64
   //   001xxxx   1xxxxxx    64
   //
   //   0001xxx   0000000    24
   //   0001xxx   0000001    32
   //   0001xxx   000001x    40
   //   0001xxx   00001xx    48
   //   0001xxx   0001xxx    56
   //   0001xxx   001xxxx    64
   //   0001xxx   01xxxxx    64
   //   0001xxx   1xxxxxx    64
   //
   //   00001xx   0000000    32
   //   00001xx   0000001    40
   //   00001xx   000001x    48
   //   00001xx   00001xx    56
   //   00001xx   0001xxx    64
   //   00001xx   001xxxx    64
   //   00001xx   01xxxxx    64
   //   00001xx   1xxxxxx    64
   //
   //   000001x   0000000    40
   //   000001x   0000001    48
   //   000001x   000001x    56
   //   000001x   00001xx    64
   //   000001x   0001xxx    64
   //   000001x   001xxxx    64
   //   000001x   01xxxxx    64
   //   000001x   1xxxxxx    64
   //
   //   0000001   0000000    48
   //   0000001   0000001    56
   //   0000001   000001x    64
   //   0000001   00001xx    64
   //   0000001   0001xxx    64
   //   0000001   001xxxx    64
   //   0000001   01xxxxx    64
   //   0000001   1xxxxxx    64
   //
   //   0000000   0000000    56
   //   0000000   0000001    64
   //   0000000   000001x    64
   //   0000000   00001xx    64
   //   0000000   0001xxx    64
   //   0000000   001xxxx    64
   //   0000000   01xxxxx    64
   //   0000000   1xxxxxx    64

   if (pt.XLEN == 64) begin
      assign a_cls[6]             =  (~short_dividend[pt.XLEN] & (short_dividend[63:56] != {8{1'b0}})) | ( short_dividend[pt.XLEN] & (short_dividend[63:55] != {9{1'b1}}));
      assign a_cls[5]             =  (~short_dividend[pt.XLEN] & (short_dividend[55:48] != {8{1'b0}})) | ( short_dividend[pt.XLEN] & (short_dividend[54:47] != {8{1'b1}}));
      assign a_cls[4]             =  (~short_dividend[pt.XLEN] & (short_dividend[47:40] != {8{1'b0}})) | ( short_dividend[pt.XLEN] & (short_dividend[46:39] != {8{1'b1}}));
      assign a_cls[3]             =  (~short_dividend[pt.XLEN] & (short_dividend[39:32] != {8{1'b0}})) | ( short_dividend[pt.XLEN] & (short_dividend[38:31] != {8{1'b1}}));
      assign a_cls[2]             =  (~short_dividend[pt.XLEN] & (short_dividend[31:24] != {8{1'b0}})) | ( short_dividend[pt.XLEN] & (short_dividend[30:23] != {8{1'b1}}));
   end else if (pt.XLEN == 32) begin
      assign a_cls[6:3]           =  4'b0;
      assign a_cls[2]             =  (~short_dividend[pt.XLEN] & (short_dividend[31:24] != {8{1'b0}})) | ( short_dividend[pt.XLEN] & (short_dividend[31:23] != {9{1'b1}}));
   end
   assign a_cls[1]                =  (~short_dividend[pt.XLEN] & (short_dividend[23:16] != {8{1'b0}})) | ( short_dividend[pt.XLEN] & (short_dividend[22:15] != {8{1'b1}}));
   assign a_cls[0]                =  (~short_dividend[pt.XLEN] & (short_dividend[15:08] != {8{1'b0}})) | ( short_dividend[pt.XLEN] & (short_dividend[14:07] != {8{1'b1}}));

   if (pt.XLEN == 64) begin
      assign b_cls[6]                =  (~m_ff[pt.XLEN]   & (          m_ff[63:56] != {8{1'b0}})) | ( m_ff[pt.XLEN]      & (          m_ff[63:56] != {8{1'b1}}));
      assign b_cls[5]                =  (~m_ff[pt.XLEN]   & (          m_ff[55:48] != {8{1'b0}})) | ( m_ff[pt.XLEN]      & (          m_ff[55:48] != {8{1'b1}}));
      assign b_cls[4]                =  (~m_ff[pt.XLEN]   & (          m_ff[47:40] != {8{1'b0}})) | ( m_ff[pt.XLEN]      & (          m_ff[47:40] != {8{1'b1}}));
      assign b_cls[3]                =  (~m_ff[pt.XLEN]   & (          m_ff[39:32] != {8{1'b0}})) | ( m_ff[pt.XLEN]      & (          m_ff[39:32] != {8{1'b1}}));
   end else if (pt.XLEN == 32) begin
      assign b_cls[6:3]           =  4'b0;
   end
   assign b_cls[2]                =  (~m_ff[pt.XLEN]      & (          m_ff[31:24] != {8{1'b0}})) | ( m_ff[pt.XLEN]      & (          m_ff[31:24] != {8{1'b1}}));
   assign b_cls[1]                =  (~m_ff[pt.XLEN]      & (          m_ff[23:16] != {8{1'b0}})) | ( m_ff[pt.XLEN]      & (          m_ff[23:16] != {8{1'b1}}));
   assign b_cls[0]                =  (~m_ff[pt.XLEN]      & (          m_ff[15:08] != {8{1'b0}})) | ( m_ff[pt.XLEN]      & (          m_ff[15:08] != {8{1'b1}}));

   if (pt.XLEN == 64) begin
      assign shortq_raw[7] =  ( (a_cls[6:5] == 2'b01     )  & (b_cls[6]   == 1'b1       ) ) |    // Shift by 64
                              ( (a_cls[6:4] == 3'b001    )  & (b_cls[6:5] == 2'b01      ) ) |
                              ( (a_cls[6:4] == 3'b001    )  & (b_cls[6]   == 1'b1       ) ) |
                              ( (a_cls[6:3] == 4'b0001   )  & (b_cls[6:4] == 3'b001     ) ) |
                              ( (a_cls[6:3] == 4'b0001   )  & (b_cls[6:5] == 2'b01      ) ) |
                              ( (a_cls[6:3] == 4'b0001   )  & (b_cls[6]   == 1'b1       ) ) |
                              ( (a_cls[6:2] == 5'b00001  )  & (b_cls[6:3] == 4'b0001    ) ) |
                              ( (a_cls[6:2] == 5'b00001  )  & (b_cls[6:4] == 3'b001     ) ) |
                              ( (a_cls[6:2] == 5'b00001  )  & (b_cls[6:5] == 2'b01      ) ) |
                              ( (a_cls[6:2] == 5'b00001  )  & (b_cls[6]   == 1'b1       ) ) |
                              ( (a_cls[6:1] == 6'b000001 )  & (b_cls[6:2] == 5'b00001   ) ) |
                              ( (a_cls[6:1] == 6'b000001 )  & (b_cls[6:3] == 4'b0001    ) ) |
                              ( (a_cls[6:1] == 6'b000001 )  & (b_cls[6:4] == 3'b001     ) ) |
                              ( (a_cls[6:1] == 6'b000001 )  & (b_cls[6:5] == 2'b01      ) ) |
                              ( (a_cls[6:1] == 6'b000001 )  & (b_cls[6]   == 1'b1       ) ) |
                              ( (a_cls[6:0] == 7'b0000001)  & (b_cls[6:1] == 6'b000001  ) ) |
                              ( (a_cls[6:0] == 7'b0000001)  & (b_cls[6:2] == 5'b00001   ) ) |
                              ( (a_cls[6:0] == 7'b0000001)  & (b_cls[6:3] == 4'b0001    ) ) |
                              ( (a_cls[6:0] == 7'b0000001)  & (b_cls[6:4] == 3'b001     ) ) |
                              ( (a_cls[6:0] == 7'b0000001)  & (b_cls[6:5] == 2'b01      ) ) |
                              ( (a_cls[6:0] == 7'b0000001)  & (b_cls[6]   == 1'b1       ) ) |
                              ( (a_cls[6:0] == 7'b0000000)  & (b_cls[6:0] == 7'b0000001 ) ) |
                              ( (a_cls[6:0] == 7'b0000000)  & (b_cls[6:1] == 6'b000001  ) ) |
                              ( (a_cls[6:0] == 7'b0000000)  & (b_cls[6:2] == 5'b00001   ) ) |
                              ( (a_cls[6:0] == 7'b0000000)  & (b_cls[6:3] == 4'b0001    ) ) |
                              ( (a_cls[6:0] == 7'b0000000)  & (b_cls[6:4] == 3'b001     ) ) |
                              ( (a_cls[6:0] == 7'b0000000)  & (b_cls[6:5] == 2'b01      ) ) |
                              ( (a_cls[6:0] == 7'b0000000)  & (b_cls[6]   == 1'b1       ) );

      assign shortq_raw[6] =  ( (a_cls[6]   == 1'b1      )  & (b_cls[6]   == 1'b1       ) ) |    // Shift by 56
                              ( (a_cls[6:5] == 2'b01     )  & (b_cls[6:5] == 2'b01      ) ) |
                              ( (a_cls[6:4] == 3'b001    )  & (b_cls[6:4] == 3'b001     ) ) |
                              ( (a_cls[6:3] == 4'b0001   )  & (b_cls[6:3] == 4'b0001    ) ) |
                              ( (a_cls[6:2] == 5'b00001  )  & (b_cls[6:2] == 5'b00001   ) ) |
                              ( (a_cls[6:1] == 6'b000001 )  & (b_cls[6:1] == 6'b000001  ) ) |
                              ( (a_cls[6:0] == 7'b0000001)  & (b_cls[6:0] == 7'b0000001 ) ) |
                              ( (a_cls[6:0] == 7'b0000000)  & (b_cls[6:0] == 7'b0000000 ) );

      assign shortq_raw[5] =  ( (a_cls[6]   == 1'b1      )  & (b_cls[6:5] == 2'b01      ) ) |   // Shift by 48
                              ( (a_cls[6:5] == 2'b01     )  & (b_cls[6:4] == 3'b001     ) ) |
                              ( (a_cls[6:4] == 3'b001    )  & (b_cls[6:3] == 4'b0001    ) ) |
                              ( (a_cls[6:3] == 4'b0001   )  & (b_cls[6:2] == 5'b00001   ) ) |
                              ( (a_cls[6:2] == 5'b00001  )  & (b_cls[6:1] == 6'b000001  ) ) |
                              ( (a_cls[6:1] == 6'b000001 )  & (b_cls[6:0] == 7'b0000001 ) ) |
                              ( (a_cls[6:0] == 7'b0000001)  & (b_cls[6:0] == 7'b0000000 ) );

      assign shortq_raw[4] =  ( (a_cls[6]   == 1'b1      )  & (b_cls[6:4] == 3'b001     ) ) |   // Shift by 40
                              ( (a_cls[6:5] == 2'b01     )  & (b_cls[6:3] == 4'b0001    ) ) |
                              ( (a_cls[6:4] == 3'b001    )  & (b_cls[6:2] == 5'b00001   ) ) |
                              ( (a_cls[6:3] == 4'b0001   )  & (b_cls[6:1] == 6'b000001  ) ) |
                              ( (a_cls[6:2] == 5'b00001  )  & (b_cls[6:0] == 7'b0000001 ) ) |
                              ( (a_cls[6:1] == 6'b000001 )  & (b_cls[6:0] == 7'b0000000 ) );

      assign shortq_raw[3] =  ( (a_cls[6]   == 1'b1      ) & (b_cls[6:3] == 4'b0001     ) ) |   // Shift by 32
                              ( (a_cls[6:5] == 2'b01     ) & (b_cls[6:2] == 5'b00001    ) ) |
                              ( (a_cls[6:4] == 3'b001    ) & (b_cls[6:1] == 6'b000001   ) ) |
                              ( (a_cls[6:3] == 4'b0001   ) & (b_cls[6:0] == 7'b0000001  ) ) |
                              ( (a_cls[6:2] == 5'b00001  ) & (b_cls[6:0] == 7'b0000000  ) );

      assign shortq_raw[2] =  ( (a_cls[6]   == 1'b1      ) & (b_cls[6:2] == 5'b00001    ) ) |   // Shift by 24
                              ( (a_cls[6:5] == 2'b01     ) & (b_cls[6:1] == 6'b000001   ) ) |
                              ( (a_cls[6:4] == 3'b001    ) & (b_cls[6:0] == 7'b0000001  ) ) |
                              ( (a_cls[6:3] == 4'b0001   ) & (b_cls[6:0] == 7'b0000000  ) );

      assign shortq_raw[1] =  ( (a_cls[6]   == 1'b1      ) & (b_cls[6:1] == 6'b000001   ) ) |   // Shift by 16
                              ( (a_cls[6:5] == 2'b01     ) & (b_cls[6:0] == 7'b0000001  ) ) |
                              ( (a_cls[6:4] == 3'b001    ) & (b_cls[6:0] == 7'b0000000  ) );

      assign shortq_raw[0] =  ( (a_cls[6]   == 1'b1      ) & (b_cls[6:0] == 7'b0000001  ) ) |   // Shift by  8
                              ( (a_cls[6:5] == 2'b01     ) & (b_cls[6:0] == 7'b0000000  ) );
   end else begin
      assign shortq_raw[3] =  ( (a_cls[2:1] == 2'b01 ) & (b_cls[2]   == 1'b1  ) ) |   // Shift by 32
                              ( (a_cls[2:0] == 3'b001) & (b_cls[2]   == 1'b1  ) ) |
                              ( (a_cls[2:0] == 3'b000) & (b_cls[2]   == 1'b1  ) ) |
                              ( (a_cls[2:0] == 3'b001) & (b_cls[2:1] == 2'b01 ) ) |
                              ( (a_cls[2:0] == 3'b000) & (b_cls[2:1] == 2'b01 ) ) |
                              ( (a_cls[2:0] == 3'b000) & (b_cls[2:0] == 3'b001) );

      assign shortq_raw[2] =  ( (a_cls[2]   == 1'b1  ) & (b_cls[2]   == 1'b1  ) ) |   // Shift by 24
                              ( (a_cls[2:1] == 2'b01 ) & (b_cls[2:1] == 2'b01 ) ) |
                              ( (a_cls[2:0] == 3'b001) & (b_cls[2:0] == 3'b001) ) |
                              ( (a_cls[2:0] == 3'b000) & (b_cls[2:0] == 3'b000) );

      assign shortq_raw[1] =  ( (a_cls[2]   == 1'b1  ) & (b_cls[2:1] == 2'b01 ) ) |   // Shift by 16
                              ( (a_cls[2:1] == 2'b01 ) & (b_cls[2:0] == 3'b001) ) |
                              ( (a_cls[2:0] == 3'b001) & (b_cls[2:0] == 3'b000) );

      assign shortq_raw[0] =  ( (a_cls[2]   == 1'b1  ) & (b_cls[2:0] == 3'b001) ) |   // Shift by  8
                              ( (a_cls[2:1] == 2'b01 ) & (b_cls[2:0] == 3'b000) );
   end

   assign shortq_enable =  valid_ff_x & (m_ff[pt.XLEN-1:0] != {pt.XLEN{1'b0}}) & (shortq_raw[pt.XLEN_BYTES-1:0] != pt.XLEN_BYTES'('b0)) & ~smallnum_case_e1;

   assign shortq_shift[pt.XLEN_BYTES-1:0] = ({pt.XLEN_BYTES{shortq_enable}} & shortq_raw[pt.XLEN_BYTES-1:0]);

   if (pt.XLEN == 32) begin
      assign shortq_shift_ff[5]     =  1'b0;
      assign shortq_shift_ff[4:0]   = ({5{shortq_shift_xx[3]}} & 5'd30) | // 31 -> 30 (required for nonblocking div so finish is no faster than E4 (div cannot finish in less than 2 cycles))
                                      ({5{shortq_shift_xx[2]}} & 5'd24) | // 24
                                      ({5{shortq_shift_xx[1]}} & 5'd16) | // 16
                                      ({5{shortq_shift_xx[0]}} & 5'd8);   //  8
   end else if (pt.XLEN == 64) begin
      assign shortq_shift_ff[5:0]   = ({6{shortq_shift_xx[7]}} & 6'd62) | // 62 (required for nonblocking div so finish is no faster than E4 (div cannot finish in less than 2 cycles))
                                      ({6{shortq_shift_xx[6]}} & 6'd54) | // 54
                                      ({6{shortq_shift_xx[5]}} & 6'd48) | // 48
                                      ({6{shortq_shift_xx[4]}} & 6'd40) | // 40
                                      ({6{shortq_shift_xx[3]}} & 6'd32) | // 32
                                      ({6{shortq_shift_xx[2]}} & 6'd24) | // 24
                                      ({6{shortq_shift_xx[1]}} & 6'd16) | // 16
                                      ({6{shortq_shift_xx[0]}} & 6'd8);   //  8
   end

   // *** End   Short Q *** }}

   assign div_clken               =  valid_in | run_state | finish | finish_ff;

   assign run_in                  = (valid_in | run_state) & ~finish & ~cancel;

   assign count_in[6:0]           = {7{run_state & ~finish & ~cancel & ~shortq_enable}} & (count[6:0] + {1'b0,shortq_shift_ff[5:0]} + 7'd1);


   assign finish                  = (smallnum_case_e4 | ((~rem_ff) ? (count[6:0] == 7'(pt.XLEN)) : (count[6:0] == 7'(pt.XLEN+1))));

   assign valid_out               =  finish_ff & ~cancel;

   assign sign_eff                =  signed_in & (divisor_in[pt.XLEN-1:0] != {pt.XLEN{1'b0}});


   assign q_in[pt.XLEN:0]         = ({pt.XLEN+1{~run_state                                   }} &  {1'b0,dividend_in[pt.XLEN-1:0]}) |
                                    ({pt.XLEN+1{ run_state &  (valid_ff_x | shortq_enable_ff)}} &  ({dividend_eff[pt.XLEN-1:0], ~a_in[pt.XLEN]} << shortq_shift_ff[5:0])) |
                                    ({pt.XLEN+1{ run_state & ~(valid_ff_x | shortq_enable_ff)}} &  {q_ff[pt.XLEN-1:0], ~a_in[pt.XLEN]});

   assign qff_enable              =  valid_in | (run_state & ~shortq_enable);

   assign dividend_eff[pt.XLEN-1:0] = (sign_ff & dividend_neg_ff) ? dividend_comp[pt.XLEN-1:0] : q_ff[pt.XLEN-1:0];


   assign m_eff[pt.XLEN:0]        = ( add ) ? m_ff[pt.XLEN:0] : ~m_ff[pt.XLEN:0];

   assign a_eff_shift[2*pt.XLEN:0] = {{pt.XLEN+1{1'b0}}, dividend_eff[pt.XLEN-1:0]} << shortq_shift_ff[5:0];

   assign a_eff[pt.XLEN:0]        = ({pt.XLEN+1{ rem_correct                    }} &  a_ff[pt.XLEN:0]                   ) |
                                    ({pt.XLEN+1{~rem_correct & ~shortq_enable_ff}} & {a_ff[pt.XLEN-1:0], q_ff[pt.XLEN]} ) |
                                    ({pt.XLEN+1{~rem_correct &  shortq_enable_ff}} &  a_eff_shift[2*pt.XLEN:pt.XLEN]    );

   assign a_shift[pt.XLEN:0]      = {pt.XLEN+1{run_state}} & a_eff[pt.XLEN:0];

   assign a_in[pt.XLEN:0]         = {pt.XLEN+1{run_state}} & (a_shift[pt.XLEN:0] + m_eff[pt.XLEN:0] + {{pt.XLEN{1'b0}}, ~add});

   assign aff_enable              =  valid_in | (run_state & ~shortq_enable & (count[6:0] != 7'(pt.XLEN+1))) | rem_correct;


   assign m_already_comp          = (divisor_neg_ff & sign_ff);

   // if m already complemented, then invert operation add->sub, sub->add
   assign add                     = (a_ff[pt.XLEN] | rem_correct) ^ m_already_comp;

   assign rem_correct             = (count[6:0] == 7'(pt.XLEN+1)) & rem_ff & a_ff[pt.XLEN];



   assign q_ff_eff[pt.XLEN-1:0]   = (sign_ff & (dividend_neg_ff ^ divisor_neg_ff)) ? q_ff_comp[pt.XLEN-1:0] : q_ff[pt.XLEN-1:0];

   assign a_ff_eff[pt.XLEN-1:0]   = (sign_ff &  dividend_neg_ff) ? a_ff_comp[pt.XLEN-1:0] : a_ff[pt.XLEN-1:0];

   assign data_out[pt.XLEN-1:0]   = ({pt.XLEN{ smallnum_case_wb          }} & {{pt.XLEN-4{1'b0}}, smallnum_ff[3:0]}) |
                                    ({pt.XLEN{                     rem_ff}} &  a_ff_eff[pt.XLEN-1:0]               ) |
                                    ({pt.XLEN{~smallnum_case_wb & ~rem_ff}} &  q_ff_eff[pt.XLEN-1:0]               );




endmodule // eh2_exu_div_existing_1bit_cheapshortq






// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
module eh2_exu_div_new_1bit_fullshortq
#(
`include "eh2_param.vh"
)
  (
   input  logic                 clk,                       // Top level clock
   input  logic                 rst_l,                     // Reset
   input  logic                 scan_mode,                 // Scan mode

   input  logic                 cancel,                    // Flush pipeline
   input  logic                 valid_in,
   input  logic                 signed_in,
   input  logic                 rem_in,
   input  logic [pt.XLEN-1:0]   dividend_in,
   input  logic [pt.XLEN-1:0]   divisor_in,

   output logic                 valid_out,
   output logic [pt.XLEN-1:0]   data_out
  );


   logic                            valid_ff_in, valid_ff;
   logic                            finish_raw, finish, finish_ff;
   logic                            running_state;
   logic                            misc_enable;
   logic        [2:0]               control_in, control_ff;
   logic                            dividend_sign_ff, divisor_sign_ff, rem_ff;
   logic                            count_enable;
   logic        [6:0]               count_in, count_ff;

   logic                            smallnum_case;
   logic        [3:0]               smallnum;

   logic                            a_enable, a_shift;
   logic        [pt.XLEN-1:0]       a_in, a_ff;

   logic                            b_enable, b_twos_comp;
   logic        [pt.XLEN:0]         b_in, b_ff;

   logic        [pt.XLEN-1:0]       q_in, q_ff;

   logic                            rq_enable, r_sign_sel, r_restore_sel, r_adder_sel;
   logic        [pt.XLEN-1:0]       r_in, r_ff;

   logic                            twos_comp_q_sel, twos_comp_b_sel;
   logic        [pt.XLEN-1:0]       twos_comp_in, twos_comp_out;

   logic                            quotient_set;
   logic        [pt.XLEN:0]         adder_out;

   logic        [(2*pt.XLEN)-1:0]   ar_shifted;
   logic        [pt.XLENW:0]        shortq;
   logic        [pt.XLENW-1:0]      shortq_shift;
   logic        [pt.XLENW-1:0]      shortq_shift_ff;
   logic                            shortq_neg_or_zero;
   logic                            shortq_enable;
   logic                            shortq_enable_ff;
   logic        [pt.XLEN:0]         shortq_dividend;

   logic                            by_zero_case;

   logic         [4:1]              special_in;
   logic         [4:1]              special_ff;

   localparam unsigned MISC_FF_WIDTH = 1 + 3 + 7 + 4 + 1 + pt.XLENW + 1;

   rvdffe #(MISC_FF_WIDTH) i_misc_ff   (.*, .clk(clk), .en(misc_enable),  .din ({valid_ff_in, control_in[2:0], count_in[6:0], special_in[4:1], shortq_enable,    shortq_shift[pt.XLENW-1:0],    finish   }),
                                                                          .dout({valid_ff,    control_ff[2:0], count_ff[6:0], special_ff[4:1], shortq_enable_ff, shortq_shift_ff[pt.XLENW-1:0], finish_ff}) );

   rvdffe #(pt.XLEN)    i_a_ff         (.*, .clk(clk), .en(a_enable),     .din(a_in[pt.XLEN-1:0]),    .dout(a_ff[pt.XLEN-1:0]));
   rvdffe #(pt.XLEN+1)  i_b_ff         (.*, .clk(clk), .en(b_enable),     .din(b_in[pt.XLEN:0]),      .dout(b_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN)    i_r_ff         (.*, .clk(clk), .en(rq_enable),    .din(r_in[pt.XLEN-1:0]),    .dout(r_ff[pt.XLEN-1:0]));
   rvdffe #(pt.XLEN)    i_q_ff         (.*, .clk(clk), .en(rq_enable),    .din(q_in[pt.XLEN-1:0]),    .dout(q_ff[pt.XLEN-1:0]));



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
   assign running_state          = (| count_ff[6:0]) | shortq_enable_ff;
   assign finish_raw             =   special_ff[3] |
                                    (count_ff[6:0] == 7'(pt.XLEN));


   assign finish                 =  finish_raw & ~cancel;
   assign count_enable           = (valid_ff | running_state) & ~finish & ~finish_ff & ~cancel & ~shortq_enable;
   assign count_in[6:0]          = {7{count_enable}} & (count_ff[6:0] + {6'b0,1'b1} + {{7-pt.XLENW{1'b0}}, shortq_shift_ff[pt.XLENW-1:0]});


   assign a_enable               =  valid_in | running_state;
   assign a_shift                =  running_state & ~shortq_enable_ff;

   assign ar_shifted[(2*pt.XLEN)-1:0]  = { {pt.XLEN{dividend_sign_ff}} , a_ff[pt.XLEN-1:0]} << shortq_shift_ff[pt.XLENW-1:0];

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


   assign r_in[pt.XLEN-1:0]      = ( {pt.XLEN{r_sign_sel      }} &  {pt.XLEN{1'b1}}                     ) |
                                   ( {pt.XLEN{r_restore_sel   }} & {r_ff[pt.XLEN-2:0] ,a_ff[pt.XLEN-1]} ) |
                                   ( {pt.XLEN{r_adder_sel     }} &  adder_out[pt.XLEN-1:0]              ) |
                                   ( {pt.XLEN{shortq_enable_ff}} &  ar_shifted[(2*pt.XLEN)-1:pt.XLEN]   ) |
                                   ( {pt.XLEN{by_zero_case    }} &  a_ff[pt.XLEN-1:0]                   );


   assign q_in[pt.XLEN-1:0]      = ( {pt.XLEN{~valid_ff       }} & {q_ff[pt.XLEN-2:0], quotient_set}  ) |
                                   ( {pt.XLEN{ smallnum_case  }} & {{pt.XLEN-4{1'b0}}, smallnum[3:0]} ) |
                                   ( {pt.XLEN{ by_zero_case   }} & {pt.XLEN{1'b1}}                  );



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

   logic [pt.XLENW:0]   dw_a_enc;
   logic [pt.XLENW:0]   dw_b_enc;
   logic [pt.XLENW+1:0] dw_shortq_raw;


   eh2_exu_div_cls i_a_cls  (
       .operand  ( shortq_dividend[pt.XLEN:0]  ),
       .cls      ( dw_a_enc[pt.XLENW-1:0]      ));

   eh2_exu_div_cls i_b_cls  (
       .operand  ( b_ff[pt.XLEN:0]        ),
       .cls      ( dw_b_enc[pt.XLENW-1:0] ));

   assign dw_a_enc[pt.XLENW]      =  1'b0;
   assign dw_b_enc[pt.XLENW]      =  1'b0;


   assign dw_shortq_raw[pt.XLENW+1:0]       =  {1'b0,dw_b_enc[pt.XLENW:0]} - {1'b0,dw_a_enc[pt.XLENW:0]} + (pt.XLENW+2)'('d1);
   assign shortq_neg_or_zero                =  dw_shortq_raw[pt.XLENW+1] | (dw_shortq_raw[pt.XLENW:0] == {pt.XLENW+1{1'b0}});
   assign shortq[pt.XLENW:0]                =  shortq_neg_or_zero  ?  (pt.XLENW+1)'('d1)  :  dw_shortq_raw[pt.XLENW:0];   // 1 is minimum SHORTQ otherwise WB too early

   assign shortq_enable                     =  valid_ff & ~shortq[pt.XLENW] & ~(shortq[pt.XLENW-1:1] ==  {pt.XLENW-1{1'b1}}) & ~cancel;

   assign shortq_shift[pt.XLENW-1:0]        = ~shortq_enable     ?  (pt.XLENW)'('d0)  :  ({pt.XLENW{1'b1}} - shortq[pt.XLENW-1:0]);

   // *** *** *** End   : Short Q }}





endmodule // eh2_exu_div_new_1bit_fullshortq






// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
module eh2_exu_div_new_2bit_fullshortq
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


   logic                            valid_ff_in, valid_ff;
   logic                            finish_raw, finish, finish_ff;
   logic                            running_state;
   logic                            misc_enable;
   logic        [2:0]               control_in, control_ff;
   logic                            dividend_sign_ff, divisor_sign_ff, rem_ff;
   logic                            count_enable;
   logic        [6:0]               count_in, count_ff;

   logic                            smallnum_case;
   logic        [3:0]               smallnum;

   logic                            a_enable, a_shift;
   logic        [pt.XLEN-1:0]       a_in, a_ff;

   logic                            b_enable, b_twos_comp;
   logic        [pt.XLEN:0]         b_in;
   logic        [pt.XLEN+2:0]       b_ff;

   logic        [pt.XLEN-1:0]       q_in, q_ff;

   logic                            rq_enable, r_sign_sel, r_restore_sel, r_adder1_sel, r_adder2_sel, r_adder3_sel;
   logic        [pt.XLEN-1:0]       r_in, r_ff;

   logic                            twos_comp_q_sel, twos_comp_b_sel;
   logic        [pt.XLEN-1:0]       twos_comp_in, twos_comp_out;

   logic        [3:1]               quotient_raw;
   logic        [1:0]               quotient_new;
   logic        [pt.XLEN:0]         adder1_out;
   logic        [pt.XLEN+1:0]       adder2_out;
   logic        [pt.XLEN+2:0]       adder3_out;

   logic        [(2*pt.XLEN)-1:0]   ar_shifted;
   logic        [pt.XLENW:0]        shortq;
   logic        [pt.XLENW-1:0]      shortq_shift;
   logic        [pt.XLENW-1:1]      shortq_shift_ff;
   logic                            shortq_neg_or_zero;
   logic                            shortq_enable;
   logic                            shortq_enable_ff;
   logic        [pt.XLEN:0]         shortq_dividend;

   logic                            by_zero_case;

   logic         [4:1]              special_in;
   logic         [4:1]              special_ff;

   localparam unsigned MISC_FF_WIDTH = 1 + 3 + 7 + 4 + 1 + (pt.XLENW-1) + 1;
   rvdffe #(MISC_FF_WIDTH) i_misc_ff     (.*, .clk(clk), .en(misc_enable),  .din ({valid_ff_in, control_in[2:0], count_in[6:0], special_in[4:1], shortq_enable,    shortq_shift[pt.XLENW-1:1],    finish   }),
                                                                            .dout({valid_ff,    control_ff[2:0], count_ff[6:0], special_ff[4:1], shortq_enable_ff, shortq_shift_ff[pt.XLENW-1:1], finish_ff}) );

   rvdffe #(pt.XLEN)    i_a_ff           (.*, .clk(clk), .en(a_enable),     .din(a_in[pt.XLEN-1:0]),    .dout(a_ff[pt.XLEN-1:0]));
   rvdffe #(pt.XLEN+1)  i_b_ff           (.*, .clk(clk), .en(b_enable),     .din(b_in[pt.XLEN:0]),      .dout(b_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN)    i_r_ff           (.*, .clk(clk), .en(rq_enable),    .din(r_in[pt.XLEN-1:0]),    .dout(r_ff[pt.XLEN-1:0]));
   rvdffe #(pt.XLEN)    i_q_ff           (.*, .clk(clk), .en(rq_enable),    .din(q_in[pt.XLEN-1:0]),    .dout(q_ff[pt.XLEN-1:0]));



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
   assign running_state          = (| count_ff[6:0]) | shortq_enable_ff;
   assign finish_raw             =   special_ff[3] |
                                    (count_ff[6:0] == 7'(pt.XLEN));


   assign finish                 =  finish_raw & ~cancel;
   assign count_enable           = (valid_ff | running_state) & ~finish & ~finish_ff & ~cancel & ~shortq_enable;
   assign count_in[6:0]          = {7{count_enable}} & (count_ff[6:0] + {5'b0,2'b10} + {{7-pt.XLENW{1'b0}}, shortq_shift_ff[pt.XLENW-1:1], 1'b0});


   assign a_enable               =  valid_in | running_state;
   assign a_shift                =  running_state & ~shortq_enable_ff;

   assign ar_shifted[(2*pt.XLEN)-1:0]  = { {pt.XLEN{dividend_sign_ff}} , a_ff[pt.XLEN-1:0]} << {shortq_shift_ff[pt.XLENW-1:1],1'b0};

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


   assign r_in[pt.XLEN-1:0]      = ( {pt.XLEN{r_sign_sel      }} &  {pt.XLEN{1'b1}}                                ) |
                                   ( {pt.XLEN{r_restore_sel   }} &  {r_ff[pt.XLEN-3:0], a_ff[pt.XLEN-1:pt.XLEN-2]} ) |
                                   ( {pt.XLEN{r_adder1_sel    }} &  adder1_out[pt.XLEN-1:0]                        ) |
                                   ( {pt.XLEN{r_adder2_sel    }} &  adder2_out[pt.XLEN-1:0]                        ) |
                                   ( {pt.XLEN{r_adder3_sel    }} &  adder3_out[pt.XLEN-1:0]                        ) |
                                   ( {pt.XLEN{shortq_enable_ff}} &  ar_shifted[(2*pt.XLEN)-1:pt.XLEN]              ) |
                                   ( {pt.XLEN{by_zero_case    }} &  a_ff[pt.XLEN-1:0]                              );


   assign q_in[pt.XLEN-1:0]      = ( {pt.XLEN{~valid_ff       }} & {q_ff[pt.XLEN-3:0], quotient_new[1:0]} ) |
                                   ( {pt.XLEN{ smallnum_case  }} & {{pt.XLEN-4{1'b0}}, smallnum[3:0]}     ) |
                                   ( {pt.XLEN{ by_zero_case   }} & {pt.XLEN{1'b1}}                        );


   assign b_ff[pt.XLEN+2:pt.XLEN+1] = {b_ff[pt.XLEN],b_ff[pt.XLEN]};


   assign adder1_out[pt.XLEN:0]     = {         r_ff[pt.XLEN-2:0],a_ff[pt.XLEN-1:pt.XLEN-2]}  +  b_ff[pt.XLEN:0];
   assign adder2_out[pt.XLEN+1:0]   = {         r_ff[pt.XLEN-1:0],a_ff[pt.XLEN-1:pt.XLEN-2]}  + {b_ff[pt.XLEN:0],1'b0};
   assign adder3_out[pt.XLEN+2:0]   = {r_ff[pt.XLEN-1],r_ff[pt.XLEN-1:0],a_ff[pt.XLEN-1:pt.XLEN-2]}  + {b_ff[pt.XLEN+1:0],1'b0}  +  b_ff[pt.XLEN+2:0];


   assign quotient_raw[1]        = (~adder1_out[pt.XLEN]   ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder1_out[pt.XLEN:0]   == {pt.XLEN+1{1'b0}}) );
   assign quotient_raw[2]        = (~adder2_out[pt.XLEN+1] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder2_out[pt.XLEN+1:0] == {pt.XLEN+2{1'b0}}) );
   assign quotient_raw[3]        = (~adder3_out[pt.XLEN+2] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-3:0] == {pt.XLEN-2{1'b0}}) & (adder3_out[pt.XLEN+2:0] == {pt.XLEN+3{1'b0}}) );

   assign quotient_new[1]        = quotient_raw[3] |  quotient_raw[2];
   assign quotient_new[0]        = quotient_raw[3] |(~quotient_raw[2] & quotient_raw[1]);


   assign twos_comp_b_sel        =  valid_ff           & ~(dividend_sign_ff ^ divisor_sign_ff);
   assign twos_comp_q_sel        = ~valid_ff & ~rem_ff &  (dividend_sign_ff ^ divisor_sign_ff) & ~special_ff[4];

   assign twos_comp_in[pt.XLEN-1:0]     = ( {pt.XLEN{twos_comp_q_sel}} & q_ff[pt.XLEN-1:0] ) |
                                          ( {pt.XLEN{twos_comp_b_sel}} & b_ff[pt.XLEN-1:0] );

   rvtwoscomp #(pt.XLEN) i_twos_comp  (.din(twos_comp_in[pt.XLEN-1:0]), .dout(twos_comp_out[pt.XLEN-1:0]));



   assign valid_out              =  finish_ff & ~cancel;

   assign data_out[pt.XLEN-1:0]  = ( {pt.XLEN{~rem_ff & ~twos_comp_q_sel}} & q_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{ rem_ff                   }} & r_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{           twos_comp_q_sel}} & twos_comp_out[pt.XLEN-1:0] );




   // *** *** *** START : SMALLNUM {{

   assign smallnum_case          = ( (a_ff[pt.XLEN-1:4]  == {pt.XLEN-4{1'b0}}) & (b_ff[pt.XLEN-1:4] == {pt.XLEN-4{1'b0}}) & ~by_zero_case & ~rem_ff & valid_ff & ~cancel) |
                                   ( (a_ff[pt.XLEN-1:0]  == {pt.XLEN{1'b0}}) &                                              ~by_zero_case & ~rem_ff & valid_ff & ~cancel);

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

   logic [pt.XLENW:0]   dw_a_enc;
   logic [pt.XLENW:0]   dw_b_enc;
   logic [pt.XLENW+1:0] dw_shortq_raw;


   eh2_exu_div_cls i_a_cls  (
       .operand  ( shortq_dividend[pt.XLEN:0]    ),
       .cls      ( dw_a_enc[pt.XLENW-1:0]        ));

   eh2_exu_div_cls i_b_cls  (
       .operand  ( b_ff[pt.XLEN:0]        ),
       .cls      ( dw_b_enc[pt.XLENW-1:0] ));

   assign dw_a_enc[pt.XLENW]      =  1'b0;
   assign dw_b_enc[pt.XLENW]      =  1'b0;


   assign dw_shortq_raw[pt.XLENW+1:0]  =  {1'b0,dw_b_enc[pt.XLENW:0]} - {1'b0,dw_a_enc[pt.XLENW:0]} + (pt.XLENW+2)'('d1);
   assign shortq_neg_or_zero           =  dw_shortq_raw[pt.XLENW+1] | (dw_shortq_raw[pt.XLENW:1] == {pt.XLENW{1'b0}});  // Also includes 1
   assign shortq[pt.XLENW:0]           =  shortq_neg_or_zero  ?  (pt.XLENW+1)'('d2)  :  dw_shortq_raw[pt.XLENW:0];      // 2 is minimum SHORTQ otherwise WB too early

   assign shortq_enable                =  valid_ff & ~shortq[pt.XLENW] & ~(shortq[pt.XLENW-1:1] ==  {pt.XLENW-1{1'b1}}) & ~cancel;

   assign shortq_shift[pt.XLENW-1:0]   = ~shortq_enable     ?  {pt.XLENW{1'b0}}  :  ({pt.XLENW{1'b1}} - shortq[pt.XLENW-1:0]);   // [0] is unused

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
   logic        [2:0]      control_in, control_ff;
   logic                   dividend_sign_ff, divisor_sign_ff, rem_ff;
   logic                   count_enable;
   logic        [6:0]      count_in, count_ff;

   logic                   smallnum_case;
   logic        [3:0]      smallnum;

   // --------------------------------------------------------------------
   // Internal working precision.
   //
   // A radix-8 iteration consumes 3 dividend bits per cycle.  The datapath
   // needs (real operand bits + sign bit) to divide evenly by 3, or the
   // final iteration's 3-bit window runs off the end of real data.
   //   RV32: XLEN+1 = 33 bits -> already a multiple of 3 (11 iters), no
   //         padding required (SIGN_PAD = 1, i.e. just the ordinary sign bit).
   //   RV64: XLEN+1 = 65 bits -> NOT a multiple of 3; next multiple is 66
   //         (22 iters), so one extra bit of headroom is required.
   //
   // The extra headroom bit(s) are added as *additional sign-extension*
   // at the top of a_ff/b_ff (never as low-order zero padding). Because
   // sign-extending a two's-complement value doesn't change the value it
   // represents, this costs nothing: the redundant leading iteration(s)
   // just produce redundant leading quotient digits, which fall off the
   // top of q_ff via the existing shift-and-evict mechanism -- exactly
   // the same mechanism RV32 already relies on for its one sign bit.
   // No special-case correction of q_ff or r_ff is required as a result.
   // --------------------------------------------------------------------
   localparam unsigned MISC_FF_WIDTH = 1 + 3 + 7 + 4 + 1 + pt.XLENW + 1;
   localparam unsigned DIV_ITER_BITS = (pt.XLEN == 32) ? 33 : 66;
   localparam unsigned SIGN_PAD      = DIV_ITER_BITS - pt.XLEN;     // redundant sign-extension bits (1 for RV32, 2 for RV64)

   logic                                     a_enable, a_shift;
   logic        [DIV_ITER_BITS-1:0]          a_in, a_ff;

   logic                                     b_enable, b_twos_comp;
   logic        [DIV_ITER_BITS-1:0]          b_in;
   logic        [DIV_ITER_BITS+3:0]          b_ff;

   logic        [pt.XLEN-1:0]                q_in, q_ff;

   logic                                     rq_enable;
   logic                                     r_sign_sel;
   logic                                     r_restore_sel;
   logic                                     r_adder1_sel, r_adder2_sel, r_adder3_sel, r_adder4_sel, r_adder5_sel, r_adder6_sel, r_adder7_sel;
   logic        [DIV_ITER_BITS-1:0]          r_in, r_ff;

   logic                                     twos_comp_q_sel, twos_comp_b_sel;
   logic        [pt.XLEN-1:0]                twos_comp_in, twos_comp_out;

   logic        [7:1]                        quotient_raw;
   logic        [2:0]                        quotient_new;
   logic        [DIV_ITER_BITS:0]            adder1_out;
   logic        [DIV_ITER_BITS+1:0]          adder2_out;
   logic        [DIV_ITER_BITS+2:0]          adder3_out;
   logic        [DIV_ITER_BITS+3:0]          adder4_out;
   logic        [DIV_ITER_BITS+3:0]          adder5_out;
   logic        [DIV_ITER_BITS+3:0]          adder6_out;
   logic        [DIV_ITER_BITS+3:0]          adder7_out;

   logic        [(2*(DIV_ITER_BITS-1))+1:0]  ar_shifted;
   logic        [pt.XLENW:0]                 shortq;
   logic        [pt.XLENW-1:0]               shortq_shift;
   logic        [pt.XLENW-1:0]               shortq_decode;
   logic        [pt.XLENW-1:0]               shortq_shift_ff;
   logic                                     shortq_enable;
   logic                                     shortq_enable_ff;
   logic        [pt.XLEN:0]                  shortq_dividend;

   logic                                     by_zero_case;

   logic         [4:1]                       special_in;
   logic         [4:1]                       special_ff;

   rvdffe #(MISC_FF_WIDTH) i_misc_ff        (.*, .clk(clk), .en(misc_enable),  .din ({valid_ff_in, control_in[2:0], count_in[6:0], special_in[4:1], shortq_enable,    shortq_shift[pt.XLENW-1:0],    finish   }),
                                                                               .dout({valid_ff,    control_ff[2:0], count_ff[6:0], special_ff[4:1], shortq_enable_ff, shortq_shift_ff[pt.XLENW-1:0], finish_ff}) );

   rvdffe #(DIV_ITER_BITS) i_a_ff    (.*, .clk(clk), .en(a_enable),     .din(a_in[DIV_ITER_BITS-1:0]),         .dout(a_ff[DIV_ITER_BITS-1:0]));
   rvdffe #(DIV_ITER_BITS) i_b_ff    (.*, .clk(clk), .en(b_enable),     .din(b_in[DIV_ITER_BITS-1:0]),         .dout(b_ff[DIV_ITER_BITS-1:0]));
   rvdffe #(DIV_ITER_BITS) i_r_ff    (.*, .clk(clk), .en(rq_enable),    .din(r_in[DIV_ITER_BITS-1:0]),         .dout(r_ff[DIV_ITER_BITS-1:0]));
   rvdffe #(pt.XLEN)       i_q_ff    (.*, .clk(clk), .en(rq_enable),    .din(q_in[pt.XLEN-1:0]),               .dout(q_ff[pt.XLEN-1:0]));



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
   assign running_state          = (| count_ff[6:0]) | shortq_enable_ff;
   assign finish_raw             =   special_ff[3] |
                                    (count_ff[6:0] == 7'(DIV_ITER_BITS));


   assign finish                 =  finish_raw & ~cancel;
   assign count_enable           = (valid_ff | running_state) & ~finish & ~finish_ff & ~cancel & ~shortq_enable;
   assign count_in[6:0]          = {7{count_enable}} & (count_ff[6:0] + {5'b0,2'b11} + {{7-pt.XLENW{1'b0}}, shortq_shift_ff[pt.XLENW-1:0]});


   assign a_enable               =  valid_in | running_state;
   assign a_shift                =  running_state & ~shortq_enable_ff;

   assign ar_shifted[(2*(DIV_ITER_BITS-1))+1:0]  = { {DIV_ITER_BITS{dividend_sign_ff}} , a_ff[DIV_ITER_BITS-1:0]} << {shortq_shift_ff[pt.XLENW-1:0]};

   assign a_in[DIV_ITER_BITS-1:0] = ( {DIV_ITER_BITS{~a_shift & ~shortq_enable_ff}} & {{SIGN_PAD{signed_in & dividend_in[pt.XLEN-1]}},dividend_in[pt.XLEN-1:0]} ) |
                                    ( {DIV_ITER_BITS{ a_shift                    }} & {a_ff[DIV_ITER_BITS-4:0],3'b0}  ) |
                                    ( {DIV_ITER_BITS{            shortq_enable_ff}} &  ar_shifted[DIV_ITER_BITS-1:0]  );



   assign b_enable               =    valid_in | b_twos_comp;
   assign b_twos_comp            =    valid_ff & ~(dividend_sign_ff ^ divisor_sign_ff);

   assign b_in[DIV_ITER_BITS-1:0] = ( {DIV_ITER_BITS{~b_twos_comp}} & { {SIGN_PAD{signed_in & divisor_in[pt.XLEN-1]}},divisor_in[pt.XLEN-1:0] } ) |
                                    ( {DIV_ITER_BITS{ b_twos_comp}} & { {SIGN_PAD{~divisor_sign_ff}},twos_comp_out[pt.XLEN-1:0] } );


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


   assign r_in[DIV_ITER_BITS-1:0] = ( {DIV_ITER_BITS{r_sign_sel      }} & {DIV_ITER_BITS{1'b1}}                                            ) |
                                    ( {DIV_ITER_BITS{r_restore_sel   }} & {r_ff[DIV_ITER_BITS-4:0] ,a_ff[DIV_ITER_BITS-1:DIV_ITER_BITS-3]} ) |
                                    ( {DIV_ITER_BITS{r_adder1_sel    }} &  adder1_out[DIV_ITER_BITS-1:0]                                   ) |
                                    ( {DIV_ITER_BITS{r_adder2_sel    }} &  adder2_out[DIV_ITER_BITS-1:0]                                   ) |
                                    ( {DIV_ITER_BITS{r_adder3_sel    }} &  adder3_out[DIV_ITER_BITS-1:0]                                   ) |
                                    ( {DIV_ITER_BITS{r_adder4_sel    }} &  adder4_out[DIV_ITER_BITS-1:0]                                   ) |
                                    ( {DIV_ITER_BITS{r_adder5_sel    }} &  adder5_out[DIV_ITER_BITS-1:0]                                   ) |
                                    ( {DIV_ITER_BITS{r_adder6_sel    }} &  adder6_out[DIV_ITER_BITS-1:0]                                   ) |
                                    ( {DIV_ITER_BITS{r_adder7_sel    }} &  adder7_out[DIV_ITER_BITS-1:0]                                   ) |
                                    ( {DIV_ITER_BITS{shortq_enable_ff}} &  ar_shifted[(2*(DIV_ITER_BITS-1))+1:DIV_ITER_BITS]               ) |
                                    ( {DIV_ITER_BITS{by_zero_case    }} & {{SIGN_PAD{1'b0}},a_ff[pt.XLEN-1:0]}                             );


   assign q_in[pt.XLEN-1:0]      = ( {pt.XLEN{~valid_ff     }} & {q_ff[pt.XLEN-4:0], quotient_new[2:0]} ) |
                                   ( {pt.XLEN{ smallnum_case}} & {{pt.XLEN-4{1'b0}}, smallnum[3:0]}     ) |
                                   ( {pt.XLEN{ by_zero_case }} & {pt.XLEN{1'b1}}                        );


   assign b_ff[DIV_ITER_BITS+3:DIV_ITER_BITS] = {b_ff[DIV_ITER_BITS-1],b_ff[DIV_ITER_BITS-1],b_ff[DIV_ITER_BITS-1],b_ff[DIV_ITER_BITS-1]};


   assign adder1_out[DIV_ITER_BITS:0]     = {                      r_ff[DIV_ITER_BITS-3:0],  a_ff[DIV_ITER_BITS-1:DIV_ITER_BITS-3]}  +   b_ff[DIV_ITER_BITS:0];
   assign adder2_out[DIV_ITER_BITS+1:0]   = {                      r_ff[DIV_ITER_BITS-2:0],  a_ff[DIV_ITER_BITS-1:DIV_ITER_BITS-3]}  +  {b_ff[DIV_ITER_BITS:0],1'b0};
   assign adder3_out[DIV_ITER_BITS+2:0]   = {                      r_ff[DIV_ITER_BITS-1:0],  a_ff[DIV_ITER_BITS-1:DIV_ITER_BITS-3]}  +  {b_ff[DIV_ITER_BITS+1:0],1'b0}  +   b_ff[DIV_ITER_BITS+2:0];
   assign adder4_out[DIV_ITER_BITS+3:0]   = {r_ff[DIV_ITER_BITS-1],r_ff[DIV_ITER_BITS-1:0],  a_ff[DIV_ITER_BITS-1:DIV_ITER_BITS-3]}  +  {b_ff[DIV_ITER_BITS+1:0],2'b0};
   assign adder5_out[DIV_ITER_BITS+3:0]   = {r_ff[DIV_ITER_BITS-1],r_ff[DIV_ITER_BITS-1:0],  a_ff[DIV_ITER_BITS-1:DIV_ITER_BITS-3]}  +  {b_ff[DIV_ITER_BITS+1:0],2'b0}  +   b_ff[DIV_ITER_BITS+3:0];
   assign adder6_out[DIV_ITER_BITS+3:0]   = {r_ff[DIV_ITER_BITS-1],r_ff[DIV_ITER_BITS-1:0],  a_ff[DIV_ITER_BITS-1:DIV_ITER_BITS-3]}  +  {b_ff[DIV_ITER_BITS+1:0],2'b0}  +  {b_ff[DIV_ITER_BITS+2:0],1'b0};
   assign adder7_out[DIV_ITER_BITS+3:0]   = {r_ff[DIV_ITER_BITS-1],r_ff[DIV_ITER_BITS-1:0],  a_ff[DIV_ITER_BITS-1:DIV_ITER_BITS-3]}  +  {b_ff[DIV_ITER_BITS+1:0],2'b0}  +  {b_ff[DIV_ITER_BITS+2:0],1'b0}  +  b_ff[DIV_ITER_BITS+3:0];

   assign quotient_raw[1]        = (~adder1_out[DIV_ITER_BITS]   ^ dividend_sign_ff) | ( (a_ff[DIV_ITER_BITS-4:0] == {DIV_ITER_BITS-3{1'b0}}) & (adder1_out[DIV_ITER_BITS:0]   == {DIV_ITER_BITS+1{1'b0}}) );
   assign quotient_raw[2]        = (~adder2_out[DIV_ITER_BITS+1] ^ dividend_sign_ff) | ( (a_ff[DIV_ITER_BITS-4:0] == {DIV_ITER_BITS-3{1'b0}}) & (adder2_out[DIV_ITER_BITS+1:0] == {DIV_ITER_BITS+2{1'b0}}) );
   assign quotient_raw[3]        = (~adder3_out[DIV_ITER_BITS+2] ^ dividend_sign_ff) | ( (a_ff[DIV_ITER_BITS-4:0] == {DIV_ITER_BITS-3{1'b0}}) & (adder3_out[DIV_ITER_BITS+2:0] == {DIV_ITER_BITS+3{1'b0}}) );
   assign quotient_raw[4]        = (~adder4_out[DIV_ITER_BITS+3] ^ dividend_sign_ff) | ( (a_ff[DIV_ITER_BITS-4:0] == {DIV_ITER_BITS-3{1'b0}}) & (adder4_out[DIV_ITER_BITS+3:0] == {DIV_ITER_BITS+4{1'b0}}) );
   assign quotient_raw[5]        = (~adder5_out[DIV_ITER_BITS+3] ^ dividend_sign_ff) | ( (a_ff[DIV_ITER_BITS-4:0] == {DIV_ITER_BITS-3{1'b0}}) & (adder5_out[DIV_ITER_BITS+3:0] == {DIV_ITER_BITS+4{1'b0}}) );
   assign quotient_raw[6]        = (~adder6_out[DIV_ITER_BITS+3] ^ dividend_sign_ff) | ( (a_ff[DIV_ITER_BITS-4:0] == {DIV_ITER_BITS-3{1'b0}}) & (adder6_out[DIV_ITER_BITS+3:0] == {DIV_ITER_BITS+4{1'b0}}) );
   assign quotient_raw[7]        = (~adder7_out[DIV_ITER_BITS+3] ^ dividend_sign_ff) | ( (a_ff[DIV_ITER_BITS-4:0] == {DIV_ITER_BITS-3{1'b0}}) & (adder7_out[DIV_ITER_BITS+3:0] == {DIV_ITER_BITS+4{1'b0}}) );

   assign quotient_new[2]        = quotient_raw[7] |   quotient_raw[6] | quotient_raw[5]  |   quotient_raw[4];
   assign quotient_new[1]        = quotient_raw[7] |   quotient_raw[6] |                    (~quotient_raw[4] & quotient_raw[3]) | (~quotient_raw[3] & quotient_raw[2]);
   assign quotient_new[0]        = quotient_raw[7] | (~quotient_raw[6] & quotient_raw[5]) | (~quotient_raw[4] & quotient_raw[3]) | (~quotient_raw[2] & quotient_raw[1]);


   assign twos_comp_b_sel        =  valid_ff           & ~(dividend_sign_ff ^ divisor_sign_ff);
   assign twos_comp_q_sel        = ~valid_ff & ~rem_ff &  (dividend_sign_ff ^ divisor_sign_ff) & ~special_ff[4];

   assign twos_comp_in[pt.XLEN-1:0] =  ( {pt.XLEN{twos_comp_q_sel}} & q_ff[pt.XLEN-1:0] ) |
                                       ( {pt.XLEN{twos_comp_b_sel}} & b_ff[pt.XLEN-1:0] );

   rvtwoscomp #(pt.XLEN) i_twos_comp  (.din(twos_comp_in[pt.XLEN-1:0]), .dout(twos_comp_out[pt.XLEN-1:0]));



   assign valid_out              =  finish_ff & ~cancel;

   assign data_out[pt.XLEN-1:0]  = ( {pt.XLEN{~rem_ff & ~twos_comp_q_sel}} & q_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{ rem_ff                   }} & r_ff[pt.XLEN-1:0]          ) |
                                   ( {pt.XLEN{           twos_comp_q_sel}} & twos_comp_out[pt.XLEN-1:0] );




   // *** *** *** START : SMALLNUM {{
   assign smallnum_case          = ( (a_ff[pt.XLEN-1:4]  == {pt.XLEN-4{1'b0}}) & (b_ff[pt.XLEN-1:4] == {pt.XLEN-4{1'b0}}) & ~by_zero_case & ~rem_ff & valid_ff & ~cancel & 1'b0) |
                                   ( (a_ff[pt.XLEN-1:0]  == {pt.XLEN{1'b0}}) &                                              ~by_zero_case & ~rem_ff & valid_ff & ~cancel & 1'b0);

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

   logic [pt.XLENW:0]   dw_a_enc;
   logic [pt.XLENW:0]   dw_b_enc;
   logic [pt.XLENW+1:0] dw_shortq_raw;


   eh2_exu_div_cls i_a_cls  (
       .operand  ( shortq_dividend[pt.XLEN:0]   ),
       .cls      ( dw_a_enc[pt.XLENW-1:0]       ));

   eh2_exu_div_cls i_b_cls  (
       .operand  ( b_ff[pt.XLEN:0]        ),
       .cls      ( dw_b_enc[pt.XLENW-1:0] ));

   assign dw_a_enc[pt.XLENW]           =  1'b0;
   assign dw_b_enc[pt.XLENW]           =  1'b0;


   assign dw_shortq_raw[pt.XLENW+1:0]  =  {1'b0,dw_b_enc[pt.XLENW:0]} - {1'b0,dw_a_enc[pt.XLENW:0]} + (pt.XLENW+2)'('d1);
   assign shortq[pt.XLENW:0]           =  dw_shortq_raw[pt.XLENW+1]    ?  {pt.XLENW+1{1'b0}}  :  dw_shortq_raw[pt.XLENW:0];

   assign shortq_enable                =  valid_ff & ~shortq[pt.XLENW] & ~(shortq[pt.XLENW-1:2] ==  {pt.XLENW-2{1'b1}}) & ~cancel;

   generate
      if (pt.XLEN == 32) begin
         assign shortq_decode[pt.XLENW-1:0]  =  ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d31)}} & (pt.XLENW)'('d00)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d30)}} & (pt.XLENW)'('d00)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d29)}} & (pt.XLENW)'('d00)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d28)}} & (pt.XLENW)'('d00)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d27)}} & (pt.XLENW)'('d03)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d26)}} & (pt.XLENW)'('d06)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d25)}} & (pt.XLENW)'('d06)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d24)}} & (pt.XLENW)'('d06)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d23)}} & (pt.XLENW)'('d09)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d22)}} & (pt.XLENW)'('d09)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d21)}} & (pt.XLENW)'('d09)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d20)}} & (pt.XLENW)'('d12)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d19)}} & (pt.XLENW)'('d12)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d18)}} & (pt.XLENW)'('d12)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d17)}} & (pt.XLENW)'('d15)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d16)}} & (pt.XLENW)'('d15)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d15)}} & (pt.XLENW)'('d15)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d14)}} & (pt.XLENW)'('d18)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d13)}} & (pt.XLENW)'('d18)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d12)}} & (pt.XLENW)'('d18)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d11)}} & (pt.XLENW)'('d21)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d10)}} & (pt.XLENW)'('d21)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d09)}} & (pt.XLENW)'('d21)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d08)}} & (pt.XLENW)'('d24)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d07)}} & (pt.XLENW)'('d24)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d06)}} & (pt.XLENW)'('d24)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d05)}} & (pt.XLENW)'('d27)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d04)}} & (pt.XLENW)'('d27)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d03)}} & (pt.XLENW)'('d27)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d02)}} & (pt.XLENW)'('d27)) |  // Using 30 will violate the minimum latency required
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d01)}} & (pt.XLENW)'('d27)) |  // Using 30 will violate the minimum latency required
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d00)}} & (pt.XLENW)'('d27));   // Using 30 will violate the minimum latency required
      end else if (pt.XLEN == 64) begin
         assign shortq_decode[pt.XLENW-1:0]  =  ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d63)}} & (pt.XLENW)'('d00)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d62)}} & (pt.XLENW)'('d00)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d61)}} & (pt.XLENW)'('d00)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d60)}} & (pt.XLENW)'('d00)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d59)}} & (pt.XLENW)'('d03)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d58)}} & (pt.XLENW)'('d03)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d57)}} & (pt.XLENW)'('d03)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d56)}} & (pt.XLENW)'('d06)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d55)}} & (pt.XLENW)'('d06)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d54)}} & (pt.XLENW)'('d06)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d53)}} & (pt.XLENW)'('d09)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d52)}} & (pt.XLENW)'('d09)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d51)}} & (pt.XLENW)'('d09)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d50)}} & (pt.XLENW)'('d12)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d49)}} & (pt.XLENW)'('d12)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d48)}} & (pt.XLENW)'('d12)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d47)}} & (pt.XLENW)'('d15)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d46)}} & (pt.XLENW)'('d15)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d45)}} & (pt.XLENW)'('d15)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d44)}} & (pt.XLENW)'('d18)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d43)}} & (pt.XLENW)'('d18)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d42)}} & (pt.XLENW)'('d18)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d41)}} & (pt.XLENW)'('d21)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d40)}} & (pt.XLENW)'('d21)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d39)}} & (pt.XLENW)'('d21)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d38)}} & (pt.XLENW)'('d24)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d37)}} & (pt.XLENW)'('d24)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d36)}} & (pt.XLENW)'('d24)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d35)}} & (pt.XLENW)'('d27)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d34)}} & (pt.XLENW)'('d27)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d33)}} & (pt.XLENW)'('d27)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d32)}} & (pt.XLENW)'('d30)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d31)}} & (pt.XLENW)'('d30)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d30)}} & (pt.XLENW)'('d30)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d29)}} & (pt.XLENW)'('d33)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d28)}} & (pt.XLENW)'('d33)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d27)}} & (pt.XLENW)'('d33)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d26)}} & (pt.XLENW)'('d36)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d25)}} & (pt.XLENW)'('d36)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d24)}} & (pt.XLENW)'('d36)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d23)}} & (pt.XLENW)'('d39)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d22)}} & (pt.XLENW)'('d39)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d21)}} & (pt.XLENW)'('d39)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d20)}} & (pt.XLENW)'('d42)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d19)}} & (pt.XLENW)'('d42)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d18)}} & (pt.XLENW)'('d42)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d17)}} & (pt.XLENW)'('d45)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d16)}} & (pt.XLENW)'('d45)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d15)}} & (pt.XLENW)'('d45)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d14)}} & (pt.XLENW)'('d48)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d13)}} & (pt.XLENW)'('d48)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d12)}} & (pt.XLENW)'('d48)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d11)}} & (pt.XLENW)'('d51)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d10)}} & (pt.XLENW)'('d51)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d09)}} & (pt.XLENW)'('d51)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d08)}} & (pt.XLENW)'('d54)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d07)}} & (pt.XLENW)'('d54)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d06)}} & (pt.XLENW)'('d54)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d05)}} & (pt.XLENW)'('d57)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d04)}} & (pt.XLENW)'('d57)) |
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d03)}} & (pt.XLENW)'('d57)) |

                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d02)}} & (pt.XLENW)'('d57)) |  // Using 60 will violate the minimum latency required
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d01)}} & (pt.XLENW)'('d57)) |  // Using 60 will violate the minimum latency required
                                                ( {pt.XLENW{shortq[pt.XLENW-1:0] == pt.XLENW'('d00)}} & (pt.XLENW)'('d57));   // Using 60 will violate the minimum latency required
      end
   endgenerate


   assign shortq_shift[pt.XLENW-1:0]   = ~shortq_enable     ?  {pt.XLENW{1'b0}}  :  shortq_decode[pt.XLENW-1:0];

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


   logic                         valid_ff_in, valid_ff;
   logic                         finish_raw, finish, finish_ff;
   logic                         running_state;
   logic                         misc_enable;
   logic         [2:0]           control_in, control_ff;
   logic                         dividend_sign_ff, divisor_sign_ff, rem_ff;
   logic                         count_enable;
   logic         [6:0]           count_in, count_ff;

   logic                         smallnum_case;
   logic         [3:0]           smallnum;

   logic                         a_enable, a_shift;
   logic        [pt.XLEN-1:0]    a_in, a_ff;

   logic                         b_enable, b_twos_comp;
   logic        [pt.XLEN:0]      b_in;
   logic        [pt.XLEN+5:0]    b_ff;

   logic        [pt.XLEN-1:0]    q_in, q_ff;

   logic                         rq_enable;
   logic                         r_sign_sel;
   logic                         r_restore_sel;
   logic                         r_adder01_sel, r_adder02_sel, r_adder03_sel;
   logic                         r_adder04_sel, r_adder05_sel, r_adder06_sel, r_adder07_sel;
   logic                         r_adder08_sel, r_adder09_sel, r_adder10_sel, r_adder11_sel;
   logic                         r_adder12_sel, r_adder13_sel, r_adder14_sel, r_adder15_sel;
   logic        [pt.XLEN:0]      r_in, r_ff;

   logic                         twos_comp_q_sel, twos_comp_b_sel;
   logic        [pt.XLEN-1:0]    twos_comp_in, twos_comp_out;

   logic        [15:1]           quotient_raw;
   logic        [3:0]            quotient_new;
   logic        [pt.XLEN+2:0]    adder01_out;
   logic        [pt.XLEN+3:0]    adder02_out;
   logic        [pt.XLEN+4:0]    adder03_out;
   logic        [pt.XLEN+5:0]    adder04_out;
   logic        [pt.XLEN+5:0]    adder05_out;
   logic        [pt.XLEN+5:0]    adder06_out;
   logic        [pt.XLEN+5:0]    adder07_out;
   logic        [pt.XLEN+5:0]    adder08_out;
   logic        [pt.XLEN+5:0]    adder09_out;
   logic        [pt.XLEN+5:0]    adder10_out;
   logic        [pt.XLEN+5:0]    adder11_out;
   logic        [pt.XLEN+5:0]    adder12_out;
   logic        [pt.XLEN+5:0]    adder13_out;
   logic        [pt.XLEN+5:0]    adder14_out;
   logic        [pt.XLEN+5:0]    adder15_out;

   logic        [2*pt.XLEN:0]    ar_shifted;
   logic        [pt.XLENW:0]     shortq;
   logic        [pt.XLENW-1:0]   shortq_shift;
   logic        [pt.XLENW-1:0]   shortq_decode;
   logic        [pt.XLENW-1:0]   shortq_shift_ff;
   logic                         shortq_enable;
   logic                         shortq_enable_ff;
   logic        [pt.XLEN:0]      shortq_dividend;

   logic                         by_zero_case;

   logic         [4:1]           special_in;
   logic         [4:1]           special_ff;

   localparam unsigned MISC_FF_WIDTH = 1 + 3 + 7 + 4 + 1 + pt.XLENW + 1;
   rvdffe #(MISC_FF_WIDTH) i_misc_ff     (.*, .clk(clk), .en(misc_enable),  .din ({valid_ff_in, control_in[2:0], count_in[6:0], special_in[4:1], shortq_enable,    shortq_shift[pt.XLENW-1:0],    finish   }),
                                                                            .dout({valid_ff,    control_ff[2:0], count_ff[6:0], special_ff[4:1], shortq_enable_ff, shortq_shift_ff[pt.XLENW-1:0], finish_ff}) );

   rvdffe #(pt.XLEN)    i_a_ff           (.*, .clk(clk), .en(a_enable),     .din(a_in[pt.XLEN-1:0]),    .dout(a_ff[pt.XLEN-1:0]));
   rvdffe #(pt.XLEN+1)  i_b_ff           (.*, .clk(clk), .en(b_enable),     .din(b_in[pt.XLEN:0]),      .dout(b_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN+1)  i_r_ff           (.*, .clk(clk), .en(rq_enable),    .din(r_in[pt.XLEN:0]),      .dout(r_ff[pt.XLEN:0]));
   rvdffe #(pt.XLEN)    i_q_ff           (.*, .clk(clk), .en(rq_enable),    .din(q_in[pt.XLEN-1:0]),    .dout(q_ff[pt.XLEN-1:0]));



   assign special_in[4:1]           = {special_ff[3] & ~cancel,
                                       special_ff[2] & ~cancel,
                                       special_ff[1] & ~cancel,
                                       (smallnum_case | by_zero_case) & ~cancel};

   assign valid_ff_in               =  valid_in  & ~cancel;

   assign control_in[2]             = (~valid_in & control_ff[2]) | (valid_in & signed_in  & dividend_in[pt.XLEN-1]);
   assign control_in[1]             = (~valid_in & control_ff[1]) | (valid_in & signed_in  &  divisor_in[pt.XLEN-1]);
   assign control_in[0]             = (~valid_in & control_ff[0]) | (valid_in & rem_in);

   assign dividend_sign_ff          =  control_ff[2];
   assign divisor_sign_ff           =  control_ff[1];
   assign rem_ff                    =  control_ff[0];


   assign by_zero_case              =  valid_ff & (b_ff[pt.XLEN-1:0] == {pt.XLEN{1'b0}});

   assign misc_enable               =  valid_in | valid_ff | cancel | running_state | finish_ff;
   assign running_state             = (| count_ff[6:0]) | shortq_enable_ff;
   assign finish_raw                =   special_ff[3] |
                                       (count_ff[6:0] == 7'(pt.XLEN));


   assign finish                    =  finish_raw & ~cancel;
   assign count_enable              = (valid_ff | running_state) & ~finish & ~finish_ff & ~cancel & ~shortq_enable;
   assign count_in[6:0]             = {7{count_enable}} & (count_ff[6:0] + 7'd4 + {{7-pt.XLENW{1'b0}}, shortq_shift_ff[pt.XLENW-1:0]});


   assign a_enable                  =  valid_in | running_state;
   assign a_shift                   =  running_state & ~shortq_enable_ff;

   assign ar_shifted[2*pt.XLEN:0]   = { {pt.XLEN+1{dividend_sign_ff}} , a_ff[pt.XLEN-1:0]} << {shortq_shift_ff[pt.XLENW-1:0]};

   assign a_in[pt.XLEN-1:0]         =  ( {pt.XLEN{~a_shift & ~shortq_enable_ff}} &  dividend_in[pt.XLEN-1:0] ) |
                                       ( {pt.XLEN{ a_shift                    }} & {a_ff[pt.XLEN-5:0],4'b0}  ) |
                                       ( {pt.XLEN{            shortq_enable_ff}} &  ar_shifted[pt.XLEN-1:0]  );



   assign b_enable                  =    valid_in | b_twos_comp;
   assign b_twos_comp               =    valid_ff & ~(dividend_sign_ff ^ divisor_sign_ff);

   assign b_in[pt.XLEN:0]           = ( {pt.XLEN+1{~b_twos_comp}} & { (signed_in & divisor_in[pt.XLEN-1]),divisor_in[pt.XLEN-1:0] } ) |
                                      ( {pt.XLEN+1{ b_twos_comp}} & {~divisor_sign_ff,twos_comp_out[pt.XLEN-1:0] }                  );


   assign rq_enable                 =  valid_in | valid_ff | running_state & ~(| special_ff[3:1]);
   assign r_sign_sel                =  valid_ff      &  dividend_sign_ff & ~by_zero_case;
   assign r_restore_sel             =  running_state & (quotient_new[3:0] == 4'd00) & ~shortq_enable_ff;
   assign r_adder01_sel             =  running_state & (quotient_new[3:0] == 4'd01) & ~shortq_enable_ff;
   assign r_adder02_sel             =  running_state & (quotient_new[3:0] == 4'd02) & ~shortq_enable_ff;
   assign r_adder03_sel             =  running_state & (quotient_new[3:0] == 4'd03) & ~shortq_enable_ff;
   assign r_adder04_sel             =  running_state & (quotient_new[3:0] == 4'd04) & ~shortq_enable_ff;
   assign r_adder05_sel             =  running_state & (quotient_new[3:0] == 4'd05) & ~shortq_enable_ff;
   assign r_adder06_sel             =  running_state & (quotient_new[3:0] == 4'd06) & ~shortq_enable_ff;
   assign r_adder07_sel             =  running_state & (quotient_new[3:0] == 4'd07) & ~shortq_enable_ff;
   assign r_adder08_sel             =  running_state & (quotient_new[3:0] == 4'd08) & ~shortq_enable_ff;
   assign r_adder09_sel             =  running_state & (quotient_new[3:0] == 4'd09) & ~shortq_enable_ff;
   assign r_adder10_sel             =  running_state & (quotient_new[3:0] == 4'd10) & ~shortq_enable_ff;
   assign r_adder11_sel             =  running_state & (quotient_new[3:0] == 4'd11) & ~shortq_enable_ff;
   assign r_adder12_sel             =  running_state & (quotient_new[3:0] == 4'd12) & ~shortq_enable_ff;
   assign r_adder13_sel             =  running_state & (quotient_new[3:0] == 4'd13) & ~shortq_enable_ff;
   assign r_adder14_sel             =  running_state & (quotient_new[3:0] == 4'd14) & ~shortq_enable_ff;
   assign r_adder15_sel             =  running_state & (quotient_new[3:0] == 4'd15) & ~shortq_enable_ff;

   assign r_in[pt.XLEN:0]           = ( {pt.XLEN+1{r_sign_sel      }} & {pt.XLEN+1{1'b1}}                             ) |
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


   assign q_in[pt.XLEN-1:0]         = ( {pt.XLEN{~valid_ff     }} & {q_ff[pt.XLEN-5:0], quotient_new[3:0]} ) |
                                      ( {pt.XLEN{ smallnum_case}} & {{pt.XLEN-4{1'b0}}, smallnum[3:0]}     ) |
                                      ( {pt.XLEN{ by_zero_case }} & {pt.XLEN{1'b1}}                        );


   assign b_ff[pt.XLEN+5:pt.XLEN+1] = {b_ff[pt.XLEN],b_ff[pt.XLEN],b_ff[pt.XLEN],b_ff[pt.XLEN],b_ff[pt.XLEN]};


   assign adder01_out[pt.XLEN+2:0]  = {              r_ff[pt.XLEN-2:0],a_ff[pt.XLEN-1:pt.XLEN-4]}  +                                                                                           b_ff[pt.XLEN+2:0];
   assign adder02_out[pt.XLEN+3:0]  = {              r_ff[pt.XLEN-1:0],a_ff[pt.XLEN-1:pt.XLEN-4]}  +                                                             {b_ff[pt.XLEN+2:0],1'b0};
   assign adder03_out[pt.XLEN+4:0]  = {              r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +                                                             {b_ff[pt.XLEN+3:0],1'b0}  +   b_ff[pt.XLEN+4:0];
   assign adder04_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +                               {b_ff[pt.XLEN+3:0],2'b0};
   assign adder05_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +                               {b_ff[pt.XLEN+3:0],2'b0}  +                                 b_ff[pt.XLEN+5:0];
   assign adder06_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +                               {b_ff[pt.XLEN+3:0],2'b0}  +  {b_ff[pt.XLEN+4:0],1'b0};
   assign adder07_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +                               {b_ff[pt.XLEN+3:0],2'b0}  +  {b_ff[pt.XLEN+4:0],1'b0}  +    b_ff[pt.XLEN+5:0];
   assign adder08_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0};
   assign adder09_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +                                                               b_ff[pt.XLEN+5:0];
   assign adder10_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +                                 {b_ff[pt.XLEN+4:0],1'b0};
   assign adder11_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +                                 {b_ff[pt.XLEN+4:0],1'b0}  +   b_ff[pt.XLEN+5:0];
   assign adder12_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +   {b_ff[pt.XLEN+3:0],2'b0};
   assign adder13_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +   {b_ff[pt.XLEN+3:0],2'b0}  +                                 b_ff[pt.XLEN+5:0];
   assign adder14_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +   {b_ff[pt.XLEN+3:0],2'b0}  +  {b_ff[pt.XLEN+4:0],1'b0};
   assign adder15_out[pt.XLEN+5:0]  = {r_ff[pt.XLEN],r_ff[pt.XLEN:0],  a_ff[pt.XLEN-1:pt.XLEN-4]}  +  {b_ff[pt.XLEN+2:0],3'b0} +   {b_ff[pt.XLEN+3:0],2'b0}  +  {b_ff[pt.XLEN+4:0],1'b0}  +    b_ff[pt.XLEN+5:0];

   assign quotient_raw[01]          = (~adder01_out[pt.XLEN+2] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder01_out[pt.XLEN+2:0] == {pt.XLEN+3{1'b0}}) );
   assign quotient_raw[02]          = (~adder02_out[pt.XLEN+3] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder02_out[pt.XLEN+3:0] == {pt.XLEN+4{1'b0}}) );
   assign quotient_raw[03]          = (~adder03_out[pt.XLEN+4] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder03_out[pt.XLEN+4:0] == {pt.XLEN+5{1'b0}}) );
   assign quotient_raw[04]          = (~adder04_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder04_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[05]          = (~adder05_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder05_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[06]          = (~adder06_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder06_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[07]          = (~adder07_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder07_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[08]          = (~adder08_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder08_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[09]          = (~adder09_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder09_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[10]          = (~adder10_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder10_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[11]          = (~adder11_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder11_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[12]          = (~adder12_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder12_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[13]          = (~adder13_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder13_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[14]          = (~adder14_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder14_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );
   assign quotient_raw[15]          = (~adder15_out[pt.XLEN+5] ^ dividend_sign_ff) | ( (a_ff[pt.XLEN-5:0] == {pt.XLEN-4{1'b0}}) & (adder15_out[pt.XLEN+5:0] == {pt.XLEN+6{1'b0}}) );


   assign quotient_new[0]           = ( quotient_raw[15:01] == 15'b000_0000_0000_0001 ) |  //  1
                                      ( quotient_raw[15:03] == 13'b000_0000_0000_01   ) |  //  3
                                      ( quotient_raw[15:05] == 11'b000_0000_0001      ) |  //  5
                                      ( quotient_raw[15:07] ==  9'b000_0000_01        ) |  //  7
                                      ( quotient_raw[15:09] ==  7'b000_0001           ) |  //  9
                                      ( quotient_raw[15:11] ==  5'b000_01             ) |  // 11
                                      ( quotient_raw[15:13] ==  3'b001                ) |  // 13
                                      ( quotient_raw[   15] ==  1'b1                  );   // 15

   assign quotient_new[1]           = ( quotient_raw[15:02] == 14'b000_0000_0000_001  ) |  //  2
                                      ( quotient_raw[15:03] == 13'b000_0000_0000_01   ) |  //  3
                                      ( quotient_raw[15:06] == 10'b000_0000_001       ) |  //  6
                                      ( quotient_raw[15:07] ==  9'b000_0000_01        ) |  //  7
                                      ( quotient_raw[15:10] ==  6'b000_001            ) |  // 10
                                      ( quotient_raw[15:11] ==  5'b000_01             ) |  // 11
                                      ( quotient_raw[15:14] ==  2'b01                 ) |  // 14
                                      ( quotient_raw[   15] ==  1'b1                  );   // 15

   assign quotient_new[2]           = ( quotient_raw[15:04] == 12'b000_0000_0000_1    ) |  //  4
                                      ( quotient_raw[15:05] == 11'b000_0000_0001      ) |  //  5
                                      ( quotient_raw[15:06] == 10'b000_0000_001       ) |  //  6
                                      ( quotient_raw[15:07] ==  9'b000_0000_01        ) |  //  7
                                      ( quotient_raw[15:12] ==  4'b000_1              ) |  // 12
                                      ( quotient_raw[15:13] ==  3'b001                ) |  // 13
                                      ( quotient_raw[15:14] ==  2'b01                 ) |  // 14
                                      ( quotient_raw[   15] ==  1'b1                  );   // 15

   assign quotient_new[3]           = ( quotient_raw[15:08] ==  8'b000_0000_1         ) |  //  8
                                      ( quotient_raw[15:09] ==  7'b000_0001           ) |  //  9
                                      ( quotient_raw[15:10] ==  6'b000_001            ) |  // 10
                                      ( quotient_raw[15:11] ==  5'b000_01             ) |  // 11
                                      ( quotient_raw[15:12] ==  4'b000_1              ) |  // 12
                                      ( quotient_raw[15:13] ==  3'b001                ) |  // 13
                                      ( quotient_raw[15:14] ==  2'b01                 ) |  // 14
                                      ( quotient_raw[   15] ==  1'b1                  );   // 15


   assign twos_comp_b_sel           =  valid_ff           & ~(dividend_sign_ff ^ divisor_sign_ff);
   assign twos_comp_q_sel           = ~valid_ff & ~rem_ff &  (dividend_sign_ff ^ divisor_sign_ff) & ~special_ff[4];

   assign twos_comp_in[pt.XLEN-1:0] =  ( {pt.XLEN{twos_comp_q_sel}} & q_ff[pt.XLEN-1:0] ) |
                                       ( {pt.XLEN{twos_comp_b_sel}} & b_ff[pt.XLEN-1:0] );

   rvtwoscomp #(pt.XLEN) i_twos_comp   (.din(twos_comp_in[pt.XLEN-1:0]), .dout(twos_comp_out[pt.XLEN-1:0]));



   assign valid_out                 =  finish_ff & ~cancel;

   assign data_out[pt.XLEN-1:0]     = ( {pt.XLEN{~rem_ff & ~twos_comp_q_sel}} & q_ff[pt.XLEN-1:0]          ) |
                                      ( {pt.XLEN{ rem_ff                   }} & r_ff[pt.XLEN-1:0]          ) |
                                      ( {pt.XLEN{           twos_comp_q_sel}} & twos_comp_out[pt.XLEN-1:0] );




   // *** *** *** START : SMALLNUM {{

   assign smallnum_case             = ( (a_ff[pt.XLEN-1:4]  == {pt.XLEN-4{1'b0}}) & (b_ff[pt.XLEN-1:4] == {pt.XLEN-4{1'b0}}) & ~by_zero_case & ~rem_ff & valid_ff & ~cancel & 1'b0) |
                                      ( (a_ff[pt.XLEN-1:0]  == {pt.XLEN{1'b0}}) &                                              ~by_zero_case & ~rem_ff & valid_ff & ~cancel & 1'b0);

   assign smallnum[3]               = ( a_ff[3] &                                  ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           );

   assign smallnum[2]               = ( a_ff[3] &                                  ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                      (            a_ff[2] &                       ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                      ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] & ~b_ff[2]                      );

   assign smallnum[1]               = (            a_ff[2] &                       ~b_ff[3] & ~b_ff[2] &            ~b_ff[0]) |
                                      (                       a_ff[1] &            ~b_ff[3] & ~b_ff[2] & ~b_ff[1]           ) |
                                      ( a_ff[3] &                                  ~b_ff[3] &            ~b_ff[1] & ~b_ff[0]) |
                                      ( a_ff[3] & ~a_ff[2] &                       ~b_ff[3] & ~b_ff[2] &  b_ff[1] &  b_ff[0]) |
                                      (~a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] & ~b_ff[2]                      ) |
                                      ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] &                       ~b_ff[0]) |
                                      ( a_ff[3] &  a_ff[2] &                       ~b_ff[3] &  b_ff[2] & ~b_ff[1]           ) |
                                      ( a_ff[3] &             a_ff[1] &            ~b_ff[3] &            ~b_ff[1]           ) |
                                      ( a_ff[3] &  a_ff[2] &  a_ff[1] &            ~b_ff[3] &  b_ff[2]                      );

   assign smallnum[0]               = (            a_ff[2] &  a_ff[1] &  a_ff[0] & ~b_ff[3] &            ~b_ff[1]           ) |
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

   assign shortq_dividend[pt.XLEN-1:0]   = {dividend_sign_ff,a_ff[pt.XLEN-1:0]};

   logic [pt.XLENW:0]      dw_a_enc;
   logic [pt.XLENW:0]      dw_b_enc;
   logic [pt.XLENW+1:0]    dw_shortq_raw;


   eh2_exu_div_cls i_a_cls  (
       .operand  ( shortq_dividend[pt.XLEN:0]  ),
       .cls      ( dw_a_enc[pt.XLENW-1:0]      ));

   eh2_exu_div_cls i_b_cls  (
       .operand  ( b_ff[pt.XLEN:0]             ),
       .cls      ( dw_b_enc[pt.XLENW-1:0]      ));

   assign dw_a_enc[pt.XLENW]           =  1'b0;
   assign dw_b_enc[pt.XLENW]           =  1'b0;


   assign dw_shortq_raw[pt.XLENW+1:0]  =  {1'b0,dw_b_enc[pt.XLENW:0]} - {1'b0,dw_a_enc[pt.XLENW:0]} + (pt.XLENW+2)'('d1);
   assign shortq[pt.XLENW:0]           =  dw_shortq_raw[pt.XLENW+1]  ?  {pt.XLENW+1{1'b0}}  :  dw_shortq_raw[pt.XLENW:0];

   assign shortq_enable                =  valid_ff & ~shortq[pt.XLENW] & ~(shortq[pt.XLENW-1:2] ==  {pt.XLENW-2{1'b1}}) & ~cancel;

   generate
      if (pt.XLEN == 32) begin
         assign shortq_decode[pt.XLENW-1:0]      = ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d31)}} & (pt.XLENW)'('d00)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d30)}} & (pt.XLENW)'('d00)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d29)}} & (pt.XLENW)'('d00)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d28)}} & (pt.XLENW)'('d00)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d27)}} & (pt.XLENW)'('d04)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d26)}} & (pt.XLENW)'('d04)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d25)}} & (pt.XLENW)'('d04)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d24)}} & (pt.XLENW)'('d04)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d23)}} & (pt.XLENW)'('d08)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d22)}} & (pt.XLENW)'('d08)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d21)}} & (pt.XLENW)'('d08)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d20)}} & (pt.XLENW)'('d08)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d19)}} & (pt.XLENW)'('d12)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d18)}} & (pt.XLENW)'('d12)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d17)}} & (pt.XLENW)'('d12)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d16)}} & (pt.XLENW)'('d12)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d15)}} & (pt.XLENW)'('d16)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d14)}} & (pt.XLENW)'('d16)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d13)}} & (pt.XLENW)'('d16)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d12)}} & (pt.XLENW)'('d16)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d11)}} & (pt.XLENW)'('d20)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d10)}} & (pt.XLENW)'('d20)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d09)}} & (pt.XLENW)'('d20)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d08)}} & (pt.XLENW)'('d20)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d07)}} & (pt.XLENW)'('d24)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d06)}} & (pt.XLENW)'('d24)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d05)}} & (pt.XLENW)'('d24)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d04)}} & (pt.XLENW)'('d24)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d03)}} & (pt.XLENW)'('d24)) |  // Using 28 will violate the minimum latency required (div cannot finish in less than 2 cycles)
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d02)}} & (pt.XLENW)'('d24)) |  // Using 28 will violate the minimum latency required
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d01)}} & (pt.XLENW)'('d24)) |  // Using 28 will violate the minimum latency required
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d00)}} & (pt.XLENW)'('d24));   // Using 28 will violate the minimum latency required
      end else if (pt.XLEN == 64) begin
         assign shortq_decode[pt.XLENW-1:0]      = ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d63)}} & (pt.XLENW)'('d00)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d62)}} & (pt.XLENW)'('d00)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d61)}} & (pt.XLENW)'('d00)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d60)}} & (pt.XLENW)'('d00)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d59)}} & (pt.XLENW)'('d04)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d58)}} & (pt.XLENW)'('d04)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d57)}} & (pt.XLENW)'('d04)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d56)}} & (pt.XLENW)'('d04)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d55)}} & (pt.XLENW)'('d08)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d54)}} & (pt.XLENW)'('d08)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d53)}} & (pt.XLENW)'('d08)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d52)}} & (pt.XLENW)'('d08)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d51)}} & (pt.XLENW)'('d12)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d50)}} & (pt.XLENW)'('d12)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d49)}} & (pt.XLENW)'('d12)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d48)}} & (pt.XLENW)'('d12)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d47)}} & (pt.XLENW)'('d16)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d46)}} & (pt.XLENW)'('d16)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d45)}} & (pt.XLENW)'('d16)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d44)}} & (pt.XLENW)'('d16)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d43)}} & (pt.XLENW)'('d20)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d42)}} & (pt.XLENW)'('d20)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d41)}} & (pt.XLENW)'('d20)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d40)}} & (pt.XLENW)'('d20)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d39)}} & (pt.XLENW)'('d24)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d38)}} & (pt.XLENW)'('d24)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d37)}} & (pt.XLENW)'('d24)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d36)}} & (pt.XLENW)'('d24)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d35)}} & (pt.XLENW)'('d28)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d34)}} & (pt.XLENW)'('d28)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d33)}} & (pt.XLENW)'('d28)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d32)}} & (pt.XLENW)'('d28)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d31)}} & (pt.XLENW)'('d32)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d30)}} & (pt.XLENW)'('d32)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d29)}} & (pt.XLENW)'('d32)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d28)}} & (pt.XLENW)'('d32)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d27)}} & (pt.XLENW)'('d36)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d26)}} & (pt.XLENW)'('d36)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d25)}} & (pt.XLENW)'('d36)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d24)}} & (pt.XLENW)'('d36)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d23)}} & (pt.XLENW)'('d40)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d22)}} & (pt.XLENW)'('d40)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d21)}} & (pt.XLENW)'('d40)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d20)}} & (pt.XLENW)'('d40)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d19)}} & (pt.XLENW)'('d44)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d18)}} & (pt.XLENW)'('d44)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d17)}} & (pt.XLENW)'('d44)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d16)}} & (pt.XLENW)'('d44)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d15)}} & (pt.XLENW)'('d48)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d14)}} & (pt.XLENW)'('d48)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d13)}} & (pt.XLENW)'('d48)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d12)}} & (pt.XLENW)'('d48)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d11)}} & (pt.XLENW)'('d52)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d10)}} & (pt.XLENW)'('d52)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d09)}} & (pt.XLENW)'('d52)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d08)}} & (pt.XLENW)'('d52)) |

                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d07)}} & (pt.XLENW)'('d56)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d06)}} & (pt.XLENW)'('d56)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d05)}} & (pt.XLENW)'('d56)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d04)}} & (pt.XLENW)'('d56)) |
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d03)}} & (pt.XLENW)'('d56)) |  // Using 60 will violate the minimum latency required (div cannot finish in less than 2 cycles)
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d02)}} & (pt.XLENW)'('d56)) |  // Using 60 will violate the minimum latency required
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d01)}} & (pt.XLENW)'('d56)) |  // Using 60 will violate the minimum latency required
                                                   ( {pt.XLENW{shortq[pt.XLENW-1:0] == (pt.XLENW)'('d00)}} & (pt.XLENW)'('d56));   // Using 60 will violate the minimum latency required
      end
   endgenerate

   assign shortq_shift[pt.XLENW-1:0]       = ~shortq_enable     ?  {pt.XLENW{1'b0}}  :  shortq_decode[pt.XLENW-1:0];

   // *** *** *** End   : Short Q }}





endmodule // eh2_exu_div_new_4bit_fullshortq






// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
module eh2_exu_div_cls
  (
   input  logic [`RV_XLEN:0]     operand,

   output logic [`RV_XLENW-1:0]  cls                  // Count leading sign bits - "n" format ignoring [pt.XLEN]
   );


   logic [`RV_XLENW-1:0]         cls_zeros;
   logic [`RV_XLENW-1:0]         cls_ones;

   generate
      if (`RV_XLEN == 32) begin
         assign cls_zeros[`RV_XLENW-1:0]  =  ({`RV_XLENW{operand[31]    ==  {           1'b1} }} & (`RV_XLENW)'('d00)) |
                                             ({`RV_XLENW{operand[31:30] ==  {{ 1{1'b0}},1'b1} }} & (`RV_XLENW)'('d01)) |
                                             ({`RV_XLENW{operand[31:29] ==  {{ 2{1'b0}},1'b1} }} & (`RV_XLENW)'('d02)) |
                                             ({`RV_XLENW{operand[31:28] ==  {{ 3{1'b0}},1'b1} }} & (`RV_XLENW)'('d03)) |
                                             ({`RV_XLENW{operand[31:27] ==  {{ 4{1'b0}},1'b1} }} & (`RV_XLENW)'('d04)) |
                                             ({`RV_XLENW{operand[31:26] ==  {{ 5{1'b0}},1'b1} }} & (`RV_XLENW)'('d05)) |
                                             ({`RV_XLENW{operand[31:25] ==  {{ 6{1'b0}},1'b1} }} & (`RV_XLENW)'('d06)) |
                                             ({`RV_XLENW{operand[31:24] ==  {{ 7{1'b0}},1'b1} }} & (`RV_XLENW)'('d07)) |
                                             ({`RV_XLENW{operand[31:23] ==  {{ 8{1'b0}},1'b1} }} & (`RV_XLENW)'('d08)) |
                                             ({`RV_XLENW{operand[31:22] ==  {{ 9{1'b0}},1'b1} }} & (`RV_XLENW)'('d09)) |
                                             ({`RV_XLENW{operand[31:21] ==  {{10{1'b0}},1'b1} }} & (`RV_XLENW)'('d10)) |
                                             ({`RV_XLENW{operand[31:20] ==  {{11{1'b0}},1'b1} }} & (`RV_XLENW)'('d11)) |
                                             ({`RV_XLENW{operand[31:19] ==  {{12{1'b0}},1'b1} }} & (`RV_XLENW)'('d12)) |
                                             ({`RV_XLENW{operand[31:18] ==  {{13{1'b0}},1'b1} }} & (`RV_XLENW)'('d13)) |
                                             ({`RV_XLENW{operand[31:17] ==  {{14{1'b0}},1'b1} }} & (`RV_XLENW)'('d14)) |
                                             ({`RV_XLENW{operand[31:16] ==  {{15{1'b0}},1'b1} }} & (`RV_XLENW)'('d15)) |
                                             ({`RV_XLENW{operand[31:15] ==  {{16{1'b0}},1'b1} }} & (`RV_XLENW)'('d16)) |
                                             ({`RV_XLENW{operand[31:14] ==  {{17{1'b0}},1'b1} }} & (`RV_XLENW)'('d17)) |
                                             ({`RV_XLENW{operand[31:13] ==  {{18{1'b0}},1'b1} }} & (`RV_XLENW)'('d18)) |
                                             ({`RV_XLENW{operand[31:12] ==  {{19{1'b0}},1'b1} }} & (`RV_XLENW)'('d19)) |
                                             ({`RV_XLENW{operand[31:11] ==  {{20{1'b0}},1'b1} }} & (`RV_XLENW)'('d20)) |
                                             ({`RV_XLENW{operand[31:10] ==  {{21{1'b0}},1'b1} }} & (`RV_XLENW)'('d21)) |
                                             ({`RV_XLENW{operand[31:09] ==  {{22{1'b0}},1'b1} }} & (`RV_XLENW)'('d22)) |
                                             ({`RV_XLENW{operand[31:08] ==  {{23{1'b0}},1'b1} }} & (`RV_XLENW)'('d23)) |
                                             ({`RV_XLENW{operand[31:07] ==  {{24{1'b0}},1'b1} }} & (`RV_XLENW)'('d24)) |
                                             ({`RV_XLENW{operand[31:06] ==  {{25{1'b0}},1'b1} }} & (`RV_XLENW)'('d25)) |
                                             ({`RV_XLENW{operand[31:05] ==  {{26{1'b0}},1'b1} }} & (`RV_XLENW)'('d26)) |
                                             ({`RV_XLENW{operand[31:04] ==  {{27{1'b0}},1'b1} }} & (`RV_XLENW)'('d27)) |
                                             ({`RV_XLENW{operand[31:03] ==  {{28{1'b0}},1'b1} }} & (`RV_XLENW)'('d28)) |
                                             ({`RV_XLENW{operand[31:02] ==  {{29{1'b0}},1'b1} }} & (`RV_XLENW)'('d29)) |
                                             ({`RV_XLENW{operand[31:01] ==  {{30{1'b0}},1'b1} }} & (`RV_XLENW)'('d30)) |
                                             ({`RV_XLENW{operand[31:00] ==  {{31{1'b0}},1'b1} }} & (`RV_XLENW)'('d31)) |
                                             ({`RV_XLENW{operand[31:00] ==  {{32{1'b0}}     } }} & (`RV_XLENW)'('d00));    // Don't care case as it will be handled as special case


         assign cls_ones[`RV_XLENW-1:0]   =  ({`RV_XLENW{operand[31:30] ==  {{ 1{1'b1}},1'b0} }} & (`RV_XLENW)'('d00)) |
                                             ({`RV_XLENW{operand[31:29] ==  {{ 2{1'b1}},1'b0} }} & (`RV_XLENW)'('d01)) |
                                             ({`RV_XLENW{operand[31:28] ==  {{ 3{1'b1}},1'b0} }} & (`RV_XLENW)'('d02)) |
                                             ({`RV_XLENW{operand[31:27] ==  {{ 4{1'b1}},1'b0} }} & (`RV_XLENW)'('d03)) |
                                             ({`RV_XLENW{operand[31:26] ==  {{ 5{1'b1}},1'b0} }} & (`RV_XLENW)'('d04)) |
                                             ({`RV_XLENW{operand[31:25] ==  {{ 6{1'b1}},1'b0} }} & (`RV_XLENW)'('d05)) |
                                             ({`RV_XLENW{operand[31:24] ==  {{ 7{1'b1}},1'b0} }} & (`RV_XLENW)'('d06)) |
                                             ({`RV_XLENW{operand[31:23] ==  {{ 8{1'b1}},1'b0} }} & (`RV_XLENW)'('d07)) |
                                             ({`RV_XLENW{operand[31:22] ==  {{ 9{1'b1}},1'b0} }} & (`RV_XLENW)'('d08)) |
                                             ({`RV_XLENW{operand[31:21] ==  {{10{1'b1}},1'b0} }} & (`RV_XLENW)'('d09)) |
                                             ({`RV_XLENW{operand[31:20] ==  {{11{1'b1}},1'b0} }} & (`RV_XLENW)'('d10)) |
                                             ({`RV_XLENW{operand[31:19] ==  {{12{1'b1}},1'b0} }} & (`RV_XLENW)'('d11)) |
                                             ({`RV_XLENW{operand[31:18] ==  {{13{1'b1}},1'b0} }} & (`RV_XLENW)'('d12)) |
                                             ({`RV_XLENW{operand[31:17] ==  {{14{1'b1}},1'b0} }} & (`RV_XLENW)'('d13)) |
                                             ({`RV_XLENW{operand[31:16] ==  {{15{1'b1}},1'b0} }} & (`RV_XLENW)'('d14)) |
                                             ({`RV_XLENW{operand[31:15] ==  {{16{1'b1}},1'b0} }} & (`RV_XLENW)'('d15)) |
                                             ({`RV_XLENW{operand[31:14] ==  {{17{1'b1}},1'b0} }} & (`RV_XLENW)'('d16)) |
                                             ({`RV_XLENW{operand[31:13] ==  {{18{1'b1}},1'b0} }} & (`RV_XLENW)'('d17)) |
                                             ({`RV_XLENW{operand[31:12] ==  {{19{1'b1}},1'b0} }} & (`RV_XLENW)'('d18)) |
                                             ({`RV_XLENW{operand[31:11] ==  {{20{1'b1}},1'b0} }} & (`RV_XLENW)'('d19)) |
                                             ({`RV_XLENW{operand[31:10] ==  {{21{1'b1}},1'b0} }} & (`RV_XLENW)'('d20)) |
                                             ({`RV_XLENW{operand[31:09] ==  {{22{1'b1}},1'b0} }} & (`RV_XLENW)'('d21)) |
                                             ({`RV_XLENW{operand[31:08] ==  {{23{1'b1}},1'b0} }} & (`RV_XLENW)'('d22)) |
                                             ({`RV_XLENW{operand[31:07] ==  {{24{1'b1}},1'b0} }} & (`RV_XLENW)'('d23)) |
                                             ({`RV_XLENW{operand[31:06] ==  {{25{1'b1}},1'b0} }} & (`RV_XLENW)'('d24)) |
                                             ({`RV_XLENW{operand[31:05] ==  {{26{1'b1}},1'b0} }} & (`RV_XLENW)'('d25)) |
                                             ({`RV_XLENW{operand[31:04] ==  {{27{1'b1}},1'b0} }} & (`RV_XLENW)'('d26)) |
                                             ({`RV_XLENW{operand[31:03] ==  {{28{1'b1}},1'b0} }} & (`RV_XLENW)'('d27)) |
                                             ({`RV_XLENW{operand[31:02] ==  {{29{1'b1}},1'b0} }} & (`RV_XLENW)'('d28)) |
                                             ({`RV_XLENW{operand[31:01] ==  {{30{1'b1}},1'b0} }} & (`RV_XLENW)'('d29)) |
                                             ({`RV_XLENW{operand[31:00] ==  {{31{1'b1}},1'b0} }} & (`RV_XLENW)'('d30)) |
                                             ({`RV_XLENW{operand[31:00] ==  {{32{1'b1}}     } }} & (`RV_XLENW)'('d31));
      end else if (`RV_XLEN == 64) begin
         assign cls_zeros[`RV_XLENW-1:0] =   ({`RV_XLENW{operand[63]    ==  {           1'b1} }} & (`RV_XLENW)'('d00)) |
                                             ({`RV_XLENW{operand[63:62] ==  {{ 1{1'b0}},1'b1} }} & (`RV_XLENW)'('d01)) |
                                             ({`RV_XLENW{operand[63:61] ==  {{ 2{1'b0}},1'b1} }} & (`RV_XLENW)'('d02)) |
                                             ({`RV_XLENW{operand[63:60] ==  {{ 3{1'b0}},1'b1} }} & (`RV_XLENW)'('d03)) |
                                             ({`RV_XLENW{operand[63:59] ==  {{ 4{1'b0}},1'b1} }} & (`RV_XLENW)'('d04)) |
                                             ({`RV_XLENW{operand[63:58] ==  {{ 5{1'b0}},1'b1} }} & (`RV_XLENW)'('d05)) |
                                             ({`RV_XLENW{operand[63:57] ==  {{ 6{1'b0}},1'b1} }} & (`RV_XLENW)'('d06)) |
                                             ({`RV_XLENW{operand[63:56] ==  {{ 7{1'b0}},1'b1} }} & (`RV_XLENW)'('d07)) |
                                             ({`RV_XLENW{operand[63:55] ==  {{ 8{1'b0}},1'b1} }} & (`RV_XLENW)'('d08)) |
                                             ({`RV_XLENW{operand[63:54] ==  {{ 9{1'b0}},1'b1} }} & (`RV_XLENW)'('d09)) |
                                             ({`RV_XLENW{operand[63:53] ==  {{10{1'b0}},1'b1} }} & (`RV_XLENW)'('d10)) |
                                             ({`RV_XLENW{operand[63:52] ==  {{11{1'b0}},1'b1} }} & (`RV_XLENW)'('d11)) |
                                             ({`RV_XLENW{operand[63:51] ==  {{12{1'b0}},1'b1} }} & (`RV_XLENW)'('d12)) |
                                             ({`RV_XLENW{operand[63:50] ==  {{13{1'b0}},1'b1} }} & (`RV_XLENW)'('d13)) |
                                             ({`RV_XLENW{operand[63:49] ==  {{14{1'b0}},1'b1} }} & (`RV_XLENW)'('d14)) |
                                             ({`RV_XLENW{operand[63:48] ==  {{15{1'b0}},1'b1} }} & (`RV_XLENW)'('d15)) |
                                             ({`RV_XLENW{operand[63:47] ==  {{16{1'b0}},1'b1} }} & (`RV_XLENW)'('d16)) |
                                             ({`RV_XLENW{operand[63:46] ==  {{17{1'b0}},1'b1} }} & (`RV_XLENW)'('d17)) |
                                             ({`RV_XLENW{operand[63:45] ==  {{18{1'b0}},1'b1} }} & (`RV_XLENW)'('d18)) |
                                             ({`RV_XLENW{operand[63:44] ==  {{19{1'b0}},1'b1} }} & (`RV_XLENW)'('d19)) |
                                             ({`RV_XLENW{operand[63:43] ==  {{20{1'b0}},1'b1} }} & (`RV_XLENW)'('d20)) |
                                             ({`RV_XLENW{operand[63:42] ==  {{21{1'b0}},1'b1} }} & (`RV_XLENW)'('d21)) |
                                             ({`RV_XLENW{operand[63:41] ==  {{22{1'b0}},1'b1} }} & (`RV_XLENW)'('d22)) |
                                             ({`RV_XLENW{operand[63:40] ==  {{23{1'b0}},1'b1} }} & (`RV_XLENW)'('d23)) |
                                             ({`RV_XLENW{operand[63:39] ==  {{24{1'b0}},1'b1} }} & (`RV_XLENW)'('d24)) |
                                             ({`RV_XLENW{operand[63:38] ==  {{25{1'b0}},1'b1} }} & (`RV_XLENW)'('d25)) |
                                             ({`RV_XLENW{operand[63:37] ==  {{26{1'b0}},1'b1} }} & (`RV_XLENW)'('d26)) |
                                             ({`RV_XLENW{operand[63:36] ==  {{27{1'b0}},1'b1} }} & (`RV_XLENW)'('d27)) |
                                             ({`RV_XLENW{operand[63:35] ==  {{28{1'b0}},1'b1} }} & (`RV_XLENW)'('d28)) |
                                             ({`RV_XLENW{operand[63:34] ==  {{29{1'b0}},1'b1} }} & (`RV_XLENW)'('d29)) |
                                             ({`RV_XLENW{operand[63:33] ==  {{30{1'b0}},1'b1} }} & (`RV_XLENW)'('d30)) |
                                             ({`RV_XLENW{operand[63:32] ==  {{31{1'b0}},1'b1} }} & (`RV_XLENW)'('d31)) |
                                             ({`RV_XLENW{operand[63:31] ==  {{32{1'b0}},1'b1} }} & (`RV_XLENW)'('d32)) |
                                             ({`RV_XLENW{operand[63:30] ==  {{33{1'b0}},1'b1} }} & (`RV_XLENW)'('d33)) |
                                             ({`RV_XLENW{operand[63:29] ==  {{34{1'b0}},1'b1} }} & (`RV_XLENW)'('d34)) |
                                             ({`RV_XLENW{operand[63:28] ==  {{35{1'b0}},1'b1} }} & (`RV_XLENW)'('d35)) |
                                             ({`RV_XLENW{operand[63:27] ==  {{36{1'b0}},1'b1} }} & (`RV_XLENW)'('d36)) |
                                             ({`RV_XLENW{operand[63:26] ==  {{37{1'b0}},1'b1} }} & (`RV_XLENW)'('d37)) |
                                             ({`RV_XLENW{operand[63:25] ==  {{38{1'b0}},1'b1} }} & (`RV_XLENW)'('d38)) |
                                             ({`RV_XLENW{operand[63:24] ==  {{39{1'b0}},1'b1} }} & (`RV_XLENW)'('d39)) |
                                             ({`RV_XLENW{operand[63:23] ==  {{40{1'b0}},1'b1} }} & (`RV_XLENW)'('d40)) |
                                             ({`RV_XLENW{operand[63:22] ==  {{41{1'b0}},1'b1} }} & (`RV_XLENW)'('d41)) |
                                             ({`RV_XLENW{operand[63:21] ==  {{42{1'b0}},1'b1} }} & (`RV_XLENW)'('d42)) |
                                             ({`RV_XLENW{operand[63:20] ==  {{43{1'b0}},1'b1} }} & (`RV_XLENW)'('d43)) |
                                             ({`RV_XLENW{operand[63:19] ==  {{44{1'b0}},1'b1} }} & (`RV_XLENW)'('d44)) |
                                             ({`RV_XLENW{operand[63:18] ==  {{45{1'b0}},1'b1} }} & (`RV_XLENW)'('d45)) |
                                             ({`RV_XLENW{operand[63:17] ==  {{46{1'b0}},1'b1} }} & (`RV_XLENW)'('d46)) |
                                             ({`RV_XLENW{operand[63:16] ==  {{47{1'b0}},1'b1} }} & (`RV_XLENW)'('d47)) |
                                             ({`RV_XLENW{operand[63:15] ==  {{48{1'b0}},1'b1} }} & (`RV_XLENW)'('d48)) |
                                             ({`RV_XLENW{operand[63:14] ==  {{49{1'b0}},1'b1} }} & (`RV_XLENW)'('d49)) |
                                             ({`RV_XLENW{operand[63:13] ==  {{50{1'b0}},1'b1} }} & (`RV_XLENW)'('d50)) |
                                             ({`RV_XLENW{operand[63:12] ==  {{51{1'b0}},1'b1} }} & (`RV_XLENW)'('d51)) |
                                             ({`RV_XLENW{operand[63:11] ==  {{52{1'b0}},1'b1} }} & (`RV_XLENW)'('d52)) |
                                             ({`RV_XLENW{operand[63:10] ==  {{53{1'b0}},1'b1} }} & (`RV_XLENW)'('d53)) |
                                             ({`RV_XLENW{operand[63:09] ==  {{54{1'b0}},1'b1} }} & (`RV_XLENW)'('d54)) |
                                             ({`RV_XLENW{operand[63:08] ==  {{55{1'b0}},1'b1} }} & (`RV_XLENW)'('d55)) |
                                             ({`RV_XLENW{operand[63:07] ==  {{56{1'b0}},1'b1} }} & (`RV_XLENW)'('d56)) |
                                             ({`RV_XLENW{operand[63:06] ==  {{57{1'b0}},1'b1} }} & (`RV_XLENW)'('d57)) |
                                             ({`RV_XLENW{operand[63:05] ==  {{58{1'b0}},1'b1} }} & (`RV_XLENW)'('d58)) |
                                             ({`RV_XLENW{operand[63:04] ==  {{59{1'b0}},1'b1} }} & (`RV_XLENW)'('d59)) |
                                             ({`RV_XLENW{operand[63:03] ==  {{60{1'b0}},1'b1} }} & (`RV_XLENW)'('d60)) |
                                             ({`RV_XLENW{operand[63:02] ==  {{61{1'b0}},1'b1} }} & (`RV_XLENW)'('d61)) |
                                             ({`RV_XLENW{operand[63:01] ==  {{62{1'b0}},1'b1} }} & (`RV_XLENW)'('d62)) |
                                             ({`RV_XLENW{operand[63:00] ==  {{63{1'b0}},1'b1} }} & (`RV_XLENW)'('d63)) |
                                             ({`RV_XLENW{operand[63:00] ==  {{64{1'b0}}     } }} & (`RV_XLENW)'('d00));    // Don't care case as it will be handled as special case

         assign cls_ones[`RV_XLENW-1:0]   =  ({`RV_XLENW{operand[63]    ==  {           1'b0} }} & (`RV_XLENW)'('d00)) |
                                             ({`RV_XLENW{operand[63:62] ==  {{ 1{1'b1}},1'b0} }} & (`RV_XLENW)'('d01)) |
                                             ({`RV_XLENW{operand[63:61] ==  {{ 2{1'b1}},1'b0} }} & (`RV_XLENW)'('d02)) |
                                             ({`RV_XLENW{operand[63:60] ==  {{ 3{1'b1}},1'b0} }} & (`RV_XLENW)'('d03)) |
                                             ({`RV_XLENW{operand[63:59] ==  {{ 4{1'b1}},1'b0} }} & (`RV_XLENW)'('d04)) |
                                             ({`RV_XLENW{operand[63:58] ==  {{ 5{1'b1}},1'b0} }} & (`RV_XLENW)'('d05)) |
                                             ({`RV_XLENW{operand[63:57] ==  {{ 6{1'b1}},1'b0} }} & (`RV_XLENW)'('d06)) |
                                             ({`RV_XLENW{operand[63:56] ==  {{ 7{1'b1}},1'b0} }} & (`RV_XLENW)'('d07)) |
                                             ({`RV_XLENW{operand[63:55] ==  {{ 8{1'b1}},1'b0} }} & (`RV_XLENW)'('d08)) |
                                             ({`RV_XLENW{operand[63:54] ==  {{ 9{1'b1}},1'b0} }} & (`RV_XLENW)'('d09)) |
                                             ({`RV_XLENW{operand[63:53] ==  {{10{1'b1}},1'b0} }} & (`RV_XLENW)'('d10)) |
                                             ({`RV_XLENW{operand[63:52] ==  {{11{1'b1}},1'b0} }} & (`RV_XLENW)'('d11)) |
                                             ({`RV_XLENW{operand[63:51] ==  {{12{1'b1}},1'b0} }} & (`RV_XLENW)'('d12)) |
                                             ({`RV_XLENW{operand[63:50] ==  {{13{1'b1}},1'b0} }} & (`RV_XLENW)'('d13)) |
                                             ({`RV_XLENW{operand[63:49] ==  {{14{1'b1}},1'b0} }} & (`RV_XLENW)'('d14)) |
                                             ({`RV_XLENW{operand[63:48] ==  {{15{1'b1}},1'b0} }} & (`RV_XLENW)'('d15)) |
                                             ({`RV_XLENW{operand[63:47] ==  {{16{1'b1}},1'b0} }} & (`RV_XLENW)'('d16)) |
                                             ({`RV_XLENW{operand[63:46] ==  {{17{1'b1}},1'b0} }} & (`RV_XLENW)'('d17)) |
                                             ({`RV_XLENW{operand[63:45] ==  {{18{1'b1}},1'b0} }} & (`RV_XLENW)'('d18)) |
                                             ({`RV_XLENW{operand[63:44] ==  {{19{1'b1}},1'b0} }} & (`RV_XLENW)'('d19)) |
                                             ({`RV_XLENW{operand[63:43] ==  {{20{1'b1}},1'b0} }} & (`RV_XLENW)'('d20)) |
                                             ({`RV_XLENW{operand[63:42] ==  {{21{1'b1}},1'b0} }} & (`RV_XLENW)'('d21)) |
                                             ({`RV_XLENW{operand[63:41] ==  {{22{1'b1}},1'b0} }} & (`RV_XLENW)'('d22)) |
                                             ({`RV_XLENW{operand[63:40] ==  {{23{1'b1}},1'b0} }} & (`RV_XLENW)'('d23)) |
                                             ({`RV_XLENW{operand[63:39] ==  {{24{1'b1}},1'b0} }} & (`RV_XLENW)'('d24)) |
                                             ({`RV_XLENW{operand[63:38] ==  {{25{1'b1}},1'b0} }} & (`RV_XLENW)'('d25)) |
                                             ({`RV_XLENW{operand[63:37] ==  {{26{1'b1}},1'b0} }} & (`RV_XLENW)'('d26)) |
                                             ({`RV_XLENW{operand[63:36] ==  {{27{1'b1}},1'b0} }} & (`RV_XLENW)'('d27)) |
                                             ({`RV_XLENW{operand[63:35] ==  {{28{1'b1}},1'b0} }} & (`RV_XLENW)'('d28)) |
                                             ({`RV_XLENW{operand[63:34] ==  {{29{1'b1}},1'b0} }} & (`RV_XLENW)'('d29)) |
                                             ({`RV_XLENW{operand[63:33] ==  {{30{1'b1}},1'b0} }} & (`RV_XLENW)'('d30)) |
                                             ({`RV_XLENW{operand[63:32] ==  {{31{1'b1}},1'b0} }} & (`RV_XLENW)'('d31)) |
                                             ({`RV_XLENW{operand[63:31] ==  {{32{1'b1}},1'b0} }} & (`RV_XLENW)'('d32)) |
                                             ({`RV_XLENW{operand[63:30] ==  {{33{1'b1}},1'b0} }} & (`RV_XLENW)'('d33)) |
                                             ({`RV_XLENW{operand[63:29] ==  {{34{1'b1}},1'b0} }} & (`RV_XLENW)'('d34)) |
                                             ({`RV_XLENW{operand[63:28] ==  {{35{1'b1}},1'b0} }} & (`RV_XLENW)'('d35)) |
                                             ({`RV_XLENW{operand[63:27] ==  {{36{1'b1}},1'b0} }} & (`RV_XLENW)'('d36)) |
                                             ({`RV_XLENW{operand[63:26] ==  {{37{1'b1}},1'b0} }} & (`RV_XLENW)'('d37)) |
                                             ({`RV_XLENW{operand[63:25] ==  {{38{1'b1}},1'b0} }} & (`RV_XLENW)'('d38)) |
                                             ({`RV_XLENW{operand[63:24] ==  {{39{1'b1}},1'b0} }} & (`RV_XLENW)'('d39)) |
                                             ({`RV_XLENW{operand[63:23] ==  {{40{1'b1}},1'b0} }} & (`RV_XLENW)'('d40)) |
                                             ({`RV_XLENW{operand[63:22] ==  {{41{1'b1}},1'b0} }} & (`RV_XLENW)'('d41)) |
                                             ({`RV_XLENW{operand[63:21] ==  {{42{1'b1}},1'b0} }} & (`RV_XLENW)'('d42)) |
                                             ({`RV_XLENW{operand[63:20] ==  {{43{1'b1}},1'b0} }} & (`RV_XLENW)'('d43)) |
                                             ({`RV_XLENW{operand[63:19] ==  {{44{1'b1}},1'b0} }} & (`RV_XLENW)'('d44)) |
                                             ({`RV_XLENW{operand[63:18] ==  {{45{1'b1}},1'b0} }} & (`RV_XLENW)'('d45)) |
                                             ({`RV_XLENW{operand[63:17] ==  {{46{1'b1}},1'b0} }} & (`RV_XLENW)'('d46)) |
                                             ({`RV_XLENW{operand[63:16] ==  {{47{1'b1}},1'b0} }} & (`RV_XLENW)'('d47)) |
                                             ({`RV_XLENW{operand[63:15] ==  {{48{1'b1}},1'b0} }} & (`RV_XLENW)'('d48)) |
                                             ({`RV_XLENW{operand[63:14] ==  {{49{1'b1}},1'b0} }} & (`RV_XLENW)'('d49)) |
                                             ({`RV_XLENW{operand[63:13] ==  {{50{1'b1}},1'b0} }} & (`RV_XLENW)'('d50)) |
                                             ({`RV_XLENW{operand[63:12] ==  {{51{1'b1}},1'b0} }} & (`RV_XLENW)'('d51)) |
                                             ({`RV_XLENW{operand[63:11] ==  {{52{1'b1}},1'b0} }} & (`RV_XLENW)'('d52)) |
                                             ({`RV_XLENW{operand[63:10] ==  {{53{1'b1}},1'b0} }} & (`RV_XLENW)'('d53)) |
                                             ({`RV_XLENW{operand[63:09] ==  {{54{1'b1}},1'b0} }} & (`RV_XLENW)'('d54)) |
                                             ({`RV_XLENW{operand[63:08] ==  {{55{1'b1}},1'b0} }} & (`RV_XLENW)'('d55)) |
                                             ({`RV_XLENW{operand[63:07] ==  {{56{1'b1}},1'b0} }} & (`RV_XLENW)'('d56)) |
                                             ({`RV_XLENW{operand[63:06] ==  {{57{1'b1}},1'b0} }} & (`RV_XLENW)'('d57)) |
                                             ({`RV_XLENW{operand[63:05] ==  {{58{1'b1}},1'b0} }} & (`RV_XLENW)'('d58)) |
                                             ({`RV_XLENW{operand[63:04] ==  {{59{1'b1}},1'b0} }} & (`RV_XLENW)'('d59)) |
                                             ({`RV_XLENW{operand[63:03] ==  {{60{1'b1}},1'b0} }} & (`RV_XLENW)'('d60)) |
                                             ({`RV_XLENW{operand[63:02] ==  {{61{1'b1}},1'b0} }} & (`RV_XLENW)'('d61)) |
                                             ({`RV_XLENW{operand[63:01] ==  {{62{1'b1}},1'b0} }} & (`RV_XLENW)'('d62)) |
                                             ({`RV_XLENW{operand[63:00] ==  {{63{1'b1}},1'b0} }} & (`RV_XLENW)'('d63)) |
                                             ({`RV_XLENW{operand[63:00] ==  {{64{1'b1}}     } }} & (`RV_XLENW)'('d00));    // Don't care case as it will be handled as special case
      end
   endgenerate

   assign cls[`RV_XLENW-1:0]           =  operand[`RV_XLEN]  ?  cls_ones[`RV_XLENW-1:0]  :  cls_zeros[`RV_XLENW-1:0];

endmodule // eh2_exu_div_cls
