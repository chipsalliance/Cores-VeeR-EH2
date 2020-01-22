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


module eh2_exu_div_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (
   input logic         clk,                       // Top level clock
   input logic         rst_l,                     // Reset
   input logic         scan_mode,                 // Scan mode

   input logic  [31:0] dividend,                  // Numerator
   input logic  [31:0] divisor,                   // Denominator

   input eh2_div_pkt_t  dp,                      // valid, sign, rem

   input logic         cancel,                    // Kill divide for write-after-write case


   output logic        finish_dly,                // finish delayed to match data
   output logic [31:0] out                        // Result
  );


   logic         div_clken;
   logic         exu_div_clk;
   logic         run_in, run_state;
   logic [5:0]   count_in, count;
   logic [32:0]  m_ff;
   logic         qff_enable;
   logic         aff_enable;
   logic [32:0]  q_in, q_ff;
   logic [32:0]  a_in, a_ff;
   logic [32:0]  m_eff;
   logic [32:0]  a_shift;
   logic         dividend_neg_ff, divisor_neg_ff;
   logic [31:0]  dividend_comp;
   logic [31:0]  dividend_eff;
   logic [31:0]  q_ff_comp;
   logic [31:0]  q_ff_eff;
   logic [31:0]  a_ff_comp;
   logic [31:0]  a_ff_eff;
   logic         sign_ff, sign_eff;
   logic         rem_ff;
   logic         add;
   logic [32:0]  a_eff;
   logic [55:0]  a_eff_shift;
   logic         rem_correct;
   logic         valid_in;
   logic         valid_e1;
   logic         finish;
   logic         finish_ff;

   logic         smallnum_case_e1, smallnum_case_e2, smallnum_case_e3, smallnum_case_e4, smallnum_case_wb;
   logic [3:0]   smallnum, smallnum_in, smallnum_ff;
   logic         m_already_comp;



   rvoclkhdr exu_div_cgc          (.*, .en(div_clken),    .l1clk(exu_div_clk));

   rvdff  #(1)  i_finish_ff       (.*, .clk(exu_div_clk), .din(finish),                                     .dout(finish_ff));
   rvdff  #(1)  e1val_ff          (.*, .clk(exu_div_clk), .din(valid_in),                                   .dout(valid_e1));
   rvdff  #(1)  runff             (.*, .clk(exu_div_clk), .din(run_in),                                     .dout(run_state));
   rvdff  #(6)  countff           (.*, .clk(exu_div_clk), .din(count_in[5:0]),                              .dout(count[5:0]));
   rvdffs #(4)  miscf             (.*, .clk(exu_div_clk), .din({dividend[31],divisor[31],sign_eff,dp.rem}), .dout({dividend_neg_ff,divisor_neg_ff,sign_ff,rem_ff}), .en(dp.valid));
   rvdff  #(8)  smallnumff        (.*, .clk(exu_div_clk), .din ({smallnum_case_e1 & ~cancel,
                                                                 smallnum_case_e2 & ~cancel,
                                                                 smallnum_case_e3 & ~cancel,
                                                                 smallnum_case_e4 & ~cancel,
                                                                 smallnum_in[3:0]}),
                                                          .dout({smallnum_case_e2,
                                                                 smallnum_case_e3,
                                                                 smallnum_case_e4,
                                                                 smallnum_case_wb,
                                                                 smallnum_ff[3:0]}));
   rvdffe #(33) mff               (.*, .en(dp.valid),     .din({ ~dp.unsign & divisor[31], divisor[31:0]}), .dout(m_ff[32:0]));
   rvdffe #(33) qff               (.*, .en(qff_enable),   .din(q_in[32:0]),                                 .dout(q_ff[32:0]));
   rvdffe #(33) aff               (.*, .en(aff_enable),   .din(a_in[32:0]),                                 .dout(a_ff[32:0]));

   rvtwoscomp #(32) i_dividend_comp (.din(q_ff[31:0]),    .dout(dividend_comp[31:0]));
   rvtwoscomp #(32) i_q_ff_comp     (.din(q_ff[31:0]),    .dout(q_ff_comp[31:0]));
   rvtwoscomp #(32) i_a_ff_comp     (.din(a_ff[31:0]),    .dout(a_ff_comp[31:0]));


   assign valid_in                = dp.valid    & ~cancel;


   // START - short circuit logic for small numbers {{

   // small number divides - any 4b / 4b is done in 1 cycle (divisor != 0)
   // to generate espresso equations:
   // 1)  smalldiv > smalldiv.e
   // 2)  espresso -Dso -oeqntott smalldiv.e | addassign > smalldiv

   // smallnum case does not cover divide by 0
   assign smallnum_case_e1        = ((q_ff[31:4] == 28'b0) & (m_ff[31:4] == 28'b0) & (m_ff[31:0] != 32'b0) & ~rem_ff & valid_e1) |
                                    ((q_ff[31:0] == 32'b0) &                         (m_ff[31:0] != 32'b0) & ~rem_ff & valid_e1);


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

   logic [3:0]   shortq_shift;
   logic [5:0]   shortq_shift_ff;
   logic         shortq_enable;
   logic         shortq_enable_ff;
   logic [32:0]  short_dividend;

   assign short_dividend[31:0]    =  q_ff[31:0];
   assign short_dividend[32]      =  sign_ff & q_ff[31];


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

   logic [2:0]   a_cls;
   logic [2:0]   b_cls;
   logic [3:0]   shortq_raw;
   logic [3:0]   shortq_shift_xx;

   assign a_cls[2]                =  (~short_dividend[32] & (short_dividend[31:24] != {8{1'b0}})) | ( short_dividend[32] & (short_dividend[31:23] != {9{1'b1}}));
   assign a_cls[1]                =  (~short_dividend[32] & (short_dividend[23:16] != {8{1'b0}})) | ( short_dividend[32] & (short_dividend[22:15] != {8{1'b1}}));
   assign a_cls[0]                =  (~short_dividend[32] & (short_dividend[15:08] != {8{1'b0}})) | ( short_dividend[32] & (short_dividend[14:07] != {8{1'b1}}));

   assign b_cls[2]                =  (~m_ff[32]           & (          m_ff[31:24] != {8{1'b0}})) | ( m_ff[32]           & (          m_ff[31:24] != {8{1'b1}}));
   assign b_cls[1]                =  (~m_ff[32]           & (          m_ff[23:16] != {8{1'b0}})) | ( m_ff[32]           & (          m_ff[23:16] != {8{1'b1}}));
   assign b_cls[0]                =  (~m_ff[32]           & (          m_ff[15:08] != {8{1'b0}})) | ( m_ff[32]           & (          m_ff[15:08] != {8{1'b1}}));

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


   assign shortq_enable           =  valid_e1 & (m_ff[31:0] != 32'b0) & (shortq_raw[3:0] != 4'b0) & ~smallnum_case_e1;

   assign shortq_shift[3:0]       = ({4{shortq_enable}} & shortq_raw[3:0]);

   rvdff  #(5)  i_shortq_ff       (.*, .clk(exu_div_clk), .din({shortq_enable,shortq_shift[3:0]}), .dout({shortq_enable_ff,shortq_shift_xx[3:0]}));

   assign shortq_shift_ff[5:0]    = ({6{shortq_shift_xx[3]}} & 6'b01_1110) |   // 31 -> 30 required for nonblocking div so finish is no faster than E4
                                    ({6{shortq_shift_xx[2]}} & 6'b01_1000) |   // 24
                                    ({6{shortq_shift_xx[1]}} & 6'b01_0000) |   // 16
                                    ({6{shortq_shift_xx[0]}} & 6'b00_1000);    //  8

`ifdef ASSERT_ON

   logic  div_assert_fail;

   assign div_assert_fail         =  (shortq_shift_xx[3] & shortq_shift_xx[2]) |
                                     (shortq_shift_xx[3] & shortq_shift_xx[1]) |
                                     (shortq_shift_xx[3] & shortq_shift_xx[0]) |
                                     (shortq_shift_xx[2] & shortq_shift_xx[1]) |
                                     (shortq_shift_xx[2] & shortq_shift_xx[0]) |
                                     (shortq_shift_xx[1] & shortq_shift_xx[0]);

   assert_exu_div_shortq_shift_error: assert #0 (~div_assert_fail) else $display("ERROR: SHORTQ_SHIFT_XX with multiple shifts ON!");

`endif


// *** End   Short *** }}





   assign div_clken               =  dp.valid | run_state | finish_ff;

   assign run_in                  = ((dp.valid & ~cancel) | (run_state & ~cancel)) & ~finish;

   assign count_in[5:0]           = {6{run_state & ~finish & ~cancel & ~shortq_enable}} & (count[5:0] + shortq_shift_ff[5:0] + 6'd1);


   assign finish                  = (smallnum_case_e4 | ((~rem_ff) ? (count[5:0] == 6'd32) : (count[5:0] == 6'd33))) & ~cancel;

   assign finish_dly              =  finish_ff & ~cancel;

   assign sign_eff                = ~dp.unsign & (divisor[31:0] != 32'b0);


   assign q_in[32:0]              = ({33{~run_state                                 }} &  {1'b0,dividend[31:0]}) |
                                    ({33{ run_state &  (valid_e1 | shortq_enable_ff)}} &  ({dividend_eff[31:0], ~a_in[32]} << shortq_shift_ff[5:0])) |
                                    ({33{ run_state & ~(valid_e1 | shortq_enable_ff)}} &  {q_ff[31:0], ~a_in[32]});

   assign qff_enable              =  dp.valid | (run_state & ~shortq_enable);




   assign dividend_eff[31:0]      = (sign_ff & dividend_neg_ff) ? dividend_comp[31:0] : q_ff[31:0];


   assign m_eff[32:0]             = ( add ) ? m_ff[32:0] : ~m_ff[32:0];

   assign a_eff_shift[55:0]       = {24'b0, dividend_eff[31:0]} << shortq_shift_ff[5:0];

   assign a_eff[32:0]             = ({33{ rem_correct                    }} &  a_ff[32:0]               ) |
                                    ({33{~rem_correct & ~shortq_enable_ff}} & {a_ff[31:0], q_ff[32]}    ) |
                                    ({33{~rem_correct &  shortq_enable_ff}} & {9'b0,a_eff_shift[55:32]} );

   assign a_shift[32:0]           = {33{run_state}} & a_eff[32:0];

   assign a_in[32:0]              = {33{run_state}} & (a_shift[32:0] + m_eff[32:0] + {32'b0,~add});

   assign aff_enable              =  dp.valid | (run_state & ~shortq_enable & (count[5:0]!=6'd33)) | rem_correct;


   assign m_already_comp          = (divisor_neg_ff & sign_ff);

   // if m already complemented, then invert operation add->sub, sub->add
   assign add                     = (a_ff[32] | rem_correct) ^ m_already_comp;

   assign rem_correct             = (count[5:0] == 6'd33) & rem_ff & a_ff[32];



   assign q_ff_eff[31:0]          = (sign_ff & (dividend_neg_ff ^ divisor_neg_ff)) ? q_ff_comp[31:0] : q_ff[31:0];

   assign a_ff_eff[31:0]          = (sign_ff &  dividend_neg_ff) ? a_ff_comp[31:0] : a_ff[31:0];

   assign out[31:0]               = ({32{ smallnum_case_wb          }} & {28'b0, smallnum_ff[3:0]}) |
                                    ({32{                     rem_ff}} &  a_ff_eff[31:0]          ) |
                                    ({32{~smallnum_case_wb & ~rem_ff}} &  q_ff_eff[31:0]          );


endmodule // eh2_exu_div_ctl
