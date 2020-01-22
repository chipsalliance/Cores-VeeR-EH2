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

module eh2_dec_gpr_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
 )  (
    input logic       tid,

    input logic [4:0] raddr0,  // logical read addresses
    input logic [4:0] raddr1,
    input logic [4:0] raddr2,
    input logic [4:0] raddr3,

    input logic       rtid0,   // read tids
    input logic       rtid1,
    input logic       rtid2,
    input logic       rtid3,

    input logic       rden0,   // read enables
    input logic       rden1,
    input logic       rden2,
    input logic       rden3,

    input logic [4:0] waddr0,  // logical write addresses
    input logic [4:0] waddr1,
    input logic [4:0] waddr2,
    input logic [4:0] waddr3,

    input logic wtid0,          // write tids
    input logic wtid1,
    input logic wtid2,
    input logic wtid3,

    input logic wen0,          // write enables
    input logic wen1,
    input logic wen2,
    input logic wen3,

    input logic [31:0] wd0,    // write data
    input logic [31:0] wd1,
    input logic [31:0] wd2,
    input logic [31:0] wd3,

    input logic       clk,
    input logic       rst_l,

    output logic [31:0] rd0,  // read data
    output logic [31:0] rd1,
    output logic [31:0] rd2,
    output logic [31:0] rd3,

    input  logic        scan_mode
);

   logic [31:1] [31:0] gpr_out;     // 31 x 32 bit GPRs
   logic [31:1] [31:0] gpr_in;
   logic [31:1] w0v,w1v,w2v,w3v;
   logic [31:1] gpr_wr_en;

   // GPR Write Enables for power savings
   assign gpr_wr_en[31:1] = (w0v[31:1] | w1v[31:1] | w2v[31:1] | w3v[31:1]);
   for ( genvar j=1; j<32; j++ )  begin : gpr
      rvdffe #(32) gprff (.*, .en(gpr_wr_en[j]), .din(gpr_in[j][31:0]), .dout(gpr_out[j][31:0]));
   end : gpr

// the read out
   always_comb begin
      rd0[31:0] = 32'b0;
      rd1[31:0] = 32'b0;
      rd2[31:0] = 32'b0;
      rd3[31:0] = 32'b0;
      w0v[31:1] = 31'b0;
      w1v[31:1] = 31'b0;
      w2v[31:1] = 31'b0;
      w3v[31:1] = 31'b0;
      gpr_in[31:1] = '0;

      // GPR Read logic
      for (int j=1; j<32; j++ )  begin
         rd0[31:0] |= ({32{rden0 & (rtid0 == tid) & (raddr0[4:0]== 5'(j))}} & gpr_out[j][31:0]);
         rd1[31:0] |= ({32{rden1 & (rtid1 == tid) & (raddr1[4:0]== 5'(j))}} & gpr_out[j][31:0]);
         rd2[31:0] |= ({32{rden2 & (rtid2 == tid) & (raddr2[4:0]== 5'(j))}} & gpr_out[j][31:0]);
         rd3[31:0] |= ({32{rden3 & (rtid3 == tid) & (raddr3[4:0]== 5'(j))}} & gpr_out[j][31:0]);
     end

     // GPR Write logic
     for (int j=1; j<32; j++ )  begin
         w0v[j]     = wen0  & (wtid0 == tid) & (waddr0[4:0]== 5'(j) );
         w1v[j]     = wen1  & (wtid1 == tid) & (waddr1[4:0]== 5'(j) );
         w2v[j]     = wen2  & (wtid2 == tid) & (waddr2[4:0]== 5'(j) );
         w3v[j]     = wen3  & (wtid3 == tid) & (waddr3[4:0]== 5'(j) );
         gpr_in[j]  = ({32{w0v[j]}} & wd0[31:0]) |
                      ({32{w1v[j]}} & wd1[31:0]) |
                      ({32{w2v[j]}} & wd2[31:0]) |
                      ({32{w3v[j]}} & wd3[31:0]);
     end
   end // always_comb begin

`ifdef ASSERT_ON

   logic write_collision_unused;

   assign write_collision_unused = ( (w0v[31:1] == w1v[31:1]) & wen0 & wen1 & (wtid0==tid) & (wtid1==tid) ) |
                                   ( (w0v[31:1] == w2v[31:1]) & wen0 & wen2 & (wtid0==tid) & (wtid2==tid) ) |
                                   ( (w0v[31:1] == w3v[31:1]) & wen0 & wen3 & (wtid0==tid) & (wtid3==tid) ) |
                                   ( (w1v[31:1] == w2v[31:1]) & wen1 & wen2 & (wtid1==tid) & (wtid2==tid) ) |
                                   ( (w1v[31:1] == w3v[31:1]) & wen1 & wen3 & (wtid1==tid) & (wtid3==tid) ) |
                                   ( (w2v[31:1] == w3v[31:1]) & wen2 & wen3 & (wtid2==tid) & (wtid3==tid ));

   // asserting that no 2 ports will write to the same gpr simultaneously
   assert_multiple_wen_to_same_gpr: assert #0 (~( write_collision_unused ) );

`endif

endmodule
