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

// all flops call the rvdff flop

//`ifdef CLOCKGATE
//`define CLOCKGATE_or_PHYSICAL
//`elsif PHYSICAL
//`define CLOCKGATE_or_PHYSICAL
//`endif

module rvdff #( parameter WIDTH=1 )
   (
     input logic [WIDTH-1:0] din,
     input logic           clk,
     input logic                   rst_l,

     output logic [WIDTH-1:0] dout
     );

`ifdef CLOCKGATE
   always @(posedge tb_top.clk) begin
      #0 $strobe("CG: %0t %m din %x dout %x clk %b width %d",$time,din,dout,clk,WIDTH);
   end
`endif

   always_ff @(posedge clk or negedge rst_l) begin
      if (rst_l == 0)
        dout[WIDTH-1:0] <= 0;
      else
        dout[WIDTH-1:0] <= din[WIDTH-1:0];
   end


endmodule

// rvdff with 2:1 input mux to flop din iff sel==1
module rvdffs #( parameter WIDTH=1 )
   (
     input logic [WIDTH-1:0] din,
     input logic             en,
     input logic           clk,
     input logic                   rst_l,
     output logic [WIDTH-1:0] dout
     );

   rvdff #(WIDTH) dffs (.din((en) ? din[WIDTH-1:0] : dout[WIDTH-1:0]), .*);

endmodule

// rvdff with en and clear
module rvdffsc #( parameter WIDTH=1 )
   (
     input logic [WIDTH-1:0] din,
     input logic             en,
     input logic             clear,
     input logic           clk,
     input logic                   rst_l,
     output logic [WIDTH-1:0] dout
     );

   logic [WIDTH-1:0]          din_new;
   assign din_new = {WIDTH{~clear}} & (en ? din[WIDTH-1:0] : dout[WIDTH-1:0]);
   rvdff #(WIDTH) dffsc (.din(din_new[WIDTH-1:0]), .*);

endmodule


module rvdffe #( parameter WIDTH=1 )
   (
     input  logic [WIDTH-1:0] din,
     input  logic           en,
     input  logic           clk,
     input  logic           rst_l,
     input  logic             scan_mode,
     output logic [WIDTH-1:0] dout
     );

   logic                      l1clk;

`ifndef PHYSICAL
   if (WIDTH >= 8) begin: genblock
`endif

`ifdef RV_FPGA_OPTIMIZE
      rvdffs #(WIDTH) dff ( .* );
`else
      rvclkhdr clkhdr ( .* );
      rvdff #(WIDTH) dff (.*, .clk(l1clk));
`endif

`ifndef PHYSICAL
   end
   else
      $error("%m: rvdffe width must be >= 8");
`endif

endmodule // rvdffe

module rvsyncss #(parameter WIDTH = 251)
   (
     input  logic                 clk,
     input  logic                 rst_l,
     input  logic [WIDTH-1:0]     din,
     output logic [WIDTH-1:0]     dout
     );

   logic [WIDTH-1:0]              din_ff1;

   rvdff #(WIDTH) sync_ff1  (.*, .din (din[WIDTH-1:0]),     .dout(din_ff1[WIDTH-1:0]));
   rvdff #(WIDTH) sync_ff2  (.*, .din (din_ff1[WIDTH-1:0]), .dout(dout[WIDTH-1:0]));

endmodule // rvsyncss

module rvarbiter2
  (
   input  logic       [1:0] ready,
   input  logic             shift,
   input  logic             clk,
   input  logic             rst_l,
   input  logic             scan_mode,
   output logic             tid
   );

   logic                    ready0, ready1, ready2;
   logic                    favor_in, favor;

   assign ready0 = ~(|ready[1:0]);

   assign ready1 = ready[1] ^ ready[0];

   assign ready2 = ready[1] & ready[0];

   assign favor_in = (ready2 & ~favor) |
                     (ready1 & ready[0]) |
                     (ready0 & favor);

   // only update if 2 ready threads
   rvdffs #(1) favor_ff (.*, .en(shift & ready2), .clk(clk), .din(favor_in),  .dout(favor) );

   // when to select tid 1
   assign tid = (ready2 & favor) |
                (ready[1] & ~ready[0]);

endmodule


// .i 3
// .o 5
// .ilb ready[0] ready[1] favor
// .ob i0_sel_i0_t1 i1_sel_i1[1] i1_sel_i0[1] i1_sel_i1[0] i1_sel_i0[0]
//
// .type fr
//
// 00 - - ----
//
// 01 - 1 1000
//
// 10 - 0 0010
//
// 11 0 0 0100
//
// 11 1 1 0001

// .i 3
// .o 1
// .ilb favor_in i0_only_in[0] i0_only_in[1]
// .ob favor_final
//
// .type fd
//
// 0 0 0  0
// 0 0 1  1
// 0 1 0  0
// 0 1 1  0
// 1 0 0  1
// 1 0 1  1
// 1 1 0  0
// 1 1 1  1

module rvarbiter2_smt
  (
   input  logic       [1:0] flush,
   input  logic       [1:0] ready_in,
   input  logic       [1:0] lsu_in,
   input  logic       [1:0] mul_in,
   input  logic       [1:0] i0_only_in,
   input  logic       [1:0] thread_stall_in,
   input  logic             shift,
   input  logic             clk,
   input  logic             rst_l,
   input  logic             scan_mode,
   output logic [1:0]       ready,
   output logic             i0_sel_i0_t1,
   output logic [1:0]       i1_sel_i1,
   output logic [1:0]       i1_sel_i0
   );

   logic [1:0]              fready;
   logic [1:0]              thread_cancel_in;
   logic                    ready0, ready1, ready2;
   logic                    favor_in, favor, favor_new, favor_final;
   logic                    ready2_in;
   logic                    lsu2_in;
   logic                    mul2_in;
   logic [1:0]              eff_ready_in;
   logic [1:0]              flush_ff;


   rvdff #(2) flushff (.*,
                        .clk(clk),
                        .din(flush[1:0]),
                        .dout(flush_ff[1:0])
                        );

   // if thread is flushed take it out of arbitration right away
   // if thread is stalled AND both threads ready make ready=0 for stall thread
   assign eff_ready_in[1:0] = ready_in[1:0] & ~({2{ready_in[1]&ready_in[0]}} & thread_stall_in[1:0]) & ~flush[1:0] & ~flush_ff[1:0];


   rvdff #(2) ready_ff (.*,
                        .clk(clk),
                        .din(eff_ready_in[1:0]),
                        .dout(ready[1:0])
                        );

   assign ready0 = ~(|ready[1:0]);

   assign ready1 = ready[1] ^ ready[0];

   assign ready2 = ready[1] & ready[0];

   assign favor_new = (ready2 & ~favor) |
                      (ready1 & ready[0]) |
                      (ready0 & favor);

   assign favor_in = (shift & ready2) ? favor_new : favor;

   // i0_only optimization : make i0_only favored if at all possible
   assign favor_final = (favor_in       & !i0_only_in[0]) |
                        (!i0_only_in[0] &  i0_only_in[1]) |
                        (favor_in       &  i0_only_in[1]);

   // only update if 2 ready threads
   rvdff #(1) favor_ff (.*, .clk(clk), .din(favor_final),  .dout(favor) );

   // SMT optimization
   assign ready2_in = eff_ready_in[1] & eff_ready_in[0];
   assign lsu2_in   = lsu_in[1] & lsu_in[0];
   assign mul2_in   = mul_in[1] & mul_in[0];

   // cancel non favored thread in the case of 2 muls or 2 load/stores
   // this case won't happen if i0_only for one or more threads
   assign thread_cancel_in[1:0] = { (lsu2_in | mul2_in) & ready2_in & ~favor_in,
                                    (lsu2_in | mul2_in) & ready2_in &  favor_in  };
   rvdff #(2) fready_ff (.*,
                        .clk(clk),
                        .din(eff_ready_in[1:0] & ~thread_cancel_in[1:0]),
                        .dout(fready[1:0])
                        );

   assign i0_sel_i0_t1 = (fready[1]&favor) | (!fready[0]);

   assign i1_sel_i1[1] = (!fready[0]);

   assign i1_sel_i0[1] = (fready[0]&fready[1]&!favor);

   assign i1_sel_i1[0] = (!fready[1]);

   assign i1_sel_i0[0] = (fready[0]&fready[1]&favor);



endmodule


module rvlsadder
  (
    input logic [31:0] rs1,
    input logic [11:0] offset,

    output logic [31:0] dout
    );

   logic                cout;
   logic                sign;

   logic [31:12]        rs1_inc;
   logic [31:12]        rs1_dec;

   assign {cout,dout[11:0]} = {1'b0,rs1[11:0]} + {1'b0,offset[11:0]};

   assign rs1_inc[31:12] = rs1[31:12] + 1;

   assign rs1_dec[31:12] = rs1[31:12] - 1;

   assign sign = offset[11];

   assign dout[31:12] = ({20{  sign ^  ~cout}}  &     rs1[31:12]) |
                        ({20{ ~sign &   cout}}  & rs1_inc[31:12]) |
                        ({20{  sign &  ~cout}}  & rs1_dec[31:12]);

endmodule // rvlsadder

// assume we only maintain pc[31:1] in the pipe

module rvbradder
  (
    input [31:1] pc,
    input [12:1] offset,

    output [31:1] dout
    );

   logic          cout;
   logic          sign;

   logic [31:13]  pc_inc;
   logic [31:13]  pc_dec;

   assign {cout,dout[12:1]} = {1'b0,pc[12:1]} + {1'b0,offset[12:1]};

   assign pc_inc[31:13] = pc[31:13] + 1;

   assign pc_dec[31:13] = pc[31:13] - 1;

   assign sign = offset[12];


   assign dout[31:13] = ({19{  sign ^  ~cout}} &      pc[31:13]) |
                        ({19{ ~sign &   cout}}  & pc_inc[31:13]) |
                        ({19{  sign &  ~cout}}  & pc_dec[31:13]);


endmodule // rvbradder


// 2s complement circuit
module rvtwoscomp #( parameter WIDTH=32 )
   (
     input logic [WIDTH-1:0] din,

     output logic [WIDTH-1:0] dout
     );

   logic [WIDTH-1:1]          dout_temp;   // holding for all other bits except for the lsb. LSB is always din

   genvar                     i;

   for ( i = 1; i < WIDTH; i++ )  begin : flip_after_first_one
      assign dout_temp[i] = (|din[i-1:0]) ? ~din[i] : din[i];
   end : flip_after_first_one

   assign dout[WIDTH-1:0]  = { dout_temp[WIDTH-1:1], din[0] };

endmodule  // 2'scomp

// find first
module rvfindfirst1 #( parameter WIDTH=32, SHIFT=$clog2(WIDTH) )
   (
     input logic [WIDTH-1:0] din,

     output logic [SHIFT-1:0] dout
     );
   logic                      done;

   always_comb begin
      dout[SHIFT-1:0] = {SHIFT{1'b0}};
      done    = 1'b0;

      for ( int i = WIDTH-1; i > 0; i-- )  begin : find_first_one
         done |= din[i];
         dout[SHIFT-1:0] += done ? 1'b0 : 1'b1;
      end : find_first_one
   end
endmodule // rvfindfirst1

module rvfindfirst1hot #( parameter WIDTH=32 )
   (
     input logic [WIDTH-1:0] din,

     output logic [WIDTH-1:0] dout
     );
   logic                      done;

   always_comb begin
      dout[WIDTH-1:0] = {WIDTH{1'b0}};
      done    = 1'b0;
      for ( int i = 0; i < WIDTH; i++ )  begin : find_first_one
         dout[i] = ~done & din[i];
         done   |= din[i];
      end : find_first_one
   end
endmodule // rvfindfirst1hot

// mask and match function matches bits after finding the first 0 position
// find first starting from LSB. Skip that location and match the rest of the bits
module rvmaskandmatch #( parameter WIDTH=32 )
   (
     input  logic [WIDTH-1:0] mask,     // this will have the mask in the lower bit positions
     input  logic [WIDTH-1:0] data,     // this is what needs to be matched on the upper bits with the mask's upper bits
     input  logic             masken,   // when 1 : do mask. 0 : full match
     output logic             match
     );

   logic [WIDTH-1:0]          matchvec;
   logic                      masken_or_fullmask;

   assign masken_or_fullmask = masken &  ~(&mask[WIDTH-1:0]);

   assign matchvec[0]        = masken_or_fullmask | (mask[0] == data[0]);
   genvar                     i;

   for ( i = 1; i < WIDTH; i++ )  begin : match_after_first_zero
      assign matchvec[i] = (&mask[i-1:0] & masken_or_fullmask) ? 1'b1 : (mask[i] == data[i]);
   end : match_after_first_zero

   assign match  = &matchvec[WIDTH-1:0];    // all bits either matched or were masked off

endmodule // rvmaskandmatch


// Check if the S_ADDR <= addr < E_ADDR
module rvrangecheck  #(CCM_SADR = 32'h0,
                       CCM_SIZE  = 128) (
   input  logic [31:0]   addr,                             // Address to be checked for range
   output logic          in_range,                            // S_ADDR <= start_addr < E_ADDR
   output logic          in_region
);

   localparam REGION_BITS = 4;
   localparam MASK_BITS = 10 + $clog2(CCM_SIZE);

   logic [31:0]          start_addr;
   logic [3:0]           region;

   assign start_addr[31:0]        = CCM_SADR;
   assign region[REGION_BITS-1:0] = start_addr[31:(32-REGION_BITS)];

   assign in_region = (addr[31:(32-REGION_BITS)] == region[REGION_BITS-1:0]);
   if (CCM_SIZE  == 48)
    assign in_range  = (addr[31:MASK_BITS] == start_addr[31:MASK_BITS]) & ~(&addr[MASK_BITS-1 : MASK_BITS-2]);
   else
    assign in_range  = (addr[31:MASK_BITS] == start_addr[31:MASK_BITS]);

endmodule  // rvrangechecker

// 16 bit even parity generator
module rveven_paritygen #(WIDTH = 16)  (
                                         input  logic [WIDTH-1:0]  data_in,         // Data
                                         output logic              parity_out       // generated even parity
                                         );

   assign  parity_out =  ^(data_in[WIDTH-1:0]) ;

endmodule  // rveven_paritygen

module rveven_paritycheck #(WIDTH = 16)  (
                                           input  logic [WIDTH-1:0]  data_in,         // Data
                                           input  logic              parity_in,
                                           output logic              parity_err       // Parity error
                                           );

   assign  parity_err =  ^(data_in[WIDTH-1:0]) ^ parity_in ;

endmodule  // rveven_paritycheck

module rvecc_encode  (
                      input [31:0] din,
                      output [6:0] ecc_out
                      );
logic [5:0] ecc_out_temp;

   assign ecc_out_temp[0] = din[0]^din[1]^din[3]^din[4]^din[6]^din[8]^din[10]^din[11]^din[13]^din[15]^din[17]^din[19]^din[21]^din[23]^din[25]^din[26]^din[28]^din[30];
   assign ecc_out_temp[1] = din[0]^din[2]^din[3]^din[5]^din[6]^din[9]^din[10]^din[12]^din[13]^din[16]^din[17]^din[20]^din[21]^din[24]^din[25]^din[27]^din[28]^din[31];
   assign ecc_out_temp[2] = din[1]^din[2]^din[3]^din[7]^din[8]^din[9]^din[10]^din[14]^din[15]^din[16]^din[17]^din[22]^din[23]^din[24]^din[25]^din[29]^din[30]^din[31];
   assign ecc_out_temp[3] = din[4]^din[5]^din[6]^din[7]^din[8]^din[9]^din[10]^din[18]^din[19]^din[20]^din[21]^din[22]^din[23]^din[24]^din[25];
   assign ecc_out_temp[4] = din[11]^din[12]^din[13]^din[14]^din[15]^din[16]^din[17]^din[18]^din[19]^din[20]^din[21]^din[22]^din[23]^din[24]^din[25];
   assign ecc_out_temp[5] = din[26]^din[27]^din[28]^din[29]^din[30]^din[31];

   assign ecc_out[6:0] = {(^din[31:0])^(^ecc_out_temp[5:0]),ecc_out_temp[5:0]};

endmodule // rvecc_encode

module rvecc_decode  (
                      input         en,
                      input [31:0]  din,
                      input [6:0]   ecc_in,
                      input         sed_ded,
                      output [31:0] dout,
                      output [6:0]  ecc_out,
                      output        single_ecc_error,
                      output        double_ecc_error

                      );

   logic [6:0]                      ecc_check;
   logic [38:0]                     error_mask;
   logic [38:0]                     din_plus_parity, dout_plus_parity;

   // Generate the ecc bits
   assign ecc_check[0] = ecc_in[0]^din[0]^din[1]^din[3]^din[4]^din[6]^din[8]^din[10]^din[11]^din[13]^din[15]^din[17]^din[19]^din[21]^din[23]^din[25]^din[26]^din[28]^din[30];
   assign ecc_check[1] = ecc_in[1]^din[0]^din[2]^din[3]^din[5]^din[6]^din[9]^din[10]^din[12]^din[13]^din[16]^din[17]^din[20]^din[21]^din[24]^din[25]^din[27]^din[28]^din[31];
   assign ecc_check[2] = ecc_in[2]^din[1]^din[2]^din[3]^din[7]^din[8]^din[9]^din[10]^din[14]^din[15]^din[16]^din[17]^din[22]^din[23]^din[24]^din[25]^din[29]^din[30]^din[31];
   assign ecc_check[3] = ecc_in[3]^din[4]^din[5]^din[6]^din[7]^din[8]^din[9]^din[10]^din[18]^din[19]^din[20]^din[21]^din[22]^din[23]^din[24]^din[25];
   assign ecc_check[4] = ecc_in[4]^din[11]^din[12]^din[13]^din[14]^din[15]^din[16]^din[17]^din[18]^din[19]^din[20]^din[21]^din[22]^din[23]^din[24]^din[25];
   assign ecc_check[5] = ecc_in[5]^din[26]^din[27]^din[28]^din[29]^din[30]^din[31];

   // This is the parity bit
   assign ecc_check[6] = ((^din[31:0])^(^ecc_in[6:0])) & ~sed_ded;

   assign single_ecc_error = en & (ecc_check[6:0] != 0) & ecc_check[6];   // this will never be on for sed_ded
   assign double_ecc_error = en & (ecc_check[6:0] != 0) & ~ecc_check[6];  // all errors in the sed_ded case will be recorded as DE

   // Generate the mask for error correctiong
   for (genvar i=1; i<40; i++) begin
      assign error_mask[i-1] = (ecc_check[5:0] == i);
   end

   // Generate the corrected data
   assign din_plus_parity[38:0] = {ecc_in[6], din[31:26], ecc_in[5], din[25:11], ecc_in[4], din[10:4], ecc_in[3], din[3:1], ecc_in[2], din[0], ecc_in[1:0]};

   assign dout_plus_parity[38:0] = single_ecc_error ? (error_mask[38:0] ^ din_plus_parity[38:0]) : din_plus_parity[38:0];
   assign dout[31:0]             = {dout_plus_parity[37:32], dout_plus_parity[30:16], dout_plus_parity[14:8], dout_plus_parity[6:4], dout_plus_parity[2]};
   assign ecc_out[6:0]           = {(dout_plus_parity[38] ^ (ecc_check[6:0] == 7'b1000000)), dout_plus_parity[31], dout_plus_parity[15], dout_plus_parity[7], dout_plus_parity[3], dout_plus_parity[1:0]};

endmodule // rvecc_decode

module rvecc_encode_64  (
                      input [63:0] din,
                      output [6:0] ecc_out
                      );
  assign ecc_out[0] = din[0]^din[1]^din[3]^din[4]^din[6]^din[8]^din[10]^din[11]^din[13]^din[15]^din[17]^din[19]^din[21]^din[23]^din[25]^din[26]^din[28]^din[30]^din[32]^din[34]^din[36]^din[38]^din[40]^din[42]^din[44]^din[46]^din[48]^din[50]^din[52]^din[54]^din[56]^din[57]^din[59]^din[61]^din[63];

   assign ecc_out[1] = din[0]^din[2]^din[3]^din[5]^din[6]^din[9]^din[10]^din[12]^din[13]^din[16]^din[17]^din[20]^din[21]^din[24]^din[25]^din[27]^din[28]^din[31]^din[32]^din[35]^din[36]^din[39]^din[40]^din[43]^din[44]^din[47]^din[48]^din[51]^din[52]^din[55]^din[56]^din[58]^din[59]^din[62]^din[63];

   assign ecc_out[2] = din[1]^din[2]^din[3]^din[7]^din[8]^din[9]^din[10]^din[14]^din[15]^din[16]^din[17]^din[22]^din[23]^din[24]^din[25]^din[29]^din[30]^din[31]^din[32]^din[37]^din[38]^din[39]^din[40]^din[45]^din[46]^din[47]^din[48]^din[53]^din[54]^din[55]^din[56]^din[60]^din[61]^din[62]^din[63];

   assign ecc_out[3] = din[4]^din[5]^din[6]^din[7]^din[8]^din[9]^din[10]^din[18]^din[19]^din[20]^din[21]^din[22]^din[23]^din[24]^din[25]^din[33]^din[34]^din[35]^din[36]^din[37]^din[38]^din[39]^din[40]^din[49]^din[50]^din[51]^din[52]^din[53]^din[54]^din[55]^din[56];

   assign ecc_out[4] = din[11]^din[12]^din[13]^din[14]^din[15]^din[16]^din[17]^din[18]^din[19]^din[20]^din[21]^din[22]^din[23]^din[24]^din[25]^din[41]^din[42]^din[43]^din[44]^din[45]^din[46]^din[47]^din[48]^din[49]^din[50]^din[51]^din[52]^din[53]^din[54]^din[55]^din[56];

   assign ecc_out[5] = din[26]^din[27]^din[28]^din[29]^din[30]^din[31]^din[32]^din[33]^din[34]^din[35]^din[36]^din[37]^din[38]^din[39]^din[40]^din[41]^din[42]^din[43]^din[44]^din[45]^din[46]^din[47]^din[48]^din[49]^din[50]^din[51]^din[52]^din[53]^din[54]^din[55]^din[56];

   assign ecc_out[6] = din[57]^din[58]^din[59]^din[60]^din[61]^din[62]^din[63];

endmodule // rvecc_encode_64


module rvecc_decode_64  (
                      input         en,
                      input [63:0]  din,
                      input [6:0]   ecc_in,
                      output        ecc_error
                      );

   logic [6:0]                      ecc_check;

   // Generate the ecc bits
   assign ecc_check[0] = ecc_in[0]^din[0]^din[1]^din[3]^din[4]^din[6]^din[8]^din[10]^din[11]^din[13]^din[15]^din[17]^din[19]^din[21]^din[23]^din[25]^din[26]^din[28]^din[30]^din[32]^din[34]^din[36]^din[38]^din[40]^din[42]^din[44]^din[46]^din[48]^din[50]^din[52]^din[54]^din[56]^din[57]^din[59]^din[61]^din[63];

   assign ecc_check[1] = ecc_in[1]^din[0]^din[2]^din[3]^din[5]^din[6]^din[9]^din[10]^din[12]^din[13]^din[16]^din[17]^din[20]^din[21]^din[24]^din[25]^din[27]^din[28]^din[31]^din[32]^din[35]^din[36]^din[39]^din[40]^din[43]^din[44]^din[47]^din[48]^din[51]^din[52]^din[55]^din[56]^din[58]^din[59]^din[62]^din[63];

   assign ecc_check[2] = ecc_in[2]^din[1]^din[2]^din[3]^din[7]^din[8]^din[9]^din[10]^din[14]^din[15]^din[16]^din[17]^din[22]^din[23]^din[24]^din[25]^din[29]^din[30]^din[31]^din[32]^din[37]^din[38]^din[39]^din[40]^din[45]^din[46]^din[47]^din[48]^din[53]^din[54]^din[55]^din[56]^din[60]^din[61]^din[62]^din[63];

   assign ecc_check[3] = ecc_in[3]^din[4]^din[5]^din[6]^din[7]^din[8]^din[9]^din[10]^din[18]^din[19]^din[20]^din[21]^din[22]^din[23]^din[24]^din[25]^din[33]^din[34]^din[35]^din[36]^din[37]^din[38]^din[39]^din[40]^din[49]^din[50]^din[51]^din[52]^din[53]^din[54]^din[55]^din[56];

   assign ecc_check[4] = ecc_in[4]^din[11]^din[12]^din[13]^din[14]^din[15]^din[16]^din[17]^din[18]^din[19]^din[20]^din[21]^din[22]^din[23]^din[24]^din[25]^din[41]^din[42]^din[43]^din[44]^din[45]^din[46]^din[47]^din[48]^din[49]^din[50]^din[51]^din[52]^din[53]^din[54]^din[55]^din[56];

   assign ecc_check[5] = ecc_in[5]^din[26]^din[27]^din[28]^din[29]^din[30]^din[31]^din[32]^din[33]^din[34]^din[35]^din[36]^din[37]^din[38]^din[39]^din[40]^din[41]^din[42]^din[43]^din[44]^din[45]^din[46]^din[47]^din[48]^din[49]^din[50]^din[51]^din[52]^din[53]^din[54]^din[55]^din[56];

   assign ecc_check[6] = ecc_in[6]^din[57]^din[58]^din[59]^din[60]^din[61]^din[62]^din[63];

   assign ecc_error = en & (ecc_check[6:0] != 0);  // all errors in the sed_ded case will be recorded as DE

 endmodule // rvecc_decode_64


module `TEC_RV_ICG
  (
   input logic SE, EN, CK,
   output Q
   );

   logic  en_ff;
   logic  enable;

   assign      enable = EN | SE;

`ifdef VERILATOR
   always @(negedge CK) begin
      en_ff <= enable;
   end
`else
   always @(CK, enable) begin
      if(!CK)
        en_ff = enable;
   end
`endif
   assign Q = CK & en_ff;

endmodule

module rvclkhdr
  (
   input  logic en,
   input  logic clk,
   input  logic scan_mode,
   output logic l1clk
   );

   logic   SE;
   assign       SE = scan_mode;

   `TEC_RV_ICG clkhdr ( .*, .EN(en), .CK(clk), .Q(l1clk));

endmodule // rvclkhdr

module rvoclkhdr
  (
   input  logic en,
   input  logic clk,
   input  logic scan_mode,
   output logic l1clk
   );

   logic   SE;
   assign       SE = scan_mode;

`ifdef RV_FPGA_OPTIMIZE
   assign l1clk = clk;
`else
   `TEC_RV_ICG clkhdr ( .*, .EN(en), .CK(clk), .Q(l1clk));
`endif

endmodule

