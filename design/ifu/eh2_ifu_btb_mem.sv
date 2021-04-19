//********************************************************************************
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

//********************************************************************************
// Branch Target Buffer (BTB) SRAM
//********************************************************************************
module eh2_ifu_btb_mem
import eh2_pkg::*;
#(
`include "eh2_param.vh"
 )(
   input logic                                        clk,
   input logic                                        active_clk,
   input logic                                        rst_l,
   input logic                                        clk_override,

   input  eh2_ccm_ext_in_pkt_t   [1:0] btb_ext_in_pkt,

   input logic                         btb_wren,
   input logic                         btb_rden,
   input logic [1:0] [pt.BTB_ADDR_HI:1] btb_rw_addr,  // per bank read addr, bank0 has write addr
   input logic [1:0] [pt.BTB_ADDR_HI:1] btb_rw_addr_f1,  // per bank read addr, bank0 has write addr
   input logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]         btb_sram_wr_data,
   input logic [1:0] [pt.BTB_BTAG_SIZE-1:0] btb_sram_rd_tag_f1,


   output eh2_btb_sram_pkt btb_sram_pkt,

   output logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank0_rd_data_f1,
   output logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank1_rd_data_f1,
   output logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank2_rd_data_f1,
   output logic [pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]      btb_vbank3_rd_data_f1,

   input  logic                                       scan_mode

);

   localparam BTB_DWIDTH =  pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5;

`define RV_TAG BTB_DWIDTH-1:BTB_DWIDTH-pt.BTB_BTAG_SIZE
   localparam PC4=4;
   localparam BOFF=3;
   localparam BV=0;

   logic [1:0][1:0] [2*BTB_DWIDTH-1:0] btb_rd_data, btb_rd_data_raw;

   logic [BTB_DWIDTH-1:0]          btb_bank0e_rd_data_f1, btb_bank0e_rd_data_p1_f1;
   logic [BTB_DWIDTH-1:0]          btb_bank1e_rd_data_f1, btb_bank1e_rd_data_p1_f1;
   logic [BTB_DWIDTH-1:0]          btb_bank0o_rd_data_f1, btb_bank0o_rd_data_p1_f1;
   logic [BTB_DWIDTH-1:0]          btb_bank1o_rd_data_f1;
   logic [BTB_DWIDTH-1:0]          btb_bank0_rd_data_way0_f1, btb_bank0_rd_data_way0_p1_f1;
   logic [BTB_DWIDTH-1:0]          btb_bank1_rd_data_way0_f1, btb_bank1_rd_data_way0_p1_f1;
   logic [BTB_DWIDTH-1:0]          btb_bank0_rd_data_way1_f1, btb_bank0_rd_data_way1_p1_f1;
   logic [BTB_DWIDTH-1:0]          btb_bank1_rd_data_way1_f1, btb_bank1_rd_data_way1_p1_f1;

   logic [(4*BTB_DWIDTH)-1:0]      btb_bit_en_vec;
   logic       wr_way0_en, wr_way1_en, btb_rden_f1, btb_wren_f1, btb_sram_wr_datav_f1;

   logic [3:0]  tag_match_way0_expanded_f1, tag_match_way1_expanded_f1,
                tag_match_way0_expanded_p1_f1, tag_match_way1_expanded_p1_f1;
   logic [1:0]  tag_match_way0_f1, tag_match_way0_p1_f1,
                tag_match_way1_f1, tag_match_way1_p1_f1;
   logic [pt.BTB_BTAG_SIZE-1:0] fetch_rd_tag_f1, fetch_rd_tag_p1_f1;
   logic [1:0]                  wren_bank;

   logic [3:0] fetch_start_f1;
   logic [3:0] btb_rd0_valid, btb_rd1_valid;
   logic [pt.BTB_SIZE-1:0] btb_write_entry, btb_valid_ns, btb_valid;

   assign fetch_start_f1[3:0] = (4'b1 << btb_rw_addr_f1[1][2:1]);

   // ----------------------------------------------------------------------
   // READ
   // ----------------------------------------------------------------------

   //
   //
   // Addresses
   // [N:4] index hash of fetch addess [31:4]+1
   // [N:4] index hash
   // [3] bank bit
   // [2:1] fetch start
   //

   // Map sram output to flop naming convention, ugh
   assign btb_bank0_rd_data_way0_f1    = btb_rw_addr_f1[1][3] ? {btb_rd_data[1][0][BTB_DWIDTH-1:1], btb_rd1_valid[0]} :
                                         {btb_rd_data[0][0][BTB_DWIDTH-1:1], btb_rd0_valid[0]};
   assign btb_bank0_rd_data_way1_f1    = btb_rw_addr_f1[1][3] ? {btb_rd_data[1][0][(2*BTB_DWIDTH)-1:BTB_DWIDTH+1], btb_rd1_valid[1]} :
                                         {btb_rd_data[0][0][(2*BTB_DWIDTH)-1:BTB_DWIDTH+1], btb_rd0_valid[1]};
   assign btb_bank1_rd_data_way0_f1    = btb_rw_addr_f1[1][3] ? {btb_rd_data[1][1][BTB_DWIDTH-1:1], btb_rd1_valid[2]} :
                                         {btb_rd_data[0][1][BTB_DWIDTH-1:1], btb_rd0_valid[2]};
   assign btb_bank1_rd_data_way1_f1    = btb_rw_addr_f1[1][3] ? {btb_rd_data[1][1][(2*BTB_DWIDTH)-1:BTB_DWIDTH+1], btb_rd1_valid[3]} :
                                         {btb_rd_data[0][1][(2*BTB_DWIDTH)-1:BTB_DWIDTH+1], btb_rd0_valid[3]};
   assign btb_bank0_rd_data_way0_p1_f1 = btb_rw_addr_f1[1][3] ? {btb_rd_data[0][0][BTB_DWIDTH-1:1], btb_rd0_valid[0]} :
                                         {btb_rd_data[1][0][BTB_DWIDTH-1:1], btb_rd1_valid[0]};
   assign btb_bank0_rd_data_way1_p1_f1 = btb_rw_addr_f1[1][3] ? {btb_rd_data[0][0][(2*BTB_DWIDTH)-1:BTB_DWIDTH+1], btb_rd0_valid[1]} :
                                         {btb_rd_data[1][0][(2*BTB_DWIDTH)-1:BTB_DWIDTH+1], btb_rd1_valid[1]};
   assign btb_bank1_rd_data_way0_p1_f1 = btb_rw_addr_f1[1][3] ? {btb_rd_data[0][1][BTB_DWIDTH-1:1], btb_rd0_valid[2]} :
                                         {btb_rd_data[1][1][BTB_DWIDTH-1:1], btb_rd1_valid[2]};
   assign btb_bank1_rd_data_way1_p1_f1 = btb_rw_addr_f1[1][3] ? {btb_rd_data[0][1][(2*BTB_DWIDTH)-1:BTB_DWIDTH+1], btb_rd0_valid[3]} :
                                         {btb_rd_data[1][1][(2*BTB_DWIDTH)-1:BTB_DWIDTH+1], btb_rd1_valid[3]};

   // 2 -way SA, figure out the way hit and mux accordingly
   assign tag_match_way0_f1[1:0] = {btb_bank1_rd_data_way0_f1[BV] & (btb_bank1_rd_data_way0_f1[`RV_TAG] == btb_sram_rd_tag_f1[0][pt.BTB_BTAG_SIZE-1:0]),
                                    btb_bank0_rd_data_way0_f1[BV] & (btb_bank0_rd_data_way0_f1[`RV_TAG] == btb_sram_rd_tag_f1[0][pt.BTB_BTAG_SIZE-1:0])} &
                                   {2{btb_rden_f1}};

   assign tag_match_way1_f1[1:0] = {btb_bank1_rd_data_way1_f1[BV] & (btb_bank1_rd_data_way1_f1[`RV_TAG] == btb_sram_rd_tag_f1[0][pt.BTB_BTAG_SIZE-1:0]),
                                    btb_bank0_rd_data_way1_f1[BV] & (btb_bank0_rd_data_way1_f1[`RV_TAG] == btb_sram_rd_tag_f1[0][pt.BTB_BTAG_SIZE-1:0])} &
                                   {2{btb_rden_f1}};


   assign tag_match_way0_p1_f1[1:0] = {btb_bank1_rd_data_way0_p1_f1[BV] & (btb_bank1_rd_data_way0_p1_f1[`RV_TAG] == btb_sram_rd_tag_f1[1][pt.BTB_BTAG_SIZE-1:0]),
                                       btb_bank0_rd_data_way0_p1_f1[BV] & (btb_bank0_rd_data_way0_p1_f1[`RV_TAG] == btb_sram_rd_tag_f1[1][pt.BTB_BTAG_SIZE-1:0])} &
                                      {2{btb_rden_f1}};

   assign tag_match_way1_p1_f1[1:0] = {btb_bank1_rd_data_way1_p1_f1[BV] & (btb_bank1_rd_data_way1_p1_f1[`RV_TAG] == btb_sram_rd_tag_f1[1][pt.BTB_BTAG_SIZE-1:0]),
                                       btb_bank0_rd_data_way1_p1_f1[BV] & (btb_bank0_rd_data_way1_p1_f1[`RV_TAG] == btb_sram_rd_tag_f1[1][pt.BTB_BTAG_SIZE-1:0])} &
                                      {2{btb_rden_f1}};


   // Both ways could hit, use the offset bit to reorder

   assign tag_match_way0_expanded_f1[3:0] = {tag_match_way0_f1[1] &  (btb_bank1_rd_data_way0_f1[BOFF] ^ btb_bank1_rd_data_way0_f1[PC4]),
                                             tag_match_way0_f1[1] & ~(btb_bank1_rd_data_way0_f1[BOFF] ^ btb_bank1_rd_data_way0_f1[PC4]),
                                             tag_match_way0_f1[0] &  (btb_bank0_rd_data_way0_f1[BOFF] ^ btb_bank0_rd_data_way0_f1[PC4]),
                                             tag_match_way0_f1[0] & ~(btb_bank0_rd_data_way0_f1[BOFF] ^ btb_bank0_rd_data_way0_f1[PC4])};

   assign tag_match_way1_expanded_f1[3:0] = {tag_match_way1_f1[1] &  (btb_bank1_rd_data_way1_f1[BOFF] ^ btb_bank1_rd_data_way1_f1[PC4]),
                                             tag_match_way1_f1[1] & ~(btb_bank1_rd_data_way1_f1[BOFF] ^ btb_bank1_rd_data_way1_f1[PC4]),
                                             tag_match_way1_f1[0] &  (btb_bank0_rd_data_way1_f1[BOFF] ^ btb_bank0_rd_data_way1_f1[PC4]),
                                             tag_match_way1_f1[0] & ~(btb_bank0_rd_data_way1_f1[BOFF] ^ btb_bank0_rd_data_way1_f1[PC4])};

   assign tag_match_way0_expanded_p1_f1[3:0] = {tag_match_way0_p1_f1[1] &  (btb_bank1_rd_data_way0_p1_f1[BOFF] ^ btb_bank1_rd_data_way0_p1_f1[PC4]),
                                                tag_match_way0_p1_f1[1] & ~(btb_bank1_rd_data_way0_p1_f1[BOFF] ^ btb_bank1_rd_data_way0_p1_f1[PC4]),
                                                tag_match_way0_p1_f1[0] &  (btb_bank0_rd_data_way0_p1_f1[BOFF] ^ btb_bank0_rd_data_way0_p1_f1[PC4]),
                                                tag_match_way0_p1_f1[0] & ~(btb_bank0_rd_data_way0_p1_f1[BOFF] ^ btb_bank0_rd_data_way0_p1_f1[PC4])};

   assign tag_match_way1_expanded_p1_f1[3:0] = {tag_match_way1_p1_f1[1] &  (btb_bank1_rd_data_way1_p1_f1[BOFF] ^ btb_bank1_rd_data_way1_p1_f1[PC4]),
                                                tag_match_way1_p1_f1[1] & ~(btb_bank1_rd_data_way1_p1_f1[BOFF] ^ btb_bank1_rd_data_way1_p1_f1[PC4]),
                                                tag_match_way1_p1_f1[0] &  (btb_bank0_rd_data_way1_p1_f1[BOFF] ^ btb_bank0_rd_data_way1_p1_f1[PC4]),
                                                tag_match_way1_p1_f1[0] & ~(btb_bank0_rd_data_way1_p1_f1[BOFF] ^ btb_bank0_rd_data_way1_p1_f1[PC4])};

   assign btb_sram_pkt.wayhit_f1[3:0] = tag_match_way0_expanded_f1[3:0] | tag_match_way1_expanded_f1[3:0];
   assign btb_sram_pkt.wayhit_p1_f1[3:0] = tag_match_way0_expanded_p1_f1[3:0] | tag_match_way1_expanded_p1_f1[3:0];
   assign btb_sram_pkt.tag_match_way0_f1[1:0] = tag_match_way0_f1[1:0];
   assign btb_sram_pkt.tag_match_way0_p1_f1[1:0] = tag_match_way0_p1_f1[1:0];
   assign btb_sram_pkt.tag_match_vway1_expanded_f1[3:0] = ( ({4{fetch_start_f1[0]}} & {tag_match_way1_expanded_f1[3:0]}) |
                                                            ({4{fetch_start_f1[1]}} & {tag_match_way1_expanded_p1_f1[0], tag_match_way1_expanded_f1[3:1]}) |
                                                            ({4{fetch_start_f1[2]}} & {tag_match_way1_expanded_p1_f1[1:0], tag_match_way1_expanded_f1[3:2]}) |
                                                            ({4{fetch_start_f1[3]}} & {tag_match_way1_expanded_p1_f1[2:0], tag_match_way1_expanded_f1[3]}) );

   assign btb_bank1o_rd_data_f1[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_f1[3]}} & btb_bank1_rd_data_way0_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{tag_match_way1_expanded_f1[3]}} & btb_bank1_rd_data_way1_f1[BTB_DWIDTH-1:0]) );
   assign btb_bank1e_rd_data_f1[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_f1[2]}} & btb_bank1_rd_data_way0_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{tag_match_way1_expanded_f1[2]}} & btb_bank1_rd_data_way1_f1[BTB_DWIDTH-1:0]) );

   assign btb_bank0o_rd_data_f1[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_f1[1]}} & btb_bank0_rd_data_way0_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{tag_match_way1_expanded_f1[1]}} & btb_bank0_rd_data_way1_f1[BTB_DWIDTH-1:0]) );
   assign btb_bank0e_rd_data_f1[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_f1[0]}} & btb_bank0_rd_data_way0_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{tag_match_way1_expanded_f1[0]}} & btb_bank0_rd_data_way1_f1[BTB_DWIDTH-1:0]) );


   assign btb_bank1e_rd_data_p1_f1[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_p1_f1[2]}} & btb_bank1_rd_data_way0_p1_f1[BTB_DWIDTH-1:0]) |
                                                        ({BTB_DWIDTH{tag_match_way1_expanded_p1_f1[2]}} & btb_bank1_rd_data_way1_p1_f1[BTB_DWIDTH-1:0]) );
   assign btb_bank0o_rd_data_p1_f1[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_p1_f1[1]}} & btb_bank0_rd_data_way0_p1_f1[BTB_DWIDTH-1:0]) |
                                                        ({BTB_DWIDTH{tag_match_way1_expanded_p1_f1[1]}} & btb_bank0_rd_data_way1_p1_f1[BTB_DWIDTH-1:0]) );
   assign btb_bank0e_rd_data_p1_f1[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{tag_match_way0_expanded_p1_f1[0]}} & btb_bank0_rd_data_way0_p1_f1[BTB_DWIDTH-1:0]) |
                                                        ({BTB_DWIDTH{tag_match_way1_expanded_p1_f1[0]}} & btb_bank0_rd_data_way1_p1_f1[BTB_DWIDTH-1:0]) );


   // virtual bank order, final 4 branches

   assign btb_vbank0_rd_data_f1[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{fetch_start_f1[0]}} &  btb_bank0e_rd_data_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[1]}} &  btb_bank0o_rd_data_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[2]}} &  btb_bank1e_rd_data_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[3]}} &  btb_bank1o_rd_data_f1[BTB_DWIDTH-1:0]) );
   assign btb_vbank1_rd_data_f1[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{fetch_start_f1[0]}} &  btb_bank0o_rd_data_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[1]}} &  btb_bank1e_rd_data_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[2]}} &  btb_bank1o_rd_data_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[3]}} &  btb_bank0e_rd_data_p1_f1[BTB_DWIDTH-1:0]) );
   assign btb_vbank2_rd_data_f1[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{fetch_start_f1[0]}} &  btb_bank1e_rd_data_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[1]}} &  btb_bank1o_rd_data_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[2]}} &  btb_bank0e_rd_data_p1_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[3]}} &  btb_bank0o_rd_data_p1_f1[BTB_DWIDTH-1:0]) );
   assign btb_vbank3_rd_data_f1[BTB_DWIDTH-1:0] = ( ({BTB_DWIDTH{fetch_start_f1[0]}} &  btb_bank1o_rd_data_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[1]}} &  btb_bank0e_rd_data_p1_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[2]}} &  btb_bank0o_rd_data_p1_f1[BTB_DWIDTH-1:0]) |
                                                     ({BTB_DWIDTH{fetch_start_f1[3]}} &  btb_bank1e_rd_data_p1_f1[BTB_DWIDTH-1:0]) );

   // ----------------------------------------------------------------------
   // WRITE
   // ----------------------------------------------------------------------

   // only write sram if validating entry
   assign wren_bank[1:0] = {btb_wren & btb_rw_addr[0][3] & btb_sram_wr_data[0], btb_wren & ~btb_rw_addr[0][3] & btb_sram_wr_data[0]};

   assign wr_way0_en = btb_wren & ~btb_rw_addr[0][1];
   assign wr_way1_en = btb_wren &  btb_rw_addr[0][1];
   // Way 0, addr 0
   assign btb_bit_en_vec[BTB_DWIDTH-1:0]              = {BTB_DWIDTH{wr_way0_en & ~btb_rw_addr[0][2]}};
   // Way 1, addr 0
   assign btb_bit_en_vec[2*BTB_DWIDTH-1:BTB_DWIDTH]   = {BTB_DWIDTH{wr_way1_en & ~btb_rw_addr[0][2]}};
   // Way 0, addr 4
   assign btb_bit_en_vec[3*BTB_DWIDTH-1:2*BTB_DWIDTH] = {BTB_DWIDTH{wr_way0_en & btb_rw_addr[0][2]}};
   // Way 1, addr 4
   assign btb_bit_en_vec[4*BTB_DWIDTH-1:3*BTB_DWIDTH] = {BTB_DWIDTH{wr_way1_en & btb_rw_addr[0][2]}};


   // Valid bit (F1)
   assign btb_write_entry[pt.BTB_SIZE-1:0] = ({{pt.BTB_SIZE-1{1'b0}},1'b1} << btb_rw_addr_f1[0]);
   assign btb_valid_ns[pt.BTB_SIZE-1:0] = (btb_wren_f1 & btb_sram_wr_datav_f1) ?
                                          (btb_valid[pt.BTB_SIZE-1:0] | btb_write_entry[pt.BTB_SIZE-1:0]) :
                                          ((btb_wren_f1 & ~btb_sram_wr_datav_f1) ? (btb_valid[pt.BTB_SIZE-1:0] & ~btb_write_entry[pt.BTB_SIZE-1:0]) :
                                           btb_valid[pt.BTB_SIZE-1:0]);
   rvdffe #(pt.BTB_SIZE) btb_valid_ff (.*, .clk(clk),
                                         .en(btb_wren_f1),
                                         .din  (btb_valid_ns[pt.BTB_SIZE-1:0]),
                                         .dout (btb_valid[pt.BTB_SIZE-1:0]));

   assign btb_rd0_valid[3] = btb_rden_f1 & btb_valid[{btb_rw_addr_f1[0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1],3'b011}];
   assign btb_rd0_valid[2] = btb_rden_f1 & btb_valid[{btb_rw_addr_f1[0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1],3'b010}];
   assign btb_rd0_valid[1] = btb_rden_f1 & btb_valid[{btb_rw_addr_f1[0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1],3'b001}];
   assign btb_rd0_valid[0] = btb_rden_f1 & btb_valid[{btb_rw_addr_f1[0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1],3'b000}];

   assign btb_rd1_valid[3] = btb_rden_f1 & btb_valid[{btb_rw_addr_f1[1][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1],3'b111}];
   assign btb_rd1_valid[2] = btb_rden_f1 & btb_valid[{btb_rw_addr_f1[1][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1],3'b110}];
   assign btb_rd1_valid[1] = btb_rden_f1 & btb_valid[{btb_rw_addr_f1[1][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1],3'b101}];
   assign btb_rd1_valid[0] = btb_rden_f1 & btb_valid[{btb_rw_addr_f1[1][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1],3'b100}];

   rvdff #(3) btb_rwdv_ff (.*, .clk(active_clk),
                           .din  ({btb_rden, btb_wren, btb_sram_wr_data[0]}),
                           .dout ({btb_rden_f1, btb_wren_f1, btb_sram_wr_datav_f1}));
   // ----------------------------------------------------------------------
   // SRAMS
   // ----------------------------------------------------------------------

   logic [1:0][1:0][pt.BTB_NUM_BYPASS_WIDTH-1:0]  wrptr_in, wrptr;

   logic [1:0][1:0] btb_b_read_en, btb_b_write_en, btb_bank_way_clken_final;

   logic [1:0][1:0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1] btb_b_rw_addr;

   logic [1:0][1:0]                                   any_addr_match, any_bypass;

   logic [1:0][1:0][pt.BTB_NUM_BYPASS-1:0]           btb_b_addr_match, btb_b_clear_en, sel_bypass, write_bypass_en;
   logic [1:0][1:0][pt.BTB_NUM_BYPASS-1:0]           write_bypass_en_ff, index_valid, sel_bypass_ff;

   logic [1:0][1:0][pt.BTB_NUM_BYPASS-1:0][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1] wb_index_hold;

   logic [1:0][1:0][pt.BTB_NUM_BYPASS-1:0][2*BTB_DWIDTH-1:0] wb_dout_hold;

   logic [1:0][1:0][2*BTB_DWIDTH-1:0]                sel_bypass_data;


`define EH2_BTB_SRAM(depth,width,bank,way)                                                                                          \
           ram_be_``depth``x``width btb_bank``bank``_``way (                                                                         \
                            // Primary ports                                                                                         \
                            .CLK     (clk),                                                                                          \
                            .ME      (btb_bank_way_clken_final[i][j]),                                                               \
                            .WE      (wren_bank[i]),                                                                                 \
                            .WEM     (btb_bit_en_vec[(j+1)*2*BTB_DWIDTH-1:j*2*BTB_DWIDTH]),                                          \
                            .ADR     (btb_rw_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1]), // This is 9:4 (bit 3 used as bank bit)      \
                            .D       ({2{btb_sram_wr_data[pt.BTB_TOFFSET_SIZE+pt.BTB_BTAG_SIZE+5-1:0]}}),                            \
                            .Q       (btb_rd_data_raw[i][j]),                                                                        \
                            .ROP (),                                                                                                 \
                            // These are used by SoC                                                                                 \
                            .TEST1   (btb_ext_in_pkt[i].TEST1),                                                                      \
                            .RME     (btb_ext_in_pkt[i].RME),                                                                        \
                            .RM      (btb_ext_in_pkt[i].RM),                                                                         \
                            .LS      (btb_ext_in_pkt[i].LS),                                                                         \
                            .DS      (btb_ext_in_pkt[i].DS),                                                                         \
                            .SD      (btb_ext_in_pkt[i].SD) ,                                                                        \
                            .TEST_RNM(btb_ext_in_pkt[i].TEST_RNM),                                                                   \
                            .BC1     (btb_ext_in_pkt[i].BC1),                                                                        \
                            .BC2     (btb_ext_in_pkt[i].BC2)                                                                         \
                            );                                                                                                       \
                                                                                                                                     \
if (pt.BTB_BYPASS_ENABLE == 1) begin                                                                                      \
                 assign wrptr_in[i][j] = (wrptr[i][j] == (pt.BTB_NUM_BYPASS-1)) ? '0 : (wrptr[i][j] + 1'd1);              \
                 rvdffs  #(pt.BTB_NUM_BYPASS_WIDTH)  wrptr_ff(.*, .clk(active_clk),  .en(|write_bypass_en[i][j]), .din (wrptr_in[i][j]), .dout(wrptr[i][j])) ;     \
                                                                                                                          \
                 assign btb_b_read_en[i][j]              =  ~wren_bank[i];                              \
                 assign btb_b_write_en[i][j]             =  wren_bank[i];                              \
                 assign btb_bank_way_clken_final[i][j]   =  ~(|sel_bypass[i][j]);                         \
                 assign btb_b_rw_addr[i][j]              =  btb_rw_addr[i][pt.BTB_ADDR_HI:pt.BTB_ADDR_LO+1];              \
                 always_comb begin                                                                                        \
                    any_addr_match[i][j] = '0;                                                                            \
                    for (int l=0; l<pt.BTB_NUM_BYPASS; l++) begin                                                         \
                       any_addr_match[i][j] |= btb_b_addr_match[i][j][l];                                                 \
                    end                                                                                                   \
                 end                                                                                                      \
                // it is an error to ever have 2 entries with the same index and both valid                               \
                for (genvar l=0; l<pt.BTB_NUM_BYPASS; l++) begin: BYPASS                                                  \
                   // full match up to bit 31                                                                             \
                   assign btb_b_addr_match[i][j][l] = (wb_index_hold[i][j][l] ==  btb_b_rw_addr[i][j]) & index_valid[i][j][l];            \
                   assign btb_b_clear_en[i][j][l]   = btb_b_write_en[i][j] &   btb_b_addr_match[i][j][l];                                 \
                   assign sel_bypass[i][j][l]       = btb_b_read_en[i][j]  &   btb_b_addr_match[i][j][l] ;                                \
                                                                                                                                          \
                   assign write_bypass_en[i][j][l]  = btb_b_read_en[i][j]  &  ~any_addr_match[i][j] & (wrptr[i][j] == l);                 \
                                                                                                                                          \
                   rvdff  #(1)  write_bypass_ff (.*, .clk(active_clk), .din(write_bypass_en[i][j][l]), .dout(write_bypass_en_ff[i][j][l])) ;        \
                   rvdffs #(1)  index_val_ff    (.*, .clk(active_clk), .en(write_bypass_en[i][j][l] | btb_b_clear_en[i][j][l]),                     \
                                                                       .din(~btb_b_clear_en[i][j][l]), .dout(index_valid[i][j][l])) ;               \
                   rvdff  #(1)  sel_hold_ff     (.*, .clk(active_clk), .din(sel_bypass[i][j][l]),      .dout(sel_bypass_ff[i][j][l])) ;             \
                   rvdffe #(.WIDTH(pt.BTB_ADDR_HI-pt.BTB_ADDR_LO), .OVERRIDE(1)) btb_addr_index    (.*, .en(write_bypass_en[i][j][l]),                                    \
                                                                       .din (btb_b_rw_addr[i][j]),     .dout(wb_index_hold[i][j][l]));              \
                   rvdffe #(2*BTB_DWIDTH) rd_data_hold_ff  (.*,        .en(write_bypass_en_ff[i][j][l]),                                            \
                                                                       .din (btb_rd_data_raw[i][j]),  .dout(wb_dout_hold[i][j][l]));           \
                end                                                                                                                       \
                always_comb begin                                                                                                         \
                 any_bypass[i][j] = '0;                                                                                                   \
                 sel_bypass_data[i][j] = '0;                                                                                              \
                 for (int l=0; l<pt.BTB_NUM_BYPASS; l++) begin                                                                            \
                    any_bypass[i][j]      |=  sel_bypass_ff[i][j][l];                                                                     \
                    sel_bypass_data[i][j] |= (sel_bypass_ff[i][j][l]) ? wb_dout_hold[i][j][l] : '0;                                       \
                 end                                                                                                                      \
                 btb_rd_data[i][j]  = any_bypass[i][j] ?  sel_bypass_data[i][j] : btb_rd_data_raw[i][j]  ;                                \
                 end                                                                                                                      \
             end                                                                                                                          \
             else begin                                                                                                                   \
                 assign btb_rd_data[i][j]  =   btb_rd_data_raw[i][j];                                                                     \
                 assign btb_bank_way_clken_final[i][j]   =  1'b1;                                                                         \
             end



   for (genvar i=0; i<2; i++) begin: BANKS
         for (genvar j=0; j<2; j++) begin: WAYS
            if      (pt.BTB_SIZE==4096) begin
              `EH2_BTB_SRAM(512,60,i,j)
            end
            else if (pt.BTB_SIZE==2048) begin
              `EH2_BTB_SRAM(256,60,i,j)
            end
            else if (pt.BTB_SIZE==1024) begin
              `EH2_BTB_SRAM(128,60,i,j)
            end
            else if (pt.BTB_SIZE==512) begin
              `EH2_BTB_SRAM(64,60,i,j)
            end
            else if (pt.BTB_SIZE==256) begin
              `EH2_BTB_SRAM(32,62,i,j)
            end
         end
   end

`undef TAG
endmodule // eh2_ifu_btb_mem


