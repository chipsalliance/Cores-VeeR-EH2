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
// $Id$
//
//
// Owner:
// Function: Top level file for load store unit
// Comments:
//
//
// DC1 -> DC2 -> DC3 -> DC4 (Commit)
//
//********************************************************************************
module eh2_lsu_ecc
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
(

   input logic                          clk,
   input logic                          scan_mode,             // scan mode
   input logic                          rst_l,
   input eh2_lsu_pkt_t                 lsu_pkt_dc3,        // packet in dc3
   input logic                          lsu_dccm_rden_dc3,  // dccm rden
   input logic                          addr_in_dccm_dc3,   // address in dccm
   input logic [pt.DCCM_BITS-1:0]       lsu_addr_dc3,    // start address
   input logic [pt.DCCM_BITS-1:0]       end_addr_dc3,    // end address
   input logic [31:0]                   store_data_dc3,  // store data
   input logic [pt.DCCM_DATA_WIDTH-1:0] stbuf_data_any,

   input logic [pt.DCCM_DATA_WIDTH-1:0] dccm_data_hi_dc3,     // raw data from mem
   input logic [pt.DCCM_DATA_WIDTH-1:0] dccm_data_lo_dc3,     // raw data from mem
   input logic [pt.DCCM_ECC_WIDTH-1:0]  dccm_data_ecc_hi_dc3, // ecc read out from mem
   input logic [pt.DCCM_ECC_WIDTH-1:0]  dccm_data_ecc_lo_dc3, // ecc read out from mem

   input logic  [pt.DCCM_DATA_WIDTH-1:0] sec_data_hi_dc5,
   input logic  [pt.DCCM_DATA_WIDTH-1:0] sec_data_lo_dc5,

   input logic                           ld_single_ecc_error_dc5,      // ld has a single ecc error
   input logic                           ld_single_ecc_error_dc5_ff,   // ld has a single ecc error
   input logic                           ld_single_ecc_error_lo_dc5_ff,   // ld has a single ecc error
   input logic                           ld_single_ecc_error_hi_dc5_ff,   // ld has a single ecc error
   input logic                           dec_tlu_core_ecc_disable,     // disables the ecc computation and error flagging
   input logic                           disable_ecc_check_lo_dc3,
   input logic                           disable_ecc_check_hi_dc3,
   input logic                           misaligned_fault_dc3,
   input logic                           access_fault_dc3,

   input logic                           dma_dccm_spec_wen,
   input logic  [31:0]                   dma_dccm_wdata_lo,
   input logic  [31:0]                   dma_dccm_wdata_hi,

   output logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_wr_data_hi,
   output logic [pt.DCCM_FDATA_WIDTH-1:0]  dccm_wr_data_lo,

   output logic [pt.DCCM_DATA_WIDTH-1:0] sec_data_hi_dc3,
   output logic [pt.DCCM_DATA_WIDTH-1:0] sec_data_lo_dc3,

   output logic [pt.DCCM_DATA_WIDTH-1:0] store_ecc_data_hi_dc3,  // final store data either from stbuf or SEC DCCM readout
   output logic [pt.DCCM_DATA_WIDTH-1:0] store_ecc_data_lo_dc3,

   output logic                          single_ecc_error_hi_dc3,                   // sec detected
   output logic                          single_ecc_error_lo_dc3,                   // sec detected on lower dccm bank
   output logic                          lsu_single_ecc_error_dc3,                  // or of the 2
   output logic                          lsu_double_ecc_error_dc3                   // double error detected

 );

   logic        double_ecc_error_hi_dc3, double_ecc_error_lo_dc3;
   logic [pt.DCCM_ECC_WIDTH-1:0]  dccm_wdata_ecc_hi_any, dccm_wdata_ecc_lo_any;

   logic        ldst_dual_dc3;
   logic        is_ldst_dc3;
   logic        is_ldst_hi_dc3, is_ldst_lo_dc3;
   logic [7:0]  ldst_byteen_dc3;
   logic [7:0]  store_byteen_dc3;
   logic [7:0]  store_byteen_ext_dc3;
   logic [pt.DCCM_BYTE_WIDTH-1:0]       store_byteen_hi_dc3, store_byteen_lo_dc3;

   logic [55:0] store_data_ext_dc3;
   logic [pt.DCCM_DATA_WIDTH-1:0]  store_data_hi_dc3, store_data_lo_dc3;
   logic [6:0]                  ecc_out_hi_nc, ecc_out_lo_nc;

   logic                       single_ecc_error_hi_raw_dc3, single_ecc_error_lo_raw_dc3;
   logic  [pt.DCCM_DATA_WIDTH-1:0] sec_data_hi_dc5_ff, sec_data_lo_dc5_ff;

   //------------------------------------------------------------------------------------------------------------
   //----------------------------------------Logic starts here---------------------------------------------------
   //------------------------------------------------------------------------------------------------------------

   assign ldst_dual_dc3 = (lsu_addr_dc3[2] != end_addr_dc3[2]);
   assign is_ldst_dc3 = lsu_pkt_dc3.valid & (lsu_pkt_dc3.load | lsu_pkt_dc3.store) & addr_in_dccm_dc3 & lsu_dccm_rden_dc3;
   assign is_ldst_lo_dc3 = is_ldst_dc3 & ~(dec_tlu_core_ecc_disable | disable_ecc_check_lo_dc3);
   assign is_ldst_hi_dc3 = is_ldst_dc3 & (ldst_dual_dc3 | lsu_pkt_dc3.dma) & ~(dec_tlu_core_ecc_disable | disable_ecc_check_hi_dc3);

   assign ldst_byteen_dc3[7:0] = ({8{lsu_pkt_dc3.by}}   & 8'b0000_0001) |
                                 ({8{lsu_pkt_dc3.half}} & 8'b0000_0011) |
                                 ({8{lsu_pkt_dc3.word}} & 8'b0000_1111) |
                                 ({8{lsu_pkt_dc3.dword}} & 8'b1111_1111);
   assign store_byteen_dc3[7:0] = ldst_byteen_dc3[7:0] & {8{~lsu_pkt_dc3.load}};

   assign store_byteen_ext_dc3[7:0] = store_byteen_dc3[7:0] << lsu_addr_dc3[1:0];
   assign store_byteen_hi_dc3[pt.DCCM_BYTE_WIDTH-1:0] = store_byteen_ext_dc3[7:4];
   assign store_byteen_lo_dc3[pt.DCCM_BYTE_WIDTH-1:0] = store_byteen_ext_dc3[3:0];

   assign store_data_ext_dc3[55:0] = {24'b0,store_data_dc3[31:0]} << {lsu_addr_dc3[1:0], 3'b000};
   assign store_data_hi_dc3[pt.DCCM_DATA_WIDTH-1:0]  = {8'b0,store_data_ext_dc3[55:32]};
   assign store_data_lo_dc3[pt.DCCM_DATA_WIDTH-1:0]  = store_data_ext_dc3[31:0];


   // Merge store data and sec data
   // This is used for loads as well for ecc error case. store_byteen will be 0 for loads
   for (genvar i=0; i<pt.DCCM_BYTE_WIDTH; i++) begin
      assign store_ecc_data_hi_dc3[(8*i)+7:(8*i)] = store_byteen_hi_dc3[i]  ? store_data_hi_dc3[(8*i)+7:(8*i)] : ({8{addr_in_dccm_dc3}} & sec_data_hi_dc3[(8*i)+7:(8*i)]);
      assign store_ecc_data_lo_dc3[(8*i)+7:(8*i)] = store_byteen_lo_dc3[i]  ? store_data_lo_dc3[(8*i)+7:(8*i)] : ({8{addr_in_dccm_dc3}} & sec_data_lo_dc3[(8*i)+7:(8*i)]);
   end

   assign dccm_wr_data_lo[pt.DCCM_DATA_WIDTH-1:0] = dma_dccm_spec_wen ? dma_dccm_wdata_lo[pt.DCCM_DATA_WIDTH-1:0] :
                                                    (ld_single_ecc_error_dc5_ff ? (ld_single_ecc_error_lo_dc5_ff ? sec_data_lo_dc5_ff[pt.DCCM_DATA_WIDTH-1:0] : sec_data_hi_dc5_ff[pt.DCCM_DATA_WIDTH-1:0]) : stbuf_data_any[pt.DCCM_DATA_WIDTH-1:0]);
   assign dccm_wr_data_hi[pt.DCCM_DATA_WIDTH-1:0] = dma_dccm_spec_wen ? dma_dccm_wdata_hi[pt.DCCM_DATA_WIDTH-1:0] :
                                                    (ld_single_ecc_error_dc5_ff ? (ld_single_ecc_error_hi_dc5_ff ? sec_data_hi_dc5_ff[pt.DCCM_DATA_WIDTH-1:0] : sec_data_lo_dc5_ff[pt.DCCM_DATA_WIDTH-1:0]) : stbuf_data_any[pt.DCCM_DATA_WIDTH-1:0]);

   assign dccm_wr_data_lo[pt.DCCM_FDATA_WIDTH-1:pt.DCCM_DATA_WIDTH] = dccm_wdata_ecc_lo_any[pt.DCCM_ECC_WIDTH-1:0];
   assign dccm_wr_data_hi[pt.DCCM_FDATA_WIDTH-1:pt.DCCM_DATA_WIDTH] = dccm_wdata_ecc_hi_any[pt.DCCM_ECC_WIDTH-1:0];

   if (pt.DCCM_ENABLE == 1) begin: Gen_dccm_enable
      //Detect/Repair for Hi/Lo
      rvecc_decode lsu_ecc_decode_hi (
         // Inputs
         .en(is_ldst_hi_dc3),
         .sed_ded (1'b0),    // 1 : means only detection
         .din(dccm_data_hi_dc3[pt.DCCM_DATA_WIDTH-1:0]),
         .ecc_in(dccm_data_ecc_hi_dc3[pt.DCCM_ECC_WIDTH-1:0]),
         // Outputs
         .dout(sec_data_hi_dc3[pt.DCCM_DATA_WIDTH-1:0]),
         .ecc_out (ecc_out_hi_nc[6:0]),
         .single_ecc_error(single_ecc_error_hi_raw_dc3),
         .double_ecc_error(double_ecc_error_hi_dc3),
         .*
      );

      rvecc_decode lsu_ecc_decode_lo (
         // Inputs
         .en(is_ldst_lo_dc3),
         .sed_ded (1'b0),    // 1 : means only detection
         .din(dccm_data_lo_dc3[pt.DCCM_DATA_WIDTH-1:0] ),
         .ecc_in(dccm_data_ecc_lo_dc3[pt.DCCM_ECC_WIDTH-1:0]),
         // Outputs
         .dout(sec_data_lo_dc3[pt.DCCM_DATA_WIDTH-1:0]),
         .ecc_out (ecc_out_lo_nc[6:0]),
         .single_ecc_error(single_ecc_error_lo_raw_dc3),
         .double_ecc_error(double_ecc_error_lo_dc3),
         .*
      );

      rvecc_encode lsu_ecc_encode_hi (
         //Inputs
         .din(dccm_wr_data_hi[pt.DCCM_DATA_WIDTH-1:0]),
         //Outputs
         .ecc_out(dccm_wdata_ecc_hi_any[pt.DCCM_ECC_WIDTH-1:0]),
         .*
      );
      rvecc_encode lsu_ecc_encode_lo (
         //Inputs
         .din(dccm_wr_data_lo[pt.DCCM_DATA_WIDTH-1:0]),
         //Outputs
         .ecc_out(dccm_wdata_ecc_lo_any[pt.DCCM_ECC_WIDTH-1:0]),
         .*
      );

      assign single_ecc_error_hi_dc3  = single_ecc_error_hi_raw_dc3 & ~(misaligned_fault_dc3 | access_fault_dc3);
      assign single_ecc_error_lo_dc3  = single_ecc_error_lo_raw_dc3 & ~(misaligned_fault_dc3 | access_fault_dc3);
      assign lsu_single_ecc_error_dc3 = single_ecc_error_hi_dc3 | single_ecc_error_lo_dc3;
      assign lsu_double_ecc_error_dc3 = double_ecc_error_hi_dc3 | double_ecc_error_lo_dc3;

      rvdffe #(.WIDTH(pt.DCCM_DATA_WIDTH)) sec_data_hi_dc5plus1ff (.din(sec_data_hi_dc5[pt.DCCM_DATA_WIDTH-1:0]), .dout(sec_data_hi_dc5_ff[pt.DCCM_DATA_WIDTH-1:0]), .en(ld_single_ecc_error_dc5), .clk(clk), .*);
      rvdffe #(.WIDTH(pt.DCCM_DATA_WIDTH)) sec_data_lo_dc5plus1ff (.din(sec_data_lo_dc5[pt.DCCM_DATA_WIDTH-1:0]), .dout(sec_data_lo_dc5_ff[pt.DCCM_DATA_WIDTH-1:0]), .en(ld_single_ecc_error_dc5), .clk(clk), .*);

   end else begin: Gen_dccm_disable // block: Gen_dccm_enable
      assign sec_data_hi_dc3[pt.DCCM_DATA_WIDTH-1:0] = '0;
      assign sec_data_lo_dc3[pt.DCCM_DATA_WIDTH-1:0] = '0;
      assign single_ecc_error_hi_dc3 = '0;
      assign double_ecc_error_hi_dc3 = '0;
      assign single_ecc_error_lo_dc3 = '0;
      assign double_ecc_error_lo_dc3 = '0;
      assign lsu_single_ecc_error_dc3 = '0;
      assign lsu_double_ecc_error_dc3 = '0;
      assign sec_data_lo_dc5_ff[pt.DCCM_DATA_WIDTH-1:0] = '0;
      assign sec_data_hi_dc5_ff[pt.DCCM_DATA_WIDTH-1:0] = '0;

   end

`ifdef ASSERT_ON


`endif

endmodule // lsu_ecc
