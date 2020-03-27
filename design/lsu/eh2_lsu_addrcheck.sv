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
// Function: Checks the memory map for the address
// Comments:
//
//********************************************************************************
module eh2_lsu_addrcheck
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)(
   input logic              lsu_c2_dc2_clk,       // clock
   input logic              lsu_c2_dc3_clk,
   input logic              clk,
   input logic              rst_l,                       // reset

   input logic [31:0]       start_addr_dc1,              // start address for lsu
   input logic [31:0]       end_addr_dc1,                // end address for lsu
   input logic [31:0]       start_addr_dc2,              // start address for lsu
   input logic [31:0]       end_addr_dc2,                // end address for lsu
   input logic [31:0]       rs1_dc1,
   input eh2_lsu_pkt_t     lsu_pkt_dc1,                 // packet in dc1
   input eh2_lsu_pkt_t     lsu_pkt_dc2,                 // packet in dc1

   input logic [31:0]  dec_tlu_mrac_ff,           // CSR read

   output logic        is_sideeffects_dc2,          // is sideffects space
   output logic        is_sideeffects_dc3,
   output logic        addr_in_dccm_region_dc1,     // address in dccm region
   output logic        addr_in_dccm_dc1,            // address in dccm
   output logic        addr_in_pic_dc1,             // address in pic
   output logic        addr_external_dc1,           // address in external
   output logic        addr_external_dc2,           // address in external

   output logic        access_fault_dc2,            // access fault
   output logic        misaligned_fault_dc2,        // misaligned
   output logic [3:0]  exc_mscause_dc2,             // Exception cause

   output logic        fir_dccm_access_error_dc2,   // Fast interrupt dccm access error
   output logic        fir_nondccm_access_error_dc2,// Fast interrupt dccm access error

   input  logic        scan_mode
);


   logic        is_sideeffects_dc1, is_aligned_dc2;
   logic        start_addr_in_dccm_dc1, end_addr_in_dccm_dc1;
   logic        start_addr_in_dccm_region_dc1, end_addr_in_dccm_region_dc1;
   logic        start_addr_in_pic_dc1, end_addr_in_pic_dc1;
   logic        start_addr_in_pic_region_dc1, end_addr_in_pic_region_dc1;
   logic        addr_in_pic_region_dc1;
   logic        start_addr_in_dccm_dc2, end_addr_in_dccm_dc2;
   logic        start_addr_in_pic_dc2, end_addr_in_pic_dc2;
   logic        start_addr_in_dccm_region_dc2, end_addr_in_dccm_region_dc2;
   logic        start_addr_in_pic_region_dc2, end_addr_in_pic_region_dc2;
   logic        addr_in_dccm_dc2, addr_in_pic_dc2;
   logic [3:0]  rs1_region_dc1, rs1_region_dc2;              // region from the rs operand of the agu
   logic [4:0]  csr_idx;
   logic        addr_in_iccm;
   logic        start_addr_dccm_or_pic;
   logic        base_reg_dccm_or_pic;
   logic [31:0] rs1_dc2;
   logic        unmapped_access_fault_dc2, mpu_access_fault_dc2, picm_access_fault_dc2, regpred_access_fault_dc2, amo_access_fault_dc2;
   logic        regcross_misaligned_fault_dc2, sideeffect_misaligned_fault_dc2;
   logic [3:0]  access_fault_mscause_dc2;
   logic [3:0]  misaligned_fault_mscause_dc2;
   logic        non_dccm_access_ok;

   if (pt.DCCM_ENABLE == 1) begin: Gen_dccm_enable
      // Start address check
      rvrangecheck #(.CCM_SADR(pt.DCCM_SADR),
                     .CCM_SIZE(pt.DCCM_SIZE)) start_addr_dccm_rangecheck (
         .addr(start_addr_dc1[31:0]),
         .in_range(start_addr_in_dccm_dc1),
         .in_region(start_addr_in_dccm_region_dc1)
      );

      // End address check
      rvrangecheck #(.CCM_SADR(pt.DCCM_SADR),
                     .CCM_SIZE(pt.DCCM_SIZE)) end_addr_dccm_rangecheck (
         .addr(end_addr_dc1[31:0]),
         .in_range(end_addr_in_dccm_dc1),
         .in_region(end_addr_in_dccm_region_dc1)
      );
   end else begin: Gen_dccm_disable // block: Gen_dccm_enable
      assign start_addr_in_dccm_dc1 = '0;
      assign start_addr_in_dccm_region_dc1 = '0;
      assign end_addr_in_dccm_dc1 = '0;
      assign end_addr_in_dccm_region_dc1 = '0;
   end

   // ICCM region check
   if (pt.ICCM_ENABLE == 1) begin : check_iccm
     assign addr_in_iccm =  (start_addr_dc2[31:28] == pt.ICCM_REGION);
   end
   else begin
     assign addr_in_iccm = 1'b0;
   end

   // PIC memory check
   // Start address check
   rvrangecheck #(.CCM_SADR(pt.PIC_BASE_ADDR),
                  .CCM_SIZE(pt.PIC_SIZE)) start_addr_pic_rangecheck (
      .addr(start_addr_dc1[31:0]),
      .in_range(start_addr_in_pic_dc1),
      .in_region(start_addr_in_pic_region_dc1)
   );

   // End address check
   rvrangecheck #(.CCM_SADR(pt.PIC_BASE_ADDR),
                  .CCM_SIZE(pt.PIC_SIZE)) end_addr_pic_rangecheck (
      .addr(end_addr_dc1[31:0]),
      .in_range(end_addr_in_pic_dc1),
      .in_region(end_addr_in_pic_region_dc1)
   );

   assign rs1_region_dc1[3:0] = rs1_dc1[31:28];
   assign rs1_region_dc2[3:0] = rs1_dc2[31:28];
   assign start_addr_dccm_or_pic  = start_addr_in_dccm_region_dc2 | start_addr_in_pic_region_dc2;
   assign base_reg_dccm_or_pic    = ((rs1_region_dc2[3:0] == pt.DCCM_REGION) & pt.DCCM_ENABLE) | (rs1_region_dc2[3:0] == pt.PIC_REGION);

   assign addr_in_dccm_region_dc1 = (rs1_region_dc1[3:0] == pt.DCCM_REGION) & pt.DCCM_ENABLE;  // We don't need to look at final address since lsu will take an exception if final region is different
   assign addr_in_pic_region_dc1  = (rs1_region_dc1[3:0] == pt.PIC_REGION);   // We don't need to look at final address since lsu will take an exception if final region is different
   assign addr_in_dccm_dc1        = (start_addr_in_dccm_dc1 & end_addr_in_dccm_dc1);
   assign addr_in_pic_dc1         = (start_addr_in_pic_dc1 & end_addr_in_pic_dc1);

   assign addr_in_dccm_dc2        = (start_addr_in_dccm_dc2 & end_addr_in_dccm_dc2);
   assign addr_in_pic_dc2         = (start_addr_in_pic_dc2 & end_addr_in_pic_dc2);

   assign addr_external_dc1  = ~(addr_in_dccm_region_dc1 | addr_in_pic_region_dc1);  // look at the region based on rs1_dc1 for timing since this goes to busreq -> nbload_dc1 -> instbuf
   assign addr_external_dc2  = ~(start_addr_in_dccm_region_dc2 | start_addr_in_pic_region_dc2);  // look at the region based on rs1_dc1 for timing since this goes to busreq -> nbload_dc1 -> instbuf
   assign csr_idx[4:0]       = {start_addr_dc2[31:28], 1'b1};
   assign is_sideeffects_dc2 = dec_tlu_mrac_ff[csr_idx] & ~(start_addr_in_dccm_region_dc2 | start_addr_in_pic_region_dc2 | addr_in_iccm);  //every region has the 2 LSB indicating ( 1: sideeffects/no_side effects, and 0: cacheable ). Ignored in internal regions
   assign is_aligned_dc2    = (lsu_pkt_dc2.word & (start_addr_dc2[1:0] == 2'b0)) |
                              (lsu_pkt_dc2.half & (start_addr_dc2[0] == 1'b0)) |
                              lsu_pkt_dc2.by;

   assign non_dccm_access_ok = (~(|{pt.DATA_ACCESS_ENABLE0,pt.DATA_ACCESS_ENABLE1,pt.DATA_ACCESS_ENABLE2,pt.DATA_ACCESS_ENABLE3,pt.DATA_ACCESS_ENABLE4,pt.DATA_ACCESS_ENABLE5,pt.DATA_ACCESS_ENABLE6,pt.DATA_ACCESS_ENABLE7})) |
                               (((pt.DATA_ACCESS_ENABLE0 & ((start_addr_dc2[31:0] | pt.DATA_ACCESS_MASK0)) == (pt.DATA_ACCESS_ADDR0 | pt.DATA_ACCESS_MASK0)) |
                                 (pt.DATA_ACCESS_ENABLE1 & ((start_addr_dc2[31:0] | pt.DATA_ACCESS_MASK1)) == (pt.DATA_ACCESS_ADDR1 | pt.DATA_ACCESS_MASK1)) |
                                 (pt.DATA_ACCESS_ENABLE2 & ((start_addr_dc2[31:0] | pt.DATA_ACCESS_MASK2)) == (pt.DATA_ACCESS_ADDR2 | pt.DATA_ACCESS_MASK2)) |
                                 (pt.DATA_ACCESS_ENABLE3 & ((start_addr_dc2[31:0] | pt.DATA_ACCESS_MASK3)) == (pt.DATA_ACCESS_ADDR3 | pt.DATA_ACCESS_MASK3)) |
                                 (pt.DATA_ACCESS_ENABLE4 & ((start_addr_dc2[31:0] | pt.DATA_ACCESS_MASK4)) == (pt.DATA_ACCESS_ADDR4 | pt.DATA_ACCESS_MASK4)) |
                                 (pt.DATA_ACCESS_ENABLE5 & ((start_addr_dc2[31:0] | pt.DATA_ACCESS_MASK5)) == (pt.DATA_ACCESS_ADDR5 | pt.DATA_ACCESS_MASK5)) |
                                 (pt.DATA_ACCESS_ENABLE6 & ((start_addr_dc2[31:0] | pt.DATA_ACCESS_MASK6)) == (pt.DATA_ACCESS_ADDR6 | pt.DATA_ACCESS_MASK6)) |
                                 (pt.DATA_ACCESS_ENABLE7 & ((start_addr_dc2[31:0] | pt.DATA_ACCESS_MASK7)) == (pt.DATA_ACCESS_ADDR7 | pt.DATA_ACCESS_MASK7)))   &
                                ((pt.DATA_ACCESS_ENABLE0 & ((end_addr_dc2[31:0]   | pt.DATA_ACCESS_MASK0)) == (pt.DATA_ACCESS_ADDR0 | pt.DATA_ACCESS_MASK0)) |
                                 (pt.DATA_ACCESS_ENABLE1 & ((end_addr_dc2[31:0]   | pt.DATA_ACCESS_MASK1)) == (pt.DATA_ACCESS_ADDR1 | pt.DATA_ACCESS_MASK1)) |
                                 (pt.DATA_ACCESS_ENABLE2 & ((end_addr_dc2[31:0]   | pt.DATA_ACCESS_MASK2)) == (pt.DATA_ACCESS_ADDR2 | pt.DATA_ACCESS_MASK2)) |
                                 (pt.DATA_ACCESS_ENABLE3 & ((end_addr_dc2[31:0]   | pt.DATA_ACCESS_MASK3)) == (pt.DATA_ACCESS_ADDR3 | pt.DATA_ACCESS_MASK3)) |
                                 (pt.DATA_ACCESS_ENABLE4 & ((end_addr_dc2[31:0]   | pt.DATA_ACCESS_MASK4)) == (pt.DATA_ACCESS_ADDR4 | pt.DATA_ACCESS_MASK4)) |
                                 (pt.DATA_ACCESS_ENABLE5 & ((end_addr_dc2[31:0]   | pt.DATA_ACCESS_MASK5)) == (pt.DATA_ACCESS_ADDR5 | pt.DATA_ACCESS_MASK5)) |
                                 (pt.DATA_ACCESS_ENABLE6 & ((end_addr_dc2[31:0]   | pt.DATA_ACCESS_MASK6)) == (pt.DATA_ACCESS_ADDR6 | pt.DATA_ACCESS_MASK6)) |
                                 (pt.DATA_ACCESS_ENABLE7 & ((end_addr_dc2[31:0]   | pt.DATA_ACCESS_MASK7)) == (pt.DATA_ACCESS_ADDR7 | pt.DATA_ACCESS_MASK7))));

   // Access fault logic
   // 0. Unmapped local memory fault: Addr in dccm region but not in dccm offset OR Addr in picm region but not in picm offset OR DCCM -> PIC cross when DCCM/PIC in same region
   // 1. Uncorrectable (double bit) ECC error
   // 3. MPU access fault: Address is not in a populated non-dccm region
   // 5. Region prediction access fault: Base Address in DCCM/PIC and Final address in non-DCCM/non-PIC region or vice versa
   // 6. Ld/St access to picm are not word aligned or word size

   assign regpred_access_fault_dc2  = (start_addr_dccm_or_pic ^ base_reg_dccm_or_pic);                            // 5. Region prediction access fault: Base Address in DCCM/PIC and Final address in non-DCCM/non-PIC region or vice versa
   assign picm_access_fault_dc2     = (addr_in_pic_dc2 & ((start_addr_dc2[1:0] != 2'b0) | ~lsu_pkt_dc2.word));    // 6. Ld/St access to picm are not word aligned or word size
   assign amo_access_fault_dc2      =  (lsu_pkt_dc2.atomic & (start_addr_dc2[1:0] != 2'b0))                     | // 7. AMO are not word aligned OR AMO address not in dccm region
                                       (lsu_pkt_dc2.valid & lsu_pkt_dc2.atomic & ~addr_in_dccm_dc2);

   if (pt.DCCM_ENABLE & (pt.DCCM_REGION == pt.PIC_REGION)) begin
      assign unmapped_access_fault_dc2 = ((start_addr_in_dccm_region_dc2 & ~(start_addr_in_dccm_dc2 | start_addr_in_pic_dc2)) |   // 0. Addr in dccm/pic region but not in dccm/pic offset
                                        (end_addr_in_dccm_region_dc2 & ~(end_addr_in_dccm_dc2 | end_addr_in_pic_dc2))         |   // 0. Addr in dccm/pic region but not in dccm/pic offset
                                        (start_addr_in_dccm_dc2 & end_addr_in_pic_dc2)                                        |   // 0. DCCM -> PIC cross when DCCM/PIC in same region
                                        (start_addr_in_pic_dc2  & end_addr_in_dccm_dc2));                                         // 0. DCCM -> PIC cross when DCCM/PIC in same region
      assign mpu_access_fault_dc2      = (~start_addr_in_dccm_region_dc2 & ~non_dccm_access_ok);                                  // 3. Address is not in a populated non-dccm region
   end else begin
      assign unmapped_access_fault_dc2 = ((start_addr_in_dccm_region_dc2 & ~start_addr_in_dccm_dc2)                            |   // 0. Addr in dccm region but not in dccm offset
                                        (end_addr_in_dccm_region_dc2 & ~end_addr_in_dccm_dc2)                                  |   // 0. Addr in dccm region but not in dccm offset
                                        (start_addr_in_pic_region_dc2 & ~start_addr_in_pic_dc2)                                |   // 0. Addr in picm region but not in picm offset
                                        (end_addr_in_pic_region_dc2 & ~end_addr_in_pic_dc2));                                      // 9. Addr in picm region but not in picm offset
      assign mpu_access_fault_dc2      = (~start_addr_in_pic_region_dc2 & ~start_addr_in_dccm_region_dc2 & ~non_dccm_access_ok);   // 3. Address is not in a populated non-dccm region
   end

   assign access_fault_dc2 = (unmapped_access_fault_dc2 | mpu_access_fault_dc2 | picm_access_fault_dc2 |
                              regpred_access_fault_dc2 | amo_access_fault_dc2) & lsu_pkt_dc2.valid & ~lsu_pkt_dc2.dma;
   assign access_fault_mscause_dc2[3:0] = unmapped_access_fault_dc2 ? 4'h2 : mpu_access_fault_dc2 ? 4'h3 : regpred_access_fault_dc2 ? 4'h5 : picm_access_fault_dc2 ? 4'h6 : amo_access_fault_dc2 ? 4'h7 : 4'h0;

   // Misaligned happens due to 2 reasons (Atomic instructions (LR/SC/AMO) will never take misaligned as per spec)
   // 0. Region cross
   // 1. sideeffects access which are not aligned
   assign regcross_misaligned_fault_dc2 = (start_addr_dc2[31:28] != end_addr_dc2[31:28]);
   assign sideeffect_misaligned_fault_dc2 = (is_sideeffects_dc2 & ~is_aligned_dc2);
   assign misaligned_fault_dc2 = (regcross_misaligned_fault_dc2 | (sideeffect_misaligned_fault_dc2 & addr_external_dc2)) & lsu_pkt_dc2.valid & ~lsu_pkt_dc2.dma & ~lsu_pkt_dc2.atomic;
   assign misaligned_fault_mscause_dc2[3:0] = regcross_misaligned_fault_dc2 ? 4'h2 : sideeffect_misaligned_fault_dc2 ? 4'h1 : 4'h0;//sideeffect_misaligned_fault_dc2;

   assign exc_mscause_dc2[3:0] = misaligned_fault_dc2 ? misaligned_fault_mscause_dc2[3:0] : access_fault_mscause_dc2[3:0];

   // Fast interrupt error logic
   assign fir_dccm_access_error_dc2    = ((start_addr_in_dccm_region_dc2 & ~start_addr_in_dccm_dc2) |
                                          (end_addr_in_dccm_region_dc2   & ~end_addr_in_dccm_dc2)) & lsu_pkt_dc2.valid & lsu_pkt_dc2.fast_int;
   assign fir_nondccm_access_error_dc2 = ~(start_addr_in_dccm_region_dc2 & end_addr_in_dccm_region_dc2) & lsu_pkt_dc2.valid & lsu_pkt_dc2.fast_int;


   rvdff #(.WIDTH(1)) start_addr_in_dccm_dc2ff       (.din(start_addr_in_dccm_dc1),        .dout(start_addr_in_dccm_dc2),        .clk(lsu_c2_dc2_clk), .*);
   rvdff #(.WIDTH(1)) end_addr_in_dccm_dc2ff         (.din(end_addr_in_dccm_dc1),          .dout(end_addr_in_dccm_dc2),          .clk(lsu_c2_dc2_clk), .*);
   rvdff #(.WIDTH(1)) start_addr_in_pic_dc2ff        (.din(start_addr_in_pic_dc1),         .dout(start_addr_in_pic_dc2),         .clk(lsu_c2_dc2_clk), .*);
   rvdff #(.WIDTH(1)) end_addr_in_pic_dc2ff          (.din(end_addr_in_pic_dc1),           .dout(end_addr_in_pic_dc2),           .clk(lsu_c2_dc2_clk), .*);
   rvdff #(.WIDTH(1)) start_addr_in_dccm_region_dc2ff(.din(start_addr_in_dccm_region_dc1), .dout(start_addr_in_dccm_region_dc2), .clk(lsu_c2_dc2_clk), .*);
   rvdff #(.WIDTH(1)) start_addr_in_pic_region_dc2ff (.din(start_addr_in_pic_region_dc1),  .dout(start_addr_in_pic_region_dc2),  .clk(lsu_c2_dc2_clk), .*);
   rvdff #(.WIDTH(1)) end_addr_in_dccm_region_dc2ff  (.din(end_addr_in_dccm_region_dc1),   .dout(end_addr_in_dccm_region_dc2),   .clk(lsu_c2_dc2_clk), .*);
   rvdff #(.WIDTH(1)) end_addr_in_pic_region_dc2ff   (.din(end_addr_in_pic_region_dc1),    .dout(end_addr_in_pic_region_dc2),    .clk(lsu_c2_dc2_clk), .*);
   rvdffe #(.WIDTH(32)) rs1_dc2ff                    (.din(rs1_dc1[31:0]),                 .dout(rs1_dc2[31:0]),                 .en(lsu_pkt_dc1.valid), .*);
   rvdff #(.WIDTH(1)) is_sideeffects_dc3ff           (.din(is_sideeffects_dc2),            .dout(is_sideeffects_dc3),            .clk(lsu_c2_dc3_clk), .*);

endmodule // lsu_addrcheck
