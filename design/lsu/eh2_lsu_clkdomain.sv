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
// Function: Clock Generation Block
// Comments: All the clocks are generate here
//
// //********************************************************************************


module eh2_lsu_clkdomain
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)(
   input logic      clk,                               // clock
   input logic      free_clk,                          // clock
   input logic      rst_l,                             // reset

   // Inputs
   input logic      clk_override,                      // chciken bit to turn off clock gating
   input logic      addr_in_dccm_dc2,                  // address in dccm
   input logic      addr_in_pic_dc2,                   // address is in pic
   input logic      dma_dccm_req,                      // dma is active
   input logic      dma_mem_write,                     // dma write is active
   input logic      store_stbuf_reqvld_dc5,            // allocating in to the store queue
   input logic [pt.NUM_THREADS-1:0]lr_vld,                            // needed for clk gating


   input logic      stbuf_reqvld_any,                  // stbuf is draining
   input logic      stbuf_reqvld_flushed_any,          // stbuf is flushed
   input logic      lsu_busreq_dc5,                    // busreq in dc5
   input logic [pt.NUM_THREADS-1:0] lsu_bus_buffer_pend_any,           // bus buffer has a pending bus entry
   input logic [pt.NUM_THREADS-1:0] lsu_bus_buffer_empty_any,          // external bus buffer is empty
   input logic [pt.NUM_THREADS-1:0] lsu_stbuf_empty_any,               // stbuf is empty

   input logic      lsu_bus_clk_en,               // bus clock enable

   input eh2_lsu_pkt_t  lsu_p,                             // lsu packet in decode
   input eh2_lsu_pkt_t  lsu_pkt_dc1,                       // lsu packet in dc1
   input eh2_lsu_pkt_t  lsu_pkt_dc2,                       // lsu packet in dc2
   input eh2_lsu_pkt_t  lsu_pkt_dc3,                       // lsu packet in dc3
   input eh2_lsu_pkt_t  lsu_pkt_dc4,                       // lsu packet in dc4
   input eh2_lsu_pkt_t  lsu_pkt_dc5,                       // lsu packet in dc5

   // Outputs
   output logic     lsu_c1_dc1_clk,                    // dc3 pipe single pulse clock
   output logic     lsu_c1_dc2_clk,                    // dc3 pipe single pulse clock
   output logic     lsu_c1_dc3_clk,                    // dc3 pipe single pulse clock
   output logic     lsu_c1_dc4_clk,                    // dc4 pipe single pulse clock
   output logic     lsu_c1_dc5_clk,                    // dc5 pipe single pulse clock

   output logic     lsu_c2_dc1_clk,                    // dc3 pipe double pulse clock
   output logic     lsu_c2_dc2_clk,                    // dc3 pipe double pulse clock
   output logic     lsu_c2_dc3_clk,                    // dc3 pipe double pulse clock
   output logic     lsu_c2_dc4_clk,                    // dc4 pipe double pulse clock
   output logic     lsu_c2_dc5_clk,                    // dc5 pipe double pulse clock

   output logic     lsu_store_c1_dc1_clk,              // store in dc1
   output logic     lsu_store_c1_dc2_clk,              // store in dc2
   output logic     lsu_store_c1_dc3_clk,              // store in dc3


   output logic     lsu_dccm_c1_dc3_clk,               // dccm clock
   output logic     lsu_pic_c1_dc3_clk,                // pic clock

   output logic     lsu_stbuf_c1_clk,
   output logic [pt.NUM_THREADS-1:0]  lsu_bus_obuf_c1_clk,               // ibuf clock
   output logic [pt.NUM_THREADS-1:0]  lsu_bus_ibuf_c1_clk,               // ibuf clock
   output logic [pt.NUM_THREADS-1:0]  lsu_bus_buf_c1_clk,                // ibuf clock
   output logic     lsu_busm_clk,                      // bus clock

   output logic     lsu_free_c2_clk,

   input  logic     scan_mode
);

   logic lsu_c1_dc1_clken,       lsu_c1_dc2_clken,       lsu_c1_dc3_clken,       lsu_c1_dc4_clken,       lsu_c1_dc5_clken;
   logic lsu_c2_dc1_clken,       lsu_c2_dc2_clken,       lsu_c2_dc3_clken,       lsu_c2_dc4_clken,       lsu_c2_dc5_clken;
   logic lsu_c1_dc1_clken_q,     lsu_c1_dc2_clken_q,     lsu_c1_dc3_clken_q,     lsu_c1_dc4_clken_q,     lsu_c1_dc5_clken_q;
   logic lsu_store_c1_dc1_clken, lsu_store_c1_dc2_clken, lsu_store_c1_dc3_clken;

   logic lsu_stbuf_c1_clken;
   logic [pt.NUM_THREADS-1:0] lsu_bus_ibuf_c1_clken, lsu_bus_obuf_c1_clken, lsu_bus_buf_c1_clken;

   logic lsu_dccm_c1_dc3_clken, lsu_pic_c1_dc3_clken;

   logic lsu_free_c1_clken, lsu_free_c1_clken_q, lsu_free_c2_clken;

   //-------------------------------------------------------------------------------------------
   // Clock Enable logic
   //-------------------------------------------------------------------------------------------

   // Also use the flopped clock enable. We want to turn on the clocks from dc1->dc5 even if there is a freeze
   assign lsu_c1_dc1_clken = lsu_p.valid | clk_override;
   assign lsu_c1_dc2_clken = lsu_pkt_dc1.valid | dma_dccm_req | lsu_c1_dc1_clken_q | clk_override;
   assign lsu_c1_dc3_clken = lsu_pkt_dc2.valid | lsu_c1_dc2_clken_q | clk_override;
   assign lsu_c1_dc4_clken = lsu_pkt_dc3.valid | lsu_c1_dc3_clken_q | clk_override;
   assign lsu_c1_dc5_clken = lsu_pkt_dc4.valid | lsu_c1_dc4_clken_q | clk_override;

   assign lsu_c2_dc1_clken = lsu_c1_dc1_clken | lsu_c1_dc1_clken_q | clk_override;
   assign lsu_c2_dc2_clken = lsu_c1_dc2_clken | lsu_c1_dc2_clken_q | clk_override;
   assign lsu_c2_dc3_clken = lsu_c1_dc3_clken | lsu_c1_dc3_clken_q | clk_override;
   assign lsu_c2_dc4_clken = lsu_c1_dc4_clken | lsu_c1_dc4_clken_q | clk_override;
   assign lsu_c2_dc5_clken = lsu_c1_dc5_clken | lsu_c1_dc5_clken_q | clk_override;

   assign lsu_store_c1_dc1_clken = ((lsu_c1_dc1_clken & (lsu_p.store | lsu_p.atomic )) | clk_override);
   assign lsu_store_c1_dc2_clken = ((lsu_c1_dc2_clken & (lsu_pkt_dc1.store | dma_mem_write | lsu_pkt_dc1.atomic)) | clk_override);
   assign lsu_store_c1_dc3_clken = ((lsu_c1_dc3_clken & (lsu_pkt_dc2.store | lsu_pkt_dc2.atomic)) | clk_override);


   assign lsu_stbuf_c1_clken = store_stbuf_reqvld_dc5 | stbuf_reqvld_any | stbuf_reqvld_flushed_any | clk_override;

   for (genvar i=0; i<pt.NUM_THREADS; i++) begin: GenBufClkEn
      assign lsu_bus_ibuf_c1_clken[i] = (lsu_busreq_dc5 & (lsu_pkt_dc5.tid == i)) | clk_override;
      assign lsu_bus_obuf_c1_clken[i] = (lsu_bus_buffer_pend_any[i] | (lsu_busreq_dc5 & (lsu_pkt_dc5.tid == i)) | clk_override) & lsu_bus_clk_en;
      assign lsu_bus_buf_c1_clken[i]  = ~lsu_bus_buffer_empty_any[i] | (lsu_busreq_dc5 & (lsu_pkt_dc5.tid == i)) | clk_override;

      rvoclkhdr lsu_bus_ibuf_c1_cgc ( .en(lsu_bus_ibuf_c1_clken[i]), .l1clk(lsu_bus_ibuf_c1_clk[i]), .* );
      rvclkhdr  lsu_bus_obuf_c1_cgc ( .en(lsu_bus_obuf_c1_clken[i]), .l1clk(lsu_bus_obuf_c1_clk[i]), .* );
      rvoclkhdr lsu_bus_buf_c1_cgc  ( .en(lsu_bus_buf_c1_clken[i]),  .l1clk(lsu_bus_buf_c1_clk[i]), .* );
   end

   assign lsu_dccm_c1_dc3_clken = ((lsu_c1_dc3_clken & addr_in_dccm_dc2) | clk_override);
   assign lsu_pic_c1_dc3_clken  = ((lsu_c1_dc3_clken & addr_in_pic_dc2) | clk_override);

   assign lsu_free_c1_clken =  lsu_p.valid | lsu_pkt_dc1.valid | lsu_pkt_dc2.valid | lsu_pkt_dc3.valid | lsu_pkt_dc4.valid | lsu_pkt_dc5.valid | (|lr_vld[pt.NUM_THREADS-1:0]) |
                              ~(&lsu_bus_buffer_empty_any[pt.NUM_THREADS-1:0]) | ~(&lsu_stbuf_empty_any[pt.NUM_THREADS-1:0]) | clk_override;
   assign lsu_free_c2_clken = lsu_free_c1_clken | lsu_free_c1_clken_q | clk_override;

    // Flops
   rvdff #(1) lsu_free_c1_clkenff (.din(lsu_free_c1_clken), .dout(lsu_free_c1_clken_q), .clk(free_clk), .*);

   rvdff #(1) lsu_c1_dc1_clkenff (.din(lsu_c1_dc1_clken), .dout(lsu_c1_dc1_clken_q), .clk(free_clk), .*);
   rvdff #(1) lsu_c1_dc2_clkenff (.din(lsu_c1_dc2_clken), .dout(lsu_c1_dc2_clken_q), .clk(free_clk), .*);
   rvdff #(1) lsu_c1_dc3_clkenff (.din(lsu_c1_dc3_clken), .dout(lsu_c1_dc3_clken_q), .clk(free_clk), .*);
   rvdff #(1) lsu_c1_dc4_clkenff (.din(lsu_c1_dc4_clken), .dout(lsu_c1_dc4_clken_q), .clk(free_clk), .*);
   rvdff #(1) lsu_c1_dc5_clkenff (.din(lsu_c1_dc5_clken), .dout(lsu_c1_dc5_clken_q), .clk(free_clk), .*);

   // Clock Headers
   rvoclkhdr lsu_c1dc1_cgc ( .en(lsu_c1_dc1_clken), .l1clk(lsu_c1_dc1_clk), .* );
   rvoclkhdr lsu_c1dc2_cgc ( .en(lsu_c1_dc2_clken), .l1clk(lsu_c1_dc2_clk), .* );
   rvoclkhdr lsu_c1dc3_cgc ( .en(lsu_c1_dc3_clken), .l1clk(lsu_c1_dc3_clk), .* );
   rvoclkhdr lsu_c1dc4_cgc ( .en(lsu_c1_dc4_clken), .l1clk(lsu_c1_dc4_clk), .* );
   rvoclkhdr lsu_c1dc5_cgc ( .en(lsu_c1_dc5_clken), .l1clk(lsu_c1_dc5_clk), .* );

   rvoclkhdr lsu_c2dc1_cgc ( .en(lsu_c2_dc1_clken), .l1clk(lsu_c2_dc1_clk), .* );
   rvoclkhdr lsu_c2dc2_cgc ( .en(lsu_c2_dc2_clken), .l1clk(lsu_c2_dc2_clk), .* );
   rvoclkhdr lsu_c2dc3_cgc ( .en(lsu_c2_dc3_clken), .l1clk(lsu_c2_dc3_clk), .* );
   rvoclkhdr lsu_c2dc4_cgc ( .en(lsu_c2_dc4_clken), .l1clk(lsu_c2_dc4_clk), .* );
   rvoclkhdr lsu_c2dc5_cgc ( .en(lsu_c2_dc5_clken), .l1clk(lsu_c2_dc5_clk), .* );

   rvoclkhdr lsu_store_c1dc1_cgc (.en(lsu_store_c1_dc1_clken), .l1clk(lsu_store_c1_dc1_clk), .*);
   rvoclkhdr lsu_store_c1dc2_cgc (.en(lsu_store_c1_dc2_clken), .l1clk(lsu_store_c1_dc2_clk), .*);
   rvoclkhdr lsu_store_c1dc3_cgc (.en(lsu_store_c1_dc3_clken), .l1clk(lsu_store_c1_dc3_clk), .*);

   rvoclkhdr lsu_stbuf_c1_cgc ( .en(lsu_stbuf_c1_clken), .l1clk(lsu_stbuf_c1_clk), .* );

   rvclkhdr lsu_busm_cgc (.en(lsu_bus_clk_en), .l1clk(lsu_busm_clk), .*);

   rvoclkhdr lsu_dccm_c1dc3_cgc (.en(lsu_dccm_c1_dc3_clken), .l1clk(lsu_dccm_c1_dc3_clk), .*);
   rvoclkhdr lsu_pic_c1dc3_cgc (.en(lsu_pic_c1_dc3_clken), .l1clk(lsu_pic_c1_dc3_clk), .*);

   rvoclkhdr lsu_free_cgc (.en(lsu_free_c2_clken), .l1clk(lsu_free_c2_clk), .*);

endmodule

