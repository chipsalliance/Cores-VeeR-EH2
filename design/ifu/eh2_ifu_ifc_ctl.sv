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
// eh2_ifu_ifc_ctl.sv
// Function: Fetch pipe control
//
// Comments:
//********************************************************************************

module eh2_ifu_ifc_ctl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (
   input logic clk,
   input logic free_clk,
   input logic active_clk,

   input logic clk_override, // overrides clock gating
   input logic rst_l, // reset enable, from core pin
   input logic scan_mode, // scan

   input logic ic_hit_f2,      // Icache hit
   input logic ic_crit_wd_rdy, // Crit word ready to be forwarded
   input logic ifu_ic_mb_empty, // Miss buffer empty

   input logic ifu_fb_consume1,  // Aligner consumed 1 fetch buffer
   input logic ifu_fb_consume2,  // Aligner consumed 2 fetch buffers

   input logic dec_tlu_flush_noredir_wb, // Don't fetch on flush
   input logic exu_flush_final, // FLush
   input logic [31:1] exu_flush_path_final, // Flush path

   input logic ifu_bp_kill_next_f2, // kill next fetch, taken target found
   input logic [31:1] ifu_bp_btb_target_f2, //  predicted target PC

   input logic ic_dma_active, // IC DMA active, stop fetching
   input logic ic_write_stall, // IC is writing, stop fetching
   input logic dma_iccm_stall_any, // force a stall in the fetch pipe for DMA ICCM access

   input logic [31:0]  dec_tlu_mrac_ff ,   // side_effect and cacheable for each region

   input logic tid,
   input logic ifc_select_tid_f1,

   output logic  fetch_uncacheable_f1, // fetch to uncacheable address as determined by MRAC

   output logic [31:1] fetch_addr_f1, // fetch addr F1

   output logic  fetch_req_f1,  // fetch request valid F1
   output logic  fetch_req_f1_raw, // for clock-gating in mem_ctl
   output logic  fetch_req_f2,  // fetch request valid F2

   output logic  pmu_fetch_stall, // pmu event measuring fetch stall

   output logic  iccm_access_f1, // fetch to ICCM region
   output logic  region_acc_fault_f1, // fetch access fault
   output logic  dma_access_ok, // fetch is not accessing the ICCM, DMA can proceed
   output logic  ready // ready to fetch
   );


   logic [31:1]  fetch_addr_bf,  miss_addr, ifc_fetch_addr_f1_raw, ifc_fetch_addr_f2;
   logic [31:3]  fetch_addr_next;
   logic [31:1]  miss_addr_ns;
   logic [4:0]   cacheable_select;
   logic [4:0]   fb_write_f1, fb_write_ns;

   logic         ifc_fetch_req_bf;
   logic         fb_full_f1_ns, fb_full_f1;
   logic         fb_right, fb_right2, fb_right3, fb_left, wfm, fetch_ns, idle;
   logic         fetch_req_f2_ns;
   logic         missff_en;
   logic         fetch_crit_word, ic_crit_wd_rdy_d1, fetch_crit_word_d1, fetch_crit_word_d2, my_bp_kill_next_f2;
   logic         sel_last_addr_bf, sel_miss_addr_bf, sel_btb_addr_bf, sel_next_addr_bf;
   logic         miss_f2, miss_a;
   logic         flush_fb, dma_iccm_stall_any_f;
   logic         mb_empty_mod, goto_idle, leave_idle;
   logic         ic_crit_wd_rdy_mod;
   logic         miss_sel_flush;
   logic         miss_sel_f2;
   logic         miss_sel_f1;
   logic         miss_sel_bf;
   logic         fetch_bf_en;
   logic         ifc_fetch_req_f2_raw;
   logic         line_wrap, lost_arb;
   logic [2:1]   fetch_addr_next_2_1;

   logic         ifc_f2_clk;

   logic         fetch_req_f1_won;
   logic         reset_delayed;
   logic         iccm_acc_in_range_f1;
   logic         iccm_acc_in_region_f1;



   if (pt.ICCM_ENABLE == 1)
     begin
        logic iccm_acc_in_region_f1;
        logic iccm_acc_in_range_f1;
     end
   logic dma_stall;

   rvoclkhdr ifu_fa2_cgc ( .en(fetch_req_f1_won | clk_override), .l1clk(ifc_f2_clk), .* );

   // FSM assignment
    typedef enum logic [1:0] { IDLE  = 2'b00,
                               FETCH = 2'b01,
                               STALL = 2'b10,
                               WFM   = 2'b11 } state_t ;
   state_t state      ;
   state_t next_state ;

   assign dma_stall = ic_dma_active | dma_iccm_stall_any_f;

   assign reset_delayed = 1'b0;

   rvdff #(2) ran_ff (.*, .clk(free_clk), .din({dma_iccm_stall_any, miss_f2}), .dout({dma_iccm_stall_any_f, miss_a}));

   // If crit word fetch is blocked, try again
   assign ic_crit_wd_rdy_mod = ic_crit_wd_rdy & ~((fetch_crit_word_d2 | ic_write_stall) & ~fetch_req_f2);

   // For Ifills, we fetch the critical word. Needed for perf and for rom bypass
   assign fetch_crit_word = ic_crit_wd_rdy_mod & ~ic_crit_wd_rdy_d1 & ~exu_flush_final & ~ic_write_stall;
   assign my_bp_kill_next_f2 = ifu_bp_kill_next_f2 & ifc_fetch_req_f2_raw;
   assign missff_en = exu_flush_final | (~ic_hit_f2 & fetch_req_f2) | fetch_crit_word_d1 | my_bp_kill_next_f2 | (fetch_req_f2 & ~fetch_req_f1_won & ~fetch_crit_word_d2);
   assign miss_sel_flush = exu_flush_final & (((wfm | idle) & ~fetch_crit_word_d1)  | dma_stall | ic_write_stall | lost_arb);
   assign miss_sel_f2 = ~exu_flush_final & ~ic_hit_f2 & fetch_req_f2;
   assign miss_sel_f1 = ~exu_flush_final & ~miss_sel_f2 & ~fetch_req_f1_won & fetch_req_f2 & ~fetch_crit_word_d2 & ~my_bp_kill_next_f2;
   assign miss_sel_bf = ~miss_sel_f2 & ~miss_sel_f1 & ~miss_sel_flush;

   assign miss_addr_ns[31:1] = ( ({31{miss_sel_flush}} & exu_flush_path_final[31:1]) |
                                 ({31{miss_sel_f2}} & ifc_fetch_addr_f2[31:1]) |
                                 ({31{miss_sel_f1}} & fetch_addr_f1[31:1]) |
                                 ({31{miss_sel_bf}} & fetch_addr_bf[31:1]));



   rvdffe #(31) faddmiss_ff (.*, .en(missff_en), .din(miss_addr_ns[31:1]), .dout(miss_addr[31:1]));


   // Fetch address mux
   // - flush
   // - Miss *or* flush during WFM (icache miss buffer is blocking)
   // - Sequential

   assign sel_last_addr_bf = ~miss_sel_flush & ~fetch_req_f1_won & fetch_req_f2 & ~my_bp_kill_next_f2;
   assign sel_miss_addr_bf = ~miss_sel_flush & ~my_bp_kill_next_f2 & ~fetch_req_f1_won & ~fetch_req_f2;
   assign sel_btb_addr_bf  = ~miss_sel_flush & my_bp_kill_next_f2;
   assign sel_next_addr_bf = ~miss_sel_flush & fetch_req_f1_won;


   assign fetch_addr_bf[31:1] = ( ({31{miss_sel_flush}} &  exu_flush_path_final[31:1]) | // FLUSH path
                                   ({31{sel_miss_addr_bf}} & miss_addr[31:1]) | // MISS path
                                   ({31{sel_btb_addr_bf}} & {ifu_bp_btb_target_f2[31:1]})| // BTB target
                                   ({31{sel_last_addr_bf}} & {fetch_addr_f1[31:1]})| // Last cycle
                                   ({31{sel_next_addr_bf}} & {fetch_addr_next[31:3],fetch_addr_next_2_1[2:1]})); // SEQ path

   assign fetch_addr_next[31:3] = fetch_addr_f1[31:3] + 29'b1;

   assign line_wrap = (fetch_addr_next[pt.ICACHE_TAG_INDEX_LO] ^ fetch_addr_f1[pt.ICACHE_TAG_INDEX_LO]);

   assign fetch_addr_next_2_1[2:1] = line_wrap ? 2'b0 : fetch_addr_f1[2:1];

   assign ifc_fetch_req_bf = (fetch_ns | fetch_crit_word) ;
   assign fetch_bf_en = (fetch_ns | fetch_crit_word);

   assign miss_f2 = fetch_req_f2 & ~ic_hit_f2;

   assign mb_empty_mod = (ifu_ic_mb_empty | exu_flush_final) & ~dma_stall & ~miss_f2 & ~miss_a;

   // Halt flushes and takes us to IDLE
   assign goto_idle = exu_flush_final & dec_tlu_flush_noredir_wb;
   // If we're in IDLE, and we get a flush, goto FETCH
   assign leave_idle = exu_flush_final & ~dec_tlu_flush_noredir_wb & idle;

//.i 7
//.o 2
//.ilb state[1] state[0] reset_delayed miss_f2 mb_empty_mod  goto_idle leave_idle
//.ob next_state[1] next_state[0]
//.type fr
//
//# fetch 01, stall 10, wfm 11, idle 00
//-- 1---- 01
//-- 0--1- 00
//00 0--00 00
//00 0--01 01
//
//01 01-0- 11
//01 00-0- 01
//
//11 0-10- 01
//11 0-00- 11

   assign next_state[1] = (~state[1] & state[0] & ~reset_delayed & miss_f2 & ~goto_idle) |
                          (state[1] & ~reset_delayed & ~mb_empty_mod & ~goto_idle);

   assign next_state[0] = (~goto_idle & leave_idle) | (state[0] & ~goto_idle) |
                          (reset_delayed);

   assign flush_fb = exu_flush_final;

   // model fb write logic to mass balance the fetch buffers
   assign fb_right = (~ifu_fb_consume1 & ~ifu_fb_consume2 & miss_f2) |  // F2 cache miss, repair mass balance
                     ( ifu_fb_consume1 & ~ifu_fb_consume2 & ~fetch_req_f1_won & ~miss_f2) | // Consumed and no new fetch
                      (ifu_fb_consume2 &  fetch_req_f1_won & ~miss_f2); // Consumed 2 and new fetch


   assign fb_right2 = (ifu_fb_consume1 & ~ifu_fb_consume2 & miss_f2) | // consume 1 and miss 1
                      (ifu_fb_consume2 & ~fetch_req_f1_won); // Consumed 2 and no new fetch

   assign fb_right3 = (ifu_fb_consume2 & miss_f2); // consume 2 and miss

   assign fb_left = fetch_req_f1_won & ~(ifu_fb_consume1 | ifu_fb_consume2) & ~miss_f2;

   assign fb_write_ns[4:0] = ( ({5{(flush_fb & ~fetch_req_f1_won)}} & 5'b00001) |
                               ({5{(flush_fb & fetch_req_f1_won)}} & 5'b00010) |
                               ({5{~flush_fb & fb_right }} & {1'b0, fb_write_f1[4:1]}) |
                               ({5{~flush_fb & fb_right2}} & {2'b0, fb_write_f1[4:2]}) |
                               ({5{~flush_fb & fb_right3}} & {3'b0, fb_write_f1[4:3]}  ) |
                               ({5{~flush_fb & fb_left  }} & {fb_write_f1[3:0], 1'b0}) |
                               ({5{~flush_fb & ~fb_right & ~fb_right2 & ~fb_left & ~fb_right3}}  & fb_write_f1[4:0]));


   assign fb_full_f1_ns = fb_write_ns[4];

   assign idle     = state      == IDLE  ;
   assign wfm      = state      == WFM   ;
   assign fetch_ns = next_state == FETCH ;

   rvdff #(2) fsm_ff (.*, .clk(active_clk), .din({next_state[1:0]}), .dout({state[1:0]}));
   rvdff #(6) fbwrite_ff (.*, .clk(active_clk), .din({fb_full_f1_ns, fb_write_ns[4:0]}), .dout({fb_full_f1, fb_write_f1[4:0]}));

if(pt.NUM_THREADS > 1) begin : ignoreconsume
   assign pmu_fetch_stall = wfm |
                                (fetch_req_f1_raw &
                                ( (fb_full_f1 & ~(exu_flush_final)) |
                                  dma_stall));
   // BTB hit kills this fetch
   assign fetch_req_f1 = ( fetch_req_f1_raw &
                               ~my_bp_kill_next_f2 &
                               ~(fb_full_f1 & ~(exu_flush_final)) &
                               ~dma_stall &
                               ~ic_write_stall &
                               ~dec_tlu_flush_noredir_wb );

end // block: ignoreconsume
else begin
   assign pmu_fetch_stall = wfm |
                                (fetch_req_f1_raw &
                                ( (fb_full_f1 & ~(ifu_fb_consume2 | ifu_fb_consume1 | exu_flush_final)) |
                                  dma_stall));
   // BTB hit kills this fetch
   assign fetch_req_f1 = ( fetch_req_f1_raw &
                               ~my_bp_kill_next_f2 &
                               ~(fb_full_f1 & ~(ifu_fb_consume2 | ifu_fb_consume1 | exu_flush_final)) &
                               ~dma_stall &
                               ~ic_write_stall &
                               ~dec_tlu_flush_noredir_wb );
end
   assign ready = fetch_req_f1;
   assign fetch_req_f1_won = fetch_req_f1 & ~(tid ^ ifc_select_tid_f1);
   assign lost_arb = tid ^ ifc_select_tid_f1;
   // kill F2 request if we flush or if the prior fetch missed the cache/mem
   assign fetch_req_f2_ns = fetch_req_f1_won & ~miss_f2;

   rvdff #(2) req_ff (.*, .clk(active_clk), .din({ifc_fetch_req_bf, fetch_req_f2_ns}), .dout({fetch_req_f1_raw, ifc_fetch_req_f2_raw}));

   assign fetch_req_f2 = ifc_fetch_req_f2_raw & ~exu_flush_final;

   rvdffe #(31) faddrf1_ff  (.*, .en(fetch_bf_en), .din(fetch_addr_bf[31:1]), .dout(ifc_fetch_addr_f1_raw[31:1]));
   rvdff #(31) faddrf2_ff (.*,  .clk(ifc_f2_clk), .din(fetch_addr_f1[31:1]), .dout(ifc_fetch_addr_f2[31:1]));

   assign fetch_addr_f1[31:1] = ( ({31{exu_flush_final}} & exu_flush_path_final[31:1]) |
                                      ({31{~exu_flush_final}} & ifc_fetch_addr_f1_raw[31:1]));

   rvdff #(3) iccrit_ff (.*, .clk(active_clk), .din({ic_crit_wd_rdy_mod, fetch_crit_word,    fetch_crit_word_d1}),
                                              .dout({ic_crit_wd_rdy_d1,  fetch_crit_word_d1, fetch_crit_word_d2}));

if (pt.ICCM_ENABLE == 1)
 begin
   rvrangecheck #( .CCM_SADR    (pt.ICCM_SADR),
                   .CCM_SIZE    (pt.ICCM_SIZE) ) iccm_rangecheck (
                                                                     .addr     ({fetch_addr_f1[31:1],1'b0}) ,
                                                                     .in_range (iccm_acc_in_range_f1) ,
                                                                     .in_region(iccm_acc_in_region_f1)
                                                                     );

   assign iccm_access_f1 = iccm_acc_in_range_f1 ;


   assign region_acc_fault_f1 = ~iccm_acc_in_range_f1 & iccm_acc_in_region_f1 ;

    if(pt.NUM_THREADS > 1) begin
   assign dma_access_ok = ( (~iccm_access_f1 |
                                 (fb_full_f1) |
                                 wfm |
                                 idle ) & ~exu_flush_final) |
                              dma_iccm_stall_any_f;
    end
    else begin
   assign dma_access_ok = ( (~iccm_access_f1 |
                                 (fb_full_f1 & ~(ifu_fb_consume2 | ifu_fb_consume1)) |
                                 wfm |
                                 idle ) & ~exu_flush_final) |
                              dma_iccm_stall_any_f;
       end

 end
else
 begin
   assign iccm_access_f1 = 1'b0 ;
   assign dma_access_ok  = 1'b0 ;
   assign region_acc_fault_f1  = 1'b0 ;
 end


   assign cacheable_select[4:0]    =  {fetch_addr_f1[31:28] , 1'b0 } ;
   assign fetch_uncacheable_f1 =  ~dec_tlu_mrac_ff[cacheable_select]  ; // bit 0 of each region description is the cacheable bit

endmodule // eh2_ifu_ifc_ctl

