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
   input logic active_clk,

   input logic rst_l, // reset enable, from core pin
   input logic scan_mode, // scan

   input logic ic_hit_f2,      // Icache hit
   input logic ic_crit_wd_rdy, // Crit word ready to be forwarded
   input logic ifu_ic_mb_empty, // Miss buffer empty

   input logic ifu_fb_consume1,  // Aligner consumed 1 fetch buffer
   input logic ifu_fb_consume2,  // Aligner consumed 2 fetch buffers

   input logic dec_tlu_flush_noredir_wb, // Don't fetch on flush
   input logic dec_tlu_flush_mp_wb,
   input logic dec_tlu_flush_lower_wb, // Flush
   input logic exu_flush_final, // FLush
   input logic [31:1] exu_flush_path_final, // Flush path
   input logic [31:1] dec_tlu_flush_path_wb, // Flush path

   input logic exu_flush_final_early, // FLush
   input logic [31:1] exu_flush_path_final_early, // Flush path

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
   output logic [31:1] fetch_addr_bf, // fetch addr F1
   output logic [31:1] fetch_addr_f2,

   output [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] fetch_btb_rd_addr_f1, // btb read hash
   output [pt.BTB_ADDR_HI:pt.BTB_ADDR_LO] fetch_btb_rd_addr_p1_f1, // btb read hash

   output logic  fetch_req_bf,
   output logic  fetch_req_f1,  // fetch request valid F1
   output logic  fetch_req_f1_raw, // for clock-gating in mem_ctl
   output logic  fetch_req_f2,  // fetch request valid F2

   output logic  pmu_fetch_stall, // pmu event measuring fetch stall

   output logic  iccm_access_f1, // fetch to ICCM region
   output logic  region_acc_fault_f1, // fetch access fault
   output logic  dma_access_ok, // fetch is not accessing the ICCM, DMA can proceed
   output logic  ready // ready to fetch
   );


   logic [31:1]  miss_addr, ifc_fetch_addr_f1_raw;
   logic [31:3]  fetch_addr_next;
   logic [31:1]  miss_addr_ns;
   logic [4:0]   cacheable_select;
   logic [4:0]   fb_write_f1, fb_write_ns;

   logic         fb_full_f1_ns, fb_full_f1;
   logic         fb_right, fb_right2, fb_right3, fb_left, wfm, fetch_ns, fetch, idle;
   logic         fetch_req_f2_ns, fetch_req_f1_raw_unqual;
   logic         missff_en;
   logic         fetch_crit_word, ic_crit_wd_rdy_f, ic_crit_wd_rdy_d1, fetch_crit_word_d1, fetch_crit_word_d2, my_bp_kill_next_f2;
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

   logic         fetch_req_f1_won;
   logic         iccm_acc_in_range_f1;
   logic         iccm_acc_in_region_f1;
   logic [31:1]  exu_flush_path_final_early_f;

   if (pt.ICCM_ENABLE == 1)
     begin
        logic iccm_acc_in_region_f1;
        logic iccm_acc_in_range_f1;
     end
   logic dma_stall, kill_fetch;

   // FSM assignment
    typedef enum logic [1:0] { IDLE  = 2'b00,
                               FETCH = 2'b01,
                               STALL = 2'b10,
                               WFM   = 2'b11 } state_t ;
   state_t state      ;
   state_t next_state ;

   assign dma_stall = ic_dma_active | dma_iccm_stall_any_f;


   rvdff #(2) ran_ff (.*, .clk(active_clk), .din({dma_iccm_stall_any, miss_f2}), .dout({dma_iccm_stall_any_f, miss_a}));

   // If crit word fetch is blocked, try again
   assign ic_crit_wd_rdy_mod = ic_crit_wd_rdy_f & ~((fetch_crit_word_d2 | ic_write_stall) & ~fetch_req_f2);

   // For Ifills, we fetch the critical word. Needed for perf and for rom bypass
   assign fetch_crit_word = ic_crit_wd_rdy_mod & ~ic_crit_wd_rdy_d1 & ~flush_fb & ~ic_write_stall;
   assign my_bp_kill_next_f2 = ifu_bp_kill_next_f2 & ifc_fetch_req_f2_raw;
   assign missff_en = flush_fb | (~ic_hit_f2 & fetch_req_f2) | fetch_crit_word_d1 | my_bp_kill_next_f2 | (fetch_req_f2 & ~fetch_req_f1_won & ~fetch_crit_word_d2);
   assign miss_sel_f2 = ~flush_fb & ~ic_hit_f2 & fetch_req_f2;
   assign miss_sel_f1 = ~flush_fb & ~miss_sel_f2 & ~fetch_req_f1_won & fetch_req_f2 & ~fetch_crit_word_d2 & ~my_bp_kill_next_f2;
   assign miss_sel_bf = ~miss_sel_f2 & ~miss_sel_f1 & ~miss_sel_flush;

   // pcie too much pressure
   rvdffe #(31) faddmiss_ff (.*, .en(missff_en), .din(miss_addr_ns[31:1]), .dout(miss_addr[31:1]));


   // Fetch address mux
   // - flush
   // - Miss *or* flush during WFM (icache miss buffer is blocking)
   // - Sequential

logic dec_tlu_flush_noredir_wb_f, flush_noredir, ic_crit_wd_rdy_qual, flush_lower_qual;
   logic [31:1] fetch_addr_bf_pre;
if(pt.BTB_USE_SRAM) begin

   // hold the early flush path
   rvdffe #(31) faddmiss_ff (.*, .en(exu_flush_final_early), .din(exu_flush_path_final_early[31:1]), .dout(exu_flush_path_final_early_f[31:1]));
   assign flush_lower_qual = dec_tlu_flush_lower_wb & ~dec_tlu_flush_mp_wb;
   assign flush_fb = exu_flush_final | flush_lower_qual;
   assign sel_last_addr_bf =  (flush_fb & ~fetch_req_f1_won) | (~fetch_req_f1_won & ~my_bp_kill_next_f2 & fetch_req_f2);
   assign sel_miss_addr_bf =  ~(flush_fb & ~fetch_req_f1_won) & ~fetch_req_f1_won & ~my_bp_kill_next_f2 & ~fetch_req_f2;
   assign sel_btb_addr_bf  =  my_bp_kill_next_f2;
   assign sel_next_addr_bf =  fetch_req_f1_won;

   assign miss_sel_flush = flush_fb & (((wfm | idle) & ~fetch_crit_word_d1)  | dma_stall | ic_write_stall | lost_arb);

   assign fetch_addr_bf_pre[31:1] = (({31{ flush_lower_qual}} & dec_tlu_flush_path_wb[31:1]) | // Flush path
                                     ({31{~flush_lower_qual & sel_miss_addr_bf}} & miss_addr[31:1]) | // MISS path
                                     ({31{~flush_lower_qual & sel_btb_addr_bf}} & {ifu_bp_btb_target_f2[31:1]})| // BTB target
                                     ({31{~flush_lower_qual & sel_last_addr_bf}} & {fetch_addr_f1[31:1]})| // Last cycle
                                     ({31{~flush_lower_qual & sel_next_addr_bf}} & {fetch_addr_next[31:3],fetch_addr_next_2_1[2:1]})); // SEQ path

   assign fetch_addr_bf[31:1] = ({31{ exu_flush_final_early}} & exu_flush_path_final_early[31:1]) |
                                ({31{~exu_flush_final_early}} & fetch_addr_bf_pre[31:1]) ;

   assign miss_addr_ns[31:1] = ( ({31{miss_sel_flush}} & (flush_lower_qual ? dec_tlu_flush_path_wb[31:1] : exu_flush_path_final_early_f[31:1])) |
                                 ({31{miss_sel_f2}} & fetch_addr_f2[31:1]) |
                                 ({31{miss_sel_f1}} & fetch_addr_f1[31:1]) |
                                 ({31{miss_sel_bf}} & fetch_addr_bf_pre[31:1]));

   assign fetch_addr_f1[31:1] = ifc_fetch_addr_f1_raw[31:1];

   assign ic_crit_wd_rdy_qual = ic_crit_wd_rdy & ~dec_tlu_flush_noredir_wb;

   rvdff #(5) iccrit_ff (.*, .clk(active_clk), .din({dec_tlu_flush_noredir_wb, ic_crit_wd_rdy_qual, ic_crit_wd_rdy_mod, fetch_crit_word,    fetch_crit_word_d1}),
                                              .dout({dec_tlu_flush_noredir_wb_f, ic_crit_wd_rdy_f, ic_crit_wd_rdy_d1,  fetch_crit_word_d1, fetch_crit_word_d2}));

   assign fetch = state == FETCH ;
   assign fetch_req_bf = (fetch | fetch_crit_word);
   assign fetch_bf_en = (fetch | fetch_crit_word | exu_flush_final_early | exu_flush_final | flush_lower_qual);
   assign flush_noredir = dec_tlu_flush_noredir_wb | dec_tlu_flush_noredir_wb_f;
   assign kill_fetch = flush_lower_qual;

end
else begin // NOT SRAM
   assign flush_fb = exu_flush_final;
   // tlu flush and exu mispredict flush are combined when not using srams for the btb
   assign miss_sel_flush = exu_flush_final & (((wfm | idle) & ~fetch_crit_word_d1)  | dma_stall | ic_write_stall | lost_arb);
   assign sel_last_addr_bf = ~miss_sel_flush & ~fetch_req_f1_won & fetch_req_f2 & ~my_bp_kill_next_f2;
   assign sel_miss_addr_bf = ~miss_sel_flush & ~my_bp_kill_next_f2 & ~fetch_req_f1_won & ~fetch_req_f2;
   assign sel_btb_addr_bf  = ~miss_sel_flush & my_bp_kill_next_f2;
   assign sel_next_addr_bf = ~miss_sel_flush & fetch_req_f1_won;
   assign fetch_addr_bf[31:1] = ( ({31{miss_sel_flush}} &  exu_flush_path_final[31:1]) | // FLUSH path
                                  ({31{sel_miss_addr_bf}} & miss_addr[31:1]) | // MISS path
                                  ({31{sel_btb_addr_bf}} & {ifu_bp_btb_target_f2[31:1]})| // BTB target
                                  ({31{sel_last_addr_bf}} & {fetch_addr_f1[31:1]})| // Last cycle
                                  ({31{sel_next_addr_bf}} & {fetch_addr_next[31:3],fetch_addr_next_2_1[2:1]})); // SEQ path

   assign miss_addr_ns[31:1] = ( ({31{miss_sel_flush}} & exu_flush_path_final[31:1]) |
                                 ({31{miss_sel_f2}} & fetch_addr_f2[31:1]) |
                                 ({31{miss_sel_f1}} & fetch_addr_f1[31:1]) |
                                 ({31{miss_sel_bf}} & fetch_addr_bf[31:1]));

   assign fetch_addr_f1[31:1] = ( ({31{exu_flush_final}} & exu_flush_path_final[31:1]) |
                                  ({31{~exu_flush_final}} & ifc_fetch_addr_f1_raw[31:1]));
   rvdff #(3) iccrit_ff (.*, .clk(active_clk), .din({ic_crit_wd_rdy_mod, fetch_crit_word,    fetch_crit_word_d1}),
                                              .dout({ic_crit_wd_rdy_d1,  fetch_crit_word_d1, fetch_crit_word_d2}));
   assign ic_crit_wd_rdy_f = ic_crit_wd_rdy;
   assign fetch_req_bf = (fetch_ns | fetch_crit_word);
   assign fetch_bf_en = (fetch_ns | fetch_crit_word);
   assign flush_noredir = dec_tlu_flush_noredir_wb;
   assign kill_fetch = '0;


end // else: !if(pt.BTB_USE_SRAM)

   assign fetch_addr_next[31:3] = fetch_addr_f1[31:3] + 29'b1;

   assign line_wrap = (fetch_addr_next[pt.ICACHE_TAG_INDEX_LO] ^ fetch_addr_f1[pt.ICACHE_TAG_INDEX_LO]);
// For verilator.... jb
   assign fetch_addr_next_2_1[2:1] = line_wrap ? 2'b0 : fetch_addr_f1[2:1];


   assign miss_f2 = fetch_req_f2 & ~ic_hit_f2;


   // Halt flushes and takes us to IDLE
   assign goto_idle = flush_fb & dec_tlu_flush_noredir_wb;
   // If we're in IDLE, and we get a flush, goto FETCH
   assign leave_idle = flush_fb & ~dec_tlu_flush_noredir_wb & idle;
   assign mb_empty_mod = (ifu_ic_mb_empty | flush_fb) & ~dma_stall & ~miss_f2 & ~miss_a;


//.i 6
//.o 2
//.ilb state[1] state[0] miss_f2 mb_empty_mod  goto_idle leave_idle
//.ob next_state[1] next_state[0]
//.type fr
//
//# fetch 01, stall 10, wfm 11, idle 00
//-- --1- 00
//00 --00 00
//00 --01 01
//
//01 1-0- 11
//01 0-0- 01
//
//11 -10- 01
//11 -00- 11

   assign next_state[1] = state_t'((~state[1] & state[0] & miss_f2 & ~goto_idle) |
                          (state[1] & ~mb_empty_mod & ~goto_idle));

   assign next_state[0] = state_t'((~goto_idle & leave_idle) | (state[0] & ~goto_idle));


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
                                ( (fb_full_f1 & ~(flush_fb)) |
                                  dma_stall));
   // BTB hit kills this fetch
   assign fetch_req_f1 = ( fetch_req_f1_raw &
                           ~kill_fetch &
                           ~my_bp_kill_next_f2 &
                           ~(fb_full_f1 & ~(flush_fb)) &
                           ~dma_stall &
                           ~ic_write_stall &
                           ~flush_noredir );

end // block: ignoreconsume
else begin
   assign pmu_fetch_stall = wfm |
                                (fetch_req_f1_raw &
                                ( (fb_full_f1 & ~(ifu_fb_consume2 | ifu_fb_consume1 | flush_fb)) |
                                  dma_stall));
   // BTB hit kills this fetch
   assign fetch_req_f1 = ( fetch_req_f1_raw &
                           ~kill_fetch &
                           ~my_bp_kill_next_f2 &
                           ~(fb_full_f1 & ~(ifu_fb_consume2 | ifu_fb_consume1 | flush_fb)) &
                           ~dma_stall &
                           ~ic_write_stall &
                           ~flush_noredir );
end

   if(pt.BTB_USE_SRAM) begin
      assign ready = fetch_req_bf &
                     ~(fb_full_f1 & ~(ifu_fb_consume2 | ifu_fb_consume1 | flush_fb));
   end
   else
     assign ready = fetch_req_f1;

   assign fetch_req_f1_won = fetch_req_f1 & ~(tid ^ ifc_select_tid_f1);
   assign lost_arb = tid ^ ifc_select_tid_f1;
   // kill F2 request if we flush or if the prior fetch missed the cache/mem
   assign fetch_req_f2_ns = fetch_req_f1_won & ~miss_f2 & ~miss_a;

   rvdff #(2) req_ff (.*, .clk(active_clk), .din({fetch_req_bf, fetch_req_f2_ns}), .dout({fetch_req_f1_raw_unqual, ifc_fetch_req_f2_raw}));
   assign fetch_req_f1_raw = fetch_req_f1_raw_unqual & ~miss_a;

   assign fetch_req_f2 = ifc_fetch_req_f2_raw & ~flush_fb;

   // this flop needs a delayed clock *if* using SRAM btb
   rvdffe #(31) faddrf1_ff  (.*, .en(fetch_bf_en), .din(fetch_addr_bf[31:1]), .dout(ifc_fetch_addr_f1_raw[31:1]));

   rvdffpcie #(31) faddrf2_ff (.*,  .en(fetch_req_f1_won), .din(fetch_addr_f1[31:1]), .dout(fetch_addr_f2[31:1]));

   // timing fix attempt
   logic [31:3] fetch_addr_p1_f1;
   eh2_btb_addr_hash #(.pt(pt)) f1hash(.pc(fetch_addr_f1[pt.BTB_INDEX3_HI:pt.BTB_INDEX1_LO]), .hash(fetch_btb_rd_addr_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]));
   assign fetch_addr_p1_f1[31:3] = fetch_addr_f1[31:3] + 29'b1;
   eh2_btb_addr_hash #(.pt(pt)) f1hash_p1(.pc(fetch_addr_p1_f1[pt.BTB_INDEX3_HI:pt.BTB_INDEX1_LO]), .hash(fetch_btb_rd_addr_p1_f1[pt.BTB_ADDR_HI:pt.BTB_ADDR_LO]));



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
                                 idle ) & ~flush_fb) |
                              dma_iccm_stall_any_f;
    end
    else begin
   assign dma_access_ok = ( (~iccm_access_f1 |
                                 (fb_full_f1 & ~(ifu_fb_consume2 | ifu_fb_consume1)) |
                                 wfm |
                                 idle ) & ~flush_fb) |
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

