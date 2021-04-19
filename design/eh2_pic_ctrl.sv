//********************************************************************************
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

//********************************************************************************
// Function: Programmable Interrupt Controller
// Comments:
//********************************************************************************

module eh2_pic_ctrl
import eh2_pkg::*;
#(
`include "eh2_param.vh"
 )
                  (

                     input  logic                   clk,                  // Core clock
                     input  logic                   free_clk,             // free clock
                     input  logic                   rst_l,                // Reset for all flops
                     input  logic                   clk_override,         // Clock over-ride for gating
                     input  logic                   io_clk_override,      // PIC IO  Clock over-ride for gating

                     input  logic [pt.NUM_THREADS-1:0] o_cpu_halt_status, // PMU interface, halted

                     input  logic [pt.PIC_TOTAL_INT_PLUS1-1:0]   extintsrc_req,  // Interrupt requests
                     input  logic [31:0]            picm_rdaddr,          // Address of the register
                     input  logic [31:0]            picm_wraddr,          // Address of the register
                     input  logic [31:0]            picm_wr_data,         // Data to be written to the register
                     input  logic                   picm_wren,            // Write enable to the register
                     input  logic                   picm_rden,            // Read enable for the register
                     input  logic                   picm_rd_thr,          // Reading thread
                     input  logic                   picm_mken,            // Read the Mask for the register

                     input  logic [pt.NUM_THREADS-1:0] [3:0]             dec_tlu_meicurpl,           // Current Priority Level
                     input  logic [pt.NUM_THREADS-1:0] [3:0]             dec_tlu_meipt,              // Current Priority Threshold

                     output logic [pt.NUM_THREADS-1:0]                   mexintpend_out,           // External Inerrupt request to the core
                     output logic [pt.NUM_THREADS-1:0] [7:0]             claimid_out,              // Claim Id of the requested interrupt
                     output logic [pt.NUM_THREADS-1:0] [3:0]             pl_out,                   // Priority level of the requested interrupt
                     output logic [pt.NUM_THREADS-1:0]                   mhwakeup_out,             // Wake-up interrupt request

                     output logic [31:0]            picm_rd_data,         // Read data of the register
                     input  logic                   scan_mode             // scan mode

);

localparam NUM_LEVELS            = $clog2(pt.PIC_TOTAL_INT_PLUS1);
localparam INTPRIORITY_BASE_ADDR = pt.PIC_BASE_ADDR ;
localparam INTPEND_BASE_ADDR     = pt.PIC_BASE_ADDR + 32'h00001000 ;
localparam INTPEND_THR_BASE_ADDR = pt.PIC_BASE_ADDR + 32'h00001800 ;
localparam INTENABLE_BASE_ADDR   = pt.PIC_BASE_ADDR + 32'h00002000 ;
localparam EXT_INTR_PIC_CONFIG   = pt.PIC_BASE_ADDR + 32'h00003000 ;
localparam EXT_INTR_GW_CONFIG    = pt.PIC_BASE_ADDR + 32'h00004000 ;
localparam EXT_INTR_GW_CLEAR     = pt.PIC_BASE_ADDR + 32'h00005000 ;
localparam EXT_INTR_DELG_REG     = pt.PIC_BASE_ADDR + 32'h00006000 ;


localparam INTPEND_SIZE          = (pt.PIC_TOTAL_INT_PLUS1 < 32)  ? 32  :
                                   (pt.PIC_TOTAL_INT_PLUS1 < 64)  ? 64  :
                                   (pt.PIC_TOTAL_INT_PLUS1 < 128) ? 128 :
                                   (pt.PIC_TOTAL_INT_PLUS1 < 256) ? 256 :
                                   (pt.PIC_TOTAL_INT_PLUS1 < 512) ? 512 :  1024 ;

localparam INT_GRPS              =   INTPEND_SIZE / 32 ;
localparam INTPRIORITY_BITS      =  4 ;
localparam ID_BITS               =  8 ;
localparam int GW_CONFIG[pt.PIC_TOTAL_INT_PLUS1-1:0] = '{default:0} ;

localparam INT_ENABLE_GRPS       =   (pt.PIC_TOTAL_INT_PLUS1 - 1)  / 4 ;

logic [pt.PIC_TOTAL_INT_PLUS1-1:0]           intenable_clk_enable ;
logic [INT_ENABLE_GRPS:0]                    intenable_clk_enable_grp ;
logic [INT_ENABLE_GRPS:0]                    gw_clk ;

logic [pt.NUM_THREADS-1:0]                   mexintpend_ff;           // External Inerrupt request to the core
logic [pt.NUM_THREADS-1:0]  [7:0]            claimid_ff;              // Claim Id of the requested interrupt
logic [pt.NUM_THREADS-1:0]  [3:0]            pl_ff;                   // Priority level of the requested interrupt
logic [pt.NUM_THREADS-1:0]                   mhwakeup_ff;             // Wake-up interrupt request

logic [pt.NUM_THREADS-1:0]                   mexintpend;
logic [pt.NUM_THREADS-1:0] [7:0]             claimid;
logic [pt.NUM_THREADS-1:0] [3:0]             pl;
logic [pt.NUM_THREADS-1:0]                   mhwakeup;


logic  addr_intpend_base_match;
logic  addr_intpend_thr_base_match;

logic  raddr_config_pic_match ;
logic  raddr_intenable_base_match;
logic  raddr_delg_base_match;
logic  raddr_intpriority_base_match;
logic  raddr_config_gw_base_match ;

logic  waddr_config_pic_match ;
logic  waddr_intpriority_base_match;
logic  waddr_intenable_base_match;
logic  waddr_delg_base_match;
logic  waddr_config_gw_base_match ;
logic  addr_clear_gw_base_match ;

logic  mexintpend_in;
logic  mhwakeup_in ;
logic  intpend_reg_read ;
logic  intpend_thr_reg_read ;

logic [31:0]                                 picm_rd_data_in, intpend_rd_out, intpend_thr_rd_out;
logic                                        intenable_rd_out ;
logic                                        delg_rd_out ;
logic [INTPRIORITY_BITS-1:0]                 intpriority_rd_out;
logic [1:0]                                  gw_config_rd_out;

logic [INTPRIORITY_BITS-1:0] meipt_inv , meicurpl_inv , meicurpl, meipt;

logic [pt.PIC_TOTAL_INT_PLUS1-1:0] [INTPRIORITY_BITS-1:0] intpriority_reg;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0] [INTPRIORITY_BITS-1:0] intpriority_reg_inv;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        intpriority_reg_we;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        intpriority_reg_re;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        delg_thr_match;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0] [1:0]                  gw_config_reg;

logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        intenable_reg;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        intenable_reg_we;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        intenable_reg_re;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        delg_reg;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        delg_reg_we;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        delg_reg_re;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        gw_config_reg_we;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        gw_config_reg_re;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        gw_clear_reg_we;

logic [INTPEND_SIZE-1:0]                     intpend_reg_extended;
logic [INTPEND_SIZE-1:0]                     thr_mx_intpend_reg_extended;

logic [pt.PIC_TOTAL_INT_PLUS1-1:0] [INTPRIORITY_BITS-1:0] intpend_w_prior_en;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0] [ID_BITS-1:0]          intpend_id;
logic [INTPRIORITY_BITS-1:0]                 maxint;
logic [INTPRIORITY_BITS-1:0]                 selected_int_priority;
logic [INT_GRPS-1:0] [31:0]                  intpend_rd_part_out ;
logic [INT_GRPS-1:0] [31:0]                  intpend_thr_rd_part_out ;

logic                                        curr_int_tid;
logic                                        curr_int_tid_ff;
logic                                        curr_int_tid_final;
logic                                        curr_int_tid_final_in;
logic                                        config_reg;
logic                                        intpriord;
logic                                        config_reg_we ;
logic                                        config_reg_re ;
logic                                        config_reg_in ;
logic                                        prithresh_reg_write , prithresh_reg_read;
logic                                        intpriority_reg_read ;
logic                                        intenable_reg_read   ;
logic                                        gw_config_reg_read   ;
logic                                        picm_wren_ff , picm_rden_ff ;
logic                                        picm_rd_thr_ff;
logic [31:0]                                 picm_raddr_ff;
logic [31:0]                                 picm_waddr_ff;
logic [31:0]                                 picm_wr_data_ff;
logic [3:0]                                  mask;
logic                                        picm_mken_ff;
logic [ID_BITS-1:0]                          claimid_in ;
logic [INTPRIORITY_BITS-1:0]                 pl_in ;
logic [INTPRIORITY_BITS-1:0]                 pl_in_q ;

logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        extintsrc_req_sync;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        extintsrc_req_gw;
logic [pt.PIC_TOTAL_INT_PLUS1-1:0]                        thr_mx_intpend_reg;
logic                                                     picm_bypass_ff;
logic                                                     delg_reg_read;

logic [(pt.PIC_TOTAL_INT_PLUS1/2**(NUM_LEVELS/2)):0] [INTPRIORITY_BITS-1:0] l2_intpend_w_prior_en_ff;
logic [(pt.PIC_TOTAL_INT_PLUS1/2**(NUM_LEVELS/2)):0] [ID_BITS-1:0]          l2_intpend_id_ff;
logic [NUM_LEVELS:NUM_LEVELS/2] [(pt.PIC_TOTAL_INT_PLUS1/2**(NUM_LEVELS/2))+1:0] [INTPRIORITY_BITS-1:0] levelx_intpend_w_prior_en;
logic [NUM_LEVELS:NUM_LEVELS/2] [(pt.PIC_TOTAL_INT_PLUS1/2**(NUM_LEVELS/2))+1:0] [ID_BITS-1:0]          levelx_intpend_id;

// clkens
   logic                                     pic_raddr_c1_clken;
   logic                                     pic_waddr_c1_clken;
   logic                                     pic_data_c1_clken;
   logic                                     pic_pri_c1_clken;
   logic                                     pic_int_c1_clken;
   logic                                     pic_del_c1_clken;
   logic                                     gw_config_c1_clken;

// clocks
   logic                                     pic_raddr_c1_clk;
   logic                                     pic_data_c1_clk;
   logic                                     pic_pri_c1_clk;
   logic                                     pic_int_c1_clk;
   logic                                     pic_del_c1_clk;
   logic                                     gw_config_c1_clk;
   logic                                     nxt_thr;




// ---- Clock gating section ------
// c1 clock enables
   assign pic_raddr_c1_clken  = picm_mken | picm_rden | clk_override;
   assign pic_data_c1_clken   = picm_wren | clk_override;
   assign pic_pri_c1_clken    = (waddr_intpriority_base_match & picm_wren_ff)  | (raddr_intpriority_base_match & picm_rden_ff) | clk_override;
   assign pic_int_c1_clken    = (waddr_intenable_base_match   & picm_wren_ff)  | (raddr_intenable_base_match   & picm_rden_ff) | clk_override;
   assign gw_config_c1_clken  = (waddr_config_gw_base_match   & picm_wren_ff)  | (raddr_config_gw_base_match   & picm_rden_ff) | clk_override;

   // C1 - 1 clock pulse for data
   rvoclkhdr pic_addr_c1_cgc   ( .en(pic_raddr_c1_clken),  .l1clk(pic_raddr_c1_clk), .* );
   rvoclkhdr pic_data_c1_cgc   ( .en(pic_data_c1_clken),   .l1clk(pic_data_c1_clk), .* );
   rvoclkhdr pic_pri_c1_cgc    ( .en(pic_pri_c1_clken),    .l1clk(pic_pri_c1_clk),  .* );
   rvoclkhdr pic_int_c1_cgc    ( .en(pic_int_c1_clken),    .l1clk(pic_int_c1_clk),  .* );
   rvoclkhdr gw_config_c1_cgc  ( .en(gw_config_c1_clken),  .l1clk(gw_config_c1_clk),  .* );

// ------ end clock gating section ------------------------

assign raddr_intenable_base_match   = (picm_raddr_ff[31:NUM_LEVELS+2] == INTENABLE_BASE_ADDR[31:NUM_LEVELS+2]) ;
assign raddr_intpriority_base_match = (picm_raddr_ff[31:NUM_LEVELS+2] == INTPRIORITY_BASE_ADDR[31:NUM_LEVELS+2]) ;
assign raddr_config_gw_base_match   = (picm_raddr_ff[31:NUM_LEVELS+2] == EXT_INTR_GW_CONFIG[31:NUM_LEVELS+2]) ;
assign raddr_config_pic_match       = (picm_raddr_ff[31:0]            == EXT_INTR_PIC_CONFIG[31:0]) ;

assign addr_intpend_base_match      = (picm_raddr_ff[31:6]            == INTPEND_BASE_ADDR[31:6]) ;

assign waddr_config_pic_match       = (picm_waddr_ff[31:0]            == EXT_INTR_PIC_CONFIG[31:0]) ;
assign addr_clear_gw_base_match     = (picm_waddr_ff[31:NUM_LEVELS+2] == EXT_INTR_GW_CLEAR[31:NUM_LEVELS+2]) ;
assign waddr_intpriority_base_match = (picm_waddr_ff[31:NUM_LEVELS+2] == INTPRIORITY_BASE_ADDR[31:NUM_LEVELS+2]) ;
assign waddr_intenable_base_match   = (picm_waddr_ff[31:NUM_LEVELS+2] == INTENABLE_BASE_ADDR[31:NUM_LEVELS+2]) ;
assign waddr_config_gw_base_match   = (picm_waddr_ff[31:NUM_LEVELS+2] == EXT_INTR_GW_CONFIG[31:NUM_LEVELS+2]) ;

if (pt.NUM_THREADS > 1 ) begin:  gt_1_thr
   assign pic_del_c1_clken    = (waddr_delg_base_match        & picm_wren_ff)  | (raddr_delg_base_match        & picm_rden_ff) | clk_override;
   rvoclkhdr pic_del_c1_cgc    ( .en(pic_del_c1_clken),    .l1clk(pic_del_c1_clk),  .* );
   assign raddr_delg_base_match        = (picm_raddr_ff[31:NUM_LEVELS+2] == EXT_INTR_DELG_REG[31:NUM_LEVELS+2]) ;
   assign waddr_delg_base_match        = (picm_waddr_ff[31:NUM_LEVELS+2] == EXT_INTR_DELG_REG[31:NUM_LEVELS+2]) ;
   assign addr_intpend_thr_base_match  = (picm_raddr_ff[31:6]            == INTPEND_THR_BASE_ADDR[31:6]) ;
end else begin: one_t
   assign raddr_delg_base_match = 1'b0 ;
   assign waddr_delg_base_match = 1'b0 ;
   assign pic_del_c1_clk = 1'b0  ;
   assign addr_intpend_thr_base_match  = 1'b0;
end

   assign picm_bypass_ff = picm_rden_ff & picm_wren_ff & ( picm_raddr_ff[31:0] == picm_waddr_ff[31:0] );    // pic writes and reads to same address together


rvdff #(32) picm_radd_flop  (.*, .din (picm_rdaddr),        .dout(picm_raddr_ff),         .clk(pic_raddr_c1_clk));
rvdff #(32) picm_wadd_flop  (.*, .din (picm_wraddr),        .dout(picm_waddr_ff),         .clk(pic_data_c1_clk));
rvdff  #(1) picm_wre_flop   (.*, .din (picm_wren),          .dout(picm_wren_ff),          .clk(free_clk));
rvdff  #(1) picm_rde_flop   (.*, .din (picm_rden),          .dout(picm_rden_ff),          .clk(free_clk));
rvdff  #(1) picm_rdt_flop   (.*, .din (picm_rd_thr),        .dout(picm_rd_thr_ff),        .clk(free_clk));
rvdff  #(1) picm_mke_flop   (.*, .din (picm_mken),          .dout(picm_mken_ff),          .clk(free_clk));
rvdff #(32) picm_dat_flop   (.*, .din (picm_wr_data[31:0]), .dout(picm_wr_data_ff[31:0]), .clk(pic_data_c1_clk));



   if (pt.NUM_THREADS==1)
     assign curr_int_tid = '0;
   else begin

      logic ready_pulse;
      logic [2:0] ready_cnt_in, ready_cnt;
      logic [1:0] ready;
      logic [1:0] active_thread;
      logic       active1;
      logic       active2;
      logic       favor;
      logic       tid;


      assign ready_pulse = ready_cnt[2:0] == 3'b111;

      assign ready_cnt_in[2:0] = (ready_pulse) ? '0 : ready_cnt[2:0] + 3'b1;

      rvdff #(3) ready_cntff (.*, .din(ready_cnt_in[2:0]), .dout(ready_cnt[2:0]), .clk(free_clk));


      assign active_thread[1:0] = (~o_cpu_halt_status[1:0] | {2{io_clk_override}});
      assign active1            = ^active_thread[1:0] ;
      assign active2            = &active_thread[1:0] ;



      assign ready[1:0] = (active2) ? 2'b11 :
                          (active1 & ~ready_pulse) ?  active_thread[1:0] :
                          (active1 &  ready_pulse) ? ~active_thread[1:0] :
                          {2{ready_pulse}};


      rvarbiter2_pic pic_arbiter (.*,
                                  .clk(free_clk),
                                  .shift(1'b1),
                                  .tid(tid),
                                  .favor(favor)
                                  );

      assign curr_int_tid = (|ready[1:0]) ? tid : favor;

   end


   rvdff  #(1) curr_thr_ff   (.*, .din (curr_int_tid),     .dout(curr_int_tid_ff),          .clk(free_clk));

if (pt.PIC_2CYCLE == 1) begin : pic2cyle
   assign curr_int_tid_final_in = curr_int_tid_ff ;
   rvdff  #(1) curr_thr_ff2  (.*, .din (curr_int_tid_ff),  .dout(curr_int_tid_final),          .clk(free_clk));
end else begin: not_pic2cycle
   assign curr_int_tid_final_in = curr_int_tid ;
   assign curr_int_tid_final = curr_int_tid_ff ;
end




genvar i ;
genvar p ;
for (p=0; p<=INT_ENABLE_GRPS ; p++) begin  : IO_CLK_GRP
wire grp_clk, grp_clken;

    assign grp_clken = |intenable_clk_enable[(p==INT_ENABLE_GRPS?pt.PIC_TOTAL_INT_PLUS1-1:p*4+3) : p*4] | io_clk_override;

  `ifndef RV_FPGA_OPTIMIZE
    rvclkhdr intenable_c1_cgc( .en(grp_clken),  .l1clk(grp_clk), .* );
  `else
    assign gw_clk[p] = 1'b0 ;
  `endif


    for(genvar i= (p==0 ? 1: 0); i< (p==INT_ENABLE_GRPS ? pt.PIC_TOTAL_INT_PLUS1-p*4 :4); i++) begin : GW
        eh2_configurable_gw gw_inst(
             .*,
            .gw_clk(grp_clk),
            .rawclk(clk),
            .clken (grp_clken),
            .extintsrc_req(extintsrc_req[i+p*4]) ,
            .meigwctrl_polarity(gw_config_reg[i+p*4][0]) ,
            .meigwctrl_type(gw_config_reg[i+p*4][1]) ,
            .meigwclr(gw_clear_reg_we[i+p*4]) ,
            .extintsrc_req_config(extintsrc_req_gw[i+p*4])
        );
    end
end

for (i=0; i<pt.PIC_TOTAL_INT_PLUS1 ; i++) begin  : SETREG

 if (i > 0 ) begin : NON_ZERO_INT
     assign intpriority_reg_we[i] =  waddr_intpriority_base_match & (picm_waddr_ff[NUM_LEVELS+1:2] == i) & picm_wren_ff;
     assign intpriority_reg_re[i] =  raddr_intpriority_base_match & (picm_raddr_ff[NUM_LEVELS+1:2] == i) & picm_rden_ff;

     assign intenable_reg_we[i]   =  waddr_intenable_base_match   & (picm_waddr_ff[NUM_LEVELS+1:2] == i) & picm_wren_ff;
     assign intenable_reg_re[i]   =  raddr_intenable_base_match   & (picm_raddr_ff[NUM_LEVELS+1:2] == i) & picm_rden_ff;

     if (pt.NUM_THREADS > 1 ) begin:   gt_1_thr
          assign delg_reg_we[i]   =  waddr_delg_base_match   & (picm_waddr_ff[NUM_LEVELS+1:2] == i) & picm_wren_ff;
          assign delg_reg_re[i]   =  raddr_delg_base_match   & (picm_raddr_ff[NUM_LEVELS+1:2] == i) & picm_rden_ff;
          rvdffs #(1)                 delg_ff        (.*, .en( delg_reg_we[i]),        .din (picm_wr_data_ff[0]),                    .dout(delg_reg[i]),        .clk(pic_del_c1_clk));
     end else begin: one_t
          assign delg_reg_re[i] = 1'b0 ;
          assign delg_reg_we[i] = 1'b0 ;
          assign delg_reg[i]    = 1'b0;
     end


     assign gw_config_reg_we[i]   =  waddr_config_gw_base_match   & (picm_waddr_ff[NUM_LEVELS+1:2] == i) & picm_wren_ff;
     assign gw_config_reg_re[i]   =  raddr_config_gw_base_match   & (picm_raddr_ff[NUM_LEVELS+1:2] == i) & picm_rden_ff;

     assign gw_clear_reg_we[i]    =  addr_clear_gw_base_match     & (picm_waddr_ff[NUM_LEVELS+1:2] == i) & picm_wren_ff ;

     rvdffs #(INTPRIORITY_BITS) intpriority_ff  (.*, .en( intpriority_reg_we[i]), .din (picm_wr_data_ff[INTPRIORITY_BITS-1:0]), .dout(intpriority_reg[i]), .clk(pic_pri_c1_clk));
     rvdffs #(1)                 intenable_ff   (.*, .en( intenable_reg_we[i]),   .din (picm_wr_data_ff[0]),                    .dout(intenable_reg[i]),   .clk(pic_int_c1_clk));
     rvdffs #(2)                 gw_config_ff   (.*, .en( gw_config_reg_we[i]),   .din (picm_wr_data_ff[1:0]),                  .dout(gw_config_reg[i]),   .clk(gw_config_c1_clk));

     assign intenable_clk_enable[i]  =  gw_config_reg[i][1] | intenable_reg_we[i] | intenable_reg[i] | gw_clear_reg_we[i] ;


 end else begin : INT_ZERO
     assign intpriority_reg_we[i] =  1'b0 ;
     assign intpriority_reg_re[i] =  1'b0 ;
     assign intenable_reg_we[i]   =  1'b0 ;
     assign intenable_reg_re[i]   =  1'b0 ;
     assign delg_reg_re[i]        =  1'b0 ;
     assign delg_reg_we[i]        =  1'b0 ;
     assign gw_config_reg_we[i]   =  1'b0 ;
     assign gw_config_reg_re[i]   =  1'b0 ;
     assign gw_clear_reg_we[i]    =  1'b0 ;

     assign gw_config_reg[i]    = '0 ;

     assign intpriority_reg[i] = {INTPRIORITY_BITS{1'b0}} ;
     assign intenable_reg[i]   = 1'b0 ;
     assign delg_reg[i]        = 1'b0 ;
     assign extintsrc_req_gw[i] = 1'b0 ;
     assign extintsrc_req_sync[i]    = 1'b0 ;
     assign intenable_clk_enable[i] = 1'b0;
 end


    assign intpriority_reg_inv[i] =  intpriord ? ~intpriority_reg[i] : intpriority_reg[i] ;
    assign delg_thr_match[i]      =  (delg_reg[i] &  curr_int_tid) |   (~delg_reg[i] & ~curr_int_tid) ;

    assign intpend_w_prior_en[i]  =  {INTPRIORITY_BITS{(extintsrc_req_gw[i] & intenable_reg[i] & delg_thr_match[i])}} & intpriority_reg_inv[i] ;
    assign intpend_id[i]          =  i ;
end


        assign pl_in[INTPRIORITY_BITS-1:0]                  =      selected_int_priority[INTPRIORITY_BITS-1:0] ;


 genvar l, m , j, k;

if (pt.PIC_2CYCLE == 1) begin : genblock
        logic [NUM_LEVELS/2:0] [pt.PIC_TOTAL_INT_PLUS1+2:0] [INTPRIORITY_BITS-1:0] level_intpend_w_prior_en;
        logic [NUM_LEVELS/2:0] [pt.PIC_TOTAL_INT_PLUS1+2:0] [ID_BITS-1:0]          level_intpend_id;

        assign level_intpend_w_prior_en[0][pt.PIC_TOTAL_INT_PLUS1+2:0] = {4'b0,4'b0,4'b0,intpend_w_prior_en[pt.PIC_TOTAL_INT_PLUS1-1:0]} ;
        assign level_intpend_id[0][pt.PIC_TOTAL_INT_PLUS1+2:0]         = {8'b0,8'b0,8'b0,intpend_id[pt.PIC_TOTAL_INT_PLUS1-1:0]} ;


        assign levelx_intpend_w_prior_en[NUM_LEVELS/2][(pt.PIC_TOTAL_INT_PLUS1/2**(NUM_LEVELS/2))+1:0] = {{1*INTPRIORITY_BITS{1'b0}},l2_intpend_w_prior_en_ff[(pt.PIC_TOTAL_INT_PLUS1/2**(NUM_LEVELS/2)):0]} ;
        assign levelx_intpend_id[NUM_LEVELS/2][(pt.PIC_TOTAL_INT_PLUS1/2**(NUM_LEVELS/2))+1:0]         = {{1*ID_BITS{1'b1}},l2_intpend_id_ff[(pt.PIC_TOTAL_INT_PLUS1/2**(NUM_LEVELS/2)):0]} ;
///  Do the prioritization of the interrupts here  ////////////
 for (l=0; l<NUM_LEVELS/2 ; l++) begin : TOP_LEVEL
    for (m=0; m<=(pt.PIC_TOTAL_INT_PLUS1)/(2**(l+1)) ; m++) begin : COMPARE
       if ( m == (pt.PIC_TOTAL_INT_PLUS1)/(2**(l+1))) begin
            assign level_intpend_w_prior_en[l+1][m+1] = '0 ;
            assign level_intpend_id[l+1][m+1]         = '0 ;
       end
       eh2_cmp_and_mux  #(.ID_BITS(ID_BITS),
                      .INTPRIORITY_BITS(INTPRIORITY_BITS)) cmp_l1 (
                      .a_id(level_intpend_id[l][2*m]),
                      .a_priority(level_intpend_w_prior_en[l][2*m]),
                      .b_id(level_intpend_id[l][2*m+1]),
                      .b_priority(level_intpend_w_prior_en[l][2*m+1]),
                      .out_id(level_intpend_id[l+1][m]),
                      .out_priority(level_intpend_w_prior_en[l+1][m])) ;

    end
 end

        for (i=0; i<=pt.PIC_TOTAL_INT_PLUS1/2**(NUM_LEVELS/2) ; i++) begin : MIDDLE_FLOPS

           rvdffie #(INTPRIORITY_BITS+ID_BITS) level2_intpend_reg  (.*,
                                                                    .din ({level_intpend_w_prior_en[NUM_LEVELS/2][i], level_intpend_id[NUM_LEVELS/2][i]}),
                                                                    .dout({l2_intpend_w_prior_en_ff[i],               l2_intpend_id_ff[i]})
                                                                    );
        end

 for (j=NUM_LEVELS/2; j<NUM_LEVELS ; j++) begin : BOT_LEVELS
    for (k=0; k<=(pt.PIC_TOTAL_INT_PLUS1)/(2**(j+1)) ; k++) begin : COMPARE
       if ( k == (pt.PIC_TOTAL_INT_PLUS1)/(2**(j+1))) begin
            assign levelx_intpend_w_prior_en[j+1][k+1] = '0 ;
            assign levelx_intpend_id[j+1][k+1]         = '0 ;
       end
            eh2_cmp_and_mux  #(.ID_BITS(ID_BITS),
                        .INTPRIORITY_BITS(INTPRIORITY_BITS))
                 cmp_l1 (
                        .a_id(levelx_intpend_id[j][2*k]),
                        .a_priority(levelx_intpend_w_prior_en[j][2*k]),
                        .b_id(levelx_intpend_id[j][2*k+1]),
                        .b_priority(levelx_intpend_w_prior_en[j][2*k+1]),
                        .out_id(levelx_intpend_id[j+1][k]),
                        .out_priority(levelx_intpend_w_prior_en[j+1][k])) ;
    end
  end

        assign claimid_in[ID_BITS-1:0]                      =      levelx_intpend_id[NUM_LEVELS][0] ;   // This is the last level output
        assign selected_int_priority[INTPRIORITY_BITS-1:0]  =      levelx_intpend_w_prior_en[NUM_LEVELS][0] ;
end
else begin : genblock

        logic [NUM_LEVELS:0] [pt.PIC_TOTAL_INT_PLUS1+1:0] [INTPRIORITY_BITS-1:0] level_intpend_w_prior_en;
        logic [NUM_LEVELS:0] [pt.PIC_TOTAL_INT_PLUS1+1:0] [ID_BITS-1:0]          level_intpend_id;

        assign level_intpend_w_prior_en[0][pt.PIC_TOTAL_INT_PLUS1+1:0] = {{2*INTPRIORITY_BITS{1'b0}},intpend_w_prior_en[pt.PIC_TOTAL_INT_PLUS1-1:0]} ;
        assign level_intpend_id[0][pt.PIC_TOTAL_INT_PLUS1+1:0] = {{2*ID_BITS{1'b1}},intpend_id[pt.PIC_TOTAL_INT_PLUS1-1:0]} ;

///  Do the prioritization of the interrupts here  ////////////
// genvar l, m , j, k;  already declared outside ifdef
 for (l=0; l<NUM_LEVELS ; l++) begin : LEVEL
    for (m=0; m<=(pt.PIC_TOTAL_INT_PLUS1)/(2**(l+1)) ; m++) begin : COMPARE
       if ( m == (pt.PIC_TOTAL_INT_PLUS1)/(2**(l+1))) begin
            assign level_intpend_w_prior_en[l+1][m+1] = '0 ;
            assign level_intpend_id[l+1][m+1]         = '0 ;
       end
       eh2_cmp_and_mux  #(.ID_BITS(ID_BITS),
                      .INTPRIORITY_BITS(INTPRIORITY_BITS)) cmp_l1 (
                      .a_id(level_intpend_id[l][2*m]),
                      .a_priority(level_intpend_w_prior_en[l][2*m]),
                      .b_id(level_intpend_id[l][2*m+1]),
                      .b_priority(level_intpend_w_prior_en[l][2*m+1]),
                      .out_id(level_intpend_id[l+1][m]),
                      .out_priority(level_intpend_w_prior_en[l+1][m])) ;

    end
 end
        assign claimid_in[ID_BITS-1:0]                      =      level_intpend_id[NUM_LEVELS][0] ;   // This is the last level output
        assign selected_int_priority[INTPRIORITY_BITS-1:0]  =      level_intpend_w_prior_en[NUM_LEVELS][0] ;

end



///////////////////////////////////////////////////////////////////////
// Config Reg`
///////////////////////////////////////////////////////////////////////
assign config_reg_we               =  waddr_config_pic_match & picm_wren_ff;
assign config_reg_re               =  raddr_config_pic_match & picm_rden_ff;

assign config_reg_in  =  picm_wr_data_ff[0] ;   //
rvdffs #(1) config_reg_ff  (.*, .clk(free_clk), .en(config_reg_we), .din (config_reg_in), .dout(config_reg));

assign intpriord  = config_reg ;


//////////////////////////////////////////////////////////////////////////
// Send the interrupt to the core if it is above the thresh-hold
//////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
/// ClaimId  Reg and Corresponding PL
///////////////////////////////////////////////////////////
assign pl_in_q[INTPRIORITY_BITS-1:0] = intpriord ? ~pl_in : pl_in ;

if (pt.NUM_THREADS > 1 ) begin:   more_than_1_thr

//  Per thread hold flops

  rvdffe  #(.WIDTH(1),.OVERRIDE(1))                mexintpend_fl_thr0  (.*, .din (mexintpend_in ), .dout(mexintpend[0]), .en(~curr_int_tid_final_in));
  rvdffe  #(.WIDTH(1),.OVERRIDE(1))                mexintpend_fl_thr1  (.*, .din (mexintpend_in ), .dout(mexintpend[1]), .en( curr_int_tid_final_in));

  rvdffe  #(.WIDTH(INTPRIORITY_BITS),.OVERRIDE(1)) pl_fl_thr0      (.*, .din (pl_in_q[INTPRIORITY_BITS-1:0]), .dout(pl[0][INTPRIORITY_BITS-1:0]), .en(~curr_int_tid_final_in));
  rvdffe  #(.WIDTH(INTPRIORITY_BITS),.OVERRIDE(1)) pl_fl_thr1      (.*, .din (pl_in_q[INTPRIORITY_BITS-1:0]), .dout(pl[1][INTPRIORITY_BITS-1:0]), .en( curr_int_tid_final_in));

  rvdffe  #(.WIDTH(ID_BITS),.OVERRIDE(1))          claimid_fl_thr0 (.*, .din (claimid_in[ID_BITS-1:00]),      .dout(claimid[0][ID_BITS-1:00]), .en(~curr_int_tid_final_in));
  rvdffe  #(.WIDTH(ID_BITS),.OVERRIDE(1))          claimid_fl_thr1 (.*, .din (claimid_in[ID_BITS-1:00]),      .dout(claimid[1][ID_BITS-1:00]), .en( curr_int_tid_final_in));

  rvdffe  #(.WIDTH(1),.OVERRIDE(1))                wake_up_ff_thr0      (.*, .din (mhwakeup_in),    .dout(mhwakeup[0]),       .en(~curr_int_tid_final_in));
  rvdffe  #(.WIDTH(1),.OVERRIDE(1))                wake_up_ff_thr1      (.*, .din (mhwakeup_in),    .dout(mhwakeup[1]),       .en( curr_int_tid_final_in));

///////


  rvdffie #(2*ID_BITS)          claimid_ff_f2     (.*, .din (claimid),      .dout(claimid_ff) );
  rvdff   #(2*INTPRIORITY_BITS) pl_ff_f2          (.*, .din (pl),           .dout(pl_ff), .clk(free_clk));
  rvdff   #(2)                  mexintpend_ff_f2  (.*, .clk(free_clk),      .din (mexintpend[1:0]), .dout(mexintpend_ff[1:0]));
  rvdff   #(2)                  wake_up_ff_f2     (.*, .clk(free_clk),      .din (mhwakeup[1:0]),   .dout(mhwakeup_ff[1:0]));

  assign claimid_out[0]  =  curr_int_tid_final ?  claimid_ff[0] : claimid[0] ;
  assign claimid_out[1]  = ~curr_int_tid_final ?  claimid_ff[1] : claimid[1] ;

  assign  pl_out[0]      =  curr_int_tid_final ?  pl_ff[0]: pl[0];
  assign  pl_out[1]      = ~curr_int_tid_final ?  pl_ff[1]: pl[1];

  assign  mexintpend_out[0]      =  curr_int_tid_final ?  mexintpend_ff[0] : mexintpend[0] ;
  assign  mexintpend_out[1]      = ~curr_int_tid_final ?  mexintpend_ff[1] : mexintpend[1] ;

  assign mhwakeup_out[0] =    curr_int_tid_final ?   mhwakeup_ff[0] : mhwakeup[0] ;
  assign mhwakeup_out[1] =   ~curr_int_tid_final ?   mhwakeup_ff[1] : mhwakeup[1] ;

  assign meipt    =  curr_int_tid_final_in ? dec_tlu_meipt[1]    : dec_tlu_meipt[0] ;
  assign meicurpl =  curr_int_tid_final_in ? dec_tlu_meicurpl[1] : dec_tlu_meicurpl[0] ;

end else begin : one_thread

  rvdff   #(1)                mexintpend_fl (.*,  .din (mexintpend_in),                 .dout(mexintpend), .clk(free_clk));
  rvdffie #(ID_BITS)          claimid_fl    (.*,  .din (claimid_in[ID_BITS-1:00]),      .dout(claimid) );
  rvdff   #(INTPRIORITY_BITS) pl_fl         (.*,  .din (pl_in_q[INTPRIORITY_BITS-1:0]), .dout(pl),         .clk(free_clk));
  rvdff   #(1)                wake_up_ff    (.*,  .din (mhwakeup_in),                   .dout(mhwakeup),   .clk(free_clk));


  assign claimid_out[pt.NUM_THREADS-1:0]    = claimid[pt.NUM_THREADS-1:0];
  assign pl_out[pt.NUM_THREADS-1:0]         = pl[pt.NUM_THREADS-1:0] ;
  assign mexintpend_out[pt.NUM_THREADS-1:0] = mexintpend[pt.NUM_THREADS-1:0] ;
  assign mhwakeup_out[pt.NUM_THREADS-1:0]   = mhwakeup[pt.NUM_THREADS-1:0] ;

  assign meipt    =  dec_tlu_meipt[0] ;
  assign meicurpl =  dec_tlu_meicurpl[0] ;

end

assign meipt_inv[INTPRIORITY_BITS-1:0]    = intpriord ? ~meipt[INTPRIORITY_BITS-1:0]    : meipt[INTPRIORITY_BITS-1:0] ;
assign meicurpl_inv[INTPRIORITY_BITS-1:0] = intpriord ? ~meicurpl[INTPRIORITY_BITS-1:0] : meicurpl[INTPRIORITY_BITS-1:0] ;
assign mexintpend_in = (( selected_int_priority[INTPRIORITY_BITS-1:0] > meipt_inv[INTPRIORITY_BITS-1:0]) &
                        ( selected_int_priority[INTPRIORITY_BITS-1:0] > meicurpl_inv[INTPRIORITY_BITS-1:0]) );

assign maxint[INTPRIORITY_BITS-1:0]      =  intpriord ? 0 : 15 ;
assign mhwakeup_in = ( pl_in_q[INTPRIORITY_BITS-1:0] == maxint) ;



//////////////////////////////////////////////////////////////////////////
//  Reads of register.
//  1- intpending
//////////////////////////////////////////////////////////////////////////

assign intpend_reg_read     =  addr_intpend_base_match      & picm_rden_ff ;
assign intpend_thr_reg_read =  addr_intpend_thr_base_match  & picm_rden_ff ;
assign intpriority_reg_read =  raddr_intpriority_base_match & picm_rden_ff;
assign intenable_reg_read   =  raddr_intenable_base_match   & picm_rden_ff;
assign delg_reg_read        =  raddr_delg_base_match        & picm_rden_ff;
assign gw_config_reg_read   =  raddr_config_gw_base_match   & picm_rden_ff;

assign thr_mx_intpend_reg[pt.PIC_TOTAL_INT_PLUS1-1:0]   = picm_rd_thr_ff ? {(extintsrc_req_gw[pt.PIC_TOTAL_INT_PLUS1-1:0] &  delg_reg[pt.PIC_TOTAL_INT_PLUS1-1:0]) } :
                                                                           {(extintsrc_req_gw[pt.PIC_TOTAL_INT_PLUS1-1:0] & ~delg_reg[pt.PIC_TOTAL_INT_PLUS1-1:0]) } ;

assign intpend_reg_extended[INTPEND_SIZE-1:0]       = {{INTPEND_SIZE-pt.PIC_TOTAL_INT_PLUS1{1'b0}},extintsrc_req_gw[pt.PIC_TOTAL_INT_PLUS1-1:0]} ;
assign thr_mx_intpend_reg_extended[INTPEND_SIZE-1:0]= {{INTPEND_SIZE-pt.PIC_TOTAL_INT_PLUS1{1'b0}},thr_mx_intpend_reg[pt.PIC_TOTAL_INT_PLUS1-1:0]} ;

   for (i=0; i<(INT_GRPS); i++) begin
            assign intpend_rd_part_out[i]     =  (({32{intpend_reg_read     &  (picm_raddr_ff[5:2] == i)}}) & intpend_reg_extended[((32*i)+31):(32*i)]) ;
            assign intpend_thr_rd_part_out[i] =  (({32{intpend_thr_reg_read &  (picm_raddr_ff[5:2] == i)}}) & thr_mx_intpend_reg_extended[((32*i)+31):(32*i)]) ;
   end

   always_comb begin : INTPEND_RD
         intpend_rd_out =  '0 ;
         intpend_thr_rd_out =  '0 ;
         for (int i=0; i<INT_GRPS; i++) begin
               intpend_rd_out     |=  intpend_rd_part_out[i] ;
               intpend_thr_rd_out |=  intpend_thr_rd_part_out[i] ;
         end
   end

   always_comb begin : INTEN_RD
         intenable_rd_out =  '0 ;
         delg_rd_out =  '0 ;
         intpriority_rd_out =  '0 ;
         gw_config_rd_out =  '0 ;
         for (int i=0; i<pt.PIC_TOTAL_INT_PLUS1; i++) begin
              if (intenable_reg_re[i]) begin
               intenable_rd_out    =  intenable_reg[i]  ;
              end
              if (delg_reg_re[i]) begin
               delg_rd_out    =  delg_reg[i]  ;
              end
              if (intpriority_reg_re[i]) begin
               intpriority_rd_out  =  intpriority_reg[i] ;
              end
              if (gw_config_reg_re[i]) begin
               gw_config_rd_out  =  gw_config_reg[i] ;
              end
         end
   end


 assign picm_rd_data_in[31:0] = ({32{intpend_reg_read      }} &   intpend_rd_out                                                    ) |
                                ({32{intpend_thr_reg_read  }} &   intpend_thr_rd_out                                                ) |
                                ({32{intpriority_reg_read  }} &  {{32-INTPRIORITY_BITS{1'b0}}, intpriority_rd_out                 } ) |
                                ({32{intenable_reg_read    }} &  {31'b0 , intenable_rd_out                                        } ) |
                                ({32{delg_reg_read         }} &  {31'b0 , delg_rd_out                                             } ) |
                                ({32{gw_config_reg_read    }} &  {30'b0 , gw_config_rd_out                                        } ) |
                                ({32{config_reg_re         }} &  {31'b0 , config_reg                                              } ) |
                                ({32{picm_mken_ff & mask[3]}} &  {30'b0 , 2'b11                                                   } ) |
                                ({32{picm_mken_ff & mask[2]}} &  {31'b0 , 1'b1                                                    } ) |
                                ({32{picm_mken_ff & mask[1]}} &  {28'b0 , 4'b1111                                                 } ) |
                                ({32{picm_mken_ff & mask[0]}} &   32'b0                                                             ) ;


assign picm_rd_data[31:0] = picm_bypass_ff ? picm_wr_data_ff[31:0] : picm_rd_data_in[31:0] ;

logic [14:0] address;

assign address[14:0] = picm_raddr_ff[14:0];

`include "pic_map_auto.h"

endmodule


module eh2_cmp_and_mux #(parameter ID_BITS=8,
                               INTPRIORITY_BITS = 4)
                    (
                        input  logic [ID_BITS-1:0]       a_id,
                        input  logic [INTPRIORITY_BITS-1:0] a_priority,

                        input  logic [ID_BITS-1:0]       b_id,
                        input  logic [INTPRIORITY_BITS-1:0] b_priority,

                        output logic [ID_BITS-1:0]       out_id,
                        output logic [INTPRIORITY_BITS-1:0] out_priority

                    );

logic   a_is_lt_b ;

assign  a_is_lt_b  = ( a_priority[INTPRIORITY_BITS-1:0] < b_priority[INTPRIORITY_BITS-1:0] ) ;

assign  out_id[ID_BITS-1:0]                = a_is_lt_b ? b_id[ID_BITS-1:0] :
                                                         a_id[ID_BITS-1:0] ;
assign  out_priority[INTPRIORITY_BITS-1:0] = a_is_lt_b ? b_priority[INTPRIORITY_BITS-1:0] :
                                                         a_priority[INTPRIORITY_BITS-1:0] ;
endmodule // cmp_and_mux


module eh2_configurable_gw (
                             input logic gw_clk,
                             input logic rawclk,
                             input logic clken,
                             input logic rst_l,
                             input logic extintsrc_req,
                             input logic meigwctrl_polarity ,
                             input logic meigwctrl_type ,
                             input logic meigwclr ,

                             output logic extintsrc_req_config
                            );

  logic  gw_int_pending_in, gw_int_pending, extintsrc_req_sync;

  rvsyncss_fpga  #(1) sync_inst (
      .dout        (extintsrc_req_sync),
      .din         (extintsrc_req),
      .*) ;
  assign gw_int_pending_in =  (extintsrc_req_sync ^ meigwctrl_polarity) | (gw_int_pending & ~meigwclr) ;
 rvdff_fpga #(1) int_pend_ff        (.*, .clk(gw_clk), .rawclk(rawclk), .clken(clken), .din (gw_int_pending_in),     .dout(gw_int_pending));


  assign extintsrc_req_config =  meigwctrl_type ? ((extintsrc_req_sync ^  meigwctrl_polarity) | gw_int_pending) : (extintsrc_req_sync ^  meigwctrl_polarity) ;

endmodule // configurable_gw









