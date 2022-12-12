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
// $Id$
//
// Function: Top level VEER core file to control the debug mode
// Comments: Responsible to put the rest of the core in quiesce mode,
//           Send the commands/address. sends WrData and Recieve read Data.
//           And then Resume the core to do the normal mode
// Author  :
//********************************************************************************
module eh2_dbg #(
`include "eh2_param.vh"
 )(
   // outputs to the core for command and data interface
   output logic [31:0]                 dbg_cmd_addr,
   output logic [31:0]                 dbg_cmd_wrdata,
   output logic                        dbg_cmd_valid,
   output logic                        dbg_cmd_tid,     // thread for debug register read
   output logic                        dbg_cmd_write,   // 1: write command, 0: read_command
   output logic [1:0]                  dbg_cmd_type,    // 0:gpr 1:csr 2: memory
   output logic [1:0]                  dbg_cmd_size,    // size of the abstract mem access debug command
   output logic                        dbg_core_rst_l,  // Debug reset

   // inputs back from the core/dec
   input logic [31:0]                  core_dbg_rddata,
   input logic                         core_dbg_cmd_done, // This will be treated like a valid signal
   input logic                         core_dbg_cmd_fail, // Exception during command run

   // Signals to dma to get a bubble
   output logic                        dbg_dma_bubble,   // Debug needs a bubble to send a valid
   input  logic                        dma_dbg_ready,    // DMA is ready to accept debug request

   // interface with the rest of the core to halt/resume handshaking
   output logic [pt.NUM_THREADS-1:0]   dbg_halt_req, // This is a pulse
   output logic [pt.NUM_THREADS-1:0]   dbg_resume_req, // Debug sends a resume requests. Pulse
   input  logic [pt.NUM_THREADS-1:0]   dec_tlu_debug_mode,        // Core is in debug mode
   input  logic [pt.NUM_THREADS-1:0]   dec_tlu_dbg_halted, // The core has finished the queiscing sequence. Core is halted now
   input  logic [pt.NUM_THREADS-1:0]   dec_tlu_mpc_halted_only,   // Only halted due to MPC
   input  logic [pt.NUM_THREADS-1:0]   dec_tlu_resume_ack, // core sends back an ack for the resume (pulse)
   input  logic [pt.NUM_THREADS-1:0]   dec_tlu_mhartstart, // running harts

   // inputs from the JTAG
   input logic                         dmi_reg_en, // read or write
   input logic [6:0]                   dmi_reg_addr, // address of DM register
   input logic                         dmi_reg_wr_en, // write instruction
   input logic [31:0]                  dmi_reg_wdata, // write data
   // output
   output logic [31:0]                 dmi_reg_rdata, // read data

   // AXI Write Channels
   output logic                        sb_axi_awvalid,
   input  logic                        sb_axi_awready,
   output logic [pt.SB_BUS_TAG-1:0]    sb_axi_awid,
   output logic [31:0]                 sb_axi_awaddr,
   output logic [3:0]                  sb_axi_awregion,
   output logic [7:0]                  sb_axi_awlen,
   output logic [2:0]                  sb_axi_awsize,
   output logic [1:0]                  sb_axi_awburst,
   output logic                        sb_axi_awlock,
   output logic [3:0]                  sb_axi_awcache,
   output logic [2:0]                  sb_axi_awprot,
   output logic [3:0]                  sb_axi_awqos,

   output logic                        sb_axi_wvalid,
   input  logic                        sb_axi_wready,
   output logic [63:0]                 sb_axi_wdata,
   output logic [7:0]                  sb_axi_wstrb,
   output logic                        sb_axi_wlast,

   input  logic                        sb_axi_bvalid,
   output logic                        sb_axi_bready,
   input  logic [1:0]                  sb_axi_bresp,

   // AXI Read Channels
   output logic                        sb_axi_arvalid,
   input  logic                        sb_axi_arready,
   output logic [pt.SB_BUS_TAG-1:0]    sb_axi_arid,
   output logic [31:0]                 sb_axi_araddr,
   output logic [3:0]                  sb_axi_arregion,
   output logic [7:0]                  sb_axi_arlen,
   output logic [2:0]                  sb_axi_arsize,
   output logic [1:0]                  sb_axi_arburst,
   output logic                        sb_axi_arlock,
   output logic [3:0]                  sb_axi_arcache,
   output logic [2:0]                  sb_axi_arprot,
   output logic [3:0]                  sb_axi_arqos,

   input  logic                        sb_axi_rvalid,
   output logic                        sb_axi_rready,
   input  logic [63:0]                 sb_axi_rdata,
   input  logic [1:0]                  sb_axi_rresp,

   input logic                         dbg_bus_clk_en,

   // general inputs
   input logic                         clk,
   input logic                         free_clk,
   input logic                         rst_l,         // This includes both top rst and debug core rst
   input logic                         dbg_rst_l,
   input logic                         clk_override,
   input logic                         scan_mode
);


   typedef enum logic [3:0] {IDLE=4'h0, HALTING=4'h1, HALTED=4'h2, CORE_CMD_START=4'h3, CORE_CMD_WAIT=4'h4, SB_CMD_START=4'h5, SB_CMD_SEND=4'h6, SB_CMD_RESP=4'h7, CMD_DONE=4'h8, RESUMING=4'h9} state_t;
   typedef enum logic [3:0] {SBIDLE=4'h0, WAIT_RD=4'h1, WAIT_WR=4'h2, CMD_RD=4'h3, CMD_WR=4'h4, CMD_WR_ADDR=4'h5, CMD_WR_DATA=4'h6, RSP_RD=4'h7, RSP_WR=4'h8, DONE=4'h9} sb_state_t;

   state_t [pt.NUM_THREADS-1:0]  dbg_state;
   state_t [pt.NUM_THREADS-1:0]  dbg_nxtstate;
   logic   [pt.NUM_THREADS-1:0]  dbg_state_en;
   // these are the registers that the debug module implements
   logic [31:0]  dmstatus_reg;        // [26:24]-dmerr, [17:16]-resume ack, [9:8]-halted, [3:0]-version
   logic [31:0]  dmcontrol_reg;       // dmcontrol register has only 6 bits implemented. 31: haltreq, 30: resumereq, 29: haltreset, 28: ackhavereset, 1: ndmreset, 0: dmactive.
   logic [31:0]  command_reg;
   logic [31:0]  abstractcs_reg;      // bits implemted are [12] - busy and [10:8]= command error
   logic [31:0]  hawindow_reg;
   logic [31:0]  haltsum0_reg;
   logic [31:0]  data0_reg;
   logic [31:0]  data1_reg;

   // data 0
   logic [31:0]  data0_din;
   logic         data0_reg_wren, data0_reg_wren0, data0_reg_wren1;
   logic [pt.NUM_THREADS-1:0]  data0_reg_wren2;
   // data 1
   logic [31:0]  data1_din;
   logic         data1_reg_wren, data1_reg_wren0, data1_reg_wren1;
   // abstractcs
   logic [pt.NUM_THREADS-1:0] abstractcs_busy;
   logic [2:0]   abstractcs_error_din;
   logic         abstractcs_error_sel0, abstractcs_error_sel1, abstractcs_error_sel2, abstractcs_error_sel3, abstractcs_error_sel4, abstractcs_error_sel5, abstractcs_error_sel6;
   logic [pt.NUM_THREADS-1:0]  dbg_sb_bus_error;
   // abstractauto
   logic         abstractauto_reg_wren;
   logic [1:0]   abstractauto_reg;
   // dmcontrol
   logic         resumereq;
   logic         dmcontrol_wren, dmcontrol_wren_Q;
   logic         dmcontrol_hasel_in, dmcontrol_hartsel_in;
   // command
   logic         execute_command_ns, execute_command;
   logic         command_wren, command_regno_wren;
   logic         command_transfer_din;
   logic         command_postexec_din;
   logic [31:0]  command_din;
   logic [3:0]   dbg_cmd_addr_incr;
   logic [31:0]  dbg_cmd_curr_addr;
   logic [31:0]  dbg_cmd_next_addr;


   // hawindow
   logic         hawindow_wren;

   // needed to send the read data back for dmi reads
   logic [31:0]  dmi_reg_rdata_din;
   logic [pt.NUM_THREADS-1:0] hart_sel;
   logic [pt.NUM_THREADS-1:0] command_sel;
   logic [pt.NUM_THREADS-1:0] dbg_halted;
   logic [pt.NUM_THREADS-1:0] dbg_running;
   logic [pt.NUM_THREADS-1:0] dbg_resumeack;
   logic [pt.NUM_THREADS-1:0] dbg_havereset;
   logic [pt.NUM_THREADS-1:0] dbg_unavailable;

   sb_state_t    sb_state;
   sb_state_t    sb_nxtstate;
   logic         sb_state_en;

   //System bus section
   logic              sbcs_wren;
   logic              sbcs_sbbusy_wren;
   logic              sbcs_sbbusy_din;
   logic              sbcs_sbbusyerror_wren;
   logic              sbcs_sbbusyerror_din;

   logic              sbcs_sberror_wren;
   logic [2:0]        sbcs_sberror_din;
   logic              sbcs_unaligned;
   logic              sbcs_illegal_size;
   logic [19:15]      sbcs_reg_int;

   // data
   logic              sbdata0_reg_wren0;
   logic              sbdata0_reg_wren1;
   logic              sbdata0_reg_wren;
   logic [31:0]       sbdata0_din;

   logic              sbdata1_reg_wren0;
   logic              sbdata1_reg_wren1;
   logic              sbdata1_reg_wren;
   logic [31:0]       sbdata1_din;

   logic              sbaddress0_reg_wren0;
   logic              sbaddress0_reg_wren1;
   logic              sbaddress0_reg_wren;
   logic [31:0]       sbaddress0_reg_din;
   logic [3:0]        sbaddress0_incr;
   logic              sbreadonaddr_access;
   logic              sbreadondata_access;
   logic              sbdata0wr_access;

   logic [pt.NUM_THREADS-1:0]  sb_abmem_cmd_done_in, sb_abmem_data_done_in;
   logic [pt.NUM_THREADS-1:0]  sb_abmem_cmd_done_en, sb_abmem_data_done_en;
   logic [pt.NUM_THREADS-1:0]  sb_abmem_cmd_done, sb_abmem_data_done;
   logic [31:0]       abmem_addr;
   logic              abmem_addr_in_dccm_region, abmem_addr_in_iccm_region, abmem_addr_in_pic_region;
   logic              abmem_addr_core_local;
   logic              abmem_addr_external;

   logic              sb_cmd_pending, sb_abmem_cmd_pending;
   logic              sb_abmem_cmd_write;
   logic [2:0]        sb_abmem_cmd_size;
   logic [31:0]       sb_abmem_cmd_addr;
   logic [31:0]       sb_abmem_cmd_wdata;

   logic [2:0]        sb_cmd_size;
   logic [31:0]       sb_cmd_addr;
   logic [63:0]       sb_cmd_wdata;

   logic              sb_bus_cmd_read, sb_bus_cmd_write_addr, sb_bus_cmd_write_data;
   logic              sb_bus_rsp_read, sb_bus_rsp_write;
   logic              sb_bus_rsp_error;
   logic [63:0]       sb_bus_rdata;

   //registers
   logic [31:0]       sbcs_reg;
   logic [31:0]       sbaddress0_reg;
   logic [31:0]       sbdata0_reg;
   logic [31:0]       sbdata1_reg;

   logic              dbg_dm_rst_l;
   logic              rst_l_sync;

   logic              sb_abmem_cmd_arvalid, sb_abmem_cmd_awvalid, sb_abmem_cmd_wvalid;
   logic              sb_abmem_read_pend;
   logic              sb_cmd_awvalid, sb_cmd_wvalid, sb_cmd_arvalid;
   logic              sb_read_pend;
   logic [31:0]       sb_axi_addr;
   logic [63:0]       sb_axi_wrdata;
   logic [2:0]        sb_axi_size;

   //clken
   logic              dbg_free_clken;
   logic              dbg_free_clk;

   logic              sb_free_clken;
   logic              sb_free_clk;

   logic              bus_clk;

   // clocking
   // used for the abstract commands.
   always_comb begin
      dbg_free_clken  = dmi_reg_en | clk_override;
      for (int i=0; i<pt.NUM_THREADS; i++) begin
         dbg_free_clken |= dec_tlu_dbg_halted[i] | dec_tlu_mpc_halted_only[i] | dec_tlu_debug_mode[i] | dbg_halt_req[i] | execute_command | dbg_state_en[i] | (dbg_state[i] != IDLE);
      end
   end

   // used for the system bus
   assign sb_free_clken = dmi_reg_en | execute_command | sb_state_en | (sb_state != SBIDLE) | clk_override;

   rvoclkhdr dbg_free_cgc     (.en(dbg_free_clken), .l1clk(dbg_free_clk), .*);
   rvoclkhdr sb_free_cgc     (.en(sb_free_clken), .l1clk(sb_free_clk), .*);

   // end clocking section

   // Reset logic
   assign dbg_dm_rst_l = dbg_rst_l & (dmcontrol_reg[0] | scan_mode);
   assign dbg_core_rst_l = ~dmcontrol_reg[1] | scan_mode;

   // synchronize the rst
   rvsyncss #(1) rstl_syncff (.din(rst_l), .dout(rst_l_sync), .clk(free_clk), .rst_l(dbg_rst_l));

   // system bus register
   // sbcs[31:29], sbcs - [22]:sbbusyerror, [21]: sbbusy, [20]:sbreadonaddr, [19:17]:sbaccess, [16]:sbautoincrement, [15]:sbreadondata, [14:12]:sberror, sbsize=32, 128=0, 64/32/16/8 are legal
   assign        sbcs_reg[31:29] = 3'b1;
   assign        sbcs_reg[28:23] = '0;
   assign        sbcs_reg[19:15] = {sbcs_reg_int[19], ~sbcs_reg_int[18], sbcs_reg_int[17:15]};
   assign        sbcs_reg[11:5]  = 7'h20;
   assign        sbcs_reg[4:0]   = 5'b01111;
   assign        sbcs_wren = (dmi_reg_addr ==  7'h38) & dmi_reg_en & dmi_reg_wr_en & (sb_state == SBIDLE); // & (sbcs_reg[14:12] == 3'b000);
   assign        sbcs_sbbusyerror_wren = (sbcs_wren & dmi_reg_wdata[22]) |
                                         (sbcs_reg[21] & dmi_reg_en & ((dmi_reg_wr_en & (dmi_reg_addr == 7'h39)) | (dmi_reg_addr == 7'h3c) | (dmi_reg_addr == 7'h3d)));
   assign        sbcs_sbbusyerror_din = ~(sbcs_wren & dmi_reg_wdata[22]);   // Clear when writing one

   rvdffs #(1) sbcs_sbbusyerror_reg  (.din(sbcs_sbbusyerror_din),  .dout(sbcs_reg[22]),    .en(sbcs_sbbusyerror_wren), .rst_l(dbg_dm_rst_l), .clk(sb_free_clk));
   rvdffs #(1) sbcs_sbbusy_reg       (.din(sbcs_sbbusy_din),       .dout(sbcs_reg[21]),    .en(sbcs_sbbusy_wren),      .rst_l(dbg_dm_rst_l), .clk(sb_free_clk));
   rvdffs #(1) sbcs_sbreadonaddr_reg (.din(dmi_reg_wdata[20]),     .dout(sbcs_reg[20]),    .en(sbcs_wren),             .rst_l(dbg_dm_rst_l), .clk(sb_free_clk));
   rvdffs #(5) sbcs_misc_reg         (.din({dmi_reg_wdata[19],~dmi_reg_wdata[18],dmi_reg_wdata[17:15]}),
                                      .dout(sbcs_reg_int[19:15]), .en(sbcs_wren),             .rst_l(dbg_dm_rst_l), .clk(sb_free_clk));
   rvdffs #(3) sbcs_error_reg        (.din(sbcs_sberror_din[2:0]), .dout(sbcs_reg[14:12]), .en(sbcs_sberror_wren),     .rst_l(dbg_dm_rst_l), .clk(sb_free_clk));

   assign sbcs_unaligned =    ((sbcs_reg[19:17] == 3'b001) &  sbaddress0_reg[0]) |
                              ((sbcs_reg[19:17] == 3'b010) &  (|sbaddress0_reg[1:0])) |
                              ((sbcs_reg[19:17] == 3'b011) &  (|sbaddress0_reg[2:0]));

   assign sbcs_illegal_size = sbcs_reg[19];    // Anything bigger than 64 bits is illegal

   assign sbaddress0_incr[3:0] = ({4{(sbcs_reg[19:17] == 3'h0)}} &  4'b0001) |
                                 ({4{(sbcs_reg[19:17] == 3'h1)}} &  4'b0010) |
                                 ({4{(sbcs_reg[19:17] == 3'h2)}} &  4'b0100) |
                                 ({4{(sbcs_reg[19:17] == 3'h3)}} &  4'b1000);

   // sbdata
   assign        sbdata0_reg_wren0   = dmi_reg_en & dmi_reg_wr_en & (dmi_reg_addr == 7'h3c);   // write data only when single read is 0
   assign        sbdata0_reg_wren1   = (sb_state == RSP_RD) & sb_state_en & ~sbcs_sberror_wren;
   assign        sbdata0_reg_wren    = sbdata0_reg_wren0 | sbdata0_reg_wren1;

   assign        sbdata1_reg_wren0   = dmi_reg_en & dmi_reg_wr_en & (dmi_reg_addr == 7'h3d);   // write data only when single read is 0;
   assign        sbdata1_reg_wren1   = (sb_state == RSP_RD) & sb_state_en & ~sbcs_sberror_wren;
   assign        sbdata1_reg_wren    = sbdata1_reg_wren0 | sbdata1_reg_wren1;

   assign        sbdata0_din[31:0]   = ({32{sbdata0_reg_wren0}} & dmi_reg_wdata[31:0]) |
                                       ({32{sbdata0_reg_wren1}} & sb_bus_rdata[31:0]);
   assign        sbdata1_din[31:0]   = ({32{sbdata1_reg_wren0}} & dmi_reg_wdata[31:0]) |
                                       ({32{sbdata1_reg_wren1}} & sb_bus_rdata[63:32]);

   rvdffe #(32)    dbg_sbdata0_reg    (.*, .din(sbdata0_din[31:0]), .dout(sbdata0_reg[31:0]), .en(sbdata0_reg_wren), .rst_l(dbg_dm_rst_l));
   rvdffe #(32)    dbg_sbdata1_reg    (.*, .din(sbdata1_din[31:0]), .dout(sbdata1_reg[31:0]), .en(sbdata1_reg_wren), .rst_l(dbg_dm_rst_l));

    // sbaddress
   assign        sbaddress0_reg_wren0   = dmi_reg_en & dmi_reg_wr_en & (dmi_reg_addr == 7'h39);
   assign        sbaddress0_reg_wren    = sbaddress0_reg_wren0 | sbaddress0_reg_wren1;
   assign        sbaddress0_reg_din[31:0]= ({32{sbaddress0_reg_wren0}} & dmi_reg_wdata[31:0]) |
                                           ({32{sbaddress0_reg_wren1}} & (sbaddress0_reg[31:0] + {28'b0,sbaddress0_incr[3:0]}));
   rvdffe #(32)    dbg_sbaddress0_reg    (.*, .din(sbaddress0_reg_din[31:0]), .dout(sbaddress0_reg[31:0]), .en(sbaddress0_reg_wren), .rst_l(dbg_dm_rst_l));

   assign sbreadonaddr_access = dmi_reg_en & dmi_reg_wr_en & (dmi_reg_addr == 7'h39) & sbcs_reg[20];   // if readonaddr is set the next command will start upon writing of addr0
   assign sbreadondata_access = dmi_reg_en & ~dmi_reg_wr_en & (dmi_reg_addr == 7'h3c) & sbcs_reg[15];  // if readondata is set the next command will start upon reading of data0
   assign sbdata0wr_access  = dmi_reg_en &  dmi_reg_wr_en & (dmi_reg_addr == 7'h3c);                   // write to sbdata0 will start write command to system bus

   // memory mapped registers
   // dmcontrol register has only 6 bits implemented. 31: haltreq, 30: resumereq, 28: ackhavereset, 26: hasel, 6:hartsel, 1: ndmreset, 0: dmactive.
   // rest all the bits are zeroed out
   // dmactive flop is reset based on core rst_l, all other flops use dm_rst_l
   assign dmcontrol_wren      = (dmi_reg_addr ==  7'h10) & dmi_reg_en & dmi_reg_wr_en;
   assign dmcontrol_reg[29]   = '0;
   assign dmcontrol_reg[27]   = '0;
   assign dmcontrol_reg[25:17] = '0;
   assign dmcontrol_reg[15:2]  = '0;
   assign dmcontrol_hasel_in  = (pt.NUM_THREADS > 1) & dmi_reg_wdata[26];   // hasel tied to 0 for single thread
   assign dmcontrol_hartsel_in = (pt.NUM_THREADS > 1) & dmi_reg_wdata[16];   // hartsel tied to 0 for single thread
   assign resumereq           = dmcontrol_reg[30] & ~dmcontrol_reg[31] & dmcontrol_wren_Q;
   rvdffs #(6) dmcontrolff (.din({dmi_reg_wdata[31:30],dmi_reg_wdata[28],dmcontrol_hasel_in,dmcontrol_hartsel_in,dmi_reg_wdata[1]}),
                            .dout({dmcontrol_reg[31:30],dmcontrol_reg[28],dmcontrol_reg[26],dmcontrol_reg[16],dmcontrol_reg[1]}), .en(dmcontrol_wren), .rst_l(dbg_dm_rst_l), .clk(dbg_free_clk));
   rvdffs #(1) dmcontrol_dmactive_ff (.din(dmi_reg_wdata[0]), .dout(dmcontrol_reg[0]), .en(dmcontrol_wren), .rst_l(dbg_rst_l), .clk(dbg_free_clk));
   rvdff  #(1) dmcontrol_wrenff(.din(dmcontrol_wren), .dout(dmcontrol_wren_Q), .rst_l(dbg_dm_rst_l), .clk(dbg_free_clk));

   // dmstatus register bits that are implemented
   // [19:18]-havereset,[17:16]-resume ack, [15:14]-available, [9]-allhalted, [8]-anyhalted, [3:0]-version
   // rest all the bits are zeroed out
   assign dmstatus_reg[31:20] = '0;
   assign dmstatus_reg[19]    = &(dbg_havereset[pt.NUM_THREADS-1:0] | ~hart_sel[pt.NUM_THREADS-1:0]);
   assign dmstatus_reg[18]    = |(dbg_havereset[pt.NUM_THREADS-1:0] & hart_sel[pt.NUM_THREADS-1:0]);
   assign dmstatus_reg[17]    = &(dbg_resumeack[pt.NUM_THREADS-1:0] | ~hart_sel[pt.NUM_THREADS-1:0]);
   assign dmstatus_reg[16]    = |(dbg_resumeack[pt.NUM_THREADS-1:0] & hart_sel[pt.NUM_THREADS-1:0]);
   assign dmstatus_reg[15:14] = '0;
   assign dmstatus_reg[13]    = &(dbg_unavailable[pt.NUM_THREADS-1:0] | ~hart_sel[pt.NUM_THREADS-1:0]);
   assign dmstatus_reg[12]    = |(dbg_unavailable[pt.NUM_THREADS-1:0] & hart_sel[pt.NUM_THREADS-1:0]);
   assign dmstatus_reg[11]    = &(dbg_running[pt.NUM_THREADS-1:0] | ~hart_sel[pt.NUM_THREADS-1:0]);
   assign dmstatus_reg[10]    = |(dbg_running[pt.NUM_THREADS-1:0] & hart_sel[pt.NUM_THREADS-1:0]);
   assign dmstatus_reg[9]     = &(dbg_halted[pt.NUM_THREADS-1:0] | ~hart_sel[pt.NUM_THREADS-1:0]);
   assign dmstatus_reg[8]     = |(dbg_halted[pt.NUM_THREADS-1:0] & hart_sel[pt.NUM_THREADS-1:0]);
   assign dmstatus_reg[7]     = '1;
   assign dmstatus_reg[6:4]   = '0;
   assign dmstatus_reg[3:0]   = 4'h2;

   // haltsum0 register
   assign haltsum0_reg[31:pt.NUM_THREADS] = '0;
   for (genvar i=0; i<pt.NUM_THREADS; i++) begin: Gen_haltsum
      assign haltsum0_reg[i]  = dbg_halted[i];
   end

   // abstractcs register
   // bits implemted are [12] - busy and [10:8]= command error
   assign        abstractcs_reg[31:13] = '0;
   assign        abstractcs_reg[11]    = '0;
   assign        abstractcs_reg[7:4]   = '0;
   assign        abstractcs_reg[3:0]   = 4'h2;    // One data register


   assign        abstractcs_error_sel0 = abstractcs_reg[12] & ~(|abstractcs_reg[10:8]) & dmi_reg_en & ((dmi_reg_wr_en & ((dmi_reg_addr == 7'h16) | (dmi_reg_addr == 7'h17)) | (dmi_reg_addr == 7'h18)) |
                                                                                                       (dmi_reg_addr == 7'h4) | (dmi_reg_addr == 7'h5));
   assign        abstractcs_error_sel1 = execute_command & ~(|abstractcs_reg[10:8]) &
                                         ((~((command_reg[31:24] == 8'b0) | (command_reg[31:24] == 8'h2)))                      |   // Illegal command
                                          (((command_reg[22:20] == 3'b011) | (command_reg[22])) & (command_reg[31:24] == 8'h2)) |   // Illegal abstract memory size (can't be DW or higher)
                                          ((command_reg[22:20] != 3'b010) & ((command_reg[31:24] == 8'h0) & command_reg[17]))   |   // Illegal abstract reg size
                                          ((command_reg[31:24] == 8'h0) & command_reg[18]));                                          //postexec for abstract register access
   assign        abstractcs_error_sel2 = ((core_dbg_cmd_done & core_dbg_cmd_fail) |                   // exception from core
                                          (execute_command & (command_reg[31:24] == 8'h0) &           // unimplemented regs
                                                (((command_reg[15:12] == 4'h1) & (command_reg[11:5] != 0)) | (command_reg[15:13] != 0)))) & ~(|abstractcs_reg[10:8]);
   assign        abstractcs_error_sel3 = execute_command & ~(|abstractcs_reg[10:8]) & ~(|(command_sel[pt.NUM_THREADS-1:0] & dbg_halted[pt.NUM_THREADS-1:0]));  //(dbg_state != HALTED);;
   assign        abstractcs_error_sel4 = (|dbg_sb_bus_error[pt.NUM_THREADS-1:0]) & dbg_bus_clk_en & ~(|abstractcs_reg[10:8]);// sb bus error for abstract memory command
   assign        abstractcs_error_sel5 = execute_command & (command_reg[31:24] == 8'h2) & ~(|abstractcs_reg[10:8]) &
                                         (((command_reg[22:20] == 3'b001) & data1_reg[0]) | ((command_reg[22:20] == 3'b010) & (|data1_reg[1:0])));  //Unaligned address for abstract memory
   assign        abstractcs_error_sel6 = (dmi_reg_addr ==  7'h16) & dmi_reg_en & dmi_reg_wr_en;

   assign        abstractcs_error_din[2:0]  = abstractcs_error_sel0 ? 3'b001 :                  // writing command or abstractcs while a command was executing. Or accessing data0
                                                 abstractcs_error_sel1 ? 3'b010 :               // writing a illegal command type to cmd field of command
                                                    abstractcs_error_sel2 ? 3'b011 :            // exception while running command
                                                       abstractcs_error_sel3 ? 3'b100 :         // writing a comnand when not in the halted state
                                                          abstractcs_error_sel4 ? 3'b101 :      // Bus error
                                                             abstractcs_error_sel5 ? 3'b111 :   // unaligned or illegal size abstract memory command
                                                                abstractcs_error_sel6 ? (~dmi_reg_wdata[10:8] & abstractcs_reg[10:8]) :   //W1C
                                                                                        abstractcs_reg[10:8];                             //hold

   assign abstractcs_reg[12] = |abstractcs_busy[pt.NUM_THREADS-1:0];

   rvdff  #(3) dmabstractcs_error_reg (.din(abstractcs_error_din[2:0]), .dout(abstractcs_reg[10:8]), .rst_l(dbg_dm_rst_l), .clk(dbg_free_clk));

    // abstract auto reg
   assign abstractauto_reg_wren  = dmi_reg_en & dmi_reg_wr_en & (dmi_reg_addr == 7'h18) & ~abstractcs_reg[12];
   rvdffs #(2) dbg_abstractauto_reg (.*, .din(dmi_reg_wdata[1:0]), .dout(abstractauto_reg[1:0]), .en(abstractauto_reg_wren), .rst_l(dbg_dm_rst_l), .clk(dbg_free_clk));

   // command register - implemented all the bits in this register
   // command[16] = 1: write, 0: read
   assign execute_command_ns = command_wren |
                               (dmi_reg_en & ~abstractcs_reg[12] & (((dmi_reg_addr == 7'h4) & abstractauto_reg[0]) | ((dmi_reg_addr == 7'h5) & abstractauto_reg[1])));
   always_comb begin
      command_wren = 1'b0;
      for (int i=0; i<pt.NUM_THREADS; i++) begin
         command_wren |= ((dmi_reg_addr == 7'h17) & dmi_reg_en & dmi_reg_wr_en & command_sel[i]);
      end
   end
   assign command_regno_wren = command_wren | ((command_reg[31:24] == 8'h0) & command_reg[19] & (dbg_state == CMD_DONE) & ~(|abstractcs_reg[10:8]));  // aarpostincrement
   assign command_postexec_din = (dmi_reg_wdata[31:24] == 8'h0) & dmi_reg_wdata[18];
   assign command_transfer_din = (dmi_reg_wdata[31:24] == 8'h0) & dmi_reg_wdata[17];
   assign command_din[31:16] = {dmi_reg_wdata[31:24],1'b0,dmi_reg_wdata[22:19],command_postexec_din,command_transfer_din, dmi_reg_wdata[16]};
   assign command_din[15:0] =  command_wren ? dmi_reg_wdata[15:0] : dbg_cmd_next_addr[15:0];
   rvdff  #(1)  execute_commandff   (.*, .din(execute_command_ns), .dout(execute_command), .clk(dbg_free_clk), .rst_l(dbg_dm_rst_l));
   rvdffe #(16) dmcommand_reg       (.*, .din(command_din[31:16]), .dout(command_reg[31:16]), .en(command_wren), .rst_l(dbg_dm_rst_l));
   rvdffe #(16) dmcommand_regno_reg (.*, .din(command_din[15:0]),  .dout(command_reg[15:0]),  .en(command_regno_wren), .rst_l(dbg_dm_rst_l));

   // hawindow reg
   assign hawindow_wren = dmi_reg_en & dmi_reg_wr_en & (dmi_reg_addr == 7'h15);
   assign hawindow_reg[31:pt.NUM_THREADS] = '0;

   for (genvar i=0; i<pt.NUM_THREADS; i++) begin: GenHAWindow
      rvdffs #(1) dbg_hawindow_reg (.*, .din(dmi_reg_wdata[i]), .dout(hawindow_reg[i]), .en(hawindow_wren), .rst_l(dbg_dm_rst_l), .clk(dbg_free_clk));
   end

   // data0 reg
   always_comb begin
      data0_reg_wren0 = 1'b0;
      data0_reg_wren1 = 1'b0;
      for (int i=0; i<pt.NUM_THREADS; i++) begin
         data0_reg_wren0   |= (dmi_reg_en & dmi_reg_wr_en & (dmi_reg_addr == 7'h4) & command_sel[i] & (dbg_state[i] == HALTED) & ~abstractcs_reg[12]);
         data0_reg_wren1   |= (core_dbg_cmd_done & (dbg_state[i] == CORE_CMD_WAIT) & ~command_reg[16]);
      end
   end
   assign data0_reg_wren    = data0_reg_wren0 | data0_reg_wren1 | (|data0_reg_wren2[pt.NUM_THREADS-1:0]);

   assign data0_din[31:0]   = ({32{data0_reg_wren0}} & dmi_reg_wdata[31:0])   |
                              ({32{data0_reg_wren1}} & core_dbg_rddata[31:0]) |
                              ({32{|data0_reg_wren2}} & sb_bus_rdata[31:0]);

   rvdffe #(32) dbg_data0_reg (.*, .din(data0_din[31:0]), .dout(data0_reg[31:0]), .en(data0_reg_wren), .rst_l(dbg_dm_rst_l));

   // data 1
   always_comb begin
      data1_reg_wren0 = 1'b0;
      data1_reg_wren1 = 1'b0;
      for (int i=0; i<pt.NUM_THREADS; i++) begin
         data1_reg_wren0   |= (dmi_reg_en & dmi_reg_wr_en & (dmi_reg_addr == 7'h5) & command_sel[i] & (dbg_state[i] == HALTED));
         data1_reg_wren1   |= ((dbg_state[i] == CMD_DONE) & (command_reg[31:24] == 8'h2) & command_reg[19] & ~(|abstractcs_reg[10:8]));   // aampostincrement
      end
   end
   assign data1_reg_wren    = data1_reg_wren0 | data1_reg_wren1;

   assign data1_din[31:0]   = ({32{data1_reg_wren0}} & dmi_reg_wdata[31:0]) |
                              ({32{data1_reg_wren1}} & dbg_cmd_next_addr[31:0]);

   rvdffe #(32)    dbg_data1_reg    (.*, .din(data1_din[31:0]), .dout(data1_reg[31:0]), .en(data1_reg_wren), .rst_l(dbg_dm_rst_l));

   // Generate the per thread sel and state
   for (genvar i=0; i<pt.NUM_THREADS; i++) begin

      logic [pt.NUM_THREADS-1:0] dbg_resumeack_wren, dbg_resumeack_din;
      logic [pt.NUM_THREADS-1:0] dbg_haveresetn_wren, dbg_haveresetn;
      logic [pt.NUM_THREADS-1:0] abstractcs_busy_wren, abstractcs_busy_din;

      assign hart_sel[i] = (dmcontrol_reg[16] == 1'(i)) | (dmcontrol_reg[26] & hawindow_reg[i]);
      assign command_sel[i] = (dmcontrol_reg[16] == 1'(i));

      // Per thread halted/resumeack/havereset signal
      assign dbg_resumeack_wren[i] = ((dbg_state[i] == RESUMING) & dec_tlu_resume_ack[i]) | (dbg_resumeack[i] & resumereq & dbg_halted[i] & hart_sel[i]);
      assign dbg_resumeack_din[i]  = (dbg_state[i] == RESUMING) & dec_tlu_resume_ack[i];

      assign dbg_haveresetn_wren[i] = (dmi_reg_addr == 7'h10) & dmi_reg_wdata[28] & dmi_reg_en & dmi_reg_wr_en & ((dmi_reg_wdata[16] == 1'(i)) | (dmi_reg_wdata[26] & hawindow_reg[i])) & dmcontrol_reg[0];
      assign dbg_havereset[i]      = ~dbg_haveresetn[i];

      assign dbg_unavailable[i] = ~rst_l_sync | dmcontrol_reg[1] | ~dec_tlu_mhartstart[i];
      assign dbg_running[i]     = ~(dbg_unavailable[i] | dbg_halted[i]);

      rvdff  #(1) dbg_halted_reg       (.din(dec_tlu_dbg_halted[i] & ~dec_tlu_mpc_halted_only[i]), .dout(dbg_halted[i]), .rst_l(dbg_dm_rst_l), .clk(dbg_free_clk));
      rvdffs #(1) dbg_resumeack_reg    (.din(dbg_resumeack_din[i]), .dout(dbg_resumeack[i]), .en(dbg_resumeack_wren[i]), .rst_l(dbg_dm_rst_l), .clk(dbg_free_clk));
      rvdffs #(1) dbg_haveresetn_reg   (.din(1'b1), .dout(dbg_haveresetn[i]), .en(dbg_haveresetn_wren[i]), .rst_l(rst_l), .clk(dbg_free_clk));
      rvdffs #(1) abstractcs_busy_reg  (.din(abstractcs_busy_din[i]), .dout(abstractcs_busy[i]), .en(abstractcs_busy_wren[i]), .rst_l(dbg_dm_rst_l), .clk(dbg_free_clk));
      rvdffs #($bits(state_t)) dbg_state_reg    (.din(dbg_nxtstate[i]), .dout({dbg_state[i]}), .en(dbg_state_en[i]), .rst_l(dbg_dm_rst_l & rst_l), .clk(dbg_free_clk));
      rvdffs #(1) sb_abmem_cmd_doneff  (.din(sb_abmem_cmd_done_in[i]),  .dout(sb_abmem_cmd_done[i]),  .en(sb_abmem_cmd_done_en[i]),  .rst_l(dbg_dm_rst_l), .clk(dbg_free_clk), .*);
      rvdffs #(1) sb_abmem_data_doneff (.din(sb_abmem_data_done_in[i]), .dout(sb_abmem_data_done[i]), .en(sb_abmem_data_done_en[i]), .rst_l(dbg_dm_rst_l), .clk(dbg_free_clk), .*);

      // FSM to control the debug mode entry, command send/recieve, and Resume flow.
      always_comb begin
         dbg_nxtstate[i]         = IDLE;
         dbg_state_en[i]         = 1'b0;
         abstractcs_busy_wren    = 1'b0;
         abstractcs_busy_din     = 1'b0;
         dbg_halt_req[i]   = dmcontrol_wren_Q & dmcontrol_reg[31] & hart_sel[i];      // single pulse output to the core. Need to drive every time this register is written since core might be halted due to MPC
         dbg_resume_req[i] = 1'b0;                                                                        // single pulse output to the core
         dbg_sb_bus_error[i]     = 1'b0;
         data0_reg_wren2[i]      = 1'b0;
         sb_abmem_cmd_done_in[i] = 1'b0;
         sb_abmem_data_done_in[i]= 1'b0;
         sb_abmem_cmd_done_en[i] = 1'b0;
         sb_abmem_data_done_en[i]= 1'b0;


         case (dbg_state[i])
            IDLE: begin
                     dbg_nxtstate[i]      = (dbg_halted[i] | dec_tlu_mpc_halted_only[i]) ? HALTED : HALTING;         // initiate the halt command to the core
                     dbg_state_en[i]      = (dmcontrol_reg[31] & hart_sel[i]) | dbg_halted[i] | dec_tlu_mpc_halted_only[i];      // when the jtag writes the halt bit in the DM register, OR when the status indicates MPC halted
                     dbg_halt_req[i]       = dmcontrol_reg[31] & hart_sel[i];      // only when jtag has written the halt_req bit in the control. Removed debug mode qualification during MPC changes
            end
            HALTING : begin
                     dbg_nxtstate[i]      = HALTED;                                       // Goto HALTED once the core sends an ACK
                     dbg_state_en[i]      = dbg_halted[i] | dec_tlu_mpc_halted_only[i];   // core indicates halted
            end
            HALTED: begin
                     // wait for halted to go away before send to resume. Else start of new command
                      dbg_nxtstate[i]      = dbg_halted[i] ? ((resumereq & hart_sel[i]) ? RESUMING :
                                                                 (((command_reg[31:24] == 8'h2) & abmem_addr_external & hart_sel[i]) ? SB_CMD_START : CORE_CMD_START)) :
                                                                                   ((dmcontrol_reg[31] & hart_sel[i]) ? HALTING : IDLE);       // This is MPC halted case
                     dbg_state_en[i]      = (dbg_halted[i] & resumereq & hart_sel[i]) | (execute_command & command_sel[i]) | ~(dbg_halted[i] | dec_tlu_mpc_halted_only[i]);         // need to be exclusive ???
                     abstractcs_busy_wren[i] = dbg_state_en[i] & ((dbg_nxtstate[i] == CORE_CMD_START) | (dbg_nxtstate[i] == SB_CMD_START));                      // write busy when a new command was written by jtag
                     abstractcs_busy_din[i]  = 1'b1;
                     dbg_resume_req[i] = dbg_state_en[i] & (dbg_nxtstate[i] == RESUMING);                       // single cycle pulse to core if resuming
            end
            CORE_CMD_START: begin
                     // Don't execute the command if cmderror or transfer=0 for abstract register access
                     dbg_nxtstate[i]      = ((|abstractcs_reg[10:8]) | ((command_reg[31:24] == 8'h0) & ~command_reg[17])) ? CMD_DONE : CORE_CMD_WAIT;     // new command sent to the core
                     dbg_state_en[i]      = dbg_cmd_valid | (|abstractcs_reg[10:8]) | ((command_reg[31:24] == 8'h0) & ~command_reg[17]);
            end
            CORE_CMD_WAIT: begin
                     dbg_nxtstate[i]      = CMD_DONE;
                     dbg_state_en[i]      = core_dbg_cmd_done;                   // go to done state for one cycle after completing current command
            end
            SB_CMD_START: begin
                     dbg_nxtstate[i]      = (|abstractcs_reg[10:8]) ? CMD_DONE : SB_CMD_SEND;
                     dbg_state_en[i]      = (dbg_bus_clk_en & ~sb_cmd_pending) | (|abstractcs_reg[10:8]);
            end
            SB_CMD_SEND: begin
                     sb_abmem_cmd_done_in[i]  = 1'b1;
                     sb_abmem_data_done_in[i] = 1'b1;
                     sb_abmem_cmd_done_en[i]  = (sb_bus_cmd_read | sb_bus_cmd_write_addr) & dbg_bus_clk_en;
                     sb_abmem_data_done_en[i] = (sb_bus_cmd_read | sb_bus_cmd_write_data) & dbg_bus_clk_en;
                     dbg_nxtstate[i]          = SB_CMD_RESP;
                     dbg_state_en[i]          = (sb_abmem_cmd_done[i] | sb_abmem_cmd_done_en[i]) & (sb_abmem_data_done[i] | sb_abmem_data_done_en[i]) & dbg_bus_clk_en;
            end
            SB_CMD_RESP: begin
                     dbg_nxtstate[i]         = CMD_DONE;
                     dbg_state_en[i]         = (sb_bus_rsp_read | sb_bus_rsp_write) & dbg_bus_clk_en;
                     dbg_sb_bus_error[i]     = (sb_bus_rsp_read | sb_bus_rsp_write) & sb_bus_rsp_error & dbg_bus_clk_en;
                     data0_reg_wren2[i]      = dbg_state_en[i] & ~sb_abmem_cmd_write & ~dbg_sb_bus_error[i];
            end
            CMD_DONE: begin
                     dbg_nxtstate[i]         = HALTED;
                     dbg_state_en[i]         = 1'b1;
                     abstractcs_busy_wren[i] = dbg_state_en[i];                    // remove the busy bit from the abstracts ( bit 12 )
                     abstractcs_busy_din[i]  = 1'b0;
                     sb_abmem_cmd_done_in[i] = 1'b0;
                     sb_abmem_data_done_in[i]= 1'b0;
                     sb_abmem_cmd_done_en[i] = 1'b1;
                     sb_abmem_data_done_en[i]= 1'b1;
            end
            RESUMING : begin
                     dbg_nxtstate[i]      = IDLE;
                     dbg_state_en[i]      = dbg_resumeack[i];
            end
            default : begin
                     dbg_nxtstate[i]         = IDLE;
                     dbg_state_en[i]         = 1'b0;
                     abstractcs_busy_wren[i] = 1'b0;
                     abstractcs_busy_din[i]  = 1'b0;
                     dbg_halt_req[i]         = 1'b0;         // single pulse output to the core
                     dbg_resume_req[i]       = 1'b0;         // single pulse output to the core
                     dbg_sb_bus_error[i]     = 1'b0;
                     data0_reg_wren2[i]      = 1'b0;
                     sb_abmem_cmd_done_in[i] = 1'b0;
                     sb_abmem_data_done_in[i]= 1'b0;
                     sb_abmem_cmd_done_en[i] = 1'b0;
                     sb_abmem_data_done_en[i]= 1'b0;

           end
         endcase
      end // always_comb begin
   end // for (genvar i=0; i<pt.NUM_THREADS; i++)

   assign dmi_reg_rdata_din[31:0] = ({32{dmi_reg_addr == 7'h4}}  & data0_reg[31:0])      |
                                    ({32{dmi_reg_addr == 7'h5}}  & data1_reg[31:0])      |
                                    ({32{dmi_reg_addr == 7'h10}} & {2'b0,dmcontrol_reg[29],1'b0,dmcontrol_reg[27:0]})  |  // Read0 to Write only bits
                                    ({32{dmi_reg_addr == 7'h11}} & dmstatus_reg[31:0])   |
                                    ({32{dmi_reg_addr == 7'h15}} & hawindow_reg[31:0]) |
                                    ({32{dmi_reg_addr == 7'h16}} & abstractcs_reg[31:0]) |
                                    ({32{dmi_reg_addr == 7'h17}} & command_reg[31:0])    |
                                    ({32{dmi_reg_addr == 7'h18}} & {30'h0,abstractauto_reg[1:0]})    |
                                    ({32{dmi_reg_addr == 7'h40}} & haltsum0_reg[31:0])   |
                                    ({32{dmi_reg_addr == 7'h38}} & sbcs_reg[31:0])       |
                                    ({32{dmi_reg_addr == 7'h39}} & sbaddress0_reg[31:0]) |
                                    ({32{dmi_reg_addr == 7'h3c}} & sbdata0_reg[31:0])    |
                                    ({32{dmi_reg_addr == 7'h3d}} & sbdata1_reg[31:0]);


   // Ack will use the power on reset only otherwise there won't be any ack until dmactive is 1
   rvdffe #(32)             dmi_rddata_reg   (.din(dmi_reg_rdata_din[31:0]), .dout(dmi_reg_rdata[31:0]), .en(dmi_reg_en), .rst_l(dbg_dm_rst_l), .clk(clk), .*);

   assign abmem_addr[31:0]      = data1_reg[31:0];
   assign abmem_addr_core_local = (abmem_addr_in_dccm_region | abmem_addr_in_iccm_region | abmem_addr_in_pic_region);
   assign abmem_addr_external   = ~abmem_addr_core_local;

   assign abmem_addr_in_dccm_region = (abmem_addr[31:28] == pt.DCCM_REGION) & pt.DCCM_ENABLE;
   assign abmem_addr_in_iccm_region = (abmem_addr[31:28] == pt.ICCM_REGION) & pt.ICCM_ENABLE;
   assign abmem_addr_in_pic_region  = (abmem_addr[31:28] == pt.PIC_REGION);

   // interface for the core
   assign dbg_cmd_addr[31:0]    = (command_reg[31:24] == 8'h2) ? data1_reg[31:0]  : {20'b0, command_reg[11:0]};
   assign dbg_cmd_wrdata[31:0]  = data0_reg[31:0];
   always_comb begin
      dbg_cmd_valid = 1'b0;
      for (int i=0; i<pt.NUM_THREADS; i++) begin
         dbg_cmd_valid  |= (dbg_state[i] == CORE_CMD_START) & ~((|abstractcs_reg[10:8]) | ((command_reg[31:24] == 8'h0) & ~command_reg[17]) | ((command_reg[31:24] == 8'h2) & abmem_addr_external)) &
                           ~((command_reg[31:24] == 8'h2) & ~dma_dbg_ready);
      end
   end
   assign dbg_cmd_tid           = dmcontrol_reg[16];
   assign dbg_cmd_write         = command_reg[16];
   assign dbg_cmd_type[1:0]     = (command_reg[31:24] == 8'h2) ? 2'b10 : {1'b0, (command_reg[15:12] == 4'b0)};
   assign dbg_cmd_size[1:0]     = command_reg[21:20];

   assign dbg_cmd_addr_incr[3:0]  = (command_reg[31:24] == 8'h2) ? (4'h1 << sb_abmem_cmd_size[1:0]) : 4'h1;
   assign dbg_cmd_curr_addr[31:0] = (command_reg[31:24] == 8'h2) ? data1_reg[31:0]  : {16'b0, command_reg[15:0]};
   assign dbg_cmd_next_addr[31:0] = dbg_cmd_curr_addr[31:0] + {28'h0,dbg_cmd_addr_incr[3:0]};

   // Ask DMA to stop taking bus trxns since debug memory request is done
   always_comb begin
      dbg_dma_bubble = 1'b0;
      for (int i=0; i<pt.NUM_THREADS; i++) begin
         dbg_dma_bubble     |= ((((dbg_state[i] == CORE_CMD_START) & ~(|abstractcs_reg[10:8])) | (dbg_state[i] == CORE_CMD_WAIT)) & (command_reg[31:24] == 8'h2));
      end
   end

   assign sb_cmd_pending       = (sb_state == CMD_RD) | (sb_state == CMD_WR) | (sb_state == CMD_WR_ADDR) | (sb_state == CMD_WR_DATA) | (sb_state == RSP_RD) | (sb_state == RSP_WR);
   assign sb_abmem_cmd_pending = (dbg_state == SB_CMD_START) | (dbg_state == SB_CMD_SEND) | (dbg_state== SB_CMD_RESP);

  // system bus FSM
  always_comb begin
      sb_nxtstate            = SBIDLE;
      sb_state_en            = 1'b0;
      sbcs_sbbusy_wren       = 1'b0;
      sbcs_sbbusy_din        = 1'b0;
      sbcs_sberror_wren      = 1'b0;
      sbcs_sberror_din[2:0]  = 3'b0;
      sbaddress0_reg_wren1   = 1'b0;
      case (sb_state)
            SBIDLE: begin
                     sb_nxtstate            = sbdata0wr_access ? WAIT_WR : WAIT_RD;
                     sb_state_en            = (sbdata0wr_access | sbreadondata_access | sbreadonaddr_access) & ~(|sbcs_reg[14:12]) & ~sbcs_reg[22];
                     sbcs_sbbusy_wren       = sb_state_en;                                                 // set the single read bit if it is a singlread command
                     sbcs_sbbusy_din        = 1'b1;
                     sbcs_sberror_wren      = sbcs_wren & (|dmi_reg_wdata[14:12]);                                            // write to clear the error bits
                     sbcs_sberror_din[2:0]  = ~dmi_reg_wdata[14:12] & sbcs_reg[14:12];
            end
            WAIT_RD: begin
                     sb_nxtstate           = (sbcs_unaligned | sbcs_illegal_size) ? DONE : CMD_RD;
                     sb_state_en           = (dbg_bus_clk_en & ~sb_abmem_cmd_pending) | sbcs_unaligned | sbcs_illegal_size;
                     sbcs_sberror_wren     = sbcs_unaligned | sbcs_illegal_size;
                     sbcs_sberror_din[2:0] = sbcs_unaligned ? 3'b011 : 3'b100;
            end
            WAIT_WR: begin
                     sb_nxtstate           = (sbcs_unaligned | sbcs_illegal_size) ? DONE : CMD_WR;
                     sb_state_en           = (dbg_bus_clk_en & ~sb_abmem_cmd_pending) | sbcs_unaligned | sbcs_illegal_size;
                     sbcs_sberror_wren     = sbcs_unaligned | sbcs_illegal_size;
                     sbcs_sberror_din[2:0] = sbcs_unaligned ? 3'b011 : 3'b100;
            end
            CMD_RD : begin
                     sb_nxtstate           = RSP_RD;
                     sb_state_en           = sb_bus_cmd_read & dbg_bus_clk_en;
            end
            CMD_WR : begin
                     sb_nxtstate           = (sb_bus_cmd_write_addr & sb_bus_cmd_write_data) ? RSP_WR : (sb_bus_cmd_write_data ? CMD_WR_ADDR : CMD_WR_DATA);
                     sb_state_en           = (sb_bus_cmd_write_addr | sb_bus_cmd_write_data) & dbg_bus_clk_en;
            end
            CMD_WR_ADDR : begin
                     sb_nxtstate           = RSP_WR;
                     sb_state_en           = sb_bus_cmd_write_addr & dbg_bus_clk_en;
            end
            CMD_WR_DATA : begin
                     sb_nxtstate           = RSP_WR;
                     sb_state_en           = sb_bus_cmd_write_data & dbg_bus_clk_en;
            end
            RSP_RD: begin
                     sb_nxtstate           = DONE;
                     sb_state_en           = sb_bus_rsp_read & dbg_bus_clk_en;
                     sbcs_sberror_wren     = sb_state_en & sb_bus_rsp_error;
                     sbcs_sberror_din[2:0] = 3'b010;
            end
            RSP_WR: begin
                     sb_nxtstate           = DONE;
                     sb_state_en           = sb_bus_rsp_write & dbg_bus_clk_en;
                     sbcs_sberror_wren     = sb_state_en & sb_bus_rsp_error;
                     sbcs_sberror_din[2:0] = 3'b010;
            end
            DONE: begin
                     sb_nxtstate            = SBIDLE;
                     sb_state_en            = 1'b1;
                     sbcs_sbbusy_wren       = 1'b1;                           // reset the single read
                     sbcs_sbbusy_din        = 1'b0;
                     sbaddress0_reg_wren1   = sbcs_reg[16] & (sbcs_reg[14:12] == 3'b0);    // auto increment was set and no error. Update to new address after completing the current command
            end
            default : begin
                     sb_nxtstate            = SBIDLE;
                     sb_state_en            = 1'b0;
                     sbcs_sbbusy_wren       = 1'b0;
                     sbcs_sbbusy_din        = 1'b0;
                     sbcs_sberror_wren      = 1'b0;
                     sbcs_sberror_din[2:0]  = 3'b0;
                     sbaddress0_reg_wren1   = 1'b0;
           end
         endcase
   end // always_comb begin

   rvdffs #($bits(sb_state_t)) sb_state_reg (.din(sb_nxtstate), .dout({sb_state}), .en(sb_state_en), .rst_l(dbg_dm_rst_l), .clk(sb_free_clk));

   assign sb_abmem_cmd_write      = command_reg[16];
   assign sb_abmem_cmd_size[2:0]  = {1'b0, command_reg[21:20]};
   assign sb_abmem_cmd_addr[31:0] = abmem_addr[31:0];
   assign sb_abmem_cmd_wdata[31:0] = data0_reg[31:0];

   assign sb_cmd_size[2:0]   = sbcs_reg[19:17];
   assign sb_cmd_wdata[63:0] = {sbdata1_reg[31:0], sbdata0_reg[31:0]};
   assign sb_cmd_addr[31:0]  = sbaddress0_reg[31:0];

   always_comb begin
      sb_abmem_cmd_awvalid = 1'b0;
      sb_abmem_cmd_wvalid  = 1'b0;
      sb_abmem_cmd_arvalid = 1'b0;
      sb_abmem_read_pend   = 1'b0;
      for (int i=0; i<pt.NUM_THREADS; i++) begin
         sb_abmem_cmd_awvalid    |= (dbg_state[i] == SB_CMD_SEND) & sb_abmem_cmd_write & ~sb_abmem_cmd_done[i];
         sb_abmem_cmd_wvalid     |= (dbg_state[i] == SB_CMD_SEND) & sb_abmem_cmd_write & ~sb_abmem_data_done[i];
         sb_abmem_cmd_arvalid    |= (dbg_state[i] == SB_CMD_SEND) & ~sb_abmem_cmd_write & ~sb_abmem_cmd_done[i] & ~sb_abmem_data_done[i];
         sb_abmem_read_pend      |= (dbg_state[i] == SB_CMD_RESP) & ~sb_abmem_cmd_write;
      end
   end

   assign sb_cmd_awvalid     = ((sb_state == CMD_WR) | (sb_state == CMD_WR_ADDR));
   assign sb_cmd_wvalid      = ((sb_state == CMD_WR) | (sb_state == CMD_WR_DATA));
   assign sb_cmd_arvalid     = (sb_state == CMD_RD);
   assign sb_read_pend       = (sb_state == RSP_RD);

   assign sb_axi_size[2:0]    = (sb_abmem_cmd_awvalid | sb_abmem_cmd_wvalid | sb_abmem_cmd_arvalid | sb_abmem_read_pend) ? sb_abmem_cmd_size[2:0] : sb_cmd_size[2:0];
   assign sb_axi_addr[31:0]   = (sb_abmem_cmd_awvalid | sb_abmem_cmd_wvalid | sb_abmem_cmd_arvalid | sb_abmem_read_pend) ? sb_abmem_cmd_addr[31:0] : sb_cmd_addr[31:0];
   assign sb_axi_wrdata[63:0] = (sb_abmem_cmd_awvalid | sb_abmem_cmd_wvalid) ? {2{sb_abmem_cmd_wdata[31:0]}} : sb_cmd_wdata[63:0];

   // Generic bus response signals
   assign sb_bus_cmd_read       = sb_axi_arvalid & sb_axi_arready;
   assign sb_bus_cmd_write_addr = sb_axi_awvalid & sb_axi_awready;
   assign sb_bus_cmd_write_data = sb_axi_wvalid  & sb_axi_wready;

   assign sb_bus_rsp_read  = sb_axi_rvalid & sb_axi_rready;
   assign sb_bus_rsp_write = sb_axi_bvalid & sb_axi_bready;
   assign sb_bus_rsp_error = (sb_bus_rsp_read & (|(sb_axi_rresp[1:0]))) | (sb_bus_rsp_write & (|(sb_axi_bresp[1:0])));

   // AXI Request signals
   assign sb_axi_awvalid              = sb_abmem_cmd_awvalid | sb_cmd_awvalid;
   assign sb_axi_awaddr[31:0]         = sb_axi_addr[31:0];
   assign sb_axi_awid[pt.SB_BUS_TAG-1:0] = '0;
   assign sb_axi_awsize[2:0]          = sb_axi_size[2:0];
   assign sb_axi_awprot[2:0]          = 3'b001;
   assign sb_axi_awcache[3:0]         = 4'b1111;
   assign sb_axi_awregion[3:0]        = sb_axi_addr[31:28];
   assign sb_axi_awlen[7:0]           = '0;
   assign sb_axi_awburst[1:0]         = 2'b01;
   assign sb_axi_awqos[3:0]           = '0;
   assign sb_axi_awlock               = '0;

   assign sb_axi_wvalid       = sb_abmem_cmd_wvalid | sb_cmd_wvalid;
   assign sb_axi_wdata[63:0]  = ({64{(sb_axi_size[2:0] == 3'h0)}} & {8{sb_axi_wrdata[7:0]}}) |
                                ({64{(sb_axi_size[2:0] == 3'h1)}} & {4{sb_axi_wrdata[15:0]}}) |
                                ({64{(sb_axi_size[2:0] == 3'h2)}} & {2{sb_axi_wrdata[31:0]}}) |
                                ({64{(sb_axi_size[2:0] == 3'h3)}} & {sb_axi_wrdata[63:0]});
   assign sb_axi_wstrb[7:0]   = ({8{(sb_axi_size[2:0] == 3'h0)}} & (8'h1 << sb_axi_addr[2:0])) |
                                ({8{(sb_axi_size[2:0] == 3'h1)}} & (8'h3 << {sb_axi_addr[2:1],1'b0})) |
                                ({8{(sb_axi_size[2:0] == 3'h2)}} & (8'hf << {sb_axi_addr[2],2'b0})) |
                                ({8{(sb_axi_size[2:0] == 3'h3)}} & 8'hff);
   assign sb_axi_wlast        = '1;

   assign sb_axi_arvalid              = sb_abmem_cmd_arvalid | sb_cmd_arvalid;
   assign sb_axi_araddr[31:0]         = sb_axi_addr[31:0];
   assign sb_axi_arid[pt.SB_BUS_TAG-1:0] = '0;
   assign sb_axi_arsize[2:0]          = sb_axi_size[2:0];
   assign sb_axi_arprot[2:0]          = 3'b001;
   assign sb_axi_arcache[3:0]         = 4'b0;
   assign sb_axi_arregion[3:0]        = sb_axi_addr[31:28];
   assign sb_axi_arlen[7:0]           = '0;
   assign sb_axi_arburst[1:0]         = 2'b01;
   assign sb_axi_arqos[3:0]           = '0;
   assign sb_axi_arlock               = '0;

   // AXI Response signals
   assign sb_axi_bready = 1'b1;

   assign sb_axi_rready = 1'b1;
   assign sb_bus_rdata[63:0] = ({64{sb_axi_size == 3'h0}} & ((sb_axi_rdata[63:0] >>  8*sb_axi_addr[2:0]) & 64'hff))       |
                               ({64{sb_axi_size == 3'h1}} & ((sb_axi_rdata[63:0] >> 16*sb_axi_addr[2:1]) & 64'hffff))    |
                               ({64{sb_axi_size == 3'h2}} & ((sb_axi_rdata[63:0] >> 32*sb_axi_addr[2]) & 64'hffff_ffff)) |
                               ({64{sb_axi_size == 3'h3}} & sb_axi_rdata[63:0]);


`ifdef RV_ASSERT_ON
// assertion.
//  when the resume_ack is asserted then the dec_tlu_dbg_halted should be 0
   for (genvar i=0; i<pt.NUM_THREADS; i++) begin
      dm_check_resume_and_halted: assert property (@(posedge clk)  disable iff(~rst_l) (~dec_tlu_resume_ack[i] | ~dec_tlu_dbg_halted[i]));

      assert_b2b_haltreq: assert property (@(posedge clk) disable iff (~rst_l) (##1 dbg_halt_req[i] |=> ~dbg_halt_req[i]));
      assert_halt_resume_onehot: assert #0 ($onehot0({dbg_halt_req[i], dbg_resume_req[i]}));
   end

`endif
endmodule
