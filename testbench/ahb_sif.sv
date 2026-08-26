// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Western Digital Corporation or its affiliates.
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
//
`ifdef RV_BUILD_AHB_LITE

module ahb_sif (
input logic [`RV_BUS_WIDTH-1:0] HWDATA,
input logic HCLK,
input logic HSEL,
input logic [3:0] HPROT,
input logic HWRITE,
input logic [1:0] HTRANS,
input logic [2:0] HSIZE,
input logic HREADY,
input logic HRESETn,
input logic [`RV_XLEN-1:0] HADDR,
input logic [2:0] HBURST,

output logic HREADYOUT,
output logic HRESP,
output logic [`RV_BUS_WIDTH-1:0] HRDATA
);

parameter MAILBOX_ADDR = {{(`RV_XLEN-32){1'b0}}, 32'hD0580000};

logic write;
logic [`RV_XLEN-1:0] laddr, addr;
logic [`RV_BUS_BYTES-1:0] strb_lat;
logic [`RV_BUS_WIDTH-1:0] rdata;

bit [7:0] mem [bit[`RV_XLEN-1:0]];
bit [7:0] wscnt;
int dws = 0;
int iws = 0;
bit dws_rand;
bit iws_rand;
bit ok;

// Wires
wire [`RV_BUS_WIDTH-1:0] WriteData = HWDATA;
wire [`RV_BUS_BYTES-1:0] strb =         HSIZE == 3'b000 ? `RV_BUS_BYTES'('h1) << HADDR[`RV_BUS_ADDR_OFF-1:0] :
                                        HSIZE == 3'b001 ? `RV_BUS_BYTES'('h3) << {HADDR[`RV_BUS_ADDR_OFF-1:1],1'b0} :
                                        HSIZE == 3'b010 ? `RV_BUS_BYTES'('hf) << {HADDR[`RV_BUS_ADDR_OFF-1:2],2'b0} :
                                        HSIZE == 3'b011 ? (`RV_BUS_WIDTH == 64 ? `RV_BUS_BYTES'('hff) : `RV_BUS_BYTES'('hff) << {HADDR[`RV_BUS_ADDR_OFF-1], 3'b0}) :
                                        HSIZE == 3'b100 & `RV_BUS_WIDTH == 128 ? `RV_BUS_BYTES'('hffff) : 0;

wire mailbox_write = write && HSEL && HREADY && laddr==MAILBOX_ADDR;


initial begin
    if ($value$plusargs("iws=%d", iws));
    if ($value$plusargs("dws=%d", dws));
    dws_rand = dws < 0;
    iws_rand = iws < 0;
end

always @ (negedge HCLK ) begin
    if(HREADY)
        addr = HADDR;
    if (write & HREADY) begin
        for (int i=0; i<`RV_BUS_BYTES; i++) begin
            if (strb_lat[i]) mem[{laddr[`RV_XLEN-1:3], 3'(i)}] = HWDATA[i*8 +: 8];
        end
    end
    if(HREADY & HSEL & |HTRANS) begin
    `ifdef VERILATOR
        if(iws_rand & ~HPROT[0])
            iws = $random & 15;
        if(dws_rand & HPROT[0])
            dws = $random & 15;
    `else
        if(iws_rand & ~HPROT[0])
            ok = std::randomize(iws) with {iws dist {0:=10, [1:3]:/2, [4:15]:/1};};
        if(dws_rand & HPROT[0])
            ok = std::randomize(dws) with {dws dist {0:=10, [1:3]:/2, [4:15]:/1};};
    `endif
    end
end

assign HRDATA = HREADY ? rdata : ~rdata;
assign HREADYOUT = wscnt == 0;
assign HRESP = 0;

always @(posedge HCLK or negedge HRESETn) begin
    if(~HRESETn) begin
        laddr <= {`RV_XLEN{1'b0}};
        write <= 1'b0;
        rdata <= '0;
        wscnt <= 0;
    end
    else begin
        if(HREADY & HSEL) begin
            laddr <= HADDR;
            write <= HWRITE & |HTRANS;
            if(|HTRANS & ~HWRITE) begin
                for (int i=0; i<`RV_BUS_BYTES; i++) begin
                    rdata[i*8 +: 8] = mem[{addr[`RV_XLEN-1:3], 3'(i)}];
                end
            end
            strb_lat <= strb;
        end
    end
    if(HREADY & HSEL & |HTRANS)
        wscnt <= HPROT[0] ? dws[7:0] : iws[7:0];
    else if(wscnt != 0)
        wscnt <= wscnt-1;
end

endmodule
`endif

`ifdef RV_BUILD_AXI4
module axi_slv #(TAGW=1) (
input                           aclk,
input                           rst_l,
input                           arvalid,
output reg                      arready,
input [`RV_XLEN-1:0]            araddr,
input [TAGW-1:0]                arid,
input [7:0]                     arlen,
input [1:0]                     arburst,
input [2:0]                     arsize,

output reg                      rvalid,
input                           rready,
output reg [`RV_BUS_WIDTH-1:0]  rdata,
output reg [1:0]                rresp,
output reg [TAGW-1:0]           rid,
output                          rlast,

input                           awvalid,
output                          awready,
input [`RV_XLEN-1:0]            awaddr,
input [TAGW-1:0]                awid,
input [7:0]                     awlen,
input [1:0]                     awburst,
input [2:0]                     awsize,

input [`RV_BUS_WIDTH-1:0]       wdata,
input [`RV_BUS_BYTES-1:0]  wstrb,
input                           wvalid,
output                          wready,

output  reg                     bvalid,
input                           bready,
output reg [1:0]                bresp,
output reg [TAGW-1:0]           bid
);

parameter MAILBOX_ADDR = {{(`RV_XLEN-32){1'b0}},32'hD0580000};
parameter MEM_SIZE_DW = 8192;

bit [7:0] mem [bit[`RV_XLEN-1:0]];
bit [`RV_BUS_WIDTH-1:0] memdata;
wire [`RV_BUS_WIDTH-1:0] WriteData;
wire mailbox_write;

wire[`RV_XLEN-1:0] raddr, waddr;

assign mailbox_write = awvalid && awaddr==MAILBOX_ADDR && rst_l;
assign WriteData = wdata;

always @ ( posedge aclk or negedge rst_l) begin
    if(!rst_l) begin
        rvalid  <= 0;
        bvalid  <= 0;
    end
    else begin
        bid     <= awid;
        rid     <= arid;
        rvalid  <= arvalid;
        bvalid  <= awvalid;
        rdata   <= memdata;
    end
end

assign raddr = {araddr[`RV_XLEN-1:`RV_BUS_ADDR_OFF], {`RV_BUS_ADDR_OFF{1'b0}}};
assign waddr = {awaddr[`RV_XLEN-1:`RV_BUS_ADDR_OFF], {`RV_BUS_ADDR_OFF{1'b0}}};

always @ ( negedge aclk) begin
    if(arvalid) begin
        for (int i=0; i<`RV_BUS_BYTES; i++) begin
            memdata[i*8 +: 8] = mem[raddr+i];
        end
    end

    if(awvalid) begin
        for (int i=0; i<`RV_BUS_BYTES; i++) begin
            if(wstrb[i]) mem[waddr+i] = wdata[i*8 +: 8];
        end
    end
end

assign arready = 1'b1;
assign awready = 1'b1;
assign wready  = 1'b1;
assign rresp   = 2'b0;
assign bresp   = 2'b0;
assign rlast   = 1'b1;

endmodule
`endif
