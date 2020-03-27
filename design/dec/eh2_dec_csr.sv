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
// eh2_dec_csr.sv
//
//
// Function: CSR decodes
//********************************************************************************


module eh2_dec_csr
import eh2_pkg::*;
#(
`include "eh2_param.vh"
)
  (


input logic [11:0] dec_csr_rdaddr_d,
input logic dec_csr_any_unq_d,
input logic dec_csr_wen_unq_d,
input logic dec_tlu_dbg_halted,

output logic dec_csr_legal_d,
output logic tlu_presync_d,
output logic tlu_postsync_d,

output eh2_csr_tlu_pkt_t tlu_csr_pkt_d
);


// file "csrdecode" is human readable file that has all of the CSR decodes defined and is part of git repo
// modify this file as needed

// to generate all the equations below from "csrdecode" except legal equation:
// 1) coredecode -in csrdecode > corecsrdecode.e
// 2) espresso -Dso -oeqntott corecsrdecode.e | addassign  > csrequations

// to generate the legal CSR equation below:
// 1) coredecode -in csrdecode -legal > csrlegal.e
// 2) espresso -Dso -oeqntott csrlegal.e | addassign  > csrlegal_equation

// coredecode -in csrdecode > corecsrdecode.e; espresso -Dso -oeqntott corecsrdecode.e | addassign  > csrequations; coredecode -in csrdecode -legal > csrlegal.e; espresso -Dso -oeqntott csrlegal.e | addassign  > csrlegal_equation


// insert "csrequations" here

logic csr_misa;
logic csr_mvendorid;
logic csr_marchid;
logic csr_mimpid;
logic csr_mhartid;
logic csr_mstatus;
logic csr_mtvec;
logic csr_mip;
logic csr_mie;
logic csr_mcyclel;
logic csr_mcycleh;
logic csr_minstretl;
logic csr_minstreth;
logic csr_mscratch;
logic csr_mepc;
logic csr_mcause;
logic csr_mscause;
logic csr_mtval;
logic csr_mrac;
logic csr_dmst;
logic csr_mdseac;
logic csr_meihap;
logic csr_meivt;
logic csr_meipt;
logic csr_meicurpl;
logic csr_meicidpl;
logic csr_dcsr;
logic csr_mcgc;
logic csr_mfdc;
logic csr_dpc;
logic csr_mtsel;
logic csr_mtdata1;
logic csr_mtdata2;
logic csr_mhpmc3;
logic csr_mhpmc4;
logic csr_mhpmc5;
logic csr_mhpmc6;
logic csr_mhpmc3h;
logic csr_mhpmc4h;
logic csr_mhpmc5h;
logic csr_mhpmc6h;
logic csr_mhpme3;
logic csr_mhpme4;
logic csr_mhpme5;
logic csr_mhpme6;
logic csr_mitctl0;
logic csr_mitctl1;
logic csr_mitb0;
logic csr_mitb1;
logic csr_mitcnt0;
logic csr_mitcnt1;
logic csr_mpmc;
logic csr_mcpc;
logic csr_meicpct;
logic csr_mdeau;
logic csr_micect;
logic csr_miccmect;
logic csr_mdccmect;
logic csr_mcountinhibit;
logic csr_mfdht;
logic csr_mfdhs;
logic csr_dicawics;
logic csr_dicad0h;
logic csr_dicad0;
logic csr_dicad1;
logic csr_dicago;
logic csr_mhartnum;
logic csr_mhartstart;
logic csr_mnmipdel;
logic presync;
logic postsync;
logic glob;

assign csr_misa = (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[0]);

assign csr_mvendorid = (dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[7]
    &!dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]);

assign csr_marchid = (dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[7]
    &dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mimpid = (dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[6]
    &dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]);

assign csr_mhartid = (dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[7]
    &dec_csr_rdaddr_d[2]);

assign csr_mstatus = (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[0]);

assign csr_mtvec = (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[0]);

assign csr_mip = (!dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[2]);

assign csr_mie = (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[0]);

assign csr_mcyclel = (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[7]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
    &!dec_csr_rdaddr_d[1]);

assign csr_mcycleh = (dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]);

assign csr_minstretl = (!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
    &dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_minstreth = (!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[7]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
    &dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mscratch = (!dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mepc = (!dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[1]
    &dec_csr_rdaddr_d[0]);

assign csr_mcause = (!dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]
    &dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mscause = (dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]);

assign csr_mtval = (!dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[1]
    &dec_csr_rdaddr_d[0]);

assign csr_mrac = (!dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]);

assign csr_dmst = (!dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]);

assign csr_mdseac = (dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]);

assign csr_meihap = (dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[3]);

assign csr_meivt = (!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[6]
    &dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]
    &!dec_csr_rdaddr_d[0]);

assign csr_meipt = (dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[1]
    &dec_csr_rdaddr_d[0]);

assign csr_meicurpl = (!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[6]
    &dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]);

assign csr_meicidpl = (dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[6]
    &dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]);

assign csr_dcsr = (dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[0]);

assign csr_mcgc = (dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[0]);

assign csr_mfdc = (dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]);

assign csr_dpc = (dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[0]);

assign csr_mtsel = (dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mtdata1 = (dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[0]);

assign csr_mtdata2 = (dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[1]);

assign csr_mhpmc3 = (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[7]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
    &dec_csr_rdaddr_d[0]);

assign csr_mhpmc4 = (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[7]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]
    &!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mhpmc5 = (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[7]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[1]
    &dec_csr_rdaddr_d[0]);

assign csr_mhpmc6 = (!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]
    &dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mhpmc3h = (dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]
    &dec_csr_rdaddr_d[0]);

assign csr_mhpmc4h = (dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]
    &!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mhpmc5h = (dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]
    &dec_csr_rdaddr_d[0]);

assign csr_mhpmc6h = (dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]
    &dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mhpme3 = (!dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
    &dec_csr_rdaddr_d[0]);

assign csr_mhpme4 = (dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]
    &!dec_csr_rdaddr_d[0]);

assign csr_mhpme5 = (dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]
    &dec_csr_rdaddr_d[0]);

assign csr_mhpme6 = (dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]
    &!dec_csr_rdaddr_d[0]);

assign csr_mitctl0 = (dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mitctl1 = (dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[3]
    &dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]);

assign csr_mitb0 = (dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[0]);

assign csr_mitb1 = (dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]
    &dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mitcnt0 = (dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[0]);

assign csr_mitcnt1 = (dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[2]
    &!dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]);

assign csr_mpmc = (dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
    &dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]);

assign csr_mcpc = (dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]);

assign csr_meicpct = (dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[6]
    &dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mdeau = (!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[7]
    &dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[3]);

assign csr_micect = (dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_miccmect = (dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[0]);

assign csr_mdccmect = (dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]);

assign csr_mcountinhibit = (!dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
    &!dec_csr_rdaddr_d[0]);

assign csr_mfdht = (dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]
    &dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_mfdhs = (dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[2]
    &dec_csr_rdaddr_d[0]);

assign csr_dicawics = (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]
    &!dec_csr_rdaddr_d[0]);

assign csr_dicad0h = (dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[4]
    &dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]);

assign csr_dicad0 = (dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[4]
    &dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]);

assign csr_dicad1 = (dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign csr_dicago = (dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]);

assign csr_mhartnum = (dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[2]);

assign csr_mhartstart = (dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]);

assign csr_mnmipdel = (dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]);

assign presync = (dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[7]
    &dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]) | (dec_csr_rdaddr_d[11]
    &!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
    &dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]) | (dec_csr_rdaddr_d[11]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[1]
    &!dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
    &dec_csr_rdaddr_d[1]);

assign postsync = (dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[2]
    &dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[1]
    &dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[0]) | (
    !dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[1]
    &dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[1]) | (!dec_csr_rdaddr_d[11]
    &!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[0]) | (
    dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
    &dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[7]
    &dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]);

assign glob = (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[6]
    &dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]
    &!dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[2]) | (dec_csr_rdaddr_d[6]
    &dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]) | (
    dec_csr_rdaddr_d[10]&!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[2]) | (dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[7]
    &dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]
    &!dec_csr_rdaddr_d[1]);

logic conditionally_illegal, valid_csr;
logic legal;

   // insert "csrlegal_equation" here
assign legal = (!dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
    &dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]
    &dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]
    &dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]
    &!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[7]
    &!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[1]) | (!dec_csr_rdaddr_d[11]
    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[11]
    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[0]) | (
    dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]
    &!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]
    &dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]
    &dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]) | (
    !dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
    &dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]
    &dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[2]
    &!dec_csr_rdaddr_d[1]) | (dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[9]
    &dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]
    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[2]) | (!dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]
    &!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]) | (dec_csr_rdaddr_d[11]
    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[7]
    &!dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]
    &!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[1]
    &!dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[9]
    &dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[2]&dec_csr_rdaddr_d[1]) | (!dec_csr_rdaddr_d[11]
    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[1]&dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[11]
    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]
    &dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]) | (!dec_csr_rdaddr_d[11]
    &dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[1]) | (
    !dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
    &dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
    &dec_csr_rdaddr_d[2]) | (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[2]) | (!dec_csr_rdaddr_d[11]
    &dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&dec_csr_rdaddr_d[1]) | (
    dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
    &dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[1]
    &!dec_csr_rdaddr_d[0]) | (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[1]) | (!dec_csr_rdaddr_d[11]
    &dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]) | (
    !dec_csr_rdaddr_d[11]&dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]
    &dec_csr_rdaddr_d[8]&dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]
    &dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[2]&!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]
    &dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]
    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &!dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[2]) | (
    dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &dec_csr_rdaddr_d[7]&dec_csr_rdaddr_d[6]&!dec_csr_rdaddr_d[5]
    &!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]&!dec_csr_rdaddr_d[1]
    &!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[7]
    &!dec_csr_rdaddr_d[5]&!dec_csr_rdaddr_d[4]&!dec_csr_rdaddr_d[3]
    &!dec_csr_rdaddr_d[1]&!dec_csr_rdaddr_d[0]) | (!dec_csr_rdaddr_d[11]
    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[3]) | (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[3]) | (!dec_csr_rdaddr_d[11]
    &!dec_csr_rdaddr_d[10]&dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]
    &!dec_csr_rdaddr_d[7]&!dec_csr_rdaddr_d[6]&dec_csr_rdaddr_d[5]
    &dec_csr_rdaddr_d[4]) | (dec_csr_rdaddr_d[11]&!dec_csr_rdaddr_d[10]
    &dec_csr_rdaddr_d[9]&dec_csr_rdaddr_d[8]&!dec_csr_rdaddr_d[6]
    &!dec_csr_rdaddr_d[5]&dec_csr_rdaddr_d[4]);



//
   assign tlu_presync_d = presync & dec_csr_any_unq_d & ~dec_csr_wen_unq_d;
   assign tlu_postsync_d = postsync & dec_csr_any_unq_d;

   // allow individual configuration of these features
   assign conditionally_illegal = ((csr_mitcnt0 | csr_mitcnt1 | csr_mitb0 | csr_mitb1 | csr_mitctl0 | csr_mitctl1) & ~pt.TIMER_LEGAL_EN) |
                                  (csr_meicpct & pt.FAST_INTERRUPT_REDIRECT);

   assign valid_csr = ( legal &
                        // not a debug only csr during running mode
                        (~(csr_dcsr | csr_dpc | csr_dmst | csr_dicawics | csr_dicad0 | csr_dicad0h | csr_dicad1 | csr_dicago) | dec_tlu_dbg_halted) &
                        // not conditionally illegal based on configuration
                        ~conditionally_illegal
                        );

   assign dec_csr_legal_d = ( dec_csr_any_unq_d &
                              valid_csr &          // of a valid CSR
                              ~(dec_csr_wen_unq_d & (csr_mvendorid | csr_marchid | csr_mimpid | csr_mhartid |
                                                     csr_mdseac | csr_meihap | csr_mhartnum)) // that's not a write to a RO CSR
                              );



   assign tlu_csr_pkt_d.csr_misa = csr_misa;
   assign tlu_csr_pkt_d.csr_mvendorid = csr_mvendorid;
   assign tlu_csr_pkt_d.csr_marchid = csr_marchid;
   assign tlu_csr_pkt_d.csr_mimpid = csr_mimpid;
   assign tlu_csr_pkt_d.csr_mhartid = csr_mhartid;
   assign tlu_csr_pkt_d.csr_mstatus = csr_mstatus;
   assign tlu_csr_pkt_d.csr_mtvec = csr_mtvec;
   assign tlu_csr_pkt_d.csr_mip = csr_mip;
   assign tlu_csr_pkt_d.csr_mie = csr_mie;
   assign tlu_csr_pkt_d.csr_mcyclel = csr_mcyclel;
   assign tlu_csr_pkt_d.csr_mcycleh = csr_mcycleh;
   assign tlu_csr_pkt_d.csr_minstretl = csr_minstretl;
   assign tlu_csr_pkt_d.csr_minstreth = csr_minstreth;
   assign tlu_csr_pkt_d.csr_mscratch = csr_mscratch;
   assign tlu_csr_pkt_d.csr_mepc = csr_mepc;
   assign tlu_csr_pkt_d.csr_mcause = csr_mcause;
   assign tlu_csr_pkt_d.csr_mscause = csr_mscause;
   assign tlu_csr_pkt_d.csr_mtval = csr_mtval;
   assign tlu_csr_pkt_d.csr_mrac = csr_mrac;
   assign tlu_csr_pkt_d.csr_dmst = csr_dmst;
   assign tlu_csr_pkt_d.csr_mdseac = csr_mdseac;
   assign tlu_csr_pkt_d.csr_meihap = csr_meihap;
   assign tlu_csr_pkt_d.csr_meivt = csr_meivt;
   assign tlu_csr_pkt_d.csr_meipt = csr_meipt;
   assign tlu_csr_pkt_d.csr_meicurpl = csr_meicurpl;
   assign tlu_csr_pkt_d.csr_meicidpl = csr_meicidpl;
   assign tlu_csr_pkt_d.csr_dcsr = csr_dcsr;
   assign tlu_csr_pkt_d.csr_mcgc = csr_mcgc;
   assign tlu_csr_pkt_d.csr_mfdc = csr_mfdc;
   assign tlu_csr_pkt_d.csr_dpc = csr_dpc;
   assign tlu_csr_pkt_d.csr_mtsel = csr_mtsel;
   assign tlu_csr_pkt_d.csr_mtdata1 = csr_mtdata1;
   assign tlu_csr_pkt_d.csr_mtdata2 = csr_mtdata2;
   assign tlu_csr_pkt_d.csr_mhpmc3 = csr_mhpmc3;
   assign tlu_csr_pkt_d.csr_mhpmc4 = csr_mhpmc4;
   assign tlu_csr_pkt_d.csr_mhpmc5 = csr_mhpmc5;
   assign tlu_csr_pkt_d.csr_mhpmc6 = csr_mhpmc6;
   assign tlu_csr_pkt_d.csr_mhpmc3h = csr_mhpmc3h;
   assign tlu_csr_pkt_d.csr_mhpmc4h = csr_mhpmc4h;
   assign tlu_csr_pkt_d.csr_mhpmc5h = csr_mhpmc5h;
   assign tlu_csr_pkt_d.csr_mhpmc6h = csr_mhpmc6h;
   assign tlu_csr_pkt_d.csr_mhpme3 = csr_mhpme3;
   assign tlu_csr_pkt_d.csr_mhpme4 = csr_mhpme4;
   assign tlu_csr_pkt_d.csr_mhpme5 = csr_mhpme5;
   assign tlu_csr_pkt_d.csr_mhpme6 = csr_mhpme6;
   assign tlu_csr_pkt_d.csr_mitctl0 = csr_mitctl0;
   assign tlu_csr_pkt_d.csr_mitctl1 = csr_mitctl1;
   assign tlu_csr_pkt_d.csr_mitb0 = csr_mitb0;
   assign tlu_csr_pkt_d.csr_mitb1 = csr_mitb1;
   assign tlu_csr_pkt_d.csr_mitcnt0 = csr_mitcnt0;
   assign tlu_csr_pkt_d.csr_mitcnt1 = csr_mitcnt1;
   assign tlu_csr_pkt_d.csr_mpmc = csr_mpmc;
   assign tlu_csr_pkt_d.csr_mcountinhibit = csr_mcountinhibit;
   assign tlu_csr_pkt_d.csr_mcpc = csr_mcpc;
   assign tlu_csr_pkt_d.csr_meicpct = csr_meicpct;
   assign tlu_csr_pkt_d.csr_mdeau = csr_mdeau;
   assign tlu_csr_pkt_d.csr_micect = csr_micect;
   assign tlu_csr_pkt_d.csr_miccmect = csr_miccmect;
   assign tlu_csr_pkt_d.csr_mdccmect = csr_mdccmect;
   assign tlu_csr_pkt_d.csr_dicawics = csr_dicawics;
   assign tlu_csr_pkt_d.csr_dicad0h = csr_dicad0h;
   assign tlu_csr_pkt_d.csr_dicad0 = csr_dicad0;
   assign tlu_csr_pkt_d.csr_dicad1 = csr_dicad1;
   assign tlu_csr_pkt_d.csr_dicago = csr_dicago;
   assign tlu_csr_pkt_d.csr_mfdht = csr_mfdht;
   assign tlu_csr_pkt_d.csr_mfdhs = csr_mfdhs;
   assign tlu_csr_pkt_d.csr_mhartnum = csr_mhartnum;
   assign tlu_csr_pkt_d.csr_mhartstart = csr_mhartstart;
   assign tlu_csr_pkt_d.csr_mnmipdel = csr_mnmipdel;
   assign tlu_csr_pkt_d.presync = presync;
   assign tlu_csr_pkt_d.postsync = postsync;
   assign tlu_csr_pkt_d.glob = glob;
   assign tlu_csr_pkt_d.legal = legal;

endmodule
