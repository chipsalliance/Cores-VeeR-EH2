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

// Assembly code for Multithreaded Hello World
// Not using only ALU ops for creating the string


#include "defines.h"

#define STDOUT 0xd0580000


// Get TID in register (default = a0)
.macro get_tid reg=a0
    csrr \reg, mhartid
    andi \reg,\reg, 0xf
.endm

// Jump to threaded label based on hartid (default label = thread)
.macro fork targ=thread reg=a0
    get_tid \reg
    bne \reg, x0, \targ
.endm

.macro set_lock id=1
    li a0, \id
    li a1, LOCK_SEMAPHORE
    amoswap.w.aq x0, a0, (a1)
.endm

.macro unlock
    li a1, LOCK_SEMAPHORE
    sw x0, 0 (a1)
.endm

.macro get_lock id=1
    li a1, LOCK_SEMAPHORE
1:
    lw a0, 0 (a1)
    bnez    a0, 1b
.endm

    .equ mhartstart,0x7fc
    .equ LOCK_SEMAPHORE, RV_DCCM_EADR - 3

// Code to execute
.section .text
.global _start
_start:

    // Clear minstret
    csrw minstret, zero
    csrw minstreth, zero

    // Set up MTVEC - not expecting to use it though
    li x1, RV_ICCM_SADR
    csrw mtvec, x1

    fork bypass_init1
    // Enable Caches in MRAC
    li x1, 0x5f555555
    csrw 0x7c0, x1

    unlock              // init Semaphore in DCCM
    set_lock

    csrwi mhartstart, 3 // enable hart1
    li x3, STDOUT
    la x4, hw_data
    j       loop

bypass_init1:
    // Load string from hw_data
    // and write to stdout address
    li x3, STDOUT
    la x4, hw_data1
    get_lock

loop:
   lb x5, 0(x4)
   sb x5, 0(x3)
   addi x4, x4, 1
   bnez x5, loop
   unlock
#if (RV_NUM_THREADS > 1)
   fork _finish
here:
   j    here
#endif
// Write 0xff to STDOUT for TB to termiate test.
_finish:
    li x3, STDOUT
    addi x5, x0, 0xff
    sb x5, 0(x3)
    beq x0, x0, _finish
.rept 100
    nop
.endr

.data
hw_data:
.ascii "----------------------------------------\n"
.ascii "Hello World from SweRV EH2 hart0 @WDC !!\n"
.ascii "----------------------------------------\n"
.byte 0

hw_data1:
.ascii "----------------------------------------\n"
.ascii "Hello World from SweRV EH2 hart1 @WDC !!\n"
.ascii "----------------------------------------\n"
.byte 0
