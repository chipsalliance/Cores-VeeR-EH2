// SPDX-License-Identifier: Apache-2.0

#define STDOUT 0xd0580000

// CSR writes replace the retirement increment. Reads return the value before
// the read instruction retires, so only the NOPs contribute to each result.
.macro check_minstret initial, nops, expected, upper=0
    li t0, \initial
    csrw minstreth, zero
    csrw minstret, t0
    .rept \nops
        nop
    .endr
    csrr t1, minstret
    li t2, \expected
    bne t1, t2, fail
    csrr t1, minstreth
    li t2, \upper
    bne t1, t2, fail
.endm

.section .text
.global _start
.option norvc
_start:
    csrw mcountinhibit, zero

    // No carry, low-byte carries, a carry into bit 16, and RV32 overflow.
    check_minstret 252, 2, 0xfe
    check_minstret 254, 2, 0x100
    check_minstret 510, 2, 0x200
    check_minstret 65534, 2, 0x10000
    check_minstret 0xffffffff, 1, 0, 1

    // Repeated carries without rewriting the counter.
    check_minstret 0, 512, 0x200

    // Inhibit retirement counting while initializing the counter, then enable
    // it again and cross the low-byte boundary.
    csrwi mcountinhibit, 4
    li t0, 254
    csrw minstreth, zero
    csrw minstret, t0
    nop
    nop
    csrr t1, minstret
    bne t1, t0, fail
    csrwi mcountinhibit, 0
    nop
    nop
    csrr t1, minstret
    li t2, 0x100
    bne t1, t2, fail

    li t0, STDOUT
    li t1, 0xff
    sb t1, 0(t0)
1:
    j 1b

fail:
    li t0, STDOUT
    li t1, 1
    sb t1, 0(t0)
2:
    j 2b
