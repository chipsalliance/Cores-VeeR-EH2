(chap-overview)=
# VeeR EH2 Core Overview

This chapter provides a high-level overview of the VeeR EH2 core and core complex.
VeeR EH2 is a machinemode (M-mode) only, 32-bit CPU core which supports RISC-V's integer (I), compressed instruction (C), multiplication and division (M), atomic (A), and instruction-fetch fence, CSR, and subset of bit manipulation instructions (Z) extensions.
The core is a 9-stage, dual-threaded, dual-issue, superscalar, mostly in-order pipeline with some out-oforder execution capability.

## Features

The VeeR EH2 core complex's feature set includes:

- RV32IMAC-compliant RISC-V core with branch predictor
- Single- (AXI4 or AHB-Lite) or dual-threaded (AXI4 only) core selected by build argument
- Optional instruction and data closely-coupled memories with ECC protection
- Optional 2- or 4-way set-associative instruction cache with parity or ECC protection (32- or 64-byte line size)
- Optional programmable interrupt controller supporting up to 255 external interrupts
- Four system bus interfaces for instruction fetch, data accesses, debug accesses, and external DMA accesses to closely-coupled memories (configurable as 64-bit AXI4 or AHB-Lite)
- Core debug unit compliant with the RISC-V Debug specification [[3]](intro.md#ref-3)
- 1.2GHz target frequency (for 16nm technology node)

## Core Complex

{ref}`Figure 2-1 <figure-2-1>` depicts the core complex and its functional blocks which are described further in Section 2.3.

```{figure} img/core_complex.png
:name: figure-2-1

VeeR EH2 Core Complex
```

## Functional Blocks

The VeeR EH2 core complex's functional blocks are described in the following sections in more detail.

### Core

{ref}`Figure 2-2 <figure-2-2>` depicts the superscalar, dual-threaded, dual-issue 9-stage core pipeline supporting four arithmetic logic units (ALUs) labeled EX1 and EX4 in two pipelines I0 and I1, one load/store pipeline, one 3-cycle latency multiplier pipeline, and one out-of-pipeline 34-cycle latency divider.
There are three stall points in the pipeline: 'Fetch1', 'Align', and 'Decode'.
In the 'Align' stage, instructions are formed from 3 fetch buffers.
In the 'Decode' stage, up to 2 instructions from 4 instruction buffers are decoded.
In the 'Commit' stage, up to 2 instructions per cycle are committed.
Finally, in the 'Writeback' stage, the architectural registers are updated.

```{figure} img/core_pipeline.png
:name: figure-2-2

VeeR EH2 Core Pipeline
```

## Standard Extensions

The VeeR EH2 core implements the following RISC-V standard extensions:

:::{list-table} VeeR EH2's RISC-V Standard Extensions
:name: table-2-1
:header-rows: 1

* - Extension
  - Description
  - References
* - M
  - Integer multiplication and division
  - Chapter 7 in [[1]](intro.md#ref-1)
* - A
  - Atomic instructions
  - Chapter 8 in [[1]](intro.md#ref-1)
* - C
  - Compressed instructions
  - Chapter 16 in [[1]](intro.md#ref-1)
* - Zicsr
  - Control and status register (CSR) instructions
  - Chapter 9 in [[1]](intro.md#ref-1)
* - Zifencei
  - Instruction-fetch fence
  - Chapter 3 in [[1]](intro.md#ref-1)
* - 'Frozen': (not expected to change)
  - Bit manipulation instructions
  - Chapter 2 in [[4]](intro.md#ref-4)
* - Zba [^1] (address calculation)
  -
  - Chapter 2 in [[4]](intro.md#ref-4)
* - Zbb [^2] (base)
  -
  - Chapter 2 in [[4]](intro.md#ref-4)
* - Zbc [^3] (carry-less multiply)
  -
  - Chapter 2 in [[4]](intro.md#ref-4)
* - Zbs [^4] (single-bit)
  -
  - Chapter 2 in [[4]](intro.md#ref-4)
* - 'Stable': (may still change)
  -
  - Chapter 2 in [[4]](intro.md#ref-4)
* - Zbe [^5] (bit compress/ decompress)
  -
  - Chapter 2 in [[4]](intro.md#ref-4)
* - Zbf [^6] (bit-field place)
  -
  - Chapter 2 in [[4]](intro.md#ref-4)
* - Zbp [^7] (bit permutation)
  -
  - Chapter 2 in [[4]](intro.md#ref-4)
* - Zbr [^8] (CRC)
  -
  - Chapter 2 in [[4]](intro.md#ref-4)
:::


[^1]: List of Zba instructions (as of 1/20/21, “frozen”): sh1add, sh2add, sh3add
[^2]: List of Zbb instructions (as of 1/20/21, “frozen”): clz, ctz, cpop, min, minu, max, maxu, sext.b, sext.h, zext.h, andn, orn, xnor, rol, ror, rori, rev8, orc.b
[^3]: List of Zbc instructions (as of 1/20/21, “frozen”): clmul, clmulh, clmulr
[^4]: List of Zbs instructions (as of 1/20/21, “frozen”): bset, bseti, bclr, bclri, binv, binvi, bext, bexti
[^5]: List of Zbe instructions (as of 1/20/21, “stable”): bcompress, bdecompress, pack, packh
[^6]: List of Zbf instructions (as of 1/20/21, “stable”): bfp, pack, packh
[^7]: List of Zbp instructions (as of 1/20/21, “stable”): andn, orn, xnor, pack, packu, packh, rol, ror, rori, grev, grevi, gorc, gorci, shfl, shfli, unshfl, unshfli, xperm.n, xperm.b, xperm.h
[^8]: List of Zbr instructions (as of 1/20/21, “stable”): crc32.b, crc32c.b, crc32.h, crc32c.h, crc32.w, crc32c.w
