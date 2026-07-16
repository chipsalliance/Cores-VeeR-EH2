![CHIPS Alliance logo](img/chips_alliance_logo.png)

(chap-intro)=
# RISC-V VeeR EH2 Programmer's Reference Manual


**Revision 1.4**


**December 22, 2022**

```
SPDX-License-Identifier: Apache-2.0 Copyright © 2022 CHIPS Alliance.
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and limitations under the License.
```

## Document Revision History

:::{list-table}
:header-rows: 1

* - Revision
  - Date
  - Contents
* - 1.0
  - Jan 23, 2020
  - Initial revision
* - 1.1
  - Mar 4, 2020
  - * Added note that `mscause` values are subject to change (Section 3.8.5)
    * Added additional details about behavior of atomic instructions (Section 3.10)
    * Added note that uninitialized DCCM may cause loads to get incorrect data (Section 4.4)
    * Added Debug Module reset description (Section 16.3.2)
    * Updated port list (Table 17-1):
      * Added `dbg_rst_l` signal
      * Added footnote clarifying trace port signals
    * Added 'Compliance Test Suite Failures' chapter (Chapter 19)
* - 1.2
  - Mar 28, 2020
  - * Fixed note how writing illegal value to `mrac` register is handled by hardware (Section 3.8.1)
    * Removed note that `mscause` values are subject to change (Section 3.8.5)
    * Updated `mscause` values (Table 3-10)
    * Added Internal Timers chapter and references throughout document (Chapter 6)
    * Incremented `mimpid` register value from `1` to `2` (Table 14-1)
* - 1.3
  - Mar 11, 2022
  - * Updated versions of RISC-V Base ISA [[1]](intro.md#ref-1) and Privileged [[2]](intro.md#ref-2) and link to RISC-V Debug [[3]](intro.md#ref-3) specifications (Reference Documents)
    * Added RISC-V Bit-manipulation sub-extensions (Reference Documents, Sections 2.1 and 2.4, and Table 9-1)
    * Removed note that AHB-Lite bus protocol is not supported (Chapter 2)
    * Added notes that dual-threading only supported with AXI4 buses (Section 2.1 and Chapter 3)
    * Updated note regarding priority of simultaneous store and non-blocking load bus errors (Section 3.7.1)
    * Added paragraph that both threads receive correctable error local interrupt indication (Section 3.7.2)
    * Added footnote that misaligned accesses to side-effect regions trigger a misaligned exception instead of the recommended access fault exception (Table 3-3)
    * Added note to `mdseac` register description clarifying captured address (Section 3.8.3)
    * Clarified that `mscause` value of `0` indicates no additional information available (Section 3.8.5)
    * Fixed register name and added cross-reference (Footnote 19)
    * Added footnote that load/store access crossing upper boundary of DCCM or PIC memory range report base address of access in `mtval` register (Footnote 21)
    * Updated note that atomic operations only supported for cores with DCCM (Section 3.10)
    * Added description of SoC access expectation (Section 3.13)
    * Added note that NMIs are fatal (Section 3.17)
    * Clarified that correctable error counter/threshold registers are always instantiated (Sections 4.5.1, 4.5.2, and 4.5.3)
    * Added note that `mitcnt0` / 1 register is not cleared if write to it coincides with internal timer interrupt (Section 6.4.1)
    * Clarified note that debug single-step action is delayed while MPC debug halted (Section 7.4)
    * Added cross-references to debug CSR descriptions (Table 7-2, Table 7-4, Table 14-2, and Sections 9.4 and 16.3.4)
    * Added note that debug single-stepping stays pending while MPC debug halted (Section 7.5.1.1)
    * Removed note that PMU halt or run request may not be acknowledged if already in requested activity state (Section 7.5.2.1)
    * Amended `debug_mode_status` signal description (Table 7-4)
    * Added note that `mpc_debug_run_req` is required to exit Debug Mode if entered after reset using `mpc_reset_run_req` (Section 7.5.2.2)
    * Added PIC I/O power reduction feature description (Sections 8.1, 8.9, 8.12.3, and 8.12.4 and Table 12-2)
    * Added note that spurious interrupts may be captured for disabled external interrupts (Section 8.3.2)
    * Added note that edge-triggered interrupt lines must be tied off to inactive state (Section 8.3.2)
    * Fixed gateway initialization macro example (Section 8.15.2)
    * Added note that `mtime` and `mtimecmp` registers must be provided by SoC (Section 9.2.1)
    * Changed value when writing unsupported event number to `mhpmevent3-6` registers to `0` (Section 9.5)
    * Added notes that both threads must be in debug halt state when I-cache control registers are accessed (Sections 10.5.1 - 10.5.5)
    * Added note that *index* field does not have WARL behavior (Table 10-1)
    * Added Debug Support chapter (Chapter 11)
    * Added 'trace disable' bit to `mfdc` register (Table 12-1)
    * Clarified effect of sepd bit of `mfdc` register (Table 12-1)
    * Updated Machine Information registers (Table 14-1):
      * Updated `misa` register that value depends on atomics support build argument
      * Incremented `mimpid` register value from `2` to `3`
    * Added note regarding physical design considerations for `rst_l` signal (Section 16.3.1)
    * Updated 'Reset to Debug-Mode' description (Section 16.3.4)
    * Updated port list (Table 17-1):
      * Updated trace port interrupt/exception signaling to new optimized scheme
      * Updated `scan_mode` and `mbist_mode` signal descriptions
    * Added erratum for abstract command register read capability (Section 20.2)
* - 1.4
  - Apr 19, 2022
  - (RTL bug fixes only)
:::

## Reference Documents

:::{list-table}
:header-rows: 1

* - **Item #**
  - **Document**
  - **Revision Used**
  - **Comment**
* - <a name="ref-1"></a>1
  - The RISC-V Instruction Set Manual Volume I: User-Level ISA
  - 20190608-Base-Ratified
  - Specification ratified
* - <a name="ref-2"></a>2
  - The RISC-V Instruction Set Manual Volume II: Privileged Architecture
  - 20190608-Priv-MSU-Ratified
  - Specification ratified
* - <a name="ref-2-plic"></a>2 (PLIC)
  - The RISC-V Instruction Set Manual Volume II: Privileged Architecture
  - 1.11-draft December 1, 2018
  - Last specification version with PLIC chapter
* - <a name="ref-3"></a>3
  - RISC-V External Debug Support
  - 0.13.2
  - Specification ratified
* - <a name="ref-4"></a>4
  - RISC-V Bitmanip Extension
  - 0.94-draft (January 20, 2022)
  - Zba, Zbb, Zbc, and Zbs sub- extensions are 'frozen'
:::

## Abbreviations

:::{list-table}
:header-rows: 1

* - Abbreviation
  - Description
* - AHB
  - Advanced High-performance Bus (by ARM®)
* - AMBA
  - Advanced Microcontroller Bus Architecture (by ARM)
* - ASIC
  - Application Specific Integrated Circuit
* - AXI
  - Advanced eXtensible Interface (by ARM)
* - CCM
  - Closely Coupled Memory (= TCM)
* - CPU
  - Central Processing Unit
* - CSR
  - Control and Status Register
* - DCCM
  - Data Closely Coupled Memory (= DTCM)
* - DEC
  - DECoder unit (part of core)
* - DMA
  - Direct Memory Access
* - DTCM
  - Data Tightly Coupled Memory (= DCCM)
* - ECC
  - Error Correcting Code
* - EXU
  - EXecution Unit (part of core)
* - ICCM
  - Instruction Closely Coupled Memory (= ITCM)
* - IFU
  - Instruction Fetch Unit
* - ITCM
  - Instruction Tightly Coupled Memory (= ICCM)
* - JTAG
  - Joint Test Action Group
* - LSU
  - Load/Store Unit (part of core)
* - MPC
  - Multi-Processor Controller
* - MPU
  - Memory Protection Unit
* - NMI
  - Non-Maskable Interrupt
* - PIC
  - Programmable Interrupt Controller
* - PLIC
  - Platform-Level Interrupt Controller
* - POR
  - Power-On Reset
* - RAM
  - Random Access Memory
* - RAS
  - Return Address Stack
* - ROM
  - Read-Only Memory
* - SECDED
  - Single-bit Error Correction/Double-bit Error Detection
* - SEDDED
  - Single-bit Error Detection/Double-bit Error Detection
* - SoC
  - System on Chip
* - TBD
  - To Be Determined
* - TCM
  - Tightly Coupled Memory (= CCM)
:::
