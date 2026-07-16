(chap-multi-threading)=
# Multi-Threading

:::{note}
Dual-threading is only supported for cores with AXI4 buses.
:::

This chapter describes the VeeR EH2 core's multi-threading capability.

## Features

The VeeR EH2's multi-threading features are:

- Support for two hardware threads (harts)
- After reset:
  - Start execution on only thread 0 (T0, master thread)
  - Thread 0 may then start execution on thread 1 (T1)
- Ability for firmware to probe total number of hardware threads supported by core
- Delegating NMI pin handling to either one of the two or both threads

## Core Features with Multi-Threading Support

Many features provided by the VeeR EH2 core are affected by the core's multi-threading capabilities.
The sections below provide a brief overview of the changes as well as cross-references to other sections with detailed descriptions, where appropriate.

### Control/Status Registers and Memory-Mapped Registers

Many control/status registers (CSRs) as well as some memory-mapped registers are per thread, instead of per core.
Each thread can only access its own copy of these registers, but not the other thread's copy.
Some control/status registers (CSRs) as well as many memory-mapped registers are still per core and may be accessed by both threads.

In each register definition in this specification, information is provided if the register is per core or per thread.
This information is also provided in the CSR and memory-mapped register summary tables (i.e., {ref}`Table 8-15 <table-8-15>`, {ref}`Table 8-16 <table-8-16>`, {ref}`Table 13-1 <table-13-1>`, {ref}`Table 13-2 <table-13-2>`, and {ref}`Table 13-3 <table-13-3>`).

### Reset

VeeR EH2 provides a single reset pin (`rst_l`) and reset vector (`rst_vec[31:1]`).
When the core comes out of reset, only hart0 (T0) immediately starts executing instructions.
Hart1 (T1) remains idle until started by hart0 T0.
See Section 5.3 below for details.

### Non-Maskable Interrupt (NMI)

VeeR EH2 provides a single NMI pin (`nmi_int`) and NMI vector (`nmi_vec[31:1]`).
The `mnmipdel` register (see Section 5.4.3 below) is used to steer NMI pin requests to either one or both threads.

### Software Interrupts

VeeR EH2 provides separate software interrupt pins (`soft_int[0/1]`), one for each thread.
See Section 3.18 for details.

### Timer Interrupts

VeeR EH2 provides separate timer interrupt pins (`timer_int[0/1]`), one for each thread.
The pins may be tied together or driven separately by the SoC.
Timer interrupt requests signaled on these pins may be masked with the respective *mtie* bit of the thread-specific standard RISC-V `mie` register (see {ref}`Table 12-1 <table-12-1>`).

### External Interrupts

VeeR EH2 provides a single multi-threading-enhanced PIC (see Section 8.3.3).
Each external interrupt source *S* can be individually controlled to delegate incoming interrupt requests to one of the two threads (see Section 8.12.14).

### Power Management

VeeR EH2 provides per-thread independent power management and debug control functionally, including the ability for firmware running on a thread to independently enter the Sleep (pmu/fw-halt) state.

## Thread Management

### Multi-Threading Control Registers

VeeR EH2 provides two multi-threading control registers.
The `mhartnum` register (see Section 5.4.1 below) enables firmware running on a thread to inquire about the core's threading capability.
The `mhartstart` register (see Section 5.4.2 below) allows hart0 (T0) to start hart1 (T1) after reset.

### Basic Startup and Run Flow

By convention, hart0 (T0) is the master hart coming out of reset.
The master hart0 is the only hart per core which starts running after a system reset (i.e., only the *start0* bit of the `mhartstart` register (see Section 5.4.2 below) is set to `1` on system reset).
When other harts are started by hart0, each hart may perform a fork and begin threadspecific execution.

Steps performed coming out of reset:

1. Master hart0 starts up.
2. It executes the startup code the reset vector is pointing at.
3. It then sets up data structures in memory for slave hart1 (T1):
    * E.g., scratchpad, stack, vector tables and handlers, and memory allocation.
    * Code to execute (i.e., jump tables and targets).

4. Master hart writes *start1* bit of the `mhartstart` register to `1` to enable slave hart1.
5. Slave hart1 starts up at shared reset vector.
    * This implies a common boot code.
6. Slave hart1 queries its `mhartid` register (see Section 13.1.3) and jumps to its unique startup code.
    * Startup code was set up by master hart0 or preloaded.

### Communication Between Harts

Harts may communicate with each other through common shared memory.
Since the two harts of a VeeR EH2 core shared one DCCM, using atomic operations on DCCM memory addresses is an efficient approach for the small number of harts of a single core to communicate.

Multi-core communication may rely on the use of SoC memory and some platform-specific external interlock mechanism to facilitate atomic operations.

### Inter-Processor Interrupts

The VeeR EH2 core supports the standard RISC-V software interrupt mechanism which may be used to interrupt other harts and trigger a look-up of shared memory data structures to communicate between harts.
The DCCM is the preferred shared memory due to its low latency for communication between harts of the same core.

Note that there is no mechanism to peek and poke another hart's state (i.e., no access to another hart's CSRs, registers, etc.).

## Control/Status Registers

A summary of platform-specific control/status registers in CSR space:

- Total Number of Harts Register (`mhartnum`) (see Section 5.4.1)
- Hart Start Control Register (`mhartstart`) (see Section 5.4.2)
- NMI Pin Delegation Register (`mnmipdel`) (see Section 5.4.3)

All reserved and unused bits in these control/status registers must be hardwired to `0`.
Unless otherwise noted, all read/write control/status registers must have WARL (Write Any value, Read Legal value) behavior.

### Total Number of Harts Register (`mhartnum`)

The `mhartnum` register is the 32-bit wide status register which provides the value of the total number of harts supported by the core.
This allows firmware running on the core to probe the number of hardware threads provided by this core.

This register is mapped to the non-standard read-only CSR address space and shared by the harts (i.e., one register per core).

:::{list-table} Total Number of Harts Register (`mhartnum`, at CSR 0xFC4)
:name: table-5-1
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - Reserved
  - 31:2
  - Reserved
  - R
  - 0
* - hartnum
  - 1:0
  - Total number of harts in this core

    :::{note}
    Depending on core build argument, VeeR EH2 provides either 1 (T0) or 2 (T0 and T1) hardware threads
    :::
  - R
  - 2 (dual- thread core) 1 (single- thread core)
:::

### Hart Start Control Register (`mhartstart`)

The `mhartstart` register is the 32-bit wide control/status register to start a hart and to provide 'running' status information of the harts.
Only hart T0 is running after reset by default.
After setting up the data structures for hart T1, hart T0 sets the *start1* bit of this register to start hart T1.
Either hart may read this register to inquire about the 'running' status of the harts.

:::{note}
A hart may only be started, but not stopped (i.e., hart T0 is always running; if hart T1 is started, it stays running).
:::

:::{note}
The values of the *start0* and *start1* bits of this register are provided on the periphery of the core (i.e., `dec_tlu_mhartstart[0/1]` pins) to allow other SoC IPs to determine if hart T1 has been started.
:::

:::{note}
For hart T1, the `mcycle` performance counter and `mitcnt`*X* internal timer counters are held in reset until hart T1 has been started (i.e., has exited the idle state).
:::

This register is mapped to the non-standard read/write CSR address space and shared by the harts (i.e., one register per core).

:::{list-table} Hart Start Control Register (`mhartstart`, at CSR 0x7FC)
:name: table-5-2
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - Reserved
  - 31:2
  - Reserved
  - R
  - 0
* - start1
  - 1
  - Hart start control and status for thread T1 (exported on `dec_tlu_mhartstart[1]` pin)

    :::{note}
    Not implemented for single-thread VeeR EH2 instantiations
    :::
  - R/W1 (dual- thread core)
  - 0
* -
  -
  -
  - R (single- thread core)
  -
* - start0
  - 0
  - Hart start status for thread T0 (exported on `dec_tlu_mhartstart[0]` pin)
  - R
  - 1
:::

### NMI Pin Delegation Register (`mnmipdel`)

The `mnmipdel` register is the 32-bit wide control/status register to delegate the handling of a pin-initiated NMI to either one of the harts or both harts.
Since the core has a single NMI pin, this register enables the flexibility to steer the handling of a pin-initiated NMI to a specific hart or harts.
Either hart may read this register to inquire which hart (or harts) has been delegated to handle a pin-initiated NMI.

:::{note}
Hardware enforces that at least one of the NMI delegation control bits is `1`.
Attempts to clear the last enabled control bit of the `mnmipdel` register are ignored.
:::

:::{note}
If the `mnmipdel` register is written by hart0 (T0) to delegate NMI pin requests to be handled solely by hart1 (T1), but hart1 has not been started yet (see Section 5.4.2 above), the handling of an NMI pin request may be delayed until hart1 has been started.
:::

This register is mapped to the non-standard read/write CSR address space and shared by the harts (i.e., one register per core).

:::{list-table} NMI Pin Delegation Register (`mnmipdel`, at CSR 0x7FE)
:name: table-5-3
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - Reserved
  - 31:2
  - Reserved
  - R
  - 0
* - nmipdel1
  - 1
  - Assertion of NMI pin handled by hart T1

    :::{note}
    Not implemented for single-thread VeeR EH2 instantiations
    :::
  - R/W (*dual-thread core*)
  - 0
* -
  -
  -
  - R (*single-thread core*)
  -
* - nmipdel0
  - 0
  - Assertion of NMI pin handled by hart T0
  - R/W
  - 1
:::
