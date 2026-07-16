(chap-adaptations)=
# Standard RISC-V CSRs with Core-Specific Adaptations

A summary of standard RISC-V control/status registers in CSR space with platform-specific adaptations:

- Machine Interrupt Enable (`mie`) and Machine Interrupt Pending (`mip`) Registers (see Section 13.1.1)
- Machine Hardware Thread ID Register (`mhartid`) (see Section 13.1.3)
- Machine Cause Register (`mcause`) (see Section 13.1.2)

All reserved and unused bits in these control/status registers must be hardwired to `0`.
Unless otherwise noted, all read/write control/status registers must have WARL (Write Any value, Read Legal value) behavior.

### Machine Interrupt Enable (`mie`) and Machine Interrupt Pending (`mip`) Registers

The standard RISC-V `mie` and `mip` registers hold the machine interrupt enable and interrupt pending bits, respectively.
Since VeeR EH2 only supports machine mode, all supervisor- and user-specific bits are not implemented.
In addition, the `mie` / `mip` registers also host the platform-specific local interrupt enable/pending bits (shown with a gray background in {ref}`Table 13-1 <table-13-1>` and {ref}`Table 13-2 <table-13-2>` below).

The `mie` register is a standard read/write CSR and hart-specific (i.e., a separate register per thread).

:::{list-table} Machine Interrupt Enable Register (`mie`, at CSR 0x304)
:name: table-13-1
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - Reserved
  - 31
  - Reserved
  - R
  - 0
* - mceie
  - 30
  - Correctable error local interrupt enable
  - R/W
  - 0
* - mitie0
  - 29
  - Internal timer 0 local interrupt enable
  - R/W
  - 0
* - mitie1
  - 28
  - Internal timer 1 local interrupt enable
  - R/W
  - 0
* - Reserved
  - 27:12
  - Reserved
  - R
  - 0
* - `meie`
  - 11
  - Machine external interrupt enable
  - R/W
  - 0
* - Reserved
  - 10:8
  - Reserved
  - R
  - 0
* - mtie
  - 7
  - Machine timer interrupt enable
  - R/W
  - 0
* - Reserved
  - 6:4
  - Reserved
  - R
  - 0
* - msie
  - 3
  - Machine software interrupt enable
  - R/W
  - 0
* - Reserved
  - 2:0
  - Reserved
  - R
  - 0
:::

The `mip` register is a standard read/write CSR and hart-specific (i.e., a separate register per thread).

:::{note}
All M-mode interrupt pending bits of the read/write `mip` register are read-only.
:::

:::{list-table} Machine Interrupt Pending Register (`mip`, at CSR 0x344)
:name: table-13-2
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - Reserved
  - 31
  - Reserved
  - R
  - 0
* - mceip
  - 30
  - Correctable error local interrupt pending
  - R
  - 0
* - mitip0
  - 29
  - Internal timer 0 local interrupt pending
  - R
  - 0
* - mitip1
  - 28
  - Internal timer 1 local interrupt pending
  - R
  - 0
* - Reserved
  - 27:12
  - Reserved
  - R
  - 0
* - `meip`
  - 11
  - Machine external interrupt pending
  - R
  - 0
* - Reserved
  - 10:8
  - Reserved
  - R
  - 0
* - mtip
  - 7
  - Machine timer interrupt pending
  - R
  - 0
* - Reserved
  - 6:4
  - Reserved
  - R
  - 0
* - msip
  - 3
  - Machine software interrupt pending
  - R
  - 0
* - Reserved
  - 2:0
  - Reserved
  - R
  - 0
:::

### Machine Cause Register (`mcause`)

The standard RISC-V `mcause` register indicates the cause for a trap as shown in {ref}`Table 13-3 <table-13-3>`, including standard exceptions/interrupts, platform-specific local interrupts (with light gray background), and NMI causes (with dark gray background).

Additional trap information is provided in the `mscause` register (see Section 3.8.5) which allows the determination of the exact cause of a trap for cases where multiple, different conditions share a single trap code.

The `mcause` register has WLRL (Write Legal value, Read Legal value) behavior.

This register is a standard read/write CSR and hart-specific (i.e., a separate register per thread).

:::{list-table} Machine Cause Register (`mcause`, at CSR 0x342)
:name: table-13-3
:header-rows: 1

* - Type
  - Trap Code
  - Value `mcause`[31:0]
  - Description
  - Section(s)
* - NMI
  - N/A
  - 0x0000_0000
  - NMI pin assertion
  - 2.17
* - Exception
  - 1
  - 0x0000_0001
  - Instruction access fault
  - 2.7.5, 2.7.7, and 3.4
* - Exception
  - 2
  - 0x0000_0002
  - Illegal instruction
  -
* - Exception
  - 3
  - 0x0000_0003
  - Breakpoint
  -
* - Exception
  - 4
  - 0x0000_0004
  - Load address misaligned
  - 2.7.6
* - Exception
  - 5
  - 0x0000_0005
  - Load access fault
  - 2.7.5, 2.7.7, and 3.4
* - Exception
  - 6
  - 0x0000_0006
  - Store/AMO address misaligned
  - 2.7.6
* - Exception
  - 7
  - 0x0000_0007
  - Store/AMO access fault
  - 2.7.5, 2.7.7, and 3.4
* - Exception
  - 11
  - 0x0000_000B
  - Environment call from M-mode
  -
* - Interrupt
  - 3
  - 0x8000_0003
  - Machine software interrupt
  - 2.18
* - Interrupt
  - 7
  - 0x8000_0007
  - Machine timer [^55] interrupt
  -
* - Interrupt
  - 11
  - 0x8000_000B
  - Machine external interrupt
  - 7
* - Interrupt
  - 28
  - 0x8000_001C
  - Machine internal timer 1 local interrupt
  - 5.3
* - Interrupt
  - 29
  - 0x8000_001D
  - Machine internal timer 0 local interrupt
  - 5.3
* - Interrupt
  - 30
  - 0x8000_001E
  - Machine correctable error local interrupt
  - 2.7.2
* - NMI
  - N/A
  - 0xF000_0000
  - Machine D-bus store error NMI
  - 2.7.1 and 2.17
* - NMI
  - N/A
  - 0xF000_0001
  - Machine D-bus non-blocking load error NMI
  - 2.7.1 and 2.17
* - NMI
  - N/A
  - 0xF000_1000
  - Machine Fast Interrupt double-bit ECC error NMI
  - 7.6.1 and 2.17
* - NMI
  - N/A
  - 0xF000_1001
  - Machine Fast Interrupt DCCM region access error NMI
  - 7.6.1 and 2.17
* - NMI
  - N/A
  - 0xF000_1002
  - Machine Fast Interrupt non-DCCM region NMI
  - 7.6.1 and 2.17
:::

:::{note}
All other values are reserved.
:::

### Machine Hardware Thread ID Register (`mhartid`)

The standard RISC-V `mhartid` register provides the integer ID of the hardware thread running the code.
Hart IDs must be unique.
Hart IDs might not necessarily be numbered contiguously in a multiprocessor system, but at least one hart must have a hart ID of zero.

:::{note}
In certain cases, it must be ensured that exactly one hart runs some code (e.g., at reset), hence the requirement for one hart to have a known hart ID of zero.
:::

The `mhartid` register is split into two fixed-sized fields.
The SoC must provide a hardwired core ID on the `core_id`[31:4] bus.
The value provided on that bus sources the `mhartid` register's *coreid* field.
If the SoC hosts more than one RISC-V core, each core must have its own unique `core_id` value.
Each hardware thread of the core has a unique, hardwired thread ID which is reflected in the `mhartid` register's *hartid* field starting at 0x0 up to 0xF.
VeeR EH2 implements two hardware threads with thread IDs 0x0 and 0x1.

This register is a standard read-only CSR and hart-specific (i.e., a separate register per thread).

:::{list-table} Machine Hardware Thread ID Register (`mhartid`, at CSR 0xF14)
:name: table-13-4
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - coreid
  - 31:4
  - Core ID of this VeeR EH2
  - R
  - `core_id`[31:4] bus value (see {ref}`Table 16-1 <table-16-1>`)
* - hartid
  - 3:0
  - Hardwired per-core hart ID:
    * 0x0: thread 0 (master thread)
    * 0x1: thread 1
  - R
  - hardwired thread ID
:::

[^55]: Core external timer
