(chap-csrs)=
# CSR Address Map

## Standard RISC-V CSRs

{ref}`Table 14-1 <table-14-1>` lists the VeeR EH2 core-specific standard RISC-V Machine Information CSRs.

:::{list-table} VeeR EH2 Core-Specific Standard RISC-V Machine Information CSRs
:name: table-14-1
:header-rows: 1

* - Number
  - Privilege
  - Name
  - Description
  - Scope [^56]
  - Value
* - 0x301
  - MRW
  - `misa`
  - ISA and extensions

    :::{note}
    writes ignored
    :::
  - C
  - 0x4000_1105 (with atomics support [^57]) 0x4000_1104 (without atomics support [^57])
* - 0xF11
  - MRO
  - `mvendorid`
  - Vendor ID
  - C
  - 0x0000_0045
* - 0xF12
  - MRO
  - `marchid`
  - Architecture ID
  - C
  - 0x0000_0011
* - 0xF13
  - MRO
  - `mimpid`
  - Implementation ID
  - C
  - 0x0000_0003
* - 0xF14
  - MRO
  - `mhartid`
  - Hardware thread ID
  - T
  - (see Section 13.1.3)
:::

{ref}`Table 14-2 <table-14-2>` lists the VeeR EH2 standard RISC-V CSR address map.

:::{list-table} VeeR EH2 Standard RISC-V CSR Address Map
:name: table-14-2
:header-rows: 1

* - Number
  - Privilege
  - Name
  - Description
  - Scope [^56]
  - Section
* - 0x300
  - MRW
  - `mstatus`
  - Machine status
  - T
  -
* - 0x304
  - MRW
  - `mie`
  - Machine interrupt enable
  - T
  - 12.1.1
* - 0x305
  - MRW
  - `mtvec`
  - Machine trap-handler base address
  - T
  -
* - 0x320
  - MRW
  - `mcountinhibit`
  - Machine counter-inhibit register
  - T
  - 8.2.1
* - 0x323
  - MRW
  - `mhpmevent3`
  - Machine performance-monitoring event selector 3
  - T
  - 8.2.1
* - 0x324
  - MRW
  - `mhpmevent4`
  - Machine performance-monitoring event selector 4
  - T
  - 8.2.1
* - 0x325
  - MRW
  - `mhpmevent5`
  - Machine performance-monitoring event selector 5
  - T
  - 8.2.1
* - 0x326
  - MRW
  - `mhpmevent6`
  - Machine performance-monitoring event selector 6
  - T
  - 8.2.1
* - 0x340
  - MRW
  - `mscratch`
  - Scratch register for machine trap handlers
  - T
  -
* - 0x341
  - MRW
  - `mepc`
  - Machine exception program counter
  - T
  -
* - 0x342
  - MRW
  - `mcause`
  - Machine trap cause
  - T
  - 12.1.2
* - 0x343
  - MRW
  - `mtval`
  - Machine bad address or instruction
  - T
  -
* - 0x344
  - MRW
  - `mip`
  - Machine interrupt pending
  - T
  - 12.1.1
* - 0x7A0
  - MRW
  - `tselect`
  - Debug/Trace trigger register select
  - T
  - 10.1.3.1
* - 0x7A1
  - MRW
  - `tdata1`
  - First Debug/Trace trigger data
  - T
  - 10.1.3.2
* - 0x7A1
  - MRW
  - `mcontrol`
  - Match control
  - T
  - 10.1.3.3
* - 0x7A2
  - MRW
  - `tdata2`
  - Second Debug/Trace trigger data
  - T
  - 10.1.3.4
* - 0x7B0
  - DRW
  - `dcsr`
  - Debug control and status register
  - T
  - 10.1.3.5
* - 0x7B1
  - DRW
  - `dpc`
  - Debug PC
  - T
  - 10.1.3.6
* - 0xB00
  - MRW
  - `mcycle`
  - Machine cycle counter
  - T [^58],[^59]
  - 8.2.1
* - 0xB02
  - MRW
  - `minstret`
  - Machine instructions-retired counter
  - T
  - 8.2.1
* - 0xB03
  - MRW
  - `mhpmcounter3`
  - Machine performance-monitoring counter 3
  - T
  - 8.2.1
* - 0xB04
  - MRW
  - `mhpmcounter4`
  - Machine performance-monitoring counter 4
  - T
  - 8.2.1
* - 0xB05
  - MRW
  - `mhpmcounter5`
  - Machine performance-monitoring counter 5
  - T
  - 8.2.1
* - 0xB06
  - MRW
  - `mhpmcounter6`
  - Machine performance-monitoring counter 6
  - T
  - 8.2.1
* - 0xB80
  - MRW
  - `mcycleh`
  - Upper 32 bits of `mcycle`, RV32I only
  - T [^58],[^59]
  - 8.2.1
* - 0xB82
  - MRW
  - `minstreth`
  - Upper 32 bits of `minstret`, RV32I only
  - T
  - 8.2.1
* - 0xB83
  - MRW
  - `mhpmcounter3h`
  - Upper 32 bits of `mhpmcounter3`, RV32I only
  - T
  - 8.2.1
* - 0xB84
  - MRW
  - `mhpmcounter4h`
  - Upper 32 bits of `mhpmcounter4`, RV32I only
  - T
  - 8.2.1
* - 0xB85
  - MRW
  - `mhpmcounter5h`
  - Upper 32 bits of `mhpmcounter5`, RV32I only
  - T
  - 8.2.1
* - 0xB86
  - MRW
  - `mhpmcounter6h`
  - Upper 32 bits of `mhpmcounter6`, RV32I only
  - T
  - 8.2.1
:::

## Non-Standard RISC-V CSRs

{ref}`Table 14-3 <table-14-3>` summarizes the VeeR EH2 non-standard RISC-V CSR address map.

:::{list-table} VeeR EH2 Non-Standard RISC-V CSR Address Map
:name: table-14-3
:header-rows: 1

* - Number
  - Privilege
  - Name
  - Description
  - Scope [^56]
  - Section
* - 0x7C0
  - MRW
  - `mrac`
  - Region access control
  - C
  - 2.8.1
* - 0x7C2
  - MRW
  - `mcpc`
  - Core pause control
  - T
  - 6.6.2
* - 0x7C4
  - DRW
  - `dmst`
  - Memory synchronization trigger (**Debug Mode only**)
  - T
  - 2.8.2
* - 0x7C6
  - MRW
  - `mpmc`
  - Power management control
  - T
  - 6.6.1
* - 0x7C8
  - DRW
  - `dicawics`
  - I-cache array/way/index selection (**Debug Mode only**)
  - T
  - 9.5.1
* - 0x7C9
  - DRW
  - `dicad0`
  - I-cache array data 0 (**Debug Mode only**)
  - T
  - 9.5.2
* - 0x7CA
  - DRW
  - `dicad1`
  - I-cache array data 1 (**Debug Mode only**)
  - T
  - 9.5.4
* - 0x7CB
  - DRW
  - `dicago`
  - I-cache array go (**Debug Mode only**)
  - T
  - 9.5.5
* - 0x7CC
  - DRW
  - `dicad0h`
  - I-cache array data 0 high (**Debug Mode only**)
  - T
  - 9.5.3
* - 0x7CE
  - MRW
  - `mfdht`
  - Force debug halt threshold
  - C
  - 6.6.3
* - 0x7CF
  - MRW
  - `mfdhs`
  - Force debug halt status
  - T
  - 6.6.4
* - 0x7D2
  - MRW
  - `mitcnt0`
  - Internal timer counter 0
  - T
  - 5.4.1
* - 0x7D3
  - MRW
  - `mitb0`
  - Internal timer bound 0
  - T
  - 5.4.2
* - 0x7D4
  - MRW
  - `mitctl0`
  - Internal timer control 0
  - T
  - 5.4.3
* - 0x7D5
  - MRW
  - `mitcnt1`
  - Internal timer counter 1
  - T
  - 5.4.1
* - 0x7D6
  - MRW
  - `mitb1`
  - Internal timer bound 1
  - T
  - 5.4.2
* - 0x7D7
  - MRW
  - `mitctl1`
  - Internal timer control 1
  - T
  - 5.4.3
* - 0x7F0
  - MRW
  - `micect`
  - I-cache error counter/threshold
  - C
  - 3.5.1
* - 0x7F1
  - MRW
  - `miccmect`
  - ICCM correctable error counter/threshold
  - C
  - 3.5.2
* - 0x7F2
  - MRW
  - `mdccmect`
  - DCCM correctable error counter/threshold
  - C
  - 3.5.3
* - 0x7F8
  - MRW
  - `mcgc`
  - Clock gating control
  - C
  - 11.1.2
* - 0x7F9
  - MRW
  - `mfdc`
  - Feature disable control
  - C
  - 11.1.1
* - 0x7FC
  - MRW
  - `mhartstart`
  - Hart start control
  - C
  - 4.4.2
* - 0x7FE
  - MRW
  - `mnmipdel`
  - NMI pin delegation
  - C
  - 4.4.3
* - 0x7FF
  - MRW
  - `mscause`
  - Machine secondary cause
  - T
  - 2.8.5
* - 0xBC0
  - MRW
  - `mdeau`
  - D-Bus error address unlock
  - T
  - 2.8.4
* - 0xBC8
  - MRW
  - `meivt`
  - External interrupt vector table
  - T
  - 7.12.7
* - 0xBC9
  - MRW
  - `meipt`
  - External interrupt priority threshold
  - T
  - 7.12.6
* - 0xBCA
  - MRW
  - `meicpct`
  - External interrupt claim ID / priority level capture trigger
  - T
  - 7.12.9
* - 0xBCB
  - MRW
  - `meicidpl`
  - External interrupt claim ID's priority level
  - T
  - 7.12.10
* - 0xBCC
  - MRW
  - `meicurpl`
  - External interrupt current priority level
  - T
  - 7.12.11
* - 0xFC0
  - MRO
  - `mdseac`
  - D-bus first error address capture
  - T
  - 2.8.3
* - 0xFC4
  - MRO
  - `mhartnum`
  - Total number harts
  - C
  - 4.4.1
* - 0xFC8
  - MRO
  - `meihap`
  - External interrupt handler address pointer
  - T
  - 7.12.8
:::


[^56]: C = per-core, T = per-thread
[^57]: Atomics support is selected with the ATOMIC_ENABLE build argument when the VeeR EH2 core is built.
       Atomics support is only available for single- and dual-threaded VeeR EH2 cores with a DCCM (see also Section 3.10).
[^58]: Note that the `mcycle`/`mcycleh` registers are implemented per thread (i.e., per hart) in the VeeR EH2 core, whereas in other cores these registers may be implemented per core.
[^59]: For hart1 (T1), the `mcycle` counter is held in reset until hart1 has been started (i.e., has exited the idle state).
