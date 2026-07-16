(chap-performance)=
# Performance Monitoring

This chapter describes the performance monitoring features of the VeeR EH2 core.

## Features

VeeR EH2 provides these performance monitoring features:

- Four standard 64-bit wide event counters
- Standard separate event selection for each counter
- Standard selective count enable/disable controllability
- Standard synchronized counter enable/disable controllability
- Standard cycle counter
- Standard retired instructions counter
- Support for standard SoC-based machine timer registers

## Control/Status Registers

### Standard RISC-V Registers

A list of performance monitoring-related standard RISC-V CSRs with references to their definitions:

- Machine Hardware Performance Monitor (`mcycle{|h}` [^40],[^41] , `minstret{|h}`, `mhpmcounter3{|h}-mhpmcounter31{|h}`, and `mhpmevent3-mhpmevent31`) (see Section 3.1.11 in [[2]](intro.md#ref-2))
- Machine Counter-Inhibit Register [^42]  (`mcountinhibit` [^43]) (see Section 3.1.13 in [[2]](intro.md#ref-2))
- Machine Timer Registers (`mtime` and `mtimecmp`) (see Section 3.1.10 in [[2]](intro.md#ref-2))

  :::{note}
  `mtime` and `mtimecmp` are memory-mapped registers which must be provided by the SoC.
  :::

## Counters

Only event counters 3 to 6 (`mhpmcounter3{|h}-mhpmcounter6{|h}`) and their corresponding event selectors (`mhpmevent3-mhpmevent6`) are functional on VeeR EH2.
Event counters 7 to 31 (`mhpmcounter7{|h}-mhpmcounter31{|h}`) and their corresponding event selectors (`mhpmevent7-mhpmevent31`) are hardwired to `0`.

## Count-Impacting Conditions

A few comments to consider on conditions that have an impact on the performance monitor counting:

- While in the pmu/fw-halt power management state, performance counters (including the `mcycle` [^40] counter) are disabled.
- While in debug halt (db-halt) state, the *stopcount* bit of the `dcsr` register (see Section 11.1.3.5) determines if performance counters are enabled.
- While in the pmu/fw-halt power management state or the debug halt (db-halt) state with the *stopcount* bit set, DMA accesses are allowed, but not counted by the performance counters.
  It would be up to the bus master to count accesses while the core is in a halt state.
- While executing PAUSE, performance counters are enabled.

Also, it is recommended that the performance counters are disabled (using the `mcountinhibit` register) before the counters and event selectors are modified, and then reenabled again.
This minimizes the impact of reading and writing the counter and event selector CSRs on the event count values, specifically for the CSR read/write events (i.e., events #16 and #17).
In general, performance counters are incremented after a read access to the counter CSRs, but before a write access to the counter CSRs.

## Events

{ref}`Table 9-1 <table-9-1>` provides a list of the countable events.

:::{note}
The event selector registers `mhpmevent3` -`mhpmevent6` have WARL behavior.
When writing either a value marked as 'Reserved' or larger than the highest supported event number, the event selector is set to `0` (i.e., no event counted).
:::

## {ref}`Table 9-1 <table-9-1>`  List of Countable Events

**Legend**: *Description*: IP = In-Pipe; OOP = Out-Of-Pipe / *Scope*: C = per-Core; T = per-Thread

:::{list-table}
:name: table-9-1
:header-rows: 1

* - Event No
  - Event Name
  - Description
  - Scope
* - 0
  -
  - Reserved (no event counted)
  -
* -
  - **Events counted while in Active (C0) state**
  -
  -
* - 1
  - cycles clocks active
  - Number of cycles clock active (OOP)
  - C
* - 2
  - I-cache hits
  - Number of I-cache hits (OOP, speculative, valid fetch & hit)
  - T
* - 3
  - I-cache misses
  - Number of I-cache misses (OOP, valid fetch & miss)
  - T
* - 4
  - instr committed - all
  - Number of all (16b+32b) instructions committed (IP, non- speculative, 0/1/2)
  - T
* - 5
  - instr committed - 16b
  - Number of 16b instructions committed (IP, non-speculative, 0/1/2)
  - T
* - 6
  - instr committed - 32b
  - Number of 32b instructions committed (IP, non-speculative, 0/1/2)
  - T
* - 7
  - instr aligned - all
  - Number of all (16b+32b) instructions aligned (OOP, speculative, 0/1/2)
  - T
* - 8
  - instr decoded - all
  - Number of all (16b+32b) instructions decoded (OOP, speculative, 0/1/2)
  - T
* - 9
  - muls committed
  - Number of multiplications committed (IP, 0/1)
  - T
* - 10
  - divs committed
  - Number of divisions and remainders committed (IP, 0/1)
  - T
* - 11
  - loads committed
  - Number of loads committed (IP, 0/1)
  - T
* - 12
  - stores committed
  - Number of stores committed (IP, 0/1)
  - T
* - 13
  - misaligned loads
  - Number of misaligned loads (IP, 0/1)
  - T
* - 14
  - misaligned stores
  - Number of misaligned stores (IP, 0/1)
  - T
* - 15
  - alus committed
  - Number of ALU [^44] operations committed (IP, 0/1/2)
  - T
* - 16
  - CSR read
  - Number of CSR read instructions committed (IP, 0/1)
  - T
* - 17
  - CSR read/write
  - Number of CSR read/write instructions committed (IP, 0/1)
  - T
* - 18
  - CSR write rd==0
  - Number of CSR write rd==0 instructions committed (IP, 0/1)
  - T
* - 19
  - `ebreak`
  - Number of ebreak instructions committed (IP, 0/1)
  - T
* - 20
  - `ecall`
  - Number of ecall instructions committed (IP, 0/1)
  - T
* - 21
  - `fence`
  - Number of fence instructions committed (IP, 0/1)
  - T
* - 22
  - `fence.i`
  - Number of fence.i instructions committed (IP, 0/1)
  - T
* - 23
  - `mret`
  - Number of mret instructions committed (IP, 0/1)
  - T
* - 24
  - branches committed
  - Number of branches committed (IP)
  - T
* - 25
  - branches mispredicted
  - Number of branches mispredicted (IP)
  - T
* - 26
  - branches taken
  - Number of branches taken (IP)
  - T
* - 27
  - unpredictable branches
  - Number of unpredictable branches (IP)
  - T
* - 28
  - cycles fetch stalled
  - Number of cycles fetch ready but stalled (OOP)
  - T
* - 29
  - cycles aligner stalled
  - Number of cycles one or more instructions valid in aligner but IB full (OOP)
  - T
* - 30
  - cycles decode stalled
  - Number of cycles one or more instructions valid in IB but decode stalled (OOP)
  - T
* - 31
  - cycles postsync stalled
  - Number of cycles postsync stalled at decode (OOP)
  - T
* - 32
  - cycles presync stalled
  - Number of cycles presync stalled at decode (OOP)
  - T
* - 33
  -
  - Reserved
  -
* - 34
  - cycles SB/WB stalled (lsu_store_stall_any)
  - Number of cycles decode stalled due to SB or WB full (OOP)
  - T
* - 35
  - cycles DMA DCCM transaction stalled (dma_dccm_stall_any)
  - Number of cycles DMA stalled due to decode for load/store (OOP)
  - C
* - 36
  - cycles DMA ICCM transaction stalled (dma_iccm_stall_any)
  - Number of cycles DMA stalled due to fetch (OOP)
  - C
* - 37
  - exceptions taken
  - Number of exceptions taken (IP)
  - T
* - 38
  - timer interrupts taken
  - Number of timer [^45] interrupts taken (IP)
  - T
* - 39
  - external interrupts taken
  - Number of external interrupts taken (IP)
  - T
* - 40
  - TLU flushes (flush lower)
  - Number of TLU flushes (flush lower) (IP)
  - T
* - 41
  - branch error flushes
  - Number of branch error flushes (IP)
  - T
* - 42
  - I-bus transactions - instr
  - Number of instr transactions on I-bus interface (OOP)
  - T
* - 43
  - D-bus transactions - ld/st
  - Number of ld/st transactions on D-bus interface (OOP)
  - T
* - 44
  - D-bus transactions - misaligned
  - Number of misaligned transactions on D-bus interface (OOP)
  - T
* - 45
  - I-bus errors
  - Number of transaction errors on I-bus interface (OOP)
  - T
* - 46
  - D-bus errors
  - Number of transaction errors on D-bus interface (OOP)
  - T
* - 47
  - cycles stalled due to I- bus busy
  - Number of cycles stalled due to AXI4 or AHB-Lite I-bus busy (OOP)
  - T
* - 48
  - cycles stalled due to D- bus busy
  - Number of cycles stalled due to AXI4 or AHB-Lite D-bus busy (OOP)
  - T
* - 49
  - cycles interrupts disabled
  - Number of cycles interrupts disabled (MSTATUS.MIE==0) (OOP)
  - T
* - 50
  - cycles interrupts stalled while disabled
  - Number of cycles interrupts stalled while disabled (MSTATUS.MIE==0) (OOP)
  - T
* - 51
  - `amo*`
  - Number of atomic [^46] instructions committed (IP, 0/1)
  - T
* - 52
  - `lr`
  - Number of lr instructions committed (IP, 0/1)
  - T
* - 53
  - `sc`
  - Number of sc [^47] instructions committed (IP, 0/1)
  - T
* - 54
  - bitmanip committed
  - Number of bit-manipulation operations committed (IP, 0/1/2)
  - T
* - 55
  - D-bus loads committed
  - Number of load instructions to D-bus committed (IP, 0/1)
  - T
* - 56
  - D-bus stores committed
  - Number of store instructions to D-bus committed (IP, 0/1)
  - T
* - 57 - 511
  -
  - Reserved
  -
* -
  - **Events counted while in Active (C0) or Sleep (C3) states**
  -
  -
* - 512
  - cycles in Sleep (C3) state
  - Number of cycles in Sleep (C3) state (OOP)
  - T
* - 513
  - DMA reads (all)
  - Total number of DMA slave read transactions (OOP)
  - C
* - 514
  - DMA writes (all)
  - Total number of DMA slave write transactions (OOP)
  - C
* - 515
  - DMA reads to DCCM
  - Number of DMA slave read transactions to DCCM (OOP)
  - C
* - 516
  - DMA writes to DCCM
  - Number of DMA slave write transactions to DCCM (OOP)
  - C
:::

:::{note}
If an event shown as 'Reserved' is selected, no error is reported but counter is not incrementing.
:::

[^40]: Note that the `mcycle/mcycleh` registers are implemented per thread (i.e., per hart) in the VeeR EH2 core, whereas in other cores these registers may be implemented per core.
[^41]: For hart1 (T1), the `mcycle` counter is held in reset until hart1 has been started (i.e., has exited the idle state).
[^42]: The standard `mcountinhibit` register which was recently added to [[2]](intro.md#ref-2) replaces the non-standard mgpmc register of the previous VeeR generation.
       The `mcountinhibit` register provides the same functionality as the mgpmc register did, but at a much finer granularity (i.e., an enable/disable control bit per standard hardware performance counter instead of a single control bit for the `mhpmcounter3` -`mhpmcounter6` counters).
[^43]: Since the `mcycle` / `mcycleh` registers are implemented per thread, the *CY* bit of the per-thread `mcountinhibit` register only controls the incrementing of the `mcycle` / `mcycleh` registers of the respective hart.
[^44]: NOP is an ALU operation.
       WFI is implemented as a NOP in VeeR EH2 and, hence, counted as an ALU operation was well.
[^45]: Events counted include interrupts triggered by the standard RISC-V platform-level timer as well as by the internal timers.
[^46]: LR and SC instructions not included.
[^47]: Independent of if sc succeeds or fails.
