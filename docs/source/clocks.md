(chap-clocks)=
# Clock and Reset

This chapter describes clocking and reset signals used by the VeeR EH2 core complex.

## Features

The VeeR EH2 core complex's clock and reset features are:

- Support for independent clock ratios for four separate system bus interfaces
  - System bus clock ratios controlled by SoC
- Single core complex clock input
  - System bus clock ratios controlled by enable signals
- Single core complex reset signal
  - Ability to reset to Debug Mode
- Separate Debug Module reset signal
  - Allows to interact with Debug Module when core complex is still in reset

## Clocking

### Regular Operation

The VeeR EH2 core complex is driven by a single clock (`clk`).
All input and output signals, except those listed in {ref}`Table 16-1 <table-16-1>`, are synchronous to `clk` .

The core complex provides three master system bus interfaces (for instruction fetch, load/store data, and debug) as well as one slave (DMA) system bus interface.
The SoC controls the clock ratio for each system bus interface via the clock enable signal (`*_bus_clk_en`).
The clock ratios selected by the SoC may be the same or different for each system bus.

{ref}`Figure 16-1 <figure-16-1>` depicts the conceptual relationship of the clock (`clk`), system bus enable (`*_bus_clk_en`) used to select the clock ratio for each system bus, and the data (`*data`) of the respective system bus.

```{figure} img/clock_timing.png
:name: figure-16-1

Conceptual Clock, Clock-Enable, and Data Timing Relationship
```

Note that the clock net is not explicitly buffered, as the clock tree is expected to be synthesized during place-and-route.
The achievable clock frequency depends on the configuration, the sizes and configuration of I-cache and I/DCCMs, and the silicon implementation technology.

### System Bus-to-Core Clock Ratios

{ref}`Figure 16-2 <figure-16-2>` to {ref}`Figure 16-9 <figure-16-9>` depict the timing relationships of clock, clock-enable, and data for the supported system bus clock ratios from 1:1 (i.e., the system bus and core run at the same rate) to 1:8 (i.e., the system bus runs eight times slower than the core).

```{figure} img/1_1_bus2core_clock_ratio.png
:name: figure-16-2

1:1 System Bus-to-Core Clock Ratio
```

```{figure} img/1_2_bus2core_clock_ratio.png
:name: figure-16-3

1:2 System Bus-to-Core Clock Ratio
```

```{figure} img/1_3_bus2core_clock_ratio.png
:name: figure-16-4

1:3 System Bus-to-Core Clock Ratio
```

```{figure} img/1_4_bus2core_clock_ratio.png
:name: figure-16-5

1:4 System Bus-to-Core Clock Ratio
```

```{figure} img/1_5_bus2core_clock_ratio.png
:name: figure-16-6

1:5 System Bus-to-Core Clock Ratio
```

```{figure} img/1_6_bus2core_clock_ratio.png
:name: figure-16-7

1:6 System Bus-to-Core Clock Ratio
```

```{figure} img/1_7_bus2core_clock_ratio.png
:name: figure-16-8

1:7 System Bus-to-Core Clock Ratio
```

```{figure} img/1_8_bus2core_clock_ratio.png
:name: figure-16-9

1:8 System Bus-to-Core Clock Ratio
```

### Asynchronous Signals

{ref}`Table 16-1 <table-16-1>` provides a list of signals which are asynchronous to the core clock (`clk`).
Signals which are inputs to the core complex are synchronized to `clk` in the core complex logic.
Signals which are outputs of the core complex must be synchronized outside of the core complex logic if the respective receiving clock domain is driven by a different clock than `clk` .

Note that each asynchronous input passes through a two-stage synchronizer.
The signal must be asserted for at least two full `clk` cycles to guarantee it is detected by the core complex logic.
Shorter pulses might be dropped by the synchronizer circuit.

:::{list-table} Core Complex Asynchronous Signals
:name: table-16-1
:header-rows: 1

* - Signal
  - Dir
  - Description
* -
  - **Interrupts**
  -
* - `extintsrc_req`[pt.PIC_TOTAL_INT:1]
  - in
  - External interrupts
* - `soft_int`[pt.NUM_THREADS-1:0]
  - in
  - Standard RISC-V software interrupts (per thread)
* - `timer_int`[pt.NUM_THREADS-1:0]
  - in
  - Standard RISC-V timer interrupt (per thread)
* - `nmi_int`
  - in
  - Non-Maskable Interrupt
* -
  - **Power Management Unit (PMU) Interface (per Thread)**
  -
* - `i_cpu_halt_req`[pt.NUM_THREADS-1:0]
  - in
  - PMU halt request to thread
* - `i_cpu_run_req`[pt.NUM_THREADS-1:0]
  - in
  - PMU run request to thread
* -
  - **Multi-Processor Controller (MPC) Debug Interface (per Thread)**
  -
* - `mpc_debug_halt_req`[pt.NUM_THREADS-1:0]
  - in
  - MPC debug halt request to thread
* - `mpc_debug_run_req`[pt.NUM_THREADS-1:0]
  - in
  - MPC debug run request to thread
* -
  - **JTAG**
  -
* - `jtag_tck`
  - in
  - JTAG Test Clock
* - `jtag_tms`
  - in
  - JTAG Test Mode Select (synchronous to `jtag_tck`)
* - `jtag_tdi`
  - in
  - JTAG Test Data In (synchronous to `jtag_tck`)
* - `jtag_trst_n`
  - in
  - JTAG Test Reset
* - `jtag_tdo`
  - out
  - JTAG Test Data Out (synchronous to `jtag_tck`)
:::

## Reset

The VeeR EH2 core complex provides two reset signals, the core complex reset (see Section 16.3.1) and the Debug Module reset (see Section 16.3.2).

### Core Complex Reset (`rst_l`)

As shown in {ref}`Figure 16-10 <figure-16-10>`, the core complex reset signal (`rst_l`) is active-low, may be asynchronously asserted, but must be synchronously deasserted to avoid any glitches.
The `rst_l` input signal is not synchronized to the core clock (`clk`) inside the core complex logic.
All core complex flops are reset asynchronously.

```{figure} img/clock_reset_timing.png
:name: figure-16-10

Conceptual Clock and Reset Timing Relationship
```

Note that the core complex clock (`clk`) must be stable before the core complex reset (`rst_l`) is deasserted.

:::{note}
From a backend perspective, care should be taken during place-and-route optimization steps to adequately build buffer tree and distribution network of the `rst_l` signal.
Slew (transition time) targets should be in the same range as functional signals and distribution delays should be closely matched to clock delays, to maintain reasonable latencies and skews.
Further, `rst_l` specific timing checks can be performed during final signoff timing to ensure proper functionality, though they are more complex and challenging to model through static timing analysis.
:::

:::{note}
The core complex reset signal resets the entire VeeR EH2 core complex, except the Debug Module.
:::

### Debug Module Reset (`dbg_rst_l`)

The Debug Module reset signal (`dbg_rst_l`) is an active-low signal which resets the VeeR EH2 core complex's Debug Module as well as the synchronizers between the JTAG interface and the core complex.
The Debug Module reset signal may be connected to the power-on reset signal of the SoC.
This allows an external debugger to interact with the Debug Module when the core complex reset signal (`rst_l`) is still asserted.

If this layered reset functionality is not required, the `dbg_rst_l` signal may be tied to the `rst_l` signal outside the core complex.

### Debugger Initiating Reset via JTAG Interface

A debugger may also initiate a reset of the core complex logic via the JTAG interface.
Note that such a reset assertion is not visible to the SoC.
Resetting the core complex while the core is accessing any SoC memory locations may result in unpredictable behavior.
Recovery may require an assertion of the SoC master reset.

### Core Complex Reset to Debug Mode

The RISC-V Debug specification [[3]](intro.md#ref-3) states a requirement that the debugger must be able to be in control from the first executed instruction of a program after a reset.

The `dmcontrol` register (see Section 11.1.2.1) of the Debug Module controls the core-complex-internal *ndmreset* (non-debug module reset) signal [^60].
This signal resets the core complex (except for the Debug Module and Debug Transport Module).

The `hawindow` register (see Section 11.1.2.5) of the Debug Module allows to select multiple harts.

The following sequence is used to reset the core and execute the first instruction for hart0, hart1, or both harts in Debug Mode (i.e., db-halt state):

1. Take Debug Module out of reset
    - Set *dmactive* bit of `dmcontrol` register (`dmcontrol` = 0x0000\_0001)
2. Reset core complex
    - Set *ndmreset* bit of `dmcontrol` register (`dmcontrol` = 0x0000\_0003)
3. If both harts should enter Debug Mode out of reset
    - Write `hawindow` register to 3 (`hawindow` = 0x0000\_0003)
4. While in reset, assert halt request with *ndmreset* still asserted
    - Set *haltreq* bit of `dmcontrol` register for selected hart(s)
      - Hart0: `dmcontrol` = 0x8000\_0003
      - Hart1: `dmcontrol` = 0x8001\_0003
      - Both harts: `dmcontrol` = 0x8400\_0003
5. Take core complex out of reset with halt request still asserted
    - Clear *ndmreset* bit of `dmcontrol` register for same selected hart(s)
      - Hart0: `dmcontrol` = 0x8000\_0001
      - Hart1: `dmcontrol` = 0x8001\_0001
      - Both harts: `dmcontrol` = 0x8400\_0001

:::{note}
Hart0 enters Debug Mode immediately after clearing the *ndmreset* bit of the `dmcontrol` register in the last step above.
Hart1 only enters Debug Mode after the *start1* bit of the `mhartstart` register (see Section 5.4.2) has been written to `1` either by code running on hart0 or by the debugger using the access register abstract command (see Section 11.1.2.7) to write this register selecting hart0.
:::

[^60]: Alternatively, the core complex may also be held in reset by asserting the `rst_l` signal (see Section 16.3.1).
       Note that step 2. as well as asserting and deasserting *ndmreset* in step 4. and 5. of the shown sequence are not needed then.
