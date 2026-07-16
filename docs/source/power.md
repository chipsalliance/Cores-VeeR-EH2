(chap-power)=
# Power Management and Multi-Core Debug Control

This chapter specifies the power management and multi-core debug control functionality provided or supported by the VeeR EH2 core.
Also documented in this chapter is how debug may interfere with core power management.

## Features

VeeR EH2 supports and provides the following power management and multi-core debug control features:

- Support for three system-level power states: Active (C0), Sleep (C3), Power Off (C6)
- Firmware-initiated halt to enter sleep state (separate per thread)
- Fine-grain clock gating in active state
- Enhanced clock gating in sleep state
- Halt/run control interface to/from SoC Power Management Unit (PMU)
- Signal indicating that thread is halted (separate per thread)
- Halt/run control interface to/from SoC debug Multi-Processor Controller (MPC) to enable cross-triggering in multi-core chips
- Timeout-based mechanism to force Debug Halt state by terminating hung bus transactions
- Signals indicating that thread is in Debug Mode and thread hit a breakpoint
- PAUSE feature to help avoid firmware spinning (separate per thread)

## Thread Control Interfaces

VeeR EH2 provides two control interfaces, one for power management and one for multi-core debug control, which enable the thread to be controlled by other SoC blocks.

### Power Management

The power management interface enables an SoC-based Power Management Unit (PMU) to:

- Halt (i.e., enter low-power sleep state) or restart (i.e., resume execution) the thread, and
- get an indication when the thread has gracefully entered the sleep state.

The power management interface signals are described in {ref}`Table 7-3 <table-7-3>`.

### Multi-Core Debug Control

The multi-core debug control interface enables an SoC-based Multi-Processor Controller (MPC) to:

- Control the reset state of the thread (i.e., either start executing or enter Debug Mode),
- halt (i.e., enter Debug Mode) or restart (i.e., resume execution) the thread,
- get an indication when the thread is in Debug Mode, and
- cross-trigger other cores or threads when this thread has entered Debug Mode due to a software or a hardware breakpoint.

The multi-core debug control interface signals are described in {ref}`Table 7-4 <table-7-4>`.

## Thread Support

Each thread's power management and debug actions are separate from the other.
Consequently, the VeeR EH2 core provides separate per-thread PMU and MPC control interfaces.
Also, each thread may enter and exit firmwareinitiated halt independently of each other.

In addition, a reset of the core affects the two threads differently.
Hart0 (the master thread) starts executing (or enters Debug Mode) immediately when reset is deasserted, but hart1 remains idle.
It is hart0's responsibility to initialize hart1's data structures before starting hart1.

The VeeR EH2 core also provides 'hart started' indication signals on its periphery to allow other SoC IPs to determine if hart1 has been started.

:::{note}
While hart1 is in the idle state, it does not acknowledge PMU or MPC requests.
These requests stay pending until hart1 has been started.
At that point, it behaves the same way as hart0 does out of reset.
:::

:::{note}
For hart1, the `mcycle` performance counter and `mitcnt`*X* internal timer counters are held in reset until hart1 has been started (i.e., has exited the idle state).
:::

## Power States

From a system's perspective, threads may be placed in one of three power states: Active (C0), Sleep (C3), and Power Off (C6).
Per-thread Active and Sleep states require hardware support from the harts, but in the Power Off state the core is power-gated so no special hardware support is needed.

{ref}`Figure 7-1 <figure-7-1>` depicts and {ref}`Table 7-2 <table-7-2>` describes the thread activity states as well as the events to transition between them.

```{figure} img/activity_states.png
:name: figure-7-1

VeeR EH2 Core Activity States
```

:::{note}
'Thread Quiesced' implies that no new instructions are executed and all outstanding thread-initiated bus transactions are completed (i.e., the unified buffer is empty, and all outstanding I-cache misses are finished).
Note that the store queue and the DMA FIFO might not be empty due to on-going DMA transactions.
:::

:::{list-table} Per-Thread Debug Resume Requests
:name: table-7-1
:header-rows: 2

* - Thread-Internal State
  - Thread-Internal State
  - Thread-Internal State
  - Thread-Internal State
  - Thread-Internal State
  - Thread-Internal State
  - Comments
* - Debug Resume
  - Debug Halt
  - MPC Halt
  - MPC Run
  - Halted (This Cycle)
  - Halted (Next Cycle)
  - Comments
* - 0
  - 0
  - 0
  - 0
  - 0
  - 0
  - No request for Debug Mode entry
* - 0
  - 0
  - 0
  - 1
  -
  -
  - No action required from core (requires coordination outside of core)
* - 0
  - 0
  - 1
  - 0
  - 1
  - 1
  - Waiting for MPC Run (thread remains in 'db-halt' state)
* - 0
  - 0
  - 1
  - 1
  - 1
  - 0
  - MPC Run Ack
* - 0
  - 1
  - 0
  - 0
  - 1
  - 1
  - Waiting for Debug Resume (thread remains in 'db-halt' state)
* - 0
  - 1
  - 0
  - 1
  -
  -
  - No action required from core (requires coordination outside of core)
* - 0
  - 1
  - 1
  - 0
  - 1
  - 1
  - Waiting for both MPC Run and Debug Resume (thread remains in 'db-halt' state)
* - 0
  - 1
  - 1
  - 1
  - 1
  - 1
  - Waiting for Debug Resume (thread remains in 'db-halt' state)
* - 1
  - 0
  - 0
  - 0
  -
  -
  - No action required from core (requires coordination outside of core)
* - 1
  - 0
  - 0
  - 1
  -
  -
  - No action required from core (requires coordination outside of core)
* - 1
  - 0
  - 1
  - 0
  -
  -
  - No action required from core (requires coordination outside of core)
* - 1
  - 0
  - 1
  - 1
  -
  -
  - No action required from core (requires coordination outside of core)
* - 1
  - 1
  - 0
  - 0
  - 1
  - 0
  - Debug Resume Ack
* - 1
  - 1
  - 0
  - 1
  -
  -
  - No action required from core (requires coordination outside of core)
* - 1
  - 1
  - 1
  - 0
  - 1
  - 1
  - Waiting for MPC Run (thread remains in 'db-halt' state)
* - 1
  - 1
  - 1
  - 1
  - 1
  - 0
  - Debug Resume Ack and MPC Run Ack
:::

:::{note}
While in 'db-halt' state, hardware ignores Debug Resume requests if the corresponding 'Debug Halt' state is not `1`.
Likewise, hardware ignores MPC Debug Run requests if the corresponding 'MPC Halt' state is not `1`.
:::

:::{note}
The thread-internal state bits are cleared upon exiting Debug Mode.
:::

:::{note}
In the time period between an MPC Debug Halt request and an MPC Debug Run request, a thread debug single-step action is stalled but stays pending.
:::

:::{note}
Even if the thread is already in Debug Mode due to a previous MPC Debug Halt request, a thread debugger must initiate a debug halt (i.e., Thread Debug Halt request) before it may start issuing other debug commands.
However, if Debug Mode was entered due to a thread debug breakpoint, a Thread Debug Halt request is not required.
:::

:::{note}
An MPC Debug Halt request may only be signaled when the thread is either not in Debug Mode or is already in Debug Mode due to a previous Thread Debug Halt request or a debug breakpoint or trigger.
Also, an MPC Debug Run request may only be signaled when the thread is in Debug Mode due to either a previous MPC Debug Halt request, a previous Thread Debug Halt request, or a debug breakpoint or trigger.
Issuing more than one MPC Debug Halt requests in succession or more than one MPC Debug Run requests in succession is a protocol violation.
:::

:::{list-table} Thread Activity States
:name: table-7-2
:header-rows: 1

* -
  - Active (C0)
  - Active (C0)
  - Sleep (C3)
* -
  - Running
  - Halted
  - Halted
* -
  -
  - db-halt
  - pmu/fw-halt
* - State Description
  - Thread operating normally
  - Thread halted in Debug Mode
  - Thread halted by PMU halt request or by thread firmware-initiated halt
* - Power Savings
  - Fine-grain clock gating integrated in core minimizes power consumption during regular operation
  - Fine-grain clock gating
  - Enhanced clock gating in addition to fine-grain clock gating
* - DMA Access
  -
  - DMA accesses allowed
  -
* - State Indication
  - * `cpu_halt_status[0/1]` is low
    * `debug_mode_status[0/1]` is low (except for Thread Debug Resume request with Single Step action)
  - * `cpu_halt_status[0/1]` is low
    * `debug_mode_status[0/1]` is high
  - * `cpu_halt_status[0/1]` is high
    * `debug_mode_status[0/1]` is low
* - Internal Timer Counters
  - `mitcnt0/1` incremented every core clock cycle (also during execution of instructions while single- stepping in Debug Mode)
  - `mitcnt0/1` not incremented
  - Depends on *halt_en* bit in `mitctl0`/1 registers: 0: `mitcnt0/1` not incremented 1: `mitcnt0/1` incremented every core clock cycle
* - Machine Cycle Performance-Monitoring Counter[^36]
  - `mcycle` incremented every core clock cycle
  - Depends on *stopcount* bit of `dcsr` register (see Section 11.1.3.5): 0: `mcycle` incremented every core clock cycle 1: `mcycle` not incremented
  - `mcycle` not incremented
:::

## Power Control

The priority order of simultaneous halt requests is as follows:

1. Any thread debug halt action:
    - a. Thread debug halt request
    - b. Thread debug single step
    - c. Thread debug breakpoint
    - d. Thread debug trigger

    or MPC debug halt request

2. PMU halt request or thread firmware-initiated halt

If the PMU sends a halt request while the thread is in Debug Mode, the thread disregards the halt request.
If the PMU's halt request is still pending when the thread exits Debug Mode, the request is honored at that time.
Similarly, thread firmware can't initiate a halt while in Debug Mode.
However, it is not possible for a thread firmware-initiated halt request to be pending when the thread exits Debug Mode.

:::{important}
There are two separate sources of debug operations: the hart (thread) itself which conforms to the standard RISC-V Debug specification [[3]](intro.md#ref-3), and the Multi-Processor Controller (MPC) IP block which provides multicore debug capabilities.
These two sources may interfere with each other and need to be carefully coordinated on a higher level outside the core.
Unintended behavior might occur if simultaneous debug operations from these two sources are not synchronized (e.g., MPC requesting a resume during the execution of an abstract command initiated by the debugger attached to the JTAG port).
:::

### Debug Mode

Debug Mode must be able to seize control of the thread.
Therefore, debug has higher priority than power control.
Debug Mode is entered under any of the following conditions:

- Thread debug halt request
- Thread debug single step
- Thread debug breakpoint with halt action
- Thread debug trigger with halt action
- Multi-core debug halt request (from MPC)

Debug Mode is exited with:

- Thread debug resume request with no single step action
- Multi-core debug run request (from MPC)

The state 'db-halt' is the only halt state allowed while in Debug Mode.

#### Single Stepping

A few notes about executing single-stepped instructions:

- Executing instructions which attempt to exit Debug Mode are ignored (e.g., writing to the `mpmc` register requesting to halt the thread does not transition the thread to the pmu/fw-halt state).
- Accesses to D-mode registers are illegal, even though the thread is in Debug Mode.
- A thread debug single-step action initiated in the time period between an MPC Debug Halt request and an MPC Debug Run request is stalled but stays pending until an MPC Debug Run request is issued.

#### Forced Debug Halt

Upon receiving a debug halt request (i.e., either a Thread Debug or MPC Debug Halt request, or a breakpoint or trigger to Debug Mode), the thread is typically quiesced before the Debug Halt (db-halt) state is entered.
However, LSU or IFU bus transactions may not complete due to SoC or other issues outside the core which may stop the thread from executing.
This may prevent the thread from entering the Debug Halt state after a debug halt request has been received.
To enable a debugger taking control of the thread, ongoing LSU and IFU bus transactions may be terminated after a programmable timeout period (see Section 7.6.3) has passed, forcing the thread into the Debug Halt state.
Once the debugger has control of the thread, it may read a status register (see Section 7.6.4) to inquire if LSU or IFU bus transactions have been terminated and data might have been lost.

:::{note}
This feature is targeted at allowing a debugger to take control of a hung thread.
Therefore, the timeout period should be set high enough to cover any reasonable delay incurred by any access to SoC memory locations and devices.
This should include potential additional delays due to congestion in the interconnect and other possible temporary conditions.
If the timeout period is long enough for all outstanding transactions to gracefully finish, program execution may be resumed after debugging has been performed.
However, if any outstanding transactions are prematurely forced to terminate, successfully resuming program execution after debug should not be expected because the data of terminated transactions may have been lost and possibly even a reset of the SoC might be necessary to bring the system back into a consistent state.
:::

### Thread Power and Multi-Core Debug Control and Status Signals

{ref}`Figure 7-2 <figure-7-2>` depicts the power and multi-core debug control and status signals which connect the VeeR EH2 core to the PMU and MPC IPs.
Signals from the PMU and MPC to the core are asynchronous and must be synchronized to the core clock domain.
Similarly, signals from the core are asynchronous to the PMU and MPC clock domains and must be synchronized to the PMU's or MPC's clock, respectively.

:::{note}
The synchronizers of the `cpu_run_req[0/1]` signals must not be clock-gated.
Otherwise, the thread may not be woken up again via the PMU interface.
:::

```{figure} img/power_debug_signals.png
:name: figure-7-2

VeeR EH2 Power and Multi-Core Debug Control and Status Signals
```

#### Power Control and Status Signals

There are three types of signals between the Power Management Unit and the VeeR EH2 core, as described in {ref}`Table 7-3 <table-7-3>`.
All signals are active-high.

:::{list-table} VeeR EH2 Power Control and Status Signals
:name: table-7-3
:header-rows: 1

* - Signal(s)
  - Description
* - `cpu_halt_req`[0/1]` and `cpu_halt_ack[0/1]`
  - Full handshake to request the thread to halt.
    The PMU requests the thread to halt (i.e., enter pmu/fw-halt) by asserting the `cpu_halt_req[0/1]` signal.
    The thread is quiesced before halting.
    The thread then asserts the `cpu_halt_ack[0/1]` signal.
    When the PMU detects the asserted `cpu_halt_ack[0/1]` signal, it deasserts the `cpu_halt_req[0/1]` signal.
    Finally, when the thread detects the deasserted `cpu_halt_req[0/1]` signal, it deasserts the `cpu_halt_ack[0/1]` signal.

    :::{note}
    `cpu_halt_req[0/1]` must be tied to `0` if PMU interface is not used.
    :::
* - `cpu_run_req[0/1]` and `cpu_run_ack[0/1]`
  - Full handshake to request the thread to run.
    The PMU requests the thread to run by asserting the `cpu_run_req[0/1]` signal.
    The thread exits the halt state and starts execution again.
    The thread then asserts the `cpu_run_ack[0/1]` signal.
    When the PMU detects the asserted `cpu_run_ack[0/1]` signal, it deasserts the `cpu_run_req[0/1]` signal.
    Finally, when the thread detects the deasserted `cpu_run_req[0/1]` signal, it deasserts the `cpu_run_ack[0/1]` signal.

    :::{note}
    `cpu_run_req[0/1]` must be tied to `0` if PMU interface is not used.
    :::
* - `cpu_halt_status[0/1]`
  - Indication from the thread to the PMU that it has been gracefully halted.
:::

:::{note}
Power control protocol violations (e.g., simultaneously sending a run and a halt request) may lead to unexpected behavior.
:::

{ref}`Figure 7-3 <figure-7-3>` and {ref}`Figure 7-4 <fig-pmu-run>` depict conceptual timing diagrams of a halt and a run request.
Note that entering Debug Mode is an asynchronous event relative to power control commands sent by the PMU.
Debug Mode has higher priority and can interrupt and override PMU requests.

##### PMU Halt Request
```{figure} img/pmu_halt_timing.png
:name: figure-7-3

VeeR EH2 PMU Halt Request Timing Diagram
```

##### PMU Run Request
```{figure} img/pmu_run_timing.png
:name: fig-pmu-run

VeeR EH2 PMU Run Request Timing Diagram
```

#### Multi-Core Debug Control and Status Signals

There are five types of signals between the Multi-Processor Controller and the VeeR EH2 core, as described in {ref}`Table 7-4 <table-7-4>`.
All signals are active-high.

:::{list-table} VeeR EH2 Multi-Core Debug Control and Status Signals
:name: table-7-4
:header-rows: 1

* - Signal(s)
  - Description
* - `mpc_debug_halt_req[0/1]` and `mpc_debug_halt_ack[0/1]`
  - Full handshake to request the thread to debug halt.
    The MPC requests the thread to halt (i.e., enter 'db-halt') by asserting the `mpc_debug_halt_req[0/1]` signal.
    The thread is quiesced before halting.
    The thread then asserts the `mpc_debug_halt_ack[0/1]` signal.
    When the MPC detects the asserted `mpc_debug_halt_ack[0/1]` signal, it deasserts the `mpc_debug_halt_req[0/1]` signal.
    Finally, when the thread detects the deasserted `mpc_debug_halt_req[0/1]` signal, it deasserts the `mpc_debug_halt_ack[0/1]` signal.
    For as long as the `mpc_debug_halt_req[0/1]` signal is asserted, the thread must assert and hold the `mpc_debug_halt_ack[0/1]` signal whether it was already in 'db-halt' or just transitioned into 'db-halt' state.

    :::{note}
    The *cause* field of the thread's `dcsr` register (see Section 11.1.3.5) is set to 3 (i.e., the same value as a debugger-requested entry to Debug Mode due to a Thread Debug Halt request).
    Similarly, the `dpc` register (see Section 11.1.3.6) is updated with the address of the next instruction to be executed at the time that Debug Mode was entered.
    :::

    :::{note}
    Signaling more than one MPC Debug Halt request in succession is a protocol violation.
    :::

    :::{note}
    `mpc_debug_halt_req[0/1]` must be tied to `0` if MPC interface is not used.
    :::
* - `mpc_debug_run_req[0/1]` and `mpc_debug_run_ack[0/1]`
  - Full handshake to request the thread to run.
    The MPC requests the thread to run by asserting the `mpc_debug_run_req[0/1]` signal.
    The thread exits the halt state and starts execution again.
    The thread then asserts the `mpc_debug_run_ack[0/1]` signal.
    When the MPC detects the asserted `mpc_debug_run_ack[0/1]` signal, it deasserts the `mpc_debug_run_req[0/1]` signal.
    Finally, when the thread detects the deasserted `mpc_debug_run_req[0/1]` signal, it deasserts the `mpc_debug_run_ack[0/1]` signal.
    For as long as the `mpc_debug_run_req[0/1]` signal is asserted, the thread must assert and hold the `mpc_debug_run_ack[0/1]` signal whether it was already in 'Running' or after transitioning into 'Running' state.

    :::{note}
    The thread remains in the 'db-halt' state if a thread debug request is also still active.
    :::

    :::{note}
    Signaling more than one MPC Debug Run request in succession is a protocol violation .
    :::

    :::{note}
    `mpc_debug_run_req[0/1]` must be tied to `0` if MPC interface is not used.
    :::
* - `mpc_reset_run_req[0/1]`
  - Thread start state control out of reset: 1: Normal Mode ('Running' or 'pmu/fw-halt' state) 0: Debug Mode halted ('db-halt' state)

    :::{note}
    The core complex does not implement a synchronizer for this signal because the timing of the first clock is critical.
    It must be synchronized to the core clock domain outside the core in the SoC.
    :::

    :::{note}
    For hart1 (T1), `mpc_reset_run_req[1]` must be stable from before the core reset is deasserted until after the starting of hart1 has been reported in the `mhartstart` register (see Section 5.4.2).
    :::

    :::{note}
    `mpc_reset_run_req[0/1]` must be tied to `1` if MPC interface is not used.
    :::
* - `debug_mode_status[0/1]`
  - Indication from the thread to the MPC that it is currently transitioning to or already in Debug Mode.
* - `debug_brkpt_status[0/1]`
  - Indication from the thread to the MPC that a software (i.e., ebreak instruction) or hardware (i.e., trigger hit) breakpoint has been triggered in the thread.
    The breakpoint signal is only asserted for breakpoints and triggers with debug halt action.
    The signal is deasserted on exiting Debug Mode.
:::

:::{note}
Multi-core debug control protocol violations (e.g., simultaneously sending a run and a halt request) may lead to unexpected behavior.
:::

:::{note}
If the thread is either not in the db-halt state (i.e., `debug_mode_status[0/1]` indication is not asserted) or is already in the db-halt state due to a previous Thread Debug Halt request or a debug breakpoint or trigger (i.e., `debug_mode_status[0/1]` indication is already asserted), asserting the `mpc_debug_halt_req[0/1]` signal is allowed and acknowledged with the assertion of the `mpc_debug_halt_ack[0/1]` signal.
Also, asserting the `mpc_debug_run_req[0/1]` signal is only allowed if the thread is in the db-halt state (i.e.,
:::

`debug_mode_status[0/1]` indication is asserted), but the thread asserts the `mpc_debug_run_ack[0/1]` signal only after the `cpu_run_req[0/1]` signal on the PMU interface has been asserted as well, if a PMU Halt request was still pending.

:::{note}
If the MPC is requesting the thread to enter Debug Mode out of reset by activating the `mpc_reset_run_req[0/1]` signal, the `mpc_debug_run_req[0/1]` signal may not be asserted until the thread is out of reset and has entered Debug Mode.
Violating this rule may lead to unexpected thread behavior.
:::

:::{note}
If Debug Mode is entered at reset by setting the `mpc_reset_run_req` signal to `0`, only a run request issued on the `mpc_debug_run_req/ack` interface allows the core to exit Debug Mode.
A core debug resume request issued by the debugger does not transition the core out of Debug Mode.
:::

{ref}`Figure 7-5 <figure-7-5>` and {ref}`Figure 7-6 <fig-mpc-run>` depict conceptual timing diagrams of a halt and a run request.

##### MPC Halt Request:

```{figure} img/mpc_halt_timing.png
:name: figure-7-5

VeeR EH2 MPC Halt Request Timing Diagram
```
1\) if thread not already quiesced and in Debug Mode due to earlier Thread Debug Halt request (i.e., in active thread debug session)

##### MPC Run Request:

```{figure} img/mpc_run_timing.png
:name: fig-mpc-run

VeeR EH2 MPC Run Request Timing Diagram
```
2\) if in active thread debug session

{ref}`Figure 7-7 <figure-7-7>` depicts conceptual timing diagrams of the breakpoint indication.

### Breakpoint Signal Assertion:

```{figure} img/breakpoint_timing_assertion.png
:name: figure-7-7

VeeR EH2 Breakpoint Signal Assertion Timing Diagrams
```
1\) if thread not already quiesced and in Debug Mode due to earlier Thread Debug Halt request (i.e., in active thread debug session)

### Breakpoing Signal Deassertion:
```{figure} img/breakpoint_timing_deassertion.png
:name: figure-7-8

VeeR EH2 Breakpoint Signal Deassertion Timing Diagrams
```

### Debug Scenarios

The following mixed thread debug and MPC debug scenarios are supported by the core:

#### Scenario 1: Thread Halt → MPC Halt → MPC Run → Thread Resume

1. Thread debugger asserts a Debug Halt request which results in the thread transitioning into Debug Halt state (db-halt).
2. In the system, another processor hits a breakpoint.
   The MPC signals a Debug Halt request to all processors to halt.
3. Thread acknowledges this Debug Halt request as it is already in Debug Halt state (db-halt).
4. MPC signals a Debug Run request, but thread is in the middle of a thread debugger operation (e.g., an Abstract Command-based access) which requires it to remain in Debug Halt state.
5. Thread completes debugger operation and waits for Thread Debug Resume request from the thread debugger.
6. When thread debugger sends a Debug Resume request, the thread then transitions to the Running state and deasserts the `debug_mode_status[0/1]` signal.
7. Finally, thread acknowledges MPC Debug Run request.

#### Scenario 2: Thread Halt → MPC Halt → Thread Resume → MPC Run

1. Thread debugger asserts a Debug Halt request which results in the thread transitioning into Debug Halt state (db-halt).
2. In the system, another processor hits a breakpoint.
   The MPC signals Debug Halt request to all processors to halt.
3. Thread acknowledges this Debug Halt request as it is already in Debug Halt state (db-halt).
4. Thread debugger completes its operations and sends a Debug Resume request to the thread.
5. Thread remains in Halted state as MPC has not yet asserted its Debug Run request.
   The `debug_mode_status[0/1]` signal remains asserted.
6. When MPC signals a Debug Run request, the thread then transitions to the Running state and deasserts the `debug_mode_status[0/1]` signal.
7. Finally, thread acknowledges MPC Debug Run request.

#### Scenario 3: MPC Halt → Thread Halt → Thread Resume → MPC Run

1. MPC asserts a Debug Halt request which results in the thread transitioning into Debug Halt state (db-halt).
2. Thread acknowledges this Debug Halt request.
3. Thread debugger signals a Debug Halt request to the thread.
   Thread is already in Debug Halt state (dbhalt).
4. Thread debugger completes its operations and sends a Debug Resume request to the thread.
5. When MPC signals a Debug Run request, the thread then transitions to the Running state and deasserts the `debug_mode_status[0/1]` signal.
6. Thread remains in Halted state as MPC has not yet asserted its Debug Run request.
   The `debug_mode_status[0/1]` signal remains asserted.
7. Finally, thread acknowledges MPC Debug Run request.

#### Scenario 4: MPC Halt → Thread Halt → MPC Run → Thread Resume

1. MPC asserts a Debug Halt request which results in the thread transitioning into Debug Halt state (db-halt).
2. Thread acknowledges this Debug Halt request.
3. Thread debugger signals a Debug Halt request to the thread.
   Thread is already in Debug Halt state (dbhalt).
4. MPC signals a Debug Run request, but thread debugger operations are still in progress.
   Thread remains in Halted state.
   The `debug_mode_status[0/1]` signal remains asserted.
5. Thread debugger completes operations and signals a Debug Resume request to the thread.
6. The thread then transitions to the Running state and deasserts the `debug_mode_status[0/1]` signal.
7. Finally, thread acknowledges MPC Debug Run request.

#### Summary

For the thread to exit out of Debug Halt state (db-halt) in cases where it has received debug halt requests from both thread debugger and MPC, it must receive debug run requests from both the thread debugger as well as the MPC, irrespective of the order in which debug halt requests came from both sources.
Until then, the thread remains halted and the `debug_mode_status[0/1]` signal remains asserted.

### Thread Wake-Up Events

When not in Debug Mode (i.e., the thread is in pmu/fw-halt state), a thread is woken up on several events:

- PMU run request
- Highest-priority external interrupt (`mhwakeup[0/1]` signal from PIC) and interrupts are enabled (per thread)
- Software interrupt (per thread)
- Timer interrupt (per thread)
- Internal timer interrupt (per thread)
- Non-maskable interrupt (NMI) (`nmi_int` signal, if thread is selected to handle NMI (see Section 5.4.3))

The PIC is part of the core logic and the `mhwakeup[0/1]` signals are connected directly inside the core.
The internal timers are part of the core and internally connected as well.
The standard RISC-V software and timer interrupt as well as NMI signals are external to the core and originate in the SoC.
If desired, these signals can be routed through the PMU and further qualified there.

### Thread Firmware-Initiated Halt

The firmware running on a thread may also initiate a halt by writing a `1` to the *halt* field of the `mpmc` register (see Section 7.6.1).
The thread is quiesced before indicating that it has gracefully halted.

### DMA Operations While Halted

When the thread is halted in the 'pmu/fw-halt' or the 'db-halt' state, DMA operations are supported.

### External Interrupts While Halted

All non-highest-priority external interrupts are temporarily ignored while halted.
Only external interrupts which activate the `mhwakeup[0/1]` signals (see Section 8.5.2, Steps 14 and 15) are honored, if the thread is enabled to service external interrupts (i.e., the *mie* bit of the `mstatus` and the *meie* bit of the *mie* standard RISC-V registers are both set, otherwise the thread remains in the 'pmu/fw-halt' state).
External interrupts which are still pending and have a sufficiently high priority to be signaled to the thread are serviced once the thread is back in the Running state.

## Control/Status Registers

A summary of platform-specific control/status registers in CSR space:

- Power Management Control Register (`mpmc`) (see Section 7.6.1)
- Forced Debug Halt Threshold Register (`mfdht`) (see Section 7.6.3)
- Core Pause Control Register (`mcpc`) (see Section 7.6.2)
- Forced Debug Halt Status Register (`mfdhs`) (see Section 7.6.4)

All reserved and unused bits in these control/status registers must be hardwired to `0`.
Unless otherwise noted, all read/write control/status registers must have WARL (Write Any value, Read Legal value) behavior.

### Power Management Control Register (`mpmc`)

The `mpmc` register provides thread power management control functionality.
It allows the firmware running on the thread to initiate a transition to the Halted (pmu/fw-halt) state.
While entering the Halted state, interrupts may optionally be enabled atomically.

The *halt* field of the `mpmc` register has W1R0 (Write 1, Read 0) behavior, as also indicated in the 'Access' column.

:::{note}
Writing a `1` to the *haltie* field of the `mpmc` register without also setting the *halt* field has no immediate effect on the *mie* bit of the `mstatus` register.
However, the *haltie* field of the `mpmc` register is updated accordingly.
:::

:::{note}
Once the *mie* bit of the `mstatus` register is set via the *haltie* field of the `mpmc` register, it remains set until other operations clear it.
Exiting the Halted (pmu/fw-halt) state does not clear the *mie* bit of the `mstatus` register set by entering the Halted state.
:::

:::{note}
In Debug Mode, writing (i.e., setting or clearing) *haltie* has no effect on the `mstatus` register's *mie* bit since the thread does not transition to the Halted (pmu/fw-halt) state.
:::

This register is mapped to the non-standard read/write CSR address space and hart-specific (i.e., a separate register per thread).

:::{list-table} Power Management Control Register (`mpmc`, at CSR 0x7C6)
:name: table-7-5
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
* - haltie
  - 1
  - Control interrupt enable (i.e., *mie* bit of `mstatus` register) when transitioning to Halted (pmu/fw-halt) state by setting *halt* bit below:
    * 0: Don't change *mie* bit of `mstatus` register
    * 1: Set *mie* bit of `mstatus` register (i.e., atomically enable interrupts)
  - R/W
  - 1
* - halt
  - 0
  - Initiate thread halt (i.e., transition to Halted (pmu/fw-halt) state)

    :::{note}
    Write ignored if in Debug Mode
    :::
  - R0/W1
  - 0
:::

### Core Pause Control Register (`mcpc`)

The `mcpc` register supports functions to temporarily stop the thread from executing instructions.
This helps to save thread power since busy-waiting loops can be avoided in the firmware.

PAUSE stops the thread from executing instructions for a specified number [^37] of clock ticks or until an interrupt is received.

:::{note}
PAUSE is a long-latency, interruptible instruction and does not change the thread's activity state (i.e., the thread remains in the Running state).
Therefore, even though this function may reduce core power, it is not part of thread power management.
:::

:::{note}
PAUSE has a skid of several cycles.
Therefore, instruction execution might not be stopped for precisely the number of cycles specified in the *pause* field of the `mcpc` register.
However, this is acceptable for the intended use case of this function.
:::

:::{note}
Depending on the *pause_en* bit of the `mitctl0`/1 registers, the internal timers might be incremented while executing PAUSE.
If an internal timer interrupt is signaled, PAUSE is terminated and normal execution resumes.
:::

:::{note}
If the PMU sends a halt request while PAUSE is still executing, the thread enters the Halted (pmu/fw-halt) state and the pause clock counter stops until the thread is back in the Running state.
:::

:::{note}
WFI is another candidate for a function that stops the thread temporarily.
Currently, the WFI instruction is implemented as NOP, which is a fully RISC-V-compliant option.
:::

The *pause* field of the `mcpc` register has WAR0 (Write Any value, Read 0) behavior, as also indicated in the 'Access' column.

This register is mapped to the non-standard read/write CSR address space and hart-specific (i.e., a separate register per thread).

:::{list-table} Thread Pause Control Register (`mcpc`, at CSR 0x7C2)
:name: table-7-6
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - pause
  - 31:0
  - Pause execution for number of core clock cycles specified

    :::{note}
    pause is decremented by 1 for each core clock cycle.
    Execution continues either when pause is 0 or any interrupt is received.
    :::
  - R0/W
  - 0
:::

### Forced Debug Halt Threshold Register (`mfdht`)

The `mfdht` register hosts the enable bit of the forced debug halt mechanism as well as the power-of-two exponent of the timeout threshold.
When enabled, if a debug halt request is received and LSU and/or IFU bus transactions are pending, a per-thread internal timeout counter starts incrementing with each core clock and keeps incrementing until the Debug Halt (db-halt) state is entered.
If all ongoing bus transactions complete within the timeout period and the thread is quiesced, the Debug Halt state is entered as usual.
However, if the timeout counter value is equal to or greater than the threshold value (= $2^{thresh}$ core clocks), all in-progress LSU and IFU bus transactions are terminated and the Debug Halt state is entered (i.e., the thread may be forced to the Debug Halt state before it is fully quiesced).
In addition, when entering the Debug Halt state in either case, the `mfdhs` register (see Section 7.6.4 below) latches the status if any LSU or IFU bus transactions have been prematurely terminated.

:::{note}
The internal timeout counter is cleared at reset as well as when the Debug Halt (db-halt) state is exited.
:::

:::{note}
The 5-bit threshold (*thresh* field) allows a timeout period of up to $2^{31}$ core clock cycles (i.e., about 2.1 seconds at a 1GHz core clock frequency).
:::

This register is mapped to the non-standard read/write CSR address space and shared by the harts (i.e., one register per core).

:::{list-table} Forced Debug Halt Threshold Register (`mfdht`, at CSR 0x7CE)
:name: table-7-7
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - Reserved
  - 31:6
  - Reserved
  - R
  - 0
* - thresh
  - 5:1
  - Power-of-two exponent of timeout threshold (= $2^{thresh}$ core clock cycles)
  - R/W
  - 0
* - enable
  - 0
  - Enable/disable forced debug halt timeout:
    * 0: Timeout mechanism disabled (default)
    * 1: Timeout mechanism enabled
  - R/W
  - 0
:::

### Forced Debug Halt Status Register (`mfdhs`)

The `mfdhs` register provides status information if any LSU and/or IFU bus transactions have been prematurely terminated when the Debug Halt (db-halt) state has been entered.
A debugger may read this register to inquire if any bus transactions have been terminated and data may have been lost while entering the Debug Halt state.
If both status bits are `0` indicates that the thread was properly quiesced.

:::{note}
A debugger may also clear the status bits if desired, but clearing is not required for proper operation.
:::

This register is mapped to the non-standard read/write CSR address space and hart-specific (i.e., a separate register per thread).

:::{list-table} Forced Debug Halt Status Register (`mfdhs`, at CSR 0x7CF)
:name: table-7-8
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
* - lsu
  - 1
  - LSU bus transaction termination status:
    * 0: No transactions have been prematurely terminated
    * 1: One or more transactions have been prematurely terminated
  - R/W
  - 0
* - ifu
  - 0
  - IFU bus transaction termination status:
    * 0: No transactions have been prematurely terminated
    * 1: One or more transactions have been prematurely terminated
  - R/W
  - 0
:::

[^36]: Note that the mcycle/mcycleh registers are implemented per thread (i.e., per hart) in the SweRV EH2 core, whereas in other cores these registers may be implemented per core.
[^37]: The field width provided by the `mcpc` register allows to pause execution for about 4 seconds at a 1 GHz core clock.
