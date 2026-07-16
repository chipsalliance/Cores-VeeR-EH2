(chap-debugging)=
# VeeR EH2 Debug Support

The VeeR EH2 core conforms to the 'RISC-V Debug Specification 0.13.2, with JTAG DTM' document [[3]](intro.md#ref-3).
This chapter provides a description of the implemented debug-related control and status register definitions.
For a RISC-V debug overview and detailed feature descriptions, refer to corresponding sections in [[3]](intro.md#ref-3).

## Control/Status Registers

The RISC-V Debug architecture defines three separate address spaces: JTAG, Debug Module Interface, and RISC-V CSR.
The registers associated with these three address spaces are described in the following sections:

- Control/Status Registers in JTAG Address Space (see Section 11.1.1)
- Control/Status Registers in RISC-V CSR Address Space (see Section 11.1.3)
- Control/Status Registers in Debug Module Interface Address Space (see Section 11.1.2)

### Control/Status Registers in JTAG Address Space

{ref}`Table 11-1 <table-11-1>` summarizes the control/status registers in the JTAG Debug Transport Module address space.

Addresses shown below are in the 5-bit JTAG address space.
A control/status register is addressed by setting the 5bit JTAG IR register.

:::{note}
The core complex clock (`clk`) frequency must be at least twice the JTAG clock (`jtag_tck`) frequency for the JTAG data to pass correctly through the clock domain crossing synchronizers.
:::

:::{list-table} Registers in JTAG Debug Transport Module Address Space
:name: table-11-1
:header-rows: 1

* - JTAG DTM Address
  - Name
  - Description
  - Scope [^49]
  - Section
* - 0x01
  - `IDCODE`
  - JTAG `IDCODE`
  - C
  - 10.1.1.1
* - 0x10
  - `dtmcs`
  - DTM control and status
  - C
  - 10.1.1.2
* - 0x11
  - `dmi`
  - Debug module interface access
  - C
  - 10.1.1.3
* - 0x1F
  - `BYPASS`
  - JTAG `BYPASS`
  - C
  - 10.1.1.4
:::

#### `IDCODE` Register (`IDCODE`)

The `IDCODE` register is a standard JTAG register.
It is selected in the JTAG TAP controller's IR register when the TAP state machine is reset.
The `IDCODE` register's definition is exactly as defined in IEEE Std 1149.1-2013.

This register is read-only.

This register is mapped to the 5-bit JTAG address space and shared by the harts (i.e., one register per core).

:::{list-table} `IDCODE` Register (`IDCODE`, at JTAG 0x01)
:name: table-11-2
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - version
  - 31:28
  - Identifies release version of this part
  - R
  - `jtag_id`[31:28] value (see {ref}`Table 16-1 <table-16-1>`)
* - partnum
  - 27:12
  - Identifies designer's part number of this part
  - R
  - `jtag_id`[27:12] value (see {ref}`Table 16-1 <table-16-1>`)
* - manufid
  - 11:1
  - Identifies designer/manufacturer of this part
  - R
  - `jtag_id`[11:1] value (see {ref}`Table 16-1 <table-16-1>`)
* - 1
  - 0
  - Must be `1`
  - R
  - 1
:::

#### DTM Control and Status Register (`dtmcs`)

The `dtmcs` register controls and provides status of the Debug Transport Module (DTM).

This register is mapped to the 5-bit JTAG address space and shared by the harts (i.e., one register per core).

:::{list-table} DTM Control and Status Register (`dtmcs`, at JTAG 0x10)
:name: table-11-3
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - Reserved
  - 31:18
  - Reserved
  - R
  - 0
* - dmihardreset
  - 17
  - Not implemented

    :::{note}
    Hard reset of DTM not required in VeeR EH2 because DMI accesses always succeed.
    Writes to this bit ignored.
    :::
  - R
  - 0
* - dmireset
  - 16
  - Not implemented

    :::{note}
    Reset of DTM's error state not required in VeeR EH2 because DMI accesses always succeed.
    Writes to this bit ignored.
    :::
  - R
  - 0
* - Reserved
  - 15
  - Reserved
  - R
  - 0
* - idle
  - 14:12
  - Hint to debugger of minimum number of cycles debugger should spend in Run-Test/Idle after every DMI scan to avoid a 'busy' return code (`dmistat` of 3).
    Debugger must still check dmistat when necessary:
      * 0: Not necessary to enter Run-Test/Idle at all.

    Other values not implemented.
  - R
  - 0
* - dmistat
  - 11:10
  - DMI status:
    * 0: No error
    * 1: Reserved
    * 2..3: Not implemented (DMI accesses always succeed)
  - R
  - 0
* - abits
  - 9:4
  - Size of *address* field in `dmi` register (see {ref}`Table 11-4 <table-11-4>`)
  - R
  - 7
* - version
  - 3:0
  - Conforming to RISC-V Debug specification Version 0.13.2
  - R
  - 1
:::

#### Debug Module Interface Access Register (`dmi`)

The `dmi` register allows access to the Debug Module Interface (DMI).

In the JTAG TAP controller's Update-DR state, the DTM starts the operation specified in the *op* field.

In the JTAG TAP controller's Capture-DR state, the DTM updates the *data* field with the result from that operation.

:::{note}
No status is reported in the *op* field.
Therefore, debuggers should refrain from batching together multiple scans.
:::

This register is mapped to the 5-bit JTAG address space and shared by the harts (i.e., one register per core).

:::{list-table} Debug Module Interface Access Register (`dmi`, at JTAG 0x11)
:name: table-11-4
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - address
  - 40:34
  - Address used for DMI access.
    In Update-DR, value used to access DM over DMI.
  - R/W
  - 0
* - data
  - 33:2
  - Data to send to DM over DMI during Update-DR, and data returned from DM as result of previous operation.
  - R/W
  - 0
* - op
  - 1:0
  - * For write:
      * 0: Ignore data and address (nop)
      * 1: Read from address (read)
      * 2: Write data to address (write)
      * 3: Not implemented (do not use)
    * For read:
      * 0: Previous operation completed successfully
      * 1..3: Not implemented (DMI accesses always succeed)
  - R/W
  - 0
:::

#### `BYPASS` Register (`BYPASS`)

The `BYPASS` register is a standard JTAG register.
It is implemented as a 1-bit register which has no functional effect, except adding a 1-bit delay.
It allows a debugger to not communicate with this TAP (i.e., bypass it).

:::{note}
All unused addresses in the 5-bit JTAG address space (i.e., all addresses except 0x01 (`IDCODE`), 0x10 (`dtmcs`), and 0x11 (`dmi`)) select the `BYPASS` register as well.
:::

This register is mapped to the 5-bit JTAG address space and shared by the harts (i.e., one register per core).

:::{list-table} `BYPASS` Register (`BYPASS`, at JTAG 0x1F)
:name: table-11-5
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - bypass
  - 0
  - Bypass
  - \---
  - 0
:::

### Control/Status Registers in Debug Module Interface Address Space

{ref}`Table 11-6 <table-11-6>` summarizes the control/status registers in the Debug Module Interface address space.

Registers in the Debug Module Interface address space are accessed through the `dmi` register in the JTAG address space (see Section 11.1.1.3).
The *address* field of the `dmi` register selects the Debug Module Interface register to be accessed, the *data* field either provides the value to be written to the selected register or captures that register's value, and the *op* field selects the operation to be performed.

Addresses shown below are offsets relative to the Debug Module base address.
VeeR EH2 supports a single Debug Module with a base address of 0x00.

:::{list-table} Registers in Debug Module Interface Address Space
:name: table-11-6
:header-rows: 1

* - DMI Address
  - Name
  - Description
  - Scope [^50]
  - Section
* - 0x04
  - `data0`
  - Abstract data 0
  - C
  - 10.1.2.9
* - 0x05
  - `data1`
  - Abstract data 1
  - C
  - 10.1.2.9
* - 0x10
  - `dmcontrol`
  - Debug module control
  - C
  - 10.1.2.1
* - 0x11
  - `dmstatus`
  - Debug module status
  - C
  - 10.1.2.2
* - 0x14
  - `hawindowsel`
  - Hart array window select
  - C
  - 10.1.2.4
* - 0x15
  - `hawindow`
  - Hart array window
  - C
  - 10.1.2.5
* - 0x16
  - `abstractcs`
  - Abstract control and status
  - C
  - 10.1.2.6
* - 0x17
  - command
  - Abstract command
  - C
  - 10.1.2.7
* - 0x18
  - `abstractauto`
  - Abstract command autoexec
  - C
  - 10.1.2.8
* - 0x38
  - `sbcs`
  - System bus access control and status
  - C
  - 10.1.2.10
* - 0x39
  - `sbaddress0`
  - System bus address 31:0
  - C
  - 10.1.2.11
* - 0x3C
  - `sbdata0`
  - System bus data 31:0
  - C
  - 10.1.2.12
* - 0x3D
  - `sbdata1`
  - System bus data 63:32
  - C
  - 10.1.2.13
* - 0x40
  - `haltsum0`
  - Halt summary 0
  - C
  - 10.1.2.3
:::

:::{note}
ICCM, DCCM, and PIC memory ranges are only accessible using the access memory abstract command method.
SoC memories are accessible using either the access memory abstract command method or the system bus access method.
:::

:::{note}
Abstract commands may only be executed when the core is in the debug halt (db-halt) state.
However, SoC memory locations may be accessed using the system bus access method, irrespective of the core's state.
:::

#### Debug Module Control Register (`dmcontrol`)

The `dmcontrol` register controls the overall Debug Module as well as the currently selected harts, as defined in hasel of the `dmcontrol` register (see {ref}`Table 11-7 <table-11-7>`).

:::{note}
On any given write, a debugger may only write `1` to either the resumereq or ackhavereset bit.
The other bit must be written to `0`.
:::

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} Debug Module Control Register (`dmcontrol`, at Debug Module Offset 0x10)
:name: table-11-7
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - haltreq
  - 31
  - Halt request:
      * 0: Clears halt request bit

      **Note**: May cancel outstanding halt request.

      * 1: Sets halt request bit

      **Note**: Running hart halts whenever halt request bit is set.

    :::{note}
    Writes apply to new value of *hartsello* field and *hasel* bit.
    :::
  - R0/W
  - 0
* - resumereq
  - 30
  - Resume request:
      * 0: No effect
      * 1: Causes hart to resume, if halted

    :::{note}
    Also clears resume ack bit for hart.
    :::

    :::{note}
    Setting *resumereq* bit is ignored if *haltreq* bit is set.
    :::

    :::{note}
    Writes apply to new value of *hartsello* field and *hasel* bit.
    :::
  - R0/W1
  - 0
* - hartreset
  - 29
  - Not implemented (i.e., 0: Deasserted)
  - R
  - 0
* - ackhavereset
  - 28
  - Reset core-internal, sticky `havereset` state:
      * 0: No effect
      * 1: Clear havereset state

    :::{note}
    Writes apply to new value of *hartsello* field and *hasel* bit.
    :::
  - R0/W1
  - 0
* - Reserved
  - 27
  - Reserved
  - R
  - 0
* - hasel
  - 26
  - Selects definition of currently selected harts:
      * 0: Single hart currently selected by *hartsello* field
      * 1: All harts currently selected by *hartsello* field plus harts selected in hart array mask register (i.e., `hawindow` register, see Section 11.1.2.5)

    :::{note}
    Debugger may probe if hart array mask register feature is implemented by setting this bit and read it back.
    :::

    :::{note}
    Not implemented for single-thread VeeR EH2 core builds.
    :::
  - R/W (dual-thread core)

    R (single-thread core)
  - 0
* - hartsello
  - 25:17
  - Not implemented (VeeR EH2 is dual-thread)
  - R
  - 0
* -
  - 16
  - Hart select:
      * 0: Hart 0 selected
      * 1: Hart 1 selected

    :::{note}
    This hart is always part of the currently selected harts.
    :::

    :::{note}
    Not implemented for single-thread VeeR EH2 core builds.
    :::
  - R/W (dual-thread core)

    R (single-thread core)
  - 0
* - hartselhi
  - 15:6
  - Not implemented (VeeR EH2 is dual-thread)
  - R
  - 0
* - Reserved
  - 5:4
  - Reserved
  - R
  - 0
* - setresethaltreq
  - 3
  - Not implemented

    :::{note}
    *hasresethaltreq* bit in `dmstatus` register ({ref}`Table 11-8 <table-11-8>`) is `0`.
    :::
  - R
  - 0
* - clrresethaltreq
  - 2
  - Not implemented

    :::{note}
    *hasresethaltreq* bit in `dmstatus` register ({ref}`Table 11-8 <table-11-8>`) is `0`.
    :::
  - R
  - 0
* - ndmreset
  - 1
  - Controls reset signal from DM to VeeR EH2 core.
    Signal resets hart, but not DM.
    To perform a reset, debugger writes `1`, and then writes `0` to deassert reset.
  - R/W
  - 0
* - dmactive
  - 0
  - Reset signal for Debug Module (DM):
      * 0: Module's state takes its reset values

      **Note:** Only *dmactive* bit may be written to value other than its reset value. Writes to all other bits of this register are ignored.

      * 1: Module functions normally

    Debugger may pulse this bit low to get Debug Module into known state.

    :::{note}
    The core complex's `dbg_rst_l` signal (see {ref}`Table 16-1 <table-16-1>`) resets the Debug Module.
    It should only be used to reset the Debug Module at power up or possibly with a global reset signal which resets the entire platform.
    :::
  - R/W
  - 0
:::

#### Debug Module Status Register (`dmstatus`)

The `dmstatus` register reports status for the overall Debug Module as well as the currently selected harts, as defined in hasel of the `dmcontrol` register (see {ref}`Table 11-7 <table-11-7>`).

This register is read-only.

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} Debug Module Status Register (`dmstatus`, at Debug Module Offset 0x11)
:name: table-11-8
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - Reserved
  - 31:23
  - Reserved
  - R
  - 0
* - impebreak
  - 22
  - Not implemented

    :::{note}
    VeeR EH2 does not implement a Program Buffer.
    :::
  - R
  - 0
* - Reserved
  - 21:20
  - Reserved
  - R
  - 0
* - allhavereset
  - 19
  - `1` when all currently selected harts have been reset and reset has not been acknowledged for any

    :::{note}
    Status of hart0 for single-thread VeeR EH2 core builds.
    :::
  - R
  - \--
* - anyhavereset
  - 18
  - `1` when at least one currently selected hart has been reset and reset has not been acknowledged for that hart

    :::{note}
    Status of hart0 for single-thread VeeR EH2 core builds.
    :::
  - R
  - \--
* - allresumeack
  - 17
  - `1` when all currently selected harts have acknowledged their last resume request

    :::{note}
    Status of hart0 for single-thread VeeR EH2 core builds.
    :::
  - R
  - \--
* - anyresumeack
  - 16
  - `1` when any currently selected hart has acknowledged its last resume request

    :::{note}
    Status of hart0 for single-thread VeeR EH2 core builds.
    :::
  - R
  - \--
* - allnonexistent
  - 15
  - Not implemented
  - R
  - 0
* - anynonexistent
  - 14
  - Not implemented
  - R
  - 0
* - allunavail
  - 13
  - `1` when all currently selected harts are unavailable [^51]

    :::{note}
    Status of hart0 for single-thread VeeR EH2 core builds.
    :::
  - R
  - \--
* - anyunavail
  - 12
  - `1` when any currently selected hart is unavailable [^51]

    :::{note}
    Status of hart0 for single-thread VeeR EH2 core builds.
    :::
  - R
  - \--
* - allrunning
  - 11
  - `1` when all currently selected harts are running

    :::{note}
    Status of hart0 for single-thread VeeR EH2 core builds.
    :::
  - R
  - \--
* - anyrunning
  - 10
  - `1` when any currently selected hart is running

    :::{note}
    Status of hart0 for single-thread VeeR EH2 core builds.
    :::
  - R
  - \--
* - allhalted
  - 9
  - `1` when all currently selected harts are halted

    :::{note}
    Status of hart0 for single-thread VeeR EH2 core builds.
    :::
  - R
  - \--
* - anyhalted
  - 8
  - `1` when any currently selected hart is halted

    :::{note}
    Status of hart0 for single-thread VeeR EH2 core builds.
    :::
  - R
  - \--
* - authenticated
  - 7
  - Not implemented (i.e., 1: Always authenticated)
  - R
  - 1
* - authbusy
  - 6
  - Not implemented (i.e., 0: Authentication module never busy)
  - R
  - 0
* - hasresethaltreq
  - 5
  - Not implemented

    :::{note}
    VeeR EH2 implements halt-on-reset with haltreq set out of reset method.
    :::
  - R
  - 0
* - confstrptrvalid
  - 4
  - Not implemented

    :::{note}
    VeeR EH2 does not provide information relevant to configuration string.
    :::
  - R
  - 0
* - version
  - 3:0
  - Debug Module present, conforming to RISC-V Debug specification Version 0.13.2
  - R
  - 2
:::

#### Halt Summary 0 Register (`haltsum0`)

Each bit in the `haltsum0` register indicates whether a specific hart is halted or not.
Since VeeR EH2 is dualthreaded, only two bits are implemented.

:::{note}
Unavailable/nonexistent harts are not considered to be halted.
:::

This register is read-only.

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} Halt Summary 0 Register (`haltsum0`, at Debug Module Offset 0x40)
:name: table-11-9
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
* - halted1
  - 1
  - `1` when hart1 halted
  - R
  - 0
* -
  -
  - :::{note}
    Not implemented for single-thread VeeR EH2 core builds.
    :::
  -
  -
* - halted0
  - 0
  - `1` when hart0 halted
  - R
  - 0
:::

#### Hart Array Window Select Register (`hawindowsel`)

The `hawindowsel` register selects which 32-bit portion of the hart array mask register is currently accessible in the `hawindow` register (see Section 11.1.2.5).
Since there are only two harts in VeeR EH2 which are mapped to the same window, the *hawindowsel* field is tied to `0`.

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} Hart Array Window Select Register (`hawindowsel`, at Debug Module Offset 0x14)
:name: table-11-10
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - Reserved
  - 31:15
  - Reserved
  - R
  - 0
* - `hawindowsel`
  - 14:0
  - Hart array window select

    :::{note}
    VeeR EH2 provides two harts which both are mapped to window 0.
    Therefore, this field is tied to `0`.
    :::
  - R0/W
  - 0
:::

#### Hart Array Window Register (`hawindow`)

The `hawindow` register provides access to a 32-bit portion of the hart array mask register.
Since VeeR EH2 is a dual-threaded core, the hart array mask register is only 2 bits wide (i.e., only two bits are implemented and all other bits are tied to `0`).

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} Hart Array Window Register (`hawindow`, at Debug Module Offset 0x15)
:name: table-11-11
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
* - maskdata1
  - 1
  - Mask for hart 1:
    * 0: Hart 1 deselected
    * 1: Hart 1 selected

    :::{note}
    Not implemented for single-thread VeeR EH2 core builds.
    :::
  - R/W (dual-thread core)

    R (single-thread core)
  - 0
* - maskdata0
  - 0
  - Mask for hart 0:
    * 0: Hart 0 deselected
    * 1: Hart 0 selected
  - R/W
  - 0
:::

#### Abstract Control and Status Register (`abstractcs`)

The `abstractcs` register provides status information of the abstract command interface and enables clearing of detected command errors.

:::{note}
Writing this register while an abstract command is executing causes its *cmderr* field to be set to `1` (i.e., 'busy'), if it is `0`.
:::

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} Abstract Control and Status Register (`abstractcs`, at Debug Module Offset 0x16)
:name: table-11-12
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - Reserved
  - 31:29
  - Reserved
  - R
  - 0
* - progbufsize
  - 28:24
  - Not implemented

    :::{note}
    VeeR EH2 does not implement a Program Buffer.
    :::
  - R
  - 0
* - Reserved
  - 23:13
  - Reserved
  - R
  - 0
* - busy
  - 12
  - Abstract command interface activity:
    * 0: Abstract command interface idle
    * 1: Abstract command currently being executed

    :::{note}
    'Busy' indication set when command register (see Section 11.1.2.7) is written, cleared after command has completed.
    :::
  - R
  - 0
* - Reserved
  - 11
  - Reserved
  - R
  - 0
* - cmderr
  - 10:8
  - Set if abstract command fails.

    Reason for failure:
      * 0 (none): No error
      * 1 (busy): Abstract command was executing when `command`, `abstractcs`, or `abstractauto` register was written, or when `data0` or `data1` register was read or written
      * 2 (not supported): Requested command or option not supported, regardless of whether hart is running or not (i.e., illegal command, access register command not word-sized or postexec bit set, or access memory command size larger than word)
      * 3 (exception): Exception occurred while executing abstract command (i.e., illegal register address, address outside of ICCM/DCCM/PIC memory range but in internal memory region, ICCM/DCCM uncorrectable ECC error, or ICCM/PIC access not word-sized)
      * 4 (halt/resume): Abstract command couldn't execute because hart wasn't in required state (running/halted), or unavailable
      * 5 (bus): Abstract command failed for SoC memory access due to bus error (e.g., unmapped address, uncorrectable error, incorrect alignment, or unsupported access size)
      * 6: Reserved
      * 7 (other): Register or memory access size not 32 bits wide or unaligned

    :::{note}
    Bits in this field remain set until cleared by writing `111`.
    :::

    :::{note}
    Next abstract command not started until value is reset to `0`.
    :::

    :::{note}
    Only contains valid value if busy is `0`.
    :::
  - R/W1C
  - 0
* - Reserved
  - 7:4
  - Reserved
  - R
  - 0
* - datacount
  - 3:0
  - 2 data registers implemented as part of abstract command interface
  - R
  - 2
:::

#### Abstract Command Register (command)

Writes to the command register cause the corresponding abstract command to be executed.
The executed command applies to the hart selected by the *hartsello* field of the `dmcontrol` register (see {ref}`Table 11-7 <table-11-7>`).

Writing this register while an abstract command is executing causes the *cmderr* field in the `abstractcs` register (see Section 11.1.2.6) to be set to `1` (i.e., 'busy'), if it is `0`.
If the *cmderr* field is non-zero, writes to the command register are ignored.

:::{note}
A non-zero *cmderr* field inhibits starting a new abstract command to accommodate debuggers which, for performance reasons, may send several commands to be executed in a row without checking the *cmderr* field in between.
Checking the *cmderr* field only at the end of a sequence of commands is safe because later commands which might depend on a previous, but failed command are not executed.
:::

:::{note}
Access register and access memory abstract commands may only be executed when the core is in the debug halt (db-halt) state.
If the debugger is requesting the execution of an abstract command while the core is not in the debug halt state, the command is aborted and the *cmderr* field is set to `4` (i.e., 'halt/resume'), if it is `0`.
:::

:::{note}
The access memory abstract command method provides access to ICCM, DCCM, and PIC memory ranges as well as to SoC memories.
:::

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} Abstract Command Register (command, at Debug Module Offset 0x17)
:name: table-11-13
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - cmdtype
  - 31:24
  - Abstract command type:
    * 0: Access Register Command
    * 2: Access Memory Command

    :::{note}
    Other values not implemented or reserved for future use.
    Writing this field to value different than `0` or `2` causes abstract command to fail and *cmderr* field of `abstractcs` register to be set to `2`.
    :::
  - R0/W
  - 0
* -
  -
  - **Access Register Command**
  -
  -
* - Reserved
  - 23
  - Reserved
  - R
  - 0
* - aarsize
  - 22:20
  - Register access size:
      * 2: 32-bit access

    :::{note}
    Other size values not implemented.
    Writing this field to value different than `2` causes abstract command to fail and *cmderr* field of `abstractcs` register to be set to `2`, except if *transfer* is `0`.
    :::
  - R/W
  - 2
* - aarpostincrement
  - 19
  - Access register post-increment control:
    * 0: No post-increment
    * 1: After every successful access register command completion, increment *regno* field (wrapping around to 0)
  - R/W
  - 0
* - postexec
  - 18
  - Not implemented (i.e., 0: No effect)

    :::{note}
    Writing to `1` causes abstract command to fail and *cmderr* field of `abstractcs` register to be set to `2`.
    :::
  - R
  - 0
* - transfer
  - 17
  - Transfer:
    * 0: Do not perform operation specified by write

      **Note:** Selection of unimplemented options (except for *aarsize* and *regno* fields) causes *cmderr* field of `abstractcs` register to be set to `2`.

    * 1: Perform operation specified by write

      **Note:** Selection of unimplemented options causes abstract command to fail and *cmderr* field of `abstractcs` register to be set to `2`.
  - R
  - 1
* - write
  - 16
  - Read or write register:
    * 0 (read): Copy data from register specified in *regno* field into `data0` register (Section 11.1.2.9)
    * 1 (write): Copy data from `data0` register (Section 11.1.2.9) into register specified in *regno* field
  - R0/W
  - 0
* - regno
  - 15:0
  - Register access:
      * 0x0000 - 0x0FFF: CSRs
      * 0x1000 - 0x101F: GPRs
      * 0x1020 - 0xFFFF: Not implemented or reserved

    :::{note}
    Selecting illegal register address causes abstract command to fail and *cmderr* field of `abstractcs` register to be set to `3`, except if transfer is `0`.
    :::

    :::{note}
    When accessing CSRs which are shared between threads (i.e., CSRs listed in {ref}`Table 13-3 <table-13-3>` with a 'C' in the 'Scope' column) as well as all I-cache control CSRs (see Section 10.5), both threads must be in debug halt (db-halt) state, otherwise unpredictable behavior may result.
    :::
  - R0/W
  - 0
* -
  -
  - **Access Memory Command (ICCM, DCCM, PIC, and SoC Memories)**
  -
  -
* - aamvirtual
  - 23
  - Not implemented (i.e., 0: Addresses are physical)

    :::{note}
    VeeR EH2 supports physical addresses only.
    Since physical and virtual address are identical, no error is flagged [^52] even if written to `1`.
    :::
  - R
  - 0
* - aamsize
  - 22:20
  - Memory access size:
    * 0: 8-bit access (for DCCM and SoC memories)
    * 1: 16-bit access (for DCCM and SoC memories)
    * 2: 32-bit access (for ICCM, DCCM, PIC, and SoC memories)

    :::{note}
    Writing this field to value `0` or `1` for ICCM or PIC memory access causes abstract command to fail and *cmderr* field of `abstractcs` register to be set to `3`.
    :::

    :::{note}
    Other size values not implemented.
    Writing this field to value higher than `2` causes abstract command to fail and *cmderr* field of `abstractcs` register to be set to `2`.
    :::
  - R/W
  - 2
* - aampostincrement
  - 19
  - Access memory post-increment control:
    * 0: No post-increment
    * 1: After every successful access memory command completion, increment `data1` register (which contains memory address, see Section 11.1.2.9) by number of bytes encoded in *aamsize* field
  - R/W
  - 0
* - Reserved
  - 18:17
  - Reserved
  - R
  - 0
* - write
  - 16
  - Read or write memory location:
    * 0 (read): Copy data from memory location specified in `data1` register (i.e., address) into `data0` register (i.e., data) (Section 11.1.2.9)
    * 1 (write): Copy data from `data0` register (i.e., data) into memory location specified in `data1` register (i.e., address) (Section 11.1.2.9)
  - R0/W
  - 0
* - target-specific
  - 15:14
  - Not implemented

    :::{note}
    VeeR EH2 does not use target-specific bits.
    :::
  - R
  - 0
* - Reserved
  - 13:0
  - Reserved
  - R
  - 0
:::

#### Abstract Command Autoexec Register (`abstractauto`)

The `abstractauto` register controls if reading or writing the `data0/1` registers (see Section 11.1.2.9) automatically triggers the next execution of the abstract command in the command register (see Section 11.1.2.7).
This feature allows more efficient burst accesses.

Writing this register while an abstract command is executing causes the *cmderr* field in the `abstractcs` register (see Section 11.1.2.6) to be set to `1` (i.e., 'busy'), if it is `0`.

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} Abstract Command Autoexec Register (`abstractauto`, at Debug Module Offset 0x18)
:name: table-11-14
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
* - autoexecdata1
  - 1
  - Auto-execution control for `data1` register:
    * 0: No automatic triggering of abstract command execution
    * 1: Reading or writing `data1` causes abstract command to be executed again
  - R/W
  - 0
* - autoexecdata0
  - 0
  - Auto-execution control for `data0` register:
    * 0: No automatic triggering of abstract command execution
    * 1: Reading or writing `data0` causes abstract command to be executed again
  - R/W
  - 0
:::

#### Abstract Data 0/1 Registers (`data0/1`)

The `data0/1` registers are basic read/write registers which may be read or changed by abstract commands.

:::{note}
The *datacount* field of the `abstractcs` register (see {ref}`Table 11-12 <table-11-12>`) indicates that 2 (out of possible 12) registers are implemented in VeeR EH2.
:::

The `data0` register sources the value for and provides the return value of an abstract command.
The `data1` register provides the address for an access memory abstract command.

:::{note}
Selecting an address outside of the ICCM, DCCM, or PIC memory range but in one of the core-internal memory regions causes the abstract command to fail and the *cmderr* field of the `abstractcs` register to be set to `3`.
Similarly, selecting an unmapped SoC memory address causes the abstract command to fail, provided the SoC responds with a bus error, and the *cmderr* field of the `abstractcs` register to be set to `5`.
:::

Accessing these registers while an abstract command is executing causes the *cmderr* field of the `abstractcs` register (see {ref}`Table 11-12 <table-11-12>`) to be set to `1` (i.e., 'busy'), if it was `0`.

Attempts to write the `data0/1` registers while the busy bit of the `abstractcs` register (see {ref}`Table 11-12 <table-11-12>`) is set does not change their value.

The values in these registers may not be preserved after an abstract command has been executed.
The only guarantees on their contents are the ones offered by the executed abstract command.
If the abstract command fails, no assumptions should be made about the contents of these registers.

These registers are mapped to the Debug Module Interface address space and shared by the harts (i.e., one set of registers per core).

:::{list-table} Abstract Data 0/1 Register (`data0/1`, at Debug Module Offset 0x04 / 0x05)
:name: table-11-15
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - data
  - 31:0
  - Abstract command data:
    * `data0`: data value (access register and access memory command)
    * `data1`: address (access memory command)
  - R/W
  - 0
:::

#### System Bus Access Control and Status Register (`sbcs`)

The `sbcs` register provides controls and status information of the system bus access interface.

:::{note}
The system bus access method provides access to SoC memories only.
Access to ICCM, DCCM, and PIC memory ranges is only available using the access memory abstract command method.
:::

:::{note}
The operation of the system bus access method does not depend on the core's state.
SoC memory locations may be accessed using this method even when the core is running.
:::

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} System Bus Access Control and Status Register (`sbcs`, at Debug Module Offset 0x38)
:name: table-11-16
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - sbversion
  - 31:29
  - System Bus interface conforms to RISC-V Debug specification, Version 0.13.2
  - R
  - 1
* - Reserved
  - 28:23
  - Reserved
  - R
  - 0
* - sbbusyerror
  - 22
  - Set when debugger attempts to read data while a read is in progress, or when debugger initiates a new access while one is still in progress (i.e., while *sbbusy* bit is set).
    Remains set until explicitly cleared by debugger.

    :::{note}
    When set, Debug Module cannot initiate more system bus accesses.
    :::
  - R/W1C
  - 0
* - sbbusy
  - 21
  - System bus master interface status:
    * 0: System bus master idle
    * 1: System bus master busy (Set when read or write access requested, remains set until access fully completed)

    :::{note}
    Writes to this register while *sbbusy* bit is set result in undefined behavior.
    Debugger must not write this register until it reads *sbbusy* bit as `0`.
    :::

    :::{note}
    Bit reflects if system bus master interface is busy, not status of system bus itself.
    :::
  - R
  - 0
* - sbreadonaddr
  - 20
  - Auto-read on address write:
    * 0: No auto-read on address write
    * 1: Every write to `sbaddress0` (see Section 11.1.2.11) automatically triggers system bus read at new address
  - R/W
  - 0
* - sbaccess
  - 19:17
  - Access size for system bus access:
    * 0: 8-bit access
    * 1: 16-bit access
    * 2: 32-bit access
    * 3: 64-bit access

    :::{note}
    Other values not supported.
    No access performed, *sberror* field set to `4`.
    :::
  - R/W
  - 2
* - sbautoincrement
  - 16
  - Auto-address increment:
    * 0: No auto-address increment
    * 1: `sbaddress0` register (see Section 11.1.2.11) incremented by access size (in bytes) selected in *sbaccess* field after every successful system bus access
  - R/W
  - 0
* - sbreadondata
  - 15
  - Auto-read on data read: 0: No auto-read on data read 1: Every read from `sbdata0` register (see Section 11.1.2.12) automatically triggers new system bus read at (possibly auto- incremented) address
  - R/W
  - 0
* - sberror
  - 14:12
  - Set when Debug Module's system bus master encounters an error:

    While this field is non-zero, no more system bus accesses can be initiated by the Debug Module.

      * 0: No bus error
      * 1: Not implemented (no timeout)
      * 2: Bad address accessed
      * 3: Alignment error
      * 4: Access of unsupported size requested
      * 5..7: Not implemented (no other error conditions)

    :::{note}
    Bits in this field remain set until cleared by writing `111`.
    :::

    :::{note}
    Debug Module may not initiate next system bus access until value is reset to `0`.
    :::
  - R/W1C
  - 0
* - sbasize
  - 11:5
  - Width of system bus addresses (in bits)
  - R
  - 32
* - sbaccess128
  - 4
  - 128-bit system bus accesses not supported
  - R
  - 0
* - sbaccess64
  - 3
  - 64-bit system bus accesses supported
  - R
  - 1
* - sbaccess32
  - 2
  - 32-bit system bus accesses supported
  - R
  - 1
* - sbaccess16
  - 1
  - 16-bit system bus accesses supported
  - R
  - 1
* - sbaccess8
  - 0
  - 8-bit system bus accesses supported
  - R
  - 1
:::

#### System Bus Address 31:0 Register (`sbaddress0`)

The `sbaddress0` register provides the address of the system bus access.

If the *sbreadonaddr* bit of the `sbcs` register is `1`, writing the `sbaddress0` register triggers a system bus read access from the new address.

:::{note}
The *sberror* and *sbbusyerror* fields of the `sbcs` register must both be `0` for a system bus read operation to be performed.
:::

:::{note}
If the system bus master interface is busy (i.e., *sbbusy* bit of the `sbcs` register is `1`) when a write access to this register is performed, the *sbbusyerror* bit in the `sbcs` register is set and the access is aborted.
:::

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} System Bus Address 31:0 Register (`sbaddress0`, at Debug Module Offset 0x39)
:name: table-11-17
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - address
  - 31:0
  - System bus address
  - R/W
  - 0
:::

#### System Bus Data 31:0 Register (`sbdata0`)

The `sbdata0` register holds the right-justified lower bits for system bus read and write accesses.

A successful system bus read updates the `sbdata0/1` registers with the value read from the system bus at the memory location addressed by the `sbaddress0` register.
If the width of the read access is less than 64 bits, the remaining high bits may take on any value.

Reading the `sbdata0` register provides the current value of this register.
If the *sbreadondata* bit of the `sbcs` register is `1`, reading this register also triggers a system bus read access which updates the `sbdata0/1` registers with the value read from the memory location addressed by the `sbaddress0` register.

Writing the `sbdata0` register triggers a system bus write access which updates the memory location addressed by the `sbaddress0` register with the new values in the `sbdata0/1` registers.

:::{note}
Only the `sbdata0` register has this behavior.
Accessing the `sbdata1` register has no side effects.
A debugger must access the `sbdata1` register first, before accessing the `sbdata0` register.
:::

:::{note}
The *sberror* and *sbbusyerror* fields of the `sbcs` register must both be `0` for a system bus read or write operation to be performed.
:::

:::{note}
If the system bus master interface is busy (i.e., *sbbusy* bit of the `sbcs` register is `1`) when a read or write access to this register is performed, the *sbbusyerror* bit in the `sbcs` register is set and the access is aborted.
:::

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} System Bus Data 31:0 Register (`sbdata0`, at Debug Module Offset 0x3C)
:name: table-11-18
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - data
  - 31:0
  - System bus data[31:0] for system bus read and write accesses
  - R/W
  - 0
:::

#### System Bus Data 63:32 Register (`sbdata1`)

The `sbdata1` register holds the upper 32 bits of the 64-bit wide system bus for read and write accesses.

:::{note}
If the system bus master interface is busy (i.e., *sbbusy* bit of the `sbcs` register is `1`) when a read or write access to this register is performed, the *sbbusyerror* bit in the `sbcs` register is set and the access is aborted.
:::

This register is mapped to the Debug Module Interface address space and shared by the harts (i.e., one register per core).

:::{list-table} System Bus Data 63:32 Register (`sbdata1`, at Debug Module Offset 0x3D)
:name: table-11-19
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - data
  - 31:0
  - System bus data[63:32] for system bus read and write accesses
  - R/W
  - 0
:::

### Control/Status Registers in RISC-V CSR Address Space

A summary of standard RISC-V control/status registers with platform-specific adaptations in CSR space:

- Trigger Select Register (`tselect`) (see Section 11.1.3.1)
- Trigger Data 1 Register (`tdata1`) (see Section 11.1.3.2)
- Match Control Register (`mcontrol`) (see Section 11.1.3.3)
- Trigger Data 2 Register (`tdata2`) (see Section 11.1.3.4)
- Debug Control and Status Register (`dcsr`) (see Section 11.1.3.5)
- Debug PC Register (`dpc`) (see Section 11.1.3.6)

All reserved and unused bits in these control/status registers must be hardwired to `0`.
Unless otherwise noted, all read/write control/status registers must have WARL (Write Any value, Read Legal value) behavior.

#### Trigger Select Register (`tselect`)

:::{note}
Since triggers can be used both by Debug Mode and M-mode, the debugger must restore this register if it modified it.
:::

This register is mapped to the standard read/write CSR address space and hart-specific (i.e., a separate register per thread).

:::{list-table} Trigger Select Register (`tselect`, at CSR 0x7A0)
:name: table-11-20
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
* - index
  - 1:0
  - Index of trigger 0..3

    :::{note}
    Triggers 0 and 2 may be chained, triggers 1 and 3 not.
    :::
  - R/W
  - 0
:::

#### Trigger Data 1 Register (`tdata1`)

This register is mapped to the standard read/write CSR address space and hart-specific (i.e., a separate register per thread).

:::{list-table} Trigger Data 1 Register (`tdata1`, at CSR 0x7A1)
:name: table-11-21
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - type
  - 31:28
  -
  - R
  - 2
* - dmode
  - 27
  - See {ref}`Table 11-22 <table-11-22>` below.
  -
  -
* - data
  - 26:0
  -
  -
  -
:::

#### Match Control Register (`mcontrol`)

:::{note}
VeeR EH2 does not support triggering on the data of a load or on the opcode of an executed instruction.
:::

This register is mapped to the standard read/write CSR address space and hart-specific (i.e., a separate register per thread).

:::{list-table} Match Control Register (`mcontrol`, at CSR 0x7A1)
:name: table-11-22
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - type
  - 31:28
  - Address/data match trigger (= `mcontrol`)
  - R
  - 2
* - dmode
  - 27
  - Mode write privileges to `tdata1/2` registers (Sections 11.1.3.2 and 11.1.3.4) selected by `tselect` register (Section 11.1.3.1):
    * 0: Both Debug Mode and M-mode may write `tdata1/2` registers selected by `tselect` register
    * 1: Only Debug Mode may write `tdata1/2` registers selected by `tselect` register. Writes from M-mode are ignored.

    :::{note}
    Only writable from Debug Mode.
    :::
  - R/W
  - 0
* - maskmax
  - 26:21
  - $2^{31}$ bytes is largest naturally aligned powers-of-two (NAPOT) range supported by hardware when *match* field is `1`.
  - R
  - 31
* - hit
  - 20
  - Set by hardware when this trigger matches.
    Allows to determine which trigger(s) matched.
    May be set or cleared by trigger's user at any time.

    :::{note}
    For chained triggers, *hit* bit of a matching second trigger is not set unless first trigger matches as well.
    :::
  - R/W
  - 0
* - select
  - 19
  - Match selection:
    * 0: Perform match on address
    * 1: Perform match on store data value
  - R/W
  - 0
* - timing
  - 18
  - Action for this trigger is taken just before instruction that triggered it is committed, but after all preceding instructions are committed.

    :::{note}
    No bus transaction is issued for an execute address trigger hit on a load to a side-effect address.
    :::
  - R
  - 0
* - sizelo
  - 17:16
  - Match size:
    * 0: Trigger attempts to match against access of any size.
      * Match against address (if *select* bit is `0`)
      * Match against store data (if *select* bit is `1`)

      :::{note}
      Data is zero extended for byte or halfword stores.
      :::

      :::{note}
      If *match* bit is `1`, the mask in the `tdata2` register is applied independent of the *select* bit value (i.e., in address or data matches).
      :::

      :::{note}
      Other match size values not implemented.
      :::
  - R
  - 0
* - action
  - 15:12
  - Action to take when trigger fires: 0: Raise breakpoint exception (used when software wants to use trigger module without external debugger attached) 1: Enter Debug Mode (only supported when trigger's *dmode* bit is `1`)

    :::{note}
    Other values reserved for future use.
    :::

    :::{note}
    Triggers do not fire if this field is `0` and interrupts are disabled [^53] (i.e., *mie* bit of `mstatus` standard RISC-V register is `0`).
    :::
  - R/W
  - 0
* - chain
  - 11
  - Trigger chaining:
    * 0: When this trigger matches, the configured action is taken.
    * 1: While this trigger does not match, it prevents the trigger with the next index from matching.

      **Note:** Supported for triggers 0 and 2 only, attempts to set this bit for triggers 1 and 3 are ignored.

    :::{note}
    In VeeR EH2, only pairs of triggers (i.e., triggers 0/1 and triggers 2/3) are chainable.
    :::

    :::{note}
    If *chain* bit of trigger 0/2 is `1`, it is chained to trigger 1/3.
    Only *action* field of trigger 1/3 is used (i.e., *action* field of trigger 0/2 is ignored).
    The action on second trigger is taken if and only if both triggers in chain match at the same time.
    :::

    :::{note}
    Because the *chain* bit affects the next trigger, hardware resets it to `0` for mcontro l register writes with *dmode* bit of `0` if the next trigger has a *dmode* bit of `1`.
    In addition, hardware ignores writes to the mcontro l register which would set the *dmode* bit to `1` if the previous trigger has both a *dmode* bit of `0` and a *chain* bit of `1`.
    Debuggers must avoid the latter case by checking the *chain* bit of the previous trigger when writing the mcontro l register.
    :::
  - R/W (for triggers 0 and 2) R (for triggers 1 and 3)
  - 0
* - match
  - 10:7
  - Match control:
    * 0: Matches when value equals `tdata2` register's (Section 11.1.3.4) value [^54]
    * 1: Matches when top M bits of value match top M bits of `tdata2` register's (Section 11.1.3.4) value (*M* is 31 minus the index of least-significant bit containing 0 in `tdata2` register)

    :::{note}
    Other values not implemented or reserved for future use.
    :::
  - R/W
  - 0
* - m
  - 6
  - When set, enable this trigger in M-mode
  - R/W
  - 0
* - Reserved
  - 5
  - Reserved
  - R
  - 0
* - s
  - 4
  - Not implemented (VeeR EH2 is M-mode only)
  - R
  - 0
* - u
  - 3
  - Not implemented (VeeR EH2 is M-mode only)
  - R
  - 0
* - execute
  - 2
  - When set, trigger fires on address of executed instruction

    :::{note}
    For writes, written to `0` if *select* bit is written to `1`.
    :::
  - R/W
  - 0
* - store
  - 1
  - When set, trigger fires on address or data of store
  - R/W
  - 0
* - load
  - 0
  - When set, trigger fires on address of load

    :::{note}
    For writes, written to `0` if *select* bit is written to `1`.
    :::
  - R/W
  - 0
:::

#### Trigger Data 2 Register (`tdata2`)

This register is mapped to the standard read/write CSR address space and hart-specific (i.e., a separate register per thread).


:::{list-table} Trigger Data 2 Register (`tdata2`, at CSR 0x7A2)
:name: table-11-23
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - value
  - 31:0
  - Match value:
    * Address or data value for match:
      * Address of load, store, or executed instruction [^54]
      * Data value of store
    * Match mask (see *match* field of `mcontrol` register ({ref}`Table 11-22 <table-11-22>`) set to `1`)
  - R/W
  - 0
:::

#### Debug Control and Status Register (`dcsr`)

The `dcsr` register controls the behavior and provides status of the hart in Debug Mode.

The RISC-V Debug specification [[3]](intro.md#ref-3), Section 4.8.1 documents some required and several optional features.
{ref}`Table 11-24 <table-11-24>` describes the required features, the partial support of optional features in VeeR EH2, and indicates features not supported with 'Not implemented'.

:::{note}
This register is accessible in **Debug Mode only**.
Attempting to access this register in machine mode raises an illegal instruction exception.
:::

This register is mapped to the standard read/write CSR address space and hart-specific (i.e., a separate register per thread).

:::{list-table} Debug Control and Status Register (`dcsr`, at CSR 0x7B0)
:name: table-11-24
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - xdebugver
  - 31:28
  - External debug support exists as described in this chapter and [[3]](intro.md#ref-3)
  - R
  - 4
* - Reserved
  - 27:16
  - Reserved
  - R
  - 0
* - ebreakm
  - 15
  - * 0: `ebreak` in M-mode behaves as described in RISC-V Privileged specification [[2]](intro.md#ref-2)
    * 1: `ebreak` in M-mode enters Debug Mode
  - R/W
  - 0
* - Reserved
  - 14
  - Reserved
  - R
  - 0
* - ebreaks
  - 13
  - Not implemented (VeeR EH2 is M-mode only)
  - R
  - 0
* - ebreaku
  - 12
  - Not implemented (VeeR EH2 is M-mode only)
  - R
  - 0
* - stepie
  - 11
  - * 0: Interrupts disabled during single stepping
    * 1: Interrupts enabled during single stepping

    :::{note}
    Debugger must not change value while hart is running.
    :::
  - R/W
  - 0
* - stopcount
  - 10
  - * 0: Increment counters as usual
    * 1: Don't increment any counters (incl. `cycle` and `instret`) while in Debug Mode or on ebreak entering Debug Mode (referred value for most debugging scenarios)
  - R/W
  - 0
* - stoptime
  - 9
  - Increment timers same as in non-debug mode
  - R
  - 0
* - cause
  - 8:6
  - Reason for Debug Mode entry (if multiple reasons in single cycle, set cause to highest priority):
    * 1: ebreak instruction was executed (priority 3)
    * 2: Trigger Module caused a breakpoint exception (priority 4, highest)
    * 3: Debugger or MPC interface (see {ref}`Table 7-4 <table-7-4>`) requested entry to Debug Mode using haltreq (priority 1)
    * 4: Hart single-stepped because step was set (priority 0, lowest)
    * 5: Hart halted directly out of reset due to resethaltreq (also acceptable to report `3`) (priority 2)

    Other values reserved for future use.
  - R
  - 0
* - Reserved
  - 5
  - Reserved
  - R
  - 0
* - mprven
  - 4
  - Not implemented (i.e., 0: *mprv* field in `mstatus` register ignored in Debug Mode)
  - R
  - 0
* - nmip
  - 3
  - Non-Maskable Interrupt (NMI) pending for hart when set

    :::{note}
    NMI may indicate a hardware error condition, reliable debugging may no longer be possible once bit is set.
    :::
  - R
  - 0
* - step
  - 2
  - When set and not in Debug Mode, hart only executes single instruction and enters Debug Mode.
    If instruction does not complete due to exception, hart immediately enters Debug Mode before executing trap handler, with appropriate exception registers set.

    :::{note}
    Debugger must not change value while hart is running.
    :::
  - R/W
  - 0
* - prv
  - 1:0
  - Indicates privilege level hart was operating in when Debug Mode was entered (3 = M-mode)
  - R
  - 3
:::

#### Debug PC Register (`dpc`)

The `dpc` register provides the debugger information about the program counter (PC) when entering Debug Mode and control where to resume (RISC-V Debug specification [[3]](intro.md#ref-3), Section 4.8.2).

Upon entry to Debug Mode, the `dpc` register is updated with the address of the next instruction to be executed.
The behavior is described in more detail in {ref}`Table 11-25 <table-11-25>` below.

When resuming, the hart's PC is updated to the address stored in the `dpc` register.
A debugger may write the `dpc` register to change where the hart resumes.

:::{note}
This register is accessible in **Debug Mode only**.
Attempting to access this register in machine mode raises an illegal instruction exception.
:::

This register is mapped to the standard read/write CSR address space and hart-specific (i.e., a separate register per thread).

:::{list-table} Debug PC Register (`dpc`, at CSR 0x7B1)
:name: table-11-25
:header-rows: 1

* - Field
  - Bits
  - Description
  - Access
  - Reset
* - `dpc`
  - 31:0
  - Address captured for:
    * ebreak:
      * Address of ebreak instruction
    * Single step:
      * Address of instruction which would be executed next if not in Debug Mode (i.e., PC + 4 for 32-bit instructions which don't change program flow, destination PC on taken jumps/branches, etc.)
    * Trigger module:
      * If timing (see *timing* bit in `mcontrol` register in {ref}`Table 11-22 <table-11-22>`) is:
        * 0: Address of instruction which caused trigger to fire
        * 1: Address of next instruction to be executed when Debug Mode was entered
    * Halt request:
      * Address of next instruction to be executed when Debug Mode was entered
  - R/W
  - 0
:::

[^49]: C = per-core, T = per-thread
[^50]: C = per-core, T = per-thread
[^51]: Hart is in reset or *ndmreset* bit of `dmstatus` register is ‘1’.
[^52]: The RISC-V Debug specification [[3]](intro.md#ref-3) states that an implementation must fail accesses that it does not support.
       However, the Debug Task Group community agreed in an email exchange on the group’s reflector as well as in a group meeting that not reporting an error is acceptable for implementations without address translation (i.e., the physical address equals the virtual address).
[^53]: To enable native debugging of M-mode code, VeeR EH2 implements the simpler but more restrictive solution of preventing triggers with the *action* field set to ‘0’ (i.e., breakpoint exception) while interrupts are disabled, as described in Section 5.1, ‘Native M-Mode Triggers’ of the RISC-V Debug specification [[3]](intro.md#ref-3).
[^54]: Bit 0 of `tdata2` register is ignored for instruction address matches.
