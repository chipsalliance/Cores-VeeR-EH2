(chap-interrupt-priority)=
# Interrupt Priorities

{ref}`Table 15-1 <table-15-1>` summarizes the VeeR EH2 platform-specific (Local) and standard RISC-V (External, Software, and Timer) relative interrupt priorities.

{ref}`Table 15-1 <table-15-1>`  VeeR EH2 Platform-specific and Standard RISC-V Interrupt Priorities

:::{list-table}
:name: table-15-1
:header-rows: 1

* - Priority
  - Interrupt
  - Section
* - Highest Interrupt Priority
  - Non-Maskable Interrupt (standard RISC-V)
  - 3.17
* -
  - External interrupt (standard RISC-V)
  - 8
* -
  - Correctable error (local interrupt)
  - 3.7.2
* -
  - Software interrupt (standard RISC-V)
  - 3.18
* -
  - Timer interrupt (standard RISC-V)
  - 9.2.1
* -
  - Internal timer 0 (local interrupt)
  - 6.3
* - Lowest Interrupt Priority
  - Internal timer 1 (local interrupt)
  - 6.3
:::
