(chap-build-args)=
# VeeR EH2 Core Build Arguments

## Memory Protection Build Arguments

### Memory Protection Build Argument Rules

The rules for valid memory protection address (INST/DATA\_ACCESS\_ADDR*x*) and mask (INST/DATA\_ACCESS\_MASK*x*) build arguments are:

- INST/DATA\_ACCESS\_ADDR*x* must be 64B-aligned (i.e., 6 least significant bits must be `0`)
- INST/DATA\_ACCESS\_MASK*x* must be an integer multiple of 64B minus 1 (i.e., 6 least significant bits must be `1`)
- For INST/DATA\_ACCESS\_MASK*x* , all `0` bits (if any) must be left-justified and all `1` bits must be rightjustified
- No bit in INST/DATA\_ACCESS\_ADDR*x* may be `1` if the corresponding bit in INST/DATA\_ACCESS\_MASK*x* is also `1` (i.e., for each bit position, at most one of the bits in INST/DATA\_ACCESS\_ADDR*x* and INST/DATA\_ACCESS\_MASK*x* may be `1`)

### Memory Protection Build Arguments

- **Instructions**
  - Instruction Access Window*x*(*x* = 0..7) ▪ Enable (INST\_ACCESS\_ENABLE*x*): 0,1 (*0 = window disabled; 1 = window enabled*) ▪ Base address (INST\_ACCESS\_ADDR*x*): 0x0000\_0000..0xFFFF\_FFC0 (*see Section 18.1.1*) ▪ Mask (INST\_ACCESS\_MASK*x*): 0x0000\_003F..0xFFFF\_FFFF (*see Section 18.1.1*)
- **Data**
  - Data Access Window*x*(*x* = 0..7) ▪ Enable (DATA\_ACCESS\_ENABLE*x*): 0,1 (*0 = window disabled; 1 = window enabled*) ▪ Base address (DATA\_ACCESS\_ADDR*x*): 0x0000\_0000..0xFFFF\_FFC0 (*see Section 18.1.1*) ▪ Mask (DATA\_ACCESS\_MASK*x*): 0x0000\_003F..0xFFFF\_FFFF (*see Section 18.1.1*)

## Core Memory-Related Build Arguments

### Core Memories and Memory-Mapped Register Blocks Alignment Rules

Placement of VeeR EH2's core memories and memory-mapped register blocks in the 32-bit address range is very flexible.
Each memory or register block may be assigned to any region and within the region's 28-bit address range to any start address on a naturally aligned power-of-two address boundary relative to its own size (i.e., *start_address = n × size*, whereas *n* is a positive integer number).

For example, the start address of an 8KB-sized DCCM may be 0x0000\_0000, 0x0000\_2000, 0x0000\_4000, 0x0000\_6000, etc.
A memory or register block with a non-power-of-two size must be aligned to the next bigger power-of-two size.
For example, the starting address of a 48KB-sized DCCM must aligned to a 64KB boundary, i.e., it may be 0x0000\_0000, 0x0001\_0000, 0x0002\_0000, 0x0003\_0000, etc.

Also, no two memories or register blocks may overlap each other, and no memory or register block may cross a region boundary.

The start address of the memory or register block is specified with an offset relative to the start address of the region.
This offset must follow the rules described above.

### Memory-Related Build Arguments

- **ICCM**
  - Enable (RV\_ICCM\_ENABLE): 0, 1 (*0 = no ICCM; 1 = ICCM enabled*)
  - Region (RV\_ICCM\_REGION): 0..15
  - Offset (RV\_ICCM\_OFFSET): (*offset in bytes from start of region satisfying rules in Section 18.2.1*)
  - Size (RV\_ICCM\_SIZE): 4, 8, 16, 32, 64, 128, 256, 512 (in KB)
- **DCCM**
  - Region (RV\_DCCM\_REGION): 0..15
  - Offset (RV\_DCCM\_OFFSET): (*offset in bytes from start of region satisfying rules in Section 18.2.1*)
  - Size (RV\_DCCM\_SIZE): 4, 8, 16, 32, 48, 64, 128, 256, 512 (in KB)
- **I-Cache**
  - Enable (RV\_ICACHE\_ENABLE): 0, 1 (*0 = no I-cache; 1 = I-cache enabled*)
  - Size (RV\_ICACHE\_SIZE): 16, 32, 64, 128, 256 (*in KB*)
  - Protection (RV\_ICACHE\_ECC): 0, 1 (*0 = parity; 1 = ECC*)
- **PIC Memory-mapped Control Registers**
  - Region (RV\_PIC\_REGION): 0..15
  - Offset (RV\_PIC\_OFFSET): (*offset in bytes from start of region satisfying rules in Section 18.2.1*)
  - Size (RV\_PIC\_SIZE): 32, 64, 128, 256 (*in KB*)
