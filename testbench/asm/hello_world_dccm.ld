
OUTPUT_ARCH( "riscv" )
ENTRY(_start)

SECTIONS {
  .text   : { *(.text*) }
 _end = .;
  . = 0xd0580000;
  .data.io .  : { *(.data.io) }
  . = 0xf0040000;
  .data  :  { *(.*data) *(.rodata*) *(.srodata*) *(.sbss*) }
  .bss : { *(.bss) }
  STACK = ALIGN(16) + 0x2000;
  . = 0xfffffff0;
  .data.ctl : { LONG(0xf0040000); LONG(STACK) }
  /DISCARD/ : { *(.eh_frame*) }
}
