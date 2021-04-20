    .equ mhartstart,0x7fc
.text
    csrwi mhartstart, 3 // enable hart1

    li  x10, 0
loop:
    addi    x10, x10, 1
    j       loop

