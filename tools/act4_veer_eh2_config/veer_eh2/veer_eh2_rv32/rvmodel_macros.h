#ifndef _RVMODEL_MACROS_H
#define _RVMODEL_MACROS_H

#define RVMODEL_DATA_SECTION \
        .pushsection .tohost,"aw",@progbits;            \
        .balign 8; .global tohost; tohost: .dword 0;    \
        .balign 8; .global fromhost; fromhost: .dword 0; \
        .popsection

##### STARTUP #####

# Perform boot operations. Can be empty or left undefined unless needed for
# DUT-specific behavior such as turning on a memory controller or
# initializing custom state.
#define RVMODEL_BOOT \
  .rept 4;           \
  nop;               \
  .endr;

//RV_COMPLIANCE_DATA_BEGIN
#define RVMODEL_DATA_BEGIN                                              \
  RVMODEL_DATA_SECTION                                                        \
  .align 4;\
  .global begin_signature; begin_signature:

//RV_COMPLIANCE_DATA_END
#define RVMODEL_DATA_END                                                      \
  .align 4;\
  .global end_signature; end_signature:


##### TERMINATION #####

# Terminate test with a pass indication.
# When the test is run in simulation, this should end the simulation.
#define RVMODEL_HALT_PASS  \
  li x1, 0xFF              ;\
  la t0, tohost           ;\
  write_tohost_pass:      ;\
    sw x1, 0(t0)          ;\
    sw x0, 4(t0)          ;\
  self_loop_pass:         ;\
    j self_loop_pass      ;\

# Terminate test with a fail indication.
# When the test is run in simulation, this should end the simulation.
#define RVMODEL_HALT_FAIL \
  li x1, 0x01             ;\
  la t0, tohost           ;\
  write_tohost_fail:      ;\
    sw x1, 0(t0)          ;\
    sw x0, 4(t0)          ;\
  self_loop_fail:         ;\
    j self_loop_fail      ;\


##### IO #####

#define RVMODEL_IO_INIT(_R1, _R2, _R3)
#define RVMODEL_IO_WRITE_STR(_R1, _R2, _R3, _STR_PTR) \
1:                           ;                        \
  lbu  _R1, 0(_STR_PTR)      ; /* Load byte */        \
  beqz _R1, 3f               ; /* Exit if null */     \
2:                           ;                        \
  li   _R2, 0xd0580000       ; /* virtual printer */  \
  sw   _R1, 0(_R2)           ;                        \
  addi _STR_PTR, _STR_PTR, 1 ; /* Next char */        \
  j 1b                       ; /* Loop */             \
3:



##### Machine Timer #####
// These definitions should be included for the exception test suite
// #define RVMODEL_MTIME_ADDRESS  0x0000BFF8  /* Address of mtime CSR */
// #define RVMODEL_MTIMECMP_ADDRESS 0x00004000 /* Address of mtimecmp CSR */

##### Supervisor Interrupts #####
#define RVMODEL_INTERRUPT_LATENCY 1
#define RVMODEL_TIMER_INT_SOON_DELAY 100

#define RVMODEL_MSIP_ADDRESS 0x81000000
#define SIG_ADDRESS          0x810C1004

#define RVMODEL_SET_MEXT_INT(_R1, _R2) \
  li _R1, (1 << 31) | (1 << 11); \
  li _R2, SIG_ADDRESS; \
  sw _R1, 0(_R2);

#define RVMODEL_CLR_MEXT_INT(_R1, _R2) \
  li _R1, (1 << 11); \
  li _R2, SIG_ADDRESS; \
  sw _R1, 0(_R2);

#define RVMODEL_SET_MSW_INT(_R1, _R2) \
   li _R1, 1; \
   li _R2, RVMODEL_MSIP_ADDRESS; \
   sw _R1, 0(_R2);

#define RVMODEL_CLR_MSW_INT(_R1, _R2) \
   li _R2, RVMODEL_MSIP_ADDRESS; \
   sw zero, 0(_R2);

#define RVMODEL_CLEAR_MTIMER_INT

#define RVMODEL_CLEAR_MEXT_INT

#endif // _RVMODEL_MACROS_H
