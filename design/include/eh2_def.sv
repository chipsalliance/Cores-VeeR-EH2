//`ifndef  EH2_DEF_SV
//`define  EH2_DEF_SV

package eh2_pkg;
// performance monitor stuff
typedef struct packed {
                       logic [1:0] trace_rv_i_valid_ip;
                       logic [63:0] trace_rv_i_insn_ip;
                       logic [63:0] trace_rv_i_address_ip;
                       logic [1:0] trace_rv_i_exception_ip;
                       logic [4:0] trace_rv_i_ecause_ip;
                       logic [1:0] trace_rv_i_interrupt_ip;
                       logic [31:0] trace_rv_i_tval_ip;
                       } eh2_trace_pkt_t;


typedef enum logic [2:0] {
                          ERR_IDLE   = 3'b000,
                          IC_WFF     = 3'b001,
                          ECC_WFF    = 3'b010,
                          ECC_CORR   = 3'b011,
                          DMA_SB_ERR = 3'b100
                         } eh2_perr_state_t;


typedef enum logic [1:0] {
                          ERR_STOP_IDLE   = 2'b00,
                          ERR_FETCH1      = 2'b01,
                          ERR_FETCH2      = 2'b10,
                          ERR_STOP_FETCH  = 2'b11
                         } eh2_err_stop_state_t;


typedef enum logic [4:0] {
                         NULL     = 5'b00000,
                         MUL      = 5'b00001,
                         LOAD     = 5'b00010,
                         STORE    = 5'b00011,
                         ALU      = 5'b00100,
                         CSRREAD  = 5'b00101,
                         CSRWRITE = 5'b00110,
                         CSRRW    = 5'b00111,
                         EBREAK   = 5'b01000,
                         ECALL    = 5'b01001,
                         FENCE    = 5'b01010,
                         FENCEI   = 5'b01011,
                         MRET     = 5'b01100,
                         CONDBR   = 5'b01101,
                         JAL      = 5'b01110,
                         BITMANIPU   = 5'b01111,
                         ATOMIC   = 5'b10000,
                         LR       = 5'b10001,
                         SC       = 5'b10010
                          } eh2_inst_pkt_t;

typedef struct packed {
                       logic valid;
                       logic wb;
                       logic stall;
                       logic [2:0] tag;
                       logic [4:0] rd;
                       } eh2_load_cam_pkt_t;

typedef struct packed {
                       logic pc0_call;
                       logic pc0_ret;
                       logic pc0_pc4;
                       logic pc1_call;
                       logic pc1_ret;
                       logic pc1_pc4;
                       } eh2_rets_pkt_t;

typedef struct packed {
                       logic ret;
                       logic [31:1] prett;  // predicted ret target
                       logic br_error;
                       logic br_start_error;
                       logic bank;
                       logic valid;
                       logic [1:0] hist;
                       logic way;
                       } eh2_br_pkt_t;

typedef struct packed {
                       logic lsu;
                       logic mul;
                       logic i0_only;
                       logic legal1;
                       logic legal2;
                       logic legal3;
                       logic legal4;
                       } eh2_predecode_pkt_t;


typedef struct packed {
                        logic  [1:0]         icaf_type;
                        logic                icaf_second;
                        logic                dbecc;
                        logic                icaf;
                        logic [31:1]         pc;
                        eh2_br_pkt_t         brp;
                        logic [31:0]         inst;
                        eh2_predecode_pkt_t predecode;
                        logic                pc4;
                        logic [15:0]         cinst;
                       } eh2_ib_pkt_t;

typedef struct packed {
                       logic valid;
                       logic [1:0] hist;
                       logic br_error;
                       logic br_start_error;
                       logic bank;
                       logic way;
                       logic middle;
                       logic tid;
                       } eh2_br_tlu_pkt_t;

typedef struct packed {// data bits - upper 19b not likely to change
                       logic [31:1] prett;
                       logic boffset;
                       logic [1:0] hist;
                       logic bank;
                       logic way;
                       // ctl bits
                       logic ataken;
                       logic valid;
                       logic pc4;
                       logic misp;
                       logic br_error;
                       logic br_start_error;
                       logic pcall;
                       logic pret;
                       logic pja;
                       } eh2_predict_pkt_t;

typedef struct packed {
                       // bits not likely to change for power
                       logic           i0icaf;
                       logic [1:0]     i0icaf_type;
                       logic           i0icaf_second;
                       logic           i0fence_i;
                       logic [3:0]     i0trigger;
                       logic [3:0]     i1trigger;
                       logic           pmu_i0_br_unpred;     // pmu
                       logic           pmu_i1_br_unpred;     // pmu
                       logic           pmu_divide;
                       logic           pmu_lsu_misaligned;
                       // bits likely to change for power
                       logic           i0legal;
                       logic           i0tid;
                       logic           i1tid;
                       logic           lsu_pipe0;
                       eh2_inst_pkt_t pmu_i0_itype;        // pmu - instruction type
                       eh2_inst_pkt_t pmu_i1_itype;        // pmu - instruction type
                       } eh2_trap_pkt_t;

typedef struct packed {
                       // bits unlikely to change
                       logic i0sc;
                       logic i0div;
                       logic i0csrwen;
                       logic i0csrwonly;
                       logic i1sc;
                       logic [11:0] i0csrwaddr;
                       // less likely to toggle
                       logic [1:0] i0rs1bype2;
                       logic [1:0] i0rs2bype2;
                       logic [3:0] i0rs1bype3;
                       logic [3:0] i0rs2bype3;
                       // less likely to toggle
                       logic [1:0] i1rs1bype2;
                       logic [1:0] i1rs2bype2;
                       logic [6:0] i1rs1bype3;
                       logic [6:0] i1rs2bype3;
                       // bits likely to change
                       logic [4:0] i0rd;
                       logic i0mul;
                       logic i0load;
                       logic i0store;
                       logic i0v;
                       logic i0valid;
                       logic i0secondary;
                       logic i0tid;
                       logic [4:0] i1rd;
                       logic i1mul;
                       logic i1load;
                       logic i1store;
                       logic i1v;
                       logic i1valid;
                       logic i1tid;
                       logic i1secondary;
                       logic           lsu_tid;
                       } eh2_dest_pkt_t;

typedef struct packed {
                       logic mul;
                       logic load;
                       logic sec;
                       logic alu;
                       } eh2_class_pkt_t;

typedef struct packed {
                       logic [4:0] rs1;
                       logic [4:0] rs2;
                       logic [4:0] rd;
                       } eh2_reg_pkt_t;


typedef struct packed {
                       // unlikely to change
                       logic clz;
                       logic ctz;
                       logic cpop;
                       logic sext_b;
                       logic sext_h;
                       logic min;
                       logic max;
                       logic pack;
                       logic packu;
                       logic packh;
                       logic rol;
                       logic ror;
                       logic grev;
                       logic gorc;
                       logic zbb;
                       logic bset;
                       logic bclr;
                       logic binv;
                       logic bext;
                       logic sh1add;
                       logic sh2add;
                       logic sh3add;
                       logic zba;
                       // likely to change
                       logic land;
                       logic lor;
                       logic lxor;
                       logic sll;
                       logic srl;
                       logic sra;
                       logic beq;
                       logic bne;
                       logic blt;
                       logic bge;
                       logic add;
                       logic sub;
                       logic slt;
                       logic unsign;
                       logic jal;
                       logic predict_t;
                       logic predict_nt;
                       logic csr_write;
                       logic csr_imm;
                       logic tid;
                       } eh2_alu_pkt_t;

typedef struct packed {
                       // unlikely to change
                       logic atomic;               // this is atomic instruction
                       logic atomic64;
                       logic fast_int;
                       logic barrier;
                       logic lr;
                       logic sc;
                       logic [4:0] atomic_instr;   // this will be decoded to get which of the amo instruction lsu is doing
                       logic dma;               // dma pkt
                       // may change
                       logic by;
                       logic half;
                       logic word;
                       logic dword;
                       logic load;
                       logic store;
                       logic pipe;   // which pipe is load/store
                       logic unsign;
/* verilator lint_off SYMRSVDWORD */
                       logic stack;
/* verilator lint_on SYMRSVDWORD */
                       logic tid;
                       logic store_data_bypass_c1;
                       logic load_ldst_bypass_c1;
                       logic store_data_bypass_c2;
                       logic store_data_bypass_i0_e2_c2;
                       logic [1:0] store_data_bypass_e4_c1;
                       logic [1:0] store_data_bypass_e4_c2;
                       logic [1:0] store_data_bypass_e4_c3;
                       logic valid;
                       } eh2_lsu_pkt_t;

typedef struct packed {
                      logic exc_valid;
                      logic single_ecc_error;
                      logic inst_type;   //0: Load, 1: Store
                      logic amo_valid;
                      logic exc_type;    //0: MisAligned, 1: Access Fault
                      logic [3:0] mscause;
                      logic [31:0] addr;
                      } eh2_lsu_error_pkt_t;

typedef struct packed {
                       logic clz;
                       logic ctz;
                       logic cpop;
                       logic sext_b;
                       logic sext_h;
                       logic min;
                       logic max;
                       logic pack;
                       logic packu;
                       logic packh;
                       logic rol;
                       logic ror;
                       logic grev;
                       logic gorc;
                       logic zbb;
                       logic bset;
                       logic bclr;
                       logic binv;
                       logic bext;
                       logic zbs;
                       logic bcompress;
                       logic bdecompress;
                       logic zbe;
                       logic clmul;
                       logic clmulh;
                       logic clmulr;
                       logic zbc;
                       logic shfl;
                       logic unshfl;
                       logic xperm_n;
                       logic xperm_b;
                       logic xperm_h;
                       logic zbp;
                       logic crc32_b;
                       logic crc32_h;
                       logic crc32_w;
                       logic crc32c_b;
                       logic crc32c_h;
                       logic crc32c_w;
                       logic zbr;
                       logic bfp;
                       logic zbf;
                       logic sh1add;
                       logic sh2add;
                       logic sh3add;
                       logic zba;
                       logic alu;
                       logic atomic;
                       logic lr;
                       logic sc;
                       logic rs1;
                       logic rs2;
                       logic imm12;
                       logic rd;
                       logic shimm5;
                       logic imm20;
                       logic pc;
                       logic load;
                       logic store;
                       logic lsu;
                       logic add;
                       logic sub;
                       logic land;
                       logic lor;
                       logic lxor;
                       logic sll;
                       logic sra;
                       logic srl;
                       logic slt;
                       logic unsign;
                       logic condbr;
                       logic beq;
                       logic bne;
                       logic bge;
                       logic blt;
                       logic jal;
                       logic by;
                       logic half;
                       logic word;
                       logic csr_read;
                       logic csr_clr;
                       logic csr_set;
                       logic csr_write;
                       logic csr_imm;
                       logic presync;
                       logic postsync;
                       logic ebreak;
                       logic ecall;
                       logic mret;
                       logic mul;
                       logic rs1_sign;
                       logic rs2_sign;
                       logic low;
                       logic div;
                       logic rem;
                       logic fence;
                       logic fence_i;
                       logic pm_alu;
                       logic i0_only;
                       logic legal;
                       } eh2_dec_pkt_t;


typedef struct packed {
                       logic valid;
                       logic rs1_sign;
                       logic rs2_sign;
                       logic low;
                       logic load_mul_rs1_bypass_e1;
                       logic load_mul_rs2_bypass_e1;
                       logic bcompress;
                       logic bdecompress;
                       logic clmul;
                       logic clmulh;
                       logic clmulr;
                       logic grev;
                       logic gorc;
                       logic shfl;
                       logic unshfl;
                       logic crc32_b;
                       logic crc32_h;
                       logic crc32_w;
                       logic crc32c_b;
                       logic crc32c_h;
                       logic crc32c_w;
                       logic bfp;
                       logic xperm_n;
                       logic xperm_b;
                       logic xperm_h;
                       } eh2_mul_pkt_t;

typedef struct packed {
                       logic valid;
                       logic unsign;
                       logic rem;
                       logic tid;
                       } eh2_div_pkt_t;

typedef struct packed {
                       logic        TEST1;
                       logic        RME;
                       logic [3:0]  RM;

                       logic        LS;
                       logic        DS;
                       logic        SD;
                       logic        TEST_RNM;
                       logic        BC1;
                       logic        BC2;
                      } eh2_ccm_ext_in_pkt_t;

typedef struct packed {
                       logic        TEST1;
                       logic        RME;
                       logic [3:0]  RM;
                       logic        LS;
                       logic        DS;
                       logic        SD;
                       logic        TEST_RNM;
                       logic        BC1;
                       logic        BC2;
                      } eh2_dccm_ext_in_pkt_t;


typedef struct packed {
                       logic        TEST1;
                       logic        RME;
                       logic [3:0]  RM;
                       logic        LS;
                       logic        DS;
                       logic        SD;
                       logic        TEST_RNM;
                       logic        BC1;
                       logic        BC2;
                      } eh2_ic_data_ext_in_pkt_t;


typedef struct packed {
                       logic        TEST1;
                       logic        RME;
                       logic [3:0]  RM;
                       logic        LS;
                       logic        DS;
                       logic        SD;
                       logic        TEST_RNM;
                       logic        BC1;
                       logic        BC2;
                      } eh2_ic_tag_ext_in_pkt_t;


typedef struct packed {
                        logic        select;
                        logic        match;
                        logic        store;
                        logic        load;
                        logic        execute;
                        logic        m;
                        logic [31:0] tdata2;
            } eh2_trigger_pkt_t;


typedef struct packed {
                        logic [70:0]  icache_wrdata;
                        logic [16:0]  icache_dicawics;
                        logic         icache_rd_valid;
                        logic         icache_wr_valid;
            } eh2_cache_debug_pkt_t;

typedef struct packed {
                       logic [3:0] wayhit_f1;
                       logic [3:0] wayhit_p1_f1;
                       logic [1:0] tag_match_way0_f1;
                       logic [1:0] tag_match_way0_p1_f1;
                       logic [3:0] tag_match_vway1_expanded_f1;
                       } eh2_btb_sram_pkt;

typedef struct packed {
                       logic csr_misa;
                       logic csr_mvendorid;
                       logic csr_marchid;
                       logic csr_mimpid;
                       logic csr_mhartid;
                       logic csr_mstatus;
                       logic csr_mtvec;
                       logic csr_mip;
                       logic csr_mie;
                       logic csr_mcyclel;
                       logic csr_mcycleh;
                       logic csr_minstretl;
                       logic csr_minstreth;
                       logic csr_mscratch;
                       logic csr_mepc;
                       logic csr_mcause;
                       logic csr_mscause;
                       logic csr_mtval;
                       logic csr_mrac;
                       logic csr_dmst;
                       logic csr_mdseac;
                       logic csr_meihap;
                       logic csr_meivt;
                       logic csr_meipt;
                       logic csr_meicurpl;
                       logic csr_meicidpl;
                       logic csr_dcsr;
                       logic csr_mcgc;
                       logic csr_mfdc;
                       logic csr_dpc;
                       logic csr_mtsel;
                       logic csr_mtdata1;
                       logic csr_mtdata2;
                       logic csr_mhpmc3;
                       logic csr_mhpmc4;
                       logic csr_mhpmc5;
                       logic csr_mhpmc6;
                       logic csr_mhpmc3h;
                       logic csr_mhpmc4h;
                       logic csr_mhpmc5h;
                       logic csr_mhpmc6h;
                       logic csr_mhpme3;
                       logic csr_mhpme4;
                       logic csr_mhpme5;
                       logic csr_mhpme6;
                       logic csr_mitctl0;
                       logic csr_mitctl1;
                       logic csr_mitb0;
                       logic csr_mitb1;
                       logic csr_mitcnt0;
                       logic csr_mitcnt1;
                       logic csr_mpmc;
                       logic csr_mcpc;
                       logic csr_meicpct;
                       logic csr_mdeau;
                       logic csr_micect;
                       logic csr_miccmect;
                       logic csr_mdccmect;
                       logic csr_dicawics;
                       logic csr_dicad0h;
                       logic csr_dicad0;
                       logic csr_dicad1;
                       logic csr_dicago;
                       logic csr_mfdht;
                       logic csr_mfdhs;
                       logic csr_mcountinhibit;
                       logic csr_mhartnum;
                       logic csr_mhartstart;
                       logic csr_mnmipdel;
                       logic valid_only;
                       logic presync;
                       logic postsync;
                       logic glob;
                       logic legal;
                       } eh2_csr_tlu_pkt_t;


endpackage // eh2_pkg
//`endif
