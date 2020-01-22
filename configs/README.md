# SweRV RISC-V EH2 core from Western Digital

## Configuration

### Contents
Name                    | Description
----------------------  | ------------------------------
swerv.config            | Configuration script for SweRV-EH2  


This script will generate a consistent set of `defines/#defines` needed for the design and testbench.  
A perl hash (*perl_configs.pl*) and a JSON format for SweRV-iss are also generated.
This set of include files :  

    $RV_ROOT/configs/snapshots/default
    ├── common_defines.vh                       # `defines for testbench or design
    ├── defines.h                               # #defines for C/assembly headers
    ├── eh2_param.vh                            # Design parameters
    ├── eh2_pdef.vh                             # Parameter structure
    ├── pd_defines.vh                           # `defines for physical design
    ├── perl_configs.pl                         # Perl %configs hash for scripting
    ├── pic_map_auto.h                          # PIC memory map based on configure size
    └── whisper.json                            # JSON file for swerv-iss



While the defines fines may be modified by hand, it is recommended that this script be used to generate a consistent set.

### Targets
There are 4 predefined target configurations: `default`, `default_mt`, `typical_pd` and `high_perf` that can be 
selected via the `-target=name` option to swerv.config.

Target                  | Description
----------------------  | ------------------------------
default                 | Default configuration. AXI4 bus interface. Single hart build.
default_mt              | Dual hart build, AXI4 bus interface
typical_pd              | No ICCM, AXI4 bus interface, dual hart
high_perf               | Dual hart, large BTB/BHT, AXI4 interface


`swerv.config` may be edited to add additional target configurations, or new configurations may be 
created via the command line `-set` or `-unset` options.
