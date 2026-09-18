#!/usr/bin/env bash
set -euo pipefail

elf="$1"
sym="${elf%.elf}.sym"

TEST_DIR="${elf%.elf}"
PROJECT_ROOT="$RV_ROOT"

mkdir -p "$TEST_DIR"

# Get hex file from ELF file
riscv64-unknown-elf-objcopy -O verilog $elf $TEST_DIR/program.hex

# Get all symbols from the ELF file
riscv64-unknown-elf-nm -B -n "$elf" > "$sym"
args=()

if grep -qE '(begin|end)_signature' "$sym"; then
    BEG=$(grep 'begin_signature' "$sym" | awk '{print $1}')
    END=$(grep 'end_signature'   "$sym" | awk '{print $1}')
    args+=(--mem-signature "$BEG" "$END")
fi

"$PROJECT_ROOT/obj_dir/Vtb_top" "${args[@]}" --outdir "$TEST_DIR"
