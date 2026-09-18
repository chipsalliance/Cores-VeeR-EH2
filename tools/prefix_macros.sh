# !/bin/bash

# Set flag to exit the script on first error
set -e

# Check if RV_ROOT is set
if [ -z "$RV_ROOT" ]; then
  echo "Error: RV_ROOT is not set."
  exit 1
fi

# Prefix that will be added to all required macro/struct/module names
PREFIX="${PREFIX:-veer0_}"
# Path to directory where common_defines.vh, eh2_param.vh, eh2_pdef.vh and pd_defines.vh reside
DEFINES_PATH="${DEFINES_PATH:-${RV_ROOT}/snapshots/default}"
# Path to directory hierarchy where RTL sources reside
DESIGN_DIR="${DESIGN_DIR:-${RV_ROOT}/design}"

COMMON_DEFINES="$DEFINES_PATH/common_defines.vh"
EH2_PARAM="$DEFINES_PATH/eh2_param.vh"
EH2_PDEF="$DEFINES_PATH/eh2_pdef.vh"
PD_DEFINES="$DEFINES_PATH/pd_defines.vh"
EH2_DEF="$DESIGN_DIR/include/eh2_def.sv"
EH2_IFU_IC_MEM="$DESIGN_DIR/ifu/eh2_ifu_ic_mem.sv"
EH2_PIC_CTRL="$DESIGN_DIR/eh2_pic_ctrl.sv"
PIC_MAP_AUTO="$DEFINES_PATH/pic_map_auto.svh"

echo "Starting script with following settings:"
echo "PREFIX=$PREFIX"
echo "DEFINES_PATH=$DEFINES_PATH"
echo -e "DESIGN_DIR=$DESIGN_DIR\n"

# Define regex patterns for matching defines
DEFINES_REGEX="s/((\`define)|(\`ifndef)|(\`undef)) ([A-Z0-9_]+).*/\5/p"
DEFINES_REPLACE_REGEX="s/((\`define)|(\`ifndef)|(\`undef)) ([A-Z0-9_]+)/\1 "$PREFIX"\5/"
STRUCT_REPLACE_REGEX="s/eh2_param_t/"$PREFIX"eh2_param_t/g"
MODULES_REGEX="s/^module ([\`A-Za-z0-9_]+).*/\1/p"

# Extract unique defines from all sources
DEFINES="$(sed -nr "$DEFINES_REGEX" $COMMON_DEFINES $PD_DEFINES $EH2_IFU_IC_MEM | sort -ur)"

# Skip files that should not be processed
SKIP_DESIGN_FILES="eh2_param.vh\|eh2_pdef.vh\|common_defines.vh\|pd_defines.vh"
DESIGN_FILES="$(find $DESIGN_DIR \( -name "*.sv" -o -name "*.vh" -o -name "*.svh" -o -name "*.v" \) | grep -v $SKIP_DESIGN_FILES)"
DESIGN_FILES+=" $EXTRA_DESIGN_FILES"
MODULES="$(sed -nr "$MODULES_REGEX" $DESIGN_FILES | sort -ur)"

if [ "${DEBUG}" = "1" ]; then
	echo "DEBUG: DEFINES_REGEX=$DEFINES_REGEX"
	echo "DEBUG: DEFINES_REPLACE_REGEX=$DEFINES_REPLACE_REGEX"
	echo "DEBUG: STRUCT_REPLACE_REGEX=$STRUCT_REPLACE_REGEX"
	echo "DEBUG: MODULES_REGEX=$MODULES_REGEX"
	echo
	echo "DEBUG: DEFINES=$DEFINES"
	echo "DEBUG: DESIGN_FILES=$DESIGN_FILES"
	echo "DEBUG: MODULES=$MODULES"
	echo
fi

# Add prefix to macro names
OUTPUT_COMMON_DEFINES=$DEFINES_PATH/"$PREFIX"common_defines.vh
OUTPUT_PD_DEFINES=$DEFINES_PATH/"$PREFIX"pd_defines.vh
echo "Adding prefix to macro names in $OUTPUT_COMMON_DEFINES and $OUTPUT_PD_DEFINES"
sed -E "$DEFINES_REPLACE_REGEX" $COMMON_DEFINES >$OUTPUT_COMMON_DEFINES
sed -E "$DEFINES_REPLACE_REGEX" $PD_DEFINES >$OUTPUT_PD_DEFINES

# Add prefix to RV_ICG macros
RV_RCG_REPLACE_REGEX="s/^(\`define "${PREFIX}"\w+_RV_ICG )(\w+)/\1"${PREFIX}"\2/g"
sed -i -E "$RV_RCG_REPLACE_REGEX" $OUTPUT_COMMON_DEFINES

# Add prefix to VeeR config struct
OUTPUT_EH2_PARAM=$DEFINES_PATH/"$PREFIX"eh2_param.vh
OUTPUT_EH2_PDEF=$DEFINES_PATH/"$PREFIX"eh2_pdef.vh
echo "Adding prefix to VeeR config struct in $OUTPUT_EH2_PARAM and $OUTPUT_EH2_PDEF"

sed "$STRUCT_REPLACE_REGEX" "$EH2_PARAM" >$OUTPUT_EH2_PARAM
sed "$STRUCT_REPLACE_REGEX" "$EH2_PDEF" >$OUTPUT_EH2_PDEF
sed -i "$STRUCT_REPLACE_REGEX" $DESIGN_FILES

# Replace renamed macros in RTL sources
echo "Replacing renamed macros in RTL sources"
for DEFINE in $DEFINES; do
	sed -i "s/\`$DEFINE/\`"$PREFIX"$DEFINE/g" $DESIGN_FILES
	sed -i -E "s/((\`ifdef)|(\`ifndef)) $DEFINE/\1 "$PREFIX"$DEFINE/g" $DESIGN_FILES
done

# Prefix all RV_* macros
echo "Prefixing all RV_* macros"
sed -i -E "s/((\`ifdef)|(\`ifndef)) (RV_[a-zA-Z0-9_]+)/\1 ${PREFIX}\4/g" $DESIGN_FILES
sed -i -E "s/(\`|\`define )(RV_[a-zA-Z0-9_]+)\b/\1${PREFIX}\2/g" $DESIGN_FILES

# Prefix all ASSERT_* macros
echo "Prefixing all ASSERT_* macros"
sed -i -E "s/(\`|\`define )(ASSERT_[a-zA-Z0-9_]+)\b/\1${PREFIX}\2/g" $DESIGN_FILES

# Prefix all EH2_* macros
echo "Prefixing all EH2_* macros"
sed -i -E "s/(\`|\`define |\`undef )(EH2_[a-zA-Z0-9_]+)\b/\1${PREFIX}\2/g" $DESIGN_FILES


# Replace include names in RTL sources
echo "Replacing include names in RTL sources"
sed -i "s/include \"eh2_param.vh\"/include \""$PREFIX"eh2_param.vh\"/g" $DESIGN_FILES
sed -i "s/include \"eh2_pdef.vh\"/include \""$PREFIX"eh2_pdef.vh\"/g" $DESIGN_FILES
sed -i "s/include \"common_defines.vh\"/include \""$PREFIX"common_defines.vh\"/g" $DESIGN_FILES
sed -i "s/include \"common_defines.vh\"/include \""$PREFIX"common_defines.vh\"/g" $OUTPUT_PD_DEFINES
sed -i "s/include \"pic_map_auto.svh\"/include \""$PREFIX"pic_map_auto.svh\"/g" $EH2_PIC_CTRL

# Ensure .svh includes are also updated with prefix
sed -i -E "s/include \"(eh2_[a-zA-Z0-9_]+)\.svh\"/include \"${PREFIX}\1.svh\"/g" $DESIGN_FILES

# Replace package name, its imports and usage in RTL sources
echo "Replacing package name and its imports in RTL sources"
sed -i "s/import eh2_pkg/import "$PREFIX"eh2_pkg/g" $DESIGN_FILES
sed -i "s/package eh2_pkg/package "$PREFIX"eh2_pkg/g" $EH2_DEF

# Add prefix to all module names
echo "Adding prefix to all module names"
perl -pi -e "s/module \`?(?!${PREFIX})([A-Za-z0-9_]+)/module ${PREFIX}\1/g" $DESIGN_FILES

# Add prefix to all module instantiations
echo "Adding prefix to all module instantiations"
for MODULE in $MODULES; do
    # Exclude the prefix from the MODULE name if it already contains the prefix
    MODULE=$(echo $MODULE | perl -pe "s/${PREFIX}//")
    echo "Processing MODULE=$MODULE"
    perl -pi -e "s/(^|[^A-Za-z0-9_])(?<!${PREFIX})${MODULE}([^A-Za-z0-9_]+)/\1${PREFIX}${MODULE}\2/g" $DESIGN_FILES
done

# Remove old header files to avoid redefining their contents during elaboration
echo "Removing old header files"
rm -f $COMMON_DEFINES $EH2_PARAM $EH2_PDEF $PD_DEFINES

# Add prefix to pic_map_auto.svh
OUTPUT_PIC_MAP_AUTO=$DEFINES_PATH/"$PREFIX"pic_map_auto.svh
mv $PIC_MAP_AUTO $OUTPUT_PIC_MAP_AUTO

# prefix memory macro names in eh2_ifu_ic_mem.sv
echo "Prefixing memory macro names in $EH2_IFU_IC_MEM"
perl -pi -e "s/(?<!${PREFIX})EH2_IC_TAG_PACKED_SRAM/${PREFIX}EH2_IC_TAG_PACKED_SRAM/g" $EH2_IFU_IC_MEM
perl -pi -e "s/(?<!${PREFIX})EH2_IC_TAG_SRAM/${PREFIX}EH2_IC_TAG_SRAM/g" $EH2_IFU_IC_MEM
perl -pi -e "s/(?<!${PREFIX})EH2_PACKED_IC_DATA_SRAM/${PREFIX}EH2_PACKED_IC_DATA_SRAM/g" $EH2_IFU_IC_MEM
perl -pi -e "s/(?<!${PREFIX})EH2_IC_DATA_SRAM/${PREFIX}EH2_IC_DATA_SRAM/g" $EH2_IFU_IC_MEM

# Add prefix to design file names
echo "Adding prefix to VeeR design file names"
for FILE_SRC in $DESIGN_FILES; do
    FILE_DIR="$(dirname -- "$(realpath "$FILE_SRC")")"
    FILE_NAME="$(basename "$FILE_SRC")"
    FILE_DEST="$FILE_DIR"/"$PREFIX""$FILE_NAME"
    echo "Renaming $FILE_SRC to $FILE_DEST"
    mv "$FILE_SRC" "$FILE_DEST"
done

echo "Script finished successfully"
