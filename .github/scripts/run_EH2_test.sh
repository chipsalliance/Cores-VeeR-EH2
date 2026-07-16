#!/bin/bash
set -e -u -o pipefail

COLOR_CLEAR="\033[0m"
COLOR_RED="\033[0;31m"
COLOR_GREEN="\033[1;32m"
COLOR_YELLOW="\033[1;33m"
COLOR_WHITE="\033[1;37m"

trap report_status EXIT

report_status(){
    rc=$?
    if [ $rc != 0 ]; then
        echo -e "${COLOR_WHITE}Test '${NAME}' ${COLOR_RED}FAILED${COLOR_CLEAR}"
    else
        echo -e "${COLOR_WHITE}Test '${NAME}' ${COLOR_GREEN}SUCCEEDED${COLOR_CLEAR}"
    fi
    exit $rc
}

check_args_count(){
    # Check argument count function is meant to be used to check if
    # the number of received arguments is equal to the expected.
    # If they are unequal, the function returns with error
    # Args:
    # argc_got - Number of received arguments, e.g.: $#
    # argc_expected - Number of expected arguments, e.g.: 2
    argc_got=$1
    argc_expected=$2
    if [ ${argc_got} -ne ${argc_expected} ]; then
        echo -e "${COLOR_WHITE}Expected ${argc_expected} arguments, but received ${argc_got} ${COLOR_RED}FAIL${COLOR_CLEAR}"
        echo -e "${COLOR_WHITE}Caller:${COLOR_CLEAR}" `caller`
        exit 1
    fi
}

run_regression_test(){
    # Run a regression test
    # Args:
    # RESULTS_DIR -
    # NAME -
    check_args_count $# 3
    RESULTS_DIR=$1
    BUS=$2
    NAME=$3

    PARAMS="-set iccm_enable"

    if [[ "${BUS}" == *axi* ]]; then
        PARAMS="${PARAMS} -set build_axi4"
    elif [[ "${BUS}" == *ahb* ]]; then
        PARAMS="${PARAMS} -set build_ahb_lite"
    fi
    echo -e "${COLOR_WHITE}========== running test '${NAME}' =========${COLOR_CLEAR}"
    echo -e "${COLOR_WHITE} RESULTS_DIR    = ${RESULTS_DIR}${COLOR_CLEAR}"
    echo -e "${COLOR_WHITE} NAME           = ${NAME}${COLOR_CLEAR}"
    echo -e "${COLOR_WHITE} SYSTEM BUS     = ${BUS}${COLOR_CLEAR}"
    echo -e "${COLOR_WHITE} CONF PARAMS    = ${PARAMS}${COLOR_CLEAR}"

    mkdir -p ${RESULTS_DIR}
    LOG="${RESULTS_DIR}/test_${NAME}_${BUS}.log"
    touch ${LOG}
    DIR="run_${NAME}"

    # Run the test
    mkdir -p ${DIR}
    make -j`nproc` -C ${DIR} -f $RV_ROOT/tools/Makefile verilator CONF_PARAMS="${PARAMS}" TEST=${NAME} 2>&1 | tee ${LOG}
}

# Example usage
# RESULTS_DIR=results
# BUS=axi
# NAME=hello_world
# run_regression_test.sh $RESULTS_DIR $BUS $NAME

check_args_count $# 3
run_regression_test "$@"
