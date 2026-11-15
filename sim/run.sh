#!/bin/sh
# Compiles the RTL with a bench and runs it in the Vivado simulator (xsim).
#   sim/run.sh [bench] [rtl dir]
#     bench    tb_checks (default) or tb_pass
#     rtl dir  default: rtl/
# Environment:
#   VIVADO_BIN  directory of vivado, xvlog, xvhdl, xelab, xsim, if they are not on PATH
#   PART        part for generating the CORDIC core (default xczu7ev-ffvc1156-2-e)
#   BUILD       build directory (default build/). tb_pass reads its input files from here.
#   NOCOMPILE   if set, reuse the bench elaborated by an earlier run
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(dirname "$HERE")
TB=${1:-tb_checks}
RTL=$(cd "${2:-$ROOT/rtl}" && pwd)
BUILD=${BUILD:-$ROOT/build}
PART=${PART:-xczu7ev-ffvc1156-2-e}
mkdir -p "$BUILD"
BUILD=$(cd "$BUILD" && pwd)

tool() {
    if command -v "$1" >/dev/null 2>&1; then echo "$1"
    elif [ -e "$VIVADO_BIN/$1.bat" ]; then echo "$VIVADO_BIN/$1.bat"
    else echo "$VIVADO_BIN/$1"; fi
}
VIVADO=$(tool vivado); XVLOG=$(tool xvlog); XVHDL=$(tool xvhdl); XELAB=$(tool xelab); XSIM=$(tool xsim)

cd "$BUILD"
if [ ! -f ip/sim_files.txt ]; then
    "$VIVADO" -mode batch -nojournal -nolog -source "$ROOT/ip/cordic_sin_cos.tcl" \
        -tclargs "$BUILD/ip" "$PART" > ip_gen.log < /dev/null
fi
if [ ! -f ip/libs.txt ]; then
    : > ip_compile.log
    sed 's/\r$//' ip/sim_files.txt | while read -r lib file; do
        case "$file" in
            *.vhd) "$XVHDL" --work "$lib" "$file" >> ip_compile.log < /dev/null ;;
            *)     "$XVLOG" --work "$lib" "$file" >> ip_compile.log < /dev/null ;;
        esac
        echo "$lib" >> ip/libs.tmp
    done
    sort -u ip/libs.tmp > ip/libs.txt
    rm -f ip/libs.tmp
fi
LIBS=""
for l in $(cat ip/libs.txt); do LIBS="$LIBS -L $l"; done

if [ -z "$NOCOMPILE" ] || [ ! -d "xsim.dir/$TB" ]; then
    "$XVLOG" "$RTL"/*.v "$HERE/$TB.v" > xvlog.log < /dev/null || { cat xvlog.log; exit 1; }
    "$XELAB" -debug off $LIBS -L unisims_ver "$TB" -s "$TB" > xelab.log < /dev/null || { tail -40 xelab.log; exit 1; }
fi
"$XSIM" "$TB" -R > "$TB.log" < /dev/null || true
grep -E -v '^[[:space:]]*(INFO|\*|Vivado|Time resolution|source |#|run|exit|quit|start_gui|\$finish|$)' "$TB.log" || true
