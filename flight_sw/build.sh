#!/bin/sh
# Build without make. Usage: ./build.sh            (uses $CC, default gcc)
#                           CC="zig cc" ./build.sh
set -e
CC=${CC:-gcc}
CFLAGS="-std=c99 -O2 -Wall -Wextra -Wpedantic -Iinclude -D_DEFAULT_SOURCE"
LIB="src/codec.c src/hdlc.c src/ccsds.c src/obc.c"
mkdir -p build
$CC $CFLAGS -o build/obc_sim $LIB src/obc_sim_main.c -lm
$CC $CFLAGS -o build/unit_tests $LIB tests/test_main.c -lm
echo "built build/obc_sim and build/unit_tests"
