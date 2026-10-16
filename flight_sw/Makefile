# MiniSat flight software. Works with gcc, clang, or `zig cc` (CC="zig cc").
CC      ?= gcc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Iinclude -D_DEFAULT_SOURCE
LDLIBS  = -lm

LIB = src/codec.c src/hdlc.c src/ccsds.c src/obc.c

all: build/obc_sim build/unit_tests

build:
	mkdir -p build

build/obc_sim: $(LIB) src/obc_sim_main.c | build
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

build/unit_tests: $(LIB) tests/test_main.c | build
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

test: build/unit_tests
	./build/unit_tests

clean:
	rm -rf build

.PHONY: all test clean
