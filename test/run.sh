#!/bin/sh
# Builds Wipeout Core against stand-in PROS headers (test/stubs) and runs the
# tests on your computer -- no robot needed. Usage: sh test/run.sh
cd "$(dirname "$0")/.." || exit 1
c++ -std=gnu++20 -pthread -Itest/stubs -Iinclude src/wpcore/*.cpp test/test.cpp -o test/wpcore_test && test/wpcore_test
