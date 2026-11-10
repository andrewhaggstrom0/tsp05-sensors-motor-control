#!/usr/bin/env bash
# Compile firmware modules against the Arduino shim and run host tests.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Werror -O1 \
    -I shim -I ../tsp05 \
    test_sensors_analog.cpp ../tsp05/sensors_analog.cpp \
    -o build/test_sensors_analog
./build/test_sensors_analog

g++ -std=c++17 -Wall -Wextra -Werror -O1 \
    -I shim -I ../tsp05 \
    test_sensors_digital.cpp ../tsp05/sensors_digital.cpp \
    -o build/test_sensors_digital
./build/test_sensors_digital
