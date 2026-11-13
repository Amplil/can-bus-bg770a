#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
json_include=${ARDUINOJSON_INCLUDE:-.pio/libdeps/seeed_wio_bg770a/ArduinoJson/src}

bash tests/ntp/run_tests.sh
bash tests/gnss/run_tests.sh
bash tests/startup/run_tests.sh
for source in tests/obd/test_session.cpp tests/integration/test_services.cpp \
    tests/integration/test_diagnostics.cpp tests/integration/test_time_samples.cpp \
    tests/integration/test_endpoint.cpp tests/integration/test_batch.cpp \
    tests/integration/test_record.cpp; do
  name=$(basename "$source" .cpp)
  "${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined \
    -Isrc -I"$json_include" -Itests/gnss/fixtures/test-support \
    -Itests/gnss/fixtures/upstream/src "$source" -o "$build/$name"
  "$build/$name"
done
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined \
  -Itests/driver/fixtures -Isrc tests/driver/test_driver.cpp src/grove-can-bus.cpp \
  -o "$build/driver"
"$build/driver"
python3 tests/integration/test_sketch.py
