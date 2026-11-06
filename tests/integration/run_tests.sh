#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
for source in tests/obd/test_session.cpp tests/integration/test_services.cpp tests/integration/test_diagnostics.cpp tests/integration/test_time_samples.cpp; do
 name=$(basename "$source" .cpp)
 "${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined -Isrc -Itests/gnss/fixtures/test-support -Itests/gnss/fixtures/upstream/src "$source" -o "$build/$name"
 "$build/$name"
done
python3 tests/integration/test_sketch.py

"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined -Itests/driver/fixtures -Isrc tests/driver/test_driver.cpp src/grove-can-bus.cpp -o "$build/driver"
"$build/driver"

json_include=${ARDUINOJSON_INCLUDE:-.pio/libdeps/seeed_wio_bg770a/ArduinoJson/src}
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined -Isrc -I"$json_include" tests/integration/test_endpoint.cpp -o "$build/endpoint"
"$build/endpoint"

python3 tests/compat/test_main_compatibility.py
