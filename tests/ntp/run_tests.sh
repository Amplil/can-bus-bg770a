#!/usr/bin/env sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
TEST="$ROOT/tests/ntp"
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT HUP INT TERM
CXX=${CXX:-c++}
for name in time clock; do
  "$CXX" -std=c++11 -Wall -Wextra -Werror -pedantic -fsanitize=undefined -I"$ROOT/src" "$TEST/test_$name.cpp" -o "$OUT/$name"
  "$OUT/$name"
done
for name in actual_transport sketch_service; do
  "$CXX" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined -I"$ROOT/src" -I"$TEST/fixtures/test-support" -I"$TEST/fixtures/upstream/src" "$TEST/test_$name.cpp" -o "$OUT/$name"
  "$OUT/$name"
done

JSON_INCLUDE=${ARDUINOJSON_INCLUDE:-"$ROOT/.pio/libdeps/seeed_wio_bg770a/ArduinoJson/src"}
if [ -f "$JSON_INCLUDE/ArduinoJson.h" ]; then
  "$CXX" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined -I"$ROOT/src" -I"$JSON_INCLUDE" "$TEST/test_json.cpp" -o "$OUT/json"
  "$OUT/json"
else
  echo "ArduinoJson integration SKIPPED: run the PlatformIO dependency installation or set ARDUINOJSON_INCLUDE"
fi
