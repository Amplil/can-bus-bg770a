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
for name in actual_transport; do
  "$CXX" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined -I"$ROOT/src" -I"$TEST/fixtures/test-support" -I"$TEST/fixtures/upstream/src" "$TEST/test_$name.cpp" -o "$OUT/$name"
  "$OUT/$name"
done
