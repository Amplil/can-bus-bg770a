#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
for source in tests/gnss/test_*.cpp; do
  name=$(basename "$source" .cpp)
  "${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined -Isrc -Itests/gnss/fixtures/test-support -Itests/gnss/fixtures/upstream/src "$source" -o "$build/$name"
  "$build/$name"
done

python3 tests/gnss/test_json_payload.py
