#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
wio_include=${WIOCELLULAR_INCLUDE:-.pio/libdeps/seeed_wio_bg770a/WioCellular/src}
if [[ ! -f "$wio_include/network/Bg770aNetwork.hpp" ]]; then
  echo "Set WIOCELLULAR_INCLUDE to the installed WioCellular/src directory." >&2
  exit 1
fi
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined \
  -Isrc -isystem "$wio_include" tests/startup/test_network_startup.cpp -o "$build/startup"
"$build/startup"
