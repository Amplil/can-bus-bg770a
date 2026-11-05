#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
output=$(mktemp)
trap 'rm -f "$output"' EXIT
${CXX:-c++} -std=c++11 -Wall -Wextra -Werror -pedantic -fsanitize=undefined -Isrc tests/obd/test_session.cpp -o "$output"
"$output"
json_include=${ARDUINOJSON_INCLUDE:-.pio/libdeps/seeed_wio_bg770a/ArduinoJson/src}
if [ -f "$json_include/ArduinoJson.h" ]; then
  generated=$(mktemp -d)
  trap 'rm -f "$output"; rm -rf "$generated"' EXIT
  python3 - "$generated/sketch-functions.h" <<'PY'
import pathlib, re, sys
sketch=pathlib.Path('examples/grove-can-bus-cellular/grove-can-bus-cellular.ino').read_text()
header=pathlib.Path('src/grove-can-bus.h').read_text()
defines='\n'.join(re.findall(r'^#define PID_.*',header,re.M))
start=sketch.index('void addCANMessageToJSON(unsigned long id,')
end=sketch.index('// 古いメッセージをクリーンアップ',start)
pathlib.Path(sys.argv[1]).write_text(defines+'\n'+sketch[start:end])
PY
  ${CXX:-c++} -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined -I"$json_include" -I"$generated" tests/obd/test_json.cpp -o "$output"
  "$output"
else
  echo 'Actual sketch JSON test NOT RUN: ArduinoJson headers unavailable'
fi
