// PlatformIO does not preprocess nested .ino files when src_dir is the repo root.
// The sketch declares its forward references so this and Arduino IDE share code.
#include <Arduino.h>
#include "../examples/grove-can-bus-cellular/grove-can-bus-cellular.ino"
