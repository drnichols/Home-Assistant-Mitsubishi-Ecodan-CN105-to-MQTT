#include <ArduinoJson.h>

#include "fakes.hpp"

// Silence DEBUG_PRINT macros when running in native tests.
#ifndef DEBUG_PRINT
#define DEBUG_PRINT(x) do { (void)(x); } while (0)
#endif
#ifndef DEBUG_PRINTLN
#define DEBUG_PRINTLN(x) do { (void)(x); } while (0)
#endif

#include "../../../src/CalculateCompCurve.inc"
