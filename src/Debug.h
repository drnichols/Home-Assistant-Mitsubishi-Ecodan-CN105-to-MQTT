// Global debug control for general app logs (MQTT, logic, etc.)
#define DEBUG 1

#define DEBUGPORT TelnetServer
#define DEBUGBAUD 115200

#ifdef DEBUG
#define DEBUG_PRINT(x) DEBUGPORT.print(x)
#define DEBUG_PRINTLN(x) DEBUGPORT.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// Separate flag and macros for low-level ECODAN serial I/O debugging
// These logs are intended to capture CN105 traffic (Ecodan.cpp) independently
// of general DEBUG logs.
extern bool gEnableEcodanSerialDebug; // defined in main.cpp

#ifndef ECODAN_SERIAL_DEBUG_PORT
#define ECODAN_SERIAL_DEBUG_PORT Serial
#endif

#ifndef ECODAN_TELNET_DEBUG_PORT
#define ECODAN_TELNET_DEBUG_PORT TelnetServer
#endif

#define ECODAN_DEBUG_PRINT(x)                                                           \
  do {                                                                                  \
    if (gEnableEcodanSerialDebug) {                                                     \
      auto _ecodan_debug_value = (x);                                                   \
      ECODAN_SERIAL_DEBUG_PORT.print(_ecodan_debug_value);                              \
      ECODAN_TELNET_DEBUG_PORT.print(_ecodan_debug_value);                              \
    }                                                                                   \
  } while (0)

#define ECODAN_DEBUG_PRINTLN(x)                                                         \
  do {                                                                                  \
    if (gEnableEcodanSerialDebug) {                                                     \
      auto _ecodan_debug_value = (x);                                                   \
      ECODAN_SERIAL_DEBUG_PORT.println(_ecodan_debug_value);                            \
      ECODAN_TELNET_DEBUG_PORT.println(_ecodan_debug_value);                            \
    }                                                                                   \
  } while (0)
