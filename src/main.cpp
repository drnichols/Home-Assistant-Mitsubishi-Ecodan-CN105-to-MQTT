/*
    Mitsubishi Ecodan Bridge with Cascade Support

    This version extends the original bridge to support cascaded ASHP systems
    with master-slave controller configurations.

    Copyright (C) <2024>
*/

#include <Arduino.h> // Required for .cpp files

#if defined(ESP8266) || defined(ESP32) // ESP32 or ESP8266 Compatibility

#include <FS.h>
#include <LittleFS.h>

#ifdef ESP8266
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <SoftwareSerial.h>
#include <sys/time.h>
#endif

#ifdef ESP32
#include <WiFi.h>
#ifndef ARDUINO_WT32_ETH01
#include <AsyncTCP.h>
#endif
#include <ESPmDNS.h>
#include <WebServer.h>
#endif
#ifdef ARDUINO_WT32_ETH01
#include <ETH.h>
#endif

#ifndef ARDUINO_WT32_ETH01
#define WEBSERVER_H "fix conflict"
#include <ESPAsyncWebServer.h>
#endif
#include "Ecodan.h"
#include "Melcloud.h"
#include <ArduinoJson.h>
#include <ESPTelnet.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
// Cascade system includes
#include "CascadeNetwork.h"
#include "Flags.h"

// Project headers below will be included after struct definitions

String FirmwareVersion = "6.4.5-CASCADE";

// Pin definitions (same as original)
#ifdef ESP8266
#define HEATPUMP_STREAM SwSerial1
#define MEL_STREAM SwSerial2
#define SERIAL_CONFIG SWSERIAL_8E1
int LDR = A0;
#define FTCCable_TxPin 16
#define FTCCable_RxPin 14
#define MEL_TxPin 1
#define MEL_RxPin 3
int Activity_LED = 2;
int Reset_Button = 4;
int Green_RGB_LED = 12;
int Blue_RGB_LED = 13;
int Red_RGB_LED = 15;
#endif

#ifdef ESP32
#define HEATPUMP_STREAM Serial1
#define MEL_STREAM Serial2
#define SERIAL_CONFIG SERIAL_8E1

#ifdef ARDUINO_M5STACK_ATOMS3
#include <LiteLED.h>
#define LED_TYPE LED_STRIP_WS2812
#define LED_TYPE_IS_RGBW 0
#define LED_GPIO 35
#define LED_BRIGHT 100
static const crgb_t L_RED = 0xff0000;
static const crgb_t L_GREEN = 0x00ff00;
static const crgb_t L_BLUE = 0x0000ff;
static const crgb_t L_ORANGE = 0xffa500;
LiteLED myLED(LED_TYPE, LED_TYPE_IS_RGBW);
int Reset_Button = 41;
#define FTCCable_RxPin 2
#define FTCCable_TxPin 1
#define FTCProxy_RxPin 38
#define FTCProxy_TxPin 39
#define MEL_RxPin 8
#define MEL_TxPin 7
#endif

#ifdef ARDUINO_WT32_ETH01
#define FTCCable_RxPin 4
#define FTCCable_TxPin 2
#define MEL_RxPin 14
#define MEL_TxPin 12
#ifndef ETH_PHY_TYPE
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#define ETH_PHY_ADDR 0
#define ETH_PHY_MDC 23
#define ETH_PHY_MDIO 18
#define ETH_PHY_POWER -1
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN
#endif
#endif
#endif

#define Heartbeat_Range 99 // Heatbeat Max value
int Heart_Value = 0;       // Heatbeat ID

unsigned long SERIAL_BAUD = 2400;
bool shouldSaveConfig = false;
const int deviceId_max_length = 15;
const int hostname_max_length = 200;
const int port_max_length = 10;
const int user_max_length = 30;
const int password_max_length = 50;
const int basetopic_max_length = 30;

float Z1_CurveFSP, Z2_CurveFSP;

// Configuration structures (extended for cascade)
struct MqttSettings {
  char deviceId[15] = "000000000000";
  char wm_device_id_identifier[10] = "device_id";

  char hostname[200] = "homeassistant.local";
  char user[30] = "Username";
  char password[50] = "Password";
  char port[10] = "1883";
  char baseTopic[30] = "Ecodan/ASHP";
  char wm_mqtt_hostname_identifier[14] = "mqtt_hostname";
  char wm_mqtt_user_identifier[10] = "mqtt_user";
  char wm_mqtt_password_identifier[14] = "mqtt_password";
  char wm_mqtt_port_identifier[10] = "mqtt_port";
  char wm_mqtt_basetopic_identifier[15] = "mqtt_basetopic";

  char hostname2[200] = "IPorDNS";
  char user2[30] = "Username";
  char password2[50] = "Password";
  char port2[10] = "1883";
  char baseTopic2[30] = "Ecodan/ASHP";
  char wm_mqtt2_hostname_identifier[15] = "mqtt2_hostname";
  char wm_mqtt2_user_identifier[11] = "mqtt2_user";
  char wm_mqtt2_password_identifier[15] = "mqtt2_password";
  char wm_mqtt2_port_identifier[11] = "mqtt2_port";
  char wm_mqtt2_basetopic_identifier[16] = "mqtt2_basetopic";

  // Cascade dedicated base topic (shared across nodes)
  char cascadeBaseTopic[30] = "Ecodan/Cascade"; // defaults to baseTopic if not set in config
  char cascade_base_topic_identifier[20] = "cascade_base_topic";

  // Cascade configuration
  bool cascadeEnabled = false;
  uint8_t cascadeNodeId = 0;
  char cascade_enabled_identifier[16] = "cascade_enabled";
  char cascade_node_id_identifier[16] = "cascade_node_id";
  // Cascade test mode (bypass HP connection requirements)
  bool cascadeTestMode = false;
  char cascade_test_mode_identifier[20] = "cascade_test_mode";
};

struct UnitSettings {
  float UnitSize = 8.5;
  float GlycolStrength = 3.9;
  char unitsize_identifier[9] = "unitsize";
  char glycol_identifier[7] = "glycol";
  char compcurve_identifier[10] = "compcurve";
  String CompCurve =
      "{\"base\":{\"zone1\":{\"curve\":[{\"flow\":60,\"outside\":-10},{"
      "\"flow\":35,\"outside\":0},{\"flow\":20,\"outside\":15},{\"flow\":10,"
      "\"outside\":20}]},\"zone2\":{\"curve\":[{\"flow\":60,\"outside\":-10},{"
      "\"flow\":35,\"outside\":0},{\"flow\":20,\"outside\":15}]}},\"zone1\":{"
      "\"active\":false},\"zone2\":{\"active\":false}}";
  float z1_manual_offset = 0;
  float z1_wind_offset = 0;
  float z1_temp_offset = 0;
  float z2_manual_offset = 0;
  float z2_wind_offset = 0;
  float z2_temp_offset = 0;
  float cloud_outdoor = 0;
  bool use_local_outdoor = true;
  bool z1_active = false;
  bool z2_active = false;
};

// Project headers that require the struct definitions
#include "Debug.h"
#include "MQTTConfig.h"
#include "MQTTDiscovery.h"
#include "TimerCallBack.h"

// Global instances
MqttSettings mqttSettings;
UnitSettings unitSettings;
ECODAN HeatPump; // Legacy single unit (now master in cascade)
MELCLOUD MELCloud;
#ifdef ESP8266
SoftwareSerial SwSerial1;
SoftwareSerial SwSerial2;
#endif
WiFiClient NetworkClient1;
WiFiClient NetworkClient2;

// WiFiClientSecure NetworkClient;              // Encryption Support
PubSubClient MQTTClient1(NetworkClient1);
PubSubClient MQTTClient2(NetworkClient2);
ESPTelnet TelnetServer;
WiFiManager wifiManager;

// Cascade-specific variables
bool cascadeMode = false;

// Flags helper implementations (after globals exist in this TU)
namespace Flags {
  bool CascadeEnabled() { return mqttSettings.cascadeEnabled; }
  bool CascadeActive() { return cascadeMode; }
  bool CascadeMaster() { return CascadeActive() && mqttSettings.cascadeNodeId == 0; }
  bool CascadeSlave() { return CascadeActive() && mqttSettings.cascadeNodeId != 0; }
  bool ConfigCascadeMaster() { return mqttSettings.cascadeEnabled && mqttSettings.cascadeNodeId == 0; }
  bool ConfigCascadeSlave() { return mqttSettings.cascadeEnabled && mqttSettings.cascadeNodeId != 0; }
  bool HasCooling() { return HeatPump.Status.HasCooling; }
  bool Has2Zone() { return HeatPump.Status.Has2Zone; }
  bool CascadeTestMode() { return mqttSettings.cascadeTestMode; }
}

// WiFiManager parameters (including cascade option)
WiFiManagerParameter custom_mqtt_server("server",
                                        "<b>Required</b> Primary MQTT Server",
                                        "TEMP", 200);
WiFiManagerParameter custom_mqtt_user("user", "Primary MQTT Username", "TEMP",
                                      30);
WiFiManagerParameter custom_mqtt_pass("pass", "Primary MQTT Password", "TEMP",
                                      50);
WiFiManagerParameter custom_mqtt_port("port", "Primary MQTT Server Port",
                                      "TEMP", 10);
WiFiManagerParameter
    custom_mqtt_basetopic("basetopic", "Primary MQTT Base Topic", "TEMP", 30);
WiFiManagerParameter
    custom_mqtt2_server("server2", "<hr><b>Optional</b> Secondary MQTT Server",
                        "TEMP", 200);
WiFiManagerParameter custom_mqtt2_user("user2", "Secondary MQTT Username",
                                       "TEMP", 30);
WiFiManagerParameter custom_mqtt2_pass("pass2", "Secondary MQTT Password",
                                       "TEMP", 50);
WiFiManagerParameter custom_mqtt2_port("port2", "Secondary MQTT Server Port",
                                       "TEMP", 10);
WiFiManagerParameter custom_mqtt2_basetopic("basetopic2",
                                            "Secondary MQTT Base Topic", "TEMP",
                                            30);
CascadeCheckboxParameter custom_cascade_enabled(
    "cascade",
    "<hr><b>Cascade Mode</b><br>Enable for multi-unit systems<br><small><i>Changing this requires a reboot to take effect.</i></small>",
    "", 6);
WiFiManagerParameter
    custom_cascade_node_id("cascade_node",
                           "<br>Cascade Node ID (0=Master, 1-7=Slave)", "0", 5);
WiFiManagerParameter custom_cascade_basetopic(
    "cascade_basetopic", "Cascade Base Topic (shared)", "TEMP", 30);
// Optional: Test mode that allows cascade functions without HP connection
CascadeTestCheckboxParameter custom_cascade_test_mode(
    "cascade_test",
    "<br><b>Cascade Test Mode</b><br>Publish cascade data without heat pump connection",
    "", 6);
WiFiManagerParameter custom_device_id("device_id", "<hr>Device ID", "TEMP", 15);

// Implement dynamic checkbox rendering after MqttSettings is defined
const char *CascadeCheckboxParameter::getCustomHTML() const {
  return Flags::CascadeEnabled() ? "type='checkbox' checked value='true'"
                                 : "type='checkbox' value='true'";
}

// Test mode checkbox renderer
const char *CascadeTestCheckboxParameter::getCustomHTML() const {
  return Flags::CascadeTestMode() ? "type='checkbox' checked value='true'"
                                  : "type='checkbox' value='true'";
}

// Function declarations
void HeatPumpQueryStateEngine(void);
void HeatPumpWriteStateEngine(void);
void MELCloudQueryReplyEngine(void);
void HeatPumpQuerySVCEngine(void);
void HeatPumpKeepAlive(void);
void Zone1Report(void);
void Zone2Report(void);
void HotWaterReport(void);
void SystemReport(void);
void ConfigurationReport(void);
void AdvancedReport(void);
void AdvancedTwoReport(void);
void EnergyReport(void);
void StatusReport(void);
void CompCurveReport(void);
void CalculateCompCurve(void);
void FastPublish(void);

void initializeCascadeSystem(void);
void processCascadeSystem(void);
void publishCascadeReports(void);
void readSettingsFromConfig(void);
void saveConfig(void);
void initializeWifiManager(void);
void setupTelnet(void);
void startTelnet(void);
void RecalculateMQTTTopics(void);
void RecalculateMQTT2Topics(void);
void initializeMQTTClient1(void);
void initializeMQTTClient2(void);
uint8_t MQTTReconnect(void);
uint8_t MQTT2Reconnect(void);
void handleMQTTState(void);
void handleMQTT2State(void);
void PublishAllReports(void);

// Report functions
void syncCurrentTime(void);
void MQTTonData(char *topic, byte *payload, unsigned int length);
void onTelnetConnect(String ip);
void onTelnetConnectionAttempt(String ip);
void onTelnetReconnect(String ip);
void onTelnetDisconnect(String ip);
double round2(double value);
float roundToHalfDecimal(float value);
String decimalToBinary(int decimal);
void FlashGreenLED(void);
void MQTTWriteReceived(String message, int MsgNumber);
void ModifyCompCurveState(int Zone, bool Active);
void handleWiFiStatus(void);
void handlePushButton(void);
void handleDHWBoost(void);
void handleDefrost(void);

// Timer callbacks (modified for cascade support)
TimerCallBack HeatPumpQuery1(400, HeatPumpQueryStateEngine);
TimerCallBack HeatPumpQuery2(30000, HeatPumpKeepAlive);
TimerCallBack HeatPumpQuery3(30000, handleMQTTState);
TimerCallBack HeatPumpQuery4(30000, handleMQTT2State);
TimerCallBack HeatPumpQuery5(1000, HeatPumpWriteStateEngine);
TimerCallBack HeatPumpQuery6(2000, FastPublish);
TimerCallBack HeatPumpQuery7(300000, CalculateCompCurve);

// Global variables (same as original)
unsigned long looppreviousMicros = 0;
unsigned long ftcpreviousMillis = 0;
unsigned long wifipreviousMillis = 0;
unsigned long postwrpreviousMillis = 0;
unsigned long postdfpreviousMillis = 0;
int FTCLoopSpeed, CPULoopSpeed;
uint8_t SvcRequested = 0;
int16_t SvcReply = 0;
bool WiFiOneShot = true;
bool CableConnected = true;
bool WiFiConnectedLastLoop = false;
bool PostWriteTrigger = false;
bool PostDefrostTimer = false;

extern int cmd_queue_length;
extern int cmd_queue_position;
extern bool WriteInProgress;
extern int CurrentWriteAttempt;
byte NormalHWBoostOperating = 0;
byte PreHWBoostSvrCtrlMode = 0;
uint8_t FTCVersionLastLoop = 0;

#ifdef ARDUINO_WT32_ETH01
static bool eth_connected = false;
#endif

void setup() {
  WiFi.mode(WIFI_STA);
  Serial.begin(115200);

  DEBUG_PRINTLN("Starting Ecodan Bridge with Cascade Support");
  DEBUG_PRINT("Firmware Version: ");
  DEBUG_PRINTLN(FirmwareVersion);

  // Initialize serial communications
  HEATPUMP_STREAM.begin(2400, SERIAL_CONFIG, FTCCable_RxPin, FTCCable_TxPin);
  HeatPump.SetStream(&HEATPUMP_STREAM);
  MEL_STREAM.begin(2400, SERIAL_CONFIG, MEL_RxPin, MEL_TxPin);
  MELCloud.SetStream(&MEL_STREAM);

#ifdef ARDUINO_WT32_ETH01
  // Network.onEvent(onEvent);  // This would need to be implemented
  ETH.begin();
#endif

#ifndef ARDUINO_WT32_ETH01
  pinMode(Reset_Button, INPUT);
#endif

  // Initialize LEDs (same as original)
#ifdef ARDUINO_M5STACK_ATOMS3
  myLED.begin(LED_GPIO, 1);
  myLED.brightness(LED_BRIGHT);
#endif
#ifdef ESP8266
  pinMode(Activity_LED, OUTPUT);
  pinMode(LDR, INPUT);
  pinMode(Red_RGB_LED, OUTPUT);
  pinMode(Green_RGB_LED, OUTPUT);
  pinMode(Blue_RGB_LED, OUTPUT);
  digitalWrite(Activity_LED, HIGH);
  digitalWrite(Red_RGB_LED, LOW);
  digitalWrite(Green_RGB_LED, LOW);
  digitalWrite(Blue_RGB_LED, LOW);
#endif

  // Load configuration
  readSettingsFromConfig();

  // Check if cascade mode is enabled
  cascadeMode = mqttSettings.cascadeEnabled;

  if (Flags::CascadeActive()) {
    DEBUG_PRINTLN("Cascade mode enabled - initializing cascade system");
    initializeCascadeSystem();
  } else {
    DEBUG_PRINTLN("Single unit mode - using legacy operation");
  }

  // Initialize WiFi and MQTT (same as original)
  initializeWifiManager();
  if (shouldSaveConfig) {
    saveConfig();
  }

  setupTelnet();
  startTelnet();

  MQTTClient1.setBufferSize(2048);
  MQTTClient2.setBufferSize(2048);

  RecalculateMQTTTopics();
  RecalculateMQTT2Topics();

  initializeMQTTClient1();
  MQTTClient1.setCallback(MQTTonData);

  initializeMQTTClient2();
  MQTTClient2.setCallback(MQTTonData);

  // Initialize cascade MQTT if enabled
  if (Flags::CascadeActive()) {
    cascadeNetwork.setMQTTClient(&MQTTClient1, String(mqttSettings.cascadeBaseTopic));
    // Reflect current Unit Size into cascade capacity
    cascadeNetwork.setLocalUnitCapacity(unitSettings.UnitSize);
  }

  wifiManager.startWebPortal();

  MDNS.begin("heatpump");
  MDNS.addService("http", "tcp", 80);

  HeatPump.Status.Write_To_Ecodan_OK = false;

  HeatPumpKeepAlive();

  DEBUG_PRINTLN("Setup complete");
}

void loop() {
  looppreviousMicros = micros();

  // Process timer callbacks
  HeatPumpQuery1.Process();
  HeatPumpQuery2.Process();
  HeatPumpQuery3.Process();
  HeatPumpQuery4.Process();
  HeatPumpQuery5.Process();
  HeatPumpQuery6.Process();
  HeatPumpQuery7.Process();

  // Process cascade system if enabled
  if (Flags::CascadeActive()) {
    processCascadeSystem();
  }

  // Process other systems (same as original)
  MELCloudQueryReplyEngine();
  MQTTClient1.loop();
  MQTTClient2.loop();
  TelnetServer.loop();

  if (Flags::CascadeActive()) {
    // In cascade mode, process network
    cascadeNetwork.process();
    HeatPump.Process();
  } else {
    // Single unit processing
    HeatPump.Process();
  }

  MELCloud.Process();
  wifiManager.process();

  // Configuration save handler
  if (shouldSaveConfig) {
    saveConfig();
  }

  // Write command handler (restored original queue logic)
  if (HeatPump.Status.Write_To_Ecodan_OK &&
      WriteInProgress) {           // A write command is executing
    DEBUG_PRINTLN(F("Write OK!")); // Pause normal processing until complete
    HeatPump.Status.Write_To_Ecodan_OK = false;  // Set back to false
    WriteInProgress = false;                     // Set back to false
    if (cmd_queue_length > cmd_queue_position) { // If the queue has items
      cmd_queue_position++;                      // Increment the position
      CurrentWriteAttempt = 0;                   //
    } else {                                     // All commands written, reset
      cmd_queue_position = 1;
      cmd_queue_length = 0;
      CurrentWriteAttempt = 0;
      PostWriteTrigger =
          true; // Allows 1s to pass, then restarts read operation
      postwrpreviousMillis = millis();
    } // Dequeue the last message that was written
    if (MQTTReconnect() || MQTT2Reconnect()) {
      if (Flags::CascadeActive()) {
        publishCascadeReports();
      } else {
        PublishAllReports();
      }
    } // Publish update to the MQTT Topics
  } else if ((WriteInProgress) &&
             (CurrentWriteAttempt > 10)) { // After 10 attempts to write
    if (cmd_queue_length > cmd_queue_position) {
      cmd_queue_position++; // Skip this write + Increment the position
      CurrentWriteAttempt = 0;
    } else {
      cmd_queue_position = 1; // All commands written, reset
      cmd_queue_length = 0;
      CurrentWriteAttempt = 0;
      PostWriteTrigger =
          true; // Allows 1s to pass, then restarts read operation
      postwrpreviousMillis = millis();
    }
  }

  // Read Operation Restart
  if ((PostWriteTrigger) &&
      (millis() - postwrpreviousMillis >=
       1000)) { // Allow 1s to pass before re-starting reads for FTC to process
    DEBUG_PRINTLN(F("Restarting Read Operations"));
    HeatPumpKeepAlive();
    PostWriteTrigger = false;
  }

  // Time sync
  if (HeatPump.Status.SyncTime) {
    syncCurrentTime();
  }

  // Service code handler
  // HeatPumpServiceCodeHandler();  // DN needed?

  // FTC7 + R290 Outdoor Limit Adjustments
  if (FTCVersionLastLoop != HeatPump.Status.FTCVersion &&
      HeatPump.Status.RefrigerantType ==
          2) { // Dynamic Update HA limit for FTC7
    if (MQTTReconnect()) {
      PublishDiscoveryTopics(1, MQTT_BASETOPIC);
    }
    if (MQTT2Reconnect()) {
      PublishDiscoveryTopics(2, MQTT_2_BASETOPIC);
    }
  }
  FTCVersionLastLoop =
      HeatPump.Status
          .FTCVersion; // On FTC version capture, if criteria met then change

  // CPU loop time monitoring
  CPULoopSpeed = micros() - looppreviousMicros;
}

void initializeCascadeSystem() {
  DEBUG_PRINTLN("Initializing distributed cascade system...");

  // Determine node type based on configuration
  CascadeNodeType nodeType = (mqttSettings.cascadeNodeId == 0)
                                 ? CASCADE_NODE_MASTER
                                 : CASCADE_NODE_SLAVE;
  String nodeName = (nodeType == CASCADE_NODE_MASTER)
                        ? "Master Node"
                        : ("Slave Node " + String(mqttSettings.cascadeNodeId));

  // Initialize cascade network
  cascadeNetwork.initialize(nodeType, mqttSettings.cascadeNodeId, nodeName);
  cascadeNetwork.setLocalUnit(&HeatPump);
  cascadeNetwork.begin();

  DEBUG_PRINT("Cascade node initialized: ");
  DEBUG_PRINT(nodeName);
  DEBUG_PRINT(" (ID: ");
  DEBUG_PRINT(mqttSettings.cascadeNodeId);
  DEBUG_PRINTLN(")");
}

void processCascadeSystem() { cascadeNetwork.process(); }

void publishCascadeReports() {
  // Individual node data is published automatically by CascadeNetwork
  // Master node publishes system-wide status
  if (cascadeNetwork.isMasterNode()) {
    cascadeNetwork.broadcastSystemStatus();
  }
}

void PublishAllReports(void) {
  // Increment the Heartbeat ID Counter
  ++Heart_Value;
  if (Heart_Value > Heartbeat_Range) {
    Heart_Value = 1;
  }

  Zone1Report();
  if (Flags::Has2Zone()) {
    Zone2Report();
  }
  HotWaterReport();
  SystemReport();
  ConfigurationReport();
  AdvancedReport();
  AdvancedTwoReport();
  EnergyReport();
  StatusReport();
  CompCurveReport();
  FlashGreenLED();
  DEBUG_PRINTLN(F("MQTT Published!"));
}

void FastPublish(void) {
  // When FTCVersion is known, publish the standard fast set once.
  if (HeatPump.Status.FTCVersion != 0) {
    SystemReport();
    HotWaterReport();
    AdvancedReport();
    AdvancedTwoReport();
  } else if (Flags::CascadeActive()) {
    // In cascade mode, publish SystemReport early so HA sensors have data
    // even before FTCVersion is populated.
    SystemReport();
  } // Don't fast publish until at least whole data set gathering is complete
}

void CalculateCompCurve() {
  DEBUG_PRINTLN("Performing Compensation Curve Calculation");
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, unitSettings.CompCurve);
  if (error) {
    DEBUG_PRINT("Failed to read: ");
    DEBUG_PRINTLN(error.c_str());
  } else {
    unitSettings.z1_active =
        doc["zone1"]["active"]; // Transfer JSON to Struct Bool
    unitSettings.z2_active = doc["zone2"]["active"];
    // if (!unitSettings.z1_active && !unitSettings.z2_active) { return; } else
    // // Only calculates (saves time, if mode enabled)
    {
      float OutsideAirTemperature = 0;

      if (!unitSettings.use_local_outdoor &&
          (MQTTClient1.connected() ||
           MQTTClient2.connected())) { // Determine Outdoor Temperature Input
        OutsideAirTemperature = doc["cloud_outdoor"];
      } else {
        OutsideAirTemperature = HeatPump.Status.OutsideTemperature;

        if (HeatPump.Status.Defrost != 0 ||
            ((PostDefrostTimer) &&
             (millis() - postdfpreviousMillis <
              240000))) { // To allow sensor to stabilise after influence from
                          // the defrost
          return; // If currently defrosting or less than 4 minutes post-defrost
                  // skip re-calculation
        } else {
          PostDefrostTimer = false;
        }
      }

      int z1_points = doc["base"]["zone1"]["curve"].size() -
                      1; // How many points are there specified on the curve
      for (int i = 0; i <= z1_points; i++) { // Iterate through the points
        float tmp_o_1 =
            doc["base"]["zone1"]["curve"][i]["outside"]; // Outside Temperature
        if ((i == 0) && (OutsideAirTemperature <
                         tmp_o_1)) { // On the first point, this determines the
                                     // Maximum Flow Temp
          Z1_CurveFSP = doc["base"]["zone1"]["curve"][i]["flow"]; //
        } else if ((i == z1_points) && (OutsideAirTemperature >
                                        tmp_o_1)) { // The last point determines
                                                    // the Minimum Flow Temp
          Z1_CurveFSP = doc["base"]["zone1"]["curve"][i]["flow"]; //
        } else { // Intermediate Flow Points
          float tmp_o_2 = doc["base"]["zone1"]["curve"][i + 1]
                             ["outside"]; // Outside Temperature
          if ((OutsideAirTemperature >= tmp_o_1) &&
              (OutsideAirTemperature <=
               tmp_o_2)) { // Validate the outside temp setpoint is in the
                           // correct range
            float y2 = doc["base"]["zone1"]["curve"][i + 1]
                          ["flow"]; // Calculate the slope using the formula: m
                                    // = (y2 - y1) / (x2 - x1)
            float y1 = doc["base"]["zone1"]["curve"][i]["flow"]; //
            float z1_delta_y = y2 - y1;                          // y2-y1
            float z1_delta_x = tmp_o_2 - tmp_o_1;                // x2-x1
            float z1_m = 0;                                      //
            if (z1_delta_x > 0) {
              z1_m = z1_delta_y / z1_delta_x;
            } // Prevent Div by 0          m = y2-y1 / x2-x1
            float z1_c = y1 - (z1_m * tmp_o_1);                // c = y-mx
            Z1_CurveFSP = z1_m * OutsideAirTemperature + z1_c; // y = mx+c
          }
        }
      }

      int z2_points = doc["base"]["zone2"]["curve"].size() -
                      1; // How many points are there specified on the curve
      for (int i = 0; i <= z2_points; i++) {
        float tmp_o_1 = doc["base"]["zone2"]["curve"][i]["outside"];
        if ((i == 0) && (OutsideAirTemperature <= tmp_o_1)) { // Max Flow Temp
          Z2_CurveFSP = doc["base"]["zone2"]["curve"][i]["flow"];
        } else if ((i == z2_points) &&
                   (OutsideAirTemperature >= tmp_o_1)) { // Min Flow Temp
          Z2_CurveFSP = doc["base"]["zone2"]["curve"][i]["flow"];
        } else {
          float tmp_o_2 = doc["base"]["zone2"]["curve"][i + 1]["outside"];
          if ((OutsideAirTemperature > tmp_o_1) &&
              (OutsideAirTemperature < tmp_o_2)) {
            float y2 = doc["base"]["zone2"]["curve"][i + 1]
                          ["flow"]; // Calculate the slope using the formula: m
                                    // = (y2 - y1) / (x2 - x1)
            float y1 = doc["base"]["zone2"]["curve"][i]["flow"];
            float z2_delta_y = y2 - y1;
            float z2_delta_x = tmp_o_2 - tmp_o_1;
            float z2_m = 0; //
            if (z2_delta_x > 0) {
              float z2_m = z2_delta_y / z2_delta_x;
            } // Prevent Div by 0          m = y2-y1 / x2-x1
            float z2_c = y2 - (z2_m * tmp_o_1);                // c = y-mx
            Z2_CurveFSP = z2_m * OutsideAirTemperature + z2_c; // y = mx+c
          }
        }
      }
    }

    // Apply Post Calculation Offsets to Calculated Curve Flow Setpoint
    Z1_CurveFSP = roundToHalfDecimal(Z1_CurveFSP + unitSettings.z1_wind_offset +
                                     unitSettings.z1_temp_offset +
                                     unitSettings.z1_manual_offset);
    Z2_CurveFSP = roundToHalfDecimal(Z2_CurveFSP + unitSettings.z2_wind_offset +
                                     unitSettings.z2_temp_offset +
                                     unitSettings.z2_manual_offset);

    // Write the Flow Setpoints to Heat Pump
    if (unitSettings.z1_active) {
      HeatPump.SetFlowSetpoint(Z1_CurveFSP,
                               HeatPump.Status.HeatingControlModeZ1, ZONE1);
      HeatPump.Status.Zone1FlowTemperatureSetpoint = Z1_CurveFSP;
    }
    if (unitSettings.z2_active) {
      HeatPump.SetFlowSetpoint(Z2_CurveFSP,
                               HeatPump.Status.HeatingControlModeZ2, ZONE2);
      HeatPump.Status.Zone2FlowTemperatureSetpoint = Z2_CurveFSP;
    }
    CompCurveReport();
  }
}

// WiFi status handler (adapted from original)
void handleWiFiStatus() {
  if (WiFi.status() != WL_CONNECTED && !wifiManager.getConfigPortalActive()) {
    if (WiFiOneShot) {
      wifipreviousMillis = millis();
      WiFiOneShot = false;
#ifdef ESP8266
      digitalWrite(Blue_RGB_LED, LOW);
      digitalWrite(Green_RGB_LED, LOW);
      digitalWrite(Red_RGB_LED, HIGH);
#endif
#ifdef ARDUINO_M5STACK_ATOMS3
      myLED.setPixel(0, L_RED, 1);
#endif
    }
    if (millis() - wifipreviousMillis >= 300000) {
#ifdef ESP8266
      digitalWrite(Red_RGB_LED, HIGH);
      delay(500);
      digitalWrite(Red_RGB_LED, LOW);
      delay(500);
      digitalWrite(Red_RGB_LED, HIGH);
      delay(500);
      digitalWrite(Red_RGB_LED, LOW);
      delay(500);
      digitalWrite(Red_RGB_LED, HIGH);
      ESP.reset();
#endif
#ifdef ARDUINO_M5STACK_ATOMS3
      myLED.setPixel(0, L_RED, 1);
      myLED.brightness(LED_BRIGHT, 1);
      delay(500);
      myLED.brightness(0, 1);
      delay(500);
      myLED.brightness(LED_BRIGHT, 1);
      delay(500);
      myLED.brightness(0, 1);
      delay(500);
      myLED.brightness(LED_BRIGHT, 1);
      ESP.restart();
#endif
    }
    WiFiConnectedLastLoop = false;
  } else if (WiFi.status() != WL_CONNECTED &&
             wifiManager.getConfigPortalActive()) {
#ifdef ESP8266
    digitalWrite(Blue_RGB_LED, HIGH);
    analogWrite(Green_RGB_LED, LOW);
    digitalWrite(Red_RGB_LED, LOW);
#endif
#ifdef ARDUINO_M5STACK_ATOMS3
    myLED.setPixel(0, L_BLUE, 1);
#endif
    WiFiConnectedLastLoop = false;
  } else {
    if (!WiFiConnectedLastLoop) {
#ifdef ESP8266
      digitalWrite(Blue_RGB_LED, LOW);
      analogWrite(Green_RGB_LED, 30);
      digitalWrite(Red_RGB_LED, LOW);
#endif
#ifdef ARDUINO_M5STACK_ATOMS3
      myLED.setPixel(0, L_GREEN, 1);
#endif
    }
    WiFiOneShot = true;
    WiFiConnectedLastLoop = true;
  }
}

// Push button handler (adapted from original)
void handlePushButton() {
#ifndef ARDUINO_WT32_ETH01
  if (digitalRead(Reset_Button) == LOW) {
    HeatPump.SetSvrControlMode(
        0, HeatPump.Status.ProhibitDHW, HeatPump.Status.ProhibitHeatingZ1,
        HeatPump.Status.ProhibitCoolingZ1, HeatPump.Status.ProhibitHeatingZ2,
        HeatPump.Status.ProhibitCoolingZ2);
    ModifyCompCurveState(1, false);
    ModifyCompCurveState(2, false);

#ifdef ESP8266
    digitalWrite(Red_RGB_LED, HIGH);
    delay(500);
    digitalWrite(Red_RGB_LED, LOW);
    delay(500);
    digitalWrite(Red_RGB_LED, HIGH);
    delay(500);
    digitalWrite(Red_RGB_LED, LOW);
    delay(500);
    digitalWrite(Red_RGB_LED, HIGH);
    delay(500);
#endif
#ifdef ARDUINO_M5STACK_ATOMS3
    myLED.setPixel(0, L_RED, 1);
    delay(500);
    myLED.brightness(0, 1);
    delay(500);
    myLED.brightness(LED_BRIGHT, 1);
    delay(500);
    myLED.brightness(0, 1);
    delay(500);
    myLED.brightness(LED_BRIGHT, 1);
    delay(500);
#endif

    if (digitalRead(Reset_Button) == LOW) {
#ifdef ESP8266
      digitalWrite(Red_RGB_LED, LOW);
      digitalWrite(Blue_RGB_LED, HIGH);
      delay(500);
#endif
#ifdef ARDUINO_M5STACK_ATOMS3
      myLED.setPixel(0, L_BLUE, 1);
#endif
      delay(500);
      wifiManager.resetSettings();
      LittleFS.format();
    }

#ifdef ESP8266
    ESP.reset();
#endif
#ifdef ESP32
    ESP.restart();
#endif
  }
#endif
}

// DHW boost handler (adapted for cascade)
void handleDHWBoost() {
  if ((HeatPump.Status.LastSystemOperationMode == 1 ||
       HeatPump.Status.LastSystemOperationMode == 6) &&
      HeatPump.Status.SystemOperationMode != 1 && NormalHWBoostOperating == 1) {
    HeatPump.SetSvrControlMode(
        PreHWBoostSvrCtrlMode, 1, HeatPump.Status.ProhibitHeatingZ1,
        HeatPump.Status.ProhibitCoolingZ1, HeatPump.Status.ProhibitHeatingZ2,
        HeatPump.Status.ProhibitCoolingZ2);
    WriteInProgress = true;
    NormalHWBoostOperating = 0;
  }
}

// Defrost handler (adapted for cascade)
void handleDefrost() {
  if (unitSettings.use_local_outdoor && HeatPump.Status.LastDefrost != 0 &&
      HeatPump.Status.Defrost == 0) {
    postdfpreviousMillis = millis();
    PostDefrostTimer = true;
  }
}

#endif // ESP8266 || ESP32
// Core function implementations (adapted from original)

void HeatPumpKeepAlive(void) {
  if (!HeatPump.HeatPumpConnected()) {
    DEBUG_PRINTLN(F("Heat Pump Disconnected"));
#ifdef ARDUINO_M5STACK_ATOMS3
    if (CableConnected) {
      DEBUG_PRINTLN(F("Trying to connect via Proxy Circuit Board"));
      HEATPUMP_STREAM.begin(SERIAL_BAUD, SERIAL_CONFIG, FTCProxy_RxPin,
                            FTCProxy_TxPin);
      HeatPump.SetStream(&HEATPUMP_STREAM);
      CableConnected = false;
    } else {
      DEBUG_PRINTLN(F("Trying to connect via Cable"));
      HEATPUMP_STREAM.begin(SERIAL_BAUD, SERIAL_CONFIG, FTCCable_RxPin,
                            FTCCable_TxPin);
      HeatPump.SetStream(&HEATPUMP_STREAM);
      CableConnected = true;
    }
#endif
  }
  ftcpreviousMillis = millis();
  HeatPump.TriggerStatusStateMachine();
}

void HeatPumpQueryStateEngine(void) {
  if (cmd_queue_length == 0) {     // If there is no commands awaiting written
    HeatPump.StatusStateMachine(); // Full Read trigged by CurrentMessage
  }

  // Call Once Full Update is complete
  if (HeatPump.UpdateComplete()) {
    DEBUG_PRINTLN(F("Update Complete"));
    FTCLoopSpeed = millis() - ftcpreviousMillis; // Loop Speed End

    if (HeatPump.Status.FTCVersion == 0) {
      HeatPump.GetFTCVersion();
      if (MQTTReconnect() || MQTT2Reconnect()) {
        StatusReport();
        CalculateCompCurve();
      }
    } else {
      HeatPump.SVCUpdateComplete();
      HeatPump.StatusSVCMachine(); // Call service codes
      if (MQTTReconnect() || MQTT2Reconnect()) {
        if (Flags::CascadeActive()) {
          publishCascadeReports();
          PublishAllReports();
        } else {
          PublishAllReports();
        }
      }
    }
  }
}

void HeatPumpQuerySVCEngine(void) { HeatPump.StatusSVCMachine(); }

void HeatPumpWriteStateEngine(void) { HeatPump.WriteStateMachine(); }

void MELCloudQueryReplyEngine(void) {
  if (MELCloud.Status.ReplyNow) {
    if (MELCloud.Status.ActiveMessage == 0x28 &&
        MELCloud.Status.MEL_Heartbeat) { // Toggle the Heartbeat High for this
                                         // request (MELCloud Only)
      DEBUG_PRINTLN("Setting Heartbeat Byte");
      Array0x28[11] = 1;
      MELCloud.Status.MEL_Heartbeat = false;
    } else if (MELCloud.Status.ActiveMessage == 0x28 &&
               !MELCloud.Status.MEL_Heartbeat) { // Toggle the Heartbeat Low for
                                                 // other requests
      Array0x28[11] = 0;
    }
    MELCloud.ReplyStatus(MELCloud.Status.ActiveMessage);
    MELCloud.Status.ReplyNow = false;

    if (MELCloud.Status.ActiveMessage == 0x32 ||
        MELCloud.Status.ActiveMessage == 0x33 ||
        MELCloud.Status.ActiveMessage == 0x34 ||
        MELCloud.Status.ActiveMessage == 0x35) { // The writes
      HeatPump.WriteMELCloudCMD(MELCloud.Status.ActiveMessage);
    }
  } else if ((MELCloud.Status.ConnectRequest) &&
             (HeatPump.Status.FTCVersion != 0)) {
    MELCloud.Connect(); // Reply to the connect request
    MELCloud.Status.ConnectRequest = false;
  } else if (MELCloud.Status.MELRequest1) {
    MELCloud.MELNegotiate1(); // Reply to the connect request
    MELCloud.Status.MELRequest1 = false;
  } else if (MELCloud.Status.MELRequest2) {
    MELCloud.MELNegotiate2(); // Reply to the connect request (MELCloud Only)
    MELCloud.Status.MELRequest2 = false;
  } else if (MELCloud.Status
                 .MEL_HB_Request) { // Reply to the MELCloud Heartbeat
    MELCloud.ReplyStatus(0x34);
    MELCloud.Status.MEL_HB_Request = false;
  }
}

// DN used?
void MQTTonDisconnect(void *response) { DEBUG_PRINTLN(F("MQTT Disconnect")); }

// void HeatPumpServiceCodeHandler(void) {
//   // Service code query engine
//   if (SvcRequested != 0) {
//     HeatPump.WriteServiceCodeCMD(SvcRequested);
//     SvcRequested = 0;
//   }
// }

// Modified MQTT callback to handle cascade commands
void MQTTonData(char *topic, byte *payload, unsigned int length) {
  payload[length] = 0;
  String Topic = topic;
  String Payload = (char *)payload;

  DEBUG_PRINT(F("\nReceived MQTT Message on topic: "));
  DEBUG_PRINT(Topic.c_str());
  DEBUG_PRINT(F(" with Payload: "));
  DEBUG_PRINTLN(Payload.c_str());

  // Check if this is a cascade network message
  if (Flags::CascadeActive()) {
    if (Topic.indexOf("/cascade/") >= 0) {
      cascadeNetwork.handleRemoteData(Topic, Payload);
      return;
    }
    // Allow cascade to consume auxiliary topics (e.g., LWT)
    if (cascadeNetwork.handleAuxTopic(Topic, Payload)) {
      return;
    }
  }

  // Original MQTT command handling for single unit or master unit commands
  // Service Codes
  if ((Topic == MQTTCommandSystemService) ||
      (Topic == MQTTCommand2SystemService)) {
    if (Payload.toInt() == 999) {
      DEBUG_PRINTLN(F("FTC Bridge Restart Request"));
#ifdef ESP8266
      ESP.reset();
#endif
#ifdef ESP32
      ESP.restart();
#endif
    } else if (Payload.toInt() == 998) {
      DEBUG_PRINTLN(F("Disconnecting from FTC"));
      HeatPump.Disconnect();
    } else if (Payload.toInt() == 997) {
      DEBUG_PRINTLN(F("Republishing Home Assistant discovery"));
      if (MQTTClient1.connected()) {
        PublishDiscoveryTopics(1, MQTT_BASETOPIC);
      }
      if (MQTTClient2.connected()) {
        PublishDiscoveryTopics(2, MQTT_2_BASETOPIC);
      }
    } else {
      HeatPump.WriteServiceCodeCMD(Payload.toInt());
      SvcRequested = Payload.toInt();
    }
  }

  // Zone 1 Temperature Setpoint Commands
  if ((Topic == MQTTCommandZone1NoModeSetpoint) ||
      (Topic == MQTTCommand2Zone1NoModeSetpoint)) {
    MQTTWriteReceived("MQTT Set Zone1 Temperature Setpoint", 6);
    HeatPump.SetZoneTempSetpoint(Payload.toFloat(),
                                 HeatPump.Status.HeatingControlModeZ1, ZONE1);
    HeatPump.Status.Zone1TemperatureSetpoint = Payload.toFloat();
  }

  // Zone 1 Flow Setpoint Commands
  if ((Topic == MQTTCommandZone1FlowSetpoint) ||
      (Topic == MQTTCommand2Zone1FlowSetpoint)) {
    MQTTWriteReceived("MQTT Set Zone1 Flow Setpoint", 6);
    HeatPump.SetFlowSetpoint(Payload.toFloat(),
                             HeatPump.Status.HeatingControlModeZ1, ZONE1);
    HeatPump.Status.Zone1FlowTemperatureSetpoint = Payload.toFloat();
  }

  // Zone 2 Temperature Setpoint Commands
  if ((Topic == MQTTCommandZone2NoModeSetpoint) ||
      (Topic == MQTTCommand2Zone2NoModeSetpoint)) {
    MQTTWriteReceived("MQTT Set Zone2 Temperature Setpoint", 6);
    HeatPump.SetZoneTempSetpoint(Payload.toFloat(),
                                 HeatPump.Status.HeatingControlModeZ2, ZONE2);
    HeatPump.Status.Zone2TemperatureSetpoint = Payload.toFloat();
  }

  // Zone 2 Flow Setpoint Commands
  if ((Topic == MQTTCommandZone2FlowSetpoint) ||
      (Topic == MQTTCommand2Zone2FlowSetpoint)) {
    MQTTWriteReceived("MQTT Set Zone2 Flow Setpoint", 6);
    HeatPump.SetFlowSetpoint(Payload.toFloat(),
                             HeatPump.Status.HeatingControlModeZ2, ZONE2);
    HeatPump.Status.Zone2FlowTemperatureSetpoint = Payload.toFloat();
  }

  // Prohibits for Server Control Mode
  if ((Topic == MQTTCommandZone1ProhibitHeating) ||
      (Topic == MQTTCommand2Zone1ProhibitHeating)) {
    MQTTWriteReceived("MQTT Zone 1 Prohibit Heating", 16);
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_HEAT_Z1_INH_Flag, Payload.toInt());
    HeatPump.Status.ProhibitHeatingZ1 = Payload.toInt();
  }

  if ((Topic == MQTTCommandZone1ProhibitCooling) ||
      (Topic == MQTTCommand2Zone1ProhibitCooling)) {
    MQTTWriteReceived("MQTT Zone 1 Prohibit Cooling", 16);
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_COOL_Z1_INH_Flag, Payload.toInt());
    HeatPump.Status.ProhibitCoolingZ1 = Payload.toInt();
  }

  if ((Topic == MQTTCommandZone2ProhibitHeating) ||
      (Topic == MQTTCommand2Zone2ProhibitHeating)) {
    MQTTWriteReceived("MQTT Zone 2 Prohibit Heating", 16);
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_HEAT_Z2_INH_Flag, Payload.toInt());
    HeatPump.Status.ProhibitHeatingZ2 = Payload.toInt();
  }

  if ((Topic == MQTTCommandZone2ProhibitCooling) ||
      (Topic == MQTTCommand2Zone2ProhibitCooling)) {
    MQTTWriteReceived("MQTT Zone 2 Prohibit Cooling", 16);
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_COOL_Z2_INH_Flag, Payload.toInt());
    HeatPump.Status.ProhibitCoolingZ2 = Payload.toInt();
  }

  if ((Topic == MQTTCommandHotwaterProhibit) ||
      (Topic == MQTTCommand2HotwaterProhibit)) {
    MQTTWriteReceived("MQTT DHW Prohibit", 16);
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_DHW_INH_Flag, Payload.toInt());
    HeatPump.Status.ProhibitDHW = Payload.toInt();
  }

  // DHW (Hot Water) Commands
  if ((Topic == MQTTCommandHotwaterMode) ||
      (Topic == MQTTCommand2HotwaterMode)) {
    MQTTWriteReceived("MQTT Set HW Mode", 15);
    HeatPump.SetDHWMode(&Payload);
  }

  if ((Topic == MQTTCommandHotwaterBoost) ||
      (Topic == MQTTCommand2HotwaterBoost)) {
    MQTTWriteReceived("MQTT Set Forced DHW Boost", 16);
    HeatPump.ForceDHW(Payload.toInt());
    HeatPump.Status.HotWaterBoostActive = Payload.toInt();
  }

  if ((Topic == MQTTCommandHotwaterSetpoint) ||
      (Topic == MQTTCommand2HotwaterSetpoint)) {
    MQTTWriteReceived("MQTT Set HW Setpoint", 6);
    HeatPump.SetHotWaterSetpoint(Payload.toFloat());
    HeatPump.Status.HotWaterSetpoint = Payload.toFloat();
  }

  if ((Topic == MQTTCommandHotwaterNormalBoost) ||
      (Topic == MQTTCommand2HotwaterNormalBoost)) {
    MQTTWriteReceived("MQTT Set Normal DHW Boost", 16);
    if (Payload.toInt() == 1) {
      PreHWBoostSvrCtrlMode =
          HeatPump.Status.SvrControlMode; // Record the Server Control Mode when
                                          // Entering Boost Only
      if (HeatPump.Status.ProhibitDHW ==
          0) { // To boost, must be at transition of On > Off, so if current
               // Prohibit Status if off first Enter SCM with Prohibit On to
               // shortly create a transition
        HeatPump.SetSvrControlMode(Payload.toInt(), Payload.toInt(),
                                   HeatPump.Status.ProhibitHeatingZ1,
                                   HeatPump.Status.ProhibitCoolingZ1,
                                   HeatPump.Status.ProhibitHeatingZ2,
                                   HeatPump.Status.ProhibitCoolingZ2);
      }
    }
    HeatPump.SetSvrControlMode(
        Payload.toInt(), 1 - Payload.toInt(), HeatPump.Status.ProhibitHeatingZ1,
        HeatPump.Status.ProhibitCoolingZ1, HeatPump.Status.ProhibitHeatingZ2,
        HeatPump.Status.ProhibitCoolingZ2);
    if (PreHWBoostSvrCtrlMode == 0) {
      HeatPump.Status.SvrControlMode = Payload.toInt();
    } // Server Control Mode is now Set to Input
    HeatPump.Status.ProhibitDHW =
        1 - Payload.toInt(); // Hot Water Boost is Inverse
    NormalHWBoostOperating =
        Payload.toInt(); // Hot Water Boost Operating is Active
  }

  // System Commands
  if ((Topic == MQTTCommandSystemHolidayMode) ||
      (Topic == MQTTCommand2SystemHolidayMode)) {
    MQTTWriteReceived("MQTT Set Holiday Mode", 16);
    HeatPump.SetHolidayMode(Payload.toInt());
    HeatPump.Status.HolidayModeActive = Payload.toInt();
  }

  if ((Topic == MQTTCommandSystemSvrMode) ||
      (Topic == MQTTCommand2SystemSvrMode)) {
    MQTTWriteReceived("MQTT Server Control Mode", 17);
    HeatPump.SetSvrControlMode(
        Payload.toInt(), HeatPump.Status.ProhibitDHW,
        HeatPump.Status.ProhibitHeatingZ1, HeatPump.Status.ProhibitCoolingZ1,
        HeatPump.Status.ProhibitHeatingZ2, HeatPump.Status.ProhibitCoolingZ2);
    HeatPump.Status.SvrControlMode = Payload.toInt();
  }

  if ((Topic == MQTTCommandSystemPower) || (Topic == MQTTCommand2SystemPower)) {
    MQTTWriteReceived("MQTT Set System Power Mode", 15);
    if (Payload == String("On")) {
      HeatPump.SetSystemPowerMode(SYSTEM_POWER_MODE_ON);
      HeatPump.Status.SystemPowerMode = SYSTEM_POWER_MODE_ON;
    } else if (Payload == String("Standby")) {
      HeatPump.SetSystemPowerMode(SYSTEM_POWER_MODE_STANDBY);
      HeatPump.Status.SystemPowerMode = SYSTEM_POWER_MODE_STANDBY;
    }
  }

  // Configuration Commands
  if ((Topic == MQTTCommandSystemUnitSize) ||
      (Topic == MQTTCommand2SystemUnitSize)) {
    MQTTWriteReceived("MQTT Set Unit Size", 15);
    unitSettings.UnitSize = Payload.toFloat();
    shouldSaveConfig =
        true; // Write the data to JSON file so if device reboots it is saved
    if (Flags::CascadeActive()) {
      cascadeNetwork.setLocalUnitCapacity(unitSettings.UnitSize);
    }
  }

  if ((Topic == MQTTCommandSystemGlycol) ||
      (Topic == MQTTCommand2SystemGlycol)) {
    MQTTWriteReceived("MQTT Set Glycol Strength", 15);
    if (Payload == String("0%")) {
      unitSettings.GlycolStrength = 4.18;
    } else if (Payload == String("10%")) {
      unitSettings.GlycolStrength = 4.12;
    } else if (Payload == String("20%")) {
      unitSettings.GlycolStrength = 4.07;
    } else if (Payload == String("30%")) {
      unitSettings.GlycolStrength = 3.9;
    }
    shouldSaveConfig =
        true; // Write the data to JSON file so if device reboots it is saved
  }

  // Zone Heating Mode Commands
  if ((Topic == MQTTCommandZone1HeatingMode) ||
      (Topic == MQTTCommand2Zone1HeatingMode)) {
    MQTTWriteReceived("MQTT Set Heating Mode Zone 1", 4);
    if (Payload == String("Heating Temperature")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_ZONE_TEMP,
                                     SET_HEATING_CONTROL_MODE_Z1);
      HeatPump.Status.HeatingControlModeZ1 = HEATING_CONTROL_MODE_ZONE_TEMP;
    } else if (Payload == String("Heating Flow")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_FLOW_TEMP,
                                     SET_HEATING_CONTROL_MODE_Z1);
      HeatPump.Status.HeatingControlModeZ1 = HEATING_CONTROL_MODE_FLOW_TEMP;
    } else if (Payload == String("Heating Compensation")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_COMPENSATION,
                                     SET_HEATING_CONTROL_MODE_Z1);
      HeatPump.Status.HeatingControlModeZ1 = HEATING_CONTROL_MODE_COMPENSATION;
    } else if (Payload == String("Cooling Temperature")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_COOL_ZONE_TEMP,
                                     SET_HEATING_CONTROL_MODE_Z1);
      HeatPump.Status.HeatingControlModeZ1 =
          HEATING_CONTROL_MODE_COOL_ZONE_TEMP;
    } else if (Payload == String("Cooling Flow")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_COOL_FLOW_TEMP,
                                     SET_HEATING_CONTROL_MODE_Z1);
      HeatPump.Status.HeatingControlModeZ1 =
          HEATING_CONTROL_MODE_COOL_FLOW_TEMP;
    } else if (Payload == String("Dry Up")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_DRY_UP,
                                     SET_HEATING_CONTROL_MODE_Z1);
      HeatPump.Status.HeatingControlModeZ1 = HEATING_CONTROL_MODE_DRY_UP;
    }
  }

  if ((Topic == MQTTCommandZone2HeatingMode) ||
      (Topic == MQTTCommand2Zone2HeatingMode)) {
    MQTTWriteReceived("MQTT Set Heating Mode Zone 2", 4);
    if (Payload == String("Heating Temperature")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_ZONE_TEMP,
                                     SET_HEATING_CONTROL_MODE_Z2);
      HeatPump.Status.HeatingControlModeZ2 = HEATING_CONTROL_MODE_ZONE_TEMP;
    } else if (Payload == String("Heating Flow")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_FLOW_TEMP,
                                     SET_HEATING_CONTROL_MODE_Z2);
      HeatPump.Status.HeatingControlModeZ2 = HEATING_CONTROL_MODE_FLOW_TEMP;
    } else if (Payload == String("Heating Compensation")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_COMPENSATION,
                                     SET_HEATING_CONTROL_MODE_Z2);
      HeatPump.Status.HeatingControlModeZ2 = HEATING_CONTROL_MODE_COMPENSATION;
    } else if (Payload == String("Cooling Temperature")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_COOL_ZONE_TEMP,
                                     SET_HEATING_CONTROL_MODE_Z2);
      HeatPump.Status.HeatingControlModeZ2 =
          HEATING_CONTROL_MODE_COOL_ZONE_TEMP;
    } else if (Payload == String("Cooling Flow")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_COOL_FLOW_TEMP,
                                     SET_HEATING_CONTROL_MODE_Z2);
      HeatPump.Status.HeatingControlModeZ2 =
          HEATING_CONTROL_MODE_COOL_FLOW_TEMP;
    } else if (Payload == String("Dry Up")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_DRY_UP,
                                     SET_HEATING_CONTROL_MODE_Z2);
      HeatPump.Status.HeatingControlModeZ2 = HEATING_CONTROL_MODE_DRY_UP;
    }
  }

  // Compensation Curve Commands (Complex JSON handling)
  if ((Topic == MQTTCommandSystemCompCurve) ||
      (Topic == MQTTCommand2SystemCompCurve)) {
    MQTTWriteReceived("MQTT Set Comp Curve", 15);
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, Payload);
    if (error) {
      DEBUG_PRINT("1 Failed to read: ");
      DEBUG_PRINTLN(error.c_str());
      if (Payload == String("ERASE")) {
        DEBUG_PRINTLN("Erasing Comp Curve");
        unitSettings.CompCurve = "{}";
        shouldSaveConfig = true;
      } // Method to erase the onboard document for recovery
    } else {
      // Method is to check if JSON key exists, then if not NULL then read it -
      // this allows for some variables to be posted in JSON but not others
      // depending on the request type Base Curve (String)
      JsonVariant baseVariant = doc["base"];
      if (!baseVariant.isNull()) { // Check if the key exists AND if its value
                                   // is not explicitly 'null'
        JsonDocument local_stored_doc; // Variable for the locally decoded JSON
        DeserializationError error = deserializeJson(
            local_stored_doc,
            unitSettings.CompCurve); // Unpack the local stored JSON document
        if (error) {
          DEBUG_PRINT("2 Failed to read: ");
          DEBUG_PRINTLN(error.c_str());
        } else {
          local_stored_doc["base"] =
              baseVariant; // Load the new Base into the correct area of the
                           // locally stored file
          local_stored_doc.shrinkToFit();
          serializeJson(local_stored_doc,
                        unitSettings.CompCurve); // Repack the JSON
          shouldSaveConfig = true; // Write the data to onboard JSON file so if
                                   // device reboots it is saved
        }
      }
      // Activation Of Mode per Zone (Bool)
      const char *error = doc["zone1"]["active"];
      if (error) {
        bool wc_z1_active = doc["zone1"]["active"];
        if (!unitSettings.z1_active &&
            wc_z1_active) { // On transition from Inactive > Active
          if (HeatPump.Status.HeatingControlModeZ1 !=
              1) { // Check if not already in Fixed Flow Mode
            HeatPump.SetHeatingControlMode(
                HEATING_CONTROL_MODE_FLOW_TEMP,
                SET_HEATING_CONTROL_MODE_Z1); // Swap to Fixed Flow for Onboard
                                              // WC to input the flow
                                              // temperature
            HeatPump.Status.HeatingControlModeZ1 =
                HEATING_CONTROL_MODE_FLOW_TEMP;
          }
        }
        ModifyCompCurveState(1, wc_z1_active); // State Save
      }
      error = doc["zone2"]["active"];
      if (error) {
        bool wc_z2_active = doc["zone2"]["active"];
        if (!unitSettings.z2_active &&
            wc_z2_active) { // On transition from Inactive > Active
          if (HeatPump.Status.HeatingControlModeZ2 !=
              1) { // Check if not already in Fixed Flow Mode
            HeatPump.SetHeatingControlMode(
                HEATING_CONTROL_MODE_FLOW_TEMP,
                SET_HEATING_CONTROL_MODE_Z2); // Swap to Fixed Flow for Onboard
                                              // WC to input the flow
                                              // temperature
            HeatPump.Status.HeatingControlModeZ2 =
                HEATING_CONTROL_MODE_FLOW_TEMP;
          }
        }
        ModifyCompCurveState(2, wc_z2_active); // State Save
      }
      // Local or Remote Outdoor Temperature Measurement (Bool)
      error = doc["use_local_outdoor"];
      if (error) {
        unitSettings.use_local_outdoor = doc["use_local_outdoor"];
      }
      // Adjustments Pre or Post WC Calculation (Float)
      float z1_manual_offset = doc["zone1"]["manual_offset"]; //
      if (z1_manual_offset) {
        unitSettings.z1_manual_offset = z1_manual_offset;
      } // Post Calcuation Zone1 Manual +/- Offset
      float z1_temp_offset = doc["zone1"]["temp_offset"]; //
      if (z1_temp_offset) {
        unitSettings.z1_temp_offset = z1_temp_offset;
      } // Post Calcuation Zone1 Temperature (e.g. Solar Gain) +/- Offset
      float z1_wind_offset = doc["zone1"]["wind_offset"]; //
      if (z1_wind_offset) {
        unitSettings.z1_wind_offset = z1_wind_offset;
      } // Post Calcuation Zone1 Wind Factor +/- Offset
      float z2_manual_offset = doc["zone2"]["manual_offset"]; //
      if (z2_manual_offset)
        unitSettings.z2_manual_offset =
            z2_manual_offset; // Post Calcuation Zone2 Manual +/- Offset
      float z2_temp_offset = doc["zone2"]["temp_offset"]; //
      if (z2_temp_offset)
        unitSettings.z2_temp_offset =
            z2_temp_offset; // Post Calcuation Zone2 Temperature (e.g. Solar
                            // Gain) +/- Offset
      float z2_wind_offset = doc["zone2"]["wind_offset"]; //
      if (z2_wind_offset)
        unitSettings.z2_wind_offset =
            z2_wind_offset; // Post Calcuation Zone2 Wind Factor +/- Offset
      float cloud_outdoor = doc["cloud_outdoor"]; //
      if (cloud_outdoor)
        unitSettings.cloud_outdoor =
            cloud_outdoor;  // Temperature Provided by a remote or cloud source
                            // when use_local_outdoor = False
      CalculateCompCurve(); // Recalculate after modification
    }
  }
}

// Utility functions
float roundToHalfDecimal(float value) { return round(value * 2.0) / 2.0; }

double round2(double value) { return (int)(value * 100 + 0.5) / 100.0; }

String decimalToBinary(int decimal) {
  String binary = "";
  for (int i = 0; i < 8; i++) { // 8 bits for a byte
    binary += (decimal >> i) & 1 ? '1' : '0';
  }
  return binary;
}

void FlashGreenLED(void) {
#ifdef ARDUINO_M5STACK_ATOMS3 // Define the M5Stack LED
  myLED.setPixel(0, L_GREEN, 1);
  myLED.brightness(255, 1);
#endif
#ifdef ESP8266                       // Define the Witty ESP8266 Ports
  digitalWrite(Green_RGB_LED, HIGH); // Flash the Green LED full brightness
#endif
  delay(10);   // Hold for 10ms then WiFi brightness will return it to 25%
#ifdef ESP8266 // Define the Witty ESP8266 Ports
  analogWrite(Green_RGB_LED, 30); // Green LED on, 25% brightness
#endif
#ifdef ARDUINO_M5STACK_ATOMS3 // Define the M5Stack LED
  myLED.brightness(LED_BRIGHT, 1);
#endif
}

void MQTTWriteReceived(String message, int MsgNumber) {
  DEBUG_PRINTLN(message);
  WriteInProgress = true;
}

void ModifyCompCurveState(int Zone, bool Active) {
  // Save the state
  JsonDocument local_stored_doc;
  DeserializationError error =
      deserializeJson(local_stored_doc, unitSettings.CompCurve);
  if (error) {
    DEBUG_PRINT("Failed to parse CompCurve JSON: ");
    DEBUG_PRINTLN(error.c_str());
    return;
  }

  if (Zone == 1) {
    local_stored_doc["zone1"]["active"] = Active;
    unitSettings.z1_active = Active;
  } else if (Zone == 2) {
    local_stored_doc["zone2"]["active"] = Active;
    unitSettings.z2_active = Active;
  }

  serializeJson(local_stored_doc, unitSettings.CompCurve);
  shouldSaveConfig = true;
}

void syncCurrentTime() {
  // Update the ESP clock from the FTC
  time_t epochTime = mktime(&HeatPump.Status.DateTimeStamp); // Convert to epoch

  if (epochTime != (time_t)(-1)) {
    struct timeval tv;
    tv.tv_sec = epochTime;
    tv.tv_usec = 0;
    if (settimeofday(&tv, nullptr) == 0) {
      DEBUG_PRINTLN("Time set successfully from FTC");
    } else {
      DEBUG_PRINTLN("Error setting time from FTC");
    }
  } else {
    DEBUG_PRINTLN("Error converting time from");
  }

  HeatPump.Status.SyncTime = false;
  return;
}

void printCurrentTime() {
  time_t now;
  struct tm timeinfo;
  char TimeBuffer[32];

  time(&now);
  localtime_r(&now, &timeinfo);

  strftime(TimeBuffer, sizeof(TimeBuffer), "%F%T -> ", &timeinfo);
  DEBUG_PRINT(TimeBuffer);
}

// Telnet functions
void setupTelnet() {
  TelnetServer.onConnect(onTelnetConnect);
  TelnetServer.onConnectionAttempt(onTelnetConnectionAttempt);
  TelnetServer.onReconnect(onTelnetReconnect);
  TelnetServer.onDisconnect(onTelnetDisconnect);
}

void startTelnet() {
  DEBUG_PRINT(F("Telnet: "));
#ifdef ARDUINO_WT32_ETH01
  if (TelnetServer.begin(23, false)) {
#else
  if (TelnetServer.begin()) {
#endif
    DEBUG_PRINTLN(F("Telnet Running"));
  } else {
    DEBUG_PRINTLN(F("Telnet Error"));
  }
}

void stopTelnet() {
  DEBUG_PRINTLN(F("Stopping Telnet"));
  TelnetServer.stop();
}

void onTelnetConnect(String ip) {
  DEBUG_PRINT(F("Telnet: "));
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(F(" connected"));
  TelnetServer.println("\nWelcome " + TelnetServer.getIP());
  TelnetServer.println(F("(Use ^] + q  to disconnect.)"));
}

void onTelnetDisconnect(String ip) {
  DEBUG_PRINT(F("Telnet: "));
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(F(" disconnected"));
}

void onTelnetReconnect(String ip) {
  DEBUG_PRINT(F("Telnet: "));
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(F(" reconnected"));
}

void onTelnetConnectionAttempt(String ip) {
  DEBUG_PRINT(F("Telnet: "));
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(F(" tried to connected"));
}

// Report Functions Implementation
void StatusReport(void) {
  JsonDocument doc;
  char Buffer[1024];
  char TmBuffer[32];

  doc[F("SSID")] = WiFi.SSID();
  doc[F("RSSI")] = WiFi.RSSI();
#ifdef ARDUINO_WT32_ETH01
  doc[F("IP")] = ETH.localIP().toString();
#else
  doc[F("IP")] = WiFi.localIP().toString();
#endif
  doc[F("Firmware")] = FirmwareVersion;
#ifdef ESP32 // Define the M5Stack LED
  doc[F("CPUTemp")] = round2(temperatureRead());
#endif
#ifdef ESP8266 // Define the M5Stack LED
  doc[F("CPUTemp")] = "None";
#endif
  doc[F("CPULoopTime")] = CPULoopSpeed;
  doc[F("FTCLoopTime")] = FTCLoopSpeed;
  doc[F("FTCReplyTime")] = HeatPump.Lastmsbetweenmsg();
  doc[F("FTCVersion")] = FTCString[HeatPump.Status.FTCVersion];
  doc[F("FTCSoftwareVersion")] = HeatPump.Status.FTCSoftware;

  strftime(TmBuffer, sizeof(TmBuffer), "%FT%TZ",
           &HeatPump.Status.DateTimeStamp);
  doc[F("FTCTime")] = TmBuffer;
  if (!Flags::CascadeSlave()) {
    doc[F("UnitSize")] = String(unitSettings.UnitSize, 1);
  }

  if (round2(unitSettings.GlycolStrength) == 4.18) {
    doc[F("Glycol")] = "0%";
  } else if (round2(unitSettings.GlycolStrength) == 4.12) {
    doc[F("Glycol")] = "10%";
  } else if (round2(unitSettings.GlycolStrength) == 4.07) {
    doc[F("Glycol")] = "20%";
  } else if (round2(unitSettings.GlycolStrength) == 3.9) {
    doc[F("Glycol")] = "30%";
  }

  doc[F("HB_ID")] = Heart_Value;

  serializeJson(doc, Buffer);
  MQTTClient1.publish(MQTT_STATUS_WIFISTATUS.c_str(), Buffer, false);
  MQTTClient2.publish(MQTT_2_STATUS_WIFISTATUS.c_str(), Buffer, false);
  MQTTClient1.publish(MQTT_LWT.c_str(), "online");
  MQTTClient2.publish(MQTT_2_LWT.c_str(), "online");
}

void SystemReport(void) {
  JsonDocument doc;
  char Buffer[1024];
  double EstInputPower = 0;
  double OutputPower = 0;
  float HeatOutputPower = 0;
  float HeatingOutputPower = 0;
  float DHWOutputPower = 0;
  float CoolOutputPower = 0;
  float EstCoolingInputPower = 0;
  float EstHeatingInputPower = 0;
  float EstDHWInputPower = 0;
  float Min_Input_Power = 0;
  float Max_Input_Power = 0;
  bool DHW_Mode = false;
  bool Non_HP_Mode = false;
  float UnitSizeFactor, Instant_CoP;

  // Unit Size Factoring
  if (unitSettings.UnitSize == 4.0) {
    UnitSizeFactor = 0.4;
  } else if (unitSettings.UnitSize == 5.0) {
    UnitSizeFactor = 0.6;
  } else if (unitSettings.UnitSize == 7.5 || unitSettings.UnitSize == 6.0) {
    UnitSizeFactor = 0.95;
  } else if (unitSettings.UnitSize == 8.5 || unitSettings.UnitSize == 11.2) {
    UnitSizeFactor = 1.1;
  } else if (unitSettings.UnitSize == 8.0) {
    UnitSizeFactor = 1.3;
  } else if (unitSettings.UnitSize == 10.0) {
    UnitSizeFactor = 1.5;
  } else if ((unitSettings.UnitSize == 12.0) ||
             (unitSettings.UnitSize == 14.0)) {
    UnitSizeFactor = 1.7;
  }

  if (HeatPump.Status.InputPower <
      2) { // To account for FTC's onboard estimation and limit the input power
           // range
    Max_Input_Power = 2;
  } else {
    Min_Input_Power = HeatPump.Status.InputPower;
    Max_Input_Power = HeatPump.Status.InputPower + 1;
  }

  float x = ((((((float)HeatPump.Status.CompressorFrequency * 2) *
                ((float)HeatPump.Status.HeaterOutputFlowTemperature * 0.8)) /
               1000) /
              2) *
             UnitSizeFactor);
  EstInputPower =
      ((x - Min_Input_Power) * (Max_Input_Power - Min_Input_Power) /
           (Max_Input_Power - Min_Input_Power) +
       Min_Input_Power); // Constrain Input Power to FTC Onboard Reading range
  OutputPower =
      (((float)HeatPump.Status.PrimaryFlowRate / 60) *
       (float)HeatPump.Status.HeaterDeltaT *
       unitSettings.GlycolStrength); // Approx Heat Capacity of Fluid in Use

  if (HeatPump.Status.ThreeWayValve == 1 ||
      HeatPump.Status.SystemOperationMode == 1 ||
      HeatPump.Status.SystemOperationMode == 6) {
    DHW_Mode = true;
  }

  if (HeatPump.Status.ImmersionActive == 1 ||
      HeatPump.Status.Booster1Active == 1 ||
      HeatPump.Status.Booster2Active ==
          1) { // Account for Immersion or Booster Instead of HP
    Non_HP_Mode = true;
    if (EstInputPower == 0) {
      EstInputPower = HeatPump.Status.InputPower;
    } // Uses Booster/Immersion Size in MRC
    if (OutputPower == 0) {
      HeatOutputPower = HeatPump.Status.OutputPower;
    }
  }

  if (HeatPump.Status.SystemOperationMode > 0) { // Pump Operating
    if (OutputPower < 0) {                       // Cooling or Defrosting Mode
      if (HeatPump.Status.Defrost != 0) {        // If Defrosting Mode
        EstHeatingInputPower =
            EstInputPower; // Input Power attributed to Heating & Cooling
        HeatingOutputPower = HeatOutputPower =
            OutputPower; // Heating is Negative (Extracting heat to defrost)
      } //
      EstCoolingInputPower = EstInputPower; //
      CoolOutputPower = fabsf(OutputPower); // Make Positive Cooling
    } else if (OutputPower > 0) {           // Heating by HP
      if (DHW_Mode) {                       // DHW Operation Mode via HP
        EstDHWInputPower = EstInputPower;   //
        DHWOutputPower = HeatOutputPower = OutputPower; //
      } else {                                // Heating Operation Mode via HP
        EstHeatingInputPower = EstInputPower; //
        HeatingOutputPower = HeatOutputPower = OutputPower; //
      } // Heating Modes
    } else if (OutputPower == 0 && Non_HP_Mode) {       // Boosters or Immersion
      if (DHW_Mode) {                                   // DHW Operation Mode
        EstDHWInputPower = EstInputPower;               //
        DHWOutputPower = OutputPower = HeatOutputPower; //
      } else {                                          // Heating Modes
        EstHeatingInputPower = EstInputPower;           //
        HeatingOutputPower = OutputPower = HeatOutputPower; //
      } //
    } //
  }

  // Instant CoP measurement from computed estimates
  if (fabsf(OutputPower) > 0 && EstInputPower > 0) {
    Instant_CoP = fabsf(OutputPower) / EstInputPower;
  } else {
    Instant_CoP = 0;
  }

  doc[F("HeaterFlow")] = HeatPump.Status.HeaterOutputFlowTemperature;
  doc[F("HeaterReturn")] = HeatPump.Status.HeaterReturnFlowTemperature;
  doc[F("FlowReturnDeltaT")] = HeatPump.Status.HeaterDeltaT;
  doc[F("OutsideTemp")] = HeatPump.Status.OutsideTemperature;
  doc[F("Defrost")] = DefrostModeString[HeatPump.Status.Defrost];
  if (!Flags::CascadeActive()) {
    doc[F("InputPower")] = HeatPump.Status.InputPower;
    doc[F("HeaterPower")] = HeatPump.Status.OutputPower;
  }
  if (!Flags::CascadeActive()) {
    doc[F("EstInputPower")] = round2(EstInputPower);
    doc[F("EstHeatingInputPower")] = round2(EstHeatingInputPower);
    doc[F("EstDHWInputPower")] = round2(EstDHWInputPower);
    doc[F("EstHeatOutputPower")] = round2(HeatOutputPower);
    doc[F("EstHeatingOutputPower")] = round2(HeatingOutputPower);
    doc[F("EstDHWOutputPower")] = round2(DHWOutputPower);
    if (Flags::HasCooling()) {
      doc[F("EstCoolingInputPower")] = round2(EstCoolingInputPower);
      doc[F("EstCoolOutputPower")] = round2(CoolOutputPower);
    }
  }
  if (!Flags::CascadeActive()) {
    doc[F("Instant_CoP")] = round2(Instant_CoP);
  }
  // In cascade master mode, suppress Compressor, FlowRate, RunHours
  if (!Flags::CascadeMaster()) {
    doc[F("Compressor")] = HeatPump.Status.CompressorFrequency;
    doc[F("FlowRate")] = HeatPump.Status.PrimaryFlowRate;
    doc[F("RunHours")] = HeatPump.Status.RunHours;
  }
  // Cascade configuration flags for HA sensors
  doc[F("CascadeNodeId")] = mqttSettings.cascadeNodeId;
  // Cascade mode status for HA sensor
  if (!Flags::CascadeEnabled()) {
    doc[F("CascadeMode")] = "Disabled";
  } else if (Flags::CascadeMaster()) {
    doc[F("CascadeMode")] = "Master";
  } else if (Flags::CascadeSlave()) {
    doc[F("CascadeMode")] = "Slave";
  }
  doc[F("SystemPower")] =
      SystemPowerModeString[HeatPump.Status.SystemPowerMode];
  if (HeatPump.Status.Defrost == 2) {
    doc[F("SystemOperationMode")] = "Defrosting";
  } else {
    doc[F("SystemOperationMode")] =
        SystemOperationModeString[HeatPump.Status.SystemOperationMode];
  }
  doc[F("HolidayMode")] = HeatPump.Status.HolidayModeActive;
  doc[F("HB_ID")] = Heart_Value;
  serializeJson(doc, Buffer);
  MQTTClient1.publish(MQTT_STATUS_SYSTEM.c_str(), Buffer, false);
  MQTTClient2.publish(MQTT_2_STATUS_SYSTEM.c_str(), Buffer, false);
}

void Zone1Report(void) {
  JsonDocument doc;
  char Buffer[512];
  doc[F("Temperature")] = HeatPump.Status.Zone1Temperature;
  doc[F("Setpoint")] = HeatPump.Status.Zone1TemperatureSetpoint;
  doc[F("HeatingControlMode")] =
      HeatingControlModeString[HeatPump.Status.HeatingControlModeZ1];
  doc[F("FSP")] = round2(HeatPump.Status.Zone1FlowTemperatureSetpoint);
  if ((HeatPump.Status.Zone2Temperature == 0) &&
      (HeatPump.Status.SystemOperationMode == 2 ||
       HeatPump.Status.SystemOperationMode == 3 ||
       HeatPump.Status.SystemOperationMode == 7)) {
    doc[F("TwoZone_Z1Working")] = 1;
  } else {
    doc[F("TwoZone_Z1Working")] = HeatPump.Status.TwoZone_Z1Working;
  }
  if (!Flags::CascadeSlave()) {
    doc[F("ProhibitHeating")] = HeatPump.Status.ProhibitHeatingZ1;
    if (Flags::HasCooling()) {
      doc[F("ProhibitCooling")] = HeatPump.Status.ProhibitCoolingZ1;
    }
  }
  doc[F("FlowTemp")] = HeatPump.Status.Zone1FlowTemperature;
  doc[F("ReturnTemp")] = HeatPump.Status.Zone1ReturnTemperature;
  doc[F("InputType")] = ThermostatString[HeatPump.Status.ThermostatZ1];
  doc[F("HB_ID")] = Heart_Value;
  serializeJson(doc, Buffer);
  MQTTClient1.publish(MQTT_STATUS_ZONE1.c_str(), Buffer, false);
  MQTTClient2.publish(MQTT_2_STATUS_ZONE1.c_str(), Buffer, false);
}

void Zone2Report(void) {
  JsonDocument doc;
  char Buffer[512];
  doc[F("Temperature")] = HeatPump.Status.Zone2Temperature;
  doc[F("Setpoint")] = HeatPump.Status.Zone2TemperatureSetpoint;
  doc[F("HeatingControlMode")] =
      HeatingControlModeString[HeatPump.Status.HeatingControlModeZ2];
  doc[F("FSP")] = round2(HeatPump.Status.Zone2FlowTemperatureSetpoint);
  doc[F("TwoZone_Z2Working")] = HeatPump.Status.TwoZone_Z2Working;
  if (!Flags::CascadeSlave()) {
    doc[F("ProhibitHeating")] = HeatPump.Status.ProhibitHeatingZ2;
    if (Flags::HasCooling()) {
      doc[F("ProhibitCooling")] = HeatPump.Status.ProhibitCoolingZ2;
    }
  }
  doc[F("FlowTemp")] = HeatPump.Status.Zone2FlowTemperature;
  doc[F("ReturnTemp")] = HeatPump.Status.Zone2ReturnTemperature;
  doc[F("InputType")] = ThermostatString[HeatPump.Status.ThermostatZ2];
  doc[F("HB_ID")] = Heart_Value;
  serializeJson(doc, Buffer);
  MQTTClient1.publish(MQTT_STATUS_ZONE2.c_str(), Buffer, false);
  MQTTClient2.publish(MQTT_2_STATUS_ZONE2.c_str(), Buffer, false);
}

void HotWaterReport(void) {
  JsonDocument doc;
  char Buffer[1024];
  if (!Flags::CascadeSlave()) {
    doc[F("Temperature")] = HeatPump.Status.HotWaterTemperature;
    doc[F("TempTHW5A")] = HeatPump.Status.HotWaterTemperatureTHW5A;
    doc[F("Setpoint")] = HeatPump.Status.HotWaterSetpoint;
  }
  doc[F("HotWaterBoostActive")] = HeatPump.Status.HotWaterBoostActive;
  doc[F("HotWaterEcoBoostActive")] = NormalHWBoostOperating;
  if (!Flags::CascadeSlave()) {
    doc[F("ProhibitDHW")] = HeatPump.Status.ProhibitDHW;
  }
  doc[F("DHWActive")] = HeatPump.Status.DHWActive;
  if (!Flags::CascadeSlave()) {
    doc[F("HotWaterControlMode")] =
        HotWaterControlModeString[HeatPump.Status.HotWaterControlMode];
    doc[F("LegionellaSetpoint")] = HeatPump.Status.LegionellaSetpoint;
    doc[F("HotWaterMaxTDrop")] = HeatPump.Status.HotWaterMaximumTempDrop;
  }
  doc[F("HotWaterPhase")] = DHWPhaseString[HeatPump.Status.DHWHeatSourcePhase];
  doc[F("HB_ID")] = Heart_Value;
  serializeJson(doc, Buffer);
  MQTTClient1.publish(MQTT_STATUS_HOTWATER.c_str(), Buffer, false);
  MQTTClient2.publish(MQTT_2_STATUS_HOTWATER.c_str(), Buffer, false);
}

void ConfigurationReport(void) {
  JsonDocument doc;
  char Buffer[2048];
  doc[F("DipSw1")] = decimalToBinary(HeatPump.Status.DipSwitch1);
  doc[F("DipSw2")] = decimalToBinary(HeatPump.Status.DipSwitch2);
  doc[F("DipSw3")] = decimalToBinary(HeatPump.Status.DipSwitch3);
  doc[F("DipSw4")] = decimalToBinary(HeatPump.Status.DipSwitch4);
  doc[F("DipSw5")] = decimalToBinary(HeatPump.Status.DipSwitch5);
  doc[F("DipSw6")] = decimalToBinary(HeatPump.Status.DipSwitch6);
  doc[F("HasCooling")] = HeatPump.Status.HasCooling;
  doc[F("Has2Zone")] = HeatPump.Status.Has2Zone;
  doc[F("HasSimple2Zone")] = HeatPump.Status.Simple2Zone;
  doc[F("RefrigerantType")] = HeatPump.Status.RefrigerantType;
  // Publish only when available
  if (!Flags::CascadeActive()) {
    if (HeatPump.SVCPopulated || HeatPump.Status.CompOpTimes != 0) {
      doc[F("CompOpTimes")] = HeatPump.Status.CompOpTimes;
    }
  }
  if (HeatPump.SVCPopulated || HeatPump.Status.LiquidTemp != 0) {
    doc[F("LiquidTemp")] = HeatPump.Status.LiquidTemp;
  }
  if (HeatPump.SVCPopulated || HeatPump.Status.TH4Discharge != 0) {
    doc[F("TH4Discharge")] = HeatPump.Status.TH4Discharge;
  }
  if (HeatPump.SVCPopulated || HeatPump.Status.Superheat != 0) {
    doc[F("Superheat")] = HeatPump.Status.Superheat;
  }
  if (HeatPump.SVCPopulated || HeatPump.Status.Subcool != 0) {
    doc[F("Subcool")] = HeatPump.Status.Subcool;
  }
  if (HeatPump.SVCPopulated || HeatPump.Status.TH8HeatSink != 0) {
    doc[F("TH8HeatSink")] = HeatPump.Status.TH8HeatSink;
  }
  if (HeatPump.SVCPopulated || HeatPump.Status.TH6Pipe != 0) {
    doc[F("TH6Pipe")] = HeatPump.Status.TH6Pipe;
  }
  if (HeatPump.SVCPopulated || HeatPump.Status.TH32Pipe != 0) {
    doc[F("TH32Pipe")] = HeatPump.Status.TH32Pipe;
  }
  if (!Flags::CascadeMaster()) {
    if (HeatPump.SVCPopulated || HeatPump.Status.Fan1RPM != 0) {
      doc[F("Fan1RPM")] = HeatPump.Status.Fan1RPM;
    }
    if (HeatPump.SVCPopulated || HeatPump.Status.Fan2RPM != 0) {
      doc[F("Fan2RPM")] = HeatPump.Status.Fan2RPM;
    }
  }
  if (HeatPump.SVCPopulated || HeatPump.Status.LEVA != 0) {
    doc[F("LEVA")] = HeatPump.Status.LEVA;
  }
  if (HeatPump.SVCPopulated || HeatPump.Status.LEVB != 0) {
    doc[F("LEVB")] = HeatPump.Status.LEVB;
  }
  if (HeatPump.SVCPopulated || HeatPump.Status.TH33 != 0) {
    doc[F("TH33")] = HeatPump.Status.TH33;
  }
  doc[F("HB_ID")] = Heart_Value;
  serializeJson(doc, Buffer);
  MQTTClient1.publish(MQTT_STATUS_CONFIGURATION.c_str(), Buffer, false);
  MQTTClient2.publish(MQTT_2_STATUS_CONFIGURATION.c_str(), Buffer, false);
}

void AdvancedReport(void) {
  JsonDocument doc;
  char Buffer[1024];
  if (!Flags::CascadeSlave()) {
    doc[F("FlowTMax")] = HeatPump.Status.FlowTempMax;
    doc[F("FlowTMin")] = HeatPump.Status.FlowTempMin;
    doc[F("BoilerFlow")] = HeatPump.Status.ExternalBoilerFlowTemperature;
    doc[F("BoilerReturn")] = HeatPump.Status.ExternalBoilerReturnTemperature;
    doc[F("Immersion")] = OFF_ON_String[HeatPump.Status.ImmersionActive];
    doc[F("Booster")] = OFF_ON_String[HeatPump.Status.Booster1Active];
    doc[F("Booster2")] = OFF_ON_String[HeatPump.Status.Booster2Active];
  }
  doc[F("MixingTemp")] = HeatPump.Status.MixingTemperature;
  doc[F("MixingStep")] = HeatPump.Status.MixingStep;
  doc[F("ThreeWayValve")] = HeatPump.Status.ThreeWayValve;
  doc[F("PrimaryWaterPump")] = OFF_ON_String[HeatPump.Status.PrimaryWaterPump];
  doc[F("RefrigeTemp")] = HeatPump.Status.RefrigeTemp;
  doc[F("CondensingTemp")] = HeatPump.Status.CondensingTemp;
  doc[F("HeatingActive")] =
      HeatingRunningBinary[HeatPump.Status.SystemOperationMode];
  if (Flags::HasCooling()) {
    doc[F("CoolingActive")] =
        CoolingRunningBinary[HeatPump.Status.SystemOperationMode];
  }
  doc[F("HB_ID")] = Heart_Value;
  serializeJson(doc, Buffer);
  MQTTClient1.publish(MQTT_STATUS_ADVANCED.c_str(), Buffer, false);
  MQTTClient2.publish(MQTT_2_STATUS_ADVANCED.c_str(), Buffer, false);
}

void AdvancedTwoReport(void) {
  JsonDocument doc;
  char Buffer[1024];
  int ErrorCode = ((String(HeatPump.Status.ErrCode1, HEX)).toInt() * 100) +
                  (String(HeatPump.Status.ErrCode2, HEX)).toInt();

  doc[F("SvrControlMode")] = HeatPump.Status.SvrControlMode;
  doc[F("WaterPump2")] = OFF_ON_String[HeatPump.Status.WaterPump2];
  doc[F("WaterPump4")] = OFF_ON_String[HeatPump.Status.WaterPump4];
  if (!HeatPump.Status.Simple2Zone) {
    doc[F("WaterPump3")] = OFF_ON_String[HeatPump.Status.WaterPump3a];
  } else {
    doc[F("WaterPump3")] = OFF_ON_String[HeatPump.Status.WaterPump3b];
  }
  doc[F("WaterPump13")] = OFF_ON_String[HeatPump.Status.WaterPump13];
  doc[F("ThreeWayValve2")] = HeatPump.Status.ThreeWayValve2;
  doc[F("RefrigeFltCode")] =
      RefrigeFltCodeString[HeatPump.Status.RefrigeFltCode];
  if (ErrorCode == 8000 || ErrorCode == 0) {
    doc[F("ErrCode")] = String("Normal");
  } else {
    doc[F("ErrCode")] = ErrorCode;
  }
  String FltCodeString = String(FltCodeLetterOne[HeatPump.Status.FltCode1]) +
                         String(FltCodeLetterTwo[HeatPump.Status.FltCode2]);
  if (FltCodeString == "A0") {
    doc[F("FltCode")] = String("Normal");
  } else {
    doc[F("FltCode")] = String(FltCodeString);
  }
  doc[F("Z1TstatDemand")] =
      OFF_ON_String[HeatPump.Status.Zone1ThermostatDemand];
  if (Flags::Has2Zone()) {
    doc[F("Z2TstatDemand")] =
        OFF_ON_String[HeatPump.Status.Zone2ThermostatDemand];
  }
  doc[F("OTstatDemand")] =
      OFF_ON_String[HeatPump.Status.OutdoorThermostatDemand];
  doc[F("OpMode")] = HPControlModeString[HeatPump.Status.HeatCool];
  if (SvcRequested == HeatPump.Status.LastServiceCodeNumber) {
    SvcReply = HeatPump.Status.ServiceCodeReply;
  }
  doc[F("LastSvc")] = SvcRequested;
  doc[F("LastSvcReply")] = SvcReply;
  doc[F("HB_ID")] = Heart_Value;
  serializeJson(doc, Buffer);
  MQTTClient1.publish(MQTT_STATUS_ADVANCED_TWO.c_str(), Buffer, false);
  MQTTClient2.publish(MQTT_2_STATUS_ADVANCED_TWO.c_str(), Buffer, false);
}

void EnergyReport(void) {
  JsonDocument doc;
  char Buffer[1024];
  float heat_cop, cool_cop, dhw_cop, ctotal, dtotal, total_cop;

  // A check for errors before calculating CoP
  if ((HeatPump.Status.DeliveredHeatingEnergy == 0) &&
      (HeatPump.Status.ConsumedHeatingEnergy > 0)) {
    HeatPump.Status.ConsumedHeatingEnergy = 0; // Re-write
  }
  if ((HeatPump.Status.DeliveredCoolingEnergy == 0) &&
      (HeatPump.Status.ConsumedCoolingEnergy > 0)) {
    HeatPump.Status.ConsumedCoolingEnergy = 0; // Re-write
  }
  if ((HeatPump.Status.DeliveredHotWaterEnergy == 0) &&
      (HeatPump.Status.ConsumedHotWaterEnergy > 0)) {
    HeatPump.Status.ConsumedHotWaterEnergy = 0; // Re-write
  }

  // CoP Calculations to avoid divide by 0 occuring
  if (HeatPump.Status.ConsumedHeatingEnergy > 0) {
    heat_cop = HeatPump.Status.DeliveredHeatingEnergy /
               HeatPump.Status.ConsumedHeatingEnergy;
  } else {
    heat_cop = 0;
  }
  if (HeatPump.Status.ConsumedCoolingEnergy > 0) {
    cool_cop = HeatPump.Status.DeliveredCoolingEnergy /
               HeatPump.Status.ConsumedCoolingEnergy;
  } else {
    cool_cop = 0;
  }
  if (HeatPump.Status.ConsumedHotWaterEnergy > 0) {
    dhw_cop = (HeatPump.Status.DeliveredHotWaterEnergy /
               HeatPump.Status.ConsumedHotWaterEnergy);
  } else {
    dhw_cop = 0;
  }

  // CoP Totals
  ctotal = (HeatPump.Status.ConsumedHeatingEnergy +
            HeatPump.Status.ConsumedCoolingEnergy +
            HeatPump.Status.ConsumedHotWaterEnergy);
  dtotal = (HeatPump.Status.DeliveredHeatingEnergy +
            HeatPump.Status.DeliveredCoolingEnergy +
            HeatPump.Status.DeliveredHotWaterEnergy);
  if (ctotal != 0) {
    total_cop = dtotal / ctotal;
  } else {
    total_cop = 0;
  }

  // Write into the JSON with 2dp rounding (disabled in cascade mode)
  if (!Flags::CascadeActive()) {
    doc[F("CHEAT")] = round2(HeatPump.Status.ConsumedHeatingEnergy);
    if (Flags::HasCooling()) {
      doc[F("CCOOL")] = round2(HeatPump.Status.ConsumedCoolingEnergy);
    }
    doc[F("CDHW")] = round2(HeatPump.Status.ConsumedHotWaterEnergy);
    doc[F("DHEAT")] = round2(HeatPump.Status.DeliveredHeatingEnergy);
    if (Flags::HasCooling()) {
      doc[F("DCOOL")] = round2(HeatPump.Status.DeliveredCoolingEnergy);
    }
    doc[F("DDHW")] = round2(HeatPump.Status.DeliveredHotWaterEnergy);
    doc[F("CTOTAL")] = round2(ctotal);
    doc[F("DTOTAL")] = round2(dtotal);
  }
  if (!Flags::CascadeActive()) {
    doc[F("HEAT_CoP")] = round2(heat_cop);
  }
  if (!Flags::CascadeActive() && Flags::HasCooling()) {
    doc[F("COOL_CoP")] = round2(cool_cop);
  }
  if (!Flags::CascadeActive()) {
    doc[F("DHW_CoP")] = round2(dhw_cop);
  }
  if (!Flags::CascadeActive()) {
    doc[F("TOTAL_CoP")] = round2(total_cop);
  }
  doc[F("HB_ID")] = Heart_Value;
  serializeJson(doc, Buffer);
  MQTTClient1.publish(MQTT_STATUS_ENERGY.c_str(), Buffer, false);
  MQTTClient2.publish(MQTT_2_STATUS_ENERGY.c_str(), Buffer, false);
}

void CompCurveReport(void) {
  JsonDocument storeddoc;
  deserializeJson(storeddoc, unitSettings.CompCurve);
  JsonObject obj = storeddoc.as<JsonObject>();
  JsonDocument doc;
  char Buffer[1024];
  doc = obj;
  doc[F("zone1")]["active"] = unitSettings.z1_active;
  doc[F("zone1")]["manual_offset"] = unitSettings.z1_manual_offset;
  doc[F("zone1")]["temp_offset"] = unitSettings.z1_temp_offset;
  doc[F("zone1")]["wind_offset"] = unitSettings.z1_wind_offset;
  doc[F("zone1")]["calculated_FSP"] = Z1_CurveFSP;
  doc[F("zone2")]["active"] = unitSettings.z2_active;
  doc[F("zone2")]["manual_offset"] = unitSettings.z2_manual_offset;
  doc[F("zone2")]["temp_offset"] = unitSettings.z2_temp_offset;
  doc[F("zone2")]["wind_offset"] = unitSettings.z2_wind_offset;
  doc[F("zone2")]["calculated_FSP"] = Z2_CurveFSP;
  doc[F("use_local_outdoor")] = unitSettings.use_local_outdoor;
  doc[F("cloud_outdoor")] = unitSettings.cloud_outdoor;
  doc[F("HB_ID")] = Heart_Value;
  serializeJson(doc, Buffer);
  MQTTClient1.publish(MQTT_STATUS_CURVE.c_str(), Buffer, false);
  MQTTClient2.publish(MQTT_2_STATUS_CURVE.c_str(), Buffer, false);
}

#ifdef ARDUINO_WT32_ETH01
// WARNING: onEvent is called from a separate FreeRTOS task (thread)!
void onEvent(arduino_event_id_t event) {
  switch (event) {
  case ARDUINO_EVENT_ETH_START:
    DEBUG_PRINTLN(F("ETH Started"));
    // The hostname must be set after the interface is started, but needs
    // to be set before DHCP, so set it from the event handler thread.
    ETH.setHostname("Ecodan-Bridge");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    DEBUG_PRINTLN(F("ETH Connected"));
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    DEBUG_PRINTLN(F("ETH Got IP"));
    DEBUG_PRINTLN(ETH);
    eth_connected = true;
    break;
  case ARDUINO_EVENT_ETH_LOST_IP:
    DEBUG_PRINTLN(F("ETH Lost IP"));
    eth_connected = false;
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    DEBUG_PRINTLN(F("ETH Disconnected"));
    eth_connected = false;
    break;
  case ARDUINO_EVENT_ETH_STOP:
    DEBUG_PRINTLN(F("ETH Stopped"));
    eth_connected = false;
    break;
  default:
    break;
  }
}
#endif
