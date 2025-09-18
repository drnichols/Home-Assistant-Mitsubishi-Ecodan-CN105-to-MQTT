#include "Debug.h"
#include "WString.h"
#include <ArduinoJson.h>
#include "MQTTDiscovery.h"
#include <ESPTelnet.h>
#if defined(ESP32)
#include "FS.h"
#include "SPIFFS.h"
#define LittleFS SPIFFS
#elif defined(ESP8266)
#include <LittleFS.h>
#endif
#include <WiFi.h>
#include <WiFiManager.h>
#include "Flags.h"

// Forward declarations for types and globals defined in main.cpp
struct ESPTelnet;
struct MqttSettings;
struct UnitSettings;
struct WiFiManagerParameter;
struct WiFiManager;
class ECODAN;          // from Ecodan.h
class PubSubClient;    // from PubSubClient.h

extern ESPTelnet TelnetServer;
extern MqttSettings mqttSettings;
extern UnitSettings unitSettings;
extern bool shouldSaveConfig;
extern bool gRequestedTelnetDebug;
extern bool gLittleFSMounted;
// Globals used by MQTTConfig helpers
extern ECODAN HeatPump;
extern PubSubClient MQTTClient1;
extern PubSubClient MQTTClient2;

// WiFiManager parameter declarations
extern WiFiManagerParameter custom_mqtt_server;
extern WiFiManagerParameter custom_mqtt_user;
extern WiFiManagerParameter custom_mqtt_pass;
extern WiFiManagerParameter custom_mqtt_port;
extern WiFiManagerParameter custom_mqtt_basetopic;
extern WiFiManagerParameter custom_mqtt2_server;
extern WiFiManagerParameter custom_mqtt2_user;
extern WiFiManagerParameter custom_mqtt2_pass;
extern WiFiManagerParameter custom_mqtt2_port;
extern WiFiManagerParameter custom_mqtt2_basetopic;
// New configuration parameters
extern WiFiManagerParameter custom_boiler_installed;
extern WiFiManagerParameter custom_booster1_installed;
extern WiFiManagerParameter custom_booster2_installed;
// Dynamic checkbox parameter that reflects current cascadeEnabled state
class CascadeCheckboxParameter : public WiFiManagerParameter {
public:
  using WiFiManagerParameter::WiFiManagerParameter;
  virtual const char *getCustomHTML() const override;
};

extern CascadeCheckboxParameter custom_cascade_enabled;
extern WiFiManagerParameter custom_cascade_node_id;
extern WiFiManagerParameter custom_cascade_basetopic;
extern WiFiManagerParameter custom_device_id;

// Other external variables
extern WiFiManager wifiManager;

// Constants
extern const int deviceId_max_length;
extern const int hostname_max_length;
extern const int user_max_length;
extern const int password_max_length;
extern const int port_max_length;
extern const int basetopic_max_length;

String MQTT_BASETOPIC = "Ecodan/ASHP";

String MQTT_LWT = MQTT_BASETOPIC + "/LWT";
String MQTT_STATUS = MQTT_BASETOPIC + "/Status";
String MQTT_COMMAND = MQTT_BASETOPIC + "/Command";

String MQTT_STATUS_ZONE1 = MQTT_STATUS + "/Zone1";
String MQTT_STATUS_ZONE2 = MQTT_STATUS + "/Zone2";
String MQTT_STATUS_HOTWATER = MQTT_STATUS + "/HotWater";
String MQTT_STATUS_SYSTEM = MQTT_STATUS + "/System";
String MQTT_STATUS_CONFIGURATION = MQTT_STATUS + "/Configuration";
String MQTT_STATUS_ADVANCED = MQTT_STATUS + "/Advanced";
String MQTT_STATUS_ADVANCED_TWO = MQTT_STATUS + "/AdvancedTwo";
String MQTT_STATUS_ENERGY = MQTT_STATUS + "/Energy";
String MQTT_STATUS_WIFISTATUS = MQTT_STATUS + "/WiFiStatus";
String MQTT_STATUS_CURVE = MQTT_STATUS + "/CompCurve";

// Debug channel topics (primary)
String MQTT_DEBUG = MQTT_BASETOPIC + "/Debug";
String MQTT_DEBUG_CALC = MQTT_DEBUG + "/CalculateCompCurve";
String MQTT_DEBUG_ENABLE =
    MQTT_DEBUG + "/Enable/MQTT"; // payload: true/false (retained)
String MQTT_DEBUG_ENABLE_SERIAL = MQTT_DEBUG + "/Enable/Serial";
String MQTT_DEBUG_ENABLE_TELNET = MQTT_DEBUG + "/Enable/Telnet";

String MQTT_COMMAND_ZONE1 = MQTT_COMMAND + "/Zone1";
String MQTT_COMMAND_ZONE2 = MQTT_COMMAND + "/Zone2";
String MQTT_COMMAND_HOTWATER = MQTT_COMMAND + "/HotWater";
String MQTT_COMMAND_SYSTEM = MQTT_COMMAND + "/System";

String MQTT_COMMAND_ZONE1_FLOW_SETPOINT = MQTT_COMMAND_ZONE1 + "/FlowSetpoint";
String MQTT_COMMAND_ZONE1_NOMODE_SETPOINT =
    MQTT_COMMAND_ZONE1 + "/ThermostatSetpoint";
String MQTT_COMMAND_ZONE1_HEAT_PROHIBIT =
    MQTT_COMMAND_ZONE1 + "/ProhibitHeating";
String MQTT_COMMAND_ZONE1_COOL_PROHIBIT =
    MQTT_COMMAND_ZONE1 + "/ProhibitCooling";
String MQTT_COMMAND_ZONE1_HEATINGMODE = MQTT_COMMAND_ZONE1 + "/HeatingMode";

String MQTT_COMMAND_ZONE2_FLOW_SETPOINT = MQTT_COMMAND_ZONE2 + "/FlowSetpoint";
String MQTT_COMMAND_ZONE2_NOMODE_SETPOINT =
    MQTT_COMMAND_ZONE2 + "/ThermostatSetpoint";
String MQTT_COMMAND_ZONE2_HEAT_PROHIBIT =
    MQTT_COMMAND_ZONE2 + "/ProhibitHeating";
String MQTT_COMMAND_ZONE2_COOL_PROHIBIT =
    MQTT_COMMAND_ZONE2 + "/ProhibitCooling";
String MQTT_COMMAND_ZONE2_HEATINGMODE = MQTT_COMMAND_ZONE2 + "/HeatingMode";

String MQTT_COMMAND_HOTWATER_MODE = MQTT_COMMAND_HOTWATER + "/Mode";
String MQTT_COMMAND_HOTWATER_SETPOINT = MQTT_COMMAND_HOTWATER + "/Setpoint";
String MQTT_COMMAND_HOTWATER_BOOST = MQTT_COMMAND_HOTWATER + "/Boost";
String MQTT_COMMAND_HOTWATER_NORM_BOOST =
    MQTT_COMMAND_HOTWATER + "/NormalBoost";
String MQTT_COMMAND_HOTWATER_PROHIBIT = MQTT_COMMAND_HOTWATER + "/Prohibit";

String MQTT_COMMAND_SYSTEM_HOLIDAYMODE = MQTT_COMMAND_SYSTEM + "/HolidayMode";
String MQTT_COMMAND_SYSTEM_SVRMODE = MQTT_COMMAND_SYSTEM + "/SvrControlMode";
String MQTT_COMMAND_SYSTEM_POWER = MQTT_COMMAND_SYSTEM + "/Power";
String MQTT_COMMAND_SYSTEM_UNITSIZE = MQTT_COMMAND_SYSTEM + "/UnitSize";
String MQTT_COMMAND_SYSTEM_GLYCOL = MQTT_COMMAND_SYSTEM + "/Glycol";
String MQTT_COMMAND_SYSTEM_SERVICE = MQTT_COMMAND_SYSTEM + "/Svc";
String MQTT_COMMAND_SYSTEM_COMPCURVE = MQTT_COMMAND_SYSTEM + "/CompCurve";

String MQTTCommandZone1FlowSetpoint = MQTT_COMMAND_ZONE1_FLOW_SETPOINT;
String MQTTCommandZone1NoModeSetpoint = MQTT_COMMAND_ZONE1_NOMODE_SETPOINT;
String MQTTCommandZone1ProhibitHeating = MQTT_COMMAND_ZONE1_HEAT_PROHIBIT;
String MQTTCommandZone1ProhibitCooling = MQTT_COMMAND_ZONE1_COOL_PROHIBIT;
String MQTTCommandZone1HeatingMode = MQTT_COMMAND_ZONE1_HEATINGMODE;

String MQTTCommandZone2FlowSetpoint = MQTT_COMMAND_ZONE2_FLOW_SETPOINT;
String MQTTCommandZone2NoModeSetpoint = MQTT_COMMAND_ZONE2_NOMODE_SETPOINT;
String MQTTCommandZone2ProhibitHeating = MQTT_COMMAND_ZONE2_HEAT_PROHIBIT;
String MQTTCommandZone2ProhibitCooling = MQTT_COMMAND_ZONE2_COOL_PROHIBIT;
String MQTTCommandZone2HeatingMode = MQTT_COMMAND_ZONE2_HEATINGMODE;

String MQTTCommandHotwaterMode = MQTT_COMMAND_HOTWATER_MODE;
String MQTTCommandHotwaterSetpoint = MQTT_COMMAND_HOTWATER_SETPOINT;
String MQTTCommandHotwaterBoost = MQTT_COMMAND_HOTWATER_BOOST;
String MQTTCommandHotwaterNormalBoost = MQTT_COMMAND_HOTWATER_NORM_BOOST;
String MQTTCommandHotwaterProhibit = MQTT_COMMAND_HOTWATER_PROHIBIT;

String MQTTCommandSystemHolidayMode = MQTT_COMMAND_SYSTEM_HOLIDAYMODE;
String MQTTCommandSystemSvrMode = MQTT_COMMAND_SYSTEM_SVRMODE;
String MQTTCommandSystemPower = MQTT_COMMAND_SYSTEM_POWER;
String MQTTCommandSystemUnitSize = MQTT_COMMAND_SYSTEM_UNITSIZE;
String MQTTCommandSystemGlycol = MQTT_COMMAND_SYSTEM_GLYCOL;
String MQTTCommandSystemService = MQTT_COMMAND_SYSTEM_SERVICE;
String MQTTCommandSystemCompCurve = MQTT_COMMAND_SYSTEM_COMPCURVE;

String MQTT_2_BASETOPIC = "00000";

String MQTT_2_LWT = MQTT_2_BASETOPIC + "/LWT";
String MQTT_2_STATUS = MQTT_2_BASETOPIC + "/Status";
String MQTT_2_COMMAND = MQTT_2_BASETOPIC + "/Command";

String MQTT_2_STATUS_ZONE1 = MQTT_2_STATUS + "/Zone1";
String MQTT_2_STATUS_ZONE2 = MQTT_2_STATUS + "/Zone2";
String MQTT_2_STATUS_HOTWATER = MQTT_2_STATUS + "/HotWater";
String MQTT_2_STATUS_SYSTEM = MQTT_2_STATUS + "/System";
String MQTT_2_STATUS_CONFIGURATION = MQTT_2_STATUS + "/Configuration";
String MQTT_2_STATUS_ADVANCED = MQTT_2_STATUS + "/Advanced";
String MQTT_2_STATUS_ADVANCED_TWO = MQTT_2_STATUS + "/AdvancedTwo";
String MQTT_2_STATUS_ENERGY = MQTT_2_STATUS + "/Energy";
String MQTT_2_STATUS_WIFISTATUS = MQTT_2_STATUS + "/WiFiStatus";
String MQTT_2_STATUS_CURVE = MQTT_2_STATUS + "/CompCurve";

// Debug channel topics (secondary)
String MQTT_2_DEBUG = MQTT_2_BASETOPIC + "/Debug";
String MQTT_2_DEBUG_CALC = MQTT_2_DEBUG + "/CalculateCompCurve";
String MQTT_2_DEBUG_ENABLE = MQTT_2_DEBUG + "/Enable/MQTT";
String MQTT_2_DEBUG_ENABLE_SERIAL = MQTT_2_DEBUG + "/Enable/Serial";
String MQTT_2_DEBUG_ENABLE_TELNET = MQTT_2_DEBUG + "/Enable/Telnet";

String MQTT_2_COMMAND_ZONE1 = MQTT_2_COMMAND + "/Zone1";
String MQTT_2_COMMAND_ZONE2 = MQTT_2_COMMAND + "/Zone2";
String MQTT_2_COMMAND_HOTWATER = MQTT_2_COMMAND + "/HotWater";
String MQTT_2_COMMAND_SYSTEM = MQTT_2_COMMAND + "/System";

String MQTT_2_COMMAND_ZONE1_FLOW_SETPOINT =
    MQTT_2_COMMAND_ZONE1 + "/FlowSetpoint";
String MQTT_2_COMMAND_ZONE1_NOMODE_SETPOINT =
    MQTT_2_COMMAND_ZONE1 + "/ThermostatSetpoint";
String MQTT_2_COMMAND_ZONE1_HEAT_PROHIBIT =
    MQTT_2_COMMAND_ZONE1 + "/ProhibitHeating";
String MQTT_2_COMMAND_ZONE1_COOL_PROHIBIT =
    MQTT_2_COMMAND_ZONE1 + "/ProhibitCooling";
String MQTT_2_COMMAND_ZONE1_HEATINGMODE = MQTT_2_COMMAND_ZONE1 + "/HeatingMode";

String MQTT_2_COMMAND_ZONE2_FLOW_SETPOINT =
    MQTT_2_COMMAND_ZONE2 + "/FlowSetpoint";
String MQTT_2_COMMAND_ZONE2_NOMODE_SETPOINT =
    MQTT_2_COMMAND_ZONE2 + "/ThermostatSetpoint";
String MQTT_2_COMMAND_ZONE2_HEAT_PROHIBIT =
    MQTT_2_COMMAND_ZONE2 + "/ProhibitHeating";
String MQTT_2_COMMAND_ZONE2_COOL_PROHIBIT =
    MQTT_2_COMMAND_ZONE2 + "/ProhibitCooling";
String MQTT_2_COMMAND_ZONE2_HEATINGMODE = MQTT_2_COMMAND_ZONE2 + "/HeatingMode";

String MQTT_2_COMMAND_HOTWATER_MODE = MQTT_2_COMMAND_HOTWATER + "/Mode";
String MQTT_2_COMMAND_HOTWATER_SETPOINT = MQTT_2_COMMAND_HOTWATER + "/Setpoint";
String MQTT_2_COMMAND_HOTWATER_BOOST = MQTT_2_COMMAND_HOTWATER + "/Boost";
String MQTT_2_COMMAND_HOTWATER_NORM_BOOST =
    MQTT_2_COMMAND_HOTWATER + "/NormalBoost";
String MQTT_2_COMMAND_HOTWATER_PROHIBIT = MQTT_2_COMMAND_HOTWATER + "/Prohibit";

String MQTT_2_COMMAND_SYSTEM_HOLIDAYMODE =
    MQTT_2_COMMAND_SYSTEM + "/HolidayMode";
String MQTT_2_COMMAND_SYSTEM_SVRMODE =
    MQTT_2_COMMAND_SYSTEM + "/SvrControlMode";
String MQTT_2_COMMAND_SYSTEM_POWER = MQTT_2_COMMAND_SYSTEM + "/Power";
String MQTT_2_COMMAND_SYSTEM_UNITSIZE = MQTT_2_COMMAND_SYSTEM + "/UnitSize";
String MQTT_2_COMMAND_SYSTEM_GLYCOL = MQTT_2_COMMAND_SYSTEM + "/Glycol";
String MQTT_2_COMMAND_SYSTEM_SERVICE = MQTT_2_COMMAND_SYSTEM + "/Svc";
String MQTT_2_COMMAND_SYSTEM_COMPCURVE = MQTT_2_COMMAND_SYSTEM + "/CompCurve";

String MQTTCommand2Zone1FlowSetpoint = MQTT_2_COMMAND_ZONE1_FLOW_SETPOINT;
String MQTTCommand2Zone1NoModeSetpoint = MQTT_2_COMMAND_ZONE1_NOMODE_SETPOINT;
String MQTTCommand2Zone1ProhibitHeating = MQTT_2_COMMAND_ZONE1_HEAT_PROHIBIT;
String MQTTCommand2Zone1ProhibitCooling = MQTT_2_COMMAND_ZONE1_COOL_PROHIBIT;
String MQTTCommand2Zone1HeatingMode = MQTT_2_COMMAND_ZONE1_HEATINGMODE;

String MQTTCommand2Zone2FlowSetpoint = MQTT_2_COMMAND_ZONE2_FLOW_SETPOINT;
String MQTTCommand2Zone2NoModeSetpoint = MQTT_2_COMMAND_ZONE2_NOMODE_SETPOINT;
String MQTTCommand2Zone2ProhibitHeating = MQTT_2_COMMAND_ZONE2_HEAT_PROHIBIT;
String MQTTCommand2Zone2ProhibitCooling = MQTT_2_COMMAND_ZONE2_COOL_PROHIBIT;
String MQTTCommand2Zone2HeatingMode = MQTT_2_COMMAND_ZONE2_HEATINGMODE;

String MQTTCommand2HotwaterMode = MQTT_2_COMMAND_HOTWATER_MODE;
String MQTTCommand2HotwaterSetpoint = MQTT_2_COMMAND_HOTWATER_SETPOINT;
String MQTTCommand2HotwaterBoost = MQTT_2_COMMAND_HOTWATER_BOOST;
String MQTTCommand2HotwaterNormalBoost = MQTT_2_COMMAND_HOTWATER_NORM_BOOST;
String MQTTCommand2HotwaterProhibit = MQTT_2_COMMAND_HOTWATER_PROHIBIT;

String MQTTCommand2SystemHolidayMode = MQTT_2_COMMAND_SYSTEM_HOLIDAYMODE;
String MQTTCommand2SystemSvrMode = MQTT_2_COMMAND_SYSTEM_SVRMODE;
String MQTTCommand2SystemPower = MQTT_2_COMMAND_SYSTEM_POWER;
String MQTTCommand2SystemUnitSize = MQTT_2_COMMAND_SYSTEM_UNITSIZE;
String MQTTCommand2SystemGlycol = MQTT_2_COMMAND_SYSTEM_GLYCOL;
String MQTTCommand2SystemService = MQTT_2_COMMAND_SYSTEM_SERVICE;
String MQTTCommand2SystemCompCurve = MQTT_2_COMMAND_SYSTEM_COMPCURVE;

char snprintbuffer[41] = "";
char DeviceID[15] = "";
const char ClientPrefix[14] = "EcodanBridge-";
char WiFiHostname[40] = "";

// Programs

#if defined(ESP8266) || defined(ESP32) // ESP32 or ESP8266 Compatibility
void readSettingsFromConfig() {
  // Clean LittleFS for testing
  // LittleFS.format();

  // Read configuration from LittleFS JSON
  DEBUG_PRINTLN(F("Mounting File System..."));
  gLittleFSMounted = false;
#ifdef ESP8266
  if (LittleFS.begin()) {
#endif
#ifdef ESP32
    if (LittleFS.begin("/storage")) {
#endif
      gLittleFSMounted = true;
      DEBUG_PRINTLN(F("Mounted File System"));
      if (LittleFS.exists("/config.json")) {
        // file exists, reading and loading
        DEBUG_PRINTLN(F("Reading config file"));
        File configFile = LittleFS.open("/config.json", "r");
        if (configFile) {
          DEBUG_PRINTLN(F("Opened config file"));
          JsonDocument doc;
          DeserializationError error = deserializeJson(doc, configFile);
          if (error) {
            DEBUG_PRINT(F("Failed to read file: "));
            DEBUG_PRINTLN(error.c_str());
          } else {
            DEBUG_PRINTLN(F("Parsed JSON: "));
            serializeJson(doc, TelnetServer);
            DEBUG_PRINTLN();

            // Build in safety check, otherwise ESP will crash out and you can't
            // get back in
            if (!doc[mqttSettings.wm_device_id_identifier].isNull()) {
              String deviceIdValue =
                  doc[mqttSettings.wm_device_id_identifier].as<String>();
              if ((deviceIdValue.length() > 0) &&
                  ((deviceIdValue.length() + 1) <= deviceId_max_length)) {
                strcpy(mqttSettings.deviceId, deviceIdValue.c_str());
              }
            } else { // For upgrading from <5.3.1, create the entry
#ifdef ESP8266
              snprintf(snprintbuffer, deviceId_max_length,
                       (String(ESP.getChipId(), HEX)).c_str());
#endif
#ifdef ESP32
              snprintf(snprintbuffer, deviceId_max_length,
                       (String(ESP.getEfuseMac(), HEX)).c_str());
#endif
              strcpy(mqttSettings.deviceId, snprintbuffer);
              shouldSaveConfig =
                  true; // Save config after exit to update the file
            }
            if (!doc[mqttSettings.wm_mqtt_hostname_identifier].isNull()) {
              String hostnameValue =
                  doc[mqttSettings.wm_mqtt_hostname_identifier].as<String>();
              if ((hostnameValue.length() > 0) &&
                  ((hostnameValue.length() + 1) <= hostname_max_length)) {
                strcpy(mqttSettings.hostname, hostnameValue.c_str());
              }
            }
            if (!doc[mqttSettings.wm_mqtt_port_identifier].isNull()) {
              String portValue =
                  doc[mqttSettings.wm_mqtt_port_identifier].as<String>();
              if ((portValue.length() > 0) &&
                  ((portValue.length() + 1) <= port_max_length)) {
                strcpy(mqttSettings.port, portValue.c_str());
              }
            }
            if (!doc[mqttSettings.wm_mqtt_user_identifier].isNull()) {
              String userValue =
                  doc[mqttSettings.wm_mqtt_user_identifier].as<String>();
              if ((userValue.length() > 0) &&
                  ((userValue.length() + 1) <= user_max_length)) {
                strcpy(mqttSettings.user, userValue.c_str());
              }
            }
            if (!doc[mqttSettings.wm_mqtt_password_identifier].isNull()) {
              String passwordValue =
                  doc[mqttSettings.wm_mqtt_password_identifier].as<String>();
              if ((passwordValue.length() > 0) &&
                  ((passwordValue.length() + 1) <= password_max_length)) {
                strcpy(mqttSettings.password, passwordValue.c_str());
              }
            }
            if (!doc[mqttSettings.wm_mqtt_basetopic_identifier].isNull()) {
              String baseTopicValue =
                  doc[mqttSettings.wm_mqtt_basetopic_identifier].as<String>();
              if ((baseTopicValue.length() > 0) &&
                  ((baseTopicValue.length() + 1) <= basetopic_max_length)) {
                strcpy(mqttSettings.baseTopic, baseTopicValue.c_str());
                MQTT_BASETOPIC = mqttSettings.baseTopic;
              }
            }
            // MQTT Stream 2
            if (!doc[mqttSettings.wm_mqtt2_hostname_identifier].isNull()) {
              String hostname2Value =
                  doc[mqttSettings.wm_mqtt2_hostname_identifier].as<String>();
              if ((hostname2Value.length() > 0) &&
                  ((hostname2Value.length() + 1) <= hostname_max_length)) {
                strcpy(mqttSettings.hostname2, hostname2Value.c_str());
              }
            } else { // For upgrading from <6.0.0, create the entry
              snprintf(snprintbuffer, hostname_max_length,
                       mqttSettings.hostname2);
              strcpy(mqttSettings.hostname2, snprintbuffer);
              shouldSaveConfig =
                  true; // Save config after exit to update the file
            }
            if (!doc[mqttSettings.wm_mqtt2_port_identifier].isNull()) {
              String port2Value =
                  doc[mqttSettings.wm_mqtt2_port_identifier].as<String>();
              if ((port2Value.length() > 0) &&
                  ((port2Value.length() + 1) <= port_max_length)) {
                strcpy(mqttSettings.port2, port2Value.c_str());
              }
            } else { // For upgrading from <6.0.0, create the entry
              snprintf(snprintbuffer, port_max_length, mqttSettings.port2);
              strcpy(mqttSettings.port2, snprintbuffer);
              shouldSaveConfig =
                  true; // Save config after exit to update the file
            }
            if (!doc[mqttSettings.wm_mqtt2_user_identifier].isNull()) {
              String user2Value =
                  doc[mqttSettings.wm_mqtt2_user_identifier].as<String>();
              if ((user2Value.length() > 0) &&
                  ((user2Value.length() + 1) <= user_max_length)) {
                strcpy(mqttSettings.user2, user2Value.c_str());
              }
            } else { // For upgrading from <6.0.0, create the entry
              snprintf(snprintbuffer, user_max_length, mqttSettings.user2);
              strcpy(mqttSettings.user2, snprintbuffer);
              shouldSaveConfig =
                  true; // Save config after exit to update the file
            }
            if (!doc[mqttSettings.wm_mqtt2_password_identifier].isNull()) {
              String password2Value =
                  doc[mqttSettings.wm_mqtt2_password_identifier].as<String>();
              if ((password2Value.length() > 0) &&
                  ((password2Value.length() + 1) <= password_max_length)) {
                strcpy(mqttSettings.password2, password2Value.c_str());
              }
            } else { // For upgrading from <6.0.0, create the entry
              snprintf(snprintbuffer, password_max_length,
                       mqttSettings.password2);
              strcpy(mqttSettings.password2, snprintbuffer);
              shouldSaveConfig =
                  true; // Save config after exit to update the file
            }
            if (!doc[mqttSettings.wm_mqtt2_basetopic_identifier].isNull()) {
              String baseTopic2Value =
                  doc[mqttSettings.wm_mqtt2_basetopic_identifier].as<String>();
              if ((baseTopic2Value.length() > 0) &&
                  ((baseTopic2Value.length() + 1) <= basetopic_max_length)) {
                strcpy(mqttSettings.baseTopic2, baseTopic2Value.c_str());
                MQTT_2_BASETOPIC = mqttSettings.baseTopic2;
              }
            } else { // For upgrading from <6.0.0, create the entry
              strcpy(mqttSettings.baseTopic2, mqttSettings.deviceId);
              MQTT_2_BASETOPIC = mqttSettings.baseTopic2;
              shouldSaveConfig =
                  true; // Save config after exit to update the file
            }
            // Cascade base topic (shared across nodes)
            if (!doc[mqttSettings.cascade_base_topic_identifier].isNull()) {
              String cascadeBase =
                  doc[mqttSettings.cascade_base_topic_identifier].as<String>();
              if ((cascadeBase.length() > 0) &&
                  ((cascadeBase.length() + 1) <= basetopic_max_length)) {
                strcpy(mqttSettings.cascadeBaseTopic, cascadeBase.c_str());
              }
            } else {
              // Default to primary base topic to preserve legacy behavior
              strcpy(mqttSettings.cascadeBaseTopic, mqttSettings.baseTopic);
              shouldSaveConfig = true;
            }
            // Cascade settings
            if (!doc[mqttSettings.cascade_enabled_identifier].isNull()) {
              // Accept bool or string; ArduinoJson handles conversion
              mqttSettings.cascadeEnabled =
                  doc[mqttSettings.cascade_enabled_identifier].as<bool>();
            } else {
              // Add missing key for upgrade
              shouldSaveConfig = true;
            }
            if (!doc[mqttSettings.cascade_node_id_identifier].isNull()) {
              int nodeId = doc[mqttSettings.cascade_node_id_identifier].as<int>();
              if (nodeId < 0)
                nodeId = 0;
              if (nodeId > 7)
                nodeId = 7;
              mqttSettings.cascadeNodeId = (uint8_t)nodeId;
            } else {
              // Add missing key for upgrade
              shouldSaveConfig = true;
            }
            // Cascade test mode removed

            if (!doc["debug_enable_telnet"].isNull()) {
              gRequestedTelnetDebug = doc["debug_enable_telnet"].as<bool>();
            } else {
              shouldSaveConfig = true;
            }

            // Installation flags (string values: 'true' or 'false')
            if (!doc[mqttSettings.wm_boiler_installed_identifier].isNull()) {
              String v = doc[mqttSettings.wm_boiler_installed_identifier].as<String>();
              if (v.length() > 0 && v.length() < sizeof(mqttSettings.boiler_installed)) {
                strcpy(mqttSettings.boiler_installed, v.c_str());
              }
            } else {
              // default to true; add to config on next save
              strcpy(mqttSettings.boiler_installed, "true");
              shouldSaveConfig = true;
            }
            if (!doc[mqttSettings.wm_booster1_installed_identifier].isNull()) {
              String v = doc[mqttSettings.wm_booster1_installed_identifier].as<String>();
              if (v.length() > 0 && v.length() < sizeof(mqttSettings.booster1_installed)) {
                strcpy(mqttSettings.booster1_installed, v.c_str());
              }
            } else {
              strcpy(mqttSettings.booster1_installed, "true");
              shouldSaveConfig = true;
            }
            if (!doc[mqttSettings.wm_booster2_installed_identifier].isNull()) {
              String v = doc[mqttSettings.wm_booster2_installed_identifier].as<String>();
              if (v.length() > 0 && v.length() < sizeof(mqttSettings.booster2_installed)) {
                strcpy(mqttSettings.booster2_installed, v.c_str());
              }
            } else {
              strcpy(mqttSettings.booster2_installed, "true");
              shouldSaveConfig = true;
            }
            // Unit Size
            if (!doc[unitSettings.unitsize_identifier].isNull()) {
              if (doc[unitSettings.unitsize_identifier].as<float>() > 0) {
                unitSettings.UnitSize = doc[unitSettings.unitsize_identifier];
              }
            } else { // For upgrading from <6.1.1, create the entry
              shouldSaveConfig =
                  true; // Save config after exit to update the file
            }
            // Glycol Strength
            if (!doc[unitSettings.glycol_identifier].isNull()) {
              if (doc[unitSettings.glycol_identifier].as<float>() > 0) {
                unitSettings.GlycolStrength =
                    doc[unitSettings.glycol_identifier].as<float>();
              }
            } else { // For upgrading from <6.1.1, create the entry
              shouldSaveConfig =
                  true; // Save config after exit to update the file
            }
            // Compensation Curve
            if (!doc[unitSettings.compcurve_identifier].isNull()) {
              String compCurveValue =
                  doc[unitSettings.compcurve_identifier].as<String>();
              if (compCurveValue.length() > 0) {
                unitSettings.CompCurve = compCurveValue;
              }
            } else { // For upgrading from <6.3.0, create the entry
              shouldSaveConfig =
                  true; // Save config after exit to update the file
            }
          }
        }
        configFile.close();
      } else {
        DEBUG_PRINTLN(F("No config file exists, using placeholder values"));
        // Populate the Dynamic Variables (Device ID)
#ifdef ESP8266
        snprintf(DeviceID, deviceId_max_length,
                 (String(ESP.getChipId(), HEX)).c_str());
#endif
#ifdef ESP32
        snprintf(DeviceID, deviceId_max_length,
                 (String(ESP.getEfuseMac(), HEX)).c_str());
#endif
        strcpy(mqttSettings.deviceId, DeviceID);
        strcpy(mqttSettings.baseTopic2,
               DeviceID); // Base topic 2 defaults to deviceID
      }
    } else {
      gLittleFSMounted = false;
      DEBUG_PRINTLN(F("Failed to mount File System"));
    }
  }

  void RecalculateMQTTTopics() {
    // The base topic may change via WiFi Manager
    MQTT_LWT = MQTT_BASETOPIC + "/LWT";
    MQTT_STATUS = MQTT_BASETOPIC + "/Status";
    MQTT_COMMAND = MQTT_BASETOPIC + "/Command";

    MQTT_STATUS_ZONE1 = MQTT_STATUS + "/Zone1";
    MQTT_STATUS_ZONE2 = MQTT_STATUS + "/Zone2";
    MQTT_STATUS_HOTWATER = MQTT_STATUS + "/HotWater";
    MQTT_STATUS_SYSTEM = MQTT_STATUS + "/System";
    MQTT_STATUS_CONFIGURATION = MQTT_STATUS + "/Configuration";
    MQTT_STATUS_ADVANCED = MQTT_STATUS + "/Advanced";
    MQTT_STATUS_ADVANCED_TWO = MQTT_STATUS + "/AdvancedTwo";
    MQTT_STATUS_ENERGY = MQTT_STATUS + "/Energy";
    MQTT_STATUS_WIFISTATUS = MQTT_STATUS + "/WiFiStatus";
    MQTT_STATUS_CURVE = MQTT_STATUS + "/CompCurve";
    // Debug topics follow base topic
    MQTT_DEBUG = MQTT_BASETOPIC + "/Debug";
    MQTT_DEBUG_CALC = MQTT_DEBUG + "/CalculateCompCurve";
    MQTT_DEBUG_ENABLE = MQTT_DEBUG + "/Enable/MQTT";
    MQTT_DEBUG_ENABLE_SERIAL = MQTT_DEBUG + "/Enable/Serial";
    MQTT_DEBUG_ENABLE_TELNET = MQTT_DEBUG + "/Enable/Telnet";

    MQTT_COMMAND_ZONE1 = MQTT_COMMAND + "/Zone1";
    MQTT_COMMAND_ZONE2 = MQTT_COMMAND + "/Zone2";
    MQTT_COMMAND_HOTWATER = MQTT_COMMAND + "/HotWater";
    MQTT_COMMAND_SYSTEM = MQTT_COMMAND + "/System";

    MQTT_COMMAND_ZONE1_FLOW_SETPOINT = MQTT_COMMAND_ZONE1 + "/FlowSetpoint";
    MQTT_COMMAND_ZONE1_NOMODE_SETPOINT =
        MQTT_COMMAND_ZONE1 + "/ThermostatSetpoint";
    MQTT_COMMAND_ZONE1_HEAT_PROHIBIT = MQTT_COMMAND_ZONE1 + "/ProhibitHeating";
    MQTT_COMMAND_ZONE1_COOL_PROHIBIT = MQTT_COMMAND_ZONE1 + "/ProhibitCooling";
    MQTT_COMMAND_ZONE1_HEATINGMODE = MQTT_COMMAND_ZONE1 + "/HeatingMode";

    MQTT_COMMAND_ZONE2_FLOW_SETPOINT = MQTT_COMMAND_ZONE2 + "/FlowSetpoint";
    MQTT_COMMAND_ZONE2_NOMODE_SETPOINT =
        MQTT_COMMAND_ZONE2 + "/ThermostatSetpoint";
    MQTT_COMMAND_ZONE2_HEAT_PROHIBIT = MQTT_COMMAND_ZONE2 + "/ProhibitHeating";
    MQTT_COMMAND_ZONE2_COOL_PROHIBIT = MQTT_COMMAND_ZONE2 + "/ProhibitCooling";
    MQTT_COMMAND_ZONE2_HEATINGMODE = MQTT_COMMAND_ZONE2 + "/HeatingMode";

    MQTT_COMMAND_HOTWATER_MODE = MQTT_COMMAND_HOTWATER + "/Mode";
    MQTT_COMMAND_HOTWATER_SETPOINT = MQTT_COMMAND_HOTWATER + "/Setpoint";
    MQTT_COMMAND_HOTWATER_BOOST = MQTT_COMMAND_HOTWATER + "/Boost";
    MQTT_COMMAND_HOTWATER_NORM_BOOST = MQTT_COMMAND_HOTWATER + "/NormalBoost";
    MQTT_COMMAND_HOTWATER_PROHIBIT = MQTT_COMMAND_HOTWATER + "/Prohibit";

    MQTT_COMMAND_SYSTEM_HOLIDAYMODE = MQTT_COMMAND_SYSTEM + "/HolidayMode";
    MQTT_COMMAND_SYSTEM_SVRMODE = MQTT_COMMAND_SYSTEM + "/SvrControlMode";
    MQTT_COMMAND_SYSTEM_POWER = MQTT_COMMAND_SYSTEM + "/Power";
    MQTT_COMMAND_SYSTEM_UNITSIZE = MQTT_COMMAND_SYSTEM + "/UnitSize";
    MQTT_COMMAND_SYSTEM_GLYCOL = MQTT_COMMAND_SYSTEM + "/Glycol";
    MQTT_COMMAND_SYSTEM_SERVICE = MQTT_COMMAND_SYSTEM + "/Svc";
    MQTT_COMMAND_SYSTEM_COMPCURVE = MQTT_COMMAND_SYSTEM + "/CompCurve";

    MQTTCommandZone1FlowSetpoint = MQTT_COMMAND_ZONE1_FLOW_SETPOINT;
    MQTTCommandZone1NoModeSetpoint = MQTT_COMMAND_ZONE1_NOMODE_SETPOINT;
    MQTTCommandZone1ProhibitHeating = MQTT_COMMAND_ZONE1_HEAT_PROHIBIT;
    MQTTCommandZone1ProhibitCooling = MQTT_COMMAND_ZONE1_COOL_PROHIBIT;
    MQTTCommandZone1HeatingMode = MQTT_COMMAND_ZONE1_HEATINGMODE;

    MQTTCommandZone2FlowSetpoint = MQTT_COMMAND_ZONE2_FLOW_SETPOINT;
    MQTTCommandZone2NoModeSetpoint = MQTT_COMMAND_ZONE2_NOMODE_SETPOINT;
    MQTTCommandZone2ProhibitHeating = MQTT_COMMAND_ZONE2_HEAT_PROHIBIT;
    MQTTCommandZone2ProhibitCooling = MQTT_COMMAND_ZONE2_COOL_PROHIBIT;
    MQTTCommandZone2HeatingMode = MQTT_COMMAND_ZONE2_HEATINGMODE;

    MQTTCommandHotwaterMode = MQTT_COMMAND_HOTWATER_MODE;
    MQTTCommandHotwaterSetpoint = MQTT_COMMAND_HOTWATER_SETPOINT;
    MQTTCommandHotwaterBoost = MQTT_COMMAND_HOTWATER_BOOST;
    MQTTCommandHotwaterNormalBoost = MQTT_COMMAND_HOTWATER_NORM_BOOST;
    MQTTCommandHotwaterProhibit = MQTT_COMMAND_HOTWATER_PROHIBIT;

    MQTTCommandSystemHolidayMode = MQTT_COMMAND_SYSTEM_HOLIDAYMODE;
    MQTTCommandSystemSvrMode = MQTT_COMMAND_SYSTEM_SVRMODE;
    MQTTCommandSystemPower = MQTT_COMMAND_SYSTEM_POWER;
    MQTTCommandSystemUnitSize = MQTT_COMMAND_SYSTEM_UNITSIZE;
    MQTTCommandSystemGlycol = MQTT_COMMAND_SYSTEM_GLYCOL;
    MQTTCommandSystemService = MQTT_COMMAND_SYSTEM_SERVICE;
    MQTTCommandSystemCompCurve = MQTT_COMMAND_SYSTEM_COMPCURVE;
  }

  void saveConfig() {
    // Read MQTT Portal Values for save to file system
    DEBUG_PRINTLN(F("Copying Portal Values..."));
    strcpy(mqttSettings.deviceId, custom_device_id.getValue());
    strcpy(mqttSettings.hostname, custom_mqtt_server.getValue());
    strcpy(mqttSettings.port, custom_mqtt_port.getValue());
    strcpy(mqttSettings.user, custom_mqtt_user.getValue());
    strcpy(mqttSettings.password, custom_mqtt_pass.getValue());
    strcpy(mqttSettings.baseTopic, custom_mqtt_basetopic.getValue());
    strcpy(mqttSettings.hostname2, custom_mqtt2_server.getValue());
    strcpy(mqttSettings.port2, custom_mqtt2_port.getValue());
    strcpy(mqttSettings.user2, custom_mqtt2_user.getValue());
    strcpy(mqttSettings.password2, custom_mqtt2_pass.getValue());
    strcpy(mqttSettings.baseTopic2, custom_mqtt2_basetopic.getValue());
    strcpy(mqttSettings.cascadeBaseTopic, custom_cascade_basetopic.getValue());
    // Installation flags from portal
    strcpy(mqttSettings.boiler_installed, custom_boiler_installed.getValue());
    strcpy(mqttSettings.booster1_installed, custom_booster1_installed.getValue());
    strcpy(mqttSettings.booster2_installed, custom_booster2_installed.getValue());
    
    // Handle cascade parameters (only "true" or "false")
    const char *cascadeVal = custom_cascade_enabled.getValue();
    mqttSettings.cascadeEnabled = (cascadeVal != nullptr && strcmp(cascadeVal, "true") == 0);
    mqttSettings.cascadeNodeId = atoi(custom_cascade_node_id.getValue());
    // cascade test mode removed

    DEBUG_PRINT(F("Saving config... "));
    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile) {
      DEBUG_PRINTLN(F("[FAILED] Unable to open config file for writing"));
    } else {
      JsonDocument doc;
      doc[mqttSettings.wm_device_id_identifier] = mqttSettings.deviceId;
      doc[mqttSettings.wm_mqtt_hostname_identifier] = mqttSettings.hostname;
      doc[mqttSettings.wm_mqtt_port_identifier] = mqttSettings.port;
      doc[mqttSettings.wm_mqtt_user_identifier] = mqttSettings.user;
      doc[mqttSettings.wm_mqtt_password_identifier] = mqttSettings.password;
      doc[mqttSettings.wm_mqtt_basetopic_identifier] = mqttSettings.baseTopic;
      doc[mqttSettings.wm_mqtt2_hostname_identifier] = mqttSettings.hostname2;
      doc[mqttSettings.wm_mqtt2_port_identifier] = mqttSettings.port2;
      doc[mqttSettings.wm_mqtt2_user_identifier] = mqttSettings.user2;
      doc[mqttSettings.wm_mqtt2_password_identifier] = mqttSettings.password2;
      doc[mqttSettings.wm_mqtt2_basetopic_identifier] = mqttSettings.baseTopic2;
      // Persist cascade settings
      doc[mqttSettings.cascade_enabled_identifier] = mqttSettings.cascadeEnabled;
      doc[mqttSettings.cascade_node_id_identifier] = mqttSettings.cascadeNodeId;
      doc[mqttSettings.cascade_base_topic_identifier] = mqttSettings.cascadeBaseTopic;
      doc["debug_enable_telnet"] = gRequestedTelnetDebug;
      // cascade test mode removed
      // Persist installation flags (string values)
      doc[mqttSettings.wm_boiler_installed_identifier] = mqttSettings.boiler_installed;
      doc[mqttSettings.wm_booster1_installed_identifier] = mqttSettings.booster1_installed;
      doc[mqttSettings.wm_booster2_installed_identifier] = mqttSettings.booster2_installed;
      doc[unitSettings.unitsize_identifier] = unitSettings.UnitSize;
      doc[unitSettings.glycol_identifier] = unitSettings.GlycolStrength;
      doc[unitSettings.compcurve_identifier] = unitSettings.CompCurve;

      if (serializeJson(doc, configFile) == 0) {
        DEBUG_PRINTLN(F("[FAILED]"));
      } else {
        DEBUG_PRINTLN(F("[DONE]"));
        DEBUG_PRINTLN(F("Restarting Web Server and mDNS..."));
        wifiManager.stopWebPortal();
        wifiManager.startWebPortal();
        MDNS.begin("heatpump");
        MDNS.addService("http", "tcp", 80);
        DEBUG_PRINTLN();
      }
    }
    configFile.close();
    shouldSaveConfig = false;
  }

  // callback notifying us of the need to save config
  void saveConfigCallback() { saveConfig(); }

  void initializeWifiManager() {
    DEBUG_PRINTLN(F("Starting WiFi Manager"));
    // Reset Wifi settings for testing
    // wifiManager.resetSettings();
    // wifiManager.setDebugOutput(true);
    wifiManager.setTitle("Ecodan Bridge");

    // Set or Update the values
    custom_device_id.setValue(mqttSettings.deviceId, deviceId_max_length);
    custom_mqtt_server.setValue(mqttSettings.hostname, hostname_max_length);
    custom_mqtt_port.setValue(mqttSettings.port, port_max_length);
    custom_mqtt_user.setValue(mqttSettings.user, user_max_length);
    custom_mqtt_pass.setValue(mqttSettings.password, password_max_length);
    custom_mqtt_basetopic.setValue(mqttSettings.baseTopic,
                                   basetopic_max_length);
    custom_mqtt2_server.setValue(mqttSettings.hostname2, hostname_max_length);
    custom_mqtt2_port.setValue(mqttSettings.port2, port_max_length);
    custom_mqtt2_user.setValue(mqttSettings.user2, user_max_length);
    custom_mqtt2_pass.setValue(mqttSettings.password2, password_max_length);
    custom_mqtt2_basetopic.setValue(mqttSettings.baseTopic2,
                                    basetopic_max_length);
    custom_cascade_basetopic.setValue(mqttSettings.cascadeBaseTopic,
                                      basetopic_max_length);
    // Install flags
    custom_boiler_installed.setValue(mqttSettings.boiler_installed, 6);
    custom_booster1_installed.setValue(mqttSettings.booster1_installed, 6);
    custom_booster2_installed.setValue(mqttSettings.booster2_installed, 6);
    
    // Set cascade parameter value for checkbox submission (always 'true' when checked)
    custom_cascade_enabled.setValue("true", 6);
    char nodeIdStr[5];
    itoa(mqttSettings.cascadeNodeId, nodeIdStr, 10);
    custom_cascade_node_id.setValue(nodeIdStr, 5);
    // cascade test mode removed

    // Add the custom MQTT parameters here
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_basetopic);
    wifiManager.addParameter(&custom_mqtt2_server);
    wifiManager.addParameter(&custom_mqtt2_user);
    wifiManager.addParameter(&custom_mqtt2_pass);
    wifiManager.addParameter(&custom_mqtt2_port);
    wifiManager.addParameter(&custom_mqtt2_basetopic);
    // Add configuration section fields before cascade section
    wifiManager.addParameter(&custom_boiler_installed);
    wifiManager.addParameter(&custom_booster1_installed);
    wifiManager.addParameter(&custom_booster2_installed);
    wifiManager.addParameter(&custom_cascade_enabled);
    wifiManager.addParameter(&custom_cascade_node_id);
    wifiManager.addParameter(&custom_cascade_basetopic);
    // cascade test mode removed
    wifiManager.addParameter(&custom_device_id);

    // set minimum quality of signal so it ignores AP's under that quality
    // defaults to 8%
    // wifiManager.setMinimumSignalQuality(20);

    snprintf(WiFiHostname, 40, "%s%s", ClientPrefix, mqttSettings.deviceId);
    WiFi.hostname(WiFiHostname);
#ifdef ESP8266                        // Define the Witty ESP8266 Ports
    digitalWrite(Blue_RGB_LED, HIGH); // Turn the Blue LED On
#endif
    wifiManager.setConfigPortalBlocking(false); // Non-Blocking portal
    wifiManager.setBreakAfterConfig(true); // Saves settings, even if WiFi Fails
    wifiManager.setSaveConfigCallback(
        saveConfigCallback); // Set config save callback
    wifiManager.setAPClientCheck(
        true); // Avoid timeout if client connected to softap

#ifndef ARDUINO_WT32_ETH01
    wifiManager.setConfigPortalTimeout(
        600); // Timeout before launching the config portal (WiFi Only)
    if (!wifiManager.autoConnect("Ecodan Bridge AP")) {
      DEBUG_PRINTLN(F("Failed to connect and hit timeout"));
    } else {
      DEBUG_PRINTLN(F("WiFi Connected!"));
    }
#endif
  }

  void PublishDiscoveryTopics(uint8_t MQTTStream, String BASETOPIC) {

    // Compile Topics
    String MQTT_DISCOVERY_TOPIC, Buffer_Topic;
    int j;

// -- Entities Configuration JSON -- //
#ifdef ESP8266
    String ChipModel = "ESP8266";
#endif
#ifdef ESP32
    String ChipModel = ESP.getChipModel();
#endif

    String ChipID = mqttSettings.deviceId;

    // JSON Formation
    JsonDocument Config;

    // Publish all the discovery topics
    for (int i = 0; i < discovery_topics; i++) {


      // If device is not 2-zone, remove all Zone 2 entities by name
      if (!Flags::Has2Zone()) {
        String sensorName = String(MQTT_SENSOR_NAME[i]);
        if (sensorName.indexOf("Zone 2") >= 0) {
          int topicPrefixIndex = 0; // default to sensor
          if (i >= 96 && i < 101) topicPrefixIndex = 1;      // climate
          else if (i >= 101 && i < 111) topicPrefixIndex = 2; // switch
          else if (i >= 111 && i < 116) topicPrefixIndex = 4; // select
          String delTopic = String(MQTT_DISCOVERY_TOPICS[topicPrefixIndex]) +
                            ChipID + String(MQTT_DISCOVERY_OBJ_ID[i]) +
                            String(MQTT_DISCOVERY_TOPICS[5]);
          if (MQTTStream == 1) {
            MQTTClient1.publish(delTopic.c_str(), "", true);
          } else if (MQTTStream == 2) {
            MQTTClient2.publish(delTopic.c_str(), "", true);
          }
          continue;
        }
      }

      // If device has no cooling, remove cooling-related sensors/switches
      if (!Flags::HasCooling()) {
        String sensorName = String(MQTT_SENSOR_NAME[i]);
        bool coolingName = (sensorName.indexOf("Cooling") >= 0);
        // Only purge non-climate/select entries; climate retains heat/off modes
        bool isSensor = (i >= 0 && i < 96);
        bool isSwitch = (i >= 101 && i < 111);
        if (coolingName && (isSensor || isSwitch)) {
          int topicPrefixIndex = isSwitch ? 2 : 0; // switch or sensor
          String delTopic = String(MQTT_DISCOVERY_TOPICS[topicPrefixIndex]) +
                            ChipID + String(MQTT_DISCOVERY_OBJ_ID[i]) +
                            String(MQTT_DISCOVERY_TOPICS[5]);
          if (MQTTStream == 1) {
            MQTTClient1.publish(delTopic.c_str(), "", true);
          } else if (MQTTStream == 2) {
            MQTTClient2.publish(delTopic.c_str(), "", true);
          }
          continue;
        }
      }

      // If boiler/boosters are not installed, remove related entities by name
      {
        String sensorName = String(MQTT_SENSOR_NAME[i]);
        // Boiler temperatures
        bool removeForBoiler = (strcmp(mqttSettings.boiler_installed, "false") == 0) &&
                               (sensorName == "Boiler Flow Temperature" || sensorName == "Boiler Return Temperature");
        // Booster heaters
        bool removeBooster1 = (strcmp(mqttSettings.booster1_installed, "false") == 0) &&
                              (sensorName == "Booster Heater 1");
        bool removeBooster2 = (strcmp(mqttSettings.booster2_installed, "false") == 0) &&
                              (sensorName == "Booster Heater 2");
        if (removeForBoiler || removeBooster1 || removeBooster2) {
          // All of these are standard sensors (not climate/select/switch)
          String delTopic = String(MQTT_DISCOVERY_TOPICS[0]) + ChipID +
                            String(MQTT_DISCOVERY_OBJ_ID[i]) +
                            String(MQTT_DISCOVERY_TOPICS[5]);
          if (MQTTStream == 1) {
            MQTTClient1.publish(delTopic.c_str(), "", true);
          } else if (MQTTStream == 2) {
            MQTTClient2.publish(delTopic.c_str(), "", true);
          }
          continue;
        }
      }

      // If cascade is enabled and this is a slave node, remove selected controls
      if (Flags::ConfigCascadeSlave()) {
        String name = String(MQTT_SENSOR_NAME[i]);
        bool match = false;
        // Thermostats (DHW/Z1/Z2 and flow thermostats)
        if (name.indexOf("Thermostat") >= 0) match = true;
        // Operation modes for zones and DHW
        if (name == "DHW Mode" ||
            name == "Heating/Cooling Operation Mode Zone 1" ||
            name == "Heating/Cooling Operation Mode Zone 2") match = true;
        // System/Server controls
        if (name == "System Power" || name == "Server Control Mode" || name == "Holiday Mode") match = true;
        // DHW boost controls
        if (name == "DHW Boost" || name == "Fast DHW Boost") match = true;
        // Prohibit controls (DHW, Z1/Z2 Heating/Cooling)
        if (name == "Prohibit DHW" || name == "Prohibit Zone 1 Heating" ||
            name == "Prohibit Zone 1 Cooling" || name == "Prohibit Zone 2 Heating" ||
            name == "Prohibit Zone 2 Cooling") match = true;
        // Immersion Heater, Booster Heaters
        if (name == "Immersion Heater" || name == "Booster Heater 1" ||
            name == "Booster Heater 2") match = true;
        // Max/Min Flow Temperature
        if (name == "Max Flow Temperature" || name == "Min Flow Temperature") match = true;
        // Boiler Flow/Return Temperatures
        if (name == "Boiler Flow Temperature" || name == "Boiler Return Temperature") match = true;
        // DHW sensors
        if (name == "DHW Control Mode" || name == "DHW Temperature" ||
            name == "DHW Temperature Upper" || name == "Legionella Setpoint" ||
            name == "DHW Max Temperature Drop") match = true;
        // Flow setpoints sensors
        if (name == "Zone 1 Flow Setpoint" || name == "Zone 2 Flow Setpoint") match = true;
        // Unit size and Glycol strength selects
        if (name == "Outdoor Unit Size (kW)" || name == "Glycol Strength") match = true;
        // Mixing tank/valve are not supported on cascade slaves
        if (name == "Mixing Tank Temperature" || name == "Mixing Valve Step") match = true;

        if (match) {
          int topicPrefixIndex = 0; // sensor by default
          if (i >= 96 && i < 101) topicPrefixIndex = 1;      // climate
          else if (i >= 101 && i < 111) topicPrefixIndex = 2; // switch
          else if (i >= 111 && i < 116) topicPrefixIndex = 4; // select

          String delTopic = String(MQTT_DISCOVERY_TOPICS[topicPrefixIndex]) +
                            ChipID + String(MQTT_DISCOVERY_OBJ_ID[i]) +
                            String(MQTT_DISCOVERY_TOPICS[5]);
          if (MQTTStream == 1) {
            MQTTClient1.publish(delTopic.c_str(), "", true);
          } else if (MQTTStream == 2) {
            MQTTClient2.publish(delTopic.c_str(), "", true);
          }
          continue;
        }
      }
      // If cascade mode is enabled, remove CoP- and energy-related sensors by name
      // (any name containing 'Computed', 'Consumed', 'Delivered', or exact 'Instant CoP',
      //  'Heating CoP Yesterday', 'Cooling CoP Yesterday', 'DHW CoP Yesterday',
      //  'Total CoP Yesterday', as well as 'Heat Pump Input Power' and
      //  'Heat Pump Output Power')
      if (Flags::CascadeEnabled()) {
        String sensorName = String(MQTT_SENSOR_NAME[i]);
        bool nameMatch = (sensorName.indexOf("Computed") >= 0 ||
                          sensorName.indexOf("Consumed") >= 0 ||
                          sensorName.indexOf("Delivered") >= 0 ||
                          sensorName == "Instant CoP" ||
                          sensorName == "Heat Pump Input Power" ||
                          sensorName == "Heat Pump Output Power" ||
                          sensorName == "Heating CoP Yesterday" ||
                          sensorName == "Cooling CoP Yesterday" ||
                          sensorName == "DHW CoP Yesterday" ||
                          sensorName == "Total CoP Yesterday");
        if (nameMatch) {
          String delTopic = String(MQTT_DISCOVERY_TOPICS[0]) + ChipID +
                            String(MQTT_DISCOVERY_OBJ_ID[i]) +
                            String(MQTT_DISCOVERY_TOPICS[5]);
          if (MQTTStream == 1) {
            MQTTClient1.publish(delTopic.c_str(), "", true);
          } else if (MQTTStream == 2) {
            MQTTClient2.publish(delTopic.c_str(), "", true);
          }
          continue;
        }
      }

      // If cascade mode is enabled, remove Heating CoP and DHW CoP discovery
      // entities by publishing empty retained configs. These correspond to
      // indices 47 (Heating CoP Yesterday) and 49 (DHW CoP Yesterday).
      if (Flags::CascadeEnabled() && (i == 47 || i == 49)) {
        String delTopic = String(MQTT_DISCOVERY_TOPICS[0]) + ChipID +
                          String(MQTT_DISCOVERY_OBJ_ID[i]) +
                          String(MQTT_DISCOVERY_TOPICS[5]);
        if (MQTTStream == 1) {
          MQTTClient1.publish(delTopic.c_str(), "", true);
        } else if (MQTTStream == 2) {
          MQTTClient2.publish(delTopic.c_str(), "", true);
        }
        continue;
      }

      // When cascade is enabled on the master node (nodeId == 0),
      // remove previously discovered Compressor Frequency, Flow Rate, Run Hours
      // (indices 10, 11, 12) and Compressor Start Quantity (by name match)
      // by publishing an empty retained config on their discovery topics.
      if (Flags::ConfigCascadeMaster() &&
          (i == 10 || i == 11 || i == 12 || String(MQTT_SENSOR_NAME[i]) == "Compressor Start Quantity")) {
        String delTopic = String(MQTT_DISCOVERY_TOPICS[0]) + ChipID +
                          String(MQTT_DISCOVERY_OBJ_ID[i]) +
                          String(MQTT_DISCOVERY_TOPICS[5]);
        if (MQTTStream == 1) {
          MQTTClient1.publish(delTopic.c_str(), "", true);
        } else if (MQTTStream == 2) {
          MQTTClient2.publish(delTopic.c_str(), "", true);
        }
        continue;
      }

      // If cascade is configured as master, do not expose Fan 1/2 Speed sensors.
      // Publish an empty retained config to remove any previously discovered entity
      // and skip re-publishing in this loop.
      if (Flags::ConfigCascadeMaster()) {
        String sname = String(MQTT_SENSOR_NAME[i]);
        if (sname == "Fan 1 Speed" || sname == "Fan 2 Speed") {
          String delTopic = String(MQTT_DISCOVERY_TOPICS[0]) + ChipID +
                            String(MQTT_DISCOVERY_OBJ_ID[i]) +
                            String(MQTT_DISCOVERY_TOPICS[5]);
          if (MQTTStream == 1) {
            MQTTClient1.publish(delTopic.c_str(), "", true);
          } else if (MQTTStream == 2) {
            MQTTClient2.publish(delTopic.c_str(), "", true);
          }
          continue;
        }
      }

      if (i == 0) { // If the first topic
        Config["device"]["identifiers"] = WiFiHostname;
        Config["device"]["manufacturer"] = "F1p";
        Config["device"]["model"] = ChipModel;
        Config["device"]["serial_number"] = ChipID;
        Config["device"]["name"] = "Ecodan ASHP";
#ifdef ARDUINO_WT32_ETH01
        Config["device"]["configuration_url"] =
            "http://" + ETH.localIP().toString() + ":80";
#else
      Config["device"]["configuration_url"] =
          "http://" + WiFi.localIP().toString() + ":80";
#endif
        Config["device"]["sw_version"] = FirmwareVersion;
      } else { // Otherwise post just identifier
        Config["device"]["identifiers"] = WiFiHostname;
      }

      // Every one has a unique_id and name
      Config["unique_id"] = String(MQTT_SENSOR_UNIQUE_ID[i]) + ChipID;
      Config["name"] = String(MQTT_SENSOR_NAME[i]);

      // Sensors
      if (i >= 0 && i < 96) {
        Config["stat_t"] =
            BASETOPIC +
            String(MQTT_TOPIC[MQTT_TOPIC_POS[i]]); // Needs a positioner
        if (MQTT_UNITS_POS[i] > 0) {               // If there is a unit
          Config["unit_of_meas"] =
              String(MQTT_SENSOR_UNITS[MQTT_UNITS_POS[i]]); // Publish Units
          if (MQTT_UNITS_POS[i] < 8) {
            Config["dev_cla"] = String(MQTT_DEVICE_CLASS[MQTT_UNITS_POS[i]]);
          } // Device classes only exist for some units
          if (MQTT_UNITS_POS[i] != 7) {
            Config["stat_cla"] = "measurement";
          } // Only some can be measurement
        }
        Config["val_tpl"] = String(MQTT_SENSOR_VALUE_TEMPLATE[i]);
        Config["icon"] = String(MQTT_MDI_ICONS[i]);

        MQTT_DISCOVERY_TOPIC = String(MQTT_DISCOVERY_TOPICS[0]);
      }

      // Climate
      if (i >= 96 && i < 101) {
        Config["object_id"] = String(MQTT_OBJECT_ID[i - 96]);
        if (i >= 96 && i < 99) {
          Config["curr_temp_t"] = BASETOPIC + String(MQTT_TOPIC[i - 92]);
          Config["curr_temp_tpl"] = String(MQTT_SENSOR_VALUE_TEMPLATE[25]);
          Config["temp_cmd_t"] = BASETOPIC + String(MQTT_TOPIC[i - 85]);
          Config["temp_stat_t"] = BASETOPIC + String(MQTT_TOPIC[i - 92]);
          Config["temp_stat_tpl"] = String(MQTT_SENSOR_VALUE_TEMPLATE[97]);
        } else if (i >= 99 && i < 101) {
          Config["curr_temp_t"] = BASETOPIC + String(MQTT_TOPIC[2]);
          Config["curr_temp_tpl"] = String(MQTT_SENSOR_VALUE_TEMPLATE[6]);
          Config["temp_cmd_t"] = BASETOPIC + String(MQTT_TOPIC[i - 75]);
          Config["temp_stat_t"] = BASETOPIC + String(MQTT_TOPIC[i - 94]);
          Config["temp_stat_tpl"] = String(MQTT_SENSOR_VALUE_TEMPLATE[98]);
        }
        Config["temp_unit"] = String(MQTT_SENSOR_UNITS[9]);
        if (HeatPump.Status.RefrigerantType == 2 &&
            i == 96) { // If R290 then DHW Max can be 70C
          Config["max_temp"] = MQTT_CLIMATE_MAX[6];
        } else {
          Config["max_temp"] = MQTT_CLIMATE_MAX[i - 96];
        }

        Config["min_temp"] = MQTT_CLIMATE_MIN[i - 96];
        Config["temp_step"] = MQTT_CLIMATE_TEMP_STEP[i - 96];
        Config["precision"] = MQTT_CLIMATE_PRECISION[i - 96];
        Config["initial"] = MQTT_CLIMATE_INITAL[i - 96];
        Config["action_topic"] = BASETOPIC + String(MQTT_TOPIC[2]);
        Config["action_template"] =
            String(MQTT_CLIMATE_MODE_STATE_TEMPLATE[i - 96]);
        Config["mode_state_topic"] = BASETOPIC + String(MQTT_TOPIC[8]);
        Config["mode_state_template"] =
            String(MQTT_CLIMATE_STATE_TOPIC[i - 96]);
        if (i == 96) {
          Config["modes"][0] = "heat";
          Config["modes"][1] = "off";
        } else {
          Config["modes"][0] = "heat";
          int m = 1;
          if (Flags::HasCooling()) {
            Config["modes"][m++] = "cool";
          }
          Config["modes"][m] = "off";
          Config["mode_command_template"] = String(MQTT_CLIMATE_MODE[0]);
          Config["mode_command_topic"] = BASETOPIC + String(MQTT_TOPIC[9]);
        }

        MQTT_DISCOVERY_TOPIC = String(MQTT_DISCOVERY_TOPICS[1]);
      }

      // Switches
      if (i >= 101 && i < 111) {
        Config["state_topic"] =
            BASETOPIC + String(MQTT_TOPIC[MQTT_SWITCH_STATE_POS[i - 101]]);
        Config["value_template"] = String(MQTT_SENSOR_VALUE_TEMPLATE[i - 2]);
        Config["command_topic"] = BASETOPIC + String(MQTT_TOPIC[i - 87]);
        if (i == 102) {
          Config["state_on"] = "On";
          Config["state_off"] = "Standby";
          Config["payload_on"] = "On";
          Config["payload_off"] = "Standby";
        } else {
          Config["state_on"] = ITEM_ON;
          Config["state_off"] = ITEM_OFF;
          Config["payload_on"] = ITEM_ON;
          Config["payload_off"] = ITEM_OFF;
        }
        Config["icon"] = String(MQTT_MDI_ICONS[i]);

        MQTT_DISCOVERY_TOPIC = String(MQTT_DISCOVERY_TOPICS[2]);
      }

      // Selects
      if (i >= 111 && i < 116) {
        Config["command_topic"] = BASETOPIC + String(MQTT_TOPIC[i - 85]);
        Config["state_topic"] = BASETOPIC + String(MQTT_TOPIC[i - 107]);
        Config["value_template"] = String(MQTT_SELECT_VALUE_TEMPLATE[i - 111]);
        if (i == 111) { // DHW Modes
          Config["options"][0] = HotWaterControlModeString[0];
          Config["options"][1] = HotWaterControlModeString[1];
        } else if (i == 114) { // Unit Sizes - for some reason it doesn't like
                               // doing this from PROGMEM in a loop on the 8266
          Config["state_topic"] = BASETOPIC + String(MQTT_TOPIC[1]);
          Config["options"][0] = "4.0";
          Config["options"][1] = "5.0";
          Config["options"][2] = "6.0";
          Config["options"][3] = "7.5";
          Config["options"][4] = "8.0";
          Config["options"][5] = "8.5";
          Config["options"][6] = "10.0";
          Config["options"][7] = "11.2";
          Config["options"][8] = "12.0";
          Config["options"][9] = "14.0";
        } else if (i == 115) { // Glycol Strengths
          Config["state_topic"] = BASETOPIC + String(MQTT_TOPIC[1]);
          Config["options"][0] = "0%";
          Config["options"][1] = "10%";
          Config["options"][2] = "20%";
          Config["options"][3] = "30%";
        } else { // Zone Options
          int idx = 0;
          Config["options"][idx++] = "Heating Temperature";
          Config["options"][idx++] = "Heating Flow";
          Config["options"][idx++] = "Heating Compensation";
          if (Flags::HasCooling()) {
            Config["options"][idx++] = "Cooling Temperature";
            Config["options"][idx++] = "Cooling Flow";
          }
          Config["options"][idx++] = "Dry Up";
        }
        MQTT_DISCOVERY_TOPIC = String(MQTT_DISCOVERY_TOPICS[4]);
      }

      // Add Availability Topics
      if (i >= 97) {
        if (i >= 105 && i < 110) { // Server Control Mode Interlocks
          Config["availability"]["topic"] = BASETOPIC + String(MQTT_TOPIC[8]);
          Config["availability"]["value_template"] =
              String(MQTT_SENSOR_VALUE_TEMPLATE[102]);
          Config["availability"]["payload_available"] = ITEM_ON;
          Config["availability"]["payload_not_available"] = ITEM_OFF;
        } else if (i >= 99 &&
                   i < 101) { // Flow Op Mode Interlocks on Climate & Number
          Config["availability"]["topic"] =
              BASETOPIC + String(MQTT_TOPIC[i - 94]);
          Config["availability"]["value_template"] =
              String(MQTT_NUMBER_AVAIL_TEMPLATE[0]);
        } else { // Everything else LWT
          Config["availability"]["topic"] = BASETOPIC + String(MQTT_TOPIC[0]);
        }
      }

      char Buffer_Payload[2048];
      size_t buf_size = serializeJson(Config, Buffer_Payload);
      Buffer_Topic = MQTT_DISCOVERY_TOPIC + ChipID +
                     String(MQTT_DISCOVERY_OBJ_ID[i]) +
                     String(MQTT_DISCOVERY_TOPICS[5]);

      if (MQTTStream == 1) {
        MQTTClient1.publish(Buffer_Topic.c_str(), (uint8_t *)&Buffer_Payload,
                            buf_size, true);
      } else if (MQTTStream == 2) {
        MQTTClient2.publish(Buffer_Topic.c_str(), (uint8_t *)&Buffer_Payload,
                            buf_size, true);
      }

      MQTT_DISCOVERY_TOPIC =
          ""; // Clear everything ready for next loop to save RAM
      Buffer_Topic = "";
      Config.clear();
    }

    // Additional discovery: Cascade Mode status (text sensor)
    // Publishes the cascade role as Disabled/Master/Slave using System status JSON
    {
      JsonDocument cfg;
      // Minimal device info for correct grouping in HA
      cfg["device"]["identifiers"] = WiFiHostname;
      cfg["unique_id"] = String("ashp_cascade_mode_") + ChipID;
      cfg["name"] = "Cascade Mode";
      cfg["stat_t"] = BASETOPIC + String(MQTT_TOPIC[2]); // /Status/System
      cfg["val_tpl"] = "{{ value_json.CascadeMode }}";
      cfg["icon"] = "mdi:source-branch";
      // LWT availability
      cfg["availability"]["topic"] = BASETOPIC + String(MQTT_TOPIC[0]);

      char payload[512];
      size_t sz = serializeJson(cfg, payload);
      String discTopic = String(MQTT_DISCOVERY_TOPICS[0]) + ChipID +
                         "cascade_mode" + String(MQTT_DISCOVERY_TOPICS[5]);
      if (MQTTStream == 1) {
        MQTTClient1.publish(discTopic.c_str(), (uint8_t *)payload, sz, true);
      } else if (MQTTStream == 2) {
        MQTTClient2.publish(discTopic.c_str(), (uint8_t *)payload, sz, true);
      }
    }

    // Additional discovery: Cascade Node ID (sensor)
    {
      JsonDocument cfg;
      cfg["device"]["identifiers"] = WiFiHostname;
      cfg["unique_id"] = String("ashp_cascade_node_id_") + ChipID;
      cfg["name"] = "Cascade Node ID";
      cfg["stat_t"] = BASETOPIC + String(MQTT_TOPIC[2]); // /Status/System
      cfg["val_tpl"] = "{{ value_json.CascadeNodeId }}";
      cfg["icon"] = "mdi:numeric";
      cfg["availability"]["topic"] = BASETOPIC + String(MQTT_TOPIC[0]);

      char payload[512];
      size_t sz = serializeJson(cfg, payload);
      String discTopic = String(MQTT_DISCOVERY_TOPICS[0]) + ChipID +
                         "cascade_node_id" + String(MQTT_DISCOVERY_TOPICS[5]);
      if (MQTTStream == 1) {
        MQTTClient1.publish(discTopic.c_str(), (uint8_t *)payload, sz, true);
      } else if (MQTTStream == 2) {
        MQTTClient2.publish(discTopic.c_str(), (uint8_t *)payload, sz, true);
      }
    }

    // Generate Publish Message
    DEBUG_PRINTLN(F("Published Discovery Topics!"));
  }

  void initializeMQTTClient1() {
    DEBUG_PRINT(F("Attempting MQTT connection to: "));
    DEBUG_PRINT(mqttSettings.hostname);
    DEBUG_PRINT(F(":"));
    DEBUG_PRINTLN(mqttSettings.port);
    MQTTClient1.setServer(mqttSettings.hostname, atoi(mqttSettings.port));
  }

  void MQTTonConnect(void) {
    DEBUG_PRINTLN(F("MQTT ON CONNECT"));
    MQTTClient1.publish(MQTT_LWT.c_str(), "online");
    delay(10);
    bool isCascadeSlave = Flags::ConfigCascadeSlave();

    if (!isCascadeSlave) {
      MQTTClient1.subscribe(MQTTCommandZone1FlowSetpoint.c_str());
      MQTTClient1.subscribe(MQTTCommandZone1NoModeSetpoint.c_str());
    }
    if (!isCascadeSlave) {
      MQTTClient1.subscribe(MQTTCommandZone1ProhibitHeating.c_str());
      if (Flags::HasCooling()) {
        MQTTClient1.subscribe(MQTTCommandZone1ProhibitCooling.c_str());
      }
    }
    if (!isCascadeSlave) {
      MQTTClient1.subscribe(MQTTCommandZone1HeatingMode.c_str());
    }
    if (Flags::Has2Zone()) {
      if (!isCascadeSlave) {
        MQTTClient1.subscribe(MQTTCommandZone2FlowSetpoint.c_str());
        MQTTClient1.subscribe(MQTTCommandZone2NoModeSetpoint.c_str());
      }
      if (!isCascadeSlave) {
        MQTTClient1.subscribe(MQTTCommandZone2ProhibitHeating.c_str());
        if (Flags::HasCooling()) {
          MQTTClient1.subscribe(MQTTCommandZone2ProhibitCooling.c_str());
        }
      }
      if (!isCascadeSlave) {
        MQTTClient1.subscribe(MQTTCommandZone2HeatingMode.c_str());
      }
    }
    if (!isCascadeSlave) {
      MQTTClient1.subscribe(MQTTCommandHotwaterMode.c_str());
      MQTTClient1.subscribe(MQTTCommandHotwaterSetpoint.c_str());
      MQTTClient1.subscribe(MQTTCommandHotwaterBoost.c_str());
      MQTTClient1.subscribe(MQTTCommandHotwaterNormalBoost.c_str());
    }
    if (!isCascadeSlave) {
      MQTTClient1.subscribe(MQTTCommandHotwaterProhibit.c_str());
    }
    // Subscribe to Holiday Mode even in cascade; restrict to master only
    if (!isCascadeSlave) {
      MQTTClient1.subscribe(MQTTCommandSystemHolidayMode.c_str());
    }
    if (!isCascadeSlave) {
      MQTTClient1.subscribe(MQTTCommandSystemPower.c_str());
      MQTTClient1.subscribe(MQTTCommandSystemSvrMode.c_str());
    }
    if (!isCascadeSlave) {
      MQTTClient1.subscribe(MQTTCommandSystemUnitSize.c_str());
    }
    if (!isCascadeSlave) {
      MQTTClient1.subscribe(MQTTCommandSystemGlycol.c_str());
    }
    MQTTClient1.subscribe(MQTTCommandSystemService.c_str());
    MQTTClient1.subscribe(MQTTCommandSystemCompCurve.c_str());
    // Subscribe to Debug enable toggles on primary broker
    MQTTClient1.subscribe(MQTT_DEBUG_ENABLE.c_str());
    MQTTClient1.subscribe(MQTT_DEBUG_ENABLE_SERIAL.c_str());
    MQTTClient1.subscribe(MQTT_DEBUG_ENABLE_TELNET.c_str());

    delay(10);
    PublishDiscoveryTopics(1, MQTT_BASETOPIC);
    publishDebugEnableStates();

#ifdef ESP8266                      // Define the Witty ESP8266 Ports
    analogWrite(Green_RGB_LED, 30); // Green LED on, 25% brightness
    digitalWrite(Red_RGB_LED, LOW); // Turn the Red LED Off
#endif
#ifdef ARDUINO_M5STACK_ATOMS3      // Define the M5Stack LED
    myLED.setPixel(0, L_GREEN, 1); // set the LED colour and show it
    myLED.brightness(LED_BRIGHT, 1);
#endif
  }

  uint8_t MQTTReconnect() {
    if (MQTTClient1.connected()) {
      return 1;
    }
#ifdef ARDUINO_WT32_ETH01
    else if (strcmp(mqttSettings.hostname, "IPorDNS") != 0 && strcmp(mqttSettings.hostname, "") != 0) {  // Do not block MQTT attempt on Ethernet
#else
  else if (strcmp(mqttSettings.hostname, "IPorDNS") != 0 && strcmp(mqttSettings.hostname, "") != 0 && WiFi.status() == WL_CONNECTED) {  // WiFi should be active to attempt connections (as MQTT connect is blocking)
#endif
      initializeMQTTClient1();
      DEBUG_PRINT(F("With Client ID: "));
      DEBUG_PRINT(mqttSettings.deviceId);
      DEBUG_PRINT(F(", Username: "));
      DEBUG_PRINT(mqttSettings.user);
      DEBUG_PRINT(F(" and Password: "));
      DEBUG_PRINTLN(mqttSettings.password);

      if (MQTTClient1.connect(mqttSettings.deviceId, mqttSettings.user,
                              mqttSettings.password, MQTT_LWT.c_str(), 0, true,
                              "offline")) {
        DEBUG_PRINTLN(F("MQTT Server Connected"));
        MQTTonConnect();
#ifdef ESP8266                             // Define the Witty ESP8266 Ports
        digitalWrite(Red_RGB_LED, LOW);    // Turn off the Red LED
        digitalWrite(Green_RGB_LED, HIGH); // Flash the Green LED
        delay(10);
        analogWrite(Green_RGB_LED, 30); // Green LED on, 25% brightness
#endif
        return 1;
      } else {
#ifdef ARDUINO_M5STACK_ATOMS3                       // Define the M5Stack LED
        if (!wifiManager.getConfigPortalActive()) { // Not got config portal
                                                    // open, change to orange:
          myLED.setPixel(0, L_ORANGE, 1); // set the LED colour and show it
        }
        //
#endif
        switch (MQTTClient1.state()) {
        case -4:
          DEBUG_PRINTLN(F("MQTT_CONNECTION_TIMEOUT"));
          break;
        case -3:
          DEBUG_PRINTLN(F("MQTT_CONNECTION_LOST"));
          break;
        case -2:
          DEBUG_PRINTLN(F("MQTT_CONNECT_FAILED"));
          break;
        case -1:
          DEBUG_PRINTLN(F("MQTT_DISCONNECTED"));
          break;
        case 0:
          DEBUG_PRINTLN(F("MQTT_CONNECTED"));
          break;
        case 1:
          DEBUG_PRINTLN(F("MQTT_CONNECT_BAD_PROTOCOL"));
          break;
        case 2:
          DEBUG_PRINTLN(F("MQTT_CONNECT_BAD_CLIENT_ID"));
          break;
        case 3:
          DEBUG_PRINTLN(F("MQTT_CONNECT_UNAVAILABLE"));
          break;
        case 4:
          DEBUG_PRINTLN(F("MQTT_CONNECT_BAD_CREDENTIALS"));
          break;
        case 5:
          DEBUG_PRINTLN(F("MQTT_CONNECT_UNAUTHORIZED"));
          break;
        }
        return 0;
      }
      return 0;
    } else {
      DEBUG_PRINTLN(F("Primary MQTT Not Set or WiFi not connected"));
      return 0;
    }
  }

  void handleMQTTState() {
    if (!MQTTClient1.connected()) {
#ifdef ARDUINO_M5STACK_ATOMS3                     // Define the M5Stack LED
      if (!wifiManager.getConfigPortalActive()) { // Not got config portal open,
                                                  // change to orange:
        myLED.setPixel(0, L_ORANGE, 1); // set the LED colour and show it
      }
#endif
#ifdef ESP8266                        // Define the Witty ESP8266 Ports
      analogWrite(Green_RGB_LED, 30); // Green LED on, 25% brightness
      digitalWrite(Red_RGB_LED,
                   HIGH); // Add the Red LED to the Green LED = Orange
#endif
      MQTTReconnect();
      delay(10);
    }
  }

  void RecalculateMQTT2Topics() {
    // The base topic may change via WiFi Manager
    MQTT_2_LWT = MQTT_2_BASETOPIC + "/LWT";
    MQTT_2_STATUS = MQTT_2_BASETOPIC + "/Status";
    MQTT_2_COMMAND = MQTT_2_BASETOPIC + "/Command";

    MQTT_2_STATUS_ZONE1 = MQTT_2_STATUS + "/Zone1";
    MQTT_2_STATUS_ZONE2 = MQTT_2_STATUS + "/Zone2";
    MQTT_2_STATUS_HOTWATER = MQTT_2_STATUS + "/HotWater";
    MQTT_2_STATUS_SYSTEM = MQTT_2_STATUS + "/System";
    MQTT_2_STATUS_CONFIGURATION = MQTT_2_STATUS + "/Configuration";
    MQTT_2_STATUS_ADVANCED = MQTT_2_STATUS + "/Advanced";
    MQTT_2_STATUS_ADVANCED_TWO = MQTT_2_STATUS + "/AdvancedTwo";
    MQTT_2_STATUS_ENERGY = MQTT_2_STATUS + "/Energy";
    MQTT_2_STATUS_WIFISTATUS = MQTT_2_STATUS + "/WiFiStatus";
    MQTT_2_STATUS_CURVE = MQTT_2_STATUS + "/CompCurve";
    // Debug topics for secondary base
    MQTT_2_DEBUG = MQTT_2_BASETOPIC + "/Debug";
    MQTT_2_DEBUG_CALC = MQTT_2_DEBUG + "/CalculateCompCurve";
    MQTT_2_DEBUG_ENABLE = MQTT_2_DEBUG + "/Enable/MQTT";
    MQTT_2_DEBUG_ENABLE_SERIAL = MQTT_2_DEBUG + "/Enable/Serial";
    MQTT_2_DEBUG_ENABLE_TELNET = MQTT_2_DEBUG + "/Enable/Telnet";

    MQTT_2_COMMAND_ZONE1 = MQTT_2_COMMAND + "/Zone1";
    MQTT_2_COMMAND_ZONE2 = MQTT_2_COMMAND + "/Zone2";
    MQTT_2_COMMAND_HOTWATER = MQTT_2_COMMAND + "/HotWater";
    MQTT_2_COMMAND_SYSTEM = MQTT_2_COMMAND + "/System";
    MQTT_2_COMMAND_SYSTEM_COMPCURVE = MQTT_2_COMMAND_SYSTEM + "/CompCurve";

    MQTT_2_COMMAND_ZONE1_FLOW_SETPOINT = MQTT_2_COMMAND_ZONE1 + "/FlowSetpoint";
    MQTT_2_COMMAND_ZONE1_NOMODE_SETPOINT =
        MQTT_2_COMMAND_ZONE1 + "/ThermostatSetpoint";
    MQTT_2_COMMAND_ZONE1_HEAT_PROHIBIT =
        MQTT_2_COMMAND_ZONE1 + "/ProhibitHeating";
    MQTT_2_COMMAND_ZONE1_COOL_PROHIBIT =
        MQTT_2_COMMAND_ZONE1 + "/ProhibitCooling";
    MQTT_2_COMMAND_ZONE1_HEATINGMODE = MQTT_2_COMMAND_ZONE1 + "/HeatingMode";

    MQTT_2_COMMAND_ZONE2_FLOW_SETPOINT = MQTT_2_COMMAND_ZONE2 + "/FlowSetpoint";
    MQTT_2_COMMAND_ZONE2_NOMODE_SETPOINT =
        MQTT_2_COMMAND_ZONE2 + "/ThermostatSetpoint";
    MQTT_2_COMMAND_ZONE2_HEAT_PROHIBIT =
        MQTT_2_COMMAND_ZONE2 + "/ProhibitHeating";
    MQTT_2_COMMAND_ZONE2_COOL_PROHIBIT =
        MQTT_2_COMMAND_ZONE2 + "/ProhibitCooling";
    MQTT_2_COMMAND_ZONE2_HEATINGMODE = MQTT_2_COMMAND_ZONE2 + "/HeatingMode";

    MQTT_2_COMMAND_HOTWATER_MODE = MQTT_2_COMMAND_HOTWATER + "/Mode";
    MQTT_2_COMMAND_HOTWATER_SETPOINT = MQTT_2_COMMAND_HOTWATER + "/Setpoint";
    MQTT_2_COMMAND_HOTWATER_BOOST = MQTT_2_COMMAND_HOTWATER + "/Boost";
    MQTT_2_COMMAND_HOTWATER_NORM_BOOST =
        MQTT_2_COMMAND_HOTWATER + "/NormalBoost";
    MQTT_2_COMMAND_HOTWATER_PROHIBIT = MQTT_2_COMMAND_HOTWATER + "/Prohibit";

    MQTT_2_COMMAND_SYSTEM_HOLIDAYMODE = MQTT_2_COMMAND_SYSTEM + "/HolidayMode";
    MQTT_2_COMMAND_SYSTEM_SVRMODE = MQTT_2_COMMAND_SYSTEM + "/SvrControlMode";
    MQTT_2_COMMAND_SYSTEM_POWER = MQTT_2_COMMAND_SYSTEM + "/Power";
    MQTT_2_COMMAND_SYSTEM_UNITSIZE = MQTT_2_COMMAND_SYSTEM + "/UnitSize";
    MQTT_2_COMMAND_SYSTEM_GLYCOL = MQTT_2_COMMAND_SYSTEM + "/Glycol";
    MQTT_2_COMMAND_SYSTEM_SERVICE = MQTT_2_COMMAND_SYSTEM + "/Svc";

    MQTTCommand2Zone1FlowSetpoint = MQTT_2_COMMAND_ZONE1_FLOW_SETPOINT;
    MQTTCommand2Zone1NoModeSetpoint = MQTT_2_COMMAND_ZONE1_NOMODE_SETPOINT;
    MQTTCommand2Zone1ProhibitHeating = MQTT_2_COMMAND_ZONE1_HEAT_PROHIBIT;
    MQTTCommand2Zone1ProhibitCooling = MQTT_2_COMMAND_ZONE1_COOL_PROHIBIT;
    MQTTCommand2Zone1HeatingMode = MQTT_2_COMMAND_ZONE1_HEATINGMODE;

    MQTTCommand2Zone2FlowSetpoint = MQTT_2_COMMAND_ZONE2_FLOW_SETPOINT;
    MQTTCommand2Zone2NoModeSetpoint = MQTT_2_COMMAND_ZONE2_NOMODE_SETPOINT;
    MQTTCommand2Zone2ProhibitHeating = MQTT_2_COMMAND_ZONE2_HEAT_PROHIBIT;
    MQTTCommand2Zone2ProhibitCooling = MQTT_2_COMMAND_ZONE2_COOL_PROHIBIT;
    MQTTCommand2Zone2HeatingMode = MQTT_2_COMMAND_ZONE2_HEATINGMODE;

    MQTTCommand2HotwaterMode = MQTT_2_COMMAND_HOTWATER_MODE;
    MQTTCommand2HotwaterSetpoint = MQTT_2_COMMAND_HOTWATER_SETPOINT;
    MQTTCommand2HotwaterBoost = MQTT_2_COMMAND_HOTWATER_BOOST;
    MQTTCommand2HotwaterNormalBoost = MQTT_2_COMMAND_HOTWATER_NORM_BOOST;
    MQTTCommand2HotwaterProhibit = MQTT_2_COMMAND_HOTWATER_PROHIBIT;

    MQTTCommand2SystemHolidayMode = MQTT_2_COMMAND_SYSTEM_HOLIDAYMODE;
    MQTTCommand2SystemSvrMode = MQTT_2_COMMAND_SYSTEM_SVRMODE;
    MQTTCommand2SystemPower = MQTT_2_COMMAND_SYSTEM_POWER;
    MQTTCommand2SystemUnitSize = MQTT_2_COMMAND_SYSTEM_UNITSIZE;
    MQTTCommand2SystemGlycol = MQTT_2_COMMAND_SYSTEM_GLYCOL;
    MQTTCommand2SystemService = MQTT_2_COMMAND_SYSTEM_SERVICE;
    MQTTCommand2SystemCompCurve = MQTT_2_COMMAND_SYSTEM_COMPCURVE;
  }

  void initializeMQTTClient2() {
    DEBUG_PRINT(F("Attempting MQTT connection to: "));
    DEBUG_PRINT(mqttSettings.hostname2);
    DEBUG_PRINT(F(":"));
    DEBUG_PRINTLN(mqttSettings.port2);
    MQTTClient2.setServer(mqttSettings.hostname2, atoi(mqttSettings.port2));
  }

  void MQTT2onConnect(void) {
    DEBUG_PRINTLN(F("MQTT 2 ON CONNECT"));
    MQTTClient2.publish(MQTT_2_LWT.c_str(), "online");
    delay(10);

    MQTTClient2.subscribe(MQTTCommand2Zone1FlowSetpoint.c_str());
    MQTTClient2.subscribe(MQTTCommand2Zone1NoModeSetpoint.c_str());
    MQTTClient2.subscribe(MQTTCommand2Zone1ProhibitHeating.c_str());
    MQTTClient2.subscribe(MQTTCommand2Zone1ProhibitCooling.c_str());
    MQTTClient2.subscribe(MQTTCommand2Zone1HeatingMode.c_str());
    MQTTClient2.subscribe(MQTTCommand2Zone2FlowSetpoint.c_str());
    MQTTClient2.subscribe(MQTTCommand2Zone2NoModeSetpoint.c_str());
    MQTTClient2.subscribe(MQTTCommand2Zone2ProhibitHeating.c_str());
    MQTTClient2.subscribe(MQTTCommand2Zone2ProhibitCooling.c_str());
    MQTTClient2.subscribe(MQTTCommand2Zone2HeatingMode.c_str());
    // Subscribe to Holiday Mode even in cascade; restrict to master only
    if (!Flags::ConfigCascadeSlave()) {
      MQTTClient2.subscribe(MQTTCommand2SystemHolidayMode.c_str());
    }
    MQTTClient2.subscribe(MQTTCommand2HotwaterMode.c_str());
    MQTTClient2.subscribe(MQTTCommand2HotwaterSetpoint.c_str());
    MQTTClient2.subscribe(MQTTCommand2HotwaterBoost.c_str());
    MQTTClient2.subscribe(MQTTCommand2HotwaterNormalBoost.c_str());
    MQTTClient2.subscribe(MQTTCommand2HotwaterProhibit.c_str());
    MQTTClient2.subscribe(MQTTCommand2SystemPower.c_str());
    MQTTClient2.subscribe(MQTTCommand2SystemSvrMode.c_str());
    MQTTClient2.subscribe(MQTTCommand2SystemUnitSize.c_str());
    MQTTClient2.subscribe(MQTTCommand2SystemGlycol.c_str());
    MQTTClient2.subscribe(MQTTCommand2SystemService.c_str());
    MQTTClient2.subscribe(MQTTCommand2SystemCompCurve.c_str());
    // Subscribe to Debug enable toggles on secondary broker
    MQTTClient2.subscribe(MQTT_2_DEBUG_ENABLE.c_str());
    MQTTClient2.subscribe(MQTT_2_DEBUG_ENABLE_SERIAL.c_str());
    MQTTClient2.subscribe(MQTT_2_DEBUG_ENABLE_TELNET.c_str());
    delay(10);
    PublishDiscoveryTopics(2, MQTT_2_BASETOPIC);
    publishDebugEnableStates();
  }

  uint8_t MQTT2Reconnect() {
    if (MQTTClient2.connected()) {
      return 1;
    } else if (strcmp(mqttSettings.hostname2, "IPorDNS") != 0 &&
               strcmp(mqttSettings.hostname2, "") != 0) {
      initializeMQTTClient2();
      DEBUG_PRINT(F("With Client ID: "));
      DEBUG_PRINT(mqttSettings.deviceId);
      DEBUG_PRINT(F(", Username: "));
      DEBUG_PRINT(mqttSettings.user2);
      DEBUG_PRINT(F(" and Password: "));
      DEBUG_PRINTLN(mqttSettings.password2);

      if (MQTTClient2.connect(mqttSettings.deviceId, mqttSettings.user2,
                              mqttSettings.password2, MQTT_2_LWT.c_str(), 0,
                              true, "offline")) {
        DEBUG_PRINTLN(F("MQTT Server 2 Connected"));
        MQTT2onConnect();
        return 1;
      } else {
        switch (MQTTClient2.state()) {
        case -4:
          DEBUG_PRINTLN(F("MQTT_2_CONNECTION_TIMEOUT"));
          break;
        case -3:
          DEBUG_PRINTLN(F("MQTT_2_CONNECTION_LOST"));
          break;
        case -2:
          DEBUG_PRINTLN(F("MQTT_2_CONNECT_FAILED"));
          break;
        case -1:
          DEBUG_PRINTLN(F("MQTT_2_DISCONNECTED"));
          break;
        case 0:
          DEBUG_PRINTLN(F("MQTT_2_CONNECTED"));
          break;
        case 1:
          DEBUG_PRINTLN(F("MQTT_2_CONNECT_BAD_PROTOCOL"));
          break;
        case 2:
          DEBUG_PRINTLN(F("MQTT_2_CONNECT_BAD_CLIENT_ID"));
          break;
        case 3:
          DEBUG_PRINTLN(F("MQTT_2_CONNECT_UNAVAILABLE"));
          break;
        case 4:
          DEBUG_PRINTLN(F("MQTT_2_CONNECT_BAD_CREDENTIALS"));
          break;
        case 5:
          DEBUG_PRINTLN(F("MQTT_2_CONNECT_UNAUTHORIZED"));
          break;
        }
        return 0;
      }
      return 0;
    } else {
      DEBUG_PRINTLN(F("Secondary MQTT Not Set"));
      return 0;
    }
  }

  void handleMQTT2State() {
    if (!MQTTClient2.connected()) {
      MQTT2Reconnect();
    }
    MQTTClient2.loop();
  }

#endif
