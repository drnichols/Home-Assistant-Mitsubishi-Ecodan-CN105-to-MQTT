/*
    Mitsubishi Ecodan Distributed Cascade Network Implementation
    
    Copyright (C) <2024>
*/

#include "CascadeNetwork.h"
#include <WiFi.h>

// Use Serial for debug output in CascadeNetwork
#define CASCADE_DEBUG_PRINT(x) Serial.print(x)
#define CASCADE_DEBUG_PRINTLN(x) Serial.println(x)

CascadeNetwork cascadeNetwork;

CascadeNetwork::CascadeNetwork() {
  localNodeType = CASCADE_NODE_MASTER;
  localNodeId = 0;
  localUnit = nullptr;
  mqttClient = nullptr;
  knownNodes = 0;
  lastDiscovery = 0;
  lastHeartbeat = 0;
  
  // Initialize node array
  for (uint8_t i = 0; i < MAX_CASCADE_UNITS; i++) {
    nodes[i].nodeId = 0xFF; // Invalid ID
    nodes[i].status = CASCADE_NODE_OFFLINE;
    nodes[i].lastSeen = 0;
  }
}

void CascadeNetwork::initialize(CascadeNodeType nodeType, uint8_t nodeId, const String& nodeName) {
  localNodeType = nodeType;
  localNodeId = nodeId;
  localNodeName = nodeName;
  
  CASCADE_DEBUG_PRINT("Initializing cascade node: ");
  CASCADE_DEBUG_PRINT(nodeName);
  CASCADE_DEBUG_PRINT(" (Type: ");
  CASCADE_DEBUG_PRINT(nodeType == CASCADE_NODE_MASTER ? "Master" : "Slave");
  CASCADE_DEBUG_PRINT(", ID: ");
  CASCADE_DEBUG_PRINT(nodeId);
  CASCADE_DEBUG_PRINTLN(")");
}

void CascadeNetwork::setMQTTClient(PubSubClient* client, const String& baseTopic) {
  mqttClient = client;
  mqttBaseTopic = baseTopic;
  cascadeTopicPrefix = baseTopic + "/cascade";
  
  CASCADE_DEBUG_PRINT("Cascade MQTT configured with base topic: ");
  CASCADE_DEBUG_PRINTLN(baseTopic);
}

void CascadeNetwork::setLocalUnit(ECODAN* unit) {
  localUnit = unit;
  CASCADE_DEBUG_PRINTLN("Local unit connection established");
}

void CascadeNetwork::begin() {
  if (!mqttClient) {
    CASCADE_DEBUG_PRINTLN("ERROR: MQTT client not configured for cascade network");
    return;
  }
  
  subscribeToTopics();
  announcePresence();
  
  CASCADE_DEBUG_PRINTLN("Cascade network started");
}

void CascadeNetwork::process() {
  if (!mqttClient || !mqttClient->connected()) return;
  
  unsigned long currentTime = millis();
  
  // Send periodic heartbeat
  if (currentTime - lastHeartbeat >= CASCADE_HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = currentTime;
  }
  
  // Send periodic discovery announcement
  if (currentTime - lastDiscovery >= CASCADE_DISCOVERY_INTERVAL) {
    announcePresence();
    lastDiscovery = currentTime;
  }
  
  // Publish local unit data
  if (localUnit && localUnit->UpdateComplete()) {
    publishLocalData();
  }
  
  // Clean up offline nodes
  cleanupOfflineNodes();
  
  // Master-specific processing
  if (isMasterNode()) {
    broadcastSystemStatus();
  }
}

void CascadeNetwork::announcePresence() {
  if (!mqttClient || !mqttClient->connected()) return;
  
  DynamicJsonDocument doc(512);
  doc["node_id"] = localNodeId;
  doc["node_type"] = (localNodeType == CASCADE_NODE_MASTER) ? "master" : "slave";
  doc["node_name"] = localNodeName;
  doc["ip_address"] = WiFi.localIP().toString();
  doc["mac_address"] = WiFi.macAddress();
  doc["timestamp"] = millis();
  doc["firmware_version"] = FirmwareVersion;
  
  if (localUnit && localUnit->HeatPumpConnected()) {
    doc["unit_connected"] = true;
    doc["unit_capacity"] = 8.5; // This could be configurable
  } else {
    doc["unit_connected"] = false;
  }
  
  String payload;
  serializeJson(doc, payload);
  
  String topic = getSystemTopic("announce");
  mqttClient->publish(topic.c_str(), payload.c_str(), false);
  
  CASCADE_DEBUG_PRINT("Announced presence: ");
  CASCADE_DEBUG_PRINTLN(localNodeName);
}

void CascadeNetwork::sendHeartbeat() {
  if (!mqttClient || !mqttClient->connected()) return;
  
  DynamicJsonDocument doc(256);
  doc["node_id"] = localNodeId;
  doc["timestamp"] = millis();
  doc["status"] = "online";
  
  if (localUnit) {
    doc["unit_connected"] = localUnit->HeatPumpConnected();
  }
  
  String payload;
  serializeJson(doc, payload);
  
  String topic = getNodeTopic(localNodeId, "heartbeat");
  mqttClient->publish(topic.c_str(), payload.c_str(), false);
}

void CascadeNetwork::publishLocalData() {
  if (!mqttClient || !mqttClient->connected() || !localUnit) return;
  
  DynamicJsonDocument doc(1024);
  doc["node_id"] = localNodeId;
  doc["timestamp"] = millis();
  
  // Standard Ecodan data available from all units
  doc["system_power"] = localUnit->Status.SystemPowerMode;
  doc["operation_mode"] = localUnit->Status.SystemOperationMode;
  doc["flow_temp"] = localUnit->Status.Zone1FlowTemperature;
  doc["return_temp"] = localUnit->Status.Zone1ReturnTemperature;
  doc["outside_temp"] = localUnit->Status.OutsideTemperature;
  doc["compressor_frequency"] = localUnit->Status.CompressorFrequency;
  
  // Unit-specific data (more detailed on slave units)
  if (localNodeType == CASCADE_NODE_SLAVE) {
    // Slave units can provide more detailed unit-specific data
    doc["fan_speed"] = localUnit->Status.Fan1RPM; // Fan RPM from message 0xa3
    doc["compressor_starts"] = 0; // Would need to be tracked
    doc["unit_power"] = 0; // Would need to be calculated
    doc["error_code"] = localUnit->Status.ErrCode1;
  }
  
  // Zone data (primarily from master)
  if (localNodeType == CASCADE_NODE_MASTER) {
    doc["zone1_temp"] = localUnit->Status.Zone1TemperatureSetpoint;
    doc["zone2_temp"] = localUnit->Status.Zone2TemperatureSetpoint;
    doc["dhw_temp"] = localUnit->Status.HotWaterSetpoint;
    doc["zone1_flow_temp"] = localUnit->Status.Zone1FlowTemperature;
    doc["zone2_flow_temp"] = localUnit->Status.Zone2FlowTemperature;
  }
  
  String payload;
  serializeJson(doc, payload);
  
  String topic = getNodeTopic(localNodeId, "data");
  mqttClient->publish(topic.c_str(), payload.c_str(), true); // Retained
}

void CascadeNetwork::handleRemoteData(const String& topic, const String& payload) {
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payload);
  
  if (error) {
    CASCADE_DEBUG_PRINT("Failed to parse cascade message: ");
    CASCADE_DEBUG_PRINTLN(error.c_str());
    return;
  }
  
  // Determine message type based on topic
  if (topic.endsWith("/announce")) {
    processNodeAnnouncement(doc);
  } else if (topic.endsWith("/heartbeat")) {
    processNodeHeartbeat(doc);
  } else if (topic.endsWith("/data")) {
    processNodeData(doc);
  } else if (topic.endsWith("/command")) {
    processSystemCommand(doc);
  }
}

void CascadeNetwork::processNodeAnnouncement(const JsonDocument& doc) {
  uint8_t nodeId = doc["node_id"];
  String nodeType = doc["node_type"];
  String nodeName = doc["node_name"];
  String ipAddress = doc["ip_address"];
  String macAddress = doc["mac_address"];
  
  // Find or create node entry
  uint8_t nodeIndex = MAX_CASCADE_UNITS;
  for (uint8_t i = 0; i < knownNodes; i++) {
    if (nodes[i].nodeId == nodeId) {
      nodeIndex = i;
      break;
    }
  }
  
  if (nodeIndex == MAX_CASCADE_UNITS && knownNodes < MAX_CASCADE_UNITS) {
    nodeIndex = knownNodes++;
  }
  
  if (nodeIndex < MAX_CASCADE_UNITS) {
    nodes[nodeIndex].nodeId = nodeId;
    nodes[nodeIndex].nodeType = (nodeType == "master") ? CASCADE_NODE_MASTER : CASCADE_NODE_SLAVE;
    nodes[nodeIndex].nodeName = nodeName;
    nodes[nodeIndex].ipAddress = ipAddress;
    nodes[nodeIndex].macAddress = macAddress;
    nodes[nodeIndex].status = CASCADE_NODE_ONLINE;
    nodes[nodeIndex].lastSeen = millis();
    
    if (doc.containsKey("unit_capacity")) {
      nodes[nodeIndex].unitCapacity = doc["unit_capacity"];
    }
    
    CASCADE_DEBUG_PRINT("Discovered cascade node: ");
    CASCADE_DEBUG_PRINT(nodeName);
    CASCADE_DEBUG_PRINT(" (");
    CASCADE_DEBUG_PRINT(ipAddress);
    CASCADE_DEBUG_PRINTLN(")");
  }
}

void CascadeNetwork::processNodeHeartbeat(const JsonDocument& doc) {
  uint8_t nodeId = doc["node_id"];
  
  // Update last seen time for known node
  for (uint8_t i = 0; i < knownNodes; i++) {
    if (nodes[i].nodeId == nodeId) {
      nodes[i].lastSeen = millis();
      nodes[i].status = CASCADE_NODE_ONLINE;
      break;
    }
  }
}

void CascadeNetwork::processNodeData(const JsonDocument& doc) {
  uint8_t nodeId = doc["node_id"];
  
  // Find node and update data
  for (uint8_t i = 0; i < knownNodes; i++) {
    if (nodes[i].nodeId == nodeId) {
      nodes[i].lastSeen = millis();
      
      // Update unit-specific data
      if (doc.containsKey("fan_speed")) {
        nodes[i].fanSpeed = doc["fan_speed"];
      }
      if (doc.containsKey("compressor_frequency")) {
        nodes[i].compressorFrequency = doc["compressor_frequency"];
      }
      if (doc.containsKey("compressor_starts")) {
        nodes[i].compressorStarts = doc["compressor_starts"];
      }
      if (doc.containsKey("unit_power")) {
        nodes[i].unitPower = doc["unit_power"];
      }
      if (doc.containsKey("error_code")) {
        nodes[i].errorCode = doc["error_code"];
      }
      
      break;
    }
  }
}

void CascadeNetwork::processSystemCommand(const JsonDocument& doc) {
  // Only slaves process system commands from master
  if (isMasterNode()) return;
  
  String command = doc["command"];
  String value = doc["value"];
  
  CASCADE_DEBUG_PRINT("Received system command: ");
  CASCADE_DEBUG_PRINT(command);
  CASCADE_DEBUG_PRINT(" = ");
  CASCADE_DEBUG_PRINTLN(value);
  
  // Process commands that affect local unit
  // Note: Most commands should go through master, but some local actions might be needed
}

void CascadeNetwork::sendSystemCommand(const String& command, const String& value) {
  // Only master can send system commands
  if (!isMasterNode() || !mqttClient || !mqttClient->connected()) return;
  
  DynamicJsonDocument doc(256);
  doc["command"] = command;
  doc["value"] = value;
  doc["timestamp"] = millis();
  doc["from_node"] = localNodeId;
  
  String payload;
  serializeJson(doc, payload);
  
  String topic = getSystemTopic("command");
  mqttClient->publish(topic.c_str(), payload.c_str(), false);
  
  CASCADE_DEBUG_PRINT("Sent system command: ");
  CASCADE_DEBUG_PRINT(command);
  CASCADE_DEBUG_PRINT(" = ");
  CASCADE_DEBUG_PRINTLN(value);
}

void CascadeNetwork::broadcastSystemStatus() {
  if (!isMasterNode() || !mqttClient || !mqttClient->connected()) return;
  
  DynamicJsonDocument doc(1024);
  doc["master_node"] = localNodeId;
  doc["online_nodes"] = getOnlineNodes();
  doc["total_capacity"] = getTotalSystemCapacity();
  doc["total_power"] = getTotalSystemPower();
  doc["average_cop"] = getAverageSystemCOP();
  doc["active_units"] = getActiveUnits();
  
  // Add individual node status
  JsonArray nodeArray = doc.createNestedArray("nodes");
  for (uint8_t i = 0; i < knownNodes; i++) {
    if (nodes[i].status != CASCADE_NODE_OFFLINE) {
      JsonObject nodeObj = nodeArray.createNestedObject();
      nodeObj["id"] = nodes[i].nodeId;
      nodeObj["name"] = nodes[i].nodeName;
      nodeObj["type"] = (nodes[i].nodeType == CASCADE_NODE_MASTER) ? "master" : "slave";
      nodeObj["status"] = (nodes[i].status == CASCADE_NODE_ONLINE) ? "online" : "error";
      nodeObj["capacity"] = nodes[i].unitCapacity;
      nodeObj["frequency"] = nodes[i].compressorFrequency;
    }
  }
  
  String payload;
  serializeJson(doc, payload);
  
  String topic = getSystemTopic("status");
  mqttClient->publish(topic.c_str(), payload.c_str(), true); // Retained
}

bool CascadeNetwork::isNodeOnline(uint8_t nodeId) {
  for (uint8_t i = 0; i < knownNodes; i++) {
    if (nodes[i].nodeId == nodeId) {
      return (nodes[i].status == CASCADE_NODE_ONLINE) && 
             ((millis() - nodes[i].lastSeen) < CASCADE_DATA_TIMEOUT);
    }
  }
  return false;
}

CascadeNodeInfo* CascadeNetwork::getNodeInfo(uint8_t nodeId) {
  for (uint8_t i = 0; i < knownNodes; i++) {
    if (nodes[i].nodeId == nodeId) {
      return &nodes[i];
    }
  }
  return nullptr;
}

uint8_t CascadeNetwork::getOnlineNodes() {
  uint8_t online = 0;
  for (uint8_t i = 0; i < knownNodes; i++) {
    if (isNodeOnline(nodes[i].nodeId)) {
      online++;
    }
  }
  return online;
}

bool CascadeNetwork::isMasterOnline() {
  return isNodeOnline(0); // Master is always node 0
}

float CascadeNetwork::getTotalSystemCapacity() {
  float total = 0.0;
  for (uint8_t i = 0; i < knownNodes; i++) {
    if (isNodeOnline(nodes[i].nodeId)) {
      total += nodes[i].unitCapacity;
    }
  }
  return total;
}

float CascadeNetwork::getTotalSystemPower() {
  float total = 0.0;
  for (uint8_t i = 0; i < knownNodes; i++) {
    if (isNodeOnline(nodes[i].nodeId)) {
      total += nodes[i].unitPower;
    }
  }
  return total;
}

float CascadeNetwork::getAverageSystemCOP() {
  float totalPower = getTotalSystemPower();
  float totalCapacity = getTotalSystemCapacity();
  return (totalPower > 0) ? (totalCapacity / totalPower) : 0.0;
}

uint8_t CascadeNetwork::getActiveUnits() {
  uint8_t active = 0;
  for (uint8_t i = 0; i < knownNodes; i++) {
    if (isNodeOnline(nodes[i].nodeId) && nodes[i].compressorFrequency > 0) {
      active++;
    }
  }
  return active;
}

void CascadeNetwork::subscribeToTopics() {
  if (!mqttClient || !mqttClient->connected()) return;
  
  // Subscribe to system-wide topics
  String announceTopic = getSystemTopic("announce");
  String commandTopic = getSystemTopic("command");
  
  mqttClient->subscribe(announceTopic.c_str());
  if (!isMasterNode()) {
    mqttClient->subscribe(commandTopic.c_str());
  }
  
  // Subscribe to all node heartbeat and data topics
  for (uint8_t i = 0; i < MAX_CASCADE_UNITS; i++) {
    if (i != localNodeId) { // Don't subscribe to own topics
      String heartbeatTopic = getNodeTopic(i, "heartbeat");
      String dataTopic = getNodeTopic(i, "data");
      mqttClient->subscribe(heartbeatTopic.c_str());
      mqttClient->subscribe(dataTopic.c_str());
    }
  }
  
  CASCADE_DEBUG_PRINTLN("Subscribed to cascade network topics");
}

void CascadeNetwork::cleanupOfflineNodes() {
  unsigned long currentTime = millis();
  
  for (uint8_t i = 0; i < knownNodes; i++) {
    if ((currentTime - nodes[i].lastSeen) > CASCADE_DATA_TIMEOUT) {
      if (nodes[i].status != CASCADE_NODE_OFFLINE) {
        CASCADE_DEBUG_PRINT("Node went offline: ");
        CASCADE_DEBUG_PRINTLN(nodes[i].nodeName);
        nodes[i].status = CASCADE_NODE_OFFLINE;
      }
    }
  }
}

String CascadeNetwork::getNodeTopic(uint8_t nodeId, const String& suffix) {
  return cascadeTopicPrefix + "/node_" + String(nodeId) + "/" + suffix;
}

String CascadeNetwork::getSystemTopic(const String& suffix) {
  return cascadeTopicPrefix + "/system/" + suffix;
}
