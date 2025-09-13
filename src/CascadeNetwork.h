/*
    Mitsubishi Ecodan Distributed Cascade Network
    Handles communication between multiple ESP modules in a cascade system
    
    Each controller (master + slaves) has its own ESP module:
    - Master ESP: Connected to master controller CN105 (system control)
    - Slave ESPs: Connected to slave controller CN105 (unit-specific data)
    
    Copyright (C) <2024>
*/

#ifndef CASCADE_NETWORK_H
#define CASCADE_NETWORK_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "Ecodan.h"

// External variables
extern String FirmwareVersion;

#define MAX_CASCADE_UNITS 8
#define CASCADE_DISCOVERY_INTERVAL 30000  // 30 seconds
#define CASCADE_HEARTBEAT_INTERVAL 10000  // 10 seconds
#define CASCADE_DATA_TIMEOUT 60000        // 60 seconds

enum CascadeNodeType {
  CASCADE_NODE_MASTER = 0,
  CASCADE_NODE_SLAVE = 1
};

enum CascadeNodeStatus {
  CASCADE_NODE_OFFLINE = 0,
  CASCADE_NODE_ONLINE = 1,
  CASCADE_NODE_ERROR = 2
};

struct CascadeNodeInfo {
  uint8_t nodeId;
  CascadeNodeType nodeType;
  String nodeName;
  String ipAddress;
  String macAddress;
  String lwtTopic; // MQTT LWT topic for this node
  CascadeNodeStatus status;
  unsigned long lastSeen;
  float unitCapacity;
  
  // Unit-specific data (only available from slave nodes)
  uint8_t fanSpeed;
  uint16_t compressorStarts;
  uint8_t compressorFrequency;
  float unitPower;
  float unitCOP;
  int16_t errorCode;
  String errorMessage;
};

class CascadeNetwork {
public:
  CascadeNetwork();
  
  // Node configuration
  void initialize(CascadeNodeType nodeType, uint8_t nodeId, const String& nodeName);
  void setMQTTClient(PubSubClient* client, const String& baseTopic);
  void setLocalUnit(ECODAN* unit);
  void setLocalUnitCapacity(float capacity);
  
  // Network management
  void begin();
  void process();
  void announcePresence();
  void sendHeartbeat();
  
  // Data sharing
  void publishLocalData();
  void handleRemoteData(const String& topic, const String& payload);
  // Handle non-cascade topics that are relevant (e.g., LWT)
  bool handleAuxTopic(const String& topic, const String& payload);
  
  // Node discovery and management
  bool isNodeOnline(uint8_t nodeId);
  CascadeNodeInfo* getNodeInfo(uint8_t nodeId);
  uint8_t getOnlineNodes();
  bool isMasterOnline();
  
  // System coordination (master node only)
  void sendSystemCommand(const String& command, const String& value);
  void broadcastSystemStatus();
  
  // Data aggregation (master node only)
  float getTotalSystemCapacity();
  float getTotalSystemPower();
  uint8_t getActiveUnits();
  
  // Leadership
  bool isLeader();
  uint8_t getLowestOnlineSlaveId();
  
  // Configuration
  bool isMasterNode() { return localNodeType == CASCADE_NODE_MASTER; }
  uint8_t getLocalNodeId() { return localNodeId; }
  
private:
  // Local node configuration
  CascadeNodeType localNodeType;
  uint8_t localNodeId;
  String localNodeName;
  ECODAN* localUnit;
  
  // Network configuration
  PubSubClient* mqttClient;
  String mqttBaseTopic;
  String cascadeTopicPrefix;
  float localUnitCapacity;
  
  // Node tracking
  CascadeNodeInfo nodes[MAX_CASCADE_UNITS];
  uint8_t knownNodes;
  unsigned long lastDiscovery;
  unsigned long lastHeartbeat;
  unsigned long lastStatus;
  bool subscriptionsReady;
  
  // Internal methods
  void upsertLocalNode();
  void subscribeToTopics();
  void processNodeAnnouncement(const JsonDocument& doc);
  void processNodeHeartbeat(const JsonDocument& doc);
  void processNodeData(const JsonDocument& doc);
  void processSystemCommand(const JsonDocument& doc);
  void updateNodeStatus(uint8_t nodeId, CascadeNodeStatus status);
  void cleanupOfflineNodes();
  void publishNodeOffline(uint8_t index);
  String getNodeTopic(uint8_t nodeId, const String& suffix);
  String getSystemTopic(const String& suffix);
};

// Global cascade network instance
extern CascadeNetwork cascadeNetwork;

#endif
