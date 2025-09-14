# Cascade System Setup Guide

This guide shows how to configure a 3-unit cascade system using the distributed ESP architecture.

## Hardware Setup

### Required Components
- 3x ESP32 modules (one per controller)
- 3x CN105 cables 
- WiFi network accessible to all ESPs
- MQTT broker (Home Assistant or standalone)

### Physical Connections
```
Master Controller CN105 ←→ ESP #1 (Master ESP)
Slave Controller 1 CN105 ←→ ESP #2 (Slave ESP)  
Slave Controller 2 CN105 ←→ ESP #3 (Slave ESP)
```

## Software Configuration

### Step 1: Flash Firmware
Flash `ECODAN_Bridge_Cascade.ino` to all three ESP modules.

### Step 2: Configure Each ESP

#### Master ESP (Node ID: 0)
```
WiFi Manager Configuration:
- SSID: YourWiFiNetwork
- Password: YourWiFiPassword
- MQTT Server: 192.168.1.100
- MQTT Username: ecodan
- MQTT Password: your_password
- MQTT Base Topic: Ecodan/ASHP
- Cascade Mode: ✓ Enabled
- Cascade Node ID: 0
```

#### Slave ESP #1 (Node ID: 1)
```
WiFi Manager Configuration:
- SSID: YourWiFiNetwork (same as master)
- Password: YourWiFiPassword (same as master)
- MQTT Server: 192.168.1.100 (same as master)
- MQTT Username: ecodan (same as master)
- MQTT Password: your_password (same as master)
- MQTT Base Topic: Ecodan/ASHP (same as master)
- Cascade Mode: ✓ Enabled
- Cascade Node ID: 1
```

#### Slave ESP #2 (Node ID: 2)
```
WiFi Manager Configuration:
- SSID: YourWiFiNetwork (same as master)
- Password: YourWiFiPassword (same as master)
- MQTT Server: 192.168.1.100 (same as master)
- MQTT Username: ecodan (same as master)
- MQTT Password: your_password (same as master)
- MQTT Base Topic: Ecodan/ASHP (same as master)
- Cascade Mode: ✓ Enabled
- Cascade Node ID: 2
```

## Verification

### MQTT Topics
After configuration, you should see these topics:

#### System Status
```
Ecodan/ASHP/cascade/system/status
Ecodan/ASHP/cascade/system/announce
```

#### Node Data
```
Ecodan/ASHP/cascade/node_0/data      # Master controller data
Ecodan/ASHP/cascade/node_0/heartbeat # Master heartbeat
Ecodan/ASHP/cascade/node_1/data      # Slave 1 controller data  
Ecodan/ASHP/cascade/node_1/heartbeat # Slave 1 heartbeat
Ecodan/ASHP/cascade/node_2/data      # Slave 2 controller data
Ecodan/ASHP/cascade/node_2/heartbeat # Slave 2 heartbeat
```

### Home Assistant Entities
The system will expose data to build entities like:
```
sensor.ecodan_cascade_total_capacity
sensor.ecodan_cascade_online_nodes
sensor.ecodan_node_0_status
sensor.ecodan_node_0_compressor_frequency
sensor.ecodan_node_1_status
sensor.ecodan_node_1_fan_speed
sensor.ecodan_node_2_status
sensor.ecodan_node_2_compressor_starts
```

## Troubleshooting

### Node Not Appearing
1. Check WiFi connection on ESP
2. Verify MQTT broker connectivity
3. Ensure unique node IDs (no duplicates)
4. Check serial console for error messages

### Missing Data
1. Verify CN105 cable connections
2. Check controller power and operation
3. Monitor MQTT traffic for data flow
4. Ensure proper baud rate (2400, 8E1)

### System Commands Not Working
1. Ensure master ESP (node 0) is online
2. Check master controller has physical remote connected
3. Verify master controller has system control authority
4. Monitor command topics for proper routing

## Advanced Configuration

### Custom Node Names
You can customize node names by modifying the firmware:
```cpp
String nodeName = "Living Room Heat Pump";  // Instead of "Slave Node 1"
```

### Capacity Configuration
Update unit capacities in the firmware:
```cpp
if (doc.containsKey("unit_capacity")) {
  doc["unit_capacity"] = 12.0;  // kW capacity for this unit
}
```

### Monitoring Intervals
Adjust timing in CascadeNetwork.h:
```cpp
#define CASCADE_HEARTBEAT_INTERVAL 5000   // 5 seconds (faster)
#define CASCADE_DATA_TIMEOUT 30000        // 30 seconds (shorter timeout)
```
