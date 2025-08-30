# Ecodan Cascade System Support

This implementation extends the original Ecodan bridge to support cascaded ASHP (Air Source Heat Pump) systems with multiple controllers in a master-slave configuration.

## Overview

In a distributed cascade system:
- **Master ESP**: Connected to master controller CN105, handles system control and coordination
- **Slave ESPs**: Connected to individual slave controller CN105 ports, gather unit-specific data
- **Network Communication**: All ESPs communicate via MQTT/WiFi to share data and coordinate

## Features

### Cascade Management
- Support for up to 8 units in cascade configuration
- Automatic master-slave detection and communication
- Individual unit monitoring and status reporting
- System-wide coordination through master controller

### Enhanced Data Collection
- Unit-specific metrics (fan speeds, compressor starts, individual power consumption)
- Aggregated system statistics (total capacity, average COP, active units)
- Individual unit health monitoring and error reporting

### MQTT Integration
- Cascade-specific MQTT topics for system overview
- Individual unit topics for detailed monitoring
- Home Assistant auto-discovery for cascade systems
- Unified command interface through master controller

### Configuration Management
- Web-based cascade configuration interface
- JSON configuration file support
- Runtime unit addition/removal
- Configuration validation and error reporting

## Hardware Requirements

### Distributed ESP Architecture
Each controller in the cascade system has its own ESP module:

#### Master ESP Module
- **Connection**: Master controller CN105 port
- **Purpose**: System control, coordination, master unit data
- **Remote**: Physical remote controller must be connected to master

#### Slave ESP Modules  
- **Connection**: Individual slave controller CN105 ports
- **Purpose**: Gather slave-specific data (fan speeds, compressor starts, etc.)
- **Data**: Unit-specific metrics not available from master

#### Supported ESP Platforms
- **ESP32 AtomS3 Lite**: CN105 on GPIO 2 (RX), GPIO 1 (TX)
- **WT32-ETH01**: CN105 on GPIO 4 (RX), GPIO 2 (TX)  
- **ESP8266**: CN105 on GPIO 14 (RX), GPIO 16 (TX)

### Network Requirements
- All ESP modules must be on the same WiFi network
- MQTT broker accessible to all modules
- Reliable network connectivity for real-time coordination

## Installation

### 1. Hardware Setup
1. **Master ESP**: Connect to master controller CN105 port
2. **Slave ESPs**: Connect each to respective slave controller CN105 ports  
3. Ensure proper 2400 baud, 8E1 serial configuration on all connections
4. Use appropriate level shifters if needed (3.3V ↔ 5V)
5. Configure each ESP with unique cascade node ID (0=master, 1-7=slaves)

### 2. Software Configuration
1. Flash `ECODAN_Bridge_Cascade.ino` to **all ESP modules**
2. Configure each ESP via WiFi Manager:
   - Enable cascade mode
   - Set unique cascade node ID (0=master, 1-7=slaves)
   - Configure same MQTT broker settings
3. Master ESP automatically coordinates with slave ESPs

### 3. MQTT Setup
The cascade system creates additional MQTT topics:

```
Ecodan/ASHP/cascade/system/status      # System-wide status (from master)
Ecodan/ASHP/cascade/system/announce    # Node discovery
Ecodan/ASHP/cascade/system/command     # System commands
Ecodan/ASHP/cascade/node_0/data        # Master node data
Ecodan/ASHP/cascade/node_0/heartbeat   # Master heartbeat
Ecodan/ASHP/cascade/node_1/data        # Slave 1 data
Ecodan/ASHP/cascade/node_1/heartbeat   # Slave 1 heartbeat
Ecodan/ASHP/cascade/node_2/data        # Slave 2 data
Ecodan/ASHP/cascade/node_2/heartbeat   # Slave 2 heartbeat
```

## Configuration

### Basic Configuration
Enable cascade mode through WiFi Manager or by setting:
```cpp
mqttSettings.cascadeEnabled = true;
```

### Advanced Configuration
Create a `cascade_config.json` file in LittleFS:

```json
{
  "cascade_enabled": true,
  "master_unit_id": 0,
  "system_name": "My Cascade System",
  "units": [
    {
      "id": 0,
      "type": "master",
      "name": "Master Unit",
      "cascade_address": 0,
      "capacity": 8.5,
      "enabled": true
    },
    {
      "id": 1,
      "type": "slave", 
      "name": "Slave Unit 1",
      "cascade_address": 1,
      "capacity": 8.5,
      "enabled": true
    },
    {
      "id": 2,
      "type": "slave", 
      "name": "Slave Unit 2",
      "cascade_address": 2,
      "capacity": 8.5,
      "enabled": true
    }
  ]
}
```

## Usage

### Home Assistant Integration
The cascade system automatically creates Home Assistant entities:

**System Entities:**
- `sensor.ecodan_cascade_total_capacity`
- `sensor.ecodan_cascade_connected_units`
- `sensor.ecodan_cascade_active_units`
- `sensor.ecodan_cascade_average_cop`

**Individual Unit Entities:**
- `sensor.ecodan_unit_0_status`
- `sensor.ecodan_unit_0_compressor_frequency`
- `sensor.ecodan_unit_1_status`
- `sensor.ecodan_unit_1_compressor_frequency`

### MQTT Commands
System-wide commands are sent through the master controller:

```bash
# Set system setpoint
mosquitto_pub -t "Ecodan/ASHP/cascade/command/setpoint/zone1" -m "21.5"

# Set system mode
mosquitto_pub -t "Ecodan/ASHP/cascade/command/mode" -m "2"
```

## Monitoring

### System Status
Monitor overall cascade system health:
- Connected units count
- Total system capacity
- Average COP across all units
- System-wide error status

### Individual Units
Track each unit's performance:
- Connection status
- Compressor frequency
- Unit-specific power consumption
- Error counts and last errors

## Troubleshooting

### Common Issues

**Units Not Connecting:**
- Verify CN105 cable connection to master controller
- Ensure proper baud rate (2400, 8E1)
- Check cascade network configuration on controllers
- Verify master controller has system control

**Master Unit Not Responding:**
- Verify master unit ID in configuration
- Check physical remote controller connection
- Ensure master unit has system control

**MQTT Topics Missing:**
- Confirm cascade mode is enabled
- Check MQTT broker connection
- Verify topic prefix configuration

### Debug Information
Enable debug output to monitor:
- Master controller connection status
- Cascade protocol communication
- Configuration validation
- MQTT message flow

Debug output is enabled by default and can be monitored via:
- Serial console (115200 baud)
- Telnet connection to device IP
- Web interface debug log

## API Reference

### CascadeManager Class
Main class for managing cascade operations:

```cpp
// Add units to cascade
cascadeManager.addUnit(unitId, type, name, cascadeAddress, capacity);

// Set master connection
cascadeManager.setMasterConnection(&HeatPump);

// Get system status
bool connected = cascadeManager.isConnected();
uint8_t activeUnits = cascadeManager.getActiveUnits();
float totalCapacity = cascadeManager.getTotalCapacity();

// Send commands (through master)
cascadeManager.setSystemSetpoint(21.5, 1);
cascadeManager.setSystemMode(2);
```

### CascadeMQTT Class
Handles MQTT integration for cascade systems:

```cpp
// Initialize MQTT
cascadeMQTT.initialize(&client1, &client2, baseTopic);

// Publish status
cascadeMQTT.publishCascadeStatus();
cascadeMQTT.publishUnitStatus(unitId);

// Handle commands
cascadeMQTT.handleCascadeCommand(topic, payload);
```

## Contributing

When contributing to cascade functionality:

1. Maintain backward compatibility with single-unit systems
2. Test with multiple unit configurations
3. Validate MQTT topic structure
4. Update Home Assistant discovery as needed
5. Document any new configuration options

## License

This cascade implementation maintains the same GPL-3.0 license as the original project.