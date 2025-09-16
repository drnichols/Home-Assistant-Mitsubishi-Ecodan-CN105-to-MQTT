#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

// Lightweight stand-in for Arduino's String class used by CalculateCompCurve.
class String {
 public:
  String() = default;
  String(const char* value) { assign(value); }
  String(const std::string& value) { assign(value); }
  String(std::string&& value) : data_(std::move(value)) { resetReader(); }
  String(int value) { assign(std::to_string(value)); }
  String(long value) { assign(std::to_string(value)); }
  String(float value) { assign(fromFloat(value)); }
  String(double value) { assign(fromFloat(value)); }

  const char* c_str() const { return data_.c_str(); }
  std::size_t length() const { return data_.length(); }
  bool isEmpty() const { return data_.empty(); }

  operator const char*() const { return data_.c_str(); }

  String operator+(const String& other) const { return String(data_ + other.data_); }
  String operator+(const char* other) const { return String(data_ + (other ? std::string(other) : std::string())); }
  String& operator+=(const String& other) {
    data_ += other.data_;
    resetReader();
    return *this;
  }
  String& operator+=(const char* other) {
    if (other) {
      data_ += other;
      resetReader();
    }
    return *this;
  }

  String& operator=(const char* value) {
    assign(value);
    return *this;
  }
  String& operator=(const std::string& value) {
    assign(value);
    return *this;
  }
  String& operator=(std::string&& value) {
    data_ = std::move(value);
    resetReader();
    return *this;
  }

  const std::string& str() const { return data_; }

  int read() {
    if (read_pos_ >= data_.size()) return -1;
    return static_cast<unsigned char>(data_[read_pos_++]);
  }

  size_t readBytes(char* buffer, size_t length) {
    if (!buffer || length == 0) return 0;
    size_t available = data_.size() - read_pos_;
    size_t to_copy = std::min(length, available);
    if (to_copy > 0) {
      std::memcpy(buffer, data_.data() + read_pos_, to_copy);
      read_pos_ += to_copy;
    }
    return to_copy;
  }

  void resetReader() { read_pos_ = 0; }

 private:
  std::string data_;
  std::size_t read_pos_ = 0;

  static std::string fromFloat(double value);
  void assign(const char* value) {
    data_ = value ? value : "";
    resetReader();
  }
  void assign(const std::string& value) {
    data_ = value;
    resetReader();
  }
};

inline String operator+(const char* lhs, const String& rhs) {
  return String(lhs ? std::string(lhs) + rhs.str() : rhs.str());
}

struct UnitSettings {
  float UnitSize = 0;
  float GlycolStrength = 0;
  String CompCurve;
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

struct FakeHeatPumpStatus {
  float OutsideTemperature = 0;
  int Defrost = 0;
  bool Simple2Zone = false;
  bool Has2Zone = false;
  float Zone1FlowTemperatureSetpoint = 0;
  float Zone2FlowTemperatureSetpoint = 0;
};

struct FlowSetpointCall {
  float flow = 0;
  int mode = 0;
  int zone = 0;
  bool invoked = false;
};

struct FakeHeatPump {
  FakeHeatPumpStatus Status;
  FlowSetpointCall lastSetpoint;
  std::vector<FlowSetpointCall> callHistory;

  void SetFlowSetpoint(float flow, int mode, int zone);
  void Reset();
};

struct FakeMQTTClient {
  bool connectedFlag = false;
  bool connected() const { return connectedFlag; }
  void setConnected(bool value) { connectedFlag = value; }
};

struct DebugCapture {
  std::string topic;
  std::string message;
  bool retain = false;
};

extern UnitSettings unitSettings;
extern FakeHeatPump HeatPump;
extern FakeMQTTClient MQTTClient1;
extern FakeMQTTClient MQTTClient2;
extern bool PostDefrostTimer;
extern unsigned long postdfpreviousMillis;
extern float Z1_CurveFSP;
extern float Z2_CurveFSP;

namespace TestSupport {
void ResetEnvironment();
void SetMillis(unsigned long value);
void AdvanceMillis(unsigned long delta);
unsigned long CurrentMillis();
const std::vector<DebugCapture>& DebugPublishes();
void ClearDebugPublishes();
int CompCurveReportCount();
void ClearCompCurveReportCount();
} // namespace TestSupport

namespace Flags {
bool Has2Zone();
void SetHas2Zone(bool value);
} // namespace Flags

float roundToHalfDecimal(float value);
unsigned long millis();
void MQTTDebugPublish(const String& subtopic, const String& message, bool retain = false);
void CompCurveReport();

constexpr int HEATING_CONTROL_MODE_FLOW_TEMP = 0x01;
constexpr int ZONE1 = 1;
constexpr int ZONE2 = 2;
