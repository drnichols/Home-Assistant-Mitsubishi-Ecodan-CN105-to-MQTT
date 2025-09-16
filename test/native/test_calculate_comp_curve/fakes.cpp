#include "fakes.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

UnitSettings unitSettings{};
FakeHeatPump HeatPump{};
FakeMQTTClient MQTTClient1{};
FakeMQTTClient MQTTClient2{};
bool PostDefrostTimer = false;
unsigned long postdfpreviousMillis = 0;
float Z1_CurveFSP = 0;
float Z2_CurveFSP = 0;

namespace {
unsigned long gMillis = 0;
std::vector<DebugCapture> gDebugPublishes;
int gCompCurveReportCount = 0;
bool gHas2Zone = false;

std::string TrimTrailingZeros(std::string value) {
  auto pos = value.find('.');
  if (pos == std::string::npos) return value;
  while (!value.empty() && value.back() == '0') value.pop_back();
  if (!value.empty() && value.back() == '.') value.pop_back();
  return value;
}
} // namespace

std::string String::fromFloat(double value) {
  std::ostringstream oss;
  oss << std::setprecision(6) << std::fixed << value;
  return TrimTrailingZeros(oss.str());
}

void FakeHeatPump::SetFlowSetpoint(float flow, int mode, int zone) {
  lastSetpoint.flow = flow;
  lastSetpoint.mode = mode;
  lastSetpoint.zone = zone;
  lastSetpoint.invoked = true;
  callHistory.push_back(lastSetpoint);
}

void FakeHeatPump::Reset() {
  lastSetpoint = {};
  callHistory.clear();
}

namespace TestSupport {
void ResetEnvironment() {
  unitSettings = UnitSettings{};
  unitSettings.use_local_outdoor = true;
  unitSettings.CompCurve = String();
  HeatPump = FakeHeatPump{};
  PostDefrostTimer = false;
  postdfpreviousMillis = 0;
  Z1_CurveFSP = 0;
  Z2_CurveFSP = 0;
  gMillis = 0;
  gDebugPublishes.clear();
  gCompCurveReportCount = 0;
  gHas2Zone = false;
}

void SetMillis(unsigned long value) { gMillis = value; }

void AdvanceMillis(unsigned long delta) { gMillis += delta; }

unsigned long CurrentMillis() { return gMillis; }

const std::vector<DebugCapture>& DebugPublishes() { return gDebugPublishes; }

void ClearDebugPublishes() { gDebugPublishes.clear(); }

int CompCurveReportCount() { return gCompCurveReportCount; }

void ClearCompCurveReportCount() { gCompCurveReportCount = 0; }
} // namespace TestSupport

namespace Flags {
bool Has2Zone() { return gHas2Zone; }
void SetHas2Zone(bool value) {
  gHas2Zone = value;
  HeatPump.Status.Has2Zone = value;
}
} // namespace Flags

float roundToHalfDecimal(float value) { return static_cast<float>(std::round(value * 2.0f) / 2.0f); }

unsigned long millis() { return gMillis; }

void MQTTDebugPublish(const String& subtopic, const String& message, bool retain) {
  gDebugPublishes.push_back(DebugCapture{subtopic.str(), message.str(), retain});
}

void CompCurveReport() { ++gCompCurveReportCount; }
