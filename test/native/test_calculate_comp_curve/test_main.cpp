#include <string>

#include <unity.h>

#include "fakes.hpp"

void setUp(void) { TestSupport::ResetEnvironment(); }
void tearDown(void) {}

extern void CalculateCompCurve();

namespace {
const char* kSampleCompCurve = R"JSON({
  "zone1": {"active": true},
  "zone2": {"active": true},
  "base": {
    "zone1": {"curve": [
      {"outside": -5, "flow": 45},
      {"outside": 5, "flow": 35},
      {"outside": 15, "flow": 25}
    ]},
    "zone2": {"curve": [
      {"outside": -5, "flow": 40},
      {"outside": 5, "flow": 30},
      {"outside": 15, "flow": 20}
    ]}
  }
})JSON";

const char* kSampleCompCurve5pts = R"JSON({
  "zone1": {"active": true},
  "zone2": {"active": true},
  "base": {
    "zone1": {"curve": [
      {"outside": -15, "flow": 50},
      {"outside": -5, "flow": 40},
      {"outside": 5, "flow": 30},
      {"outside": 15, "flow": 20},
      {"outside": 25, "flow": 10}
    ]},
    "zone2": {"curve": [
      {"outside": -15, "flow": 50},
      {"outside": -5, "flow": 40},
      {"outside": 5, "flow": 30},
      {"outside": 15, "flow": 20},
      {"outside": 25, "flow": 10}
    ]}
  }
})JSON";

bool HasDebugMessage(const std::string& topic, const std::string& needle) {
  for (const auto& publish : TestSupport::DebugPublishes()) {
    if (publish.topic == topic && publish.message.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}
} // namespace

void test_reports_json_error_and_skips(void) {
  unitSettings.CompCurve = String("{ not json");

  CalculateCompCurve();

  TEST_ASSERT_TRUE_MESSAGE(HasDebugMessage("CalculateCompCurve/Error", "JSON parse error"),
                           "expected JSON error publish");
  TEST_ASSERT_EQUAL_INT(0, TestSupport::CompCurveReportCount());
  TEST_ASSERT_FALSE(HeatPump.lastSetpoint.invoked);
}

void test_skips_when_defrost_active(void) {
  unitSettings.CompCurve = String(kSampleCompCurve);
  HeatPump.Status.OutsideTemperature = 3;
  HeatPump.Status.Defrost = 1;

  CalculateCompCurve();

  TEST_ASSERT_TRUE_MESSAGE(HasDebugMessage("CalculateCompCurve/Skip", "defrost"),
                           "expected defrost skip publish");
  TEST_ASSERT_EQUAL_INT(0, TestSupport::CompCurveReportCount());
  TEST_ASSERT_FALSE(HeatPump.lastSetpoint.invoked);
}

void test_local_outdoor_interpolation_with_offsets(void) {
  unitSettings.CompCurve = String(kSampleCompCurve);
  HeatPump.Status.OutsideTemperature = 0;
  Flags::SetHas2Zone(true);
  HeatPump.Status.Simple2Zone = false;
  unitSettings.z1_temp_offset = 0.2f;
  unitSettings.z1_manual_offset = 0.1f;

  CalculateCompCurve();

  TEST_ASSERT_EQUAL_INT(1, TestSupport::CompCurveReportCount());
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(HeatPump.callHistory.size()));
  TEST_ASSERT_EQUAL_FLOAT(40.5f, Z1_CurveFSP);
  TEST_ASSERT_EQUAL_FLOAT(35.0f, Z2_CurveFSP);
  TEST_ASSERT_EQUAL_FLOAT(40.5f, HeatPump.Status.Zone1FlowTemperatureSetpoint);
  TEST_ASSERT_EQUAL_FLOAT(35.0f, HeatPump.Status.Zone2FlowTemperatureSetpoint);
  TEST_ASSERT_EQUAL_FLOAT(40.5f, HeatPump.callHistory[0].flow);
  TEST_ASSERT_EQUAL_INT(ZONE1, HeatPump.callHistory[0].zone);
  TEST_ASSERT_EQUAL_FLOAT(35.0f, HeatPump.callHistory[1].flow);
  TEST_ASSERT_EQUAL_INT(ZONE2, HeatPump.callHistory[1].zone);
}

void test_uses_cloud_outdoor_when_available(void) {
  unitSettings.CompCurve = String(kSampleCompCurve);
  unitSettings.use_local_outdoor = false;
  unitSettings.cloud_outdoor = 15;
  MQTTClient1.setConnected(true);
  Flags::SetHas2Zone(true);
  HeatPump.Status.Simple2Zone = false;

  CalculateCompCurve();

  TEST_ASSERT_EQUAL_FLOAT(25.0f, Z1_CurveFSP);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, Z2_CurveFSP);
  TEST_ASSERT_TRUE(HasDebugMessage("CalculateCompCurve/OATSource", "cloud"));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(HeatPump.callHistory.size()));
  TEST_ASSERT_EQUAL_FLOAT(25.0f, HeatPump.callHistory[0].flow);
  TEST_ASSERT_EQUAL_INT(ZONE1, HeatPump.callHistory[0].zone);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, HeatPump.callHistory[1].flow);
  TEST_ASSERT_EQUAL_INT(ZONE2, HeatPump.callHistory[1].zone);
  TEST_ASSERT_EQUAL(2, HeatPump.lastSetpoint.zone);
}

void test_calc_comp_curve_belowmin(void) {
  unitSettings.CompCurve = String(kSampleCompCurve);
  unitSettings.use_local_outdoor = false;
  unitSettings.cloud_outdoor = -25;
  MQTTClient1.setConnected(true);
  Flags::SetHas2Zone(true);
  HeatPump.Status.Simple2Zone = false;

  CalculateCompCurve();

  TEST_ASSERT_EQUAL_FLOAT(45.0f, Z1_CurveFSP);
  TEST_ASSERT_EQUAL_FLOAT(40.0f, Z2_CurveFSP);
  TEST_ASSERT_TRUE(HasDebugMessage("CalculateCompCurve/OATSource", "cloud"));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(HeatPump.callHistory.size()));
  TEST_ASSERT_EQUAL_FLOAT(45.0f, HeatPump.callHistory[0].flow);
  TEST_ASSERT_EQUAL_INT(ZONE1, HeatPump.callHistory[0].zone);
  TEST_ASSERT_EQUAL_FLOAT(40.0f, HeatPump.callHistory[1].flow);
  TEST_ASSERT_EQUAL_INT(ZONE2, HeatPump.callHistory[1].zone);
  TEST_ASSERT_EQUAL(2, HeatPump.lastSetpoint.zone);
}

void test_calc_comp_curve_min(void) {
  unitSettings.CompCurve = String(kSampleCompCurve);
  unitSettings.use_local_outdoor = false;
  unitSettings.cloud_outdoor = -5;
  MQTTClient1.setConnected(true);
  Flags::SetHas2Zone(true);
  HeatPump.Status.Simple2Zone = false;

  CalculateCompCurve();

  TEST_ASSERT_EQUAL_FLOAT(45.0f, Z1_CurveFSP);
  TEST_ASSERT_EQUAL_FLOAT(40.0f, Z2_CurveFSP);
  TEST_ASSERT_TRUE(HasDebugMessage("CalculateCompCurve/OATSource", "cloud"));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(HeatPump.callHistory.size()));
  TEST_ASSERT_EQUAL_FLOAT(45.0f, HeatPump.callHistory[0].flow);
  TEST_ASSERT_EQUAL_INT(ZONE1, HeatPump.callHistory[0].zone);
  TEST_ASSERT_EQUAL_FLOAT(40.0f, HeatPump.callHistory[1].flow);
  TEST_ASSERT_EQUAL_INT(ZONE2, HeatPump.callHistory[1].zone);
  TEST_ASSERT_EQUAL(2, HeatPump.lastSetpoint.zone);
}

void test_calc_comp_curve_minus10(void) {
  unitSettings.CompCurve = String(kSampleCompCurve5pts);
  unitSettings.use_local_outdoor = false;
  unitSettings.cloud_outdoor = -10;
  MQTTClient1.setConnected(true);
  Flags::SetHas2Zone(true);
  HeatPump.Status.Simple2Zone = false;

  CalculateCompCurve();

  TEST_ASSERT_EQUAL_FLOAT(45.0f, Z1_CurveFSP);
  TEST_ASSERT_EQUAL_FLOAT(45.0f, Z2_CurveFSP);
  TEST_ASSERT_TRUE(HasDebugMessage("CalculateCompCurve/OATSource", "cloud"));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(HeatPump.callHistory.size()));
  TEST_ASSERT_EQUAL_FLOAT(45.0f, HeatPump.callHistory[0].flow);
  TEST_ASSERT_EQUAL_INT(ZONE1, HeatPump.callHistory[0].zone);
  TEST_ASSERT_EQUAL_FLOAT(45.0f, HeatPump.callHistory[1].flow);
  TEST_ASSERT_EQUAL_INT(ZONE2, HeatPump.callHistory[1].zone);
  TEST_ASSERT_EQUAL(2, HeatPump.lastSetpoint.zone);
}

void test_calc_comp_curve_plus10(void) {
  unitSettings.CompCurve = String(kSampleCompCurve5pts);
  unitSettings.use_local_outdoor = false;
  unitSettings.cloud_outdoor = 10;
  MQTTClient1.setConnected(true);
  Flags::SetHas2Zone(true);
  HeatPump.Status.Simple2Zone = false;

  CalculateCompCurve();

  TEST_ASSERT_EQUAL_FLOAT(25.0f, Z1_CurveFSP);
  TEST_ASSERT_EQUAL_FLOAT(25.0f, Z2_CurveFSP);
  TEST_ASSERT_TRUE(HasDebugMessage("CalculateCompCurve/OATSource", "cloud"));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(HeatPump.callHistory.size()));
  TEST_ASSERT_EQUAL_FLOAT(25.0f, HeatPump.callHistory[0].flow);
  TEST_ASSERT_EQUAL_INT(ZONE1, HeatPump.callHistory[0].zone);
  TEST_ASSERT_EQUAL_FLOAT(25.0f, HeatPump.callHistory[1].flow);
  TEST_ASSERT_EQUAL_INT(ZONE2, HeatPump.callHistory[1].zone);
  TEST_ASSERT_EQUAL(2, HeatPump.lastSetpoint.zone);
}

void test_calc_comp_curve_mid(void) {
  unitSettings.CompCurve = String(kSampleCompCurve);
  unitSettings.use_local_outdoor = false;
  unitSettings.cloud_outdoor = 6;
  MQTTClient1.setConnected(true);
  Flags::SetHas2Zone(true);
  HeatPump.Status.Simple2Zone = false;

  CalculateCompCurve();

  TEST_ASSERT_EQUAL_FLOAT(34.0f, Z1_CurveFSP);
  TEST_ASSERT_EQUAL_FLOAT(29.0f, Z2_CurveFSP);
  TEST_ASSERT_TRUE(HasDebugMessage("CalculateCompCurve/OATSource", "cloud"));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(HeatPump.callHistory.size()));
  TEST_ASSERT_EQUAL_FLOAT(34.0f, HeatPump.callHistory[0].flow);
  TEST_ASSERT_EQUAL_INT(ZONE1, HeatPump.callHistory[0].zone);
  TEST_ASSERT_EQUAL_FLOAT(29.0f, HeatPump.callHistory[1].flow);
  TEST_ASSERT_EQUAL_INT(ZONE2, HeatPump.callHistory[1].zone);
  TEST_ASSERT_EQUAL(2, HeatPump.lastSetpoint.zone);
}

void test_calc_comp_curve_max(void) {
  unitSettings.CompCurve = String(kSampleCompCurve);
  unitSettings.use_local_outdoor = false;
  unitSettings.cloud_outdoor = 20;
  MQTTClient1.setConnected(true);
  Flags::SetHas2Zone(true);
  HeatPump.Status.Simple2Zone = false;

  CalculateCompCurve();

  TEST_ASSERT_EQUAL_FLOAT(25.0f, Z1_CurveFSP);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, Z2_CurveFSP);
  TEST_ASSERT_TRUE(HasDebugMessage("CalculateCompCurve/OATSource", "cloud"));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(HeatPump.callHistory.size()));
  TEST_ASSERT_EQUAL_FLOAT(25.0f, HeatPump.callHistory[0].flow);
  TEST_ASSERT_EQUAL_INT(ZONE1, HeatPump.callHistory[0].zone);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, HeatPump.callHistory[1].flow);
  TEST_ASSERT_EQUAL_INT(ZONE2, HeatPump.callHistory[1].zone);
  TEST_ASSERT_EQUAL(2, HeatPump.lastSetpoint.zone);
}

void test_calc_comp_curve_abovemax(void) {
  unitSettings.CompCurve = String(kSampleCompCurve);
  unitSettings.use_local_outdoor = false;
  unitSettings.cloud_outdoor = 30;
  MQTTClient1.setConnected(true);
  Flags::SetHas2Zone(true);
  HeatPump.Status.Simple2Zone = false;

  CalculateCompCurve();

  TEST_ASSERT_EQUAL_FLOAT(25.0f, Z1_CurveFSP);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, Z2_CurveFSP);
  TEST_ASSERT_TRUE(HasDebugMessage("CalculateCompCurve/OATSource", "cloud"));
  TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(HeatPump.callHistory.size()));
  TEST_ASSERT_EQUAL_FLOAT(25.0f, HeatPump.callHistory[0].flow);
  TEST_ASSERT_EQUAL_INT(ZONE1, HeatPump.callHistory[0].zone);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, HeatPump.callHistory[1].flow);
  TEST_ASSERT_EQUAL_INT(ZONE2, HeatPump.callHistory[1].zone);
  TEST_ASSERT_EQUAL(2, HeatPump.lastSetpoint.zone);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_reports_json_error_and_skips);
  RUN_TEST(test_skips_when_defrost_active);
  RUN_TEST(test_local_outdoor_interpolation_with_offsets);
  RUN_TEST(test_uses_cloud_outdoor_when_available);
  RUN_TEST(test_calc_comp_curve_belowmin);
  RUN_TEST(test_calc_comp_curve_min);
  RUN_TEST(test_calc_comp_curve_mid);
  RUN_TEST(test_calc_comp_curve_max);
  RUN_TEST(test_calc_comp_curve_abovemax);
  RUN_TEST(test_calc_comp_curve_minus10);
  RUN_TEST(test_calc_comp_curve_plus10);
  return UNITY_END();
}
