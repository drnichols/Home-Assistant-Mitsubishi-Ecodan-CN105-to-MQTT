void CalculateCompCurve() {
  DEBUG_PRINTLN("Performing Compensation Curve Calculation");
  MQTTDebugPublish("CalculateCompCurve/Start", String("Start"));
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, unitSettings.CompCurve);
  if (error) {
    DEBUG_PRINT("Failed to read: ");
    DEBUG_PRINTLN(error.c_str());
    MQTTDebugPublish("CalculateCompCurve/Error", String("JSON parse error: ") + error.c_str());
  } else {
    unitSettings.z1_active = doc["zone1"]["active"]; // Transfer JSON to Struct Bool
    unitSettings.z2_active = doc["zone2"]["active"];
    MQTTDebugPublish("CalculateCompCurve/ZoneStates",
                     String("z1=") + (unitSettings.z1_active ? "1" : "0") + ", z2=" + (unitSettings.z2_active ? "1" : "0"));
    //if (!unitSettings.z1_active && !unitSettings.z2_active) { return; }  // Only calculates (saves time, if mode enabled)
    //else
    {
      float OutsideAirTemperature = 0;

      if (!unitSettings.use_local_outdoor && (MQTTClient1.connected() || MQTTClient2.connected())) { // Determine Outdoor Temperature Input
        // Use the runtime setting captured from MQTT (not the CompCurve JSON)
        OutsideAirTemperature = unitSettings.cloud_outdoor;
        MQTTDebugPublish("CalculateCompCurve/OATSource", String("cloud, ") + String(OutsideAirTemperature));
      } else {
        OutsideAirTemperature = HeatPump.Status.OutsideTemperature;
        MQTTDebugPublish("CalculateCompCurve/OATSource", String("local, ") + String(OutsideAirTemperature));

        if (HeatPump.Status.Defrost != 0 || ((PostDefrostTimer) && (millis() - postdfpreviousMillis < 240000))) { // To allow sensor to stabilise after influence from the defrost
          DEBUG_PRINTLN("Compensation Curve Calculation skipped due to defrost");
          MQTTDebugPublish("CalculateCompCurve/Skip", String("defrost active or cooldown"));
          return; // If currently defrosting or less than 4 minutes post-defrost skip re-calculation
        } else {
          PostDefrostTimer = false;
        }
      }

      int z1_points = doc["base"]["zone1"]["curve"].size() - 1; // How many points are there specified on the curve
      MQTTDebugPublish("CalculateCompCurve/Z1/Points", String(z1_points + 1));
      for (int i = 0; i <= z1_points; i++) { // Iterate through the points
        float tmp_o_1 = doc["base"]["zone1"]["curve"][i]["outside"]; // Outside Temperature for this point
        if ((i == 0) && (OutsideAirTemperature <= tmp_o_1)) { // On the first point, this determines the Maximum Flow Temp
          Z1_CurveFSP = doc["base"]["zone1"]["curve"][i]["flow"]; // Set to Max Flow Temp
          DEBUG_PRINTLN("Z1_CurveFSP set to Max");
        } else if ((i == z1_points) && (OutsideAirTemperature >= tmp_o_1)) { // The last point determines the Minimum Flow Temp
          Z1_CurveFSP = doc["base"]["zone1"]["curve"][i]["flow"]; // Set to Min Flow Temp
          DEBUG_PRINTLN("Z1_CurveFSP set to Min");
        } else { // Intermediate Flow Points are calculated
          float tmp_o_2 = doc["base"]["zone1"]["curve"][i + 1]["outside"]; // Outside Temperature of the next point (warmer)
          if ((OutsideAirTemperature >= tmp_o_1) && (OutsideAirTemperature <= tmp_o_2)) { // Validate the current outside temp value is in the correct range between points
            float y1 = doc["base"]["zone1"]["curve"][i + 1]["flow"]; // Calculate the slope using the formula: m = (y2 - y1) / (x2 - x1)
            float y2 = doc["base"]["zone1"]["curve"][i]["flow"]; //
            float z1_delta_y = y2 - y1;                          // y2-y1
            float z1_delta_x = tmp_o_1 - tmp_o_2;                // x2-x1
            float z1_m = 0;                                      //
            if (z1_delta_x != 0) { z1_m = z1_delta_y / z1_delta_x; }  // Prevent Div by 0          m = y2-y1 / x2-x1
            float z1_c = y2 - (z1_m * tmp_o_1);                // c = y-mx at point
            Z1_CurveFSP = (z1_m * OutsideAirTemperature) + z1_c; // y = mx+c
            MQTTDebugPublish("CalculateCompCurve/Z1/Intermediate", String("m=") + String(z1_m) + ", c=" + String(z1_c) + ", fsp=" + String(Z1_CurveFSP));
          }
        }
      }
      MQTTDebugPublish("CalculateCompCurve/Z1/BeforeOffsets", String(Z1_CurveFSP));

      int z2_points = doc["base"]["zone2"]["curve"].size() - 1; // How many points are there specified on the curve
      MQTTDebugPublish("CalculateCompCurve/Z2/Points", String(z2_points + 1));
      for (int i = 0; i <= z2_points; i++) {
        float tmp_o_1 = doc["base"]["zone2"]["curve"][i]["outside"];
        if ((i == 0) && (OutsideAirTemperature <= tmp_o_1)) { // Max Flow Temp
          Z2_CurveFSP = doc["base"]["zone2"]["curve"][i]["flow"];
        } else if ((i == z2_points) &&
                   (OutsideAirTemperature >= tmp_o_1)) { // Min Flow Temp
          Z2_CurveFSP = doc["base"]["zone2"]["curve"][i]["flow"];
        } else {
          float tmp_o_2 = doc["base"]["zone2"]["curve"][i + 1]["outside"];
          if ((OutsideAirTemperature >= tmp_o_1) && (OutsideAirTemperature <= tmp_o_2)) {
            float y1 = doc["base"]["zone2"]["curve"][i + 1]["flow"]; // Calculate the slope using the formula: m = (y2 - y1) / (x2 - x1)
            float y2 = doc["base"]["zone2"]["curve"][i]["flow"];
            float z2_delta_y = y2 - y1;
            float z2_delta_x = tmp_o_1 - tmp_o_2;
            float z2_m = 0; //
            if (z2_delta_x != 0) { z2_m = z2_delta_y / z2_delta_x;} // Prevent Div by 0          m = y2-y1 / x2-x1
            float z2_c = y2 - (z2_m * tmp_o_1);                // c = y-mx
            Z2_CurveFSP = (z2_m * OutsideAirTemperature) + z2_c; // y = mx+c
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
    MQTTDebugPublish("CalculateCompCurve/Z1/AfterOffsets", String(Z1_CurveFSP));
    MQTTDebugPublish("CalculateCompCurve/Z2/AfterOffsets", String(Z2_CurveFSP));

    // Write the Flow Setpoints to Heat Pump
    if (unitSettings.z1_active) {
      DEBUG_PRINT("Performing Compensation Curve Calculation - Writing to Heat Pump for Zone 1: ");
      DEBUG_PRINT(Z1_CurveFSP);
      DEBUG_PRINTLN("");
      HeatPump.SetFlowSetpoint(Z1_CurveFSP, HEATING_CONTROL_MODE_FLOW_TEMP, ZONE1);
      HeatPump.Status.Zone1FlowTemperatureSetpoint = Z1_CurveFSP;
      MQTTDebugPublish("CalculateCompCurve/Write/Z1", String(Z1_CurveFSP));
    }
    if (unitSettings.z2_active && Flags::Has2Zone() && !HeatPump.Status.Simple2Zone) {  // User must have Complex 2 zone to set different flow temp in different zones
      DEBUG_PRINT("Performing Compensation Curve Calculation - Writing to Heat Pump for Zone 2: ");
      DEBUG_PRINT(Z2_CurveFSP);
      DEBUG_PRINTLN("");
      HeatPump.SetFlowSetpoint(Z2_CurveFSP, HEATING_CONTROL_MODE_FLOW_TEMP, ZONE2);
      HeatPump.Status.Zone2FlowTemperatureSetpoint = Z2_CurveFSP;
      MQTTDebugPublish("CalculateCompCurve/Write/Z2", String(Z2_CurveFSP));
    }
    CompCurveReport();
    MQTTDebugPublish("CalculateCompCurve/End", String("End"));
  }
}
