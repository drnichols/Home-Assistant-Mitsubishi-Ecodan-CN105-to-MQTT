// Shared helpers for reading and persisting CompCurve outdoor source settings.

void loadOutdoorSourceSettingsFromCompCurve() {
  if (unitSettings.CompCurve.isEmpty()) {
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, unitSettings.CompCurve);
  if (error) {
    DEBUG_PRINT("Failed to read CompCurve while loading outdoor settings: ");
    DEBUG_PRINTLN(error.c_str());
    return;
  }

  JsonVariant useLocal = doc["use_local_outdoor"];
  if (!useLocal.isNull()) {
    unitSettings.use_local_outdoor = useLocal.as<bool>();
  }

  JsonVariant cloudValue = doc["cloud_outdoor"];
  if (!cloudValue.isNull()) {
    unitSettings.cloud_outdoor = cloudValue.as<float>();
  }
}

bool persistUseLocalOutdoor(bool newValue) {
  if (unitSettings.CompCurve.isEmpty()) {
    unitSettings.CompCurve = "{}";
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, unitSettings.CompCurve);
  if (error) {
    DEBUG_PRINT("Failed to read CompCurve while persisting use_local_outdoor: ");
    DEBUG_PRINTLN(error.c_str());
    return false;
  }

  JsonVariant current = doc["use_local_outdoor"];
  bool changed = current.isNull() || current.as<bool>() != newValue;

  doc["use_local_outdoor"] = newValue;
  doc.shrinkToFit();
  unitSettings.CompCurve = String();
  serializeJson(doc, unitSettings.CompCurve);

  return changed;
}
