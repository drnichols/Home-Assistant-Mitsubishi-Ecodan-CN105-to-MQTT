// Lightweight helper for common feature/cascade flags
#pragma once

namespace Flags {

// Config vs runtime state
bool CascadeEnabled();
bool CascadeActive();

// Node role (runtime and config variants)
bool CascadeMaster();
bool CascadeSlave();

bool ConfigCascadeMaster();
bool ConfigCascadeSlave();

// Heat pump capability flags
bool HasCooling();
bool Has2Zone();

} // namespace Flags
