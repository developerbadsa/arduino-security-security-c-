#pragma once

#include "app_state.h"

const char* stateToString(DeviceStateId st);
DeviceStateId getCurrentState_locked();
