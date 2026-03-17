#pragma once

#include "app_state.h"

void queueReportFromLoop(const char* event, const char* cmdId);
void executeCommand(const char* cmd, const char* cmdId);
