#pragma once

#include "../core/app_state.h"

void gpsPump();
bool isGoodFix();
bool isInsideBangladesh(double lat, double lon);
void initIMU();
bool recalibrateImuBias();
void updateImuMotion();
float getFilteredSpeedKmph_locked();
bool isBikeStationary_locked();
bool isBikeStationary();
