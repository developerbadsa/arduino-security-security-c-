#pragma once

#include "../core/app_state.h"

void recordSamplePoint();
void recordHeartbeatPoint(const __FlashStringHelper* reason);
int buildCompressedTrack(TrackPoint* out, int maxOut, const TrackPoint* src, int srcCount);
