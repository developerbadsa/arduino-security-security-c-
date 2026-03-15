#include "state_logic.h"

const char* stateToString(DeviceStateId st) {
  switch (st) {
    case ST_LOCKED: return "LOCKED";
    case ST_UNLOCKED: return "UNLOCKED";
    case ST_MOVING: return "MOVING";
    case ST_ALARM: return "ALARM";
    default: return "UNLOCKED";
  }
}

DeviceStateId getCurrentState_locked() {
  if (alarmLatched) return ST_ALARM;
  return locked ? ST_LOCKED : ST_UNLOCKED;
}
