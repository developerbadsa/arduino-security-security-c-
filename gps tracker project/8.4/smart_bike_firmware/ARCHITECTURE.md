# SmartBike Firmware Architecture

This firmware is split into focused modules so runtime behavior stays the same while maintenance gets easier.

## Entry Point

- `smart_bike_firmware.ino`
  - Owns `setup()` and `loop()`
  - Wires modules together
  - Keeps loop work non-blocking

## Modules

- `src/config/app_config.h`
  - Device, network, timing, pin, and debug constants

- `src/core/app_types.h`
  - Shared enums and message/track structs

- `src/core/app_state.h`
- `src/core/app_state.cpp`
  - Global runtime state
  - Shared FreeRTOS handles
  - Hardware object instances

- `src/core/state_logic.h`
- `src/core/state_logic.cpp`
  - Device state helpers such as `stateToString()` and `getCurrentState_locked()`

- `src/core/command_processor.h`
- `src/core/command_processor.cpp`
  - Command execution
  - Loop-to-network reporting queue bridge

- `src/actuators/actuators.h`
- `src/actuators/actuators.cpp`
  - Relay, LED, and buzzer behavior

- `src/sensors/sensors.h`
- `src/sensors/sensors.cpp`
  - GPS pumping
  - IMU initialization and motion detection
  - Speed filtering and stationary detection

- `src/tracking/tracking.h`
- `src/tracking/tracking.cpp`
  - Sample capture
  - Heartbeat capture
  - Track compression

- `src/security/security.h`
- `src/security/security.cpp`
  - Touch, knock, and sustained-motion alarm logic

- `src/portal/local_portal.h`
- `src/portal/local_portal.cpp`
- `src/portal/portal_html.h`
  - Local captive portal
  - Local control endpoints
  - Local status JSON

- `src/network/network.h`
- `src/network/network.cpp`
  - Wi-Fi maintenance
  - Internet probing
  - HTTP POSTs
  - Poll/push/report handling
  - FreeRTOS network task

## Concurrency Model

- `loop()` runs on Core 1 and must stay responsive
- `netTask()` runs on Core 0 and may block on HTTP/network work
- `stateMutex` protects shared state accessed by both cores
- `cmdQueue` delivers server commands from `netTask()` to `loop()`
- `reportQueue` delivers loop-side events back to `netTask()`

## Refactor Rule Of Thumb

- Put new sensor logic in `src/sensors`
- Put new actuator logic in `src/actuators`
- Put new server/API behavior in `src/network`
- Put new local UI/control behavior in `src/portal`
- Put shared state additions in `src/core/app_state.*`
