# SmartBike SMS Command Guide

To control your bike via SMS, you must include your **Device ID** and **Security PIN** in every message.

**Device ID**: `BIKE01` (Default)
**Security PIN**: `2234` (Default)

### Command Format
`[DeviceID] [PIN] [Command]`

### List of Commands

| Command | Action | Example |
| :--- | :--- | :--- |
| **`LOCK`** | Locks the bike | `BIKE01 2234 LOCK` |
| **`UNLOCK`** | Unlocks the bike | `BIKE01 2234 UNLOCK` |
| **`LOC`** / **`STATUS`** | Get current location & state | `BIKE01 2234 LOC` |
| **`AP ON`** | Turn on WiFi Hotspot (30m) | `BIKE01 2234 AP ON` |
| **`AP OFF`** | Turn off WiFi Hotspot | `BIKE01 2234 AP OFF` |

> [!IMPORTANT]
> - All commands are case-insensitive.
> - The bike will reply with a confirmation SMS for every valid command.
> - If GPS signal is lost, `LOC` will send the last known location (if under 1 hour old).
