#include "sensors.h"

#include "../logging/logger.h"

namespace {

inline float mag3(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

constexpr uint8_t IMU_WHO_AM_I_REG = 0x75;

int readWhoAmI(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(IMU_WHO_AM_I_REG);
  if (Wire.endTransmission(false) != 0) return -1;

  const int readCount = Wire.requestFrom(static_cast<int>(address), 1);
  if (readCount != 1 || !Wire.available()) return -1;
  return Wire.read();
}

const __FlashStringHelper* describeWhoAmI(int whoAmI) {
  switch (whoAmI) {
    case 0x70: return F("MPU6500");
    case 0x68: return F("MPU6050");
    case 0x71: return F("MPU9250");
    case 0x73: return F("MPU9255");
    default: return F("UNKNOWN");
  }
}

bool captureGyroBias(uint16_t sampleCount, uint16_t sampleDelayMs, GyroBias* outBias) {
  if (!outBias) return false;

  float sx = 0.0f;
  float sy = 0.0f;
  float sz = 0.0f;
  uint16_t ok = 0;

  for (int i = 0; i < 20; i++) {
    imu.Read();
    delay(3);
  }

  for (uint16_t i = 0; i < sampleCount; i++) {
    if (imu.Read()) {
      sx += imu.gyro_x_radps();
      sy += imu.gyro_y_radps();
      sz += imu.gyro_z_radps();
      ok++;
    }
    delay(sampleDelayMs);
  }

  if (!ok) return false;

  outBias->x = sx / ok;
  outBias->y = sy / ok;
  outBias->z = sz / ok;
  return true;
}

}  // namespace

void gpsPump() {
  int count = 0;
  while (SerialGPS.available() && count < 64) {
    gps.encode(static_cast<char>(SerialGPS.read()));
    count++;
  }
}

bool isGoodFix() {
  if (!gps.location.isValid()) return false;
  if (gps.location.age() > 5000) return false;
  if (!gps.satellites.isValid() || gps.satellites.value() < 4) return false;
  if (gps.hdop.isValid() && gps.hdop.hdop() > 3.0) return false;
  return true;
}

bool isInsideBangladesh(double lat, double lon) {
  return (lat >= 20.0 && lat <= 27.0 && lon >= 88.0 && lon <= 93.0);
}

void initIMU() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

#if !SMARTBIKE_HAS_MPU6500
  imuReady = false;
  logPrintln("IMU", F("driver not installed, IMU disabled"));
  return;
#endif

  const int whoPrimary = readWhoAmI(bfs::Mpu6500::I2C_ADDR_PRIM);
  const int whoSecondary = readWhoAmI(bfs::Mpu6500::I2C_ADDR_SEC);

  if (whoPrimary >= 0) {
    const String whoPrimaryText = String(describeWhoAmI(whoPrimary));
    logPrintf("IMU", "scan 0x68=0x%02X %s", whoPrimary, whoPrimaryText.c_str());
  } else {
    logPrintln("IMU", F("scan 0x68=NA"));
  }

  if (whoSecondary >= 0) {
    const String whoSecondaryText = String(describeWhoAmI(whoSecondary));
    logPrintf("IMU", "scan 0x69=0x%02X %s", whoSecondary, whoSecondaryText.c_str());
  } else {
    logPrintln("IMU", F("scan 0x69=NA"));
  }

  bfs::Mpu6500::I2cAddr imuAddr = bfs::Mpu6500::I2C_ADDR_PRIM;
  if (whoPrimary == 0x70) {
    imuAddr = bfs::Mpu6500::I2C_ADDR_PRIM;
  } else if (whoSecondary == 0x70) {
    imuAddr = bfs::Mpu6500::I2C_ADDR_SEC;
  } else {
    imuReady = false;
    logPrintln("IMU", F("no MPU6500 detected on 0x68/0x69"));
    return;
  }

  imu.Config(&Wire, imuAddr);

  if (!imu.Begin()) {
    imuReady = false;
    logPrintln("IMU", F("init failed"));
    return;
  }
  imu.ConfigAccelRange(bfs::Mpu6500::ACCEL_RANGE_4G);
  imu.ConfigGyroRange(bfs::Mpu6500::GYRO_RANGE_500DPS);
  imu.ConfigDlpfBandwidth(bfs::Mpu6500::DLPF_BANDWIDTH_20HZ);
  imu.ConfigSrd(19);
  imuReady = true;

  GyroBias newBias = {};
  if (!captureGyroBias(CAL_SAMPLES, CAL_DELAY_MS, &newBias)) {
    imuBias = {};
    imuReady = false;
    logPrintln("IMU", F("bias calibration failed"));
    return;
  }
  imuBias = newBias;
  logPrintf("IMU", "ready addr=0x%02X", static_cast<int>(imuAddr));
}

bool recalibrateImuBias() {
  if (!imuReady) return false;

  GyroBias newBias = {};
  if (!captureGyroBias(ARM_CAL_SAMPLES, ARM_CAL_DELAY_MS, &newBias)) return false;

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;
  imuBias = newBias;
  imuMoving = false;
  imuMovingHoldUntil = 0;
  lastImuReadAt = 0;
  lastADeltaG = 0.0f;
  lastGMag = 0.0f;
  xSemaphoreGive(stateMutex);
  return true;
}

void updateImuMotion() {
  if (!imuReady) return;
  const uint32_t now = millis();
  if (now - lastImuReadAt < IMU_READ_EVERY_MS) return;
  lastImuReadAt = now;
  if (!imu.Read()) return;

  const float ax = imu.accel_x_mps2();
  const float ay = imu.accel_y_mps2();
  const float az = imu.accel_z_mps2();
  const float gx = imu.gyro_x_radps() - imuBias.x;
  const float gy = imu.gyro_y_radps() - imuBias.y;
  const float gz = imu.gyro_z_radps() - imuBias.z;

  const float aMag = mag3(ax, ay, az);
  const float gMag = mag3(gx, gy, gz);
  const float aDeltaG = fabsf(aMag - G_MPS2) / G_MPS2;

  lastADeltaG = aDeltaG;
  lastGMag = gMag;

  bool newMoving = imuMoving;
  if (!imuMoving) {
    if ((gMag > GYRO_MOVING_ON) || (aDeltaG > ACCEL_DELTA_ON)) {
      newMoving = true;
      imuMovingHoldUntil = now + MOVING_HOLD_MS;
    }
  } else if (now >= imuMovingHoldUntil) {
    if ((gMag < GYRO_MOVING_OFF) && (aDeltaG < ACCEL_DELTA_OFF)) {
      newMoving = false;
    } else {
      imuMovingHoldUntil = now + 120;
    }
  }

  if (newMoving != imuMoving) {
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      imuMoving = newMoving;
      xSemaphoreGive(stateMutex);
    }
  }
}

float getFilteredSpeedKmph_locked() {
  if (!gps.speed.isValid() || gps.speed.age() > 3000) {
    return hasFilteredInit ? filteredSpeed : 0.0f;
  }
  const float raw = gps.speed.kmph();
  if (raw > MAX_SPEED_KMPH) return filteredSpeed;

  if (!hasFilteredInit) {
    filteredSpeed = raw;
    hasFilteredInit = true;
  } else {
    filteredSpeed = SPEED_ALPHA * raw + (1.0f - SPEED_ALPHA) * filteredSpeed;
  }

  if (filteredSpeed < MIN_SPEED_KMPH) {
    if (++lowSpeedCount >= STOP_COUNT_REQ) filteredSpeed = 0.0f;
  } else {
    lowSpeedCount = 0;
  }
  return filteredSpeed;
}

bool isBikeStationary_locked() {
  const float spd = getFilteredSpeedKmph_locked();
  if (!imuReady) return spd < 1.0f;
  return (!imuMoving && spd < 3.0f);
}

bool isBikeStationary() {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) return true;
  const bool still = isBikeStationary_locked();
  xSemaphoreGive(stateMutex);
  return still;
}
