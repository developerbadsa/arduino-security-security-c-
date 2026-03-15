#include "app_state.h"

SemaphoreHandle_t stateMutex = nullptr;
QueueHandle_t cmdQueue = nullptr;
QueueHandle_t reportQueue = nullptr;
TaskHandle_t netTaskHandle = nullptr;

HardwareSerial SerialGPS(1);
TinyGPSPlus gps;
bfs::Mpu6500 imu(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);

volatile bool locked = false;
volatile bool armed = false;
volatile bool alarmLatched = false;
volatile uint32_t alarmCooldownUntil = 0;

TrackPoint rawBuf[MAX_RAW_POINTS];
int rawCount = 0;

float filteredSpeed = 0.0f;
bool hasFilteredInit = false;
int lowSpeedCount = 0;
bool imuMoving = false;

bool imuReady = false;
GyroBias imuBias;
uint32_t imuMovingHoldUntil = 0;
uint32_t lastImuReadAt = 0;
float lastADeltaG = 0.0f;
float lastGMag = 0.0f;
uint32_t tamperMotionSince = 0;

uint8_t knockCount = 0;
uint32_t firstKnockAt = 0;
uint32_t lastKnockAt = 0;
uint32_t touchSince = 0;
uint32_t lastTouchFireAt = 0;

uint32_t lastSample = 0;
uint32_t lastHeartbeat = 0;

volatile bool gInternetOk = false;
volatile uint32_t lastInternetOkAt = 0;
uint32_t lastNetCheckAt = 0;
uint32_t lastPush = 0;
uint32_t lastPoll = 0;
bool wifiConnecting = false;
bool wifiWasConnected = false;
uint32_t wifiConnectStartedAt = 0;
uint32_t nextNetTryAt = 0;
uint32_t netBackoffMs = 15000;
uint8_t netFailStreak = 0;
bool pendingReport = false;
char pendingReportEvent[24] = {};
char pendingReportCmdId[40] = {};
uint32_t lastReportTry = 0;

WebServer localServer(80);
DNSServer localDns;
bool localApActive = false;
uint32_t localApLastActivityAt = 0;
uint8_t localPinFailCount = 0;
uint32_t localPinLockedUntil = 0;
