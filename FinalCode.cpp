#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <SPI.h>
#include <ICM_20948.h>

// BLE Libraries
#include <NimBLEDevice.h>

#include "ui.h"
#include "screens.h"

// TinyGSM for SIM7000G (GPS + Cellular)
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>

/******************** PIN DEFINITIONS ********************/

// --- CELLULAR PINS (SIM7000G) ---
#define MODEM_PWRKEY  38  
#define MODEM_DTR     9   
#define MODEM_RX      18  
#define MODEM_TX      17  
#define MODEM_BAUD    115200

// --- DISPLAY PINS (TFT) ---
#define TFT_BL        3   

// --- SPI PINS (Shared: LCD + IMU) ---
#define SPI_MISO      5
#define SPI_MOSI      6
#define SPI_SCLK      7
#define LCD_CS_PIN    12
#define IMU_CS_PIN    15

// --- TOUCH PINS (FT6336) ---
#define TOUCH_INT     4   
#define TOUCH_RST     21  
#define TOUCH_SDA     10   
#define TOUCH_SCL     11   

// --- SENSOR PINS ---
#define REED_SWITCH_PIN 39 
#define CHARGE_PIN      1

// --- IMU SPI Settings ---
#define IMU_SPI_FREQ  4000000

/******************** SETTINGS ********************/
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320
#define FT6336_ADDR 0x38

// Cellular settings
const char ntfyTopic[] = "Ryder-Capstone-Debug";
const char apn[]       = "hologram";
const char server[]    = "ntfy.sh";
const int port         = 80;

// Wheel parameters
const float WHEEL_CIRCUMFERENCE = 2.100f;
const unsigned long ZERO_TIMEOUT = 2500;

// Moving average parameters for Speed
const int MAX_MA_SIZE = 10;
const int MA_VERY_SLOW = 10;
const int MA_SLOW = 7;
const int MA_MEDIUM = 5;
const int MA_FAST = 3;

const float SPEED_VERY_SLOW = 8.0f;
const float SPEED_SLOW = 15.0f;
const float SPEED_MEDIUM = 25.0f;

/******************** FEATURE TOGGLES ********************/
#define FEAT_COMPASS    1
#define FEAT_SPEED      1
#define FEAT_IMU        1
#define FEAT_LCD        1  // Set to 1 to enable display
#define FEAT_BT         1
#define FEAT_GPS        1
#define FEAT_HR         1
#define FEAT_CHARGE     1
#define FEAT_ATTEND     1
#define FEAT_EMERGENCY  1
#define FEAT_CELLULAR   1

/******************** TIMING HELPER ********************/
static inline bool runEvery(uint32_t &t, const uint32_t dt){
  uint32_t now = millis();
  if(now - t >= dt){ t = now; return true; }
  return false;
}

/******************** TELEMETRY DATA MODEL ********************/
struct Telemetry {
  // Raw sensor values
  float headingDeg;
  float speedMps;
  float leanDeg;
  float turnDeg;
  double latitude;
  double longitude;
  bool gpsValid;
  uint16_t bpm;
  bool charging;
  bool btConnected;
  bool attendanceChecked;
  
  // Emergency State
  bool emergencyTriggered; 
  bool emergencyActive;    
  bool emergencySent;      
  
  // Cellular State
  bool cellularConnected;
  bool cellularSending;
  int signalQuality;
  
  // GUI Strings
  char bpm_str[32];
  char turn_angle_str[32];
  char tilt_str[32];
  char speed_str[32];
  char gps_coord_str[64];
} telem;

/******************** SPEED SENSOR STATE ********************/
struct SpeedSensor {
  volatile unsigned long lastTriggerTime = 0;
  volatile unsigned long triggerIntervals[MAX_MA_SIZE] = {0};
  volatile int intervalIndex = 0;
  volatile int intervalCount = 0;
  volatile bool lastState = HIGH;
  int currentMASize = MA_MEDIUM;
} speedSensor;

/******************** IMU GLOBALS ********************/
ICM_20948_SPI imu;
bool imuInitialized = false;

// IMU data accumulation for averaging
struct IMUAccum {
  float accX_sum = 0, accY_sum = 0, accZ_sum = 0;
  int count = 0;
} imuAccum;

/******************** BLE GLOBALS & UUIDS ********************/
static NimBLEAddress targetDeviceAddress("00:08:e1:2a:12:34", 0);
static NimBLEUUID serviceUUID("180D");     
static NimBLEUUID charUUID("2A37");        

bool bleConnectRequested = false;
bool bleConnected = false;
static NimBLEClient* pClient = nullptr;

/******************** MODEM GLOBALS ********************/
HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);     
TinyGsmClient client(modem);

/******************** TFT & TOUCH GLOBALS ********************/
TFT_eSPI tft = TFT_eSPI();
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCREEN_WIDTH * 10];

bool touchEnabled = false;
uint16_t touchX = 0, touchY = 0;
bool touched = false;

/******************** APP FSM ********************/
enum class Phase : uint8_t {
  BOOT,
  READ_SENSORS,
  PROCESS,
  RENDER,
  COMM,
  IDLE
};

struct AppFSM {
  Phase phase = Phase::BOOT;
  uint32_t loopCounter = 0;
  
  void begin() { 
    phase = Phase::READ_SENSORS; 
  }
  
  void next() {
    switch(phase) {
      case Phase::BOOT:        phase = Phase::READ_SENSORS; break;
      case Phase::READ_SENSORS: phase = Phase::PROCESS;     break;
      case Phase::PROCESS:     phase = Phase::RENDER;       break;
      case Phase::RENDER:      phase = Phase::COMM;         break;
      case Phase::COMM:        phase = Phase::IDLE;         break;
      case Phase::IDLE:        phase = Phase::READ_SENSORS; break;
    }
  }
} app;

/******************** INLINED VARS & ACTIONS ********************/
static char var_bpm_val[32] = "-- bpm";
static char var_turn_angle[32] = "--";
static char var_tilt_str[32] = "--";
static char var_speed_val[32] = "-- mph";
static char var_gps_coord[64] = "Searching...";
static bool var_is_hidden = true;
static bool var_is_charging = false;

extern "C" {
  const char* get_var_bpm_val() { return var_bpm_val; }
  const char* get_var_turn_angle() { return var_turn_angle; }
  const char* get_var_tilt_str() { return var_tilt_str; }
  const char* get_var_speed_val() { return var_speed_val; }
  const char* get_var_gps_coord() { return var_gps_coord; }
  bool get_var_is_hidden() { return var_is_hidden; }
  bool get_var_is_charging() { return var_is_charging; }

  void set_var_bpm_val(const char* value) { strncpy(var_bpm_val, value, 31); }
  void set_var_turn_angle(const char* value) { strncpy(var_turn_angle, value, 31); }
  void set_var_tilt_str(const char* value) { strncpy(var_tilt_str, value, 31); }
  void set_var_speed_val(const char* value) { strncpy(var_speed_val, value, 31); }
  void set_var_gps_coord(const char* value) { strncpy(var_gps_coord, value, 63); }
  void set_var_is_hidden(bool value) { var_is_hidden = value; }
  void set_var_is_charging(bool value) { var_is_charging = value; }

  void action_emergency_window(lv_event_t* e) {
    Serial.println("[UI] Emergency button pressed");
    loadScreen(SCREEN_ID_EMERGENCY_WINDOW);
  }

  void action_send_sms(lv_event_t* e) {
    Serial.println("[UI] EMERGENCY CONFIRMED");
    telem.emergencyTriggered = true;
    loadScreen(SCREEN_ID_MAIN);
  }

  void action_main(lv_event_t* e) {
    loadScreen(SCREEN_ID_MAIN);
  }

  void action_disconnect_popup(lv_event_t* e) {
    loadScreen(SCREEN_ID_DISCONNECT_POPUP);
  }

  void action_connect_ecg(lv_event_t* e) {
    Serial.println("[UI] ECG Connect Requested (Direct Connect)...");
    bleConnectRequested = true;
    loadScreen(SCREEN_ID_MAIN); 
  }
}

/******************** BLE CALLBACKS & LOGIC ********************/
static void notifyCallback(
  NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    if (length >= 2) {
      uint8_t flags = pData[0];
      uint16_t bpmVal = 0;
      if ((flags & 0x01) == 0) bpmVal = pData[1];
      else if(length >= 3) bpmVal = pData[1] | (pData[2] << 8);
      telem.bpm = bpmVal;
    }
}

class MyClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) {
    Serial.println("[BLE] Connected!");
    bleConnected = true;
    telem.btConnected = true;
  }

  void onDisconnect(NimBLEClient* pclient) {
    Serial.println("[BLE] Disconnected.");
    bleConnected = false;
    telem.btConnected = false;
    telem.bpm = 0;
  }
};

bool connectToServer() {
    Serial.print("[BLE] Connecting to: ");
    Serial.println(targetDeviceAddress.toString().c_str());

    for(int i = 1; i <= 5; i++) {
        Serial.printf("\n[BLE] Attempt %d/5... ", i);
        
        if(pClient == nullptr) {
            pClient = NimBLEDevice::createClient();
            pClient->setClientCallbacks(new MyClientCallback(), false);
            pClient->setConnectTimeout(10); 
            pClient->setConnectionParams(12, 12, 0, 51);
        }

        if(pClient->connect(targetDeviceAddress, false)) {
            Serial.println("Success!");
            goto setup_services;
        }
        
        Serial.println("Failed (Immediate rejection).");
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        Serial.println("[BLE] Waiting 2s for strap advertisement window...");
        delay(2000); 
    }
    
    Serial.println("\n[BLE] Could not connect after 5 attempts.");
    return false;

    setup_services:
    Serial.println("[BLE] Discovering Services...");

    NimBLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.println("[BLE] Service 0x180D not found");
      pClient->disconnect();
      return false;
    }

    NimBLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.println("[BLE] Char 0x2A37 not found");
      pClient->disconnect();
      return false;
    }

    if(pRemoteCharacteristic->canNotify()) {
      if(pRemoteCharacteristic->subscribe(true, notifyCallback)) {
         Serial.println("[BLE] Notifications enabled. HR Data incoming...");
         return true;
      }
    }
    return true;
}

/******************** REED SWITCH ISR ********************/
void IRAM_ATTR reedSwitchISR() {
  unsigned long currentTime = millis();
  int currentState = digitalRead(REED_SWITCH_PIN);
  
  if (currentState == LOW && speedSensor.lastState == HIGH) {
    if (speedSensor.lastTriggerTime > 0) {
      unsigned long interval = currentTime - speedSensor.lastTriggerTime;
      if (interval > 50) {
        speedSensor.triggerIntervals[speedSensor.intervalIndex] = interval;
        speedSensor.intervalIndex = (speedSensor.intervalIndex + 1) % MAX_MA_SIZE;
        if (speedSensor.intervalCount < MAX_MA_SIZE) {
          speedSensor.intervalCount++;
        }
      }
    }
    speedSensor.lastTriggerTime = currentTime;
  }
  speedSensor.lastState = currentState;
}

/******************** MODEM FUNCTIONS ********************/
void pulsePWRKEY() {
  Serial.println("[HW] Pulsing PWRKEY (1.2s)...");
  digitalWrite(MODEM_PWRKEY, HIGH); 
  delay(1200); 
  digitalWrite(MODEM_PWRKEY, LOW);
  Serial.println("[HW] Waiting 5s for boot/voltage stabilization...");
  delay(5000); 
}

bool syncBaudRate(unsigned long timeout_ms) {
  Serial.print("[HW] Attempting Auto-Baud Sync...");
  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    SerialAT.println("AT");
    delay(500); 
    if (SerialAT.available()) {
      String resp = SerialAT.readString();
      if (resp.indexOf("OK") != -1) {
        Serial.println(" [OK]");
        return true; 
      }
    }
    Serial.print(".");
  }
  Serial.println(" [TIMEOUT]");
  return false;
}

void wakeModem() {
  digitalWrite(MODEM_DTR, LOW);
  delay(100); 
}

void sleepModem() {
  digitalWrite(MODEM_DTR, HIGH);
}

bool reconnectNetwork() {
  Serial.print("[NET] Reconnecting...");
  if (!modem.waitForNetwork(180000L)) {
    Serial.println(" FAILED");
    return false;
  }
  
  if (!modem.isGprsConnected()) {
    Serial.print("GPRS: Reconnecting...");
    if (!modem.gprsConnect(apn, "", "")) {
      Serial.println(" FAILED");
      return false;
    }
  }
  Serial.println(" OK");
  return true;
}

bool sendNtfyNotification(String title, String message, String priority = "default") {
  wakeModem();
  Serial.println("\n[CELL] --- Sending Notification ---");
  
  if (!modem.isGprsConnected()) {
    Serial.println("[CELL] GPRS not connected. Reconnecting...");
    if (!reconnectNetwork()) {
      sleepModem();
      return false;
    }
  }
  
  Serial.print("[CELL] Connecting to ");
  Serial.print(server);
  
  if (!client.connect(server, port)) {
    Serial.println("... Connection failed!");
    sleepModem();
    return false;
  }
  Serial.println("... Connected!");
  
  String body = message;
  
  client.print("POST /");
  client.print(ntfyTopic);
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(server);
  client.print("Title: ");
  client.println(title);
  client.print("Priority: ");
  client.println(priority);
  client.println("Content-Type: text/plain");
  client.print("Content-Length: ");
  client.println(body.length());
  client.println("Connection: close");
  client.println();
  client.print(body);
  
  Serial.println("[CELL] Request sent, waiting...");
  
  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 10000L) {
    while (client.available()) {
      client.readStringUntil('\n');
    }
  }
  client.stop();
  Serial.println("[CELL] Notification Sent!");
  
  sleepModem();
  return true;
}

/******************** SENSOR READING FUNCTIONS ********************/

bool readCompass() {
#if FEAT_COMPASS
  static uint32_t lastRead = 0;
  if (runEvery(lastRead, 100)) {
    telem.headingDeg = 0.0f; 
    return true;
  }
#endif
  return false;
}

bool readSpeed() {
#if FEAT_SPEED
  static uint32_t lastRead = 0;
  if (runEvery(lastRead, 500)) {
    unsigned long currentTime = millis();
    
    float speedKmh = telem.speedMps * 3.6f;
    if (speedKmh < SPEED_VERY_SLOW) speedSensor.currentMASize = MA_VERY_SLOW;
    else if (speedKmh < SPEED_SLOW) speedSensor.currentMASize = MA_SLOW;
    else if (speedKmh < SPEED_MEDIUM) speedSensor.currentMASize = MA_MEDIUM;
    else speedSensor.currentMASize = MA_FAST;
    
    float speedMps = 0.0f;
    if (speedSensor.intervalCount >= speedSensor.currentMASize) {
      unsigned long avgInterval = 0;
      for (int i = 0; i < speedSensor.currentMASize; i++) {
        int idx = (speedSensor.intervalIndex - 1 - i + MAX_MA_SIZE) % MAX_MA_SIZE;
        avgInterval += speedSensor.triggerIntervals[idx];
      }
      avgInterval /= speedSensor.currentMASize;
      
      if (avgInterval > 0) {
        speedMps = WHEEL_CIRCUMFERENCE / (avgInterval / 1000.0f);
      }
    }
    
    if (currentTime - speedSensor.lastTriggerTime > ZERO_TIMEOUT && speedSensor.lastTriggerTime > 0) {
      speedMps = 0.0f;
    }
    
    telem.speedMps = speedMps;
    return true;
  }
#endif
  return false;
}

bool readImu() {
#if FEAT_IMU
  static uint32_t lastRead = 0;
  static uint32_t lastProcess = 0;
  
  if (!imuInitialized) return false;
  
  // Read IMU at 100Hz and accumulate
  if (runEvery(lastRead, 10)) {
    digitalWrite(LCD_CS_PIN, HIGH);  // Ensure LCD is deselected
    
    if (imu.dataReady()) {
      imu.getAGMT();
      
      imuAccum.accX_sum += imu.accX();
      imuAccum.accY_sum += imu.accY();
      imuAccum.accZ_sum += imu.accZ();
      imuAccum.count++;
    }
  }
  
  // Process averaged data every 100ms
  if (runEvery(lastProcess, 100)) {
    if (imuAccum.count > 0) {
      float accX = imuAccum.accX_sum / imuAccum.count;
      float accY = imuAccum.accY_sum / imuAccum.count;
      float accZ = imuAccum.accZ_sum / imuAccum.count;
      
      // Roll = Turn (LEFT negative, RIGHT positive)
      telem.turnDeg = atan2(accY, accZ) * 180.0 / PI;
      
      // Pitch = Lean (FORWARD positive, BACKWARD negative)
      telem.leanDeg = atan2(-accX, sqrt(accY*accY + accZ*accZ)) * 180.0 / PI;
      
      // Reset accumulators
      imuAccum.accX_sum = 0;
      imuAccum.accY_sum = 0;
      imuAccum.accZ_sum = 0;
      imuAccum.count = 0;
    }
    return true;
  }
#endif
  return false;
}

bool readGps() {
#if FEAT_GPS
  static uint32_t lastRead = 0;
  if (runEvery(lastRead, 1000)) {
    wakeModem();
    
    modem.sendAT("+CGNSINF");
    String response = "";
    
    if (modem.waitResponse(10000L, response) == 1) {
      int firstComma = response.indexOf(',');
      if (firstComma != -1) {
        // Extract fix status (second field)
        int secondComma = response.indexOf(',', firstComma + 1);
        String fixStatus = response.substring(firstComma + 1, secondComma);
        
        if (fixStatus == "1") {
          telem.gpsValid = true;
          
          // Build comma position array
          int commaPos[20];
          int commaCount = 0;
          int pos = firstComma;
          
          while (pos != -1 && commaCount < 20) {
            commaPos[commaCount++] = pos;
            pos = response.indexOf(',', pos + 1);
          }
          
          if (commaCount >= 6) {
            // Latitude (field 3)
            telem.latitude = response.substring(commaPos[2] + 1, commaPos[3]).toDouble();
            // Longitude (field 4)
            telem.longitude = response.substring(commaPos[3] + 1, commaPos[4]).toDouble();
          }
        } else {
          telem.gpsValid = false;
        }
      }
    }
    sleepModem();
    return true;
  }
#endif
  return false;
}

bool readCharging() {
#if FEAT_CHARGE
  static uint32_t lastRead = 0;
  if (runEvery(lastRead, 2000)) {
    telem.charging = (digitalRead(CHARGE_PIN) == LOW);
    return true;
  }
#endif
  return false;
}

bool readAttendance() {
#if FEAT_ATTEND
  static uint32_t lastRead = 0;
  if (runEvery(lastRead, 5000)) {
    telem.attendanceChecked = false; 
    return true;
  }
#endif
  return false;
}

bool readEmergencyButton() {
#if FEAT_EMERGENCY
  static uint32_t lastRead = 0;
  static uint32_t lastMessageTime = 0;
  const uint32_t MESSAGE_INTERVAL = 30000;
  
  if (runEvery(lastRead, 100)) {
    if (telem.emergencyTriggered && !telem.emergencyActive) {
      telem.emergencyActive = true;
      telem.emergencySent = false;
      lastMessageTime = 0;
      Serial.println("\n[EMERGENCY] EMERGENCY ACTIVATED!");
      telem.emergencyTriggered = false;
    }
    
    if (telem.emergencyActive && telem.emergencySent) {
      if (millis() - lastMessageTime >= MESSAGE_INTERVAL) {
        Serial.println("[EMERGENCY] Resending periodic update...");
        telem.emergencySent = false; 
        lastMessageTime = millis();
      }
    }
    
    if (telem.emergencyActive && telem.emergencySent && lastMessageTime == 0) {
      lastMessageTime = millis();
    }
    return true;
  }
#endif
  return false;
}

/******************** PROCESS & RENDER ********************/
void processData() {
  snprintf(telem.bpm_str, sizeof(telem.bpm_str), "%u bpm", telem.bpm);
  
  // Turn angle with direction
  const char* turnDir = (telem.turnDeg < 0) ? "LEFT" : "RIGHT";
  snprintf(telem.turn_angle_str, sizeof(telem.turn_angle_str), "%.1f %s", fabs(telem.turnDeg), turnDir);
  
  // Lean angle with direction
  const char* leanDir = (telem.leanDeg > 0) ? "FWD" : "BWD";
  snprintf(telem.tilt_str, sizeof(telem.tilt_str), "%.1f %s", fabs(telem.leanDeg), leanDir);
  
  float speedMph = telem.speedMps * 2.237f;
  snprintf(telem.speed_str, sizeof(telem.speed_str), "%.1f mph", speedMph);
  
  if (telem.gpsValid) {
    snprintf(telem.gps_coord_str, sizeof(telem.gps_coord_str), 
             "%.6f, %.6f", telem.latitude, telem.longitude);
  } else {
    snprintf(telem.gps_coord_str, sizeof(telem.gps_coord_str), "Searching...");
  }
}

void updateGuiVars() {
  set_var_bpm_val(telem.bpm_str);
  set_var_turn_angle(telem.turn_angle_str);
  set_var_tilt_str(telem.tilt_str);
  set_var_speed_val(telem.speed_str);
  set_var_gps_coord(telem.gps_coord_str);
  set_var_is_hidden(!telem.btConnected); 
  set_var_is_charging(!telem.charging);   
}

/******************** COMMS ********************/
bool commBluetooth() {
#if FEAT_BT
  if (bleConnectRequested) {
    if (!bleConnected) {
       if (connectToServer()) {
           Serial.println("[BLE] Connection Established.");
       } else {
           Serial.println("[BLE] Final Failure. Check Strap Battery.");
       }
    }
    bleConnectRequested = false;
  }
  telem.btConnected = bleConnected;
  return true;
#endif
  return false;
}

bool commCellular() {
#if FEAT_CELLULAR
  static uint32_t lastStatusCheck = 0;
  
  if (runEvery(lastStatusCheck, 10000)) {
    wakeModem();
    if (modem.isGprsConnected()) {
      telem.cellularConnected = true;
      telem.signalQuality = modem.getSignalQuality();
    } else {
      telem.cellularConnected = false;
      Serial.println("[CELL] Lost connection. Reconnecting...");
      reconnectNetwork();
    }
    sleepModem();
  }
  
  if (telem.emergencyActive && !telem.emergencySent) {
    telem.cellularSending = true;
    
    String message = "EMERGENCY BUTTON TRIGGERED ON RYDR DEVICE\n\n";
    
    if (telem.gpsValid) {
      message += "Location: " + String(telem.latitude, 6) + ", " + String(telem.longitude, 6) + "\n\n";
      message += "Google Maps:\n";
      message += "https://maps.google.com/?q=" + String(telem.latitude, 6) + "," + String(telem.longitude, 6);
      
      Serial.println("[EMERGENCY] GPS Coordinates:");
      Serial.printf("  Lat: %.6f, Lon: %.6f\n", telem.latitude, telem.longitude);
      Serial.printf("  https://maps.google.com/?q=%.6f,%.6f\n", telem.latitude, telem.longitude);
    } else {
      message += "Location: GPS not available";
      Serial.println("[EMERGENCY] GPS not available");
    }
    
    if (sendNtfyNotification("RYDR EMERGENCY ALERT", message, "urgent")) {
      telem.emergencySent = true;
    } 
    
    telem.cellularSending = false;
  }
  return true;
#endif
  return false;
}

/******************** TOUCH INPUT ********************/
void readTouch() {
  if (!touchEnabled) return;
  Wire.beginTransmission(FT6336_ADDR);
  Wire.write(0x02);
  Wire.endTransmission();
  Wire.requestFrom(FT6336_ADDR, 1);
  uint8_t touches = Wire.read() & 0x0F;
  
  if (touches > 0) {
    Wire.beginTransmission(FT6336_ADDR);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(FT6336_ADDR, 4);
    uint8_t data[4];
    for (int i = 0; i < 4; i++) data[i] = Wire.read();
    
    uint16_t raw_x = ((data[0] & 0x0F) << 8) | data[1];
    uint16_t raw_y = ((data[2] & 0x0F) << 8) | data[3];
    
    touchX = raw_y;
    touchY = 320 - raw_x;
    if (touchX >= SCREEN_WIDTH) touchX = SCREEN_WIDTH - 1;
    if (touchY >= SCREEN_HEIGHT) touchY = SCREEN_HEIGHT - 1;
    touched = true;
  } else {
    touched = false;
  }
}

void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  
  digitalWrite(IMU_CS_PIN, HIGH);  // Ensure IMU is deselected
  
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)color_p, w * h, true);
  tft.endWrite();
  lv_disp_flush_ready(disp_drv);
}

void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data) {
  readTouch();
  if (touched) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

/******************** INITIALIZATION ********************/

void initSPI() {
  Serial.println("[SPI] Initializing shared SPI bus...");
  
  // Configure CS pins first - both HIGH (deselected)
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(IMU_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(IMU_CS_PIN, HIGH);
  
  // Initialize SPI with custom pins
  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, -1);
  
  Serial.printf("[SPI] MISO:%d MOSI:%d SCLK:%d LCD_CS:%d IMU_CS:%d\n", 
                SPI_MISO, SPI_MOSI, SPI_SCLK, LCD_CS_PIN, IMU_CS_PIN);
}

void initIMU() {
  Serial.println("[IMU] Initializing ICM-20948...");
  
  digitalWrite(LCD_CS_PIN, HIGH);  // Ensure LCD is deselected
  
  ICM_20948_Status_e stat = imu.begin(IMU_CS_PIN, SPI, IMU_SPI_FREQ);
  
  if (stat != ICM_20948_Stat_Ok) {
    Serial.println("[IMU] ERROR: Failed to initialize ICM-20948!");
    Serial.printf("[IMU] Status: %d\n", stat);
    imuInitialized = false;
    return;
  }
  
  Serial.print("[IMU] WHO_AM_I: 0x");
  Serial.println(imu.getWhoAmI(), HEX);
  
  // Software reset
  imu.swReset();
  delay(100);
  
  // Wake up and configure
  imu.sleep(false);
  imu.lowPower(false);
  imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  
  // Configure full scale
  ICM_20948_fss_t fss;
  fss.a = gpm4;
  fss.g = dps500;
  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
  
  // Configure DLPF
  ICM_20948_dlpcfg_t dlp;
  dlp.a = acc_d50bw4_n68bw8;
  dlp.g = gyr_d51bw2_n73bw3;
  imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlp);
  imu.enableDLPF(ICM_20948_Internal_Acc, true);
  imu.enableDLPF(ICM_20948_Internal_Gyr, true);
  
  imuInitialized = true;
  Serial.println("[IMU] ICM-20948 initialized successfully!");
}

void initDisplay() {
  Serial.println("[DISP] Initializing display...");
  
  // Backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  
  // Ensure IMU is deselected before LCD init
  digitalWrite(IMU_CS_PIN, HIGH);
  
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.invertDisplay(true);
  
  // Touch Controller Reset
  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, HIGH); delay(20);
  digitalWrite(TOUCH_RST, LOW); delay(20);
  digitalWrite(TOUCH_RST, HIGH); delay(500);
  
  // Detect Touch
  Wire.beginTransmission(FT6336_ADDR);
  if (Wire.endTransmission() == 0) {
    touchEnabled = true;
    Serial.println("[DISP] Touch found!");
  }
  
  // LVGL Init
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, SCREEN_WIDTH * 10);
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_WIDTH;
  disp_drv.ver_res = SCREEN_HEIGHT;
  disp_drv.flush_cb = disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read;
  lv_indev_drv_register(&indev_drv);
  
  Serial.println("[DISP] Display initialized!");
}

void initSpeedSensor() {
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REED_SWITCH_PIN), reedSwitchISR, CHANGE);
}

void initCellularAndGPS() {
  Serial.println("[INIT] Initializing SIM7000G...");
  
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_PWRKEY, LOW);
  digitalWrite(MODEM_DTR, LOW);
  
  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(100);
  
  pulsePWRKEY();
  
  if (!syncBaudRate(6000)) {
    Serial.println("[INIT] Sync failed. Retrying pulse...");
    pulsePWRKEY();
    if (!syncBaudRate(6000)) {
      Serial.println("[INIT] Modem dead.");
      return;
    }
  }
  
  if (!modem.init()) {
    Serial.println("[INIT] TinyGSM init failed");
    return;
  }
  
  if (!reconnectNetwork()) {
     Serial.println("[INIT] Network connect failed (will retry in loop)");
  }
  
  modem.sendAT("+CGNSPWR=1");
  modem.waitResponse();
  modem.sendAT("+CGNSURC=0");
  modem.waitResponse();
  
  modem.sendAT("+CSCLK=1");
  modem.waitResponse(10000L);
  
  sendNtfyNotification("Ryder Rev2", "System Booted", "high");
}

/******************** FSM UPDATE ********************/
void updateFSM() {
  switch(app.phase) {
    case Phase::BOOT: break;
    
    case Phase::READ_SENSORS:
      readCompass();
      readSpeed();
      readImu();
      readGps();
      readCharging();
      readAttendance();
      readEmergencyButton();
      break;

    case Phase::PROCESS:
      processData();
      updateGuiVars();
      break;

    case Phase::RENDER:
      digitalWrite(IMU_CS_PIN, HIGH);  // Ensure IMU deselected for LCD
      #if FEAT_LCD
      lv_timer_handler();
      ui_tick();
      #endif
      break;

    case Phase::COMM:
      commBluetooth();
      commCellular();
      break;

    case Phase::IDLE:
      delay(1);
      break;
  }
  
  app.next();
  app.loopCounter++;
}

/******************** MAIN LOOP ********************/
void setup() {
  Serial.begin(115200);
  
  // Wait for serial
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000)) {
    delay(100);
  }
  
  Serial.println("\n--- RYDR Bike HUD (Rev 2 + IMU) ---");
  
  pinMode(CHARGE_PIN, INPUT_PULLUP);
  
  // Initialize SPI bus first (shared by LCD and IMU)
  initSPI();
  
  initSpeedSensor();
  initDisplay();
  
  // Initialize IMU after display (both share SPI)
  initIMU();
  
  ui_init();
  
  // Init BLE
  Serial.println("[INIT] Initializing BLE...");
  NimBLEDevice::init("Ryder Bike Tracker");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  
  initCellularAndGPS();
  
  app.begin();
  Serial.println("[INIT] Setup complete!");
}

void loop() {
  updateFSM();
  
  static uint32_t lastDebug = 0;
  if (runEvery(lastDebug, 5000)) {
    Serial.println("\n========== DEBUG ==========");
    Serial.printf("Loop: %lu\n", app.loopCounter);
    
    // Orientation (IMU)
    Serial.println("[IMU]");
    Serial.printf("  Turn: %s | Lean: %s\n", telem.turn_angle_str, telem.tilt_str);
    
    // Speed
    Serial.println("[SPEED]");
    Serial.printf("  Speed: %s (%.2f m/s) | MA Size: %d | Samples: %d\n", 
      telem.speed_str, telem.speedMps, speedSensor.currentMASize, speedSensor.intervalCount);
    
    // GPS
    Serial.println("[GPS]");
    if (telem.gpsValid) {
      Serial.printf("  Status: VALID | Lat: %.6f | Lon: %.6f\n", telem.latitude, telem.longitude);
      Serial.printf("  https://maps.google.com/?q=%.6f,%.6f\n", telem.latitude, telem.longitude);
    } else {
      Serial.println("  Status: Searching...");
    }
    
    // Cellular
    Serial.println("[CELLULAR]");
    Serial.printf("  Connected: %s | Signal: %d | Sending: %s\n", 
      telem.cellularConnected ? "YES" : "NO", 
      telem.signalQuality,
      telem.cellularSending ? "YES" : "NO");
      
    Serial.println("[BLE HR]");
    Serial.printf("  Connected: %s | BPM: %d\n", 
      bleConnected ? "YES" : "NO",
      telem.bpm);

    Serial.println("===========================\n");
  }
}
