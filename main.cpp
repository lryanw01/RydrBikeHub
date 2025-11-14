/*
  Bike HUD with FSM and LVGL GUI - SKELETON VERSION
  State Machine: READ → PROCESS → RENDER → COMM → IDLE
  
  TEAM MEMBER INSTRUCTIONS:
  1. Find your sensor section in the SENSOR READING FUNCTIONS area
  2. Implement your readXXX() function with actual sensor code
  3. Update the corresponding processXXX() function to format your data
  4. Test independently, then integrate
*/

#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Wire.h>

#include "ui.h"
#include "vars.h"

/******************** DISPLAY PINS ********************/
#define TFT_BL 38

// Touch pins
#define TOUCH_INT 13  
#define TOUCH_RST 21  
#define TOUCH_SDA 10   
#define TOUCH_SCL 11   

// Display settings
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320

// Touch I2C address
#define FT6336_ADDR 0x38

/******************** FEATURE TOGGLES ********************/
// Set to 0 to disable a feature during development
#define FEAT_COMPASS    1
#define FEAT_SPEED      1
#define FEAT_IMU        1
#define FEAT_LCD        1
#define FEAT_BT         1
#define FEAT_GPS        1
#define FEAT_HR         1
#define FEAT_HAPTIC     1
#define FEAT_CHARGE     1
#define FEAT_ATTEND     1

/******************** TIMING HELPER ********************/
// Use this in your sensor functions for non-blocking periodic reads
// Example: if (runEvery(lastRead, 100)) { /* read sensor every 100ms */ }
static inline bool runEvery(uint32_t &t, const uint32_t dt){
  uint32_t now = millis();
  if(now - t >= dt){ t = now; return true; }
  return false;
}

/******************** TELEMETRY DATA MODEL ********************/
struct Telemetry {
  // Raw sensor values - UPDATE THESE IN YOUR readXXX() FUNCTIONS
  float headingDeg;      // Compass: 0-360 degrees
  float speedMps;        // Speed: meters per second
  float leanDeg;         // IMU: lean angle in degrees (+ = right, - = left)
  double latitude;       // GPS: decimal degrees
  double longitude;      // GPS: decimal degrees
  uint16_t bpm;          // Heart Rate: beats per minute
  bool charging;         // Charging status: true = charging
  bool btConnected;      // Bluetooth: true = connected
  bool attendanceChecked;// Attendance: true = present
  
  // Processed string values for GUI - UPDATED IN processData()
  char bpm_str[32];
  char turn_angle_str[32];
  char tilt_str[32];
  char speed_str[32];
  char gps_coord_str[64];
} telem;

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

/******************** SENSOR READING FUNCTIONS ********************/
/*
  INSTRUCTIONS FOR SENSOR IMPLEMENTATION:
  1. Use static local variables to track timing (see examples)
  2. Use runEvery() for non-blocking periodic reads
  3. Update the corresponding telem.XXX value
  4. Return true if sensor was read successfully
  5. Keep functions SHORT and NON-BLOCKING (no delay()!)
*/

//═══════════════════════════════════════════════════════════════
// COMPASS / MAGNETOMETER
// Team Member: _______________
// Sensor: QMC5883L / HMC5883L / Other: _______________
// I2C Address: _______________
//═══════════════════════════════════════════════════════════════
bool readCompass() {
#if FEAT_COMPASS
  static uint32_t lastRead = 0;
  
  // TODO: Read compass sensor every 100ms (adjust as needed)
  if (runEvery(lastRead, 100)) {
    
    // ┌─────────────────────────────────────────────────────┐
    // │ ADD YOUR COMPASS READING CODE HERE                  │
    // │                                                      │
    // │ 1. Read magnetometer X, Y, Z values                 │
    // │ 2. Calculate heading angle (0-360 degrees)          │
    // │ 3. Apply calibration/correction if needed           │
    // │ 4. Update: telem.headingDeg = calculated_heading    │
    // │                                                      │
    // │ Example libraries:                                   │
    // │   - QMC5883LCompass                                 │
    // │   - Adafruit_HMC5883_U                              │
    // └─────────────────────────────────────────────────────┘
    
    // REMOVE THIS DEFAULT VALUE WHEN IMPLEMENTING:
    telem.headingDeg = 0.0f;  // Replace with actual sensor read
    
    return true;
  }
#endif
  return false;
}

//═══════════════════════════════════════════════════════════════
// SPEED SENSOR (Reed Switch)
// Team Member: _______________
// Pin: _______________
// Wheel Circumference: _______________ meters
//═══════════════════════════════════════════════════════════════
bool readSpeed() {
#if FEAT_SPEED
  static uint32_t lastRead = 0;
  
  // TODO: Calculate speed from reed switch pulses
  if (runEvery(lastRead, 500)) {
    
    // ┌─────────────────────────────────────────────────────┐
    // │ ADD YOUR SPEED CALCULATION CODE HERE                │
    // │                                                      │
    // │ Method 1: Interrupt-based pulse counting            │
    // │   - attachInterrupt() on reed switch pin            │
    // │   - Count pulses in ISR                             │
    // │   - Calculate speed = (pulses * circumference) / time│
    // │                                                      │
    // │ Method 2: Time between pulses                       │
    // │   - Measure time between falling/rising edges       │
    // │   - Calculate speed = circumference / time          │
    // │                                                      │
    // │ 3. Update: telem.speedMps = calculated_speed        │
    // └─────────────────────────────────────────────────────┘
    
    // REMOVE THIS DEFAULT VALUE WHEN IMPLEMENTING:
    telem.speedMps = 0.0f;  // Replace with actual speed calculation
    
    return true;
  }
#endif
  return false;
}

//═══════════════════════════════════════════════════════════════
// IMU (Lean/Tilt Angle)
// Team Member: _______________
// Sensor: MPU6050 / ICM20948 / Other: _______________
// I2C Address: _______________
//═══════════════════════════════════════════════════════════════
bool readImu() {
#if FEAT_IMU
  static uint32_t lastRead = 0;
  
  // TODO: Read IMU and calculate lean angle every 100ms
  if (runEvery(lastRead, 100)) {
    
    // ┌─────────────────────────────────────────────────────┐
    // │ ADD YOUR IMU READING CODE HERE                      │
    // │                                                      │
    // │ 1. Read accelerometer X, Y, Z values                │
    // │ 2. Read gyroscope X, Y, Z values (optional)         │
    // │ 3. Calculate roll/pitch angle                       │
    // │    roll = atan2(accelY, accelZ) * 180/PI            │
    // │    pitch = atan2(-accelX, sqrt(Y²+Z²)) * 180/PI     │
    // │ 4. Apply complementary/Kalman filter (optional)     │
    // │ 5. Update: telem.leanDeg = calculated_angle         │
    // │                                                      │
    // │ Example libraries:                                   │
    // │   - MPU6050_tockn                                   │
    // │   - Adafruit_MPU6050                                │
    // │   - ICM20948_WE                                     │
    // └─────────────────────────────────────────────────────┘
    
    // REMOVE THIS DEFAULT VALUE WHEN IMPLEMENTING:
    telem.leanDeg = 0.0f;  // Replace with actual IMU read
    
    return true;
  }
#endif
  return false;
}

//═══════════════════════════════════════════════════════════════
// GPS MODULE
// Team Member: _______________
// Module: NEO-6M / NEO-M8N / Other: _______________
// UART Pins: TX=___ RX=___
//═══════════════════════════════════════════════════════════════
bool readGps() {
#if FEAT_GPS
  static uint32_t lastRead = 0;
  
  // TODO: Parse GPS NMEA data
  if (runEvery(lastRead, 1000)) {
    
    // ┌─────────────────────────────────────────────────────┐
    // │ ADD YOUR GPS PARSING CODE HERE                      │
    // │                                                      │
    // │ 1. Read NMEA sentences from GPS module serial       │
    // │ 2. Parse GGA/RMC sentences for lat/lon             │
    // │ 3. Check for valid fix (location.isValid())        │
    // │ 4. Update:                                          │
    // │    telem.latitude = gps.location.lat()              │
    // │    telem.longitude = gps.location.lng()             │
    // │                                                      │
    // │ Example library:                                     │
    // │   - TinyGPS++ (recommended)                         │
    // │                                                      │
    // │ Don't forget to call gps.encode(Serial.read())      │
    // │ frequently to process incoming data!                │
    // └─────────────────────────────────────────────────────┘
    
    // REMOVE THESE DEFAULT VALUES WHEN IMPLEMENTING:
    telem.latitude = 42.3398;   // Replace with actual GPS lat
    telem.longitude = -71.0892; // Replace with actual GPS lon
    
    return true;
  }
#endif
  return false;
}

//═══════════════════════════════════════════════════════════════
// HEART RATE MONITOR
// Team Member: _______________
// Sensor: MAX30102 / Polar H10 BLE / Other: _______________
// Interface: I2C / BLE
//═══════════════════════════════════════════════════════════════
bool readHeartRate() {
#if FEAT_HR
  static uint32_t lastRead = 0;
  
  // TODO: Read heart rate sensor every 1 second
  if (runEvery(lastRead, 1000)) {
    
    // ┌─────────────────────────────────────────────────────┐
    // │ ADD YOUR HEART RATE READING CODE HERE               │
    // │                                                      │
    // │ Option A: MAX30102 Optical Sensor                   │
    // │   1. Read IR and Red LED values                     │
    // │   2. Apply peak detection algorithm                 │
    // │   3. Calculate BPM from peak intervals              │
    // │                                                      │
    // │ Option B: BLE Heart Rate Monitor (Polar H10)        │
    // │   1. Connect via BLE                                │
    // │   2. Subscribe to HR measurement characteristic     │
    // │   3. Parse standard BLE HR format                   │
    // │                                                      │
    // │ 4. Update: telem.bpm = calculated_bpm               │
    // │                                                      │
    // │ Example libraries:                                   │
    // │   - MAX30105 (SparkFun)                             │
    // │   - ESP32 BLE Arduino                               │
    // └─────────────────────────────────────────────────────┘
    
    // REMOVE THIS DEFAULT VALUE WHEN IMPLEMENTING:
    telem.bpm = 0;  // Replace with actual HR read
    
    return true;
  }
#endif
  return false;
}

//═══════════════════════════════════════════════════════════════
// CHARGING STATUS
// Team Member: _______________
// Pin: _______________ (or battery gauge I2C)
// Method: Digital pin / Fuel gauge IC
//═══════════════════════════════════════════════════════════════
bool readCharging() {
#if FEAT_CHARGE
  static uint32_t lastRead = 0;
  
  // TODO: Read charging status every 2 seconds
  if (runEvery(lastRead, 2000)) {
    
    // ┌─────────────────────────────────────────────────────┐
    // │ ADD YOUR CHARGING DETECTION CODE HERE               │
    // │                                                      │
    // │ Method 1: Simple digital pin                        │
    // │   telem.charging = digitalRead(CHARGE_DETECT_PIN)   │
    // │                                                      │
    // │ Method 2: Battery fuel gauge (MAX17048, etc)        │
    // │   1. Read battery gauge via I2C                     │
    // │   2. Check charging status register                 │
    // │   3. Update telem.charging based on status          │
    // │                                                      │
    // │ Method 3: Voltage monitoring                        │
    // │   1. Read battery voltage via ADC                   │
    // │   2. If voltage increasing -> charging = true       │
    // └─────────────────────────────────────────────────────┘
    
    // REMOVE THIS DEFAULT VALUE WHEN IMPLEMENTING:
    telem.charging = false;  // Replace with actual charging status
    
    return true;
  }
#endif
  return false;
}

//═══════════════════════════════════════════════════════════════
// ATTENDANCE CHECKING
// Team Member: _______________
// Method: BLE Proximity / GPS Geofence
//═══════════════════════════════════════════════════════════════
bool readAttendance() {
#if FEAT_ATTEND
  static uint32_t lastRead = 0;
  
  // TODO: Check attendance every 5 seconds
  if (runEvery(lastRead, 5000)) {
    
    // ┌─────────────────────────────────────────────────────┐
    // │ ADD YOUR ATTENDANCE CHECKING CODE HERE              │
    // │                                                      │
    // │ Method 1: BLE Proximity Detection                   │
    // │   1. Scan for specific BLE beacon/device            │
    // │   2. Check RSSI (signal strength)                   │
    // │   3. If beacon found and close -> present           │
    // │                                                      │
    // │ Method 2: GPS Geofence                              │
    // │   1. Get current GPS coordinates                    │
    // │   2. Calculate distance to target location          │
    // │   3. If within radius -> present                    │
    // │                                                      │
    // │ 4. Update: telem.attendanceChecked = is_present     │
    // └─────────────────────────────────────────────────────┘
    
    // REMOVE THIS DEFAULT VALUE WHEN IMPLEMENTING:
    telem.attendanceChecked = false;  // Replace with actual check
    
    return true;
  }
#endif
  return false;
}

/******************** PROCESSING FUNCTIONS ********************/
/*
  INSTRUCTIONS FOR PROCESSING:
  Convert raw sensor values to formatted STRING values for GUI display.
  All GUI variables MUST be strings with appropriate units.
*/

void processData() {
  // ┌─────────────────────────────────────────────────────┐
  // │ PROCESS BPM (Heart Rate)                            │
  // │ Input:  telem.bpm (uint16_t)                        │
  // │ Output: telem.bpm_str (string with "bpm" unit)      │
  // └─────────────────────────────────────────────────────┘
  snprintf(telem.bpm_str, sizeof(telem.bpm_str), "%u bpm", telem.bpm);
  
  // ┌─────────────────────────────────────────────────────┐
  // │ PROCESS TURN ANGLE (Compass Heading)                │
  // │ Input:  telem.headingDeg (float, 0-360°)            │
  // │ Output: telem.turn_angle_str (string with "°")      │
  // │                                                      │
  // │ Optional enhancements:                               │
  // │ - Add cardinal directions (N, NE, E, etc)           │
  // │ - Add turn arrow indicators (← →)                   │
  // └─────────────────────────────────────────────────────┘
  snprintf(telem.turn_angle_str, sizeof(telem.turn_angle_str), "%.0f°", telem.headingDeg);
  
  // ┌─────────────────────────────────────────────────────┐
  // │ PROCESS TILT/LEAN ANGLE (IMU)                       │
  // │ Input:  telem.leanDeg (float, degrees)              │
  // │ Output: telem.tilt_str (string with "°")            │
  // │                                                      │
  // │ Optional enhancements:                               │
  // │ - Add visual indicators (/ | \)                     │
  // │ - Color coding based on lean severity               │
  // └─────────────────────────────────────────────────────┘
  snprintf(telem.tilt_str, sizeof(telem.tilt_str), "%.1f°", telem.leanDeg);
  
  // ┌─────────────────────────────────────────────────────┐
  // │ PROCESS SPEED                                        │
  // │ Input:  telem.speedMps (float, meters/second)       │
  // │ Output: telem.speed_str (string in mph)             │
  // │                                                      │
  // │ Conversion: 1 m/s = 2.237 mph                       │
  // │                                                      │
  // │ Optional: Allow switching between mph/km/h          │
  // └─────────────────────────────────────────────────────┘
  float speedMph = telem.speedMps * 2.237f;
  snprintf(telem.speed_str, sizeof(telem.speed_str), "%.1f mph", speedMph);
  
  // ┌─────────────────────────────────────────────────────┐
  // │ PROCESS GPS COORDINATES                              │
  // │ Input:  telem.latitude, telem.longitude (double)    │
  // │ Output: telem.gps_coord_str (DMS format string)     │
  // │                                                      │
  // │ Current format: DD MM' SS" N/S DD MM' SS" E/W       │
  // │                                                      │
  // │ Optional formats:                                    │
  // │ - Decimal degrees: "42.3398° N, 71.0892° W"        │
  // │ - Simplified DMS: "42°20'N 71°05'W"                │
  // │ - Plus code / what3words integration                │
  // └─────────────────────────────────────────────────────┘
  
  // Convert decimal degrees to Degrees Minutes Seconds
  float latAbs = fabs(telem.latitude);
  float lonAbs = fabs(telem.longitude);
  
  int latDeg = (int)latAbs;
  float latMinFloat = (latAbs - latDeg) * 60.0f;
  int latMin = (int)latMinFloat;
  int latSec = (int)((latMinFloat - latMin) * 60.0f);
  
  int lonDeg = (int)lonAbs;
  float lonMinFloat = (lonAbs - lonDeg) * 60.0f;
  int lonMin = (int)lonMinFloat;
  int lonSec = (int)((lonMinFloat - lonMin) * 60.0f);
  
  char latDir = (telem.latitude >= 0) ? 'N' : 'S';
  char lonDir = (telem.longitude >= 0) ? 'E' : 'W';
  
  snprintf(telem.gps_coord_str, sizeof(telem.gps_coord_str), 
           "%d %d' %d\" %c %d %d' %d\" %c",
           latDeg, latMin, latSec, latDir,
           lonDeg, lonMin, lonSec, lonDir);
}

void updateGuiVars() {
  // ┌─────────────────────────────────────────────────────┐
  // │ UPDATE GUI VARIABLES                                 │
  // │ Push processed STRING values to the GUI system      │
  // │ These will be displayed in the next RENDER phase    │
  // └─────────────────────────────────────────────────────┘
  
  set_var_bpm_val(telem.bpm_str);
  set_var_turn_angle(telem.turn_angle_str);
  set_var_tilt_str(telem.tilt_str);
  set_var_speed_val(telem.speed_str);
  set_var_gps_coord(telem.gps_coord_str);
  
  // Update boolean states (inverted for HIDDEN flag logic in GUI)
  set_var_is_hidden(!telem.btConnected);  // Hidden when NOT connected
  set_var_is_charging(!telem.charging);   // Hidden when NOT charging
}

/******************** COMMUNICATION ********************/

//═══════════════════════════════════════════════════════════════
// BLUETOOTH COMMUNICATION
// Team Member: _______________
// Protocol: BLE / Classic Bluetooth
//═══════════════════════════════════════════════════════════════
bool commBluetooth() {
#if FEAT_BT
  static uint32_t lastComm = 0;
  
  // TODO: Handle BLE communication every 1 second
  if (runEvery(lastComm, 1000)) {
    
    // ┌─────────────────────────────────────────────────────┐
    // │ ADD YOUR BLUETOOTH CODE HERE                        │
    // │                                                      │
    // │ Transmit (Server mode):                             │
    // │   1. Create BLE server & service                    │
    // │   2. Create characteristics for each telemetry value│
    // │   3. Update characteristic values periodically      │
    // │   4. Notify connected clients                       │
    // │                                                      │
    // │ Receive (Client mode):                              │
    // │   1. Scan for other bike HUD devices                │
    // │   2. Connect to discovered devices                  │
    // │   3. Read their telemetry characteristics           │
    // │                                                      │
    // │ Both:                                                │
    // │   - Handle connection/disconnection events          │
    // │   - Update telem.btConnected status                 │
    // │   - Implement data protocol/format                  │
    // │                                                      │
    // │ Example library:                                     │
    // │   - ESP32 BLE Arduino (built-in)                    │
    // └─────────────────────────────────────────────────────┘
    
    // REMOVE THIS DEFAULT WHEN IMPLEMENTING:
    telem.btConnected = false;  // Replace with actual BLE status
    
    return true;
  }
#endif
  return false;
}

//═══════════════════════════════════════════════════════════════
// HAPTIC FEEDBACK
// Team Member: _______________
// Driver: DRV2605 / Simple motor / Pin: _______________
//═══════════════════════════════════════════════════════════════
void hapticNotify(uint16_t ms) {
#if FEAT_HAPTIC
  
  // ┌─────────────────────────────────────────────────────┐
  // │ ADD YOUR HAPTIC FEEDBACK CODE HERE                  │
  // │                                                      │
  // │ Method 1: Simple vibration motor                    │
  // │   digitalWrite(HAPTIC_PIN, HIGH);                   │
  // │   delay(ms);  // or use non-blocking timer          │
  // │   digitalWrite(HAPTIC_PIN, LOW);                    │
  // │                                                      │
  // │ Method 2: DRV2605 Haptic Driver                     │
  // │   1. Send I2C command to DRV2605                    │
  // │   2. Select waveform/effect                         │
  // │   3. Trigger playback                               │
  // │                                                      │
  // │ Use cases:                                           │
  // │   - Turn warnings (sharp turn ahead)                │
  // │   - Speed alerts (too fast/slow)                    │
  // │   - Navigation cues                                 │
  // │   - Emergency notifications                         │
  // └─────────────────────────────────────────────────────┘
  
  (void)ms;  // Remove this when implementing
#endif
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
    for (int i = 0; i < 4; i++) {
      data[i] = Wire.read();
    }
    
    uint16_t raw_x = ((data[0] & 0x0F) << 8) | data[1];
    uint16_t raw_y = ((data[2] & 0x0F) << 8) | data[3];
    
    // Transform for landscape
    touchX = raw_y;
    touchY = 320 - raw_x;
    
    if (touchX >= SCREEN_WIDTH) touchX = SCREEN_WIDTH - 1;
    if (touchY >= SCREEN_HEIGHT) touchY = SCREEN_HEIGHT - 1;
    
    touched = true;
  } else {
    touched = false;
  }
}

/******************** LVGL CALLBACKS ********************/
void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

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

/******************** DISPLAY INITIALIZATION ********************/
void initDisplay() {
  Serial.println("Initializing display...");
  
  // Backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  
  // TFT init
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.invertDisplay(true);
  Serial.printf("Display: %dx%d\n", tft.width(), tft.height());
  
  // Touch I2C
  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  delay(100);
  
  // Reset touch controller
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, HIGH);
  delay(20);
  digitalWrite(TOUCH_RST, LOW);
  delay(20);
  digitalWrite(TOUCH_RST, HIGH);
  delay(500);
  
  // Check touch controller
  Wire.beginTransmission(FT6336_ADDR);
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    touchEnabled = true;
    Serial.println("Touch controller found!");
  } else {
    Serial.printf("Touch error %d - continuing without touch\n", error);
  }
  
  // LVGL init
  Serial.println("Initializing LVGL...");
  lv_init();
  
  // LVGL display buffer
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, SCREEN_WIDTH * 10);
  
  // Register display driver
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_WIDTH;
  disp_drv.ver_res = SCREEN_HEIGHT;
  disp_drv.flush_cb = disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  
  // Register input driver
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read;
  lv_indev_drv_register(&indev_drv);
  
  Serial.println("Display initialized!");
}

/******************** FSM UPDATE ********************/
void updateFSM() {
  switch(app.phase) {
    case Phase::BOOT:
      // Should not reach here after setup
      break;

    case Phase::READ_SENSORS:
      // ═══════════════════════════════════════════════
      // READ PHASE: Poll all sensors (non-blocking)
      // Each readXXX() function updates telem.XXX values
      // ═══════════════════════════════════════════════
      readCompass();
      readSpeed();
      readImu();
      readHeartRate();
      readGps();
      readCharging();
      readAttendance();
      break;

    case Phase::PROCESS:
      // ═══════════════════════════════════════════════
      // PROCESS PHASE: Convert raw values to strings
      // Format data with units for GUI display
      // ═══════════════════════════════════════════════
      processData();
      updateGuiVars();
      break;

    case Phase::RENDER:
      // ═══════════════════════════════════════════════
      // RENDER PHASE: Update LVGL GUI
      // Display updated values on screen
      // ═══════════════════════════════════════════════
      lv_timer_handler();
      ui_tick();
      break;

    case Phase::COMM:
      // ═══════════════════════════════════════════════
      // COMM PHASE: Handle Bluetooth communication
      // Publish telemetry and receive data from peers
      // ═══════════════════════════════════════════════
      commBluetooth();
      break;

    case Phase::IDLE:
      // ═══════════════════════════════════════════════
      // IDLE PHASE: Brief yield to keep system responsive
      // Prevents watchdog timer resets
      // ═══════════════════════════════════════════════
      delay(1);
      break;
  }
  
  // Advance to next phase
  app.next();
  app.loopCounter++;
}

/******************** ARDUINO SETUP ********************/
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔═════════════════════════════════════════╗");
  Serial.println("║   Bike HUD - FSM + LVGL (SKELETON)     ║");
  Serial.println("╚═════════════════════════════════════════╝\n");
  
  // ┌─────────────────────────────────────────────────────┐
  // │ SENSOR INITIALIZATION                                │
  // │ Add your sensor initialization code here             │
  // │                                                      │
  // │ Examples:                                            │
  // │   - compass.begin();                                │
  // │   - mpu.begin();                                    │
  // │   - GPS_Serial.begin(9600, SERIAL_8N1, RX, TX);     │
  // │   - particleSensor.begin(Wire, I2C_SPEED_FAST);     │
  // │   - BLEDevice::init("BikeHUD");                     │
  // └─────────────────────────────────────────────────────┘
  
  // Initialize telemetry with safe default values
  telem.headingDeg = 0.0f;
  telem.speedMps = 0.0f;
  telem.leanDeg = 0.0f;
  telem.latitude = 0.0;
  telem.longitude = 0.0;
  telem.bpm = 0;
  telem.charging = false;
  telem.btConnected = false;
  telem.attendanceChecked = false;
  
  // Initialize string values
  strcpy(telem.bpm_str, "-- bpm");
  strcpy(telem.turn_angle_str, "--°");
  strcpy(telem.tilt_str, "--°");
  strcpy(telem.speed_str, "-- mph");
  strcpy(telem.gps_coord_str, "-- --' --\" - -- --' --\" -");
  
  // Initialize display and GUI
  initDisplay();
  
  Serial.println("Initializing UI...");
  ui_init();
  
  // Start FSM
  app.begin();
  
  Serial.println("\n╔═════════════════════════════════════════╗");
  Serial.println("║          Setup Complete!                ║");
  Serial.println("║          FSM Running...                 ║");
  Serial.println("╚═════════════════════════════════════════╝\n");
  
  Serial.println("📋 TODO List:");
  Serial.println("  [ ] Implement readCompass()");
  Serial.println("  [ ] Implement readSpeed()");
  Serial.println("  [ ] Implement readImu()");
  Serial.println("  [ ] Implement readGps()");
  Serial.println("  [ ] Implement readHeartRate()");
  Serial.println("  [ ] Implement readCharging()");
  Serial.println("  [ ] Implement readAttendance()");
  Serial.println("  [ ] Implement commBluetooth()");
  Serial.println("  [ ] Implement hapticNotify()");
  Serial.println();
}

/******************** ARDUINO LOOP ********************/
void loop() {
  // ═══════════════════════════════════════════════════════
  // Main loop: FSM cycles through phases continuously
  // Keep this loop CLEAN - all work happens in FSM phases
  // ═══════════════════════════════════════════════════════
  updateFSM();
  
  // Debug output every 5 seconds
  static uint32_t lastDebug = 0;
  if (runEvery(lastDebug, 5000)) {
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.printf("║ Loop #%-8lu                                        ║\n", app.loopCounter);
    Serial.println("╠════════════════════════════════════════════════════════╣");
    Serial.printf("║ BPM:     %-20s                      ║\n", telem.bpm_str);
    Serial.printf("║ Speed:   %-20s                      ║\n", telem.speed_str);
    Serial.printf("║ Heading: %-20s                      ║\n", telem.turn_angle_str);
    Serial.printf("║ Tilt:    %-20s                      ║\n", telem.tilt_str);
    Serial.printf("║ GPS:     %-45s║\n", telem.gps_coord_str);
    Serial.printf("║ BT:      %-20s                      ║\n", telem.btConnected ? "Connected" : "Disconnected");
    Serial.printf("║ Charging:%-20s                      ║\n", telem.charging ? "Yes" : "No");
    Serial.println("╚════════════════════════════════════════════════════════╝\n");
  }
}

