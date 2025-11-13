/*
  Arduino Bike HUD / Wearable Telemetry — SINGLE‑SCREEN SKELETON
  Non‑blocking phase FSM (READ → PROCESS → RENDER → COMM → IDLE)
  EEZ Studio + LVGL ready. Minimal content, lots of structure + empty functions.

  Features scaffolded
    • Compass (heading)
    • Speed via Reed Switch
    • Lean angle via IMU
    • LCD (LVGL + TFT_eSPI + FT6336)
    • Bluetooth (BLE skeleton)
    • GPS / Navigation (lat, lon)
    • Heart‑rate monitor
    • Haptic driver (stub)
    • Charging indicator
    • Attendance checking (stub)
*/

/******************** INCLUDES ********************/
#include <Arduino.h>
#include <Wire.h>

// --- Display stack (aligns with your existing main + EEZ Studio/LVGL) ---
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <FT6336_I2C.h>   // capacitive touch controller

// --- Optional comms (comment in when wiring up) ---
// #include <BLEDevice.h>     // or BluetoothSerial on ESP32 Classic
// #include <TinyGPSPlus.h>   // if you use TinyGPS++ for NMEA parsing

/******************** FEATURE TOGGLES ********************/
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

/******************** PIN MAP (EDIT ME) ********************/
// Sensors / I/O (example pins; change to match your board)
static const uint8_t PIN_REED        = 2;   // speed sensor
static const uint8_t PIN_HR          = 3;   // heart-rate signal (digital or analog)
static const uint8_t PIN_HAPTIC      = 5;   // DRV pin / MOSFET to vibration motor
static const uint8_t PIN_CHARGE_DET  = 6;   // charger IC STAT or fuel gauge IRQ

// Display & touch (align to your board’s wiring)
#define TFT_BL 38
#define TOUCH_INT 13
#define TOUCH_RST 16
#define TOUCH_SDA 8
#define TOUCH_SCL 9
#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 320

/******************** UTIL: TIME-SLICED SCHEDULER ********************/
static inline bool runEvery(uint32_t &t, const uint32_t dt){
  uint32_t now = millis();
  if(now - t >= dt){ t = now; return true; }
  return false;
}

/******************** SHARED DATA MODEL ********************/
struct Telemetry {
  // core
  float  headingDeg   = NAN;   // compass
  float  speedMps     = NAN;   // reed
  float  leanDeg      = NAN;   // IMU roll/pitch
  double latitude     = NAN;   // GPS
  double longitude    = NAN;   // GPS
  uint16_t bpm        = 0;     // heart rate
  bool charging       = false; // charging indicator
  bool btConnected    = false; // bluetooth link
  // attendance
  bool attendanceChecked = false;
} telem;

/******************** SENSOR READ API (stubs) ********************/
// Each function should be non‑blocking and update `telem`. Return true if new data applied.

bool readCompass(){
#if FEAT_COMPASS
  // TODO: read magnetometer via I2C and compute heading (deg)
  // telem.headingDeg = ...;
#endif
  return false;
}

bool readSpeed(){
#if FEAT_SPEED
  // TODO: compute speed from reed pulses
  // telem.speedMps = ...;
#endif
  return false;
}

bool readImu(){
#if FEAT_IMU
  // TODO: read IMU accel/gyro and compute lean angle (deg)
  // telem.leanDeg = ...;
#endif
  return false;
}

bool readGps(){
#if FEAT_GPS
  // TODO: parse NMEA and update lat/lon
  // telem.latitude  = ...;
  // telem.longitude = ...;
#endif
  return false;
}

bool readHeartRate(){
#if FEAT_HR
  // TODO: read HR sensor (e.g., MAX3010x) and compute BPM
  // telem.bpm = ...;
#endif
  return false;
}

bool readCharging(){
#if FEAT_CHARGE
  // Simple digital charge detect (replace with fuel‑gauge read when available)
  telem.charging = digitalRead(PIN_CHARGE_DET);
  return true;
#else
  return false;
#endif
}

bool readAttendance(){
#if FEAT_ATTEND
  // TODO: mark present if BT device seen OR within GPS geofence
  // telem.attendanceChecked = ...;
#endif
  return false;
}

/******************** COMM / ACTUATION STUBS ********************/
bool commBluetooth(){
#if FEAT_BT
  // TODO: publish telemetry / process RX commands
  // telem.btConnected = ...;
#endif
  return false;
}

void hapticNotify(uint16_t ms){
#if FEAT_HAPTIC
  // TODO: enqueue non‑blocking vibration for ms
  (void)ms;
#endif
}

/******************** APP FSM (one-screen pipeline) ********************/
// Single HUD screen. FSM phases to cooperatively time-slice work.
enum class Phase : uint8_t {
  BOOT,
  READ_SENSORS,   // poll IMU, compass, reed, HR, GPS (non‑blocking)
  PROCESS,        // filter/fuse, compute derived metrics
  RENDER,         // update LVGL HUD
  COMM,           // BT publish / host comms
  IDLE            // short idle/yield
};

struct AppFSM {
  Phase phase = Phase::BOOT;
  void begin(){ phase = Phase::READ_SENSORS; }
  void next(){
    switch(phase){
      case Phase::BOOT:        phase = Phase::READ_SENSORS; break;
      case Phase::READ_SENSORS:phase = Phase::PROCESS;      break;
      case Phase::PROCESS:     phase = Phase::RENDER;       break;
      case Phase::RENDER:      phase = Phase::COMM;         break;
      case Phase::COMM:        phase = Phase::IDLE;         break;
      case Phase::IDLE:        phase = Phase::READ_SENSORS; break;
    }
  }
  void update(){ /* loop‑driven via updateManagers() */ }
} app;

/******************** DISPLAY (LVGL + EEZ Studio hook) ********************/
#if FEAT_LCD
// LVGL draw buffers & display instance
TFT_eSPI tft = TFT_eSPI();
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCREEN_WIDTH * SCREEN_HEIGHT / 8];
FT6336_I2C touch(TOUCH_INT, TOUCH_RST, SCREEN_WIDTH, SCREEN_HEIGHT);

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p){
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t*)color_p, w*h, true);
  tft.endWrite();
  lv_disp_flush_ready(disp_drv);
}

static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data){
  touch.read();
  if (touch.isTouched && touch.touches > 0){
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = SCREEN_WIDTH  - 1 - touch.points[0].x;
    data->point.y = SCREEN_HEIGHT - 1 - touch.points[0].y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

class DisplayManager {
  // HUD widget handles (bind these to EEZ/LVGL labels in bindWidgets())
  lv_obj_t* lblSpeed   = nullptr;
  lv_obj_t* lblHR      = nullptr;
  lv_obj_t* lblHeading = nullptr;
  lv_obj_t* lblLean    = nullptr;
  lv_obj_t* lblGPS     = nullptr;
  lv_obj_t* lblCharge  = nullptr;
  lv_obj_t* lblAttend  = nullptr;
public:
  void begin(){
    pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
    tft.init();
    tft.setRotation(1); // landscape
    tft.fillScreen(TFT_BLACK);

    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    delay(50);
    touch.begin();
    touch.setRotation(1);

    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, sizeof(buf1)/sizeof(lv_color_t));

    static lv_disp_drv_t disp_drv; lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH; disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = disp_flush; disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv; lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER; indev_drv.read_cb = touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // EEZ Studio generated UI bootstrap
    // ui_init();
    bindWidgets();
  }

  void bindWidgets(){
    // TODO: map EEZ/LVGL objects to pointers above, e.g.:
    // lblSpeed = lv_obj_get_child_by_id(lv_scr_act(), SPEED_LABEL_ID);
    // Or create simple labels programmatically if not using exported UI yet.
  }

  // --- One call to refresh the whole HUD from `telem` ---
  void drawHud(){
    displaySpeed();
    displayHeartRate();
    displayHeading();
    displayLean();
    displayGPS();
    displayCharging();
    displayAttendance();
  }

  // --- Per-field display helpers (safe if labels are null) ---
  void displaySpeed(){ if(!lblSpeed) return; /* lv_label_set_text_fmt(lblSpeed, "%.1f m/s", telem.speedMps); */ }
  void displayHeartRate(){ if(!lblHR) return; /* lv_label_set_text_fmt(lblHR, "%u bpm", telem.bpm); */ }
  void displayHeading(){ if(!lblHeading) return; /* lv_label_set_text_fmt(lblHeading, "%.0f°", telem.headingDeg); */ }
  void displayLean(){ if(!lblLean) return; /* lv_label_set_text_fmt(lblLean, "%.1f°", telem.leanDeg); */ }
  void displayGPS(){ if(!lblGPS) return; /* lv_label_set_text_fmt(lblGPS, "%.6f, %.6f", telem.latitude, telem.longitude); */ }
  void displayCharging(){ if(!lblCharge) return; /* lv_label_set_text(lblCharge, telem.charging ? "Charging" : "On battery"); */ }
  void displayAttendance(){ if(!lblAttend) return; /* lv_label_set_text(lblAttend, telem.attendanceChecked ? "Present" : "—"); */ }

  void update(){
    static uint32_t lastLv = 0;
    if (runEvery(lastLv, 5)) {
      lv_timer_handler();
      // ui_tick(); // if your EEZ export needs it
    }
  }
} displayMgr;
#endif

/******************** APP LIFECYCLE ********************/
void beginManagers(){
  app.begin();
#if FEAT_LCD
  displayMgr.begin();
#endif
#if FEAT_CHARGE
  pinMode(PIN_CHARGE_DET, INPUT);
#endif
#if FEAT_SPEED
  pinMode(PIN_REED, INPUT_PULLUP);
#endif
#if FEAT_HAPTIC
  pinMode(PIN_HAPTIC, OUTPUT);
  digitalWrite(PIN_HAPTIC, LOW);
#endif
}

void updateManagers(){
  // Phase-ordered cooperative pipeline
  switch(app.phase){
    case Phase::BOOT: break;

    case Phase::READ_SENSORS:
      (void)readCompass();
      (void)readSpeed();
      (void)readImu();
      (void)readHeartRate();
      (void)readGps();
      (void)readCharging();
      (void)readAttendance();
      break;

    case Phase::PROCESS:
      // TODO: place lightweight filters/fusion/units conversions here
      break;

    case Phase::RENDER:
#if FEAT_LCD
      displayMgr.drawHud();
      displayMgr.update();
#endif
      break;

    case Phase::COMM:
      (void)commBluetooth();
      break;

    case Phase::IDLE:
      // very short idle to keep UI responsive
      delay(1);
      break;
  }
  app.next();
}

/******************** ARDUINO ENTRY ********************/
void setup(){
  Serial.begin(115200);
  delay(50);
  Serial.println("
BikeHUD Skeleton starting...");
  beginManagers();
  Serial.println("Managers started.");
}

void loop(){
  updateManagers();
  // small idle hint; keep loop snappy
  delay(1);
}

/******************** EXTENSION NOTES ********************/
/*
  1) EEZ Studio / LVGL
     - Export your GUI as ui.h/ui.c and include in this file.
     - Call ui_init() in DisplayManager::begin().
     - Bind HUD widgets in bindWidgets() and update them in the display*() helpers.

  2) Sensors
     - Teammates only implement their sensor’s readX() and displayX() bodies.
     - Maintain non‑blocking patterns (no delay()).

  3) FSM
     - Single screen: the FSM only orchestrates phases for cooperative "parallel" work.

  4) Concurrency
     - Each phase should stay short; use runEvery() inside readX() if needed.
*/
