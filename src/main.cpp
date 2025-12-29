#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include "board_config.h"
#include <ImuMPU.h>
#include <Motors.h>
#include <Servo360.h>
#include <TofVL53.h>
#include <Encoders.h>
#include <Detection.h>     
#include <DataLogger.h>
#include <LogConsole.h>
#include <Fsm.h>


#define ENABLE_PERIPHERALS 1
#define ENABLE_STREAM 0
// use data saving only when the stream is turned off, otherwise the robot will struggle with performance
#define ENABLE_SAVING_DATA 1 // means enable coming back with a captured item

// ===========================
// WiFi
// ===========================
const char *ssid = "pawagawa";
const char *password = "essa1234";

// ===========================
// Motors
// ===========================
Motors motors;
const int pwmA = 9, in1A = 8, in2A = 7; 
const int pwmB = 4, in1B = 3, in2B = 2;

const int freq = 20000;
const int res  = 8;

// ===========================
// Servo
// ===========================
Servo360 servo1;
const int SERVO_PIN = 1; 
const int SERVO_STOP_US = 1500;

// ===========================
// I2C - GPIO5=SDA, GPIO6=SCL
// ===========================
const int SDA_PIN = 5;
const int SCL_PIN = 6;

ImuMPU imu;
TofVL53 tof;

// ===========================
// Encoders (slot-type optical interrupter, LM393 comparator)
// ===========================
Encoders enc;
static const uint8_t PIN_ENC_L = 44;  // left wheel
static const uint8_t PIN_ENC_R = 43;  // right wheel

static const bool ENC_USE_PULLUP = false;           // LM393 has its own pull-up
static const unsigned long ENC_REPORT_MS = 50;      //20Hz; 500ms=2Hz
static const float PULSES_PER_REV_L = 35.0f;        // Left shield pulses/rotation (22.0f)
static const float PULSES_PER_REV_R = 33.0f;        // Right shield pulses/rotation (22.0f)
static const unsigned long ENC_MIN_PULSE_US = 1500; // time filter (anti-vibration)

// ===============================================
// Camera 
// ===============================================
void startCameraServer();
void setupLedFlash();

// ===============================================
// Detection
// ===============================================
Detection detection;

// detection parameters
const uint16_t TARGET_WIDTH          = 20;     // big ball => close, need to slow down
const float CALIB_CENTER_X           = 52.0f;  // calibrated center - based on the camera location on my robot
const float MIN_CONFIDENCE           = 0.4f;
const int EI_CLASSIFIER_INPUT_WIDTH  = 96;
const int EI_CLASSIFIER_INPUT_HEIGHT = 96;

// PD control parameters
const float KP_TURN   = 35.0f;    // 25.0f
const float KD_ERR    = 8;        // 6
const float DEAD_BAND = 0.14f;    // 0.14f
const float ALPHA_ERR = 0.6f;     // 0.5f = error filtration (0..1)
const float ALPHA_D   = 0.4f;     // 0.5f = derr filtration  (0..1)

// drive paramaters
const int   DRIVE_SPEED      = 120;
const int   DRIVE_BASE_SPEED = 80;
const int   DRIVE_MAX_SPEED  = 200;

// system state 
const uint32_t  HOLD_MS     = 300;   // how long after the last detection do we still trust the old result: 0.3 s without a ball => STOP
static uint32_t lastSeenMs  = 0;
static bool     haveTarget  = false;
static float    errFilt     = 0.0f;
static float    prevErr     = 0.0f;
static float    derrFilt    = 0.0f;

// ===============================================
// Data Logging (encoders + IMU)
// ===============================================
DataLogger logger;
LogConsole console;

const int BTN_PIN = 0;   // BOOT/B button to manually save data

// ===============================================
// Finite State Machine
// ===============================================
Fsm fsm;

// ===============================================
void setup() {
  
  // --- Serial ---
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  #if ENABLE_SAVING_DATA
  // --- Data Logging ---
  DataLogger::Config lcfg;
  lcfg.periodMs = 50;           // LOG_PERIOD_MS    logging every 50ms = 20Hz
  lcfg.maxSamples = 1000;       // LOG_SAMPLES_MAX  ~40 kB RAM
  bool ok = logger.begin(lcfg);
  Serial.println(ok ? "LittleFS ready" : "LittleFS mount failed");

  LogConsole::Config ccfg;
  ccfg.btnPin = BTN_PIN;
  console.begin(ccfg, logger, Serial);
  #endif

  #if ENABLE_PERIPHERALS
  // --- Servo, do not overwrite timers before motors configuration ---
  //servo1.begin(SERVO_PIN, SERVO_STOP_US);   // stop at the beggining (1500us)
  //servo1.setSpeed(0.6f);                    // constant forward rotation
  delay(100);
  
  // --- Motors + PWM on particular channels ---
  Motors::Pins mpins;
  mpins.pwmA = pwmA; mpins.in1A = in1A; mpins.in2A = in2A;
  mpins.pwmB = pwmB; mpins.in1B = in1B; mpins.in2B = in2B;
  mpins.chA = 4;     mpins.chB = 5;
  motors.begin(mpins, freq, res);
  motors.setScale(1.0f, 0.93f);  //1.0f, 0.93f
  delay(100);

  // --- I2C sensors initialization on GPIO5/6 (Camera has its own SCCB on 39/40) ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(50000);        // 100000UL = 100 kHz
  delay(100);       

  // --- VL53L0K (I2C) ---
  Serial.println("Calling tof.begin()...");
  tof.begin(Wire);   
  delay(100);

  // --- MPU6050 (I2C) ---
  Serial.println("Calling imu.begin()...");
  imu.begin(Wire); 
  imu.setPeriodMs(50);
  delay(100);

  // --- Encoders ---
  Encoders::Config ecfg;
  ecfg.usePullup = false;
  ecfg.pinL = PIN_ENC_L; ecfg.pulsesPerRevL = PULSES_PER_REV_L;
  ecfg.pinR = PIN_ENC_R; ecfg.pulsesPerRevR = PULSES_PER_REV_R;
  ecfg.minPulseUs = ENC_MIN_PULSE_US;
  ecfg.reportMs = ENC_REPORT_MS;
  enc.begin(ecfg);
  delay(100);
  #endif
  
  // --- Camera ---
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;      
  config.pin_sccb_scl = SIOC_GPIO_NUM;      
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;

  
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_240X240;

  config.grab_mode = CAMERA_GRAB_LATEST;   // CAMERA_GRAB_WHEN_EMPTY - lighter version
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;                // stream setting
  config.fb_count = 1;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically
  if (s->id.PID == OV3660_PID) {

    // WATCH OUT FOR THIS!!
    // model's training data must be the same with adjustments!!!
    
    s->set_vflip(s, 1);        // flip it back        
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, 0);   // basic saturation
  }

  // --- Detection ---
  Detection::Config dcfg;
  dcfg.debug_nn = false;
  dcfg.log = false;
  dcfg.periodMs = 150;
  dcfg.confidenceThreshold = 0.4f;
  detection.begin(dcfg);

  // --- FSM ---
  Fsm::Config fcfg;
  fcfg.minConfidence   = MIN_CONFIDENCE;
  fcfg.holdMs          = HOLD_MS;

  fcfg.nnWidth         = EI_CLASSIFIER_INPUT_WIDTH;
  fcfg.calibCenterX    = CALIB_CENTER_X;
  fcfg.targetWidth     = TARGET_WIDTH;      // not used

  fcfg.kpTurn          = KP_TURN;
  fcfg.kdErr           = KD_ERR;
  fcfg.deadBand        = DEAD_BAND;
  fcfg.alphaErr        = ALPHA_ERR;
  fcfg.alphaD          = ALPHA_D;

  fcfg.driveSpeed      = DRIVE_SPEED;
  fcfg.driveBaseSpeed  = DRIVE_BASE_SPEED;
  fcfg.driveMaxSpeed   = DRIVE_MAX_SPEED;  // not used

  fcfg.printTransitions = true;
  fcfg.dbgPeriodMs      = 200;

  fsm.begin(motors, detection, logger, fcfg);
  fsm.setState(Fsm::State::IDLE);         // start from IDLE


  // --- WiFi + camera server ---
  #if ENABLE_STREAM
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  // http://172.20.10.2:80/stream
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  #endif

}


void loop() {
  const uint32_t now = millis();

  // --- peripherals ---
  tof.update();
  imu.update();
  enc.update();
  fsm.setMeasuredWheelV(enc.vL(), enc.vR());

  detection.update();  
  fsm.update(now);         

  #if ENABLE_SAVING_DATA
    logger.tick(now,
                imu.ax_g(), imu.ay_g(), imu.az_g(),
                imu.gz_dps(),
                enc.pulsesL(), enc.pulsesR(),
                enc.vL(), enc.vR());
    console.update();
  #endif

  delay(10);
}