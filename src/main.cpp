#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include "img_converters.h"



#include "board_config.h"
#include <ImuMPU.h>
#include <Motors.h>
#include <Servo360.h>
#include <TofVL53.h>
#include <Encoders.h>
#include <VisionBall.h>

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
static const unsigned long ENC_REPORT_MS = 500;     // 2Hz
static const float PULSES_PER_REV_L = 20.0f;        // Left shield pulses/rotation !(TO ADJUST)!
static const float PULSES_PER_REV_R = 20.0f;        // Right shield pulses/rotation !(TO ADJUST)!
static const unsigned long ENC_MIN_PULSE_US = 200;  // time filter (anti-vibration)

// ===============================================
// Camera 
// ===============================================
void startCameraServer();
void setupLedFlash();

// ===============================================
// Detection
// ===============================================
VisionBall vision;

static inline int clampi(int v, int lo, int hi){
  return v<lo?lo:(v>hi?hi:v);
}

// throttle, turn: -1..1
void driveDifferential(float throttle, float turn){
  if (throttle > 1) throttle = 1;
  if (throttle < -1) throttle = -1;
  if (turn > 1) turn = 1;
  if (turn < -1) turn = -1;

  float l = throttle - turn;
  float r = throttle + turn;

  float maxAbs = max(fabs(l), fabs(r));
  if (maxAbs > 1.0f) { l /= maxAbs; r /= maxAbs; }

  int pwmL = (int)(l * 255.0f);
  int pwmR = (int)(r * 255.0f);

  motors.setA(clampi(pwmL, -255, 255));
  motors.setB(clampi(pwmR, -255, 255));
}

void visionTask(void*){
  // parametry sterowania
  const float KP_TURN   = 0.70f;  // skręt od centroidu
  const float AREA_STOP = 0.15f;  // jak plama > 15% -> jesteśmy blisko
  const float SEARCH_TURN = 0.6f; // szukanie - obrót w miejscu

  for(;;){
    vision.update();
    if (vision.hasNew()){
      float throttle = 0.0f;
      float turn     = 0.0f;

      if (!vision.seen()){
        // nie widzę piłki -> obracaj się, żeby znaleźć
        throttle = 0.0f;
        turn = SEARCH_TURN;
      } else {
        float area = vision.areaRatio();
        float cx   = vision.cxNorm();

        // im bliżej (większa plama), tym wolniej
        float slow = area / AREA_STOP;
        if (slow > 1.0f) slow = 1.0f;
        throttle = 1.0f - slow;     // 1 -> 0

        // skręt proporcjonalny do odchyłki centroidu
        turn = KP_TURN * cx;

        if (area >= AREA_STOP){
          throttle = 0.0f;
          turn = 0.0f;
        }
      }

      driveDifferential(throttle, turn);
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // mały oddech RTOS
  }
}


// ===============================================
void setup() {
  
  // --- Serial ---
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // --- Servo, do not overwrite timers before motors configuration ---
  servo1.begin(SERVO_PIN, SERVO_STOP_US);   // stop at the beggining (1500us)
  servo1.setSpeed(0.6f);                    // constant forward rotation
  delay(100);
  
  // --- Motors + PWM on particular channels
  Motors::Pins mpins;
  mpins.pwmA = pwmA; mpins.in1A = in1A; mpins.in2A = in2A;
  mpins.pwmB = pwmB; mpins.in1B = in1B; mpins.in2B = in2B;
  mpins.chA = 4;     mpins.chB = 5;
  motors.begin(mpins, freq, res);
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
  imu.begin(Wire); //bool ok = 
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
  config.frame_size = FRAMESIZE_UXGA;

  config.pixel_format = PIXFORMAT_JPEG;     // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition

  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif  

  // --- WiFi + camera server ---
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

  // --- Camera ---
  VisionBall::Params vp;
  vp.sampleStride = 6;
  vp.periodMs = 100;

  vp.hsv.hMin = 36; //22
  vp.hsv.hMax = 75; //60
  vp.hsv.sMin = 55; //18
  vp.hsv.vMin = 55; //24
  vision.begin(vp); 


  xTaskCreatePinnedToCore(visionTask, "vision", 8192, nullptr, 1, nullptr, 1);
}


// ===============================================
void loop() {
  uint32_t now = millis();


  // --- IMU ---
  imu.update();
  if (imu.hasNew()) {}

  // --- TOF ---
  tof.update();
  if (tof.hasNew()) {}

  // --- Encoders ---
  enc.update();
  if (enc.hasNew()) {}

  delay(1);
}
