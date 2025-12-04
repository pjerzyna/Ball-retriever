#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include "img_converters.h" //?
#include "board_config.h"
#include <ImuMPU.h>
#include <Motors.h>
#include <Servo360.h>
#include <TofVL53.h>
#include <Encoders.h>
//#include <VisionBall.h>
#include "Detection.h" 


#define ENABLE_PERIPHERALS 0
#define ENABLE_STREAM 0

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
static const float PULSES_PER_REV_L = 20.0f;        // Left shield pulses/rotation
static const float PULSES_PER_REV_R = 20.0f;        // Right shield pulses/rotation
static const unsigned long ENC_MIN_PULSE_US = 200;  // time filter (anti-vibration)

// ===============================================
// Camera 
// ===============================================
void startCameraServer();
void setupLedFlash();

// ===============================================
// Detection
// ===============================================
Detection detection;



// ===============================================
void setup() {
  
  // --- Serial ---
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  #if ENABLE_PERIPHERALS
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
  imu.begin(Wire); 
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

  config.grab_mode = CAMERA_GRAB_LATEST; //CAMERA_GRAB_WHEN_EMPTY - to jest lżejsze
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  // fb_count = 2 nie daje faktycznej korzyści przy FOMO ~150 ms
  config.fb_count = 1;   //2 - po jakimś czasie: Stack canary (cam_task)

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {

    // WATCH OUT FOR THIS!!
    // model's training data must be with same adjustments!!!
    
    s->set_vflip(s, 1);        // flip it back        
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, 0);   // basic saturation  // bylo wczesniej na -2
  }

  // --- Detection ---
  Detection::Config dcfg;
  dcfg.debug_nn = false;
  detection.begin(dcfg);

  /*
  Detection::Config dcfg;
  dcfg.debug_nn            = false;
  dcfg.periodMs            = 200;   // detekcja co ~200 ms
  dcfg.confidenceThreshold = 0.6f;  // wymagaj trochę wyższej pewności

  if (!detection.begin(dcfg)) {
      Serial.println("Detection init failed");
  }

  */


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


// ===============================================
void loop() {
  uint32_t now = millis();

  // --- IMU ---
  //imu.update();

  // --- TOF ---
  //tof.update();

  // --- Encoders ---
  //enc.update();

  // --- FOMO --- 
  detection.update();
 
  /*
  if (detection.hasResult()) {
        auto r = detection.lastResult();
        if (r.valid) {
            // np. sterowanie robotem po bboxie
            // r.x, r.y, r.width, r.height, r.score
        }
    }
  */

  delay(1);
}