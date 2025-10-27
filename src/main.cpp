#define sensor_t adafruit_sensor_t   // redirect typedef from Adafruit (same name as in esp32_camera)
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#undef sensor_t                      // camera name returned, collision problem solved
#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>




// ===========================
// Camera model from board_config.h
// ===========================
#include "board_config.h"

// ===========================
// WiFi
// ===========================
const char *ssid = "pawagawa";
const char *password = "essa1234";

// ===========================
// Motors
// ===========================
const int pwmA = 9, in1A = 8, in2A = 7;
const int pwmB = 4, in1B = 3, in2B = 2;

const int freq = 20000;          // 20kHz
const int res = 8;               // 8-bit resolution (0..255)


// CONSTANT SPEED VALUE for prototyping
const uint8_t MotorSpeed1 = 128;
const uint8_t MotorSpeed2 = 128;
 
// ===========================
// Servo pinout
// ===========================
const int SERVO_PIN = 1; 
const int STOP_US = 1500;
const int SPEED_US = 200;  
Servo servo1;

// ===========================
// I2C - GPIO5=SDA, GPIO6=SCL
// ===========================
const int SDA_PIN = 5;
const int SCL_PIN = 6;

// ===============================================
// Distance sensor - VL53L0K 
// ===============================================
const int VL53L0K_addr = 0x29;
Adafruit_VL53L0X lox;
static const uint32_t VL53LL0K_PERIOD_MS = 100;  //10Hz

// ===============================================
// IMU - MPU6050 
// ===============================================
const int MPU_addr = 0x68; // AD0=LOW
Adafruit_MPU6050 mpu;
static const uint32_t MPU_PERIOD_MS = 100;  // 10Hz
static const float RAD2DEG = 57.2957795f;   // 180/pi
static const float ONE_G = 9.80665f;
bool mpu_ok = false;
float acc_bias_g[3]  = {0,0,0};
float gyro_bias_dps[3] = {0,0,0};



// ===============================================
// Encoder sensors (slot-type optical interrupter, LM393 comparator)
// ===============================================
// Each sensor outputs a digital pulse when the slot is interrupted.
// GPIO43/44 are TX/RX UART0, but with USB-CDC usage, they are free.
static const uint8_t PIN_ENC_L = 44;  // left wheel
static const uint8_t PIN_ENC_R = 43;  // right wheel

static const bool ENC_USE_PULLUP = false;           // LM393 has its own pull-up
static const unsigned long ENC_REPORT_MS = 500;     // 2Hz
static const float PULSES_PER_REV_L = 20.0f;        // Left shield pulses/rotation !(TO ADJUST)!
static const float PULSES_PER_REV_R = 20.0f;        // Right shield pulses/rotation !(TO ADJUST)!
static const unsigned long ENC_MIN_PULSE_US = 200;  // time filter (anti-vibration)

// counters shared with ISR – active falling edge (LM393 = active LOW)
volatile uint32_t encL_count = 0, encR_count = 0;
volatile uint32_t encL_last_us = 0, encR_last_us = 0;

// Left Interrupt Service Routine
void IRAM_ATTR encLISR() {
  uint32_t now = micros();
  if (now - encL_last_us >= ENC_MIN_PULSE_US) {
    encL_count++;
    encL_last_us = now;
  }
}

// Right Interrupt Service Routine
void IRAM_ATTR encRISR() {
  uint32_t now = micros();
  if (now - encR_last_us >= ENC_MIN_PULSE_US) {
    encR_count++;
    encR_last_us = now;
  }
}

// ===============================================
// Camera 
// ===============================================
void startCameraServer();
void setupLedFlash();

void calibrateIMU(unsigned samples = 500, unsigned settle_ms = 500) {
  // some time to calm down robot
  delay(settle_ms);

  double sum_ax=0, sum_ay=0, sum_az=0;
  double sum_gx=0, sum_gy=0, sum_gz=0;

  for (unsigned i=0; i<samples; ++i) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    // a.acceleration in m/s^2 --> na g
    double ax_g = a.acceleration.x / ONE_G;
    double ay_g = a.acceleration.y / ONE_G;
    double az_g = a.acceleration.z / ONE_G;

    // g.gyro in rad/s --> na deg/s
    double gx_dps = g.gyro.x * RAD2DEG;
    double gy_dps = g.gyro.y * RAD2DEG;
    double gz_dps = g.gyro.z * RAD2DEG;

    sum_ax += ax_g; sum_ay += ay_g; sum_az += az_g;
    sum_gx += gx_dps; sum_gy += gy_dps; sum_gz += gz_dps;

    delay(2); // 500*2ms = 1s of sampling
  }

  // Means
  double mean_ax = sum_ax / samples;
  double mean_ay = sum_ay / samples;
  double mean_az = sum_az / samples;

  double mean_gx = sum_gx / samples;
  double mean_gy = sum_gy / samples;
  double mean_gz = sum_gz / samples;

  // Biases:
  // - Accelerometer: (0,0,1) is desired
  acc_bias_g[0] = (float)mean_ax;         
  acc_bias_g[1] = (float)mean_ay;         
  acc_bias_g[2] = (float)(mean_az - 1.0); 

  // - Gyroscope: (0,0,0) deg/s is desired
  gyro_bias_dps[0] = (float)mean_gx;
  gyro_bias_dps[1] = (float)mean_gy;
  gyro_bias_dps[2] = (float)mean_gz;

  Serial.println(F("IMU calibrated."));
  Serial.print(F("acc_bias_g = "));  Serial.print(acc_bias_g[0],3); Serial.print(" ");
  Serial.print(acc_bias_g[1],3); Serial.print(" "); Serial.println(acc_bias_g[2],3);
  Serial.print(F("gyro_bias_dps = ")); Serial.print(gyro_bias_dps[0],2); Serial.print(" ");
  Serial.print(gyro_bias_dps[1],2); Serial.print(" "); Serial.println(gyro_bias_dps[2],2);
}

void setup() {

  // --- Motors + PWM on particular channels
  pinMode(in1A, OUTPUT), pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT), pinMode(in2B, OUTPUT);

  ledcSetup(4, freq, res); // Channel 4 for Motor A
  ledcSetup(5, freq, res); // Channel 5 for Motor B
  ledcAttachPin(pwmA, 4);  // Attach Motor A PWM pin to channel 4
  ledcAttachPin(pwmB, 5);  // Attach Motor B PWM pin to channel 5

  // Motor A forward
  digitalWrite(in1A, LOW);  
  digitalWrite(in2A, HIGH);
 
 // Motor B forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  
  ledcWrite(4, MotorSpeed1); // Write speed to channel 4 (Motor A)
  ledcWrite(5, MotorSpeed2); // Write speed to channel 5 (Motor B)

  // --- Serial ---
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // --- Servo, do not overwrite timers before motors configuration ---
  servo1.attach(SERVO_PIN);
  servo1.writeMicroseconds(STOP_US + SPEED_US);

  // --- I2C sensors initialization on GPIO5/6 (Camera has its own SCCB on 39/40) ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000UL);   // 100 kHz for stability
  delay(100);


  // --- VL53L0X (I2C) ---
  if (!lox.begin(VL53L0K_addr, false, &Wire)) {
    Serial.println(F("ERROR: VL53L0X doesn't response at 0x29 address."));
  } else {
    Serial.println(F("OK: VL53L0X is ready (mm)."));
  }


  // --- MPU6050 (I2C) ---
  mpu_ok = mpu.begin(0x68, &Wire);
  if (!mpu_ok) {
    mpu_ok = mpu.begin(0x69, &Wire);
  }
  if (!mpu_ok) {
    Serial.println(F("ERROR: MPU6050 doesn't response (0x68/0x69)."));
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    calibrateIMU(600, 600); // 500 samples, 500 ms are default values
    Serial.println(F("OK: MPU6050 is ready (accelerometer + gyroscope)."));
  }

  // --- slot-type detector, input and interrupt configuration ---
  //  when is active = LOW --> count falling edge
  pinMode(PIN_ENC_L, ENC_USE_PULLUP ? INPUT_PULLUP : INPUT);
  pinMode(PIN_ENC_R, ENC_USE_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), encLISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), encRISR, FALLING);
  
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
}



void loop() {


  // --- VL53L0KL: ~100ms ---
  static uint32_t t0 = 0;
  uint32_t now = millis();
  if (now - t0 >= VL53LL0K_PERIOD_MS) {   // 100ms = 10Hz  
    t0 = now;

    VL53L0X_RangingMeasurementData_t m;
    lox.rangingTest(&m, false); // false = without library debugs

    if (m.RangeStatus != 4 && m.RangeMilliMeter > 0) {
     Serial.print(F("[VL53] "));
     Serial.print(m.RangeMilliMeter);
      Serial.println(F(" mm"));
    } else {
      Serial.println(F("[VL53] poza zakresem / błąd"));
    }
  }


  // --- MPU6050: ~100ms ---
  static uint32_t t_mpu = 0;
  if (mpu_ok && (now - t_mpu >= MPU_PERIOD_MS)) {
    t_mpu = now;

    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float ax_g = a.acceleration.x / ONE_G - acc_bias_g[0];
    float ay_g = a.acceleration.y / ONE_G - acc_bias_g[1];
    float az_g = a.acceleration.z / ONE_G - acc_bias_g[2];

    float gx_dps = g.gyro.x * RAD2DEG - gyro_bias_dps[0];
    float gy_dps = g.gyro.y * RAD2DEG - gyro_bias_dps[1];
    float gz_dps = g.gyro.z * RAD2DEG - gyro_bias_dps[2];


    Serial.print(F("[MPU] Acc[g]: "));
    Serial.print(ax_g, 3); Serial.print(' ');
    Serial.print(ay_g, 3); Serial.print(' ');
    Serial.print(az_g, 3);

    // Angular speed
    Serial.print(F("  |  Gyr[deg/s]: "));  
    Serial.print(gx_dps, 1); Serial.print(' ');
    Serial.print(gy_dps, 1); Serial.print(' ');
    Serial.print(gz_dps, 1);
    Serial.println();
  }


  // --- Enkodery: ~500ms ---
  static uint32_t enc_t0 = millis();
  uint32_t now_ms = millis();
  if (now_ms - enc_t0 >= ENC_REPORT_MS) {
    float dt = (now_ms - enc_t0) / 1000.0f;
    enc_t0 = now_ms;

    // Atomic read and reset of encoder counters
    // Uninterruptible process prevents ISR interference
    noInterrupts();
    uint32_t pL = encL_count; encL_count = 0;
    uint32_t pR = encR_count; encR_count = 0;
    interrupts();

    float hzL  = (dt > 0.0f) ? (pL / dt) : 0.0f;
    float hzR  = (dt > 0.0f) ? (pR / dt) : 0.0f;
    float rpmL = (PULSES_PER_REV_L > 0.0f) ? (hzL * 60.0f / PULSES_PER_REV_L) : 0.0f;
    float rpmR = (PULSES_PER_REV_R > 0.0f) ? (hzR * 60.0f / PULSES_PER_REV_R) : 0.0f;

    Serial.print(F("[ENC] L: imp=")); Serial.print(pL);
    Serial.print(F(" Hz="));         Serial.print(hzL, 2);
    Serial.print(F(" RPM="));        Serial.print(rpmL, 1);
    Serial.print(F(" | R: imp="));   Serial.print(pR);
    Serial.print(F(" Hz="));         Serial.print(hzR, 2);
    Serial.print(F(" RPM="));        Serial.println(rpmR, 1);
  }

  // --- Looped servo movement is executed in setup ---


  // --- Web server is executed in setup ---


}