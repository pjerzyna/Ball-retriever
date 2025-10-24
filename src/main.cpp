// Distance sensor and IMU
#define sensor_t adafruit_sensor_t   // przekieruj typedef z Adafruit (taka sama nazwa jak w bibliotece esp32_camera_)
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#undef sensor_t                      // przywróć nazwę dla kamery, bo problem kolizyjny zostal zazegnany

#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>




// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "pawagawa";
const char *password = "essa1234";

// Motor A & Motor B (watch out with RX/TX pins! - do not override them)
const int pwmA = 9, in1A = 8, in2A = 7;
const int pwmB = 4, in1B = 3, in2B = 2;

const int freq = 20000;          // 20kHz
const int res = 8;               // 8-bit resolution (0..255)

// Motor Speed Values - const
const uint8_t MotorSpeed1 = 128;
const uint8_t MotorSpeed2 = 128;
 
// Servo pinout
const int SERVO_PIN = 1; 
const int STOP_US = 1500;
const int SPEED_US = 200;  
Servo servo1;

 // I2C sensors pinout
const int SDA_PIN = 5;
const int SCL_PIN = 6;

// ===============================================
// Distance sensor VL53 - global obejct and params
// ===============================================
const int VL53L0K_addr = 0x29;
Adafruit_VL53L0X lox;
static const uint32_t VL53LL0K_PERIOD_MS = 100;  //10Hz

// ===============================================
// IMU - MPU6050 
// ===============================================
const int MPU_addr = 0x68; // AD0=LOW
Adafruit_MPU6050 mpu;
static const uint32_t MPU_PERIOD_MS = 100;  //10Hz
static const float RAD2DEG = 57.2957795f; 
static const float ONE_G = 9.80665f;
bool mpu_ok = false;

// Camera declaration
void startCameraServer();
void setupLedFlash();

void setup() {
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

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Servo initialization - do not overwrite timers before motors configuration
  servo1.attach(SERVO_PIN);
  servo1.writeMicroseconds(STOP_US + SPEED_US);

  // I2C sensors initialization (GPIO5/6, camera has own SCCB on 39/40)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000UL);   // 100 kHz for stability
  delay(100);


  // --- VL53L0X ---
  if (!lox.begin(VL53L0K_addr, false, &Wire)) {
    Serial.println(F("BLAD: VL53L0X doesn't response at 0x29 address."));
  } else {
    Serial.println(F("OK: VL53L0X is ready (mm)."));
  }


  // --- MPU6050 ---
  mpu_ok = mpu.begin(0x68, &Wire);
  if (!mpu_ok) {
    mpu_ok = mpu.begin(0x69, &Wire);
  }
  if (!mpu_ok) {
    Serial.println(F("BLAD: MPU6050 nie odpowiada (0x68/0x69)."));
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println(F("OK: MPU6050 gotowy (akcelerometr + zyroskop)."));
  }
  

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

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}




void loop() {

  // Measured distance
  static uint32_t t0 = 0;
  uint32_t now = millis();
  if (now - t0 >= VL53LL0K_PERIOD_MS) {   // ma oznaczac ~10 Hz   idk czy VL53LL0K_PERIOD_MS sie do tych hz odnosi, ale bylo 100
    t0 = now;

    VL53L0X_RangingMeasurementData_t m;
    lox.rangingTest(&m, false); // false = bez debugów z biblioteki

    if (m.RangeStatus != 4 && m.RangeMilliMeter > 0) {
      Serial.print(F("[VL53] "));
      Serial.print(m.RangeMilliMeter);
      Serial.println(F(" mm"));
    } else {
      Serial.println(F("[VL53] poza zakresem / błąd"));
    }
  }

// ZLE JEST CHYBA USTAWIONE PRZYSPIESZENIE I NIE MA DOKALDNEGO POMIARU JAK NIC SIE NIE DZIEJE POIWNNO BYC IMO 0 0 1 
// [MPU] Acc[g]: 0.044 -0.041 1.185  |  Gyr[deg/s]: 8.5 -3.1 0.2
// [VL53] 60 mm
// [MPU] Acc[g]: 0.042 -0.043 1.187  |  Gyr[deg/s]: 8.5 -3.2 0.3
// [VL53] 58 mm
// [MPU] Acc[g]: 0.043 -0.043 1.186  |  Gyr[deg/s]: 8.5 -3.1 0.2


  // IMU Measured odometry
  static uint32_t t_mpu = 0;
  if (mpu_ok && (now - t_mpu >= MPU_PERIOD_MS)) {
    t_mpu = now;

    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float ax_g = a.acceleration.x / ONE_G;
    float ay_g = a.acceleration.y / ONE_G;
    float az_g = a.acceleration.z / ONE_G;

    float gx_dps = g.gyro.x * RAD2DEG;
    float gy_dps = g.gyro.y * RAD2DEG;
    float gz_dps = g.gyro.z * RAD2DEG;

    Serial.print(F("[MPU] Acc[g]: "));
    Serial.print(ax_g, 3); Serial.print(' ');
    Serial.print(ay_g, 3); Serial.print(' ');
    Serial.print(az_g, 3);

    Serial.print(F("  |  Gyr[deg/s]: "));
    Serial.print(gx_dps, 1); Serial.print(' ');
    Serial.print(gy_dps, 1); Serial.print(' ');
    Serial.print(gz_dps, 1);
    Serial.println();
  }

  // Looped servo movement is executed in setup

  // Web server is executed in setup

}
