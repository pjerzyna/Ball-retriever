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
const int pwmB = 6, in1B = 5, in2B = 4;

const int freq = 20000;          // 20kHz
const int res = 8;               // 8-bit resolution (0..255)

// Motor Speed Values - const
const uint8_t MotorSpeed1 = 128;
const uint8_t MotorSpeed2 = 128;
 
// Servo pinout
const int SERVO_PIN = 1;   
Servo servo1;

// Distance sensor pinout
const int ECHO = 2;
const int TRIG = 3;
 

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
  ledcWrite(pwmA, MotorSpeed1);
  ledcWrite(pwmB, MotorSpeed2);

  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  // Servo initialization - do not overwrite timers before motors configuration
  servo1.attach(SERVO_PIN);

  Serial.setDebugOutput(true);
  Serial.println();

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
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming !!!!!!!!!!!!!!!!
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

int measure_distance() {
  long duration, distance;

  // trigger mechanism
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn(ECHO, HIGH);
  distance = duration / 58;

  return distance;
}

void loop() {

  // Measured distance
  Serial.print(measure_distance());
  Serial.println(" cm");
  delay(500);

  // Looped servo movement
  for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
        servo1.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }

    for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
        servo1.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }
  

  // Web server is executed in setup

}
