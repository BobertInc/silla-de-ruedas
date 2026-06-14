#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

// ESP NOW DATA
typedef struct __attribute__((packed)) {
  float x;
  float y;
} ControlData;

ControlData data;

uint8_t receiverMac[] = {0x88, 0x13, 0xBF, 0x09, 0x49, 0xC8};

float Total_angle_x = 0;
float Total_angle_y = 0;

float gyroOffsetX = 0;
float gyroOffsetY = 0;

float elapsedTime;
unsigned long previousTime;

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP NOW INIT FAILED");
    while (1);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("PEER ADD FAILED");
    while (1);
  }

  Serial.println("ESP NOW READY");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);

  if (Wire.endTransmission() == 0) {
    Serial.println("MPU6050 FOUND");
  } else {
    Serial.println("MPU6050 NOT FOUND");
    while (1);
  }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);

  // GYRO CALIBRATION
  // KEEP MPU STILL
  Serial.println("CALIBRATING GYRO");

  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 4, true);

    if(Wire.available() >= 4) {
      int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
      int16_t rawGyroY = (Wire.read() << 8) | Wire.read();

      gyroOffsetX += (float)rawGyroX / 131.0;
      gyroOffsetY += (float)rawGyroY / 131.0;
    }

    delay(5);
  }

  gyroOffsetX /= 500.0;
  gyroOffsetY /= 500.0;

  Serial.println("CALIBRATION DONE");

  previousTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  elapsedTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
 
  int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroY = (Wire.read() << 8) | Wire.read();

  float gyroX = ((float)rawGyroX / 131.0) - gyroOffsetX;
  float gyroY = ((float)rawGyroY / 131.0) - gyroOffsetY;

  // READ ACCEL
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t rawAccX = (Wire.read() << 8) | Wire.read();
  int16_t rawAccY = (Wire.read() << 8) | Wire.read();
  int16_t rawAccZ = (Wire.read() << 8) | Wire.read();

  float accX = (float)rawAccX / 16384.0;
  float accY = (float)rawAccY / 16384.0;
  float accZ = (float)rawAccZ / 16384.0;
  
  // ACCEL ANGLES
  float Acc_angle_x = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  float Acc_angle_y = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  // COMPLEMENTARY FILTER
  Total_angle_x = 0.98 * (Total_angle_x + gyroX * elapsedTime) + 0.02 * Acc_angle_x;
  Total_angle_y = 0.98 * (Total_angle_y + gyroY * elapsedTime) + 0.02 * Acc_angle_y;
  
  // SEND DATA
  data.x = constrain(Total_angle_x, -45.0, 45.0);
  data.y = constrain(Total_angle_y, -45.0, 45.0);
  esp_err_t result = esp_now_send(receiverMac, (uint8_t*)&data, sizeof(data));

  // DEBUG
  Serial.print("X: ");
  Serial.print(data.x);
  Serial.print(" | Y: ");
  Serial.print(data.y);
  Serial.print(" | SEND: ");

  if (result == ESP_OK) {
    Serial.println("OK");
  } else {
    Serial.println("FAIL");
  }

  delay(30);
}
