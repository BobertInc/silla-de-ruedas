#include <esp_now.h>
#include <WiFi.h>

#define AIN1 16
#define AIN2 17
#define PWMA 18
#define BIN1 19
#define BIN2 21
#define PWMB 22

#define STBY 23

#define ANGLE_DEADZONE 20
#define MAX_PWM 110
#define TURN_STRENGTH 1
#define RAMP_STEP 4
#define SIGNAL_TIMEOUT 500

// ESP NOW COMMUNICATION
typedef struct {
  float x;
  float y;
} structControlData;

structControlData incomingData;
unsigned long lastPacketTime = 0;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingDataBytes, int len) {
  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));
  lastPacketTime = millis();
}

// MOTOR LOGIC
int currentLeftPWM = 0;
int currentRightPWM = 0;

void driveMotor(int pwm, int in1, int in2, int pwmPin) {
  if (pwmPin != PWMA) {
    pwm = constrain(pwm, -255, 255);
  } else {
    pwm = pwm * 1.25;
    pwm = constrain(pwm, -255, 255);
  }

  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, pwm);

  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, abs(pwm));

  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP NOW INIT FAILED");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // FAILSAFE SIGNAL DELAY
  if (millis() - lastPacketTime > SIGNAL_TIMEOUT) {
    driveMotor(0, AIN1, AIN2, PWMA);
    driveMotor(0, BIN1, BIN2, PWMB);
    return;
  }                                                                                                                                                     

  float x = incomingData.x;
  float y = incomingData.y;

  // DEADZONE
  if (abs(x) < ANGLE_DEADZONE) x = 0;
  if (abs(y) < ANGLE_DEADZONE) y = 0;

  // MAP TO CONTROL
  int throttle = map(constrain((int)y, -45, 45), -45, 45, -MAX_PWM, MAX_PWM);
  int steering = map(constrain((int)x, -45, 45), -45, 45, -MAX_PWM * TURN_STRENGTH, MAX_PWM * TURN_STRENGTH);

  // DIFFERENTIAL DRIVE

  int targetLeft = throttle + steering;
  int targetRight = throttle - steering;
  if (targetLeft < 60) targetLeft = 0;
  if (targetRight < 60) targetRight = 0;
  targetLeft = constrain(targetLeft, -MAX_PWM, MAX_PWM);
  targetRight = constrain(targetRight, -MAX_PWM, MAX_PWM);

  // RAMPING
  if (currentLeftPWM < targetLeft) currentLeftPWM += RAMP_STEP;
  if (currentLeftPWM > targetLeft) currentLeftPWM -= RAMP_STEP;
  if (currentRightPWM < targetRight) currentRightPWM += RAMP_STEP;
  if (currentRightPWM > targetRight) currentRightPWM -= RAMP_STEP;

  // DRIVE
  driveMotor(currentLeftPWM, AIN1, AIN2, PWMA);
  driveMotor(currentRightPWM, BIN1, BIN2, PWMB);

  Serial.print("L: ");
  Serial.print(currentLeftPWM);
  Serial.print(" R: ");
  Serial.println(currentRightPWM);
  delay(20);
}
