#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
Servo servo1;
Servo servo2;

float pitch = 0;
unsigned long lastPitchTime = 0;
unsigned long lastServoTime = 0;
int servoStep = 0;
int ms_delay_to_capture_accel = 50;

void updatePitch() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  float dt = (now - lastPitchTime) / 1000.0;
  lastPitchTime = now;

  float pitchAcc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  pitch = 0.96 * (pitch + g.gyro.y * dt * 180 / PI) + 0.04 * pitchAcc;
}

float findAccelX() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float pitch_rad = pitch * (PI / 180.0);

  float use_less_g_on_x = -9.81 * sin(pitch_rad);
  float accel_x_minus_g = a.acceleration.x - use_less_g_on_x;
  float true_x_accel = accel_x_minus_g * cos(pitch_rad);

  float use_less_g_on_z = 9.81 * cos(pitch_rad);
  float accel_z_minus_g = a.acceleration.z - use_less_g_on_z;
  float true_z_accel_in_x_direction = accel_z_minus_g * sin(pitch_rad);

  return true_x_accel + true_z_accel_in_x_direction;
}

void setup() {
  Serial.begin(115200);
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  servo1.attach(4);
  servo2.attach(3);
  lastPitchTime = millis();
  lastServoTime = millis();
}

float Reward(){
    float accelX = findAccelX();
    float reward = 0.0;

    if (accelX >= 2.0) {
        reward = accelX - 2.0;
    } else if (accelX <= -2.0) {
        reward = accelX + 2.0;
    }

    return reward;
}

void loop() {
  // Update pitch every 5ms
  if (millis() - lastPitchTime >= 1) {
    updatePitch();
  }

  // Run servo sequence every 200ms
  if (millis() - lastServoTime >= 1000) {
    lastServoTime = millis();

    switch (servoStep) {
      case 0:
        servo1.write(10);
        Serial.print("target1: 10  | Reward: ");
        delay(ms_delay_to_capture_accel);
        Serial.println(Reward());
        break;
      case 1:
        servo2.write(0);   // 180-180
        Serial.print("target2: 180 | Reward: ");
        delay(ms_delay_to_capture_accel);
        Serial.println(Reward());
        break;
      case 2:
        servo1.write(75);
        Serial.print("target1: 20  | Reward: ");
        delay(ms_delay_to_capture_accel);
        Serial.println(Reward());
        break;
      case 3:
        servo2.write(150); // 180-30
        Serial.print("target2: 30  | Reward: ");
        delay(ms_delay_to_capture_accel);
        Serial.println(Reward());
        break;
      case 4:
        servo1.write(0);
        Serial.print("target1: 0   | Reward: ");
        delay(ms_delay_to_capture_accel);
        Serial.println(Reward());
        break;
      case 5:
        servo2.write(150); // 180-30
        Serial.print("target2: 30  | Reward: ");
        delay(ms_delay_to_capture_accel);
        Serial.println(Reward());
        break;
      case 6:
        Serial.println("___________________________");

      //04:58:49.625 -> target1: 10  | accelX: 0.00
      // 04:58:50.626 -> target2: 180 | accelX: 0.00
      // 04:58:51.646 -> target1: 20  | accelX: 0.00
      // 04:58:52.642 -> target2: 30  | accelX: 1.78
      // 04:58:53.634 -> target1: 0   | accelX: -1.90
      // 04:58:54.642 -> target2: 30  | accelX: 0.00
      // 04:58:55.650 -> ___________________________
      // 04:58:56.666 -> target1: 10  | accelX: 0.00
      // 04:58:57.650 -> target2: 180 | accelX: 0.00
      // 04:58:58.652 -> target1: 20  | accelX: 0.00
      // 04:58:59.667 -> target2: 30  | accelX: 1.08
      // 04:59:00.667 -> target1: 0   | accelX: -3.44
      // 04:59:01.659 -> target2: 30  | accelX: 0.00
      // 04:59:02.642 -> ___________________________

    }

    servoStep++;
    if (servoStep > 6) servoStep = 0;
  }
}