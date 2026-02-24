#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
Servo servo_primary;
Servo servo_secondary;

float pitch = 0;
unsigned long lastPitchTime = 0;
unsigned long lastServoTime = 0;
int servoStep = 0;
int ms_delay_to_capture_accel = 50;

int primary_arm_discrete_positions = 4;
int secondary_arm_discete_positions = 7;
int total_actions = primary_arm_discrete_positions + secondary_arm_discete_positions;


struct State {
  int row;
  int col;
};

struct ServoPositions{
  int primaryArmPosition;
  int secondaryArmPosition;
};
struct StepStruct{
  State nextState;
  float reward;
};

State initialState;
State state;

void setup() {
  Serial.begin(115200);

  //seed stuff begins
  long seed = 0;
  for (int i = 0; i < 64; i++) {
    seed += analogRead(A0);
    delay(5);
  }
  randomSeed(seed);
  //seed stuff ends
  
  int Q[total_actions][secondary_arm_discete_positions][primary_arm_discrete_positions] = {0}; //the Q table

  initialState = {random(0, 7), random(0, 4)};
  
  Serial.print("Row: "); Serial.println(initialState.row);
  Serial.print("Col: "); Serial.println(initialState.col);

  state = initialState; //used only once in the loop

  
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
  servo_primary.attach(4);
  servo_secondary.attach(3);
  lastPitchTime = millis();
  lastServoTime = millis();

  
}

int PolicyRandom() {
  return random(1, 12);
}

StepStruct Step(State currentState, int action ) {
  StepStruct result;
  ServoPositions servoPosition;

  if (action < primary_arm_discrete_positions+1) {
    result.nextState = {currentState.row, action - 1};
    servoPosition = ConvertStateToServoPosition(result.nextState);

    servo_primary.write(servoPosition.primaryArmPosition);
    Serial.print("Moved primary arm to  :   "); Serial.println(servoPosition.primaryArmPosition);
    delay(ms_delay_to_capture_accel);
    result.reward = Reward();
  }
  else{
    result.nextState = {action - (primary_arm_discrete_positions+1), currentState.col};
    servoPosition = ConvertStateToServoPosition(result.nextState);
    
    
    servo_secondary.write(servoPosition.secondaryArmPosition);
    Serial.print("Moved secondary arm to  :   "); Serial.println(servoPosition.secondaryArmPosition);

    delay(ms_delay_to_capture_accel);
    result.reward = Reward();
  }
  return result;
}

ServoPositions ConvertStateToServoPosition(State state){
  ServoPositions servoPosition;
  
  servoPosition.primaryArmPosition = state.col * 30;
  servoPosition.secondaryArmPosition = state.row * 30;

  return servoPosition;
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

void updatePitch() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  float dt = (now - lastPitchTime) / 1000.0;
  lastPitchTime = now;

  float pitchAcc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  pitch = 0.96 * (pitch + g.gyro.y * dt * 180 / PI) + 0.04 * pitchAcc;
}

void loop() {

  // Update pitch every 1ms
  if (millis() - lastPitchTime >= 1) {
    updatePitch();
  }

  // Run servo sequence every 1000ms
  if (millis() - lastServoTime >= 1000) {
    lastServoTime = millis();

    

    int action = PolicyRandom();
    Serial.print("action:  "); Serial.println(action);
    

    StepStruct step = Step(state, action);
    state = step.nextState;
    Serial.print("next state: "); Serial.print(state.row); Serial.print(","); Serial.println(state.col);
    Serial.print("reward:  "); Serial.println(step.reward);
  }
}
