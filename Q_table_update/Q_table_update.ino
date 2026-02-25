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

constexpr int primary_arm_discrete_positions = 4;
constexpr int secondary_arm_discete_positions = 7;
constexpr int total_actions = primary_arm_discrete_positions + secondary_arm_discete_positions;

//Q stuff
float Q[total_actions][secondary_arm_discete_positions][primary_arm_discrete_positions] = {0.0};//the Q table defined as [layer][row][col]
float discount_factor = 0.9; // discount factor
float learn_rate = 0.01; //learn rate


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
    Serial.print("Moved PRIMARY ARM to  --- "); Serial.println(servoPosition.primaryArmPosition);
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

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


bool training = true;
unsigned long trainingStart = 0;  

void loop() {

  

  // Update pitch every 1ms
  if (millis() - lastPitchTime >= 1) {
    updatePitch();
  }

  // Run servo sequence every 750ms
  if (millis() - lastServoTime >= 750) {
    lastServoTime = millis();

    // Check if 5 minutes passed
    if (millis() - trainingStart >= 15UL * 60UL * 1000UL) {
      training = false;
    }

    if (training) {
      // --- TRAINING ---
      int action = PolicyRandom();

      StepStruct step = Step(state, action);

      State nextState = step.nextState;  

      int next_action = 0;
      float best = -999;
      for (int a = 0; a < 12; a++) {
        if (Q[a][nextState.row][nextState.col] > best) {
          best = Q[a][nextState.row][nextState.col];
          next_action = a;
        }
      }

  
      Q[action][state.row][state.col] += learn_rate * (step.reward + discount_factor * Q[next_action][nextState.row][nextState.col] - Q[action][state.row][state.col]);
      Serial.print("training in progress. Q table updated.  "); Serial.print((15UL * 60UL * 1000UL - millis())/ 60000); Serial.println("  minutes left");
      state = nextState;
      Serial.print("Ram left:   "); Serial.println(freeRam());

    } else {
      // --- GREEDY POLICY ---
      int best_action = 0;
      float best = -999;
      for (int a = 0; a < 12; a++) {
        if (Q[a][state.row][state.col] > best) {
          best = Q[a][state.row][state.col];
          best_action = a;
        }
      }

      StepStruct step = Step(state, best_action);
      state = step.nextState;
      Serial.println("Executing greedy policy  ");
      Serial.print("Ram left:   "); Serial.println(freeRam());
    }
  }
}

