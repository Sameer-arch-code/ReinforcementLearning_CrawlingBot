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


//state space and action space definition
constexpr int primary_arm_discrete_positions = 16;  //clipped between 15 and 90 degree . Allowed positions: [2, 3, 4, 6, 7, 10, 11, 16, 19, 31, 46, 91]
constexpr int secondary_arm_discete_positions = 19;  //clipped between 20 and 170 degree. Allowed positions: [2, 3, 4, 5, 6, 7, 10, 11, 13, 16, 19, 21, 31, 37, 46, 61, 91, 181]
constexpr int total_actions = primary_arm_discrete_positions + secondary_arm_discete_positions;

//Q stuff
//float Q[total_actions][secondary_arm_discete_positions][primary_arm_discrete_positions] = {0.0};//the Q table defined as [layer][row][col]
float discount_factor = 0.9; // discount factor
float learn_rate = 0.01; //learn rate

//training stuff
bool training = true;
unsigned long trainingStart = 0;  
unsigned long trainingTime = 15;

bool forward = false;

//stuff from pc stuff
String command;


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
  
  

  initialState = {random(0, secondary_arm_discete_positions), random(0, primary_arm_discrete_positions)};
  
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
  trainingStart = millis();
  Serial.print("trainingStart: "); Serial.println(trainingStart);

  
}

int PolicyRandom() {
  return random(0, total_actions); //generates between 0 and total_actions - 1
}

StepStruct Step(State currentState, int action ) {
  StepStruct result;
  ServoPositions servoPosition;

  if (action < primary_arm_discrete_positions) {
    result.nextState = {currentState.row, action};
    servoPosition = ConvertStateToServoPosition(result.nextState);

    servo_primary.write(servoPosition.primaryArmPosition);
    //Serial.print("Moved PRIMARY ARM to  --- "); Serial.println(servoPosition.primaryArmPosition);
    delay(ms_delay_to_capture_accel);
    result.reward = Reward();
  }
  else{
    result.nextState = {action - primary_arm_discrete_positions, currentState.col};
    servoPosition = ConvertStateToServoPosition(result.nextState);
    
    
    servo_secondary.write(servoPosition.secondaryArmPosition);
    //Serial.print("Moved secondary arm to  :   "); Serial.println(servoPosition.secondaryArmPosition);

    delay(ms_delay_to_capture_accel);
    result.reward = Reward();
  }
  return result;
}

ServoPositions ConvertStateToServoPosition(State state){
  ServoPositions servoPosition;
  
  servoPosition.primaryArmPosition = max(state.col * (90/(primary_arm_discrete_positions-1)), 15);
  servoPosition.secondaryArmPosition = max(min(state.row * (180/(secondary_arm_discete_positions-1)), 170), 20);

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

// int find_best_forward_action(State s) {
//     int best_action = 0;
//     float best = 9999;
//     for (int a = 0; a < total_actions; a++) {
//         if (Q[a][s.row][s.col] < best) {
//             best = Q[a][s.row][s.col];
//             best_action = a;
//         }
//     }
//     return best_action;
// }

// int find_best_backward_action(State s) {
//     int best_action = 0;
//     float best = -9999;
//     for (int a = 0; a < total_actions; a++) {
//         if (Q[a][s.row][s.col] > best) {
//             best = Q[a][s.row][s.col];
//             best_action = a;
//         }
//     }
//     return best_action;
// }


void loop() {
  //receiving data from pc
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
  }

  // Update pitch every 1ms
  if (millis() - lastPitchTime >= 1) {
    updatePitch();
  }

  // Run servo sequence every 750ms
  if (millis() - lastServoTime >= 750) {
    lastServoTime = millis();

    // Check if 5 minutes passed
    if (millis() - trainingStart >= trainingTime * 60UL * 1000UL) {
      //Serial.print("----------------------------------------------------------------------------------------millis:    "); 
      //Serial.println(millis()); Serial.print(15UL * 60UL * 1000UL);Serial.print("   is lesser than  : ");Serial.print(millis() - trainingStart);
      training = false;
    }

    
    if (training) {
      // --- TRAINING ---

      //requesting action
      Serial.print("ACTION_REQUEST:");
      
      //int action = PolicyRandom();//stuff that must happen in python

      int action = 0; 
      
      if (command.startsWith("Action:")) {
          action = command.substring(7).toInt();
          Serial.print("DEBUG: Action set to: ");
          Serial.println(action);
      } else {
          Serial.println("DEBUG: Command doesn't start with Action:");
      }
      

      Serial.print("DEBUG: Using action: ");
      Serial.println(action);

      StepStruct step = Step(state, action); //stuff that must happen here:

      State nextState = step.nextState;  

      //int next_action = forward ? find_best_forward_action(nextState) : find_best_backward_action(nextState);

      //sending data to pc
      Serial.print("PROCESS_DATA:");

      Serial.print("state_row:");
      Serial.print(state.row);

      Serial.print("state_col:");
      Serial.print(state.col);

      Serial.print("step_reward:");
      Serial.print(step.reward);

      Serial.print("next_state_row:");
      Serial.print(nextState.row);

      Serial.print("next_state_col:");
      Serial.println(nextState.col);



      //Q[action][state.row][state.col] += learn_rate * (step.reward + discount_factor * Q[next_action][nextState.row][nextState.col] - Q[action][state.row][state.col]);
      Serial.print("training in progress. Q table updated.  "); Serial.print((trainingTime * 60UL * 1000UL - (millis() - trainingStart)) / 60000UL); Serial.println("  minutes left");
      state = nextState;
      Serial.print("Ram left:   "); Serial.println(freeRam());

     } else {
      // --- GREEDY POLICY ---
      //int best_action = forward ? find_best_forward_action(state) : find_best_backward_action(state);

      //requesting action
      Serial.print("GREEDY_ACTION_REQUEST:");

      int best_action = 0; 
      
      if (command.startsWith("GreedyAction:")) {
          best_action = command.substring(13).toInt();
          Serial.print("DEBUG: Action set to: ");
          Serial.println(best_action);
      } else {
          Serial.println("DEBUG: Command doesn't start with Action:");
      }

      StepStruct step = Step(state, best_action); //stuff that must happen here
      state = step.nextState;
      Serial.println("Executing greedy policy  ");
      Serial.print("Ram left:   "); Serial.println(freeRam());
    }
  }
}

