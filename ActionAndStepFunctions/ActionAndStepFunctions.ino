// int myArray[4][3][3] = {
//   {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}},
//   {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}},
//   {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}},
//   {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}
// };
// Access with myArray[index][row][col], e.g. myArray[0][1][2] = 6.

struct State {
  int row;
  int col;
};
struct StepStruct{
  State nextState;
  float reward;
};

void setup() {
  Serial.begin(9600);
  long seed = 0;
  for (int i = 0; i < 64; i++) {
    seed += analogRead(A0);
    delay(5);
  }
  randomSeed(seed);
  
  int Q[14][7][7] = {0};

  
}

int PolicyRandom() {
  return random(1, 15);
}

StepStruct Step(State currentState, int action ) {
  StepStruct result;

  if (action < 8) {
    result.nextState = {currentState.row, action - 1};
    result.reward = 0.0;
  }
  else{
    result.nextState = {action - 8, currentState.col};
    result.reward = 0.0;
  }

  return result;
}

void loop() {

  State initialState = {random(0, 7), random(0, 7)};
  Serial.print("Row: "); Serial.println(initialState.row);
  Serial.print("Col: "); Serial.println(initialState.col);

  int action = PolicyRandom();
  Serial.print("action:  "); Serial.println(action);
  

  StepStruct step = Step(initialState, action);
  Serial.print("next state: "); Serial.print(step.nextState.row); Serial.print(","); Serial.println(step.nextState.col);
  Serial.print("reward:  "); Serial.println(step.reward);
  
}
