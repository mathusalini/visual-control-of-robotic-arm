/*
  Day 4: Omni Wheel Tank Control
  Goal: Forward, Backward, Turn Left, Turn Right.
*/

// --- Pin Definitions (From your Verification) ---
// Front Left
int FL_PWM = 2;   int FL_DIR = 3;   
// Front Right
int FR_PWM = 11;  int FR_DIR = 12;
// Back Right
int BR_PWM = 9;   int BR_DIR = 8;
// Back Left
int BL_PWM = 7;  int BL_DIR = 10;

void setup() {
  pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
  pinMode(BR_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);
  pinMode(BL_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);

  Serial.begin(9600);
  Serial.println("--- Day 4: Tank Control Start ---");
  delay(3000); 
}

void loop() {
  // 1. Move Forward
  Serial.println("FORWARD");
  moveTank(1, 1); // Left side Forward, Right side Forward
  delay(1500);
  stopRobot();
  delay(500);

  // 2. Move Backward
  Serial.println("BACKWARD");
  moveTank(-1, -1); // Left side Back, Right side Back
  delay(1500);
  stopRobot();
  delay(500);

  // 3. Spin Left (Rotate in place)
  Serial.println("SPIN LEFT");
  moveTank(-1, 1); // Left side Back, Right side Forward
  delay(1500);
  stopRobot();
  delay(500);

  // 4. Spin Right (Rotate in place)
  Serial.println("SPIN RIGHT");
  moveTank(1, -1); // Left side Forward, Right side Back
  delay(1500);
  stopRobot();
  
  Serial.println("--- Waiting... ---");
  delay(3000);
}

// --- Helper Function for Tank Drive ---
// leftInput: 1 (Fwd), -1 (Back)
// rightInput: 1 (Fwd), -1 (Back)
void moveTank(int leftInput, int rightInput) {
  // Control Left Side (Front Left + Back Left)
  setMotor(FL_PWM, FL_DIR, leftInput);
  setMotor(BL_PWM, BL_DIR, leftInput);

  // Control Right Side (Front Right + Back Right)
  setMotor(FR_PWM, FR_DIR, rightInput);
  setMotor(BR_PWM, BR_DIR, rightInput);
}

// Basic Motor Function
void setMotor(int pwmPin, int dirPin, int direction) {
  int speed = 100; // Speed 100/255
  
  if (direction == 1) {          
    digitalWrite(dirPin, HIGH); // Forward
    analogWrite(pwmPin, speed);
  } 
  else if (direction == -1) {    
    digitalWrite(dirPin, LOW);  // Backward
    analogWrite(pwmPin, speed);
  } 
  else {                         
    analogWrite(pwmPin, 0);     // Stop
  }
}

void stopRobot() {
  moveTank(0, 0);
}
