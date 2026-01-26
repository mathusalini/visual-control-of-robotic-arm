// --- PIN DEFINITIONS ---

// 1. MOTOR MUSCLES (Driver Pins)
// Based on standard 4WD shields or your previous code:
// FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
const int motorFL_PWM = 3;   // Front Left Speed
const int motorFL_Dir = 2;  // Front Left Direction (or vice versa depending on wiring)

const int motorFR_PWM = 11;   // Front Right Speed  <-- CONFLICT with Encoder!
const int motorFR_Dir = 12;   // Front Right Direction

// Assuming standard shield mapping for Back Motors (often duplicates of front or specific pins)
// If you are using a 4-channel driver, you need 4 more pins. 
// If you are using a 2-channel driver with 4 motors, the back motors share pins with front motors.
// **I will assume Back Motors share the Front Motor pins** (common in simple 4WD chassis).
// If you have 4 SEPARATE control pins for back motors, please provide them. 
// For now, we drive Front & Back together.

// 2. ENCODER EYES (Sensors)
// Front Right (FR): Pins 4 & 5
const int encFR_A = A5;
// const int encFR_B = A4; // DISABLED: Pin 5 is used for Motor PWM

// Front Left (FL): Pins A0 & A1
const int encFL_A = A2;
const int encFL_B = A3;

// Back Right (BR): Pins A4 & A5
const int encBR_A = A5;
const int encBR_B = A4;

// Back Left (BL): Pins A2 & A3
const int encBL_A = A0;
const int encBL_B = AA1;

void setup() {
  Serial.begin(9600);

  // --- SETUP MOTORS ---
  pinMode(motorFL_PWM, OUTPUT);
  pinMode(motorFL_Dir, OUTPUT);
  pinMode(motorFR_PWM, OUTPUT); // Pin 5 is forced to Output here
  pinMode(motorFR_Dir, OUTPUT);

  // --- SETUP ENCODERS ---
  pinMode(encFR_A, INPUT);
  // Pin 5 skipped (Used by Motor)
  
  pinMode(encFL_A, INPUT);
  pinMode(encFL_B, INPUT);
  
  pinMode(encBR_A, INPUT);
  pinMode(encBR_B, INPUT);
  
  pinMode(encBL_A, INPUT);
  pinMode(encBL_B, INPUT);

  Serial.println("--- MOTOR TEST START ---");
  delay(1000);
}

void loop() {
  // 1. SPIN FORWARD
  Serial.println("Moving FORWARD...");
  moveMotors(200, true); // Speed 200, Forward
  readEncodersForDuration(2000); // Read sensors for 2 seconds

  // 2. STOP
  Serial.println("STOPPING...");
  stopMotors();
  delay(1000);

  // 3. SPIN BACKWARD
  Serial.println("Moving BACKWARD...");
  moveMotors(200, false); // Speed 200, Backward
  readEncodersForDuration(2000);

  // 4. STOP
  Serial.println("STOPPING...");
  stopMotors();
  delay(1000);
}

// --- HELPER FUNCTIONS ---

void moveMotors(int speed, boolean forward) {
  // If forward is true, we set Direction Low and PWM High (or vice versa)
  if (forward) {
    // Front Left
    digitalWrite(motorFL_PWM, HIGH); // Max speed (simple test)
    digitalWrite(motorFL_Dir, LOW);
    
    // Front Right
    digitalWrite(motorFR_PWM, HIGH); 
    digitalWrite(motorFR_Dir, LOW);
  } else {
    // Front Left
    digitalWrite(motorFL_PWM, LOW); 
    digitalWrite(motorFL_Dir, HIGH);
    
    // Front Right
    digitalWrite(motorFR_PWM, LOW); 
    digitalWrite(motorFR_Dir, HIGH);
  }
}

void stopMotors() {
  digitalWrite(motorFL_PWM, LOW);
  digitalWrite(motorFL_Dir, LOW);
  digitalWrite(motorFR_PWM, LOW);
  digitalWrite(motorFR_Dir, LOW);
}

void readEncodersForDuration(int duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    // Read all encoders
    int valFR = digitalRead(encFR_A);
    int valFL = digitalRead(encFL_A);
    int valBR = digitalRead(encBR_A);
    int valBL = digitalRead(encBL_A);

    // Print Status
    Serial.print("FR:"); Serial.print(valFR);
    Serial.print(" | FL:"); Serial.print(valFL);
    Serial.print(" | BR:"); Serial.print(valBR);
    Serial.print(" | BL:"); Serial.println(valBL);
    
    delay(50); // Slow down so we can read text
  }
}
