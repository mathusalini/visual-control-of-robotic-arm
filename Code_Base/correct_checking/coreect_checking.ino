// --- Pin Definitions (Based on Standard Nexus 4WD) ---

// Front Left

int FL_PWM = 3;   // Speed

int FL_DIR = 2;   // Direction



// Front Right

int FR_PWM = 11;

int FR_DIR = 12;



// Back Right

int BR_PWM = 9;

int BR_DIR = 8;



// Back left

int BL_PWM = 10;

int BL_DIR = 7;



void setup() {

  // 1. Setup all pins as OUTPUTs so we can send power

  pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);

  pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);

  pinMode(BL_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);

  pinMode(BR_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);



  // 2. Start Serial Monitor so you can read what the robot is doing

  Serial.begin(9600);

  Serial.println("--- STARTING MOTOR TEST ---");

  delay(3000); // Give you 3 seconds to get ready

}



void loop() {

  // --- Test 1: Front Left ---

  Serial.println("TESTING: Front Left Wheel (Pin 3/2)");

  runMotor(FL_PWM, FL_DIR);

  delay(1000); // Wait 1 second



  // --- Test 2: Front Right ---

  Serial.println("TESTING: Front Right Wheel (Pin 11/12)");

  runMotor(FR_PWM, FR_DIR);

  delay(1000);



  // --- Test 3: Back Left ---

  Serial.println("TESTING: Back Right Wheel (Pin 9/8)");

  runMotor(BR_PWM, BR_DIR);

  delay(1000);



  // --- Test 4: Back Right ---

  Serial.println("TESTING: Back Left Wheel (Pin 10/7)");

  runMotor(BL_PWM, BL_DIR);

  delay(1000);



  Serial.println("--- DONE. Waiting 5 seconds... ---");

  delay(5000);

}



// Helper function to spin a motor forward for 1.5 seconds

void runMotor(int pwmPin, int dirPin) {

  digitalWrite(dirPin, HIGH);  // Set Direction (Try HIGH first)

  analogWrite(pwmPin, 100);    // Set Speed (0-255, 100 is slow/safe)

  delay(1500);                 // Run for 1.5 seconds

  analogWrite(pwmPin, 0);      // Stop

}
