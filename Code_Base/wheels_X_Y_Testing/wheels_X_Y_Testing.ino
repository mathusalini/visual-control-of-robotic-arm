// --- ODOMETRY TEST (X and Y) ---
// Goal: Verify robot calculates position correctly.

// --- 1. PIN DEFINITIONS (YOUR CONFIRMED PINS) ---

// Front Left (A2, A3)
const int FL_ENC_A = A2; 
const int FL_ENC_B = A3; 

// Front Right (A5, A4)
const int FR_ENC_A = A5;  
const int FR_ENC_B = A4;  

// Back Right (5, 4)
const int BR_ENC_A = 5; 
const int BR_ENC_B = 4;

// Back Left (A0, A1)
const int BL_ENC_A = A0; 
const int BL_ENC_B = A1;

// --- 2. ROBOT SETTINGS (Adjust these later for perfect accuracy) ---
const float ticks_per_rev = 980.0; // Typical for these motors
const float wheel_radius = 0.05;    // 50mm radius (100mm wheels)

// --- 3. VARIABLES ---
long pos_FL = 0; long pos_FR = 0;
long pos_BL = 0; long pos_BR = 0;

int last_FL = 0; int last_FR = 0;
int last_BL = 0; int last_BR = 0;

unsigned long last_print = 0;

void setup() {
  Serial.begin(9600);

  // Setup Pins
  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP); pinMode(BR_ENC_B, INPUT_PULLUP);
  pinMode(BL_ENC_A, INPUT_PULLUP); pinMode(BL_ENC_B, INPUT_PULLUP);

  // Read initial state
  last_FL = digitalRead(FL_ENC_A);
  last_FR = digitalRead(FR_ENC_A);
  last_BR = digitalRead(BR_ENC_A);
  last_BL = digitalRead(BL_ENC_A);

  Serial.println("--- X / Y TEST START ---");
  Serial.println("Push Forward -> X should increase.");
  Serial.println("Slide Right  -> Y should change.");
}

void loop() {
  // --- A. READ ENCODERS ---
  readEncoder(FL_ENC_A, FL_ENC_B, last_FL, pos_FL);
  readEncoder(FR_ENC_A, FR_ENC_B, last_FR, pos_FR);
  readEncoder(BR_ENC_A, BR_ENC_B, last_BR, pos_BR);
  readEncoder(BL_ENC_A, BL_ENC_B, last_BL, pos_BL);

  // --- B. CALCULATE X & Y ---
  // Only print every 100ms
  if (millis() - last_print > 100) {
    
    // 1. Convert Ticks to Meters
    float meters_per_tick = (2 * 3.14159 * wheel_radius) / ticks_per_rev;
    
    float d_FL = pos_FL * meters_per_tick;
    float d_FR = pos_FR * meters_per_tick;
    float d_BL = pos_BL * meters_per_tick;
    float d_BR = pos_BR * meters_per_tick;

    // 2. Kinematics Formulas (Mecanum)
    // X = Average of all wheels (Forward)
    float x_pos = (d_FL + d_FR + d_BL + d_BR) / 4.0;
    
    // Y = Difference for Strafe (Left/Right sliding)
    // Formula: (-FL + FR + BL - BR) / 4  <-- Standard Mecanum Formula
    float y_pos = (-d_FL + d_FR + d_BL - d_BR) / 4.0;

    // 3. Print Output
    Serial.print("FL:"); Serial.print(pos_FL);
    Serial.print(" FR:"); Serial.print(pos_FR);
    Serial.print(" BL:"); Serial.print(pos_BL);
    Serial.print(" BR:"); Serial.print(pos_BR);
    
    Serial.print("  |  METERS ->  X: "); Serial.print(x_pos);
    Serial.print("   Y: "); Serial.println(y_pos);

    last_print = millis();
  }
}

// Helper Function
void readEncoder(int pinA, int pinB, int &lastState, long &count) {
  int currentState = digitalRead(pinA);
  if (currentState != lastState) { 
    if (digitalRead(pinB) != currentState) {
      count++; 
    } else {
      count--; 
    }
  }
  lastState = currentState;
}
