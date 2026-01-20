// --- INDIVIDUAL WHEEL RAW TICK TESTER ---
// Goal: Spin each wheel by hand to verify direction.

// --- 1. PIN DEFINITIONS ---
// I have set these to the "Corrected" settings we discussed.
// If a wheel counts backwards, swap the two numbers for that wheel here.

// Front Left
const int FL_ENC_A = A2; 
const int FL_ENC_B = A3; 

// Front Right 
const int FR_ENC_A = A5;  
const int FR_ENC_B = A4;  

// Back Right 
const int BR_ENC_A = 5; 
const int BR_ENC_B = 4;

// Back Left 
const int BL_ENC_A = A0; 
const int BL_ENC_B = A1;

// --- 2. VARIABLES ---
long pos_FL = 0;
long pos_FR = 0;
long pos_BL = 0;
long pos_BR = 0;

int last_FL = 0; int last_FR = 0;
int last_BL = 0; int last_BR = 0;

void setup() {
  Serial.begin(9600);

  // Set pins to Input with Pullup (Prevents "floating" signals)
  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP); pinMode(BR_ENC_B, INPUT_PULLUP);
  pinMode(BL_ENC_A, INPUT_PULLUP); pinMode(BL_ENC_B, INPUT_PULLUP);

  // Read initial state
  last_FL = digitalRead(FL_ENC_A);
  last_FR = digitalRead(FR_ENC_A);
  last_BR = digitalRead(BR_ENC_A);
  last_BL = digitalRead(BL_ENC_A);

  Serial.println("--- RAW TICK TEST START ---");
  Serial.println("1. Lift Robot.");
  Serial.println("2. Spin ONE wheel Forward.");
  Serial.println("3. Verify number goes POSITIVE (+).");
}

void loop() {
  // Read all encoders continuously
  readEncoder(FL_ENC_A, FL_ENC_B, last_FL, pos_FL);
  readEncoder(FR_ENC_A, FR_ENC_B, last_FR, pos_FR);
  readEncoder(BR_ENC_A, BR_ENC_B, last_BR, pos_BR);
  readEncoder(BL_ENC_A, BL_ENC_B, last_BL, pos_BL);

  // Print Output (Only every 150ms to keep screen readable)
  static unsigned long last_print = 0;
  if (millis() - last_print > 150) {
    Serial.print("Front Left: "); Serial.print(pos_FL);
    Serial.print("   |   Front Right: "); Serial.print(pos_FR);
    Serial.print("   |   Back Left: "); Serial.print(pos_BL);
    Serial.print("   |   Back Right: "); Serial.println(pos_BR);
    last_print = millis();
  }
}

// Logic to count ticks
void readEncoder(int pinA, int pinB, int &lastState, long &count) {
  int currentState = digitalRead(pinA);
  
  // If Pin A changed...
  if (currentState != lastState) { 
    // Check Pin B to determine direction
    if (digitalRead(pinB) != currentState) {
      count++; // Forward
    } else {
      count--; // Backward
    }
  }
  lastState = currentState;
}
