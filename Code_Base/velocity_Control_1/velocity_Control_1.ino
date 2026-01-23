// --- STEP 2.4: MOTOR + ENCODER SANITY CHECK ---
// No PID Math. Just drive and count. 
// Uses YOUR EXACT PINS.

// 1. PIN DEFINITIONS (Updated to your list)
// --- FRONT LEFT ---
const int FL_PWM = 3;   const int FL_DIR = 2;
const int FL_ENC_A = A2; const int FL_ENC_B = A3;

// --- FRONT RIGHT ---
const int FR_PWM = 11;  const int FR_DIR = 12;
const int FR_ENC_A = A5; const int FR_ENC_B = A4;

// --- BACK LEFT ---
const int BL_PWM = 10;  const int BL_DIR = 7;
const int BL_ENC_A = A0; const int BL_ENC_B = A1;

// --- BACK RIGHT ---
const int BR_PWM = 9;   const int BR_DIR = 8;
const int BR_ENC_A = 5;  const int BR_ENC_B = 4;

// 2. VARIABLES
volatile long fl_ticks = 0;
volatile long fr_ticks = 0;
volatile long bl_ticks = 0;
volatile long br_ticks = 0;

// Polling State
int last_fl = 0, last_fr = 0, last_bl = 0, last_br = 0;

void setup() {
  Serial.begin(115200);

  // Setup Motor Pins
  pinMode(FL_PWM, OUTPUT); pinMode(FL_DIR, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_DIR, OUTPUT);
  pinMode(BL_PWM, OUTPUT); pinMode(BL_DIR, OUTPUT);
  pinMode(BR_PWM, OUTPUT); pinMode(BR_DIR, OUTPUT);

  // Setup Encoder Pins
  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(BL_ENC_A, INPUT_PULLUP); pinMode(BL_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP); pinMode(BR_ENC_B, INPUT_PULLUP);

  Serial.println("--- SANITY CHECK START ---");
  delay(1000);
}

void loop() {
  // 1. FORCE MOTORS TO MOVE (Power 100 out of 255)
  digitalWrite(FL_DIR, HIGH); analogWrite(FL_PWM, 100);
  digitalWrite(FR_DIR, HIGH); analogWrite(FR_PWM, 100);
  digitalWrite(BL_DIR, HIGH); analogWrite(BL_PWM, 100);
  digitalWrite(BR_DIR, HIGH); analogWrite(BR_PWM, 100);

  // 2. READ ENCODERS (FAST POLL)
  // We check the pins 2000 times in a row before printing.
  // This ensures we don't miss ticks while printing text.
  for(int i=0; i<2000; i++) {
    pollEncoders();
  }

  // 3. PRINT REPORT
  // We only print occasionally so we don't slow down the processor
  Serial.print("FL:"); Serial.print(fl_ticks);
  Serial.print(" | FR:"); Serial.print(fr_ticks);
  Serial.print(" | BL:"); Serial.print(bl_ticks);
  Serial.print(" | BR:"); Serial.println(br_ticks);
}

// Optimized Polling Function
void pollEncoders() {
  // Front Left
  int curr_fl = digitalRead(FL_ENC_A);
  if (curr_fl != last_fl && curr_fl == HIGH) fl_ticks++;
  last_fl = curr_fl;

  // Front Right
  int curr_fr = digitalRead(FR_ENC_A);
  if (curr_fr != last_fr && curr_fr == HIGH) fr_ticks++;
  last_fr = curr_fr;

  // Back Left
  int curr_bl = digitalRead(BL_ENC_A);
  if (curr_bl != last_bl && curr_bl == HIGH) bl_ticks++;
  last_bl = curr_bl;

  // Back Right
  int curr_br = digitalRead(BR_ENC_A);
  if (curr_br != last_br && curr_br == HIGH) br_ticks++;
  last_br = curr_br;
}
