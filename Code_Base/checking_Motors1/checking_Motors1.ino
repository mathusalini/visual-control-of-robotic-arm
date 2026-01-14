/*
  Simple Motor Test Code
  Based on Nexus Robot Manual PWM Pins: 5, 6, 9, 10
*/

void setup() {
  // Set all motor pins to OUTPUT mode
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop() {
  // PHASE 1: SPIN ALL WHEELS (Full Speed)
  // We use analogWrite because these are PWM pins [cite: 475]
  analogWrite(5, 255);   // Pin 5
  analogWrite(6, 255);   // Pin 6
  analogWrite(9, 255);   // Pin 9
  analogWrite(10, 255);  // Pin 10
  
  delay(2000); // Run for 2 seconds

  // PHASE 2: STOP (Rest)
  analogWrite(5, 0);
  analogWrite(6, 0);
  analogWrite(9, 0);
  analogWrite(10, 0);
  
  delay(1000); // Stop for 1 second
}