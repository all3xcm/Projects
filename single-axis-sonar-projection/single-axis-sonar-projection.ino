// Setup //

// Import Libraries and external Structure //

#include "Read.h"
#include <Servo.h>
#include <NewPing.h>
  
// Create Objects //
  
Servo scanner;
NewPing sonar(9, 12, 200);
  
// Create Logic //

// Setup Variables //

int i = 0;

// Servo Sweep //

void sweep(int time) {
    delay(time);
    scanner.write(i);
}
    
// LED Blink //

void blink(int time) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(time);
  digitalWrite(LED_BUILTIN, LOW);
  delay(time);
}
  
// Measure Distance & Angle with external Structure //

Read ping() {
  Read r;
  r.theta = scanner.read();
  r.radius = sonar.ping_cm();
  return r;
}

// Print Values //

void transmit(int angle, float radius) {
  Serial.println(angle);
  Serial.println(radius);
}

// Execute Program //

// Execute Setup //

void setup() {

  // Initiate Serial //
	
  Serial.begin(9600);

  // Assign to Pins //
  
  scanner.attach(3);

  // Assign Pin Types //

  pinMode(LED_BUILTIN, OUTPUT);

  // Code //

  for (i = 0; i<= 180; i++) {
    Read r = ping();
    transmit(r.theta, r.radius);
    sweep(10);
  }
}

// Execute Loop //

void loop() {

  // Code //
  
  blink(1000);
}