// Include Library //

#include <NewPing.h>

// Create Object //

NewPing sonar(9, 12, 200);

// Ultrasonic Sensor Test //

void ping(int time) {
  digitalWrite(LED_BUILTIN, HIGH);
  float radius = sonar.ping_cm();
  Serial.println(radius);
  digitalWrite(LED_BUILTIN, LOW);
  delay(time);
}

void setup() {
  
  // Initiate Serial //

  Serial.begin(9600);

  // Setup Pin Mode //

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  // Code //

  ping(3000);
}
