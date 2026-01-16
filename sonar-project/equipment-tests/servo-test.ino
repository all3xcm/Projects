// Include Library //

#include <Servo.h>

// Create Object //

Servo testservo;

// Servo Motor Test //

int i = 0;

void sweep(int time) {
  for (i = 0; i <= 180; i ++) {
    testservo.write(i);
    Serial.println(i);
    delay(time);
  }
}

// Blink //

void blink(int time) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(time);
  digitalWrite(LED_BUILTIN, LOW);
  delay(time);
}

void setup() {

  // Inititate Serial //

  Serial.begin(9600);
  
  // Set PinMode //

  pinMode(LED_BUILTIN, OUTPUT);
  
  // Attach Servo and Set to 0 Degrees //

  testservo.attach(10);
  testservo.writeMicroseconds(1000);
  
  // Code //

  sweep(10);

}

void loop() {

  // Code //

  blink(3000);
  Serial.println("Blink");

}
