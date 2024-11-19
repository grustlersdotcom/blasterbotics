#include <Servo.h>

Servo esc;  // Create a Servo object to control the ESC


int Throttle = 1100;  // The Throttle controls the speed and direction of the motor  
//int specificDutyCycle = 650;  I don't think we need this

void setup() {
  Serial.begin(115200); 
  esc.attach(11);      
}

void loop() {

  // Run wheels
  esc.writeMicroseconds(Throttle);
  delay(500); 

  // Stop Wheels
  esc.writeMicroseconds(0);
  delay(200); 

  // Increase throttle by 10
  Throttle = Throttle + 10;
  Serial.println(Throttle);

  //1380 slow backwards
  //1650 slow forwards
}
