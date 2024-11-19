#include <Servo.h>

Servo esc;  // Create a Servo object to control the ESC

int Throttle = 1100;        
int specificDutyCycle = 650;  

void setup() {
  Serial.begin(115200); 
  esc.attach(11);      
}

void loop() {

  esc.writeMicroseconds(Throttle);
  delay(500); 
  esc.writeMicroseconds(0);
  delay(200); 
  Throttle = Throttle + 10;
  Serial.println(Throttle);

  //1380
  //1650
}
