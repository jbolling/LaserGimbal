#include <Servo.h>

Servo x_servo;
Servo y_servo;

int x_pin = 3;
int y_pin = 5;

int pos;

void setup() {
  x_servo.attach(x_pin);
  y_servo.attach(y_pin);
  Serial.begin(115200);
  pos = 0;
}

void loop() {
  byte buf[3];
  /*
  //update servo values
  if(Serial.available() >= 3){
    Serial.readBytes(buf, 3);
    
    if(buf[0] != 0xFF){
      //packeting error
      Serial.print("Packeting error");
    }
    else{
      x_servo.write(( buf[1]);
      y_servo.write((int) buf[2]);
    }
  }
  */
  
  x_servo.write(pos);
  pos++;
  delay(40);
  
  
}
