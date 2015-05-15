#include <SoftwareSerial.h>
#include <Servo.h>

/*
 *NB - servo center is at 99, 107.
 */

const int X_SERVO_OFFSET =  9;
const int Y_SERVO_OFFSET = 17;

/////////////////////
// Pin Assignments //
/////////////////////
const int x_pin = 3;
const int y_pin = 5;
const int led_pin = 13;
const int laser_rx = 10;
const int laser_tx = 11;

Servo x_servo;
Servo y_servo;
SoftwareSerial laserSerial(laser_rx, laser_tx);

void setup() {
  x_servo.attach(x_pin);
  y_servo.attach(y_pin);
  
  Serial.begin(115200);
  laserSerial.begin(115200);
  
  pinMode(led_pin,OUTPUT);
  digitalWrite(led_pin, LOW);
  laserSerial.write("*00004#");
}

void loop() {
  char terminal;
  long x_pos;
  long y_pos;
  char buf[64];
  
  //update servo values
  if(Serial.available() >= 4){
    x_pos = Serial.parseInt() + X_SERVO_OFFSET;
    y_pos = Serial.parseInt() + Y_SERVO_OFFSET;
    terminal = Serial.read();
    
    if(terminal != ';'){
      //packeting error
      digitalWrite(led_pin, HIGH);
    }else{
      x_servo.write(x_pos);
      y_servo.write(y_pos);
    }
  }
  
  //Read Laser
  
  laserSerial.write("*00004#");
  delay(3000);
  /*
  laserSerial.readBytesUntil(
  delay(250);
  if(laserSerial.available()){
    Serial.write(laserSerial.read());
  }
  delay(1000);*/
}
