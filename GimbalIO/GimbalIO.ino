#include <Servo.h>

/*
 *NB - servo center is at 99, 107.
 */

const int X_SERVO_OFFSET =  9;
const int Y_SERVO_OFFSET = 17;

Servo x_servo;
Servo y_servo;

int x_pin = 3;
int y_pin = 5;
int led_pin = 13;

void setup() {
  x_servo.attach(x_pin);
  y_servo.attach(y_pin);
  
  Serial.begin(115200);
  
  pinMode(led_pin,OUTPUT);
  digitalWrite(led_pin, LOW);
}

void loop() {
  char terminal;
  long x_pos;
  long y_pos;
  
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
}
