#include <Arduino.h>
#include <ESP32Servo.h>

Servo servo;

 
void setup(){
  servo.attach(26);
  servo.setPeriodHertz(50);
}


void loop(){

    servo.write(0);
    delay(2000);
    servo.write(180);
    delay(2000);


}

