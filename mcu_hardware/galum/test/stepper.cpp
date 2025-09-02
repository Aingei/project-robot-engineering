#include <Arduino.h>

// กำหนดขาของ Stepper Motor (มอเตอร์เดียว)
#define STEP_PIN 32
#define DIR_PIN  25

void moveMotor(bool clockwise, int steps)
{
    digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
    for (int i = 0; i < steps; i++)
    {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500);
    }
}

void setup()
{
    Serial.begin(9600);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    delay(1000);
    Serial.println("เริ่มทดสอบ Stepper Motor 1 ตัว...");
}

void loop()
{
    // หมุนไปข้างหน้า 200 steps
    moveMotor(true, 200);
    delay(1000);

    // หมุนกลับหลัง 200 steps
    moveMotor(false, 200);
    delay(1000);
}
