#include <Arduino.h>
#include "MagneticEn.h"
#include <ESP32Servo.h>

#define SERVO_A_PIN 15
#define SERVO_B_PIN 16

MagneticEn mag;
Servo servo_wheel1;


float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.1;

float target_angle = 90.0;
float previous_error = 0;
float integral = 0;


// Normalize angle difference between -180 to 180
float getAngleError(float target, float current) {
    float error = target - current;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    return error;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.begin(21, 22);

    servo_wheel1.attach(SERVO_A_PIN);
    servo_wheel1.writeMicroseconds(1500);  // Neutral PWM

    delay(1000); // Allow system to stabilize
}

void loop() {
    
    float current_angle = mag.getAngle();
    float error = getAngleError(target_angle, current_angle);

    // PID terms
    integral += error;
    float derivative = error - previous_error;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    float min_pwm_offset = 60;
    float pwm_output;   
    
    if (abs(output) < min_pwm_offset && abs(error) > 1.0) {
        pwm_output = 1500 + (output > 0 ? min_pwm_offset + 30 : -min_pwm_offset);
    } else {
        pwm_output = 1500 + output;
    }
    pwm_output = constrain(pwm_output, 1000, 2000);

    servo_wheel1.writeMicroseconds(pwm_output);

    Serial.print("Target: "); Serial.print(target_angle);
    Serial.print(" | Current: "); Serial.print(current_angle);
    Serial.print(" | PWM: "); Serial.println(pwm_output);

    delay(20);  // Adjust PID loop rate
}