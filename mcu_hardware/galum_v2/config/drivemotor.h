#include <Arduino.h>
#include <math.h>
#include <geometry_msgs/msg/twist.h>

void DriveMotor(float rpm, float max_rpm, uint8_t pinA, uint8_t pinB ) {
    // Clamp RPM
    rpm = constrain(rpm, -max_rpm, max_rpm);

    // แปลง rpm เป็น duty cycle
    uint8_t duty = fabs(rpm);

    // สั่งหมุนตามทิศทาง กลับมาดูดด้วยยยยยยยยยยยยย
    if (rpm > 0) {
        ledcWrite(pinB, duty);
        ledcWrite(pinA, 0);
    } else if (rpm < 0) {
        ledcWrite(pinB, 0);
        ledcWrite(pinA, duty);
    } else {
        ledcWrite(pinB, 0);
        ledcWrite(pinA, 0);
    }
}
