#ifndef GALUM_MOVE_H
#define GALUM_MOVE_H

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

#define AIN1 21
#define AIN2 19
#define BIN1 5
#define BIN2 16
#define CIN1 33
#define CIN2 32
#define DIN1 12
#define DIN2 14


#define PWM_FREQ 5000        // 5 kHz PWM frequency
#define PWM_RESOLUTION 8     // 8-bit resolution
#define PWM_CHANNEL_AIN1 0
#define PWM_CHANNEL_AIN2 1
#define PWM_CHANNEL_BIN1 2
#define PWM_CHANNEL_BIN2 3
#define PWM_CHANNEL_CIN1 4
#define PWM_CHANNEL_CIN2 5
#define PWM_CHANNEL_DIN1 6
#define PWM_CHANNEL_DIN2 7

#endif