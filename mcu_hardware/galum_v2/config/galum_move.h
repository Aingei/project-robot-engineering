#ifndef GALUM_MOVE_H
#define GALUM_MOVE_H

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1(A)  MOTOR2(B)  (2WD/ACKERMANN)
    MOTOR3(C)  MOTOR4(D)  (4WD/MECANUM)
         BACK
*/

#define AIN1 17 
#define AIN2 5
#define BIN1 32
#define BIN2 33
#define CIN1 18
#define CIN2 19
#define DIN1 34
#define DIN2 35

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  
    MOTOR3  MOTOR4  
         BACK
*/


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