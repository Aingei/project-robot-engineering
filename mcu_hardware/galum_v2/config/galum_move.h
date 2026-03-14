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
#define AIN2 16
#define BIN1 25
#define BIN2 26
#define CIN1 18
#define CIN2 19
#define DIN1 32
#define DIN2 33


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

// ---------------- Pins & Config ----------------
#define STEP_PIN 25
#define DIR_PIN  32
#define LED_PIN   2

// Servos
#define SERVO1_PIN 26 //พับแขน180
#define SERVO2_PIN 27 //เปิดปิดก้านบน360 
#define SERVO3_PIN 14 //หมุนปลูก180
#define SERVO4_PIN 19 //เจาะดิน360

// Ultrasonics
#define TRIG_F 16  
#define ECHO_F 17  // Changed from 17 to avoid conflict with DIR_PIN
#define TRIG_R 5
#define ECHO_R 18

// Physics & Timing
#define DIR_SETUP_US            40
#define STEP_RATE_SPS           1200.0f
#define MIN_STEP_INTERVAL_US    60 
#define STEP_PULSE_US           8
#define MIN_PULSES_PER_CMD      10
#define ACCEL_STEPS_PER_S2      2000.0f
#define JERK_FILTER_US          50

#define SOUND_SPEED             0.0343f
#define US_PUBLISH_PERIOD_MS    100
#define MAX_RANGE_M             4.00f
#define MIN_RANGE_M             0.02f
#define ECHO_TIMEOUT_US         24000


#endif