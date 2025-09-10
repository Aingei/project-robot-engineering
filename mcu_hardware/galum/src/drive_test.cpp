#include <Arduino.h>
#include <Wire.h>
// #include <cmath>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <ESP32Encoder.h>
#include "../config/galum_move.h"

#include <geometry_msgs/msg/twist.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

// -------- Hardware objects --------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

// -------- Robot geometry --------
float wheel_separation = 0.20; // meters between wheels
float wheel_radius = 0.05;     // meters radius of wheel
float max_rpm = 150.0;         // motor maximum RPM


// -------- Helpers --------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)


//------------------------------ < Define > -------------------------------------//

rcl_publisher_t debug_motor_publisher;
geometry_msgs__msg__Twist debug_motor_msg;

rcl_subscription_t motor_subscriber;
geometry_msgs__msg__Twist motor_msg;

rcl_publisher_t encoder_publisher;
geometry_msgs__msg__Twist encoder_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long current_time = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;


//------------------------------ < Fuction Prototype > ------------------------------//

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
struct timespec getTime();
void fullStop();

void Move();

//------------------------------ < Main > -------------------------------------//



void setup()
{   

    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    Wire.begin(21,22);

    encoderLeft.attachFullQuad(17, 5);
    encoderLeft.clearCount();
    encoderRight.attachFullQuad(18, 19);
    encoderRight.clearCount();

    bno.begin();

    bno.setExtCrystalUse(true);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(CIN1, OUTPUT);
    pinMode(CIN2, OUTPUT);
    pinMode(DIN1, OUTPUT);
    pinMode(DIN2, OUTPUT);

    ledcSetup(PWM_CHANNEL_AIN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_AIN2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_BIN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_BIN2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_CIN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_CIN2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_DIN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_DIN2, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(AIN1, PWM_CHANNEL_AIN1);
    ledcAttachPin(AIN2, PWM_CHANNEL_AIN2);
    ledcAttachPin(BIN1, PWM_CHANNEL_BIN1);
    ledcAttachPin(BIN2, PWM_CHANNEL_BIN2);
    ledcAttachPin(CIN1, PWM_CHANNEL_CIN1);
    ledcAttachPin(CIN2, PWM_CHANNEL_CIN2);
    ledcAttachPin(DIN1, PWM_CHANNEL_DIN1);
    ledcAttachPin(DIN2, PWM_CHANNEL_DIN2);

}

void loop()
{
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
    case AGENT_DISCONNECTED:
        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Fuction > -------------------------------------//


void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        Move();
        publishData();
        
    }
}

void twistCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
 
    prev_cmd_time = millis();

      // Store desired velocities from teleop
    motor_msg.linear.x  = msg->linear.x;   // Forward/backward (m/s)
    motor_msg.angular.z = msg->angular.z;  // Rotation (rad/s)
}


bool createEntities()
{
    allocator = rcl_get_default_allocator();
    geometry_msgs__msg__Twist__init(&encoder_msg);

    geometry_msgs__msg__Twist__init(&debug_motor_msg);

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 96);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);


    RCCHECK(rclc_node_init_default(&node, "galum_run_node", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_motor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/galum/debug/cmd_move/rpm"));

    RCCHECK(rclc_subscription_init_default(
        &motor_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/galum/cmd_move/rpm"));

    RCCHECK(rclc_publisher_init_best_effort(
        &encoder_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/galum/encoder"));

    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &motor_subscriber,
        &motor_msg,
        &twistCallback,
        ON_NEW_DATA));
        
    
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    syncTime();

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_motor_publisher, &node);
    rcl_publisher_fini(&encoder_publisher, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void Move() {
    float v = motor_msg.linear.x;      // m/s
    float omega = motor_msg.angular.z; // rad/s

    // คำนวณความเร็วล้อซ้าย/ขวา (Differential drive)
    float v_left  = v - (omega * wheel_separation / 2.0);
    float v_right = v + (omega * wheel_separation / 2.0);

    // แปลง linear velocity -> RPM
    float rpm_left  = (v_left  / (2 * M_PI * wheel_radius)) * 60.0;
    float rpm_right = (v_right / (2 * M_PI * wheel_radius)) * 60.0;

    // จำกัด RPM
    rpm_left  = constrain(rpm_left,  -max_rpm, max_rpm);
    rpm_right = constrain(rpm_right, -max_rpm, max_rpm);

    // duty cycle 0–255
    uint8_t duty_left  = (uint8_t)((fabs(rpm_left)  / max_rpm) * 255.0);
    uint8_t duty_right = (uint8_t)((fabs(rpm_right) / max_rpm) * 255.0);

    // ---------------- FRONT DRIVE ----------------
    // Left Front (Motor A)
    if (rpm_left > 0) {
        ledcWrite(PWM_CHANNEL_AIN1, duty_left);
        ledcWrite(PWM_CHANNEL_AIN2, 0);
    } else if (rpm_left < 0) {
        ledcWrite(PWM_CHANNEL_AIN1, 0);
        ledcWrite(PWM_CHANNEL_AIN2, duty_left);
    } else {
        ledcWrite(PWM_CHANNEL_AIN1, 0);
        ledcWrite(PWM_CHANNEL_AIN2, 0);
    }

    // Right Front (Motor B)
    if (rpm_right > 0) {
        ledcWrite(PWM_CHANNEL_BIN1, duty_right);
        ledcWrite(PWM_CHANNEL_BIN2, 0);
    } else if (rpm_right < 0) {
        ledcWrite(PWM_CHANNEL_BIN1, 0);
        ledcWrite(PWM_CHANNEL_BIN2, duty_right);
    } else {
        ledcWrite(PWM_CHANNEL_BIN1, 0);
        ledcWrite(PWM_CHANNEL_BIN2, 0);
    }

    // ---------------- BACK DRIVE ----------------
    // Left Back (Motor C)
    if (rpm_left > 0) {
        ledcWrite(PWM_CHANNEL_CIN1, duty_left);
        ledcWrite(PWM_CHANNEL_CIN2, 0);
    } else if (rpm_left < 0) {
        ledcWrite(PWM_CHANNEL_CIN1, 0);
        ledcWrite(PWM_CHANNEL_CIN2, duty_left);
    } else {
        ledcWrite(PWM_CHANNEL_CIN1, 0);
        ledcWrite(PWM_CHANNEL_CIN2, 0);
    }

    // Right Back (Motor D)
    if (rpm_right > 0) {
        ledcWrite(PWM_CHANNEL_DIN1, duty_right);
        ledcWrite(PWM_CHANNEL_DIN2, 0);
    } else if (rpm_right < 0) {
        ledcWrite(PWM_CHANNEL_DIN1, 0);
        ledcWrite(PWM_CHANNEL_DIN2, duty_right);
    } else {
        ledcWrite(PWM_CHANNEL_DIN1, 0);
        ledcWrite(PWM_CHANNEL_DIN2, 0);
    }

    // Debug (ส่ง duty ของซ้าย/ขวา)
    debug_motor_msg.linear.x = duty_left;
    debug_motor_msg.linear.y = duty_right;
}


void publishData()
{
    encoder_msg.linear.x = encoderLeft.getCount();
    encoder_msg.linear.y = encoderRight.getCount();
    rcl_publish(&encoder_publisher, &encoder_msg, NULL);
    
    Serial.print("Publishing debug_motor_msg: ");
    Serial.print(debug_motor_msg.linear.x);
    Serial.print(", ");
    Serial.println(debug_motor_msg.linear.y);
    
    rcl_ret_t ret = rcl_publish(&debug_motor_publisher, &debug_motor_msg, NULL);
    if (ret != RCL_RET_OK) {
        Serial.print("Publish failed with error: ");
        Serial.println(rcl_get_error_string().str);
        rcl_reset_error();
    }
}

void syncTime()
{

    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop()
{
    // Example implementation: blink LED rapidly then reboot or halt

    const int LED_PIN = 13;
    pinMode(LED_PIN, OUTPUT);

    while (1)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }

    // Or, if you want to reboot:
    // NVIC_SystemReset();  // if supported on your platform
}