#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <ESP32Encoder.h>
#include <geometry_msgs/msg/twist.h>
#include <Adafruit_BNO055.h>

// Include your custom headers
#include "../config/galum_move.h"
#include "../config/drivemotor.h"

// -------- Macros --------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static uint32_t prev_ms = 0; \
  if (millis() - prev_ms >= MS) { \
    X; \
    prev_ms = millis(); \
  } \
} while (0)

// -------- Globals --------
enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

rcl_publisher_t debug_motor_publisher;
rcl_publisher_t encoder_publisher;
rcl_subscription_t motor_subscriber;
geometry_msgs__msg__Twist motor_msg;
geometry_msgs__msg__Twist encoder_msg;
geometry_msgs__msg__Twist debug_motor_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

// Hardware
ESP32Encoder encoderLeft, encoderRight;
float max_rpm = 150.0;
unsigned long last_msg_time = 0;

// -------- Functions --------

void fullStop() {
    DriveMotor(0, max_rpm, PWM_CHANNEL_AIN1, PWM_CHANNEL_AIN2);
    DriveMotor(0, max_rpm, PWM_CHANNEL_BIN1, PWM_CHANNEL_BIN2);
    DriveMotor(0, max_rpm, PWM_CHANNEL_CIN1, PWM_CHANNEL_CIN2);
    DriveMotor(0, max_rpm, PWM_CHANNEL_DIN1, PWM_CHANNEL_DIN2);
}

void twistCallback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    motor_msg = *msg; 
    last_msg_time = millis(); // Watchdog timestamp
}

bool createEntities() {
    allocator = rcl_get_default_allocator();

    // Init Support
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Init Node
    RCCHECK(rclc_node_init_default(&node, "galum_run_node", "", &support));

    // Init Publishers
    RCCHECK(rclc_publisher_init_best_effort(&debug_motor_publisher, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/galum/debug/cmd_move/rpm"));
    RCCHECK(rclc_publisher_init_best_effort(&encoder_publisher, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/galum/encoder"));

    // Init Subscriber
    RCCHECK(rclc_subscription_init_default(&motor_subscriber, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/galum/cmd_move/rpm"));

    // Init Timer (50Hz)
    RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(20), 
        [](rcl_timer_t *timer, int64_t last_call_time) {
            // Watchdog: If no message for 500ms, stop motors
            if(millis() - last_msg_time > 500) {
                motor_msg.linear.x = 0; motor_msg.linear.y = 0;
                motor_msg.angular.x = 0; motor_msg.angular.y = 0;
            }

            // Move Motors
            DriveMotor(motor_msg.linear.x, max_rpm, PWM_CHANNEL_AIN1, PWM_CHANNEL_AIN2);
            DriveMotor(motor_msg.linear.y, max_rpm, PWM_CHANNEL_BIN1, PWM_CHANNEL_BIN2);
            DriveMotor(motor_msg.angular.x, max_rpm, PWM_CHANNEL_CIN1, PWM_CHANNEL_CIN2);
            DriveMotor(motor_msg.angular.y, max_rpm, PWM_CHANNEL_DIN1, PWM_CHANNEL_DIN2);

            // Publish Feedback
            encoder_msg.linear.x = encoderLeft.getCount();
            encoder_msg.linear.y = encoderRight.getCount();
            rcl_publish(&encoder_publisher, &encoder_msg, NULL);
        }
    ));

    // Init Executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &twistCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    return true;
}

void destroyEntities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_motor_publisher, &node);
    rcl_publisher_fini(&encoder_publisher, &node);
    rcl_subscription_fini(&motor_subscriber, &node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_node_fini(&node);
    rclc_support_fini(&support);
    
    fullStop();
}

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    
    // Hardware Setup...
    Wire.begin(21,22);
    encoderLeft.attachFullQuad(17, 5);
    encoderRight.attachFullQuad(18, 19);
    
    // Motor Pin setup remains same...
    state = WAITING_AGENT;
}

void loop() {
    switch (state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, {
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
            });
            break;

        case AGENT_AVAILABLE:
            state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) destroyEntities();
            break;

        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, {
                if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
                    state = AGENT_DISCONNECTED;
                }
            });
            
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            }
            break;

        case AGENT_DISCONNECTED:
            destroyEntities();
            state = WAITING_AGENT;
            break;
    }
}