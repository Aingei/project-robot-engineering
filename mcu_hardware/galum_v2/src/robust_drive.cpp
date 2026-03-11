#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <ESP32Encoder.h>
#include "../config/galum_move.h"
#include "../config/drivemotor.h"

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

// ─────────────────────────────────────────────
//  Constants & Tuning
// ─────────────────────────────────────────────
static constexpr uint32_t AGENT_PING_INTERVAL_MS   = 500;   // how often to ping while waiting
static constexpr uint32_t AGENT_PING_TIMEOUT_MS     = 200;   // max wait for ping reply
static constexpr uint8_t  AGENT_PING_RETRIES        = 3;     // retries before declaring disconnected
static constexpr uint32_t CONTROL_PERIOD_MS         = 20;    // main control loop period
static constexpr uint32_t HEALTH_CHECK_INTERVAL_MS  = 200;   // liveness check period
static constexpr uint32_t CMD_TIMEOUT_MS            = 500;   // stop motors if no cmd received
static constexpr uint32_t RECONNECT_BACKOFF_MS      = 2000;  // wait before reattempting connect
static constexpr uint8_t  STATUS_LED_PIN            = 2;     // built-in LED for status

// Robot geometry
static constexpr float WHEEL_SEPARATION = 0.20f;
static constexpr float WHEEL_RADIUS     = 0.05f;
static constexpr float MAX_RPM          = 150.0f;

// ─────────────────────────────────────────────
//  Hardware objects
// ─────────────────────────────────────────────
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

ESP32Encoder encoderFL; // Front Left  (Motor A)
ESP32Encoder encoderFR; // Front Right (Motor B)
ESP32Encoder encoderRL; // Rear Left   (Motor C)
ESP32Encoder encoderRR; // Rear Right  (Motor D)

// ─────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────
#define RCCHECK(fn)       { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) { rclErrorLoop(); } }
#define RCSOFTCHECK(fn)   { rcl_ret_t _rc = (fn); (void)_rc; }

#define EXECUTE_EVERY_N_MS(MS, X) do {                              \
    static volatile int64_t _init = -1;                             \
    if (_init == -1) { _init = uxr_millis(); }                      \
    if ((uxr_millis() - _init) > (MS)) { X; _init = uxr_millis(); } \
} while (0)

// ─────────────────────────────────────────────
//  ROS2 objects
// ─────────────────────────────────────────────
rcl_publisher_t    debug_motor_publisher;
rcl_publisher_t    encoder_publisher;
rcl_subscription_t motor_subscriber;
rcl_timer_t        control_timer;

geometry_msgs__msg__Twist motor_msg;
geometry_msgs__msg__Twist debug_motor_msg;
geometry_msgs__msg__Twist encoder_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_init_options_t init_options;

// ─────────────────────────────────────────────
//  State & Timing
// ─────────────────────────────────────────────
enum class AgentState : uint8_t {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

AgentState         agentState       = AgentState::WAITING_AGENT;
unsigned long long time_offset      = 0;
unsigned long      prev_cmd_time    = 0;
unsigned long      last_reconnect_attempt = 0;
bool               entitiesCreated  = false;

// ─────────────────────────────────────────────
//  Function Prototypes
// ─────────────────────────────────────────────
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
void fullStop();
void moveMotors();
void statusLED(AgentState s);
struct timespec getTime();

// ─────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    Wire.begin(21, 22);

    // Encoders
    encoderFL.attachFullQuad(14, 27); encoderFL.clearCount();
    encoderFR.attachFullQuad(34, 35); encoderFR.clearCount();
    encoderRL.attachFullQuad(4, 21);  encoderRL.clearCount();
    encoderRR.attachFullQuad(36, 39); encoderRR.clearCount();

    // IMU
    bno.begin();
    bno.setExtCrystalUse(true);

    // Motor GPIO
    const uint8_t motorPins[] = {AIN1, AIN2, BIN1, BIN2, CIN1, CIN2, DIN1, DIN2};
    for (uint8_t pin : motorPins) { pinMode(pin, OUTPUT); }

    // PWM channels
    const uint8_t pwmChannels[] = {
        PWM_CHANNEL_AIN1, PWM_CHANNEL_AIN2,
        PWM_CHANNEL_BIN1, PWM_CHANNEL_BIN2,
        PWM_CHANNEL_CIN1, PWM_CHANNEL_CIN2,
        PWM_CHANNEL_DIN1, PWM_CHANNEL_DIN2
    };
    const uint8_t pwmPins[] = {AIN1, AIN2, BIN1, BIN2, CIN1, CIN2, DIN1, DIN2};
    for (int i = 0; i < 8; i++) {
        ledcSetup(pwmChannels[i], PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(pwmPins[i], pwmChannels[i]);
    }

    fullStop();
    agentState = AgentState::WAITING_AGENT;
}

// ─────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────
void loop()
{
    statusLED(agentState);

    switch (agentState)
    {
    // ── Waiting: ping every AGENT_PING_INTERVAL_MS ──────────────────────────
    case AgentState::WAITING_AGENT:
        EXECUTE_EVERY_N_MS(AGENT_PING_INTERVAL_MS, {
            bool agentFound = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_RETRIES));
            if (agentFound) {
                Serial.println("[microROS] Agent found → AGENT_AVAILABLE");
                agentState = AgentState::AGENT_AVAILABLE;
            }
        });
        break;

    // ── Available: try to create entities ───────────────────────────────────
    case AgentState::AGENT_AVAILABLE:
        if (createEntities()) {
            Serial.println("[microROS] Entities created → AGENT_CONNECTED");
            agentState = AgentState::AGENT_CONNECTED;
            prev_cmd_time = millis(); // reset command watchdog
        } else {
            Serial.println("[microROS] Failed to create entities → retry");
            destroyEntities();
            agentState = AgentState::WAITING_AGENT;
        }
        break;

    // ── Connected: spin executor, health-check, command watchdog ────────────
    case AgentState::AGENT_CONNECTED:

        // Command-loss watchdog: stop motors if no cmd received recently
        if ((millis() - prev_cmd_time) > CMD_TIMEOUT_MS) {
            fullStop();
        }

        // Periodic liveness check
        EXECUTE_EVERY_N_MS(HEALTH_CHECK_INTERVAL_MS, {
            bool alive = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_RETRIES));
            if (!alive) {
                Serial.println("[microROS] Agent lost → AGENT_DISCONNECTED");
                agentState = AgentState::AGENT_DISCONNECTED;
            }
        });

        if (agentState == AgentState::AGENT_CONNECTED) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;

    // ── Disconnected: clean up, wait backoff, retry ──────────────────────────
    case AgentState::AGENT_DISCONNECTED:
        fullStop(); // safety first
        destroyEntities();
        Serial.println("[microROS] Cleaned up, waiting before reconnect...");
        agentState = AgentState::WAITING_AGENT;
        // Brief blocking backoff to prevent rapid reconnect spam
        delay(RECONNECT_BACKOFF_MS);
        break;

    default:
        agentState = AgentState::WAITING_AGENT;
        break;
    }
}

// ─────────────────────────────────────────────
//  Callbacks
// ─────────────────────────────────────────────
void controlCallback(rcl_timer_t *timer, int64_t /*last_call_time*/)
{
    if (timer == NULL) { return; }
    moveMotors();
    publishData();
}

void twistCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
        reinterpret_cast<const geometry_msgs__msg__Twist *>(msgin);

    prev_cmd_time = millis(); // reset watchdog

    motor_msg.linear.x  = msg->linear.x;   // FL rpm
    motor_msg.linear.y  = msg->linear.y;   // FR rpm
    motor_msg.angular.x = msg->angular.x;  // RL rpm
    motor_msg.angular.y = msg->angular.y;  // RR rpm
}

// ─────────────────────────────────────────────
//  Entity management
// ─────────────────────────────────────────────
bool createEntities()
{
    if (entitiesCreated) { return true; }

    allocator = rcl_get_default_allocator();

    geometry_msgs__msg__Twist__init(&encoder_msg);
    geometry_msgs__msg__Twist__init(&debug_motor_msg);
    geometry_msgs__msg__Twist__init(&motor_msg);

    init_options = rcl_get_zero_initialized_init_options();
    if (RCL_RET_OK != rcl_init_options_init(&init_options, allocator))       { return false; }
    if (RCL_RET_OK != rcl_init_options_set_domain_id(&init_options, 96))     { return false; }
    if (RCL_RET_OK != rclc_support_init_with_options(&support, 0, NULL,
                                                      &init_options, &allocator)) { return false; }

    if (RCL_RET_OK != rclc_node_init_default(&node, "galum_run_node", "", &support))
        { return false; }

    if (RCL_RET_OK != rclc_publisher_init_best_effort(
            &debug_motor_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/galum/debug/cmd_move/rpm"))                                     { return false; }

    if (RCL_RET_OK != rclc_publisher_init_best_effort(
            &encoder_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/galum/encoder"))                                                { return false; }

    if (RCL_RET_OK != rclc_subscription_init_default(
            &motor_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/galum/cmd_move/rpm"))                                           { return false; }

    if (RCL_RET_OK != rclc_timer_init_default(
            &control_timer, &support,
            RCL_MS_TO_NS(CONTROL_PERIOD_MS), controlCallback))               { return false; }

    executor = rclc_executor_get_zero_initialized_executor();
    if (RCL_RET_OK != rclc_executor_init(&executor, &support.context, 5, &allocator))
        { return false; }
    if (RCL_RET_OK != rclc_executor_add_subscription(
            &executor, &motor_subscriber, &motor_msg,
            &twistCallback, ON_NEW_DATA))                                    { return false; }
    if (RCL_RET_OK != rclc_executor_add_timer(&executor, &control_timer))   { return false; }

    syncTime();
    entitiesCreated = true;
    return true;
}

bool destroyEntities()
{
    if (!entitiesCreated) { return true; }

    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

    RCSOFTCHECK(rcl_publisher_fini(&debug_motor_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&encoder_publisher,     &node));
    RCSOFTCHECK(rcl_subscription_fini(&motor_subscriber,  &node)); // was missing!
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
    RCSOFTCHECK(rcl_init_options_fini(&init_options));

    entitiesCreated = false;
    return true;
}

// ─────────────────────────────────────────────
//  Motor helpers
// ─────────────────────────────────────────────
void moveMotors()
{
    DriveMotor(motor_msg.linear.x,  MAX_RPM, PWM_CHANNEL_AIN1, PWM_CHANNEL_AIN2); // FL
    DriveMotor(motor_msg.linear.y,  MAX_RPM, PWM_CHANNEL_BIN1, PWM_CHANNEL_BIN2); // FR
    DriveMotor(motor_msg.angular.x, MAX_RPM, PWM_CHANNEL_CIN1, PWM_CHANNEL_CIN2); // RL
    DriveMotor(motor_msg.angular.y, MAX_RPM, PWM_CHANNEL_DIN1, PWM_CHANNEL_DIN2); // RR

    debug_motor_msg.linear.x  = constrain(motor_msg.linear.x,  -MAX_RPM, MAX_RPM);
    debug_motor_msg.linear.y  = constrain(motor_msg.linear.y,  -MAX_RPM, MAX_RPM);
    debug_motor_msg.angular.x = constrain(motor_msg.angular.x, -MAX_RPM, MAX_RPM);
    debug_motor_msg.angular.y = constrain(motor_msg.angular.y, -MAX_RPM, MAX_RPM);
}

void fullStop()
{
    motor_msg.linear.x  = 0;
    motor_msg.linear.y  = 0;
    motor_msg.angular.x = 0;
    motor_msg.angular.y = 0;

    DriveMotor(0, MAX_RPM, PWM_CHANNEL_AIN1, PWM_CHANNEL_AIN2);
    DriveMotor(0, MAX_RPM, PWM_CHANNEL_BIN1, PWM_CHANNEL_BIN2);
    DriveMotor(0, MAX_RPM, PWM_CHANNEL_CIN1, PWM_CHANNEL_CIN2);
    DriveMotor(0, MAX_RPM, PWM_CHANNEL_DIN1, PWM_CHANNEL_DIN2);
}

// ─────────────────────────────────────────────
//  Publish encoder + debug
// ─────────────────────────────────────────────
void publishData()
{
    encoder_msg.linear.x  = static_cast<double>(encoderFL.getCount());
    encoder_msg.linear.y  = static_cast<double>(encoderFR.getCount());
    encoder_msg.angular.x = static_cast<double>(encoderRL.getCount());
    encoder_msg.angular.y = static_cast<double>(encoderRR.getCount());

    RCSOFTCHECK(rcl_publish(&encoder_publisher,     &encoder_msg,     NULL));
    RCSOFTCHECK(rcl_publish(&debug_motor_publisher, &debug_motor_msg, NULL));
}

// ─────────────────────────────────────────────
//  Time sync
// ─────────────────────────────────────────────
void syncTime()
{
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    time_offset = rmw_uros_epoch_millis() - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec  = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000UL;
    return tp;
}

// ─────────────────────────────────────────────
//  Status LED: blink pattern reflects state
// ─────────────────────────────────────────────
void statusLED(AgentState s)
{
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    unsigned long period = 0;

    switch (s) {
        case AgentState::WAITING_AGENT:      period = 1000; break; // slow blink
        case AgentState::AGENT_AVAILABLE:    period = 200;  break; // fast blink
        case AgentState::AGENT_CONNECTED:    period = 0;    break; // solid on
        case AgentState::AGENT_DISCONNECTED: period = 100;  break; // very fast blink
    }

    if (period == 0) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        return;
    }

    if ((millis() - lastBlink) > period) {
        ledState = !ledState;
        digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
        lastBlink = millis();
    }
}

// ─────────────────────────────────────────────
//  Fatal error loop
// ─────────────────────────────────────────────
void rclErrorLoop()
{
    fullStop();
    while (true) {
        digitalWrite(STATUS_LED_PIN, HIGH); delay(50);
        digitalWrite(STATUS_LED_PIN, LOW);  delay(50);
    }
}