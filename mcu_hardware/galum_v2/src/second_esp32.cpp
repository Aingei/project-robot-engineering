#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// ใช้ Float32MultiArray เพื่อส่งค่า Front/Rear พร้อมกัน
#include <std_msgs/msg/float32_multi_array.h>

// ─────────────────────────────────────────────
//  1. Constants & Configuration
// ─────────────────────────────────────────────
// Network / Agent Logic
static constexpr uint32_t AGENT_PING_INTERVAL_MS   = 500;
static constexpr uint32_t AGENT_PING_TIMEOUT_MS    = 200;
static constexpr uint8_t  AGENT_PING_RETRIES       = 3;
static constexpr uint32_t RECONNECT_BACKOFF_MS     = 2000;
static constexpr uint32_t HEALTH_CHECK_INTERVAL_MS = 200;

static constexpr uint8_t  STATUS_LED_PIN           = 2;

// Hardware Pins (ปรับตามที่ต่อจริง)
// ตัวหน้า (Front)
static constexpr uint8_t  TRIG_F                   = 16;
static constexpr uint8_t  ECHO_F                   = 17;
// ตัวหลัง (Rear)
static constexpr uint8_t  TRIG_R                   = 5;
static constexpr uint8_t  ECHO_R                   = 18;

// Sensor Physics
static constexpr float    SOUND_SPEED              = 0.0343f; // cm/us (343 m/s)
static constexpr uint32_t PUBLISH_PERIOD_MS        = 100;     // 10Hz
static constexpr float    MAX_RANGE_M              = 4.00f;   // 4m
static constexpr float    MIN_RANGE_M              = 0.02f;   // 2cm
static constexpr float    FOV_RAD                  = 0.26f;   // ~15 degrees

// คำนวณ Timeout อัตโนมัติ: (MaxDist * 2) / Speed
// 400cm * 2 / 0.0343 ~= 23323 us -> ปัดเป็น 24000
static constexpr unsigned long ECHO_TIMEOUT_US     = 24000; 

// ─────────────────────────────────────────────
//  2. Objects & Variables
// ─────────────────────────────────────────────
rcl_publisher_t      us_publisher;
rcl_timer_t          timer;
rcl_node_t           node;
rcl_allocator_t      allocator;
rclc_support_t       support;
rclc_executor_t      executor;
rcl_init_options_t   init_options;

std_msgs__msg__Float32MultiArray us_msg;
float us_data_buffer[2]; // Buffer เก็บค่า [Front, Rear]

// State Machine
enum class AgentState : uint8_t { 
    WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED 
};
AgentState agentState = AgentState::WAITING_AGENT;
bool entitiesCreated = false;

// ─────────────────────────────────────────────
//  3. Function Prototypes
// ─────────────────────────────────────────────
void rclErrorLoop();
bool createEntities();
bool destroyEntities();
void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
void statusLED(AgentState s);
float getDistance(uint8_t trig, uint8_t echo);

// Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = (fn); if((temp_rc != RCL_RET_OK)){ rclErrorLoop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = (fn); (void)temp_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t _init = -1; \
    if (_init == -1) { _init = uxr_millis(); } \
    if ((uxr_millis() - _init) > (MS)) { X; _init = uxr_millis(); } \
} while (0)


// ─────────────────────────────────────────────
//  4. Sensor Logic (Core Physics)
// ─────────────────────────────────────────────
float getDistance(uint8_t trig, uint8_t echo) {
    // 1. Trigger Pulse
    digitalWrite(trig, LOW); delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // 2. Measure Echo (with Physics Timeout)
    long duration = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);

    if (duration == 0) return 0.0f; // Timeout or Error

    // 3. Calculate Distance (Physics Formula)
    // Distance(cm) = Duration(us) * Speed(cm/us) / 2
    // Convert to Meters: / 100
    float dist_m = (duration * SOUND_SPEED) / 200.0f;

    // 4. Filter Range
    if (dist_m > MAX_RANGE_M || dist_m < MIN_RANGE_M) return 0.0f;

    return dist_m;
}

// ─────────────────────────────────────────────
//  5. ROS Callbacks & Entity Management
// ─────────────────────────────────────────────
void timerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;

    // อ่านค่า Sensor ทีละตัว (สำคัญมากต้องมี Delay กันคลื่นตีกัน)
    float dist_f = getDistance(TRIG_F, ECHO_F);
    delay(15); 
    float dist_r = getDistance(TRIG_R, ECHO_R);

    // แพ็คข้อมูลลง Array
    us_msg.data.data[0] = dist_f;
    us_msg.data.data[1] = dist_r;
    us_msg.data.size = 2;

    // ส่งข้อมูล
    RCSOFTCHECK(rcl_publish(&us_publisher, &us_msg, NULL));
}

bool createEntities() {
    if (entitiesCreated) return true;
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 96); // Matching Agent ID
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    rclc_node_init_default(&node, "galum_dual_us_node", "", &support);

    // Publisher Config (Topic: /galum/us_dual)
    rclc_publisher_init_best_effort(
        &us_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/galum/us_dual");

    // Timer Config
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), timerCallback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    // Initialize Array Memory
    us_msg.data.capacity = 2;
    us_msg.data.size = 2;
    us_msg.data.data = us_data_buffer;

    entitiesCreated = true;
    return true;
}

bool destroyEntities() {
    if (!entitiesCreated) return true;
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

    rcl_publisher_fini(&us_publisher, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    rcl_init_options_fini(&init_options);
    entitiesCreated = false;
    return true;
}

// ─────────────────────────────────────────────
//  6. Setup & Loop
// ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    pinMode(STATUS_LED_PIN, OUTPUT);
    
    // Config Pins
    pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
    pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
    
    digitalWrite(TRIG_F, LOW);
    digitalWrite(TRIG_R, LOW);
}

void loop() {
    statusLED(agentState);

    switch (agentState) {
        case AgentState::WAITING_AGENT:
            EXECUTE_EVERY_N_MS(AGENT_PING_INTERVAL_MS, {
                if (RMW_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_RETRIES)) {
                    agentState = AgentState::AGENT_AVAILABLE;
                }
            });
            break;
        case AgentState::AGENT_AVAILABLE:
            if (createEntities()) {
                agentState = AgentState::AGENT_CONNECTED;
            } else {
                destroyEntities();
                agentState = AgentState::WAITING_AGENT;
            }
            break;
        case AgentState::AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(HEALTH_CHECK_INTERVAL_MS, {
                if (RMW_RET_OK != rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_RETRIES)) {
                    agentState = AgentState::AGENT_DISCONNECTED;
                }
            });
            if (agentState == AgentState::AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AgentState::AGENT_DISCONNECTED:
            destroyEntities();
            delay(RECONNECT_BACKOFF_MS);
            agentState = AgentState::WAITING_AGENT;
            break;
    }
}

// ─────────────────────────────────────────────
//  7. Status LED & Error Loop
// ─────────────────────────────────────────────
void statusLED(AgentState s) {
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    unsigned long period = 0;

    switch (s) {
        case AgentState::WAITING_AGENT:      period = 1000; break; // Slow Blink
        case AgentState::AGENT_AVAILABLE:    period = 200;  break; // Fast Blink
        case AgentState::AGENT_CONNECTED:    period = 0;    break; // Solid ON
        case AgentState::AGENT_DISCONNECTED: period = 100;  break; // Very Fast Blink
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

void rclErrorLoop() {
    while (true) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        delay(100);
    }
}