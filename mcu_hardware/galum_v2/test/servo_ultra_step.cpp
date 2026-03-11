// ============================================================================
// ESP32 micro-ROS: 1 Stepper, 4 Servos, 2 Ultrasonics (Integrated Version)
// ============================================================================

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "driver/gpio.h"
#include "esp32-hal-timer.h"
#include <ESP32Servo.h>
#include "../config/galum_move.h" // Ensure this path matches your project structure

// ------------- Macros -------------
#define RCCHECK(fn)  { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { rclErrorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = (fn); (void)temp_rc; }
#define RCRET_IGNORE(expr) do { rcl_ret_t _rc = (expr); (void)_rc; } while (0)
#define EXECUTE_EVERY_N_MS(MS, X) do { static int64_t t=-1; if (t==-1) t = uxr_millis(); \
  if ((int32_t)(uxr_millis()-t) > (MS)) { X; t = uxr_millis(); } } while (0)

// ---------------- micro-ROS objects ----------------
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

rcl_timer_t     control_timer; 
rcl_timer_t     us_timer;      

// Stepper
rcl_subscription_t cmd_sub;
geometry_msgs__msg__Twist cmd_msg;
rcl_publisher_t status_pub;
geometry_msgs__msg__Twist status_msg;

// Servo & Debug Publisher
rcl_subscription_t servo_sub;
geometry_msgs__msg__Twist servo_msg;
rcl_publisher_t servo_status_pub;
geometry_msgs__msg__Twist servo_status_msg;

// Ultrasonic
rcl_publisher_t us_publisher;
std_msgs__msg__Float32MultiArray us_msg;
float us_data_buffer[2];

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

Servo servo1, servo2, servo3, servo4;

// Stepper variables
volatile long step_position = 0;
volatile long pending_steps = 0;
volatile bool current_dir_cw = true;
volatile bool continuous_mode = false;
portMUX_TYPE step_mux = portMUX_INITIALIZER_UNLOCKED;

volatile long     v_pending_steps_isr = 0;
volatile bool     v_dir_cw_isr        = true;
volatile bool     step_high_phase     = false;
volatile uint32_t current_interval_us = 800;
volatile uint32_t target_interval_us  = 800;
volatile uint32_t min_interval_us     = MIN_STEP_INTERVAL_US;

hw_timer_t* stepTimer = nullptr;
portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- Prototypes ----------------
void rclErrorLoop();
bool createEntities();
bool destroyEntities();
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void usTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
void cmdCallback(const void *msgin);
void servoCallback(const void *msgin);

// ---------------- Hardware Logic ----------------

static inline uint32_t accel_to_dInterval_us(float accel_sps2, uint32_t) { return JERK_FILTER_US; }

void updateTargetIntervalFromRate(float rate_sps) {
  float tf = 1000000.0f / rate_sps;
  if (tf < (float)min_interval_us) tf = (float)min_interval_us;
  uint32_t t = (uint32_t)tf;
  portENTER_CRITICAL(&isrMux); target_interval_us = t; portEXIT_CRITICAL(&isrMux);
}

void queueStepsFromApp(long add_steps, bool dir_cw) {
  portENTER_CRITICAL(&isrMux);
  v_dir_cw_isr = dir_cw; v_pending_steps_isr += dir_cw ? add_steps : -add_steps;
  portEXIT_CRITICAL(&isrMux);

  portENTER_CRITICAL(&step_mux);
  pending_steps += dir_cw ? add_steps : -add_steps; current_dir_cw = dir_cw;
  portEXIT_CRITICAL(&step_mux);
}

float getDistance(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW); delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    
    long duration = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
    if (duration == 0) return 0.0f;
    
    float dist_m = (duration * SOUND_SPEED) / 200.0f;
    if (dist_m > MAX_RANGE_M || dist_m < MIN_RANGE_M) return 0.0f;
    return dist_m;
}

void IRAM_ATTR onStepTimer() {
  if (!step_high_phase) {
      bool idle_now = (!continuous_mode && (v_pending_steps_isr == 0));
      if (idle_now) {
        gpio_set_level((gpio_num_t)STEP_PIN, 0);
        timerAlarmWrite(stepTimer, 1000, true);
        return;
      }
    gpio_set_level((gpio_num_t)DIR_PIN, v_dir_cw_isr ? 1 : 0);
    gpio_set_level((gpio_num_t)STEP_PIN, 1);
    step_high_phase = true;
    timerAlarmWrite(stepTimer, STEP_PULSE_US, true);
  } else {
    gpio_set_level((gpio_num_t)STEP_PIN, 0);
    step_high_phase = false;

    portENTER_CRITICAL_ISR(&step_mux);
    step_position += v_dir_cw_isr ? 1 : -1;
    if (!continuous_mode) { pending_steps += v_dir_cw_isr ? -1 : +1; }
    current_dir_cw = v_dir_cw_isr;
    portEXIT_CRITICAL_ISR(&step_mux);

    if (!continuous_mode) { v_pending_steps_isr += v_dir_cw_isr ? -1 : +1; } 

    uint32_t cur = current_interval_us;
    uint32_t tgt = target_interval_us;
    if (cur != tgt) {
      uint32_t d = accel_to_dInterval_us(ACCEL_STEPS_PER_S2, cur);
      if (cur > tgt) cur = (cur - d > tgt) ? (cur - d) : tgt;
      else cur = (cur + d < tgt) ? (cur + d) : tgt;
      current_interval_us = (cur < min_interval_us) ? min_interval_us : cur;
    }
    uint32_t low_us = (current_interval_us > STEP_PULSE_US) ? (current_interval_us - STEP_PULSE_US) : min_interval_us;
    timerAlarmWrite(stepTimer, low_us, true); 
  }
}

// ---------------- Callbacks ----------------

void cmdCallback(const void *msgin) {
  const auto *msg = (const geometry_msgs__msg__Twist*) msgin;
  if (msg->linear.x == 0.0 && msg->linear.y == 0.0) {
    portENTER_CRITICAL(&isrMux); v_pending_steps_isr = 0; continuous_mode = false; portEXIT_CRITICAL(&isrMux);
    portENTER_CRITICAL(&step_mux); pending_steps = 0; portEXIT_CRITICAL(&step_mux);
    return;
  }
  if (msg->linear.x != 0.0) {
    bool dir_cw = (msg->linear.x > 0.0);
    portENTER_CRITICAL(&isrMux); v_dir_cw_isr = dir_cw; portEXIT_CRITICAL(&isrMux);
    portENTER_CRITICAL(&step_mux); current_dir_cw = dir_cw; portEXIT_CRITICAL(&step_mux);
  }
  long pulses = (long) llround(msg->linear.y);
  if (pulses >= MIN_PULSES_PER_CMD) {
    portENTER_CRITICAL(&isrMux); continuous_mode = false; portEXIT_CRITICAL(&isrMux);
    queueStepsFromApp(pulses, (msg->linear.x >= 0.0));
  } else if (msg->linear.y == 0.0 && msg->angular.x > 0.0) {
    updateTargetIntervalFromRate((float)msg->angular.x);
    portENTER_CRITICAL(&isrMux); continuous_mode = true; portEXIT_CRITICAL(&isrMux);
  }
  if (msg->angular.x > 0.0) updateTargetIntervalFromRate((float)msg->angular.x);
}

void servoCallback(const void *msgin) {
  const auto *m = (const geometry_msgs__msg__Twist*) msgin;
  double deg1 = max(0.0, min(180.0, m->linear.x));
  double deg2 = max(0.0, min(180.0, m->linear.y));
  double deg3 = max(0.0, min(180.0, m->linear.z));
  double deg4 = max(0.0, min(180.0, m->angular.x)); // 4th servo mapped to angular.x
  
  servo1.write((int)llround(deg1));
  servo2.write((int)llround(deg2));
  servo3.write((int)llround(deg3));
  servo4.write((int)llround(deg4));
}

void controlCallback(rcl_timer_t *, int64_t) {
  // Stepper Status
  long pos, pend; uint32_t us_per_step;
  portENTER_CRITICAL(&step_mux); pos = step_position; pend = pending_steps; portEXIT_CRITICAL(&step_mux);
  portENTER_CRITICAL(&isrMux); us_per_step = current_interval_us; portEXIT_CRITICAL(&isrMux);

  status_msg.linear.x = (double)pos;
  status_msg.linear.y = (double)pend;
  status_msg.angular.z = (double)us_per_step;
  RCRET_IGNORE(rcl_publish(&status_pub, &status_msg, NULL));

  // Servo Status Debug
  servo_status_msg.linear.x = (double)servo1.read();
  servo_status_msg.linear.y = (double)servo2.read();
  servo_status_msg.linear.z = (double)servo3.read();
  servo_status_msg.angular.x = (double)servo4.read(); 
  RCSOFTCHECK(rcl_publish(&servo_status_pub, &servo_status_msg, NULL));
}

void usTimerCallback(rcl_timer_t *, int64_t) {
    // อ่านค่า Sensor ทีละตัว (สำคัญมากต้องมี Delay กันคลื่นตีกัน)
    float dist_f = getDistance(TRIG_F, ECHO_F); 
    delay(15); 
    float dist_r = getDistance(TRIG_R, ECHO_R);

    // แพ็คข้อมูลลง Array
    us_msg.data.data[0] = dist_f; 
    us_msg.data.data[1] = dist_r; 
    us_msg.data.size = 2;
    
    // ส่งข้อมูลแบบ Best Effort
    RCSOFTCHECK(rcl_publish(&us_publisher, &us_msg, NULL));
}

// ---------------- Setup & Loop ----------------
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // Initialize all 4 servos
  servo1.attach(SERVO1_PIN); servo1.setPeriodHertz(50); servo1.write(90);
  servo2.attach(SERVO2_PIN); servo2.setPeriodHertz(50); servo2.write(90);
  servo3.attach(SERVO3_PIN); servo3.setPeriodHertz(50); servo3.write(90);
  servo4.attach(SERVO4_PIN); servo4.setPeriodHertz(50); servo4.write(90);

  pinMode(STEP_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT); pinMode(LED_PIN, OUTPUT);
  gpio_set_level((gpio_num_t)STEP_PIN, 0); gpio_set_level((gpio_num_t)DIR_PIN, 0);

  // Initialize Ultrasonic Pins
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT); digitalWrite(TRIG_F, LOW);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT); digitalWrite(TRIG_R, LOW);

  updateTargetIntervalFromRate(STEP_RATE_SPS); current_interval_us = target_interval_us;
  stepTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(stepTimer, &onStepTimer, true);
  timerAlarmWrite(stepTimer, current_interval_us, true);
  timerAlarmEnable(stepTimer);
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      digitalWrite(LED_PIN, LOW); break;
    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroyEntities(); break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) { digitalWrite(LED_PIN, HIGH); rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)); }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities(); state = WAITING_AGENT; break;
  }
}

// ---------------- Entity Mgt ----------------
bool createEntities() {
  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rcl_init_options_set_domain_id(&init_options, 96);

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "actuator_node", "", &support));

  RCCHECK(rclc_subscription_init_default(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/galum/stepper/angle"));
  RCCHECK(rclc_subscription_init_default(&servo_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/galum/servo/angle"));
  
  RCCHECK(rclc_publisher_init_default(&status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/galum/stepper/status"));
  RCCHECK(rclc_publisher_init_default(&servo_status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/galum/servo/status"));
  
  // Publisher Config (Topic: /galum/us_dual)
  RCCHECK(rclc_publisher_init_best_effort(&us_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/galum/us_dual"));

  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(20), controlCallback));
  
  // Timer Config สำหรับ Ultrasonic
  RCCHECK(rclc_timer_init_default(&us_timer, &support, RCL_MS_TO_NS(US_PUBLISH_PERIOD_MS), usTimerCallback));

  // Initialize Array Memory
  us_msg.data.capacity = 2; us_msg.data.size = 2; us_msg.data.data = us_data_buffer;

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator)); 
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmdCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servoCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &us_timer));

  return true;
}

bool destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
  RCRET_IGNORE(rcl_timer_fini(&control_timer)); RCRET_IGNORE(rcl_timer_fini(&us_timer));
  RCRET_IGNORE(rcl_subscription_fini(&cmd_sub, &node)); RCRET_IGNORE(rcl_subscription_fini(&servo_sub, &node));
  RCRET_IGNORE(rcl_publisher_fini(&status_pub, &node)); RCRET_IGNORE(rcl_publisher_fini(&servo_status_pub, &node));
  RCRET_IGNORE(rcl_publisher_fini(&us_publisher, &node));
  RCRET_IGNORE(rclc_executor_fini(&executor)); RCRET_IGNORE(rcl_node_fini(&node));
  RCRET_IGNORE(rclc_support_fini(&support));
  return true;
}

void rclErrorLoop() {
  pinMode(LED_PIN, OUTPUT);
  while (1) { digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW); delay(100); }
}