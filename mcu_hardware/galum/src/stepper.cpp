#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

// -------- Hardware config --------
#define STEP_PIN 32
#define DIR_PIN  17
#define LED_PIN   2         // onboard LED (ESP32)
#define DIR_SETUP_US 10   // หน่วงสั้น ๆ หลังเปลี่ยนทิศ (5–20us ปลอดภัย)


// ---- Step timing (tune these) ----
#define MAX_ABS_speed          8000.0f   // max |steps/s|
#define MIN_STEP_INTERVAL_US     60     // clamp fastest to ~6.6 kHz
#define STEP_PULSE_US              8     // step HIGH width (>=2us for most drivers)

// -------- Helpers --------
#define RCCHECK(fn)  { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { rclErrorLoop(); } }
#define RCSOFTCHECK(fn) (void)(fn)
#define RCRET_IGNORE(expr) do { rcl_ret_t _rc = (expr); (void)_rc; } while (0)
#define EXECUTE_EVERY_N_MS(MS, X) do { static int64_t t=-1; if (t==-1) t = uxr_millis(); \
  if ((int32_t)(uxr_millis()-t) > (MS)) { X; t = uxr_millis(); } } while (0)

// -------- micro-ROS objects --------
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     control_timer;

rcl_subscription_t cmd_sub;
geometry_msgs__msg__Twist cmd_msg;

rcl_publisher_t status_pub;
geometry_msgs__msg__Twist status_msg;

// -------- Agent state --------
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

// -------- Stepper control state --------
volatile long     step_position    = 0;        // total steps since boot
volatile float    target_speed     = 0.0f;     // commanded speed (steps/s)
volatile uint32_t step_interval_us = 1000000;  // derived from target_speed
volatile bool     dir_clockwise    = true;
uint32_t          last_step_us     = 0;

// -------- Time sync (optional for stamped msgs) --------
unsigned long long time_offset = 0;

// ---------------- Prototypes ----------------
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void cmdCallback(const void *msgin);

// ---------------- Setup ---------------------
void setup() {
  // Serial transport to Pi 5 micro-ros-agent
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(LED_PIN,  OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN,  LOW);
  digitalWrite(LED_PIN,  LOW);
}

// ---------------- Loop ----------------------
void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      digitalWrite(LED_PIN, LOW);
      break;

    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroyEntities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED) {
        digitalWrite(LED_PIN, HIGH);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }

  // Non-blocking step generation (driven by micros)
  if (target_speed != 0.0f) {
    static bool prev_clockwise = true;   // << จำทิศครั้งก่อน
    uint32_t now = micros();

    if ((uint32_t)(now - last_step_us) >= step_interval_us) {

      bool clockwise = (target_speed > 0.0f);

      // ถ้า “เพิ่ง” เปลี่ยนทิศ: ตั้ง DIR แล้วรอ DIR_SETUP_US ก่อนยิงพัลส์แรก
      if (clockwise != prev_clockwise) {
        digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
        delayMicroseconds(DIR_SETUP_US);     // *** สำคัญ: setup time ของ DIR ***
        prev_clockwise = clockwise;
      } else {
        // ทิศเดิม: ตั้ง DIR ตามปกติ
        digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
      }

      // Emit one step pulse
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(STEP_PULSE_US);
      digitalWrite(STEP_PIN, LOW);

      // Update position
      step_position += clockwise ? 1 : -1;

      // เดิน clock ต่อแบบสะสม ลด jitter
      last_step_us += step_interval_us;
    }
  }
}

// --------------- micro-ROS wiring ---------------
bool createEntities() {
  allocator = rcl_get_default_allocator();

  // init messages
  geometry_msgs__msg__Twist__init(&cmd_msg);
  geometry_msgs__msg__Twist__init(&status_msg);

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  // ใส่ ROS_DOMAIN_ID ให้ตรงกับฝั่ง agent
  rcl_init_options_set_domain_id(&init_options, 96);

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  RCCHECK(rclc_node_init_default(&node, "stepper_node", "", &support));

  // Subscriber: /galum/stepper (Twist)
  RCCHECK(rclc_subscription_init_default(
    &cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/galum/stepper/angle"));

  // Publisher: /galum/stepper/status (Twist)
  // ใหม่ (Reliable)
  RCCHECK(rclc_publisher_init_default(
  &status_pub, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "/galum/stepper/status"));


  // Control timer @ 20 ms (50 Hz): publish status & house-keeping
  const unsigned int period_ms = 20;
  RCCHECK(rclc_timer_init_default(
    &control_timer, &support, RCL_MS_TO_NS(period_ms), controlCallback));

  // Executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmdCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  syncTime();
  return true;
}

bool destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  // ใช้ RCRET_IGNORE เพื่อตัด warning "ignoring return value"
  RCRET_IGNORE(rcl_timer_fini(&control_timer));
  RCRET_IGNORE(rcl_subscription_fini(&cmd_sub, &node));
  RCRET_IGNORE(rcl_publisher_fini(&status_pub, &node));
  RCRET_IGNORE(rclc_executor_fini(&executor));
  RCRET_IGNORE(rcl_node_fini(&node));
  RCRET_IGNORE(rclc_support_fini(&support));
  return true;
}

// -------- Callbacks --------
void cmdCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // ใช้ linear.x เป็นความเร็ว steps/s (signed)
  float speed = (float)msg->linear.x;

  // Limit
  if (speed >  MAX_ABS_speed) speed =  MAX_ABS_speed;
  if (speed < -MAX_ABS_speed) speed = -MAX_ABS_speed;

  target_speed  = speed;
  dir_clockwise = (target_speed >= 0.0f);

  if (target_speed == 0.0f) {
    step_interval_us = 1000000;
  } else {
    float interval_f = 1000000.0f / fabsf(target_speed);
    if (interval_f < MIN_STEP_INTERVAL_US) interval_f = MIN_STEP_INTERVAL_US;
    step_interval_us = (uint32_t)interval_f;
  }

  // เริ่มก้าวทันทีหลังได้รับคำสั่ง
  last_step_us = micros();

  // Debug สั้นๆ
  Serial.print("[CMD] speed="); Serial.print(target_speed);
  Serial.print(" dir="); Serial.print(dir_clockwise ? "CW" : "CCW");
  Serial.print(" us="); Serial.println(step_interval_us);
}

void controlCallback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
  // Publish status @ 50 Hz
  status_msg.linear.x  = (double)step_position;     // position (steps)
  status_msg.linear.y  = (double)target_speed;      // commanded speed
  status_msg.angular.z = (double)step_interval_us;  // effective us

  RCRET_IGNORE(rcl_publish(&status_pub, &status_msg, NULL));
}

// -------- Time sync (optional) --------
void syncTime() {
  RCSOFTCHECK(rmw_uros_sync_session(10));
  unsigned long now_ms = millis();
  unsigned long long ros_ms = rmw_uros_epoch_millis();
  time_offset = ros_ms - now_ms;
}

// -------- Fatal error handling --------
void rclErrorLoop() {
  pinMode(LED_PIN, OUTPUT);
  while (1) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}
