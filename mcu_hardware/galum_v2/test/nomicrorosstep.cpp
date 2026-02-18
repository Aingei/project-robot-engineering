#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

// === ใช้ GPIO driver ที่ไวกว่า digitalWrite ===
#include "driver/gpio.h"

// มิวเท็กซ์กัน race ระหว่าง loop() กับ callback
portMUX_TYPE step_mux = portMUX_INITIALIZER_UNLOCKED;

// -------- Hardware config --------
#define STEP_PIN 25
#define DIR_PIN  17
#define LED_PIN   2         // onboard LED (ESP32)
#define DIR_SETUP_US 20     // หน่วงสั้น ๆ หลังเปลี่ยนทิศ (5–20us ปลอดภัย)

// ---- Pulse mode params ----
#define PULSES_PER_CLICK       50      // กดครั้งเดียว = กี่พัลส์ (ดีฟอลต์)
#define STEP_RATE_SPS        1200.0f   // ยิงพัลส์กี่ steps/s
#define MIN_STEP_INTERVAL_US   60      // กันเร็วเกินไปสำหรับไดรเวอร์
#define STEP_PULSE_US           5      // ความกว้างพัลส์ HIGH

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

// -------- Stepper state (pulse-per-click mode) --------
volatile long     step_position     = 0;        // สะสมตำแหน่งทั้งหมด
volatile long     pending_steps     = 0;        // พัลส์ยกค้าง (signed)
volatile bool     current_dir_cw    = true;     // ทิศล่าสุดที่กำลังยิง
uint32_t          step_interval_us  = 500;      // คำนวณจาก STEP_RATE_SPS
uint32_t          last_step_us      = 0;

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

  float interval_f = 1000000.0f / STEP_RATE_SPS;
  if (interval_f < MIN_STEP_INTERVAL_US) interval_f = MIN_STEP_INTERVAL_US;
  step_interval_us = (uint32_t)interval_f;
  last_step_us = micros();
}

// ---------------- Loop ----------------------
void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      digitalWrite(LED_PIN, LOW);
      break;

    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroyEntities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1000,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED) {
        digitalWrite(LED_PIN, HIGH);
        // ลดเวลาบล็อก เพื่อไม่ให้ขาดพัลส์
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }

  // -------- Pulse-per-click stepping (non-blocking) --------
  // ใช้ micros() คุมเฟส และปิด race ด้วย critical section
  // อ่านค่า pending อย่างปลอดภัย
  long local_pending;
  portENTER_CRITICAL(&step_mux);
  local_pending = pending_steps;
  portEXIT_CRITICAL(&step_mux);

  if (local_pending != 0) {
    uint32_t now = micros();
    if ((uint32_t)(now - last_step_us) >= step_interval_us) {

      bool want_cw = (local_pending > 0);
      static bool prev_cw = true;

      // เปลี่ยนทิศ? ตั้ง DIR แล้วดีเลย์ก่อนพัลส์แรกของทิศใหม่
      if (want_cw != prev_cw) {
        gpio_set_level((gpio_num_t)DIR_PIN, want_cw ? 1 : 0);
        delayMicroseconds(DIR_SETUP_US + 200);   // กันเม็ดแรกพลาดหลังสลับทิศ
        prev_cw = want_cw;
        current_dir_cw = want_cw;
        last_step_us = micros();                 // บังคับรอ interval ใหม่ก่อนยิงพัลส์แรก
      } else {
        gpio_set_level((gpio_num_t)DIR_PIN, want_cw ? 1 : 0);
        current_dir_cw = want_cw;
      }

      // ยิงพัลส์ 1 ครั้ง (ใช้ gpio_set_level แทน digitalWrite เพื่อลด jitter)
      gpio_set_level((gpio_num_t)STEP_PIN, 1);
      ets_delay_us(STEP_PULSE_US);
      gpio_set_level((gpio_num_t)STEP_PIN, 0);

      // อัปเดตตัวนับอย่างอะตอมมิก
      portENTER_CRITICAL(&step_mux);
      step_position += current_dir_cw ? 1 : -1;
      pending_steps += current_dir_cw ? -1 : +1;
      local_pending  = pending_steps; // sync กับรอบถัดไป
      portEXIT_CRITICAL(&step_mux);

      // รีล็อกเฟสด้วยเวลา now ช่วยลดอาการรีบไล่เฟสจนกระตุก
      last_step_us = now;
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

  // Subscriber: /galum/stepper/angle (Twist)
  RCCHECK(rclc_subscription_init_default(
    &cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/galum/stepper/angle"));

  // Publisher: /galum/stepper/status (Twist) - reliable default
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

  // รีเซ็ตสถานะ
  portENTER_CRITICAL(&step_mux);
  pending_steps = 0;
  step_position = 0;
  portEXIT_CRITICAL(&step_mux);

  syncTime();
  return true;
}

bool destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

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
  const auto *msg = (const geometry_msgs__msg__Twist*) msgin;

  // ปุ่มหยุดฉุกเฉิน / เคลียร์คิว: ส่ง (0,0)
  if (msg->linear.x == 0.0 && msg->linear.y == 0.0) {
    portENTER_CRITICAL(&step_mux);
    pending_steps = 0;
    portEXIT_CRITICAL(&step_mux);
    Serial.println("[STOP] clear pending");
    return;
  }

  // รับทิศ: ต้องเป็นบวก/ลบเท่านั้น  (0 = ไม่ทำอะไร)
  if (msg->linear.x == 0.0) return;
  bool dir_cw = (msg->linear.x > 0.0);

  // จำนวนพัลส์: รับเฉพาะ >=20 เท่านั้น กันเม็ดเล็ก ๆ ที่ทำให้ทริกบ่อย
  long pulses = (long) llround(msg->linear.y);
  if (pulses < 20) return;

  portENTER_CRITICAL(&step_mux);
  if (dir_cw) pending_steps += pulses;
  else        pending_steps -= pulses;
  portEXIT_CRITICAL(&step_mux);

  // Serial.printf("[CLICK] dir=%s add=%ld pending=%ld\n",
  //               dir_cw ? "CW" : "CCW", pulses, pending_steps);
}

void controlCallback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
  // Publish status @ 50 Hz
  long pos, pend;
  portENTER_CRITICAL(&step_mux);
  pos  = step_position;
  pend = pending_steps;
  portEXIT_CRITICAL(&step_mux);

  status_msg.linear.x  = (double)pos;              // position (steps)
  status_msg.linear.y  = (double)pend;             // pending steps (signed)
  status_msg.angular.z = (double)step_interval_us; // effective us/step
  status_msg.angular.x = (double)555555555555;     // debug value as-is

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
