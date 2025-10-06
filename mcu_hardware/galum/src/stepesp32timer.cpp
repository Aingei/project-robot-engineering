// ============================================================================
// ESP32 + DRV8825 micro-ROS Stepper (Single File)
// - ใช้ Hardware Timer ISR ยิงพัลส์ (เนียน/คงที่กว่า loop+micros())
// - มี ramp ไล่ความเร็ว (current_interval_us -> target_interval_us) แบบนุ่ม ๆ
// - รับคำสั่งเป็น "คลิก" (จำนวนพัลส์ + ทิศ) ผ่าน geometry_msgs/Twist
//   * linear.x > 0 => CW, < 0 => CCW, =0 => ไม่เปลี่ยน
//   * linear.y = จำนวนพัลส์ (แนะนำ >= 10)
//   * (ทางเลือก) angular.x = target speed (steps/s)
// - ส่งสถานะที่ /galum/stepper/status  (position, pending, us/step)
// ============================================================================

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
// === Hardware timer (ESP32) ===
#include "esp32-hal-timer.h"
#include <ESP32Servo.h>   // แทน <Servo.h>

// ---------------- User Config ----------------
#define STEP_PIN 25
#define DIR_PIN  17
#define LED_PIN   2
#define SERVO_PIN 26

// ดีเลย์หลังสลับทิศ (us)
#define DIR_SETUP_US            40

// ความเร็วมาตรฐาน (steps/s) เมื่อไม่ได้สั่งผ่าน msg.angular.x
#define STEP_RATE_SPS        1200.0f

// ค่าต่ำสุดไมโครสเต็ปกันมันเร็วเกินไป อย่าลืมว่ายังต้องคูณ/หารด้วยไมโครสเต็ปและสเปกมอเตอร์
#define MIN_STEP_INTERVAL_US    60 

// ความกว้างพัลส์ HIGH (us)
#define STEP_PULSE_US            8 //(รอบ/นาที = รอบ/วินาที × 60)

// เกณฑ์ขั้นต่ำของจำนวนพัลส์ที่ยอมรับต่อคลิก (กันสั่งถี่ ๆ เม็ดเล็ก ๆ )
#define MIN_PULSES_PER_CMD      10

// อัตราเร่ง (ใช้เป็น limiter เปลี่ยน interval ทีละก้าวเล็ก ๆ )
#define ACCEL_STEPS_PER_S2    2000.0f
#define JERK_FILTER_US           50    // จำกัดการเปลี่ยน interval ต่อรอบ (us)

// ------------- Helpers / Macros -------------
#define RCCHECK(fn)  { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { rclErrorLoop(); } }
#define RCSOFTCHECK(fn) (void)(fn)
#define RCRET_IGNORE(expr) do { rcl_ret_t _rc = (expr); (void)_rc; } while (0)
#define EXECUTE_EVERY_N_MS(MS, X) do { static int64_t t=-1; if (t==-1) t = uxr_millis(); \
  if ((int32_t)(uxr_millis()-t) > (MS)) { X; t = uxr_millis(); } } while (0)

// ---------------- micro-ROS objects ----------------
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     control_timer;

rcl_subscription_t cmd_sub;     // /galum/stepper/angle
geometry_msgs__msg__Twist cmd_msg;

rcl_publisher_t status_pub;     // /galum/stepper/status
geometry_msgs__msg__Twist status_msg;

rcl_subscription_t servo_sub;                     // <— เพิ่ม
geometry_msgs__msg__Twist servo_msg;

// ---------------- Agent state machine ----------------
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

Servo servo; //

// ---------------- Stepper shared state โชว์สถานะ ----------------
volatile long step_position = 0;     // ตำแหน่งรวม (นับเป็นสเต็ป)
volatile long pending_steps = 0;     // งานค้าง (signed)
volatile bool current_dir_cw = true;

portMUX_TYPE step_mux = portMUX_INITIALIZER_UNLOCKED; // กัน race (main <-> ISR/Callbacks)

// ---------------- Time sync (optional) ----------------
unsigned long long time_offset = 0;

// ---------------- Hardware Timer ISR state ยิงพัลล์จริง ----------------
volatile long     v_pending_steps_isr = 0;
volatile bool     v_dir_cw_isr        = true;
volatile bool     step_high_phase     = false;

volatile uint32_t current_interval_us = 800;  // ไล่เข้า target
volatile uint32_t target_interval_us  = 800;  // มาจาก STEP_RATE_SPS หรือคำสั่ง
volatile uint32_t min_interval_us     = MIN_STEP_INTERVAL_US;

hw_timer_t* stepTimer = nullptr; //มิวเทคโครงสร้างข้อมูลหรืออ็อบเจกต์ที่ทำหน้าที่เป็น กลไกควบคุมการเข้าถึงทรัพยากรที่ใช้ร่วมกัน กัน race ตอน main/ISR อัปเดต position/pending
portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED; //กัน race ตอน main ปรับค่าที่ ISR ใช้ (เช่น target interval, ทิศ)

// ---------------- Prototypes ----------------
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void cmdCallback(const void *msgin);
void servoCallback(const void *msgin);

static inline uint32_t accel_to_dInterval_us(float accel_sps2, uint32_t /*interval_us*/) {
  (void)accel_sps2;
  return JERK_FILTER_US; // limiter ง่าย ๆ ให้เนียน
}

void updateTargetIntervalFromRate(float rate_sps) {
  float tf = 1000000.0f / rate_sps;
  if (tf < (float)min_interval_us) tf = (float)min_interval_us;
  uint32_t t = (uint32_t)tf;
  portENTER_CRITICAL(&isrMux);
  target_interval_us = t;
  portEXIT_CRITICAL(&isrMux);
}

void queueStepsFromApp(long add_steps, bool dir_cw) {
  portENTER_CRITICAL(&isrMux);
  v_dir_cw_isr = dir_cw;
  v_pending_steps_isr += dir_cw ? add_steps : -add_steps;
  portEXIT_CRITICAL(&isrMux);

  // sync กับตัวนับหลัก (ไว้โชว์สถานะให้ทันใจ)
  portENTER_CRITICAL(&step_mux);
  pending_steps += dir_cw ? add_steps : -add_steps;
  current_dir_cw = dir_cw;
  portEXIT_CRITICAL(&step_mux);
}

// ---------------- ISR: ยิงพัลส์คุมเวลา ----------------
void IRAM_ATTR onStepTimer() {
  // สองเฟส: HIGH ช่วง STEP_PULSE_US, LOW ช่วง (interval - STEP_PULSE_US)
  if (!step_high_phase) {
    // ก่อนยกขอบขึ้น ถ้าไม่มีงานก็พัก
    if (v_pending_steps_isr == 0) {
      gpio_set_level((gpio_num_t)STEP_PIN, 0);
      timerAlarmWrite(stepTimer, 1000, true); // idle 1ms
      return;
    }

    // ตั้งทิศ ขณะ STEP=LOW
    gpio_set_level((gpio_num_t)DIR_PIN, v_dir_cw_isr ? 1 : 0);

    // ขอบขึ้น
    gpio_set_level((gpio_num_t)STEP_PIN, 1);
    step_high_phase = true;

    // กว้างพัลส์ HIGH
    timerAlarmWrite(stepTimer, STEP_PULSE_US, true);
  } else {
    // ขอบลง: นับหนึ่งสเต็ปที่ขอบลง
    gpio_set_level((gpio_num_t)STEP_PIN, 0);
    step_high_phase = false;

    // อัปเดตตำแหน่ง/คิวฝั่ง main อย่างปลอดภัย
    portENTER_CRITICAL_ISR(&step_mux);
    step_position += v_dir_cw_isr ? 1 : -1;
    pending_steps  += v_dir_cw_isr ? -1 : +1;
    current_dir_cw  = v_dir_cw_isr;
    portEXIT_CRITICAL_ISR(&step_mux);

    // ลดงานคิวใน ISR ด้วย
    v_pending_steps_isr += v_dir_cw_isr ? -1 : +1;

    // ไล่ interval เข้า target ให้เนียน
    uint32_t cur = current_interval_us;
    uint32_t tgt = target_interval_us;
    if (cur != tgt) {
      if (cur > tgt) {
        uint32_t d = accel_to_dInterval_us(ACCEL_STEPS_PER_S2, cur);
        cur = (cur - d > tgt) ? (cur - d) : tgt;
      } else {
        uint32_t d = accel_to_dInterval_us(ACCEL_STEPS_PER_S2, cur);
        cur = (cur + d < tgt) ? (cur + d) : tgt;
      }
      current_interval_us = (cur < min_interval_us) ? min_interval_us : cur;
    }

    // LOW phase = interval - STEP_PULSE_US
    uint32_t low_us = (current_interval_us > STEP_PULSE_US)
                        ? (current_interval_us - STEP_PULSE_US) //คาบ LOW phase
                        : min_interval_us;
    timerAlarmWrite(stepTimer, low_us, true); 
    //หนึ่งรอบ (HIGH+LOW)” รวมเท่ากับ current_interval_us
  }
}

// ---------------- Setup ----------------
void setup() {
  // micro-ROS transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  servo.attach(SERVO_PIN);
  servo.setPeriodHertz(50);
  servo.write(90);
  // ถ้าต้องการคาลิเบรตพัลส์: servo.attach(SERVO_PIN, 500, 2400);

  // IO init
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(LED_PIN,  OUTPUT);
  gpio_set_level((gpio_num_t)STEP_PIN, 0);
  gpio_set_level((gpio_num_t)DIR_PIN,  0);
  digitalWrite(LED_PIN, LOW);

  // ตั้ง target interval จากความเร็วเริ่มต้น
  updateTargetIntervalFromRate(STEP_RATE_SPS);
  current_interval_us = target_interval_us;

  // ตั้ง Hardware Timer: 80 MHz / 80 = 1 MHz (1 tick = 1us ไมโคร)
  stepTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(stepTimer, &onStepTimer, true);
  timerAlarmWrite(stepTimer, current_interval_us, true); //set time start
  timerAlarmEnable(stepTimer);
}

// ---------------- Loop ----------------
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
        // หมุน executor แบบเบามือเพื่อไม่ไปรบกวน ISR
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }
}

// ---------------- micro-ROS wiring ----------------
bool createEntities() {
  allocator = rcl_get_default_allocator();

  geometry_msgs__msg__Twist__init(&cmd_msg);
  geometry_msgs__msg__Twist__init(&status_msg);
  geometry_msgs__msg__Twist__init(&servo_msg); 

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  // ตั้ง ROS_DOMAIN_ID ให้ตรงกับ agent ของคุณ
  rcl_init_options_set_domain_id(&init_options, 96);

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "stepper_node", "", &support));

  // Subscriber: รับคำสั่ง "คลิก" ที่ /galum/stepper/angle
  RCCHECK(rclc_subscription_init_default(
    &cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/galum/stepper/angle"));

  // Publisher: ส่งสถานะที่ /galum/stepper/status
  RCCHECK(rclc_publisher_init_default(
    &status_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/galum/stepper/status"));
  
  RCCHECK(rclc_subscription_init_default(
    &servo_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/galum/servo/angle"));

  // Control timer @ 50 Hz
  const unsigned int period_ms = 20;
  RCCHECK(rclc_timer_init_default(
    &control_timer, &support, RCL_MS_TO_NS(period_ms), controlCallback));

  // Executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmdCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servoCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  // รีเซ็ตสถานะ
  portENTER_CRITICAL(&step_mux);
  pending_steps = 0;
  step_position = 0;
  current_dir_cw = true;
  portEXIT_CRITICAL(&step_mux);

  // sync time (optional)
  syncTime();
  return true;
}

bool destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  RCRET_IGNORE(rcl_timer_fini(&control_timer));
  RCRET_IGNORE(rcl_subscription_fini(&cmd_sub, &node));
  RCRET_IGNORE(rcl_subscription_fini(&servo_sub, &node));
  RCRET_IGNORE(rcl_publisher_fini(&status_pub, &node));
  RCRET_IGNORE(rclc_executor_fini(&executor));
  RCRET_IGNORE(rcl_node_fini(&node));
  RCRET_IGNORE(rclc_support_fini(&support));
  return true;
}

// ---------------- Callbacks ----------------
void cmdCallback(const void *msgin) {
  const auto *msg = (const geometry_msgs__msg__Twist*) msgin;

  // (0,0) = STOP/clear queue
  if (msg->linear.x == 0.0 && msg->linear.y == 0.0) {
    portENTER_CRITICAL(&isrMux);
    v_pending_steps_isr = 0;
    portEXIT_CRITICAL(&isrMux);

    portENTER_CRITICAL(&step_mux);
    pending_steps = 0;
    portEXIT_CRITICAL(&step_mux);
    return;
  }

  // ทิศ: >0 = CW, <0 = CCW, =0 = ไม่เปลี่ยน
  if (msg->linear.x != 0.0) {
    bool dir_cw = (msg->linear.x > 0.0);
    portENTER_CRITICAL(&isrMux);
    v_dir_cw_isr = dir_cw;
    portEXIT_CRITICAL(&isrMux);

    portENTER_CRITICAL(&step_mux);
    current_dir_cw = dir_cw;
    portEXIT_CRITICAL(&step_mux);
  }

  // จำนวนพัลส์
  long pulses = (long) llround(msg->linear.y);
  if (pulses >= MIN_PULSES_PER_CMD) {
    queueStepsFromApp(pulses, (msg->linear.x >= 0.0)); // ถ้า x=0 จะถือเป็น CW
  }

  // (ทางเลือก) ตั้ง target speed ผ่าน angular.x (steps/s)
  if (msg->angular.x > 0.0) {
    updateTargetIntervalFromRate((float)msg->angular.x);
  }
}

void controlCallback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
  // Publish status @ 50 Hz
  long pos, pend;
  uint32_t us_per_step;

  portENTER_CRITICAL(&step_mux);
  pos  = step_position;
  pend = pending_steps;
  portEXIT_CRITICAL(&step_mux);

  // อ่าน interval ปัจจุบัน (ไม่ critical ก็ได้ แต่เอาชัวร์)
  portENTER_CRITICAL(&isrMux);
  us_per_step = current_interval_us;
  portEXIT_CRITICAL(&isrMux);

  status_msg.linear.x  = (double)pos;           // position (steps) จน. step สะสม
  status_msg.linear.y  = (double)pend;          // pending steps (signed)
  status_msg.angular.z = (double)us_per_step;   // effective us/step ไมโครวินาที/step ที่กำลังวิ่งอยู่จริง
  status_msg.angular.x = 0.0;                   // ใช้ช่องนี้เป็นฟรี/ดีบักได้

  RCRET_IGNORE(rcl_publish(&status_pub, &status_msg, NULL));
}

void servoCallback(const void *msgin) {
  const auto *m = (const geometry_msgs__msg__Twist*) msgin;
  double deg = m->linear.x;         // รับองศาจาก linear.x
  if (deg < 0.0)   deg = 0.0;
  if (deg > 180.0) deg = 180.0;
  servo.write((int)llround(deg));   // เขียนเซอร์โวทันที
}

// ---------------- Time sync (optional) ----------------
void syncTime() {
  RCSOFTCHECK(rmw_uros_sync_session(10));
  unsigned long now_ms = millis();
  unsigned long long ros_ms = rmw_uros_epoch_millis();
  time_offset = ros_ms - now_ms;
}

// ---------------- Fatal error handling ----------------
void rclErrorLoop() {
  pinMode(LED_PIN, OUTPUT);
  while (1) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}
