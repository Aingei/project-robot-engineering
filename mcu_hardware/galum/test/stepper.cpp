<<<<<<< HEAD
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

=======
>>>>>>> 2a8e2dd (web)
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

<<<<<<< HEAD
// === ใช้ GPIO driver ที่ไวกว่า digitalWrite ===
#include "driver/gpio.h"
// === Hardware timer (ESP32) ===
#include "esp32-hal-timer.h"

// ---------------- User Config ----------------
#define STEP_PIN 25
#define DIR_PIN  17
#define LED_PIN   2

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

//กันกดปุ่มไม่เท่ากัน
#define DEBOUNCE_MS          120      // กันเด้ง/กันสแปม (ปรับ 80–200ms ตามนิสัยปุ่ม)
#define REPEAT_MS               180
#define FIXED_PULSES_PER_CLICK 50     // กำหนดพัลส์ตายตัวต่อ 1 คลิก

// ------------- Helpers / Macros -------------
=======
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
>>>>>>> 2a8e2dd (web)
#define RCCHECK(fn)  { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { rclErrorLoop(); } }
#define RCSOFTCHECK(fn) (void)(fn)
#define RCRET_IGNORE(expr) do { rcl_ret_t _rc = (expr); (void)_rc; } while (0)
#define EXECUTE_EVERY_N_MS(MS, X) do { static int64_t t=-1; if (t==-1) t = uxr_millis(); \
  if ((int32_t)(uxr_millis()-t) > (MS)) { X; t = uxr_millis(); } } while (0)

<<<<<<< HEAD
// ---------------- micro-ROS objects ----------------
=======
// -------- micro-ROS objects --------
>>>>>>> 2a8e2dd (web)
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     control_timer;

<<<<<<< HEAD
rcl_subscription_t cmd_sub;     // /galum/stepper/angle
geometry_msgs__msg__Twist cmd_msg;

rcl_publisher_t status_pub;     // /galum/stepper/status
geometry_msgs__msg__Twist status_msg;

// ---------------- Agent state machine ----------------
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

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

volatile uint32_t last_click_ms = 0;  // เวลา “คลิก” ล่าสุด
volatile bool     pressed_latch = false; // สถานะกดค้าง (สำหรับ edge detect)
volatile int8_t   last_dir_sign = 0;    // -1 = CCW, +1 = CW, 0 = ว่าง


hw_timer_t* stepTimer = nullptr; //มิวเทคโครงสร้างข้อมูลหรืออ็อบเจกต์ที่ทำหน้าที่เป็น กลไกควบคุมการเข้าถึงทรัพยากรที่ใช้ร่วมกัน กัน race ตอน main/ISR อัปเดต position/pending
portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED; //กัน race ตอน main ปรับค่าที่ ISR ใช้ (เช่น target interval, ทิศ)

=======
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

>>>>>>> 2a8e2dd (web)
// ---------------- Prototypes ----------------
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void cmdCallback(const void *msgin);

<<<<<<< HEAD
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
=======
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
>>>>>>> 2a8e2dd (web)
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      digitalWrite(LED_PIN, LOW);
      break;

    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroyEntities();
      break;

    case AGENT_CONNECTED:
<<<<<<< HEAD
      EXECUTE_EVERY_N_MS(1000,
=======
      EXECUTE_EVERY_N_MS(200,
>>>>>>> 2a8e2dd (web)
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED) {
        digitalWrite(LED_PIN, HIGH);
<<<<<<< HEAD
        // หมุน executor แบบเบามือเพื่อไม่ไปรบกวน ISR
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
=======
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
>>>>>>> 2a8e2dd (web)
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }
<<<<<<< HEAD
}

// ---------------- micro-ROS wiring ----------------
bool createEntities() {
  allocator = rcl_get_default_allocator();

=======

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
>>>>>>> 2a8e2dd (web)
  geometry_msgs__msg__Twist__init(&cmd_msg);
  geometry_msgs__msg__Twist__init(&status_msg);

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
<<<<<<< HEAD
  // ตั้ง ROS_DOMAIN_ID ให้ตรงกับ agent ของคุณ
  rcl_init_options_set_domain_id(&init_options, 96);

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "stepper_node", "", &support));

  // Subscriber: รับคำสั่ง "คลิก" ที่ /galum/stepper/angle
=======
  // ใส่ ROS_DOMAIN_ID ให้ตรงกับฝั่ง agent
  rcl_init_options_set_domain_id(&init_options, 96);

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  RCCHECK(rclc_node_init_default(&node, "stepper_node", "", &support));

  // Subscriber: /galum/stepper (Twist)
>>>>>>> 2a8e2dd (web)
  RCCHECK(rclc_subscription_init_default(
    &cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/galum/stepper/angle"));

<<<<<<< HEAD
  // Publisher: ส่งสถานะที่ /galum/stepper/status
  RCCHECK(rclc_publisher_init_default(
    &status_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/galum/stepper/status"));

  // Control timer @ 50 Hz
=======
  // Publisher: /galum/stepper/status (Twist)
  // ใหม่ (Reliable)
  RCCHECK(rclc_publisher_init_default(
  &status_pub, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "/galum/stepper/status"));


  // Control timer @ 20 ms (50 Hz): publish status & house-keeping
>>>>>>> 2a8e2dd (web)
  const unsigned int period_ms = 20;
  RCCHECK(rclc_timer_init_default(
    &control_timer, &support, RCL_MS_TO_NS(period_ms), controlCallback));

  // Executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmdCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

<<<<<<< HEAD
  // รีเซ็ตสถานะ
  portENTER_CRITICAL(&step_mux);
  pending_steps = 0;
  step_position = 0;
  current_dir_cw = true;
  portEXIT_CRITICAL(&step_mux);

  // sync time (optional)
=======
>>>>>>> 2a8e2dd (web)
  syncTime();
  return true;
}

bool destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

<<<<<<< HEAD
=======
  // ใช้ RCRET_IGNORE เพื่อตัด warning "ignoring return value"
>>>>>>> 2a8e2dd (web)
  RCRET_IGNORE(rcl_timer_fini(&control_timer));
  RCRET_IGNORE(rcl_subscription_fini(&cmd_sub, &node));
  RCRET_IGNORE(rcl_publisher_fini(&status_pub, &node));
  RCRET_IGNORE(rclc_executor_fini(&executor));
  RCRET_IGNORE(rcl_node_fini(&node));
  RCRET_IGNORE(rclc_support_fini(&support));
  return true;
}

<<<<<<< HEAD
// ---------------- Callbacks ----------------
void cmdCallback(const void *msgin) {
  const auto *msg = (const geometry_msgs__msg__Twist*) msgin;

  const double x = msg->linear.x;
  const long   y_raw = (long) llround(msg->linear.y);
  const uint32_t now_ms = millis();

  // ---- STOP: เคลียร์คิว + ปลด latch ----
  if (x == 0.0 && y_raw == 0) {
    portENTER_CRITICAL(&isrMux);   v_pending_steps_isr = 0; portEXIT_CRITICAL(&isrMux);
    portENTER_CRITICAL(&step_mux); pending_steps = 0;       portEXIT_CRITICAL(&step_mux);
    pressed_latch = false;
    last_dir_sign = 0;
    return;
  }

  // ---- ปล่อยปุ่ม: x==0 -> ปลด latch และรอคลิกใหม่ ----
  if (x == 0.0) {
    pressed_latch = false;
    last_dir_sign = 0;
    return;
  }

  // ---- มีการกด (x != 0) ----
  const bool dir_cw = (x > 0.0);
  const int8_t this_sign = dir_cw ? +1 : -1;

  // ปรับความเร็วเป้าหมายถ้ามีส่งมา
  if (msg->angular.x > 0.0) {
    updateTargetIntervalFromRate((float)msg->angular.x);
  }

  // ตั้งทิศทันที (ไม่ต้องรอคิว)
  portENTER_CRITICAL(&isrMux);  v_dir_cw_isr = dir_cw;  portEXIT_CRITICAL(&isrMux);
  portENTER_CRITICAL(&step_mux); current_dir_cw = dir_cw; portEXIT_CRITICAL(&step_mux);

  // กำหนดจำนวนพัลส์ต่อคลิก (ถ้า y เล็กไปให้ใช้ค่ามาตรฐาน)
  long pulses = (y_raw >= MIN_PULSES_PER_CMD) ? y_raw : FIXED_PULSES_PER_CLICK;

  // ---- กรณีเปลี่ยนทิศขณะกดค้าง: ยิงทันทีและรีเซ็ต latch/debounce ----
  if (last_dir_sign != 0 && this_sign != last_dir_sign) {
    // reset timing แล้ว "คลิกทันที"
    last_click_ms = now_ms;
    queueStepsFromApp(pulses, dir_cw);
    pressed_latch = true;
    last_dir_sign = this_sign;
    return;
  }

  // ---- Edge + Debounce + Auto-repeat ----
  if (!pressed_latch) {
    // คลิกแรกของรอบ: เช็ค debounce
    if ((uint32_t)(now_ms - last_click_ms) < DEBOUNCE_MS) return;
    last_click_ms = now_ms;
    queueStepsFromApp(pulses, dir_cw);
    pressed_latch = true;
    last_dir_sign = this_sign;
    return;
  } else {
    // กดค้างอยู่ → ยิงซ้ำตาม REPEAT_MS
    if ((uint32_t)(now_ms - last_click_ms) >= REPEAT_MS) {
      last_click_ms = now_ms;
      queueStepsFromApp(pulses, dir_cw);
      last_dir_sign = this_sign;
    }
    return;
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
=======
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
>>>>>>> 2a8e2dd (web)

  RCRET_IGNORE(rcl_publish(&status_pub, &status_msg, NULL));
}

<<<<<<< HEAD
// ---------------- Time sync (optional) ----------------
=======
// -------- Time sync (optional) --------
>>>>>>> 2a8e2dd (web)
void syncTime() {
  RCSOFTCHECK(rmw_uros_sync_session(10));
  unsigned long now_ms = millis();
  unsigned long long ros_ms = rmw_uros_epoch_millis();
  time_offset = ros_ms - now_ms;
}

<<<<<<< HEAD
// ---------------- Fatal error handling ----------------
=======
// -------- Fatal error handling --------
>>>>>>> 2a8e2dd (web)
void rclErrorLoop() {
  pinMode(LED_PIN, OUTPUT);
  while (1) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}
