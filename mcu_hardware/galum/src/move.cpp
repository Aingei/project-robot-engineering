#include <Arduino.h>
#include <Wire.h>
#include <cmath>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ESP32Encoder.h>

// -------- Hardware objects --------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

// -------- Motor pins --------
#define AIN1 33
#define AIN2 25
#define BIN1 26
#define BIN2 27

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
// #define PWM_CHANNEL_AIN1 0
// #define PWM_CHANNEL_AIN2 1
// #define PWM_CHANNEL_BIN1 2
// #define PWM_CHANNEL_BIN2 3

// Left wheels
#define PWM_CHANNEL_FL1 0
#define PWM_CHANNEL_FL2 1
#define PWM_CHANNEL_BL1 2
#define PWM_CHANNEL_BL2 3

// Right wheels
#define PWM_CHANNEL_FR1 4
#define PWM_CHANNEL_FR2 5
#define PWM_CHANNEL_BR1 6
#define PWM_CHANNEL_BR2 7


// -------- Robot geometry --------
float wheel_separation = 0.20; // meters between wheels
float wheel_radius = 0.05;     // meters radius of wheel
float max_rpm = 150.0;         // motor maximum RPM

// -------- ROS entities --------
rcl_publisher_t encoder_publisher;
geometry_msgs__msg__Twist encoder_msg;

rcl_publisher_t imu_publisher;
geometry_msgs__msg__Twist imu_msg;

rcl_subscription_t motor_subscriber;
geometry_msgs__msg__Twist motor_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// -------- Helpers --------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
void Move();
struct timespec getTime();

// =====================================================
//                   SETUP
// =====================================================
void setup() {   
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  Wire.begin(21, 22);

  encoderLeft.attachFullQuad(17, 5);
  encoderLeft.clearCount();
  encoderRight.attachFullQuad(18, 19);
  encoderRight.clearCount();

  bno.begin();

  bno.setExtCrystalUse(true);

  // pinMode(AIN1, OUTPUT);
  // pinMode(AIN2, OUTPUT);
  // pinMode(BIN1, OUTPUT);
  // pinMode(BIN2, OUTPUT);
  
  pinMode(FL1, OUTPUT);
  pinMode(FL2, OUTPUT);
  pinMode(FR1, OUTPUT);
  pinMode(FR2, OUTPUT);
  pinMode(BL1, OUTPUT);
  pinMode(BL2, OUTPUT);
  pinMode(BR1, OUTPUT);
  pinMode(BR2, OUTPUT);

  // ledcSetup(PWM_CHANNEL_AIN1, PWM_FREQ, PWM_RESOLUTION);
  // ledcSetup(PWM_CHANNEL_AIN2, PWM_FREQ, PWM_RESOLUTION);
  // ledcSetup(PWM_CHANNEL_BIN1, PWM_FREQ, PWM_RESOLUTION);
  // ledcSetup(PWM_CHANNEL_BIN2, PWM_FREQ, PWM_RESOLUTION);

  // ----------------- Setup Left Wheels -----------------
  ledcSetup(PWM_CHANNEL_FL1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_FL2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_BL1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_BL2, PWM_FREQ, PWM_RESOLUTION);

  // ----------------- Setup Right Wheels -----------------
  ledcSetup(PWM_CHANNEL_FR1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_FR2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_BR1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_BR2, PWM_FREQ, PWM_RESOLUTION);


  // ledcAttachPin(AIN1, PWM_CHANNEL_AIN1);
  // ledcAttachPin(AIN2, PWM_CHANNEL_AIN2);
  // ledcAttachPin(BIN1, PWM_CHANNEL_BIN1);
  // ledcAttachPin(BIN2, PWM_CHANNEL_BIN2);

  // Left wheels
  ledcAttachPin(AIN1, PWM_CHANNEL_FL1);
  ledcAttachPin(AIN2, PWM_CHANNEL_FL2);
  ledcAttachPin(BIN1, PWM_CHANNEL_BL1);
  ledcAttachPin(BIN2, PWM_CHANNEL_BL2);

  // Right wheels (สมมติใช้ pins อื่นของ ESP32)
  ledcAttachPin(PIN_FR1, PWM_CHANNEL_FR1);
  ledcAttachPin(PIN_FR2, PWM_CHANNEL_FR2);
  ledcAttachPin(PIN_BR1, PWM_CHANNEL_BR1);
  ledcAttachPin(PIN_BR2, PWM_CHANNEL_BR2);

}

// =====================================================
//                   MAIN LOOP
// =====================================================
void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) { destroyEntities(); }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
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

// =====================================================
//                   ROS CALLBACKS
// =====================================================
void controlCallback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    Move();
    publishData();
  }
}

void twistCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  prev_cmd_time = millis();

  // Store desired velocities from teleop
  motor_msg.linear.x  = msg->linear.x;   // Forward/backward (m/s)
  motor_msg.angular.z = msg->angular.z;  // Rotation (rad/s)
}

// =====================================================
//                   CREATE / DESTROY ENTITIES
// =====================================================
bool createEntities() {
  allocator = rcl_get_default_allocator();
  geometry_msgs__msg__Twist__init(&encoder_msg);
  geometry_msgs__msg__Twist__init(&imu_msg);

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 96);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  RCCHECK(rclc_node_init_default(&node, "esp32_robot_node", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(
      &encoder_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/galum/encoder"));

  RCCHECK(rclc_publisher_init_best_effort(
      &imu_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/galum/imu"));

  RCCHECK(rclc_subscription_init_default(
      &motor_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/galum/cmd_vel"));

  const unsigned int control_timeout = 20;
  RCCHECK(rclc_timer_init_default(
      &control_timer, &support,
      RCL_MS_TO_NS(control_timeout), controlCallback));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));

  RCCHECK(rclc_executor_add_subscription(
      &executor, &motor_subscriber, &motor_msg, &twistCallback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  syncTime();

  return true;
}

bool destroyEntities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&encoder_publisher, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  return true;
}

// =====================================================
//                   4-WHEEL ROBOT CONTROL
// =====================================================
void Move() {
  float v = motor_msg.linear.x;       // m/s
  float omega = motor_msg.angular.z;  // rad/s

  // Convert to wheel linear velocities
  float v_left  = v - (omega * wheel_separation / 2.0);
  float v_right = v + (omega * wheel_separation / 2.0);

  // Convert to RPM
  float rpm_left  = (v_left  / (2 * M_PI * wheel_radius)) * 60.0;
  float rpm_right = (v_right / (2 * M_PI * wheel_radius)) * 60.0;

  // Clamp RPM
  rpm_left  = constrain(rpm_left,  -max_rpm, max_rpm);
  rpm_right = constrain(rpm_right, -max_rpm, max_rpm);

  // Map RPM to PWM duty cycle
  uint8_t duty_left  = (uint8_t)((fabs(rpm_left)  / max_rpm) * 255.0);
  uint8_t duty_right = (uint8_t)((fabs(rpm_right) / max_rpm) * 255.0);

  // ----------------- Left Wheels -----------------
  // Front Left
  if (rpm_left > 0) {
    ledcWrite(PWM_CHANNEL_FL1, duty_left);
    ledcWrite(PWM_CHANNEL_FL2, 0);
  } else if (rpm_left < 0) {
    ledcWrite(PWM_CHANNEL_FL1, 0);
    ledcWrite(PWM_CHANNEL_FL2, duty_left);
  } else {
    ledcWrite(PWM_CHANNEL_FL1, 0);
    ledcWrite(PWM_CHANNEL_FL2, 0);
  }

  // Rear Left (ตามหน้า)
  if (rpm_left > 0) {
    ledcWrite(PWM_CHANNEL_BL1, duty_left);
    ledcWrite(PWM_CHANNEL_BL2, 0);
  } else if (rpm_left < 0) {
    ledcWrite(PWM_CHANNEL_BL1, 0);
    ledcWrite(PWM_CHANNEL_BL2, duty_left);
  } else {
    ledcWrite(PWM_CHANNEL_BL1, 0);
    ledcWrite(PWM_CHANNEL_BL2, 0);
  }

  // ----------------- Right Wheels -----------------
  // Front Right
  if (rpm_right > 0) {
    ledcWrite(PWM_CHANNEL_FR1, duty_right);
    ledcWrite(PWM_CHANNEL_FR2, 0);
  } else if (rpm_right < 0) {
    ledcWrite(PWM_CHANNEL_FR1, 0);
    ledcWrite(PWM_CHANNEL_FR2, duty_right);
  } else {
    ledcWrite(PWM_CHANNEL_FR1, 0);
    ledcWrite(PWM_CHANNEL_FR2, 0);
  }

  // Rear Right (ตามหน้า)
  if (rpm_right > 0) {
    ledcWrite(PWM_CHANNEL_BR1, duty_right);
    ledcWrite(PWM_CHANNEL_BR2, 0);
  } else if (rpm_right < 0) {
    ledcWrite(PWM_CHANNEL_BR1, 0);
    ledcWrite(PWM_CHANNEL_BR2, duty_right);
  } else {
    ledcWrite(PWM_CHANNEL_BR1, 0);
    ledcWrite(PWM_CHANNEL_BR2, 0);
  }
}

void publishData() {
  encoder_msg.linear.x = encoderLeft.getCount();
  encoder_msg.linear.y = encoderRight.getCount();
  rcl_publish(&encoder_publisher, &encoder_msg, NULL);

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // sensors_event_t orientationData;
  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  imu_msg.angular.z = euler.x(); // yaw
  // imu_msg.angular.x = orientationData.orientation.z; // roll
  // imu_msg.angular.y = orientationData.orientation.y; // pitch
  rcl_publish(&imu_publisher, &imu_msg, NULL);
}

// =====================================================
//                   TIME SYNC
// =====================================================
void syncTime() {
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - now;
}

struct timespec getTime() {
  struct timespec tp = {0};
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}

// =====================================================
//                   ERROR LOOP
// =====================================================
void rclErrorLoop() {
  const int LED_PIN = 13;
  pinMode(LED_PIN, OUTPUT);
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}