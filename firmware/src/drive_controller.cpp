// drive_controller.cpp — Differential Drive Controller
// Converts geometry_msgs/Twist (cmd_vel) to wheel velocities.
// Supports both PWM servos and serial bus servos via compile-time flag.

#include "drive_controller.h"
#include "otto_config.h"
#include <Arduino.h>
#include <geometry_msgs/msg/twist.h>

#if SERVO_TYPE_SERIAL_BUS
#include <SCServo.h>
#else
#include <ESP32Servo.h>
#endif

// micro-ROS handles
static rcl_subscription_t cmd_vel_sub;
static geometry_msgs__msg__Twist cmd_vel_msg;
static unsigned long last_cmd_vel_time = 0;

#if SERVO_TYPE_SERIAL_BUS
static SMS_STS sms_sts;
#else
static Servo servo_left;
static Servo servo_right;
#endif

// Forward declaration
void drive_attach();

// cmd_vel callback — differential drive kinematics
static void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    last_cmd_vel_time = millis();

    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    // Differential drive: convert (v, omega) to individual wheel velocities
    double v_left  = linear_x - (angular_z * WHEEL_BASE / 2.0);
    double v_right = linear_x + (angular_z * WHEEL_BASE / 2.0);

#if SERVO_TYPE_SERIAL_BUS
    // Serial bus servo: map velocity to speed value
    // STS servos in wheel mode accept speed as signed int16 (-1023 to 1023)
    int16_t speed_left  = (int16_t)constrain(v_left  * SERVO_SPEED_SCALE, -1023, 1023);
    int16_t speed_right = (int16_t)constrain(-v_right * SERVO_SPEED_SCALE, -1023, 1023);  // Inverted (mirror-mounted)
    sms_sts.WriteSpe(SERVO_BUS_ID_LEFT, speed_left, 0);
    sms_sts.WriteSpe(SERVO_BUS_ID_RIGHT, speed_right, 0);
#else
    // PWM servo: compute microsecond offset from calibrated neutral point
    int neutral_left  = SERVO_STOP_US + SERVO_LEFT_TRIM;
    int neutral_right = SERVO_STOP_US + SERVO_RIGHT_TRIM;

    int offset_left  = (int)(v_left  * SERVO_SPEED_SCALE);
    int offset_right = (int)(v_right * SERVO_SPEED_SCALE);

    // Dead band: if the offset is smaller than the servo's mechanical dead zone,
    // clamp to neutral. This prevents the servo from briefly spinning the wrong
    // direction when the command crosses its internal dead band.
    if (abs(offset_left)  < SERVO_DEAD_BAND_US) offset_left  = 0;
    if (abs(offset_right) < SERVO_DEAD_BAND_US) offset_right = 0;

    int us_left  = constrain(neutral_left  + offset_left,  SERVO_MIN_US, SERVO_MAX_US);
    int us_right = constrain(neutral_right - offset_right,  SERVO_MIN_US, SERVO_MAX_US);  // Inverted (mirror-mounted)

    // If both wheels are at neutral, detach to eliminate any residual PWM jitter
    if (offset_left == 0 && offset_right == 0) {
        drive_stop();
        return;
    }

    // Re-attach servos if they were detached by a previous stop
    drive_attach();

    servo_left.writeMicroseconds(us_left);
    servo_right.writeMicroseconds(us_right);
#endif
}

void drive_init() {
#if SERVO_TYPE_SERIAL_BUS
    Serial1.begin(SERVO_BUS_BAUD, SERIAL_8N1, SERVO_BUS_RXD, SERVO_BUS_TXD);
    sms_sts.pSerial = &Serial1;
    delay(100);
    // Set both servos to wheel (continuous rotation) mode
    sms_sts.WheelMode(SERVO_BUS_ID_LEFT);
    sms_sts.WheelMode(SERVO_BUS_ID_RIGHT);
#else
    servo_left.attach(PIN_SERVO_LEFT, SERVO_MIN_US, SERVO_MAX_US);
    servo_right.attach(PIN_SERVO_RIGHT, SERVO_MIN_US, SERVO_MAX_US);
    servo_left.writeMicroseconds(SERVO_STOP_US + SERVO_LEFT_TRIM);
    servo_right.writeMicroseconds(SERVO_STOP_US + SERVO_RIGHT_TRIM);
#endif
}

void drive_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator) {
    rclc_subscription_init_best_effort(
        &cmd_vel_sub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
}

void drive_add_to_executor(rclc_executor_t *executor) {
    rclc_executor_add_subscription(executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
}

void drive_destroy_entities(rcl_node_t *node) {
    rcl_subscription_fini(&cmd_vel_sub, node);
}

void drive_stop() {
#if SERVO_TYPE_SERIAL_BUS
    sms_sts.WriteSpe(SERVO_BUS_ID_LEFT, 0, 0);
    sms_sts.WriteSpe(SERVO_BUS_ID_RIGHT, 0, 0);
#else
    // Detach stops the PWM signal entirely — no creep from miscalibrated neutral
    servo_left.detach();
    servo_right.detach();
#endif
}

void drive_attach() {
#if !SERVO_TYPE_SERIAL_BUS
    if (!servo_left.attached()) {
        servo_left.attach(PIN_SERVO_LEFT, SERVO_MIN_US, SERVO_MAX_US);
        servo_left.writeMicroseconds(SERVO_STOP_US + SERVO_LEFT_TRIM);
    }
    if (!servo_right.attached()) {
        servo_right.attach(PIN_SERVO_RIGHT, SERVO_MIN_US, SERVO_MAX_US);
        servo_right.writeMicroseconds(SERVO_STOP_US + SERVO_RIGHT_TRIM);
    }
#endif
}

void drive_check_timeout() {
    if (last_cmd_vel_time > 0 && (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS)) {
        drive_stop();
        last_cmd_vel_time = 0;  // Prevent repeated stop calls
    }
}
