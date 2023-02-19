#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/utils/motion.h>
#include <math.h>

#define TIME_STEP 32
#define PI 3.14159265359
#define L1 0.2
#define L2 0.2

WbDeviceTag motor1, motor2;
WbDeviceTag ps1, ps2;
WbDeviceTag imu;

void set_position(WbDeviceTag motor, double position) {
  wb_motor_set_position(motor, position);
  wb_motor_set_velocity(motor, INFINITY);
}

double read_position(WbDeviceTag sensor) {
  return wb_position_sensor_get_value(sensor);
}

double read_imu() {
  const double *values = wb_inertial_unit_get_roll_pitch_yaw(imu);
  return values[1]; // return pitch angle
}

void move_arm(double x, double y) {
  double theta2 = acos((x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2));
  double theta1 = atan2(y, x) - atan2((L2*sin(theta2)), (L1+L2*cos(theta2)));
  set_position(motor1, theta1);
  set_position(motor2, theta2);
}

int main(int argc, char **argv) {
  wb_robot_init();

  motor1 = wb_robot_get_device("motor1");
  motor2 = wb_robot_get_device("motor2");
  ps1 = wb_robot_get_device("ps1");
  ps2 = wb_robot_get_device("ps2");
  imu = wb_robot_get_device("imu");

  wb_motor_set_position(motor1, INFINITY);
  wb_motor_set_position(motor2, INFINITY);
  wb_position_sensor_enable(ps1, TIME_STEP);
  wb_position_sensor_enable(ps2, TIME_STEP);
  wb_inertial_unit_enable(imu, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    // Get current position of end-effector
    double x = L1*cos(read_position(ps1)) + L2*cos(read_position(ps1)+read_position(ps2));
    double y = L1*sin(read_position(ps1)) + L2*sin(read_position(ps1)+read_position(ps2));
   
    // Move to desired position
    move_arm(0.1, 0.1); // Change this to move to a different position
   
    // Print current and desired position
    printf("Current position: (%.2f, %.2f), Desired position: (%.2f, %.2f)\n", x, y, 0.1, 0.1);
  }

  wb_robot_cleanup();

  return 0;
}
