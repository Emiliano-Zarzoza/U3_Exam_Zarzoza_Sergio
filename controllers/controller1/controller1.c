#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include <stdio.h>
#include <math.h>

#define TIME_STEP 64
#define OBSTACLE_DISTANCE 25.0
#define PI 3.14159


int main(int argc, char **argv) {

  wb_robot_init();
  wb_keyboard_enable();

  int key_pressed;
  double PosSensor1 = 0;
  double PosSensor2 = 0;
  double PosSensor3 = 0;
  int left = 0;
  int right = 0;
  double ComparingValue = 0;
  double distSensor1_value = 0;
  double distSensor2_value = 0;

  //distance sensor components
  WbDeviceTag dist_sensor[2];
  WbDeviceTag dist_sensor[1] = wb_robot_get_device("dist_sensor1");
  WbDeviceTag dist_sensor[2] = wb_robot_get_device("dist_sensor2");

  wb_distance_sensor_enable(dist_sensor[1], TIME_STEP);
  wb_distance_sensor_enable(dist_sensor[2], TIME_STEP);

  //Motor components
  WbDeviceTag wheels[3];
  wheels[1] = wb_robot_get_device("right_wheel")
  wheels[2] = wb_robot_get_device("left_wheel")
  wheels[3] = wb_robot_get_device("back_wheel")

  wb_motor_set_position (wheel[1], INFINITY);
  wb_motor_set_position (wheel[2], INFINITY);
  wb_motor_set_position (wheel[3], INFINITY);

  //encoder components
  WbDeviceTag encoder[3];
  encoder[1] = wb_robot_get_device("pos_sensor1")
  encoder[2] = wb_robot_get_device("pos_sensor2")
  encoder[3] = wb_robot_get_device("pos_sensor3")

  wb_position_sensor_enable(encoder[1], TIME_STEP);
  wb_position_sensor_enable(encoder[2], TIME_STEP);
  wb_position_sensor_enable(encoder[3], TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {

    PosSensor1 = wb_position_sensor_get_value(encoder[1]);
    PosSensor2 = wb_position_sensor_get_value(encoder[2]);
    PosSensor3 = wb_position_sensor_get_value(encoder[3]);
    printf("encoder_1: %lf\n", Encoder[1]);
    printf("encoder_2: %lf\n", encoder[2]);
    printf("encoder_3: %lf\n", encoder[3]);

    key_pressed = wb_keyboard_get_key();

    printf("Comparing angle value %lf\n", ComparingValue);

    distSensor1_value = wb_distance_sensor_get_value(dist_sensor[1]);
    distSensor2_value = wb_distance_sensor_get_value(dist_sensor[2]);
    printf("Distance sensor 1: %lf\n", distSensor1_value);
    printf("Distance sensor 2: %lf\n", distSensor2_value);


    if (key_pressed == WB_KEYBOARD_DOWN) {
      wb_motor_set_velocity(wheels[1], 6.36);
      wb_motor_set_velocity(wheel[2], -6.36);
      wb_motor_set_velocity(wheel[3], 0);
    } else if (key_pressed == WB_KEYBOARD_UP) {
      wb_motor_set_velocity(wheel[1], -6.36);
      wb_motor_set_velocity(wheel[2], 6.36);
      wb_motor_set_velocity(wheel[3], 0);
    }  else if (key_pressed == WB_KEYBOARD_LEFT) {
      wb_motor_set_velocity(wheel[1], 0);
      wb_motor_set_velocity(wheel[2], -6.36);
      wb_motor_set_velocity(wheel[3], 6.36);
    } else if (key_pressed == WB_KEYBOARD_RIGHT) {
      wb_motor_set_velocity(wheel[1], 6.36);
      wb_motor_set_velocity(wheel[2], 0);
      wb_motor_set_velocity(wheel[3], -6.36);
    } else if (key_pressed == 'S' ) {
          ComparingValue = PosSensor1 + 0.785398;
          left = 1;
         }

         
         }

  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
