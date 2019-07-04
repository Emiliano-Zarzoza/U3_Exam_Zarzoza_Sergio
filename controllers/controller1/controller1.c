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

void RightRobot (WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 6.36);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], -6.36);
}

void LeftRobot (WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], -6.36);
  wb_motor_set_velocity(wheels[2], 6.36);
}

void DownRobot (WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 6.36);
  wb_motor_set_velocity(wheels[1], -6.36);
  wb_motor_set_velocity(wheels[2], 0);
}

void UpRobot (WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], -6.36);
  wb_motor_set_velocity(wheels[1], 6.36);
  wb_motor_set_velocity(wheels[2], 0);
}

void TurnRight (WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 6.36);
  wb_motor_set_velocity(wheels[1], 6.36);
  wb_motor_set_velocity(wheels[2], 6.36);
}

void TurnLeft (WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], -6.36);
  wb_motor_set_velocity(wheels[1], -6.36);
  wb_motor_set_velocity(wheels[2], -6.36);
}

void StaticRobot (WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 0);
}

void ForwardDirectionRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], -6.66);
  wb_motor_set_velocity(wheels[1], 6.66);
  wb_motor_set_velocity(wheels[2], 0);
}
void TurnLeftAutonomous (WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 6.66);
  wb_motor_set_velocity(wheels[2], -6.66);
}

void TurnRightAutonomous (WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], -6.66);
  wb_motor_set_velocity(wheels[2], 6.66);
}



void ManualMode() {
  int key_pressed;
  int left = 0;
  int right = 0;
  double ComparingValue = 0;
  double distSensor1_value = 0;
  double distSensor2_value = 0;
  double PosSensor1 = 0;

  key_pressed = wb_keyboard_get_key();

  WbDeviceTag dist_sensor[2];
  dist_sensor[0] = wb_robot_get_device("dist_left_sensor1");
  dist_sensor[1] = wb_robot_get_device("dist_right_sensor2");

  wb_distance_sensor_enable(dist_sensor[0], TIME_STEP);
  wb_distance_sensor_enable(dist_sensor[1], TIME_STEP);

  //Motor components
  WbDeviceTag wheels[3];
  wheels[0] = wb_robot_get_device("right_wheel");
  wheels[1] = wb_robot_get_device("left_wheel");
  wheels[2] = wb_robot_get_device("back_wheel");

  wb_motor_set_position(wheels[0], INFINITY);
  wb_motor_set_position(wheels[1], INFINITY);
  wb_motor_set_position(wheels[2], INFINITY);

  //encoder components
  WbDeviceTag encoder[3];
  encoder[0] = wb_robot_get_device("pos_sensor1");
  encoder[1] = wb_robot_get_device("pos_sensor2");
  encoder[2] = wb_robot_get_device("pos_sensor3");

  wb_position_sensor_enable(encoder[0], TIME_STEP);
  wb_position_sensor_enable(encoder[1], TIME_STEP);
  wb_position_sensor_enable(encoder[2], TIME_STEP);

    distSensor1_value = wb_distance_sensor_get_value(dist_sensor[0]);
    distSensor2_value = wb_distance_sensor_get_value(dist_sensor[1]);
    printf("Distance sensor 1: %lf\n", distSensor1_value);
    printf("Distance sensor 2: %lf\n", distSensor2_value);

    printf("Comparing angle value %lf\n", ComparingValue);

    PosSensor1 = wb_position_sensor_get_value(encoder[0]);




    if (key_pressed == WB_KEYBOARD_DOWN) {
      DownRobot(wheels);
    } else if (key_pressed == WB_KEYBOARD_UP) {
      UpRobot(wheels);
    }  else if (key_pressed == WB_KEYBOARD_LEFT) {
      LeftRobot(wheels);
    } else if (key_pressed == WB_KEYBOARD_RIGHT) {
      RightRobot(wheels);
    } else if (key_pressed == 'S' ) {
      ComparingValue = PosSensor1 + 0.785398;
        left = 1;
    } else if (left == 1) {
        if (PosSensor1 <= ComparingValue) {
          TurnRight(wheels);
        } else {
          StaticRobot(wheels);
          left = 0;
        }
      } else if (key_pressed == 'A') {
        ComparingValue = PosSensor1 - 0.785398;
        right = 1;
      } else if (right == 1) {
        if (PosSensor1 >= ComparingValue) {
          TurnLeft(wheels);
        } else {
          StaticRobot(wheels);
          right = 0;
        }
      } else {
        StaticRobot(wheels);
      }
  }

  void AutonomousMode() {
    double ComparingValue = 0;
    double distSensor1_value = 0;
    double distSensor2_value = 0;
    WbDeviceTag dist_sensor[2];
    dist_sensor[0] = wb_robot_get_device("dist_left_sensor1");
    dist_sensor[1] = wb_robot_get_device("dist_right_sensor2");

    wb_distance_sensor_enable(dist_sensor[0], TIME_STEP);
    wb_distance_sensor_enable(dist_sensor[1], TIME_STEP);

    //Motor components
    WbDeviceTag wheels[3];
    wheels[0] = wb_robot_get_device("right_wheel");
    wheels[1] = wb_robot_get_device("left_wheel");
    wheels[2] = wb_robot_get_device("back_wheel");

    wb_motor_set_position(wheels[0], INFINITY);
    wb_motor_set_position(wheels[1], INFINITY);
    wb_motor_set_position(wheels[2], INFINITY);

    //encoder components
    WbDeviceTag encoder[3];
    encoder[0] = wb_robot_get_device("pos_sensor1");
    encoder[1] = wb_robot_get_device("pos_sensor2");
    encoder[2] = wb_robot_get_device("pos_sensor3");

    wb_position_sensor_enable(encoder[0], TIME_STEP);
    wb_position_sensor_enable(encoder[1], TIME_STEP);
    wb_position_sensor_enable(encoder[2], TIME_STEP);


    distSensor1_value = wb_distance_sensor_get_value(dist_sensor[0]);
    distSensor2_value = wb_distance_sensor_get_value(dist_sensor[1]);
    printf("Distance sensor 1: %lf\n", distSensor1_value);
    printf("Distance sensor 2: %lf\n", distSensor2_value);

    printf("Comparing angle value %lf\n", ComparingValue);

    ForwardDirectionRobot(wheels);

    if(distSensor1_value < distSensor2_value && distSensor1_value < 200) {
      TurnLeftAutonomous(wheels);
    } else if (distSensor1_value > distSensor2_value && distSensor2_value < 200) {
      TurnRightAutonomous(wheels);
    }
  }

int main(int argc, char **argv) {

  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);

  int key_pressed;
  //double PosSensor1 = 0;
  //double PosSensor2 = 0;
  //double PosSensor3 = 0;
  //int left = 0;
  //int right = 0;
  //double ComparingValue = 0;
  //double distSensor1_value = 0;
  //double distSensor2_value = 0;
  int w = 0;
  int g = 1;

  //distance sensor components
  WbDeviceTag dist_sensor[2];
  dist_sensor[0] = wb_robot_get_device("dist_left_sensor1");
  dist_sensor[1] = wb_robot_get_device("dist_right_sensor2");

  wb_distance_sensor_enable(dist_sensor[0], TIME_STEP);
  wb_distance_sensor_enable(dist_sensor[1], TIME_STEP);

  //Motor components
  WbDeviceTag wheels[3];
  wheels[0] = wb_robot_get_device("right_wheel");
  wheels[1] = wb_robot_get_device("left_wheel");
  wheels[2] = wb_robot_get_device("back_wheel");

  wb_motor_set_position(wheels[0], INFINITY);
  wb_motor_set_position(wheels[1], INFINITY);
  wb_motor_set_position(wheels[2], INFINITY);

  //encoder components
  WbDeviceTag encoder[3];
  encoder[0] = wb_robot_get_device("pos_sensor1");
  encoder[1] = wb_robot_get_device("pos_sensor2");
  encoder[2] = wb_robot_get_device("pos_sensor3");

  wb_position_sensor_enable(encoder[0], TIME_STEP);
  wb_position_sensor_enable(encoder[1], TIME_STEP);
  wb_position_sensor_enable(encoder[2], TIME_STEP);

  //wb_motor_set_velocity(wheels[0], 0);
  //wb_motor_set_velocity(wheels[1], 0);
  //wb_motor_set_velocity(wheels[2], 0);


  while (wb_robot_step(TIME_STEP) != -1) {
    key_pressed = wb_keyboard_get_key();

    if(key_pressed == 'W') {
      w = 1;
      g = 0;
    } else if (key_pressed == 'G') {
      g = 1;
      w = 0;
    }
    printf("w = %i     \n", w);
    printf("g = %i     \n", g);

    if(w == 1)
    ManualMode();
    if(g == 1)
    AutonomousMode();
  };

  wb_robot_cleanup();

  return 0;
}
