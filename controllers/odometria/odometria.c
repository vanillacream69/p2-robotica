#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gyro.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16

#define MAX_SPEED 48
#define SPEED 8
#define ULTRASONIC_SENSORS 5
#define IR_SENSORS 8
#define RADIO_RUEDA 0.021
#define ESPACIO_ENTRE_RUEDAS 0.1054
#define RADIO_ENTRE_RUEDAS ESPACIO_ENTRE_RUEDAS/2
#define TAMAÑO_CELDA 0.25*0.25
#define VELOCIDAD_ANGULAR 4.05

WbDeviceTag sensores_ultr[5];
WbDeviceTag sensores_ir[8];
WbDeviceTag left_motor, right_motor;
WbDeviceTag gyro;

static const char *sensores_ultrasonidos[ULTRASONIC_SENSORS]={
  "left ultrasonic sensor", "front left ultrasonic sensor", "front ultrasonic sensor", "front right ultrasonic sensor",
  "right ultrasonic sensor"};

static const char *sensores_infrarrojos[IR_SENSORS]={
  "rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
  "front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor"};


void init(){
  wb_robot_init();
  for(int i=0; i<5; ++i){
    sensores_ultr[i]=wb_robot_get_device(sensores_ultrasonidos[i]);
    wb_distance_sensor_enable(sensores_ultr[i], TIME_STEP);
  }

  for(int i=0; i<5; ++i){
    sensores_ir[i]=wb_robot_get_device(sensores_infrarrojos[i]);
    wb_distance_sensor_enable(sensores_ir[i], TIME_STEP);
  }

  // wb_gyro_enable(gyro, 1000);

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // set up the motor speeds at 10% of the MAX_SPEED.
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}

void fw(WbDeviceTag left, WbDeviceTag right){
  wb_motor_set_velocity(left, SPEED);
  wb_motor_set_velocity(right, SPEED);
}

void bw(WbDeviceTag left, WbDeviceTag right){
  wb_motor_set_velocity(left, -SPEED);
  wb_motor_set_velocity(right, -SPEED);
}

// void moverseUnaCasilla(){
//   float duration;
//   float start;
//   current_time = wb_robot_get_time();
// }

// double current_time = 0;

int main(int argc, char **argv) {
  init();
  wb_gyro_enable(gyro, 1000);
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *a=wb_gyro_get_values(gyro);
    fw(left_motor,right_motor);
    printf("TIME: %f\n",wb_robot_get_time());
    printf("PERIOD: %f\n",a[0]);
  }

  wb_robot_cleanup();

  return 0;
}