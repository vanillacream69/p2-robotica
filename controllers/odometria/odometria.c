#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
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
WbDeviceTag left_encoder, right_encoder;
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
  left_encoder = wb_robot_get_device("left wheel sensor");
  right_encoder = wb_robot_get_device("right wheel sensor");

  wb_position_sensor_enable(left_encoder,TIME_STEP);
  wb_position_sensor_enable(right_encoder,TIME_STEP);

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // set up the motor speeds at 10% of the MAX_SPEED.
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}

void move(WbDeviceTag left, WbDeviceTag right, int sp){
  wb_motor_set_velocity(left, sp);
  wb_motor_set_velocity(right, sp);
}

void fw(WbDeviceTag left, WbDeviceTag right){
  wb_motor_set_velocity(left, SPEED);
  wb_motor_set_velocity(right, SPEED);
}

void bw(WbDeviceTag left, WbDeviceTag right){
  wb_motor_set_velocity(left, -SPEED);
  wb_motor_set_velocity(right, -SPEED);
}

void stop(WbDeviceTag left, WbDeviceTag right){
  wb_motor_set_velocity(left, 0.0);
  wb_motor_set_velocity(right, 0.0);
}

// int done = 0;
float current_time = 0;
float start_time = 0;
float duration = 0;
float velocidad_lineal = 0;
int i=0;
// void moverseUnaCasilla(){
//   // int done = 0;
//   // float start_time = wb_robot_get_time();
//   float current_time = 0, duration = 0;
//   if(current_time < start_time + duration){
//     current_time = wb_robot_get_time();
//     duration = 4*TAMAÑO_CELDA/(RADIO_RUEDA * SPEED);
//     printf("TIME: %f\t START: %f\n",current_time,start_time);
//     printf("DURATION: %f\n",duration);
//     fw(left_motor, right_motor);
//   } else {
//     stop(left_motor, right_motor);
//   }
// }

int casilla(){
  int sp = SPEED;
  current_time = wb_robot_get_time();
  duration = 4*TAMAÑO_CELDA/(RADIO_RUEDA * SPEED);
  if(current_time >= start_time + duration){
    sp = 0.0;
    printf("MOVIDO %i CASILLA!\n",++i);
    start_time = wb_robot_get_time();
  } else {
    sp = SPEED;
  }
  if (i==3){
    sp = 0.0;
  }
  return sp;
}

int main(int argc, char **argv) {
  float sp = SPEED;

  start_time = wb_robot_get_time();
  init();
  while (wb_robot_step(TIME_STEP) != -1) {
    // current_time = wb_robot_get_time();
    // duration = 4*TAMAÑO_CELDA/(RADIO_RUEDA * SPEED);
    // if(current_time >= start_time + duration){
    //   sp = 0.0;
    //   printf("MOVIDO %i CASILLA!\n",++i);
    //   start_time = wb_robot_get_time();
    // } else {
    //   sp = SPEED;
    // }
    // if(i==3){
    //   sp = 0.0;
    // }
    move(left_motor, right_motor, casilla());
    // moverseUnaCasilla(left_motor, right_motor);
  }
  wb_robot_cleanup();
  return 0;
}

// int main(int argc, char **argv) {
//   int done = 0;
//   float sp = SPEED;
//   start_time = wb_robot_get_time();
//   init();
//   while (wb_robot_step(TIME_STEP) != -1) {
//     current_time = wb_robot_get_time();
//     duration = 4*TAMAÑO_CELDA/(RADIO_RUEDA * SPEED);
//     if(current_time > start_time + duration){
//       stop(left_motor, right_motor);
//     } else {
//       printf("TIME: %f\t START: %f\n",current_time,start_time);
//       printf("DURATION: %f\n",duration);
//       fw(left_motor, right_motor);
//     }
//     // moverseUnaCasilla(left_motor, right_motor);
//   }
//   wb_robot_cleanup();
//   return 0;
// }