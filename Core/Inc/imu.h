#ifndef _IMU_
#define _IMU_
#include "mpu6050.h"
#include "comminicate.h"
#include "car_task.h"
typedef struct imu{
    int data[100];
    int tail;
    void (*update)(Upload_Data *data);
    void (*wait_init)();
    void (*init)();
}IMU;



#endif