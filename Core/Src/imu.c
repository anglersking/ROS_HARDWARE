#include "imu.h"
#include "inv_mpu_user.c"
#include "comminicate.h"
#include "car_task.h"
static void update( Upload_Data *data)
{
    data->Sensor_Str.Link_Accelerometer.X_data=OutMpu.acc_x ;
    data->Sensor_Str.Link_Accelerometer.Y_data=OutMpu.acc_y;
    data->Sensor_Str.Link_Accelerometer.Z_data=OutMpu.acc_z;
    data->Sensor_Str.Link_Gyroscope.X_data=OutMpu.gyro_x;
    data->Sensor_Str.Link_Gyroscope.Y_data=OutMpu.gyro_y;
    data->Sensor_Str.Link_Gyroscope.Z_data=OutMpu.gyro_z;


}


static  void wait_init()
{
    while(mpu_dmp_init());
}
void init()
{
    MPU_Init();
    wait_init();
    
}