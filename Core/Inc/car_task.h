#ifndef __CAR_TASK_H
#define __CAR_TASK_H

struct mpu6050_data{
	
		short acc_x;
		short acc_y;
		short acc_z;
		
		short gyro_x;
		short gyro_y;
		short gyro_z;
	
		float pitch;    //������
	  float roll;     //������
	  float yaw;      //ƫ����
};

extern struct mpu6050_data OutMpu;


#define ESC_output_PWM_LIMT			 	950
#define MOTO_DEAD_TIMER_COMPENSATION 	200


void Moto_Control_speed(float current_speed,float target_speed, unsigned char Moto_ID);
void Car_Task_200HZ(void);
void Car_Task_100HZ(void);
void Car_Task_5HZ(void);

#endif

