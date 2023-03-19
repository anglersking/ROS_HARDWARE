#include "car_task.h"
#include "mpu6050.h"
#include "inv_mpu_user.h"
#include "contrl.h"
#include "ROS_USART_DEVICE.h"

extern Upload_Data Send_Data, Recive_Data;


int  Balance_Pwm,Velocity_Pwm,Turn_Pwm;        //PID计算的PWM值
int  Motor1, Motor2;                  //左右电机PWM值
int  Encoder_left, Encoder_right;     //检测速度
float Movement = 0;                   //速度调节  
int  Contrl_Turn = 64;                //转向调节变量

//环境数据采集任务
void Car_Task_200HZ(void)
{
		static struct mpu6050_data Last_Data;
	
		if(mpu_dmp_get_data() !=0 )
			OutMpu = Last_Data;
		else
			 Last_Data = OutMpu;
		
		
		Send_Data.Sensor_Str.Link_Accelerometer.X_data=OutMpu.acc_x;
		Send_Data.Sensor_Str.Link_Accelerometer.Y_data=OutMpu.acc_y;
		Send_Data.Sensor_Str.Link_Accelerometer.Z_data=OutMpu.acc_z;
		Send_Data.Sensor_Str.Link_Gyroscope.X_data=OutMpu.gyro_x;
		Send_Data.Sensor_Str.Link_Gyroscope.Y_data=OutMpu.gyro_y;
		Send_Data.Sensor_Str.Link_Gyroscope.Z_data=OutMpu.gyro_z;
		
		
		Robot_Encoder_Get_CNT();
		/*
		MPU_Get_Accelerometer(&Send_Data.Sensor_Str.Link_Accelerometer);	//通过IIC读取加速度信息
		MPU_Get_Gyroscope(&Send_Data.Sensor_Str.Link_Gyroscope);			//通过IIC读取角速度信息
		
		*/
		
		
		
			
}

void Car_Task_100HZ(void)
{
	Encoder_left  = Read_Encoder(1);
	Encoder_right = -Read_Encoder(2);
	
	//1、确定直立环PWM
	
		Balance_Pwm = Vertical_Ring_PD(OutMpu.pitch, OutMpu.gyro_x);
	
	//2、确定速度环PWM
	
	  Velocity_Pwm = Vertical_speed_PI(Encoder_left,Encoder_right,OutMpu.pitch, Movement );
	
	
	//3、确定转向环PWM
	
		Turn_Pwm = Vertical_turn_PD(Contrl_Turn, OutMpu.gyro_z);
	
	//4、确定最终左右电机的PWM
	  printf("Balance_Pwm:%d,Velocity_Pwm:%d,Turn_Pwm:%d",Balance_Pwm,Velocity_Pwm,Turn_Pwm);
		Motor1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
	  Motor2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm;
	
		PWM_Limiting(&Motor1,&Motor2);
	
	
	//5、设置电机
		Set_PWM(Motor1,Motor2);
	
}


#define KP   225.9f 
#define KI   165.0f 
#define KD   110.0f		


/*
 @ describetion: Moto speed PID control function
 @ param: float current_speed,float target_speed, unsigned char Moto_ID
 @ return: none
 @ author: Wanshan Pang
 @ date : 2022-9-16
 @ function : void Moto_Control_speed(float current_speed,float target_speed, unsigned char Moto_ID)
*/
//#define _Debug_LineShow_
void Moto_Control_speed(float current_speed,float target_speed, unsigned char Moto_ID)
{
	float Error   = 0;
	float P_Error = 0;
	float I_Error = 0;
	float D_Error = 0;
	float add     = 0;
	
	if(Moto_ID == MOTO_LEFT)
	{
		Error = target_speed - current_speed;
		//Update the current one-two third-order error for the current proportional integral and differential calculation
		P_Error = Error;
		I_Error = Error - Left_moto.L_Error;
		D_Error = Error - 2*Left_moto.L_Error + Left_moto.LL_Error;
		
		//calculation current proportional integral and differential 
		add = KP * P_Error + KI * I_Error + KD * D_Error;
		Left_moto.ESC_Output_PWM += add;
		
		Left_moto.LL_Error = Left_moto.L_Error;
		Left_moto.L_Error = Error;
		
		if(Left_moto.ESC_Output_PWM > ESC_output_PWM_LIMT)	Left_moto.ESC_Output_PWM = ESC_output_PWM_LIMT;		
		else if(Left_moto.ESC_Output_PWM < -ESC_output_PWM_LIMT)	Left_moto.ESC_Output_PWM = -ESC_output_PWM_LIMT;
	}
	else if(Moto_ID == MOTO_RIGHT)
	{
		Error = target_speed - current_speed;
		//Update the current one-two third-order error for the current proportional integral and differential calculation
		P_Error = Error;
		I_Error = Error - Right_moto.L_Error;
		D_Error = Error - 2*Right_moto.L_Error + Right_moto.LL_Error;

		//calculation current proportional integral and differential 
		add = KP * P_Error + KI * I_Error + KD * D_Error;
		Right_moto.ESC_Output_PWM += add;
		
		Right_moto.LL_Error = Right_moto.L_Error;
		Right_moto.L_Error = Error;
		
		if(Right_moto.ESC_Output_PWM > ESC_output_PWM_LIMT)	Right_moto.ESC_Output_PWM = ESC_output_PWM_LIMT;		
		else if(Right_moto.ESC_Output_PWM < -ESC_output_PWM_LIMT) Right_moto.ESC_Output_PWM = -ESC_output_PWM_LIMT;
		
		#ifdef _Debug_LineShow_
		{	
			send_data[0] = current_speed*1000;
			send_data[1] = target_speed*1000;
			send_data[2] = add;
			send_data[3] = Right_moto.ESC_Output_PWM;
			
			shanwai_send_data1((uint8_t*)&send_data,sizeof(send_data));
		}
		#endif
	}
	// moto control function
	//PWM_Output(Left_moto.ESC_Output_PWM, Right_moto.ESC_Output_PWM);
	Set_PWM(Left_moto.ESC_Output_PWM, Right_moto.ESC_Output_PWM);
	//Huanyu_PWM_Output(500, 500);
}


void Car_Task_5HZ(void)
{
		printf("acc_x = %d\n",OutMpu.acc_x);
		printf("acc_y = %d\n",OutMpu.acc_y);
		printf("acc_z = %d\n",OutMpu.acc_z);
		printf("gyro_x = %d\n",OutMpu.gyro_x);
		printf("gyro_y = %d\n",OutMpu.gyro_y);
		printf("gyro_z = %d\n",OutMpu.gyro_z);
	  printf("pitch = %f\n",OutMpu.pitch);
	  printf("roll = %f\n",OutMpu.roll);
	  printf("yaw = %f\n",OutMpu.yaw);
	
	  printf("\r\n");
	  SendTo_UbuntuPC();		
}





