#include "car_task.h"
#include "mpu6050.h"

#include "contrl.h"
#include "comminicate.h"
#include "usart.h"
#include "inv_mpu_user.h"
#include "imu.h"
extern Upload_Data Send_Data, Recive_Data;


int  Balance_Pwm,Velocity_Pwm,Turn_Pwm;        //PID�����PWMֵ
int  Motor1, Motor2;                  //���ҵ��PWMֵ
int  Encoder_left, Encoder_right;     //����ٶ�
float Movement = 0;                   //�ٶȵ���  
int  Contrl_Turn = 64;                //ת����ڱ���


void Car_Task_200HZ(void)
{
		static struct mpu6050_data Last_Data;

		if(mpu_dmp_get_data() !=0 )
			OutMpu = Last_Data;
		else
			 Last_Data = OutMpu;
#ifdef DEBUG_Car_Task_200HZ
        HAL_UART_Transmit(&huart1,(uint8_t *) "IMU TASK\n", 9, 100);
        HAL_UART_Transmit(&huart1,(uint8_t *) mpu_dmp_get_data(), 2, 100);
#endif
#ifdef USE_ENCODER
    		Robot_Encoder_Get_CNT();
#endif


#ifdef OLD_METHOD
        MPU_Get_Accelerometer(&Send_Data.Sensor_Str.Link_Accelerometer);	//ͨ��IIC��ȡ���ٶ���Ϣ
		MPU_Get_Gyroscope(&Send_Data.Sensor_Str.Link_Gyroscope);			//ͨ��IIC��ȡ���ٶ���Ϣ
        MPU_Get_Accelerometer(&OutMpu.acc_x,&OutMpu.acc_y,&OutMpu.acc_z);	//ͨ��IIC��ȡ���ٶ���Ϣ
        MPU_Get_Gyroscope(&Send_Data.Sensor_Str.Link_Gyroscope);			//ͨ��IIC��ȡ���ٶ���Ϣ
        HAL_UART_Transmit(&huart1,(uint8_t *) OutMpu.acc_x, 8, 100);
#endif

//        IMU imu;
//        imu.update(&Send_Data);
        Send_Data.Sensor_Str.Link_Accelerometer.X_data=OutMpu.acc_x ;
        Send_Data.Sensor_Str.Link_Accelerometer.Y_data=OutMpu.acc_y;
        Send_Data.Sensor_Str.Link_Accelerometer.Z_data=OutMpu.acc_z;
        Send_Data.Sensor_Str.Link_Gyroscope.X_data=OutMpu.gyro_x;
        Send_Data.Sensor_Str.Link_Gyroscope.Y_data=OutMpu.gyro_y;
        Send_Data.Sensor_Str.Link_Gyroscope.Z_data=OutMpu.gyro_z;



}

void Car_Task_100HZ(void)
{


    Left_moto.Current_Speed = Left_moto.Target_Speed;
    Right_moto.Current_Speed = Right_moto.Target_Speed ;
    Moto_Control_speed(Right_moto.Current_Speed, Right_moto.Target_Speed ,MOTO_RIGHT);
    Moto_Control_speed(Left_moto.Current_Speed,  Left_moto.Target_Speed  ,MOTO_LEFT );
#ifdef DEBUG_CAR_TASK_100HZ
    HAL_UART_Transmit(&huart1,(uint8_t *) "control Task\n", 13, 100);

#endif
#ifdef USE_PID


	Encoder_left  = Read_Encoder(1);
	Encoder_right = -Read_Encoder(2);

	//1��ȷ��ֱ����PWM

    Balance_Pwm = 0;
    Vertical_Ring_PD(OutMpu.pitch, OutMpu.gyro_x);

	//2��ȷ���ٶȻ�PWM

    Velocity_Pwm =0;
    Vertical_speed_PI(Encoder_left,Encoder_right,OutMpu.pitch, Movement );


	//3��ȷ��ת��PWM

    Turn_Pwm = 0;
    Vertical_turn_PD(Contrl_Turn, OutMpu.gyro_z);

	//4��ȷ���������ҵ����PWM
    Motor1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
    Motor2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm;

    PWM_Limiting(&Motor1,&Motor2);

	//5�����õ��
    Set_PWM(Motor1,Motor2);
#endif
	
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
//	PWM_Output(Left_moto.ESC_Output_PWM, Right_moto.ESC_Output_PWM);
    PWM_Limiting(&Left_moto.ESC_Output_PWM,&Right_moto.ESC_Output_PWM);
	Set_PWM(Left_moto.ESC_Output_PWM, Right_moto.ESC_Output_PWM);

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





