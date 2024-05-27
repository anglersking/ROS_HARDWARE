#include "car_task.h"
//#include "mpu6050.h"
//#include "inv_mpu_user.h"
#include "contrl.h"
#include "comminicate.h"

extern Upload_Data Send_Data, Recive_Data;


int  Balance_Pwm,Velocity_Pwm,Turn_Pwm;        //PID�����PWMֵ
int  Motor1, Motor2;                  //���ҵ��PWMֵ
int  Encoder_left, Encoder_right;     //����ٶ�
float Movement = 0;                   //�ٶȵ���  
int  Contrl_Turn = 64;                //ת����ڱ���

//�������ݲɼ�����
void Car_Task_200HZ(void)
{
//		static struct mpu6050_data Last_Data;
//

//		Send_Data.Sensor_Str.Link_Accelerometer.X_data=OutMpu.acc_x;
//		Send_Data.Sensor_Str.Link_Accelerometer.Y_data=OutMpu.acc_y;
//		Send_Data.Sensor_Str.Link_Accelerometer.Z_data=OutMpu.acc_z;
//		Send_Data.Sensor_Str.Link_Gyroscope.X_data=OutMpu.gyro_x;
//		Send_Data.Sensor_Str.Link_Gyroscope.Y_data=OutMpu.gyro_y;
//		Send_Data.Sensor_Str.Link_Gyroscope.Z_data=OutMpu.gyro_z;
		
		
		Robot_Encoder_Get_CNT();
		/*
		MPU_Get_Accelerometer(&Send_Data.Sensor_Str.Link_Accelerometer);	//ͨ��IIC��ȡ���ٶ���Ϣ
		MPU_Get_Gyroscope(&Send_Data.Sensor_Str.Link_Gyroscope);			//ͨ��IIC��ȡ���ٶ���Ϣ
		
		*/
		
		
		
			
}

void Car_Task_100HZ(void)
{
	Encoder_left  = Read_Encoder(1);
	Encoder_right = -Read_Encoder(2);
	
	//1��ȷ��ֱ����PWM
	
		Balance_Pwm = 0;
                //Vertical_Ring_PD(OutMpu.pitch, OutMpu.gyro_x);
	
	//2��ȷ���ٶȻ�PWM
	
	  Velocity_Pwm =0;
              //Vertical_speed_PI(Encoder_left,Encoder_right,OutMpu.pitch, Movement );
	
	
	//3��ȷ��ת��PWM
	
		Turn_Pwm = 0;
                //Vertical_turn_PD(Contrl_Turn, OutMpu.gyro_z);
	
	//4��ȷ���������ҵ����PWM
//	  printf("Balance_Pwm:%d,Velocity_Pwm:%d,Turn_Pwm:%d",Balance_Pwm,Velocity_Pwm,Turn_Pwm);
      Motor1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
	  Motor2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm;
	
      PWM_Limiting(&Motor1,&Motor2);
	
	
	//5�����õ��
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
	  SendTo_UbuntuPC();		
}





