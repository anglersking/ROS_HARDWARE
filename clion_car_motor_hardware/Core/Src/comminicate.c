#include "main.h"
#include "comminicate.h"
#include "contrl.h"
Upload_Data Send_Data, Recive_Data;

/*
 @ describetion: Chassis kinematics Positive solution function
 @ param: float vx,float vz
 @ return: none
 @ author: Wanshan Pang
 @ date : 2022-9-15
 @ note: 
 @ function: void Kinematics_Positive(float vx,float vz)
*/

void Kinematics_Positive(float vx,float vz)
{
	if(vx == 0.0f){			//ԭ����ת��ֹ
		Right_moto.Target_Speed = vz * Base_Width / 2.0f;
		Left_moto.Target_Speed  = (-1) * Right_moto.Target_Speed;
	}
	else if(vz == 0.0f){	//��ֹ����ǰ���˶�
		Right_moto.Target_Speed = Left_moto.Target_Speed = vx;
	}	
	else{					//��ǰ�����ߺ��˹�����ת��
		Left_moto.Target_Speed  = vx - vz * Base_Width / 2.0f;
		Right_moto.Target_Speed = vx + vz * Base_Width / 2.0f;
	}
}


/*
 @ describetion:usart send a char data
 @ param: b:data
 @ return: none
 @ author: Wanshan Pang
 @ date : 2022-9-15
 @ note: 
 @ function: void USART1_SendChar(unsigned char b)
*/
int ch;
void USART1_SendChar(int ch)
{
   while( ! (USART1->SR & (1<<7)));  //��ʾ�ȴ�TDRΪ��
	
		USART1->DR = ch;
}


/*
 @ describetion:Send Data To ubuntu pc
 @ param: b:data
 @ return: none
 @ author: Wanshan Pang
 @ date : 2022-9-15
 @ note: 
 @ function: void SendTo_UbuntuPC()
*/
extern int  Encoder_left, Encoder_right;   

void SendTo_UbuntuPC()
{
	unsigned char i = 0;
	Send_Data.Sensor_Str.Header   = PROTOCOL_HEADER;
	Send_Data.Sensor_Str.End_flag = PROTOCOL_END;
	
	Send_Data.Sensor_Str.X_speed = (Left_moto.Current_Speed + Right_moto.Current_Speed)/2.0f;
	Send_Data.Sensor_Str.Y_speed = 0.0;
	Send_Data.Sensor_Str.Z_speed = (Left_moto.Current_Speed - Right_moto.Current_Speed)/Base_Width;
	
	Send_Data.Sensor_Str.Source_Voltage = 100;//Source_Valtage;
//    printf("%f\n",Send_Data.Sensor_Str.Source_Voltage);
	
	for(i=0; i<PROTOCL_DATA_SIZE; i++)
	{
		USART1_SendChar(Send_Data.buffer[i]);
	}
}
