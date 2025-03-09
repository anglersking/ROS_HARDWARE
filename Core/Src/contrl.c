#include "math.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "contrl.h"
#include "stdio.h"

 
_Moto_Str Left_moto;
_Moto_Str Right_moto;


int   Dead_Zone=100;    //�������
int   control_turn=64;                             //ת�����


//PID���ڲ���
struct pid_arg PID = {
	.Balance_Kp=200,
	.Balance_Kd=1,
	.Velocity_Kp=-52,
	.Velocity_Ki=-0.26,
	.Turn_Kp = 18,
	.Turn_Kd = 0.18,
};

/**************************************************************************************************************
*������:Read_Encoder()
*����:��ȡ������ֵ(����С����ǰǰ�����ٶ�)
*�β�:(uint8_t TIMX):xΪ������1����2
*����ֵ:��
*************************************************************************************************************/
int Read_Encoder(uint8_t TIMX)
{
    int Encoder_TIM;  
		
   switch(TIMX)
	 {
	   case 1:  Encoder_TIM= (short)TIM1 -> CNT;  TIM1 -> CNT=0;break;
		 case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
		 default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}



void  Robot_Encoder_Get_CNT(void)
{

	Left_moto.Encoder_Value   = Read_Encoder(1);		//��ȡ�������ӵ������ۼ���
	Right_moto.Encoder_Value  = -Read_Encoder(2);
	
	//�����������ӵ������ٶȣ��ٶ� =�������ӵ�ֱ�� * 3.14 * �������������� / ����һȦ���۵�����������/ �������ڣ�
	Left_moto.Current_Speed \
		= -((ROBOT_INITIATIVE_DIAMETER *Pi_v * (Left_moto.Encoder_Value  / ENCODER_TTL_COUNT_VALUE))/CONTROL_TIMER_CYCLE);
	Right_moto.Current_Speed\
		= ((ROBOT_INITIATIVE_DIAMETER  *Pi_v * (Right_moto.Encoder_Value / ENCODER_TTL_COUNT_VALUE))/CONTROL_TIMER_CYCLE);
	

}




/**************************************************************************************************************
*������:Vertical_Ring_PD()
*����:ֱ����PD����
*�β�:(float Angle):x��ĽǶ�/(float Gyro):x��Ľ��ٶ�
*����ֵ:����PIDת��֮���PWMֵ
**************************************************************************************************************/
//ֱ������PD


int	Vertical_Ring_PD(float Angle,float Gyro)
{
	 float Bias;
	 int balance;
   Bias=Angle-Mechanical_balance;
   balance=PID.Balance_Kp*Bias+ Gyro*PID.Balance_Kd;
	
	return balance;
		
	 //printf("balance = %f\n",balance);
}


/**************************************************************************************************************
*������:Vertical_speed_PI()
*���ܣ��ٶȻ�PI����
*�β�:(int encoder_left):���ֱ�����ֵ/(int encoder_right):���������ֵ�ֵ/(float Angle):x��Ƕ�ֵ
*����ֵ:
**************************************************************************************************************/

int Vertical_speed_PI(int encoder_left,int encoder_right,float Angle,float Movement )
{
	static float Velocity,Encoder_Least,Encoder;
	static float Encoder_Integral;
	Encoder_Least =(encoder_left+encoder_right)-0;    //��ȡ�����ٶ�ƫ��=�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩
	Encoder *= 0.8f;																	//һ�׵�ͨ�˲��� ���ϴε��ٶ�ռ85%
	Encoder += Encoder_Least*0.2f;                   //һ�׵�ͨ�˲����� ���ε��ٶ�ռ15% 
	Encoder_Integral +=Encoder;                       //���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_Integral=Encoder_Integral-Movement; 
	
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;           //�����޷�
	if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //�����޷�

	Velocity=Encoder*PID.Velocity_Kp+Encoder_Integral*PID.Velocity_Ki;      //�ٶȿ���
	
	
	if(Turn_off(Angle)==1)   Encoder_Integral=0;            //����رպ��������
	return Velocity;
}


/**************************************************************************************************************
*������:Vertical_turn_PD()
*����:ת��PD
*�β�:��  CCDС��34��ת��CCD����64��ת�� yaw = z����������ֵ
*����ֵ:��
***************************************************************************************************************/
int Vertical_turn_PD(uint8_t CCD,short yaw)
{
		float Turn;     
    float Bias;	  
	  Bias=CCD-64;
	  Turn=-Bias*PID.Turn_Kp-yaw*PID.Turn_Kd;
	  return Turn;
}



/**************************************************************************************************************
*������:PWM_Limiting()
*����:PWM�޷�����
*�β�:��
*����ֵ:��
***************************************************************************************************************/
void PWM_Limiting(int *motor1,int *motor2)
{
	int Amplitude=5800;
	if(*motor1<-Amplitude) *motor1=-Amplitude;	
	if(*motor1>Amplitude)  *motor1=Amplitude;	
	if(*motor2<-Amplitude) *motor2=-Amplitude;	
	if(*motor2>Amplitude)  *motor2=Amplitude;		
}


/**************************************************************************************************************
*������:Turn_off()
*����:�رյ��
*�β�:(const float Angle):x��Ƕ�ֵ
*����ֵ:1:С����ǰ����ֹͣ״̬/0:С����ǰ��������״̬
***************************************************************************************************************/
uint8_t FS_state;

uint8_t Turn_off(const float Angle)
{
	uint8_t temp;
	if(fabs(Angle)>80){
		FS_state=1;
		temp=1;
		AIN2(0),			AIN1(0);
		BIN1(0),			BIN2(0);
	}
	else 
		temp=0;
		FS_state=0;
	return temp;
}

/**************************************************************************************************************
*������:Set_PWM()
*����:���PWM���Ƶ��
*�βΣ�(int motor1):���1��Ӧ��PWMֵ/(int motor2):���2��Ӧ��PWMֵ
*����ֵ:��
*************************************************************************************************************/


void Set_PWM(int motor1,int motor2)
{
	if(motor1>0)			AIN1(1),			AIN2(1);
	else 	          	AIN1(0),			AIN2(0);
	PWMA=Dead_Zone+(abs(motor1))*1.17;
    PWMC=Dead_Zone+(abs(motor1))*1.17;

	if(motor2>0)			BIN1(0),			BIN2(0);
	else       		 		BIN1(1),			BIN2(1);
	PWMB=Dead_Zone+(abs(motor2))*1.17;
    PWMD=Dead_Zone+(abs(motor2))*1.17;


//	printf("PWMA = %d\n",PWMA);
//  printf("PWMB = %d\n",PWMB);
}



