#include "math.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "contrl.h"
#include "stdio.h"

 
_Moto_Str Left_moto;
_Moto_Str Right_moto;


int   Dead_Zone=100;    //电机死区
int   control_turn=64;                             //转向控制


//PID调节参数
struct pid_arg PID = {
	.Balance_Kp=200,
	.Balance_Kd=1,
	.Velocity_Kp=-52,
	.Velocity_Ki=-0.26,
	.Turn_Kp = 18,
	.Turn_Kd = 0.18,
};

/**************************************************************************************************************
*函数名:Read_Encoder()
*功能:读取编码器值(当作小车当前前进的速度)
*形参:(uint8_t TIMX):x为编码器1或者2
*返回值:无
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

	Left_moto.Encoder_Value   = Read_Encoder(1);		//读取左右轮子的脉冲累计数
	Right_moto.Encoder_Value  = -Read_Encoder(2);
	
	//计算左右轮子的线性速度，速度 =（（轮子的直径 * 3.14 * （编码器脉冲数 / 轮子一圈积累的脉冲数））/ 采样周期）
	Left_moto.Current_Speed \
		= -((ROBOT_INITIATIVE_DIAMETER *Pi_v * (Left_moto.Encoder_Value  / ENCODER_TTL_COUNT_VALUE))/CONTROL_TIMER_CYCLE);
	Right_moto.Current_Speed\
		= ((ROBOT_INITIATIVE_DIAMETER  *Pi_v * (Right_moto.Encoder_Value / ENCODER_TTL_COUNT_VALUE))/CONTROL_TIMER_CYCLE);
	

}




/**************************************************************************************************************
*函数名:Vertical_Ring_PD()
*功能:直立环PD控制
*形参:(float Angle):x轴的角度/(float Gyro):x轴的角速度
*返回值:经过PID转换之后的PWM值
**************************************************************************************************************/
//直立环的PD


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
*函数名:Vertical_speed_PI()
*功能；速度环PI控制
*形参:(int encoder_left):左轮编码器值/(int encoder_right):编码器右轮的值/(float Angle):x轴角度值
*返回值:
**************************************************************************************************************/

int Vertical_speed_PI(int encoder_left,int encoder_right,float Angle,float Movement )
{
	static float Velocity,Encoder_Least,Encoder;
	static float Encoder_Integral;
	Encoder_Least =(encoder_left+encoder_right)-0;    //获取最新速度偏差=测量速度（左右编码器之和）-目标速度（此处为零）
	Encoder *= 0.8f;																	//一阶低通滤波器 ，上次的速度占85%
	Encoder += Encoder_Least*0.2f;                   //一阶低通滤波器， 本次的速度占15% 
	Encoder_Integral +=Encoder;                       //积分出位移 积分时间：10ms
	Encoder_Integral=Encoder_Integral-Movement; 
	
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;           //积分限幅
	if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //积分限幅

	Velocity=Encoder*PID.Velocity_Kp+Encoder_Integral*PID.Velocity_Ki;      //速度控制
	
	
	if(Turn_off(Angle)==1)   Encoder_Integral=0;            //电机关闭后清除积分
	return Velocity;
}


/**************************************************************************************************************
*函数名:Vertical_turn_PD()
*功能:转向环PD
*形参:无  CCD小于34左转、CCD大于64右转。 yaw = z轴陀螺仪数值
*返回值:无
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
*函数名:PWM_Limiting()
*功能:PWM限幅函数
*形参:无
*返回值:无
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
*函数名:Turn_off()
*功能:关闭电机
*形参:(const float Angle):x轴角度值
*返回值:1:小车当前处于停止状态/0:小车当前处于正常状态
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
*函数名:Set_PWM()
*功能:输出PWM控制电机
*形参；(int motor1):电机1对应的PWM值/(int motor2):电机2对应的PWM值
*返回值:无
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



