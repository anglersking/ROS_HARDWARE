#include "math.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "contrl.h"
#include "stdio.h"

 
_Moto_Str Left_moto;
_Moto_Str Right_moto;


int   Dead_Zone=100;    //motor dead zone
int   control_turn=64;                             //turning control


//PID tuning parameters
struct pid_arg PID = {
	.Balance_Kp=200,
	.Balance_Kd=1,
	.Velocity_Kp=-52,
	.Velocity_Ki=-0.26,
	.Turn_Kp = 18,
	.Turn_Kd = 0.18,
};

/**************************************************************************************************************
*Function: Read_Encoder()
*Purpose : read encoder counter (used as the current forward speed)
*Args    : (uint8_t TIMX) x = encoder 1 or 2
*Return  : none
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

	Left_moto.Encoder_Value   = Read_Encoder(1);		//read the accumulated pulse count of the left/right wheel
	Right_moto.Encoder_Value  = -Read_Encoder(2);
	
	//linear wheel speed = (wheel diameter * pi * (pulses / pulses per revolution)) / sample period
	Left_moto.Current_Speed \
		= -((ROBOT_INITIATIVE_DIAMETER *Pi_v * (Left_moto.Encoder_Value  / ENCODER_TTL_COUNT_VALUE))/CONTROL_TIMER_CYCLE);
	Right_moto.Current_Speed\
		= ((ROBOT_INITIATIVE_DIAMETER  *Pi_v * (Right_moto.Encoder_Value / ENCODER_TTL_COUNT_VALUE))/CONTROL_TIMER_CYCLE);
	

}




/**************************************************************************************************************
*Function: Vertical_Ring_PD()
*Purpose : upright / balance loop PD control
*Args    : (float Angle) x-axis angle / (float Gyro) x-axis angular rate
*Return  : PWM value after the PD loop
**************************************************************************************************************/
//balance loop PD


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
*Function: Vertical_speed_PI()
*Purpose : speed loop PI control
*Args    : (int encoder_left) left encoder / (int encoder_right) right encoder / (float Angle) x-axis angle
*Return  : none
**************************************************************************************************************/

int Vertical_speed_PI(int encoder_left,int encoder_right,float Angle,float Movement )
{
	static float Velocity,Encoder_Least,Encoder;
	static float Encoder_Integral;
	Encoder_Least =(encoder_left+encoder_right)-0;    //latest speed error = measured speed (sum of both encoders) - target speed (0 here)
	Encoder *= 0.8f;																	//1st-order low-pass filter, previous speed weighted 85%
	Encoder += Encoder_Least*0.2f;                   //1st-order low-pass filter, current speed weighted 15% 
	Encoder_Integral +=Encoder;                       //integrate to displacement (integral time 10ms)
	Encoder_Integral=Encoder_Integral-Movement; 
	
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;           //clamp integral
	if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //clamp integral

	Velocity=Encoder*PID.Velocity_Kp+Encoder_Integral*PID.Velocity_Ki;      //speed control
	
	
	if(Turn_off(Angle)==1)   Encoder_Integral=0;            //clear the integral after the motor is switched off
	return Velocity;
}


/**************************************************************************************************************
*Function: Vertical_turn_PD()
*Purpose : steering / turn loop PD
*Args    : none. CCD<34 turn left, CCD>64 turn right. yaw = z-axis gyro
*Return  : none
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
*Function: PWM_Limiting()
*Purpose : PWM limiting
*Args    : none
*Return  : none
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
*Function: Turn_off()
*Purpose : stop the motor
*Args    : (const float Angle) x-axis angle
*Return  : 1 = car is currently stopped / 0 = car is normal
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
	{
		temp=0;
	}
		FS_state=0;
	return temp;
}

/**************************************************************************************************************
*Function: Set_PWM()
*Purpose : output PWM to drive the motors
*Args    : (int motor1) PWM command of motor group 1 / (int motor2) PWM command of motor group 2
*Return  : none
*************************************************************************************************************/


void Set_PWM(int motor1,int motor2)
{
	uint32_t period = TIM3->ARR;
	uint32_t duty1 = 0, duty2 = 0;
	int magnitude;

	//left side: CONVERT1/CONVERT2 ; right side: CONVERT3/CONVERT4.
	//the two sides use opposite levels, so the same sign means "go straight".
	if(motor1 > 0)			AIN1(1), AIN2(1);
	else					AIN1(0), AIN2(0);

	if(motor1 != 0)
	{
		magnitude = abs(motor1);
		if(magnitude > MOTOR_PWM_ABS_MAX) magnitude = MOTOR_PWM_ABS_MAX;
		duty1 = (uint32_t)Dead_Zone
		      + ((period - (uint32_t)Dead_Zone) * (uint32_t)magnitude) / MOTOR_PWM_ABS_MAX;
		if(duty1 > period) duty1 = period;
	}
	PWMA = duty1;
	PWMC = duty1;

	if(motor2 > 0)			BIN1(0), BIN2(0);
	else					BIN1(1), BIN2(1);

	if(motor2 != 0)
	{
		magnitude = abs(motor2);
		if(magnitude > MOTOR_PWM_ABS_MAX) magnitude = MOTOR_PWM_ABS_MAX;
		duty2 = (uint32_t)Dead_Zone
		      + ((period - (uint32_t)Dead_Zone) * (uint32_t)magnitude) / MOTOR_PWM_ABS_MAX;
		if(duty2 > period) duty2 = period;
	}
	PWMB = duty2;
	PWMD = duty2;
}


