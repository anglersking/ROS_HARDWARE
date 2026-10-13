#ifndef _CONTRIL_H_
#define _CONTRIL_H_

//#include "sys.h"
#include "stm32f4xx_hal.h"


// Mechanical balance zero point
#define Mechanical_balance 0

// Left-side direction pins (new hub: CONVERT1 / CONVERT2)
#define AIN1(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_8, (GPIO_PinState)PinState)
#define AIN2(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_9, (GPIO_PinState)PinState)
//#define AIN3(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_12, (GPIO_PinState)PinState)
//#define AIN4(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_7, (GPIO_PinState)PinState)

//
//#define BIN1(PinState)    HAL_GPIO_WritePin( GPIOC, GPIO_PIN_3, (GPIO_PinState)PinState)
//#define BIN2(PinState)    HAL_GPIO_WritePin( GPIOA, GPIO_PIN_3, (GPIO_PinState)PinState)
#define BIN1(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_12, (GPIO_PinState)PinState)
#define BIN2(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_7, (GPIO_PinState)PinState)
//#define BIN3(PinState)    HAL_GPIO_WritePin( GPIOC, GPIO_PIN_3, (GPIO_PinState)PinState)
//#define BIN4(PinState)    HAL_GPIO_WritePin( GPIOA, GPIO_PIN_3, (GPIO_PinState)PinState)


// Brake pins (new hub: BREAK1..BREAK4)
#define BK1(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_14, (GPIO_PinState)PinState)
#define BK2(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_15, (GPIO_PinState)PinState)

#define BK3(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_10, (GPIO_PinState)PinState)

#define BK4(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_13, (GPIO_PinState)PinState)


// PWM outputs: all four PWM channels come from TIM3 (CH1..CH4 -> PC6..PC9).
// NOTE: the 4th channel is TIM3_CH4 (PC9), NOT TIM4.
#define PWMA   TIM3->CCR1
#define PWMB   TIM3->CCR2

#define PWMC   TIM3->CCR3
#define PWMD   TIM3->CCR4

// Open-loop PWM calibration. Set_PWM() takes -MOTOR_PWM_ABS_MAX..+MOTOR_PWM_ABS_MAX
// and normalises the duty cycle to TIM3->ARR.
#define MOTOR_PWM_ABS_MAX                 950    // keep in sync with ESC_output_PWM_LIMT
#define MOTOR_OPEN_LOOP_MIN_PWM           320    // startup floor: lets low speed overcome static friction
#define MOTOR_OPEN_LOOP_SPEED_FULL_SCALE  0.35f  // target speed (m/s) that maps to full duty

/***********robot***********/

#define ENCODER_TTL_COUNT_VALUE	   	933.0f
#define ROBOT_INITIATIVE_DIAMETER	0.067f	// wheel diameter (m)
#define CONTROL_TIMER_CYCLE			0.1f	
#define Pi_v						3.1415f
#define Base_Width					0.175f


#define MOTO_LEFT		2
#define MOTO_RIGHT		3
typedef struct _Moto_
{
	int Encoder_Value;
	float Current_Speed;
	float Target_Speed;
	short ESC_Output_PWM;
	float L_Error;
	float LL_Error;
}_Moto_Str;


extern _Moto_Str Left_moto;
extern _Moto_Str Right_moto;
void  Robot_Encoder_Get_CNT(void);

/**********Balance**********/

extern volatile int Encoder_Left,Encoder_Right;		      // left/right wheel speed from the encoder




struct pid_arg{
	
	float Balance_Kp;
	float Balance_Ki;
	float Balance_Kd;
	
	float Velocity_Kp;
	float Velocity_Ki;
	float Velocity_Kd;
	
	float  Turn_Kp;
	float  Turn_Ki;
	float  Turn_Kd;

};
extern struct pid_arg PID;

int Read_Encoder(uint8_t TIMX);
int	Vertical_Ring_PD(float Angle,float Gyro);
int Vertical_speed_PI(int encoder_left,int encoder_right,float Angle,float Movement );
int Vertical_turn_PD(uint8_t CCD,short yaw);


void PWM_Limiting(int *motor1,int *motor2);
uint8_t Turn_off(const float Angle);
void Set_PWM(int motor1,int motor2);


#endif
