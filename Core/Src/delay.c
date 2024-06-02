#include "delay.h"
#include "stm32f4xx.h"
	
//��ʱnus
//nusΪҪ��ʱ��us��.	
//nus:0~190887435(���ֵ��2^32/fac_us@fac_us=168)
static uint8_t fac_us = 168;    //������ʱ��Ϊ168M, ������1us��ticks���168��	


void delay_init(u8 SYSCLK)
{
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);       //SysTickƵ��ΪHCLK
	fac_us=SYSCLK;
}								    

void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;	//װ��ֵ	    	 
	ticks=nus*fac_us; //��Ҫ�Ľ����� 
	told=SysTick->VAL; //�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//�������ݼ�
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;	//ʱ�䳬��������ӳٵ�ʱ��ʱ�˳�.
		}  
	};
}

//ms��ʱ
void delay_ms(u16 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}

//������ʱ��ʱ��
void SetTime(tTimeDelay *TimeDelay,uint32_t TimeInter)
{
	TimeDelay->TMStart = HAL_GetTick();
	
	TimeDelay->TMInter = TimeInter;
}

//�Ƚ���ʱ��ʱ���Ƿ񵽴�ﵽ�򷵻��棬���򷵻�0
uint8_t  CompareTime(tTimeDelay *TimeDelay)
{
	return ((HAL_GetTick() - TimeDelay->TMStart) >= TimeDelay->TMInter) ;
}


