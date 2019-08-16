/*
  ******************************************************************************
  *project��  	MOTOR_BASE	V2.1
  *developer��	XYC
  *data��     	2018-08-06
  *CPU��      	STM8S103F3 SOP-20  
  *version:		BASE(������)
  *comment�� 	 ���Ļ����Ƴ���,�ɼ���λ����ѹ�������Ӧռ�ձ�PWM
  ******************************************************************************
*/ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_it.h"
#include "extern.h"
#include "timer.h"
#include "uart1.h"
//#include "config.h"
#include "adc.h"
#include "eeprom.h"
#include "check_uid.h"


#define ID_Check
#define Temp_Protect

TimeCycleBit TimeCycle;
User_Value SysUser_Value;


WORD Duty = 0;
WORD Real_val = 0;
int ADC_Val = 0;
WORD ADCClose_Val = 0;
int err = 0,err1 = 0;


void GPIO_Config(void)
{
	TEMP_PinInit;
	LED_PinInit;
	LED_OFF;
	M_PWM_PinInit;
	M_PWM_H;
	ANLOG_PinInit;
	SW_PinInit;
}



//ϵͳ��ʼ��
void System_Init(void)
{
	//ʹ���ڲ�����16M������1��Ƶ
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);	
	Delay_ms(20);
	
	GPIO_Config();
        //PWM��ʼ��
	Tim2_PWM_Init();

        //TIM��ʼ��
	Tim4_Tim_Init();

	//��ʼ������
#ifdef UART_DEBUG
	Uart1_Init();
#endif
	//EEPROM��ʼ��
//	Flash_eeprom_readwrite_Init();
	//�����û�����
//	Load_UserDataFromFlash();
        //�������ж�
	enableInterrupts();  

//�˲��λ��λ��
	Check_SetRes();
	
	UserChipID_Data.ID_RightFlag = TRUE;
	SysUser_Value.CtrlFlag_TempLock = FALSE;
}     

//������
int main(void)
{
	//ϵͳ��ʼ��
	System_Init();	
	//��ʼ���û�����
	memset(&SysUser_Value,0,sizeof(User_Value));
	memset(&TimeCycle,0,sizeof(TimeCycleBit));

	while (1)
	{  
		Pro_Task();
	}
}



void Pro_Task(void)
{
	SW_ONOFF();
	if(SysUser_Value.Run_OK == TRUE)
	{
		PWM_Control();
		if(TimeCycle.T1s == TRUE)
		{
			TimeCycle.T1s = FALSE;
			LED_TURN;
		}
	}
	else
	{
		LED_OFF;
		ADC_Val = 0;
		Duty = 0;
		Real_val = 0;

		TIM2_SetCompare3(0);
		if(TimeCycle.T200ms == TRUE)
		{
			TimeCycle.T200ms = FALSE;
			ADCClose_Val = GET_ADC_Val();
		}  
	}
//IDУ��	
#ifdef ID_Check
	if(TimeCycle.TCheckID == TRUE)
	{
		TimeCycle.TCheckID = FALSE;
		ChipIDCheck_Task();
	}
#endif

//�¶ȱ���
#ifdef Temp_Protect
	TempProtect_Task();
#endif


}

void PWM_Control(void)
{
	if(UserChipID_Data.ID_RightFlag == TRUE)		//ID�������С��ID_MAX_ERR_CNT / 2������Ϊ����ȷ
	{
		if(TimeCycle.T200ms == TRUE)
		{
			TimeCycle.T200ms = FALSE;
			
			ADC_Val = (int)GET_ADC_Val() - (int)ADCClose_Val ;
			if(ADC_Val <= 0)
				ADC_Val = 0;
			Duty =(WORD)( (DWORD)ADC_Val  * 2000 / 865 );
			if(Duty > PWM_DUTY_CNT) 
				Duty = PWM_DUTY_CNT;
		}

		if(TimeCycle.T20ms == TRUE)			//10ms����Ϊ2��100msΪ20 = 10%Duty
		{
			TimeCycle.T20ms = FALSE;
			if(Real_val < Duty)
			{
				Real_val += 2;
			}
			else if(Real_val > Duty)
			{
				if(Real_val > 3)
					Real_val -= 3;
				else
					Real_val = 0;
			}
			else
			{
				Real_val = Duty;
			}

			if(Real_val > PWM_DUTY_CNT)
				Real_val = PWM_DUTY_CNT;
			
			TIM2_SetCompare3(Real_val);
		}
	}
	else	//ID����ȡ�������������ֱ�����
	{
		ADC_Val = (int)GET_ADC_Val() - (int)ADCClose_Val ;
		if(ADC_Val <= 0)
			ADC_Val = 0;
		Duty =(WORD)( (DWORD)ADC_Val  * 2000 / 865);

		if(Duty > PWM_DUTY_CNT) 
			Duty = PWM_DUTY_CNT;
		TIM2_SetCompare3(Duty);
	}
}


void SW_ONOFF(void)
{
	if(GET_SW == RESET)
	{
		Delay_ms(10);
		if(GET_SW == RESET)
		{
			SysUser_Value.Run_OK = TRUE;
		}
		else
			SysUser_Value.Run_OK = FALSE;
	}
	else
		SysUser_Value.Run_OK = FALSE;
}

//�¶ȱ�������
void TempProtect_Task(void)
{
	if(SysUser_Value.CtrlFlag_TempLock == TRUE)	//����
	{	
		ADC_Val = 0;
		Duty = 0;
		Real_val = 0;
		TIM2_SetCompare3(0);				//���ͣת
		
		while(1)
		{
			SW_ONOFF();
			if(TimeCycle.T100ms == TRUE)
			{
				TimeCycle.T100ms = FALSE;
				LED_TURN;
			}

			if(SysUser_Value.Run_OK == FALSE  &&  SysUser_Value.CtrlFlag_TempLock == FALSE)	
			{
				LED_OFF;
				break;
			}
		}
	}		
}

//�¶ȱ���(��ת����)��⣬ִ��Ƶ��100ms
void Temp_Check(void)
{
	static BYTE cnt = 0;

	if(GET_TEMPLIMT == RESET)
	{
		cnt = 0;
		SysUser_Value.CtrlFlag_TempLock = FALSE;
	}
	else
	{
		if(cnt < 3)
			cnt++;
		else
		{
			cnt = 0;
			SysUser_Value.CtrlFlag_TempLock = TRUE;
		}		
	}
}

//����λ��λ��
void Check_SetRes(void)
{
	while(1)				//�ϵ������λ�����ش��ڹر�λ������������
	{	
		SW_ONOFF();
		Delay_ms(100);
		
		if(SysUser_Value.Run_OK == FALSE)	//��λ�����عرպ�ʼ��������������
			break;
	}
//��ȡ��λ����С��ѹֵ
	ADCClose_Val = GET_ADC_Val();	
}



//PID����
WORD Val_Near(WORD target_value, WORD now_value)
{
	int add_val = 0;
	WORD new_val = 0;

	err = (int)(target_value - now_value);
	add_val =(int) (KP*(err - err1)/2 + KI*err);
	new_val = (WORD)(now_value + add_val);

 	err1 = err;
	
 	return new_val;
}
















#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
