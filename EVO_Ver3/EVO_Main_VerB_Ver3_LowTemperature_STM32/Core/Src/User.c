/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : User.c
  * @brief          : User function and task program 
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

// #include "usbd_cdc_if.h"
// #include "Flash_EEPROM.h"
//#include "stm32f1xx_hal_tim.h"


/* Define ------------------------------------------------------------*/

#define ADDR_FLASH_SECTOR_63     ((uint32_t)0x0800FC00)  /* Base address of Sector 63, 1 Kbytes  */
#define FLASH_MEMORY_ADDR         ADDR_FLASH_SECTOR_63

#define ADC_REMOTE_LAMP_CUR 	0
#define ADC_LOCAL_LAMP_CUR 		1
#define ADC_TOP_BATT_TEMP 		2
#define ADC_BUT_BATT_TEMP  		3
#define ADC_BATT_VOLTAGE 			4



/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
/*
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

*/


System_State_TypeDef System_State_Machine = STANDBY;

struct  sys_time System_time ={1,0,0};

uint32_t time_stamp_start;

NV_Profile_TypeDef const NV_Profile_Default = 
{
  1,
	0x2e782dade227f669,         //signature
  8,             							// Total wattage of the unit, optianl of 8W,16W,32W and 64W
  4,            							// Lamp head wattage: 4W, 8W and 16W
  1,               						// heater installation
  1,										      // Self test enable 
	300,          							// Delay after AC resumed, uint: seconds, options - 300s, 600s, 900s
  0,                          // time of testing which was failed
  0,           								// 0°C
  20,
  0x85,
  0x85	
};

uint32_t CC1_val;
uint32_t CC2_val;
uint32_t IR_CC1_val;
uint32_t IR_CC2_val;

float    Duty_Cycle;
float    AC_Freq;


uint32_t ADC_Value[5];

uint32_t ADC_Read_Buf[5];

uint32_t ADC_reading;
uint32_t ADC_lamp_current_reading;

ButtonPress_TypeDef TEST_Button = Idle;




/* Private function prototypes -----------------------------------------------*/


/* USER CODE BEGIN PFP */
extern void Flash_write(void);
extern void Flash_read(void);
extern void Flash_clear(void);
extern HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);

extern NV_Profile_TypeDef NV_Profile_Runtime;



//USB init and reset
/*
void USB_Init(void)   // added USB reset
{
    
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
	
    HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
    HAL_Delay(20);
			
} 
 
 */
 uint8_t write_buf ;
 


/* USER CODE BEGIN 4 */
// For local LAMP

void Lamp_ctrl(FunctionalState EN, uint8_t LAMP_WALT) 
{
  if (EN != 0) 
	   {
				uint8_t walt = LAMP_WALT;
				uint8_t lamp_level;
				uint8_t i;				
				switch (walt)
					{
					  case 0:     lamp_level = 0               ;break;
						case _4W: 	lamp_level = LAMP_LEVEL_4W;   break;
						case _8W: 	lamp_level = LAMP_LEVEL_8W;   break;
						case _16W: 	lamp_level = LAMP_LEVEL_16W; 	break;
						default: break;
					}
	      
				for ( i=0; i< lamp_level; i++) 	 
				{ 
					__HAL_TIM_SetCompare(&htim4, LAMP_PWM_CH, i); // the LAMP_LEVEL controls the brightness, value range 0-99, represent n/100 of full power
					HAL_Delay(10);
				}
		 }	
		
  else __HAL_TIM_SetCompare(&htim4, LAMP_PWM_CH, 0);
}

 


// Remote LAMP
void Remote_Lamp_EN(FunctionalState EN)
{
  if (EN != 0)
	   HAL_GPIO_WritePin(REMOTE_LAMP_EN_GPIO_Port,REMOTE_LAMP_EN_Pin,GPIO_PIN_SET);  //Enable remote lamp power
	else
	   HAL_GPIO_WritePin(REMOTE_LAMP_EN_GPIO_Port,REMOTE_LAMP_EN_Pin,GPIO_PIN_RESET);  //Disable remote lamp power
}





//For CHARGE


CHR_TypeDef CHR_Stage = NO_BATT;

void CHARGE_task(void)
{
  uint8_t CHARGE_current;
	
	ADC_reading = ADC_READ(ADC_BATT_VOLTAGE);
  switch (CHR_Stage)
  {
    case NO_BATT:
		{
			if( ADC_reading > V_Low )
				{
				  __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, 10);
				  CHR_Stage = PRE_CHR;
				}
			else 
				{
					__HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, 0);
				}
					
			break;
		}

		case PRE_CHR:
	  {
      if (ADC_reading > V_PreChr)
		    {
					__HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, 80);
					CHR_Stage = CC_CHR;
				}
		  break;	 
	  }


		case CC_CHR:
		{
			if ( ADC_reading >= V_FullChr)
			{ 
				__HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, 0);
			  CHR_Stage = Termination_CHR;
			}
			break;
		}

	 case RC_CHR:
		{
		  if ( ADC_reading >= V_FullChr)
			{ __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, 0);
			  CHR_Stage = Termination_CHR;
			}
			break;
		}

	 case Termination_CHR:
		{
			 if ( ADC_reading < V_ReChr )
			 { 
				__HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, 50);
			  CHR_Stage = RC_CHR;
			 }
			  break;
		}

	 default:
					break;
  }

}


// For LED
void LED_task(uint8_t red,uint8_t green,uint8_t blue, uint8_t brightness, FunctionalState Blink_EN)           // the LAMP_LEVEL controls the brightness, value range 0-99, represent n/100 of full power
{

    //HAL_TIM_PWM_Start(&htim4, LAMP_PWM_CH);
    if (red ==1)
    {
			__HAL_TIM_SetCompare(&htim1, LED_PWM_CH_RED, LED_level_5); 
    }
    else
		{
		  __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_RED, LED_level_0); 
		}

    if (green ==1)
    {
			__HAL_TIM_SetCompare(&htim1, LED_PWM_CH_GREEN, LED_level_5); 
    }
    else
		{
		  __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_GREEN, LED_level_0); 
		}

    if (blue ==1)
    {
			__HAL_TIM_SetCompare(&htim1, LED_PWM_CH_BLUE, LED_level_5); 
    }
    else
		{
		  __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_BLUE, LED_level_0); 
		}





}


//for zero cross and IR remote

uint8_t Zero_Cross_Task(void)
{
  if ( AC_Freq < 58 )   // check if the AC applied or zero cross circuit is working
  {
   return AC_Fail;
	}

	else 
	{
			if ( Duty_Cycle > AC_GoodVoltage )
			{
				return AC_Good;
			}
			else if ( Duty_Cycle < AC_FailVoltage )
			{ 
				return AC_Fail;
			}
	    else 
				return Invalid;
	} 
  

}







#define slide_windows_size  30
float sum =0;
float sample_buf[slide_windows_size] = {0};
uint8_t IR_bit_cnt=0;

static uint16_t IR_CMD;
static uint16_t IR_buf ;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
	
{
 if (htim == &htim2)                                    // This is for zero cross function
    {	
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
			static float Duty_temp =0;
			
			if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) 
			{
				CC1_val = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1); // freqency cycle cnt value
			} 
			else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) 
			{
				CC2_val = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2); // duty cycle cnt value
			}
		
			if (CC1_val != 0)
			{
				Duty_temp = (float)CC2_val*100/(float)CC1_val;
				AC_Freq = 1000000/(float)CC1_val/2;
			}
			else
			{
				Duty_temp = 0;
				AC_Freq   = 0;
			}
			
			
			uint8_t i=0;
			sum =0;
			memcpy (&sample_buf[0], &sample_buf[1], (slide_windows_size - 1)*4); // shifting the samples, first data removed and leave the last for new data
			sample_buf[slide_windows_size - 1] = Duty_temp;
			for ( i= 0; i < slide_windows_size; i++)
					sum += sample_buf[i];
		
			Duty_Cycle = sum / slide_windows_size; // calculate average 
			
		}			
else if (htim == &htim3)                    // This is for IR deconding
     {
        __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
			  
       if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) 
				 {
				 	IR_CC1_val = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1); // duty cycle cnt value
				 	
			  
				 	if ((IR_CC1_val>2000) || (IR_CC1_val<150))
	            {
                IR_bit_cnt =0;
	            }
				 } 
			  
			 else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) 
				 {
				 	IR_CC2_val = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2); // freqency cnt value
			  
          if (IR_CC2_val!=0 ) 
				 		{
				 			IR_buf  = IR_buf << 1;
							if (IR_CC2_val <600)   //258
				 					{
				 						IR_buf = IR_buf | 0;
				 					}
			        else  
				 			    {
				 			      IR_buf = IR_buf | 0x0001;
				 			    }
			        IR_bit_cnt ++;
				 	  }
				 }

			if  (IR_bit_cnt ==12)
				{
//					USBVcom_printf("IR_Code:%x\r\n\r\n", IR_CMD);
				  IR_bit_cnt=0;
		      IR_CMD = IR_buf&0x0FFF;
				}
			
		 }
			
}

//	USBVcom_printf("INT:%u\r\n\r\n", cnt); 
								
//	USBVcom_printf("CC1_cnt:%u INT:%u\r\n\r\n", cnt, IR_zero_cnt);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  // overflow interrupt
{
	
	if (htim == &htim3) 
  {
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
		IR_CMD |= 0x1000;                                  // the first byte "0" is valid IR data, "1" is overdue data either has been red or lifetime is due
	}

  if (htim == &htim2) 
  {
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
		Duty_Cycle = 0;                                   
	}





}




// For ADC

#define ADC_slide_windows_size  10
float ADC_sum[5];
static uint32_t ADC_Sample_buf[5][ADC_slide_windows_size] = {0,0};

uint32_t ADC_READ(uint8_t channel)
{
  uint8_t i =0;
	for(i = 0; i < 5; i++)  // i: channel ID, 5 channels for Remote/local lamps, Top/Bottom battery temperature and battery voltage
		{
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 100);
				
				uint32_t state = HAL_ADC_GetState(&hadc1);
				if (( state & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
				{
					ADC_Read_Buf[i] = HAL_ADC_GetValue(&hadc1);
				}
		}


	HAL_ADC_Stop(&hadc1);
		
//	USBVcom_printf("ADC value [%d]:%u\r\n\n", i,ADC_Read_Buf[4]);   
  
	uint8_t k = 0 ;	
  for (k=0; k<5; k++)
      {
		ADC_sum[k] = 0;
		  }
	
	uint8_t h = 0 ;
	for ( h=0; h<5; h++)			
	    {	
		    
				memcpy (&ADC_Sample_buf[h][0], &ADC_Sample_buf[h][1], (ADC_slide_windows_size - 1)*4); // shifting the samples, first data removed and leave the last for new data
				ADC_Sample_buf[h][ADC_slide_windows_size - 1] = ADC_Read_Buf[h];
	      
				uint8_t j=0;    // slide window id		
				for ( j= 0; j < ADC_slide_windows_size; j++)
						ADC_sum[h] += ADC_Sample_buf[h][j];
			
				ADC_Value[h] = ADC_sum[h] / ADC_slide_windows_size; // calculate average   
				USBVcom_printf("ADC value [%d]:%u\r\n",h,ADC_Value[h]);   
       }
return ADC_Value[channel];       
}			                           // ADC_REMOTE_LAMP_CUR 	0
																 // ADC_LOCAL_LAMP_CUR 		1
																 // ADC_TOP_BATT_TEMP 		2	
                                 // ADC_BUT_BATT_TEMP  		3
                                 // ADC_BATT_VOLTAGE 			4



// For Test button GPIO interrupt


uint32_t Button_Press_StartTime =0; 
uint32_t Button_Press_Duration =0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 
 if(GPIO_Pin == TEST_BUTTON_Pin)
 {
   Button_Press_StartTime = HAL_GetTick();  // record starting time
   while(HAL_GPIO_ReadPin(TEST_BUTTON_GPIO_Port, TEST_BUTTON_Pin) == 0 )
   {
	  	Button_Press_Duration = HAL_GetTick()-Button_Press_StartTime;  
			if ((Button_Press_Duration > 100) &&( Button_Press_Duration<3000))	
			{
			 TEST_Button = Short_Press;
			 continue;
			}
	    else if (Button_Press_Duration > 4000)
	    {
			 TEST_Button = Long_Press;
			}
	 
	 }
  Button_Press_Duration=0;
  }

	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin); // for proventing from switch shaking 

}








/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
