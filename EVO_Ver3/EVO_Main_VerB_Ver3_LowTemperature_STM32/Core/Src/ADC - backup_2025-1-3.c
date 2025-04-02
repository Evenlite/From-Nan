/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ADC.c
  * @brief          : ADC task program 
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ADC.h"
#include "stm32f1xx_it.h"




/* Define ------------------------------------------------------------*/





/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/

//float ADC_sum[6];
//uint32_t ADC_Value[6];


extern Battery_Temperature_TypeDef Batt_Temperature[2];  // [0]: buttom batt temperature, [1]: top batt temperature


uint32_t ADC_DMA_VALUE[6];
uint32_t ADC_CHANNCEL_VAL;




/*																 
ADC channel defined:
  ADC_REMOTE_LAMP_CUR 	0															 
  ADC_LOCAL_LAMP_CUR 	1                               
  ADC_TOP_BATT_TEMP 	2	                                
  ADC_BUT_BATT_TEMP  	3
  ADC_BATT_VOLTAGE 		4
  ADC_CHR_VOLTAGE        5

*/

extern uint32_t ADC_temp_reading[2];                        // temperature ADC value reading buffer, [0]: buttom , [1]: top 

void BATT_TEMP_task(void)
{

//  uint32_t ADC_TEMP_buffer0 =0;
//  uint32_t ADC_TEMP_buffer1 =0; 
  
  uint8_t i =0;
//  uint8_t j =0;
// Read 10 samples from the ADC
/*
   for (j = 0; j < 10; j++) 
   {
     ADC_TEMP_buffer0  += ADC_DMA_VALUE[ADC_BUT_BATT_TEMP];
     ADC_TEMP_buffer1  += ADC_DMA_VALUE[ADC_TOP_BATT_TEMP];
     HAL_Delay(1);                                    // Small delay to get different samples (1 ms delay)
   }

  ADC_temp_reading[0] = ADC_TEMP_buffer0/10;
  ADC_temp_reading[1] = ADC_TEMP_buffer1/10 ;
*/	
	
// For buttom batt pack

  for (i=0; i<2; i++)
  {  
	if (ADC_temp_reading[i] <=  Temp_80)                // temperature: >80°C -- 580(0x244)
	{
	  Batt_Temperature[i] = TEMP_IS_Hot;                                               
	}
	else if ( ADC_temp_reading[i] <= Temp_80 )          // temperature:10-80°C -- 1500(0x5DC)
	{
	  Batt_Temperature[i] = TEMP_IS_Normal;
	} 
//	else if ( ADC_temp_reading[i] <= Temp_30 )           // temperature 30-40°C -- 2300(0x8FC)
//	{
//	  Batt_Temperature[i] = TEMP_IS_Warm;
//	} 
//	else if ( ADC_temp_reading[i] <= ( Temp_10 - 20) )    // temperature: < 10°C  leave a gape of 50 for no action in case of heater and charger bouncing on/off 
//	{ 
//	  Batt_Temperature[i] = TEMP_IS_Comfort;
//	} 
//	else if ( ( ADC_temp_reading[i] <=  Temp_10 ) && (ADC_temp_reading[i] > ( Temp_10 - 50)) )
//	{
//	 Batt_Temperature[i] = Batt_Temperature[i];          // buffering range where the state remian the same as current
//	}
	else if ( ( ADC_temp_reading[i] >  Temp_10 ) && (ADC_temp_reading[i] <= Temp_No_batt) )      // 3840         
	{
	  Batt_Temperature[i] = TEMP_IS_Cold;
	} 
	else                                                 // no battery is installed -- 3840(0xF00)
	{
	  Batt_Temperature[i] = No_Battery;   
					
	}
   }
//   if ((Batt_Temperature[0]== No_Battery) && ( Batt_Temperature[1]== No_Battery))
//   { LED_task(Batt_Disconnect, ENABLE); }
//   else 
//   { LED_task(Batt_Disconnect, DISABLE); }

}


