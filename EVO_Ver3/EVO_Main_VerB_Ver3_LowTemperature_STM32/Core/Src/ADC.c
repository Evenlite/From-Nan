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
	
	switch(Batt_Temperature[i])
	{
	  case No_Battery:
	  {
	    if (ADC_temp_reading[i] <= Temp_No_batt)
	    {
	      Batt_Temperature[i] = TEMP_IS_Cold;
	    }
         break;
	  }
	  case TEMP_IS_Cold:
	  {
	    if (ADC_temp_reading[i] <= Temp_5)
	    {
	      Batt_Temperature[i] = TEMP_IS_Cool;
	    }
	    else if (ADC_temp_reading[i] > Temp_No_batt + 20 )
	    {
	      Batt_Temperature[i] = No_Battery;
	    }
	    break;
	  }
	  case TEMP_IS_Cool:
	  {
	    if (ADC_temp_reading[i] <= Temp_10)
	    {
	      Batt_Temperature[i] = TEMP_IS_Normal;
	    }
	    else if (ADC_temp_reading[i] > Temp_5 + 10 )
	    {
	      Batt_Temperature[i] = TEMP_IS_Cold;
	    }
	    break;
	  }
	  case TEMP_IS_Normal:
	  {
	    if (ADC_temp_reading[i] <= Temp_30)
	    {
	      Batt_Temperature[i] = TEMP_IS_Warm;
	    }
	    else if (ADC_temp_reading[i] > Temp_10 + 10)
	    {
	      Batt_Temperature[i] = TEMP_IS_Cool;
	    }
	    break;
	  }
	  case TEMP_IS_Warm:
	  {
	    if (ADC_temp_reading[i] <= Temp_80)
	    {
	      Batt_Temperature[i] = TEMP_IS_Hot;
	    }
	    else if (ADC_temp_reading[i] > Temp_30 + 10 )
	    {
	      Batt_Temperature[i] = TEMP_IS_Normal;
	    }
	    break;
	  }
	  case TEMP_IS_Hot:
	  {
	    if (ADC_temp_reading[i] > Temp_80 + 10 )
	    {
	      Batt_Temperature[i] = TEMP_IS_Warm;
	    }
	    break;
	  }
	  
	  default: break;
	  
	}
	
	
   }
//   if ((Batt_Temperature[0]== No_Battery) && ( Batt_Temperature[1]== No_Battery))
//   { LED_task(Batt_Disconnect, ENABLE); }
//   else 
//   { LED_task(Batt_Disconnect, DISABLE); }

}


