/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Charge.c
  * @brief          : Charger task program 
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Charge.h"
#include "ADC.h"


/* Define ------------------------------------------------------------*/







/* Macro -------------------------------------------------------------*/



/* Variables ---------------------------------------------------------*/

extern uint8_t  Charg_Current_Percentage; // 0-99 for PWM value backup to resume the charging current
extern TIM_HandleTypeDef htim4;

//extern uint32_t ADC_batt_reading;    
//extern uint32_t ADC_charger_reading; 
CHR_TypeDef CHR_Stage = IDL_CHR;
Battery_Temperature_TypeDef Batt_Temperature[2] = { No_Battery, No_Battery };  // [0]: buttom batt temperature, [1]: top batt temperature

extern System_State_TypeDef System_State_Machine;
uint32_t ADC_batt_reading;               // battery voltage ADC value
uint32_t ADC_charger_reading;            // charger voltage ADC value
uint8_t  Charge_rate;                    // charger PWM value in fast charge stage

extern uint32_t ADC_DMA_VALUE[6];
extern uint32_t ADC_CHANNCEL_VAL;

uint32_t Charger_StartTime = 0;               // for charger sampling time gap, unit: millisecond
uint32_t Charger_CurrentTime = 0;  
uint32_t Charger_ElapsedTime =0;	




//* functions





//For CHARGE

/* temperature task result
#define No_Battery 			0     // all the battery packs are not installed
#define TEMP_IS_Cold  		1     // temperature is lower  than 5�C,  heater is turned on to heat the pack/packs 
#define TEMP_IS_Comfort		2     // temperature is higher than 5�C,  lower than 20�C, heater is turned off no heatinig
#define TEMP_IS_Warm  		3     // temperature is higher than 20�C, lower than 40�C heater is turned off no heatinig   
#define TEMP_IS_Hot 	     4	 // temperature is higher than 40�C, lower than 80�C, charge is limited, only Pre-CHR and RC_CHR is allowed
#define TEMP_IS_Extreme       5     // temperature is higher than 80�C, charge is prohibited, no charge is allowed

*/

uint32_t ElapsedTimer(uint32_t StartTime)
{
  uint32_t CurrentTime = HAL_GetTick();
  if ( CurrentTime >= StartTime)
  {
    return CurrentTime - StartTime;
  }
  else
  {
    return (UINT32_MAX - StartTime + CurrentTime +1) ;
  }
}

extern uint32_t ADC_temp_reading[2];

 void ADC_BATT_CHR_ACQ(void)  // colect battery and charger filtered ADC value  
{
  uint32_t ADC_Batt_buffer[10] = {0};
  uint32_t ADC_Chr_buffer[10]  = {0}; 
  uint32_t min_batt, max_batt;
  uint32_t min_chr, max_chr;
  uint32_t sum_batt = 0;  
  uint32_t sum_chr = 0;  
  
  uint32_t ADC_TEMP_buffer0 =0;
  uint32_t ADC_TEMP_buffer1 =0;
  
  
  uint8_t i =0;
   
  // Read 8 samples from the ADC
  for (i = 0; i < 8; i++) 
  {
    ADC_Batt_buffer[i] = ADC_DMA_VALUE[ADC_BATT_VOLTAGE];
    ADC_Chr_buffer[i]  = ADC_DMA_VALUE[ADC_CHR_VOLTAGE];
    ADC_TEMP_buffer0  += ADC_DMA_VALUE[ADC_BUT_BATT_TEMP];
    ADC_TEMP_buffer1  += ADC_DMA_VALUE[ADC_TOP_BATT_TEMP];
    HAL_Delay(1);                                    // Small delay to get different samples (1 ms delay)
  }
  
  ADC_temp_reading[0] = ADC_TEMP_buffer0/8;
  ADC_temp_reading[1] = ADC_TEMP_buffer1/8 ;
  
 // Initialize min and max
    min_batt = max_batt = ADC_Batt_buffer[0];
    min_chr  = max_chr = ADC_Chr_buffer[0];
 
 // Find min, max, and calculate sum
  for ( i = 0; i < 8; i++) 
  {
    sum_batt += ADC_Batt_buffer[i];
    if (ADC_Batt_buffer[i] < min_batt) 
    {
      min_batt = ADC_Batt_buffer[i];
    }
    if (ADC_Batt_buffer[i] > max_batt) 
    {
      max_batt = ADC_Batt_buffer[i];
    }
  }
  sum_batt = (sum_batt - max_batt - min_batt);

  for ( i = 0; i < 8; i++) 
  {
    sum_chr += ADC_Chr_buffer[i];
    if (ADC_Chr_buffer[i] < min_batt) 
    {
      min_batt = ADC_Chr_buffer[i];
    }
    if (ADC_Chr_buffer[i] > max_batt) 
    {
      max_chr = ADC_Chr_buffer[i];
    }
  }
  sum_chr = (sum_chr - max_chr - min_chr);
 
  ADC_batt_reading = sum_batt/6;
  ADC_charger_reading = sum_chr/6 ;

}


void CHARGE_task()
{

 if ( (Batt_Temperature[0] != No_Battery) || (Batt_Temperature[1] != No_Battery))
 {	
   if ( System_State_Machine == STANDBY )     // in sandby mode 
   {
	if ( ( Batt_Temperature[0] != TEMP_IS_Hot ) && ( Batt_Temperature[1] != TEMP_IS_Hot )  // none of the batteries is extremely hot and cold
	   
	     && 
		(  (( NV_Profile_Runtime.HasHeater !=0 ) && (Batt_Temperature[0] != TEMP_IS_Cold) && (Batt_Temperature[1] != TEMP_IS_Cold) ) ||    // for the heater installed options
		   ( NV_Profile_Runtime.HasHeater == 0 )                                                                                   )     ) // for the no heater options
	
	{
					
	  switch (CHR_Stage)
	  {
	    case IDL_CHR:
	    {

	      if ( ADC_batt_reading <= V_Low_Capacity)      // stay in the Low Batt state
	      {
		   Charger_ElapsedTime = ElapsedTimer(Charger_StartTime);
	        if (Charger_ElapsedTime < 600000)    // 10 min
		   {
			Charg_Current_Percentage = Charge_rate/2;
	        	__HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, Charg_Current_Percentage);
	        }
	        else 
		   {
               Charg_Current_Percentage = 0;
			__HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, Charg_Current_Percentage);
			LED_task(Batt_Disconnect, ENABLE);
		   }
	      }
	      else
	      {
	        Charg_Current_Percentage = Charge_rate/2;
	        __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, Charg_Current_Percentage);   // battery detected, go to the pre_charge state and will charge the batt at 10% rate
	        		     

	        LED_task(Fast_Charge, ENABLE);
	        CHR_Stage = PRE_CHR;
//		   Charger_ElapsedTime = 0;
		   Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger ADC
									 
	      }							
	      break;
	    }
	        
	    case PRE_CHR:   
	    {
	      if( ADC_batt_reading >= V_PreChr )       // stay in this state until the voltage raised to the CC charge threshold -> V_PreChr
	      {
	        Charg_Current_Percentage = Charge_rate; 
	        __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, Charg_Current_Percentage);  
	        			  	 
	        LED_task(Fast_Charge, ENABLE);                      // active the charge LED display
	        CHR_Stage = CC_CHR;
		   Charger_StartTime = HAL_GetTick() ;                 //load the start time for the charger ADC
	      }
	      else if ( ADC_batt_reading < V_Low_Capacity)
	      {                                                     //  battery is dead or disconnected
	        __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, 0); 
		   HAL_Delay(20);                                      // waiting for the charger to stop
	        LED_task(Fast_Charge, DISABLE); 
	        LED_task(Batt_Disconnect, ENABLE);                  // dead battery or no battery
                          
	        CHR_Stage = IDL_CHR;
		   Charger_ElapsedTime = 0;
		   Charger_StartTime = HAL_GetTick() ;                 //load the start time for the charger timer  
	      }
		 else  // check the charger
		 {
		   Charger_ElapsedTime =  ElapsedTimer(Charger_StartTime) ;    
		   if (Charger_ElapsedTime >= 1000 )
		   {
		     if ( ADC_charger_reading < (ADC_batt_reading ) )
			      LED_task( Charger_Fail, ENABLE );
		     else 											
	                LED_task( Charger_Fail, DISABLE );
		   }
		 }
 	      break;	
	   }
	                                                                
	   case CC_CHR:
	   {
          if ( ADC_batt_reading >= V_FullChr)   // charge the battery in this state until reach the full voltage
	     { 
	        __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, 0);  // terminate the charge when it reach the full voltage
	        			  	
	        LED_task(Fast_Charge, DISABLE);                  // turn off the fast charge display and turn on the full charge display
	        LED_task(Full_Float_Charge, ENABLE);
	        			   
	        CHR_Stage = Termination_CHR;                     // go to CV state
		   Charger_StartTime = HAL_GetTick() ;              // load the start time for the charger ADC
	     }
	     else    // check the charger
		{
		  Charger_ElapsedTime =  ElapsedTimer(Charger_StartTime) ;    
		  if (Charger_ElapsedTime >= 1000 )
		  {
			if ( ADC_charger_reading < (ADC_batt_reading)) 
			     LED_task( Charger_Fail, ENABLE );
			else 											
	               LED_task( Charger_Fail, DISABLE );
		  }
		}
									 
	     break;
	   }
	        			
	   case RC_CHR:
	   {
	     if ( ADC_batt_reading >= V_FullChr)
	     {
	       __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, 0); 
				   						                    // wait until charger stopped
		  LED_task(Full_Float_Charge,ENABLE);
	       CHR_Stage = Termination_CHR;
//		  Charger_ElapsedTime = 0;
		  Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger ADC
											
	     }
          else if ( ADC_batt_reading <= V_ReChr )            
	     {
	       Charg_Current_Percentage = Charge_rate;
	       __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, Charg_Current_Percentage);     
	       HAL_Delay(2);                                  // wait until the charger stable  
	       LED_task(Fast_Charge, ENABLE); 
	        			     
	       CHR_Stage = CC_CHR;
		  Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger ADC
	     }
										
		else     // check the charger
		{
		  Charger_ElapsedTime =  ElapsedTimer(Charger_StartTime) ;    
		  if (Charger_ElapsedTime >= 1000 )
		  {
			if ( ADC_charger_reading < ADC_batt_reading  )
				LED_task( Charger_Fail, ENABLE );
			else 											
	               LED_task( Charger_Fail, DISABLE );
		  }
		}
                
	     break;
	   }
	        
	   case Termination_CHR:
	   {
	     Charger_ElapsedTime =  ElapsedTimer(Charger_StartTime) ;   
		if (Charger_ElapsedTime >= 1000)
		{
		  if ( ADC_batt_reading < V_ReChr ) 
	       {
		     Charg_Current_Percentage = Charge_rate/2;
	        	__HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, Charg_Current_Percentage);
	        	HAL_Delay(20);                                    // wait until the charger stable 
	        	CHR_Stage = RC_CHR;
	       }
		}  
	     break;
	   }
	        
	   default:
	     break;
	  }
	     
	}
     else   // one or both batteries are cold or extremely hot
	{
	  __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH,0);       // stop charging
	  HAL_Delay(10);                                       // wait until charger stopped	
       LED_task(Fast_Charge, DISABLE);
	  LED_task(Full_Float_Charge,DISABLE);    
                  
	  CHR_Stage = IDL_CHR;
	
	  Charger_StartTime = HAL_GetTick() ;                  //load the start time for the charger timer 
	}
    }
	
    else      // in the modes other than standby               
    {
  	 __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH,0); 
	 LED_task( Charger_Fail, DISABLE );
	 HAL_Delay(20);                                         // wait until charger stopped	
      LED_task(Fast_Charge, DISABLE);
	 LED_task(Full_Float_Charge,DISABLE);    
                  
	 CHR_Stage = IDL_CHR;
	// Charger_ElapsedTime = 0;
	 Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger timer 
    }	
	      
 } 
 else  // no battery on top and buttom at all
 {
   __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH,0); 
   HAL_Delay(10);                                               // wait until charger stopped
   LED_task(Fast_Charge, DISABLE);
 
   LED_task(Batt_Disconnect, ENABLE);         
   CHR_Stage = IDL_CHR;
  
 }

}


				
