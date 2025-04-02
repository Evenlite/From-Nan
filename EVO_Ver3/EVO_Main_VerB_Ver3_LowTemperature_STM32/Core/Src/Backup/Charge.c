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
CHR_TypeDef CHR_Stage = LOW_BATT;
Battery_Temperature_TypeDef Batt_Temperature[2];  // [0]: buttom batt temperature, [1]: top batt temperature

extern System_State_TypeDef System_State_Machine;
uint32_t ADC_batt_reading;               // battery voltage ADC value
uint32_t ADC_charger_reading;            // charger voltage ADC value
extern uint32_t ADC_DMA_VALUE[6];
extern uint32_t ADC_CHANNCEL_VAL;

uint32_t Charger_StartTime = 0;               // for charger sampling time gap, unit: millisecond
uint32_t Charger_CurrentTime = 0;  
uint32_t Charger_ElapsedTime =0;	






//* functions





//For CHARGE

/* temperature task result
#define No_Battery 				  	0     // all the battery packs are not installed
#define TEMP_IS_Cold  		  	1     // temperature is lower  than 5°C,  heater is turned on to heat the pack/packs 
#define TEMP_IS_Comfort		  	2     // temperature is higher than 5°C,  lower than 20°C, heater is turned off no heatinig
#define TEMP_IS_Warm  		    3     // temperature is higher than 20°C, lower than 40°C heater is turned off no heatinig   
#define TEMP_IS_Hot 	        4	    // temperature is higher than 40°C, lower than 80°C, charge is limited, only Pre-CHR and RC_CHR is allowed
#define TEMP_IS_Extreme       5     // temperature is higher than 80°C, charge is prohibited, no charge is allowed

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

void CHARGE_task()
{
 if ( (Batt_Temperature[0] != No_Battery) || (Batt_Temperature[1] != No_Battery))
	{	
	  if ( System_State_Machine == STANDBY)     // in sandby mode 
	    {
	     	if ( ( Batt_Temperature[0] != TEMP_IS_Extreme ) && ( Batt_Temperature[1] != TEMP_IS_Extreme )) // none of the batteries is extremely hot
				{
	     	      
	        //USBVcom_printf("ADC_batt_reading:%d\r\n",ADC_batt_reading);
	        ADC_batt_reading = ADC_DMA_VALUE[ADC_BATT_VOLTAGE];    // due to the DMA ADC value changed too quickly, so should read the value for all the condition checking cycle
					ADC_charger_reading = ADC_DMA_VALUE[ADC_CHR_VOLTAGE];
					
					
					
	        switch (CHR_Stage)
	        	{
	        		case LOW_BATT:
	        		{
	       // 			Charger_StartTime = HAL_GetTick();
	        			if ( ADC_batt_reading <= V_Low_Capacity)      // stay in the Low Batt state
	        			 {
									 Charger_ElapsedTime = ElapsedTimer(Charger_StartTime);
	        				 if (Charger_ElapsedTime < 600000)    // 10 min
									 {
										 Charg_Current_Percentage = 40;
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
	        			   Charg_Current_Percentage = 40;
	        				 __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, Charg_Current_Percentage);   // battery detected, go to the pre_charge state and will charge the batt at 10% rate
	        		     
	        				 LED_task(Batt_Disconnect, DISABLE);
	        				 LED_task(Fast_Charge, ENABLE);
	        				 CHR_Stage = PRE_CHR;
									 Charger_ElapsedTime = 0;
									 Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger ADC
									 
	        			 }							
	        		 break;
	        		}
	        
	        		case PRE_CHR:   
	        		{
	        			if( ADC_batt_reading >= V_PreChr )       // stay in this state until the voltage raised to the CC charge threshold -> V_PreChr
	        			  {
	        			   	 Charg_Current_Percentage = 100; 
	        			  	 __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, Charg_Current_Percentage);  
	        			  	 
	        			  	 LED_task(Fast_Charge, ENABLE);                     // active the charge LED display
	        			  	 CHR_Stage = CC_CHR;
										 Charger_ElapsedTime = 0;
										 Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger ADC
	        			  }
	        			 else if ( ADC_batt_reading < V_Low_Capacity)
	        			  {                                                     //  battery is dead or disconnected
	        			  	__HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, 0); 
										HAL_Delay(20);                                      // waiting for the charger to stop
	        			  	LED_task(Fast_Charge, DISABLE); 
	        			  	LED_task(Batt_Disconnect, ENABLE);                  // dead battery or no battery
                          
	        				  CHR_Stage = LOW_BATT;
										Charger_ElapsedTime = 0;
										Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger timer  
	        			  }
	              // check the charger
								 else
								  {
								    Charger_ElapsedTime =  ElapsedTimer(Charger_StartTime) ;    
								    if (Charger_ElapsedTime >= 500 )
								    {
								      if ( (ADC_charger_reading < V_FullChr) && ( ADC_charger_reading < (ADC_batt_reading - 500 )) )
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
	        			  	LED_task(Full_Float_Charge,ENABLE);
	        			   
	        			  	CHR_Stage = Termination_CHR;                     // go to CV state
										Charger_ElapsedTime = 0;
										Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger ADC
	        			  }
	        			 else if ( ADC_batt_reading <= V_PreChr )            // battery discharged and need to pre-charge state to restart the charge cycle
	        			  {
                     Charg_Current_Percentage = 20;
	        			  	__HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, Charg_Current_Percentage);     
	        			   	 
	        			    LED_task(Fast_Charge, DISABLE);   
	        			   
	        			    CHR_Stage = PRE_CHR;
										Charger_ElapsedTime = 0;
										Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger ADC
                   }
	        				else    // check the charger
									 {
								     Charger_ElapsedTime =  ElapsedTimer(Charger_StartTime) ;    
								     if (Charger_ElapsedTime >= 500 )
								     {
								       if ((ADC_charger_reading < V_FullChr) && ( ADC_charger_reading < (ADC_batt_reading - 500 )))
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
											Charger_ElapsedTime = 0;
											Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger ADC
											
	        			   	}
                  else if ( ADC_batt_reading <= V_ReChr )            
	        			   	{
	        			   		Charg_Current_Percentage = 100;
	        			   		__HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, Charg_Current_Percentage);     
	        			   		HAL_Delay(2);                                  // wait until the charger stable  
	        			   	  LED_task(Fast_Charge, ENABLE); 
	        						LED_task(Full_Float_Charge,DISABLE);
	        			     
	        			   	  CHR_Stage = CC_CHR;
											Charger_ElapsedTime = 0;
											Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger ADC
	        			   	}
										
								  else    // check the charger
									  {
								      Charger_ElapsedTime =  ElapsedTimer(Charger_StartTime) ;    
								      if (Charger_ElapsedTime >= 100 )
								      {
								        if ( ADC_charger_reading < ADC_batt_reading )
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
								if (Charger_ElapsedTime >= 100)
								{
								  if ( ADC_batt_reading < V_ReChr ) 
	        			   {
	        			   	 __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH, 50);
	        			     HAL_Delay(2);                                    // wait until the charger stable 
	        			     CHR_Stage = RC_CHR;
	        			   }
								}  
	         			break;
	        		}
	        
	        	default:
	        			break;
	        	}
	     
	      }
			  else   // one or both batteries are extremely hot
				  {
			      __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH,0); 
						HAL_Delay(20);                                       // wait until charger stopped	
           	LED_task(Fast_Charge, DISABLE);
	     	  	LED_task(Full_Float_Charge,DISABLE);    
                  
	     	  	CHR_Stage = LOW_BATT;
						Charger_ElapsedTime = 0;
				    Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger timer 
			    }
		  }
	
	    else      // in the modes other than standby               
	      {
          ADC_batt_reading = ADC_DMA_VALUE[ADC_BATT_VOLTAGE];    // due to the DMA ADC value changed too quickly, so should read the value for all the condition checking cycle
					ADC_charger_reading = ADC_DMA_VALUE[ADC_CHR_VOLTAGE];
					__HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH,0); 
					HAL_Delay(20);                                         // wait until charger stopped	
          LED_task(Fast_Charge, DISABLE);
	     	  LED_task(Full_Float_Charge,DISABLE);    
                  
	     	  CHR_Stage = LOW_BATT;
					Charger_ElapsedTime = 0;
	        Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger timer 
	      }	
	      
	} 
 else  // no battery on top and buttom 
  {
    __HAL_TIM_SetCompare(&htim4, CHARGE_PWM_CH,0); 
		HAL_Delay(20);                                               // wait until charger stopped
    LED_task(Fast_Charge, DISABLE);
	  LED_task(Full_Float_Charge,DISABLE);    
    LED_task(Batt_Disconnect, ENABLE);         
	  CHR_Stage = LOW_BATT;
  
  }
	
}


				
