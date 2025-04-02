/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IR.c
  * @brief          : IR remote and Zero cross function and task program 
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "IR_Zero_Cross.h"
#include "Charge.h"
//#include "LED_Process.h"


/* Define ------------------------------------------------------------*/

#define ADDR_FLASH_SECTOR_63     ((uint32_t)0x0800FC00)  /* Base address of Sector 63, 1 Kbytes  */
#define FLASH_MEMORY_ADDR         ADDR_FLASH_SECTOR_63

#define ADC_REMOTE_LAMP_CUR 	0
#define ADC_LOCAL_LAMP_CUR 		1
#define ADC_TOP_BATT_TEMP 		2
#define ADC_BUT_BATT_TEMP  		3
#define ADC_BATT_VOLTAGE 			4
#define ADC_CHR_VOLTAGE 			5

#define LED_Run_State_stopped    0  // define the Run_states
#define LED_Run_State_run        1
#define LED_Blink_mode_once      0
#define LED_Blink_mode_continue  1
#define LED_Blink_rate_normal    0
#define LED_Blink_rate_slow      1

#define LED_Blink_times_1        1
#define LED_Blink_times_2        2
#define LED_Blink_times_3        3
#define LED_Blink_times_5        5
#define LED_Solid_on             255
#define LED_off                  0
#define LED_color_red            1
#define LED_color_green          1
#define LED_color_blue           1





/* Macro -------------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Variables ---------------------------------------------------------*/

extern struct sys_time System_time;
extern System_State_TypeDef System_State_Machine;

extern NV_Profile_TypeDef NV_Profile_Runtime;
extern NV_Profile_TypeDef NV_Profile_Default;
extern CHR_TypeDef CHR_Stage;
extern Heater_state_TypeDef Heater_state[2];

extern uint32_t ADC_DMA_VALUE[6];
extern uint32_t ADC_StartTime;
extern uint8_t Charg_Current_Percentage;

extern uint32_t ADC_batt_reading;   


uint32_t CC1_val;
uint32_t CC1a_val;
uint32_t CC1b_val;

uint32_t CC2_val;
uint32_t CC2a_val;
uint32_t CC2b_val;

uint32_t IR_CC1_val;
uint32_t IR_CC2_val;

float    Duty_Cycle;
float    AC_Freq;
uint8_t  Zero_cross_err_cnt;

uint32_t time_stamp_start;
uint32_t Emergency_ElapsedTime;
extern uint32_t Charger_ElapsedTime;
extern uint32_t Charger_StartTime;



//* functions

extern void LED_task(LED_Display_TypeDef Display_Type, FunctionalState Enable);
extern void Flash_write(void);

extern void LAMP_Init(void);


// zero cross 

uint8_t Zero_Cross_Task(void)
{
	
   if( System_State_Machine == EMERGENCY )
	    {
	     if ( (Duty_Cycle >= AC_GoodVoltage) && (Duty_Cycle < 100))  // 47.5  measured zero-cross duty cycle value; based on 85% of 120V/AC: 102V
	        {
	       		
						return AC_Good;
							
	       	}
       else if ((Duty_Cycle >= 0) && (Duty_Cycle <= AC_FailVoltage)) // 37.6 measured zero-cross duty cycle value; based on 75% of 120V/AC - 90V
	        {
	         return AC_Fail;
	        }
	     else 
	       return Invalid;
	    }
		
	  else if( System_State_Machine == STANDBY )
	  	{
	  	 if ( Duty_Cycle < AC_FailVoltage)   // 32.8  measured zero-cross duty cycle value; based on 75% of 120V/AC - 90V
	  	  { 
	  		 return AC_Fail;
	  	  }
	  	  else if ( ( Duty_Cycle > AC_FailVoltage) && (Duty_Cycle <100) )
	       {
					 
	         return AC_Good;
	       }
	      else 
	         return Invalid;  
	  	}
	  
	  else 
	      return AC_Good;  // other state won't run this task			 

}


// zero cross and IR receiving interrup callback

#define slide_windows_size  15
//static float sum =0;
//static float sample_buf[slide_windows_size] = {0};
uint8_t IR_bit_cnt=0;

static uint16_t IR_CMD;
static uint16_t IR_CMD_Buf =0;
static uint16_t IR_CMD_Reg;

static uint16_t IR_shift_reg;
float sample_max;
float sample_min;
uint8_t AC_Cycle = 0;  // indicator of the AC cycle: 0->first half period, 1-> second half period 

uint8_t IR_receiving =0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
	
{
 
 if (htim == &htim2)                                    // This is for zero cross function
    {	
	//	  HAL_GPIO_WritePin(Test_Prob_GPIO_Port,Test_Prob_Pin,GPIO_PIN_SET);  //testing flag	
    
			float  AC_Freq_temp;
			static float Duty_temp =0;
			
			
			if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) 
			{
							
				if (AC_Cycle == 0)
				{
			//	 HAL_GPIO_WritePin(Test_Prob_GPIO_Port,Test_Prob_Pin,GPIO_PIN_SET);  //testing flag			
				 CC2a_val = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2); // duty cycle cnt value
				 AC_Cycle = 1;	
				}
				else if (AC_Cycle == 1)
				{
					CC2b_val = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
					CC2_val = CC2a_val + CC2b_val;
		      AC_Cycle = 0;
				}
				
				if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) != RESET) 
				{
          __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1); 
				}
			}
			
			else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) 
			{
				
				if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) != RESET) 
				{
          __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC2); 
					
        }
				if (AC_Cycle == 1)
				{
				  CC1a_val = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1); // freqency cycle cnt value
				}
				else if (AC_Cycle == 0)
				{
				  CC1b_val = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
					CC1_val = CC1a_val + CC1b_val; 
					
					if (( CC1_val>7700 )&&(CC1_val<9100)&&( CC2_val > 1000 ))   // AC_Freq in the range of 55Hz-65Hz
					{
					  AC_Freq_temp = 500000/(float)CC1_val;
						Duty_temp = (float)CC2_val/(float)CC1_val*100;
					}
      
									
					if ( (AC_Freq_temp > 55) && ( AC_Freq_temp < 65)  )  
			       {
				       AC_Freq = AC_Freq_temp;
							 if ( Duty_temp < 90) 
			          {
				         Duty_Cycle = Duty_temp;
			          } 
               Zero_cross_err_cnt = 0;
								
			       }
		
  
				 }
				
			} 

		 __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
			
				
	
			
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
				 			IR_shift_reg  = IR_shift_reg << 1;
							if (IR_CC2_val <610)   //258
				 					{
				 						IR_shift_reg = IR_shift_reg | 0;
				 					}
			        else  
				 			    {
				 			      IR_shift_reg = IR_shift_reg | 0x0001;
				 			    }
			        IR_bit_cnt ++;
				 	  }
				 }

			if  (IR_bit_cnt ==12)
				{
				  IR_bit_cnt = 0;
         
					IR_CMD_Reg = IR_shift_reg&0x0FFF;  // 12bytes code is shifted and get last 12 bytes
					if (IR_receiving == 0 )            // get and verify the two first packages are the same to issue the IR command
						{
						  IR_CMD = IR_CMD_Buf;           // buffering the IR code 
							IR_CMD_Buf = IR_CMD_Reg;
							if (IR_CMD == IR_CMD_Buf)      // and verify the second code
							{
							 IR_receiving = 1;             // matched, set the flag of receiving for decoding 
							// LED_task(IR_Signal,ENABLE);
							}                  
						}
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
		
    IR_receiving = 0;   // reset the flag to terminate the receiving 

    IR_CMD_Buf = 0;
    
	}

  if (htim == &htim2) 
  {
 //   HAL_GPIO_WritePin(Test_Prob_GPIO_Port,Test_Prob_Pin,GPIO_PIN_RESET);  //testing flag
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    Zero_cross_err_cnt ++;
    if (Zero_cross_err_cnt == 3)
    {
		 Duty_Cycle = 0;  
     AC_Freq = 0;	
	  }		 
	}

}


// IR task

void IR_task(void)
{
 
	
	if (IR_CMD!= 0)  
  {
	  
	  switch (IR_CMD)
		{
			case IR_CMD_SHORT_TEST:   // 30 second test command
		  {
        IR_CMD =0;
				if ( System_State_Machine == STANDBY ) 
				{	
					Lamp_ctrl(ENABLE, NV_Profile_Runtime.HeadWattage);
					Remote_Lamp_EN(ENABLE);
					
					time_stamp_start = System_time.Sec;
					LED_task(_30s_TEST,ENABLE);
					
					System_State_Machine = SHORT_TEST;
					
					Charger_ElapsedTime = 0;
					Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger timer 
					ADC_StartTime = HAL_GetTick();
				}
				else if ((System_State_Machine == SHORT_TEST) || (System_State_Machine == LONG_TEST))
				{
				  Lamp_ctrl(DISABLE, 0);  // turn off the lamps return to standby 
  				Remote_Lamp_EN(DISABLE);
										
					LED_task(_30s_TEST,DISABLE);
          LED_task(_90M_TEST,DISABLE);	
					LED_task(Low_Capacity,DISABLE);	
					
          NV_Profile_Runtime.LastTestFailedFlag = 0;
				  while (IR_receiving != 0){};
					Flash_write();
				
					Charger_ElapsedTime = 0;
					Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger timer  
          System_State_Machine = STANDBY;
				}
				
				break;
			}
		
		case IR_CMD_LONG_TEST:   // 90 minute test command
		  {
				IR_CMD =0;
				if( System_State_Machine == STANDBY )
				{
					
          if(( CHR_Stage == RC_CHR) || ( CHR_Stage == Termination_CHR))
					 {
					   Lamp_ctrl(ENABLE, NV_Profile_Runtime.HeadWattage);
					   Remote_Lamp_EN(ENABLE);
			 		   							
					   time_stamp_start = System_time.Sec;
					   
				     LED_task(_90M_TEST,ENABLE);
					   NV_Profile_Runtime.LastTestFailedFlag =1;
				     while (IR_receiving != 0){};
					   Flash_write();
							 
					   System_time.Period = 0; 
						 System_time.Day = 0;	 
					   System_State_Machine = LONG_TEST;
						 	 
					   ADC_StartTime = HAL_GetTick();
							 
					 }
					else if ( CHR_Stage == LOW_BATT )             // battery is bad needs to be replaced, no need to do the test
					 {                                            // set the long test failure flag
						LED_task( Low_Capacity,ENABLE);
						NV_Profile_Runtime.LastTestFailedFlag = 1;
				    while (IR_receiving != 0){};
					  Flash_write(); 
					 }
					 
				}
				else if ((System_State_Machine == SHORT_TEST) || (System_State_Machine == LONG_TEST))
				{
				  Lamp_ctrl(DISABLE, 0);                // turn off the lamps return to standby 
  				Remote_Lamp_EN(DISABLE);
	        					
          LED_task(_30s_TEST,DISABLE);
          LED_task(_90M_TEST,DISABLE);
					LED_task(Low_Capacity,DISABLE);	
					
					if (System_State_Machine == LONG_TEST)
				   {
						 NV_Profile_Runtime.LastTestFailedFlag =0;
				     while (IR_receiving != 0){};
					   Flash_write();
					 }
					Charger_ElapsedTime = 0;
					Charger_StartTime = HAL_GetTick() ;            //load the start time for the charger timer 	
 					System_State_Machine = STANDBY;
				}
				
			  break;
			}
		
		case IR_CMD_CFG_SD_ENABLE:     // self test enable
		  {
        
				if(System_State_Machine == STANDBY)
				{
            IR_CMD = 0;
  					NV_Profile_Runtime.SelfDiagnosticEnabled =1;	
						while (IR_receiving != 0){};
					  Flash_write();
				}
				break;
			}
		
		case IR_CMD_CFG_SD_DISABLE:   // self test disable
		  {
				IR_CMD = 0;
				if(System_State_Machine == STANDBY)
				{
					NV_Profile_Runtime.SelfDiagnosticEnabled =0;	
					while (IR_receiving != 0){};
					Flash_write();
				}
				break;
			}
		
		case IR_CMD_CFG_TD_OFF:    // time delay function disabled
		  {                        // time delay function keeps the lamp for several minutes after the AC power comes back. 
				IR_CMD = 0;
				if(System_State_Machine == STANDBY)
				{
  				NV_Profile_Runtime.TimeDelayPeriod	=0;
					while (IR_receiving != 0){};
					Flash_write();
				}
			  break;
			}
		
		case IR_CMD_CFG_TD_5M:    // time delay 5 minute (300seconds)
		  {
				if(System_State_Machine == STANDBY)
				{
       		IR_CMD = 0; 	
				  NV_Profile_Runtime.TimeDelayPeriod	=300;
					while (IR_receiving != 0){};
					Flash_write();
				}
			  break;
			}
		
		case IR_CMD_CFG_TD_10M:   // time delay 10 minute (600seconds)
		  {
       if(System_State_Machine == STANDBY) 
				{
			    IR_CMD = 0; 
					NV_Profile_Runtime.TimeDelayPeriod	=600;	
					while (IR_receiving != 0){};
					Flash_write();
				}
			break;
			}
		
		case IR_CMD_CFG_TD_15M:   // time delay 15 minute (900seconds)
		  {
				IR_CMD = 0;
        if(System_State_Machine == STANDBY)
				{				
				  NV_Profile_Runtime.TimeDelayPeriod	=900;	
					while (IR_receiving != 0){};
					Flash_write();
				}
			break;
			}	
			
		case IR_CMD_CFG_HAS_HEATER:   // heater installed 
		  {
        IR_CMD = 0; 
				if(System_State_Machine == STANDBY)
				{
  				NV_Profile_Runtime.HasHeater =1;	
					while (IR_receiving != 0){};
					Flash_write();
				}
			break;
			}	
			
		case IR_CMD_CFG_NO_HEATER:   // no heater installed 
		  {
				IR_CMD = 0; 
        if(System_State_Machine == STANDBY)
				{				
				  NV_Profile_Runtime.HasHeater =0;	
					while (IR_receiving != 0){};
					Flash_write();
				}
					
			break;
			}	
			
					
		case IR_CMD_SPECIAL_FCN_A :     // restore the default profile to NV memory
		  {
				IR_CMD = 0; 
				if(System_State_Machine == STANDBY)
				{
				  memcpy (&NV_Profile_Runtime, &NV_Profile_Default, sizeof(NV_Profile_Runtime)); 	
			    while (IR_receiving != 0){};
					Flash_write();
					NVIC_SystemReset() ; 
				  
				}
			
			break;
			}		
			
		case IR_CMD_SPECIAL_FCN_B:      // Control the bottom heater for debug
		  {
	      IR_CMD = 0;
				
			
			break;
			}	
			
		case IR_CMD_ENTER_EOL_TEST_MODE:     // the factory final test mode
		  {
		    IR_CMD = 0;
		
			break;
			}	
			
		case IR_CMD_ENTER_USER_DEMO_MODE :      // the demo mode
		  {
				IR_CMD = 0;
		
			break;
			}	
			
		case IR_CMD_PROG_EVO_BODY_8W:          // unit total wattage is 8W
		  {
        IR_CMD = 0; 
				if(System_State_Machine == STANDBY)
				{
					NV_Profile_Runtime.UnitWattage = 8;	
					while (IR_receiving != 0){};
					Flash_write();
				}
			break;
			}	
			
		case IR_CMD_PROG_EVO_BODY_16W:        // unit total wattage is 16W
		  {
        IR_CMD = 0; 
				if(System_State_Machine == STANDBY)
				{
  				NV_Profile_Runtime.UnitWattage = 16;	
					while (IR_receiving != 0){};
					Flash_write();
				}
			break;
			}	
			
		case IR_CMD_PROG_EVO_BODY_32W :     // unit total wattage is 32W
		  {
	      IR_CMD = 0;
				if(System_State_Machine == STANDBY)
				{
          IR_CMD = 0; 
					NV_Profile_Runtime.UnitWattage = 32;	
					while (IR_receiving != 0){};
					Flash_write();
				}
			break;
			}		
			
		case IR_CMD_PROG_EVO_BODY_64W:     // unit total wattage is 64W
		  {
        IR_CMD = 0; 
				if(System_State_Machine == STANDBY)
				{
  				NV_Profile_Runtime.UnitWattage =64;	
					while (IR_receiving != 0){};
					Flash_write();
	      }
			break;
			}	
			
		case IR_CMD_PROG_EVO_HEAD_4W:     // lamp head wattage is 4W
		  {
        IR_CMD = 0; 
				if(System_State_Machine == STANDBY)
				{
  				NV_Profile_Runtime.HeadWattage =4;	
					while (IR_receiving != 0){};
					Flash_write();
					LAMP_Init();
					//NVIC_SystemReset() ;	// reset system code to take the new lamp waltage effect
		    }
			break;
			}	
			
		case IR_CMD_PROG_EVO_HEAD_8W:     // lamp head wattage is 8W
		  {
        IR_CMD = 0;
				if(System_State_Machine == STANDBY)
				{
  				NV_Profile_Runtime.HeadWattage =8;	
					while (IR_receiving != 0){};
					Flash_write();
					LAMP_Init();
			//		NVIC_SystemReset() ;	// reset system code to take the new lamp waltage effect
				}
			break;
			}		
			
		case IR_CMD_PROG_EVO_HEAD_16W:    // lamp head wattage is 16W
		  {
				IR_CMD = 0;
				if(System_State_Machine == STANDBY)
				{
  				NV_Profile_Runtime.HeadWattage =16;	
					while (IR_receiving != 0){};
					Flash_write();
					LAMP_Init();	
			//		NVIC_SystemReset() ;	// reset system code to take the new lamp waltage effect
				}
			break;
			}		
			
		default:
      break;				

		}
	LED_task(IR_Signal,ENABLE);
	}

}





