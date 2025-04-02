/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LED_Process.c
  * @brief          : LED indication function and task program 
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "LED_Process.h"
#include "stm32f1xx_it.h"




/* Define ------------------------------------------------------------*/

#define ADDR_FLASH_SECTOR_63     ((uint32_t)0x0800FC00)  /* Base address of Sector 63, 1 Kbytes  */
#define FLASH_MEMORY_ADDR         ADDR_FLASH_SECTOR_63
/*
#define ADC_REMOTE_LAMP_CUR 	   0
#define ADC_LOCAL_LAMP_CUR 		   1
#define ADC_TOP_BATT_TEMP 		   2
#define ADC_BUT_BATT_TEMP  		   3
#define ADC_BATT_VOLTAGE 			   4
#define ADC_CHR_VOLTAGE 			   5
*/

#define LED_RUN_STATE_STOPPED     0  // define the Run_states
#define LED_RUN_STATE_RUNNING     1

#define LED_BLINK_MODE_ONCE       0
#define LED_BLINK_MODE_CONTINUOUS 1

#define LED_BLINK_RATE_NORMAL     0
#define LED_BLINK_RATE_SLOW       1
#define LED_BLINK_RATE_QUICK      2
																  
#define LED_BLINK_TIMES_1         1
#define LED_BLINK_TIMES_2         2
#define LED_BLINK_TIMES_3         3
#define LED_BLINK_TIMES_4         4
#define LED_BLINK_TIMES_5         5
#define LED_SOLID_ON              255
#define LED_OFF                   0
																 
#define LED_COLOR_RED             1
#define LED_COLOR_GREEN           1
#define LED_COLOR_BLUE            1





/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
extern System_State_TypeDef System_State_Machine;


extern ButtonPressType_TypeDef TEST_Button;
extern TIM_HandleTypeDef htim1;

uint16_t LED_timer = 0;

LED_Display_TypeDef Display_type;

/*
typedef struct
{
  uint8_t Priority;       // 0-8
	uint8_t Run_state;      // run =1; stopped = 0 
	uint8_t Blink_mode;     // 0: once, 1: continue
	uint8_t Blink_rate;     // 0:normal, 1: slow, 2: quick
	uint8_t Blink_times;    // 0 - 5 times, 255: solid on 0:off
	uint8_t Color_red;      // 1:on, 0:off
	uint8_t Color_green;    // 1:on, 0:off
	uint8_t Color_blue;     // 1:on, 0:off
}	LED_Profile_TypeDef;
*/                                                                                                                                                


LED_Profile_TypeDef LED_Profiles[11] = {  // the sequence is the priority from high to low
//Priority  acctive flag:          blink mode:                blink rate :           blink cnt:         color: 
  {0,       LED_RUN_STATE_STOPPED, LED_BLINK_MODE_ONCE,       LED_BLINK_RATE_QUICK,  LED_BLINK_TIMES_4, LED_COLOR_RED, LED_COLOR_GREEN, LED_COLOR_BLUE}, // LED_Profile_IR_Signal	: quick Blinks white 
  {1,       LED_RUN_STATE_STOPPED, LED_BLINK_MODE_CONTINUOUS, LED_BLINK_RATE_SLOW,   LED_BLINK_TIMES_3, LED_COLOR_RED, LED_OFF,         LED_OFF}, 	     // LED_Profile_Low_Capacity: Battery needs to be replaced because it won't be charged
  {2,       LED_RUN_STATE_STOPPED, LED_BLINK_MODE_CONTINUOUS, LED_BLINK_RATE_SLOW,   LED_BLINK_TIMES_1, LED_OFF,       LED_COLOR_GREEN, LED_OFF},		     // LED_Profile_90M_TEST	: Slow Blinks green 
  {3,       LED_RUN_STATE_STOPPED, LED_BLINK_MODE_CONTINUOUS, LED_BLINK_RATE_NORMAL, LED_BLINK_TIMES_1, LED_OFF,       LED_COLOR_GREEN, LED_OFF},		     // LED_Profile_30s_TEST: Blinks green
  {4,       LED_RUN_STATE_STOPPED, LED_BLINK_MODE_CONTINUOUS, LED_BLINK_RATE_NORMAL, LED_BLINK_TIMES_2, LED_COLOR_RED, LED_OFF,         LED_OFF},        // LED_Profile_Batt_Disconnect: Blinks red 2 times
  {5,       LED_RUN_STATE_STOPPED, LED_BLINK_MODE_CONTINUOUS, LED_BLINK_RATE_NORMAL, LED_BLINK_TIMES_3, LED_COLOR_RED, LED_OFF,         LED_OFF},		     // LED_Profile_Charger_Fail: Blinks red 3 times
  {6,       LED_RUN_STATE_STOPPED, LED_BLINK_MODE_CONTINUOUS, LED_BLINK_RATE_NORMAL, LED_BLINK_TIMES_5, LED_COLOR_RED, LED_OFF,         LED_OFF},		     // LED_Profile_Lamp_Fail: Blinks red 5 times
  {7,       LED_RUN_STATE_STOPPED, LED_BLINK_MODE_CONTINUOUS, LED_BLINK_RATE_NORMAL, LED_OFF,           LED_OFF,       LED_OFF,         LED_OFF},		     // LED_Profile_Emergency: Off
  {8,       LED_RUN_STATE_STOPPED, LED_BLINK_MODE_CONTINUOUS, LED_BLINK_RATE_NORMAL, LED_BLINK_TIMES_1, LED_COLOR_RED, LED_COLOR_GREEN, LED_OFF},	       // LED_Profile_Heating: Blinks orange
  {9,       LED_RUN_STATE_STOPPED, LED_BLINK_MODE_CONTINUOUS, LED_BLINK_RATE_NORMAL, LED_SOLID_ON,      LED_COLOR_RED, LED_OFF,         LED_OFF},		     // LED_Profile_Fast_Charge: Solid red
  {10,      LED_RUN_STATE_STOPPED, LED_BLINK_MODE_CONTINUOUS, LED_BLINK_RATE_NORMAL, LED_SOLID_ON,      LED_OFF,       LED_COLOR_GREEN, LED_OFF},	       // LED_Profile_Full_Float_Charge: Solid green
};

LED_Profile_TypeDef  *Pointer_Profile_running;
 
LED_Drive_state_TypeDef LED_drive_state = LED_On;




// For LED driver 
uint8_t  Blink_cnt =0;
uint16_t Blink_On_period =0;
uint16_t Blink_Cycle_period =0;
uint16_t Blink_Interval_period =0;
uint8_t  Blink_Alt_color =0;

uint16_t LED_Brightness_R   =0;
uint16_t LED_Brightness_G   =0;
uint16_t LED_Brightness_B   =0;

LED_Display_TypeDef  Current_Priority ;
uint8_t  Profile_load_Flag[11] = {1};
//uint8_t  Blink_times =0;
uint8_t  Blink_speed =0;
uint8_t  Blink_mode =1;
uint16_t Blink_Color_R =0;
uint16_t Blink_Color_G =0;
uint16_t Blink_Color_B =0;








uint8_t Run_state_check_buf[11] ={0};

//
void LED_Driver(void) // this function will handle the LED display patern accoding to the profile 
{                     // and running permission according to the priority
  
	if ( (Pointer_Profile_running->Blink_times != LED_SOLID_ON) && (Pointer_Profile_running->Blink_times != LED_OFF) )
	{  
			
			switch (LED_drive_state) //excute the color driving
		  {
			  case LED_On:
				{
				  if(LED_timer > Blink_On_period)
				   	{
							__HAL_TIM_SetCompare(&htim1, LED_PWM_CH_RED,   0); 
              __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_BLUE,  0);
					    if (Blink_Alt_color == 1)                            // red/green alternative blink
					      {
								  __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_GREEN, LED_Brightness_R); // use the brightness of red LED to turn on the green LED
								}         
			        else
					      {
									__HAL_TIM_SetCompare(&htim1, LED_PWM_CH_GREEN, 0);
								}
							Blink_cnt--;	
							LED_drive_state = LED_Off;
					  }
					break;
				}
				
				case LED_Off:
				{
				  if(LED_timer > Blink_Cycle_period)
				    {
					  	if ( Blink_cnt > 0 ) // blink cycles are not completed
					  		{
					  		  
									__HAL_TIM_SetCompare(&htim1, LED_PWM_CH_RED,  LED_Brightness_R); 		
	                __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_GREEN,LED_Brightness_G);
	                __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_BLUE, LED_Brightness_B);
									
									
									LED_timer = 0;
					  			LED_drive_state = LED_On;
						 	  }
					    else                // blink cycles have completed
					  		{
					  		   
					  			if (Blink_Alt_color ==1)                   
									  {
									    __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_GREEN, 0);  //turn off the green LED
										}
										
									Blink_cnt = 0;	
									LED_timer =0;
								  LED_drive_state = Cycle_Interval;
						     }					
					  } 
					break;
				}
						
			  case Cycle_Interval:
				{
				  if ( LED_timer > Blink_Interval_period)
		       // interval due, go to the next cycle or disable the current display
					{
					  if (Blink_mode == LED_BLINK_MODE_ONCE)           // run once mode
 					   {
               LED_task( Current_Priority, DISABLE);         // deactive the running profile on this mode
				     }
					  else
					   {						 
					     __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_RED,  LED_Brightness_R); 		
	             __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_GREEN,LED_Brightness_G);
	             __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_BLUE, LED_Brightness_B);
						   
						   Blink_cnt = Pointer_Profile_running->Blink_times;
						   LED_timer=0;
						   LED_drive_state = LED_On;                 // continue the next blink cycle
					   }
					}
	
					break;
				}
			
			  default: 
					break;
				
			}
		}
}


void LED_Init(uint8_t priority_id)
{
  
	if (priority_id == Current_Priority )   // load the profile to run
	{return;}  
 
  Pointer_Profile_running = &LED_Profiles[priority_id];   // set the running profile pointing to the profile location on which is going to run
	
// set basic control varibles	
	
	Current_Priority  = (LED_Display_TypeDef)(*Pointer_Profile_running).Priority;               // the current profile priority
  
	Blink_cnt         =  Pointer_Profile_running->Blink_times;                            			// blink numers in one display type cycle
	Blink_speed       =  Pointer_Profile_running->Blink_rate;							// blink speed
  Blink_mode        =  Pointer_Profile_running->Blink_mode;             // blink mode: single or continue

  
		
// set speical color blink pattern
	if ( Current_Priority == 1)   // blink red/green alternatively
		{
		 Blink_Alt_color =1;
		}
	else if ((Current_Priority == 0)&&(TEST_Button != Idle))
		{
		 Blink_Alt_color =2;
		}
	else
		{
		 Blink_Alt_color =0;
		}
		
// set  color 
	LED_Brightness_R = LED_LUMNOUS * Pointer_Profile_running->Color_red ;
	LED_Brightness_B = LED_LUMNOUS * Pointer_Profile_running->Color_blue;	
	if( Current_Priority == 8 )                                            // this is for heating display color : orange, as the green LED is too bright than others, so dim it down.
		{
			LED_Brightness_G = (LED_LUMNOUS * Pointer_Profile_running->Color_green)/2;
		}
	else if (Blink_Alt_color ==2)
	 {
	   LED_Brightness_G = 0;
		 LED_Brightness_B = 0;
	 }
	else
	  {
		  LED_Brightness_G = LED_LUMNOUS * Pointer_Profile_running->Color_green;
		}
  
// set blink LED turn on time period according to the blink speed	and number of times:
		
    // this is slow blinking: long turn on time and off time period 
		if (Blink_speed == LED_BLINK_RATE_SLOW )                                      
		 { 
			 if ( Blink_Alt_color == 1 )                              
         { 
			     Blink_On_period 			= LED_ON_TIME*2;
           Blink_Cycle_period 	= LED_CYCLE_TIME*2;
		     }
       else
			   {
			    Blink_On_period 		= LED_ON_TIME*4;
          Blink_Cycle_period 	= LED_CYCLE_TIME*4;
			   }
		 }
		 
    // for normal blinking speed
 	  else if(Blink_speed == LED_BLINK_RATE_NORMAL )                                 
		 {
			 if (Blink_cnt == LED_SOLID_ON )                    // solid on/off: the LED on time period equal to the cycle time
		    {
			    Blink_On_period 			= LED_ON_TIME;        
		      Blink_Cycle_period 		= LED_ON_TIME;
		    }
		   else                                                 // for blinking
		    {
		      Blink_On_period 			= LED_ON_TIME;        
		      Blink_Cycle_period 		= LED_CYCLE_TIME;
		    }
		 }	
     // this is quick blinkig speed  
		else if(Blink_speed == LED_BLINK_RATE_QUICK) 
		 {
		   Blink_On_period 		 = LED_ON_TIME/4;
       Blink_Cycle_period  = LED_CYCLE_TIME/4;
		 }

// set the blinking burst gap period according to the blink times and mode
	if( Blink_cnt == LED_SOLID_ON )                       // solid on: no intervel period
			{
			  Blink_Interval_period = 0;                        //  no blink, no gap
		  }
	else if ( Blink_cnt == LED_BLINK_TIMES_1 )                                
			{
			  Blink_Interval_period = 10;                       //one blink, very short intervel time gap 
		  }	
	else if (Current_Priority ==0)
	    {
	      Blink_Interval_period 	= LED_INTERVAL_TIME/10;
	    }
	else
			{
			 Blink_Interval_period 	= LED_INTERVAL_TIME; // blink number of times, has gap of 1 second
			}
	
   __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_RED,  LED_Brightness_R); 		
	 __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_GREEN,LED_Brightness_G);
	 __HAL_TIM_SetCompare(&htim1, LED_PWM_CH_BLUE, LED_Brightness_B);

			
   LED_timer =0;
	 LED_drive_state = LED_On;	
			
	

}



void LED_task(LED_Display_TypeDef Display_Type, FunctionalState Enable) // enable the LED display accoding to conditions
{
	
   switch (Display_Type)
   {
  	case IR_Signal:
  	{
  	  if (Enable !=RESET) {	LED_Profiles[0].Run_state = LED_RUN_STATE_RUNNING; }	 			 // make sure turn it off when leave this mode		
  		else {LED_Profiles[0].Run_state = LED_RUN_STATE_STOPPED;}	
    	
  		break;
  	} 
  	 
  	case Low_Capacity:
  	{
      if (Enable !=RESET) {LED_Profiles[1].Run_state = LED_RUN_STATE_RUNNING; }
  	  else {LED_Profiles[1].Run_state = LED_RUN_STATE_STOPPED;}
  		break;
  	}
  	
  	case _90M_TEST:
  	{
  	  if (Enable !=RESET) {	LED_Profiles[2].Run_state = LED_RUN_STATE_RUNNING; }	 			 // make sure turn it off when leave this mode		
  		else {LED_Profiles[2].Run_state = LED_RUN_STATE_STOPPED;}	
    	
  		break;
  	}
  
  	case _30s_TEST:                                          // make sure turn it off when leave this mode
  	{
  		if (Enable !=RESET) {	LED_Profiles[3].Run_state = LED_RUN_STATE_RUNNING; }	 			 // make sure turn it off when leave this mode		
  		else {LED_Profiles[3].Run_state = LED_RUN_STATE_STOPPED;}						
  				
  		break;
  	}
  
    case Batt_Disconnect:
  	{
      if (Enable !=RESET) {LED_Profiles[4].Run_state = LED_RUN_STATE_RUNNING; }
  	  else {LED_Profiles[4].Run_state = LED_RUN_STATE_STOPPED;}
  		break;
  	}
  
    case Charger_Fail:
  	{
  		if (Enable !=RESET) {LED_Profiles[5].Run_state = LED_RUN_STATE_RUNNING; }
  	  else {LED_Profiles[5].Run_state = LED_RUN_STATE_STOPPED;}
  
  		break;
  	}
  
    case Lamp_Fail:
  	{
  		if (Enable !=RESET) {LED_Profiles[6].Run_state = LED_RUN_STATE_RUNNING; }
  	  else {LED_Profiles[6].Run_state = LED_RUN_STATE_STOPPED;}
  		break;
  	}
  
  	case Emergency:
  	{
  		if (Enable !=RESET) {	LED_Profiles[7].Run_state = LED_RUN_STATE_RUNNING;}	 			 // make sure turn it off when leave this mode		
  		else {LED_Profiles[7].Run_state = LED_RUN_STATE_STOPPED;}	
      
  		break;
  	}
  
  	case Heating:
  	{
  		if (Enable !=RESET) {	LED_Profiles[8].Run_state = LED_RUN_STATE_RUNNING; }	 			 // make sure turn it off when leave this mode		
  		else {LED_Profiles[8].Run_state = LED_RUN_STATE_STOPPED;}					
  		
  		break;
  	}
  	
  	case Fast_Charge:
  	{
  		if (Enable !=RESET) {LED_Profiles[9].Run_state = LED_RUN_STATE_RUNNING; }
  	  else {LED_Profiles[9].Run_state = LED_RUN_STATE_STOPPED;}
  		
  		break;
  	}
  	
    case Full_Float_Charge:
  	{
  		if (Enable !=RESET) {LED_Profiles[10].Run_state = LED_RUN_STATE_RUNNING;}
  	  else {LED_Profiles[10].Run_state = LED_RUN_STATE_STOPPED;}
  		
  		break;
  	}
   	
  	default:
  		break;
   }
 
  uint8_t Priority_ID = 0;
	
  for (Priority_ID =0; Priority_ID < 11; Priority_ID++)                               // find the highest priority profile ID is in active to run 
	 {
    	 
    if ( LED_Profiles[Priority_ID].Run_state == 1 )    
		  {  
			  LED_Init( Priority_ID );
				break; 
			}
 	 }
  
 
}





