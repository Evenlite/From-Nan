/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "Flash_EEPROM.h"
#include "IR_Zero_Cross.h"
#include "Charge.h"
#include "ADC.h"
#include "stm32f1xx_it.h"

//#include "stm32f1xx_hal_tim.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDR_FLASH_SECTOR_63     ((uint32_t)0x0800FC00)  /* Base address of Sector 63, 1 Kbytes  */
#define FLASH_MEMORY_ADDR         ADDR_FLASH_SECTOR_63

/*
#define Seconds_Per_Day          	86400         // should be 86400-1 *nan
#define Days_Per_Period           28            // should be 28 days *nan
#define Long_Test_Period_Num 		  13            // should be 13: total of 13 period *nan
*/





#define AC_Fail                  	0  
#define AC_Good                 	1
#define Invalid							      2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern LED_Profile_TypeDef  *Pointer_Profile_running;

extern uint16_t time_stamp_start;
extern uint16_t Emergency_ElapsedTime;

uint32_t Time_delay_Elapsed;

System_State_TypeDef System_State_Machine = STANDBY;



struct sys_time System_time ={0,32760,27,0}; // 27 is for UL test -> 24hours later to start the short test

uint8_t Short_test_En = 0;
uint8_t Long_test_En  = 0;



NV_Profile_TypeDef NV_Profile_Default = 
{
  1,
	0x2e782dade227f669,         //signature
  8,             							// Total wattage of the unit, optianl of 8W,16W,32W and 64W
  4,            							// Lamp head wattage: 4W, 8W and 16W
  1,               						// heater installation
  1,										      // Self test enable 
	0,          							  // Delay after AC resumed, uint: seconds, options - 300s, 600s, 900s
  0,                          // the last testing failure flag
 };

extern LED_Profile_TypeDef LED_Profile_90M_TEST;	
extern LED_Profile_TypeDef LED_Profile_30s_TEST;		
extern LED_Profile_TypeDef LED_Profile_Batt_Disconnect; 	
extern LED_Profile_TypeDef LED_Profile_Charger_Fail;
extern LED_Profile_TypeDef LED_Profile_Lamp_Fail; 	
extern LED_Profile_TypeDef LED_Profile_Low_Capacity; 	
extern LED_Profile_TypeDef LED_Profile_Emergency;		
extern LED_Profile_TypeDef LED_Profile_Heating;
extern LED_Profile_TypeDef LED_Profile_Fast_Charge;			
extern LED_Profile_TypeDef LED_Profile_Full_Float_Charge;


int32_t  ChargerTest_VoltRaised =0;   // voltage increas amount during the testing period
uint8_t  Charg_Current_Percentage =0; // 0-99 for PWM value backup to resume the charging current
extern CHR_TypeDef CHR_Stage;
extern uint32_t Charger_StartTime ;
extern uint32_t Charger_ElapsedTime;
 
 
uint32_t ADC_StartTime =0;               // for ADC sampling time gap, unit: millisecond
uint32_t ADC_CurrentTime =0;
uint32_t ADC_ElapsedTime =0;	
 
uint32_t ADC_Local_lamp_FB_init;         //local lamp current measured when power up for diagnostic
uint32_t ADC_Remote_lamp_FB_init;

uint32_t ADC_Local_lamp_FB_reading;      //local lamp current reading when diagnostic
uint32_t ADC_Remote_lamp_FB_reading;

// For diming
uint8_t  Lamp_Diming_level;              // the lamp brightness PWM control for diming, decreaed by 1 on each time
uint8_t  Lamp_Diming_timer;              // the timing for emergency diming time period, diming level decreaes when timer count down to zero 
uint8_t  Lamp_Diming_cnt;                // the decreasing times during the diming period of 16 min 


uint16_t Lamp_Diming_startTime;
uint8_t  Lamp_Dimng_elapsedTime;


extern uint32_t ADC_batt_reading;    
extern uint32_t ADC_charger_reading; 

extern uint32_t ADC_DMA_VALUE[6];
extern uint32_t ADC_CHANNCEL_VAL;



Heater_state_TypeDef Heater_state[2] = {Heater_Off}; // [0]: buttom heater, [1]: top heater



ButtonPressType_TypeDef TEST_Button = Idle;
//Button_State_TypeDef Button_State = Button_Release;

uint32_t ADC_temp_reading[2];                        // temperature ADC value reading buffer, [0]: buttom , [1]: top 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
extern void Flash_write(void);
extern void Flash_read(void);
extern void Flash_clear(void);
extern HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);


NV_Profile_TypeDef NV_Profile_Runtime;

uint32_t time_delta(uint32_t Current_time, uint32_t start_time);  // for calculate the time duration of seconds or minutes

uint32_t ElapsedTimer_ADC(uint32_t start_time);

extern void LED_task(LED_Display_TypeDef Display_Type, FunctionalState Enable);
extern void IR_task(void);
extern uint8_t Zero_Cross_Task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//USB init and reset
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
 
 uint8_t Local_Lamp_Flag =0;
 uint8_t Remote_Lamp_Flag =0;

// Lamp init

void LAMP_Init(void)
{
      Lamp_ctrl(ENABLE, NV_Profile_Runtime.HeadWattage);
  		Remote_Lamp_EN(ENABLE);
			
	    HAL_Delay(400);         // turn on lamp for inducation of reset
      
			uint8_t l =0;
      for (l=0; l < ADC_slide_windows_size+10; l++)           // measure local lamp current
      {
			  ADC_Local_lamp_FB_init = ADC_DMA_VALUE[ADC_LOCAL_LAMP_CUR];
			}	
			ADC_Local_lamp_FB_init = ADC_DMA_VALUE[ADC_LOCAL_LAMP_CUR];	

			for (l=0; l< ADC_slide_windows_size+10;l++)              // measure remote lamp current
      {
			  ADC_Remote_lamp_FB_init = ADC_DMA_VALUE[ADC_REMOTE_LAMP_CUR];
			}
	  	ADC_Remote_lamp_FB_init = ADC_DMA_VALUE[ADC_REMOTE_LAMP_CUR];
			
			Lamp_ctrl(DISABLE, 0);     
			Remote_Lamp_EN(DISABLE);
			
			if (ADC_Local_lamp_FB_init < 255)   
				{
					Local_Lamp_Flag = 0;           // this flag indicate if there is lamp installed-> 0: not installed; 1:installed
				  ADC_Local_lamp_FB_init = 0;
				}             
			else 
			  {
				  Local_Lamp_Flag = 1;
				}
				
			if (ADC_Remote_lamp_FB_init <255)
			 {
			   Remote_Lamp_Flag = 0;
			   ADC_Remote_lamp_FB_init =0;
			 }
			else 
			 {
			  Remote_Lamp_Flag = 1;
			 }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

	 USB_Init();

	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
// ---------- USB init -----------------

  HAL_GPIO_WritePin(nUSB_EN_GPIO_Port,nUSB_EN_Pin,GPIO_PIN_RESET);  //Enable USB : pull up DP
  HAL_GPIO_WritePin(BATTERY_PSU_EN_GPIO_Port,BATTERY_PSU_EN_Pin,GPIO_PIN_SET);  //Enable battery power

// ------------ LED PWM Init ---------------------
 
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);	//Enable  LED PWM Channels  
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);  
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);




	

//------------- lamp PWM init --------------

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);  // Enable Lamp PWM

// ----------- zero cross capture init ----------

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);  //start zero croess duty cycle capture
		HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);

// ----------- IR capture init ----------
		HAL_TIM_Base_Start_IT(&htim3);              // start IR remote capture
    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1); 
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
    HAL_Delay(20);

// ----------- Charge PWM init ----------

    HAL_GPIO_WritePin(nCHARGE_SHUNT_GPIO_Port,nCHARGE_SHUNT_Pin,GPIO_PIN_RESET);  // enable shunt resister
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);	                                    //Enable  charge PWM

// ------------ ADC init -------------
 
	HAL_ADCEx_Calibration_Start(&hadc1);          // ADC calibration
	HAL_ADC_Start_DMA(&hadc1, ADC_DMA_VALUE, 6 ); // start ADC 



// load profile
		
		  memcpy (&NV_Profile_Runtime,(uint32_t*)FLASH_MEMORY_ADDR, sizeof(NV_Profile_Runtime)); // load  profile from Flash Memory to ram
	    if ( NV_Profile_Runtime.Flag == 0xFF )                    // check if the flash memory is blank
		  {
  			memcpy (&NV_Profile_Runtime, &NV_Profile_Default, sizeof(NV_Profile_Runtime)); //the FLASH is blank, load default and save to the flash, this will occure only the first time running the code at factory
        Flash_write();  // save the profile data from runtime ram to flash
      }
           
			__HAL_TIM_SetCompare(&htim1, LED_PWM_CH_RED, LED_LUMNOUS); 
			__HAL_TIM_SetCompare(&htim1, LED_PWM_CH_GREEN, LED_LUMNOUS); 
			__HAL_TIM_SetCompare(&htim1, LED_PWM_CH_BLUE, LED_LUMNOUS); 
			
			
			
			BATT_TEMP_task();
			CHARGE_task(); 
			
	// lamp init, measure the Lamp current as reference for diagnostic
			LAMP_Init();
			
			 
			if (NV_Profile_Runtime.LastTestFailedFlag == 1) 
			{
			  LED_task(Low_Capacity, ENABLE);  // indicate the batter was fail on the last Long Test
	
			}
			 
			 

			
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
 while (1)
 {
 /*  
				USBVcom_printf("ms: %llu\r\n", System_time.ms);
				HAL_Delay(1);
				USBVcom_printf("Second: %u\r\n", System_time.Sec);
				HAL_Delay(1);
	 	    USBVcom_printf("Day: %hu\r\n\r\n", System_time.Day);
 */
		
	  switch (System_State_Machine)
		 {
      
			 case STANDBY: // normal mode
			 {
				LED_Driver(); 
				 
				if (Zero_Cross_Task() == AC_Fail ) 
			   {
					 if(Local_Lamp_Flag ==1) 
           {
					  Lamp_ctrl(ENABLE, NV_Profile_Runtime.HeadWattage);
					 }
					 
					 if(Remote_Lamp_Flag == 1 )
           {
             Remote_Lamp_EN(ENABLE);
           }
					 			     
					 HAL_Delay(200);
           
					 time_stamp_start = System_time.Sec; // emergency timer start 
			   	 System_State_Machine = EMERGENCY;  // has to excute this before disable the heaters for proventing the heater state machine go into the heating stage
				   
					 LED_task(Emergency, ENABLE);
					 ADC_StartTime = HAL_GetTick();     // start ADC timer
           break;
					 
				 }
				 
		    BATT_TEMP_task();   // check the battery temperature, has to be done prior to the CHARGE_Task because it need to report the temperature that used for Charge
         
		    HEATER_task();
        
			  CHARGE_task();      // check and auto control the battery charge
				
				Button_task();      // check the button and start the test mode
				
  			if (NV_Profile_Runtime.SelfDiagnosticEnabled != 0)
				 {
					 Scheduler_task();
				 }
				
				IR_task();          // check the IR remote command
	
	      break;
			 } 


			 case EMERGENCY: 
			  {
			    ADC_ElapsedTime = ElapsedTimer_ADC(ADC_StartTime);           // ADC timing
          if(System_time.Sec < time_stamp_start)                       // Emergency duration timing
					{
						Emergency_ElapsedTime = UINT32_MAX/1000 + System_time.Sec - time_stamp_start +1;
					}
					else 
					{	
					  Emergency_ElapsedTime = System_time.Sec - time_stamp_start;	 				
					}
	/* ------------*/				
				  if ((Emergency_ElapsedTime == 10) && (NV_Profile_Runtime.LastTestFailedFlag ==0)) // write the long test flag for testing the emergency battery 90 min lasting.
					 {
					   NV_Profile_Runtime.LastTestFailedFlag = 1;                   
					   Flash_write();
					 }
	/* ----------- */				 
				  if (( (ADC_ElapsedTime)%1500 == 0) && ADC_ElapsedTime > 1499)   //every 1.5s to read the ADC
				   {
				     ADC_batt_reading = ADC_DMA_VALUE[ADC_BATT_VOLTAGE];          
             if (ADC_batt_reading < V_Low_Capacity )                      // turn off the lamps when the battery is two low
				  	  {
				  		  Lamp_ctrl(DISABLE, 0);  
				  	  }
				   
				     ADC_StartTime = HAL_GetTick() ;   //reload the start time for the next 1.5 Sec delay
				   }
					 
				  Lamp_Diming();	 
					
          if (Zero_Cross_Task() == AC_Good ) 
			     {
            Time_delay_Elapsed = System_time.Sec - time_stamp_start;	
					  
						ADC_batt_reading = ADC_DMA_VALUE[ADC_BATT_VOLTAGE]; //
					   
				    if( Time_delay_Elapsed > NV_Profile_Runtime.TimeDelayPeriod)
				     {
				       Lamp_ctrl(DISABLE, 0);             // turn off the lamps return to standby 
  			    	 Remote_Lamp_EN(DISABLE);
				      						 
               LED_task(Emergency, DISABLE);
             	 CHR_Stage = LOW_BATT;		 
				    	 						 
					  	 NV_Profile_Runtime.LastTestFailedFlag = 0;                                  //clear the fail flag
				       Flash_write();
					  	 
					  	 
					  	 if ((Short_test_En == 1) || ( Long_test_En == 1))   //  when emergency duration is over the short test or long test schedule, delay 1 day.
					  	 {
					  		 System_time.Sec = 0;
								 System_time.Day = 27;
					  		 Short_test_En = 0;
					  		 Long_test_En = 0;
					  	 }
							 							 
				    	 System_State_Machine = STANDBY;
					  	 HAL_Delay(10);
					  	 Charger_ElapsedTime = 0;
					  	 Charger_StartTime = HAL_GetTick();            //load the start time for the charger timer 
				    	 break;
             }
				   } 
				   
				  if ( Emergency_ElapsedTime == 5400 )
				   {
				      NV_Profile_Runtime.LastTestFailedFlag = 0;                                  //clear the fail flag when the emergency last more than 90min( 5400s )
				      Flash_write();
				   }
					 
	         CHARGE_task();
			     LED_Driver();
			     break;
			  }

			 case SHORT_TEST:
			  {
					
					ADC_ElapsedTime = ElapsedTimer_ADC(ADC_StartTime);	
					if (( (ADC_ElapsedTime%1000) <= 9) && ADC_ElapsedTime > 950)   //every 1.0s to measure the ADC
				  {
				    if (ADC_DMA_VALUE[ADC_BATT_VOLTAGE] < V_PreChr)                // 
						 {
							LED_task(Batt_Disconnect, ENABLE); 
						 }
					  else
					   {						
						  LED_task(Batt_Disconnect, DISABLE); 
						 
    				  if(Local_Lamp_Flag ==1) {
						 	  ADC_Local_lamp_FB_reading = ADC_DMA_VALUE[ADC_LOCAL_LAMP_CUR];
						 	  }
						  if(Remote_Lamp_Flag ==1){
						 	  ADC_Remote_lamp_FB_reading = ADC_DMA_VALUE[ADC_REMOTE_LAMP_CUR];
						 	  }
						 
						  if (ADC_DMA_VALUE[ADC_BATT_VOLTAGE] < V_PreChr)      // turn off the lamps when the battery is dead
						    {
						 	    LED_task(Batt_Disconnect, ENABLE); 
						    }
						 
						  if ( (ADC_Local_lamp_FB_reading < (3*ADC_Local_lamp_FB_init/4)) || (ADC_Remote_lamp_FB_reading < (3*ADC_Remote_lamp_FB_init/4)) )
						 	  {
						 		  LED_task(Lamp_Fail, ENABLE); 
						 	  }
						  else 
						 	  {
						 		  LED_task(Lamp_Fail, DISABLE);
						 	  }
					   }				
								
						 ADC_StartTime = HAL_GetTick() ;   //reload the start time for the nest 2s delay
				   }
				
				  if( time_delta(System_time.Sec,time_stamp_start) >= 30 )                    // 30 for 30 second test 
				    { 
				     Lamp_ctrl(DISABLE, 0);  // turn off the lamps return to standby 
  				   Remote_Lamp_EN(DISABLE);
		         						 
             TEST_Button = Idle;
					  
					   LED_task(_30s_TEST, DISABLE);
				     Charger_ElapsedTime = 0;
					   System_State_Machine = STANDBY;
				    }
				 
				  CHARGE_task();	
          IR_task(); 
				  
				  Button_task(); 		
	        LED_Driver();
				  
			    break;
			  }

	     case LONG_TEST:
			  {
			   ADC_ElapsedTime = ElapsedTimer_ADC(ADC_StartTime);			
				 if(System_time.Sec < time_stamp_start)                       // Long test duration timing using the emergency timer
					{
						Emergency_ElapsedTime = UINT32_MAX/1000 + System_time.Sec - time_stamp_start +1;
					}
					else 
					{	
					  Emergency_ElapsedTime = System_time.Sec - time_stamp_start;	 				
					}
					
				 if (( (ADC_ElapsedTime%1000) <= 9) && ADC_ElapsedTime > 950)   //every 1.0s to measure the ADC
				 {
					 if (ADC_DMA_VALUE[ADC_BATT_VOLTAGE] < V_PreChr)                // 
						 {
							LED_task(Batt_Disconnect, ENABLE); 
						 }
					 else
					   {						
						   LED_task(Batt_Disconnect, DISABLE); 
					     if(Local_Lamp_Flag ==1) 
					     	{
				 	        ADC_Local_lamp_FB_reading = ADC_DMA_VALUE[ADC_LOCAL_LAMP_CUR];
				 	       }
				       if(Remote_Lamp_Flag ==1)
					     	{
				 	        ADC_Remote_lamp_FB_reading = ADC_DMA_VALUE[ADC_REMOTE_LAMP_CUR];
				 	       }
					     
				       if ( (ADC_Local_lamp_FB_reading < (3*ADC_Local_lamp_FB_init/4)) || (ADC_Remote_lamp_FB_reading < (3*ADC_Remote_lamp_FB_init/4)) )
				 	       {
				 	     	  LED_task(Lamp_Fail, ENABLE); 
				 	       }
					     else 
					      	  {
					      		  LED_task(Lamp_Fail, DISABLE);
					      	  }
							}
				    ADC_StartTime = HAL_GetTick() ;   //reset the start time for the next 1s delay
				 }
				 
				 if ( ADC_DMA_VALUE[ADC_BATT_VOLTAGE] < V_Low_Capacity )  // fail the 90M test before the 90M is due
				 {
					 Lamp_ctrl(DISABLE, 0);  // turn off the lamps return to standby 
  				 Remote_Lamp_EN(DISABLE);
		       
					 TEST_Button = Idle;	
					 
					 
					 LED_task(Low_Capacity, ENABLE);
					 LED_task(_90M_TEST, DISABLE); 
					 System_State_Machine = STANDBY;
         }
         else                                                                            
				 {                                                                                 // pass the 90M test
					 if (time_delta(System_time.Sec, time_stamp_start) >= 5400)                       //20   // should be 5400 for 90min test *nan
				     {
				      
					      Lamp_ctrl(DISABLE, 0);  // turn off the lamps return to standby 
  				      Remote_Lamp_EN(DISABLE);
		            					   	  
                TEST_Button = Idle;	
					      NV_Profile_Runtime.LastTestFailedFlag = 0;                                  //clear the fail flag
				        Flash_write();
					     	
							                                                  
					   		LED_task(_90M_TEST, DISABLE); 
					   	 	System_State_Machine = STANDBY;
				     }
					 
				 }
				 
				 Lamp_Diming();	 
				 
				 CHARGE_task();
	       IR_task(); 
				 Button_task(); 
		     LED_Driver();
			 
			   break;
			  } 
			 
			 case EOL:
			 {
			  break;
			 } 
			 
			 case DEMO:
			 {
			  break;
			 } 
		 
			 default: break;
	  }	

// -------------------
		
	//	  USBVcom_printf("Freq:%f\r\n", AC_Freq);
//			USBVcom_printf("Duty:%f\r\n\r\n", Duty_Cycle);
//    printf("Duty:%.2f %% \r\n",(float)Duty_Cycle);
//		printf("Freq:%u\r\n", AC_Freq);
		

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 17999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 15;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 143;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 359;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TOP_HEATER_EN_Pin|nCHARGE_SHUNT_Pin|REMOTE_LAMP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Test_Prob_Pin|BATTERY_PSU_EN_Pin|BOTTOM_HEATER_EN_Pin|nRF_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nUSB_EN_GPIO_Port, nUSB_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TOP_HEATER_EN_Pin nCHARGE_SHUNT_Pin REMOTE_LAMP_EN_Pin */
  GPIO_InitStruct.Pin = TOP_HEATER_EN_Pin|nCHARGE_SHUNT_Pin|REMOTE_LAMP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Test_Prob_Pin */
  GPIO_InitStruct.Pin = Test_Prob_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Test_Prob_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BATTERY_PSU_EN_Pin BOTTOM_HEATER_EN_Pin nRF_RST_Pin */
  GPIO_InitStruct.Pin = BATTERY_PSU_EN_Pin|BOTTOM_HEATER_EN_Pin|nRF_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : nUSB_EN_Pin */
  GPIO_InitStruct.Pin = nUSB_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(nUSB_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_INT_Pin */
  GPIO_InitStruct.Pin = RF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_BUTTON_Pin */
  GPIO_InitStruct.Pin = TEST_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TEST_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// For local LAMP


void Lamp_ctrl(FunctionalState EN, uint8_t LAMP_WATT) 
{
  if (EN != 0) 
	   {
				uint8_t walt = LAMP_WATT;
				uint8_t lamp_level;
				uint8_t i;				
				switch (walt)
					{
					  case 0:     {lamp_level = 0               ;break;}
						case _4W: 	{lamp_level = LAMP_LEVEL_4W;   break;}
						case _8W: 	{lamp_level = LAMP_LEVEL_8W;   break;}
						case _16W: 	{lamp_level = LAMP_LEVEL_16W;  break;}
						default: {break;}
					}
					
	      Lamp_Diming_cnt = lamp_level*3/10 + 1;     // calculate that how many times the diming operation will decrease the PWM level on 70% final level
				Lamp_Diming_timer = 960/Lamp_Diming_cnt;	 // the total diming period elapsed 16min(960s), calculate how long the diming will operate to decrese the level down by 1
				Lamp_Diming_level =	lamp_level;            // load the initial lamp level value to diming control variable
				 	
					
				HAL_GPIO_WritePin(nCHARGE_SHUNT_GPIO_Port,nCHARGE_SHUNT_Pin,GPIO_PIN_SET);  // disable shunt resister
					
				for ( i=0; i< lamp_level; i++) 	 
				{ 
					__HAL_TIM_SetCompare(&htim4, LAMP_PWM_CH, i); // the LAMP_LEVEL controls the brightness, value range 0-99, represent n/100 of full power
					HAL_Delay(10);
				}
		 }	
		
  else
	{
		HAL_GPIO_WritePin(nCHARGE_SHUNT_GPIO_Port,nCHARGE_SHUNT_Pin,GPIO_PIN_RESET);  // enable shunt resister
		__HAL_TIM_SetCompare(&htim4, LAMP_PWM_CH, 0);
	}
}

void Lamp_Diming(void)
{
  
	if (Emergency_ElapsedTime == (DIMING_START_TIME -1))
	 {
	  Lamp_Diming_startTime = Emergency_ElapsedTime;
		
	 }
	 
	if ((Emergency_ElapsedTime >= DIMING_START_TIME ) && (Emergency_ElapsedTime < (DIMING_START_TIME + 960 ))) // starts on 37min and stops at 53min (16 min -> 960s)
   {
     Lamp_Dimng_elapsedTime = Emergency_ElapsedTime - Lamp_Diming_startTime;
     if (Lamp_Dimng_elapsedTime == Lamp_Diming_timer)
      {
       Lamp_Diming_level--;
   	
       __HAL_TIM_SetCompare(&htim4, LAMP_PWM_CH, Lamp_Diming_level);
   	 
   	   Lamp_Diming_startTime = Emergency_ElapsedTime ;                  // reload the timer
   	
      }
   }
}	


// Remote LAMP
void Remote_Lamp_EN(FunctionalState EN)
{
  if (EN != 0)
	{ 
		HAL_GPIO_WritePin(nCHARGE_SHUNT_GPIO_Port,nCHARGE_SHUNT_Pin,GPIO_PIN_SET);      // disable shunt resister
		HAL_GPIO_WritePin(REMOTE_LAMP_EN_GPIO_Port,REMOTE_LAMP_EN_Pin,GPIO_PIN_SET);    //Enable remote lamp power
	}
	else
	{
	  HAL_GPIO_WritePin(nCHARGE_SHUNT_GPIO_Port,nCHARGE_SHUNT_Pin,GPIO_PIN_RESET);    // enable shunt resister 
		HAL_GPIO_WritePin(REMOTE_LAMP_EN_GPIO_Port,REMOTE_LAMP_EN_Pin,GPIO_PIN_RESET);  //Disable remote lamp power
	}
}



		 
/*
#define Temp_No_Batt          3840(0xF00)  // no battery is installed
#define Temp_5         				2870(0xB36)  // 5°C the temperature heater is turning on at full power	
#define Temp_20        				2250(0x8CA)  // 20°C   Cheater is turned off	
#define Temp_40             	1500(0x5DC)  // 40°C   CC-charge state is not allowed	
#define Temp_80              	580 (0x244)  // 80°C   hot the charge needs to be turned off	

*/

// the two heater will be turned on at  the same time on the EVO V3.5 and higer version board which equipped with 60W power supply
void HEATER_task(void)
{

	
 if (System_State_Machine == STANDBY && NV_Profile_Runtime.HasHeater != 0)
 {
		// for buttom battery pack
		if ( Batt_Temperature[0] != No_Battery) 
			{
				switch (Heater_state[0])
				{
					case Heater_Off:
					{
						if (Batt_Temperature[0] == TEMP_IS_Cold)
						{
							HAL_GPIO_WritePin(BOTTOM_HEATER_EN_GPIO_Port, BOTTOM_HEATER_EN_Pin, GPIO_PIN_SET); // turn on bottom heater
								
							Heater_state[0] = On_Heating;
							LED_task(Heating, ENABLE);
						}
			
						break;
					}
			
					case On_Heating:
					{
						if ( (Batt_Temperature[0]  == TEMP_IS_Warm) || (Batt_Temperature[0]  == TEMP_IS_Hot) || (Batt_Temperature[0]  == TEMP_IS_Extreme ) ) // heated to the enough temperature and heating is complete
							{
								HAL_GPIO_WritePin(BOTTOM_HEATER_EN_GPIO_Port, BOTTOM_HEATER_EN_Pin, GPIO_PIN_RESET); // turn off the heater
 							  Heater_state[0] = Heater_Off;
								
								if (Heater_state[1] != On_Heating) 
								 {
									 LED_task(Heating, DISABLE);
								 }
							}
						break;
					}
				
					default:
					{
						HAL_GPIO_WritePin(BOTTOM_HEATER_EN_GPIO_Port, BOTTOM_HEATER_EN_Pin, GPIO_PIN_RESET);
						Heater_state[0] = Heater_Off;
						LED_task(Heating, DISABLE);
						break;
					}	
			
				}
			}
		 	else
			{				
				HAL_GPIO_WritePin(BOTTOM_HEATER_EN_GPIO_Port, BOTTOM_HEATER_EN_Pin, GPIO_PIN_RESET);
				Heater_state[0] = Heater_Off;
				if (Heater_state[1] != On_Heating) 
					{
					 LED_task(Heating, DISABLE);
					}
			}

			
			
//		top battery pack, 
			if (Batt_Temperature[1] != No_Battery) 
			 {
				switch (Heater_state[1])
					{
						case Heater_Off: 
						{
							if ( Batt_Temperature[1] == TEMP_IS_Cold)
							{
               	HAL_GPIO_WritePin(TOP_HEATER_EN_GPIO_Port, TOP_HEATER_EN_Pin, GPIO_PIN_SET);       // turn on top heater 
							  
								Heater_state[1] = On_Heating;
								LED_task(Heating, ENABLE);
							}
				
							break;
						}
				
						case On_Heating:
						{
							if ((Batt_Temperature[1]  == TEMP_IS_Warm) || (Batt_Temperature[1]  == TEMP_IS_Hot) || (Batt_Temperature[1]  == TEMP_IS_Extreme)) // heated to the enough temperature and heating is complete
								{
									HAL_GPIO_WritePin(TOP_HEATER_EN_GPIO_Port, TOP_HEATER_EN_Pin, GPIO_PIN_RESET);        // turn off top heater
									Heater_state[1] = Heater_Off;
									
									if (Heater_state[0] != On_Heating) 
									 {
										 LED_task(Heating, DISABLE);
									 }
								}
							break;
						}
					
						default:
						{
							HAL_GPIO_WritePin(TOP_HEATER_EN_GPIO_Port, TOP_HEATER_EN_Pin, GPIO_PIN_RESET);                
							Heater_state[1] = Heater_Off;
							LED_task(Heating, DISABLE);
							break;
						}	
				
					}
			 }
	     else
				 {
			     HAL_GPIO_WritePin(TOP_HEATER_EN_GPIO_Port, TOP_HEATER_EN_Pin, GPIO_PIN_RESET);
				   Heater_state[1] = Heater_Off;
				   if (Heater_state[0] != On_Heating) 
						 {
						  LED_task(Heating, DISABLE);
						 }		 
			   }			
  }		
 else // in all the state other than STANDBY will clear the heaters
	{
    HAL_GPIO_WritePin(BOTTOM_HEATER_EN_GPIO_Port, BOTTOM_HEATER_EN_Pin, GPIO_PIN_RESET); // turn off heaters
		HAL_GPIO_WritePin(TOP_HEATER_EN_GPIO_Port,    TOP_HEATER_EN_Pin,    GPIO_PIN_RESET);
		Heater_state[0] = Heater_Off;
		Heater_state[1] = Heater_Off;
		LED_task(Heating, DISABLE);
  }
}		




// For Test button GPIO interrupt

Button_Progress_TypeDef Button_Porgress = Button_Progress_Complete;

uint32_t Button_Press_StartTime = 0; 
uint32_t Button_Press_Duration = 0;
uint8_t  tap_cnt = 0;
uint8_t  hold_registed = 0;
uint32_t temp_time;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 
  if(GPIO_Pin == TEST_BUTTON_Pin)
  {
    
		Button_Porgress = Button_in_Progress;
  	tap_cnt =0;
 // 	 HAL_GPIO_WritePin(Test_Prob_GPIO_Port,Test_Prob_Pin,GPIO_PIN_SET);  //testing flag
  	while (Button_Porgress == Button_in_Progress )
  	{
  	   Button_Press_StartTime = HAL_GetTick();   // timing start
  		 while(HAL_GPIO_ReadPin( TEST_BUTTON_GPIO_Port, TEST_BUTTON_Pin) == 0 ) // check the press period
       {
HAL_GPIO_WritePin(Test_Prob_GPIO_Port,Test_Prob_Pin,GPIO_PIN_SET);  //testing flag 
				  temp_time = HAL_GetTick();
				 if ( temp_time >= Button_Press_StartTime ) {					 
				     Button_Press_Duration = HAL_GetTick() - Button_Press_StartTime;
				 }
				 else { 
				     Button_Press_Duration = UINT32_MAX + HAL_GetTick() - Button_Press_StartTime + 1;
				 }
				 
       }
HAL_GPIO_WritePin(Test_Prob_GPIO_Port,Test_Prob_Pin,GPIO_PIN_RESET);  //testing flag 
			 
  	   if ((Button_Press_Duration < Button_ShortTap_TimeMax) || (Button_Press_Duration >= Button_Press_TimeOut))    // too short or too long ingnor
  		 { 
  		  tap_cnt =0;
  		 }
  		 else if ((Button_Press_Duration >= Button_ShortTap_TimeMax) &&( Button_Press_Duration < Button_Tap_TimeMax))	 
  		 {
  		  tap_cnt++;
  		 }
  	   else if ((Button_Press_Duration >= Button_Tap_TimeMax) && (Button_Press_Duration < Button_Hold_TimeMin))
  	   {
  		  hold_registed = Short_Hold;				 
  		 } 
  	   else 
  		 {
  			hold_registed = Long_Hold;
  		 }
			 
  	   Button_Press_Duration =0;
			 HAL_Delay(5);
			 Button_Press_StartTime = HAL_GetTick();   // timing start
			 
  		 while (Button_Press_Duration < Button_Release_TimeMax)       //	 check the release time period for 300ms
  		 {
         temp_time = HAL_GetTick();
				 Button_Press_Duration = temp_time - Button_Press_StartTime; 
  			 if (HAL_GPIO_ReadPin( TEST_BUTTON_GPIO_Port, TEST_BUTTON_Pin) == 0)	 // next press detect
  			 {
  				 Button_Porgress = Button_in_Progress;
  		 	   break;
  			 }
				 else
				 {
				  Button_Porgress = Button_Progress_Complete;
				 }
  		 }
			
  	}
  	 
  	if((hold_registed != Short_Hold) && (hold_registed != Long_Hold))
  	 {
  	  if      (tap_cnt ==1) {TEST_Button = Single_Tap;}
  	  else if (tap_cnt ==2) {TEST_Button = Double_Tap;}
      else if (tap_cnt ==3) {TEST_Button = Triple_Tap;}
      else if (tap_cnt ==4) {TEST_Button = Quad_Tap;}
  	  else                  {TEST_Button = Idle;}
  	 }
		else if (hold_registed == Short_Hold)
		 {
  	  if      (tap_cnt ==0) {TEST_Button = Short_Hold;}
      else if (tap_cnt ==1) {TEST_Button = Single_Tap_Short_Hold;}
  	  else if (tap_cnt ==2) {TEST_Button = Double_Tap_Short_Hold;}
      else if (tap_cnt ==3) {TEST_Button = Triple_Tap_Short_Hold;}
      else if (tap_cnt ==4) {TEST_Button = Quad_Tap_Short_Hold;}
  	  else                  {TEST_Button = Idle;}
  	 } 
		else if (hold_registed == Long_Hold)
		 {
  	  if      (tap_cnt ==0) {TEST_Button = Long_Hold;}  
      else if (tap_cnt ==1) {TEST_Button = Single_Tap_Long_Hold;}
  	  else if (tap_cnt ==2) {TEST_Button = Double_Tap_Long_Hold;}
      else if (tap_cnt ==3) {TEST_Button = Triple_Tap_Long_Hold;}
      else if (tap_cnt ==4) {TEST_Button = Quad_Tap_Long_Hold;}
  	  else                  {TEST_Button = Idle;}
  	 } 
		 
  	 tap_cnt =0;
		 hold_registed = 0;
  }

 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin); // reset IRQ flag
 
}

void Button_task(void)
{
	switch (System_State_Machine)
	{
	 case STANDBY:
	 {		
     switch (TEST_Button)
		 { 
			 case Idle:
			  {
			  	break;
			  }
  		 case Single_Tap:
     		{
               			
     			Lamp_ctrl(ENABLE, NV_Profile_Runtime.HeadWattage);
     			Remote_Lamp_EN(ENABLE);
     			
          System_State_Machine = SHORT_TEST;
     			TEST_Button = Idle;  	
					
     			LED_task(_30s_TEST,ENABLE);
     			time_stamp_start = System_time.Sec;
					Charger_ElapsedTime = 0;   
          Charger_StartTime = HAL_GetTick() ;  					
     			ADC_StartTime = HAL_GetTick() ;             //get the starting time to start ADC: unit second
     	    break;
     		}
			 case Double_Tap:
			 case Triple_Tap:
			 case Quad_Tap:
			 case Short_Hold:
			   {
			    TEST_Button = Idle;
			    break;
			   }
			 
			 case Long_Hold:
     		{
          if(( CHR_Stage == RC_CHR) || ( CHR_Stage == Termination_CHR))   // battery is full to run the long test
					{
					  Lamp_ctrl(ENABLE, NV_Profile_Runtime.HeadWattage);
     			  Remote_Lamp_EN(ENABLE);
     			  
     			  System_State_Machine = LONG_TEST;
     			  			
     			  LED_task(_90M_TEST,ENABLE);
     			  NV_Profile_Runtime.LastTestFailedFlag =1;
     			  Flash_write();
     			  
//						System_time.Period = 0; 
//						System_time.Day = 0;	
					  time_stamp_start = System_time.Sec;
						Charger_ElapsedTime = 0;
						Charger_StartTime = HAL_GetTick() ;  
     			  ADC_StartTime = HAL_GetTick() ;      //get the starting time to start ADC 
					}
					else if ( CHR_Stage == LOW_BATT )      // battery is bad needs to be replaced, no need to do the test
					{                                      // set the long test failure flag
					  System_time.Period = 0;
						LED_task( Low_Capacity,ENABLE);
						NV_Profile_Runtime.LastTestFailedFlag = 1;
				    Flash_write(); 
					}
					LED_task(IR_Signal,ENABLE);
					TEST_Button = Idle;	
					break;
     		}
				
			 case Single_Tap_Short_Hold:
			 case Double_Tap_Short_Hold:	
			  {
					TEST_Button = Idle;
					break;
			  } 			 
			 case Triple_Tap_Short_Hold:
				 { 
				  TEST_Button = Idle;
					NV_Profile_Runtime.LastTestFailedFlag = 0;
				  Flash_write(); 
					NVIC_SystemReset();	
          break;					
        }	
			 case Quad_Tap_Short_Hold:
       case Single_Tap_Long_Hold:
			 case Double_Tap_Long_Hold: 
			 case Triple_Tap_Long_Hold:
			 case Quad_Tap_Long_Hold:
        {
					TEST_Button = Idle;
					break;
			  }
			 default: 
				  break;
			}
			  
		break;
		}
	 
		case EMERGENCY:
		 {
		  TEST_Button = Idle;	
			 break;
		 }
				 
		case SHORT_TEST:
		case LONG_TEST:
		 {
			if (TEST_Button != Idle)
	     {
		    TEST_Button = Idle;
		    Lamp_ctrl(DISABLE, 0);  // turn off the lamps return to standby 
  	    Remote_Lamp_EN(DISABLE);
		     			    
    //    System_time.Day = 0;  
		//    System_time.Sec = 0;	
	 	    LED_task(_30s_TEST, DISABLE);
		    LED_task(_90M_TEST, DISABLE);
				LED_task(Low_Capacity, DISABLE);
				 
				NV_Profile_Runtime.LastTestFailedFlag =0;
				Flash_write();
				 
		    Charger_ElapsedTime = 0;
				Charger_StartTime = HAL_GetTick();  
		    System_State_Machine = STANDBY;
			} 
		  break;
		 }	
		default:  
       break;				
	}	
}

// For scheduler


int16_t delta;

uint32_t time_delta(uint32_t Current_time, uint32_t start_time)
{

 delta = Current_time - start_time;
 if (delta < 0)
	 {
		 delta = (UINT32_MAX/1000) + Current_time - start_time + 1; // *nan should be 86400 
   }

 return delta;
}


uint16_t time_temp =0;

void Scheduler_task(void)
{
 
	 	 if ( Short_test_En == 1)
			{
				  Short_test_En = 0;                  // clear the short test active flag
				
				  Lamp_ctrl(ENABLE, NV_Profile_Runtime.HeadWattage);
					Remote_Lamp_EN(ENABLE);
					
													
					System_State_Machine = SHORT_TEST;
										
					time_stamp_start = System_time.Sec;
					
					LED_task(_30s_TEST,ENABLE);
					System_State_Machine = SHORT_TEST;
					ADC_StartTime = HAL_GetTick() ;

			}
			
		 if ( Long_test_En == 1)
			{
        ADC_batt_reading = ADC_DMA_VALUE[ADC_BATT_VOLTAGE];
				if ( ( CHR_Stage == RC_CHR) || (  CHR_Stage == Termination_CHR))    // the battery is full to do the long test or wait until it is fully charged
				{
					 Lamp_ctrl(ENABLE, NV_Profile_Runtime.HeadWattage);
					 Remote_Lamp_EN(ENABLE);
					 
					 time_stamp_start = System_time.Sec;
					 Long_test_En = 0;                          // clear the long test active flag
					 LED_task(_90M_TEST,ENABLE);
				   
					 
					 System_time.Period = 0; 
					 System_time.Day = 0;	    
					 System_State_Machine = LONG_TEST;
					 NV_Profile_Runtime.LastTestFailedFlag =1;   // write the long test flag
				   Flash_write();
					 ADC_StartTime = HAL_GetTick() ;
				}
				
				else if (CHR_Stage == LOW_BATT )             // battery is bad, no need to do the test but set the failure flag
				{
				  System_time.Period = 0; 
					System_time.Day = 0;
					Long_test_En = 0; 
					NV_Profile_Runtime.LastTestFailedFlag =1;   // write the long test flag
				  Flash_write();
				}
				
			}
	 
}

uint32_t ElapsedTimer_ADC(uint32_t start_time)
{
	uint32_t current_time = HAL_GetTick();
  if ( current_time >= start_time)
	{
	  return current_time - start_time;
	}
  else
	{
	  return (UINT32_MAX - start_time + current_time +1) ;
	}
}

/*********************************************************************************************************/

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
