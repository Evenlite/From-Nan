/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    LED_Process.h
  ******************************************************************************
  
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LED_Process_H
#define LED_Process_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

	
typedef enum // LED control state
{
  LED_On,  
	LED_Off,  
	Cycle_Interval, 
}	LED_Drive_state_TypeDef;


typedef struct
{
  uint8_t Priority;
	uint8_t Run_state;      // run =1; stopped = 0
	uint8_t Blink_mode;     // 0: once, 1: repeat
	uint8_t Blink_rate;     // 0:slow, 1: fast
	uint8_t Blink_times;    // 0 - 6 times, 255: solid on, 0:off
	uint8_t Color_red;
	uint8_t Color_green;
	uint8_t Color_blue;
}	LED_Profile_TypeDef;

typedef enum 
{
 IR_Signal              = 0x00U,
 Low_Capacity           = 0x01U, 		
 _90M_TEST 							= 0x02U,					
 _30s_TEST							= 0x03U,
 Batt_Disconnect				= 0x04U,	
 Charger_Fail						= 0x05U,
 Lamp_Fail							= 0x06U, 
 Emergency							= 0x07U,			
 Heating								= 0x08U,	
 Fast_Charge						= 0x09U, 	
 Full_Float_Charge			= 0x0AU,
 			
}	LED_Display_TypeDef;


void LED_Driver(void);	

void LED_task(LED_Display_TypeDef Display_Type, FunctionalState Enable) ;


/* USER CODE END EC */


/* USER CODE BEGIN Private defines */

#define LED_level_0 		0
#define LED_level_1 		1
#define LED_level_2 		2
#define LED_level_3 		3
#define LED_level_4			4
#define LED_level_5 		5
#define LED_level_6			6
#define LED_level_7			7
#define LED_level_8			8
#define LED_level_9 		9
#define LED_level_10 		10
#define LED_level_11 		11
#define LED_level_12 		12
#define LED_level_13 		13
#define LED_level_14		14
#define LED_level_15 		15
#define LED_LUMNOUS     LED_level_8
 
/*

#define LED_Profile_IR_Signal  				0
#define LED_Profile_Low_Capacity   		1
#define LED_Profile_90M_TEST   				2
#define LED_Profile_30M_TEST   				3
#define LED_Profile_Batt_Disconnct   	4
#define LED_Profile_Charger_Fail   		5
#define LED_Profile_Lamp_Fail   			6
#define LED_Profile_Emergency         7
#define LED_Profile_Heating   				8
#define LED_Profile_Fast_Charge   		9
#define LED_Profile_Full_Float_Charge 10

*/

#define LED_ON_TIME     	    300   // the LED turn on time period: unit ms
#define LED_CYCLE_TIME  	    500   // LED one blink period
#define LED_INTERVAL_TIME     2000  // interval time between the two whole blink cycles. for example: the time between two 3 blinks, this is for telling the blink numbers during continuos display.









/* USER CODE END Private defines */



/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */







