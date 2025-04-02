/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx.h"
#include "LED_Process.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TOP_HEATER_EN_Pin GPIO_PIN_13
#define TOP_HEATER_EN_GPIO_Port GPIOC
#define nCHARGE_SHUNT_Pin GPIO_PIN_14
#define nCHARGE_SHUNT_GPIO_Port GPIOC
#define REMOTE_LAMP_EN_Pin GPIO_PIN_15
#define REMOTE_LAMP_EN_GPIO_Port GPIOC
#define REMOTE_LAMP_CURRENT_FB_Pin GPIO_PIN_0
#define REMOTE_LAMP_CURRENT_FB_GPIO_Port GPIOA
#define LAMP_CURRENT_FB_Pin GPIO_PIN_1
#define LAMP_CURRENT_FB_GPIO_Port GPIOA
#define TOP_BATTERY_TEMP_FB_Pin GPIO_PIN_2
#define TOP_BATTERY_TEMP_FB_GPIO_Port GPIOA
#define BOTTOM_BATTERY_TEMP_FB_Pin GPIO_PIN_3
#define BOTTOM_BATTERY_TEMP_FB_GPIO_Port GPIOA
#define RF_CS_Pin GPIO_PIN_4
#define RF_CS_GPIO_Port GPIOA
#define RF_SCK_Pin GPIO_PIN_5
#define RF_SCK_GPIO_Port GPIOA
#define RF_MISO_Pin GPIO_PIN_6
#define RF_MISO_GPIO_Port GPIOA
#define RF_MOSI_Pin GPIO_PIN_7
#define RF_MOSI_GPIO_Port GPIOA
#define BATTERY_VOLTAGE_FB_Pin GPIO_PIN_0
#define BATTERY_VOLTAGE_FB_GPIO_Port GPIOB
#define CHARGER_VOLTAGE_FB_Pin GPIO_PIN_1
#define CHARGER_VOLTAGE_FB_GPIO_Port GPIOB
#define Test_Prob1_Pin GPIO_PIN_10
#define Test_Prob1_GPIO_Port GPIOB
#define Test_Prob2_Pin GPIO_PIN_11
#define Test_Prob2_GPIO_Port GPIOB
#define BATTERY_PSU_EN_Pin GPIO_PIN_12
#define BATTERY_PSU_EN_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_13
#define LED_B_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_15
#define LED_R_GPIO_Port GPIOB
#define nUSB_EN_Pin GPIO_PIN_8
#define nUSB_EN_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define ZERO_CROSS_Pin GPIO_PIN_15
#define ZERO_CROSS_GPIO_Port GPIOA
#define RF_INT_Pin GPIO_PIN_3
#define RF_INT_GPIO_Port GPIOB
#define RF_INT_EXTI_IRQn EXTI3_IRQn
#define IR_INPUT_Pin GPIO_PIN_4
#define IR_INPUT_GPIO_Port GPIOB
#define TEST_BUTTON_Pin GPIO_PIN_5
#define TEST_BUTTON_GPIO_Port GPIOB
#define TEST_BUTTON_EXTI_IRQn EXTI9_5_IRQn
#define CHARGE_PWM_Pin GPIO_PIN_6
#define CHARGE_PWM_GPIO_Port GPIOB
#define LAMP_PWM_Pin GPIO_PIN_7
#define LAMP_PWM_GPIO_Port GPIOB
#define BOTTOM_HEATER_EN_Pin GPIO_PIN_8
#define BOTTOM_HEATER_EN_GPIO_Port GPIOB
#define nRF_RST_Pin GPIO_PIN_9
#define nRF_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// For system main
typedef enum 
{
  STANDBY      = 0x00u, 
  EMERGENCY    = 0x01u,
  SHORT_TEST   = 0x02u,
  LONG_TEST    = 0x03u,
  EOL          = 0x04u,
  DEMO         = 0x05u, 
  SHIP         = 0x06u,		
}System_State_TypeDef;

uint32_t ElapsedTimer_TimeDelay(uint32_t start_time);


// For LED:
#define LED_PWM_CH_BLUE		TIM_CHANNEL_1 
#define LED_PWM_CH_GREEN	     TIM_CHANNEL_2 
#define LED_PWM_CH_RED		TIM_CHANNEL_3 


void ADC_task(void);

// For LAMP
 
#define LAMP_PWM_CH			TIM_CHANNEL_2
#define LAMP_LEVEL_4W 		26
#define LAMP_LEVEL_8W 		48
#define LAMP_LEVEL_16W 		86 //79
	



typedef enum 
{
   _4W  =4u, 
   _8W  =8u, 
   _16W =16u
} LAMP_HEAD_TypeDef ;

#define DIMING_START_TIME          	90u    // starts on 2nd min and 11th min of the emergency state, 10 min period
	
	
void Lamp_ctrl(FunctionalState EN, uint8_t LAMP_WATT);   // LAMP_LEVEL range: 0-99
void Lamp_Diming(void);

void Remote_Lamp_EN(FunctionalState EN);
void Charger_Init(void);

	
typedef enum 
{
  Heater_Off = 0u,
  On_Heating
}Heater_state_TypeDef;	

#define CHARGE_PWM_CH	TIM_CHANNEL_1 
typedef enum 
{
  IDL_CHR,
  PRE_CHR,
  CC_CHR, 
  RC_CHR, 
  Termination_CHR,
}CHR_TypeDef ;
	
typedef enum 
{
  No_Battery, 				// all the battery packs are not installed
  TEMP_IS_Cold,               // temperature is lower  than 5°C,  heater is turned on to heat the pack/packs
  TEMP_IS_Cool,  		  	// temperature is lower  than 10°C, heater is turned on to heat the pack/packs 
  TEMP_IS_Normal,		     // temperature is higher than 10°C, lower than 30°C, heater is turned off
  TEMP_IS_Warm,  		     // temperature is higher than 30°C, lower than 80°C heater is turned off no heatinig   
  TEMP_IS_Hot,                // temperature is higher than 80°C, charge is prohibited, no charge is allowed
}Battery_Temperature_TypeDef;		

#define Batt_Pack_Capacity 	6700 // mAH
#define Chr_Rate_Big_Batt     39   // 22 to 78: fast charge rate; 13 to 39:fload charge rate
#define Chr_Rate_Small_Batt   21   // two bug batter will be doubled
                                    
	
#define V_FullChr	 		3550 // 10950 mV,  3650mV per cell
#define V_ReChr             	3380 //3480 // V_FullChr (3240: -300 mV), 3490:10.86V
#define V_PreChr            	2700 // 8400 mV  
#define V_Low_Capacity        2450 // 7500 mV    less than 5% of the battery full capacity
#define V_Dead_Battery        1950 // 6000 mV    2.0V each cell, battery disonnected
#define V_Low_Temp            1770 // 5500 mV    1.8V each cell

/* temerature measurements
No Batt: 	3880
0°C:  	3080
5°C:  	2870
10°C: 	2300
20°C: 	2250
30°C:     2050
40°C: 	1500
80°C: 	580
*/


#define Temp_No_batt          3880     // no battery is installed
#define Temp_0         		3080     // on this temperature and above, heater is turning on 	
#define Temp_5				2870
#define Temp_10			2300          
#define Temp_20        		2250     // above this point, the heater is turned off	
#define Temp_30			2050     
#define Temp_40             	1500     // above this point, the CC-charge state is not allowed	
#define Temp_80              	580      // too hot, the charge needs to be turned off	
	
#define Button_Tap_TimeMax 	  	1000   //ms
#define Button_Hold_TimeMin     	3000   //ms
#define Button_Release_TimeMax  	300    //ms
#define Button_ShortTap_TimeMax 	50     //ms
#define Button_Press_TimeOut    	10000  //ms

void CHARGE_task(void);                // CHARGE_LEVEL range: 0-99
void HEATER_task(void); 
 
void Scheduler_task(void);
void BATT_TEMP_task(void);

void Button_task(void); 

// For ADC

#define   Remote_LAMP_Current		0
#define	Local_LAMP_Current		1
#define	Top_Batt_Temp			2
#define	Bottom_Batt_Temp		3
#define	Batt_Voltage			4
#define	Charger_Voltage	  	5

uint32_t ElapsedTimer_ADC(uint32_t start_time);

// For Test button


	
typedef enum 
{
  /* 1. each of the pressing duration which is greater than 100ms and less than 1.5s is a tap
     2. multi-taps time interval between the adjacent taps is less than 200ms, any released button longer than 200ms complete the button activity.  
     3. a pressing longer than 1.5s treated as Hold; a Hold shorter than 3.5s is a Hold_Short, longer than 3.5s is a Hold_long		*/
  Idle,
  Single_Tap, 
  Double_Tap,  
  Triple_Tap, 
  Quad_Tap,
  
  Short_Hold,
  Long_Hold, 
  
  Single_Tap_Short_Hold,
  Double_Tap_Short_Hold,
  Triple_Tap_Short_Hold,
  Quad_Tap_Short_Hold,
  
  Single_Tap_Long_Hold,
  Double_Tap_Long_Hold,
  Triple_Tap_Long_Hold,
  Quad_Tap_Long_Hold,
	
} ButtonPressType_TypeDef;
	
	
typedef enum 
{
  Button_Progress_Complete,
  Button_in_Progress,
} Button_Progress_TypeDef ;
	
	
	
	
// clock 
 struct sys_time
{
  uint16_t  ms;
  uint32_t  Sec;
  uint8_t   Day;
  uint8_t   Period;
};



#define second 0
#define minute 1

#pragma pack (1) 

typedef struct                          // total 16 bytes
{
  uint8_t   Flag;                       //  
  uint64_t  _Signature;
  uint8_t   UnitWattage;             	// Total wattage of the unit
  uint8_t   HeadWattage;             	// Lamp uint wattage
  uint8_t   HasHeater;               	// heater installation
  uint8_t   SelfDiagnosticEnabled;      // Self test enable 
  uint16_t  TimeDelayPeriod;          	// Delay after AC resumed, uint: seconds
  uint8_t   LastTestFailedFlag;         // the last testing which was failed
  uint32_t  LocalLampCurrentInit;       // the saved local lamp current value
  uint32_t  RemoteLampCurrentInit;      // the saved remote lamp current value
  
} NV_Profile_TypeDef;


//#pragma pack () 
 



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
