/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    IR.h
  ******************************************************************************
  
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IR_Zero_Cross_H
#define IR_Zero_Cross_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include <string.h>


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */




void IR_task(void);




/* USER CODE END EC */


/* USER CODE BEGIN Private defines */

// for IR

#define IR_CMD_RESET 			     (short)0x0D80  // restart and reset error code
#define IR_CMD_SHORT_TEST 			(short)0x0D82 
#define IR_CMD_LONG_TEST            	(short)0x0D88 

#define IR_CMD_CFG_SD_ENABLE 			(short)0x0DA0 
#define IR_CMD_CFG_SD_DISABLE 		(short)0x0DA1
#define IR_CMD_CFG_TD_OFF 			(short)0x0DA2
#define IR_CMD_CFG_TD_5M 			(short)0x0DA5
#define IR_CMD_CFG_TD_10M 			(short)0x0DA4
#define IR_CMD_CFG_TD_15M 			(short)0x0DA3
#define IR_CMD_CFG_HAS_HEATER 		(short)0x0DE0
#define IR_CMD_CFG_NO_HEATER 			(short)0x0DE1
	
#define IR_CMD_SPECIAL_FCN_A 			(short)0x0DE4 // restore the defuault NV profile
#define IR_CMD_SPECIAL_FCN_B 			(short)0x0DE5  
#define IR_CMD_SPECIAL_FCN_C 			(short)0x0DE6  

#define IR_CMD_ENTER_EOL_TEST_MODE 	(short)0x0DF0 
#define IR_CMD_ENTER_USER_DEMO_MODE	(short)0x0DF2 
#define IR_CMD_ENTER_USER_SHIP_MODE	(short)0x0DF3 

#define IR_CMD_PROG_EVO_HEAD_4W 		(short)0x0DC0
#define IR_CMD_PROG_EVO_HEAD_8W 		(short)0x0DC1
#define IR_CMD_PROG_EVO_HEAD_16W 		(short)0x0DC2
#define IR_CMD_PROG_EVO_HEAD_0W 		(short)0x0DC3

#define IR_CMD_PROG_EVO_BODY_8W 		(short)0x0DC8 // defined but not used in this product
#define IR_CMD_PROG_EVO_BODY_16W 		(short)0x0DC9
#define IR_CMD_PROG_EVO_BODY_32W 		(short)0x0DCA
#define IR_CMD_PROG_EVO_BODY_64W 		(short)0x0DCB


#define AC_Fail                  	     0  
#define AC_Good                 	     1
#define Invalid                         2

#define AC_FailVoltage          	     40.9    // measured zero-cross duty cycle value; based on 75% of 120V/AC - 90V (old value on V3.5 is 37.6)
#define AC_GoodVoltage                  49.7   // measured zero-cross duty cycle value; based on 85% of 120V/AC - 102V (old value on V3.5 is 47.5)

uint8_t Zero_Cross_Task(void);







/* USER CODE END Private defines */



/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */







