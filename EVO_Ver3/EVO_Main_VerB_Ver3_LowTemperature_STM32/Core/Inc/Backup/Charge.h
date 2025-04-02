/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Charge.h
  ******************************************************************************
  
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef Charge_H
#define Charge_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "ADC.h"
#include <string.h>
//#include "LED_Process.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */



/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


extern System_State_TypeDef System_State_Machine;

extern Battery_Temperature_TypeDef Batt_Temperature[2];  // [0]: buttom batt temperature, [1]: top batt temperature




/* USER CODE END EC */


/* USER CODE BEGIN Private defines */

// For Charging 
	










/* USER CODE END Private defines */



/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* Charge_H */







