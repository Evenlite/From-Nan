/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ADC.h
  ******************************************************************************
  
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ADC_H
#define ADC_H


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





/* USER CODE END EC */


/* USER CODE BEGIN Private defines */

// for ADC

#define ADC_slide_windows_size    4
#define ADC_REMOTE_LAMP_CUR 	    0  // ADC channel define
#define ADC_LOCAL_LAMP_CUR 		    1
#define ADC_TOP_BATT_TEMP 		    2
#define ADC_BUT_BATT_TEMP  		    3
#define ADC_BATT_VOLTAGE 			    4
#define ADC_CHR_VOLTAGE 			    5







/* USER CODE END Private defines */



/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */



extern ADC_HandleTypeDef hadc1;


 



/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* A_H */







