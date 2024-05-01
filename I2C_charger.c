/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : I2C_charger.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * 2024-2 NAN LIU
  *
  *
  ******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "I2C_charger.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

Power_StateMachine_TypeDef Power_StateMachine = PW_INIT ;

MP2760_Status_TypeDef MP2760_Status;
MP2760_ADC_TypeDef  MP2760_ADC;  
Charge_Stage_TypeDef Charge_Stage;
uint16_t Charging_Current;
uint16_t Charging_Voltage;
uint16_t System_Voltage;
uint16_t Charger_Input_Voltage;
uint16_t Charger_Input_Current;
uint16_t Junction_Temperature;
uint16_t Input_Power;
Battery_Temperature_TypeDef Battery_Temperature;

ACPower_Status_TypeDef AC_Power;
Battery_Status_TypeDef Batt_Status;
PowerError_Status_TypeDef PowerError;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */



/****************************************************************************/
/* 1 byte register read function */
uint8_t I2C_Byte_Read(uint8_t i2c_addr, uint8_t reg)   // read 1 bytes register's value, read twice from 16bit register
{
	uint8_t buf = 0;
	
	HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, &reg, 1, 10); 
	HAL_I2C_Master_Receive(&hi2c1,  i2c_addr, &buf, 1, 10);
	
	return buf;
} 

                                	

/*  1 byte register write function  */

void I2C_Byte_Write(uint8_t i2c_addr, uint8_t reg, uint8_t value) 			// write 1 byte, write twoice to the 16bit register
{
	uint8_t buf[2]= {0};	
	buf[0] = reg;
	buf[1] = value; 

	HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, buf, 2, 10);
		
}

/*  1 word register write function  */

void I2C_Word_Write(uint8_t i2c_addr, uint8_t reg, uint16_t value)			// write 1 word
{
	uint8_t buf[3]= {0};	
	buf[0] = reg;
	buf[1] = (uint8_t)(value & 0xFF); // LSB
	buf[2] = (uint8_t)(value >> 8);   // MSB

	HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, buf, 3, 10);
		
}

/* 1 word register read function */
uint16_t I2C_Word_Read(uint8_t i2c_addr, uint8_t reg)   // read 1 word register's value
{
	uint8_t buf[2] = {0};
	uint16_t data;
	HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, &reg, 1, 10); 
	HAL_I2C_Master_Receive(&hi2c1, i2c_addr, buf, 2, 10);
	data = (buf[1] << 8) | buf[0];
	return data;
} 


// multiple-register read function
void I2C_MultiByte_Read(uint8_t i2c_addr, uint8_t Reg, uint8_t *Reg_Data, uint8_t len)  // Reg: the multiple register's first reg ID
{
	
	HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, &Reg, 1, 10); 
			
	HAL_I2C_Master_Receive(&hi2c1, i2c_addr, Reg_Data, len, 10);
	
}  	

// multiple-register write function
void I2C_MultiByte_Write(uint8_t i2c_addr, uint8_t reg, uint8_t *value, uint8_t len) 	// len <= 72 the maximum register counts
{
	uint8_t buf[100] = {0};
	uint8_t i;
	
	buf[0] = reg;
  
  for(i = 0; i < len; i++)
      buf[i+1]= value[i]; 
	
  HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, buf, (len+1), 10);
	
	
}







/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */



/* **************************** battery processing functions ***************************************/

void Charger_Init(void)
{
		
	I2C_Word_Write(MP2760_I2C_ADDR, RegID_Input_Min_Voltage, Input_Min_Voltage);
	I2C_Word_Write(MP2760_I2C_ADDR, RegID_System_Min_Voltage, System_Min_Voltage);
  I2C_Word_Write(MP2760_I2C_ADDR, RegID_Input_Current_Limit, Input_Current_Limit);
	I2C_Word_Write(MP2760_I2C_ADDR, RegID_Batt_Low_Voltage, Batt_Low_Voltage);
	I2C_Word_Write(MP2760_I2C_ADDR, RegID_Batt_Low_Voltage, Batt_Low_Voltage);
	I2C_Word_Write(MP2760_I2C_ADDR, RegID_Temperature_Protection, Temperature_Protection);
	I2C_Word_Write(MP2760_I2C_ADDR, RegID_Config_Reg_0, Config_Reg_0);
	I2C_Word_Write(MP2760_I2C_ADDR, RegID_Config_Reg_2, Config_Reg_2);
	I2C_Word_Write(MP2760_I2C_ADDR, RegID_Charge_Current, Charge_Current);
	I2C_Word_Write(MP2760_I2C_ADDR, RegID_Batt_Full_Voltage, Batt_Full_Voltage);
	
}



void Charger_Read_Status(void)
{
  MP2760_Status.Val_Status_Fault_Reg_0  		=  I2C_Word_Read( MP2760_I2C_ADDR, RegID_Status_Fault_Reg_0 );
	MP2760_Status.Val_Status_Fault_Reg_1  		=  I2C_Word_Read( MP2760_I2C_ADDR, RegID_Status_Fault_Reg_1 );			  
	MP2760_Status.Val_INT_Mask_Reg_0  				=  I2C_Word_Read( MP2760_I2C_ADDR, RegID_INT_Mask_Reg_0 );	
  MP2760_Status.Val_INT_Mask_Reg_1          =  I2C_Word_Read( MP2760_I2C_ADDR, RegID_INT_Mask_Reg_1 );	
  MP2760_Status.Val_Input_Current_Limit_DAC =  I2C_Word_Read( MP2760_I2C_ADDR, RegID_Input_Current_Limit_DAC );
	
} ;  					
  						
 

void Charger_Read_ADC(void)
{

	MP2760_ADC.Val_ADC_Input_Voltage 					= I2C_Word_Read( MP2760_I2C_ADDR, RegID_ADC_Input_Voltage	);		      
	MP2760_ADC.Val_ADC_Input_Current  				= I2C_Word_Read( MP2760_I2C_ADDR, RegID_ADC_Input_Current	);	      	
	MP2760_ADC.Val_ADC_Batt_Voltage_Cell			= I2C_Word_Read( MP2760_I2C_ADDR, RegID_ADC_Batt_Voltage_Cell	);		  
	MP2760_ADC.Val_ADC_System_Voltage					= I2C_Word_Read( MP2760_I2C_ADDR, RegID_ADC_System_Voltage );    		
	MP2760_ADC.Val_ADC_Batt_Charge_Current		= I2C_Word_Read( MP2760_I2C_ADDR, RegID_ADC_Batt_Charge_Current	);		
	MP2760_ADC.Val_ADC_NTC_Voltage_Ratio			= I2C_Word_Read( MP2760_I2C_ADDR, RegID_ADC_NTC_Voltage_Ratio );  		
	MP2760_ADC.Val_ADC_TS_Voltage_Ratio				= I2C_Word_Read( MP2760_I2C_ADDR, RegID_ADC_TS_Voltage_Ratio );		  	
	MP2760_ADC.Val_ADC_Junction_Temperature		= I2C_Word_Read( MP2760_I2C_ADDR, RegID_ADC_Junction_Temperature );		
	MP2760_ADC.Val_ADC_Batt_Discharge_Current	= I2C_Word_Read( MP2760_I2C_ADDR, RegID_ADC_Batt_Discharge_Current );
	
}

/* big endian to little endian convert*/

void CONVERT_Big_to_Little (uint16_t *pdata_big, uint16_t *pdata_little, uint8_t len)
{

	 uint8_t i = 0;
   for (i=0; i<len; i++)
	 {
	   *(pdata_little+i) = (uint16_t)(*(pdata_big+i) >> 8 & 0x00ff ) ;
     *(pdata_little+i) |=  (uint16_t)(*(pdata_big+i)  << 8 & 0xff00) ; 
	 }
	 
}




//watch dog feed//
void MP2760_Watch_Dog_Feed(void)
{
	 uint16_t buf;
	 buf = I2C_Word_Read( MP2760_I2C_ADDR, RegID_Config_Reg_4 );
	 buf = buf | 0x0200;
	 I2C_Word_Write(MP2760_I2C_ADDR, RegID_Config_Reg_4, buf);
}

void MP2760_Reset(void)
{
	 uint16_t buf;
	 buf = I2C_Word_Read( MP2760_I2C_ADDR, RegID_Config_Reg_4 );
	 buf = buf | 0x0040;
	 I2C_Word_Write(MP2760_I2C_ADDR, RegID_Config_Reg_4, buf);
}	




void Power_Supply_Thread(void)
{
  while(1)
	{
	  switch (Power_StateMachine)
	  	{	
	  	 case PW_INIT:
           {
	  				 
	  				   MP2760_Reset();
	  				   HAL_Delay(100);
	  				   Charger_Init();
	  				   Power_StateMachine = PW_OPERATING;
						   break;
	  			 }
	  	
       case PW_OPERATING:
	  		   {
	  		
	  					Charger_Read_Status();
	  					Charger_Read_ADC();		
	  												
	  					Charge_Stage = (Charge_Stage_TypeDef)((MP2760_Status.Val_Status_Fault_Reg_0 & MP2760_MASK_Charger_stage) /64) ;
	  					
	  					Charging_Current =  (MP2760_ADC.Val_ADC_Batt_Charge_Current)*25/2;
	  					Charging_Voltage =   MP2760_ADC.Val_ADC_Batt_Voltage_Cell*5*3;	
	  					System_Voltage = MP2760_ADC.Val_ADC_System_Voltage*20;	
	  					Charger_Input_Voltage = MP2760_ADC.Val_ADC_Input_Voltage*20;	
	  					Charger_Input_Current = MP2760_ADC.Val_ADC_Input_Current*25/4;	
	  					Junction_Temperature = 314 - (MP2760_ADC.Val_ADC_Junction_Temperature)*0.5703;
	  					Input_Power = Charger_Input_Voltage *	Charger_Input_Current /1000;
	  					Battery_Temperature = (Battery_Temperature_TypeDef)(MP2760_Status.Val_Status_Fault_Reg_1 & MP2760_MASK_NTC_Status) ;
	  					
	  				 /* ******** */	
	  					if ((MP2760_Status.Val_Status_Fault_Reg_0 & (1<<13)) != 0 )
	  							AC_Power =  AC_Good;
	  					else   
	  							AC_Power =  AC_Down;
	           /* ******** */	
	  					
	  					if ((MP2760_Status.Val_Status_Fault_Reg_0 & (1<<10)) != 0 )
	  						Batt_Status = Battery_Removed;
	  					else
	  						Batt_Status = Battery_Normal;
	  					
	  				 /* ******** */		
	  					if ( (MP2760_Status.Val_Status_Fault_Reg_1 & 0x3D70) !=0 )
	  					   {
	  							 PowerError = Power_Error;
	  							 Power_StateMachine = PW_ERROR; 
	  					   }
	  					else 
	  						   PowerError = No_Error;
	
              HAL_Delay(2000);
							break;
	  		   }
        case PW_ERROR:
	  		  {
	  				HAL_Delay(200);
	  				if ( (MP2760_Status.Val_Status_Fault_Reg_1 & 0x3D70) !=0 )
	  					   {
	  							 PowerError = Power_Error;
	  							 
	  					   }
	  				else 
						   {
						     PowerError = No_Error;
								 Power_StateMachine = PW_INIT; 
						   }
						break;
	  		  }	
	  			
        case PW_SHIP:  
	  		  {
	  		    uint16_t temp=0;
	  				temp = temp | 0x2000; 
	  				
	  		    while(1)
	  				{
	  				 I2C_Word_Write(MP2760_I2C_ADDR, RegID_Config_Reg_2, temp);
	  				}
	  		  
	  		  }
		  
	     }
	  
  }
}
/*

uint8_t Check_VBUS()
{
	uint8_t buf;
	buf = BQ25798_Reg_Read(RegID_CHARGER_STATUS_0);
  if (buf &= (uint8_t)BQ25798_CHARGER_STATUS_0_VBUS_PRESENT_STAT)
	 return 1;
	else 
	 return 0;
}

uint8_t Check_VBAT()
{
	uint8_t buf;
	buf = BQ25798_Reg_Read(RegID_CHARGER_STATUS_0);
  if (buf &= (uint8_t)BQ25798_CHARGER_STATUS_2_VBAT_PRESENT_STAT)
	 return 1;
	else 
	 return 0;
}
*/

/* USER CODE END 0 */ 




