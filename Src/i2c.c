/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#define ACCEL_ADD  (0x19 << 1)
#define CTRL_REG1 (0x20)

#define COLOR_SENSOR_ADD 0x52 //

#define ENABLE_REG_ADD 0xa0 // register address + command bits 1010000
#define ATIME_REG_ADD 0xa1 // register address + command bits
#define WTIME_REG_ADD 0xa3 // register address + command bits
#define CONFIG_REG_ADD 0xad // register address + command bits
#define CONTROL_REG_ADD 0xaf // register address + command bits
#define ID_REF_ADD 0xb2 // register address + command bits
#define COLOR_DATA_REG 0xb4 // register address + command bit


uint8_t i2cWriteBuff[10];
uint8_t i2cReadBuff[10];

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void initAccel(void){
	i2cWriteBuff[0]= CTRL_REG1;
	//this value 0x47 will enable x,y and z
	// and it will put sensor in normal mode instead low power
	// it will put at 50 hrz sample rate
	i2cWriteBuff[1]= 0x47; 
	
	HAL_I2C_Master_Transmit(&hi2c1, ACCEL_ADD,i2cWriteBuff,2,100);
}

volatile static signed short accelX=0;
volatile static signed short accelY=0;
volatile static signed short accelZ=0;
void readAccel(void){
	i2cWriteBuff[0]=0x28; //very bad practice please hash define and call it DATA_START_ADD
	i2cWriteBuff[0]= i2cWriteBuff[0]|0x80;
	HAL_I2C_Master_Transmit(&hi2c1, ACCEL_ADD, i2cWriteBuff, 1,100);
	HAL_I2C_Master_Receive(&hi2c1, ACCEL_ADD,i2cReadBuff, 6,100);
	
	accelX=(signed short) i2cReadBuff[1]; //reconstruct the accel value from two bytes 
	accelX= (accelX<<8)|i2cReadBuff[0];
	
	accelY=(signed short) i2cReadBuff[3]; //reconstruct the accel value from two bytes 
	accelY= (accelY<<8)|i2cReadBuff[2];

	accelZ=(signed short) i2cReadBuff[5]; //reconstruct the accel value from two bytes 
	accelZ= (accelZ<<8)|i2cReadBuff[4];
	
}

//====
//Color Sensor Functions

void initColorSensor(void)
{
  i2cWriteBuff[0] = ATIME_REG_ADD;
	i2cWriteBuff[1] = 0x10;
	HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADD,i2cWriteBuff, 2, 100);

	i2cWriteBuff[0] = CONFIG_REG_ADD;
	i2cWriteBuff[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADD,i2cWriteBuff, 2, 100);

	i2cWriteBuff[0] = CONTROL_REG_ADD;
	i2cWriteBuff[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADD,i2cWriteBuff, 2, 100);
	  
	i2cWriteBuff[0] = ENABLE_REG_ADD;
	i2cWriteBuff[1] = 0x03;
	HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADD,i2cWriteBuff, 2, 100);
}

/*
Reads the register values for clear, red, green, and blue.
*/
volatile static uint16_t  redColor = 0;
volatile static uint16_t greenColor = 0;
volatile static uint16_t blueColor = 0;
void readColorSensor(void)
{
	i2cWriteBuff[0] = COLOR_DATA_REG;
	HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADD,i2cWriteBuff, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, COLOR_SENSOR_ADD, i2cReadBuff,8, 100);
	
	redColor = (uint16_t)i2cReadBuff[3];
	redColor=(redColor<<8)|i2cReadBuff[2];
	
	greenColor = (uint16_t)i2cReadBuff[5];
	greenColor=(greenColor<<8)|i2cReadBuff[4];
	
	blueColor = (uint16_t)i2cReadBuff[7];
	blueColor=(blueColor<<8)|i2cReadBuff[6];
	
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
