#include "stm32f1xx_hal.h"
#include "pin_description.h"
extern void Error_Handler(void);

void HAL_MspInit(void)
{
	__HAL_RCC_AFIO_CLK_ENABLE();
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);

	__HAL_AFIO_REMAP_SWJ_NOJTAG();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    //GPIO_InitStruct.Pull = GPIO_PULLUP;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi){
	GPIO_InitTypeDef GPIO_InitStruct;
	if(hspi->Instance==SPI1)
	{
		__HAL_RCC_SPI1_CLK_ENABLE();

		/**SPI1 GPIO Configuration
		PB0     ------> SPI1_NSS
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);/**/


		GPIO_InitStruct.Pin = GPIO_PIN_4;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);/**/

		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SPI1_IRQn);
	}
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi){
	if(hspi->Instance==SPI1){
		__HAL_RCC_SPI1_CLK_DISABLE();

		/**SPI1 GPIO Configuration
		PB0     ------> SPI1_NSS
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI
		*/
	//	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
		HAL_NVIC_DisableIRQ(SPI1_IRQn);
	}
}

void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc){
  if(hrtc->Instance==RTC){
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKP_CLK_ENABLE();
    __HAL_RCC_RTC_ENABLE();
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc){
  if(hrtc->Instance==RTC){
    __HAL_RCC_RTC_DISABLE();
    HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
  }
}

void HAL_WWDG_MspInit(WWDG_HandleTypeDef* hwwdg){
  if(hwwdg->Instance==WWDG)
    __HAL_RCC_WWDG_CLK_ENABLE();
}

void HAL_WWDG_MspDeInit(WWDG_HandleTypeDef* hwwdg){
  if(hwwdg->Instance==WWDG)
    __HAL_RCC_WWDG_CLK_DISABLE();
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base){
  if(htim_base->Instance==TIM2){
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
}
