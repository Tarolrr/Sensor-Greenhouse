#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "tm_stm32_ds18b20.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern RTC_HandleTypeDef hrtc;

extern volatile bool operationDone;
extern volatile uint8_t opType;
extern volatile uint8_t opResult;
extern TM_OneWire_t	hOneWire;

void NMI_Handler(void){}
void HardFault_Handler(void){while (1);}
void MemManage_Handler(void) {while (1);}
void BusFault_Handler(void) {while (1);}
void UsageFault_Handler(void) {while (1);}
void DebugMon_Handler(void) {}

void SysTick_Handler(void){
	osSystickHandler();
}

void SPI1_IRQHandler(void){
  HAL_SPI_IRQHandler(&hspi1);
}

void TIM1_UP_IRQHandler(void){
  HAL_TIM_IRQHandler(&htim1);
}

void RTC_Alarm_IRQHandler(void){
  HAL_RTC_AlarmIRQHandler(&hrtc);
}

uint8_t tmp = 0;

void TIM2_IRQHandler(void){
	/*__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_DISABLE(&htim2);
	switch(opType){
		case 0:
			switch(tmp){
				case 0:
					ONEWIRE_HIGH(&hOneWire);
					ONEWIRE_DELAY(67);
					tmp++;
					break;
				case 1:
					opResult = HAL_GPIO_ReadPin(hOneWire.GPIOx, hOneWire.GPIO_Pin);
					ONEWIRE_DELAY(450);
					tmp++;
					break;
				case 2:
					operationDone = true;
					tmp = 0;
			}
			break;
		case 1:
			switch(tmp){
				case 0:
					ONEWIRE_HIGH(&hOneWire);
					if(opResult)
						ONEWIRE_DELAY(85);
					else
						ONEWIRE_DELAY(10);
					tmp++;
					break;
				case 1:
					operationDone = true;
					tmp = 0;
			}
			break;
		case 2:
			switch(tmp){
				case 0:
					ONEWIRE_HIGH(&hOneWire);
					ONEWIRE_DELAY(9);
					tmp++;
					break;
				case 1:
					opResult = HAL_GPIO_ReadPin(hOneWire.GPIOx, hOneWire.GPIO_Pin);
					ONEWIRE_DELAY(80);
					tmp++;
					break;
				case 2:
					operationDone = true;
					tmp = 0;
			}
	}*/


}
