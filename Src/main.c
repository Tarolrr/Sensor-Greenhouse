//V1.1.1b:
//	-added defines for two types of temperature sensor
//V1.1.0b:
//	-another sensor used (AM2320)
//V1.1.0:
//	-non-blocking operation of DS18B20 library (using TIM2). Needs serious refactoring
//	-defines for debugging and using various versions of hardware.
//V1.0.3:
//	-added watchdog and corresponding wake up in 20 sec intervals
//V1.0.2:
//	-added reset in measurement loop
//V1.0.1:

//V1.0.0 - Original version
//TODO stopLed is unnecessary
//TODO unused warnings from static keyword
//TODO nonblocking sx tx
#include "stm32f1xx_hal.h"
#include "SX1278Drv.h"
#include <string.h>
#include "main.h"
#include "cmsis_os.h"

#define SENSOR_AM2320
#define SENSOR_DS18B20_
#define MySTM_
#define IWD_Enable_

#ifdef SENSOR_DS18B20
	#ifdef SENSOR_AM2320
	#error "Choose one of the sensors"
	#endif
#endif

#ifdef SENSOR_DS18B20
#include "tm_stm32_ds18b20.h"
#endif
I2C_HandleTypeDef hi2c1;
IWDG_HandleTypeDef hiwdg;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
osThreadId hMainTask;
osTimerId hRxTimer;
osTimerId hAlertTimer;
RTC_HandleTypeDef hrtc;
RTC_AlarmTypeDef hMainAlarm;
RTC_AlarmTypeDef hIWDAlarm;
#ifdef SENSOR_DS18B20
TM_OneWire_t	hOneWire;
#endif
volatile bool alerted;
SX1278Drv_LoRaConfiguration cfg;
uint8_t DS18B20ROM[8];
float temperature;
uint8_t sendedMessageCount;
volatile bool deepSleep = false;
volatile bool tim2Update = false;
bool dateChanged;
uint16_t crc16(uint8_t *ptr, uint8_t len);
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
#ifdef SENSOR_AM2320
static void MX_I2C1_Init(void);
#endif
void mainTaskFxn(void const * argument);
static void RxTimerCallback(void const * argument);
static void alertTimerCallback(void const * argument);
void I2C_ClearBusyFlagErratum(struct I2C_Module* i2c);

bool handleAlarms();
bool addTimeToAlarm(RTC_AlarmTypeDef *a, uint8_t h, uint8_t m, uint8_t s);
int8_t compareAlarms(RTC_AlarmTypeDef *a, RTC_AlarmTypeDef *b);

#ifdef DEBUG_I2C

#define I2C_TIMEOUT_FLAG          ((uint32_t)35)     /*!< Timeout 35 ms */
#define I2C_TIMEOUT_ADDR_SLAVE    ((uint32_t)10000)  /*!< Timeout 10 s  */
#define I2C_TIMEOUT_BUSY_FLAG     ((uint32_t)10000)  /*!< Timeout 10 s  */

#endif
int main(void){


	HAL_Init();

	SystemClock_Config();
	HAL_Delay(2000);
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_RTC_Init();
#ifdef SENSOR_DS18B20
	MX_TIM2_Init();
#endif


#ifdef SENSOR_AM2320
	MX_I2C1_Init();

	struct I2C_Module i2c;
	i2c.instance = hi2c1;
	i2c.sclPin = GPIO_PIN_6;
	i2c.sdaPin = GPIO_PIN_7;
	i2c.sclPort = GPIOB;
	i2c.sdaPort = GPIOB;

	I2C_ClearBusyFlagErratum(&i2c);
	MX_I2C1_Init();
	uint8_t data[3] = {0x03, 0x02, 0x02};
	HAL_StatusTypeDef s1 = HAL_I2C_Master_Transmit(&hi2c1,0xB8,data,3,-1);

#endif

	#ifdef IWD_Enable
	MX_IWDG_Init();

	HAL_IWDG_Start(&hiwdg);

	#endif

#ifdef SENSOR_DS18B20

	TM_OneWire_Init(&hOneWire, GPIOB, GPIO_PIN_7);

	while(!TM_OneWire_First(&hOneWire));
		TM_OneWire_GetFullROM(&hOneWire,DS18B20ROM);

#endif
	cfg.bw = SX1278Drv_RegLoRaModemConfig1_BW_125;
	cfg.cr = SX1278Drv_RegLoRaModemConfig1_CR_4_8;
	cfg.crc = SX1278Drv_RegLoRaModemConfig2_PayloadCrc_ON;
	#ifdef MySTM
	cfg.frequency = 434e6;
	#else
	cfg.frequency = 868e6;
	#endif
	cfg.hdrMode = SX1278Drv_RegLoRaModemConfig1_HdrMode_Explicit;
	cfg.power = 10;
	cfg.preambleLength = 20;
	cfg.sf = SX1278Drv_RegLoRaModemConfig2_SF_12;
	cfg.spi = &hspi1;
	#ifdef MySTM
	cfg.spi_css_pin = &SPICSMyPin;
	#else
	cfg.spi_css_pin = &SPICSPin;
	#endif
	//cfg.rx_led = &LoRaTxRxPin;
	#ifdef MySTM
	cfg.tx_led = &LoRaTxRxPin;
	#else
	cfg.rx_en = &LoRaRxEnPin;
	cfg.tx_en = &LoRaTxEnPin;
	#endif
	cfg.sleepInIdle = true;

	uint16_t coordAddress = 0;

	SX1278Drv_Init(&cfg);
	SX1278Drv_SetAdresses(0, &coordAddress, 1);
	SX1278Drv_Config();
	SX1278Drv_Suspend();

	osThreadDef(MainTask, mainTaskFxn, osPriorityNormal, 0, 512);
	hMainTask = osThreadCreate(osThread(MainTask), NULL);

	osTimerDef(RxTimer, RxTimerCallback);
	hRxTimer = osTimerCreate(osTimer(RxTimer), osTimerOnce, NULL);

	osTimerDef(AlertTimer, alertTimerCallback);
	hAlertTimer = osTimerCreate(osTimer(AlertTimer), osTimerOnce, NULL);

	osKernelStart();

	return 0;
}

void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		Error_Handler();

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
		Error_Handler();

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		Error_Handler();

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void){
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
		Error_Handler();
	GPIO_PIN_SET(&SPICSPin);
}

static void MX_GPIO_Init(void){
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
}

static void MX_RTC_Init(void){
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
    Error_Handler();

  sTime.Hours 		= 0;
  sTime.Minutes 	= 0;
  sTime.Seconds 	= 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    Error_Handler();

  DateToUpdate.WeekDay 	= RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month 	= RTC_MONTH_JANUARY;
  DateToUpdate.Date 	= 1;
  DateToUpdate.Year 	= 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
    Error_Handler();
}

static void MX_IWDG_Init(void){
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    Error_Handler();
}

/* TIM2 init function */
static void MX_TIM2_Init(void){
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    Error_Handler();
}

#ifdef SENSOR_AM2320
static void MX_I2C1_Init(void){
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    Error_Handler();
}
#endif

void Error_Handler(void){
  while(1);
}

#ifdef SENSOR_AM2320
uint16_t crc16(uint8_t *ptr, uint8_t len){
	uint16_t crc =0xFFFF;
	uint8_t i;
    while(len--)
    {
        crc ^=*ptr++;
        for(i=0;i<8;i++)
        {
        	if(crc & 0x01){
                crc>>=1;
                crc^=0xA001;
            }else
                crc>>=1;
        }
    }
    return crc;
}
#endif

void mainTaskFxn(void const * argument){

	#ifdef SENSOR_DS18B20

	uint32_t timer;
	TM_DS18B20_StartAll(&hOneWire);

	timer = HAL_GetTick();
	while(1){
		if(HAL_GetTick() - timer > 1000)
			NVIC_SystemReset();
		if(TM_DS18B20_Read(&hOneWire,DS18B20ROM,&temperature))
			break;
	}

	#endif

	for(;;){

		if(handleAlarms()){

			if(deepSleep){
				deepSleep = false;

				RCC_OscInitTypeDef RCC_OscInitStruct;
				RCC_ClkInitTypeDef RCC_ClkInitStruct;

				RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
				RCC_OscInitStruct.HSEState = RCC_HSE_ON;
				RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
				RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
				RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
				RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
				if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
					Error_Handler();

				RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
				RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
				if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
					Error_Handler();
			}

			#ifdef SENSOR_AM2320

			int16_t tmpTemp;
			uint8_t data[6] = {0x03, 0x02, 0x02};
			uint16_t rxCRC;
			while(1){
				HAL_StatusTypeDef s1 = HAL_I2C_Master_Transmit(&hi2c1,0xB8,data,3,-1);
				HAL_StatusTypeDef s2 = HAL_I2C_Master_Transmit(&hi2c1,0xB8,data,3,-1);
				HAL_Delay(2);
				HAL_StatusTypeDef s3 = HAL_I2C_Master_Receive(&hi2c1,0xB8,data,6,-1);
				memcpy((uint8_t *)&rxCRC,data+4,2);
				if(rxCRC == crc16(data,4))
					break;
			}

			tmpTemp = data[3] | (data[2] << 8);
			temperature = (float)(tmpTemp)/10;

			#endif

			#ifdef SENSOR_DS18B20

			TM_DS18B20_StartAll(&hOneWire);

			timer = HAL_GetTick();
			while(1){
				if(HAL_GetTick() - timer > 1000)
					NVIC_SystemReset();
				if(TM_DS18B20_Read(&hOneWire,DS18B20ROM,&temperature))
					break;
			}

			if(temperature > 80)
							continue;

			#endif

			SX1278Drv_Resume();

			sendedMessageCount = 0;
			osTimerStart(hRxTimer,SX1278Drv_GetRandomDelay(MinimumRxTimeout,1000));
		}
		#ifdef IWD_Enable
		HAL_IWDG_Refresh(&hiwdg);
		#endif
		#ifdef DEBUG_WO_SENSOR
		GPIO_PIN_SET(&LoRaTxRxPin);
		#endif
		vTaskSuspend(hMainTask);
		#ifdef DEBUG_WO_SENSOR
		GPIO_PIN_RESET(&LoRaTxRxPin);
		#endif

	}
}

bool handleAlarms(){
	if((compareAlarms(&hMainAlarm, &hIWDAlarm) <= 0) ^ dateChanged){
		if(alerted)
			dateChanged ^= addTimeToAlarm(&hMainAlarm, 0, AlertWakeupPeriodInMinutes, 0);
		else
			dateChanged ^= addTimeToAlarm(&hMainAlarm, 0, NormalWakeupPeriodInMinutes, 0);

		return true;
	}
	dateChanged ^= addTimeToAlarm(&hIWDAlarm, 0, 0, 20);
	HAL_RTC_SetAlarm_IT(&hrtc,&hIWDAlarm,RTC_FORMAT_BIN);
	return false;
}

static void RxTimerCallback(void const * argument){
	#ifdef IWD_Enable
	HAL_IWDG_Refresh(&hiwdg);
	#endif

	if(SX1278Drv_IsBusy())
		return;

	if(sendedMessageCount == RetryCount){
		HAL_RTC_GetTime(&hrtc,&hIWDAlarm.AlarmTime,RTC_FORMAT_BIN);
		dateChanged ^= addTimeToAlarm(&hIWDAlarm, 0, 0, 20);
		HAL_RTC_SetAlarm_IT(&hrtc,&hIWDAlarm,RTC_FORMAT_BIN);

		SX1278Drv_Suspend();
		return;
	}

	sendedMessageCount++;
	LoRa_Message msg;
	msg.address = 0;
	msg.payloadLength = sizeof(temperature);
	memcpy(msg.payload, &temperature, msg.payloadLength);
	SX1278Drv_SendMessage(&msg);
}

static void alertTimerCallback(void const * argument){
	alerted = false;
}

void SX1278Drv_LoRaRxCallback(LoRa_Message *msg){
	#ifdef IWD_Enable
	HAL_IWDG_Refresh(&hiwdg);
	#endif

	if(sendedMessageCount == 0){
		SX1278Drv_LoRaRxError();
		return;
	}

	if(msg->payloadLength != 1){
		SX1278Drv_LoRaRxError();
		return;
	}

	osTimerStop(hRxTimer);
	osTimerStop(hAlertTimer);

	if(msg->payload[0] == 1){
		osTimerStart(hAlertTimer,AlertWakeupPeriodInMinutes*3*60000);
		alerted = true;
	}
	else if(msg->payload[0] == 0)
		alerted = false;

	HAL_RTC_GetTime(&hrtc,&hIWDAlarm.AlarmTime,RTC_FORMAT_BIN);
	dateChanged ^= addTimeToAlarm(&hIWDAlarm, 0, 0, 20);
	HAL_RTC_SetAlarm_IT(&hrtc,&hIWDAlarm,RTC_FORMAT_BIN);

	SX1278Drv_Suspend();
}

void SX1278Drv_LoRaRxError(){
	osTimerStop(hRxTimer);
	osTimerStart(hRxTimer,SX1278Drv_GetRandomDelay(MinimumRxTimeout,1000));
}

void SX1278Drv_LoRaTxCallback(LoRa_Message *msg){
	osTimerStop(hRxTimer);
	osTimerStart(hRxTimer,SX1278Drv_GetRandomDelay(MinimumRxTimeout,1000));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM1)
    HAL_IncTick();
  if (htim->Instance == TIM2){
	  tim2Update = true;
	  HAL_TIM_Base_Stop_IT(htim);
  }
}

bool addTimeToAlarm(RTC_AlarmTypeDef *a, uint8_t h, uint8_t m, uint8_t s){
	a->AlarmTime.Seconds += s;
	if(a->AlarmTime.Seconds >= 60){
		a->AlarmTime.Seconds -= 60;
		a->AlarmTime.Minutes++;
	}
	a->AlarmTime.Minutes += m;
	if(a->AlarmTime.Minutes >= 60){
		a->AlarmTime.Minutes -= 60;
		a->AlarmTime.Hours++;
	}
	a->AlarmTime.Hours += h;
	if(a->AlarmTime.Hours >= 24){
		a->AlarmTime.Hours -= 24;
		return true;
	}
	return false;
}

int8_t compareAlarms(RTC_AlarmTypeDef *a, RTC_AlarmTypeDef *b){
	if(a->AlarmTime.Hours > b->AlarmTime.Hours)
		return 1;
	else if(a->AlarmTime.Hours < b->AlarmTime.Hours)
		return -1;
	else{
		if(a->AlarmTime.Minutes > b->AlarmTime.Minutes)
			return 1;
		else if(a->AlarmTime.Minutes < b->AlarmTime.Minutes)
			return -1;
		else{
			if(a->AlarmTime.Seconds > b->AlarmTime.Seconds)
				return 1;
			else if(a->AlarmTime.Seconds < b->AlarmTime.Seconds)
				return -1;
			else
				return 0;
		}
	}
}



void I2C_ClearBusyFlagErratum(struct I2C_Module* i2c)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // 1. Clear PE bit.
  i2c->instance.Instance->CR1 &= ~(0x0001);

  //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_OD;
  //GPIO_InitStructure.Alternate    = I2C_PIN_MAP;
  //GPIO_InitStructure.Pull         = GPIO_PULLUP;
  GPIO_InitStructure.Speed        = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 3. Check SCL and SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET);

  //  5. Check SDA Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET);

  //  7. Check SCL Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  // 9. Check SCL High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 11. Check SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
  GPIO_InitStructure.Mode         = GPIO_MODE_AF_OD;
 // GPIO_InitStructure.Alternate    = I2C_PIN_MAP;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

  // 13. Set SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 |= 0x8000;

  asm("nop");

  // 14. Clear SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 &= ~0x8000;

  asm("nop");

  // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
  i2c->instance.Instance->CR1 |= 0x0001;

  // Call initialization function.
  HAL_I2C_Init(&(i2c->instance));
}

