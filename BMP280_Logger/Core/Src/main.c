/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFF_SIZE   64  /* Max Received data 1KB */
#define DATA_BUFF_SIZE 256
#define SLEEP_TIME	   60U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LED_ON()	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
#define LED_OFF()	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
#define BMP280_ON()	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
#define BMP280_OFF()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
USBD_HandleTypeDef  hUsbDeviceFS;
volatile uint8_t usb_connected = false;
volatile uint8_t sleep_enable = true;
uint8_t alarm_flag = 0;
volatile int8_t usb_cdc_open = -1;
volatile uint8_t usb_buffer[RX_BUFF_SIZE];
enum STATE
{
	STATE_UNINITIALIZED,
	STATE_ERASING_FLASH,
	STATE_LOGGING,
	STATE_DOWNLOADING,
	STATE_SETTING_TIME,
	STATE_MEMORY_FULL
};

#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_61   			/* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     END_OF_FLASH - 0x10				/* End @ of user Flash area */
#define EPOCH_ADDR 				FLASH_USER_END_ADDR - 0x20
#define EPOCH_ADDR_BACKUP		FLASH_USER_END_ADDR - 0x10
#define FLASH_DATA_END_ADDR		FLASH_USER_END_ADDR - 0x30
#define FLASH_CURRENT_ADDR		FLASH_USER_END_ADDR - 0x20 + 4U

BMP280_HandleTypedef bmp280;

float pressure;
float temperature;
float humidity;

int32_t i32temperature;
uint32_t u32pressure;
uint32_t u32humidity;

uint16_t size;
uint8_t Data[DATA_BUFF_SIZE];
uint16_t sample = 1;

volatile uint32_t current_address = FLASH_USER_START_ADDR;

volatile enum STATE current_state = STATE_UNINITIALIZED;

//bool time_set = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void setAlarm(void);
void resetTime(void);
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc); //This routine is called when alarm interrupt occurs
void enterSleepMode(void);
void enterStopMode(void);
void enterStandbyMode(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void configurePA11Interrupt(void);
uint8_t isUSBConnected();
uint32_t readFlash(uint32_t addr);
uint32_t readFlashCurrentAddress();
void writeFlashCurrentAddress(uint32_t addr);
void writeEpochToFlash(uint32_t epoch_time);
void serial_menu_usb();
HAL_StatusTypeDef eraseFlash(uint32_t startAddress, uint32_t endAddress);
uint32_t writeFlashU16(uint32_t startAddress, uint16_t data);
uint32_t writeFlashU32(uint32_t startAddress, uint32_t data);
void sendStoredData(uint32_t start, uint32_t end);
void sleepTest(void);
void stopTest(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	const uint16_t DELAY_TIME = 1000;

	uint32_t data;
	data = readFlash(current_address);

	while ((data != 0xFFFFFFFF) && (current_address < FLASH_DATA_END_ADDR))
	{
		current_address += 4;
		data = readFlash(current_address);
	}

	resetTime();

	BMP280_ON();

	LED_ON();
	HAL_Delay(DELAY_TIME);
	LED_OFF();

	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	while (!bmp280_init(&bmp280, &bmp280.params))
	{
		size = sprintf((char*) Data, "BMP280 initialization failed\r\n");
		HAL_UART_Transmit(&huart1, Data, size, HAL_MAX_DELAY);
		CDC_Transmit_FS(Data, size);
		HAL_Delay(DELAY_TIME * 2);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (current_state == STATE_MEMORY_FULL)
		{
			char msg[] = "Memory full\r\n";
			while (CDC_Transmit_FS((uint8_t*) msg, strlen(msg)) != USBD_OK)
			{
				HAL_Delay(100);
			}
			serial_menu_usb();
			HAL_Delay(DELAY_TIME);
		}
		else
		{
			if (alarm_flag)
			{
				alarm_flag = 0;
				LED_ON();
				resetTime();
				setAlarm();
				// Read sensor
				bmp280_read_fixed(&bmp280, &i32temperature, &u32pressure,
						&u32humidity);

				// Write to flash
				current_address += writeFlashU16(current_address,
						(uint16_t) i32temperature);
			}
		}

		// Check if memory is full
		if (current_address >= FLASH_DATA_END_ADDR + 0x0E)
		{
			current_state = STATE_MEMORY_FULL;
		}

		if (usb_cdc_open == 1)
		{
			serial_menu_usb();
		}

		sleep_enable = !(isUSBConnected());

		if (sleep_enable)
		{
			usb_connected = 0;
			USBD_Stop(&hUsbDeviceFS);
			configurePA11Interrupt();
			LED_OFF();
			BMP280_OFF();
			enterStopMode();			// ~100uA
			// System clock set to HSI RC after waking from stop mode
			SystemClock_Config();
			HAL_ResumeTick();
			MX_USB_DEVICE_Init();
			LED_ON();
			BMP280_ON();
			MX_I2C1_Init();
			// Initialize sensor after stop mode
			bmp280_init_default_params(&bmp280.params);
			bmp280.addr = BMP280_I2C_ADDRESS_0;
			bmp280.i2c = &hi2c1;

			while (!bmp280_init(&bmp280, &bmp280.params))
			{
				size = sprintf((char*) Data, "BMP280 initialization failed\r\n");
				CDC_Transmit_FS(Data, size);
				HAL_Delay(100);
			}

			alarm_flag = 1;
			HAL_Delay(500);
		}
		else
		{
			LED_ON();
			HAL_Delay(DELAY_TIME);
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void EXTI15_10_IRQHandler(void)
{
//	if ((EXTI->PR & EXTI_PR_PR14) != 0 || (EXTI->PR & EXTI_PR_PR11) != 0) /* Check line 11 has triggered the IT */
	if ((EXTI->PR & EXTI_PR_PR11) != 0) /* Check line 11 has triggered the IT */
	{
		usb_connected = 1;
//		EXTI->PR |= EXTI_PR_PR14; /* Clear the pending bit */
		EXTI->PR |= EXTI_PR_PR11; /* Clear the pending bit */
	}
	EXTI->PR |= 0U;
	NVIC_DisableIRQ(EXTI15_10_IRQn);
}

static void configurePA11Interrupt(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/*Configure GPIO pin : PA11 */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	AFIO->EXTICR[3] = AFIO_EXTICR3_EXTI11_PA;

	EXTI->IMR = EXTI_IMR_MR11;
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

uint8_t isUSBConnected()
{
	uint8_t connected = 0;
	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
	{
		connected = 1;
	}
	return (connected);
}

void writeEpochToFlash(uint32_t epoch_time)
{
	writeFlashU32(EPOCH_ADDR, epoch_time);
	writeFlashU32(EPOCH_ADDR_BACKUP, epoch_time);
}


void serial_menu_usb()
{
	char menu[] = "d - Download data\r\nl - Start logging\r\n";

	CDC_Transmit_FS((uint8_t*) menu, strlen(menu));

	size = strlen(usb_buffer);

	if (size != 0)
	{
//		enum STATE previous_state;
		switch (usb_buffer[0])
		{
		case 'd':
//			current_state = STATE_DOWNLOADING;
			sendStoredData(FLASH_USER_START_ADDR, current_address);
			memset(Data, 0, DATA_BUFF_SIZE);
			memset(usb_buffer, 0, RX_BUFF_SIZE);
			eraseFlash(FLASH_USER_START_ADDR, END_OF_FLASH);
			current_address = FLASH_USER_START_ADDR;
			break;
		case 'l':
//			previous_state = current_state;
//			current_state = STATE_SETTING_TIME;
			size = sprintf(Data, "Set time\r\nEnter epoch time: ");
			while (CDC_Transmit_FS(Data, size) != USBD_OK)
			{
				HAL_Delay(100);
			}
			size = sprintf(Data, "\r\n");
			CDC_Transmit_FS(Data, size);
			//			CDC_Transmit_FS((char *)'\n', 1);
			memset(Data, 0, DATA_BUFF_SIZE);
			memset(usb_buffer, 0, RX_BUFF_SIZE);
			// Delay until epoch time received - 10 characters
			for (uint8_t i = 0; i < 60; i++)
			{
				if (strlen(usb_buffer) > 10)
					break;
				HAL_Delay(1000);
			}
			if (strlen(usb_buffer) < 10)
			{
				size = sprintf(Data, "\r\nNo input received\r\n");
				while (CDC_Transmit_FS(Data, size) != USBD_OK)
				{
					HAL_Delay(100);
				}
			}
			else
			{
				CDC_Transmit_FS(usb_buffer, strlen(usb_buffer));
				uint32_t epoch_time;
				sscanf(usb_buffer, "%zu", &epoch_time);
				// TODO: Check epoch time is valid
				current_state = STATE_ERASING_FLASH;
				size = sprintf(Data, "Erasing flash...\r\n");
				while (CDC_Transmit_FS(Data, size) != USBD_OK)
				{
					HAL_Delay(100);
				}
				eraseFlash(FLASH_USER_START_ADDR, END_OF_FLASH);
				writeEpochToFlash(epoch_time);
				// TODO: Verify epoch time saved to flash
				size = sprintf(Data, "Time saved! - %u\r\n", epoch_time);
				while (CDC_Transmit_FS(Data, size) != USBD_OK)
				{
					HAL_Delay(100);
				}
				size = sprintf(Data, "Logging started\r\n");
				while (CDC_Transmit_FS(Data, size) != USBD_OK)
				{
					HAL_Delay(100);
				}
				current_state = STATE_LOGGING;
				current_address = FLASH_USER_START_ADDR;
				// Debugging
				sample = 1;
			}
			break;
		default:
			break;
		}
	}
}

void sendStoredData(uint32_t start, uint32_t end)
{
	int32_t data = -1;
	int16_t temp1;
	int16_t temp2;

	uint16_t sample_no = 1;

	while (start < end)
	{
		data = readFlash(start);

		temp2 = data >> 16;
		temp1 = (uint16_t) data;

		if (temp1 == -1)
			break;

		size = sprintf((char*) Data, "%d, %.2f\r\n", sample_no,
				(float) (temp1 / 100.0f));
		sample_no++;

		CDC_Transmit_FS(Data, size);

		if (temp2 == -1)
			break;

		size = sprintf((char*) Data, "%d, %.2f\r\n", sample_no,
				(float) (temp2 / 100.0f));
		sample_no++;

		CDC_Transmit_FS(Data, size);

		start += 4;
	}
}

void sleepTest(void)
{
	while (1)
	{
		enterSleepMode();
	}
}

void stopTest(void)
{
	while (1)
	{
		enterStopMode();
	}
}

uint32_t readFlash(uint32_t addr)
{
	uint32_t data = *(__IO uint32_t*) (addr);
	return data;
}

uint32_t readFlashCurrentAddress()
{
	return readFlash(FLASH_CURRENT_ADDR);
}
void writeFlashCurrentAddress(uint32_t addr)
{
	//	eraseFlash();
	writeFlashU32(FLASH_CURRENT_ADDR, addr);
}

HAL_StatusTypeDef eraseFlash(uint32_t startAddress, uint32_t endAddress)
{
	HAL_FLASH_Unlock();
	HAL_StatusTypeDef status = HAL_ERROR;

	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = startAddress;
	EraseInitStruct.NbPages = (endAddress - startAddress) / FLASH_PAGE_SIZE;
	uint32_t PAGEError = 0;

	status = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);//Fill FF, as -1.

	HAL_FLASH_Lock();
	return status;
}

uint32_t writeFlashU16(uint32_t startAddress, uint16_t data)
{
	uint32_t return_value = 0;
	if (startAddress + 2 >= END_OF_FLASH) { return (0); }
	HAL_FLASH_Unlock();

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, startAddress, data)
			== HAL_OK)
	{
		return_value = 2;
	}

	HAL_FLASH_Lock();
	return (return_value);
}

uint32_t writeFlashU32(uint32_t startAddress, uint32_t data)
{
	uint32_t return_value = 0;
	if (startAddress + 4 >= END_OF_FLASH) { return (0); }
	HAL_FLASH_Unlock();

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startAddress, data) == HAL_OK)
	{
		return_value = 4;
	}

	HAL_FLASH_Lock();
	return (return_value);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	alarm_flag = 1;
}

void enterSleepMode(void)
{
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);
}

void enterStopMode(void)
{
	EXTI->EMR = EXTI_EMR_MR17;				// event unmask line 17 (RTC)
	EXTI->RTSR = EXTI_RTSR_TR17;			// rising edge detection

	PWR->CR &= ~PWR_CR_PDDS;				// Enter stop mode when the CPU enters deepsleep.
	PWR->CR |= PWR_CR_LPDS; 				// Voltage regulator in low-power mode during Stop mode
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;			// sleep mode

	RCC->APB1ENR |= (RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN); //Enable the power and backup interface clocks
    RCC->BDCR |=  RCC_BDCR_BDRST;			// reset backup domain
    RCC->BDCR &= ~RCC_BDCR_BDRST;			// reset backup domain

	PWR->CR |= PWR_CR_DBP;             	  	// enable access to the backup registers and the RTC.
	/* Note: The LSEON, LSEBYP, RTCSEL and RTCEN bits of the Backup domain control register (RCC_BDCR)
	 * are in the Backup domain. As a result, after Reset, these bits are write- protected and the DBP bit
	 * in the Power control register (PWR_CR) has to be set before these can be modified.
	 * These bits are only reset after a Backup domain Reset. Any internal or external Reset will not have
	 * any effect on these bits. */

	RCC->BDCR |= RCC_BDCR_LSEON;			// external low freq oscillator
	while(!(RCC->BDCR & RCC_BDCR_LSERDY)) asm("nop"); 	// Wait for LSE ready

//	RCC->CSR |= RCC_CSR_LSION;				// internal low freq oscillator
//	while(!(RCC->CSR & RCC_CSR_LSIRDY)); 	// Wait for LSI ready

//	RCC->BDCR |= RCC_BDCR_RTCSEL_LSI;		// LSI source
	RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;		// LSE source
	RCC->BDCR |= RCC_BDCR_RTCEN;			// RTC on

	RTC->CRL &= (uint16_t)~RTC_FLAG_RSF;	// clear RSF flag
	while (!(RTC->CRL & RTC_CRL_RSF));		// Wait for the RSF to be set by hardware

	// config mode to set prescaler and alarm setting
	while (!(RTC->CRL & RTC_CRL_RTOFF));			// wait until RTOFF = 1
	RTC->CRL |= RTC_CRL_CNF;						// enter configuration mode
	RTC->PRLH = (uint16_t)0;
	RTC->PRLL = (uint16_t)32767;					// RTC prescaler - RTCCLK = 32.768 kHz = 0x7FFF = 1 second
	RTC->ALRH = (uint16_t)0;
	RTC->ALRL = (uint16_t)SLEEP_TIME;						// alarm N seconds
	RTC->CRL &= (uint16_t)~((uint16_t)RTC_CRL_CNF); // exit configuration mode
	while (!(RTC->CRL & RTC_CRL_RTOFF));			// wait until RTOFF = 1

	EXTI->PR &= EXTI_PR_PR17;							// reset pending interrupts
	RTC->CRL &= (uint16_t)~((uint16_t)RTC_CRL_ALRF);	// clear alarm flag

	__WFE();								// enter stop mode
}

void enterStandbyMode(void)
{
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnterSTANDBYMode();
}

void setAlarm(void)
{
	RTC_AlarmTypeDef sAlarm;

	sAlarm.AlarmTime.Hours = 0U;
	sAlarm.AlarmTime.Minutes = 0U;
	sAlarm.AlarmTime.Seconds = SLEEP_TIME;
	sAlarm.Alarm = RTC_ALARM_A;

	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}

void resetTime(void)
{
	RTC_TimeTypeDef sTime;

	sTime.Hours = 0U;
	sTime.Minutes = 0U;
	sTime.Seconds = 0U;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
