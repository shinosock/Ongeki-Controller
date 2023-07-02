/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <stdio.h>
#define KEYA	HAL_GPIO_ReadPin(GPIOB,KEYA_Pin)//读取按键 A
#define KEYS	HAL_GPIO_ReadPin(GPIOB,KEYS_Pin)//读取按键 S
#define KEYD	HAL_GPIO_ReadPin(GPIOB,KEYD_Pin)//读取按键 D
#define KEYLS	HAL_GPIO_ReadPin(GPIOB,KEYLS_Pin)//读取按键 LSHIFT
#define KEYJ	HAL_GPIO_ReadPin(GPIOB,KEYJ_Pin)//读取按键 A
#define KEYK	HAL_GPIO_ReadPin(GPIOB,KEYK_Pin)//读取按键 S
#define KEYL	HAL_GPIO_ReadPin(GPIOB,KEYL_Pin)//读取按键 D
#define KEYRS	HAL_GPIO_ReadPin(GPIOB,KEYRS_Pin)//读取按键 LSHIFT
#define RXBUFFERSIZE  256
char RxBuffer[RXBUFFERSIZE]; 
uint16_t ADCBuffer[6] = {0, 0, 0, 0, 0, 0}; 
 struct HID_t {
      uint8_t BYTE1;
      uint8_t BYTE2; 
      uint8_t BYTE3;
      uint8_t BYTE4;
			uint8_t BYTE5;
      uint8_t BYTE6;
      uint8_t BYTE7;
      uint8_t BYTE8;
			uint8_t BYTE9;
  };
 unsigned char keyboard[9] = {1,0,0,0,0,0,0,0,0};
extern  USBD_HandleTypeDef *hUsbDeviceFS;
uint32_t ADC_Value;
uint32_t Last_ADC_Value1;
extern UART_HandleTypeDef huart1;   //声明串口
	/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
 
/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}
// struct HID_t keyboardHID_t;
struct HID_t mouseHID_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* uint8_t KEY_Scan(uint8_t mode)
{
	static uint8_t key_up=1;	//按键按松开标志
	if(mode) key_up = 1;
	if(key_up&&(KEYA==0||KEYS==0||KEYD==0||KEYLS==0))
	{
		HAL_Delay(10);	//去抖动
		key_up=0;
		if(KEYA==0)return  KEYA_PRES; else if(KEYS==0)return KEYS_PRES; else if(KEYD==0)return KEYD_PRES;
		else if(KEYLS==0)return LS_PRES;
	}
	else if(KEYA==1&&KEYS==1&&KEYD==1&&KEYLS==1)
	{
	key_up=1; 
	return 0;
	}
		
	return 0;// 无按键按下
} */

uint8_t keyscan(void)
{
  if (KEYA == 0) keyboard[3] = 0x04; else keyboard[3] = 0;
  if (KEYS == 0) keyboard[4] = 0x16; else keyboard[4] = 0;
  if (KEYD == 0) keyboard[5] = 0x07; else keyboard[5] = 0;
  if (KEYJ == 0) keyboard[6] = 0x0d; else keyboard[6] = 0;
  if (KEYK == 0) keyboard[7] = 0x0e; else keyboard[7] = 0;
  if (KEYL == 0) keyboard[8] = 0x0f; else keyboard[8] = 0;
  if (KEYLS == 0) mouseHID_t.BYTE2  |= 1; else keyboard[3] &= 0xfe;
  if (KEYRS == 0) mouseHID_t.BYTE2  |= 2; else keyboard[3] &= 0xfd;
  //ksdvbjjksdfoasdvhbjsdvsdvbjsdvbjkdvbjkls
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// uint8_t flag;
	uint8_t i;
	uint8_t j = 2;
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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
		mouseHID_t.BYTE1 = 2;
		mouseHID_t.BYTE2 = 0;
		mouseHID_t.BYTE3 = Last_ADC_Value1*127/4096;
		mouseHID_t.BYTE4 = 50;
		mouseHID_t.BYTE5 = 0;	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		
//		keyboardHID_t.BYTE1 = 2;
//		keyboardHID_t.BYTE2 = 0;
//		keyboardHID_t.BYTE3 = 127;
//		keyboardHID_t.BYTE4 = 0;
//		keyboardHID_t.BYTE5 = 0;
//		if (flag == 0)
//		{
//			for(i = 0; i<10 ; i++)
//			{
//				HAL_Delay(10);				
//				USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&keyboardHID_t, sizeof(struct keyboardHID_t));
//			}		
//			flag = 1;
//		}
//		
/* 		flag = KEY_Scan(1);
		if (flag)
		{
			if (j ++== 8)
			{
				j = 2;
			}
			switch (flag)
			{
				case KEYA_PRES:
				{
					keyboard[j] = 0x04;		
					keyboard[j - 1] = 0;					
					break;
				}
				case KEYS_PRES:
				{
					keyboard[j] = 0x16;			
					keyboard[j - 1] = 0;					
					break;
				}
			}
		}
		else
		{
			keyboard[3] = 0;
			keyboard[4] = 0;
			keyboard[5] = 0;
			keyboard[6] = 0;
			keyboard[7] = 0;
			keyboard[8] = 0;
		} */
    keyscan();
		HAL_Delay(5);
		USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&keyboard, sizeof(struct HID_t));
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 50);   //等待转换完成，50为最大等待时间，单位为ms
		for(uint8_t i = 10;i>1;i--)
		{
			ADC_Value = HAL_ADC_GetValue(&hadc1);	
			Last_ADC_Value1 = ADC_Value + Last_ADC_Value1;
		}		
		Last_ADC_Value1 = Last_ADC_Value1/10;
		HAL_Delay(5);
		mouseHID_t.BYTE3 = Last_ADC_Value1*127/4096;  //Last_ADC_Value1*127/4096
		USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&mouseHID_t, sizeof(struct HID_t));
//		ADC_Value = HAL_ADC_GetValue(&hadc1);   //获取AD值
////		printf("ADC1 Reading : %d \r\n",ADC_Value);
////		printf("PA3 True Voltage value : %.4f \r\n",ADC_Value*3.3f/4096);		
//		xside=ADCBuffer[5];
//		xside += ADCBuffer[4];
//		xside += ADCBuffer[3];
//		xside -= ADCBuffer[2];
//		xside -= ADCBuffer[1];
//		xside -= ADCBuffer[0];
//		printf("xside : %d \r\n",xside);
			
//		printf("ADC1 Reading : %d \r\n",ADC_Value);
//		printf("PA3 True Voltage value : %.4f \r\n",Last_ADC_Value1*3.3f/4096);	
		
//		HAL_Delay(5);
//		keyboardHID_t.BYTE1 = 1;
//		keyboardHID_t.BYTE2 = 0;
//		keyboardHID_t.BYTE3 = 0;
//		keyboardHID_t.BYTE4 = 0x04;
//		keyboardHID_t.BYTE5 = 0x05;
//		keyboardHID_t.BYTE6 = 0x06;
//		keyboardHID_t.BYTE7 = 0x07;
//		keyboardHID_t.BYTE8 = 0x08;
//		keyboardHID_t.BYTE9 = 0x09;
//		USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&keyboardHID_t, sizeof(struct keyboardHID_t));
//		HAL_Delay(5);
//		keyboardHID_t.BYTE1 = 1;
//		keyboardHID_t.BYTE2 = 0;
//		keyboardHID_t.BYTE3 = 0;
//		keyboardHID_t.BYTE4 = 0;
//		keyboardHID_t.BYTE5 = 0;
//		keyboardHID_t.BYTE6 = 0;
//		keyboardHID_t.BYTE7 = 0;
//		keyboardHID_t.BYTE8 = 0;
//		keyboardHID_t.BYTE9 = 0;
//		USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&keyboardHID_t, sizeof(struct keyboardHID_t));
		
		
//		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
