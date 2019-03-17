/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "one_wire.h"
#include "spi_gpio.h"
#include "ff.h"
#include "sour_fs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);

  UART_HandleTypeDef huart;

  huart.Instance = USART3;
  huart.Init.BaudRate = 115200;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits = UART_STOPBITS_1;
  huart.Init.Parity = UART_PARITY_NONE;
  huart.Init.Mode = UART_MODE_TX_RX;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE; 
  huart.Init.OverSampling = UART_OVERSAMPLING_8;

  HAL_UART_Init(&huart);

   // ONE_WIRE_GPIO_Port->ODR &= ~(ONE_WIRE_Pin);
   // ONE_WIRE_GPIO_Port->MODER &= (3 << (ONE_WIRE_Pin * 2));
  char tempStr[256];
  int len;

  init_spi_gpio();

  init_sd_card();

  FATFS fs;
  // DIR root_dir;
  // FILINFO file_info;
  FRESULT res;

  res = f_mount(&fs, "", 0);
  if (res != FR_OK) {
    while (1);
  }

  uint32_t trial_num;

  trial_num = create_trial();

  if (trial_num == 0) {
    while(1);
  }

  struct TrialEntry entry;

  entry.ts = 1;
  entry.temp_dn[0] = 1000;
  entry.temp_dn[1] = 1001;
  entry.temp_dn[2] = 1002;
  entry.temp_dn[3] = 1003;
  entry.temp_dn[4] = 1004;
  entry.have_img = false;
  entry.img_num = 0;

  if(!add_trial_entry(trial_num, &entry, NULL, 0)) {
    while (1);
  }

  entry.ts = 2;
  entry.temp_dn[0] = 1100;
  entry.temp_dn[1] = 1101;
  entry.temp_dn[2] = 1102;
  entry.temp_dn[3] = 1103;
  entry.temp_dn[4] = 1104;
  entry.have_img = true;
  entry.img_num = 1;

  uint8_t dummy_img[] = "MyJpg";

  if (!add_trial_entry(trial_num, &entry, dummy_img, sizeof(dummy_img))) {
    while (1);
  }

  entry.ts = 3;
  entry.temp_dn[0] = 1200;
  entry.temp_dn[1] = 1201;
  entry.temp_dn[2] = 1202;
  entry.temp_dn[3] = 1203;
  entry.temp_dn[4] = 1204;
  entry.have_img = false;
  entry.img_num = 0;

  if (!add_trial_entry(trial_num, &entry, NULL, 0)) {
    while (1);
  }

  while (1);

  // res = f_opendir(&root_dir, "/");
  // if (res != FR_OK) {
  //   while (1);
  // }

  // do {

  //   f_readdir(&root_dir, &file_info);
  //   if (res != FR_OK) {
  //     break;
  //   }

  //   if (file_info.fname[0] != '\0') {
  //     len = snprintf(tempStr, 256, "/%s\r\n", file_info.fname);
  //     HAL_UART_Transmit(&huart, tempStr, len, 10000);
  //   }


  // } while (file_info.fname[0] != '\0');

  while(1);

  one_wire_init();

  // one_wire_reset();

  // one_wire_write_bit(0);
  // one_wire_write_bit(1);

  // (void)one_wire_read_bit();

  volatile uint16_t temp;


  (void)temp;


  struct OneWireDevice devices[8];

  int i;
  for (i = 0; i < 8; i++) {
    int j;
    for (j = 0; j < 8; j++) {
      devices[i].id[j] = 0;
    }
  }

  volatile int32_t found = one_wire_discover(devices, 8);

  len = snprintf(tempStr, 256, "Found Devices: \r\n");
  HAL_UART_Transmit(&huart, tempStr, len, 100000);

  for (i = 0; i < found; i++) {
    len = snprintf(tempStr, 256, " %.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X\r\n",
                   devices[i].id[0],
                   devices[i].id[1],
                   devices[i].id[2],
                   devices[i].id[3],
                   devices[i].id[4],
                   devices[i].id[5],
                   devices[i].id[6],
                   devices[i].id[7]
                   );
    HAL_UART_Transmit(&huart, tempStr, len, 100000);
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint16_t temps[8];

    for (i = 0; i < found; i++) {
      temps[i] = one_wire_read_temp(&devices[i]);
    }

    // Home cursor
    len = snprintf(tempStr, 256, "\x1B[2J");
    HAL_UART_Transmit(&huart, tempStr, len, 100000);

    for (i = 0; i < found; i++) {
      int temp_f = temps[i] / 16;
      len = snprintf(tempStr, 256, " %.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X: %d\r\n",
                    devices[i].id[0],
                    devices[i].id[1],
                    devices[i].id[2],
                    devices[i].id[3],
                    devices[i].id[4],
                    devices[i].id[5],
                    devices[i].id[6],
                    devices[i].id[7],
                    temp_f
                    );
      HAL_UART_Transmit(&huart, tempStr, len, 100000);
    }

    // temp = one_wire_read_temp();

    // snprintf(tempStr, 256, "%.4X\r\n", temp);

    // HAL_UART_Transmit(&huart, tempStr, 6, 100000);
    

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
