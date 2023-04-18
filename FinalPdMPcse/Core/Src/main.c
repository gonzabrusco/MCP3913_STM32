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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MCP3913.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{
  INIT_ADCS,
  ADQUIRE_SAMPLES_FROM_ONE_ADC,
  SWITCH_TO_NEXT_ADC,
} adc_fsm_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCP3913_ADC_QTY 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
static MCP3913_handle_t adc[MCP3913_ADC_QTY]; // Array de estructuras de configuracion de ADC MCP3913
static int32_t adc_values[MCP3913_ADC_QTY][MCP3913_ADC_CHANNELS_QTY]; // Matriz donde se guardaron los valores leidos.
static adc_fsm_state_t adc_fsm_state; // Variable que indica el estado de la MEF de adquisicion de ADCs
static uint8_t adc_being_adquired; // Variable que indica que ADC esta siendo adquirido

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/*
 * @brief Inicializa la maquina de estados del manejo de los adc
 */
void ADC_FSM_Init();
/*
 * @brief Actualiza la maquina de estados del manejo de los adc
 */
void ADC_FSM_Update();

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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  ADC_FSM_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ADC_FSM_Update();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * @brief Inicializa la maquina de estados del manejo de los adc
 */
void ADC_FSM_Init() {
  adc_fsm_state = INIT_ADCS; // La MEF inicializa para configurar los ADC
  adc_being_adquired = 0; // Comienza con el ADC0
}

/*
 * @brief Actualiza la maquina de estados del manejo de los adc
 */
void ADC_FSM_Update() {
  switch (adc_fsm_state) {
    case INIT_ADCS:
      // Seteo los chip select en uno (por defecto el MX_GPIO_Init generado por el STMCUBE los setea en cero)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(1); // Fuerzo un delay de 1ms para reiniciar la interfaz SPI de los ADCs. No implemento MEF para el retardo porque se ejecuta una sola vez al inicio y nunca más.

      MCP3913_Load_Default_Config(&adc[0]); // Cargo valores por defecto en la estrucutra de configuracion del ADC 0
      adc[0].spi_handle = (void *)&hspi2;   // SPI con el que va a trabajar
      adc[0].spi_cs_port = (void *)GPIOA;   // Puerto del CS
      adc[0].spi_cs_pin = GPIO_PIN_2;       // Pin del CS
      adc[0].dev_address = MCP3913_DEFAULT_DEV_ADDRESS; // Address por defecto del ADC (especificada por el fabricante)
      MCP3913_Init(&adc[0]); // Inicializo ADC 0

      MCP3913_Load_Default_Config(&adc[1]); // Cargo valores por defecto en la estrucutra de configuracion del ADC 1
      adc[1].spi_handle = (void *)&hspi2;   // SPI con el que va a trabajar
      adc[1].spi_cs_port = (void *)GPIOA;   // Puerto del CS
      adc[1].spi_cs_pin = GPIO_PIN_3;       // Pin del CS
      adc[1].dev_address = MCP3913_DEFAULT_DEV_ADDRESS; // Address por defecto del ADC (especificada por el fabricante)
      MCP3913_Init(&adc[1]); // Inicializo ADC 1

      MCP3913_Load_Default_Config(&adc[2]); // Cargo valores por defecto en la estrucutra de configuracion del ADC 2
      adc[2].spi_handle = (void *)&hspi2;   // SPI con el que va a trabajar
      adc[2].spi_cs_port = (void *)GPIOA;   // Puerto del CS
      adc[2].spi_cs_pin = GPIO_PIN_4;       // Pin del CS
      adc[2].dev_address = MCP3913_DEFAULT_DEV_ADDRESS; // Address por defecto del ADC (especificada por el fabricante)
      MCP3913_Init(&adc[2]); // Inicializo ADC 2

      adc_fsm_state = ADQUIRE_SAMPLES_FROM_ONE_ADC;
      break;
    case ADQUIRE_SAMPLES_FROM_ONE_ADC:
      MCP3913_Read_All_Channels(&adc[adc_being_adquired], adc_values[adc_being_adquired]);

      adc_fsm_state = SWITCH_TO_NEXT_ADC;
      break;
    case SWITCH_TO_NEXT_ADC:
      adc_being_adquired = (adc_being_adquired + 1) % MCP3913_ADC_QTY;

      adc_fsm_state = ADQUIRE_SAMPLES_FROM_ONE_ADC;
      break;
    default:
      // Si la MEF cae en esta condicion no esperada, reinicializo la MEF.
      adc_fsm_state = INIT_ADCS;
      break;
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
