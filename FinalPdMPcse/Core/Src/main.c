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
#include "DELAY.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{
  INIT_ADCS,
  ADQUIRE_SAMPLES_FROM_ONE_ADC,
  SWITCH_TO_NEXT_ADC,
  WAIT_FOR_RESTART,
} adc_fsm_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCP3913_ADC_QTY 3
#define ADC_FSM_RESTART_DELAY 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static MCP3913_handle_t adc[MCP3913_ADC_QTY]; // Array de estructuras de configuracion de ADC MCP3913
static int32_t adc_values[MCP3913_ADC_QTY][MCP3913_ADC_CHANNELS_QTY]; // Matriz donde se guardaron los valores leidos.
static adc_fsm_state_t adc_fsm_state; // Variable que indica el estado de la MEF de adquisicion de ADCs
static uint8_t adc_being_adquired; // Variable que indica que ADC esta siendo adquirido
static delay_t adc_delay; // Variable para manejar el delay de la MEF de ADC

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
  ADC_FSM_Init(); // Inicializo maquina de estados de ADC

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ADC_FSM_Update(); // Actualizo la maquina de estados de adquisicion
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI2);
  /* USER CODE BEGIN SPI2_Init 2 */

  // Quater threshold for 1 byte / 8 bits
  LL_SPI_SetRxFIFOThreshold(SPI2, LL_SPI_RX_FIFO_TH_QUARTER);

  // Enable SPI
  LL_SPI_Enable(SPI2);

  // Wait until SPI is ready
  while(LL_SPI_IsActiveFlag_BSY(SPI2));

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  DELAY_Init(&adc_delay, ADC_FSM_RESTART_DELAY); // Inicializo el delay
}

/*
 * @brief Actualiza la maquina de estados del manejo de los adc
 */
void ADC_FSM_Update() {
  switch (adc_fsm_state) {
    case INIT_ADCS:
      // Seteo los chip select en uno (por defecto el MX_GPIO_Init generado por el STMCUBE los setea en cero)
      LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
      LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
      LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
      HAL_Delay(1); // Fuerzo un delay de 1ms para reiniciar la interfaz SPI de los ADCs. No implemento MEF para el retardo porque se ejecuta una sola vez al inicio y nunca más.

      MCP3913_Load_Default_Config(&adc[0]); // Cargo valores por defecto en la estrucutra de configuracion del ADC 0
      adc[0].spi_handle = (void *)SPI2;   // SPI con el que va a trabajar
      adc[0].spi_cs_port = (void *)GPIOA;   // Puerto del CS
      adc[0].spi_cs_pin = LL_GPIO_PIN_2;       // Pin del CS
      adc[0].dev_address = MCP3913_DEFAULT_DEV_ADDRESS; // Address por defecto del ADC (especificada por el fabricante)
      MCP3913_Init(&adc[0]); // Inicializo ADC 0

      MCP3913_Load_Default_Config(&adc[1]); // Cargo valores por defecto en la estrucutra de configuracion del ADC 1
      adc[1].spi_handle = (void *)SPI2;   // SPI con el que va a trabajar
      adc[1].spi_cs_port = (void *)GPIOA;   // Puerto del CS
      adc[1].spi_cs_pin = LL_GPIO_PIN_3;       // Pin del CS
      adc[1].dev_address = MCP3913_DEFAULT_DEV_ADDRESS; // Address por defecto del ADC (especificada por el fabricante)
      MCP3913_Init(&adc[1]); // Inicializo ADC 1

      MCP3913_Load_Default_Config(&adc[2]); // Cargo valores por defecto en la estrucutra de configuracion del ADC 2
      adc[2].spi_handle = (void *)SPI2;   // SPI con el que va a trabajar
      adc[2].spi_cs_port = (void *)GPIOA;   // Puerto del CS
      adc[2].spi_cs_pin = LL_GPIO_PIN_4;       // Pin del CS
      adc[2].dev_address = MCP3913_DEFAULT_DEV_ADDRESS; // Address por defecto del ADC (especificada por el fabricante)
      MCP3913_Init(&adc[2]); // Inicializo ADC 2

      adc_fsm_state = ADQUIRE_SAMPLES_FROM_ONE_ADC;
      break;
    case ADQUIRE_SAMPLES_FROM_ONE_ADC:
      // Leo todos los canales de un solo ADC
      MCP3913_Read_All_Channels(&adc[adc_being_adquired], adc_values[adc_being_adquired]); // Leo todos los canales de un ADC

      adc_fsm_state = SWITCH_TO_NEXT_ADC;
      break;
    case SWITCH_TO_NEXT_ADC:
      // Cambio de ADC
      adc_being_adquired = (adc_being_adquired + 1) % MCP3913_ADC_QTY; // Cambio al siguiente ADC

      if(adc_being_adquired == 0) {
        adc_fsm_state = WAIT_FOR_RESTART;
      }
      else {
        adc_fsm_state = ADQUIRE_SAMPLES_FROM_ONE_ADC;
      }
      break;
    case WAIT_FOR_RESTART:
      // Espero (no bloqueante) antes de reiniciar la consulta de los ADC.
      if(DELAY_Read(&adc_delay)) {
        adc_fsm_state = ADQUIRE_SAMPLES_FROM_ONE_ADC;
      }
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
