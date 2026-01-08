/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - ADC Multi-Channel with UART
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Approximate 2S LiPo voltage-to-percentage curve
typedef struct {
    float voltage;
    int percent;
} BatteryPoint;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VREF        3.3f
#define ADC_MAX     4095.0f
#define R_FIXED     100000.0f   // 100K resistor for thermistor dividers
#define SAMPLES     100

// Battery divider values
#define R1_BAT      10000.0f
#define R2_BAT      2000.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t adc_values[4];  // Array to store ADC readings from 4 channels
float voltages[4];       // Array to store converted voltages
char uart_buffer[128];   // Buffer for UART transmission

// Battery percentage lookup table
BatteryPoint battery_table[] = {
    {6.0, 0}, {6.4, 10}, {6.8, 20}, {7.2, 40}, {7.6, 60}, {8.0, 80}, {8.4, 100}
};

// Thermistor Temperature Table
const float temp_table[] = {
    -40,-35,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50,
    55,60,65,70,75,80,85,90,95,100,105,110,115,120,125,130,135,140,145,150
};

const float resist_table[] = {
    332094,239900,175200,129287,96358,72500,55046,42157,32554,25339,19872,15698,
    12488,10000,8059,6535,5330,4372,3605,2990,2490,2084,1753,1481,1256,1070,915.4,
    786.0,677.3,585.7,508.3,442.6,386.6,338.7,297.7,262.4,231.9,205.5,182.6
};

const int table_len = sizeof(temp_table)/sizeof(temp_table[0]);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Read_ADC_Channels(void);
void Send_ADC_Data_UART(void);
float interpolate(float x, float x0, float y0, float x1, float y1);
float Get_Temperature(uint16_t adc_value);
float Get_BatteryPercent(uint16_t adc_value);
uint16_t ADC_Average(uint8_t channel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Read all 4 ADC channels sequentially with averaging
  * @retval None
  */
void Read_ADC_Channels(void)
{
    // Use averaging for each channel like the reference code
    adc_values[0] = ADC_Average(1);  // Channel 1 (PA1)
    adc_values[1] = ADC_Average(2);  // Channel 2 (PA2) 
    adc_values[2] = ADC_Average(3);  // Channel 3 (PA3)
    adc_values[3] = ADC_Average(4);  // Channel 4 (PA4)
    
    // Convert ADC values to voltages (assuming 3.3V reference)
    for(int i = 0; i < 4; i++)
    {
        voltages[i] = (adc_values[i] * VREF) / ADC_MAX;
    }
}

// Simple linear interpolation
float interpolate(float x, float x0, float y0, float x1, float y1) {
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

uint16_t ADC_Average(uint8_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES; // longer for accuracy

    uint32_t sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    return (uint16_t)(sum / SAMPLES);
}


// Get temperature (°C) from ADC
float Get_Temperature(uint16_t adc_value) {
    float vout = (adc_value / ADC_MAX) * VREF;
    if (vout <= 0.001f || vout >= (VREF - 0.001f)) return -273.15f;

    // If thermistor is on top (3.3V side): R_therm = R_FIXED * (VREF - vout) / vout
    float r_therm = (R_FIXED * (VREF - vout)) / vout;

    // Out of range
    if (r_therm > resist_table[0]) return temp_table[0];
    if (r_therm < resist_table[table_len - 1]) return temp_table[table_len - 1];

    // Find interval and interpolate
    for (int i = 0; i < table_len - 1; i++) {
        if (r_therm <= resist_table[i] && r_therm >= resist_table[i + 1])
            return interpolate(r_therm, resist_table[i], temp_table[i],
                                         resist_table[i + 1], temp_table[i + 1]);
    }
    return -273.15f;
}

// Battery percentage estimation
float Get_BatteryPercent(uint16_t adc_value) {
    float vout = (adc_value / ADC_MAX) * VREF;
    float vin = vout * ((R1_BAT + R2_BAT) / R2_BAT);

    if (vin <= battery_table[0].voltage) return 0;
    if (vin >= battery_table[6].voltage) return 100;

    for (int i = 0; i < 6; i++) {
        if (vin >= battery_table[i].voltage && vin <= battery_table[i + 1].voltage)
            return interpolate(vin, battery_table[i].voltage, battery_table[i].percent,
                                    battery_table[i + 1].voltage, battery_table[i + 1].percent);
    }
    return 0;
}

/**
  * @brief  Send ADC data via UART with temperature and battery calculations
  * @retval None
  */
void Send_ADC_Data_UART(void)
{
    int len;
    
    // Calculate temperatures and battery percentage
    float t0 = Get_Temperature(adc_values[0]);  // T1 from Channel 1
    float t1 = Get_Temperature(adc_values[1]);  // T2 from Channel 2  
    float t2 = Get_Temperature(adc_values[2]);  // T3 from Channel 3
    float batf = Get_BatteryPercent(adc_values[3]);  // Battery from Channel 4
    
    // Debug: Calculate and print resistance values
    float vout0 = (adc_values[0] / ADC_MAX) * VREF;
    float r0 = (R_FIXED * (VREF - vout0)) / vout0;
    float vout1 = (adc_values[1] / ADC_MAX) * VREF;
    float r1 = (R_FIXED * (VREF - vout1)) / vout1;
    float vout2 = (adc_values[2] / ADC_MAX) * VREF;
    float r2 = (R_FIXED * (VREF - vout2)) / vout2;
   
    // Convert to millivolts and ohms (integers for integer formatting)
    int mv0 = (int)(vout0 * 1000.0f);
    int mv1 = (int)(vout1 * 1000.0f);
    int mv2 = (int)(vout2 * 1000.0f);
    int ohms0 = (int)r0;
    int ohms1 = (int)r1;
    int ohms2 = (int)r2;
   
    // Print raw ADC values
    len = sprintf(uart_buffer, "Raw ADC: %lu %lu %lu %lu\r\n", 
                  adc_values[0], adc_values[1], adc_values[2], adc_values[3]);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
    
    // Print voltage values
    len = sprintf(uart_buffer, "V(mV): %d %d %d\r\n", mv0, mv1, mv2);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
    
    // Print resistance values
    len = sprintf(uart_buffer, "R(ohms): %d %d %d\r\n", ohms0, ohms1, ohms2);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
    
    // Debug battery voltage
    float vout_bat = (adc_values[3] / ADC_MAX) * VREF;
    float vin_bat = vout_bat * ((R1_BAT + R2_BAT) / R2_BAT);
    int bat_mv = (int)(vin_bat * 1000.0f);
    len = sprintf(uart_buffer, "Battery: ADC=%lu, Vout=%dmV, Vin=%dmV\r\n",
                 adc_values[3], (int)(vout_bat * 1000.0f), bat_mv);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
   
    // Convert to tenths to avoid %f formatting
    int t0_tenths = (int)roundf(t0 * 10.0f);   // e.g. 235 -> 23.5°C
    int t1_tenths = (int)roundf(t1 * 10.0f);
    int t2_tenths = (int)roundf(t2 * 10.0f);
    int bat_tenths = (int)roundf(batf * 10.0f);

    // Build formatted string using only integer formatting
    len = sprintf(uart_buffer,
        "T1=%d.%dC, T2=%d.%dC, T3=%d.%dC, Bat=%d.%d%%\r\n",
        t0_tenths / 10, abs(t0_tenths % 10),
        t1_tenths / 10, abs(t1_tenths % 10),
        t2_tenths / 10, abs(t2_tenths % 10),
        bat_tenths / 10, abs(bat_tenths % 10)
    );

    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
}

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // Send startup message
  char startup_msg[] = "STM32F401 ADC Multi-Channel Reader Started\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)startup_msg, strlen(startup_msg), HAL_MAX_DELAY);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // Read all ADC channels
    Read_ADC_Channels();
    
    // Send data via UART
    Send_ADC_Data_UART();
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    // Delay 500ms between readings
    HAL_Delay(500);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 4;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    // --- Configure channels individually ---

    // Channel 1 (PA1)
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES; // longer for accuracy
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Channel 2 (PA2)
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Channel 3 (PA3)
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = 3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Channel 4 (PA4) – battery
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = 4;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // PA4 as analog input
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

