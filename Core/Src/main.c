/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "gpdma.h"
#include "icache.h"
#include "memorymap.h"
#include "sai.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* Must include the CMSIS-DSP library header */
#define ARM_MATH_CM33
#include "arm_math.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// --- Core Algorithm Defines ---
#define SAMPLE_RATE         48000
#define FFT_SIZE            1024

// Structure for UART data packets for a linear array with averaging
#define UART_PACKET_HEADER 0xABCD
#define NUM_MICS            4
#define NUM_PAIRS           3
#define UART_TX_CYCLE_INTERVAL 5 // Transmit every 5 processing cycles

#define MIC_1 0
#define MIC_2 1
#define MIC_3 0
#define MIC_4 1

// DMA buffers receive interleaved data (L/R) for each block
#define SAI_RX_BUFFER_SIZE  (FFT_SIZE * 2)

// --- Physical Constants ---
#define SPEED_OF_SOUND      343.0f
// Distance between ADJACENT microphones in the linear array.
#define MIC_ADJACENT_DISTANCE 0.035 // e.g., 10 cm, 3.5 cm

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct UART_DataPacket{
    uint16_t header;
    uint16_t angle_q10; // Averaged angle in degrees (0-180), scaled by 2^10
    uint32_t timestamp; // Milliseconds since startup (from HAL_GetTick())
    int16_t lags[NUM_PAIRS];    // Lags for Mic1-2, Mic2-3, Mic3-4
    int16_t reserved;
    float32_t correlation_pair1[FFT_SIZE];
    float32_t correlation_pair2[FFT_SIZE];
    float32_t correlation_pair3[FFT_SIZE];
}UART_DataPacket;


// Double buffers for circular DMA for each SAI block
int32_t sai_a_dma_buffer[SAI_RX_BUFFER_SIZE * 2];
int32_t sai_b_dma_buffer[SAI_RX_BUFFER_SIZE * 2];

// Buffers for de-interleaved, single-channel floating point audio data
float32_t mic_buffers[NUM_MICS][FFT_SIZE];

// Buffers for FFT and correlation processing
float32_t fft_buffer1[FFT_SIZE * 2];
float32_t fft_buffer2[FFT_SIZE * 2];
float32_t correlation_output[FFT_SIZE * 2];

// CMSIS-DSP RFFT instance
arm_rfft_fast_instance_f32 fft_instance;

uint32_t time_diff_ms = 0; // Time difference in milliseconds for timestamping

// --- Synchronization and Data Pointers ---
volatile uint8_t data_ready_flags = 0;
volatile int32_t* processing_ptr_a = NULL;
volatile int32_t* processing_ptr_b = NULL;

// UART transmit buffer
UART_DataPacket uart_tx_packet;


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
void PeriphCommonClock_Config(void);
static void SystemPower_Config(void);
/* USER CODE BEGIN PFP */

void process_audio_data(void);
int16_t calculate_tdoa_lag(float32_t* mic1_data, float32_t* mic2_data, float32_t* correlation_result);
float32_t calculate_angle_from_lag(int16_t lag, float32_t distance);
void transmit_uart_data(float32_t angle, int16_t* lags, float32_t correlation_results[NUM_PAIRS][FFT_SIZE]);

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

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_SAI1_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  if (arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE) != ARM_MATH_SUCCESS) {
    HAL_GPIO_WritePin(led_blue_GPIO_Port, led_blue_Pin, GPIO_PIN_SET);  
  }

    // Start SAI audio capture for BOTH blocks

  if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*)sai_a_dma_buffer, SAI_RX_BUFFER_SIZE * 2) != HAL_OK) {
      HAL_GPIO_WritePin(led_green_GPIO_Port, led_green_Pin, GPIO_PIN_SET);  
  }


   if (HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)sai_b_dma_buffer, SAI_RX_BUFFER_SIZE * 2) != HAL_OK) {
       HAL_GPIO_WritePin(led_green_GPIO_Port, led_green_Pin, GPIO_PIN_SET);
   }

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Set Mics SAI1 A Left and Right
  // TODO: add m3 and m4
  HAL_GPIO_WritePin(m1_lr_sel_GPIO_Port, m1_lr_sel_Pin, MIC_1);
  HAL_GPIO_WritePin(m2_lr_sel_GPIO_Port, m2_lr_sel_Pin, MIC_2);
  HAL_GPIO_WritePin(m3_lr_sel_GPIO_Port, m3_lr_sel_Pin, MIC_3);
  HAL_GPIO_WritePin(m4_lr_sel_GPIO_Port, m4_lr_sel_Pin, MIC_4);
  
  while (1)
  {

    if (data_ready_flags) {

    	// Taking 18 ms to process
        process_audio_data();
        HAL_GPIO_TogglePin(led_green_GPIO_Port, led_green_Pin); // Indicate processing done
        
        // Reset flag after processing
        data_ready_flags = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the common periph clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInit.PLL2.PLL2Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLL2.PLL2M = 3;
  PeriphClkInit.PLL2.PLL2N = 15;
  PeriphClkInit.PLL2.PLL2P = 2;
  PeriphClkInit.PLL2.PLL2Q = 2;
  PeriphClkInit.PLL2.PLL2R = 2;
  PeriphClkInit.PLL2.PLL2RGE = RCC_PLLVCIRANGE_1;
  PeriphClkInit.PLL2.PLL2FRACN = 2048;
  PeriphClkInit.PLL2.PLL2ClockOut = RCC_PLL2_DIVP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Processes a block of audio data to calculate sound direction.
  * @retval None
  */
void process_audio_data(void)
{
    static uint8_t tx_cycle_counter = 0;
    float32_t all_correlation_outputs[NUM_PAIRS][FFT_SIZE];

//    SCB_InvalidateDCache_by_Addr((uint32_t*)processing_ptr_a, SAI_RX_BUFFER_SIZE * sizeof(int32_t));
//    SCB_InvalidateDCache_by_Addr((uint32_t*)processing_ptr_b, SAI_RX_BUFFER_SIZE * sizeof(int32_t));

    for (int i = 0; i < FFT_SIZE; i++) {
        mic_buffers[0][i] = (float32_t)(processing_ptr_a[i * 2] >> 8);
        mic_buffers[1][i] = (float32_t)(processing_ptr_a[i * 2 + 1] >> 8);
        mic_buffers[2][i] = (float32_t)(processing_ptr_b[i * 2] >> 8);
        mic_buffers[3][i] = (float32_t)(processing_ptr_b[i * 2 + 1] >> 8);
    }

    int16_t lags[NUM_PAIRS];
    lags[0] = calculate_tdoa_lag(mic_buffers[0], mic_buffers[1], all_correlation_outputs[0]);
    lags[1] = calculate_tdoa_lag(mic_buffers[1], mic_buffers[2], all_correlation_outputs[1]);
    lags[2] = calculate_tdoa_lag(mic_buffers[2], mic_buffers[3], all_correlation_outputs[2]);

    float32_t angles[NUM_PAIRS];
    float32_t angle_sum = 0.0f;
    uint8_t valid_pairs = 0;
    for (int i = 0; i < NUM_PAIRS; i++) {
        angles[i] = calculate_angle_from_lag(lags[i], MIC_ADJACENT_DISTANCE);
        if (angles[i] > 0.0f && angles[i] < 180.0f) {
             angle_sum += angles[i];
             valid_pairs++;
        }
    }
    
    float32_t average_angle = (valid_pairs > 0) ? (angle_sum / valid_pairs) : 0.0f;
    
    // Check if it's time to transmit
    tx_cycle_counter++;
    if (tx_cycle_counter >= UART_TX_CYCLE_INTERVAL) {
        transmit_uart_data(average_angle, lags, all_correlation_outputs);
        tx_cycle_counter = 0; // Reset counter
    }
}


/**
 * @brief Fills and transmits a data packet over UART using DMA.
 * @param angle The final averaged angle in degrees.
 * @param lags  Pointer to an array of the 3 individual lag values.
 * @param correlation_results 2D array containing the full correlation output for each pair.
 */
void transmit_uart_data(float32_t angle, int16_t* lags, float32_t correlation_results[NUM_PAIRS][FFT_SIZE]) {
    if (huart1.gState != HAL_UART_STATE_READY) {
        return; 
    }

    uart_tx_packet.header = UART_PACKET_HEADER;
    uart_tx_packet.angle_q10 = (uint16_t)(angle * 1024.0f);
    uart_tx_packet.timestamp = HAL_GetTick();
    memcpy(uart_tx_packet.lags, lags, sizeof(uart_tx_packet.lags));
    uart_tx_packet.reserved = 0;

    // Copy all three full correlation results into the packet
    memcpy(uart_tx_packet.correlation_pair1, correlation_results[0], FFT_SIZE * sizeof(float32_t));
    memcpy(uart_tx_packet.correlation_pair2, correlation_results[1], FFT_SIZE * sizeof(float32_t));
    memcpy(uart_tx_packet.correlation_pair3, correlation_results[2], FFT_SIZE * sizeof(float32_t));

    // Start the non-blocking DMA transfer
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&uart_tx_packet, sizeof(UART_DataPacket));
}

/**
  * @brief  Calculates the direction angle from a TDOA lag and pair distance.
  * @retval Angle in degrees (0-180).
  */
float32_t calculate_angle_from_lag(int16_t lag, float32_t distance)
{
    float32_t dt = (float32_t)lag / SAMPLE_RATE;
    float32_t cos_theta = (dt * SPEED_OF_SOUND) / distance;
    cos_theta = fmaxf(-1.0f, fminf(1.0f, cos_theta));
    float32_t angle_rad = acosf(cos_theta);
    return angle_rad * 180.0f / M_PI;
}

/**
  * @brief  Calculates the time delay (lag) using GCC-PHAT and stores the result.
  * @param  mic1_data, mic2_data: Pointers to input audio buffers.
  * @param  correlation_result: Pointer to a buffer to store the IFFT output.
  * @retval The lag in samples.
  */
int16_t calculate_tdoa_lag(float32_t* mic1_data, float32_t* mic2_data, float32_t* correlation_result)
{
    float32_t temp_corr_buffer[FFT_SIZE * 2];
    memcpy(fft_buffer1, mic1_data, FFT_SIZE * sizeof(float32_t));
    memcpy(fft_buffer2, mic2_data, FFT_SIZE * sizeof(float32_t));
    arm_rfft_fast_f32(&fft_instance, fft_buffer1, fft_buffer1, 0);
    arm_rfft_fast_f32(&fft_instance, fft_buffer2, fft_buffer2, 0);

    for (int i = 0; i < FFT_SIZE * 2; i += 2) {
        float32_t X_re = fft_buffer1[i], X_im = fft_buffer1[i+1];
        float32_t Y_re = fft_buffer2[i], Y_im = fft_buffer2[i+1];
        float32_t R_re = X_re * Y_re + X_im * Y_im;
        float32_t R_im = X_im * Y_re - X_re * Y_im;
        float32_t R_mag;
        arm_sqrt_f32(R_re * R_re + R_im * R_im, &R_mag);
        if (R_mag > 1e-9) {
            temp_corr_buffer[i]   = R_re / R_mag;
            temp_corr_buffer[i+1] = R_im / R_mag;
        } else {
            temp_corr_buffer[i]   = 0.0f;
            temp_corr_buffer[i+1] = 0.0f;
        }
    }
    
    arm_rfft_fast_f32(&fft_instance, temp_corr_buffer, correlation_result, 1);
    
    float32_t max_val;
    uint32_t max_idx;
    arm_max_f32(correlation_result, FFT_SIZE, &max_val, &max_idx);
    int16_t lag_val = max_idx;
    if (lag_val >= FFT_SIZE / 2) {
        lag_val -= FFT_SIZE;
    }
    return lag_val;
}

// --- DMA Transfer Complete Callbacks ---
volatile uint8_t dma_sync_flags = 0;
void trigger_processing(uint8_t half) {
    if (dma_sync_flags == 0x03) {
        if(half == 0) {
            processing_ptr_a = &sai_a_dma_buffer[0];
            processing_ptr_b = &sai_b_dma_buffer[0];
        } else {
            processing_ptr_a = &sai_a_dma_buffer[SAI_RX_BUFFER_SIZE];
            processing_ptr_b = &sai_b_dma_buffer[SAI_RX_BUFFER_SIZE];
        }
        data_ready_flags = 1;
        dma_sync_flags = 0;
    }
}
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI1_Block_A) {
      dma_sync_flags |= (1 << 0);
  } else if (hsai->Instance == SAI1_Block_B) {
      dma_sync_flags |= (1 << 1);
  }
  trigger_processing(0);
}
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI1_Block_A) {
      dma_sync_flags |= (1 << 0);
  } else if (hsai->Instance == SAI1_Block_B) {
      dma_sync_flags |= (1 << 1);
  }
  trigger_processing(1);
}

/**
  * @brief  UART Transmit Complete Callback.
  * @param  huart: UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  // This callback is called by the HAL driver when the UART DMA transfer is complete.
  // Its existence is all that's needed for the HAL driver to correctly reset
  // the huart->gState back to HAL_UART_STATE_READY, allowing the next transfer to start.
  if (huart->Instance == USART1)
  {
      // You could add code here if you needed to, e.g., to toggle an LED.
  }
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
