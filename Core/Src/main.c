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

#define VCP_PACKET_HEADER 0xABCD
#define MIC_DATA_DOWNSAMPLE_FACTOR 8 // Send 1 out of every 8 samples
#define VCP_MIC_SAMPLES (FFT_SIZE / MIC_DATA_DOWNSAMPLE_FACTOR)

// --- Core Algorithm Defines ---
#define SAMPLE_RATE         48000
#define FFT_SIZE            1024
#define NUM_MICS            4
#define NUM_PAIRS           3

typedef struct {
    uint16_t header;
    uint16_t angle_q10; // Averaged angle in degrees (0-180), scaled by 2^10
    int16_t lags[3];    // Lags for Mic1-2, Mic2-3, Mic3-4
    int16_t reserved;   // Padding for alignment
    int16_t mic_data[NUM_MICS][VCP_MIC_SAMPLES];
} VCP_DataPacket;


// Structure for UART data packets
#define UART_PACKET_HEADER 0xABCD
#define NUM_MICS_DEBUG 2
#define UART_MIC_SAMPLES (FFT_SIZE / MIC_DATA_DOWNSAMPLE_FACTOR)

typedef struct {
    uint16_t header;
    uint16_t angle_q10; // Angle in degrees (0-180), scaled by 2^10
    uint32_t timestamp;
    int16_t lag;        // The single lag value between the two mics
    int16_t reserved;   // Padding for alignment
    int16_t mic_data[NUM_MICS_DEBUG][UART_MIC_SAMPLES];
} UART_DataPacket;


#define BUFFER_SIZE 1024
#define AUDIO_BUFFER_SIZE 4

// DMA buffers receive interleaved data (L/R) for each block
#define SAI_RX_BUFFER_SIZE  (FFT_SIZE * 2) // *2 for stereo (L/R)

// --- Physical Constants ---
#define SPEED_OF_SOUND      343.0f
// Distance between ADJACENT microphones in the linear array.
#define MIC_DISTANCE 0.1f // e.g., 10 cm

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Double buffers for circular DMA for each SAI block
int32_t sai_a_dma_buffer[SAI_RX_BUFFER_SIZE * 2];
int32_t sai_b_dma_buffer[SAI_RX_BUFFER_SIZE * 2];

// Buffers for de-interleaved, single-channel floating point audio data
float32_t mic_buffers[NUM_MICS_DEBUG][FFT_SIZE];

// Buffers for FFT and correlation processing
float32_t fft_buffer1[FFT_SIZE * 2];
float32_t fft_buffer2[FFT_SIZE * 2];
float32_t correlation_output[FFT_SIZE * 2];


// CMSIS-DSP RFFT instance
arm_rfft_fast_instance_f32 fft_instance;

// --- Synchronization and Data Pointers ---
// Use bits to flag completion: bit 0 for HalfCplt, bit 1 for Cplt
// bits 4,5 for BlockA/B. e.g., 0x11 means HalfCplt for BlockA is ready.
volatile uint8_t data_ready_flag= 0;
volatile int32_t* processing_ptr_a = NULL;
volatile int32_t* processing_ptr_b = NULL;

// VCP transmit buffer
VCP_DataPacket vcp_packet;
// UART transmit buffer
UART_DataPacket uart_packet;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */






// void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){

//   timestamp = HAL_GetTick();
//   l_sample = (int) (audio_buffer[0] << 16) | (audio_buffer[1]);
//   r_sample = (int) (audio_buffer[2] << 16) | (audio_buffer[3]);
//   sprintf(serial_Buffer, "left=%d,right=%d,%lu\n", l_sample, r_sample,timestamp);
//   HAL_UART_Transmit(&huart1, serial_Buffer, strlen(serial_Buffer), 250);
// }




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
/* USER CODE BEGIN PFP */
void process_audio_data(void);
int16_t calculate_tdoa_lag(float32_t* mic1_data, float32_t* mic2_data);
float32_t calculate_angle_from_lag(int16_t lag, float32_t distance);
void transmit_vcp_data(float32_t angle, int16_t* lags);

float32_t calculate_direction(int16_t lag);
void transmit_uart_data(float32_t angle, int16_t lag);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t data[8];
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


//  if (HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)sai_b_dma_buffer, SAI_RX_BUFFER_SIZE * 2) != HAL_OK) {
//      HAL_GPIO_WritePin(led_green_GPIO_Port, led_green_Pin, GPIO_PIN_SET);
//  }

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
  HAL_GPIO_WritePin(m1_lr_sel_GPIO_Port, m1_lr_sel_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m2_lr_sel_GPIO_Port, m2_lr_sel_Pin, GPIO_PIN_SET);


  
  while (1)
  {

    if (data_ready_flag) {
        process_audio_data();
        // Reset flag after processing
        data_ready_flag = 0;
    }
	   HAL_Delay(500);
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
//    // --- Step 0: Cache Maintenance ---
//    SCB_InvalidateDCache_by_Addr((uint32_t*)processing_ptr_a, SAI_RX_BUFFER_SIZE * sizeof(int32_t));
//    SCB_InvalidateDCache_by_Addr((uint32_t*)processing_ptr_b, SAI_RX_BUFFER_SIZE * sizeof(int32_t));


    // // --- Step 1: De-interleave the raw DMA buffers into separate mic buffers ---
    // for (int i = 0; i < FFT_SIZE; i++) {
    //     mic_buffers[0][i] = (float32_t)(processing_ptr_a[i * 2] >> 8);     // Mic 1
    //     mic_buffers[1][i] = (float32_t)(processing_ptr_a[i * 2 + 1] >> 8); // Mic 2
    //     mic_buffers[2][i] = (float32_t)(processing_ptr_b[i * 2] >> 8);     // Mic 3
    //     mic_buffers[3][i] = (float32_t)(processing_ptr_b[i * 2 + 1] >> 8); // Mic 4
    // }

    // // --- Step 2: Calculate TDOA lag for each adjacent microphone pair ---
    // int16_t lags[NUM_PAIRS];
    // lags[0] = calculate_tdoa_lag(mic_buffers[0], mic_buffers[1]); // Pair 1: Mic1-Mic2
    // lags[1] = calculate_tdoa_lag(mic_buffers[1], mic_buffers[2]); // Pair 2: Mic2-Mic3
    // lags[2] = calculate_tdoa_lag(mic_buffers[2], mic_buffers[3]); // Pair 3: Mic3-Mic4

    // // --- Step 3: Convert each lag into an angle and average the results ---
    // float32_t angles[NUM_PAIRS];
    // float32_t angle_sum = 0.0f;

    // for (int i = 0; i < NUM_PAIRS; i++) {
    //     angles[i] = calculate_angle_from_lag(lags[i], MIC_ADJACENT_DISTANCE);
    //     angle_sum += angles[i];
    // }
    // float32_t average_angle = angle_sum / NUM_PAIRS;


        // --- Step 1: De-interleave the raw DMA buffer into separate mic buffers ---
    for (int i = 0; i < FFT_SIZE; i++) {
        // Data is 24-bit in a 32-bit word, left-aligned. Right-shift to normalize.
        mic_buffers[0][i] = (float32_t)(processing_ptr_a[i * 2] >> 8);     // Mic 1
        mic_buffers[1][i] = (float32_t)(processing_ptr_a[i * 2 + 1] >> 8); // Mic 2
    }

    // --- Step 2: Calculate TDOA lag for the microphone pair ---
    int16_t lag = calculate_tdoa_lag(mic_buffers[0], mic_buffers[1]);

    // --- Step 3: Convert lag into a direction angle (0-180 degrees) ---
    float32_t angle_deg = calculate_direction(lag);


    // --- Step 4: Transmit results and diagnostic data over UART ---
    transmit_uart_data(angle_deg, lag);



    // --- Step 4: Transmit results and diagnostic data over VCP ---
//    transmit_vcp_data(angle_deg, lag);
}

/**
  * @brief  Calculates the direction angle for a linear array.
  * @param  lag: The lag in samples between the two microphones.
  * @retval Angle in degrees (0-180).
  */
float32_t calculate_direction(int16_t lag)
{
    float32_t dt = (float32_t)lag / SAMPLE_RATE;

    // Calculate the argument for acos, clamp to [-1, 1] to avoid domain errors
    float32_t cos_theta = (dt * SPEED_OF_SOUND) / MIC_DISTANCE;
    cos_theta = fmaxf(-1.0f, fminf(1.0f, cos_theta));
    
    // Calculate final angle in degrees
    float32_t angle_rad = acosf(cos_theta);
    float32_t angle_deg = angle_rad * 180.0f / M_PI;

    return angle_deg;
}


/**
 * @brief Fills and transmits a data packet over USB VCP.
 * @param angle The final averaged angle in degrees.
 * @param lags  Pointer to an array of the 3 individual lag values.
 */
void transmit_vcp_data(float32_t angle, int16_t* lags) {
    vcp_packet.header = VCP_PACKET_HEADER;
    vcp_packet.angle_q10 = (uint16_t)(angle * 1024.0f);
    
    // Copy the individual lags into the packet
    memcpy(vcp_packet.lags, lags, sizeof(vcp_packet.lags));
    vcp_packet.reserved = 0;

    // Downsample raw mic data for transmission
    for(int i = 0; i < NUM_MICS; i++) {
        for(int j = 0; j < VCP_MIC_SAMPLES; j++) {
            // Convert float back to int16 for smaller packet size
            vcp_packet.mic_data[i][j] = (int16_t)mic_buffers[i][j * MIC_DATA_DOWNSAMPLE_FACTOR];
        }
    }

    // Send the packet over USB
    //CDC_Transmit_FS((uint8_t*)&vcp_packet, sizeof(VCP_DataPacket));
    HAL_UART_Transmit(&huart1, (uint8_t*)&vcp_packet, sizeof(VCP_DataPacket), HAL_MAX_DELAY);
    // Optionally, toggle an LED to indicate data transmission
    HAL_GPIO_TogglePin(led_green_GPIO_Port, led_green_Pin);
}

/**
 * @brief Fills and transmits a data packet over UART.
 * @param angle The calculated angle in degrees.
 * @param lag The TDOA lag between the two mics.
 */
void transmit_uart_data(float32_t angle, int16_t lag) {
    uart_packet.header = UART_PACKET_HEADER;
    uart_packet.angle_q10 = (uint16_t)(angle * 1024.0f);
    uart_packet.timestamp = HAL_GetTick();
    uart_packet.lag = lag;
    uart_packet.reserved = 0;

    // Downsample raw mic data for transmission
    for(int i = 0; i < NUM_MICS_DEBUG; i++) {
        for(int j = 0; j < UART_MIC_SAMPLES; j++) {
            uart_packet.mic_data[i][j] = (int16_t)mic_buffers[i][j * MIC_DATA_DOWNSAMPLE_FACTOR];
        }
    }

    // Send the packet over UART
    // Note: Using HAL_UART_Transmit in blocking mode for simplicity.
    // For higher performance, HAL_UART_Transmit_DMA could be used.
    HAL_UART_Transmit(&huart1, (uint8_t*)&uart_packet, sizeof(UART_DataPacket), 100);
    // Optionally, toggle an LED to indicate data transmission
    HAL_GPIO_TogglePin(led_blue_GPIO_Port, led_blue_Pin);
}

/**
  * @brief  Calculates the direction angle from a TDOA lag and pair distance.
  * @param  lag: The lag in samples between two microphones.
  * @param  distance: The distance in meters between the same two microphones.
  * @retval Angle in degrees (0-180).
  */
float32_t calculate_angle_from_lag(int16_t lag, float32_t distance)
{
    float32_t dt = (float32_t)lag / SAMPLE_RATE;

    // Calculate the argument for acos, clamp to [-1, 1] to avoid domain errors
    float32_t cos_theta = (dt * SPEED_OF_SOUND) / distance;
    cos_theta = fmaxf(-1.0f, fminf(1.0f, cos_theta));
    
    // Calculate final angle in degrees
    float32_t angle_rad = acosf(cos_theta);
    float32_t angle_deg = angle_rad * 180.0f / M_PI;

    return angle_deg;
}

/**
  * @brief  Calculates the time delay (lag) between two signals using GCC-PHAT.
  * @param  mic1_data: Pointer to the first microphone's audio buffer.
  * @param  mic2_data: Pointer to the second microphone's audio buffer.
  * @retval The lag in samples.
  */
int16_t calculate_tdoa_lag(float32_t* mic1_data, float32_t* mic2_data)
{
    // Copy data to FFT buffers
    memcpy(fft_buffer1, mic1_data, FFT_SIZE * sizeof(float32_t));
    memcpy(fft_buffer2, mic2_data, FFT_SIZE * sizeof(float32_t));

    // Perform forward FFT on both signals
    arm_rfft_fast_f32(&fft_instance, fft_buffer1, fft_buffer1, 0);
    arm_rfft_fast_f32(&fft_instance, fft_buffer2, fft_buffer2, 0);

    // GCC-PHAT Calculation
    for (int i = 0; i < FFT_SIZE * 2; i += 2) {
        float32_t X_re = fft_buffer1[i], X_im = fft_buffer1[i+1];
        float32_t Y_re = fft_buffer2[i], Y_im = fft_buffer2[i+1];
        float32_t R_re = X_re * Y_re + X_im * Y_im;
        float32_t R_im = X_im * Y_re - X_re * Y_im;
        float32_t R_mag;
        arm_sqrt_f32(R_re * R_re + R_im * R_im, &R_mag);
        if (R_mag > 1e-9) {
            correlation_output[i]   = R_re / R_mag;
            correlation_output[i+1] = R_im / R_mag;
        } else {
            correlation_output[i]   = 0.0f;
            correlation_output[i+1] = 0.0f;
        }
    }

    // Inverse FFT
    arm_rfft_fast_f32(&fft_instance, correlation_output, correlation_output, 1);

    // Find the peak of the correlation
    float32_t max_val;
    uint32_t max_idx;
    arm_max_f32(correlation_output, FFT_SIZE, &max_val, &max_idx);
    
    int16_t lag_val = max_idx;
    if (lag_val >= FFT_SIZE / 2) {
        lag_val -= FFT_SIZE;
    }
    return lag_val;
}

// --- DMA Transfer Complete Callbacks ---

// Synchronization state for the two DMA channels
volatile uint8_t dma_sync_flags = 0;

void trigger_processing(uint8_t half) {
    // Both DMAs for a given half (0 or 1) must be complete
    if (dma_sync_flags == 0x03) {
        if(half == 0) { // First half
            processing_ptr_a = &sai_a_dma_buffer[0];
            processing_ptr_b = &sai_b_dma_buffer[0];
        } else { // Second half
            processing_ptr_a = &sai_a_dma_buffer[SAI_RX_BUFFER_SIZE];
            processing_ptr_b = &sai_b_dma_buffer[SAI_RX_BUFFER_SIZE];
        }
        data_ready_flag = 1; // Signal main loop to process
        dma_sync_flags = 0; // Reset for next round
    }
}

// --- Simplified DMA Transfer Complete Callbacks ---

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  // Point to the first half of the DMA buffer
  processing_ptr_a = &sai_a_dma_buffer[0];
  // Set flag to process this data in the main loop
  data_ready_flag = 1;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  // Point to the second half of the DMA buffer
  processing_ptr_a = &sai_a_dma_buffer[SAI_RX_BUFFER_SIZE];
  // Set flag to process this data in the main loop
  data_ready_flag = 1;
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
