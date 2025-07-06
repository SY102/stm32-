/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24.h"
#include "NRF24_conf.h"
#include "stdio.h"
#include "string.h"
extern SPI_HandleTypeDef hspi1;
#include "NRF24_reg_addresses.h"
#define MAX_PAYLOAD 32
extern UART_HandleTypeDef huart2;
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
uint8_t rx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

void debug_dump_settings(void)
{
    uint8_t ch  = nrf24_r_reg(RF_CH,      1);    // RF 채널 번호
    uint8_t pw  = nrf24_r_reg(RX_PW_P0,   1);    // 파이프0 수신 페이로드 크기
    uint8_t addr[5];

    // 파이프0의 RX_ADDR_P0 레지스터(5바이트) 읽기
    csn_low();
    {
        uint8_t cmd = R_REGISTER | RX_ADDR_P0;
        HAL_SPI_Transmit(&hspi1, &cmd,     1, 100);
        HAL_SPI_Receive (&hspi1, addr,     5, 100);
    }
    csn_high();

    // UART로 출력
    printf("=== NRF24 DEBUG ===\r\n");
    printf(" RF_CH       = %u\r\n",      ch);
    printf(" RX_PW_P0    = %u bytes\r\n", pw);
    printf(" RX_ADDR_P0  = %02X %02X %02X %02X %02X\r\n",
           addr[0], addr[1], addr[2], addr[3], addr[4]);
    printf("==================\r\n");
}


void nrf24_receiver_setup(void)
{
	  // 1) 기본값 복원 & 이전 데이터 클리어
    nrf24_defaults();                               //레지스터 기본값으로 리셋
    HAL_Delay(5);                                   //전원, spi안정화 대기(최소 4.5ms이상 필요)
    nrf24_stop_listen();

    nrf24_pwr_up();
      HAL_Delay(5);
    //ce=low =>송신, 수신 모드 비활성화

    nrf24_set_channel(40);
    nrf24_set_payload_size(32); //무선 채널 40설정
    nrf24_auto_ack_all(disable);                    //ack 비활성화
               //패킷 최대크기 32바이트로 고정
    nrf24_rx_mode();

    nrf24_dpl(enable);              // FEATURE 레지스터: DPL 켜기
    nrf24_set_rx_dpl(0, enable);    // 파이프0에 대해서만 DPL 사용

    //PRIM_RX비트 설정=> 수신 모드 준비
    nrf24_open_rx_pipe(0, rx_address);              //파이프 0에 수신주소 설정


    nrf24_listen();                                 //CE=high => 실제 수신 대기모드 진입



}



int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}


void receive_data(void)
{
	   uint8_t  payload_width;
	    char     buf[MAX_PAYLOAD + 1] = {0};

	    if (nrf24_data_available()) {
	        // 1) 실제 수신된 페이로드 길이 읽어오기
	        payload_width = nrf24_r_pld_wid();
	        if (payload_width > MAX_PAYLOAD) payload_width = MAX_PAYLOAD;

	        // 2) 그만큼만 읽어서 버퍼에 저장
	        nrf24_receive((uint8_t*)buf, payload_width);
	        buf[payload_width] = '\0';  // 널 종료

	        // 3) UART로 깔끔하게 출력
	        printf("Received: %s\r\n", buf);

	        nrf24_clear_rx_dr();        // Status.RX_DR 클리어

	        // 수신 성공 LED 표시
	        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	    }
    else                                              //수신된 데이터 없으면 6번 led on
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    }

    HAL_Delay(100);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(CE_Pin_GPIO_Port,  CE_Pin_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CSN_Pin_GPIO_Port, CSN_Pin_Pin, GPIO_PIN_SET);

  nrf24_init();

  nrf24_receiver_setup();

  debug_dump_settings();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  receive_data();


	  //debug_dump_settings();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
