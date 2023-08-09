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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdbool.h"
#include <stdio.h>
#include <limits.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Rejestry
#define LSM303_ACC_ADDRESS (0x19 << 1) // adres akcelerometru: 0011 001x
#define LSM303_ACC_CTRL_REG1_A 0x20 // rejestr ustawien 1
#define LSM303_ACC_CTRL_REG3_A 0x22 // rejestr ustawien 3
#define LSM303_ACC_Z_H_A 0x2D // wyzszy bajt danych osi Z
#define LSM303_ACC_Z_L_A 0x2C // nizszy bajt danych osi Z
#define LSM303_ACC_X_L_A 0x28 // nizszy bajt danych osi X

// mlodszy bajt danych osi Z z najstarszym bitem ustawionym na 1 w celu
// wymuszenia autoinkrementacji adresow rejestru w urzadzeniu docelowym
// (zeby moc odczytac wiecej danych na raz)
#define LSM303_ACC_Z_L_A_MULTI_READ (LSM303_ACC_Z_L_A | 0x80)

// mlodszy bajt danych osi X z najstarszym bitem ustawionym na 1 w celu
// wymuszenia autoinkrementacji adresow rejestru w urzadzeniu docelowym
// (zeby moc odczytac wiecej danych na raz)
#define LSM303_ACC_X_L_A_MULTI_READ (LSM303_ACC_X_L_A | 0x80)

// Maski bitowe
// CTRL_REG1_A = [ODR3][ODR2][ODR1][ODR0][LPEN][ZEN][YEN][XEN]
#define LSM303_ACC_Z_ENABLE 0x04 // 0000 0100
#define LSM303_ACC_XYZ_ENABLE 0x07 // 0000 0111
#define LSM303_ACC_100HZ 0x50 //0101 0000
#define LSM303_ACC_1HZ 0x10 //0001 0000

// CTRL_REG3_A = [CLICK][AOI1][AOI2][DRDY_1][DRDY_2][WTM][OVERRUN][---]
#define LSM303_ACC_I1_DRDY1 0x10 //0001 0000
#define LSM303_ACC_I1_DRDY2 0x08 //0000 1000

#define LSM303_ACC_RESOLUTION 2.0 // Maksymalna wartosc mierzalnego przyspieszenia [g]

// Zmienne
uint8_t Data[6]; // Zmienna do bezposredniego odczytu danych z akcelerometru
int16_t Xaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi X
int16_t Yaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Y
int16_t Zaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Z

int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;

uint8_t cubeX = 0, cubeY = 0;
uint8_t cubeSiz = 25;
uint8_t cubeJmp = 10;

uint32_t value = 0;

volatile static uint16_t joystick[2];
int16_t pl1_ay = 0, pl2_ay = 0;

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

bool isUserButtonPressed(void){
	return (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) != GPIO_PIN_RESET) ? true : false;
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
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_TIM9_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // wypelnieine zmiennej konfiguracyjnej odpowiednimi opcjami
    //uint8_t Settings = LSM303_ACC_XYZ_ENABLE | LSM303_ACC_100HZ;

  	// Wpisanie konfiguracji do rejestru akcelerometru
    //HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1, &Settings, 1, 100);

  	//Settings = LSM303_ACC_I1_DRDY2;
  	//HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG3_A, 1, &Settings, 1, 100);

  	//HAL_TIM_IC_Start(&htim9, TIM_CHANNEL_1);
  	//HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

  	//HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lcd_init();

  //int rotationThresholdBinaryShift = 10;
  //int zThresholdBinaryShift = 10;

   uint8_t playerScreenMargin = 5;
   uint8_t screenXMargin = 10;
   uint8_t playerSpeed = 1;
   uint8_t player_xsize = 4;
   uint8_t player_ysize = 20;
   int32_t pl1_x = screenXMargin + playerScreenMargin;
   int32_t pl2_x = LCD_WIDTH - (screenXMargin + playerScreenMargin + player_xsize);
   int32_t pl1_y = (LCD_HEIGHT / 2) - (player_ysize / 2);
   int32_t pl2_y = (LCD_HEIGHT / 2) - (player_ysize / 2);


   int8_t ballXSpeed = 3;
   int8_t ballYSpeed = 2;
   int32_t ballx = LCD_WIDTH / 2;
   int32_t bally = LCD_HEIGHT / 2;
   uint8_t ballsize = 4;

   HAL_ADC_Start_DMA(&hadc1, (uint32_t*)joystick, 2);

  while (1)
  {

	  /*
	  	  HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_X_L_A_MULTI_READ, 1, Data, 6, 100);

	  		// Konwersja odebranych bajtow danych na typ int16_t
	  		Xaxis = ((Data[1] << 8) | Data[0]);
	  		Yaxis = ((Data[3] << 8) | Data[2]);
	  		//Zaxis = ((Data[5] << 8) | Data[4]);

	  		ax = Xaxis >> rotationThresholdBinaryShift;
	  		ay = Yaxis >> rotationThresholdBinaryShift;
	  		//az = Zaxis >> zThresholdBinaryShift;
	  		 */

	  //lcd_fillBox(0,0, LCD_WIDTH, LCD_HEIGHT, BLUE);
	  //boxController(&cubeX, &cubeY, cubeSiz, ax, ay, 1);

	  //value = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_1);
	  //HAL_Delay(1000);

	  //PLAYER ACCELERATION

	  pl1_y += ((joystick[1] - 2048) / 200) * playerSpeed;
	  pl2_y += ((joystick[0] - 2048) / 200) * playerSpeed;

	  //PLAYER COLLISSION SCRIPT

	  int maxAllowedY = LCD_HEIGHT - playerScreenMargin;
	  int minAllowedY = playerScreenMargin;

	  if(pl1_y > maxAllowedY){
		  pl1_y = maxAllowedY;
	  }
	  if (pl1_y < minAllowedY) {
	      pl1_y = minAllowedY;
	   }

	  if(pl2_y > maxAllowedY){
	  	pl2_y = maxAllowedY;
	  }
	  if (pl2_y < minAllowedY) {
	  	pl2_y = minAllowedY;
	  }

	  //BALL ACCELERATION

	  ballx += ballXSpeed;
	  bally += ballYSpeed;

	  //BALL COLLISSION

	  if(bally < playerScreenMargin){
		  bally = playerScreenMargin;
		  ballYSpeed = -ballYSpeed;
	  }else if(bally > LCD_HEIGHT - ballsize) {
		  bally = LCD_HEIGHT - ballsize;
		  ballYSpeed = -ballYSpeed;
	  }

	  //PLAYER BALL COLLISSION
/*
	  if(bally + ballsize >= pl1_y && bally <= pl1_y + player_ysize && ballx <= pl1_x + player_xsize && ballx + ballsize >= pl1_x){
	  		  ballXSpeed = -ballXSpeed;
	  	  }

	  if(bally + ballsize >= pl2_y && bally <= pl2_y + player_ysize && ballx >= pl2_x + player_xsize && ballx + ballsize <= pl2_x){
	  	  		  ballXSpeed = -ballXSpeed;
	  	  }
	  	  */
	  if(bally + ballsize >= pl1_y && bally <= pl1_y + player_ysize && ballx <= pl1_x + player_xsize && ballx + ballsize >= pl1_x){
	  	  		  ballXSpeed = -ballXSpeed;
	  	  	  }

	  	  if(bally + ballsize >= pl2_y && bally <= pl2_y + player_ysize && ballx + ballsize >= pl2_x){
	  	  	  		  ballXSpeed = -ballXSpeed;
	  	  	  }


	  //RESET SCRIPT

	  if(ballx < playerScreenMargin){
		  NVIC_SystemReset();
	  }else if(ballx > LCD_WIDTH - ballsize) {
	  	  NVIC_SystemReset();
	  }



	  lcd_fillBox(0,0, LCD_WIDTH, LCD_HEIGHT, BLACK);

	  lcd_fillBox(pl1_x, pl1_y, player_xsize, player_ysize, WHITE);

	  lcd_fillBox(pl2_x, pl2_y, player_xsize, player_ysize, WHITE);

	  lcd_fillBox(ballx, bally, ballsize, ballsize, WHITE);

	    lcd_copy();

	  HAL_Delay(50);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
