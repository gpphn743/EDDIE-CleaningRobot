


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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include <stdio.h> // Required for sprintf
#include <string.h>
#include "pms7003.h"
#include "L298N.h"
#include "HC-SR04.h"
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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rxByte;
uint8_t rxBufferPMS[32];
uint8_t rxIndex = 0;
PMS_Data_t sensorData;

/* control from web: 1 = allow autonomous driving; 0 = user forced stop */
volatile uint8_t wheelEnabled = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* -- Helper: build & send Web page via ESP-01 (AT) -- */
/* Sends header + body as a single payload length indicated in AT+CIPSEND */
static void send_web_page_via_esp(int channel, int wheelState, int brushState, int relayState)
{
    char body_buf[1400];
    const char *body_template =
      "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
      "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
      "<style>body{background:#000;color:#fff;font-family:Arial;padding:12px}"
      ".button{display:inline-block;padding:10px 14px;margin:6px;border-radius:6px;text-decoration:none;background:#444;color:#fff}"
      ".on{background:#2e8b57}.off{background:#8b0000}</style>"
      "</head><body>"
      "<h3>STM32 - Cleaning Robot</h3>"
      "<p>Controls:</p>"
      "<a class=\"button %s\" href=\"/wheelon\">WHEEL ON</a>"
      "<a class=\"button %s\" href=\"/wheeloff\">WHEEL OFF</a><br/>"
      "<a class=\"button %s\" href=\"/brushon\">BRUSH ON</a>"
      "<a class=\"button %s\" href=\"/brushoff\">BRUSH OFF</a><br/>"
      "<a class=\"button %s\" href=\"/relayon\">RELAY ON</a>"
      "<a class=\"button %s\" href=\"/relayoff\">RELAY OFF</a>"
      "<p>Status: Wheel: <b>%s</b> &nbsp; Brush: <b>%s</b> &nbsp; Relay: <b>%s</b></p>"
      "</body></html>";

    const char *wheel_on_cls = wheelState==0 ? "on" : "off";
    const char *wheel_off_cls = wheelState==1 ? "on" : "off";
    const char *brush_on_cls = brushState==0 ? "on" : "off";
    const char *brush_off_cls = brushState==1 ? "on" : "off";
    const char *relay_on_cls = relayState==1 ? "on" : "off";
    const char *relay_off_cls = relayState==0 ? "on" : "off";
    const char *wheel_status = wheelState==0 ? "ON" : "OFF";
    const char *brush_status = brushState==0 ? "ON" : "OFF";
    const char *relay_status = relayState==1 ? "ON" : "OFF";

    int body_len = snprintf(body_buf, sizeof(body_buf), body_template,
      wheel_on_cls, wheel_off_cls, brush_on_cls, brush_off_cls,
      relay_on_cls, relay_off_cls,
      wheel_status, brush_status, relay_status);

    if (body_len < 0) return;
    if (body_len >= (int)sizeof(body_buf)) body_len = sizeof(body_buf) - 1;

    char header[200];
    int header_len = snprintf(header, sizeof(header),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n",
        body_len);

    if (header_len < 0) return;

    int total_len = header_len + body_len;

    char atcmd[64];
    int atcmd_len = snprintf(atcmd, sizeof(atcmd), "AT+CIPSEND=%d,%d\r\n", channel, total_len);
    if (atcmd_len < 0) return;

    uint8_t rxTmp[512];
    memset(rxTmp,0,sizeof(rxTmp));
    // send CIPSEND
    HAL_UART_Transmit(&huart1, (uint8_t*)atcmd, atcmd_len, 1000);
    // wait for '>' prompt (small timeout)
    HAL_UART_Receive(&huart1, rxTmp, sizeof(rxTmp), 200);
    if (strchr((char*)rxTmp, '>') == NULL) {

      HAL_Delay(30);
    }

    // send header then body
    HAL_UART_Transmit(&huart1, (uint8_t*)header, header_len, 2000);
    HAL_UART_Transmit(&huart1, (uint8_t*)body_buf, body_len, 2000);

    // read ESP response (SEND OK)
    memset(rxTmp,0,sizeof(rxTmp));
    HAL_UART_Receive(&huart1, rxTmp, sizeof(rxTmp), 300);
}

static void safeDelay(uint32_t ms)
{
    const uint32_t slice = 50; // 50 ms per check
    uint32_t remaining = ms;
    while (remaining > 0) {
        if (wheelEnabled) break; // user forced stop -> exit early
        uint32_t w = (remaining > slice) ? slice : remaining;
        HAL_Delay(w);
        remaining -= w;
    }
}

void wifiConfiguration(uint8_t rxBuffer[], uint8_t ATisOK, int channel, char ATcommand[]){

  // Reset module
  sprintf(ATcommand,"AT+RST\r\n");
  memset(rxBuffer,0,sizeof(rxBuffer));
  HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
  HAL_UART_Receive (&huart1, rxBuffer, 512, 200);
  HAL_Delay(500);

  // CWMODE -> AP
  ATisOK = 0;
  while(!ATisOK){
	sprintf(ATcommand,"AT+CWMODE_CUR=2\r\n");
	  memset(rxBuffer,0,sizeof(rxBuffer));
	  HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	  HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
	if(strstr((char *)rxBuffer,"OK")){
	  ATisOK = 1;
	}
	HAL_Delay(200);
  }

  // SoftAP config: SSID "STM32", password "cleaningrobot"
  ATisOK = 0;
  while(!ATisOK){
	sprintf(ATcommand,"AT+CWSAP_CUR=\"STM32\",\"cleaningrobot\",1,3,4,0\r\n");
	  memset(rxBuffer,0,sizeof(rxBuffer));
	  HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	  HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
	if(strstr((char *)rxBuffer,"OK")){
	  ATisOK = 1;
	}
	HAL_Delay(200);
  }

  // IP
  ATisOK = 0;
  while(!ATisOK){
	sprintf(ATcommand,"AT+CIPAP_CUR=\"192.168.51.1\"\r\n");
	memset(rxBuffer,0,sizeof(rxBuffer));
	HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
	if(strstr((char *)rxBuffer,"OK")){
	  ATisOK = 1;
	}
	HAL_Delay(200);
  }

  // enable multiple connections
  ATisOK = 0;
  while(!ATisOK){
	sprintf(ATcommand,"AT+CIPMUX=1\r\n");
	  memset(rxBuffer,0,sizeof(rxBuffer));
	  HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	  HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
	  if(strstr((char *)rxBuffer,"OK")){
		ATisOK = 1;
	  }
	  HAL_Delay(200);
  }

  // start server port 80
  ATisOK = 0;
  while(!ATisOK){
	sprintf(ATcommand,"AT+CIPSERVER=1,80\r\n");
	memset(rxBuffer,0,sizeof(rxBuffer));
	HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
	if(strstr((char *)rxBuffer,"OK")){
		ATisOK = 1;
	}
	HAL_Delay(200);
  }
  HAL_Delay(500);
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
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start the first interrupt-driven receive for PMS sensor
  HAL_UART_Receive_IT(&huart2, &rxByte, 1);

  /* Start PWM on TIM1 CH1 (ENA) & CH4 (ENB) */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);

  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
  HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, 0); // brush relay initial
  HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, 0); // extra relay initial

  ILI9341_Init();
  ILI9341_FillScreen(WHITE);
  ILI9341_SetRotation(SCREEN_HORIZONTAL_2);
  ILI9341_DrawText("EDDIE: The Cleaning Robot", FONT3, 30, 10, BLUE, WHITE);

  /* Prepare ESP8266 (ESP-01) as AP + server */

  uint8_t rxBuffer[512] = {0};
  uint8_t ATisOK;
  int channel = 100;
  char ATcommand[128];
  wifiConfiguration(rxBuffer, ATisOK, channel, ATcommand);


  char displayBuf[64]; // Buffer for string formatting
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  float front = HCSR04_Read(2);
	  HAL_Delay(20);
	  float left  = HCSR04_Read(1);
	  HAL_Delay(20);
	  float right = HCSR04_Read(3);

	  if (front < 0) front = 1000.0f;
	  if (left  < 0) left  = 1000.0f;
	  if (right < 0) right = 1000.0f;

	  const float TH = 15.0f;


  // Test
//	  	  		GoForward(60);
//	  	  		HAL_Delay(2000);
//
//	  	  		GoBack(60);
//	  	  		HAL_Delay(2000);
	  	if((wheelEnabled == 0) || (sensorData.pm2_5_env >= 50)){
			if(front < TH)
			{
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
				safeDelay(500);
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
				if( right < TH && left < TH)
				{
					GoBack(60);
					safeDelay(600);
					float front = HCSR04_Read(2);
					HAL_Delay(20);
					float left  = HCSR04_Read(3);
					HAL_Delay(20);
					float right = HCSR04_Read(1);

					if (front < 0) front = 1000.0f;
					if (left  < 0) left  = 1000.0f;
					if (right < 0) right = 1000.0f;
					if(left > TH )
					{
						TurnLeft(60);
						safeDelay(600);
					}
					else if (right > TH)
					{
						TurnRight(60);
						safeDelay(600);
					}
					else
					{
						GoBack(60);
						safeDelay(600);
					}
				}
				else if(right < TH)
				{
					TurnLeft(60);
					safeDelay(600);
				}
				else if (left < TH)
				{
					TurnRight(60);
					safeDelay(600);
				}
			}
			else
			{
				GoForward(60);
			}
	  	} else {StopCoast();}

      static int wheelState = 1; // 1 = OFF, 0 = ON
      static int brushState = 1; // 1 = OFF, 0 = ON
      static int relayState = 0; // 1 = OFF, 0 = ON
      static int wheel = 1;
      static int brush = 1;

      uint8_t rxBuffer[512];
      memset(rxBuffer,0,sizeof(rxBuffer));
      HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);

      // Determine channel from +IPD string
      int channel = 100;
      if(strstr((char *)rxBuffer,"+IPD,0")) channel = 0;
      else if(strstr((char *)rxBuffer,"+IPD,1")) channel = 1;
      else if(strstr((char *)rxBuffer,"+IPD,2")) channel = 2;
      else if(strstr((char *)rxBuffer,"+IPD,3")) channel = 3;
      else if(strstr((char *)rxBuffer,"+IPD,4")) channel = 4;
      else if(strstr((char *)rxBuffer,"+IPD,5")) channel = 5;
      else if(strstr((char *)rxBuffer,"+IPD,6")) channel = 6;
      else if(strstr((char *)rxBuffer,"+IPD,7")) channel = 7;

      if (strstr((char*)rxBuffer, "GET /wheelon")) {
          wheelEnabled = 0;
      } else if (strstr((char*)rxBuffer, "GET /wheeloff")) {
          wheelEnabled = 1;
      }

      if (strstr((char*)rxBuffer, "GET /brushon")) {
          brushState = 0;
      } else if (strstr((char*)rxBuffer, "GET /brushoff")) {
          brushState = 1;
      }

      if (strstr((char*)rxBuffer, "GET /relayon")) {
          relayState = 1;
      } else if (strstr((char*)rxBuffer, "GET /relayoff")) {
          relayState = 0;
      }

      if (brushState) {
          HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
      } else {
          HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
      }

      if (relayState) {
          HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
      } else {
          HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
      }

      if (channel < 8) {
		  if (wheelEnabled) {
			  StopCoast();
		  }

          send_web_page_via_esp(channel, wheelEnabled, brushState, relayState);

          // close connection
          char ATcmdClose[32];
          sprintf(ATcmdClose, "AT+CIPCLOSE=%d\r\n", channel);
          memset(rxBuffer,0,sizeof(rxBuffer));
          HAL_UART_Transmit(&huart1,(uint8_t *)ATcmdClose,strlen(ATcmdClose),1000);
          HAL_UART_Receive (&huart1, rxBuffer, 512, 200);
          channel = 100;
      }

      sprintf(displayBuf, "Distance: %3.1f cm", front);
      ILI9341_DrawText(displayBuf, FONT4, 20, 50, BLACK, WHITE);

      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET)
      {
          sprintf(displayBuf, "Brush State: ON ");
      } else {
          sprintf(displayBuf, "Brush State: OFF");
      }
      ILI9341_DrawText(displayBuf, FONT4, 20, 170, RED, WHITE);

      sprintf(displayBuf, "PM1.0: %u ug/m3  ", sensorData.pm1_0_env);
      ILI9341_DrawText(displayBuf, FONT3, 20, 80, BLUE, WHITE);

      sprintf(displayBuf, "PM2.5: %u ug/m3  ", sensorData.pm2_5_env);
      ILI9341_DrawText(displayBuf, FONT4, 20, 110, RED, WHITE);

      sprintf(displayBuf, "PM10: %u ug/m3 ", sensorData.pm10_env);
      ILI9341_DrawText(displayBuf, FONT3, 20, 140, MAGENTA, WHITE);

      HAL_Delay(1000);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

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
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_Pin|IN2_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_Pin|RESET_Pin|BUZZER_Pin|TRIG1_Pin
                          |TRIG2_Pin|TRIG3_Pin|RELAY1_Pin|RELAY2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin RESET_Pin BUZZER_Pin RELAY1_Pin
                           RELAY2_Pin */
  GPIO_InitStruct.Pin = DC_Pin|RESET_Pin|BUZZER_Pin|RELAY1_Pin
                          |RELAY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG1_Pin TRIG2_Pin TRIG3_Pin */
  GPIO_InitStruct.Pin = TRIG1_Pin|TRIG2_Pin|TRIG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Logic to align the 32-byte frame
        if (rxIndex == 0 && rxByte != 0x42) {
            // Wait for first header byte
            rxIndex = 0;
        } else if (rxIndex == 1 && rxByte != 0x4D) {
            // Ensure second header byte matches
            rxIndex = 0;
        } else {
            rxBufferPMS[rxIndex++] = rxByte;
        }

        if (rxIndex == 32) {
            // Checksum verification
            uint16_t checksum = 0;
            for (uint8_t i = 0; i < 30; i++) {
                checksum += rxBufferPMS[i];
            }
            uint16_t receivedChecksum = (rxBufferPMS[30] << 8) | rxBufferPMS[31];

            if (checksum == receivedChecksum) {
                // Mapping Standard (Lab) values
                sensorData.pm1_0_std = (rxBufferPMS[4] << 8) | rxBufferPMS[5];
                sensorData.pm2_5_std = (rxBufferPMS[6] << 8) | rxBufferPMS[7];
                sensorData.pm10_std  = (rxBufferPMS[8] << 8) | rxBufferPMS[9];

                // Mapping Atmospheric (Real World) values
                sensorData.pm1_0_env = (rxBufferPMS[10] << 8) | rxBufferPMS[11];
                sensorData.pm2_5_env = (rxBufferPMS[12] << 8) | rxBufferPMS[13];
                sensorData.pm10_env  = (rxBufferPMS[14] << 8) | rxBufferPMS[15];
            }
            rxIndex = 0; // Reset for next frame
        }

        // Re-enable interrupt for the next byte
        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

