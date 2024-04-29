#include "main.h"
#include <string.h>
#include <math.h>
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);


#define CRC_8 (0x07)
uint8_t CRC_u8Checksum = 0x00;
uint8_t calculate_crc(uint8_t *data, uint8_t length);
#define STPM3x_FRAME_LEN (4)
void delay_ms(uint32_t ms);
int count = 0;
int response_delay= 0;
char buf_tx[20];
char buf_tx1[] = "\n btn off!\r\n";
char buf_tx2[10];
char buf_tx3[20];
char buf_tx4[20];
char buf_tx5[20];
char buf_tx6[20];
char buf_tx7[20];
char buf_tx8[20];
char buf_tx9[20];
char buf_tx10[20];
int trama_enviada =0;

/* Data buffer to receive response */
uint8_t rx_data[5];

uint8_t rx_data1[5];
uint8_t rx_data2[5];
uint8_t rx_data3[5];
uint8_t data1[5];
uint8_t data2[5];
uint8_t data3[5];

double Ks = 0.3e-3;



int main(void)
{
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();


 char *texto = "\n\r Reinicio ... \n";
  HAL_UART_Transmit(&huart2, (uint8_t*)texto, strlen(texto), HAL_MAX_DELAY);
  /*Pin CS down para bloquear la eleccion de protocolo SPI*/
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
  while (1)
  {
	  if (HAL_GPIO_ReadPin(GPIOC, BUTTON_Pin) == GPIO_PIN_RESET && trama_enviada == 0) {

    //Pin PA_9 pulso para encender la placa de medicion atraves de circuito mosfet.
	 HAL_GPIO_WritePin(GPIOA, OUT3_3V_Pin ,GPIO_PIN_SET);

	  reinicioRegistro1();

	  for(int j=0;j<16;j++){
      capturaDatos();
	  /*CS Inicio de comunicacion */
      HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);

      //LED ON
      HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin, GPIO_PIN_SET);
      /*---------------------    Solicitud direccion de lectura direccion 0x48 fila 36  --------------------------*/
      uint8_t data1[5] = {0x48, 0xFF, 0xFF, 0xFF, 0x00};
      /* Calculate CRC */
      data1[4] = calculate_crc(data1, 4);
      /* Transmit data + CRC */
      HAL_SPI_Transmit(&hspi1, data1, STPM3x_FRAME_LEN + 1, HAL_MAX_DELAY); // Transmitir la trama completa (datos + CRC)
 /*--------------------Recepcion de datos de la direccion 48 (32-bit data + CRC)-------------- */
      HAL_SPI_Receive(&hspi1, rx_data1, sizeof(rx_data1), HAL_MAX_DELAY);

      if(trama_enviada == 0){
    	  char *texto1 = "Trama enviada por SPI \n";
    	  HAL_UART_Transmit(&huart2, (uint8_t*)texto1, strlen(texto1), HAL_MAX_DELAY);
      // Enviar datos por UART en formato hexadecimal
      for (int i = 0; i < sizeof(data1); i++) {
    	  sprintf(buf_tx, "%02X ", data1[i]); // Convertir el byte a formato hexadecimal y guardarlo en el buffer
    	  HAL_UART_Transmit(&huart2, (uint8_t*)buf_tx, strlen(buf_tx), HAL_MAX_DELAY); // Transmitir el byte por UART
      }
      	  char *texto1_1 = "\n Vrms y Irms  direccion registro 0x48 \n";
      	  HAL_UART_Transmit(&huart2, (uint8_t*)texto1_1, strlen(texto1_1), HAL_MAX_DELAY);
      	  trama_enviada=1;
      }
      for (int i = sizeof(rx_data1) - 1; i >= 0; i--) {
    	  sprintf(buf_tx2, "%02X ", rx_data1[i]); // Convertir el byte a formato hexadecimal y guardarlo en el buffer
    	  HAL_UART_Transmit(&huart2, (uint8_t*)buf_tx2, strlen(buf_tx2), HAL_MAX_DELAY); // Transmitir el byte por UART
      }

      /*---Conversion de los registros a valores reales ----*/
			  uint32_t received_data = (uint32_t)rx_data1[3] << 24 |
									  (uint32_t)rx_data1[2] << 16 |
									  (uint32_t)rx_data1[1] << 8 |
									  (uint32_t)rx_data1[0];
				  // Bytes [14:0] para calcular Vrms
				  uint32_t vrms_bytes = received_data & 0x7FFF; // Máscara para obtener los primeros 15 bits
				  // Calcular el valor VRMS
				  long long int potencia = pow(2,15);
				  float vrms_decimal = ((float)vrms_bytes * 1.2 * (1 + 810000.0 / 470)) / (0.875 * 2 * potencia);

				  // Obtener los bytes [31:15] para calcular Irms
				  uint32_t irms_bytes = received_data >> 15;
				  long long int potencia2 = pow(2,17);
				  float irms_decimal = ((float)irms_bytes * 1.2)/ (0.875*16* potencia2 *Ks*1);

	  /*Deteccion de errores utilizando el metodo CRC*/
	  uint8_t calculated_crc = calculate_crc(rx_data1, 4);
		if (calculated_crc == rx_data1[4]) {
			 char texto3[50];
			 char texto4[50];

			 sprintf(buf_tx3, "%.2f",(float)vrms_decimal);
			 sprintf(texto3, "-> Vrms (%s) ", buf_tx3);
			 HAL_UART_Transmit(&huart2, (uint8_t*)texto3, strlen(texto3), HAL_MAX_DELAY);

			 sprintf(buf_tx4, "%.5f",(float)irms_decimal);
			 sprintf(texto4, " Irms (%s) \n", buf_tx4);
			 HAL_UART_Transmit(&huart2, (uint8_t*)texto4, strlen(texto4), HAL_MAX_DELAY);

		} else {
			 char *texto5 = "->Incorrect CRC\r\n	";
			 HAL_UART_Transmit(&huart2, (uint8_t*)texto5, strlen(texto5), HAL_MAX_DELAY);
		}


			 calculated_crc = 0;
			 memset(rx_data1, 0, sizeof(rx_data1));
			 /* Se cierra la comunicacion SCS --> up */
			 HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
			 delay_ms(500);

	  	  }//-->end for
	  delay_ms(100);
	  activEnergy();
	  delay_ms(200);
	  activePower();


	  	  // LED OFF
	  	 HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin, GPIO_PIN_RESET);
	  	 char *texto_fin = "FIN de comunicacion \n";
		 HAL_UART_Transmit(&huart2, (uint8_t*)texto_fin, strlen(texto_fin), HAL_MAX_DELAY);
		 delay_ms(10);

	  }//-->end if BUTTON_Pin
  }
}



void reinicioRegistro1(void) {
/* Pull SYN pin up for 25ms */
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
delay_ms(25);
/* Tiempo para bloquear la elección de la interfaz */
HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
//Tiempo entre el encendido y el reset del pin(SYN)
delay_ms (25);
/*----------------- Pulsos de reinicio de registros STPM32-------------------*/

/* Generate 2 pulses on SYN pin  */
/*Restablecimiento de registros de medición y reinicio de contadores*/
for (int i = 0; i < 2; i++) {
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
delay_ms(1);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
delay_ms(1);
}
}

void capturaDatos(void){
/* Pull SYN pin up for 25ms */
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
delay_ms(25);
/* Tiempo para bloquear la elección de la interfaz */
HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
//Tiempo entre el encendido y el reset del pin(SYN)
delay_ms (25);
/* Generate 1 pulses on SYN pin  */
/*Para la captura de datos*/
for (int i = 0; i < 1; i++) {
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
delay_ms(1);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
delay_ms(1);
}
}

void reinicioDSP(void) {
  	  /* Pull SYN pin up for 25ms */
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  	  delay_ms(25);
  	  /* Tiempo para bloquear la elección de la interfaz */
  	  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
  	  //Tiempo entre el encendido y el reset del pin(SYN)
  	  delay_ms (25);
  	  /*----------------- Pulsos de reinicio de registros STPM32-------------------*/

  	   /* Generate 3 pulses on SYN pin reiniciar configuracion */
  	   for (int i = 0; i < 3; i++) {
  	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  	  	  delay_ms(1);
  	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  	  	  delay_ms(1);
  	   }
  	   /* Leave SYN pin high */
  	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  	   delay_ms(1);//Delay from SYN to SCS
  	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
  	   delay_ms(1);//Reset pulse width
  	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
}

void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}


void activEnergy (void){
for(int q=0;q<7;q++){
capturaDatos();
/*CS Inicio de comunicacion */
HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);

//LED ON
HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin, GPIO_PIN_SET);

/*------------------- Solicitud direccion 0x56 Fila 42  PH1 Active Energy [31:0]-------------*/
uint8_t data3[5] = {0x56, 0xFF, 0xFF, 0xFF, 0x00};
/* Calculate CRC */
data3[4] = calculate_crc(data2, 4);
/* Transmit data + CRC */
HAL_SPI_Transmit(&hspi1, data3, STPM3x_FRAME_LEN + 1, HAL_MAX_DELAY); // Transmitir la trama completa (datos + CRC)
/*--------------------Recepcion de datos de la direccion 5C (32-bit data + CRC)-------------- */
HAL_SPI_Receive(&hspi1, rx_data3, sizeof(rx_data3), HAL_MAX_DELAY);

if(trama_enviada == 1){
char *texto9 = "Trama enviada por SPI \n";
HAL_UART_Transmit(&huart2, (uint8_t*)texto9, strlen(texto9), HAL_MAX_DELAY);
// Enviar datos por UART en formato hexadecimal
for (int i = 0; i < sizeof(data3); i++) {
sprintf(buf_tx8, "%02X ", data3[i]); // Convertir el byte a formato hexadecimal y guardarlo en el buffer
HAL_UART_Transmit(&huart2, (uint8_t*)buf_tx8, strlen(buf_tx8), HAL_MAX_DELAY); // Transmitir el byte por UART
}
char *texto9_1 = "\n Energia Activa direccion registro 0x56 \n";
HAL_UART_Transmit(&huart2, (uint8_t*)texto9_1, strlen(texto9_1), HAL_MAX_DELAY);
trama_enviada=2;
}

for (int i = sizeof(rx_data3) - 1; i >= 0; i--) {
sprintf(buf_tx9, "%02X ", rx_data3[i]); // Convertir el byte a formato hexadecimal y guardarlo en el buffer
HAL_UART_Transmit(&huart2, (uint8_t*)buf_tx9, strlen(buf_tx9), HAL_MAX_DELAY); // Transmitir el byte por UART
}
uint32_t received_data_EA =   (uint32_t)rx_data3[3] << 24 |
	  (uint32_t)rx_data3[2] << 16 |
	  (uint32_t)rx_data3[1] << 8 |
	  (uint32_t)rx_data3[0];

// Bytes [28:0] para calcular LSBp
uint32_t LSBe_bytes = received_data_EA & 0xFFFFFFF; // Esta máscara tiene los primeros 28 bits
// Calcular LSBp según la ecuación dada
long long int potencia3 = pow(2,17);
float Vref_2 = pow(1.2,2);
float LSBe_decimal = ((float)LSBe_bytes * Vref_2 * (1 + 810000.0 / 470)) / (1*3600*7812.5*2*16*Ks*0.875*0.875* potencia3);

uint8_t calculated_crc3 = calculate_crc(rx_data3, 4);
if (calculated_crc3 == rx_data3[4]) {
char texto10[50];

sprintf(buf_tx10, "%.6f",(float)LSBe_decimal);
sprintf(texto10, "-> LSBe (%s)[Wh] \n ", buf_tx10);
HAL_UART_Transmit(&huart2, (uint8_t*)texto10, strlen(texto10), HAL_MAX_DELAY);

} else {
char *texto11 = "->Incorrect CRC\r\n	";
HAL_UART_Transmit(&huart2, (uint8_t*)texto11, strlen(texto11), HAL_MAX_DELAY);
}
calculated_crc3 = 0;
memset(rx_data3, 0, sizeof(rx_data3));
/* Se cierra la comunicacion SCS --> up */
HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
delay_ms(800);
}//--> end for q
}


void activePower(void){
	  for(int p=0;p<7;p++){
		  capturaDatos();
/*CS Inicio de comunicacion */
HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);

//LED ON
HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin, GPIO_PIN_SET);

/*------------------- Solicitud direccion 0x5C Fila 46  PH1 ActivePower [28:0]-------------*/
uint8_t data2[5] = {0x5C, 0xFF, 0xFF, 0xFF, 0x00};
/* Calculate CRC */
data2[4] = calculate_crc(data2, 4);
/* Transmit data + CRC */
HAL_SPI_Transmit(&hspi1, data2, STPM3x_FRAME_LEN + 1, HAL_MAX_DELAY); // Transmitir la trama completa (datos + CRC)
/*--------------------Recepcion de datos de la direccion 5C (32-bit data + CRC)-------------- */
HAL_SPI_Receive(&hspi1, rx_data2, sizeof(rx_data2), HAL_MAX_DELAY);

if(trama_enviada == 2){
char *texto6 = "Trama enviada por SPI \n";
HAL_UART_Transmit(&huart2, (uint8_t*)texto6, strlen(texto6), HAL_MAX_DELAY);
// Enviar datos por UART en formato hexadecimal
for (int i = 0; i < sizeof(data2); i++) {
sprintf(buf_tx5, "%02X ", data2[i]); // Convertir el byte a formato hexadecimal y guardarlo en el buffer
HAL_UART_Transmit(&huart2, (uint8_t*)buf_tx5, strlen(buf_tx5), HAL_MAX_DELAY); // Transmitir el byte por UART
}
char *texto6_1 = "\n Potecia Activa direccion registro 0x5C \n";
HAL_UART_Transmit(&huart2, (uint8_t*)texto6_1, strlen(texto6_1), HAL_MAX_DELAY);
trama_enviada=3;
}

for (int i = sizeof(rx_data2) - 1; i >= 0; i--) {
sprintf(buf_tx6, "%02X ", rx_data2[i]); // Convertir el byte a formato hexadecimal y guardarlo en el buffer
HAL_UART_Transmit(&huart2, (uint8_t*)buf_tx6, strlen(buf_tx6), HAL_MAX_DELAY); // Transmitir el byte por UART
}
uint32_t received_data_PA =   (uint32_t)rx_data2[3] << 24 |
						  (uint32_t)rx_data2[2] << 16 |
						  (uint32_t)rx_data2[1] << 8 |
						  (uint32_t)rx_data2[0];

// Bytes [28:0] para calcular LSBp
uint32_t LSBp_bytes = received_data_PA & 0x0FFFFFFF; // Esta máscara tiene los primeros 28 bits
// Calcular LSBp según la ecuación dada
long long int potencia2 = pow(2,28);
double Vref2 = pow(1.2,2);
float LSBp_decimal = ((float)LSBp_bytes * Vref2 * (1 + 810000.0 / 470)) / (1*2*16*Ks*0.875*0.875* potencia2);

uint8_t calculated_crc2 = calculate_crc(rx_data2, 4);
if (calculated_crc2 == rx_data2[4]) {
char texto7[50];
//char texto7_1[50];

sprintf(buf_tx7, "%.5f",(float)LSBp_decimal);
sprintf(texto7, "-> LSBp (%s)[W] \n ", buf_tx7);
HAL_UART_Transmit(&huart2, (uint8_t*)texto7, strlen(texto7), HAL_MAX_DELAY);

/* sprintf(buf_tx7, "%.5f",(float)irms_decimal);
sprintf(texto7_1, " Irms (%s) \n", buf_tx7);
HAL_UART_Transmit(&huart2, (uint8_t*)texto7_1, strlen(texto7_1), HAL_MAX_DELAY);*/

} else {
char *texto8 = "->Incorrect CRC\r\n	";
HAL_UART_Transmit(&huart2, (uint8_t*)texto8, strlen(texto8), HAL_MAX_DELAY);
}

			 calculated_crc2 = 0;
			 memset(rx_data2, 0, sizeof(rx_data1));
			 /* Se cierra la comunicacion SCS --> up */
			 HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
			 delay_ms(400);
		  }//--> end for int p
}



/* -----------------  CRC calculation function  ----------------------- */
// Función para calcular el CRC
static void Crc8Calc(uint8_t u8Data)
{
    uint8_t loc_u8Idx;
    uint8_t loc_u8Temp;
    loc_u8Idx = 0;
    while (loc_u8Idx < 8)
    {
        loc_u8Temp = u8Data ^ CRC_u8Checksum;
        CRC_u8Checksum <<= 1;
        if (loc_u8Temp & 0x80)
        {
            CRC_u8Checksum ^= CRC_8;
        }
        u8Data <<= 1;
        loc_u8Idx++;
    }
}

// Función para calcular el CRC de una trama de datos
uint8_t calculate_crc(uint8_t *data, uint8_t length)
{
    CRC_u8Checksum = 0x00;
    for (int i = 0; i < length; i++)
    {
        Crc8Calc(data[i]);
    }
    return CRC_u8Checksum;
}







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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|OUT3_3V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SYN_GPIO_Port, SYN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT3_3V_Pin */
  GPIO_InitStruct.Pin = OUT3_3V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT3_3V_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SYN_Pin */
  GPIO_InitStruct.Pin = SYN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYN_GPIO_Port, &GPIO_InitStruct);

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
