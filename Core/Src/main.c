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
******************************************************************************/


//------------------------------------------
// INCLUDES
//------------------------------------------
#include "app_uart.h"
#include "main.h"
#include "funcoes_SPI_display.h"
#include "stm32f1xx_hal_conf.h"
#include <stdint.h>
#include <stdbool.h>


//------------------------------------------
// DEFINES
//------------------------------------------

// Definições de intervalos gerais
#define MS_ADC 						199       // dt = 199 ms, sample rate 5 samples/seg
#define MS_VARREDURA_DISPLAY		6      	  // dt = ~7ms p/ varrer display (142 vz/s)
#define MS_REFRESH					89        // DT = ~90ms p/ nova requisição de dados
#define MS_INTERVALO_ALTER_A1		3999
#define MS_INTERVALO_ALTER_A2		1999
#define MS_LEDS 					199       // intervalo tempo para piscar leds
#define MS_CRONOMETRO 				99        // dt = 99 ms, entra no 100 e ajusta crono
#define MS_DEBOUCING_LOW			200
#define MS_DEBOUCING_HIGH 			100

// Modo de operação do cronometro:
// =0 incrementa, =1 decrementa
#define MODO_CRONOMETRO 			0

// Definições relativas ao buzzer
#define QTDE_TOCA_BUZZER	 		5
#define MS_INTERVALO_TOCA_BUZZER	200

// Definições dos pontos decimais
#define PTO_CRONOMETRO				X_X_
#define PTO_ADC						X___
#define DEFAULT_DISPLAY				{8,8,8,8}

//------------------------------------------
// TYPEDEFS
//------------------------------------------

//Estados deboucing botoes
typedef enum
{
	DB_NORMAL,
	DB_FALL,
	DB_LOW,
	DB_RISING,

} enum_estado_botao;

// Estados requisição externa
typedef enum
{
	IDLEA2,
	REQUISITA_CRONO,
	AGUARDA_CRONO,
	REQUISITA_ADC,
	AGUARDA_ADC,

} enum_estado_REQA2;

// Estados gerais
typedef enum
{
	ESTADO_INICIAL,
	ESTADO_NORMAL,
	ESTADO_ALTERNANCIA_A1,
	ESTADO_ALTERNANCIA_A2,
	ESTADO_ACIONAR_BUZZER,

} enum_estado_geral;

// Estrutura de controle de um botão
typedef struct
{
	GPIO_TypeDef * gpio_base;
	uint16_t gpio_pino;
	enum_estado_botao estado;
	uint32_t last_timestap;
	void (*ptr_func_tratamento)(void);

} t_botao;

// Estados selecao LED
typedef enum
{
	PISCA_LED1,
	PISCA_LED2,
	PISCA_LED3,
	PISCA_LED4,
	APAGA_LEDS,

} enum_estado_leds;

// Estado de um LED
typedef enum
{
	INILED,
	LIGALED,
	DSLGLED,

} enum_stt_leds;
//------------------------------------------
// LOCAL FUNCTIONS PROTOTYPES
//------------------------------------------
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_NVIC_Init(void);

static void verifica_botao(t_botao * botao);
static void trata_botao_a1(void);
static void trata_botao_a2(void);
static void trata_botao_a3(void);

static void maquina_controle(void);
static void maquina_exibicao_display(void);
static void maquina_leds(void);
static void maquina_tocabuzzer(void);
static void maquina_requisicaoA2(void);


//------------------------------------------
// LOCAL VARIABLES
//------------------------------------------
ADC_HandleTypeDef handler_adc1;

// os vetores seguintes tem idx[0] = digito menos significativo no display
volatile int8_t Crono[] = {0,0,0,0};            // vetor com vals dec do cronometro
volatile int8_t Default[] = DEFAULT_DISPLAY;
volatile int8_t ValAdc[] = {0,0,0,0};           // vetor com vals decimais do ADC
volatile int8_t ExCrono[] = DEFAULT_DISPLAY;          // vetor externo vals dec do crono
volatile int8_t ExValAdc[] = DEFAULT_DISPLAY;         // vetor externo vals dec do ADC

const int8_t array_teste_display[] = DEFAULT_DISPLAY;

enum_estado_leds estadoleds = APAGA_LEDS;
enum_stt_leds sttLED = INILED;
enum_stt_buzzer sttBUZZER = IDLE;
enum_estado_REQA2 sttreqa2 = IDLEA2;
enum_estado_geral estadoAtual = ESTADO_INICIAL;

//tempos decorridos
uint32_t tempoAnteriorLED = 0;
uint32_t tempoAnteriorBUZZER = 0;
uint32_t tempoAnteriorREQA2 = 0;
uint32_t tempoAnterior = 0;
uint32_t timestamp_varredura = 0;   // salva tempo última varredura

//Flag que detecta se A1 foi pressionado
uint32_t flagA1 = 0;


bool exibirCronometro = true; // Controla se exibe o cronômetro ou ADC
int32_t contadorAlternanciaA2 = 0; // Controla a alternância de 4 valores no A2
int32_t contbuzzer= 0; // Controla a quantidade de buzzer

// Estrutura de controle botão A1
t_botao botao_a1 = (t_botao)
{
	.gpio_base = GPIOA,
	.gpio_pino = GPIO_PIN_1,
	.last_timestap = 0,
	.estado = DB_NORMAL,
	.ptr_func_tratamento = trata_botao_a1,
};

// Estrutura de controle botão A2
t_botao botao_a2 = (t_botao)
{
	.gpio_base = GPIOA,
	.gpio_pino = GPIO_PIN_2,
	.last_timestap = 0,
	.estado = DB_NORMAL,
	.ptr_func_tratamento = trata_botao_a2,
};

// Estrutura de controle botão A3
t_botao botao_a3 = (t_botao)
{
	.gpio_base = GPIOA,
	.gpio_pino = GPIO_PIN_3,
	.last_timestap = 0,
	.estado = DB_NORMAL,
	.ptr_func_tratamento = trata_botao_a3,
};

//------------------------------------------
// MAIN
//------------------------------------------
int main(void)
{
	// Reset of all peripherals, Initializes the Flash interface and the Systick.
	HAL_Init();

	SystemClock_Config();
	// Configure the system clock

	// Initialize all configured peripherals
	MX_GPIO_Init();
	MX_ADC1_Init();

	// Inicializa a aplicação da UART
	init_app_uart();

	// Initialize interrupts
	MX_NVIC_Init();

	// Começa SPI emulada com pinos = 'high'
	reset_pinos_emula_SPI();

	while (1)
	{
		verifica_botao(&botao_a1);
		verifica_botao(&botao_a2);
		verifica_botao(&botao_a3);

		//Maquinas de estado
		maquina_controle();
		maquina_exibicao_display();
		maquina_leds();
		maquina_tocabuzzer();
		maquina_requisicaoA2();

		// Comunicacao UART
		executa_transmissao_uart();
	}
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&handler_adc1);
}


//------------------------------------------
// LOCAL FUNCTIONS
//------------------------------------------

/**
  * @brief  : Apaga todos os leds
  * @param  : None
  * @retval : None
  */
static void apagaleds(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	}




/**
  * @brief  : Máquina para tocar o buzzer
  * @param  : None
  * @retval : None
  */
static void maquina_tocabuzzer(void){
	switch(sttBUZZER){
	case IDLE:
		break;
	case INIBUZZ:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		sttBUZZER = TOCABUZZ;
		tempoAnteriorBUZZER = HAL_GetTick();
		break;

	case TOCABUZZ:
		if (HAL_GetTick() - tempoAnteriorBUZZER >= MS_INTERVALO_TOCA_BUZZER){
			tempoAnteriorBUZZER = HAL_GetTick();
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			sttBUZZER = DESLBUZZ;
		}
	break;
	case DESLBUZZ:
		if (HAL_GetTick() - tempoAnteriorBUZZER >= MS_INTERVALO_TOCA_BUZZER){
			tempoAnteriorBUZZER = HAL_GetTick();
			contbuzzer++;
			if(contbuzzer<QTDE_TOCA_BUZZER){
				sttBUZZER = INIBUZZ;
			}
			else{
				sttBUZZER = IDLE;
			}
		}

	break;
	}
}



/**
  * @brief  : Executa a máquina de controle dos leds
  * @param  : None
  * @retval : None
  */
static void maquina_leds(void)
{
	switch(estadoleds)
	{
		case PISCA_LED1:
			switch(sttLED){
				case INILED:
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
					sttLED = LIGALED;             
					break;
				case LIGALED:                    // estado para ligar o LED
					if ((HAL_GetTick() - tempoAnteriorLED) > MS_LEDS) { 
						tempoAnteriorLED = HAL_GetTick();
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); 
						sttLED = DSLGLED;
					}         
        			break;
				case DSLGLED:                    // estado para desligar o LED
					if ((HAL_GetTick() - tempoAnteriorLED) > MS_LEDS) { 
						tempoAnteriorLED = HAL_GetTick();          
						sttLED = INILED;             // muda o estado de desligado
        			}
			}
		break;
		case PISCA_LED2:
			switch(sttLED){
				case INILED:                
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
					sttLED = LIGALED;              
					break;
				case LIGALED:                    
					if ((HAL_GetTick() - tempoAnteriorLED) > MS_LEDS) { 
						tempoAnteriorLED = HAL_GetTick();
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); 
						sttLED = DSLGLED;
					}          
        			break;
				case DSLGLED:                    
					if ((HAL_GetTick() - tempoAnteriorLED) > MS_LEDS) {
						tempoAnteriorLED = HAL_GetTick();          
						sttLED = INILED;             
        			}
			}
		break;
		case PISCA_LED3:
			switch(sttLED){
				case INILED:                 
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
					sttLED = LIGALED;              
					break;
				case LIGALED:                   
					if ((HAL_GetTick() - tempoAnteriorLED) > MS_LEDS) { 
						tempoAnteriorLED = HAL_GetTick();
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 
						sttLED = DSLGLED;
					}          
        			break;
				case DSLGLED:                    
					if ((HAL_GetTick() - tempoAnteriorLED) > MS_LEDS) { 
						tempoAnteriorLED = HAL_GetTick();          
						sttLED = INILED;             
        			}
			}
		break;
		case PISCA_LED4:
			switch(sttLED){
				case INILED:                 
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
					sttLED = LIGALED;              
					break;
				case LIGALED:                    
					if ((HAL_GetTick() - tempoAnteriorLED) > MS_LEDS) { 
						tempoAnteriorLED = HAL_GetTick();
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); 
						sttLED = DSLGLED;
					}         
        			break;
				case DSLGLED:                    
					if ((HAL_GetTick() - tempoAnteriorLED) > MS_LEDS) { 
						tempoAnteriorLED = HAL_GetTick();          
						sttLED = INILED;             
        			}
			}
		break;
		case APAGA_LEDS:
			apagaleds();
		break;
		
		
	}
}


/**
  * @brief  : Máquina de estados para controle geral
  * @param  : None
  * @retval : None
  */
static void maquina_controle(void)
{
	switch(estadoAtual)
	{
		case ESTADO_INICIAL:
		break;

		case ESTADO_ALTERNANCIA_A1:
			if (HAL_GetTick() - tempoAnterior >= MS_INTERVALO_ALTER_A1)
			{
				// Alternar entre cronômetro e ADC
				exibirCronometro = !exibirCronometro;
				apagaleds();
				tempoAnterior = HAL_GetTick();
			}
		break;

		case ESTADO_ALTERNANCIA_A2:
			if (HAL_GetTick() - tempoAnterior >= MS_INTERVALO_ALTER_A2)
			{
				tempoAnterior = HAL_GetTick();

				// Alterar para o próximo valor a ser exibido no ciclo A2
				contadorAlternanciaA2++;
				apagaleds();
				if(contadorAlternanciaA2 > 3)
				{
					contadorAlternanciaA2 = 0;
				}
			}
		break;

		default:
			estadoAtual = ESTADO_INICIAL;
		break;
	}
}

/**
  * @brief  : Executa a máquina de controle do display
  * @param  : None
  * @retval : None
  */
static void maquina_exibicao_display(void)
{
	// MAQUINA EXIBICACAO DISPLAY
	if( (HAL_GetTick() - timestamp_varredura) > MS_VARREDURA_DISPLAY)
	{
		timestamp_varredura = HAL_GetTick();
		switch(estadoAtual)
		{
			 case ESTADO_INICIAL:
				exibe_display(array_teste_display, XXXX);
			 break;

			 case ESTADO_ALTERNANCIA_A1:
				 if(exibirCronometro)
				 {
					 exibe_display(Crono, PTO_CRONOMETRO);
					 estadoleds = PISCA_LED1;
				 }
				else
				{
					exibe_display(ValAdc, PTO_ADC);
					estadoleds = PISCA_LED2;
				}
		 break;

		 case ESTADO_ALTERNANCIA_A2:
			 switch(contadorAlternanciaA2)
			 {
				 case 0:
					if(flagA1 == 1){
						exibe_display(Crono, PTO_CRONOMETRO);
					}
					else{
						exibe_display(Default, XXXX);
					}
					estadoleds = PISCA_LED1;
				 break;

				 case 1:
						if(flagA1 == 1){
							exibe_display(ValAdc, PTO_ADC);
						}
						else{
							exibe_display(Default, XXXX);
						}
						estadoleds = PISCA_LED2;
				 break;

				 case 2:
					exibe_display(ExCrono, PTO_CRONOMETRO);
					estadoleds = PISCA_LED3;
				 break;

				 case 3:
					exibe_display(ExValAdc, PTO_ADC);
					estadoleds = PISCA_LED4;
				 break;
				 default:
				 break;
			 }
		 break;
		 default:
		 break;
	 }
  }
}

/**
  * @brief  : Máquina para controlar a requisicao pela UART
  * @param  : None
  * @retval : None
  */
static void maquina_requisicaoA2(void){
	switch(sttreqa2){
	case IDLEA2:
		break;

	case REQUISITA_CRONO:
		insere_comando_uart(UART_REQ_CRONOMETRO);
		tempoAnteriorREQA2 = HAL_GetTick();
		sttreqa2 = AGUARDA_CRONO;
	break;

	case AGUARDA_CRONO:
		if(HAL_GetTick() -tempoAnteriorREQA2 > MS_REFRESH){
			sttreqa2 = REQUISITA_ADC;
		}
	break;

	case REQUISITA_ADC:
		insere_comando_uart(UART_REQ_ADC);
		tempoAnteriorREQA2 = HAL_GetTick();
		sttreqa2 = AGUARDA_ADC;
	break;

	case AGUARDA_ADC:
		if(HAL_GetTick() -tempoAnteriorREQA2 > MS_REFRESH){
			sttreqa2 = REQUISITA_CRONO;
		}
	break;
	}
}

/**
  * @brief  : Verifica o acionamento e trata os botões
  * @param  : t_botao * botao = Ponteiro para botao a ser tratado
  * @retval : None
  */
static void verifica_botao(t_botao * botao)
{
	switch(botao->estado)
	{
		case DB_NORMAL:
			if(HAL_GPIO_ReadPin(botao->gpio_base, botao->gpio_pino) == 0)
			{
				botao->last_timestap = HAL_GetTick();
				botao->estado = DB_FALL;
			  	botao->ptr_func_tratamento();
			}
		break;

		case DB_FALL:
			if ( (HAL_GetTick() - botao->last_timestap) > MS_DEBOUCING_LOW)
			{
				botao->estado = DB_LOW;
			}
		break;

		case DB_LOW:
			if(HAL_GPIO_ReadPin(botao->gpio_base, botao->gpio_pino) == 1)
			{
				botao->last_timestap = HAL_GetTick();
				botao->estado = DB_RISING;
			}
		break;

		case DB_RISING:
			if ( (HAL_GetTick() - botao->last_timestap) > MS_DEBOUCING_HIGH)
			{
				botao->estado = DB_NORMAL;
			}
		break;

		default:
		break;
	}
}

/**
  * @brief  : Rotina de tratamento do botão A1
  * @param  : None
  * @retval : None
  */
static void trata_botao_a1(void)
{
	estadoAtual = ESTADO_ALTERNANCIA_A1;
	apagaleds();
	flagA1 = 1;
	sttreqa2 = IDLEA2;
}

/**
  * @brief  : Rotina de tratamento do botão A2
  * @param  : None
  * @retval : None
  */
static void trata_botao_a2(void)
{
	sttreqa2 = REQUISITA_CRONO;
	estadoAtual = ESTADO_ALTERNANCIA_A2;
	apagaleds();
}

/**
  * @brief  : Rotina de tratamento do botão A1
  * @param  : None
  * @retval : None
  */
static void trata_botao_a3(void)
{
	insere_comando_uart(UART_REQ_BUZZER);


}

/**
  * @brief  : Callback do systick, ajusta o cronometro
  * 		  e dispara conversão do ADC
  * @param  : None
  * @retval : None
  */
void Ajusta_Crono_ADC(void)
{
	static uint16_t conta = 0;
	static uint16_t conta_ADC = 0;

	if (conta >= MS_CRONOMETRO)
	{
		conta = 0;
		if (MODO_CRONOMETRO == 0) {
			// MD_CRONO = 0 incrementa o cronômetro
			++Crono[0];
			if (Crono[0] > 9) {
				Crono[0] = 0;
				++Crono[1];
				if (Crono[1] > 9) {
					Crono[1] = 0;
					++Crono[2];
					if (Crono[2] > 5) {
						Crono[2] = 0;
						++Crono[3];
						if (Crono[3] > 9) {
							Crono[3] = 0;
		} } } } } else {
			// MD_CRONO = 1 decrementa o cronômetro
			--Crono[0];
			if (Crono[0] < 0) {
				Crono[0] = 9;
				--Crono[1];
				if (Crono[1] < 0) {
					Crono[1] = 9;
					--Crono[2];
					if (Crono[2] < 0) {
						Crono[2] = 5;
						--Crono[3];
						if (Crono[3] < 0) {
							Crono[3] = 9;
		} } } } } } else {
		++conta;
	}

	if (conta_ADC >= MS_ADC)
	{
		conta_ADC = 0;

		// Dispara conversao do ADC por IRQ
		HAL_ADC_Start_IT(&handler_adc1);
	}
	else
	{
		++conta_ADC;
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
	/* ADC1_2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

	/* USART1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Common config
	*/
	handler_adc1.Instance = ADC1;
	handler_adc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	handler_adc1.Init.ContinuousConvMode = DISABLE;
	handler_adc1.Init.DiscontinuousConvMode = DISABLE;
	handler_adc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	handler_adc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	handler_adc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&handler_adc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	*/
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&handler_adc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
						  |GPIO_PIN_15|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_9, GPIO_PIN_SET);

	/*Configure GPIO pins : PA1 PA2 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB12 PB13 PB14
						   PB15 PB5 PB6 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
						  |GPIO_PIN_15|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// fn que atende ao callback da ISR do conversor ADC1
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint16_t val_adc = 0;                // define var para ler ADC

	// se veio ADC1
	if(hadc->Instance == ADC1)
	{
		val_adc = HAL_ADC_GetValue(&handler_adc1);// capta valor adc
		// converter o valor lido em valores hexa p/ display
		int miliVolt = val_adc*3300/4095;
		int uniADC = miliVolt/1000;
		int decADC = (miliVolt-(uniADC*1000))/100;
		int cnsADC = (miliVolt-(uniADC*1000)-(decADC*100))/10;
		int mlsADC = miliVolt-(uniADC*1000)-(decADC*100)-(cnsADC*10);
		ValAdc[3] = uniADC;         // dig mais significativo
		ValAdc[2] = decADC;
		ValAdc[1] = cnsADC;
		ValAdc[0] = mlsADC;
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
