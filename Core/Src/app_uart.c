/*-----------------------------------------------------------------------
* Nome do projeto: trabalho_final
* Ano            : 2024
* Autor(es)      : Pedro Henrique Domingos
* Arquivo		 : app_uart.c
* Descrição      : Implementação da aplicação referente à UART
*
* "rqcrn" requisita o envio do valor do CRONOMETRO do outro kit para o seu
* "rqadc" req o valor de tensao em Volts, do ADC do outro kit para o seu
* "rqbzz" req que o colega toque o buzzer (5x 200ms por 200 ms OFF)
* "XXXXa" seu kit recebe/envia os digitos do ADC em ASCII + 'a'
* "XXXXc" seu kit recebe/envia os digitos do cronometro em ASCII + 'c'
* Para receber um valor é necessário requisitá-lo.
* ----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------
    INCLUDES
-----------------------------------------------------------------------*/
#include "app_uart.h"


/*-----------------------------------------------------------------------
    DEFINES
-----------------------------------------------------------------------*/

// Tamanho dos buffers de TX e RX da UART
#define SIZE_BUFFS_UART				5

// Num. de comandos armazenados no ring buffer de transmissão
#define SIZE_BUFF_COMANDO_TX		20

// Mensagens de requisão de dados
#define MSG_REQ_CRONOMETRO 			"rqcrn"
#define MSG_REQ_ADC 				"rqadc"
#define MSG_REQ_BUZZER 				"rqbzz"


/*-----------------------------------------------------------------------
    TYPEDEFS
-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------
    LOCAL VARIABLES
-----------------------------------------------------------------------*/
static UART_HandleTypeDef handler_uart1;

// Relativas ao ring buffer de comandos a serem transmitidos
static t_ring_buffer ring_buffer_comando_tx;
static enum_comando_uart buffer_comando_tx[SIZE_BUFF_COMANDO_TX];

// Buffers para entrada e saida de dados via USART
 static uint8_t buffer_tx[SIZE_BUFFS_UART];
 static uint8_t buffer_rx[SIZE_BUFFS_UART];


/*-----------------------------------------------------------------------
    LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------*/
static void MX_USART1_UART_Init(void);
static bool envia_comando_uart(enum_comando_uart comando);
static void trata_comando_recebido(const uint8_t * buffer);


/*-----------------------------------------------------------------------
    GLOBAL FUNCTIONS
-----------------------------------------------------------------------*/

/**
  * @brief  : Incializa a aplicação da UART
  * @param  : None
  * @retval : None
  */
void init_app_uart(void)
{
	// Inicializa o hardware da UART
	MX_USART1_UART_Init();

	// Para a UART começar a receber os dados
	HAL_UART_Receive_IT(&handler_uart1, buffer_rx, sizeof(buffer_rx));

	// Incializa o ring buffer de transmissão dos comandos
	ring_buffer_init(&ring_buffer_comando_tx,
					 buffer_comando_tx,
					 SIZE_BUFF_COMANDO_TX,
					 sizeof(enum_comando_uart));
}

/**
  * @brief  : Insere o comando desejado no ring buffer de transmissão
  * @param  : comando = Comando a ser inserido
  * @retval : None
  */
void insere_comando_uart(enum_comando_uart comando)
{
	ring_buffer_push(&ring_buffer_comando_tx, (void *)&comando);
}

/**
  * @brief  : Rotina para executar a transmissão do que
  * 		  estiver armazenado no ring buffer. Pensada
  * 		  para ser usada dentro de um super loop.
  * @param  : None
  * @retval : None
  */
void executa_transmissao_uart(void)
{
	// Somente prossegue com o envio se não há transmissões
	// em andamento
	if(__HAL_UART_GET_FLAG(&handler_uart1, UART_FLAG_TC) == RESET)
	{
		return;
	}

	enum_comando_uart * ptr_comando;

	// Verifica se há algum comando a ser transmitido no ring buffer
	if((ptr_comando = (enum_comando_uart *)ring_buffer_pop(&ring_buffer_comando_tx)) != NULL)
	{
		(void)envia_comando_uart(*ptr_comando);
	}
}

/**
  * @brief  : Incializa a aplicação da UART
  * @param  : None
  * @retval : None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	trata_comando_recebido(buffer_rx);

	HAL_UART_Receive_IT(&handler_uart1, buffer_rx, sizeof(buffer_rx));
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&handler_uart1);
}


/*-----------------------------------------------------------------------
    LOCAL FUNCTIONS
-----------------------------------------------------------------------*/

/**
  * @brief  : Envia um comando na UART
  * @param  : comando = Comando desejado
  * @retval : true = Mensagem enviada
  * 		  false = Mensagem não enviada
  */
static bool envia_comando_uart(enum_comando_uart comando)
{
	// Identifica a mensagem do comando desejado
	switch(comando)
	{
		case UART_REQ_CRONOMETRO:
			memcpy(buffer_tx, MSG_REQ_CRONOMETRO, sizeof(buffer_tx));
		break;

		case UART_REQ_ADC:
			memcpy(buffer_tx, MSG_REQ_ADC, sizeof(buffer_tx));
		break;

		case UART_REQ_BUZZER:
			memcpy(buffer_tx, MSG_REQ_BUZZER, sizeof(buffer_tx));
		break;

		case UART_SEND_CRONOMETRO:
		{
			buffer_tx[0] = conv_num_ASC(Crono[0]);
			buffer_tx[1] = conv_num_ASC(Crono[1]);
			buffer_tx[2] = conv_num_ASC(Crono[2]);
			buffer_tx[3] = conv_num_ASC(Crono[3]);
			buffer_tx[4] = 'c';
		}
		break;

		case UART_SEND_ADC:
		{
			buffer_tx[0] = conv_num_ASC(ValAdc[0]);
			buffer_tx[1] = conv_num_ASC(ValAdc[1]);
			buffer_tx[2] = conv_num_ASC(ValAdc[2]);
			buffer_tx[3] = conv_num_ASC(ValAdc[3]);
			buffer_tx[4] = 'a';
		}
		break;

		default:
			return false;
	}

	// Transmite a mensagem
	if(HAL_UART_Transmit_IT(&handler_uart1, buffer_tx, sizeof(buffer_tx)) == HAL_OK)
	{
		//flag_finish_tx = false;
		return true;
	}
	else
	{
		return false;
	}
}

/**
  * @brief  : Trata comando recebido na UART
  * @param  : buffer = Ponteiro para o buffer a ser tratado
  * @retval : None
  */
static void trata_comando_recebido(const uint8_t * buffer)
{
	// Verifica se é uma requisição
	if(buffer[0] == 'r')
	{
		if(memcmp(buffer, MSG_REQ_CRONOMETRO, SIZE_BUFFS_UART) == 0)
		{
			// Envia comando para enviar valor do cronometro
			envia_comando_uart(UART_SEND_CRONOMETRO);
		}
		else if(memcmp(buffer, MSG_REQ_ADC, SIZE_BUFFS_UART) == 0)
		{
			// Envia comando para enviar valor do ADC
			envia_comando_uart(UART_SEND_ADC);
		}
		else if(memcmp(buffer, MSG_REQ_BUZZER, SIZE_BUFFS_UART) == 0)
		{
			sttBUZZER = INIBUZZ;
			contbuzzer = 0;
		}
	}
	// Verifica se é dados de cronômetro
	else if(buffer[4] == 'c')
	{
		// Preenche o valor do cronometro recebido
		ExCrono[0] = conv_ASC_num(buffer[0]);
		ExCrono[1] = conv_ASC_num(buffer[1]);
		ExCrono[2] = conv_ASC_num(buffer[2]);
		ExCrono[3] = conv_ASC_num(buffer[3]);
	}
	// Verifica se é dados do ADC
	else if(buffer[4] == 'a')
	{
		// Preenche o valor do ADC recebido
		ExValAdc[0] = conv_ASC_num(buffer[0]);
		ExValAdc[1] = conv_ASC_num(buffer[1]);
		ExValAdc[2] = conv_ASC_num(buffer[2]);
		ExValAdc[3] = conv_ASC_num(buffer[3]);
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
	handler_uart1.Instance = USART1;
	handler_uart1.Init.BaudRate = 115200;
	handler_uart1.Init.WordLength = UART_WORDLENGTH_8B;
	handler_uart1.Init.StopBits = UART_STOPBITS_1;
	handler_uart1.Init.Parity = UART_PARITY_NONE;
	handler_uart1.Init.Mode = UART_MODE_TX_RX;
	handler_uart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	handler_uart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&handler_uart1) != HAL_OK)
	{
		Error_Handler();
	}
}
