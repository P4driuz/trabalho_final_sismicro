/*-----------------------------------------------------------------------
* Nome do projeto: trabalho_final
* Ano            : 2024
* Autor(es)      : Pedro Henrique Domingos
* Arquivo		 : app_uart.h
* Descrição      : Header referente à aplicação da UART
* ----------------------------------------------------------------------*/

#ifndef APP_UART_H_
#define APP_UART_H_

/*-----------------------------------------------------------------------
    INCLUDES
-----------------------------------------------------------------------*/
#include <ring_buffer.h>
#include "main.h"
#include "funcoes_SPI_display.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


/*-----------------------------------------------------------------------
    TYPEDEFS
-----------------------------------------------------------------------*/
typedef enum
{
	UART_REQ_CRONOMETRO,
	UART_REQ_ADC,
	UART_REQ_BUZZER,
	UART_SEND_CRONOMETRO,
	UART_SEND_ADC,

} enum_comando_uart;


/*-----------------------------------------------------------------------
    GLOBAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------*/

/**
  * @brief  : Incializa a aplicação da UART
  * @param  : None
  * @retval : None
  */
void init_app_uart(void);

/**
  * @brief  : Insere o comando desejado no ring buffer de transmissão
  * @param  : comando = Comando a ser inserido
  * @retval : None
  */
void insere_comando_uart(enum_comando_uart comando);

/**
  * @brief  : Rotina para executar a transmissão do que
  * 		  estiver armazenado no ring buffer. Pensada
  * 		  para ser usada dentro de um super loop.
  * @param  : None
  * @retval : None
  */
void executa_transmissao_uart(void);


#endif /* APP_UART_H_ */
