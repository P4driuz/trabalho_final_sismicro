/*----------------------------------------------------------------------- 
* Nome do projeto: trabalho_final
* Ano            : 2024
* Autor(es)      : Pedro Henrique Domingos
* Arquivo		 : ring_buffer.h
* Descrição      : Header da implementação do ring buffer
* ----------------------------------------------------------------------*/

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

/*----------------------------------------------------------------------- 
    INCLUDES
-----------------------------------------------------------------------*/ 
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


/*----------------------------------------------------------------------- 
    TYPEDEFS
-----------------------------------------------------------------------*/ 

// Estrutura de controle do ring buffer
typedef struct ring_buffer_structure
{
  uint32_t capacity;
  uint8_t * buffer;
  size_t size_of_element;
  uint8_t * buffer_end;
  uint32_t num_elements;
  uint8_t * ptr_head;
  uint8_t * ptr_tail;

} t_ring_buffer;


/*----------------------------------------------------------------------- 
    GLOBAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------*/ 

/**
  * @brief  : Inicializa uma estrutura de controle de ring buffer
  * @param  : ptr_ring_buffer = Ponteiro para a estrutura do ring buffer
  * 		  buffer = Ponteiro para o buffer a ser gerenciado
  * 		  capacity = Capacidade total de elementos
  * 		  size_of_element = Tamanho de cada elemento
  * @retval : None
  */
void ring_buffer_init(t_ring_buffer * ptr_ring_buffer,
					   uint8_t *buffer,
					   uint32_t capacity,
					   uint32_t size_of_element);

/**
  * @brief  : Coloca um elemento no ring buffer, sobrescrevendo
  * 		  caso esteja cheio
  * @param  : ptr_ring_buffer = Ponteiro para a estrutura do ring buffer
  * 		  element = Ponteiro para elemento a ser inserido
  * @retval : true = Comando enviado
  * 		  false = Comando não enviado
  */
bool ring_buffer_push(t_ring_buffer * ptr_ring_buffer, const void * element);

/**
  * @brief  : Retira um elemento de um ring buffer
  * @param  : ptr_ring_buffer = Ponteiro para a estrutura do ring buffer
  * @retval : NULL = Não há elemento à ser retirado
  */
void * ring_buffer_pop(t_ring_buffer * ptr_ring_buffer);


#endif /* RING_BUFFER_H_ */
