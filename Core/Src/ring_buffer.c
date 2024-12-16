/*----------------------------------------------------------------------- 
* Nome do projeto: trabalho_final
* Ano            : 2024
* Autor(es)      : Pedro Henrique Domingos
* Arquivo		 : ring_buffer.c
* Descrição      : Implementação do ring buffer
* ----------------------------------------------------------------------*/


/*----------------------------------------------------------------------- 
    INCLUDES
-----------------------------------------------------------------------*/ 
#include <ring_buffer.h>


/*----------------------------------------------------------------------- 
    GLOBAL FUNCTIONS
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
					  uint32_t size_of_element)
{
	*ptr_ring_buffer = (t_ring_buffer){
		.capacity = capacity,
		.buffer = buffer,
		.size_of_element = size_of_element,
		.buffer_end = buffer + (capacity*size_of_element),
		.num_elements = 0,
		.ptr_head = buffer,
		.ptr_tail = buffer
	};
}

/**
  * @brief  : Coloca um elemento no ring buffer
  * @param  : ptr_ring_buffer = Ponteiro para a estrutura do ring buffer
  * 		  element = Ponteiro para elemento a ser inserido
  * @retval : true = Comando enviado
  * 		  false = Comando não enviado
  */
bool ring_buffer_push(t_ring_buffer * ptr_ring_buffer, const void * element)
{
	// Verifica se o ring buffer está cheio
	if(ptr_ring_buffer->ptr_head == ptr_ring_buffer->ptr_tail &&
	   ptr_ring_buffer->num_elements > 0)
	{
		return false;
	}

	memcpy(ptr_ring_buffer->ptr_head, element, ptr_ring_buffer->size_of_element);

	ptr_ring_buffer->ptr_head += ptr_ring_buffer->size_of_element;

	if(ptr_ring_buffer->num_elements < ptr_ring_buffer->capacity)
	{
		ptr_ring_buffer->num_elements++;
	}

	if(ptr_ring_buffer->ptr_head >= ptr_ring_buffer->buffer_end)
	{
		ptr_ring_buffer->ptr_head = ((uint8_t*)ptr_ring_buffer->buffer);
	}

	return true;
}

/**
  * @brief  : Retira um elemento de um ring buffer
  * @param  : ptr_ring_buffer = Ponteiro para a estrutura do ring buffer
  * @retval : NULL = Não há elemento à ser retirado
  */
void * ring_buffer_pop(t_ring_buffer * ptr_ring_buffer)
{
	if(ptr_ring_buffer->num_elements == 0)
	{
		return NULL;
	}

	void * ptr_element = ptr_ring_buffer->ptr_tail;
	ptr_ring_buffer->ptr_tail += ptr_ring_buffer->size_of_element;

	if(ptr_ring_buffer->ptr_tail >= ptr_ring_buffer->buffer_end)
	{
		ptr_ring_buffer->ptr_tail = ((uint8_t*)ptr_ring_buffer->buffer);
	}

	ptr_ring_buffer->num_elements--;

	return ptr_element;
}

