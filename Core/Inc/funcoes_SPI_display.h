/* --- arquivo header para serial do display 4x7seg e conversor bcdx7seg ----
Arquivo: funcoes_SPI_display.h
Created on: 2024.08
Author: JRanhel
-----------------------------------------------------------------------------*/

#ifndef FUNCOES_SPI_DISPLAY_H_
#define FUNCOES_SPI_DISPLAY_H_

#include "main.h"

// as constantes usadas nos arquivos com as funcoes.c
#define TIPO_DISPLAY 0                 // tipo de display 0 = Anodo comum

#define MAX_DIGITOS_DISPLAY		4

// Defines para abstração dos quatro pontos decimais
#define ____			0x00
#define ___X			0x01
#define __X_			0x02
#define _X__			0x04
#define X___			0x08
#define __XX			0x03
#define _XXX			0x07
#define X_X_			0x0A
#define XXXX			0x0F


// fn converte um valor hexa (valHEX = uint8_t) em 7-seg (retorna um uint16_t).
// Argumentos: valHEX + KTE TIPO_DISPLAY
uint16_t conv_7_seg(int8_t valHEX);

// fn serializa dados p/ 74HC595 (shield multifuction do Arduino)
// Baixa RCLK, envia 16 bits p/ SDATA e pulsa SCLK. Ao final, sobe RCLK
void serializar(uint16_t ser_data);    // prot fn serializa dados p/ 74HC595

// fn que converte um dos valores numericos em codigo ASCII
// faz critica do valor 'n' que tem que ser entre 0 e 9
uint8_t conv_num_ASC(int8_t n);

// fn que converte ASCII de um numero em valor decimal
// faz critica de cados, o caractere 'c' tem que estar entre 0x30 e 0x39
int8_t conv_ASC_num(uint8_t c);

// fn que faz a varredura e mostra no display os dados do vetor Dsp[]
void mostrar_no_display(int8_t D[], uint8_t pto);

void exibe_display(const int8_t * buff, int8_t ponto_decimal);

// fn que reseta pinos PB6, PB9, PB10 para começarem em nível '1'
void reset_pinos_emula_SPI (void);

#endif  /* FUNCOES_DISPLAY_H_ */
