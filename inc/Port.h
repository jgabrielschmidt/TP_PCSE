/*
 * Archivo: Port.h
 * Autor: Jorge Gabriel Schmidt
 *
 * Introduccion:
 *
 * Este archivo contiene las funciones para configurar y leer los registros del NRF24L01.
 * Las funciones manejan la comunicacion del modulo NRF24L01 y la placa STM32F429Zi a
 * traves del puerto SPI 1.
 * Los pines de control del modulo NRF24L01 estan mapeados en los siguientes pines de la
 * placa STM32F429Zi:
 *
 * CE: 	 PA_7
 * CSN:  PA_15
 * MOSI: PB_5
 * MISO: PB_4
 * SCK:  PB_3
 * IRQ:  NO CONFIGURADO
 *
 * EL MÓDULO NRF24L01 funciona en SPI Mode 0 (CPOL = 0, CPHA = 0), MSb first.
 *
 * <Instruction word: MSBit to LSBit (one byte)>
 * <Data bytes: LSByte to MSByte, MSBit in each byte first>
 *
 * */


#ifndef INC_PORT_H_
#define INC_PORT_H_

#include "main.h"


/* Exported macro ------------------------------------------------------------*/

/* Direcciones de los registros del NRF24L01 */
#define PORT_NRF24L01_REG_CONFIG		0x00	//Configuration Register
#define PORT_NRF24L01_REG_EN_AA			0x01	//Enable ‘Auto Acknowledgment’ Function
#define PORT_NRF24L01_REG_EN_RXADDR		0x02	//Enabled RX Addresses
#define PORT_NRF24L01_REG_SETUP_AW		0x03	//Setup of Address Widths (common for all data pipes)
#define PORT_NRF24L01_REG_SETUP_RETR	0x04	//Setup of Automatic Retransmission
#define PORT_NRF24L01_REG_RF_CH			0x05	//RF Channel
#define PORT_NRF24L01_REG_RF_SETUP		0x06	//RF Setup Register
#define PORT_NRF24L01_REG_STATUS		0x07	//Status Register
#define PORT_NRF24L01_REG_OBSERVE_TX	0x08	//Read Only. Transmit observe register.
#define PORT_NRF24L01_REG_CD			0x09	//Read Only. Received Power Detector. CD: Carrier Detect en NRF24L01+
/* Registro para la direccion del TX */
#define PORT_NRF24L01_REG_TX_ADDR		0x10	//Transmit address. Used for a PTX device only
/* Registros para la direccion del seteo
 * del tamaño del payload de cada byte */
#define PORT_NRF24L01_REG_RX_PW_P0		0x11	//Number of bytes in RX payload in data pipe 0 (1 to 32 bytes).
#define PORT_NRF24L01_REG_RX_PW_P1		0x12	//Number of bytes in RX payload in data pipe 1 (1 to 32 bytes).
#define PORT_NRF24L01_REG_RX_PW_P2		0x13	//Number of bytes in RX payload in data pipe 2 (1 to 32 bytes).
#define PORT_NRF24L01_REG_RX_PW_P3		0x14	//Number of bytes in RX payload in data pipe 3 (1 to 32 bytes).
#define PORT_NRF24L01_REG_RX_PW_P4		0x15	//Number of bytes in RX payload in data pipe 4 (1 to 32 bytes).
#define PORT_NRF24L01_REG_RX_PW_P5		0x16	//Number of bytes in RX payload in data pipe 5 (1 to 32 bytes).
/* Registro saber el estado del FIFO */
#define PORT_NRF24L01_REG_FIFO_STATUS	0x17	//FIFO Status Register

/* Valores iniciales por default de los registros, util para hacer un reseteo
 * a los valores por defecto */

#define PORT_NRF24L01_REG_DEFAULT_VAL_EN_AA			0x3F	// Enable Auto ACK
#define PORT_NRF24L01_REG_DEFAULT_VAL_SETUP_AW		0x03	// Address width 5 Bytes
#define PORT_NRF24L01_REG_DEFAULT_VAL_RF_CH			0x02	// Channel 2
#define PORT_NRF24L01_REG_DEFAULT_VAL_RF_SETUP		0x0F	// 2 Mbps & 0dBm & Setup LNA gain
#define PORT_NRF24L01_REG_DEFAULT_VAL_STATUS		0x0E	// RX FIFO Empty & no interrupt detected

/* Direccion del Pipe 2-5 por default solo el LSB */
#define PORT_NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P2		0xC3
#define PORT_NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P3		0xC4
#define PORT_NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P4		0xC5
#define PORT_NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P5		0xC6



/* Configuracion de los registros
 *
 * Mascaras utilizadas para modificar solo un bit de ciertos
 * registros.
 * */

/* Posiciones de los bits del registro de configuracion */
#define PORT_NRF24L01_MASK_RX_DR	6
#define PORT_NRF24L01_MASK_TX_DS	5
#define PORT_NRF24L01_MASK_MAX_RT	4
#define PORT_NRF24L01_EN_CRC		3
#define PORT_NRF24L01_CRCO			2
#define PORT_NRF24L01_PWR_UP		1
#define PORT_NRF24L01_PRIM_RX		0

/* Setear el intervalo de tiempo y la cantidad de intentos de retransmision */
#define PORT_NRF24L01_ARD			4 //4 bits superiores del Byte
#define PORT_NRF24L01_ARC			0 //4 bits inferiores del Byte

/* Seteo del registro de RF */
//#define NRF24L01_PLL_LOCK		4 // bit 4
#define PORT_NRF24L01_RF_DR			3 // bit 3
#define PORT_NRF24L01_RF_PWR		1 // bits 1, 2
#define PORT_NRF24L01_LNA_HCURR		0 // bit 0

/* Registro del status del NRF24L01 */
#define PORT_NRF24L01_RX_DR			6 // Data Ready RX FIFO interrupt. Set high when new data arrives RX FIFO15. Write 1 to clear bit.
#define PORT_NRF24L01_TX_DS			5 // Data Sent TX FIFO interrupt. Set high when packet sent on TX....
#define PORT_NRF24L01_MAX_RT		4 // Maximum number of TX retries interrupt.....
#define PORT_NRF24L01_RX_P_NO		1 //3 bits
#define PORT_NRF24L01_TX_FULL		0

/* Registro de observacion de transmision */
#define PORT_NRF24L01_PLOS_CNT		4 //4 bits superiores del Byte

/* bit para detectar si ay portadora en algun canal */
#define PORT_NRF24L01_CD			0 // LSb

/* FIFO status*/
#define PORT_NRF24L01_TX_REUSE		6
#define PORT_NRF24L01_TXFIFO_FULL	5
#define PORT_NRF24L01_TX_EMPTY		4
								// bit 2 y 3 reservado
#define PORT_NRF24L01_RX_FULL		1
#define PORT_NRF24L01_RX_EMPTY		0

/* Nombre de instrucciones */
//#define NRF24L01_REUSE_TX_PL_MASK	0xE3



/**
 * Estados de transmision
 */
typedef enum _NRF24L01_Tx_Status_t {
	NRF24L01_Tx_Status_Lost = 0x00,   /* Transmision fallida */
	NRF24L01_Tx_Status_Ok = 0x01,     /*Mensaje enviado exitosamente */
	NRF24L01_Tx_Status_Sending = 0xFF /* Transmision en proceso */
} NRF24L01_Tx_Status_t;



/******************************************************************************/
/*																			  */
/* Encabezados de las funciones para setear la comunicacion SPI del NRF24L01  */
/*																			  */
/******************************************************************************/

void vNRF24L01_SPI1_Init(void);

void vNRF24L01_SPI1_CE_CSN_Iniciar(void);

/******************************************************************************/
/*																			  */
/* 	Encabezados de las funciones para Leer/Escribir registros del NRF24L01    */
/*																			  */
/******************************************************************************/


/* Funcion utilizada para leer un registro de 8bits del modulo NRF24L01.
 *
 * Parametros:
 * 			registro: Es la direccion de algun registro del NRF24L01. 0x00 - 0x17
 *
 * Return: Devuelve el byte leido del registro indicado en el argumento de la funcion
 * */
uint8_t byNRF24L01_LeerRegistro(uint8_t registro);

/* Funcion utilizada para leer multiples registros de 8bits del modulo NRF24L01.
 *
 * Parametros:
 * 			registro: Es la direccion de algun registro del NRF24L01.
 * 			   		  Puede ser la direccion del Pipe 0, del pipe 1 o del TX.
 * 			   		  Todos de 5 bytes maximo.
 *
 *			* data: Puntero a vector de bytes donde guardara los bytes leidos.
 *			cuenta: Es la cantidad de bytes a leer.
 *
 * Return: No devuelve valores.
 * */
void vNRF24L01_LeerMultiRegistros(uint8_t registro, uint8_t* data, uint8_t cuenta);

/* Funcion utilizada para escribir un registro de 8bits del modulo NRF24L01.
 *
 * Parametros:
 * 				registro: Es la direccion del registro que quiero modificar del
 * 						  NRF24L01. Rango 0x00 - 0x17
 *
 * Return: No devuelve valores.
 * */
void vNRF24L01_EscribirRegistro(uint8_t registro, uint8_t byteDato);

/* Funcion utilizada para escribir multiples registros de 8bits del modulo NRF24L01.
 *
 * Parametros:
 * 			registro: Es la direccion de algun registro del NRF24L01.
 * 			   		  Puede ser la direccion del Pipe 0, del pipe 1 o del TX.
 * 			   		  Todos de 5 bytes maximo.
 *
 *			* data: Puntero a vector de bytes con los bytes que escribira.
 *			cuenta: Es la cantidad de bytes a escribir.
 *
 * Return: No devuelve valores.
 * */
void vNRF24L01_EscribirMultiRegistros(uint8_t registro, uint8_t *data, uint8_t cuenta);

/* Funcion utilizada para escribir el payload de 32Bytes max del modulo NRF24L01.
 *
 * Parametros:
 *			* data: Puntero a vector de bytes con los bytes que escribira.
 *			PayloadSize: Es la cantidad de bytes a escribir.
 *
 * Return: No devuelve valores.
 * */
void vNRF24L01_EscribirPayload(uint8_t *data, uint8_t PayloadSize);

/* Funcion utilizada para leer el payload de 32Bytes max del modulo NRF24L01.
 *
 * Parametros:
 *			* data: Puntero a vector de bytes para guardar los bytes que leera.
 *			PayloadSize: Es la cantidad de bytes a leer.
 *
 * Return: No devuelve valores.
 * */
void vNRF24L01_LeerPayload(uint8_t *data, uint8_t PayloadSize);

/* Funcion utilizada para borrar el FIFO del TX del modulo NRF24L01.
 *
 * Parametros: ninguno
 *
 * Return: No devuelve valores.
 * */
void vNRF24L01_FlushTx(void);

/* Funcion utilizada para borrar el FIFO del RX del modulo NRF24L01.
 *
 * Parametros: ninguno
 *
 * Return: No devuelve valores.
 * */
void vNRF24L01_FlushRx(void);

/* Funcion utilizada para leer el status (0x07) del modulo NRF24L01.
 *
 * 7: Reserved = 0 solamente permitido
 * 6: RX_DR:  Data Ready RX FIFO interrupt. Set high when new data arrives RX FIFO
 * 5: TX_DS:  Data Sent TX FIFO interrupt. Set high when packet sent on TX. If AUTO_ACK
			  is activated, this bit will be set high only when ACK is received.
 * 4: MAX_RT: Maximum number of TX retries interrupt. If MAX_RT is set it must be cleared
 * 			  to enable further communication.
 * 3 -1: RX_P_NO: Data pipe number for the payload available for reading from RX_FIFO
				  000-101: 110: Not Used , 111: RX FIFO Empty.
 * 0: TX_FULL: TX FIFO full flag. 1: TX FIFO full. 0:Available locations in TX FIFO.
 *
 * Parametros:
 *			* data: Puntero a vector de bytes para guardar los bytes que leera.
 *			PayloadSize: Es la cantidad de bytes a leer.
 *
 * Return: Devuelve el byte de status.
 * */
uint8_t byNRF24L01_GetStatus(void);

/* Funcion utilizada para escribir sobre un bit de un registro.
 *
 * Parametros:
			registro: Direccion del registro al que le quiero modificar un bit.
			bit:	  Posicion del bit a modificar: 0 -7
			setClear: 1: set bit, 0: clear bit
 *
 * Return: No devuelve valores.
 * */
void vNRF24L01_EscribirBit(uint8_t registro, uint8_t bit, uint8_t setClear);

/* Funcion utilizada para leer un bit de un registro.
 *
 * Parametros:
			registro: Direccion del registro al que le quiero modificar un bit.
			bit:	  Posicion del bit a leer: 0 -7
 *
 * Return: Devuelve 1 si el bit vale 1 o 0 caso contrario.
 * */
uint8_t bNRF24L01_LeerBit(uint8_t registro, uint8_t bit);

/*
 * */
void vNRF24L01_RxModo(void);

/*
 * Funcion utilizada para cambiar al modo TX. El NRF24L01 permanecera en este estado
 * hasta que termina de transmitir el paquete. Al terminar de transmitir vuelve al
 * modo Stby I: CE = 0 y bit PWR UP = 1, si el TX FIFO esta vacio. Si no, permanece
 * en TX hasta que lo este. Si el AACK no esta habilitado no dejar en modo TX por mas
 * de 4mSeg.
 *
 * Parametro:void
 *
 * Return: void.
 * */
void vNRF24L01_TxModo(void);

/* Standby-I se utiliza para minimizar el consumo de potencia manteniendo tiempos
 * cortos de encendido para TX o RX. El SPI puede utilizarse para leer o escribir
 * registros.
 *
 * Parametro:void
 *
 * Return: 1: Si pudo cambiar a Stby-I, 0: Si no pudo.
*/
uint8_t bNRF24L01_StbyModoIOn(void);

/* Se utiliza saber si el NRF24L01 esta en modo Stby-I.
 *
 * Parametro:void
 *
 * Return: 1: Si esta en Stby-I, 0: Si no.
*/
uint8_t bNRF24L01_IsStbyModoIOn(void);

/* Se utiliza para encender el NRF24L01.
 *
 * Parametro:void
 *
 * Return: 1: Si pudo cambiar a PowerUp, 0: Si no.
*/
uint8_t bNRF24L01_PowerUp(void);

/* Se utiliza para apagar el NRF24L01. Queda en modo bajo consumo. El dispositivo
 * no puede enviar ni recibir paquetes via RF, pero si mantiene los valores de los
 * registros y puede LEERSE los registros via SPI.
 *
 * Parametro:void
 *
 * Return: 1: Si pudo cambiar a PowerDown, 0: Si no.
*/
uint8_t bNRF24L01_PowerDown(void);

#endif /* INC_PORT_H_ */
