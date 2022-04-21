/*
 * Archivo: nrf24.h
 * Autor: Jorge Gabriel Schmidt
 *
 * Introducción:
 *
 * Este archivo utiliza las funciones de Port.h para funciones de más alto nivel.
 *
 * Para utilizar el módulo NRF24L01 hay que incluir el archivo nrf24.h en el lugar donde
 * se utilizará alguna función del módulo en cuestión.
 *
 * Primero se debe invocar a la función: void vNRF24L01_SPI1_Iniciar(void);
 *
 * Luego, el módulo estará con todas las configuraciones por default, según la hoja de
 * datos. Si el usuario desea cambiar alguna deberá utilizar las funciones para configurar
 * el módulo.
 *
 * Para utilizar el módulo, deberá usar las funciones para operar con él, o para determinar
 * el estado de algún parámetro.
 *
 * */
#ifndef INC_NRF24_H_
#define INC_NRF24_H_


#include <Port.h>
#include "main.h"


/**
 * Macros para el data rate. Lo utiliza la función:
 *
 * 	bNRF24L01_SetRF(NRF24L01_DataRate_t DataRate, NRF24L01_Potencia_t Potencia)
 */
typedef enum _NRF24L01_DataRate_t {
	NRF24L01_DataRate_2M = 0x00, /* 2 Mbps */
	NRF24L01_DataRate_1M,		 /* 1 Mbps */
} NRF24L01_DataRate_t;

/**
 * Macros para la potencia de salida.  Lo utiliza la función:
 *
 * 	bNRF24L01_SetRF(NRF24L01_DataRate_t DataRate, NRF24L01_Potencia_t Potencia)
 */
typedef enum _NRF24L01_Potencia_t {
	NRF24L01_Potencia_m18dBm = 0x00,/* Potencia de -18dBm */
	NRF24L01_Potencia_M12dBm,       /* Potencia de -12dBm */
	NRF24L01_Potencia_M6dBm,        /* Potencia de -6dBm */
	NRF24L01_Potencia_0dBm          /* Potencia de  0dBm */
} NRF24L01_Potencia_t;


/**
 * Macros para elegir un canal de recepción para setearle diferentes caracteristicas.
 */
typedef enum _Pipe_Number_t {
	Pipe0 = 0,
	Pipe1 = 1,
	Pipe2 = 2,
	Pipe3 = 3,
	Pipe4 = 4,
	Pipe5 = 5,
	PipeAll = 6, /* Utilizado para los registros en que se pueden re/setear todos a la vez */
} Pipe_Number_t;



/******************************************************************************/
/*          Encabezado de las funciones  							          */
/******************************************************************************/
/**
 * Inicializa el puerto SPI 1, el pin de CE y el de CSN. Esta función se debe llamar
 * antes de utilizar cualquier otra funcion relativa al NRF24L01.
 * parametros: ninguno
 * retval ninguno
 */
void vNRF24L01_Iniciar(void);

/**
 * Setea el tamaño del CRC.
 * parametros: enDis: '1' - 1 byte , '2' – 2 bytes
 * retval ninguno
 */
void vNRF24L01_CRCScheme(uint8_t enDis);

/**
 * Inicializa el modulo NRF24L01, resetea los registros a los valores iniciales segun
 * la hoja de datos.
 * parametros: ninguno
 * retval 1 si la escritura de los registros fue exitosa, o 0 si no lo fue
 */
uint8_t bNRF24L01_DefaultIniciar(void);

/**
 * Setea el canal de trabajo: F = 2400MHz + CanalDeRF
 * @note   Channel value is just an offset in units MHz from 2.4GHz
 *         For example, if you select channel 65, then operation frequency will be set to 2.465GHz.
 * param  CanalDeRF: Canal de RF
 * retval 1: si el seteo del canal fue exitoso, 0: si no lo fue.
 */
uint8_t bNRF24L01_SetCanalRF(uint8_t CanalDeRF);

/**
 * Setea los parametros de RF para el NRF24L01
 * parametro  DataRate: 1Mbps o 2Mbps
 * parametro  Potencia: NRF24L01_Potencia_m18dBm a NRF24L01_Potencia_0dBm
 * return 1: Si el ingreso de datos y la escritura del registro fue correcta.
 * return 0: Caso contrario.
 */
uint8_t bNRF24L01_SetRF(NRF24L01_DataRate_t DataRate, NRF24L01_Potencia_t Potencia);

/**
 *	Setea el tiempo entre intentos de retransmision y la cantidad, en caso de no recibir el
 *	ACK del RX.
 *	Parametros:
 *			tiempoEntreIntentos: 0 - 15. Siendo 0 = 250+86uS
 *			maxCantIntentos:	 0 - 15. Siendo 0 retransmision deshabilitada
 *	return: 1 si los valores ingresados y la escritura del correspondiente registro fue valida.
 *			0 si los valores ingresados estan fuera de rango, o la escritura del registro fue erronea.
 */
uint8_t bNRF24L01_SetRetransmisiones(uint16_t tiempoEntreIntentos, uint16_t maxCantIntentos);

/**
 *	Setea el tamaño de las direcciones, comun para TX y todos los pipes de RX: 3 - 5 bytes.
 *
 *	Parametros:
 *			CantidadBytes: 3 - 5
 *	return: 1: si los valores ingresados son correctos.
 *			0: si los valores ingesados estan fuera de rango.
 */
uint8_t bNRF24L01_SetupTamañoDirecciones(uint8_t CantidadBytes);

/**
 *	Habilita un canal para recibir datos.
 *
 *	Parametros:
 *			pipe:del pipe0 al pipe5 o pipeall para habilitar todos
 *			en_dis: 1 = habilita, 0 = deshabilita.
 *	return void
 */
void NRF24L01_HabilitarRxPipeN(Pipe_Number_t pipe, uint8_t en_dis);

/**
 *	Habilita el autoACK para un canal o para todos.
 *
 *	Parametros:
 *			pipe:del pipe0 al pipe5 o pipeall para habilitar todos
 *			en_dis: 1 = habilita, 0 = deshabilita.
 *	return void
 */
void NRF24L01_HabilitarAACKPN(Pipe_Number_t pipe, uint8_t en_dis);

/**
 *	Setea la direccion de un canal de recepcion, esta sera la misma que el canal
 *	de TX del que recibira.
 *
 *	Parametros:
 *			pipe:del pipe0 al pipe5
 *			direccion: Direccion de 3 - 5 Bytes, segun el acho de la direccion.
 *	return void
 */
uint8_t bNRF24L01_SetDireccionPipeRx(Pipe_Number_t pipe, uint8_t *direccion);

/**
 *	Setea el ancho del payload de cada pipe de RX, 0 canal no utilizado, 1 = 1Byte
 *	...32 = 32 bytes.
 *
 *	Parametros:
 *			pipe:del pipe0 al pipe5
 *			payload: 0 - 32.
 *	return void
 */
void NRF24L01_SetRxPipePayloadWidth(Pipe_Number_t pipe, uint8_t payload);

/**
 *	Setea la direccion de transmision - a que canal de recepcion transmitira.
 *	Si el AutoACK esta habilitado, el pipe0 debe tener la misma direccion.
 *	Esta funcion lo hace automaticamente ese seteo.
 *
 *	Parametros:
 *			direccion: direccion de 3 - 5 bytes de tamaño
 *	return void
 */
void NRF24L01_SetDireccionTx(uint8_t *direccion);

/**
 *	Se fija si el FIFO de TX esta vacio.
 *
 *	Parametros:
 *			ninguno.
 *	return: 1 si esta vacio, 0 caso contrario.
 */
uint8_t NRF24L01_TXFIFOEmpty(void);

/**
 *	Transmite los datos pasados por el argumento. Transmitira en modo 1, esto es:
 *	pasara de Standby-1 a TX mode, transmitira lo que hay en el TXFIFO, y luego
 *	volvera a Standby-1, cambiando CE = 0.
 *
 *	Parametros:
 *			data: Datos a transmitirse, el tamaño maximo es el payload size.
 *			cantidadBytes: Cantidad de bytes a transmitir, 1 - 32 max.
 *	return: 1: si pudo transmitir, esto es si recibio el ACK en caso de que este
 *			configurado.
 *			0: Si no recibio el AACK, o el dato pasado es NULL, o no pudo pasar al
 *			estado Standby-1
 */
uint8_t bNRF24L01_TXModoI(uint8_t *data, uint8_t cantidadBytes);

/**
 *	Lee los bits 4 (MAX_RT) y 5 (TX_DS) del registro STATUS (0x07)
 *
 *	Parametros:
 *			void.
 *	return:
 *		NRF24L01_Transmit_Status_Ok: Termino de enviar el mensaje, esto sera asi
 *									 si recibio el AACK del receptor, si esta funcion
 *									 esta habilitada.
 *
 *		NRF24L01_Transmit_Status_Lost: Si se cumplio la cantidad de retransmisiones
 *									   y no recibio el AACK, se entiende que el mensaje
 *									   no llego.
 *
 *		NRF24L01_Transmit_Status_Sending: Si el bit 4 y 5 se mantienen en 0, el NRF24L01
 *										  esta en proceso de transmision.
 */
NRF24L01_Tx_Status_t NRF24L01_GetTransmissionStatus(void);

/**
 *	Detecta si hay una señal de portadora en el canal seteado en:
 *		bNRF24L01_SetCanalRF(uint8_t CanalDeRF);
 *
 *	Parametros:
 *			void.
 *	return: 1: si detecta una portadora, 0 caso contrario.
 */
uint8_t bNRF24L01_DetectarPortadora(void);

/**
 *	Cuenta las veces que tuvo que reenviar un paquete hasta recibir el AACK.
 *	Este valor se resetea al comenzar una nueva transmision.
 *
 *	Parametros:
 *			void.
 *	return: 0-15: cantidad de reenvios.
 */
uint8_t byNRF24L01_GetRetransmissionsCount(void);

/**
 *	Borra los flags del registro STATUS (0x07): RX_DR, TX_DS, MAX_RT. Se refieren a
 *	interrupcion de: recepcion de datos, Envio de datos terminado, maxima cantidad de
 *	intentos de envio alcanzada.
 *
 *	Parametros:
 *			void.
 *	return:
 *			void.
 */
void vNRF24L01_BorrarFlagsInterrupciones(void);

#endif

