Autor: 
	Jorge Gabriel Schmidt

Propósito:

	El driver desarrollado tiene la intención de brindar las funciones para poder configurar y utilizar el módulo NRF24L01. El mismo es un dispositivo utilizado 
para la comunicación vía RF (2.4 GHz + 125 canales). Se puede configurar como transmisor y como receptor, además de poder permanecer en modo de bajo consumo.

Para utilizar las funciones de operación y configuración del módulo se debe incluir el archivo nrf24.h.

Para adaptar el driver a otra plataforma se deberá modificar el archivo Port.c El cual maneja la configuración de la comunicación SPI y el manejo de los pines
CE y CSN.

Fue desarrollado para la placa STM32F429Zi y se comunica con la misma por medio del puerto SPI 1. Los pines utilizados en detalle son:

 * CE: 	 PA_7
 * CSN:  PA_15
 * MOSI: PB_5
 * MISO: PB_4
 * SCK:  PB_3
 * IRQ:  NO CONFIGURADO

No se desarrollaron las funciones para el manejo de interrupciones (Trabajo futuro...), por lo tanto, el modo de transmisión (Es el que esta implementado hasta el momento)
devuelve "1": si el envío fue exitoso o "0" si no lo fue (Una función que la utilice deberá preguntar por este resultado).

