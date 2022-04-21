/**
 *
 *
 *
 *
 *
 *
 *
 */
/* Private includes ----------------------------------------------------------*/

#include "nrf24.h"
//#include "stm32f4xx_nucleo_144.h"

/* Private define ------------------------------------------------------------*/
#define NRF24L01_MAX_DIR_SIZE 5
#define NRF24L01_MIN_DIR_SIZE 3

#define NRF24L01_MAX_RETRANSMISIONES 15
#define NRF24L01_MAX_PAYLOAD_SIZE_BY 32

#define NRF24L01_MAX_CANT_CANALES 125

/* Private typedef -----------------------------------------------------------*/

/* Esta estructura se utilizara en futuras funciones que devuelva algunos parametros
 * del NRF24L01.....
 *
 * */
typedef struct {

	/* Comun a todos los pipes */
	uint8_t TamañoDeDireccion;		// Tamaño de la direcciones de TX/RXs
	uint8_t CanalDeRF;				// Canal de RF seleccionado: 2400MHz + CanalDeRF
	NRF24L01_Potencia_t Potencia;	// Potencia de TX
	NRF24L01_DataRate_t DataRate;	// Data rate
	uint8_t CRCScheme;				// CRC encoding scheme
	/* Caracteristicas particulares de cada registro */
	uint8_t DireccionTx[NRF24L01_MAX_DIR_SIZE];			// Direccion del canal de transmision
	uint8_t DireccionPipe0[NRF24L01_MAX_DIR_SIZE];		// Direccion del canal de de recepcion 0
	uint8_t DireccionPipe1[NRF24L01_MAX_DIR_SIZE];		// Direccion del canal de de recepcion 1
	uint8_t DireccionPipes2_5[4];						// Ultimo byte de la direccion de los canales de recepcion 2-5
	uint8_t TamañoPayload[6];							// Tamaño del Payload del pipe 0 al 5

} NRF24L01_t;


/**
 * Tipo enumerativo para ser utilizado en funciones que operan sobre los registros referentes a los
 * canales de recepcion.
 */
typedef enum _Pipe_Number_Dir_t {
	Pipe0Dir = 0x0A,
	Pipe1Dir = 0x0B,
	Pipe2Dir = 0x0C,
	Pipe3Dir = 0x0D,
	Pipe4Dir = 0x0E,
	Pipe5Dir = 0x0F,
} Pipe_Number_Dir_t;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Estructura del NRF24L01 - Contiene como esta configurado */

static NRF24L01_t NRF24L01_Struct;

const Pipe_Number_Dir_t PipeDir[] = {Pipe0Dir, Pipe1Dir, Pipe2Dir, Pipe3Dir, Pipe4Dir, Pipe5Dir};

/* Private function prototypes -----------------------------------------------*/

/* Funciones de inicio del SPI1 */

void vNRF24L01_SPI1_Iniciar(void);

/* Funciones para configurar el NRF24L01 */
void vNRF24L01_CRCScheme(uint8_t enDis);

uint8_t bNRF24L01_DefaultIniciar(void);

uint8_t bNRF24L01_SetCanalRF(uint8_t channel);
uint8_t bNRF24L01_SetRF(NRF24L01_DataRate_t DataRate, NRF24L01_Potencia_t OutPwr);
uint8_t bNRF24L01_SetRetransmisiones(uint16_t tiempoEntreIntentos, uint16_t maxCantIntentos);
uint8_t bNRF24L01_SetupTamañoDirecciones(uint8_t CantidadBytes);

void NRF24L01_HabilitarRxPipeN(Pipe_Number_t pipe, uint8_t en_dis);
void NRF24L01_HabilitarAACKPN(Pipe_Number_t pipe, uint8_t en_dis);
uint8_t bNRF24L01_SetDireccionPipeRx(Pipe_Number_t pipe, uint8_t *adr);
void NRF24L01_SetRxPipePayloadWidth(Pipe_Number_t pipe, uint8_t payload);

void NRF24L01_SetDireccionTx(uint8_t *adr);
uint8_t NRF24L01_TXFIFOEmpty(void);

/* Funciones operar con el NRF24L01 o determinar el estado de algun parametros del mismo */

uint8_t bNRF24L01_TXModoI(uint8_t *data, uint8_t cantidadBytes) ;
NRF24L01_Tx_Status_t NRF24L01_GetTransmissionStatus(void);
uint8_t bNRF24L01_DetectarPortadora(void);
uint8_t byNRF24L01_GetRetransmissionsCount(void);

/* Funciones para el seteo o manejo de interrupciones */

void vNRF24L01_BorrarFlagsInterrupciones(void);


void NRF24L01_falla(void);

/******************************************************************************/
/*           Codigo													          */
/******************************************************************************/
void vNRF24L01_CRCScheme(uint8_t enDis){
	/* enDis: ´0´- 1 byte , '1' – 2 bytes */
	vNRF24L01_EscribirBit(NRF24L01_REG_CONFIG, NRF24L01_CRCO, enDis);
	if(!enDis){
		NRF24L01_Struct.CRCScheme = 1;
	}else{
		NRF24L01_Struct.CRCScheme = 2;
	}
}


void vNRF24L01_SPI1_Iniciar(void){
	/* Inicio los pines de Chip enable y chip select not */
	vNRF24L01_SPI1_CE_CSN_Iniciar();
	/* Inicializo el puerto SPI 1 */
	vNRF24L01_SPI1_Init();
}


uint8_t bNRF24L01_DefaultIniciar(void){

	uint8_t pipe0[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7},
			pipe1[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};

	// Seteo el CRC a 1 byte
	vNRF24L01_CRCScheme(0);
	// Habilitar todos los pipes con AACK
	NRF24L01_HabilitarAACKPN(PipeAll, 1);
	// Habilitar el pipe0 para recibir
	NRF24L01_HabilitarRxPipeN(Pipe0, 1);
	// Habilitar el pipe1 para recibir
	NRF24L01_HabilitarRxPipeN(Pipe1, 1);
	// Direcciones para los canales de recepcion de 3 bytes
	bNRF24L01_SetupTamañoDirecciones(NRF24L01_REG_DEFAULT_VAL_SETUP_AW);
	/* Setear el tiempo de retransmision (250+86uS) y la cantidad de intentos (3) */
	bNRF24L01_SetRetransmisiones( 0,  3);
	/* Setear el canal de RF */
	bNRF24L01_SetCanalRF(NRF24L01_REG_DEFAULT_VAL_RF_CH); // Channel 2
	// Setup: 2 Mbps & 0dBm & LNA gain
	bNRF24L01_SetRF(NRF24L01_DataRate_2M, NRF24L01_Potencia_0dBm);
	//TX & P0
	NRF24L01_SetDireccionTx(pipe0);
	//P1
	bNRF24L01_SetDireccionPipeRx(Pipe1, pipe1);
	//P2
	bNRF24L01_SetDireccionPipeRx(Pipe2, (uint8_t *)NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P2);
	//P3
	bNRF24L01_SetDireccionPipeRx(Pipe3, (uint8_t *)NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P3);
	//P4
	bNRF24L01_SetDireccionPipeRx(Pipe4, (uint8_t *)NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P4);
	//P5
	bNRF24L01_SetDireccionPipeRx(Pipe5, (uint8_t *)NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P5);
	/* Setear ancho del Payload */
	NRF24L01_SetRxPipePayloadWidth(PipeAll, 0x00);
	/* Borrar Tx & Rx FIFOs */
	vNRF24L01_FlushTx();
	vNRF24L01_FlushRx();
	/* Borrar flags de interrupciones */
	vNRF24L01_BorrarFlagsInterrupciones();
	/* Return OK */
	return 1;
}


uint8_t bNRF24L01_SetCanalRF(uint8_t CanalDeRF) {
	/* Chequeo de errores de ingreso de valor */
	if(CanalDeRF >= 0 && CanalDeRF <= NRF24L01_MAX_CANT_CANALES && CanalDeRF != NRF24L01_Struct.CanalDeRF)
	{
		/* Guardar el nuevo valor del canal */
		NRF24L01_Struct.CanalDeRF = CanalDeRF;
		/* Escribir el valor del canal en el registro del NRF24L01 */
		vNRF24L01_EscribirRegistro(NRF24L01_REG_RF_CH, CanalDeRF);
		/* Chequeo que la escritura fue exitosa */
		if(byNRF24L01_LeerRegistro(NRF24L01_REG_RF_CH) != CanalDeRF )
		{
			/* Asumo que no pudo escribir el dato */
			return 0;
		}else{
				/* Escritura exitosa */
				return 1;
			}
	}else if(CanalDeRF == NRF24L01_Struct.CanalDeRF){
		/* Lo tomo como valido */
		return 1;
	}else{
			/* Ingreso de valor invalido */
			return 0;
		}
}


uint8_t bNRF24L01_SetRF(NRF24L01_DataRate_t DataRate, NRF24L01_Potencia_t Potencia) {
	/* Valido la entrada de potencia de RF */
	if( Potencia >= NRF24L01_Potencia_m18dBm && Potencia <= NRF24L01_Potencia_0dBm)
	{
		/* Valido la entrada de tasa de envio de datos */
		if( DataRate == NRF24L01_DataRate_1M || DataRate == NRF24L01_DataRate_2M )
		{
			uint8_t byteTemp = 0;

			if (DataRate == NRF24L01_DataRate_2M) {
				byteTemp |= 1 << NRF24L01_RF_DR;     // NRF24L01_RF_DR = 3; lo corro 3 lugares al bit
			}
			/* Seteo la potencia de transmision */
			if (Potencia == NRF24L01_Potencia_0dBm) {
				byteTemp |= 3 << NRF24L01_RF_PWR;            // NRF24L01_RF_PWR = 1
			} else if (Potencia == NRF24L01_Potencia_M6dBm) {
				byteTemp |= 2 << NRF24L01_RF_PWR;
			} else if (Potencia == NRF24L01_Potencia_M12dBm) {
				byteTemp |= 1 << NRF24L01_RF_PWR;
			}
			// el caso NRF24L01_OutputPower_M18dBm es 0X00
			/* Setear ganancia LNA */
			byteTemp |= 1 << NRF24L01_LNA_HCURR;
			/* Cargo en el registro los datos */
			vNRF24L01_EscribirRegistro(NRF24L01_REG_RF_SETUP, byteTemp);
			/* Verifico la carga del byte en el registro */
			if(byNRF24L01_LeerRegistro(NRF24L01_REG_RF_SETUP) == byteTemp )
			{
				/* Escritura valida */
				return 1;
			}else
				{
					/* Asumo que la escritura fue fallida */
					return 0;
				}
		}else
			{
				/* Datos de data rate ingresados invalidos */
				return 0;
			}
	}else
		{
			/* Datos de potencia ingresados invalidos */
			return 0;
		}
}


uint8_t bNRF24L01_SetRetransmisiones(uint16_t tiempoEntreIntentos, uint16_t maxCantIntentos){
	if(tiempoEntreIntentos >= 0 && tiempoEntreIntentos <= 15 &&
			maxCantIntentos >= 0 && maxCantIntentos <= NRF24L01_MAX_RETRANSMISIONES){
		vNRF24L01_EscribirRegistro(NRF24L01_REG_RF_SETUP, ( ( tiempoEntreIntentos << NRF24L01_ARD ) | maxCantIntentos) );
		return 1;
	}else{
		return 0;
	}
}


uint8_t bNRF24L01_SetupTamañoDirecciones(uint8_t CantidadBytes){
	if(CantidadBytes >= NRF24L01_MIN_DIR_SIZE && CantidadBytes <= NRF24L01_MAX_DIR_SIZE){
		NRF24L01_Struct.TamañoDeDireccion = CantidadBytes;
		vNRF24L01_EscribirRegistro(NRF24L01_REG_SETUP_AW, 	CantidadBytes - 2 );
		return 1;
	}else{
		/* Ingreso invalido de datos */
		return 0;
	}
}



void NRF24L01_HabilitarRxPipeN(Pipe_Number_t pipe, uint8_t en_dis){
	if(pipe >= Pipe0 && pipe <= Pipe5)
	{
		// Modifico el bit correspondiente en el registro
		vNRF24L01_EscribirBit(NRF24L01_REG_EN_RXADDR, pipe, en_dis);
		if(!en_dis){
			/* Si lo deshabilito seteo un payload size de 0 bytes */
			NRF24L01_SetRxPipePayloadWidth(pipe, 0);
		}
	}else if( pipe == PipeAll )
	{
		if(en_dis)
			vNRF24L01_EscribirRegistro(NRF24L01_REG_EN_RXADDR, 0x3F); // Habilito todos
		else
		{
			vNRF24L01_EscribirRegistro(NRF24L01_REG_EN_RXADDR, 0x00); // deshabilito todos
		}
	}
}


void NRF24L01_HabilitarAACKPN(Pipe_Number_t pipe, uint8_t en_dis){

	if(pipe >= Pipe0 && pipe <= Pipe5){
		/* Chequeo que el canal de recepción este habilitado */
		if(bNRF24L01_LeerBit(NRF24L01_REG_EN_RXADDR, pipe))
			vNRF24L01_EscribirBit(NRF24L01_REG_EN_AA, pipe, en_dis); // Modifico el Byte
	}else if( pipe == PipeAll ){
		if(en_dis)
			vNRF24L01_EscribirRegistro(NRF24L01_REG_EN_AA, 0x3F);// Habilito todos
		else
			vNRF24L01_EscribirRegistro(NRF24L01_REG_EN_AA, 0x00);// deshabilito todos
	}
}


uint8_t bNRF24L01_SetDireccionPipeRx(Pipe_Number_t pipe, uint8_t *direccion) {

	if(direccion && pipe >= Pipe0 && pipe <= Pipe5)
	{
		if( pipe == Pipe1 || pipe == Pipe0 ){
			vNRF24L01_EscribirMultiRegistros(PipeDir[pipe], direccion, NRF24L01_Struct.TamañoDeDireccion);
		}else{
				vNRF24L01_EscribirRegistro(PipeDir[pipe], direccion[0]);
				NRF24L01_Struct.DireccionPipes2_5[ pipe - 2 ] = direccion[0];
			}
	}else{
		return 0;
	}
	return 1;
}


void NRF24L01_SetRxPipePayloadWidth(Pipe_Number_t pipe, uint8_t payload){

	if(pipe >= Pipe0 && pipe <= PipeAll && payload >= 0 && payload <= NRF24L01_MAX_PAYLOAD_SIZE_BY)
	{
		if(pipe == PipeAll)
		{
			/* Escribo todos los payload de los pipes 0 - 5 de igual tamaño */
			for(uint8_t i = Pipe0; i < PipeAll; i++){
				vNRF24L01_EscribirRegistro(PipeDir[i], payload);
				NRF24L01_Struct.TamañoPayload[i] = payload;
			}
		}else{
			/* Seteo el payload de un solo pipe */
			vNRF24L01_EscribirRegistro(PipeDir[pipe], payload);
			NRF24L01_Struct.TamañoPayload[pipe] = payload;
		}
	}
}


void NRF24L01_SetDireccionTx(uint8_t *direccion) {

	vNRF24L01_EscribirMultiRegistros(NRF24L01_REG_TX_ADDR, direccion, NRF24L01_Struct.TamañoDeDireccion);   // Es la mismma direccion que el canal de recepcion - asi recibe el ACK POR P0

	for(uint8_t i = 0; i < NRF24L01_Struct.TamañoDeDireccion; i++)
		NRF24L01_Struct.DireccionTx[i] = direccion[i];
	/* Llena con 0 las posiciones del vector */
	if( NRF24L01_Struct.TamañoDeDireccion < NRF24L01_MAX_DIR_SIZE ){
		for(uint8_t i = NRF24L01_Struct.TamañoDeDireccion; i < NRF24L01_MAX_DIR_SIZE; i++)
			NRF24L01_Struct.DireccionTx[i] = 0x00;
	}

	/* chequear si el AutoACK esta habilitado para el Pipe0 */
	if(bNRF24L01_LeerBit(NRF24L01_REG_EN_AA, 0))
	{
		/* Si esta habilitado, cargo la misma direccion para el AACK */
		vNRF24L01_EscribirMultiRegistros(Pipe0Dir, direccion, NRF24L01_Struct.TamañoDeDireccion);
		/* Cargo la direccion del pipe0 en la estructura */
		for(uint8_t i = 0; i < NRF24L01_Struct.TamañoDeDireccion; i++)
			NRF24L01_Struct.DireccionPipe0[i] = direccion[i];
		/* Llena con 0 las posiciones del vector si la direccion mide menos de 5 bytes*/
		if( NRF24L01_Struct.TamañoDeDireccion < NRF24L01_MAX_DIR_SIZE ){
			for(uint8_t i = NRF24L01_Struct.TamañoDeDireccion; i < NRF24L01_MAX_DIR_SIZE; i++)
				NRF24L01_Struct.DireccionPipe0[i] = 0x00;
		}
	}
}


uint8_t NRF24L01_TXFIFOEmpty(void){
	return bNRF24L01_LeerBit(NRF24L01_REG_FIFO_STATUS, NRF24L01_TX_EMPTY);
}


uint8_t bNRF24L01_TXModoI(uint8_t *data, uint8_t cantidadBytes) {

	/* Borrar el FIFO de RX */
	vNRF24L01_FlushRx(); // Tengo que ver si el ACK de P0 se borra en cada nueva tx
	/* Chequeo que el dato no sea nulo */
	if(data && cantidadBytes > 0 && cantidadBytes <= NRF24L01_MAX_PAYLOAD_SIZE_BY){
		/* Borro los flags de interrupciones del registro Status */
		vNRF24L01_BorrarFlagsInterrupciones();
		/* Escribo el payload antes de ir al modo TX, si no ira al modo Standby II */
		vNRF24L01_EscribirPayload(data, cantidadBytes);
		/* Voy al modo StandBy - si esta apagado lo prende */
		if(!bNRF24L01_StbyModoIOn())
		{
			/* No pudo pasar del estado Off a stby */
			return 0;
		}
		/* Ir al modo TX - Transmitir */
		vNRF24L01_TxModo();
	}else{
		return 0;
	}
	/* ¡¡¡ Transmitiendo... !!! */

	/* Creo variable para saber el estado de la transmision */
	NRF24L01_Tx_Status_t TX_state;
	/* Pregunto si termino de transmitir chequeando un flag TX_DS del Status  */
	while( ( TX_state = NRF24L01_GetTransmissionStatus() ) == NRF24L01_Tx_Status_Sending);
	/* Vuelvo al modo Stand By I, para bajar el consumo una vez que transmitio (FIFO = 0)
	 * o que realizo todos los intentos por transmitir (Si AACK esta habilitado)*/
	bNRF24L01_StbyModoIOn();
	/* Chequeo el resultado de la transmision */
	if( TX_state == NRF24L01_Tx_Status_Ok)
	{
		return 1; // Transmision exitosa. TX_DS = 1
	}
	else
	{
		return 0; // Transmision fallida: No recibio el ACK luego de N intentos
	}
}


NRF24L01_Tx_Status_t NRF24L01_GetTransmissionStatus(void){
	uint8_t status = byNRF24L01_GetStatus();
	if (bNRF24L01_LeerBit(status, NRF24L01_TX_DS)) {
		/* Envio del dato exitoso */
		return NRF24L01_Tx_Status_Ok;
	} else if (bNRF24L01_LeerBit(status, NRF24L01_MAX_RT)) {
		/* No se recibio el AACK - se entiende como mensaje perdido */
		return NRF24L01_Tx_Status_Lost;
	}
	/* Enviando mensaje: Esto es si TX_DS y/o MAX_RT valen '0' */
	return NRF24L01_Tx_Status_Sending;
}


uint8_t bNRF24L01_DetectarPortadora(void){
	return bNRF24L01_LeerBit( byNRF24L01_LeerRegistro(NRF24L01_REG_CD), NRF24L01_CD);
}


uint8_t byNRF24L01_GetRetransmissionsCount(void){
	/* Low 4 bits */
	return byNRF24L01_LeerRegistro(NRF24L01_REG_OBSERVE_TX) & 0x0F;
}


void vNRF24L01_BorrarFlagsInterrupciones(void) {

	vNRF24L01_EscribirRegistro(NRF24L01_REG_STATUS , NRF24L01_REG_DEFAULT_VAL_STATUS);
}


/*

void NRF24L01_falla(void){

	BSP_LED_Init(LED2);
	BSP_LED_On(LED2);
	while(1){
		HAL_Delay(250);
		BSP_LED_Off(LED2);
		HAL_Delay(250);
		BSP_LED_On(LED2);
	}
}
*/
