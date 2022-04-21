/* USER CODE BEGIN Header */



/* USER CODE END Header */

/* Includes ----------------------------------------------------------*/

#include <Port.h>
#include "stm32f4xx_nucleo_144.h"
#include "main.h"
/* Private typedef -----------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* Private define ------------------------------------------------------------*/

#define PORT_NRF24L01_ESPERA_SPI 15	// 5 mSeg

#define PORT_NRF24_CE_Pin GPIO_PIN_7
#define PORT_NRF24_CE_GPIO_Port GPIOA

#define PORT_NRF24_CSN_Pin GPIO_PIN_15
#define PORT_NRF24_CSN_GPIO_Port GPIOA

/* Seteo de pines - Con macros facilita la portabilidad */
#define PORT_NRF24L01_CE_LOW	 HAL_GPIO_WritePin(PORT_NRF24_CE_GPIO_Port, PORT_NRF24_CE_Pin, GPIO_PIN_RESET)
#define PORT_NRF24L01_CE_HIGH	 HAL_GPIO_WritePin(PORT_NRF24_CE_GPIO_Port, PORT_NRF24_CE_Pin, GPIO_PIN_SET)
#define PORT_NRF24L01_CE_IS_HIGH HAL_GPIO_ReadPin(PORT_NRF24_CE_GPIO_Port, PORT_NRF24_CE_Pin)

#define PORT_NRF24L01_CSN_HIGH 	HAL_GPIO_WritePin(PORT_NRF24_CSN_GPIO_Port, PORT_NRF24_CSN_Pin, GPIO_PIN_SET)
#define PORT_NRF24L01_CSN_LOW 	HAL_GPIO_WritePin(PORT_NRF24_CSN_GPIO_Port, PORT_NRF24_CSN_Pin, GPIO_PIN_RESET)

/* Defino una macro para la funcion Delay - Con macros facilita la portabilidad */
#define PORT_NRF24L01_DELAY_MS(delay) HAL_Delay(delay)


#define PORT_NRF24L01_WR_REGISTER_MASK		0x1F  // 000x xxxx escribo o leo un registro
#define PORT_NRF24L01_R_RX_PAYLOAD_MASK		0x61
#define PORT_NRF24L01_W_TX_PAYLOAD_MASK		0xA0
#define PORT_NRF24L01_FLUSH_TX_MASK			0xE1
#define PORT_NRF24L01_FLUSH_RX_MASK			0xE2

#define PORT_NRF24L01_NOP_MASK			0xFF // No Operation. Usada para leer el registro STATUS

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

void vNRF24L01_SPI1_Init(void);
void vNRF24L01_SPI1_CE_CSN_Iniciar(void);
void SPI_Error(void);

/* Generar comando de lectura o escritura */
void vNRF24L01_LeerComando(uint8_t *registro);
void vNRF24L01_EscribirComando(uint8_t *registro);

/* Leer y escribir registros */
uint8_t byNRF24L01_LeerRegistro(uint8_t registro);
void vNRF24L01_LeerMultiRegistros(uint8_t registro, uint8_t* vBytes, uint8_t cantidad);
void vNRF24L01_EscribirRegistro(uint8_t registro, uint8_t byteDato);
void vNRF24L01_EscribirMultiRegistros(uint8_t registro, uint8_t *vBytes, uint8_t count);

/* Escribir el buffer de TX y leer el de RX */
void vNRF24L01_EscribirPayload(uint8_t *vBytes, uint8_t PayloadSize);
void vNRF24L01_LeerPayload(uint8_t *vBytes, uint8_t PayloadSize);
/* Vacia los buffer de TX y RX */
void vNRF24L01_FlushTx(void);
void vNRF24L01_FlushRx(void);

/* Estado del NRF24L01 */
uint8_t byNRF24L01_GetStatus(void);

/* Leer/escribir bit de un registro */
void vNRF24L01_EscribirBit(uint8_t registro, uint8_t bit, uint8_t setClear);
uint8_t bNRF24L01_LeerBit(uint8_t registro, uint8_t bit);

/* Modos de operacion - estados del NRF24L01 */
void vNRF24L01_RxModo(void);
void vNRF24L01_TxModo(void);
uint8_t bNRF24L01_StbyModoIOn(void);
uint8_t bNRF24L01_IsStbyModoIOn(void);
uint8_t bNRF24L01_PowerUp(void);
uint8_t bNRF24L01_PowerDown(void);

/* Private user code ---------------------------------------------------------*/


/******************************************************************************/
/*           SPI functions code 									          */
/******************************************************************************/

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void vNRF24L01_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}



void vNRF24L01_SPI1_CE_CSN_Iniciar(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PORT_NRF24_CE_GPIO_Port, PORT_NRF24_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PORT_NRF24_CSN_GPIO_Port, PORT_NRF24_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CE_NRF24_Pin CSN_NRF24_Pin */
  GPIO_InitStruct.Pin = PORT_NRF24_CE_Pin|PORT_NRF24_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

}


/**
  * SPI Error Function
  *
  *
  */
void SPI_Error(void){
	/* Deshabilito el dispositivo para el SPI */
	PORT_NRF24L01_CSN_HIGH;

	BSP_LED_Init(LED1);
	BSP_LED_On(LED1);
	while(1){
		HAL_Delay(250);
		BSP_LED_Off(LED1);
		HAL_Delay(250);
		BSP_LED_On(LED1);
	}
}

/******************************************************************************/
/*																			  */
/* 		Codigo de funciones para Leer/Escribir registros del NRF24L01    	  */
/*																			  */
/******************************************************************************/

/* Recibe un registro del NRF24L01: 0x00 al 0x17.
 * Modifica el bit 6 con un 0 porque leera el registro.
 *  */
void vNRF24L01_LeerComando(uint8_t *registro){
	/* 0x1F & *registro: R_REGISTER - > 000A AAAA donde ´A´ es un registro del NRF24L01 */
	*registro = PORT_NRF24L01_WR_REGISTER_MASK & *registro;
}

/* Recibe un registro del NRF24L01: 0x00 al 0x17.
 * Modifica el bit 6 con un 1 porque escribira en el registro.
 *  */
void vNRF24L01_EscribirComando(uint8_t *registro){
	/* 0x20 | 0x1F & *registro = W_REGISTER - > 001A AAAA donde ´A´ es un registro del NRF24L01 */
	*registro = 0x20 | (PORT_NRF24L01_WR_REGISTER_MASK & *registro);
}

/* Recibe un registro pasado por valor.
 * Devuelve el byte leido del registro pasado en el argumento de la funcion.
 * */
uint8_t byNRF24L01_LeerRegistro(uint8_t registro) {
	/* Un valor inicial de 0xFF puede servir como control de error de lectura
	 * La mayoria de los registros son < 0x80 */
	uint8_t byteLeido = 0xFF;
	/* Armo el comando de lectura para el registro pasado a la funcion */
	vNRF24L01_LeerComando(&registro);
	/* Chip select nivel bajo - Selecciono el dispositivo para el SPI*/
	PORT_NRF24L01_CSN_LOW;
	/* Enviar el comando leer el registro pedido */
	if(HAL_SPI_Transmit(&hspi1, &registro, sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) == HAL_OK)
	{
		// NRF24L01_NOP_MASK: 0xFF. No Operation. Might be used to read the STATUS register
		registro = PORT_NRF24L01_NOP_MASK;
		/* Leer el byte recibido */
		if(HAL_SPI_TransmitReceive(&hspi1, &registro, &byteLeido, sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) != HAL_OK){
			SPI_Error();
		}
	}
	else{
		SPI_Error();
	}
	/* Deshabilito el dispositivo para el SPI */
	PORT_NRF24L01_CSN_HIGH;

	return byteLeido;
}


void vNRF24L01_LeerMultiRegistros(uint8_t registro, uint8_t* vBytes, uint8_t cantidad) {
	/* Armo el comando de lectura para el registro pasado a la funcion */
	vNRF24L01_LeerComando(&registro);
	/* Chip select nivel bajo - Selecciono el dispositivo para el SPI*/
	PORT_NRF24L01_CSN_LOW;
	/* Enviar el comando leer el registro pedido */
	if(HAL_SPI_Transmit(&hspi1, &registro, sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) == HAL_OK)
	{
		// NRF24L01_NOP_MASK: 0xFF. No Operation.
		registro = PORT_NRF24L01_NOP_MASK;
		/* Leer los bytes recibidos */
		if(HAL_SPI_TransmitReceive(&hspi1, &registro, vBytes, cantidad * sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) != HAL_OK)
		{
			SPI_Error();
		}
	}
	else{
		SPI_Error();
	}
	/* Deshabilito el dispositivo para el SPI */
	PORT_NRF24L01_CSN_HIGH;
}


void vNRF24L01_EscribirRegistro(uint8_t registro, uint8_t byteDato) {
	/* Armo el comando de escritura en el registro pasado a la funcion */
	vNRF24L01_EscribirComando(&registro);
	/* Chip select nivel bajo - Selecciono el dispositivo para el SPI*/
	PORT_NRF24L01_CSN_LOW;
	/* Enviar comando de escritura */
	if(HAL_SPI_Transmit(&hspi1, &registro, sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) == HAL_OK)
	{
		/* Enviar byte al registro */
		if(HAL_SPI_Transmit(&hspi1, &byteDato, sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) != HAL_OK)
		{
			SPI_Error();
		}
	}
	else{
		SPI_Error();
	}
	/* Deshabilito el dispositivo para el SPI */
	PORT_NRF24L01_CSN_HIGH;

	for(int i = 0; i <100; i++);// Solo para Test....
	byNRF24L01_LeerRegistro(registro & PORT_NRF24L01_WR_REGISTER_MASK); // Solo para Test....

}


void vNRF24L01_EscribirMultiRegistros(uint8_t registro, uint8_t *vBytes, uint8_t count) {
	/* Armo el comando de escritura en el registro pasado a la funcion */
	vNRF24L01_EscribirComando(&registro);
	/* Chip select nivel bajo - Selecciono el dispositivo para el SPI*/
	PORT_NRF24L01_CSN_LOW;
	/* Enviar comando de escritura */
	if(HAL_SPI_Transmit(&hspi1, &registro, sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) == HAL_OK)
	{
		/* Enviar bytes para escribir */
		if(HAL_SPI_Transmit(&hspi1, vBytes, count*sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) != HAL_OK)
		{
			SPI_Error();
		}
	}
	else{
		SPI_Error();
	}
	/* Deshabilito el dispositivo para el SPI */
	PORT_NRF24L01_CSN_HIGH;
}



/******************************************************************************/
/*         Codigo para funciones que escribe o lee el payload - TX o RX       */
/*         y borra el FIFO de TX o de RX									  */
/******************************************************************************/

/* Escribe el payload - TX */
void vNRF24L01_EscribirPayload(uint8_t *vBytes, uint8_t PayloadSize){

	uint8_t byte_temp = PORT_NRF24L01_W_TX_PAYLOAD_MASK;
	/* Chip select nivel bajo - Selecciono el dispositivo para el SPI*/
	PORT_NRF24L01_CSN_LOW;
	/* Enviar el comando para escribir el payload de TX */
	if(HAL_SPI_Transmit(&hspi1, &byte_temp, sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) == HAL_OK)
	{
		/* Enviar los datos al payload*/
		if(HAL_SPI_Transmit(&hspi1, vBytes, PayloadSize*sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) != HAL_OK)
		{
			SPI_Error();
		}
	}
	else{
		SPI_Error();
	}
	/* Deshabilito el dispositivo para el SPI */
	PORT_NRF24L01_CSN_HIGH;
}

/* Lee el payload - RX */
void vNRF24L01_LeerPayload(uint8_t *vBytes, uint8_t PayloadSize){

	uint8_t byte_temp = PORT_NRF24L01_R_RX_PAYLOAD_MASK;
	/* Chip select nivel bajo - Selecciono el dispositivo para el SPI*/
	PORT_NRF24L01_CSN_LOW;
	/* Enviar el comando para leer el payload de RX */
	if(HAL_SPI_Transmit(&hspi1, &byte_temp, sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) == HAL_OK)
	{
		byte_temp = PORT_NRF24L01_NOP_MASK; // NRF24L01_NOP_MASK: No Operation. Might be used to read the STATUS register
		/* Leer el payload */
		if(HAL_SPI_TransmitReceive(&hspi1, &byte_temp, vBytes, PayloadSize * sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) != HAL_OK)
		{
			SPI_Error();
		}
	}
	else{
		SPI_Error();
	}
	/* Deshabilito el dispositivo para el SPI */
	PORT_NRF24L01_CSN_HIGH;
}

/* Borra el FIFO de TX - 3 niveles */
void vNRF24L01_FlushTx(void){

	uint8_t byte_comando = PORT_NRF24L01_FLUSH_TX_MASK;
	/* Chip select nivel bajo - Selecciono el dispositivo para el SPI*/
	PORT_NRF24L01_CSN_LOW;
	/* Envio el comando para que borre el FIFO de TX */
	if(HAL_SPI_Transmit(&hspi1, &byte_comando, sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) != HAL_OK)
	{
		SPI_Error();
	}
	/* Deshabilito el dispositivo para el SPI */
	PORT_NRF24L01_CSN_HIGH;
}

/* Borra el FIFO de RX */
void vNRF24L01_FlushRx(void){
	uint8_t byte_comando = PORT_NRF24L01_FLUSH_RX_MASK;
	/* Chip select nivel bajo - Selecciono el dispositivo para el SPI*/
	PORT_NRF24L01_CSN_LOW;
	/* Envio el comando para que borre el FIFO de RX */
	if(HAL_SPI_Transmit(&hspi1, &byte_comando, sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) != HAL_OK)
	{
		SPI_Error();
	}
	/* Deshabilito el dispositivo para el SPI */
	PORT_NRF24L01_CSN_HIGH;
}


uint8_t byNRF24L01_GetStatus(void) {

	uint8_t status, byte_NOP = PORT_NRF24L01_NOP_MASK;
	/* Chip select nivel bajo - Selecciono el dispositivo para el SPI*/
	PORT_NRF24L01_CSN_LOW;
	/* El primer byte recibido es el de status (0x07) */
	if(HAL_SPI_TransmitReceive(&hspi1, &byte_NOP, &status, sizeof(uint8_t), PORT_NRF24L01_ESPERA_SPI) != HAL_OK)
	{
		SPI_Error();
	}
	/* Deshabilito el dispositivo para el SPI */
	PORT_NRF24L01_CSN_HIGH;

	return status;
}

/******************************************************************************/
/*         Codigo para funciones que modifican o leen un bit                  */
/******************************************************************************/

void vNRF24L01_EscribirBit(uint8_t registro, uint8_t bit, uint8_t setClear) {

	/* Leer el registro para modificar algun bit */
	 uint8_t byte = byNRF24L01_LeerRegistro(registro);
	/* Operacion */
	if (setClear) {
		byte |= 1 << bit;
	} else {
		byte &= ~(1 << bit);
	}
	/* Escribir el byte modificado */
	vNRF24L01_EscribirRegistro(registro, byte);
}


uint8_t bNRF24L01_LeerBit(uint8_t registro, uint8_t bit) {
	/* Leo el registro, y lo multiplico por un byte con un 1 en la posicion que quiero
	 * y el resultado lo vuelvo bit posiciones hacia el bit menos significativo */
	return (byNRF24L01_LeerRegistro(registro) & (1 << bit)) >> bit;
}



/******************************************************************************/
/*         Codigo para funciones que setean el estado del NRF24L01            */
/******************************************************************************/

/* Cambia al modo RX */
void vNRF24L01_RxModo(void) {

	/* Borro el buffer de RX */
	vNRF24L01_FlushRx();
	/* Seteo modo RX */
	vNRF24L01_EscribirBit(PORT_NRF24L01_REG_CONFIG, PORT_NRF24L01_PRIM_RX, 1);
	/* Comenzar a "escuchar" */
	PORT_NRF24L01_CE_HIGH;
	// Tiempo de establecimiento minimo 130uSeg
	PORT_NRF24L01_DELAY_MS(1);
}

/* Cambia al modo TX */
void vNRF24L01_TxModo(void) {

	/* Setear el bit de TX en el registro de configuracion del NRF24L01 */
	vNRF24L01_EscribirBit(PORT_NRF24L01_REG_CONFIG, PORT_NRF24L01_PRIM_RX, 0);
	/* CE_HIGH para el modo TX - Transmitira si el FIFO de TX no esta vacio */
	PORT_NRF24L01_CE_HIGH;
	// CE = 1 por mas de 10µs
	PORT_NRF24L01_DELAY_MS(1);
}

/* Cambia al modo Stby - I */
uint8_t bNRF24L01_StbyModoIOn(void){
	/* Para estar en modo StandBy debe estar encendido - Pregunto, si no esta
	 * prendido, lo enciendo */
	if(!bNRF24L01_LeerBit(PORT_NRF24L01_REG_CONFIG, PORT_NRF24L01_PWR_UP))
	{
		if(bNRF24L01_PowerUp())
		{
			return 1;
		}else
		{
			return 0;
		}
	}else
		{
			/* Si ya estaba encendido solo reseteo el pin Chip enable */
		PORT_NRF24L01_CE_LOW;
			return 1;
		}
}

/* Chequea si esta en el modo Stby - I */
uint8_t bNRF24L01_IsStbyModoIOn(void){
	/* Leo el pin CE - CE = 0 esta en Stby*/
	return !PORT_NRF24L01_CE_IS_HIGH;
}

/* Cambia al modo Stby - I desde PowerDown */
uint8_t bNRF24L01_PowerUp(void){
	/* Me aseguro que CE esta en nivel bajo */
	PORT_NRF24L01_CE_LOW;
	/* Seteo el bit de encendido/apagado */
	vNRF24L01_EscribirBit(PORT_NRF24L01_REG_CONFIG, PORT_NRF24L01_PWR_UP, 1);
	/* Tiempo de encendido 1.5ms */
	PORT_NRF24L01_DELAY_MS(2);
	/* Chequeo que la escritura fue exitosa */
	if(bNRF24L01_LeerBit( PORT_NRF24L01_REG_CONFIG, PORT_NRF24L01_PWR_UP) )
	{
		/* Lo encendio correctamente */
		return 1;
	}else
	{
		/* Asumo que la escritura fue erronea */
		return 0;
	}
}

/* Cambia al modo PowerDown */
uint8_t bNRF24L01_PowerDown(void){
	/* Reseteo el bit de encendido/apagado */
	vNRF24L01_EscribirBit(PORT_NRF24L01_REG_CONFIG, PORT_NRF24L01_PWR_UP, 0);
	/* Chequeo que la escritura fue exitosa */
	if(!bNRF24L01_LeerBit( PORT_NRF24L01_REG_CONFIG, PORT_NRF24L01_PWR_UP) )
	{
		/* Lo apago correctamente */
		return 1;
	}else
	{
		/* Asumo que la escritura fue erronea */
		return 0;
	}
}
