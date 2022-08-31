



#include "CDD_L9963_LLD.h"

extern SPI_HandleTypeDef hspi2;
void GCDD_L9963_TxData(uint8_t *txframe,uint8_t *rxframe, uint32_t size)
{
	
	HAL_SPI_TransmitReceive( &hspi2, txframe, rxframe, size,  100);

//	HAL_SPI_TransmitReceive( hspi2, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,  uint32_t Timeout)
	
	///HAL_SPI_Transmit(&hspi2,&t,1,100);
	
	//Spi_SetupEB(SpiConf_SpiChannel_SpiChannel_2_DCh2,txframe,rxframe,size);
	//Spi_SyncTransmit(SpiConf_SpiSequence_SpiSequence_2_DCh2);
}
