#ifndef GCDD_L9963_DRV_INTERFACE_H_
#define GCDD_L9963_DRV_INTERFACE_H_
#include "CDD_L9963_drv.h"
#define G_L9963_DEVICE_NUMBER 1
#define G_L9963_CELL_CHANNEL  14
#define ERROR_WRITE				0

#define NULL_PTR ((void *)0)
#define NOP() asm("nop")
extern uint8_t ID[15];
extern uint8_t id[15];

//#include "Can.h"

/*Sample Voltage*/
extern uint32_t G_L9963_CellVoltage[G_L9963_DEVICE_NUMBER][G_L9963_CELL_CHANNEL];
extern uint32_t G_L9963_CellTemp[G_L9963_DEVICE_NUMBER][G_L9963_CELL_CHANNEL];
extern uint32_t G_L9963_ChipTemp[G_L9963_DEVICE_NUMBER];
extern void GCDD_L9963_Chip_Temp(void);
extern void GCDD_L9963_Task_Init(void);
extern void GCDD_L9963_Task_step(void);
extern uint8_t GCDD_L9963_CellVol_Burst_Read(void);
extern uint8 GCDD_L9963_GPIO_Measure_Burst_Read(void);
extern uint8_t GCDD_L9963_Chip_Temp_Read(void);
extern uint8_t GCDD_L9963_Cell_Balance_Set(uint8_t device_id, UART_BALANCE_CMD balance_bits);
extern void GCDD_L9963_ADC_Init(void);
extern void GCDD_L9963_SPI_start(void);
extern uint8_t GCDD_L9963_Configure_ID(uint8_t totalID, uint8_t* ID, uint8_t* ACK);
extern uint8_t GCDD_L9963_Single_Read(L9963_Addr_t addr, uint8_t device_id, L9963_SPI_Rx_Inst_t* spi_rx);
extern uint8_t GCDD_L9963_Single_Write(L9963_Addr_t addr, uint8_t device_id, uint32_t data, L9963_SPI_Rx_Inst_t* spi_rx);
extern uint8_t GCDD_L9963_Burst_Read(L9963_Addr_t addr, uint8_t device_id, uint8_t* spi_miso, BURST_SIZE burst_size);
//extern void GCDD_L9963_CAN_Transmit(void);
static void Delay(unsigned long cnt);

#endif /*GCDD_L9963_DRV_INTERFACE_H_*/
