#include "CDD_L9963_drv_interface.h"




void GCDD_L9963_Task_Init(void)
{
	L9963_SPI_start();
	L9963_Test_Init();
}

void GCDD_L9963_Task_step(void)
{
	GCDD_L9963_CellVol_Burst_Read();
	GCDD_L9963_Chip_Temp();
	GCDD_L9963_GPIO_Measure_Burst_Read();
//	GCDD_L9963_CAN_Transmit();
}

uint32_t G_L9963_CellVoltage[G_L9963_DEVICE_NUMBER][G_L9963_CELL_CHANNEL];
uint8_t GCDD_L9963_CellVol_Burst_Read(void)
{
	uint8_t vol;
	vol = L9963_Test_Voltage();
	uint8 i = 0;
	for(;i< G_L9963_DEVICE_NUMBER;i++)
	{
		G_L9963_CellVoltage[i][0] = L9963_Reg[*(ID+i)].VCELL1.B.VOLT;
		G_L9963_CellVoltage[i][1] = L9963_Reg[*(ID+i)].VCELL2.B.VOLT;
		G_L9963_CellVoltage[i][2] = L9963_Reg[*(ID+i)].VCELL3.B.VOLT;
		G_L9963_CellVoltage[i][3] = L9963_Reg[*(ID+i)].VCELL4.B.VOLT;
		G_L9963_CellVoltage[i][4] = L9963_Reg[*(ID+i)].VCELL5.B.VOLT;
		G_L9963_CellVoltage[i][5] = L9963_Reg[*(ID+i)].VCELL6.B.VOLT;
		G_L9963_CellVoltage[i][6] = L9963_Reg[*(ID+i)].VCELL7.B.VOLT;
		G_L9963_CellVoltage[i][7] = L9963_Reg[*(ID+i)].VCELL8.B.VOLT;
		G_L9963_CellVoltage[i][8] = L9963_Reg[*(ID+i)].VCELL9.B.VOLT;
		G_L9963_CellVoltage[i][9] = L9963_Reg[*(ID+i)].VCELL10.B.VOLT;
		G_L9963_CellVoltage[i][10] = L9963_Reg[*(ID+i)].VCELL11.B.VOLT;
		G_L9963_CellVoltage[i][11] = L9963_Reg[*(ID+i)].VCELL12.B.VOLT;
		G_L9963_CellVoltage[i][12] = L9963_Reg[*(ID+i)].VCELL13.B.VOLT;
		G_L9963_CellVoltage[i][13] = L9963_Reg[*(ID+i)].VCELL14.B.VOLT;
	}
	return vol;
}

uint32_t G_L9963_ChipTemp[G_L9963_DEVICE_NUMBER];
void GCDD_L9963_Chip_Temp(void)
{
	uint8 i = 0;
	for(i = 0 ; i< G_L9963_DEVICE_NUMBER ; i++ )
	{
		L9963_Single_Read_temp(TempChip_ADD,id[i],&spi_rx_t);
		L9963_Reg[id[i]].TempChip.R = spi_rx_t.B.DATA;
		G_L9963_ChipTemp[i] = L9963_Reg[*( ID + i)].TempChip.B.TempChip;
	}

}

uint32_t G_L9963_CellTemp[G_L9963_DEVICE_NUMBER][G_L9963_CELL_CHANNEL];
uint8 GCDD_L9963_GPIO_Measure_Burst_Read(void)
{
	uint8_t vol;
	vol = L9963_Test_Cell_Temp();
	uint8 i = 0;
	for(;i< G_L9963_DEVICE_NUMBER;i++)
	{
		G_L9963_CellTemp[i][0] = L9963_Reg[*ID+i].GPIO3_MEAS.B.GPIO3_MEAS_value;
		G_L9963_CellTemp[i][1] = L9963_Reg[*ID+i].GPIO4_MEAS.B.GPIO4_MEAS_value;
		G_L9963_CellTemp[i][2] = L9963_Reg[*ID+i].GPIO5_MEAS.B.GPIO5_MEAS_value;
		G_L9963_CellTemp[i][3] = L9963_Reg[*ID+i].GPIO6_MEAS.B.GPIO6_MEAS_value;
		G_L9963_CellTemp[i][4] = L9963_Reg[*ID+i].GPIO7_MEAS.B.GPIO7_MEAS_value;
		G_L9963_CellTemp[i][5] = L9963_Reg[*ID+i].GPIO8_MEAS.B.GPIO8_MEAS_value;
		G_L9963_CellTemp[i][6] = L9963_Reg[*ID+i].GPIO9_MEAS.B.GPIO9_MEAS_value;
	}

	return vol;
}

uint8_t GCDD_L9963_Cell_Balance_Set(uint8_t device_id, UART_BALANCE_CMD balance_bits)
{
	uint8_t return_code = true;
	/* Enable Cell function */
	L9963_Reg[device_id].VCELLS_EN.R = balance_bits.R;
	return_code &= L9963_Single_Write(VCELLS_EN_ADD, device_id,
		(uint32_t)L9963_Reg[device_id].VCELLS_EN.R, &spi_rx_t);
	/* Stop the Balancing, before configure the CELL*/
	L9963_Reg[device_id].Bal_1_t.B.bal_start = 0;
	L9963_Reg[device_id].Bal_1_t.B.bal_stop = 1;
	return_code &= L9963_Single_Write(BAL_1_ADD, device_id,
		(uint32_t)L9963_Reg[device_id].Bal_1_t.R, &spi_rx_t);
	L9963_Reg[device_id].BalCell14_7act_t.B.BAL14 =
		balance_bits.B.bal_14 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell14_7act_t.B.BAL13 =
		balance_bits.B.bal_13 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell14_7act_t.B.BAL12 =
		balance_bits.B.bal_12 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell14_7act_t.B.BAL11 =
		balance_bits.B.bal_11 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell14_7act_t.B.BAL10 =
		balance_bits.B.bal_10 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell14_7act_t.B.BAL9 =
		balance_bits.B.bal_9 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell14_7act_t.B.BAL8 =
		balance_bits.B.bal_8 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell14_7act_t.B.BAL7 =
		balance_bits.B.bal_7 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/

	return_code &= L9963_Single_Write(BalCell14_7act_ADD, device_id,
		(uint32_t)L9963_Reg[device_id].BalCell14_7act_t.R, &spi_rx_t);
	L9963_Reg[device_id].BalCell6_1act_t.B.BAL6 =
		balance_bits.B.bal_6 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell6_1act_t.B.BAL5 =
		balance_bits.B.bal_5 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell6_1act_t.B.BAL4 =
		balance_bits.B.bal_4 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell6_1act_t.B.BAL3 =
		balance_bits.B.bal_3 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell6_1act_t.B.BAL2 =
		balance_bits.B.bal_2 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	L9963_Reg[device_id].BalCell6_1act_t.B.BAL1 =
		balance_bits.B.bal_1 ? CELL_BALANCE_START : CELL_BALANCE_STOP; /* if the bits is 1, enable the balance, otherwise disable the balance*/
	return_code &= L9963_Single_Write(BalCell6_1act_ADD, device_id,
		(uint32_t)L9963_Reg[device_id].BalCell6_1act_t.R, &spi_rx_t);

	if (balance_bits.R != 0)
	{
		L9963_Reg[device_id].Bal_1_t.B.bal_start = 1; /* Start the Balancing*/
		L9963_Reg[device_id].Bal_1_t.B.bal_stop = 0;
	}
	else
	{
		L9963_Reg[device_id].Bal_1_t.B.bal_start = 0; /* Stop the Balancing*/
		L9963_Reg[device_id].Bal_1_t.B.bal_stop = 1;
	}
	return_code &= L9963_Single_Write(BAL_1_ADD, device_id,
		(uint32_t)L9963_Reg[device_id].Bal_1_t.R, &spi_rx_t);

	return return_code;
}

void GCDD_L9963_ADC_Init(void)
{
	L9963_ADC_Init();
}

void GCDD_L9963_SPI_start(void)
{
	L9963_SPI_start();
}

uint8_t GCDD_L9963_Burst_Read(L9963_Addr_t addr, uint8_t device_id, uint8_t* spi_miso, BURST_SIZE burst_size)
{
	uint8_t result;
	result = L9963_Burst_Read(addr, device_id, spi_miso, burst_size);
	return result;
}

uint8_t GCDD_L9963_Configure_ID(uint8_t totalID, uint8_t* ID, uint8_t* ACK)
{
	uint32_t temp_register;
	uint8_t i;
	uint8_t error_time = 0;
	uint8_t return_code = true;
	SET_L9963T_ISO_LOWSPEED;
	Delay(2000);

	for (i = 0; i < totalID; i++)
	{
		L9963_Reg[i + 1].DEV_GEN_CFG.R = 0;
		L9963_Reg[i + 1].DEV_GEN_CFG.B.chipid = *(ID + i); /* configure device ID */
#ifdef USE_L9963T
		L9963_Wakeup(*(ID + i), &spi_rx_t);
#else
		return_code &= L9963_Wakeup(*(ID + i), &spi_rx_t);/* Wakeup device*/
#endif

		Delay(6590);
		L9963_Reg[i + 1].DEV_GEN_CFG.R = 0;
		L9963_Reg[i + 1].DEV_GEN_CFG.B.chipid = *(ID + i); /* configure device ID */
#ifdef USE_L9963T
		L9963_Wakeup(*(ID + i), &spi_rx_t);
#else
		return_code &= L9963_Wakeup(*(ID + i), &spi_rx_t);/* Wakeup device*/
#endif

		Delay(6590);
		L9963_Reg[i + 1].DEV_GEN_CFG.B.isotx_en_h = 1;
		L9963_Reg[i + 1].DEV_GEN_CFG.B.iso_freq_sel = 0;
		L9963_Reg[i + 1].DEV_GEN_CFG.B.out_res_tx_iso = 3;
		temp_register = L9963_Reg[i + 1].DEV_GEN_CFG.B.chipid;
#ifdef USE_L9963T
		L9963_Single_Write(GEN_STATUS_ADD, BROADCAST_ID,
			(uint32_t)L9963_Reg[i + 1].DEV_GEN_CFG.R, &spi_rx_t);
#else
		return_code &= GCDD_L9963_Single_Write(GEN_STATUS_ADD, BROADCAST_ID,
			(uint32_t)L9963_Reg[i + 1].DEV_GEN_CFG.R, &spi_rx_t);
#endif
		Delay(2519); /*Total Delay time need to be 219us */

		L9963_Reg[i + 1].Bal_1_t.B.comm_timeout_dis = 1; /* Set the device always wake up*/
		L9963_Reg[BROADCAST_ID].Bal_1_t.B.comm_timeout_dis = 1; /* Set the device always wake up*/
#ifdef USE_L9963T
		L9963_Single_Write(BAL_1_ADD, *(ID + i),
			(uint32_t)L9963_Reg[i + 1].Bal_1_t.R, &spi_rx_t);
#else
		return_code &= L9963_Single_Write(BAL_1_ADD, *(ID + i),
			(uint32_t)L9963_Reg[i + 1].Bal_1_t.R, &spi_rx_t);
#endif
		Delay(2519);

		// Add Liyi 20200522
		/*
				L9963_Reg[i + 1].FASTCH_BALUV.B.CommTimOut = 1;
				L9963_Reg[BROADCAST_ID].FASTCH_BALUV.B.CommTimOut = 1;
		#ifdef USE_L9963T
				L9963_Single_Write(FASTCH_BAL_UV_ADD, *(ID + i),
						(uint32_t) L9963_Reg[i + 1].FASTCH_BALUV.R, &spi_rx_t);
		#else
				return_code &= L9963_Single_Write(FASTCH_BAL_UV_ADD, *(ID + i),
						(uint32_t) L9963_Reg[i + 1].FASTCH_BALUV.R, &spi_rx_t);
		#endif
				Delay(2519);
				*/
				//end add
		return_code &= GCDD_L9963_Single_Read(GEN_STATUS_ADD, *(ID + i), &spi_rx_t); /* check the return code */
		L9963_Reg[i + 1].DEV_GEN_CFG.R = spi_rx_t.B.DATA;
		Delay(2519); /*Total Delay time need to be 219us */

		if (temp_register == L9963_Reg[i + 1].DEV_GEN_CFG.B.chipid) /* Judge if configure successful*/
		{
			if (ACK != NULL_PTR)
				*(ACK + i) = 0x01;

		}
		else
		{

			error_time += 1;
			if (error_time > ERROR_WRITE)
			{
				error_time = 0;
				if (ACK != NULL_PTR)
					*(ACK + i) = 0x00;
				return_code = false;
			}
			else
			{
				i -= 1;
			}
		}
	}
	if (return_code == true)
	{
		L9963_Reg[BROADCAST_ID].DEV_GEN_CFG.B.isotx_en_h = 1;
		L9963_Reg[BROADCAST_ID].DEV_GEN_CFG.B.out_res_tx_iso = 3;
		L9963_Reg[BROADCAST_ID].DEV_GEN_CFG.B.iso_freq_sel = 3; /* Set the ISO frequency to High*/
		return_code &= GCDD_L9963_Single_Write(GEN_STATUS_ADD, BROADCAST_ID,
			(uint32_t)L9963_Reg[BROADCAST_ID].DEV_GEN_CFG.R, &spi_rx_t);
		//SET_L9963T_ISO_HIGHSPEED;
		Delay(2519);                               /* L9963T need a delay between ISO_SPEED_HIGH and sleep enable*/
		GCDD_L9963_Single_Read(GEN_STATUS_ADD, 1, &spi_rx_t); /* check the return code */
		L9963_Reg[1].DEV_GEN_CFG.R = spi_rx_t.B.DATA;
		SET_L9963T_ISO_HIGHSPEED;
		Delay(2519); /*Total Delay time need to be 219us */
//		SET_L9963T_SLEEP_ENABLE;
	}
	else
	{

	}
	Delay(6519);                                  /* L9963T disable enable need a delay*/
//	SET_L9963T_SLEEP_DISABLE;
	return return_code;
}

uint8_t GCDD_L9963_Single_Read(L9963_Addr_t addr, uint8_t device_id, L9963_SPI_Rx_Inst_t* spi_rx)
{
	L9963_SPI_Tx_Inst_t spi_tx_inst;
	uint8_t* spi_tx_data;
	uint8_t spi_mosi[5];
	/*1) package instruction*/
	spi_tx_inst.R = 0;
	spi_tx_inst.B.PA = 1;
	spi_tx_inst.B.RW = 0;
	spi_tx_inst.B.DEVID = device_id;
	spi_tx_inst.B.ADDRESS = addr;
	spi_tx_inst.B.GSW = 0;
	spi_tx_inst.B.DATA = 0;
	spi_tx_inst.B.CRC_1 = CrcCompute_L9963(spi_tx_inst.R);

	/*2) package data*/
	spi_tx_data = (uint8_t*)&spi_tx_inst;

	spi_mosi[0] = *(spi_tx_data + 3); /*first 24 bit is reseved*/
	spi_mosi[1] = *(spi_tx_data + 4);
	spi_mosi[2] = *(spi_tx_data + 5);
	spi_mosi[3] = *(spi_tx_data + 6);
	spi_mosi[4] = *(spi_tx_data + 7);
	/*3) exchange data over SPI bus*/
	L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(&spi_mosi[0],
		(((uint8_t*)spi_rx) + 3), 5);
	L9963_SPI_CS_UNSELECT;
	Delay(319); /*Total Delay time need to be 219us */
	L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(&spi_mosi[0],
		(((uint8_t*)spi_rx) + 3), 5);
	L9963_SPI_CS_UNSELECT;
	Delay(319); /*Total Delay time need to be 219us */
	/* Doing CRC verification*/
	if (spi_rx->B.CRC_1 == CrcCompute_L9963(spi_rx->R))
	{
		return true;
	}
	else
	{
		return false;
	}

}

uint8_t GCDD_L9963_Single_Write(L9963_Addr_t addr, uint8_t device_id, uint32_t data, L9963_SPI_Rx_Inst_t* spi_rx)
{
	L9963_SPI_Tx_Inst_t spi_tx_inst;
	uint8_t* spi_tx_data;
	uint8_t spi_mosi[5];

	/*1) package instruction*/
	spi_tx_inst.R = 0;
	spi_tx_inst.B.PA = 1;
	spi_tx_inst.B.RW = 1;
	spi_tx_inst.B.DEVID = device_id;
	spi_tx_inst.B.ADDRESS = addr;
	spi_tx_inst.B.GSW = 0;
	spi_tx_inst.B.DATA = data;
	spi_tx_inst.B.CRC_1 = CrcCompute_L9963(spi_tx_inst.R);

	/*2) package data*/
	spi_tx_data = (uint8_t*)&spi_tx_inst;

	spi_mosi[0] = *(spi_tx_data + 3); /*first 24 bit is reseved*/
	spi_mosi[1] = *(spi_tx_data + 4);
	spi_mosi[2] = *(spi_tx_data + 5);
	spi_mosi[3] = *(spi_tx_data + 6);
	spi_mosi[4] = *(spi_tx_data + 7);
	/*3) exchange data over SPI bus*/

	L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(&spi_mosi[0],
		(((uint8_t*)spi_rx) + 3), 5);
	L9963_SPI_CS_UNSELECT;


	/* Doing CRC verification*/
	if (spi_rx->B.CRC_1 == CrcCompute_L9963(spi_rx->R))
	{
		return true;
	}
	else
	{
		return false;
	}
}


//void GCDD_L9963_CAN_Transmit(void)
//{
//	Can_PduType GCDD_CanMessageTest ;
//	uint8 GCDD_CanData[8] = {0x0};
//	GCDD_CanMessageTest.id = 0x600U;
//	GCDD_CanMessageTest.length = 8;
//	Can_SetControllerMode(CanConf_CanController_L9963_CANNODE_0,CAN_T_START);
//				Can_MainFunction_Mode();
//	uint8 i = 0;
//	for(i=0;i<G_L9963_DEVICE_NUMBER;i++)
//	{
//
////		GCDD_CanData[0] = ((L9963_Reg[*ID+i].VCELL1.B.VOLT)&0xff);
////		GCDD_CanData[1] = ((L9963_Reg[*ID+i].VCELL1.B.VOLT)>>8);
////		GCDD_CanData[2] = ((L9963_Reg[*ID+i].VCELL2.B.VOLT)&0xff);
////		GCDD_CanData[3] = ((L9963_Reg[*ID+i].VCELL2.B.VOLT)>>8);
////		GCDD_CanData[4] = ((L9963_Reg[*ID+i].VCELL3.B.VOLT)&0xff);
////		GCDD_CanData[5] = ((L9963_Reg[*ID+i].VCELL3.B.VOLT)>>8);
////		GCDD_CanData[6] = ((L9963_Reg[*ID+i].VCELL4.B.VOLT)&0xff);
////		GCDD_CanData[7] = ((L9963_Reg[*ID+i].VCELL4.B.VOLT)>>8);
//		GCDD_CanData[0] = ((G_L9963_CellVoltage[i][0])&0xff);
//		GCDD_CanData[1] = ((G_L9963_CellVoltage[i][0])>>8);
//		GCDD_CanData[2] = ((G_L9963_CellVoltage[i][1])&0xff);
//		GCDD_CanData[3] = ((G_L9963_CellVoltage[i][1])>>8);
//		GCDD_CanData[4] = ((G_L9963_CellVoltage[i][2])&0xff);
//		GCDD_CanData[5] = ((G_L9963_CellVoltage[i][2])>>8);
//		GCDD_CanData[6] = ((G_L9963_CellVoltage[i][3])&0xff);
//		GCDD_CanData[7] = ((G_L9963_CellVoltage[i][3])>>8);
//		GCDD_CanMessageTest.sdu = GCDD_CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_1,&GCDD_CanMessageTest);
//
//		GCDD_CanMessageTest.id++;
//
////		GCDD_CanData[0] = ((L9963_Reg[*ID+i].VCELL5.B.VOLT)&0xff);
////		GCDD_CanData[1] = ((L9963_Reg[*ID+i].VCELL5.B.VOLT)>>8);
////		GCDD_CanData[2] = ((L9963_Reg[*ID+i].VCELL6.B.VOLT)&0xff);
////		GCDD_CanData[3] = ((L9963_Reg[*ID+i].VCELL6.B.VOLT)>>8);
////		GCDD_CanData[4] = ((L9963_Reg[*ID+i].VCELL7.B.VOLT)&0xff);
////		GCDD_CanData[5] = ((L9963_Reg[*ID+i].VCELL7.B.VOLT)>>8);
////		GCDD_CanData[6] = ((L9963_Reg[*ID+i].VCELL8.B.VOLT)&0xff);
////		GCDD_CanData[7] = ((L9963_Reg[*ID+i].VCELL8.B.VOLT)>>8);
//		GCDD_CanData[0] = ((G_L9963_CellVoltage[i][4])&0xff);
//		GCDD_CanData[1] = ((G_L9963_CellVoltage[i][4])>>8);
//		GCDD_CanData[2] = ((G_L9963_CellVoltage[i][5])&0xff);
//		GCDD_CanData[3] = ((G_L9963_CellVoltage[i][5])>>8);
//		GCDD_CanData[4] = ((G_L9963_CellVoltage[i][6])&0xff);
//		GCDD_CanData[5] = ((G_L9963_CellVoltage[i][6])>>8);
//		GCDD_CanData[6] = ((G_L9963_CellVoltage[i][7])&0xff);
//		GCDD_CanData[7] = ((G_L9963_CellVoltage[i][7])>>8);
//		GCDD_CanMessageTest.sdu = GCDD_CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_2,&GCDD_CanMessageTest);
//
//		GCDD_CanMessageTest.id++;
//
////		GCDD_CanData[0] = ((L9963_Reg[*ID+i].VCELL9.B.VOLT)&0xff);
////		GCDD_CanData[1] = ((L9963_Reg[*ID+i].VCELL9.B.VOLT)>>8);
////		GCDD_CanData[2] = ((L9963_Reg[*ID+i].VCELL10.B.VOLT)&0xff);
////		GCDD_CanData[3] = ((L9963_Reg[*ID+i].VCELL10.B.VOLT)>>8);
////		GCDD_CanData[4] = ((L9963_Reg[*ID+i].VCELL11.B.VOLT)&0xff);
////		GCDD_CanData[5] = ((L9963_Reg[*ID+i].VCELL11.B.VOLT)>>8);
////		GCDD_CanData[6] = ((L9963_Reg[*ID+i].VCELL12.B.VOLT)&0xff);
////		GCDD_CanData[7] = ((L9963_Reg[*ID+i].VCELL12.B.VOLT)>>8);
//		GCDD_CanData[0] = ((G_L9963_CellVoltage[i][8])&0xff);
//		GCDD_CanData[1] = ((G_L9963_CellVoltage[i][8])>>8);
//		GCDD_CanData[2] = ((G_L9963_CellVoltage[i][9])&0xff);
//		GCDD_CanData[3] = ((G_L9963_CellVoltage[i][9])>>8);
//		GCDD_CanData[4] = ((G_L9963_CellVoltage[i][10])&0xff);
//		GCDD_CanData[5] = ((G_L9963_CellVoltage[i][10])>>8);
//		GCDD_CanData[6] = ((G_L9963_CellVoltage[i][11])&0xff);
//		GCDD_CanData[7] = ((G_L9963_CellVoltage[i][11])>>8);
//		GCDD_CanMessageTest.sdu = GCDD_CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_3,&GCDD_CanMessageTest);
//		GCDD_CanMessageTest.id++;
//
////		*((uint32*)(CanData)) = L9963_Reg[*(ID+i)].VCELL13.B.VOLT
////							|((L9963_Reg[*(ID+i)].VCELL14.B.VOLT)<<16);
////		*((uint32*)(CanData+4)) = L9963_Reg[*(ID+i)].VSUMBAT.B.VSUM_BAT19_2
////							|((L9963_Reg[*(ID+i)].VBATTDIV.B.VBATTDIV)<<16);
////		GCDD_CanData[0] = ((L9963_Reg[*ID+i].VCELL13.B.VOLT)&0xff);
////		GCDD_CanData[1] = ((L9963_Reg[*ID+i].VCELL13.B.VOLT)>>8);
////		GCDD_CanData[2] = ((L9963_Reg[*ID+i].VCELL14.B.VOLT)&0xff);
////		GCDD_CanData[3] = ((L9963_Reg[*ID+i].VCELL14.B.VOLT)>>8);
//		GCDD_CanData[0] = ((G_L9963_CellVoltage[i][12])&0xff);
//		GCDD_CanData[1] = ((G_L9963_CellVoltage[i][12])>>8);
//		GCDD_CanData[2] = ((G_L9963_CellVoltage[i][13])&0xff);
//		GCDD_CanData[3] = ((G_L9963_CellVoltage[i][13])>>8);
//		GCDD_CanData[4] = ((L9963_Reg[*ID+i].VSUMBAT.B.VSUM_BAT19_2)&0xff);
//		GCDD_CanData[5] = ((L9963_Reg[*ID+i].VSUMBAT.B.VSUM_BAT19_2)>>8);
//		GCDD_CanData[6] = ((L9963_Reg[*ID+i].VBATTDIV.B.VBATTDIV)&0xff);
//		GCDD_CanData[7] = ((L9963_Reg[*ID+i].VBATTDIV.B.VBATTDIV)>>8);
//		GCDD_CanMessageTest.sdu = GCDD_CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_4,&GCDD_CanMessageTest);
//		GCDD_CanMessageTest.id++;
//
//		*((uint32*)(GCDD_CanData)) = ((G_L9963_ChipTemp[i])<<24);
//		*((uint32*)(GCDD_CanData+4)) = 0;
//
//		GCDD_CanMessageTest.sdu = GCDD_CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_5,&GCDD_CanMessageTest);
//		GCDD_CanMessageTest.id++;
//
//		*((uint32*)(GCDD_CanData)) = G_L9963_CellTemp[i][1]
//									|G_L9963_CellTemp[i][0]<<16;
//		*((uint32*)(GCDD_CanData+4)) = G_L9963_CellTemp[i][3]
//									|G_L9963_CellTemp[i][2]<<16;
//
//		GCDD_CanMessageTest.sdu = GCDD_CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_6,&GCDD_CanMessageTest);
//		GCDD_CanMessageTest.id++;
//
//		*((uint32*)(GCDD_CanData)) = G_L9963_CellTemp[i][5]
//									|G_L9963_CellTemp[i][4]<<16;
//		*((uint32*)(GCDD_CanData+4)) = G_L9963_CellTemp[i][6];
//
//		GCDD_CanMessageTest.sdu = GCDD_CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_7,&GCDD_CanMessageTest);
//		GCDD_CanMessageTest.id++;
//	}
//}

static void Delay(unsigned long cnt)
{

	cnt *= 10;
	for(;cnt!=0;cnt--)
	{
		__nop();
		//NOP();
	}
}

