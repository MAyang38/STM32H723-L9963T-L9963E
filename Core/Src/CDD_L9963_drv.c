/*
 L9963_driver - Copyright (C) 2018 STMicroelectronics

* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms SLA0089 at www.st.com.
*
* THIS SOFTWARE IS DISTRIBUTED "AS IS," AND ALL WARRANTIES ARE DISCLAIMED,
* INCLUDING MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
* EVALUATION ONLY 鈥� NOT FOR USE IN PRODUCTION

 Author: Leslie Lu

 This file has been passed for MISRA-2012 check
 */
#include "CDD_L9963_drv.h"
#include "typedefs.h"

#define NULL_PTR ((void *)0)
#define L9963_DEVICE_NUMBER            4
#define L9963T_BURST_HIGHSPEED 1
#define DEBUG
uint8 total_id = 4;
uint8 id[15] =
{ 1, 2, 3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8 ID[15] =
{ 1, 2, 3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/*L9963_Reg[BROADCAST_ID] is a broadcast register*/
/*L9963_Reg[BROADCAST_ID]--[15] is device registers*/
L9963T_NO L9963T_DEVICE = L9963T1;
L9963_Reg_type L9963_Reg[16];
L9963_SPI_Rx_Inst_t spi_rx_t;
#define WORD_LEN 40UL
#define CRC_LEN 6UL
#define CRC_POLY (uint64)0x0000000000000059 /*L9301 CRC Poly:x^3+x^2+x+1*/
#define CRC_SEED (uint64)0x0000000000000038
#define CRC_INIT_SEED_MASK (CRC_SEED<<(WORD_LEN-CRC_LEN))
#define CRC_INIT_MASK (CRC_POLY<<(WORD_LEN-CRC_LEN-1))
#define FIRST_BIT_MASK ((uint64)1<<(WORD_LEN-1))  // 0x80000000
#define CRC_LOWER_MASK ((uint8)(1<<CRC_LEN)-1) //0b111

#define ERROR_WRITE             0             /* If configure ID fails it will config the ID again*/
//#define CRC_TABLE
#define USE_L9963T
//#define NOP()	_asm("nop")
static void Delay(unsigned long cnt){


	cnt *= 100;//7
   for(;cnt!=0;cnt--){
		 
		 __nop();
	//   NOP();
   }

}


tabcrcConfigSet_t L9963crc_Config =
{ (40 - 6),    //frame_lengh (frame length number of active data bits [0..2040])
		6,             //crc_length (length of CRC in bits [1..7])
		0x59,          //polynom (CRC polynom [1..255])
		0x38,          //seed_value (CRC seed value [0..255])
		0, //frame_position highest (first) bit position in input streem  [0..7])
		0, 0, 0, 0, 0,
		{ 0 } };
// **************************9**************************************************
// Project Name:        L9963 comunication library
//
// Filename:            crc.c
// Version:             $Revision: 1.2
// Date(dd.mm.yyyy):    $Date: 2011-08-25
// Author(s):           Jiri Ryba
// Company:             STMicroelectronics
//                      Pobrezni 3
//                      186 00 Praha 8, Czech Republic
//
// Processor:           GeneraL 'C' code
// Compiler:            GHS
//
// Description:         l9963 CRC calculation
//
// ***************************** Revision History *****************************
//
// Date           Name         Description
// 21.01.2019     Jiri Ryba    Original Issue
// 23.01.2019     Jiri Ryba    "C" level optimization
// 11.02.2019     Jiri Ryba    Modified to be general up to 8 bit polynom and 7 bit CRC
// 12.02.2019     Jiri Ryba    Bugfix
//
// ****************************************************************************

// ****************************************************************************
// Name:        tabcrcInit
// Parameters:
//              tabcrcConfigSet_t * config - pointer to configuration the
//              configuration have folowed fields which must be configured
//              before the init function is used:
//                uint16 frame_lengh;        //frame length number of active data bits [8..2040]
//                uint8 crc_length;         //length of CRC in bits [1..7]
//                uint8 polynom;            //CRC polynom [1..255]
//                uint8 seed_value;         //CRC seed value [0..255]
//                uint8 frame_position;     //highest (first) bit position in input streem [0..7]
//
// Returns:     none
// Description:
//              CRC driver initialization. The function calculates
//              constants used during CRC calculation
// ****************************************************************************
void tabcrcInit(tabcrcConfigSet_t * config)
{
	uint16 input_frame;
	uint16 bit_mask;
	uint16 temp_data;
	uint16 polynom;
	uint16 counter;
	uint8 polyshift = 15 - config->crc_length; //number of active bits in polynom

	config->polynom |= (uint16)(1 << config->crc_length);
	for (input_frame = 0; input_frame < 256; input_frame++)
	{
		bit_mask = 0x8000;
		temp_data = input_frame << 8;
		polynom = (uint16)(config->polynom << polyshift);
		counter = 8;
		while (counter > 0)
		{
			counter--;
			if (bit_mask & temp_data)
			{
				temp_data = temp_data ^ polynom;
			}
			bit_mask = bit_mask >> 1;
			polynom = polynom >> 1;
		}
		config->crc_tab_p[input_frame] = temp_data;
	}

	config->frame_last_index = (config->frame_position + config->frame_lengh)
			/ 8;
	config->seed_value_aligned = config->seed_value << (8 - config->crc_length);
	config->rest_bits = config->frame_lengh & 0x7;
	config->result_shift = 8 - config->crc_length - config->rest_bits;
	config->polynom_shifted = config->polynom << (7 - config->crc_length);
}

// ****************************************************************************
// Name:        L9963_CrcCalc8bitLookupTab
// Parameters:
//             tabcrcConfigSet_t  config - pointer to configuration
//             uint8 * frame - pointer to highest byte of frame
// Returns:
//               CRC
// Description:
//              CRC calculation based on precalculated 8bits table
//              tabcrcInit must be executed before driver using
// ****************************************************************************

#ifdef CRC_SLOW
uint8  CrcCalc8bitLookupTab_B(tabcrcConfigSet_t * config, uint8 * frame)
{

	uint8 dataout;
	uint8 bitmask, bitnumber, poly;
	uint8 lastIndex = config->frame_last_index;
	uint8 frameShiftl = config->frame_position;
	frame[4] &= 0xC0;
	if (0 == frameShiftl)
	{
		uint8 tabPointer = frame[0] ^ config->seed_value_aligned;
		uint8 frameIndex = 1;
		dataout = config->crc_tab_p[tabPointer];
		while (frameIndex < lastIndex)
		{
			tabPointer = frame[frameIndex] ^ dataout;
			dataout = config->crc_tab_p[tabPointer];
			frameIndex++;
		}
		dataout = frame[frameIndex] ^ dataout;
	}
	else
	{
		uint8 frameShiftr = 8 - frameShiftl;
		uint8 tabPointer = frame[0] << frameShiftl;
		uint8 frameIndex = 1;
		tabPointer |= frame[1] >> frameShiftr;
		tabPointer ^= config->seed_value_aligned;
		dataout = config->crc_tab_p[tabPointer];
		while (frameIndex < lastIndex)
		{
			tabPointer = frame[frameIndex] << frameShiftl;
			frameIndex++;
			tabPointer |= frame[frameIndex] >> frameShiftr;
			tabPointer ^= dataout;
			dataout = config->crc_tab_p[tabPointer];
		}
		tabPointer = frame[frameIndex] << frameShiftl;
		frameIndex++;
		tabPointer |= frame[frameIndex] >> frameShiftr;
		dataout ^= tabPointer;
	}
	//Process rest of bits if CRC is lower than 8bits
	bitmask = 0x80;
	bitnumber = config->rest_bits;
	poly = config->polynom_shifted;
	while (bitnumber > 0)
	{
		if (bitmask & dataout)
		{                     //last bit 7, so the rest is bit 0..6
			dataout = dataout ^ poly;
		}
		bitnumber--;
		bitmask = bitmask >> 1;
		poly = poly >> 1;
	}
	dataout = dataout >> (config->result_shift);
	return dataout;
}
#else
const uint16 CrcCalc8bitLookupTab_B[] =
{ 0x3123, 0x0005, 0x8474, 0x3143, 0x0007, 0x1809, 0xa800, 0x74e0, 0x0031,
		0x9404, 0xe228, 0x3104, 0x0000, 0x18aa, 0xa801, 0x8603, 0x7d07, 0x0278,
		0x74e5, 0x063f, 0x7d63, 0x2a14, 0x316b, 0x000b, 0x7a05, 0x00fe, 0x192a,
		0x84fe, 0x7c8c, 0x2378, 0x7528, 0x063f, 0x1808, 0x8001, 0x00b0, 0x18ec,
		0x0001, 0x7ce6, 0x5a78, 0x7ca3, 0x3214, 0x3165, 0x000b, 0x7a20, 0xfff0,
		0x7c84, 0x50ae, 0x7d67, 0x2278, 0xe844, 0x1989, 0xb008, 0x8054, 0x3104,
		0x0001, 0x18aa, 0xa801, 0x758c, 0x063f, 0x8663, 0x7cab, 0x4830, 0x7d07,
		0x6630, 0x7d60, 0x3b78, 0x7c8b, 0x2378, 0x7c05, 0x3278, 0x4816, 0x74a7,
		0x063f, 0x7ca3, 0x3a14, 0x8b55, 0x7a05, 0x0090, 0x1906, 0x8001, 0x18eb,
		0x0001, 0x7506, 0x063f, 0x7ce7, 0x4830, 0x7d04, 0x30ae, 0x7c06, 0x5040,
		0x7d00, 0x6630, 0x4407, 0x74e0, 0x063f, 0x7ca5, 0x0278, 0x7ce3, 0x2a14,
		0x8b57, 0xe2ea, 0x1946, 0x8001, 0x754b, 0x063f, 0x7c84, 0x58ae, 0x7d08,
		0x4830, 0x7c89, 0x6630, 0x7d2c, 0x4378, 0x7ca6, 0x6278, 0x74c7, 0x063f,
		0x3143, 0x0008, 0x8a03, 0x180a, 0xa800, 0xe614, 0x192a, 0x84ff, 0x6386,
		0x752c, 0x063f, 0x190c, 0x8001, 0x7d09, 0x03a6, 0x7c0b, 0x3a78, 0x7cc8,
		0x3839, 0x6810, 0x6816, 0x7ce7, 0x589e, 0x7a20, 0xfff0, 0x8933, 0x7ce5,
		0x1e30, 0x74a3, 0x063f, 0x0004, 0x7160, 0x0002, 0xe8d3, 0x7140, 0x0001,
		0xe894 };
#endif
/***************************************************
 Description:
 example
 InputWord: 32bits data need to add 3bit lower crc
 data returned is 32bit with lower 3bit lower crc
 return :  32bits with 3bits crc


 ***************************************************/
uint8 CrcCompute_L9963(unsigned long long InputWord)
{
#ifdef CRC_TABLE
	return CrcCalc8bitLookupTab(&L9963crc_Config,
			(((uint8 *) &InputWord) ));              /* [MISRA 2012 Rule 1.3, violate]
			                                                 because we use the crc table algorithm*/
#else
	uint64 TestBitMask;
	uint64 CRCMask;
	uint64 BitCount;
	uint64 LeftAlignedWord;

	InputWord &= 0xFFFFFFFFFFFFFFC0; /* Clear the CRC bit in the data frame*/
	LeftAlignedWord = InputWord ^ CRC_INIT_SEED_MASK;

	TestBitMask = ((uint64) 1 << ( WORD_LEN - 1));
	CRCMask = CRC_INIT_MASK; // 1111 <<
	BitCount = ( WORD_LEN - CRC_LEN);
	while (0 != BitCount--)
	{
		if (0 != (LeftAlignedWord & TestBitMask))
		{
			LeftAlignedWord ^= CRCMask;
		} /* endif */
		CRCMask >>= 1;
		TestBitMask >>= 1;
	} /* endwhile */

	LeftAlignedWord &= (uint64)CRC_LOWER_MASK;
	return LeftAlignedWord;
#endif
}

/***************************************************
 Description:Convert Complement of 2 to original data
 example
 InputWord: 32bits data need to add 3bit lower crc
 data returned is 32bit with lower 3bit lower crc
 return :  32bits with 3bits crc


 ***************************************************/
int32_t Calculate_original_data_18bit(uint32 InputWord)
{
	uint32 flag;
	int OutputWord;
	flag = InputWord & 0x20000;
	if (flag)         //If negative, then convert it to original data
	{
		OutputWord = -(((~InputWord) & 0x00003FFFF) + 1);
	}
	else
	{
		OutputWord = InputWord;
	}
	return OutputWord;
}

/***************************************************
 Description:Convert Complement of 2 to original data
 example
 InputWord: 32bits data need to add 3bit lower crc
 data returned is 32bit with lower 3bit lower crc
 return :  32bits with 3bits crc


 ***************************************************/
int32_t Calculate_original_data_32bit(uint32 InputWord)
{
	uint32 flag;
	int OutputWord;
	flag = InputWord & 0x80000000;
	if (flag)         //If negative, then convert it to original data
	{
		OutputWord = -((~InputWord) + 1);
	}
	else
	{
		OutputWord = InputWord;
	}
	return OutputWord;
}
/****************************************************************************
 * Name:L9963_SPI_start
 * Parameters:NULL_PTR
 *
 *
 * Returns:NULL_PTR
 *
 *
 * Description:L9963_SPI_start
 *
 *
 *
 *
 *
 *
 * ****************************************************************************/
void L9963_SPI_start(void)
{

}

/****************************************************************************
 * Name:L9963_SPI_start
 * Parameters:NULL_PTR
 *
 *
 * Returns:NULL_PTR
 *
 *
 * Description:L9963_SPI_start
 *
 *
 *
 *
 *
 *
 * ****************************************************************************/
void L9963T_SPI_start(void)
{

}

/****************************************************************************
 * Name:L9963_SPI_start
 * Parameters:NULL_PTR
 *
 *
 * Returns:NULL_PTR
 *
 *
 * Description:L9963_SPI_start
 *
 *
 *
 *
 *
 *
 * ****************************************************************************/
void L9963_DIG_ISO_SPI_start(void)
{

}
/****************************************************************************
 * Name:
 * Parameters:
 *
 *
 * Returns:
 *
 *		L9963_SPI_Rx_t* spi_rx  -> data returned on SPI bus
 * Description:
 *		Write One Register of L9963
 *
 *
 *
 *
 *
 * ****************************************************************************/
uint8 L9963_Single_Write(L9963_Addr_t addr, uint8 device_id, uint32 data,
		L9963_SPI_Rx_Inst_t* spi_rx)
{
	L9963_SPI_Tx_Inst_t spi_tx_inst;
	uint8 *spi_tx_data;
	uint8 spi_mosi[5];

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
	spi_tx_data = (uint8*) &spi_tx_inst;

//	spi_mosi[0] = *(spi_tx_data + 3); /*first 24 bit is reseved*/
//	spi_mosi[1] = *(spi_tx_data + 4);
//	spi_mosi[2] = *(spi_tx_data + 5);
//	spi_mosi[3] = *(spi_tx_data + 6);
//	spi_mosi[4] = *(spi_tx_data + 7);
//	/*3) exchange data over SPI bus*/
//	
	
	
	spi_mosi[0] = *(spi_tx_data + 4); /*first 24 bit is reseved*/
	spi_mosi[1] = *(spi_tx_data + 3);
	spi_mosi[2] = *(spi_tx_data + 2);
	spi_mosi[3] = *(spi_tx_data + 1);
	spi_mosi[4] = *(spi_tx_data + 0);
	/*3) exchange data over SPI bus*/
	
	
//	L9963_SPI_CS_SELECT;
//	GCDD_L9963_TxData(&spi_mosi[0],
//			(((uint8 *) spi_rx) + 3), 5);
//	L9963_SPI_CS_UNSELECT;


L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(&spi_mosi[0],
			(((uint8 *) spi_rx) ), 5);
	L9963_SPI_CS_UNSELECT;


	/* Doing CRC verification*/
	if (spi_rx->B.CRC_1 == CrcCompute_L9963(spi_rx->R))
	{
		return true;
	}
	else
	{
		//return false;
		return true;
	}

}

/****************************************************************************
 * Name:
 * Parameters:
 *
 *
 * Returns:
 *
 *		L9963_SPI_Rx_t* spi_rx  -> data returned on SPI bus
 * Description:
 *		Write One Register of L9963
 *
 *
 *
 *
 *
 * ****************************************************************************/
uint8 L9963_Direct_Write(uint8 *spi_mosi, uint8 *spi_miso)
{
	L9963_SPI_Rx_Inst_t spi_rx;
	/*3) exchange data over SPI bus*/
	L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(spi_mosi, spi_miso, 5);
	L9963_SPI_CS_UNSELECT;


	spi_rx.R = ((uint64) * spi_miso << 32)
			+ ((uint32) *(spi_miso + 1) << 24)
			+ ((uint32) *(spi_miso + 2) << 16)
			+ ((uint16) *(spi_miso + 3) << 8) + *(spi_miso + 4);

	/* Doing CRC verification*/
	if (spi_rx.B.CRC_1 == CrcCompute_L9963(spi_rx.R))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/****************************************************************************
 * Name:
 * Parameters:
 *
 *
 * Returns:
 *
 *		L9963_SPI_Rx_t* spi_rx  -> data returned on SPI bus
 * Description:
 *		Write One Register of L9963
 *
 *
 *
 *
 *
 * ****************************************************************************/
uint8 L9963_Single_Read(L9963_Addr_t addr, uint8 device_id,
		L9963_SPI_Rx_Inst_t* spi_rx)
{
	L9963_SPI_Tx_Inst_t spi_tx_inst;
	uint8 *spi_tx_data;
	uint8 spi_mosi[5];
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
	spi_tx_data = (uint8*) &spi_tx_inst;

//	spi_mosi[0] = *(spi_tx_data + 3); /*first 24 bit is reseved*/
//	spi_mosi[1] = *(spi_tx_data + 4);
//	spi_mosi[2] = *(spi_tx_data + 5);
//	spi_mosi[3] = *(spi_tx_data + 6);
//	spi_mosi[4] = *(spi_tx_data + 7);
	
	
	spi_mosi[0] = *(spi_tx_data + 4); /*first 24 bit is reseved*/
	spi_mosi[1] = *(spi_tx_data + 3);
	spi_mosi[2] = *(spi_tx_data + 2);
	spi_mosi[3] = *(spi_tx_data + 1);
	spi_mosi[4] = *(spi_tx_data + 0);
	
	
	
//	/*3) exchange data over SPI bus*/
//	L9963_SPI_CS_SELECT;
//	GCDD_L9963_TxData(&spi_mosi[0],
//			(((uint8 *) spi_rx) + 3), 5);
//L9963_SPI_CS_UNSELECT;
//	Delay(319); /*Total Delay time need to be 219us */
//	L9963_SPI_CS_SELECT;
//	GCDD_L9963_TxData(&spi_mosi[0],
//			(((uint8 *) spi_rx) + 3), 5);
//L9963_SPI_CS_UNSELECT;
//	Delay(319); /*Total Delay time need to be 219us */
//	
//	
	
//	/*3) exchange data over SPI bus*/
//	L9963_SPI_CS_SELECT;
//	GCDD_L9963_TxData(&spi_mosi[0],
//			(((uint8 *) spi_rx) + 3), 5);
//L9963_SPI_CS_UNSELECT;
//	Delay(319); /*Total Delay time need to be 219us */
//	L9963_SPI_CS_SELECT;
//	GCDD_L9963_TxData(&spi_mosi[0],
//			(((uint8 *) spi_rx) + 3), 5);
//L9963_SPI_CS_UNSELECT;
//	Delay(319); /*Total Delay time need to be 219us */
//	
//	
		/*3) exchange data over SPI bus*/
	L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(&spi_mosi[0],
			(((uint8 *) spi_rx) ), 5);
L9963_SPI_CS_UNSELECT;
	Delay(319); /*Total Delay time need to be 219us */
	L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(&spi_mosi[0],
			(((uint8 *) spi_rx) ), 5);
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
uint8 L9963_Single_Read_temp(L9963_Addr_t addr, uint8 device_id,
		L9963_SPI_Rx_Inst_t* spi_rx)
{
	L9963_SPI_Tx_Inst_t spi_tx_inst;
	uint8 *spi_tx_data;
	uint8 spi_mosi[5];
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
	spi_tx_data = (uint8*) &spi_tx_inst;

	spi_mosi[0] = *(spi_tx_data + 3); /*first 24 bit is reseved*/
	spi_mosi[1] = *(spi_tx_data + 4);
	spi_mosi[2] = *(spi_tx_data + 5);
	spi_mosi[3] = *(spi_tx_data + 6);
	spi_mosi[4] = *(spi_tx_data + 7);
	/*3) exchange data over SPI bus*/
	L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(&spi_mosi[0],
			(((uint8 *) spi_rx) + 3), 5);
L9963_SPI_CS_UNSELECT;
	Delay(319); /*Total Delay time need to be 219us */
	Delay(319);
	Delay(319);Delay(319);
	Delay(319);
	Delay(319);
	L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(&spi_mosi[0],
			(((uint8 *) spi_rx) + 3), 5);
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
/****************************************************************************
 * Name:
 * Parameters:
 *
 *
 * Returns:
 *
 *		L9963_SPI_Rx_t* spi_rx  -> data returned on SPI bus
 * Description:
 *		Write One Register of L9963
 *
 *
 *
 *
 *
 * ****************************************************************************/
uint8 L9963_Wakeup(uint8 device_id, L9963_SPI_Rx_Inst_t* spi_rx)
{
	uint8 return_code = true;
	return_code &= L9963_Single_Read(GEN_STATUS_ADD, device_id, spi_rx);
	/* Doing CRC verification*/
	return return_code;

}

/****************************************************************************
 * Name:
 * Parameters:
 *
 *
 * Returns:
 *
 *		L9963_SPI_Rx_t* spi_rx  -> data returned on SPI bus
 * Description:
 *		Write One Register of L9963
 *
 *
 *
 *
 *
 * ****************************************************************************/
uint8 L9963_Sleep(uint8 device_id, L9963_SPI_Rx_Inst_t* spi_rx)
{
	uint8 return_code = true;
	L9963_Reg[device_id].FSM_t.B.GO2SLP = 2; /* when written to 10 it resets all reg map registers */
	return_code &= L9963_Single_Write(FSM_ADD, device_id,
			(uint32) L9963_Reg[device_id].FSM_t.R, spi_rx);
	return return_code;

}
/****************************************************************************
 * Name:
 * Parameters:
 *
 *
 * Returns:
 *
 *		L9963_SPI_Rx_t* spi_rx  -> data returned on SPI bus
 * Description:
 *		Write One Register of L9963
 *
 *
 *
 *
 *
 * ****************************************************************************/
L9963_SPI_Rx_Inst_t spi_rx2;
uint8 L9963_Direct_Read(uint8 *spi_mosi, uint8 *spi_miso)
{
	L9963_SPI_Rx_Inst_t spi_rx;
	/*3) exchange data over SPI bus*/
	L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(spi_mosi, spi_miso, 5);
L9963_SPI_CS_UNSELECT;
	Delay(319);
L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(spi_mosi, spi_miso, 5);
L9963_SPI_CS_UNSELECT;

	spi_rx.R = ((uint64) * spi_miso << 32)
			+ ((uint32) *(spi_miso + 1) << 24)
			+ ((uint32) *(spi_miso + 2) << 16)
			+ ((uint16) *(spi_miso + 3) << 8) + *(spi_miso + 4);
	spi_rx2.R = ((uint64) * spi_miso << 32)
					+ ((uint32) *(spi_miso + 1) << 24)
					+ ((uint32) *(spi_miso + 2) << 16)
					+ ((uint16) *(spi_miso + 3) << 8) + *(spi_miso + 4);
	/* Doing CRC verification*/
	if (spi_rx.B.CRC_1 == CrcCompute_L9963(spi_rx.R))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/****************************************************************************
 * Name:
 * Parameters:
 *
 *
 * Returns:
 *
 *		L9963_SPI_Rx_t* spi_rx  -> data returned on SPI bus
 * Description:
 *		Write One Register of L9963
 *
 *
 *
 * ****************************************************************************/
uint8 L9963_Burst_Bug_fix(L9963_Addr_t addr, uint8 device_id,
		uint8* spi_miso, uint8 burst_data_frameno)
{
	uint8 *spi_miso_address;
	spi_miso_address = spi_miso;
	L9963_SPI_Rx_Inst_t spi_rx;
	/* Convert the Frame number to the address number*/
	/* Frame Number is 2, the Burst read Frame No is 0x62(d98), You need to convert it to real number that is 1, So it need to minus 97*/
	burst_data_frameno -= 97;
	if (addr == BURST_7ACMD)
	{
		return false; /* Only run in Burst read function, otherwise return error*/
	}
	else
	{
		if (L9963_Single_Read(addr, device_id, &spi_rx) == true)
		{
			spi_miso = spi_miso_address + 5 * burst_data_frameno;
			spi_miso[0] = *((uint8*) &spi_rx + 3); /*first 24 bit is reseved*/
			spi_miso[1] = *((uint8*) &spi_rx + 4);
			spi_miso[2] = *((uint8*) &spi_rx + 5);
			spi_miso[3] = *((uint8*) &spi_rx + 6);
			spi_miso[4] = *((uint8*) &spi_rx + 7);
			Delay(319); /*Total Delay time need to be 219us */
			return true;
		}
		else
		{
			return false;

		}

	}
}
/****************************************************************************
 * Name:
 * Parameters:
 *
 *
 * Returns:
 *
 *		L9963_SPI_Rx_t* spi_rx  -> data returned on SPI bus
 * Description:
 *		Write One Register of L9963
 *
 *
 *
 *
 *
 * ****************************************************************************/
uint8 L9963_Burst_Read(L9963_Addr_t addr, uint8 device_id,
		uint8* spi_miso, BURST_SIZE burst_size)
{
	L9963_SPI_Tx_Inst_t spi_tx_inst;
	L9963_SPI_Rx_Inst_t spi_rx;
	uint8 *spi_tx_data;
	uint8 spi_mosi[5];
	uint8 *spi_miso_address;
	uint8 i;
	uint8 return_code = true;
//#ifdef USE_L9963T
//	uint64 Time_start = 0;
//#endif


	spi_miso_address = spi_miso;
	/*1) package instruction*/
	spi_tx_inst.R = 0;
	spi_tx_inst.B.PA = 1;
	spi_tx_inst.B.RW = 0; /* In burst mode,R/W = 0*/
	spi_tx_inst.B.DEVID = device_id;
	spi_tx_inst.B.ADDRESS = addr;
	spi_tx_inst.B.GSW = 0;
	spi_tx_inst.B.DATA = 0xFF; /* In burst mode,any data is possible*/
	spi_tx_inst.B.CRC_1 = CrcCompute_L9963(spi_tx_inst.R);

	/*2) package data*/
	spi_tx_data = (uint8*) &spi_tx_inst;

//	spi_mosi[0] = *(spi_tx_data + 3); /*first 24 bit is reseved*/
//	spi_mosi[1] = *(spi_tx_data + 4);
//	spi_mosi[2] = *(spi_tx_data + 5);
//	spi_mosi[3] = *(spi_tx_data + 6);
//	spi_mosi[4] = *(spi_tx_data + 7);
//	
	
	
		spi_mosi[0] = *(spi_tx_data + 4); /*first 24 bit is reseved*/
	spi_mosi[1] = *(spi_tx_data + 3);
	spi_mosi[2] = *(spi_tx_data + 2);
	spi_mosi[3] = *(spi_tx_data + 1);
	spi_mosi[4] = *(spi_tx_data + 0);
	
	

	/*3) exchange data over SPI bus*/
	L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(&spi_mosi[0], spi_miso, 5);
	L9963_SPI_CS_UNSELECT;

	spi_rx.R = ((uint64) * spi_miso << 32)
			+ ((uint32) *(spi_miso + 1) << 24)
			+ ((uint32) *(spi_miso + 2) << 16)
			+ ((uint16) *(spi_miso + 3) << 8) + *(spi_miso + 4);
#ifndef USE_L9963T
	/* Doing CRC verification*/
	if (spi_rx.B._1 == CrcCompute_L9963(spi_rx.R))
	{
	}
	else
	{
		//return_code = false;
		return_code = true;
	}
#endif
	Delay(519); /*Total Delay time need to be 419 */
	/*Select the SPI to Receive the Burst data*/
	//LY_NEED_LOW_CS_BEFORE_READ_?_
	//spi_lld_select(SPID_Temp);
	L9963_SPI_CS_SELECT;
    spi_mosi[0] = 0xFF; /*Burst read is reseved*/
	spi_mosi[1] = 0xFF; /*Burst read bit is reseved*/
	spi_mosi[2] = 0xFF; /*Burst read bit is reseved*/
	spi_mosi[3] = 0xFF; /*Burst read bit is reseved*/
	spi_mosi[4] = 0xFF; /*Burst read bit is reseved*/

	//spi_lld_polled_exchange_multiframe(SPID_Temp, &spi_mosi[0], spi_miso, 5);     /*Burst command feedback*/

	for (i = 0; i < burst_size; i++)
	{
#ifdef USE_L9963T
		//LY_change
		//Time_start = osalThreadGetMicroseconds();

//		while (L9963T_BNE_STATUS == 0)
//		{
//
//			//DONOT BNE Timeout detection
//			/*
//			if (osalThreadGetMicroseconds() - Time_start > 1500)
//			{
//				return_code = false;
//				return return_code;
//			}
//			*/
//		}
		L9963_SPI_CS_SELECT;

		GCDD_L9963_TxData(&spi_mosi[0], spi_miso, 5);

		L9963_SPI_CS_UNSELECT;
#else
		GCDD_L9963_TxData(SPID_Temp, &spi_mosi[0], spi_miso, 5);
#endif
//		spi_rx.R = *(vuint64*) (spi_miso - 3);//SOPA CHANGE
		//spi_rx.R = *(uint64*) (spi_miso - 3);
		
		
				spi_rx.R = *(uint64*) (spi_miso - 3);

#ifdef DEBUG
		while(L9963_Register_Update(&spi_rx, addr, device_id)!=true);
#endif
		if (spi_rx.B.CRC_1 == CrcCompute_L9963(spi_rx.R))
		{
			spi_miso += 5;
		}
		else
		{
//			L9963_SPI_CS_UNSELECT;
//			return_code = false;
			
			spi_miso += 5;
			return_code = true;
		}
	}
	L9963_SPI_CS_UNSELECT;

	/* Here is the code for fix a bug when using the Burst, because the BURST_7ACMD will not update the diagnostic information*/
	if (addr == BURST_7ACMD)
	{
		/* VCELLUV is the 6th of the Burst data*/
		if (L9963_Burst_Bug_fix(VCELL_UV_ADD, device_id, spi_miso_address, FRAME6)
				== false)
		{
			return false;
		}
		else
		{

		}

		/* VCELLOV is the 7th of the Burst data*/
		if (L9963_Burst_Bug_fix(VCELL_OV_ADD, device_id, spi_miso_address, FRAME7)
				== false)
		{
			return false;
		}
		else
		{

		}

		/* GPIOUTOT is the 7th of the Burst data*/
		if (L9963_Burst_Bug_fix(VGPIO_OT_UT_ADD, device_id, spi_miso_address, FRAME8)
				== false)
		{
			return false;
		}
		else
		{

		}

	}
	else
	{

	}
	return return_code;
}

/****************************************************************************
 * Name:
 * Parameters:
 *
 *
 * Returns:
 *
 *		L9963_SPI_Rx_t* spi_rx  -> data returned on SPI bus
 * Description:
 *		Write One Register of L9963
 *
 *
 *
 *
 *
 * ****************************************************************************/
uint8 L9963_Burst_Read_Onchip(L9963_Addr_t addr, uint8 device_id,
		L9963_SPI_Rx_Inst_t* spi_rx, BURST_SIZE burst_size)
{
	L9963_SPI_Tx_Inst_t spi_tx_inst;
	uint8 *spi_tx_data;
	uint8 spi_miso[5];
	uint8 spi_mosi[5];
	uint8 i_burst_read_onchip;
	/*1) package instruction*/
	spi_tx_inst.R = 0;
	spi_tx_inst.B.PA = 1;
	spi_tx_inst.B.RW = 0; /* In burst mode,R/W = 0*/
	spi_tx_inst.B.DEVID = device_id;
	spi_tx_inst.B.ADDRESS = addr;
	spi_tx_inst.B.GSW = 0;
	spi_tx_inst.B.DATA = 0xFF; /* In burst mode,any data is possible*/
	spi_tx_inst.B.CRC_1 = CrcCompute_L9963(spi_tx_inst.R);

	/*2) package data*/
	spi_tx_data = (uint8*) &spi_tx_inst;

	spi_mosi[0] = *(spi_tx_data + 3); /*first 24 bit is reseved*/
	spi_mosi[1] = *(spi_tx_data + 4);
	spi_mosi[2] = *(spi_tx_data + 5);
	spi_mosi[3] = *(spi_tx_data + 6);
	spi_mosi[4] = *(spi_tx_data + 7);
	/*3) exchange data over SPI bus*/
	L9963_SPI_CS_SELECT;
	GCDD_L9963_TxData(&spi_mosi[0], &spi_miso[0], 5);
L9963_SPI_CS_UNSELECT;

	spi_rx->R = ((uint64) spi_miso[0] << 32) + ((uint32) spi_miso[1] << 24)
			+ ((uint32) spi_miso[2] << 16) + ((uint16) spi_miso[3] << 8)
			+ spi_miso[4];

	/* Doing CRC verification*/
	if (spi_rx->B.CRC_1 == CrcCompute_L9963(spi_rx->R))
	{
	}
	else
	{
		return false;
	}
	Delay(419); /*Total Delay time need to be 419 */
	/*Select the SPI to Receive the Burst data*/
	L9963_SPI_CS_SELECT;

	spi_mosi[0] = 0xFF; /*Burst read is reseved*/
	spi_mosi[1] = 0xFF; /*Burst read bit is reseved*/
	spi_mosi[2] = 0xFF; /*Burst read bit is reseved*/
	spi_mosi[3] = 0xFF; /*Burst read bit is reseved*/
	spi_mosi[4] = 0xFF; /*Burst read bit is reseved*/
	for (i_burst_read_onchip = 0; i_burst_read_onchip < burst_size; i_burst_read_onchip++)
	{
		GCDD_L9963_TxData(&spi_mosi[0], &spi_miso[0], 5);
		//spi_lld_polled_exchange_multiframe(SPID_Temp, &spi_mosi[0],
		//		((uint8 *) spi_rx) + 3, 5);

		while(L9963_Register_Update(spi_rx, addr, device_id)!=true);

		if (spi_rx->B.CRC_1 == CrcCompute_L9963(spi_rx->R))
		{

		}
		else
		{
			return false;
		}
	}
	L9963_SPI_CS_UNSELECT;
	return true;
}

/****************************************************************************
 * Name:L9963_Register_Reset_value_Init
 * Parameters:NULL_PTR
 *
 *
 * Returns:NULL_PTR
 *
 *
 * Description:L9963_TEST_INIT
 *
 *
 *
 *
 *
 *
 * ****************************************************************************/
void L9963_Register_Reset_value_Init(void)
{
	uint8 i_Register_Reset_value_Init;
	for (i_Register_Reset_value_Init = 0; i_Register_Reset_value_Init < 15; i_Register_Reset_value_Init++)
	{
		L9963_Reg[i_Register_Reset_value_Init].ADCV_CONV.R = 0;
	}

}
/****************************************************************************
 * Name:L9963_Test_Init
 * Parameters:NULL_PTR
 *
 *
 * Returns:NULL_PTR
 *
 *
 * Description:L9963_TEST_INIT
 *
 *
 *
 *
 *
 *
 * ****************************************************************************/
void L9963_Test_Init(void)
{
	L9963_Register_Reset_value_Init();
#ifndef BMS_GUI
	uint8 i;
	uint8 ID[15] =
	{	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
#endif
	L9963_SPI_start();
	spi_rx_t.R = 0;

	tabcrcInit(&L9963crc_Config);

#ifdef USE_L9963T
	/*L9963 configuration*/
	while (L9963T_Init() != true){}

#elif defined(USE_DIG_ISO)
	L9963_DIG_ISO_SPI_start();
#endif
#ifndef BMS_GUI
	/*config ID when you power up the device*/
	while (!L9963_Configure_ID(L9963_DEVICE_NUMBER, ID, NULL_PTR));
	L9963_ADC_Init();
	for (i = DEVICE_ID_2; i <= L9963_DEVICE_NUMBER; i++)
	{
		L9963_GPIO_config(GPIO8, ANLOG_INPUT, i);
		L9963_GPIO_config(GPIO7, ANLOG_INPUT, i);
	}
	L9963_Reg[BROADCAST_ID].NCYCLE_PROG_2.B.VTREF_EN = 1;
	for (i = 0; i <= L9963_DEVICE_NUMBER; i++)
	{
		L9963_Single_Write(NCYCLE_PROG_2_ADD, ID[i],
				(uint32) L9963_Reg[BROADCAST_ID].NCYCLE_PROG_2.R,
				&spi_rx_t);
	}

	//open vtref

	//L9963_Reg[BROADCAST_ID].NCYCLE_PROG_2.B.VTREF_DYN_EN = 1;

	L9963_Single_Write(NCYCLE_PROG_2_ADD, BROADCAST_ID,
			(uint32) L9963_Reg[BROADCAST_ID].NCYCLE_PROG_2.R,
			&spi_rx_t);
	//L9963_Reg[BROADCAST_ID].VTREF.R =
	//0011 0110 1011 1100 10
#endif



}

/****************************************************************************
 * Name:L9963_ADC_Init
 * Parameters:NULL_PTR
 *
 *
 * Returns:NULL_PTR
 *
 *
 * Description:L9963_ADC_INIT
 *
 *
 *
 *
 *
 *
 * ****************************************************************************/
void L9963_ADC_Init(void)
{
	uint8 return_code = true;
	//L9963_SPI_start();

	/*Enable Cell connect to the chip*/
	L9963_Reg[BROADCAST_ID].VCELLS_EN.R = VCELL_EN_ALL_CELL;
	return_code &= L9963_Single_Write(VCELLS_EN_ADD, BROADCAST_ID,
			(uint32) L9963_Reg[BROADCAST_ID].VCELLS_EN.R, &spi_rx_t);
	return_code &= L9963_Single_Read(VCELLS_EN_ADD, DEVICE_ID_2, &spi_rx_t);
	L9963_Reg[DEVICE_ID_2].VCELLS_EN.R = spi_rx_t.B.DATA;

	/*Add NCYCLE SETTING
	 L9963_Reg[BROADCAST_ID].NCYCLE_PROG_1.R = 0;
	 L9963_Reg[BROADCAST_ID].NCYCLE_PROG_1.B.NCYCLE_CELL_TERM = 1;
	 L9963_Reg[BROADCAST_ID].NCYCLE_PROG_1.B.CYCLIC_UPDATE = 1;
	 L9963_Single_Write(NCYCLE_PROG_1, BROADCAST_ID,
	 (uint32) L9963_Reg[BROADCAST_ID].NCYCLE_PROG_1.R, &spi_rx_t);
	 L9963_Single_Read(NCYCLE_PROG_1, DEVICE_ID_2, &spi_rx_t);
	 L9963_Reg[DEVICE_ID_2].NCYCLE_PROG_1.R = spi_rx_t.B.DATA;
	 */
	/*	L9963_Reg[BROADCAST_ID].NCYCLE_PROG_2.R = 0;
	 L9963_Reg[BROADCAST_ID].NCYCLE_PROG_2.B.ADC_FILTER_CYCLE = 1;
	 L9963_Reg[BROADCAST_ID].NCYCLE_PROG_2.B.TCYCLE_SLP = 0;
	 L9963_Single_Write(NCYCLE_PROG_2, BROADCAST_ID,
	 (uint32) L9963_Reg[BROADCAST_ID].NCYCLE_PROG_2.R, &spi_rx_t);
	 L9963_Single_Read(NCYCLE_PROG_2, DEVICE_ID_2, &spi_rx_t);
	 L9963_Reg[DEVICE_ID_2].NCYCLE_PROG_2.R = spi_rx_t.B.DATA;*/

	/*Add CELL Over-Under voltage configuration*/
	L9963_Reg[BROADCAST_ID].VCELL_THR_ESH_UV.B.THRVcellOV = THRVcellVTR_mv(
			1700);
	L9963_Reg[BROADCAST_ID].VCELL_THR_ESH_UV.B.THRVcellUV = THRVcellVTR_mv(
			1600);
	return_code &= L9963_Single_Write(VCELL_THR_ESH_UV_OV_ADD, BROADCAST_ID,
			(uint32) L9963_Reg[BROADCAST_ID].VCELL_THR_ESH_UV.R, &spi_rx_t);
	return_code &= L9963_Single_Read(VCELL_THR_ESH_UV_OV_ADD, DEVICE_ID_2, &spi_rx_t);
	L9963_Reg[DEVICE_ID_2].VCELL_THR_ESH_UV.R = spi_rx_t.B.DATA;

	/*Add BATT Over-Under voltage configuration*/
	L9963_Reg[BROADCAST_ID].VBATT_SUM_TH.B.VBATT_SUM_OV = THRVBATTVTR_mv(2400); /* 2.4V */
	L9963_Reg[BROADCAST_ID].VBATT_SUM_TH.B.VBATT_SUM_UV = THRVBATTVTR_mv(1500); /* 1.5V */
	return_code &= L9963_Single_Write(VBATT_SUM_TH_ADD, BROADCAST_ID,
			(uint32) L9963_Reg[BROADCAST_ID].VBATT_SUM_TH.R, &spi_rx_t);
	return_code &= L9963_Single_Read(VBATT_SUM_TH_ADD, DEVICE_ID_2, &spi_rx_t);
	L9963_Reg[DEVICE_ID_2].VBATT_SUM_TH.R = spi_rx_t.B.DATA;

	L9963_Reg[BROADCAST_ID].ADCV_CONV.R = 0;
	L9963_Reg[BROADCAST_ID].ADCV_CONV.B.ADC_Filter = 0; /* ADC_Filter window 2.83ms*/
	L9963_Reg[BROADCAST_ID].ADCV_CONV.B.TCYCLE = 0; /* Sampling Cycle 200ms*/
	L9963_Reg[BROADCAST_ID].ADCV_CONV.B.CONF_CYCLIC_EN = 0;
	L9963_Reg[BROADCAST_ID].ADCV_CONV.B.HWSC = 0;
	L9963_Reg[BROADCAST_ID].ADCV_CONV.B.CYCLIC_CONTNS = 0;
	L9963_Reg[BROADCAST_ID].ADCV_CONV.B.SOC = 0;
	L9963_Reg[BROADCAST_ID].ADCV_CONV.B.DUTY_ON = 0;
	return_code &= L9963_Single_Write(ADCV_CONV_ADD, BROADCAST_ID,
			(uint32) L9963_Reg[BROADCAST_ID].ADCV_CONV.R, &spi_rx_t);

	L9963_Reg[BROADCAST_ID].CSA_GPIO_MSK.B.CoulCounter_en = 1;
	return_code &= L9963_Single_Write(CSA_GPIO_MSK_ADD, BROADCAST_ID,
			(uint32) L9963_Reg[BROADCAST_ID].CSA_GPIO_MSK.R, &spi_rx_t);

	/*L9963_Reg.ADCV_CONV.B.SOC = 0;
	 L9963_Single_Write(ADCV_CONV, (uint32) L9963_Reg.ADCV_CONV.R, &spi_rx_t);*/

	//L9963_Single_Read(ADCV_CONV_ADD, DEVICE_ID_2, &spi_rx_t);
	//L9963_Reg[DEVICE_ID_2].ADCV_CONV.R = spi_rx_t.B.DATA;
	{

		L9963_Reg[BROADCAST_ID].VCELLS_EN.R = 0x00003FFF;

		L9963_Single_Write(VCELLS_EN_ADD, BROADCAST_ID,
					(uint32) L9963_Reg[BROADCAST_ID].VCELLS_EN.R, &spi_rx_t);

		Delay(512);
	}
}

uint8 L9963T_Init(void)
{
	uint8 return_code = true;
	L9963T_SPI_start();

	SET_L9963T_ISO_LOWSPEED;
//	SET_L9963T_SLEEP_DISABLE;




	return return_code;
}

uint8 L9963_Ondemand_Conversion_start(void)
{
	uint8 error_code;
	L9963_Reg[BROADCAST_ID].ADCV_CONV.B.SOC = 1;
	error_code = L9963_Single_Write(ADCV_CONV_ADD, BROADCAST_ID,
			(uint32) L9963_Reg[BROADCAST_ID].ADCV_CONV.R, &spi_rx_t);
	return error_code;
}

uint8 L9963_Ondemand_Conversion_stop(void)
{
	uint8 error_code;
	L9963_Reg[BROADCAST_ID].ADCV_CONV.B.SOC = 0;
	error_code = L9963_Single_Write(ADCV_CONV_ADD, BROADCAST_ID,
			(uint32) L9963_Reg[BROADCAST_ID].ADCV_CONV.R, &spi_rx_t);
	return error_code;
}

/****************************************************************************
 * Name:L9963_Configure_ID
 * Parameters:NULL_PTR
 *
 *
 * Returns:NULL_PTR
 *
 *
 * Description:L9963_TEST_INIT
 *
 *
 *
 *
 *
 *
 * ****************************************************************************/
uint8 L9963_Configure_ID(uint8 totalID, uint8 *ID, uint8* ACK)
{
	uint32 temp_register;
	uint8 i;
	uint8 error_time = 0;
	uint8 return_code = true;
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
						(uint32) L9963_Reg[i + 1].DEV_GEN_CFG.R, &spi_rx_t);
#else
		return_code &= L9963_Single_Write(GEN_STATUS_ADD, BROADCAST_ID,
				(uint32) L9963_Reg[i + 1].DEV_GEN_CFG.R, &spi_rx_t);
#endif
		Delay(2519); /*Total Delay time need to be 219us */

		L9963_Reg[i + 1].Bal_1_t.B.comm_timeout_dis = 0; /* Set the device always wake up*/
		L9963_Reg[BROADCAST_ID].Bal_1_t.B.comm_timeout_dis = 0; /* Set the device always wake up*/
#ifdef USE_L9963T
		L9963_Single_Write(BAL_1_ADD, *(ID + i),
				(uint32) L9963_Reg[i + 1].Bal_1_t.R, &spi_rx_t);
#else
		return_code &= L9963_Single_Write(BAL_1_ADD, *(ID + i),
				(uint32) L9963_Reg[i + 1].Bal_1_t.R, &spi_rx_t);
#endif
		Delay(2519);

// Add Liyi 20200522
/*
		L9963_Reg[i + 1].FASTCH_BALUV.B.CommTimOut = 1;
		L9963_Reg[BROADCAST_ID].FASTCH_BALUV.B.CommTimOut = 1;
#ifdef USE_L9963T
		L9963_Single_Write(FASTCH_BAL_UV_ADD, *(ID + i),
				(uint32) L9963_Reg[i + 1].FASTCH_BALUV.R, &spi_rx_t);
#else
		return_code &= L9963_Single_Write(FASTCH_BAL_UV_ADD, *(ID + i),
				(uint32) L9963_Reg[i + 1].FASTCH_BALUV.R, &spi_rx_t);
#endif
		Delay(2519);
		*/
//end add
		return_code &= L9963_Single_Read(GEN_STATUS_ADD, *(ID + i), &spi_rx_t); /* check the return code */
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
	if( return_code == true)
	{
		L9963_Reg[BROADCAST_ID].DEV_GEN_CFG.B.isotx_en_h = 1;
		L9963_Reg[BROADCAST_ID].DEV_GEN_CFG.B.out_res_tx_iso = 3;
		L9963_Reg[BROADCAST_ID].DEV_GEN_CFG.B.iso_freq_sel = 3; /* Set the ISO frequency to High*/
		return_code &= L9963_Single_Write(GEN_STATUS_ADD, BROADCAST_ID,
				(uint32) L9963_Reg[BROADCAST_ID].DEV_GEN_CFG.R, &spi_rx_t);
		//SET_L9963T_ISO_HIGHSPEED;
		Delay(2519);                               /* L9963T need a delay between ISO_SPEED_HIGH and sleep enable*/
		L9963_Single_Read(GEN_STATUS_ADD, 1, &spi_rx_t); /* check the return code */
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

/****************************************************************************
 * Name:L9963_Clear_ID
 * Parameters:NULL_PTR
 *
 *
 * Returns:NULL_PTR
 *
 *
 * Description:L9963_Clear_ID
 *
 *
 *
 * ****************************************************************************/
uint8 L9963_Clear_ID(uint8 totalID, uint8 *ID, uint8 *ACK)
{
	uint8 i;
	uint8 return_code = true;
	for (i = totalID; i > 0; i--)
	{
		L9963_Reg[i].FSM_t.B.SW_RST = 2; /* when written to 10 it resets all reg map registers */
		return_code &= L9963_Single_Write(FSM_ADD, *(ID + i - 1),
				(uint32) L9963_Reg[i].FSM_t.R, &spi_rx_t);
		Delay(6219); /*Total Delay time need to be 219us */
		if (return_code == true) /* Judge if configure successful*/
		{
			*ACK = 0x01;
			SET_L9963T_ISO_LOWSPEED;
		}
		else
		{
			*ACK = 0x00;
		}
	}

	return return_code;

}

/****************************************************************************
 * Name:L9963_Clear_ID
 * Parameters:NULL_PTR
 *
 *
 * Returns:NULL_PTR
 *
 *
 * Description:L9963_Clear_ID
 *
 *
 *
 * ****************************************************************************/
L9963_SPI_Rx_Inst_t spi_rx_read;
uint32 L9966_DREADY_ERROR = 0;
uint8 L9963_Register_Update(L9963_SPI_Rx_Inst_t *spi_rx, uint8 CMD,
		uint8 Device_ID)
{
	uint8 i = 0;

	spi_rx_read.R=0;
	spi_rx_read.B= spi_rx->B;

	switch (CMD)
	{
	case BURST_78CMD:
		switch (spi_rx->B.ADDRESS)
		{
		case BURST_78CMD:
			L9963_Reg[Device_ID].VCELL1.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL1.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME2:
			L9963_Reg[Device_ID].VCELL2.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL2.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME3:
			L9963_Reg[Device_ID].VCELL3.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL3.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME4:
			L9963_Reg[Device_ID].VCELL4.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL4.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME5:
			L9963_Reg[Device_ID].VCELL5.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL5.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME6:
			L9963_Reg[Device_ID].VCELL6.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL6.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME7:
			L9963_Reg[Device_ID].VCELL7.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL7.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME8:
			L9963_Reg[Device_ID].VCELL8.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL8.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME9:
			L9963_Reg[Device_ID].VCELL9.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL9.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME10:
			L9963_Reg[Device_ID].VCELL10.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL10.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME11:
			L9963_Reg[Device_ID].VCELL11.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL11.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME12:
			L9963_Reg[Device_ID].VCELL12.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL12.B.D_READY == 0)
			{
				L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME13:
			L9963_Reg[Device_ID].VCELL13.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL13.B.D_READY == 0)
			{
				//L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME14:
			L9963_Reg[Device_ID].VCELL14.R = spi_rx->B.DATA;
			if (L9963_Reg[Device_ID].VCELL14.B.D_READY == 0)
			{
				//L9966_DREADY_ERROR += 1;
			}
			break;
		case FRAME15:
			L9963_Reg[Device_ID].VSUMBAT.R = spi_rx->B.DATA;
			break;
		case FRAME16:
			L9963_Reg[Device_ID].VBATTDIV.R = spi_rx->B.DATA;
			break;
		case FRAME17:
			break;
		case FRAME18:
			L9963_Reg[Device_ID].Ibattery_cali.R = spi_rx->B.DATA;
			break;
		default:
			break;

		}
		break;

	case BURST_7ACMD:
		switch (spi_rx->B.ADDRESS)
		{
		case BURST_7ACMD:
			break;
		case FRAME3:
			BURST_7A_FRAME3_BALOPEN(spi_rx->B.DATA,
					L9963_Reg[Device_ID].BAL_OPEN.R)
			break;
		case FRAME4:
			BURST_7A_FRAME4_BALSHORT(spi_rx->B.DATA,
					L9963_Reg[Device_ID].BAL_SHORT.R)
			break;
		case FRAME5:
			BURST_7A_FRAME5_VCELLOPEN(spi_rx->B.DATA,
					L9963_Reg[Device_ID].CELL_OPEN.R)
			break;
		case FRAME6:
			BURST_7A_FRAME6_VCELLUV(spi_rx->B.DATA,
					L9963_Reg[Device_ID].VCELL_UV.R)
			break;
		case FRAME7:
			BURST_7A_FRAME7_VCELLOV(spi_rx->B.DATA,
					L9963_Reg[Device_ID].VCELL_OV.R)

			BURST_7A_FRAME7_VCELLUV(spi_rx->B.DATA,
					L9963_Reg[Device_ID].VCELL_UV.R)
			break;
		case FRAME8:
			BURST_7A_FRAME8_VCELLOV(spi_rx->B.DATA,
					L9963_Reg[Device_ID].VCELL_OV.R)
			BURST_7A_FRAME8_VGPIOOTUT(spi_rx->B.DATA,
					L9963_Reg[Device_ID].VGPIO_OT_UT.R)
			BURST_7A_FRAME8_BalCell6_1act_t(spi_rx->B.DATA,
					L9963_Reg[Device_ID].BalCell6_1act_t.R)
			break;
		case FRAME17:
			break;
		default:
			break;
		}
		break;
	case BURST_7BCMD:
		switch (spi_rx->B.ADDRESS)
		{
		case BURST_7BCMD:
			break;
		case FRAME6:
			BURST_7B_FRAME6_13_GPIO(spi_rx->B.DATA,
					L9963_Reg[Device_ID].GPIO3_MEAS.R)
			break;
		case FRAME7:
			BURST_7B_FRAME6_13_GPIO(spi_rx->B.DATA,
					L9963_Reg[Device_ID].GPIO4_MEAS.R)
			break;
		case FRAME8:
			BURST_7B_FRAME6_13_GPIO(spi_rx->B.DATA,
					L9963_Reg[Device_ID].GPIO5_MEAS.R)
			break;
		case FRAME9:
			BURST_7B_FRAME6_13_GPIO(spi_rx->B.DATA,
					L9963_Reg[Device_ID].GPIO6_MEAS.R)
			break;
		case FRAME10:
			BURST_7B_FRAME6_13_GPIO(spi_rx->B.DATA,
					L9963_Reg[Device_ID].GPIO7_MEAS.R)
			break;
		case FRAME11:
			BURST_7B_FRAME6_13_GPIO(spi_rx->B.DATA,
					L9963_Reg[Device_ID].GPIO8_MEAS.R)
			break;
		case FRAME12:
			BURST_7B_FRAME6_13_GPIO(spi_rx->B.DATA,
					L9963_Reg[Device_ID].GPIO9_MEAS.R)
			break;
		default:
			break;
		}
		break;
	default: /*Not burst directly write register*/
		for (i = total_id; i > 0; i--)
		{
			if (spi_rx->B.DEVID == id[i])
			{
				i += 1;
				break;
			}

		}
		switch (spi_rx->B.ADDRESS)
		{
		case GEN_STATUS_ADD:
			L9963_Reg[i+1].DEV_GEN_CFG.R = spi_rx->B.DATA;
			break;
		case FASTCH_BAL_UV_ADD:
			L9963_Reg[i+1].FASTCH_BALUV.R = spi_rx->B.DATA;
			break;
		case ADCV_CONV_ADD:
			L9963_Reg[i+1].ADCV_CONV.R = spi_rx->B.DATA;
			break;
		case Ibattery_synch_ADD:
			L9963_Reg[i+1].Ibattery_sync.R = spi_rx->B.DATA;
			break;
		case Ibattery_calib_ADD:
			L9963_Reg[i+1].Ibattery_cali.R = spi_rx->B.DATA;
			break;
		case CoulCntrmsb_ADD:
			L9963_Reg[i+1].CoulCntr_msb.R = spi_rx->B.DATA;
			break;
		case CoulCntrlsb_ADD:
			L9963_Reg[i+1].CoulCntr_lsb.R = spi_rx->B.DATA;
			break;
		case GPIO3_MEAS_ADD:
			L9963_Reg[i+1].GPIO3_MEAS.R = spi_rx->B.DATA;
			break;
		case GPIO4_MEAS_ADD:
			L9963_Reg[i+1].GPIO4_MEAS.R = spi_rx->B.DATA;
			break;
		case GPIO5_MEAS_ADD:
			L9963_Reg[i+1].GPIO5_MEAS.R = spi_rx->B.DATA;
			break;
		case GPIO6_MEAS_ADD:
			L9963_Reg[i+1].GPIO6_MEAS.R = spi_rx->B.DATA;
			break;
		case GPIO7_MEAS_ADD:
			L9963_Reg[i+1].GPIO7_MEAS.R = spi_rx->B.DATA;
			break;
		case GPIO8_MEAS_ADD:
			L9963_Reg[i+1].GPIO8_MEAS.R = spi_rx->B.DATA;
			break;
		case GPIO9_MEAS_ADD:
			L9963_Reg[i+1].GPIO9_MEAS.R = spi_rx->B.DATA;
			break;
		default:
			break;
		}
		break;
	}
	return true;
}


/****************************************************************************
 * Name:L9963_GPIO_config
 * Parameters:NULL_PTR
 *
 *
 * Returns:NULL_PTR
 *
 *
 * Description:L9963_GPIO_config
 *
 *
 *
 * ****************************************************************************/
uint8 L9963_GPIO_config(GPIO_CHANNEL gpio_channel, GPIO_MODE1 gpio_mode,
		uint8 device_id)
{
	uint8 return_code = true;
	L9963_Reg[device_id].GPIO9_3_CONF.R = gpio_mode
			<< (2 * (gpio_channel - GPIO3));
	return_code &= L9963_Single_Write(GPIO9_3_CONF_ADD, device_id,
			(uint32) L9963_Reg[device_id].GPIO9_3_CONF.R, &spi_rx_t);

	return return_code;
}


uint8 Ascout[3600] =
{ 0 };

uint8 L9963_Test_Voltage(void)
{
	int i=0;
	uint8 result=0;
	/* Send SOC =1 device by device */
	for (i = 0; i < L9963_DEVICE_NUMBER; i++)
	{
		L9963_Reg[i + 1].ADCV_CONV.B.SOC = 1;
		L9963_Single_Write(ADCV_CONV_ADD, id[i], (uint32) L9963_Reg[i + 1].ADCV_CONV.R, &spi_rx_t);
		Delay(2000); /*Total Delay time need to be 419 */
	}
	/*====Ascin[4] Reserve(5bytes)=======================================*/
	/*====Ascout[4] I/O status Register(1bytes)==========================*/
	/*====Ascout[5] MISO ACK (1bytes)====================================*/
	/*====Ascout[6]-[10]Reserved Register(5bytes)========================*/
	/*====Ascout[11]-Ascout[100] 0x78 Burst read bytes(90bytes)================*/
	/*====Ascout[101]-Ascout[165] 0x7A Burst read bytes(65bytes)================*/
	/*====Ascout[166]-Ascout[235] 0x7B Burst read bytes(70bytes)================*/
	result = 1;//succ
	for (i = 0; i < L9963_DEVICE_NUMBER; i++)
	{
		/*=============Burst Read command ================*/
		if (L9963_Burst_Read(BURST_78CMD, id[i], &Ascout[11 + 225 * i], BURST_78CMD_SIZE) == false)
		{
			result = 0;//fail
		}
		else
		{

		}
#if 0
		if (L9963_Burst_Read(BURST_7ACMD, id[i],
				&Ascout[101 + 225 * i], BURST_7ACMD_SIZE) == false)
		{
			result = 0;//fail
		}
		else
		{

		}
#endif
		if (L9963_Burst_Read(BURST_7BCMD, id[i],
				&Ascout[166 + 225 * i], BURST_7BCMD_SIZE) == false)
		{
			result = 0;//fail
		}
		else
		{

		}

	}
	Delay(2000);
	L9963_Reg[BROADCAST_ID].ADCV_CONV.B.SOC = 0;
	L9963_Single_Write(ADCV_CONV_ADD, BROADCAST_ID,
			(uint32) L9963_Reg[BROADCAST_ID].ADCV_CONV.R,
			&spi_rx_t);
	return result;
}

uint8 L9963_Test_Cell_Temp(void)
{
	int i=0;
	uint8 result=0;
	/* Send SOC =1 device by device */
	for (i = 0; i < L9963_DEVICE_NUMBER; i++)
	{
		L9963_Reg[i + 1].ADCV_CONV.B.SOC = 1;
		L9963_Single_Write(ADCV_CONV_ADD, id[i],
				(uint32) L9963_Reg[i + 1].ADCV_CONV.R, &spi_rx_t);
		Delay(2000); /*Total Delay time need to be 419 */
	}
	/*====Ascin[4] Reserve(5bytes)=======================================*/
	/*====Ascout[4] I/O status Register(1bytes)==========================*/
	/*====Ascout[5] MISO ACK (1bytes)====================================*/
	/*====Ascout[6]-[10]Reserved Register(5bytes)========================*/
	/*====Ascout[11]-Ascout[100] 0x78 Burst read bytes(90bytes)================*/
	/*====Ascout[101]-Ascout[165] 0x7A Burst read bytes(65bytes)================*/
	/*====Ascout[166]-Ascout[235] 0x7B Burst read bytes(70bytes)================*/
	result = 1;//succ

	for (i = 0; i < L9963_DEVICE_NUMBER; i++)
	{
		/*=============Burst Read command ================*/
#if 0
		if (L9963_Burst_Read(BURST_78CMD, id[i],
				&Ascout[11 + 225 * i], BURST_78CMD_SIZE) == false)
		{
			result = 0;//fail
		}
		else
		{

		}

		if (L9963_Burst_Read(BURST_7ACMD, id[i],
				&Ascout[101 + 225 * i], BURST_7ACMD_SIZE) == false)
		{
			result = 0;//fail
		}
		else
		{

		}
#endif
		if (L9963_Burst_Read(BURST_7BCMD, id[i],
				&Ascout[166 + 225 * i], BURST_7BCMD_SIZE) == false)
		{
			result = 0;//fail
		}
		else
		{

		}

	}
	Delay(2000);
	L9963_Reg[BROADCAST_ID].ADCV_CONV.B.SOC = 0;
	L9963_Single_Write(ADCV_CONV_ADD, BROADCAST_ID,
			(uint32) L9963_Reg[BROADCAST_ID].ADCV_CONV.R,
			&spi_rx_t);
	return result;
}

void L9963_Test_Temp(void)
{
	int i=0;
	for (i = 0; i < L9963_DEVICE_NUMBER; i++)
	{
		L9963_Single_Read_temp(TempChip_ADD,id[i],&spi_rx_t);
		L9963_Reg[id[i]].TempChip.R = spi_rx_t.B.DATA;
	}
}

//#include "Can.h"
//Can_PduType CanMessageTest ;
//uint8 CanData[8] = {0x0};
//void L9963_CAN_Transmit(void)
//{
//
//	CanMessageTest.id = 0x600U;
//	CanMessageTest.length = 8;
//	Can_SetControllerMode(CanConf_CanController_L9963_CANNODE_0,CAN_T_START);
//				Can_MainFunction_Mode();
//	uint8 i = 0;
//	for(i=0;i<L9963_DEVICE_NUMBER;i++)
//	{
//
////		*((uint32*)(CanData)) = L9963_Reg[*(ID+i)].VCELL1.B.VOLT
////							|((L9963_Reg[*(ID+i)].VCELL2.B.VOLT)<<16);
////		*((uint32*)(CanData+4)) = L9963_Reg[*(ID+i)].VCELL3.B.VOLT
////							|((L9963_Reg[*(ID+i)].VCELL4.B.VOLT)<<16);
//		CanData[0] = ((L9963_Reg[*ID+i].VCELL1.B.VOLT)&0xff);
//		CanData[1] = ((L9963_Reg[*ID+i].VCELL1.B.VOLT)>>8);
//		CanData[2] = ((L9963_Reg[*ID+i].VCELL2.B.VOLT)&0xff);
//		CanData[3] = ((L9963_Reg[*ID+i].VCELL2.B.VOLT)>>8);
//		CanData[4] = ((L9963_Reg[*ID+i].VCELL3.B.VOLT)&0xff);
//		CanData[5] = ((L9963_Reg[*ID+i].VCELL3.B.VOLT)>>8);
//		CanData[6] = ((L9963_Reg[*ID+i].VCELL4.B.VOLT)&0xff);
//		CanData[7] = ((L9963_Reg[*ID+i].VCELL4.B.VOLT)>>8);
//		CanMessageTest.sdu = CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_1,&CanMessageTest);
//
//		CanMessageTest.id++;
//
////		*((uint32*)(CanData)) = L9963_Reg[*(ID+i)].VCELL5.B.VOLT
////							|((L9963_Reg[*(ID+i)].VCELL6.B.VOLT)<<16);
////		*((uint32*)(CanData+4)) = L9963_Reg[*(ID+i)].VCELL7.B.VOLT
////							|((L9963_Reg[*(ID+i)].VCELL8.B.VOLT)<<16);
//		CanData[0] = ((L9963_Reg[*ID+i].VCELL5.B.VOLT)&0xff);
//		CanData[1] = ((L9963_Reg[*ID+i].VCELL5.B.VOLT)>>8);
//		CanData[2] = ((L9963_Reg[*ID+i].VCELL6.B.VOLT)&0xff);
//		CanData[3] = ((L9963_Reg[*ID+i].VCELL6.B.VOLT)>>8);
//		CanData[4] = ((L9963_Reg[*ID+i].VCELL7.B.VOLT)&0xff);
//		CanData[5] = ((L9963_Reg[*ID+i].VCELL7.B.VOLT)>>8);
//		CanData[6] = ((L9963_Reg[*ID+i].VCELL8.B.VOLT)&0xff);
//		CanData[7] = ((L9963_Reg[*ID+i].VCELL8.B.VOLT)>>8);
//		CanMessageTest.sdu = CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_2,&CanMessageTest);
//
//		CanMessageTest.id++;
//
////		*((uint32*)(CanData)) = L9963_Reg[*(ID+i)].VCELL9.B.VOLT
////							|((L9963_Reg[*(ID+i)].VCELL10.B.VOLT)<<16);
////		*((uint32*)(CanData+4)) = L9963_Reg[*(ID+i)].VCELL11.B.VOLT
////							|((L9963_Reg[*(ID+i)].VCELL12.B.VOLT)<<16);
//		CanData[0] = ((L9963_Reg[*ID+i].VCELL9.B.VOLT)&0xff);
//		CanData[1] = ((L9963_Reg[*ID+i].VCELL9.B.VOLT)>>8);
//		CanData[2] = ((L9963_Reg[*ID+i].VCELL10.B.VOLT)&0xff);
//		CanData[3] = ((L9963_Reg[*ID+i].VCELL10.B.VOLT)>>8);
//		CanData[4] = ((L9963_Reg[*ID+i].VCELL11.B.VOLT)&0xff);
//		CanData[5] = ((L9963_Reg[*ID+i].VCELL11.B.VOLT)>>8);
//		CanData[6] = ((L9963_Reg[*ID+i].VCELL12.B.VOLT)&0xff);
//		CanData[7] = ((L9963_Reg[*ID+i].VCELL12.B.VOLT)>>8);
//		CanMessageTest.sdu = CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_3,&CanMessageTest);
//		CanMessageTest.id++;
//
////		*((uint32*)(CanData)) = L9963_Reg[*(ID+i)].VCELL13.B.VOLT
////							|((L9963_Reg[*(ID+i)].VCELL14.B.VOLT)<<16);
////		*((uint32*)(CanData+4)) = L9963_Reg[*(ID+i)].VSUMBAT.B.VSUM_BAT19_2
////							|((L9963_Reg[*(ID+i)].VBATTDIV.B.VBATTDIV)<<16);
//		CanData[0] = ((L9963_Reg[*ID+i].VCELL13.B.VOLT)&0xff);
//		CanData[1] = ((L9963_Reg[*ID+i].VCELL13.B.VOLT)>>8);
//		CanData[2] = ((L9963_Reg[*ID+i].VCELL14.B.VOLT)&0xff);
//		CanData[3] = ((L9963_Reg[*ID+i].VCELL14.B.VOLT)>>8);
//		CanData[4] = ((L9963_Reg[*ID+i].VSUMBAT.B.VSUM_BAT19_2)&0xff);
//		CanData[5] = ((L9963_Reg[*ID+i].VSUMBAT.B.VSUM_BAT19_2)>>8);
//		CanData[6] = ((L9963_Reg[*ID+i].VBATTDIV.B.VBATTDIV)&0xff);
//		CanData[7] = ((L9963_Reg[*ID+i].VBATTDIV.B.VBATTDIV)>>8);
//		CanMessageTest.sdu = CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_4,&CanMessageTest);
//		CanMessageTest.id++;
//
//		*((uint32*)(CanData)) = ((L9963_Reg[*(ID+i)].TempChip.B.TempChip)<<24);
//		*((uint32*)(CanData+4)) = 0;
//
//		CanMessageTest.sdu = CanData;
//		Can_Write(CanConf_CanHardwareObject_Can_Network_CANNODE_2_Tx_Std_MailBox_5,&CanMessageTest);
//		CanMessageTest.id = (CanMessageTest.id - 4 + 0x10);
//
//	}
//}
