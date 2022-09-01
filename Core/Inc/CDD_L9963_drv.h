/*
 L9963_driver header only for BA - Copyright (C) 2018 STMicroelectronics

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
#ifndef L9963_DRV_H
#define L9963_DRV_H
#include "stm32h7xx_hal.h"
#include "typedefs.h"
#include "CDD_L9963_crc.h"
#include "CDD_L9963_LLD.h"
//#include "core_cm7.h"
//#include "Dio.h"
#define L9963_SPI_CS_SELECT          	  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
#define L9963_SPI_CS_UNSELECT          		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

#define SET_L9963T_ISO_LOWSPEED               HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
#define SET_L9963T_ISO_HIGHSPEED              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);


//#define SET_L9963T_SLEEP_DISABLE            Dio_WriteChannel(DioConf_DioChannel_DO_DCh_EN,0)
//#define SET_L9963T_SLEEP_ENABLE             Dio_WriteChannel(DioConf_DioChannel_DO_DCh_EN,1)

#define L9963T_BNE_STATUS                   0

#define SPEEDHIGH                           1
#define SPEEDLOW                            0
#define SLEEPDISABLE                        0
#define SLEEPENABLE                         1

#define READ_L9963                          0
#define WRITE_L9963 		                1
#define RW_MASK                             0xBF

/*Data Exchange format Over SPI bus*/
typedef volatile unsigned long long   vuint64_t;      /**< Volatile unsigned 64 bits. */
#define true		 0
#define false 		1

/*==========================Regsiter Mask===================================*/
#define VCELL_EN_ALL_CELL                   0x3fff
/* Vcell Threshold volage to regsiter value*/
#define THRVcellVTR_mv(mv)                  (mv/22.8)

/* VBATT Threshold volage to regsiter value*/
#define THRVBATTVTR_mv(mv)                  (mv/369.5)
/* VGPIO measurement conversion */
#define VGPIORto_V(registervalue)            (float)((float)(registervalue)*(float)89/(float)1000000)      /*Register value to V*/
#define VGPIORto_mV(registervalue)           (float)((float)(registervalue)*(float)89/(float)1000)     /*Register value to mV*/
/* VISENSE measurement conversion */
#define ICELLSENSERto_A(registervalue)            (float)((float)Calculate_original_data_18bit(registervalue)*(float)1.33/(float)0.1/1000)      /*Register value to A*/
#define ICELLSENSERto_mA(registervalue)           (float)((float)Calculate_original_data_18bit(registervalue)*(float)1.33/(float)0.1)      /*Register value to mA*/
/* VCELL measurement conversion */
#define VCELLRto_V(registervalue)            (float)( (float)(registervalue)*(float)89/(float)1000000)      /*Register value to V*/
#define VCELLRto_mV(registervalue)           (float)( (float)(registervalue)*(float)89/(float)1000)         /*Register value to mV*/
/* Coulumb measurement conversion */
#define DeltaTime                           328.25
#define Coulumb_AH(VCoulumb)               ((double)Calculate_original_data_32bit(VCoulumb)*DeltaTime*(double)1.33/(double)0.1/1000)

/*==========================================================================*/
#define CELL_BALANCE_START                  2
#define CELL_BALANCE_STOP                   1

/*==============L9963_BURST_CMD_MASK=============================*/
#define BURST_7A_FRAME3_BALOPEN(spi_data,L9963_register)             L9963_register |= (((spi_data)&0x03FFF)<<2);L9963_register &= (((spi_data)|(~0x03FFF))<<2);
#define BURST_7A_FRAME4_BALSHORT(spi_data,L9963_register)            L9963_register |= (((spi_data)&0x03FFF)<<2);L9963_register &= (((spi_data)|(~0x03FFF))<<2);
#define BURST_7A_FRAME5_VCELLOPEN(spi_data,L9963_register)           L9963_register |= ((spi_data)&0x03FFF);L9963_register &= ((spi_data)|(~0x03FFF));
#define BURST_7A_FRAME6_VCELLUV(spi_data,L9963_register)             L9963_register |= ((spi_data)&0x03FFF);L9963_register &= ((spi_data)|(~0x03FFF));
#define BURST_7A_FRAME7_VCELLOV(spi_data,L9963_register)             L9963_register |= ((spi_data)&0x03FFF);L9963_register &= ((spi_data)|(~0x03FFF));\
																				 L9963_register |= (((spi_data)&0x20000)>>1);L9963_register &= (((spi_data)|(~0x20000)>>1));
#define BURST_7A_FRAME7_VCELLUV(spi_data,L9963_register)             L9963_register |= ((spi_data)&0x1C000);L9963_register &= ((spi_data)|(~0x1C000));
#define BURST_7A_FRAME8_VCELLOV(spi_data,L9963_register)             L9963_register |= ((spi_data)&0x0C000);L9963_register &= ((spi_data)|(~0x0C000));
#define BURST_7A_FRAME8_VGPIOOTUT(spi_data,L9963_register)           L9963_register |= ((spi_data)&0x03FFF);L9963_register &= ((spi_data)|(~0x03FFF));
#define BURST_7A_FRAME8_BalCell6_1act_t(spi_data,L9963_register)     L9963_register |= (((spi_data)&0x30000)>>16);L9963_register &= (((spi_data)|(~0x30000)>>16));

#define BURST_7B_FRAME6_13_GPIO(spi_data,L9963_register)             L9963_register |= ((spi_data)&0x1FFFF);L9963_register &= ((spi_data)|(~0x1FFFF));
/*==============END=============================================*/
//typedef union L9963_SPI_Tx_Inst_tag
//{
//	vuint64_t R;
//	struct
//	{
//		vuint64_t RESERVED :24;
//		vuint64_t PA :1;
//		vuint64_t RW :1;
//		vuint64_t DEVID :5;
//		vuint64_t ADDRESS :7;
//		vuint64_t GSW :2;
//		vuint64_t DATA :18;
//		vuint64_t CRC_1 :6;

//	//	uint32_t W_DATA :24;
//	} B;
//} L9963_SPI_Tx_Inst_t;


//typedef union L9963_SPI_Rx_Inst_tag
//{
//	vuint64_t R;
//	struct
//	{
//		vuint64_t RESERVED :24;
//		vuint64_t PA :1;
//		vuint64_t BURST :1;
//		vuint64_t DEVID :5;
//		vuint64_t ADDRESS :7;
//		vuint64_t GSW :2;
//		vuint64_t DATA :18;
//		vuint64_t CRC_1 :6;

//	//	uint32_t W_DATA :24;
//	} B;
//} L9963_SPI_Rx_Inst_t;





typedef union L9963_SPI_Tx_Inst_tag
{
	vuint64_t R;
	struct
	{
		
		vuint64_t CRC_1 :6;
		vuint64_t DATA :18;
		vuint64_t GSW :2;
		vuint64_t ADDRESS :7;
		vuint64_t DEVID :5;
		vuint64_t RW :1;
		vuint64_t PA :1;
		vuint64_t RESERVED :24;



		
	
	
		

	//	uint32_t W_DATA :24;
	} B;
} L9963_SPI_Tx_Inst_t;


typedef union L9963_SPI_Rx_Inst_tag
{
	vuint64_t R;
	struct
	{
		vuint64_t CRC_1 :6;
		vuint64_t DATA :18;
		vuint64_t GSW :2;
		vuint64_t ADDRESS :7;
		vuint64_t DEVID :5;
		vuint64_t BURST :1;
		vuint64_t PA :1;
		vuint64_t RESERVED :24;
		

		

	//	uint32_t W_DATA :24;
	} B;
} L9963_SPI_Rx_Inst_t;





//typedef union L9963_SPI_Tx_Inst_tag
//{
//	vuint64_t R;
//	struct
//	{
//		uint32_t RESERVED :24;
//		uint32_t PA :1;
//		uint32_t RW :1;
//		uint32_t DEVID :5;
//		uint32_t ADDRESS :7;
//		uint32_t GSW :2;
//		uint32_t DATA :18;
//		uint32_t CRC_1 :6;

//	//	uint32_t W_DATA :24;
//	} B;
//} L9963_SPI_Tx_Inst_t;



//typedef union L9963_SPI_Rx_Inst_tag
//{
//	
//	vuint64_t R;
//	struct
//	{
//		uint32_t RESERVED :24;
//		uint32_t PA :1;
//		uint32_t BURST :1;
//		uint32_t DEVID :5;
//		uint32_t ADDRESS :7;
//		uint32_t GSW :2;
//		uint32_t DATA :18;
//		uint32_t CRC_1 :6;

//	} B;
//} L9963_SPI_Rx_Inst_t;





typedef struct L9963_Reg_tag
{
	/*******************************************************/
	/*          0x01     GEN_STATUS                        */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t Fault_for_CE :1;
			uint32_t Farthest_Unit :1;
			uint32_t HeartBeat_EN :1;
			uint32_t FaultH_EN :1;
		  uint32_t HeartBeat :3;
		  uint32_t Noreg7 :1;
	    uint32_t iso_freq_sel :2;
			uint32_t out_res_tx_iso :2;
			uint32_t isotx_en_h :1;
		  uint32_t chipid :5; /*Chip_ID*/
      uint32_t reserved :14;
	
			
		} B;
	} DEV_GEN_CFG;
	/*******************************************************/
	/*           0x02    FASTCH_BALuv                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t CommTimOut :2;
			uint32_t Gpio_OT_delta :8;
			uint32_t Vcell_bal_UV_delta :8;
		} B;
	} FASTCH_BALUV;
	/*******************************************************/
	/*             0x03      BAL1                          */
	/*******************************************************/
	//0x03 Bal_1 bit fields
	union
	{
		uint32_t R;
		struct
		{	
			
			uint32_t WDTimedBalTimer :7; //RO during timed balanging it's the wd counter echo
			uint32_t TimedBalTimer :7; //RO during timed balanging it's the counter echo
			uint32_t bal_stop :1; //RW  set to 01 stops the balancing
			uint32_t bal_start :1; //RW set to 10 starts balancing if balancing status is IDLE
			uint32_t slp_bal_conf :1; //RW  Must be 1 to have silent balancing when device is going to sleep
			uint32_t comm_timeout_dis :1; //RW  it enables the Hbeat generation
	
			uint32_t res :14; //NA reserved
		
		} B;
	} Bal_1_t;
	/*******************************************************/
	/*             0x04      BAL2                          */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; //NA reserved
			uint32_t Balmode :2; //RW  Balance mode
			uint32_t TimedBalacc :1; //RW  it sets the accuracy of timed balancing
			uint32_t ThrTimedBalCell14 :7; //timethreshold for timed cell14 balancing
			uint32_t Noreg :1; //RO
			uint32_t ThrTimedBalCell13 :7; //timethreshold for timed cell13 balancing
		} B;
	} Bal_2_t;
	/*******************************************************/
	/*             0x05      BAL3                          */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; //NA reserved
			uint32_t first_wup_done :1; //RO
			uint32_t trimming_retrigger :1; //RW
			uint32_t Lock_isoh_freq :1; //RW
			uint32_t ThrTimedBalCell12 :7; //timethreshold for timed cell12 balancing
			uint32_t Noreg7 :1; //RO
			uint32_t ThrTimedBalCell11 :7; //timethreshold for timed cell11 balancing
		} B;
	} Bal_3_t;
	/*******************************************************/
	/*             0x06      BAL4                          */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; //NA reserved
			uint32_t clk_mon_en :1; //RW
			uint32_t noreg16 :1; //RW
			uint32_t clk_mon_init_done :1; //RO
			uint32_t ThrTimedBalCell10 :7; //timethreshold for timed cell10 balancing
			uint32_t Noreg7 :1; //RO
			uint32_t ThrTimedBalCell9 :7; //timethreshold for timed cell9 balancing
		} B;
	} Bal_4_t;
	/*******************************************************/
	/*             0x07      BAL5                          */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; //NA reserved
			uint32_t trans_on :1; //RW
			uint32_t trans_vaild :1; //RW
			uint32_t Noreg15 :1; //RO
			uint32_t ThrTimedBalCell8 :7; //timethreshold for timed cell8 balancing
			uint32_t Noreg7 :1; //RO
			uint32_t ThrTimedBalCell7 :7; //timethreshold for timed cell7 balancing
		} B;
	} Bal_5_t;
	/*******************************************************/
	/*             0x08      BAL6                          */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; //NA reserved
			uint32_t noreg17 :1; //RO
			uint32_t noreg16 :1; //RO
			uint32_t noreg15 :1; //RO
			uint32_t ThrTimedBalCell6 :7; //timethreshold for timed cell6 balancing
			uint32_t Noreg7 :1; //RO
			uint32_t ThrTimedBalCell5 :7; //timethreshold for timed cell5 balancing
		} B;
	} Bal_6_t;
	/*******************************************************/
	/*             0x09      BAL7                          */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; //NA reserved
			uint32_t noreg17 :1; //RO
			uint32_t noreg16 :1; //RO
			uint32_t noreg15 :1; //RO
			uint32_t ThrTimedBalCell4 :7; //timethreshold for timed cell4 balancing
			uint32_t Noreg7 :1; //RO
			uint32_t ThrTimedBalCell3 :7; //timethreshold for timed cell3 balancing
		} B;
	} Bal_7_t;
	/*******************************************************/
	/*             0x0A      BAL8                          */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; //NA reserved
			uint32_t noreg17 :1; //RO
			uint32_t noreg16 :1; //RO
			uint32_t noreg15 :1; //RO
			uint32_t ThrTimedBalCell2 :7; //timethreshold for timed cell2 balancing
			uint32_t Noreg8 :1; //RO
			uint32_t ThrTimedBalCell1 :7; //timethreshold for timed cell1 balancing
		} B;
	} Bal_8_t;
	/*******************************************************/
	/*             0x0B VCELL_THR_ESH_UV                   */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t THRVcellUV :8;
			uint32_t THRVcellOV :8;
			uint32_t reserved :16;
			
		} B;
	} VCELL_THR_ESH_UV;
	/*******************************************************/
	/*             0x0C   VBATT_SUM_TH                     */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t VBATT_SUM_UV :8;
			uint32_t VBATT_SUM_OV :8;
			uint32_t reserved :16;
			
		
		} B;
	} VBATT_SUM_TH;
	/*******************************************************/
	/*              0x0D   ADCV_CONV                       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t CYCLIC_CONTNS :1;
			uint32_t TCYCLE :3;
			uint32_t HWSC :1;
			uint32_t BAL_TERM_CON :1;
			uint32_t CELL_TERM_CON :1;
			uint32_t GPIO_TERM_CON :1;
			uint32_t GPIO_CONV :1;
			uint32_t ADC_Filter :3;
			uint32_t DUTY_ON :1;
			uint32_t CONF_CYCLIC_EN :1;
			uint32_t OVR_LATCH :1;
			uint32_t SOC :1;
			uint32_t TCYCLE_OVF :1;
			uint32_t ADC_CROSS_CHECK :1;
			uint32_t reserved :14;

			
		} B;
	} ADCV_CONV;
	/*******************************************************/
	/*          0x0E    NYCLE_PROG_1                       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			
			uint32_t Noreg0 :1;
			uint32_t PCB_open_en_even_curr :1;
			uint32_t PCB_open_en_odd_curr :1;
			uint32_t CROSS_ODD_EVEN_CELL :1;
			uint32_t CYCLIC_UPDATE :1;
			uint32_t BAL_AUTO_PAUSE :1;
			uint32_t BAL_TIM_AUTO_PAUSE :1;
			uint32_t NCYCLE_BAL_TERM :3;
			uint32_t NCYCLE_CELL_TERM :3;
			uint32_t NCYCLE_GPIO_TERM :3;
			uint32_t T_CELL_SET :2;
			uint32_t reserved :14;

			
			
		} B;
	} NCYCLE_PROG_1;
	/*******************************************************/
	/*          0x0F    NYCLE_PROG_2                       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			
			uint32_t ADC_FILTER_SLP :3;
			uint32_t TCYCLE_SLP :3;
			uint32_t ADC_FILTER_CYCLE :3;
			uint32_t NOreg9 :1;
			uint32_t NCYCLE_HWSC :3;
			uint32_t NCYCLE_GPIO :3;
			uint32_t VTREF_DYN_EN :1;
			uint32_t VTREF_EN :1;
			uint32_t reserved :14;
			
			
		
		} B;
	} NCYCLE_PROG_2;

	/*******************************************************/
	/*          0x10 BalCell14_7act bit fields             */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t Noreg16 :1; // RO
			uint32_t BAL14 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL13 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL12 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL11 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL10 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL9 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL8 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL7 :2; // RW when = 10 it enables cell balancing
		} B;
	} BalCell14_7act_t;

	/*******************************************************/
	/*          0x11 BalCell16_7act bit fields             */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t Noreg16 :1; // RO
			uint32_t BAL6 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL5 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL4 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL3 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL2 :2; // RW when = 10 it enables cell balancing
			uint32_t BAL1 :2; // RW when = 10 it enables cell balancing
			uint32_t Noreg3 :1; // RO
			uint32_t Noreg2 :1; // RO
			uint32_t bal_on :1; // RO when 1 balancing is still going on
			uint32_t eof_bal :1; // RO when 1 balancing has stopped and waiting for STOP command
		} B;
	} BalCell6_1act_t;

	/*******************************************************/
	/*               0x12 ADDR_FSM bit fields              */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t Noreg16 :1; // RO
			uint32_t SW_RST :2; // WO resets all reg map registers
			uint32_t GO2SLP :2; // WO sleep/init/normal/cyclic
			uint32_t FSMstatus :4; // RO normal value can be read
			uint32_t Noreg7 :1; // RO
			uint32_t Noreg6 :1; // RO
			uint32_t Noreg5 :1; // RO
			uint32_t wu_gpio7 :1; // RO wakeup source was GPIO7
			uint32_t wu_spi :1; // RO wakeup source was SPI
			uint32_t wu_isoline :1; // RO wakeup source was ISOLINE
			uint32_t wu_faulth :1; // RO wakeup source was external fault
			uint32_t wu_cyc_wup :1; // RO wakeup source was internal cyclic wakeup
		} B;
	} FSM_t;
	/*******************************************************/
	/*               0x13 GPOxOn锟斤拷锟斤拷GPIO9_3 bit fields       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t GPO9on :1; // 1 sets GPIO to High
			uint32_t GPO8on :1; // 1 sets GPIO to High
			uint32_t GPO7on :1; // 1 sets GPIO to High
			uint32_t GPO6on :1; // 1 sets GPIO to High
			uint32_t GPO5on :1; // 1 sets GPIO to High
			uint32_t GPO4on :1; // 1 sets GPIO to High
			uint32_t GPO3on :1; // 1 sets GPIO to High
			uint32_t Noreg10 :1; // reserved
			uint32_t Noreg9 :1; // reserved
			uint32_t GPI9 :1; // Read back GPIO status
			uint32_t GPI8 :1; // Read back GPIO status
			uint32_t GPI7 :1; // Read back GPIO status
			uint32_t GPI6 :1; // Read back GPIO status
			uint32_t GPI5 :1; // Read back GPIO status
			uint32_t GPI4 :1; // Read back GPIO status
			uint32_t GPI3 :1; // Read back GPIO status
			uint32_t Noreg1 :1; // reserved
			uint32_t Noreg0 :1; // reserved
		} B;
	} GPOxOn;
	/*******************************************************/
	/*               0x14 GPIO9_3_CONF bit fields          */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			
			
			uint32_t Noreg0 :1;
			uint32_t Noreg1 :1;
			uint32_t Noreg2 :1;
			uint32_t GPIO7_WUP_EN :1;
			uint32_t GPIO3_CONFIG :2; // GPIO3_CONFIG
			uint32_t GPIO4_CONFIG :2; // GPIO4_CONFIG
			uint32_t GPIO5_CONFIG :2; // GPIO5_CONFIG
			uint32_t GPIO6_CONFIG :2; // GPIO6_CONFIG
			uint32_t GPIO7_CONFIG :2; // GPIO7_CONFIG
			uint32_t GPIO8_CONFIG :2; // GPIO8_CONFIG
			uint32_t GPIO9_CONFIG :2; // GPIO9_CONFIG
			uint32_t res :14; // NA reserved
		
			
			
		} B;
	} GPIO9_3_CONF;
	/*******************************************************/
	/*                   0x15 GPIO3_THR                    */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t GPIO3_OT_TH :9; // GPIO3_overtemperature_threshold
			uint32_t GPIO3_UT_TH :9; // GPIO3_undertemperature_threshold
		} B;
	} GPIO3_THR;
	/*******************************************************/
	/*                   0x16 GPIO4_THR                    */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t GPIO4_OT_TH :9; // GPIO4_overtemperature_threshold
			uint32_t GPIO4_UT_TH :9; // GPIO4_undertemperature_threshold
		} B;
	} GPIO4_THR;
	/*******************************************************/
	/*                   0x17 GPIO5_THR                    */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t GPIO5_OT_TH :9; // GPIO5_overtemperature_threshold
			uint32_t GPIO5_UT_TH :9; // GPIO5_undertemperature_threshold
		} B;
	} GPIO5_THR;
	/*******************************************************/
	/*                   0x18 GPIO6_THR                    */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t GPIO6_OT_TH :9; // GPIO6_overtemperature_threshold
			uint32_t GPIO6_UT_TH :9; // GPIO6_undertemperature_threshold
		} B;
	} GPIO6_THR;
	/*******************************************************/
	/*                   0x19 GPIO7_THR                    */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t GPIO7_OT_TH :9; // GPIO7_overtemperature_threshold
			uint32_t GPIO7_UT_TH :9; // GPIO7_undertemperature_threshold
		} B;
	} GPIO7_THR;
	/*******************************************************/
	/*                   0x1A GPIO8_THR                    */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t GPIO8_OT_TH :9; // GPIO8_overtemperature_threshold
			uint32_t GPIO8_UT_TH :9; // GPIO8_undertemperature_threshold
		} B;
	} GPIO8_THR;
	/*******************************************************/
	/*                   0x1B GPIO9_THR                    */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t GPIO9_OT_TH :9; // GPIO9_overtemperature_threshold
			uint32_t GPIO9_UT_TH :9; // GPIO9_undertemperature_threshold
		} B;
	} GPIO9_THR;
	/*******************************************************/
	/*                   0x1C VCELLS_EN                    */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			
			uint32_t VCELL1_EN :1;
			uint32_t VCELL2_EN :1;
			uint32_t VCELL3_EN :1;
			uint32_t VCELL4_EN :1;
			uint32_t VCELL5_EN :1;
			uint32_t VCELL6_EN :1;
			uint32_t VCELL7_EN :1;
			uint32_t VCELL8_EN :1;
			uint32_t VCELL9_EN :1;
			uint32_t VCELL10_EN :1;
			uint32_t VCELL11_EN :1;
			uint32_t VCELL12_EN :1;
			uint32_t VCELL13_EN :1;
			uint32_t VCELL14_EN :1;
			uint32_t reserved :18;
			

		} B;
	} VCELLS_EN;
	/*******************************************************/
	/*               0x1D  FaultMask                       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :18;
			uint32_t VCELL14_BAL_UV :1;
			uint32_t VCELL13_BAL_UV :1;
			uint32_t VCELL12_BAL_UV :1;
			uint32_t VCELL11_BAL_UV :1;
			uint32_t VCELL10_BAL_UV :1;
			uint32_t VCELL9_BAL_UV :1;
			uint32_t VCELL8_BAL_UV :1;
			uint32_t VCELL7_BAL_UV :1;
			uint32_t VCELL6_BAL_UV :1;
			uint32_t VCELL5_BAL_UV :1;
			uint32_t VCELL4_BAL_UV :1;
			uint32_t VCELL3_BAL_UV :1;
			uint32_t VCELL2_BAL_UV :1;
			uint32_t VCELL1_BAL_UV :1;
		} B;
	} FaultMask;
	/*******************************************************/
	/*                   0x1E  FaultMask2                  */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t EEPROM_DWNLD_DONE :1;
			uint32_t EEPROM_CRC_ERR_SECT_0 :1;
			uint32_t EEPROM_CRC_ERRMSK_SECT_0 :1;
			uint32_t EEPROM_CRC_ERR_CAL_RAM :1;
			uint32_t EEPROM_CRC_ERRMSK_CAL_RAM :1;
			uint32_t EEPROM_CRC_ERR_CAL_FF :1;
			uint32_t EEPROM_CRC_ERRMSK_CAL_FF :1;
			uint32_t RAM_CRC_ERR :1;
			uint32_t RAM_CRC_ERRMSK :1;
			uint32_t trim_dwnl_tried :1;
			uint32_t TrimmCalOk :1;
			uint32_t GPIO9_FAST_OT :1;
			uint32_t GPIO8_FAST_OT :1;
			uint32_t GPIO7_FAST_OT :1;
			uint32_t GPIO6_FAST_OT :1;
			uint32_t GPIO5_FAST_OT :1;
			uint32_t GPIO4_FAST_OT :1;
			uint32_t GPIO3_FAST_OT :1;
		} B;
	} FaultMask2;
	/*******************************************************/
	/*                   0x1F  CSA_THRESH_NORM             */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t adc_ovc_curr_TH_norm :18;
		} B;
	} CSA_THRESH_NORM;
	/*******************************************************/
	/*                   0x20  CSA_GPIO_MSK                */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t Gpio3_OT_UT_MSK :1;
			uint32_t Gpio4_OT_UT_MSK :1;
			uint32_t Gpio5_OT_UT_MSK :1;
			uint32_t Gpio6_OT_UT_MSK :1;
			uint32_t Gpio7_OT_UT_MSK :1;
			uint32_t Gpio8_OT_UT_MSK :1;
			uint32_t Gpio9_OT_UT_MSK :1;
			uint32_t Noreg7 :1;
			uint32_t sense_minus_open :1;
			uint32_t sense_plus_open :1;
			uint32_t ovc_norm_msk :1;
			uint32_t ovc_sleep_msk :1;
			uint32_t CoulCounter_en :1;
			uint32_t adc_OC_sleep :5;
			uint32_t reserved :14;
			
			
		} B;
	} CSA_GPIO_MSK;
	/*******************************************************/
	/*                  0x21 Vcell1                        */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{	
			
			uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL1;
	/*******************************************************/
	/*                  0x22 Vcell2                        */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
				uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL2;
	/*******************************************************/
	/*                  0x23 Vcell3                        */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL3;
	/*******************************************************/
	/*                  0x24 Vcell4                        */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL4;
	/*******************************************************/
	/*                  0x25 Vcell5                        */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL5;
	/*******************************************************/
	/*                  0x26 Vcell6                        */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL6;
	/*******************************************************/
	/*                  0x27 Vcell7                        */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
				uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL7;
	/*******************************************************/
	/*                  0x28 Vcell8                        */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
				uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL8;
	/*******************************************************/
	/*                  0x29 Vcell9                        */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
				uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL9;
	/*******************************************************/
	/*                  0x2A Vcell10                       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
				uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL10;
	/*******************************************************/
	/*                  0x2B Vcell11                       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
		  uint32_t VOLT :16;
		 	uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL11;
	/*******************************************************/
	/*                  0x2C Vcell12                       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL12;
	/*******************************************************/
	/*                  0x2D Vcell13                       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL13;
	/*******************************************************/
	/*                  0x2E Vcell14                       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t VOLT :16;
			uint32_t D_READY :1;
			uint32_t reserved :15;

		} B;
	} VCELL14;
	/*******************************************************/
	/*                  0x2F Ibattery_sync                 */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t CUR_INST_sync :18;
		} B;
	} Ibattery_sync;

	/*******************************************************/
	/*                  0x30 Ibattery_cali                 */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t CUR_INST_calib :18;
		} B;
	} Ibattery_cali;
	/*******************************************************/
	/*                   0x31 CoulCntrTime                 */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :16;
			uint32_t NUM_Time :16;
		} B;
	} CoulCntrTime;
	/*******************************************************/
	/*                   0x32 CoulCntrmsb                  */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :16;
			uint32_t CoulombCounter_msb :16;
		} B;
	} CoulCntr_msb;
	/*******************************************************/
	/*                  0x33 CoulCntrlsb                   */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :16;
			uint32_t CoulombCounter_lsb :16;
		} B;
	} CoulCntr_lsb;
	/*******************************************************/
	/*            0x34      GPIO3_MEAS                     */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t ratio_abs_3_sel :1;
			uint32_t d_rdy_gpio3 :1;
			uint32_t GPIO3_MEAS_value :16;
		} B;
	} GPIO3_MEAS;
	/*******************************************************/
	/*            0x35      GPIO4_MEAS                     */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t ratio_abs_4_sel :1;
			uint32_t d_rdy_gpio4 :1;
			uint32_t GPIO4_MEAS_value :16;
		} B;
	} GPIO4_MEAS;
	/*******************************************************/
	/*            0x36      GPIO5_MEAS                     */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t ratio_abs_5_sel :1;
			uint32_t d_rdy_gpio5 :1;
			uint32_t GPIO5_MEAS_value :16;
		} B;
	} GPIO5_MEAS;
	/*******************************************************/
	/*            0x37      GPIO6_MEAS                     */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t ratio_abs_6_sel :1;
			uint32_t d_rdy_gpio6 :1;
			uint32_t GPIO6_MEAS_value :16;
		} B;
	} GPIO6_MEAS;
	/*******************************************************/
	/*            0x38      GPIO7_MEAS                     */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t ratio_abs_7_sel :1;
			uint32_t d_rdy_gpio7 :1;
			uint32_t GPIO7_MEAS_value :16;
		} B;
	} GPIO7_MEAS;
	/*******************************************************/
	/*            0x39      GPIO8_MEAS                     */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t ratio_abs_8_sel :1;
			uint32_t d_rdy_gpio8 :1;
			uint32_t GPIO8_MEAS_value :16;
		} B;
	} GPIO8_MEAS;
	/*******************************************************/
	/*            0x3A      GPIO9_MEAS                     */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t ratio_abs_9_sel :1;
			uint32_t d_rdy_gpio9 :1;
			uint32_t GPIO9_MEAS_value :16;
		} B;
	} GPIO9_MEAS;
	/*******************************************************/
	/*            0x3B       TempChip                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; //NA reserved
			uint32_t noreg17 :1; //RO
			uint32_t noreg16 :1; //RO
			uint32_t noreg15 :1; //RO
			uint32_t noreg14 :1; //RO
			uint32_t noreg13 :1; //RO
			uint32_t noreg12 :1; //RO
			uint32_t noreg11 :1; //RO
			uint32_t noreg10 :1; //RO
			uint32_t noreg9 :1; //RO
			uint32_t OTChip :1; //RLR
			uint32_t TempChip :8; //RO TjADC
		} B;
	} TempChip;
	/*******************************************************/
	/*            0x3C       Faults1                       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; //NA reserved
			uint32_t noreg17 :1; //RO
			uint32_t noreg16 :1; //RO
			uint32_t noreg15 :1; //RO
			uint32_t noreg14 :1; //RO
			uint32_t VANA_OV :1; //RLR
			uint32_t VDIG_OV :1; //RLR
			uint32_t VTREF_UV :1; //RLR
			uint32_t VTREF_OV :1; //RLR
			uint32_t VREG_UV :1; //RLR
			uint32_t VREG_OV :1; //RLR
			uint32_t VCOM_UV :1; //RLR
			uint32_t VCOM_OV :1; //RLR
			uint32_t HeartBeat_fault :1; //RLR
			uint32_t FaultHline_fault :1; //RLR
			uint32_t FaultLline_status :1; //RO
			uint32_t noreg2 :1; //RO
			uint32_t noreg1 :1; //RO
			uint32_t Comm_timeout_flt :1; //RLR
		} B;
	} Faults1;
	/*******************************************************/
	/*            0x3D       Faults2                       */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; //NA reserved
			uint32_t noreg17 :1; //RO
			uint32_t noreg16 :1; //RO
			uint32_t noreg15 :1; //RO
			uint32_t noreg14 :1; //RO
			uint32_t noreg13 :1; //RO
			uint32_t SPIENlatch :1; //RO
			uint32_t noreg11 :1; //RO
			uint32_t OSCFail :1; //RLR
			uint32_t noreg9 :1; //RO
			uint32_t loss_agnd :1; //RLR
			uint32_t loss_dgnd :1; //RLR
			uint32_t loss_cgnd :1; //RLR
			uint32_t noreg5 :1; //RO
			uint32_t noreg4 :1; //RO
			uint32_t CoCouOvF :1; //RLR
			uint32_t EoBtimeerror :1; //RLR
			uint32_t curr_sense_ovc_sleep :1; //RLR
			uint32_t curr_sense_ovc_norm :1; //RLR
		} B;
	} Faults2;
	/*******************************************************/
	/*            0x3E       BAL_OPEN                     */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t noreg17 :1;
			uint32_t noreg16 :1;
			uint32_t BAL14_OPEN :1;
			uint32_t BAL13_OPEN :1;
			uint32_t BAL12_OPEN :1;
			uint32_t BAL11_OPEN :1;
			uint32_t BAL10_OPEN :1;
			uint32_t BAL9_OPEN :1;
			uint32_t BAL8_OPEN :1;
			uint32_t BAL7_OPEN :1;
			uint32_t BAL6_OPEN :1;
			uint32_t BAL5_OPEN :1;
			uint32_t BAL4_OPEN :1;
			uint32_t BAL3_OPEN :1;
			uint32_t BAL2_OPEN :1;
			uint32_t BAL1_OPEN :1;
			uint32_t noreg1 :1;
			uint32_t noreg0 :1;
		} B;
	} BAL_OPEN;
	/*******************************************************/
	/*            0x3F       BAL_SHORT                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t noreg17 :1;
			uint32_t noreg16 :1;
			uint32_t BAL14_SHORT :1;
			uint32_t BAL13_SHORT :1;
			uint32_t BAL12_SHORT :1;
			uint32_t BAL11_SHORT :1;
			uint32_t BAL10_SHORT :1;
			uint32_t BAL9_SHORT :1;
			uint32_t BAL8_SHORT :1;
			uint32_t BAL7_SHORT :1;
			uint32_t BAL6_SHORT :1;
			uint32_t BAL5_SHORT :1;
			uint32_t BAL4_SHORT :1;
			uint32_t BAL3_SHORT :1;
			uint32_t BAL2_SHORT :1;
			uint32_t BAL1_SHORT :1;
			uint32_t noreg1 :1;
			uint32_t noreg0 :1;
		} B;
	} BAL_SHORT;
	/*******************************************************/
	/*                   0x40 VSUMBAT                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t VSUM_BAT19_2 :18;
		} B;
	} VSUMBAT;

	/*******************************************************/
	/*                  0x41 VBATTDIV                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t VSUM_BAT1_0 :2;
			uint32_t VBATTDIV :16;
		} B;
	} VBATTDIV;
	/*******************************************************/
	/*               0x42    CELL_OPEN                     */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t reserved :14;
			uint32_t VSUM_BAT1_0 :1;
			uint32_t data_ready :1;
			uint32_t noreg15 :1;
			uint32_t CELL14_OPEN :1;
			uint32_t CELL13_OPEN :1;
			uint32_t CELL12_OPEN :1;
			uint32_t CELL11_OPEN :1;
			uint32_t CELL10_OPEN :1;
			uint32_t CELL9_OPEN :1;
			uint32_t CELL8_OPEN :1;
			uint32_t CELL7_OPEN :1;
			uint32_t CELL6_OPEN :1;
			uint32_t CELL5_OPEN :1;
			uint32_t CELL4_OPEN :1;
			uint32_t CELL3_OPEN :1;
			uint32_t CELL2_OPEN :1;
			uint32_t CELL1_OPEN :1;
			uint32_t CELL0_OPEN :1;
		} B;
	} CELL_OPEN;

	/*******************************************************/
	/*          0x43 VCELL_UV bit fields                   */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t VBATT_WRN_UV :1; // RLR VBAT undervoltage through analog comparator
			uint32_t VBATTCRIT_UV :1; // RLR 1 if Vbatt monitor < VBATT_CRITICAL_UV_TH (voltage conv routine)
			uint32_t VSUM_UV :1; // RLR 1 if Sum of Vcells < VBATT_SUM_UV_TH (voltage conv routine)
			uint32_t VCELL14_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL13_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL12_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL11_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL10_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL9_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL8_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL7_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL6_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL5_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL4_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL3_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL2_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
			uint32_t VCELL1_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
		} B;
	} VCELL_UV;

	/*******************************************************/
	/*          0x44 VCELL_OV bit fields                   */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t VBATT_WRN_OV :1; // RLR VBAT undervoltage through analog comparator
			uint32_t VBATTCRIT_OV :1; // RLR 1 if Vbatt monitor < VBATT_CRITICAL_OV_TH (voltage conv routine)
			uint32_t VSUM_OV :1; // RLR 1 if Sum of Vcells > VBATT_SUM_OV_TH (voltage conv routine)
			uint32_t VCELL14_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL13_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL12_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL11_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL10_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL9_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL8_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL7_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL6_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL5_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL4_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL3_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL2_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
			uint32_t VCELL1_OV :1; // RLR Vcell  (voltage conv routine) > threshVcellOV
		} B;
	} VCELL_OV;

	/*******************************************************/
	/*          0x45 GPIO_OT_UT bit fields                 */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t Noreg16 :1; // RO
			uint32_t Noreg15 :1; // RO
			uint32_t Noreg14 :1; // RO
			uint32_t GPIO9_OT :1; // RLR VBAT undervoltage through analog comparator
			uint32_t GPIO8_OT :1; // RLR 1 if Vbatt monitor < VBATT_CRITICAL_OV_TH (voltage conv routine)
			uint32_t GPIO7_OT :1; // RLR 1 if Sum of Vcells < VBATT_SUM_OV_TH (voltage conv routine)
			uint32_t GPIO6_OT :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
			uint32_t GPIO5_OT :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
			uint32_t GPIO4_OT :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
			uint32_t GPIO3_OT :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
			uint32_t GPIO9_UT :1; // RLR VBAT undervoltage through analog comparator
			uint32_t GPIO8_UT :1; // RLR 1 if Vbatt monitor < VBATT_CRITICAL_OV_TH (voltage conv routine)
			uint32_t GPIO7_UT :1; // RLR 1 if Sum of Vcells < VBATT_SUM_OV_TH (voltage conv routine)
			uint32_t GPIO6_UT :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
			uint32_t GPIO5_UT :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
			uint32_t GPIO4_UT :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
			uint32_t GPIO3_UT :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
		} B;
	} VGPIO_OT_UT;
	/*******************************************************/
	/*          0x46 VCELL_BAL_UV bit fields                   */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t Noreg16 :1; // RLR VBAT undervoltage through analog comparator
			uint32_t Noreg15 :1; // RLR 1 if Vbatt monitor < VBATT_CRITICAL_UV_TH (voltage conv routine)
			uint32_t Noreg14 :1; // RLR 1 if Sum of Vcells < VBATT_SUM_UV_TH (voltage conv routine)
			uint32_t VCELL14_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL13_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL12_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL11_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL10_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL9_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL8_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL7_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL6_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL5_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL4_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL3_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL2_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellBAL_UV
			uint32_t VCELL1_BAL_UV :1; // RLR Vcell  (voltage conv routine) < threshVcellUV
		} B;
	} VCELL_BAL_UV;

	/*******************************************************/
	/*          0x47 GPIO_fastchg_OT bit fields                   */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t Noreg16 :1; // RO
			uint32_t Noreg15 :1; // RO
			uint32_t Noreg14 :1; // RO
			uint32_t GPIO9_OPEN :1; // RLR VBAT undervoltage through analog comparator
			uint32_t GPIO8_OPEN :1; // RLR 1 if Vbatt monitor < VBATT_CRITICAL_OV_TH (voltage conv routine)
			uint32_t GPIO7_OPEN :1; // RLR 1 if Sum of Vcells < VBATT_SUM_OV_TH (voltage conv routine)
			uint32_t GPIO6_OPEN :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
			uint32_t GPIO5_OPEN :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
			uint32_t GPIO4_OPEN :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
			uint32_t GPIO3_OPEN :1; // RLR Vcell  (voltage conv routine) < threshVcellOV
			uint32_t GPIO9_fastchg_OT :1; // RLR VBAT undervoltage through analog comparator
			uint32_t GPIO8_fastchg_OT :1; // RLR 1 if Vbatt monitor < VBATT_CRITICAL_OV_TH (voltage conv rofastchg_OTine)
			uint32_t GPIO7_fastchg_OT :1; // RLR 1 if Sum of Vcells < VBATT_SUM_OV_TH (voltage conv rofastchg_OTine)
			uint32_t GPIO6_fastchg_OT :1; // RLR Vcell  (voltage conv rofastchg_OTine) < threshVcellOV
			uint32_t GPIO5_fastchg_OT :1; // RLR Vcell  (voltage conv rofastchg_OTine) < threshVcellOV
			uint32_t GPIO4_fastchg_OT :1; // RLR Vcell  (voltage conv rofastchg_OTine) < threshVcellOV
			uint32_t GPIO3_fastchg_OT :1; // RLR Vcell  (voltage conv rofastchg_OTine) < threshVcellOV
		} B;
	} GPIO_fastchg_OT;
	/*******************************************************/
	/*          0x48 MUX_BIST_FAIL bit fields              */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t Noreg16 :1; // RO
			uint32_t Noreg15 :1; // RO
			uint32_t HWSC_DONE :1; // RLR
			uint32_t MUX_BIST_FAIL :14; // RLR 1 if Vbatt monitor < VBATT_CRITICAL_OV_TH (voltage conv routine)
		} B;
	} MUX_BIST_FAIL;
	/*******************************************************/
	/*          0x49 BIST_COMP bit fields                  */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t VBAT_COMP_BIST_FAIL :1; // RLR
			uint32_t VREG_COMP_BIST_FAIL :1; // RLR
			uint32_t VCOM_COMP_BIST_FAIL :1; // RLR
			uint32_t VTREF_COMP_BIST_FAIL :1; // RLR
			uint32_t BIST_BAL_COMP_HS_FAIL :7; // RLR
			uint32_t BIST_BAL_COMP_LS_FAIL :7; // RLR
		} B;
	} BIST_COMP;
	/*******************************************************/
	/*          0x4A OPEN_BIST_FAIL bit fields             */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t Noreg16 :1; // RO
			uint32_t Noreg15 :1; // RO
			uint32_t Noreg14 :1; // RO
			uint32_t OPEN_BIST_FAIL :14; // RLR
		} B;
	} OPEN_BIST_FAIL;
	/*******************************************************/
	/*          0x4B GPIO_BIST_FAIL bit fields             */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t GPIO9Short :1; // RLR
			uint32_t GPIO8Short :1; // RLR
			uint32_t GPIO7Short :1; // RLR
			uint32_t GPIO6Short :1; // RLR
			uint32_t GPIO5Short :1; // RLR
			uint32_t GPIO4Short :1; // RLR
			uint32_t GPIO3Short :1; // RLR
			uint32_t Noreg10 :1; // RO
			uint32_t Noreg9 :1; // RO
			uint32_t VBAT_BIST_FAIL :1; // RLR
			uint32_t VTREF_BIST_FAIL :1; // RLR
			uint32_t GPIO_BIST_FAIL :7; // RLR
		} B;
	} GPIO_BIST_FAIL;
	/*******************************************************/
	/*          0x4C VTREF bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t d_rdy_vtref :1; // RLR
			uint32_t VTREF_MEAS :16; // RO
		} B;
	} VTREF;
	/*******************************************************/
	/*          0x4D NVM_WR1 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_WR_15_0 :16; // RW
		} B;
	} NVM_WR1;
	/*******************************************************/
	/*          0x4E NVM_WR2 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_WR_31_16 :16; // RW
		} B;
	} NVM_WR2;
	/*******************************************************/
	/*          0x4F NVM_WR3 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_WR_47_32 :16; // RW
		} B;
	} NVM_WR3;
	/*******************************************************/
	/*          0x50 NVM_WR4 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_WR_63_48 :16; // RW
		} B;
	} NVM_WR4;
	/*******************************************************/
	/*          0x51 NVM_WR5 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_WR_79_64 :16; // RW
		} B;
	} NVM_WR5;
	/*******************************************************/
	/*          0x52 NVM_WR6 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_WR_95_80 :16; // RW
		} B;
	} NVM_WR6;
	/*******************************************************/
	/*          0x53 NVM_WR7 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_WR_111_96 :16; // RW
		} B;
	} NVM_WR7;
	/*******************************************************/
	/*          0x54 NVM_RD1 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_RD_15_0 :16; // RW
		} B;
	} NVM_RD1;
	/*******************************************************/
	/*          0x55 NVM_RD2 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_RD_31_16 :16; // RW
		} B;
	} NVM_RD2;
	/*******************************************************/
	/*          0x56 NVM_RD3 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_RD_47_32 :16; // RW
		} B;
	} NVM_RD3;
	/*******************************************************/
	/*          0x57 NVM_RD4 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_RD_63_48 :16; // RW
		} B;
	} NVM_RD4;
	/*******************************************************/
	/*          0x58 NVM_RD5 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_RD_79_64 :16; // RW
		} B;
	} NVM_RD5;
	/*******************************************************/
	/*          0x59 NVM_RD6 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_RD_95_80 :16; // RW
		} B;
	} NVM_RD6;
	/*******************************************************/
	/*          0x5A NVM_RD7 bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :16; // NA reserved
			uint32_t NVM_RD_111_96 :16; // RW
		} B;
	} NVM_RD7;
	/*******************************************************/
	/*          0x5B NVM_CMD_CNTR bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :20; // NA reserved
			uint32_t NVM_WR_BUSY :1; // RW
			uint32_t NVM_OPER :2; // RW
			uint32_t NVM_PROGRAM :1; // RW
			uint32_t NVM_CNTR :8; // RW
		} B;
	} NVM_CMD_CNTR;
	/*******************************************************/
	/*          0x5C NVM_UNLCK_PRG bit fields                      */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t NVM_UNLOCK_START :18; // WO
		} B;
	} NVM_UNLCK_PRG;
	/*******************************************************/
	/*          0x7E DEVICE_SERVICE_RD bit fields          */
	/*******************************************************/
	union
	{
		uint32_t R;
		struct
		{
			uint32_t res :14; // NA reserved
			uint32_t Noreg17 :1; // RO
			uint32_t Noreg16 :1; // RO
			uint32_t Noreg15 :1; // RO
			uint32_t Noreg14 :1; // RO
			uint32_t Noreg13 :1; // RO
			uint32_t Chipversion :8; // RO
			uint32_t TM_PIN_FBCK :1; // RO
			uint32_t SPI_TEST_EN :1; // RO
			uint32_t TM_FM_STATUS :3; // RO
		} B;
	} DEVICE_SERVICE_RD;
} L9963_Reg_type;

typedef union
{
	uint16_t R;
	struct
	{
		uint16_t res :2; /* reserved bits*/
		uint16_t bal_14 :1; /* reserved bits*/
		uint16_t bal_13 :1; /* reserved bits*/
		uint16_t bal_12 :1; /* reserved bits*/
		uint16_t bal_11 :1; /* reserved bits*/
		uint16_t bal_10 :1; /* reserved bits*/
		uint16_t bal_9 :1; /* reserved bits*/
		uint16_t bal_8 :1; /* reserved bits*/
		uint16_t bal_7 :1; /* reserved bits*/
		uint16_t bal_6 :1; /* reserved bits*/
		uint16_t bal_5 :1; /* reserved bits*/
		uint16_t bal_4 :1; /* reserved bits*/
		uint16_t bal_3 :1; /* reserved bits*/
		uint16_t bal_2 :1; /* reserved bits*/
		uint16_t bal_1 :1; /* reserved bits*/
	} B;
} UART_BALANCE_CMD;
/*L9966  Register Address*/
typedef enum
{
	GEN_STATUS_ADD = 0x01,
	FASTCH_BAL_UV_ADD = 0x02,
	BAL_1_ADD = 0x03,
	BAL_2_ADD = 0x04,
	BAL_3_ADD = 0x05,
	BAL_4_ADD,
	BAL_5_ADD,
	BAL_6_ADD,
	BAL_7_ADD,
	BAL_8_ADD,
	VCELL_THR_ESH_UV_OV_ADD,
	VBATT_SUM_TH_ADD,
	ADCV_CONV_ADD,
	NCYCLE_PROG_1_ADD,
	NCYCLE_PROG_2_ADD,
	BalCell14_7act_ADD,
	BalCell6_1act_ADD,
	FSM_ADD,
	GPOxOn_and_GPI93_ADD,
	GPIO9_3_CONF_ADD,
	GPIO3_THR_ADD,
	GPIO4_THR_ADD,
	GPIO5_THR_ADD,
	GPIO6_THR_ADD,
	GPIO7_THR_ADD,
	GPIO8_THR_ADD,
	GPIO9_THR_ADD,
	VCELLS_EN_ADD,
	Faultmask_ADD,
	Faultmask2_ADD,
	CSA_THRESH_NORM_ADD,
	CSA_GPIO_MSK_ADD,
	vcell1_ADD,
	vcell2_ADD,
	vcell3_ADD,
	vcell4_ADD,
	vcell5_ADD,
	vcell6_ADD,
	vcell7_ADD,
	vcell8_ADD,
	vcell9_ADD,
	vcell10_ADD,
	vcell11_ADD,
	vcell12_ADD,
	vcell13_ADD,
	vcell14_ADD,
	Ibattery_synch_ADD,
	Ibattery_calib_ADD,
	CoulCntrTime_ADD,
	CoulCntrmsb_ADD,
	CoulCntrlsb_ADD,
	GPIO3_MEAS_ADD,
	GPIO4_MEAS_ADD,
	GPIO5_MEAS_ADD,
	GPIO6_MEAS_ADD,
	GPIO7_MEAS_ADD,
	GPIO8_MEAS_ADD,
	GPIO9_MEAS_ADD,
	TempChip_ADD,
	Faults1_ADD,
	Faults2_ADD,
	BAL_OPEN_ADD,
	BAL_SHORT_ADD,
	VSUMBATT_ADD,
	VBATTDIV_ADD,
	CELL_OPEN_ADD,
	VCELL_UV_ADD,
	VCELL_OV_ADD,
	VGPIO_OT_UT_ADD,
	VCELL_BAL_UV_ADD,
	GPIO_fastchg_OT_ADD,
	MUX_BIST_FAIL_ADD,
	BIST_COMP_ADD,
	OPEN_BIST_FAIL_ADD,
	GPIO_BIST_FAIL_ADD,
	VTREF_ADD,
	NVM_WR1_ADD,
	NVM_WR2_ADD,
	NVM_WR3_ADD,
	NVM_WR4_ADD,
	NVM_WR5_ADD,
	NVM_WR6_ADD,
	NVM_WR7_ADD,
	NVM_RD1_ADD,
	NVM_RD2_ADD,
	NVM_RD3_ADD,
	NVM_RD4_ADD,
	NVM_RD5_ADD,
	NVM_RD6_ADD,
	NVM_RD7_ADD,
	NVM_CMD_CNTR,
	NVM_UNLCK_PRG,
	BURST_78CMD = 0x78,
	BURST_7ACMD = 0x7A,
	BURST_7BCMD = 0x7B,
	DEVICE_SERVICE_RD_ADD = 0x7E
} L9963_Addr_t;

typedef enum
{
	BROADCAST_ID,
	DEVICE_ID_1 = 1,
	DEVICE_ID_2,
	DEVICE_ID_3,
	DEVICE_ID_4,
	DEVICE_ID_5,
	DEVICE_ID_6,
	DEVICE_ID_7,
	DEVICE_ID_8,
	DEVICE_ID_9,
	DEVICE_ID_10,
	DEVICE_ID_11,
	DEVICE_ID_12,
	DEVICE_ID_13,
	DEVICE_ID_14,
	DEVICE_ID_15
} DEVICE_ID;

typedef enum
{
	L9963T1 = 1, L9963T2 = 2,
} L9963T_NO;
typedef enum
{
	BURST_7ACMD_SIZE = 13, BURST_7BCMD_SIZE = 14, BURST_78CMD_SIZE = 18
} BURST_SIZE;

typedef enum
{
	FRAME2 = 0x62,
	FRAME3 = 0x63,
	FRAME4 = 0x64,
	FRAME5 = 0x65,
	FRAME6 = 0x66,
	FRAME7 = 0x67,
	FRAME8 = 0x68,
	FRAME9 = 0x69,
	FRAME10 = 0x6A,
	FRAME11 = 0x6B,
	FRAME12 = 0x6C,
	FRAME13 = 0x6D,
	FRAME14 = 0x6E,
	FRAME15 = 0x6F,
	FRAME16 = 0x70,
	FRAME17 = 0x71,
	FRAME18 = 0x72,
	FRAME19 = 0x73
} BURST_FRAME_NUMBER;

typedef enum
{
	ANLOG_INPUT = 0, NOT_USED = 1, DIGITAL_INPUT = 2, DIGITAL_OUTPUT = 3
}GPIO_MODE1;


typedef enum
{
	GPIO3 = 3, GPIO4 = 4, GPIO5 = 5, GPIO6 = 6, GPIO7 = 7, GPIO8 = 8, GPIO9 = 9
} GPIO_CHANNEL;


extern L9963_Reg_type L9963_Reg[16];
extern L9963_SPI_Rx_Inst_t spi_rx_t;
uint8 L9963_Single_Write(L9963_Addr_t addr, uint8 device_id, uint32_t data,
		L9963_SPI_Rx_Inst_t* spi_rx);
uint8 L9963_Single_Read(L9963_Addr_t addr, uint8 device_id,
		L9963_SPI_Rx_Inst_t* spi_rx);
void L9963_Test_Init(void);
void L9963_ADC_Init(void);
void L9963_CellVotage_Read(void);
uint8 L9963_Burst_Read(L9963_Addr_t addr, uint8 device_id,
		uint8* spi_miso, BURST_SIZE burst_size);
uint8 L9963_Burst_Read_Onchip(L9963_Addr_t addr, uint8 device_id,
		L9963_SPI_Rx_Inst_t* spi_rx, BURST_SIZE burst_size);
uint8 L9963_Configure_ID(uint8 totalID, uint8 *ID, uint8* ACK);
uint8 L9963_Clear_ID(uint8 totalID, uint8 *ID, uint8 *ACK);
uint8 L9963_Direct_Write(uint8 *spi_mosi, uint8 *spi_miso);
uint8 L9963_Direct_Read(uint8 *spi_mosi, uint8 *spi_miso);
uint8 L9963_Wakeup(uint8 device_id, L9963_SPI_Rx_Inst_t* spi_rx);
uint8 L9963_Register_Update(L9963_SPI_Rx_Inst_t *spi_rx, uint8 CMD,
		uint8 Device_ID);
void L9963_SPI_start(void);
void L9963T_SPI_start(void);
void L9963_DIG_ISO_SPI_start(void);
uint8 L9963_Ondemand_Conversion_start(void);
uint8 L9963_Ondemand_Conversion_stop(void);
int32_t Calculate_original_data_18bit(uint32_t InputWord);
int32_t Calculate_original_data_32bit(uint32_t InputWord);
uint8 L9963_GPIO_config(GPIO_CHANNEL gpio_channel, GPIO_MODE1 gpio_mode, uint8 device_id);
uint8 L9963_Test_Cell_Temp(void);
uint8 L9963_Single_Read_temp(L9963_Addr_t addr, uint8 device_id, L9963_SPI_Rx_Inst_t* spi_rx);
void smallEndToBigEnd(uint8* spi_rx);


void L9963_Register_Reset_value_Init(void);
uint8 L9963_Sleep(uint8 device_id, L9963_SPI_Rx_Inst_t* spi_rx);
uint8 L9963T_Init(void);
uint8 CrcCompute_L9963(unsigned long long InputWord);
void L9963T_DEVICE_USE(L9963T_NO device_NO);
uint8 L9963_Test_Voltage(void);
void L9963_CAN_Transmit(void);

#endif
