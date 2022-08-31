/*
 * crc_L9963.h
 *
 *  Created on: Oct 17, 2018
 *      Author: leslie lu
 */

#ifndef UTILITY_INC_CRC_L9963_H_
#define UTILITY_INC_CRC_L9963_H_
#include "typedefs.h"

// ****************************************************************************
// ******************************* Public Constants *******************************
// ****************************************************************************
#define CRC_TABLE_SIZE   256

#define CRC_SLOW
#ifndef CRC_SLOW
extern const uint16_t CrcCalc8bitLookupTab_B[];
#endif
//CRC configuration structure
typedef struct
{
	//public configuration
	uint16_t frame_lengh;    //frame length number of active data bits [8..2040]
	uint8 crc_length;         //length of CRC in bits [1..7]
	uint8 polynom;            //CRC polynom [1..255]
	uint8 seed_value;         //CRC seed value [0..255]
	uint8 frame_position; //highest (first) bit position in input streem [0..7]
	//local variables
	uint8 seed_value_aligned; //Internal variable will be calculated during init
	uint8 frame_last_index; //Internal variable will be calculated during init
	uint8 rest_bits;  //Number of rest bits which needs to be process serially
	uint8 result_shift;         //Results shift
	uint8 polynom_shifted;      //Polynom shifted shift
	uint8 crc_tab_p[CRC_TABLE_SIZE]; //CRC table
} tabcrcConfigSet_t;

// ****************************************************************************
// **************************** Functions Prototypes **************************
// ****************************************************************************

// ****************************************************************************
// Name:        tabcrcInit
// Parameters:
//              tabcrcConfigSet_t * config - pointer to configuration the
//              configuration have folowed fields which must be configured
//              before the init function is used:
//                uint16_t frame_lengh;        //frame length number of active data bits [8..2040]
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
void tabcrcInit(tabcrcConfigSet_t * config);

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
typedef uint8 (*pCrcCalc8bitLookupTab)(tabcrcConfigSet_t * config,
		uint8 * frame);

#define CrcCalc8bitLookupTab	((pCrcCalc8bitLookupTab)CrcCalc8bitLookupTab_B)


#endif /* UTILITY_INC_CRC_L9963_H_ */
