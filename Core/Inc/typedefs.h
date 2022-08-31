#ifndef __TYPEDEFS_H__
#define __TYPEDEFS_H__

#ifdef __MWERKS__    /* Metrowerk CodeWarrior */
//    #include <stdint.h>
//
//    /*  Standard typedefs used by header files, based on ISO C standard */
//    typedef volatile int8_t vint8_t;
//    typedef volatile uint8_t vuint8_t;
//
//    typedef volatile int16_t vint16_t;
//    typedef volatile uint16_t vuint16_t;
//
//    typedef volatile int32_t vint32_t;
//    typedef volatile uint32_t vuint32_t;
//
//#elif defined(__ghs__)    /* GreenHills */
//    #include <stdint.h>
//
//    /* Standard typedefs used by header files, based on ISO C standard */
//    typedef volatile int8_t vint8_t;
//    typedef volatile uint8_t vuint8_t;
//
//    typedef volatile int16_t vint16_t;
//    typedef volatile uint16_t vuint16_t;
//
//    typedef volatile int32_t vint32_t;
//    typedef volatile uint32_t vuint32_t;
//
//#elif defined(__HIGHTEC__)   /* HighTec GCC */
//    #include <stdint.h>
//
//    /* Standard typedefs used by header files, based on ISO C standard */
//    typedef volatile int8_t vint8_t;
//    typedef volatile uint8_t vuint8_t;
//
//    typedef volatile int16_t vint16_t;
//    typedef volatile uint16_t vuint16_t;
//
//    typedef volatile int32_t vint32_t;
////    typedef volatile uint32 vuint32_t;

#else
//#include "Platform_Types.h"
#include "stdint.h"
    /* This is needed for compilers that don't have a stdint.h file */
    typedef unsigned short  uint16;

    //typedef unsigned short  int16_t;
    typedef signed char int8_t;
    typedef unsigned char uint8_t;
    typedef volatile signed char vint8_t;
    typedef volatile unsigned char vuint8_t;
		
    //typedef uint16 int16_t;
    typedef unsigned short uint16_t;
    typedef volatile signed short vint16_t;
    typedef volatile unsigned short vuint16_t;

    typedef signed int int32_t;
    typedef unsigned int uint32_t;
    typedef volatile signed int vint32_t;
    typedef volatile unsigned int vuint32_t;

    /*
     * Derived generic types.
     */


		/*-------Standard Integer Data Types--------*/
		/*TRACE[SWS_Platform_00016]:This standard AUTOSAR type shall be of 8 bit signed. */
		 /* Range : -128 .. +127 */
		/*           0x80..0x7F */
		typedef signed char     sint8;

		/* TRACE[SWS_Platform_00013]: This standard AUTOSAR type shall be of 8 bit unsigned.*/
		 /* Range : 0 .. 255     */
		 /*         0x00 .. 0xFF */
		typedef unsigned char   uint8;

		/* TRACE[SWS_Platform_00017]: This standard AUTOSAR type shall be of 16 bit signed. */
		/* Range : -32768 .. +32767 */
		/*          0x8000..0x7FFF  */
		typedef signed short    sint16;

		/*TRACE[SWS_Platform_00014]: This standard AUTOSAR type shall be of 16 bit unsigned. */
		/* Range : 0 .. 65535      */
		/*         0x0000..0xFFFF  */
		typedef unsigned short  uint16;

		/*TRACE[SWS_Platform_00018]:This standard AUTOSAR type shall be 32 bit signed. */
		/*Range : -2147483648 .. +2147483647   */
		/*         0x80000000..0x7FFFFFFF      */
		typedef signed int     sint32;

		/*TRACE[SWS_Platform_00067]: This standard AUTOSAR type shall be 64 bit signed. */
		/*Range: -9223372036854775808..+9223372036854775807*/
		/*        0x8000000000000000..0x7FFFFFFFFFFFFFFF   */
		typedef signed long long sint64;

		/*TRACE[SWS_Platform_00015]:This standard AUTOSAR type shall be 32 bit unsigned. */
		/* Range:  0 .. 4294967295           */
		/*         0x00000000..0xFFFFFFFF    */
		typedef unsigned int   uint32;

		/*TRACE[SWS_Platform_00066]: This standard AUTOSAR type shall be 64 bit unsigned.*/
		/* Range :   0 ..18446744073709551615              */
		/*           0x0000000000000000..0xFFFFFFFFFFFFFFFF*/
		typedef unsigned long long   uint64;

		/* Standard Float Data Types */
		/* ------------------------- */
		/* TRACE[SWS_Platform_00041]: This standard AUTOSAR type shall follow the 32-bit binary interchange format
			according to IEEE 754-2008 with encoding parameters specified in chapter 3.6, table 3.5, column "binary32".*/
		typedef float   float32;
		/*TRACE[SWS_Platform_00042]:This standard AUTOSAR type shall follow the 64-bit binary interchange format according
								to IEEE 754-2008 with encoding parameters specified in chapter 3.6, table 3.5, column "binary64". */
		typedef double  float64;

		/*-------- Boolean Data Type--------- */
		/* MR12 DIR 1.1 VIOLATION: the type _Bool is mapped here to the AUTOSAR type boolean to prevent the direct use of 
		the type _Bool */
		/* TRACE[SWS_Platform_00026]:This standard AUTOSAR type shall only be used together with the definitions TRUE and
		FALSE. */
		typedef unsigned char   boolean;

		/* Optimized Integer Data Types */
		/* ---------------------------- */
		/* TRACE[SWS_Platform_00023]:This optimized AUTOSAR type shall be at least 8 bit signed. */
		 /* At least -128..+127 */
		/*   0x80..0x7F  */
		typedef signed long     sint8_least;

		/* TRACE[SWS_Platform_00020]:This optimized AUTOSAR type shall be at least 8 bit unsigned. */
		typedef unsigned long   uint8_least;    /* At least 0..255  */

		/* TRACE[SWS_Platform_00024]:This optimized AUTOSAR type shall be at least 16 bit signed. */
			 /*At least -32768..+32767*/
			 /* 0x8000..0x7FFF */
		typedef signed long     sint16_least;

		/* TRACE[SWS_Platform_00021]:This optimized AUTOSAR type shall be at least 16 bit unsigned. */
			 /*At least 0..65535 */
			/*0x0000..0xFFFF   */
		typedef unsigned long   uint16_least;

		/* TRACE[SWS_Platform_00025]:This optimized AUTOSAR type shall be at least 32 bit signed.*/
		/* At least -2147483648..+2147483647*/
		/*0x80000000..0x7FFFFFFF*/
		typedef signed long sint32_least;

		/* SWS_Platform_00022:This optimized AUTOSAR type shall be at least 32 bit unsigned. */
		/* At least 0..4294967295*/
		/* 0x00000000..0xFFFFFFFF */
		typedef unsigned long   uint32_least;




#endif /* __MWERKS__ */

#endif /* __TYPEDEFS_H__ */

/*********************************************************************
 *
 * Copyright:
 *	Freescale Semiconductor, INC. All Rights Reserved.
 *  You are hereby granted a copyright license to use, modify, and
 *  distribute the SOFTWARE so long as this entire notice is
 *  retained without alteration in any modified and/or redistributed
 *  versions, and that such modified versions are clearly identified
 *  as such. No licenses are granted by implication, estoppel or
 *  otherwise under any patents or trademarks of Freescale
 *  Semiconductor, Inc. This software is provided on an "AS IS"
 *  basis and without warranty.
 *
 *  To the maximum extent permitted by applicable law, Freescale
 *  Semiconductor DISCLAIMS ALL WARRANTIES WHETHER EXPRESS OR IMPLIED,
 *  INCLUDING IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A
 *  PARTICULAR PURPOSE AND ANY WARRANTY AGAINST INFRINGEMENT WITH
 *  REGARD TO THE SOFTWARE (INCLUDING ANY MODIFIED VERSIONS THEREOF)
 *  AND ANY ACCOMPANYING WRITTEN MATERIALS.
 *
 *  To the maximum extent permitted by applicable law, IN NO EVENT
 *  SHALL Freescale Semiconductor BE LIABLE FOR ANY DAMAGES WHATSOEVER
 *  (INCLUDING WITHOUT LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS,
 *  BUSINESS INTERRUPTION, LOSS OF BUSINESS INFORMATION, OR OTHER
 *  PECUNIARY LOSS) ARISING OF THE USE OR INABILITY TO USE THE SOFTWARE.
 *
 *  Freescale Semiconductor assumes no responsibility for the
 *  maintenance and support of this software
 *
 ********************************************************************/


