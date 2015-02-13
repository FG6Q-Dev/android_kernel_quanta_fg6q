/*
 *  Quanta header file
 *  
 *  Copyright (c) 2010 Atmel Corporation
 *  Copyright (C) 2010 Ulf Samuelsson (ulf@atmel.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *  See the file "COPYING" in the main directory of this archive
 *  for more details.
 *
 */
#ifndef __HW_MAINBOARD_VERSION__
#define __HW_MAINBOARD_VERSION__

enum 
{
	HW_REV_A	= 0,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_F,
	HW_REV_G,
	HW_REV_H,
	HW_REV_I,
};
/*
 * Elpida   : R241(del),R367(del),R221(add),R152(add)   ID :00 (RAM_CODE1,RAM_CODE0)
 * Micron  	: R241(add),R367(del),R221(del),R152(add)   ID :01 (RAM_CODE1,RAM_CODE0)
 * Samsung 	: R241(del),R367(add),R221(add),R152(del)   ID :10 (RAM_CODE1,RAM_CODE0)
 * Hynix 	: R241(add),R367(add),R221(del),R152(del)   ID :11 (RAM_CODE1,RAM_CODE0)
*/
enum 
{
	HW_RAMCODES_ELPIDA	= 0,
	HW_RAMCODES_MICRON,
	HW_RAMCODES_SAMSUNG,
	HW_RAMCODES_HYNIX,
};


///////////////////////////////////////////////////////////////////////
// Function Description
//    this function is for HW mainboard detection
// Parameter
//    none
// Return
//    will return the version of mainboard
int qci_mainboard_version(void);
int qci_ramcodes_version(void);

#endif

