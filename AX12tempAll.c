
/*
 * $Id: AX12dumpAll.c,v 1.2 2010/06/14 19:14:07 clivewebster Exp $
 *
 * Revision History
 * ================
 * $Log: AX12dumpAll.c,v $
 * Revision 1.2  2010/06/14 19:14:07  clivewebster
 * Add copyright license info
 *
 * Revision 1.1  2010/03/24 19:49:27  clivewebster
 * Alpha release
 *
 * ===========
 *
 * Copyright (C) 2010 Clive Webster (webbot@webbot.org.uk)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * 
 *
 * AX12dumpAll.c
 *
 *  Created on: 23 Mar 2010
 *      Author: Clive Webster
 *  Adapted by Eric Relson, April 2012
 */


// #include "AX12.h"
// #include <Servos/Dynamixel/AX12.h>
// #include "../../core.h"
// #include <core.h>
#include "hardware.h"
// #include "HeaderDefs.h"

// void ax12TempAll(const DYNAMIXEL_AX12_DRIVER* driver){
	// ax12TempAllTo(stdout,driver);
// }

void ax12TempAll(const DYNAMIXEL_AX12_DRIVER* driver){
	// PRINTF(f,"AX12 ID Speed Load Volts Temp(C) Moving\r\n");
	//rprintf("AX12 ID Speed Load Volts Temp(C) Moving\r\n");
	for(int i=0; i<driver->num_servos; i++){
		DYNAMIXEL_AX12* servo = (DYNAMIXEL_AX12*)pgm_read_word(&driver->servos[i]);
		uint16_t  status = ax12GetInfo(servo);
		if(status == 0){	//Servo is good to go
			// PRINTF(f,"%7u %6d %5d %2u.%c %5u      %c\r\n",
			// rprintf("S%7uT%5uK%c\r\n",
			rprintf("S(%u ) T(%u ) ",
				(uint16_t)servo->id,
				(uint16_t)servo->info.temperature //,
				// (servo->info.moving) ? 'Y' : 'N'
				);
				
		// }else{
			// PRINTF(f,"%7u Status: %u\r\n",(uint16_t)servo->id,status);
			// rprintf("%7u Status: %u\r\n",(uint16_t)servo->id,status);
		}
	}
}
