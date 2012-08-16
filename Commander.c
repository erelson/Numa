#include "hardware.h"
#include "math.h"
#include "HeaderDefs.h"

//#include "servoPosConsts.h"


#include "Commander.h"


/* process messages coming from Commander 
 *  format = 0xFF RIGHT_H RIGHT_V LEFT_H LEFT_V BUTTONS EXT checksum_cmdr */
int CmdrReadMsgs(){
	//while(LISTEN.available() > 0){
	while(uartReceiveBufferIsEmpty(LISTEN) == FALSE){
		if(index_cmdr == -1){         // looking for new packet
			if(uartGetByte(LISTEN) == 0xff){ //read until packet start
				index_cmdr = 0;
				checksum_cmdr = 0;
			}
		}else if(index_cmdr == 0){
			vals[index_cmdr] = (unsigned char) uartGetByte(LISTEN);
			if(vals[index_cmdr] != 0xff){            
				checksum_cmdr += (int) vals[index_cmdr];
				index_cmdr++;
			}
		}else{
			vals[index_cmdr] = (unsigned char) uartGetByte(LISTEN); //loops will sequentially read bytes and store them here
			checksum_cmdr += (int) vals[index_cmdr];
			index_cmdr++;
			
			// DEBUG: if all packets go through, shoudl see x2 through x7 when Commander input is being received.
			// rprintf("x%u ",index_cmdr);
			
			if(index_cmdr == 7){ // packet complete
				if(checksum_cmdr%256 != 255){
					// packet error!
					rprintf("\npacket error!\n");
					index_cmdr = -1;
					return 0;
				}else{
					char buttonval = vals[4];
					short dowalking = zTRUE;
					
					//rprintf("\t%d\t",(int)buttonval);
					//Turn gait...
					if((buttonval&0x40) > 0){ //if(buttonval & BUT_LT){
						if(PRINT_DEBUG_COMMANDER){rprintf("lft\t");}
						turnleft = zTRUE;
						turnright = zFALSE;
						
						dowalking = zFALSE;
					}
					else if((buttonval&0x80) > 0){ //if(buttonval & BUT_RT){
						if(PRINT_DEBUG_COMMANDER){rprintf("rgt\t");}
						turnright = zTRUE;
						turnleft = zFALSE;
						
						dowalking = zFALSE;
					}
					else { // Do nothing
						turnright = zFALSE;
						turnleft = zFALSE;
						turn = zFALSE;
					}
					if((buttonval&BUT_L6) > 0){
						gunbutton = zTRUE;
						if(PRINT_DEBUG_COMMANDER){rprintf("guns\t");}
					}
					else{gunbutton = zFALSE;}

					if((buttonval&BUT_R3) > 0){
						panicbutton = zTRUE;
						if(PRINT_DEBUG_COMMANDER){rprintf("panic\t");}
					}
					else{panicbutton = zFALSE;}
					
					if((buttonval&BUT_L4) > 0){
						infobutton = zTRUE;
						if(PRINT_DEBUG_COMMANDER){rprintf("info\t");}
					}
					else{infobutton = zFALSE;}
					
					if((buttonval&BUT_R2) > 0){
						pan_pos = PAN_CENTER;
						tilt_pos = TILT_CENTER;
						if(PRINT_DEBUG_COMMANDER){rprintf("look\t");}
					}
					// else{infobutton = zFALSE;}
					
					if((buttonval&BUT_R1) > 0){
						agitbutton = zTRUE;
						if(PRINT_DEBUG_COMMANDER){rprintf("agit\t");}
					}
					else{agitbutton = zFALSE;}
					
					if((southpaw&0x01) > 0){     // SouthPaw
						walkV = (signed char)( (int)vals[0]-128 );
						walkH = (signed char)( (int)vals[1]-128 );
						// lookV = (signed char)( (int)vals[2]-128 );
						// lookH = (signed char)( (int)vals[3]-128 );
					}else if (dowalking){
						// vals - 128 gives look a vlaue in the range from -128 to 127?
						lookV = (signed char)( (int)vals[0]-128 );
						lookH = (signed char)( (int)vals[1]-128 );
						// if( (int)vals[0] >= 128){
							// tilt_pos = interpolateU( (int)vals[0],128,128+102,TILT_CENTER,servo52Max);
						// }
						// else {
							// tilt_pos = interpolateU( (int)vals[0],128-102,128,servo52Min,TILT_CENTER);
						// }
						
						int pan_add = (-(float)lookH)/17;
						int tilt_add = (-(float)lookV)/25;
						
						
						// pan_pos = interpolate( (int)vals[1],0,255,servo51Max,servo51Min);
						
						pan_pos = CLAMP(pan_pos + pan_add, servo51Min, servo51Max);
						tilt_pos = CLAMP(tilt_pos + tilt_add, servo52Min, servo52Max);
						
						//Default handling in original Commander.c - sets to range of -127 to 127 or so...
						walkV = (signed char)( (int)vals[2]-128 );
						walkH = (signed char)( (int)vals[3]-128 );
					}
					// pan = (vals[0]<<8) + vals[1];
					// tilt = (vals[2]<<8) + vals[3];
					// buttons = vals[4];
					// ext = vals[5];
				}
				index_cmdr = -1;
				//LISTEN.flush(); //flush after reading an entire packet... why?
				uartFlushReceiveBuffer(LISTEN);
				return 1;
			}
		}
	}
	return 0;
}

void UseSouthPaw(){
	southpaw |= 0x01;
}