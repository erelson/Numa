#include "hardware.h"
#include <math.h>
#include "Maths/Vector2D.h"
#include <Gait/GaitRunner.h>

#include "HeaderDefs.h"

// Send standing positions to all servos.
void g8Stand(){

	ax12SetGOAL_POSITION(&servo11, 639 + 20);
	ax12SetGOAL_POSITION(&servo21, 383 - 20);
	ax12SetGOAL_POSITION(&servo31, 639);
	ax12SetGOAL_POSITION(&servo41, 383);
	ax12SetGOAL_POSITION(&servo12, 358);
	ax12SetGOAL_POSITION(&servo22, 664);
	ax12SetGOAL_POSITION(&servo32, 358);
	ax12SetGOAL_POSITION(&servo42, 664);
	ax12SetGOAL_POSITION(&servo13, 664);
	ax12SetGOAL_POSITION(&servo23, 358);
	ax12SetGOAL_POSITION(&servo33, 664);
	ax12SetGOAL_POSITION(&servo43, 358);
    
	ax12SetGOAL_POSITION(&servo14, 511);//409+39);
	ax12SetGOAL_POSITION(&servo24, 511);//613-39);
	ax12SetGOAL_POSITION(&servo34, 511);//409+39);
	ax12SetGOAL_POSITION(&servo44, 511);//613-39);
	
//	delay_ms(2000); //probably too short, but a long wait is scary, too.
	//gaitRunnerStop(&gait); // unnecessary?
	
	//stop IK and Gait from processing, whichever was active...
	walk = zFALSE;
	do_gait = zFALSE;
}

// Send standing positions to all servos. BUT don't rotate legs to center position
void g8FeetDown(){

	// ax12SetGOAL_POSITION(&servo11, 639 + 20);
	// ax12SetGOAL_POSITION(&servo21, 383 - 20);
	// ax12SetGOAL_POSITION(&servo31, 639);
	// ax12SetGOAL_POSITION(&servo41, 383);
	ax12SetGOAL_POSITION(&servo12, 358);
	ax12SetGOAL_POSITION(&servo22, 664);
	ax12SetGOAL_POSITION(&servo32, 358);
	ax12SetGOAL_POSITION(&servo42, 664);
	ax12SetGOAL_POSITION(&servo13, 664);
	ax12SetGOAL_POSITION(&servo23, 358);
	ax12SetGOAL_POSITION(&servo33, 664);
	ax12SetGOAL_POSITION(&servo43, 358);
    
	ax12SetGOAL_POSITION(&servo14, 511);//409+39);
	ax12SetGOAL_POSITION(&servo24, 511);//613-39);
	ax12SetGOAL_POSITION(&servo34, 511);//409+39);
	ax12SetGOAL_POSITION(&servo44, 511);//613-39);
	
//	delay_ms(2000); //probably too short, but a long wait is scary, too.
	//gaitRunnerStop(&gait); // unnecessary?
	
	//stop IK and Gait from processing, whichever was active...
	walk = zFALSE;
	do_gait = zFALSE;
}

// Send standing positions to all servos.
void g8Flop(){

	ax12SetGOAL_POSITION(&servo11, 639 + 20);
	ax12SetGOAL_POSITION(&servo21, 383 - 20);
	ax12SetGOAL_POSITION(&servo31, 639);
	ax12SetGOAL_POSITION(&servo41, 383);

	ax12SetGOAL_POSITION(&servo12, 358 - 105);
	ax12SetGOAL_POSITION(&servo22, 664 + 105);
	ax12SetGOAL_POSITION(&servo32, 358 - 105);
	ax12SetGOAL_POSITION(&servo42, 664 + 105);

	ax12SetGOAL_POSITION(&servo13, 358 - 130);
	ax12SetGOAL_POSITION(&servo23, 664 + 130);
	ax12SetGOAL_POSITION(&servo33, 358 - 130);
	ax12SetGOAL_POSITION(&servo43, 664 + 130);
    
	ax12SetGOAL_POSITION(&servo14, 511);//409+39);
	ax12SetGOAL_POSITION(&servo24, 511);//613-39);
	ax12SetGOAL_POSITION(&servo34, 511);//409+39);
	ax12SetGOAL_POSITION(&servo44, 511);//613-39);
	
//	delay_ms(2000); //probably too short, but a long wait is scary, too.
	//gaitRunnerStop(&gait); // unnecessary?
	
	//stop IK and Gait from processing, whichever was active...
	walk = zFALSE;
	do_gait = zFALSE;
}

void g8Crouch(){

	// ax12SetGOAL_POSITION(&servo11, 639 + 20);
	// ax12SetGOAL_POSITION(&servo21, 383 - 20);
	// ax12SetGOAL_POSITION(&servo31, 639);
	// ax12SetGOAL_POSITION(&servo41, 383);

	ax12SetGOAL_POSITION(&servo12, 358 - 175);
	ax12SetGOAL_POSITION(&servo22, 664 + 175);
	ax12SetGOAL_POSITION(&servo32, 358 - 175);
	ax12SetGOAL_POSITION(&servo42, 664 + 175);

	ax12SetGOAL_POSITION(&servo13, 358 + 350 + 150);
	ax12SetGOAL_POSITION(&servo23, 664 - 350 - 150);
	ax12SetGOAL_POSITION(&servo33, 358 + 350 + 150);
	ax12SetGOAL_POSITION(&servo43, 664 - 350 - 150);
    
	ax12SetGOAL_POSITION(&servo14, 511);//409+39);
	ax12SetGOAL_POSITION(&servo24, 511);//613-39);
	ax12SetGOAL_POSITION(&servo34, 511);//409+39);
	ax12SetGOAL_POSITION(&servo44, 511);//613-39);
	
//	delay_ms(2000); //probably too short, but a long wait is scary, too.
	//gaitRunnerStop(&gait); // unnecessary?
	
	//stop IK and Gait from processing, whichever was active...
	walk = zFALSE;
	do_gait = zFALSE;
	
	delay_ms(200);
	
	ax12SetTORQUE_ENABLE(&servo12, 0);
	ax12SetTORQUE_ENABLE(&servo22, 0);
	ax12SetTORQUE_ENABLE(&servo32, 0);
	ax12SetTORQUE_ENABLE(&servo42, 0);
	
}
