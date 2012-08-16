#include "hardware.h"
#include <math.h>
#include "Maths/Vector2D.h"
#include <Gait/GaitRunner.h>

#include "HeaderDefs.h"

#include "servoPosConsts.h"
#include "timingConsts.h"

#include "Commander.h"

#include "gait2.h"
//#endif

short derp;
short turn_loops;
short turn_dir;
int8_t standing;

#define zTRUE 1
#define zFALSE 0

//#define LISTEN UART2toXbee38400
//#define LISTEN uart1

#define TRAV_DEFAULT 25

//#define TURNG8 G8_ANIM_TURNLEFT
#define TURNG8 G8_ANIM_TURNSLOW

#define PROG_LOOP_TIME 19500 // in microseconds

#define AGITATE_TIME 1000 //milliseconds - how long it takes to do an agitate sequence.

#define USE_ONE_SPEED 0
#define THE_ONE_SPEED 3
#define THE_TURN_SPEED 7
#define MAX_WALK_SPD 5

									
ACTUATOR_LIST PROGMEM all[] = {&servo11.actuator,&servo21.actuator,&servo31.actuator,&servo41.actuator,
						&servo12.actuator,&servo22.actuator,&servo32.actuator,&servo42.actuator,
						&servo13.actuator,&servo23.actuator,&servo33.actuator,&servo43.actuator,
						&servo14.actuator,&servo24.actuator,&servo34.actuator,&servo44.actuator};

// This list is used to set values on all servos.
static DYNAMIXEL_AX12_LIST AX12_driver_list[] = {
	&servo1,
	&servo11,&servo12,&servo13,&servo14,
	&servo21,&servo22,&servo23,&servo24,
	&servo31,&servo32,&servo33,&servo34,
	&servo41,&servo42,&servo43,&servo44,
	&servo51,&servo52
};

G8_RUNNER gait = MAKE_G8_RUNNER(all,animations);

short walk = zFALSE;
short turn = zFALSE;
short light = zTRUE;
short kneeling = zFALSE;
short flopCnt = 0;
short panic = zFALSE;

//Command settings/interpretation variables
int currentAnim = 0;
int playingAnim = -1;

int IRcnt = 1;

///MATHEMATICA CODE
///loopSpeed = 1000;
///Plot[65.536*loopSpeed/speed, {speed, 0, 128}, PlotRange -> {500, 4000}]
const int g8loopSpeed = 1000;
int g8speed = 25;
int g8playbackDir = 1; // value should only ever be -1 or 1.
int g8repeatCount = 0;
int ch = '!';
int ch_old = '?';
int do_gait = 1;
short guns_firing = zFALSE;

//Setting default walking variables...
int16_t loopLength = 0;// = 1800;
//int16_t half_loopLength = 0;// = loopLength/2; //reduced the scope and made this local where its used...
int16_t travRate;// = 0;//could be a uint8_t
int16_t double_travRate = 0;//= 2 * travRate;
TICK_COUNT turnTimeOffset = 0;

const int16_t travRate_default = TRAV_DEFAULT;
int16_t ang_dir = 0;
TICK_COUNT oldLoopStart = 0;

// Initialise the hardware
void appInitHardware(void) {
	initHardware();
	// gaitRunnerInit(&gait);
	// act_setConnected(&antijam, zFALSE);  // Setting this later so servo centers initially.
}

// Initialise the software
TICK_COUNT appInitSoftware(TICK_COUNT loopStart){
	rprintf("It begins.... \n");
	initTrig();
	
	//ax12SetID(&servo1, 1);
	
	//Call gait for Standing
	g8Stand(); 

	myServoReturnLevels(AX12_driver_list);
	rprintf("ServoReturnLevelsSet!\n");
	initServoLims();
	rprintf("ServoLimsSet!\n");
	myServoSpeeds(AX12_driver_list);
	rprintf("ServoSpeedsSet!\n");

	act_setConnected(&antijam, zFALSE);  // Stop microservo
	
	//Setting mathy initial values for walking
	loopLength = 1800;
	//half_loopLength = loopLength/2; redundant
	travRate = travRate_default;
	double_travRate = 2 * travRate;

	standing = 1;
	g8Stand();

	return 0;
}
// This is the main loop
TICK_COUNT appControl(LOOP_COUNT loopCount, TICK_COUNT loopStart) {

	if (PRINT_DEBUG_LOOP){
		rprintf("\n%ld", loopStart - oldLoopStart);
		oldLoopStart = loopStart;
	}
	
	// Stop IK and g8s from coinciding... make Numa stop in place.
//	if (walk == zTRUE && do_gait == zTRUE){ g8Stand(); }
	
	// -------- Start Switch/Button-------
	// Switch/Button - see switch.
	// To test if it is pressed then
	if (SWITCH_pressed(&button)){

        // Triggers gun test					//Want to run motors at 7.2V, so do PWM:
		act_setSpeed(&LeftGun,-70); 	//NOTE: (7.2V / 12.6V) * 127 = 72.5714286
		act_setSpeed(&RightGun,-70); 	//NOTE: (7.2V / 12.6)V * 127 = 72.5714286

		// pressed
		// We use the light variable to toggle stuff.
		if (light == zTRUE){
			LED_on(&statusLED);
			light = zFALSE;
			//rprintf("on!\n");
		}

		else {
			LED_off(&statusLED);
			light = zTRUE;
			//rprintf("off\n");
		}
	}
	
	//Check whether to stop firing guns
	if (guns_firing && clockHasElapsed(guns_firing_start_time, guns_firing_duration)){
		guns_firing_duration = 0;
		guns_firing = zFALSE;
		act_setSpeed(&LeftGun,0); 	//NOTE: (7.2 / 12.6) * 127 = 72.5714286
		act_setSpeed(&RightGun,0); 	//NOTE: (7.2 / 12.6) * 127 = 72.5714286
		guns_firing_start_time = clockGetus();
	}
	
	// To test if it is released then
	if(SWITCH_released(&button)){
		// released
		act_setSpeed(&LeftGun, 0);
		act_setSpeed(&RightGun, 0);
		
	}
	// -------- End   Switch/Button-------

	// -------- Start Dynamixel AX-12 Driver-------
	// Dump the current values for all servos on AX12_driver to rprintf
//	ax12DumpAll(&AX12_driver);
	// -------- End   Dynamixel AX-12 Driver-------

	///////////////////////////////////////////
	
	CmdrReadMsgs();
	
	TICK_COUNT ms = (loopStart) / 1000 + spdChngOffset;		// Get current time in ms

	//We always move the turret to the position specified by the Commander.
	ax12SetGOAL_POSITION(&servo52, tilt_pos);
	ax12SetGOAL_POSITION(&servo51, pan_pos);

	if (infobutton){
		ax12TempAll(&UART3toAX12_driver);
		infobutton = zFALSE;
	}
	
	if (panicbutton){
		flopCnt += 1;
		// rprintf("%d\t",flopCnt);
		if( flopCnt >= 3){
			g8Crouch();
			panic = zTRUE;
			rprintf("\nHowdydoo? %d\n",flopCnt);
			flopCnt = 0;
		}
		else{
			// Exit crouch/panic, enable standing, and re-enable torque to 2nd servo of each leg.
			panic = zFALSE;
			standing = 0;
			
			ax12SetTORQUE_ENABLE(&servo12, 1);
			ax12SetTORQUE_ENABLE(&servo22, 1);
			ax12SetTORQUE_ENABLE(&servo32, 1);
			ax12SetTORQUE_ENABLE(&servo42, 1);
			
		}
		panicbutton = zFALSE;
		//rprintf("poink\n");

		// Limit to one press toggling at a time.
		delay_ms(100);
		uartFlushReceiveBuffer(LISTEN);
	}

	//FIRE THE GUNS!!!!!
	if (gunbutton){
		guns_firing = zTRUE;
		act_setSpeed(&LeftGun,-65); 	//NOTE: (7.2 / 12.6) * 127 = 72.5714286
		act_setSpeed(&RightGun,-65); 	//NOTE: (7.2 / 12.6) * 127 = 72.5714286
		guns_firing_start_time = clockGetus();
	}

	agitatorLogic((loopStart) / 1000);
	
	//We put "panic" before anything else that might move the legs.
	if(panic){ return 25000; } //micro seconds
	
	// Decrement turn_loops continuously
	if (turn_loops > 0) { turn_loops -= 1;}
	
	if(turnleft || turnright){
		// LED_off(&statusLED);
		if(PRINT_DEBUG){rprintf("Turn!  %u\t%u\n",turnright, turnleft);}
		
		if(turn_loops < 1) {
			loopLength = loopLengthList[THE_TURN_SPEED - 1];
			// Two parts:
			// 1) how far 'ms' is from the beginning of a turn animation
			// 2) how far from the beginning of a turn animation we want to start at
			turnTimeOffset = (ms % loopLength) - (TICK_COUNT)(0.2 * loopLength);
		}
		
		turn = zTRUE;
		turn_loops = 20;
		turn_dir = 1;
		if(turnleft){ turn_dir = -1; } // Reverse turn dir here
		
		standing = 0;
	}
	
	else if(turn_loops > 0) {turn_loops = turn_loops;} // ?
	
	//Else, walking, possibly
	else{
		
		turnTimeOffset = 0;
		
		// walkNewDirIK(0);
		walkSPD = sqrt(walkV*walkV+walkH*walkH);
		walkDIR = atan2(walkH,walkV);
		
		walkSPD = interpolate(walkSPD, 0,102, 0,6);
		
		if(walkSPD == 0 && turn_loops == 0){
			//g8Stand();
			walk = zFALSE;
			if (standing < 6) { standing += 1;}
		}
		
		else if (walkSPD > 0){
			// Debug info
			if(PRINT_DEBUG){rprintf("walk! %f ", (walkDIR * 180.0 / M_PI));}
		
			//Disable turning when walking joystick is moved.
			turn_loops = 0;  //REDUNDANT
			//Disable standing g8
			standing = 0;
		
			if(walkSPD > MAX_WALK_SPD){ walkSPD = MAX_WALK_SPD; }
			else if(walkSPD == 1 || walkSPD == 2){ walkSPD = 3; }
			
			if(USE_ONE_SPEED){
				walkSPD = THE_ONE_SPEED;
			}
			
			int16_t newLoopLength = loopLengthList[walkSPD - 1]; //Temp storage
			if(newLoopLength != loopLength){					// So we can check for change
				spdChngOffset += speedPhaseFix(loopStart, loopLength, newLoopLength);
				loopLength = newLoopLength;
				//spdChngOffset = spdChngOffset%loopLength;
			}
			walk = zTRUE;
			walkNewDirIK((int16_t)(walkDIR * 180.0 / M_PI));
		}
		do_gait = zFALSE;
	}
	//////////////////////////////////////////
	
	int16_t half_loopLength = loopLength/2;
	//travRate = 30 - 10; // this was redundant
	double_travRate = 2 * travRate;

	////////////////////////////
	// -------- Start Leg stuff-------

	//the 'now' variables are essentially sawtooth waves.
	now2 = (ms - turnTimeOffset) % (TICK_COUNT)loopLength;

	now3 = (ms - turnTimeOffset +  (TICK_COUNT)half_loopLength) % (TICK_COUNT)loopLength;

	now4 = (TICK_COUNT)loopLength - (ms - turnTimeOffset) % (TICK_COUNT)loopLength;

	now1 = (TICK_COUNT)loopLength - (ms - turnTimeOffset + (TICK_COUNT)half_loopLength) % (TICK_COUNT)loopLength;

	/// Above is where the commander input is interpretted.
	///
	/// The next few blocks are where we determine what gait to use.
	///
	///
	
	// WALKING WITH IK
	//if(0){  //Disables walking
	if(walk == zTRUE && turn_loops == 0){
		
		walkCode(loopLength, half_loopLength, travRate, double_travRate);
	// //Do this in the middle of the calculations to give guns a better firing time accuracy
		// if(guns_firing && clockHasElapsed(guns_firing_start_time, guns_firing_duration)){
			// guns_firing_duration = 0;
			// guns_firing = zFALSE;
			// act_setSpeed(&LeftGun,0); 	//NOTE: (7.2 / 12.6) * 127 = 72.5714286
			// act_setSpeed(&RightGun,0); 	//NOTE: (7.2 / 12.6) * 127 = 72.5714286
			// guns_firing_start_time = clockGetus();
		// }		
	}

	//Turnning with IK
	else if(turn_loops > 0 && walk == zFALSE){
	    
		turnCode(turn_dir, loopLength, half_loopLength);
				
		//rprintf("\n%d\t%d\t%d\t%d",s12pos,s42pos, footH13,footH24);
	}
	
	else if(standing > 0 && standing <= 5) {
		// g8Stand();
		g8FeetDown();
	}
	
	else if(turn_loops > 0 && walk == zTRUE){
		// g8Stand();
		g8FeetDown();
	}
	
	turnright = zFALSE;
	turnleft = zFALSE;
	
	
	/**** */
	///Move all servos
	if(walk == zTRUE || turn_loops > 0){
		ax12SetGOAL_POSITION(&servo11, s11pos); //, servo11, servo11Max, servo11Min);
		ax12SetGOAL_POSITION(&servo21, s21pos); //, servo21, servo21Max, servo21Min);
		ax12SetGOAL_POSITION(&servo31, s31pos); //, servo31, servo31Max, servo31Min);
		ax12SetGOAL_POSITION(&servo41, s41pos); //, servo41, servo41Max, servo41Min);

		ax12SetGOAL_POSITION(&servo12, s12pos); //, servo21, servo21Max, servo21Min);
		ax12SetGOAL_POSITION(&servo13, s13pos); //, servo31, servo31Max, servo31Min);
		ax12SetGOAL_POSITION(&servo14, s14pos); //, servo41, servo41Max, servo41Min);

		ax12SetGOAL_POSITION(&servo22, s22pos); //, servo21, servo21Max, servo21Min);
		ax12SetGOAL_POSITION(&servo23, s23pos); //, servo31, servo31Max, servo31Min);
		ax12SetGOAL_POSITION(&servo24, s24pos); //, servo41, servo41Max, servo41Min);

		ax12SetGOAL_POSITION(&servo32, s32pos); //, servo21, servo21Max, servo21Min);
		ax12SetGOAL_POSITION(&servo33, s33pos); //, servo31, servo31Max, servo31Min);
		ax12SetGOAL_POSITION(&servo34, s34pos); //, servo41, servo41Max, servo41Min);

		ax12SetGOAL_POSITION(&servo42, s42pos); //, servo21, servo21Max, servo21Min);
		ax12SetGOAL_POSITION(&servo43, s43pos); //, servo31, servo31Max, servo31Min);
		ax12SetGOAL_POSITION(&servo44, s44pos); //, servo41, servo41Max, servo41Min);
		
		// // ax12SetGOAL_POSITION(&servo11, s31pos);
		// ax12SetGOAL_POSITION(&servo12, s12pos);
		// ax12SetGOAL_POSITION(&servo13, s13pos);
		// ax12SetGOAL_POSITION(&servo14, s14pos);
		
		// // ax12SetGOAL_POSITION(&servo31, s31pos);
		// ax12SetGOAL_POSITION(&servo32, s12pos);
		// ax12SetGOAL_POSITION(&servo33, s13pos);
		// ax12SetGOAL_POSITION(&servo34, s14pos);
	}

	
	if(PRINT_IR_RANGE){
		IRcnt += 1;
		
		if(IRcnt >= 8){
			
			distanceRead(distance);
			rprintf("L");
			distanceDump(distance);
			rprintf("\t");
			
			distanceRead(distance2);
			rprintf("R");
			distanceDump(distance2);
			rprintfCRLF();
			
		IRcnt = 0;
		}
	}
	
	if(PRINT_DEBUG && walk == zTRUE){rprintf("\n");}
	// else if(PRINT_DEBUG_IK == zTRUE && turn == zTRUE){rprintf("\n");}
	else if(PRINT_DEBUG_IK == zTRUE && turn_loops > 0){rprintf("\n");}
	
	return PROG_LOOP_TIME;//45000; //micro seconds
	//return 0;
}


void myServoSpeeds(DYNAMIXEL_AX12_LIST* myList){

	for(uint16_t cnt = 0; cnt < 16; cnt++){
		ax12SetMOVING_SPEED( (AX12_driver_list[cnt]), MY_SERVO_SPEED);
		delay_ms(25);
	}
	ax12SetMOVING_SPEED(&servo14, MY_COAX_SPEED);
	delay_ms(30);
	ax12SetMOVING_SPEED(&servo24, MY_COAX_SPEED);
	delay_ms(30);
	ax12SetMOVING_SPEED(&servo34, MY_COAX_SPEED);
	delay_ms(30);
	ax12SetMOVING_SPEED(&servo44, MY_COAX_SPEED);
	delay_ms(30);
	ax12SetMOVING_SPEED(&servo52, MY_TURRET_SERVO_SPEED);
	delay_ms(30);
	ax12SetMOVING_SPEED(&servo51, MY_TURRET_SERVO_SPEED);
}


void myServoReturnLevels(DYNAMIXEL_AX12_LIST* myList){

	for(uint16_t cnt = 0; cnt < 18; cnt++){
		ax12SetSTATUS_RTN_LEVEL( (AX12_driver_list[cnt]), RTN_LVL);
		delay_ms(25);
	}
}


TICK_COUNT speedPhaseFix(TICK_COUNT clocktime, TICK_COUNT loopLenOld, TICK_COUNT loopLen){
    //rprintf("%lu\t%lu\t%lu\t",clocktime,loopLenOld,loopLen);
    return (TICK_COUNT)((((clocktime/1000)%loopLenOld)/((float)loopLenOld) - \
        ((clocktime/1000)%loopLen)/((float)loopLen))*loopLen);
}

// Method runs anti-jam servo
void agitatorLogic(TICK_COUNT ms_b){
	short n_cycles = 1;
	if(agitbutton == zTRUE && agitate == 0){
		if(PRINT_DEBUG_AGITATOR){rprintf("agitate loop start\t");}
		agitate = ms_b;
		act_setConnected(&antijam, zTRUE);
	}
	else if (agitate > 0) {
		// If we have exceeded the agitate time length...
		if ((ms_b - agitate) > AGITATE_TIME) {
			agitate = 0;
			act_setConnected(&antijam, zFALSE);
			if(PRINT_DEBUG_AGITATOR){rprintf("agitate loop end\t");}
		}
		// Else, interpolate the position for the servo.
		else {
			int16_t now = abs( \
					(((ms_b - agitate) + AGITATE_TIME / (n_cycles*4)) % (AGITATE_TIME/n_cycles)) - AGITATE_TIME/6); 	// 
			// Map it into DRIVE_SPEED range
			DRIVE_SPEED speed = interpolate(now, 0, AGITATE_TIME/(2*n_cycles), -97+15, 97+15); // DRIVE_SPEED_MIN, DRIVE_SPEED_MAX);
			// Set speed for all motors/servos
			act_setSpeed(&antijam,speed);
		}
	}
	else if (agitate < 0){ // Not possible to go negative? TBD...
		if(PRINT_DEBUG_AGITATOR){rprintf("zeroing from negative\t%d\t", agitate);}
		agitate = 0;
		act_setConnected(&antijam, zFALSE);
	}
}