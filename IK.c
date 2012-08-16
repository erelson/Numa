#include "hardware.h"
#include "math.h"
#include "Maths/Vector2D.h"
#include "HeaderDefs.h"


// Method ???
void walkNewDirIK(int16_t new_dir){
	if(ang_dir == new_dir){ return; }
	
	//if not previously walking with IK...
	if(walk == zFALSE){
		g8Stand();  //note: walk is now FALSE; set walk after this.
		ang_dir = new_dir;
		// NEED TO SET TIMING HERE
	}
	// If too big a change in direction, change to standing position, then start fresh
	else if( (abs(new_dir - ang_dir) % 360) >= 20 ){
		g8Stand();  //note: walk is now FALSE; set walk after this.
		ang_dir = new_dir;
		//walk = zTRUE;
		//
	}
	// else... e.g. 
	else {
		ang_dir = new_dir;
	}
	return;
}


// Method receives a LOT of variables... and does IK calcualtions for the non-coax parts of a leg
void doLegKinematics(uint16_t myT, uint16_t* mys2pos, uint16_t* mys3pos, uint16_t* mys4pos, short posSwap, \
		 float cos_s1Ang, float sin_s1Ang, float my_trav_cdir, float my_trav_sdir, \
		 int16_t myFootH, float myTrav, short debug){

	// myPosSwap determines which direction to offset servo position from centered.
	short myPosSwap;
	if(posSwap == 1 || posSwap == -1){ myPosSwap = posSwap;	}
	else { myPosSwap = 1; }

	float legLen = L0;
	if(cos_s1Ang == 0 && sin_s1Ang == 0){ // Turning
		legLen = 1.05 * (float)L0;
	}
	else {
		//Top down x-y plane
		// Yes, my_trav_sdir goes to lenx and etc. (per measuring angle from 0 deg == forward, CCW)
		float lenx = L0 * cos_s1Ang + my_trav_sdir;
		float leny = L0 * sin_s1Ang + my_trav_cdir;

		//LEG LENGTH (length of projection of leg onto floor plane)
		legLen = sqrt( lenx * lenx + leny * leny ); //current length of leg 1
	}
	//PRINT LEG LENGTH
	// rprintf("%u,", myT);
	
	//now x y are side view of leg
	uint16_t len24x = legLen - L12;
	uint16_t len24y = bodyH - (L45/*could factor in foot angle*/ + myFootH); // body height minus height of point 4
	float L24 = sqrt( len24x * len24x + len24y * len24y); // Length btw points 2 and 4 in leg.
	float sqL24 = L24*L24;
	
	//angle between horizontal and hypotenuse (L24)
	float alph1 = acos(len24x/L24);
	//float alph1 = acos( ( legLen - L12 ) / L24); //old way same as new way but with more math whooops
	
	//angle... Law of cosines: cos(C) = (a a + b b - c c) / (2 a b)
	float alph2 = acos( (sqL34 + sqL24 - sqL23) / ( 2 * L34 * L24 ) );
	
	if(PRINT_DEBUG_IK && debug == 1){rprintf("IK: %u _ %u %u %f %f  ",myFootH,len24x,len24y,L24,(alph1)*57.3);}
	
	/////////////
	//Everything seems to be OK up to here
	///////////
	
	float c_alph1plus2 = cos(alph1 + alph2);
	float s_alph1plus2 = sin(alph1 + alph2);
	
	//Determine and create leg segment vectors
	//For diagram of math, see: .................. .nb
	//Orientation: Am I using the XY plane, not the (-X,Y) plane? YES
	VECTOR2D v12 = MAKE_VECTOR2D(-1, 0); //Assuming always horiz, but measured from vertical. 
	VECTOR2D v12vert = MAKE_VECTOR2D(0,1);
	VECTOR2D v23 = MAKE_VECTOR2D( L12 -  (legLen - L34 * c_alph1plus2), \
				bodyH - ((myFootH + L45) + L34 * s_alph1plus2) );
	VECTOR2D v34 = MAKE_VECTOR2D( -L34 * c_alph1plus2, \
				L34 * s_alph1plus2 ); //
	//VECTOR2D v34 = MAKE_VECTOR2D( (legLen - L34 * c_alph1plus2) - legLen, \
		//		((myFootH + L45) + L34 * s_alph1plus2) - (myFootH + L45) ); //
	
	// To avoid problems with sign of angle being incorrectly handled by vector2D_Angle
	// ... we instead compare a near right angle and later subtract Pi/2 radians
	VECTOR2D v45 = MAKE_VECTOR2D( -1 , 0 );
	//VECTOR2D v45 = MAKE_VECTOR2D( 0 /* ll1 - ll1 */, L45 /* + footH - footH */ );
	
	//For calculating the angles, we want both vectors v23 and v32... so we make v32 by inverting v23.
	VECTOR2D v32 = v23;
	vector2d_Scale(&v32, -1);
	
	//Calculate the angles with scale factor 195.2 for radians ->  300deg/1022 scale
	//   .... Note that the offsetServo variables are sometimes negative.
	*mys2pos = 511 + myPosSwap * ( (M_PI/2.0 - vector2d_AngleRadians(&v23, &v12vert) ) * 195.2 + offsetServo2); // 70.08 ;
	*mys3pos = 511 + myPosSwap * ((M_PI - vector2d_AngleRadians(&v34, &v32)) * 195.2 + offsetServo3); //- 70.08);
	*mys4pos = 511 - myPosSwap * ((vector2d_AngleRadians(&v34, &v45) - M_PI/2) * -195.2 + offsetServo4); //+ 39.19);
	//*mys4pos = 511 - myPosSwap * (vector2d_AngleRadians(&v34, &v45) * 195.2 + offsetServo4); //+ 39.19);

	if(PRINT_DEBUG_IK && debug == 1){rprintf("_ %f *%f %f %d *%d %d ", vector2d_AngleRadians(&v23, &v12), \
				vector2d_AngleRadians(&v34, &v32), \
				vector2d_AngleRadians(&v34, &v45), \
				*mys2pos, \
				*mys3pos, \
				*mys4pos \
				);}
	
	return;
}


//
// uint8_t calc_foot_h(int16_t now, uint16_t foot_h_max, float time_down_frac, 
	// int16_t half_loopLength){
	// // Important note, now must be in the range from 0 to 2 * half_loopLength
	
	// int8_t my_footH = 0;
	
	// if(now >= (2 - time_down_frac) * half_loopLength ){
		// my_footH = (int)(foot_h_max * ((float)(2*half_loopLength - now))/ \
			// (time_down_frac * half_loopLength));
		// // my_footH = (int)(foot_h_max - foot_h_max * ((float)(2*half_loopLength - now))/ \
			// // (time_down_frac * half_loopLength));
		// // my_footH = 0;
	// }
	// else if(now >= (1 + time_down_frac) * half_loopLength){
		// my_footH = foot_h_max;
	// }
	// else if(now >= half_loopLength){	
		// my_footH = (int)(foot_h_max * ((float)(now - half_loopLength))/ \
			// (time_down_frac * half_loopLength));
		// // my_footH = 0;
	// }
	// else {
		// my_footH = 0;
	// }
	
	// return my_footH;
// }
uint8_t calc_foot_h(int16_t now, uint16_t foot_h_max, float time_down_frac, 
	int16_t half_loopLength, float transition_frac, float height_frac){
	// Important note, now must be in the range from 0 to 2 * half_loopLength
	
	int8_t my_footH = 0;
	
	if(now >= (2.0 - time_down_frac) * half_loopLength ){
		my_footH = (int)(height_frac * foot_h_max * ((float)(2.0 * half_loopLength - now)) / \
			(time_down_frac * half_loopLength));
	}
	else if(now >= (2.0 - transition_frac) * half_loopLength){
		my_footH = (int)((height_frac * foot_h_max + (1.0 - height_frac) * foot_h_max) * \
			((2.0 - time_down_frac) * half_loopLength - now) / \
			((transition_frac - time_down_frac) * half_loopLength));
	}
	else if(now >= (1 + transition_frac) * half_loopLength){
		my_footH = foot_h_max;
	}
	else if(now >= half_loopLength * (1.0 + time_down_frac)){	
		my_footH = (int)(height_frac * foot_h_max + (1.0 - height_frac)*foot_h_max * \
			((float)(now - half_loopLength * (1.0 + time_down_frac)) / \
			((transition_frac - time_down_frac) * half_loopLength)));
	}
	else if(now >= half_loopLength){
		my_footH = (int)(height_frac*foot_h_max*((float)(now - half_loopLength)) / \
			(time_down_frac * half_loopLength));
	}
	else {
		my_footH = 0;
	}
	
	return my_footH;
}