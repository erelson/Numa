#include <math.h>
#include "HeaderDefs.h"

#define zTRUE 1
#define zFALSE 0

#define RTN_LVL 1
#define MY_COAX_SPEED 200
#define MY_SERVO_SPEED 300
#define MY_TURRET_SERVO_SPEED 200

void initTrig();  //function prototype
					//function was in servoPosConsts.h

void initServoLims();  //function prototype

void setAXposition(int16_t, DYNAMIXEL_AX12, int16_t, int16_t);
void myServoSpeeds(DYNAMIXEL_AX12_LIST*);
void myServoReturnLevels(DYNAMIXEL_AX12_LIST*);
void g8Stand();
void g8Flop();
void walkNewDirIK(int16_t);

//void doLegKinematics(uint16_t, uint16_t*, uint16_t*, uint16_t*, short, float, float, float, float, int16_t, float);

//Trig

//Servo position variables; defaulting these to centered...
uint16_t s11pos = 511;
uint16_t s12pos = 511;
uint16_t s13pos = 511;
uint16_t s14pos = 511;

uint16_t s21pos = 511;
uint16_t s22pos = 511;
uint16_t s23pos = 511;
uint16_t s24pos = 511;

uint16_t s31pos = 511;
uint16_t s32pos = 511;
uint16_t s33pos = 511;
uint16_t s34pos = 511;

uint16_t s41pos = 511;
uint16_t s42pos = 511;
uint16_t s43pos = 511;
uint16_t s44pos = 511;

uint16_t s51pos = 511; // added a 45° offset
uint16_t s52pos = 511;

//Servo position limits, range from 0 to 1023
uint16_t servo11Min = 511 - 4 * 16; 
uint16_t servo11Max = 511 + 4 * 55;
uint16_t servo12Min = 511 - 4 * 84;
uint16_t servo12Max = 511 + 4 * 27;
uint16_t servo13Min = 511 - 4 * 77;
uint16_t servo13Max = 511 + 4 * 87;
uint16_t servo14Min = 511 - 4 * 77;
uint16_t servo14Max = 511 + 4 * 77;

uint16_t servo21Min = 511 - 4 * 55;
uint16_t servo21Max = 511 + 4 * 16;
uint16_t servo22Min = 511 - 4 * 27;
uint16_t servo22Max = 511 + 4 * 84;
uint16_t servo23Min = 511 - 4 * 87;
uint16_t servo23Max = 511 + 4 * 77;
uint16_t servo24Min = 511 - 4 * 77;
uint16_t servo24Max = 511 + 4 * 77;

uint16_t servo31Min = 511 - 4 * 16;
uint16_t servo31Max = 511 + 4 * 55;
uint16_t servo32Min = 511 - 4 * 84;
uint16_t servo32Max = 511 + 4 * 27;
uint16_t servo33Min = 511 - 4 * 77;
uint16_t servo33Max = 511 + 4 * 87;
uint16_t servo34Min = 511 - 4 * 77;
uint16_t servo34Max = 511 + 4 * 77;

uint16_t servo41Min = 511 - 4 * 55;
uint16_t servo41Max = 511 + 4 * 16;
uint16_t servo42Min = 511 - 4 * 27;
uint16_t servo42Max = 511 + 4 * 84;
uint16_t servo43Min = 511 - 4 * 87;
uint16_t servo43Max = 511 + 4 * 77;
uint16_t servo44Min = 511 - 4 * 77;
uint16_t servo44Max = 511 + 4 * 77;

// See WalkingOmni.nb
//Center angles for coax servo of each leg.plus test offsets.
//Measured from 0deg = front of bot  --------- this doesn't look right.
///But are the 
// #define s11A0 225
// #define s11Aoff 10
// #define s21A0 -45 //315 //Hmmm no idea here....
// #define s21Aoff -10
// #define s31A0 45
// #define s31Aoff 0
// #define s41A0 135
// #define s41Aoff 10


// #define c_s11A0 cos(s11A0 + s11Aoff)
// #define s_s11A0 sin(s11A0 + s11Aoff)
// #define c_s21A0 cos(s21A0 + s21Aoff)
// #define s_s21A0 sin(s21A0 + s21Aoff)
// #define c_s31A0 cos(s31A0 + s31Aoff)
// #define s_s31A0 sin(s31A0 + s31Aoff)
// #define c_s41A0 cos(s41A0 + s41Aoff)
// #define s_s41A0 sin(s41A0 + s41Aoff)


//Center angles for each leg.
float servo11Ang = M_PI / 180 * (s11A0 + s11Aoff); //radians
float servo21Ang = M_PI / 180 * (s21A0 + s21Aoff);
float servo31Ang = M_PI / 180 * (s31A0 + s31Aoff);
float servo41Ang = M_PI / 180 * (s41A0 + s41Aoff);

float cos_servo11Ang = 0;
float sin_servo11Ang = 0;
float cos_servo21Ang = 0;
float sin_servo21Ang = 0;
float cos_servo31Ang = 0;
float sin_servo31Ang = 0;
float cos_servo41Ang = 0;
float sin_servo41Ang = 0;

// Leg consts
uint16_t bodyH = 130 + 20; //mm

uint16_t L0 = 105 + 5; //mm - pretty close to actual...

//Leg part lengths
#define preL12 58
#define preL23 63
#define preL34 67
#define preL45 57

uint16_t y11 = 0;
uint16_t y21 = 0;
uint16_t y31 = 0;
uint16_t y41 = 0;

int L12 = preL12;
int L23 = preL23;
int L34 = preL34;
int L45 = preL45;

int sqL23 = preL23 * preL23;
int sqL34 = preL34 * preL34;
//not used:
//int sqL45 = preL45 * preL45;

//????????
int x11 = preL12;
int x21 = preL12;
int x31 = preL12;
int x41 = preL12;

//Leg ServoPos Offsets
//  this is in units of... 10bit position?
//  currently not used... manually entered values are in 
int offsetServo1 = 0;
int offsetServo2 = -237; // 70; 
int offsetServo3 = -70;
int offsetServo4 = 0*39 - 0*20; //39 per CAD with original feet.

///////////////////////
/////TURRET////////////
///////////////////////
//+153 is 45 deg offset for pan servo's mounting scheme.
// #define PAN_CENTER 511 + 153 
// #define TILT_CENTER 511 + 95

int d_tilt = 9;
int d_pan = 12;
int tilt_pos = TILT_CENTER; // where tilt servo ought to be
int pan_pos = PAN_CENTER;  // where pan servo ought to be

// 4*52 -> 60 degrees?
uint16_t servo51Min = PAN_CENTER - 4 * (52+30);// - 200;
uint16_t servo51Max = PAN_CENTER + 4 * (52+30);// + 200;
uint16_t servo52Min = 511 - 4 * 31;// - 200;
uint16_t servo52Max = 511 + 4 * 65;// + 200;



short turnright = zFALSE;
short turnleft = zFALSE;
short gunbutton = zFALSE;
short panicbutton = zFALSE;
short infobutton = zFALSE;
short agitbutton = zFALSE;

TICK_COUNT agitate = 0;

int walkV = 0;
int walkH = 0;
int lookV = 0;
int lookH = 0;
int walkSPD = 0;
float walkDIR = 0;
