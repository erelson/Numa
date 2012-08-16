#include "hardware.h"
#include <math.h>
#include "Maths/Vector2D.h"
#include <Gait/GaitRunner.h>

#include "HeaderDefs.h"

void initTrig()
{
	cos_servo11Ang = cos(servo11Ang);
	sin_servo11Ang = sin(servo11Ang);
	cos_servo21Ang = cos(servo21Ang);
	sin_servo21Ang = sin(servo21Ang);
	cos_servo31Ang = cos(servo31Ang);
	sin_servo31Ang = sin(servo31Ang);
	cos_servo41Ang = cos(servo41Ang);
	sin_servo41Ang = sin(servo41Ang);
	return;
}

void initServoLims()
{
	ax12SetCCW_ANGLE_LIMIT( &servo11,servo11Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo11,servo11Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo12,servo12Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo12,servo12Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo13,servo13Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo13,servo13Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo14,servo14Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo14,servo14Min);
	delay_ms(30);
	
	ax12SetCCW_ANGLE_LIMIT( &servo21,servo21Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo21,servo21Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo22,servo22Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo22,servo22Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo23,servo23Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo23,servo23Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo24,servo24Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo24,servo24Min);
	delay_ms(30);
	
	ax12SetCCW_ANGLE_LIMIT( &servo31,servo31Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo31,servo31Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo32,servo32Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo32,servo32Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo33,servo33Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo33,servo33Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo34,servo34Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo34,servo34Min);
	delay_ms(30);
	
	ax12SetCCW_ANGLE_LIMIT( &servo41,servo41Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo41,servo41Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo42,servo42Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo42,servo42Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo43,servo43Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo43,servo43Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo44,servo44Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo44,servo44Min);
	delay_ms(30);
	
	ax12SetCCW_ANGLE_LIMIT( &servo51,servo51Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo51,servo51Min);
	delay_ms(30);
	ax12SetCCW_ANGLE_LIMIT( &servo52,servo52Max);
	delay_ms(30);
	ax12SetCW_ANGLE_LIMIT(&servo52,servo52Min);

}