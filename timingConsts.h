////////////////////
/////Timing/////////
///////////////////
TICK_COUNT guns_firing_start_time = 4294967294;
TICK_COUNT guns_firing_start_time_default = 4294967294;
TICK_COUNT guns_firing_duration = 250000; // shooting burst time = ms * 1000 (us/ms)
TICK_COUNT spdChngOffset = 0;

int16_t now2 = 0;
int16_t now3 = 0;
int16_t now4 = 0;
int16_t now1 = 0;

//Larger value means slower.
uint16_t loopLengthList[] = {6500,2900,2300,1800,1600,1450,/*Turn:*/1000};