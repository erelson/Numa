//#ifndef Commander_h
//#define Commander_h
//
//#define LISTEN UART2toXbee38400

/* bitmasks for buttons array */
#define BUT_R1      0x01
#define BUT_R2      0x02
#define BUT_R3      0x04
#define BUT_L4      0x08
#define BUT_L5      0x10
#define BUT_L6      0x20
#define BUT_RT      0x40
#define BUT_LT      0x80

int CmdrReadMsgs();
void UseSouthPaw();

// variables used for reading messages
unsigned char vals[7];  // temporary values, moved after we confirm checksum_cmdr
int index_cmdr;              // -1 = waiting for new packet
int checksum_cmdr;
unsigned char southpaw; 

extern int walkV;
extern int walkH;
extern int lookV;
extern int lookH;
extern int walkSPD;
extern float walkDIR;

//#endif