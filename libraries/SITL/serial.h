
/** Must declare with C linkage because it is serial.c not .cpp */
#include "checksum.h"
#include "ether2.h"

#include "FastCRC.h"

#define BUFFERSIZE 1024*100
#define PORT_OFF  0
#define PORT_ON   1
#define PORT_SITL 2
#define PORT_PLAYBACK 3
#define PORT_WORKER 4

#define MAX_WORKER_PORT 10

#define FCS20_NIOS 333

#define MODE_SERIAL 1
#define MODE_UDP    2
#define MODE_TCP    3
#define MODE_SPI    4


#define NDDS_SERIAL_DOMAIN 0
#define BSD_BASE_PORT 2000

#define SERIAL_SYNC1 0xaa
#define SERIAL_SYNC2 0x44
#define SERIAL_SEEK_SYNC1   0
#define SERIAL_SEEK_SYNC2   1
#define SERIAL_SEEK_ID_0    2
#define SERIAL_SEEK_ID_1    3
#define SERIAL_SEEK_SIZE_0  4
#define SERIAL_SEEK_SIZE_1  5
#define SERIAL_SEEK_CSUM    6
#define SERIAL_SEEK_HCSUM   7
#define SERIAL_SEEK_DATA    8


#define SERIALMSG_HEADER_SIZE 8




#ifdef __cplusplus
extern "C" {
#endif


void readPort(  struct serialPort_ref *s );
void writePort( struct serialPort_ref *s, char *buffer, int size );
void clearPort( struct serialPort_ref *s, int shift );
void closePort( struct serialPort_ref *s );
bool update_link(struct serialPort_ref *s, struct quadArduMessageHeader_ref *header ,struct optimAeroMessageFDM_ref *optimAeroFDM);

//tmp code to use for checking new messaging checksum bs
void update_gerp(struct serialPort_ref *s,struct optimMB2PixMsg_Ref *mb1, struct optimMB2PixMsg2_Ref *mb2, struct optimToyoRPM_ref *rpm, struct optimBrainRC_ref *rc);
void update_motors(struct serialPort_ref *s,struct optimPix2Brain_ref *pix);

#ifdef __cplusplus
}
#endif


