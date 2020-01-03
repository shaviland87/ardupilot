

/** Must declare with C linkage because it is serial.c not .cpp */

#ifdef __cplusplus
extern "C" {
#endif


    int openCom(struct serialPort_ref *s);
    int closeCom(struct serialPort_ref *s);
    int readCom(struct serialPort_ref *s, void *buf, int nbytes);
    int writeCom(struct serialPort_ref *s, void *buf, int nbytes);

    int openSock(struct serialPort_ref *s);
    int closeSock(struct serialPort_ref *s);
    int readSock(struct serialPort_ref *s, void *buf, int nbytes);
    int writeSock(struct serialPort_ref *s, void *buf, int nbytes);

#ifdef __cplusplus
}
#endif





#define MIN(x1,x2) ((x1)<(x2)?(x1):(x2))
#define MAX(x1,x2) ((x1)>(x2)?(x1):(x2))
#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))
#define ABS(x) ((x)<0.0?(-(x)):(x))
#define SQ(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))
#define SIGN(x) ((x)<(0)?(-1):((x)>(0)?(1):(0)))
#define INRANGE(x,x1,x2) ((x)<=(x2)&&(x)>=(x1)?1:0)
