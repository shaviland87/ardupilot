
/* This file is place holder of empty functions for OS specific funcs
 * that do not work everywhere all of which is actually in ether.cpp */

    #include <sys/types.h>
    #include <sys/stat.h>
    #include <sys/ioctl.h>
    #include <sys/socket.h>
    #include <stdlib.h>
    #include <fcntl.h>
    #include <termios.h>
    #include <stdio.h>
    #include <string.h>
    #include <unistd.h>
    #include <netdb.h>
    #include <errno.h>
    #include <strings.h>
    #include <netinet/in.h>

    #include <arpa/inet.h>
    #include "checksum.h"
    #include "ether2.h"

int closeCom(struct serialPort_ref *s) {
    if (s->fd > 0) {
        close(s->fd);
        s->fd = 0;
        return 0;
    }
    return -1;
}



int writeCom(struct serialPort_ref *s, void *buf, int nbytes) {

    int written;
    written = write( s->fd, (char*)buf, nbytes );
    /*tcflush(s->fd, TCOFLUSH);*/
    if (  written != nbytes ) {
        //printerrno(errno);
        s->dropwrite += nbytes - written;
        printf( " serial: write error on %s\n", s->name );
    }
    s->sent += written;
    return written;
}

/*static int totalread = 0;*/
int readCom(struct serialPort_ref *s, void *buf, int nbytes) {

    int newBytes;
    newBytes = 0;
    ioctl( s->fd, FIONREAD, (long)(&newBytes) );
    if ( newBytes > 0 ) {
        newBytes = MIN( newBytes, nbytes);
        newBytes = read( s->fd, (char*)buf, newBytes );
        //printf("\r                     new = %d, nbytes  = %d",newBytes,nbytes);
        s->received += MAX(0,newBytes);
        //writeCom(s,s->buffer,newBytes); // for testing only
    }
    return newBytes;
}



int openCom(struct serialPort_ref *s) {
    termios options;
    int dcbbaud;

    /*
    if (s->name[0] != '/') {
        sprintf(s->name,"/dev/ttyS%d",s->port-1);
    }*/


    if (s->blocking) {
        s->fd = open(s->name, O_RDWR | O_NOCTTY); //use to be hardcoded to /dev/ttyUSB0
    } else {
        s->fd = open(s->name, O_RDWR | O_NOCTTY | O_NONBLOCK); //use to be hardcoded to /dev/ttyUSB0
    }


    /* get options */

    tcgetattr(s->fd,&options);

    memset(&options,0,sizeof(options));

    //options.c_cflag = CRTSCTS | CLOCAL | CREAD;
    options.c_cflag = CLOCAL | CREAD;
    /*options.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;*/
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;  /* set input mode (non-canonical, no echo,...) */

    // set character size (default is 8 bits per character )
    switch (s->termios[0]) {
    case '7':
        options.c_cflag |= CS7;
        break;
    default:
    case '8':
        options.c_cflag |= CS8;
        break;
    };

    // set parity (default is no parity )
    switch (s->termios[1]) {
    case 'e':
    case 'E':
        options.c_cflag |= PARENB;	// default is even
        break;
    case 'o':
    case 'O':
        options.c_cflag |= PARENB | PARODD;
        break;
    default:
    case 'n':
    case 'N':
        break;
    };

    // set stop bits
    switch (s->termios[2]) {
    case '2':
        options.c_cflag |= CSTOPB;
        break;
    default:
    case '1':
        // this corresponds to 1 stop bit (default in posix)
        break;
    };

    // no flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);


    /* Convert the Baud Rate to the Com library define for the DCB */
    switch ( s->baud ) {
    case 110:
        dcbbaud = B110;
        break;
    case 300:
        dcbbaud = B300;
        break;
    case 600:
        dcbbaud = B600;
        break;
    case 1200:
        dcbbaud = B1200;
        break;
    case 2400:
        dcbbaud = B2400;
        break;
    case 4800:
        dcbbaud = B4800;
        break;
    case 9600:
        dcbbaud = B9600;
        break;
    case 19200:
        dcbbaud = B19200;
        break;
    case 38400:
        dcbbaud = B38400;
        break;
    case 57600:
        dcbbaud = B57600;
        break;
    case 115200:
        dcbbaud = B115200;
        break;

    case 230400:
        dcbbaud = B230400;
        break;

    case 460800:
        dcbbaud = B460800;
        break;

    case 500000:
        dcbbaud = B500000;
        break;
   
    default:
        return -1;
    }


    cfsetispeed(&options, dcbbaud);
    cfsetospeed(&options, dcbbaud);



    if (s->fd == 0 || s->fd == -1) {
        s->portIsOpen = 0;
        printf( " serial: Comport %s Open FAIL!\n", s->name );
    } else {
        s->portIsOpen = 1;
        printf("\n");
        printf( " serial: Comport %s Open SUCCESS baud %d %s (handle %d)\n",
                s->name, s->baud, s->termios, s->fd );
                printf("\n");
    }


    /* now clean the modem line and activate the settings for the port*/
    tcflush(s->fd, TCIFLUSH);
    tcsetattr(s->fd,TCSANOW,&options);

    return 0;
}

///

static int getaddr(struct sockaddr_in *ad, char *hostname, unsigned short port) {
    struct hostent *hp;
    memset(ad, 0, sizeof(struct sockaddr_in)); /* clear our address */
    hp = gethostbyname(hostname);
    if ( hp == NULL) {
        printf(" getaddr: lookup of %s failed\n",hostname);
        return(-1);
    }
    memcpy((char*)(&(ad->sin_addr)), hp->h_addr,hp->h_length);

    ad->sin_family = AF_INET;
    ad->sin_port = htons(port);
    return 0;

}

class PosixUDP {
public:
    struct sockaddr_in ma;
    struct sockaddr_in ra;
    struct sockaddr_in bma;
    struct sockaddr_in bra;

    int openSock(struct serialPort_ref *s);
    int closeSock(struct serialPort_ref *s);
    int readSock(struct serialPort_ref *s, void *buf, int nbytes);
    int writeSock(struct serialPort_ref *s, void *buf, int nbytes);
    void copyaddr() {
        memcpy(&ma,&bma,sizeof(bma));
        memcpy(&ra,&bra,sizeof(bra));
    }
    bool tryConnect(struct serialPort_ref *s);
    bool isAlreadyConnectedError();
    bool isBlockingError();
    bool isConnReset();

};

int PosixUDP::openSock(struct serialPort_ref *s) {
    unsigned long one = 1;
    int error;

    /*int tmp;*/

    // formulate addresses of my port and remote ports
    s->myport     = s->portNum;
    if(1 == s->setRemotePort){
        if(s->remotePortRequest == s->portNum){ //don't want them to be using same port
            s->remoteport = s->remotePortRequest+1;
        }else{
            s->remoteport = s->remotePortRequest;
        }
    }else{
        s->remoteport = s->portNum + 1000;
    }


    error = gethostname(s->myname,sizeof(s->myname));
    if(error ==0){
        printf("got hostname  \n");
        printf("hostname is %s \n ",s->myname);
    }else{
        printf("failed to get hostname\n");
    }
    strcpy(s->remotename,s->connectTo);
    printf("remote name is %s \n",s->remotename);
    if ( getaddr(&bma,s->myname,s->myport) != 0 ) return -1;
    if ( getaddr(&bra,s->remotename,s->remoteport) != 0 ) return -1;
    copyaddr();
    printf("copy addr\n");

    // create the socket
    s->fd = socket(AF_INET, SOCK_DGRAM, 0);


    if (s->fd == -1 ) {
        printf(" serial: openSock Cannot make socket\n");
        return -1;
    } else {
        ioctl(s->fd, FIONBIO, &one);
    }

        if (bind(s->fd, (struct sockaddr *)(&ma), sizeof(ma)) == -1) {
            printf(" serial: open Cannot bind socket to %s:%d\n",s->myname,s->myport);
            s->portIsOpen = 0;
            return(-1);
        } else {
             printf("\n");
            printf( " serial: Comport %d Open SUCCESS (UDP Sockets listening at %s:%d to %s:%d)\n",
                    s->port,s->myname,s->myport,s->remotename,s->remoteport);
                     printf("\n");
        }

    s->portIsOpen = 1;
    return 0;
}




int PosixUDP::closeSock(struct serialPort_ref *s) {
    if (s->fd > 0) {
        close(s->fd);
        s->fd = 0;
    }
    s->portIsOpen = 0;
    s->connectState = 0;
    return 0;
}

/*int PosixUDP::readSock(struct serialPort_ref *s, void *buf, int nbytes) {
    int toread, status;
    socklen_t sizeofsockaddr;
    toread = 0;
    sizeofsockaddr = sizeof(struct sockaddr);

    copyaddr();
    if(!this->tryConnect(s)) return 0;


    // if socket is connected
    if (s->portIsOpen) {
        status = ioctl(s->fd,FIONREAD,(long)&toread);
        if (status == -1) {
            printf(" serial: problem receiving at %s:%d from %s:%d\n",
                   s->myname,s->myport,s->remotename,s->remoteport);
            return 0;
        }

         status = recvfrom(s->fd, (char*)buf, MIN(toread,nbytes),  0, (struct sockaddr*)(&ra), &sizeofsockaddr );


        //printf("\n status = %d", status);
        if (status == -1) {
            return 0;
        } else {
            s->received += status;
            return status;
        }

    }
    return 0;
}*/
int PosixUDP::readSock(struct serialPort_ref *s, void *buf, int nbytes) {
    int toread, status, totalNew = 0;
    socklen_t sizeofsockaddr;
    toread = 0;
    sizeofsockaddr = sizeof(struct sockaddr);

    copyaddr();
    if(!this->tryConnect(s)) return 0;


    // if socket is connected
    if (s->portIsOpen ) {
        do {
            status = ioctl(s->fd,FIONREAD,(long)&toread);
            if (status == -1) {
                printf(" serial: problem receiving at %s:%d from %s:%d\n",
                       s->myname,s->myport,s->remotename,s->remoteport);
                return totalNew;
            }

            //ret = recvfrom(sd, (char*)buf, MIN(toread,(unsigned long)nbytes),  0, (struct sockaddr*)ra, &sizeofsockaddr );
            status = recvfrom(s->fd, (char*)buf, MIN(toread,nbytes),  0, (struct sockaddr*)(&ra), &sizeofsockaddr );

            //printf("\n status = %d", status);
            if (status == -1) {
                return totalNew;
            } else {
                buf += status;
                totalNew += status;
                s->received += status;
            }
        } while( status > 0 );

    }
    return totalNew;
}

int PosixUDP::writeSock(struct serialPort_ref *s, void *buf, int nbytes) {
    int ret;
    copyaddr();
    if(!this->tryConnect(s)) return 0;
    // if socket is connected
    if (s->portIsOpen) {

        ret = sendto(s->fd, (char*)buf, nbytes, 0, (struct sockaddr*)(&ra), sizeof(struct sockaddr) );

        if (ret == -1) {
            printf("err in write \n");
            return 0;
        } else {
            s->sent += ret;
            return ret;
        }
    }
    return 0;
}




bool PosixUDP::isAlreadyConnectedError() {
    if(errno == EISCONN) {
        return true;
    } else {
        return false;
    }
}
bool PosixUDP::isBlockingError() {
    switch (errno) {
    case EWOULDBLOCK: // always == NET_EAGAIN?
    case EALREADY:
    case EINPROGRESS:
        return true;
    }
    return false;
}

bool PosixUDP::isConnReset() {
    switch (errno) {
    case ECONNRESET: // always == NET_EAGAIN?
        return true;
    }
    return false;
}


bool PosixUDP::tryConnect(struct serialPort_ref *s) {

        if( s->portIsOpen) {
            return true;
        } else {
            return false;
        }

}
int openSock(struct serialPort_ref *s) {
    PosixUDP *sdev;
    sdev = new PosixUDP();
    s->sockDevice = (void*)sdev;
    return sdev->openSock(s);

}

int closeSock(struct serialPort_ref *s) {
    int ret;
    PosixUDP *sdev;
    sdev = (PosixUDP*)(s->sockDevice);
    ret =  sdev->closeSock(s);
    delete sdev;
    s->sockDevice = NULL;

    if(ret > 10000){printf("wtf\n");}

    return 0;
}

int readSock(struct serialPort_ref *s, void *buf, int nbytes) {
    PosixUDP *sdev;
    sdev = (PosixUDP*)(s->sockDevice);
    return sdev->readSock(s,buf,nbytes);
}

int writeSock(struct serialPort_ref *s, void *buf, int nbytes) {
    PosixUDP *sdev;
    sdev = (PosixUDP*)(s->sockDevice);
    return sdev->writeSock(s,buf,nbytes);
}


















