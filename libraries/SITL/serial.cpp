
/***
 * $Id: serial.cpp
 * contains the shared memory interface and generic routines
 * to read and write data packets, irrespective of whether
 * it goes through shared memory, serial or udp
 ***/
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include "serial.h"



static void initPort( struct serialPort_ref *s ) {

    /* safety check to make sure port is not negative */
    if( s->port <= 0 ) {
        s->port = 1;
    }

    s->dataSource = 1;
    printf("init port bro \n");
    printf("init port bro \n");
   // sprintf( s->name, s->nameFmt, s->port - 1 );

    if(s->useSerial == 1)
    {
        s->newConnectionMode = MODE_SERIAL;
    }else{
        s->newConnectionMode = MODE_UDP;
    }


    closePort(s);

    // Don't change the connection mode; just re-initialize
    s->connectionMode = s->newConnectionMode;
    s->newConnectionMode = 0;


    if(s->dataSource && s->useSerial){
        printf("opening com\n");
        openCom(s);
        //printf("attempted to open com port\n");
    }else if(s->dataSource && s->useSock){
        printf("opening socket \n");
        openSock(s);
        //printf("opened socket  \n");
    }



    s->bytesread = 0;
    s->init = 0;

}



/** This function can be nilpotently. It always closes a valid file descriptor
 *  If the file descriptor is invalid we do not care anyway -
 */
void closePort( struct serialPort_ref *s )  {

        if( (s->connectionMode == MODE_UDP || s->connectionMode == MODE_TCP) && (s->portIsOpen == 1)  ){
            closeSock(s);
            s->portIsOpen = 0;
            s->bytesread = 0;
        }else if(  (s->connectionMode == MODE_SERIAL) && (s->portIsOpen == 1 ) ){
            printf("closing com\n");
            closeCom(s);
            s->portIsOpen = 0;
            s->bytesread = 0;
        }



}


void readPort( struct serialPort_ref *s ) {

    int newBytes = 0;

    if( 1 == s->init )
    {
        initPort( s );
    }

    if(s->connectionMode == MODE_SERIAL){

        if( s->bytesread >= BUFFERSIZE || s->bytesread < 0 ) s->bytesread = 0; /* something very wrong... */
                if( (s->connectionMode == MODE_SERIAL) && s->portIsOpen) {
                    newBytes = readCom( s, s->buffer + s->bytesread, BUFFERSIZE - s->bytesread );
                    newBytes = LIMIT( newBytes, 0, BUFFERSIZE - s->bytesread ); /* extra protection in case return arguments are wierd in error cases */
                    if( newBytes > 0 ) {
                        s->bytesread += newBytes;
                        memcpy( s->bufferView, s->buffer, 8 );
                    }
                }
    }


    if(s->connectionMode == MODE_UDP){
        if( s->bytesread >= BUFFERSIZE || s->bytesread < 0 ) s->bytesread = 0; /* something very wrong... */

        newBytes = readSock(s, s->buffer + s->bytesread, BUFFERSIZE - s->bytesread);
        newBytes = LIMIT( newBytes, 0, BUFFERSIZE - s->bytesread ); /* extra protection in case return arguments are weird in error cases */

        if( newBytes > 0 ) {
            s->bytesread += newBytes;
            if( newBytes ) memcpy( s->bufferView, s->buffer, 8 );
        }
    }

}

void writePort( struct serialPort_ref *s, char *buffer, int size ) {

    int written =0 ;

    /* should probably add functionality to check if send buffer is empty... */

    if( 1 == s->init )
    {
        initPort( s );
    }

     
    if( (s->connectionMode == MODE_SERIAL) && s->portIsOpen ) {
        written = writeCom( s, buffer, size );
    }else if (s->connectionMode == MODE_UDP && s->portIsOpen){
        written = writeSock( s, buffer, size );
    }


	if(written > 100000){printf("wtf\n");}




}

void clearPort( struct serialPort_ref *s, int shift ) {

    if( shift != 0 ) {
        if( s->dataSource == PORT_ON ) {
            if( shift > 0 ) {
                if( shift >= s->bytesread ) {
                    s->bytesread = 0;
                } else {
                    memmove( s->buffer, &(s->buffer[shift]), s->bytesread - shift );
                    /*memcpy( s->buffer, &(s->buffer[shift]), s->bytesread - shift );*/
                    s->bytesread -= shift;
                }
                memcpy( s->bufferView, s->buffer, 8 );
            } else {
                s->bytesread = 0;
            }
        }
    }

}


unsigned char portCsumCompute( unsigned char *buf, int byteCount ) {

  int m;
  unsigned char csum;

  csum = 0;
  for( m=0; m<byteCount; m++ )
    csum ^= buf[m];

  return( csum );

}


bool update_link(struct serialPort_ref *s, struct quadArduMessageHeader_ref *header ,struct optimAeroMessageFDM_ref *optimAeroFDM){


    readPort( s );

    unsigned char *bf;
    int  done, index;
    unsigned short rxCsum;
    bool gotFDM = false;
    bool gotIMU = false;
    done = 0;
    index = 0;

    while ( ( index <= s->bytesread - SAS_MESSAGE_HDR_SIZE ) && !done )
    {
        if ( ( s->buffer[index]  == 0xa1 ) )
        {
           // printf("got sync\n");
            bf = &(s->buffer[index]);

            memcpy( header, bf, sizeof( struct quadArduMessageHeader_ref ) );

            if ( header->messageSize >= sizeof( struct quadArduMessageHeader_ref ) && header->messageSize < BUFFERSIZE )
            {

                if (  header->messageSize + index <= s->bytesread ) {

                    memcpy( &rxCsum, &bf[header->messageSize - sizeof( unsigned short )], sizeof( unsigned short) );

                    if (fletcher16(bf, (short)( header->messageSize - sizeof( unsigned short )) ) == rxCsum)
                    {

                        switch ( header->messageID )
                        {
                            case OPTIM_AERO_MESSAGE_FDM:
                                memcpy( optimAeroFDM, bf, header->messageSize );
                                //printf("got imu\n");
                                gotIMU = true;
                                break;

                            default:
                                printf("unknown message of size %d and id %d\n",header->messageSize, header->messageID);
                                break;
                        }

                    } else
                    {
                        /* checksum bad */
                        printf("CHECK SUM BAD!! (ID = %d, size %d)\n", header->messageID,header->messageSize);

                    }
                    index += header->messageSize - 1;

                }else
                {
                    index--;
                    done = 1;
                }
            }
            else {
                index += sizeof( struct quadArduMessageHeader_ref );
            }
        }

        index++; /* start seq not found, go to next byte */

        if( index < 0 ) index = BUFFERSIZE - 1;
    }

    clearPort( s, index );

    gotFDM = gotIMU;

    return gotFDM;


 }

void update_motors(struct serialPort_ref *s,struct optimPix2Brain_ref *pix){
    // function to read motor commands
    readPort( s );

    unsigned char *bf;
    int done, index;
    unsigned char rxCsum;
    unsigned char csum;
    //unsigned char readID;
    FastCRC8 crc8;
    unsigned char readSize = sizeof(struct optimPix2Brain_ref);
    done = 0;
    index = 0;

    while ( ( index <= s->bytesread - SAS_MESSAGE_HDR_SIZE ) && !done )
    {
        //printf("index %d",index);
        if ( ( s->buffer[index]  == OPTIM_TOYO_SYNC_RECV ) )
        {
            bf = &(s->buffer[index]);
            if ( index <= s->bytesread - readSize) {
                rxCsum  =   bf[readSize-1];
                csum    =   crc8.maxim(bf,readSize-1);
                //printf(" CSUM %d == %d \n",rxCsum, csum);
                if(rxCsum == csum )
                {
                    //got message
                    memcpy(pix, bf, sizeof(struct optimPix2Brain_ref));
                    printf("\n[");
                    for(int i=0; i<4; i++){
                        printf("%d,",pix->motor_cmds[i]);
                    }
                    printf("]\n");
                    printf("\n and motors are %d armed", pix->control_flag);
                }
                //index +=  2; // 3 is header size -1
                index += readSize;
            }else
            {
                done = 1;
            }
        }else{
            index++; ///* start seq not found, go to next byte */
        }
        if( index < 0 ) index = BUFFERSIZE - 1;
    }
    clearPort( s, index );


}



void update_gerp(struct serialPort_ref *s,struct optimMB2PixMsg_Ref *mb1, struct optimMB2PixMsg2_Ref *mb2, struct optimToyoRPM_ref *rpm, struct optimBrainRC_ref *brainRC){

    readPort( s );

    unsigned char *bf;
    int done, index;
    unsigned char rxCsum;
    unsigned char csum;
    FastCRC8 crc8;
    unsigned char readID;
    unsigned char readSize;
    done = 0;
    index = 0;

    while ( ( index <= s->bytesread - SAS_MESSAGE_HDR_SIZE ) && !done )
    {
        //printf("index %d",index);
        if ( ( s->buffer[index]  == OPTIM_TOYO_SYNC_RECV ) )
        {
            //printf("got sync\n");
            bf = &(s->buffer[index]);
            readID      = bf[1];
            readSize    = bf[2];
            //count is 3
            //printf("read size is %d\n",readSize);
             
            if ( readSize < OPTIM_TOYO_MAX_MSG_SIZE && readSize > OPTIM_TOYO_HDR_SIZE )
            {

                if ( index <= s->bytesread - readSize) {
                    rxCsum  =   bf[readSize-1];
                    csum    =   crc8.maxim(bf,readSize-1);
                    //printf("readsize is %d, and bytes read are %d \n",readSize, s->bytesread);
                    //printf("checksum compare %d , %d \n",rxCsum, csum);
                    //printf("readId is %d \n ",readID);

                    if(rxCsum == csum ){

                        switch(readID){
                            case OPTIM_TOYO_MB2PIX_MSG:
                                if(readSize== sizeof(struct optimMB2PixMsg_Ref)){
                                    memcpy(mb1, bf, sizeof(struct optimMB2PixMsg_Ref));
                                    printf("got brain message \n");
                                }break;
                            case OPTIM_TOYO_MB2PIX_MSG2:
                                if(readSize == sizeof(struct optimMB2PixMsg2_Ref)){
                                    memcpy(mb2, bf, sizeof(struct optimMB2PixMsg2_Ref));
                                    printf("got brain 2 message \n");
                                }break;

                            case OPTIM_TOYO_RPM:
                                if(readSize == sizeof(struct optimToyoRPM_ref)){
                                    memcpy(rpm, bf, sizeof(struct optimToyoRPM_ref));
                                    //printf("got rpm\n");
                                }break;
                            case OPTIM_TOYO_BRAINRC_MSG:
                                if(readSize == sizeof(struct optimBrainRC_ref)){
                                    memcpy(brainRC, bf, sizeof(struct optimBrainRC_ref));
                                }break;

                                default:
                                printf("unknown id\n");
                                break;
                        }

                    }else{printf("bad cksum(%d != %d )\n", rxCsum, csum);}
                    //index +=  2; // 3 is header size -1
                    index += readSize;
                }else
                {
                    printf("Gerp done \n");
                    done = 1;
                }
            }
            else {
                //index++;
                printf("REad size is %d whyy\n",readSize);
            }
        }else{
            index++; ///* start seq not found, go to next byte */
        }
        //printf("index is %d",index);

        if( index < 0 ) index = BUFFERSIZE - 1;
    }
    if(index>0)
        //printf("\n index is %d\n",index);
    clearPort( s, index );


}




