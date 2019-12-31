



//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#include "checksum.h"

//------------------------------------------------------------------------
// Local macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------


#define EPSILON_VALUE 0.0000000000000001

bool cmpf(float A, float B)
{
  return (fabs(A - B) < EPSILON_VALUE);
}


double rande( void ) {

  int j;
  long k;
  static long iy=0;
  static long iv[NTAB];
  double temp;
  static long seed = 0;

  if( seed == 0 )
    seed = -(long)time( NULL );

  if( seed <= 0 || !iy ) {
    if( -(seed) < 1 ) seed = 1;
    else seed = -(seed);
    for( j=NTAB+7; j>=0; j-- ) {
      k = (seed)/IQ;
      seed = IA*( seed - k*IQ ) - IR*k;
      if( seed < 0 ) seed += IM;
      if( j < NTAB ) iv[j] = seed;
    }
    iy = iv[0];
  }
  k = (seed)/IQ;
  seed = IA*( seed - k*IQ ) - IR*k;
  if( seed < 0 ) seed += IM;
  j = iy/NDIV;
  iy = iv[j];
  iv[j] = seed;
  if( ( temp = AM*iy ) > RNMX ) return RNMX;
  else return temp;

}

/* Normal distribution,
  from Numerical Recipes in C pp. 289-290 */

double randne( void ) {

  static double gset;
  static int iset = 0;
  double fac, rsq, v1, v2;

  if( iset == 0 ) {
    do {
      v1 = 2.0*rande() - 1.0;
      v2 = 2.0*rande() - 1.0;
      rsq = v1*v1 + v2*v2;
    } while( rsq >= 1.0 || cmpf(rsq,0.0f) );
    fac = sqrt( -2.0*log( rsq )/rsq );
    gset = v1*fac;
    iset = 1;
    return v2*fac;
  } else {
    iset = 0;
    return gset;
  }

}

unsigned int fletcher16( unsigned char *data, int len ) {

    unsigned int check;
    unsigned int sum1 = 0xff, sum2 = 0xff;

    while (len) {
        int tlen = len > 21 ? 21 : len;
        len -= tlen;

        do {
            sum1 += *data++;
            sum2 += sum1;
        } while (--tlen);

        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
    }

    // Second reduction step to reduce sums to 8 bits
    sum1 = (sum1 & 0xff) + (sum1 >> 8);
    sum2 = (sum2 & 0xff) + (sum2 >> 8);

    check = (unsigned int)(sum1 << 8) | (unsigned int)(sum2);

    return check;
}



/**
 * \brief Calculate and return the checksum of a buffer array
 *
 * Calculates the checksum of a character buffer using a (modified) 32-bit
 * Fletcher checksum. (The modification allows the handling of an odd number of
 * bytes by calculating the checksum as ifthere were an additional zero-byte
 * appended to the end of the data.)
 *
 * The code of this function is copied from the corresponding sources in GUST.
 *
 * \param buf pointer to a character buffer array
 * \param byteCount the size in bytes of the character buffer
 */
unsigned int datalinkCheckSumCompute( unsigned char *buf, int byteCount ) {

    unsigned int sum1 = 0xffff;
    unsigned int sum2 = 0xffff;
    unsigned int tlen = 0;
    unsigned int shortCount = byteCount / sizeof(short);
    unsigned int oddLength  = byteCount % 2;

    /* this is Fletcher32 checksum modified to handle buffers with an odd number of bytes */

    while( shortCount ) {
        /* 360 is the largest number of sums that can be performed without overflow */
        tlen = shortCount > 360 ? 360 : shortCount;
        shortCount -= tlen;
        do {
            sum1 +=  *buf++;
            sum1 += (*buf++ << 8);
            sum2 += sum1;
        } while (--tlen);

        /* add last byte if there's an odd number of bytes (equivalent to appending a zero-byte) */
        if( (oddLength==1) && (shortCount<1) ) {
            sum1 += *buf++;
            sum2 += sum1;
        }

        sum1 = (sum1 & 0xffff) + (sum1 >> 16);
        sum2 = (sum2 & 0xffff) + (sum2 >> 16);
    }

    /* Second reduction step to reduce sums to 16 bits */
    sum1 = (sum1 & 0xffff) + (sum1 >> 16);
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);

    return( sum2 << 16 | sum1 );
}


/**
 * \brief datalinkCheckSumEncode sets the header checksum and payload checksum of a
 * character buffer to be sent as a datalink message
 *
 * \param buf pointer to a character buffer
 * \param byteCount size of the character buffer in bytes
 */
void datalinkCheckSumEncode( unsigned char *buf, int byteCount ) {

    struct datalinkHeader_ref *h = (struct datalinkHeader_ref *)buf;

    h->hcsum = datalinkCheckSumCompute(  buf, sizeof( struct datalinkHeader_ref ) - sizeof( int )*2 );
    h->csum  = datalinkCheckSumCompute( &(buf[sizeof( struct datalinkHeader_ref )]),
                                  byteCount - sizeof( struct datalinkHeader_ref ) );

}
