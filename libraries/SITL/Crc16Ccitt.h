/** @file
 * @author Justin McAllister
 * @brief CRC software computation
 *    CRC Class using the standard CRC-CCITT polynomial of 0x1021
 *    See http://www.netrino.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
 *
 *  $LastChangedBy: Cremean $
 *  $LastChangedDate: 2011-10-31 14:28:19 -0700 (Mon, 31 Oct 2011) $
 *  $LastChangedRevision: 33376 $
*/

#include "Crc.h"
#include "stdint.h"


#ifndef __CRC_16_CCITT_H
#define __CRC_16_CCITT_H

class Crc16Ccitt : public Crc<uint16_t>
{
   public:

      Crc16Ccitt()
      {
         myCrc = presetValue = 0;
      }

      Crc16Ccitt(uint16_t data) 
      {
         myCrc = presetValue = data;
      }

      virtual void Reset(void) override
      {
         myCrc = presetValue;
      }

      virtual void Reset(uint16_t data) override
      {
         myCrc = data;
      }

      virtual uint16_t Compute(uint8_t data) override;

      virtual uint16_t Compute(uint8_t buffer[], uint32_t buffer_size, bool reset = true) override;

      uint16_t GetCrc()
      {
         return myCrc;
      }

   protected:
      /** Add a byte to an existing CRC */
      inline virtual void CCITT(uint8_t data)
      {
         uint16_t i = CCITT_TABLE[((this->myCrc >> 8) & 0xFF) ^ data];
         this->myCrc = (i ^ ( ( this->myCrc << 8 ) & 0xFF00 ) );
      }

      uint16_t myCrc;
      uint16_t presetValue;
      static const uint16_t CCITT_TABLE[256];
};

#endif