

#ifndef __CRC_H_TEMPLATE
#define __CRC_H_TEMPLATE

#include "stdint.h"

template<class T>class Crc
{
   public:

      /** Reset with no parameters will set the crc to 0*/
      virtual void Reset(void) = 0;

      /** Reset with a value will set the crc to init_value.
          @param init_value value to seed the crc with */
      virtual void Reset(T init_value) = 0;

      /** Compute adds another byte to the crc.
          @param data next byte to add to the CRC
          @return current CRC value */
      virtual T Compute(uint8_t data) = 0;

      virtual T Compute(uint8_t buffer[], uint32_t buffer_size, bool reset = true) = 0;

      /** Lock and Unlock only need to be implemented by hardware crc modules.
          They are used directly by the CrcHwWrapper to transparently allow multiple
          threads to share a hardware crc by having different instances of CrcHwWrapper.
          when using the CrcHwWrapper, lock and unlock are not needed to be called by the app*/
      virtual void Lock()
      {
         return;
      }
      virtual void Unlock()
      {
         return;
      }

      virtual ~Crc() {}

};

#endif
