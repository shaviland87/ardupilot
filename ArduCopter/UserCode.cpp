#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    #if OPTIMAERO_LIBRARY_ENABLED == ENABLED
        init_optimaero();
    #endif

    //hal.uartE->begin(115200,256,256);
    //hal.uartE->flush();
    //hal.uartE->printf("init uarte\n");

    #if 1==0
        hal.uartD->begin(115200,256,256);
        printf("starting uartD\n");
    #endif

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    #if OPTIMAERO_LIBRARY_ENABLED == ENABLED
        update_optimaero_Fast();
    #endif
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    #if OPTIMAERO_LIBRARY_ENABLED == ENABLED
        update_optimaero_10Hz();
    #endif


    #if 1==0

        int bytesAvail = 0;
        while(hal.uartD->available() && bytesAvail < 10){
            printf("uartD[%d] == %d \n",bytesAvail, hal.uartD->read());
            bytesAvail++;
        }
       
    #endif

}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    #if OPTIMAERO_LIBRARY_ENABLED == ENABLED
        update_optimaero_Slow();
    #endif
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif



#ifdef USERHOOK_400HZ
void Copter::userhook_400hz(){
        
    #if OPTIMAERO_LIBRARY_ENABLED == ENABLED
        update_optimaero_400Hz();
    #endif
}
#endif