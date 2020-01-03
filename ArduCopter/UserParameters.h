#pragma once

#include <AP_Param/AP_Param.h>


#if OPTIMAERO_LIBRARY_ENABLED == ENABLED
    #include <AP_OA_EXT/optimAero.h>
#endif


class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];
    
    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Int8 get_int8Param() const { return _int8; }
    AP_Int16 get_int16Param() const { return _int16; }
    AP_Float get_floatParam() const { return _float; }
    AP_Int8 get_chirpEnabled() const {return _chirpEnabled;}
    AP_Int8 get_chirpAxes() const {return _chirpAxes;}
    AP_Float get_chirpMaxFreq() const {return _chirpMaxFreq;}
    AP_Float get_chirpMinFreq() const {return _chirpMinFreq;}
    AP_Float get_chirpAmplitude() const {return _chirpAmplitude;}
    AP_Int8 get_chirpOctaves() const {return _chirpOctaves;}
private:
    // Put your parameter variable definitions here
    AP_Int8 _int8;
    AP_Int16 _int16;
    AP_Float _float;
    AP_Int8 _chirpEnabled;
    AP_Int8 _chirpAxes;
    AP_Float _chirpMaxFreq;
    AP_Float _chirpMinFreq;
    AP_Float _chirpAmplitude;
    AP_Int8 _chirpOctaves;
};
