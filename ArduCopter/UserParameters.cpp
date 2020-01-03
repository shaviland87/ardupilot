#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),
    AP_GROUPINFO("_CHRPENABLE",3, UserParameters, _chirpEnabled, 0),
    AP_GROUPINFO("_CHRPAXES",4, UserParameters, _chirpAxes, 0 ),
    AP_GROUPINFO("_CHRPMAXHZ", 5, UserParameters, _chirpMaxFreq, 0),
    AP_GROUPINFO("_CHRPMINHZ", 6, UserParameters, _chirpMinFreq, 0),
    AP_GROUPINFO("_CHRPAMP", 7, UserParameters, _chirpAmplitude, 0),
    AP_GROUPINFO("_CHRPOCTAVE", 8, UserParameters, _chirpOctaves, 0),
    
    AP_GROUPEND
};
