#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 2),
    AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),
    AP_GROUPINFO("_CHRPENABLE",3, UserParameters, _chirpEnabled, 0),
    AP_GROUPINFO("_CHRPAXES",4, UserParameters, _chirpAxes, 0 ),
    AP_GROUPINFO("_CHRPMAXHZ", 5, UserParameters, _chirpMaxFreq, 0),
    AP_GROUPINFO("_CHRPMINHZ", 6, UserParameters, _chirpMinFreq, 0),
    AP_GROUPINFO("_CHRPAMP", 7, UserParameters, _chirpAmplitude, 0),
    AP_GROUPINFO("_CHRPOCTAVE", 8, UserParameters, _chirpOctaves, 0),
    AP_GROUPINFO("_RLL_KP",9,UserParameters,_rollKp,3.1),
    AP_GROUPINFO("_RLL_KI",10,UserParameters,_rollKi,0.65),
    AP_GROUPINFO("_RLL_KD",11,UserParameters,_rollKd,2.0),
    AP_GROUPINFO("_PIT_KP",12,UserParameters,_pitchKp,3.1),
    AP_GROUPINFO("_PIT_KI",13,UserParameters,_pitchKi,0.65),
    AP_GROUPINFO("_PIT_KD",14,UserParameters,_pitchKd,2.0),    
    AP_GROUPINFO("_YAW_KP",15,UserParameters,_yawKp,0.344),
    AP_GROUPINFO("_YAW_KI",16,UserParameters,_yawKi,0.025),
    AP_GROUPINFO("_YAW_KD",17,UserParameters,_yawKd,0.83),
    AP_GROUPINFO("_RP_INT",18,UserParameters,_rpIntLimit,0.1),
    AP_GROUPINFO("_YAW_INT",19,UserParameters,_yawIntLimit,0.2617),
    AP_GROUPINFO("_YAW_ERR",20,UserParameters,_yawErrorLimit,0.2617),
    AP_GROUPINFO("_THR_G",21,UserParameters,_throttleGain,12.7486),
    AP_GROUPINFO("_MIN_THR",22,UserParameters,_minThrottle,0.01),
    AP_GROUPINFO("_MIN_THR_R",23,UserParameters,_minThrottleRampTime,2.0),
    AP_GROUPINFO("_ATT_TAU",24,UserParameters,_attitudeTau,1.0),
    AP_GROUPINFO("_THR_RATE",25,UserParameters,_throttleRateLimit,19.6133),
    AP_GROUPINFO("_PILOT_RP",26,UserParameters,_pilotRPgain,25.0),
    AP_GROUPINFO("_PILOT_YAW",27,UserParameters,_pilotYawgain,50.0),
    
    AP_GROUPEND
};

 
 