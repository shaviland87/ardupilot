// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES    

#if OPTIMAERO_LIBRARY_ENABLED == ENABLED
    #if OPTIMAERO_CHIRP_ENABLED == ENABLED

    /*chirp stuff */

    const float TAU = 6.28318530718f; // 2*pi
    const float LN2 = 0.69314718056f;

    //constants
    /*
    Amplitude:  3% throttle =  30 pwms
    Start frequency: 0.2 Hz
    Stop frequency: 8 Hz  (Nyquist frequency assuming 16 Hz sampling)
    Cycles per octave: 8
    */
    float m_startFrequency = 0.2f;  // rad/iteration
    float m_stopFrequency  = 8.0f;  // rad/iteration  
    float m_ratio;                  // rad/iteration^2
    float cycles_per_octave = 8.0f;
    float m_phaseInRate;            // cycles/iteration
    float m_dt;
    float m_amplitude; 
    float m_input;                  // input value - chirp useage but not committed out as its used in log
    float m_output;                 // output signal for logging purposes
    uint8_t m_counter_ = 0;

    // Time-varying values.
    float m_phaseInGain;            
    float m_frequency;                  // rad/iteration
    float m_phase;                      // radians
    float m_value;                      // output value
    float disturbance_injection;        // disturbance angle injected
    float gerp_gork                     = 0.0f;
    bool  m_completedFlag;              // chirp complete flag
    uint32_t chirp_Duration             = 0;
    float    chirp_Freqency             = 0.0f;
    float chirp_throttle_settings[4]    = {950.0f, 900.0f, 800.0f,500.0f};
    float chirp_step_input_throttle     = 200.0f; //step up/down throttle this much per second
    float chirp_throttle_steady_state   = 0.0f;
    float throttle_to_send              = 0.0f;
    //param to start the chirp
    bool chirp_started                  = false;

    bool chirp_ready                    = false; //once the chrip tests starts
    bool chirp_setting_done             = false;
    bool chirp_init                     = false; //init flag
    bool chirp_no_spin                  = false;

    unsigned char chirp_throttles       = 0; //value used to go thru the throttle settings
    unsigned int chirp_incrementer      = 0;
    unsigned char chirp_hold_three      = 0; // hold for three seconds after chirp
    //////////////////////////////////////////////////////////////////////////////////

    #endif
#endif


#endif  // USERHOOK_VARIABLES


