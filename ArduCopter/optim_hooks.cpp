
#include "Copter.h"
#if OPTIMAERO_LIBRARY_ENABLED == ENABLED

    void Copter::init_optimaero(void){

        g2.oa_module.set_log_optimAero(MASK_LOG_ANY);
        g2.oa_module.init();
    }

    void Copter::update_optimaero_Fast(void){

        g2.oa_module.updateFast();
        
    }
        
    void Copter::update_optimaero_10Hz(void){

        g2.oa_module.update10Hz();
        
    }
    
    void Copter::update_optimaero_Slow(void){

        g2.oa_module.updateSlow();
                
    }

    void Copter::update_optimaero_400Hz(void){
        g2.oa_module.update400hz();
    }

    void Copter::update_400hz_chirp(void)
    {
      
      #if OPTIMAERO_CHIRP_ENABLED == ENABLED

        //also only want to do this in any mode but stabilize
        if(copter.control_mode != Mode::Number::STABILIZE)
        {
            // we aren't in stabilize mode
        if( ( g2.user_parameters.get_chirpEnabled() ) &&
            ( g2.user_parameters.get_chirpAxes() <= 10) &&
              motors->armed()   )
              { //enabled, chirp_axes is good and motors are armed

                if(g2.rc_channels.rc_channel(4)->get_radio_in() > 1400 && g2.rc_channels.rc_channel(4)->get_radio_in() < 1600 )
                {

                  //we are in middle mode
                  if(copter.chirp_ready && !copter.chirp_init)
                  {
                      //to chk if we need to init
                      copter.ChirpInit(g2.user_parameters.get_chirpMinFreq(),
                                      g2.user_parameters.get_chirpMaxFreq(),
                                      (float)g2.user_parameters.get_chirpOctaves(),
                                      0.0025, g2.user_parameters.get_chirpAmplitude());

                      copter.chirp_init = true;
                      copter.chirp_setting_done = false;
                      copter.chirp_no_spin = false;
                      gcs().send_text(MAV_SEVERITY_INFO, "CHIRP INIT");

                  }else if(copter.chirp_ready && copter.chirp_init)
                  {
                    copter.chirp_setting_done = copter.ChirpIsCompleted();//check to see if done

                    if(copter.chirp_setting_done == true )
                    {
                      //we are done
                      if(copter.chirp_no_spin == false){
                          copter.chirp_no_spin = true;
                          gcs().send_text(MAV_SEVERITY_INFO, "CHIRP COMPLETE");
                      }
                    }else
                    {
                      //chirp not done
                      copter.ChirpUpdate();
                      copter.chirp_Duration = copter.ChirpDuration();
                      copter.chirp_Freqency = copter.ChirpFrequencyInHz();

                      switch( g2.user_parameters.get_chirpAxes() )
                      {
                          case 0:
                          break;

                          case 1:
                          break;
                          
                          case 2:
                          break;
                          
                          case 3:
                          break;
                          
                          case 4: //roll mixer input
                          copter.m_input          = motors->get_roll();
                          attitude_control->actuator_roll_sysid(copter.m_value);
                          copter.m_output         = ahrs.get_gyro().x;    //copter.ins.get_gyro(0).x;
                          break;

                          case 5: //pitch mixer input
                          copter.m_input          = motors->get_pitch();
                          attitude_control->actuator_pitch_sysid(copter.m_value);
                          copter.m_output         = ahrs.get_gyro().y;
                          break;

                          case 6: //yaw mixer input
                          copter.m_input          = motors->get_yaw();
                          attitude_control->actuator_yaw_sysid(copter.m_value);
                          copter.m_output         = ahrs.get_gyro().z;
                          break;
                          
                          case 7:
                          break;

                          case 8: 
                          copter.m_input          = attitude_control->rate_bf_targets().x;
                          attitude_control->rate_bf_roll_sysid(radians(copter.m_value));
                          copter.m_output         = ahrs.get_gyro().x;
                          break;

                          case 9:
                          copter.m_input          = attitude_control->rate_bf_targets().y;
                          attitude_control->rate_bf_pitch_sysid(radians(copter.m_value));
                          copter.m_output         = ahrs.get_gyro().y;
                          break;

                          case 10:
                          copter.m_input          = attitude_control->rate_bf_targets().z;
                          attitude_control->rate_bf_yaw_sysid(radians(copter.m_value));
                          copter.m_output         = ahrs.get_gyro().z;
                          break;
                          
                          default:
                          break;
                      }
                  }
                ///end of chirp_ready/chirp_init
                }
                //end of we are in middle mode
                }else{
                  //we are not  in middle mode
                  copter.chirp_init = false;
                  copter.chirp_ready = true;
                  copter.m_input = 0.0f;
                  copter.disturbance_injection = 0.0f;
                  copter.m_output = 0.0f;
                  copter.m_value  = 0.0f;
                }
              }//end of chirp enabled, and motors armed

          //want to record preInjected signal, the signal itself, and output signal
          /* preInject -> copter.m_input                  */
          /* signal -> copter.m_value -> chirp itself     */
          /* output -> copter.m_output -> output channel  */

          //outside of chirp code..want to log minimally at 10hz and when in chirp log at 400hz
          if(copter.chirp_ready && copter.chirp_init && !copter.chirp_setting_done){
              //chirp is ready and initalized and the chirp has not been completed yet ...log at 400
              copter.Log_Write_Chirp_Short(g2.user_parameters.get_chirpAxes(), copter.m_input,  copter.m_value , copter.m_output);
              copter.m_counter_++;
          }else{
              // we aren't doing chirp..log at 10hz
              copter.m_counter_++;
              if( (copter.m_counter_ % 40) == 0 ){
                copter.Log_Write_Chirp_Short(g2.user_parameters.get_chirpAxes(), copter.m_input,  copter.m_value , copter.m_output);
              }
          }
        }//end of not-stabilize

      #endif  
    }

#if OPTIMAERO_CHIRP_ENABLED == ENABLED

/** The Chirp class produces a Chirp frequency-sweep profile for which
 * each doubling of frequency takes half as long as the previous. */
/** Sets the amplitude of oscillation for the Chirp.
 @param amplitude_in Amplitude of oscillation */
void Copter::ChirpSetAmplitude( float amplitude_in) { m_amplitude = amplitude_in; }

/** Sets the start and stop frequencies of the Chirp
 @param start_frequency_hz Starting frequency in Hz
@param stop_frequency_hz Final frequency in Hz
@param dt time step in seconds (rate at which Update will be called) */
void Copter::ChirpSetFrequencies( float start_frequency_hz,
                                  float stop_frequency_hz,
                                  float dt)
{
    
  m_dt = dt;

  // Sanity-check inputs.  Set parameters to zero if inputs are bad.
  // Frequencies should be increasing and less than Nyquist.
  if ((start_frequency_hz >= stop_frequency_hz) ||
      ((stop_frequency_hz*2.f) > (1.f/m_dt)))
  {
      m_phaseInRate     = 0.f;
      m_startFrequency  = 0.f;  
      m_stopFrequency   = 0.f;
  }
  else
  {
      m_phaseInRate     = start_frequency_hz*m_dt;
      m_startFrequency  = m_phaseInRate*TAU;  
      m_stopFrequency   = stop_frequency_hz*TAU*m_dt;
  }
}

/** Resets the state of the Chirp to the beginning of the profile */
void Copter::ChirpReset()
{
  // The phase-in stage should start with a cosine wave, but the
  // termination stage is simpler using a sine wave.  Starting at
  // TAU/4 makes it look like a cosine.
  const float starting_phase = TAU / 4.f;

  m_phase           = starting_phase;
  m_phaseInGain     = 0.f;
  m_frequency       = m_startFrequency;
  m_value           = (float)0.0f;
  m_completedFlag   = false;
} 

/** Initialize an instance of a Chirp
 @param start_frequency_hz Starting frequency in Hz
@param stop_frequency_hz Final frequency in Hz
@param CPO Number of complete oscillations before doubling in frequency
@param dt time step in seconds (rate at which Update will be called)
@param amplitude_in Amplitude of oscillation (default 1) */
void Copter::ChirpInit(float start_frequency_hz,
          float stop_frequency_hz,
          float CPO,
          float dt,
          float amplitude_in)
{
  ChirpSetFrequencies(start_frequency_hz, stop_frequency_hz, dt);

  if (CPO < 1.f) cycles_per_octave = 1.f;

  // The expression for 'ratio' is an approximation of the following:
  //
  //  ratio = (exp(1.f/(TAU*cycles_per_octave))-1)*(float)M_LN2;
  //
  // This approximation is very accurate for cycles_per_octave values
  // greater than 1, without the cost of the 'exp' call.
  m_ratio = LN2 / (TAU*cycles_per_octave - 0.5f);

  ChirpSetAmplitude(amplitude_in);

  ChirpReset();
}



/** Executes one iteration of the chirp waveform.  Must be called at the dt
 update period at which it was configured to get the desired frequencies.

The Chirp waveform has 3 stages:
- the phase-in stage increases the amplitude of the oscillation from
  zero to its full value for one cycle.  The phase-in period is at the
  start_frequency.
- the frequency-sweep phase, during which the frequency increases from
  start_frequency to stop_frequency at a constant amplitude
- the termination phase.  The oscillation continues at the stop_frequency
  until it approaches zero from below, then holds at zero until reset.
@return the value of the chirp waveform for the current time step. */
float Copter::ChirpUpdate()
{
  // phase-in period.  Linear ramp amplitude from 0 to full for one cycle
  // without increasing frequency.  This phase-in period integrates to zero
  // so the integral of the chirp has no bias.
  if (m_phaseInGain < (1.f - m_phaseInRate/2.f))
  {
      m_phase       += m_startFrequency;
      m_phaseInGain += m_phaseInRate;
      m_value        = m_amplitude*(float)(sinf(m_phase)*m_phaseInGain);
  }
  // Termination.  Wait for current cycle to complete, then set to zero.
  else if(m_frequency >= m_stopFrequency)
  {
      m_phase          += m_stopFrequency;
      if(m_phase > TAU)
      {
        m_value         = (float)0.0f;
        m_completedFlag = true;
      }
      else
      {
        m_value         = m_amplitude*(float)sinf(m_phase);
      }
  }
  else
  {
      m_phase           += m_frequency/2.f;
      m_frequency       /= 1.f-m_ratio*m_frequency;
      m_phase           += m_frequency/2.f;

      if(m_phase > TAU) m_phase -= TAU;
      m_value           = m_amplitude*(float)sinf(m_phase);
  }

  return m_value;
}

/** Returns the approximate number of loop iterations to execute the chirp
   profile as configured.
@return The duration of the chirp profile in number of loop iterations. */
uint32_t Copter::ChirpDuration()
{
  return (uint32_t)((1.f/m_phaseInRate) + 
                    ((m_stopFrequency-m_startFrequency)
                        / (m_ratio*m_startFrequency*m_stopFrequency)) +
                    (TAU/m_stopFrequency/2.f) + 0.5f);
}

/** Returns the frequency in Hz of the current oscillation */
float Copter::ChirpFrequencyInHz()
{
  return (float) m_frequency / m_dt / TAU;
}

/** Accesses the current value of the Chirp profile. Implemented as a
   type-cast operator.
@return The value of the Chirp */
 

bool Copter::ChirpIsCompleted(void)
{
  return m_completedFlag;
}



#endif //end of chirp


#endif //end of lib