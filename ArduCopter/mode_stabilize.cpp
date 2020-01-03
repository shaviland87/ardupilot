#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

#if OPTIMAERO_LIBRARY_ENABLED == ENABLED
    #if OPTIMAERO_CHIRP_ENABLED == ENABLED
                            
    float pilot_throttle_scaled = get_pilot_desired_throttle();

    if(g2.user_parameters.get_chirpEnabled() &&
        g2.user_parameters.get_chirpAxes() <= 7 )
        {
            //chirp enabled and chirp axes is throttle channel
            if(g2.rc_channels.rc_channel(4)->get_radio_in() > 1400 && 
            g2.rc_channels.rc_channel(4)->get_radio_in() < 1600 )
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

                    if(copter.chirp_setting_done == true ){
                        //we are done
                        if(copter.chirp_no_spin == false){
                            copter.chirp_no_spin = true;
                            gcs().send_text(MAV_SEVERITY_INFO, "CHIRP COMPLETE");
                        }
                    }else{
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
                            //throttle
                            copter.m_input              = pilot_throttle_scaled; 
                            pilot_throttle_scaled       = pilot_throttle_scaled + copter.m_value;
                            break;
                            
                            case 4:
                            break;
                            
                            case 7:
                            //throttle
                            copter.m_input              = pilot_throttle_scaled; 
                            pilot_throttle_scaled       = pilot_throttle_scaled + copter.m_value;
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
            }

        //end of chirp enabled and axes set correctly
        }


    #endif
#endif


    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

#if OPTIMAERO_LIBRARY_ENABLED == ENABLED
    #if OPTIMAERO_CHIRP_ENABLED == ENABLED
    if(g2.user_parameters.get_chirpAxes() == 7 ){ 
        //throttle mode
        // output pilot's throttle
        attitude_control->set_throttle_out(pilot_throttle_scaled,
                                        false,
                                        g.throttle_filt);
    }else{ 
        // standard mode
        // output pilot's throttle
        attitude_control->set_throttle_out(pilot_throttle_scaled,
                                        true,
                                        g.throttle_filt);
    }

    #else
    // output pilot's throttle
    attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       true,
                                       g.throttle_filt);
    #endif

#else
    // output pilot's throttle
    attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       true,
                                       g.throttle_filt);

#endif

}
