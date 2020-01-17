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
    // want to log constantly at 10 hz...in chirp status log instead at 400hz
                            
    float pilot_throttle_scaled = get_pilot_desired_throttle();

    if(g2.user_parameters.get_chirpEnabled() &&
        g2.user_parameters.get_chirpAxes() <= 10 )
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
                            //roll angle
                            copter.m_input          = target_roll;
                            target_roll             = target_roll + copter.m_value;
                            copter.m_output         = attitude_control->get_att_target_euler_cd().x;
                            break;

                            case 1:
                            //pitch angle
                            copter.m_input          = target_pitch;
                            target_pitch            = target_pitch + copter.m_value;
                            copter.m_output         = attitude_control->get_att_target_euler_cd().y;
                            break;
                            
                            case 2:
                            //yaw rate
                            copter.m_input          = target_yaw_rate;
                            target_yaw_rate         = target_yaw_rate + copter.m_value;   
                            copter.m_output         = wrap_360_cd(attitude_control->get_att_target_euler_cd().z);
                            break;
                            
                            case 3:
                            //throttle
                            copter.m_input          = pilot_throttle_scaled; 
                            pilot_throttle_scaled   = pilot_throttle_scaled + copter.m_value;
                            copter.m_output         = copter.ins.get_accel(0).z;
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
                            //throttle
                            copter.m_input          = motors->get_throttle();
                            pilot_throttle_scaled   = pilot_throttle_scaled + copter.m_value;
                            copter.m_output         = copter.ins.get_accel(0).z;
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

        //end of chirp enabled and axes set correctly
        }

        //outside of chirp code..want to log minimally at 10hz and when in chirp log at 400hz
        if(copter.chirp_ready && copter.chirp_init && !copter.chirp_setting_done){
            //chirp is ready and initalized and the chirp has not been completed yet ...log at 400
            log_data();
            counter_++;
        }else{
            // we aren't doing chirp..log at 10hz
            counter_++;
            if( (counter_ % 40) == 0 ){
                log_data(); //we are going to log at lower data rate when not doing a chirp
            }
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

/*
log needs to contain axes, input,output
possible to do mimo eventually
*/

void ModeStabilize::log_data(){

    //want to record preInjected signal, the signal itself, and output signal
    /* preInject -> copter.m_input                  */
    /* signal -> copter.m_value -> chirp itself     */
    /* output -> copter.m_output -> output channel  */
    #if OPTIMAERO_LIBRARY_ENABLED == ENABLED
        #if OPTIMAERO_CHIRP_ENABLED == ENABLED
            copter.Log_Write_Chirp_Short(g2.user_parameters.get_chirpAxes(), copter.m_input,  copter.m_value , copter.m_output);
        #endif
    #endif

}