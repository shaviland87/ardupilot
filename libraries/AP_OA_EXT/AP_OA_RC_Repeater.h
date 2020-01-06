#pragma once

#include "optimAero.h"
#include "AP_OA_Backend.h"
#include "OA_Parser.h"
#include "OA_MessageTypes.h"
#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS_MAVLink.h> 	// to be able to send data to GCS

#define HRT_RATE_HZ 1
#define MISSED_HEARTS_LIMIT 3
#define CHECK_HEART_RATE_HZ 1 
class AP_OA_RC_Repeater : public AP_OA_Backend
{

public:
	//const
	AP_OA_RC_Repeater(optimAero::optimAero_State &_state,
			AP_OA_Params &_params,
			uint8_t serial_instance);

	//static detection
	static bool detect(uint8_t serial_instance);

	//update state
	void update400hz(void) override;
	void updateFast(void) override;
	void update10Hz(void) override;
	void updateSlow(void) override;
	void populateBuffer(OA_Parser &parser_in) override; //sim only
	void getRcOut(void);
	void updateMotorInterlock(void);

	void checkForData(void);	/*function to read serial data and update structures*/


protected:

private:
	AP_HAL::UARTDriver *uart = nullptr;
	OA_Parser 								parse_params_;
	struct rc_repeater_heartbeat_ref 		rc_heartbeat_;
	struct external_ap_rc_out_ref			motor_out_;
	AP_Motors *vehicle_motors = nullptr;

	bool 			motor_init;	
	unsigned char 	hrt_beat_rcvd_;
	unsigned char 	missed_hearts_;

		/*
    if (channel_auto.get() == 1) {
        AP_Motors *motors = AP_Motors::get_singleton();
        if (motors) {
            mask |= motors->get_motor_mask();
        }
    }*/

};
