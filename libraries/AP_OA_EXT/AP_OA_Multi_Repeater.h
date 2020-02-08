#pragma once

#include "optimAero.h"
#include "AP_OA_Backend.h"
#include "OA_Parser.h"
#include "OA_MessageTypes.h"
#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS_MAVLink.h> 	// to be able to send data to GCS
#include <GCS_MAVLink/GCS.h>


class AP_OA_Multi_Repeater : public AP_OA_Backend
{

public:
	//const
	AP_OA_Multi_Repeater(optimAero::optimAero_State &_state,
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
//////////////////////////////////////////////////
    /*arduino related code*/
	
    void checkTemperatures(void);
	void checkArmConnects(void);

    /*these tell us if it makes sense to check the data itself for temps/analog/battery etc*/
	/*these give warnings to GCS if we lose/gain data*/
	bool updateTemperatureHealth(void);
	bool updateAnalogHealth(void);
	bool updateBatteryHealth(void);
    
    /*end of arduino related code*/
////////////////////////////////////////////////////

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

    /*arduino related private*/
    struct MsgBattCell_ref 			battery_in_;
	struct MsgAnalogData_ref 		analog_in_;
	struct MsgThermistersData_ref 	thermisters_in;
	struct MsgBattCellSplit_ref		battery_split_in_;
	bool 							battery_updated_;
	bool							battery_split_updated_;
	bool 							analog_updated_;
	bool 							thermister_updated_;
	char		 					arm_connects[MAX_ANALOG_CAPACITY];
	unsigned char 					battery_health_cntr_;
	unsigned char 					analog_health_cntr_;
	unsigned char 					temp_health_cntr_;

	bool 							send_battery_health_warning_;
	bool							send_analog_health_warning_;
	bool							send_temperature_health_warning_;
	bool 							arm_disconnected_;
	bool 							max_temperature_reached_;

};
