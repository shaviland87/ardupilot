
#pragma once

#include "optimAero.h"
#include "AP_OA_Backend.h"
#include "OA_Parser.h"
#include "OA_MessageTypes.h"
#include <GCS_MAVLink/GCS_MAVLink.h> 	// to be able to send data to GCS

class AP_OA_Arduino : public AP_OA_Backend
{

public:
	//const
	AP_OA_Arduino(optimAero::optimAero_State &_state,
			AP_OA_Params &_params,
			uint8_t serial_instance);

	//static detection
	static bool detect(uint8_t serial_instance);

	//update state
	void update400hz(void) override;
	void updateFast(void) override;
	void update10Hz(void) override;
	void updateSlow(void) override;
	void populateBuffer(OA_Parser &parser_in) override; //sim only use

	void checkForData(void);	/*function to read serial data and update structures*/
	void checkTemperatures(void);
	void checkArmConnects(void);

	/*these tell us if it makes sense to check the data itself for temps/analog/battery etc*/
	/*these give warnings to GCS if we lose/gain data*/
	bool updateTemperatureHealth(void);
	bool updateAnalogHealth(void);
	bool updateBatteryHealth(void);

	/*need something to check temperature data*/
	/*need something to check analog values if we are in arm-connect mode*/
	/*hopefullly ap-battery code handles all battery stuff*/

	/* THESE SHOULD BE A CLASS!*/
	// (battery_updated_, battery_health_cntr_, send_battery_health_warning_) and others!!!! TBD

protected:

private:
	AP_HAL::UARTDriver *uart 		= nullptr;

	OA_Parser						parse_params_;
	struct MsgBattCell_ref 			battery_in_;
	struct MsgAnalogData_ref 		analog_in_;
	struct MsgThermistersData_ref 	thermisters_in;
	bool 							battery_updated_;
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
