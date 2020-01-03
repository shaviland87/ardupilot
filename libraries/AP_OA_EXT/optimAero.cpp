/*external library for comms with ardupilot dev by optimaero*/

#include "optimAero.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h> 					/*to send data to GCS */


//supported features
#include "AP_OA_Arduino.h"
#include "AP_OA_RC_Repeater.h"

extern const AP_HAL::HAL &hal;

//table of user set params
const AP_Param::GroupInfo optimAero::var_info[] = {

	// @Group: 1_
	// @Path: AP_OA_Params.cpp
	AP_SUBGROUPINFO(params[0], "1_", 25, optimAero, AP_OA_Params),
	/*TBD figure out what SUBGROUPINFO takes in that 25 is BS*/
	AP_SUBGROUPVARPTR(drivers[0], "1_", 57, optimAero, backend_var_info[0]),

#if OA_MAX_INSTANCES > 1
	// @Group: 1_
	// @Path: AP_OA_Params.cpp
	AP_SUBGROUPINFO(params[1], "2_", 27, optimAero, AP_OA_Params),
	/*TBD figure out what SUBGROUPINFO takes in that 27 is BS*/
	AP_SUBGROUPVARPTR(drivers[1], "2_", 58, optimAero, backend_var_info[1]),
#endif

#if OA_MAX_INSTANCES > 2
	// @Group: 1_
	// @Path: AP_OA_Params.cpp
	AP_SUBGROUPINFO(params[2], "3_", 29, optimAero, AP_OA_Params),
	/*TBD figure out what SUBGROUPINFO takes in that 25 is BS*/
	AP_SUBGROUPVARPTR(drivers[2], "3_", 59, optimAero, backend_var_info[2]),
#endif

	AP_GROUPEND
};

const AP_Param::GroupInfo *optimAero::backend_var_info[OA_MAX_INSTANCES];

optimAero::optimAero()
{

	if (_singleton) {

		#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
		AP_HAL::panic("Too many oa modules");
		#endif

		return;
	}
	_singleton = this;

	AP_Param::setup_object_defaults(this, var_info);
	
}

/*
 * Get the  singleton
 */
optimAero *optimAero::_singleton;
optimAero *optimAero::get_singleton()
{
    return _singleton;
}

//function skipped -> convert_params(void) - might convert from old pixhawks to new?

void optimAero::init(void)
{

	if(num_instances != 0 ){
		//init called a 2nd time
		return;
	}

	for(uint8_t i=0, serial_instance =0; i<OA_MAX_INSTANCES; i++) {
		//serial_instance will be increased in detect_instance
		detect_instance(i, serial_instance);

		if(drivers[i] != nullptr){
			//we loaded a driver for this instance
			num_instances = i+1;
		}

		state[i].status = optimAero_NotConnected;

	}//end of for loop

}


void optimAero::updateFast(void)
{

	for(uint8_t i=0; i < num_instances; i++){
		if(drivers[i] != nullptr) {
			if(params[i].type == optimAero_TYPE_NONE){
				//allow user to disable at run time
				state[i].status = optimAero_NotConnected;
				continue;
			}
			
			drivers[i]->updateFast();
		}

	}//end of for
}

void optimAero::update10Hz(void)
{

	for(uint8_t i=0; i < num_instances; i++){
		if(drivers[i] != nullptr) {
			if(params[i].type == optimAero_TYPE_NONE){
				//allow user to disable at run time
				state[i].status = optimAero_NotConnected;
				continue;
			}
			
			drivers[i]->update10Hz();
		}

	}//end of for
}

void optimAero::updateSlow(void)
{

	for(uint8_t i=0; i < num_instances; i++){
		if(drivers[i] != nullptr) {
			if(params[i].type == optimAero_TYPE_NONE){
				//allow user to disable at run time
				state[i].status = optimAero_NotConnected;
				continue;
			}
			
			drivers[i]->updateSlow();
			#ifndef HAL_BUILD_AP_PERIPH
				Log_optimAero();
			#endif
		}

	}//end of for
}

bool optimAero::_add_backend(AP_OA_Backend *backend)
{

	if(!backend){
		return false;
	}

	if(num_instances == OA_MAX_INSTANCES){
		AP_HAL::panic("Too many oa unit backends");
	}

	drivers[num_instances++] = backend;
	return true;
}

/*detect if an instance of oa is connected*/
void optimAero::detect_instance(uint8_t instance, uint8_t& serial_instance)
{ 
	enum optimAero_Type _type = (enum optimAero_Type)params[instance].type.get();

	switch(_type){
	
		/* _add_backend insert, detect(serial_instance)...*/
		case optimAero_TYPE_ARDUINO:
			if(AP_OA_Arduino::detect(serial_instance)) {
				drivers[instance] = new AP_OA_Arduino(state[instance], params[instance], serial_instance++);
			}
			break;
		case optimAero_TYPE_EXTCOMP:
			break;
		case optimAero_TYPE_EXTNAV:
			break;
		case optimAero_TYPE_EXTRC:
			if(AP_OA_RC_Repeater::detect(serial_instance)) {
				drivers[instance] = new AP_OA_RC_Repeater(state[instance], params[instance], serial_instance++);
			}
			break;
		default:
			break;

	}//end of switch

	if(drivers[instance] && state[instance].var_info){
		backend_var_info[instance] = state[instance].var_info;
		AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);
	}
}


AP_OA_Backend *optimAero::get_backend(uint8_t id) const{

	if(id >= num_instances){
		return nullptr;
	}

	if(drivers[id] != nullptr){
		if(drivers[id]->type() == optimAero_TYPE_NONE){
			//it isn't her disabed at runtime
			return nullptr;
		}
	}
	return drivers[id];
}

AP_OA_Backend *optimAero::find_instance(void) const
{
	//adapted code not sure if requried
	for(uint8_t i=0; i<num_instances; i++){
		AP_OA_Backend *backend = get_backend(i);
		if(backend != nullptr){
			return backend;
		}
	}//end of for

	return nullptr;

}


// get_status (status_orient)
// handle_msg

void optimAero::Log_optimAero()
{

	/*if(_log_oa_bit == uint32_t(-1)){
		return;
	}*/

	#if 0
	AP_Logger &logger = AP::logger();
	if(!logger.should_log(_log_oa_bit)){
		return;
	}

	for(uint8_t i=0; i<OA_MAX_INSTANCES; i++){
		const AP_OA_Backend *s = get_backend(i);
		if( s == nullptr){
			continue;
		}

		//define const struct log packet

		// AP::logger().WriteBlock(&pkt, sizeof(pkt));

	}//endfor
	#endif
    AP_Logger *logger = AP_Logger::get_singleton();
    logger->Write_OA_Arduino(0); //hrt
	logger->Write_OA_Arduino(1); //thermister and arm data
	//logger->Write_OA_Arduino(2); //analog


}

float optimAero::get_batt_voltage(uint8_t instance) const {

	if( instance > num_instances){
		return 0.0f;
	}else{
		float tmp = 0.0;
		for(int i=0;i<NUMCELL*NUMBATT;i++){
			tmp += state[instance].batt_cells.cells[i]/1000.0f; //in millivolts --convert to v
		}

		return tmp;
	}

}

void optimAero::sendBuffer(OA_Parser &parser_in){


	for(uint8_t i=0; i < num_instances; i++){
		if(drivers[i] != nullptr) {			
			drivers[i]->populateBuffer(parser_in);
		}

	}//end of for
}

namespace AP {

	optimAero *oa()
	{
		return optimAero::get_singleton();
	}
};


/*

    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger->should_log(_log_battery_bit)) {
        logger->Write_Current();
        logger->Write_Power();
    }

*/