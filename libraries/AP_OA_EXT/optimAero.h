/* optimAero library for AP*/

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_OA_Params.h"
#include "OA_MessageTypes.h" 			// to grab cell size etc..
#include "OA_Parser.h" 					// sim only use
#include <GCS_MAVLink/GCS_MAVLink.h> 	// to be able to send data to GCS


#ifndef OA_MAX_INSTANCES
	// only allow so many of these 	
	#define OA_MAX_INSTANCES 2
#endif


class AP_OA_Backend;

class optimAero
{
	friend class AP_OA_Backend;

public:
	optimAero();
	/*don't allow copies*/
	optimAero(const optimAero &other) = delete;
	optimAero &operator=(const optimAero&) = delete;

	static optimAero *get_singleton();
	static optimAero *_singleton;

	enum optimAero_Type {
		optimAero_TYPE_NONE = 0,
		optimAero_TYPE_ARDUINO = 1,
		optimAero_TYPE_EXTCOMP = 2,
		optimAero_TYPE_EXTNAV = 3,
		optimAero_TYPE_EXTRC = 4,
	};

	enum optimAero_Status {
		optimAero_NotConnected = 0,
		optimAero_NoData,
		optimAero_ArduinoGood,
		optimAero_RepeaterGood,
		optimAero_ArduinoBad,
		optimAero_RepeaterBad
	};

    struct ardu_cells {
        uint16_t cells[NUMCELL*NUMBATT];
    };

	struct optimAero_State {
		enum optimAero_Status status; 		// oa Status
		uint32_t 	last_reading_ms; 		// system time of last update
		ardu_cells 	batt_cells;
		int8_t 		ardu_analog_data[10];
		float 		temperatures[4];
		uint16_t 	badChecksums;
		uint16_t 	badIDs;

		const struct AP_Param::GroupInfo *var_info;
	};

	
	static const struct AP_Param::GroupInfo *backend_var_info[OA_MAX_INSTANCES];
	
	//params for each instance
	static const struct AP_Param::GroupInfo var_info[];

	uint8_t num_oa_connections(void) const {
		return num_instances;
	}

	// indicate which bit in log_bitmask indicates OA should be logged
	void set_log_optimAero(uint32_t log_oa_bit) {_log_oa_bit = log_oa_bit;}

	// update state for all optimAero external units-> called in either mainloop or usercode -> this needs to be overriden in gerps
	void update400hz(void);
	void updateFast(void);
	void update10Hz(void);
	void updateSlow(void);
	void sendBuffer(OA_Parser &parser_in); //sim only

    /// get_type - returns battery monitor type
	AP_Int8 get_type() const {return get_type(0);}
	AP_Int8 get_type(uint8_t instance) const {return params[instance].type;}

	// first version -- 
	//	ardu_cells get_batt_cells() const {return state->batt_cells;}
	//	void set_batt_cells(ardu_cells &cell_)  {state->batt_cells = cell_;}	//sim only

	ardu_cells get_batt_cells() const {return get_batt_cells(0); }
	ardu_cells get_batt_cells(uint8_t instance) const {return state[instance].batt_cells;}

	void set_batt_cells(ardu_cells &cell_)  {state[0].batt_cells = cell_;}							//sim only
	void set_batt_cells(uint8_t instance, ardu_cells &cell_)  {state[instance].batt_cells = cell_;}	//sim only

	//first version
	//void set_batt_cells(ardu_cells &cell_)  {state[instance]->batt_cells = cell_;}	//sim only

	float get_batt_voltage(uint8_t instance) const;	

	AP_OA_Backend *find_instance(void) const;

	void init(void);

	AP_OA_Backend *get_backend(uint8_t id) const;

	uint16_t get_badChecksums(uint8_t instance) const {return state[instance].badChecksums;}
	uint16_t get_IDs(uint8_t instance) const {return state[instance].badIDs;}
	int8_t 	 get_analog_value(uint8_t instance, uint8_t index) const 
	{
		if(index < MAX_ANALOG_CAPACITY){
			return state[instance].ardu_analog_data[index];
		}else{
			return 0;
		}		
	}
	float 	get_temperature_value(uint8_t instance, uint8_t index) const
	{
		if(index < NUM_THERMISTERS){
			return state[instance].temperatures[index];
		}else{
			return 0.0f;
		}
	} 

protected:
	AP_OA_Params params[OA_MAX_INSTANCES];

private:
	optimAero_State state[OA_MAX_INSTANCES];
	AP_OA_Backend *drivers[OA_MAX_INSTANCES];
	uint8_t num_instances;

	void detect_instance(uint8_t instance, uint8_t &serial_instance);
	bool _add_backend(AP_OA_Backend *driver);
	
	uint32_t _log_oa_bit = -1;
	void Log_optimAero();

};

namespace AP {
	optimAero *oa();
}
