#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_OA_Multi_Repeater.h"
#include <GCS_MAVLink/GCS.h> 					/*to send data to GCS */

/*
    // initialise pointers to serial ports
    state[1].uart = hal.uartC;  // serial1, uartC, normally telem1
    state[2].uart = hal.uartD;  // serial2, uartD, normally telem2
    state[3].uart = hal.uartB;  // serial3, uartB, normally 1st GPS
    state[4].uart = hal.uartE;  // serial4, uartE, normally 2nd GPS
    state[5].uart = hal.uartF;  // serial5
    state[6].uart = hal.uartG;  // serial6
    state[7].uart = hal.uartH;  // serial7*/
	// SerialProtocol_optimAero = 27,              //custom optimAero


extern const AP_HAL::HAL& hal;

/* const inits the oa-units not called till detect() returns true*/

AP_OA_Multi_Repeater::AP_OA_Multi_Repeater(optimAero::optimAero_State &_state,
				AP_OA_Params &_params,
				uint8_t serial_instance) :
	AP_OA_Backend(_state,_params)
{

	motor_out_.sync 		= OPTIM_MESSAGE_SYNC;
	motor_out_.messageID 	= ID_EXTERNAL_AP_RC_OUT;
	motor_out_.messageSize 	= sizeof(external_ap_rc_out_ref);
	motor_out_.count 		= 0;

	const AP_SerialManager &serial_manager = AP::serialmanager();
	uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_optimAero, serial_instance);

	if(uart != nullptr){
		uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_optimAero, serial_instance));
	} //tbd might want this a param

    //not sure if these are needed
	motor_init = false;
	if(nullptr != vehicle_motors ){
		vehicle_motors 	= AP_Motors::get_singleton();
		motor_init 		= true;
	}

	parse_params_.bytesInBuffer = 0; //make sure it knows its zero
	hrt_beat_rcvd_ 				= 0; //number of hrt beats recevd since last check
	missed_hearts_ 				= 0;

    //////////////////////////////////////////////////
    //arduino related init
    /*updates these if we have recvd them from arduino*/
	battery_updated_ 		= false;
	battery_split_updated_ 	= false;
	analog_updated_			= false;
	thermister_updated_ 	= false;

	for(int i=0;i<MAX_ANALOG_CAPACITY;i++){
		arm_connects[i] 	= ARDUINO_ARM_NOT_ENABLED;
		analog_in_.data[i] 	= ARDUINO_ARM_NOT_ENABLED;
	}

	/*counters to monitor healthy signals*/
	temp_health_cntr_ 		= 0;
	analog_health_cntr_ 	= 0;
	battery_health_cntr_ 	= 0;

	/*booleans used to send text warnings to ground*/
	send_analog_health_warning_ 		= false;
	send_battery_health_warning_ 		= false;
	send_temperature_health_warning_ 	= false;
    ////////////////////////end of arduino related

}

/*detect if connected*/

bool AP_OA_Multi_Repeater::detect(uint8_t serial_instance)
{
	return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_optimAero, serial_instance) != nullptr;
}

void AP_OA_Multi_Repeater::update400hz(void){
	
	//send motor commands	
	getRcOut(); // populates the rc outputs  -> probably should have another loop@400hz that does this
	if(nullptr != uart){
		parse_params_.optim_serialWrite(uart, (uint8_t *)&motor_out_, motor_out_.messageSize);
	}
}

void AP_OA_Multi_Repeater::updateFast(void)
{
	checkForData(); /*read data from repeater - nothing gets sent fast back*/
}

void AP_OA_Multi_Repeater::update10Hz(void){
	/*probably dont need to do this but just in case values get mod'd somehow*/
	motor_out_.sync 		= OPTIM_MESSAGE_SYNC;
	motor_out_.messageID 	= ID_EXTERNAL_AP_RC_OUT;
	motor_out_.messageSize 	= sizeof(external_ap_rc_out_ref);

    //arduino related code....
	if(params.function | MASK_ARDUINO_ARMS){

		if(updateAnalogHealth()){
			checkArmConnects(); 	//only worth checking if data is healthy...
		}
	}


}
void AP_OA_Multi_Repeater::updateSlow(void){

	updateMotorInterlock();

	if(hrt_beat_rcvd_ < HRT_RATE_HZ*CHECK_HEART_RATE_HZ){
		missed_hearts_++;
	}else{
		missed_hearts_ = 0;
	}

	if(missed_hearts_ > MISSED_HEARTS_LIMIT){
		gcs().send_text(MAV_SEVERITY_WARNING,"EXT RC DOWN");
	}

///////////////////////////////////////////////////////////////////
    //arduino related code...........
	
	if(params.function | MASK_ARDUINO_TEMPERATURE){

		if(updateTemperatureHealth()){
			checkTemperatures(); //only worth checking if data is health

			if( true == max_temperature_reached_){
				gcs().send_text(MAV_SEVERITY_WARNING,"MAX TEMP REACHED");
			}
		}
	}

	if(params.function | MASK_ARDUINO_BATT){
		updateBatteryHealth(); //determines health -> also will bitch about no data hopefully
	}

	if(params.function | MASK_ARDUINO_ARMS){
		if(true == arm_disconnected_ ){
			//alert GCS about arm disconnect
			gcs().send_text(MAV_SEVERITY_WARNING, "ARM DISCONNECTED");
		}
	}
//////////////////////////////////////////////////////////////////////////////////    




}
void AP_OA_Multi_Repeater::populateBuffer(OA_Parser &parser_in){
	/*sim function only*/

	if(parser_in.bytesInBuffer > 0 && parse_params_.bytesInBuffer < OA_PARSER_MAX_BUFFER_SIZE)
	{
		// populate a buffer if data avail and buffer container size is not full

		for(int i=0; i<parser_in.bytesInBuffer; i++){
			parse_params_.datalinkBuffer[parse_params_.bytesInBuffer] = parser_in.datalinkBuffer[i];
			#if 0
				printf("byte[%d] = %02X \n",parse_params_.bytesInBuffer,parse_params_.datalinkBuffer[parse_params_.bytesInBuffer]);
			#endif
			parse_params_.bytesInBuffer++;
		}
		
		memset (parser_in.datalinkBuffer,0,parser_in.bytesInBuffer);
		parser_in.bytesInBuffer = 0;
	}
}


void AP_OA_Multi_Repeater::getRcOut(void){

	motor_out_.count++;

	for(int i=0;i<EXTERNAL_AP_NUM_MOT; i++){
		motor_out_.rc[i] = 	hal.rcout->read(i);
	}
	/*interlock is updated at slower rate*/
	motor_out_.csum 		= parse_params_.fletcher16((unsigned char *)&motor_out_, motor_out_.messageSize - OPTIM_MESSAGE_CSUM_SIZE);

}

void AP_OA_Multi_Repeater::updateMotorInterlock(void){
	motor_out_.motor_enable = hal.util->safety_switch_state(); //read state of switch
}

void AP_OA_Multi_Repeater::checkForData(void){

	if(nullptr != uart ){
		// uart setup good to go

		while(uart->available() > 0 && parse_params_.bytesInBuffer < OA_PARSER_MAX_BUFFER_SIZE)
		{
			// populate a buffer if data avail and buffer container size is not full
			parse_params_.datalinkBuffer[parse_params_.bytesInBuffer] = uart->read();
			//hal.uartE->printf("byte[%d] = %02X ,",parse_params_.bytesInBuffer,parse_params_.datalinkBuffer[parse_params_.bytesInBuffer]);
			parse_params_.bytesInBuffer++;
		}

		int iData       = 0;
    	int dataDone    = 0;

		while( (iData <= parse_params_.bytesInBuffer - OPTIM_MESSAGE_HDR_SIZE) && !dataDone )
		{
			if(parse_params_.datalinkBuffer[iData] == OPTIM_MESSAGE_SYNC )
			{

				parse_params_.bf      = &(parse_params_.datalinkBuffer[iData]);
				parse_params_.readID  = parse_params_.bf[1];
				parse_params_.readSize= parse_params_.bf[2];
				//hal.uartE->printf("  sync \n");

				if( parse_params_.readSize < OPTIM_MESSAGE_MAX_SIZE && parse_params_.readSize >= OPTIM_MESSAGE_HDR_SIZE )
				{ // read size is in range

					if( iData <= parse_params_.bytesInBuffer - parse_params_.readSize )
					{ // have we read enough bytes
						parse_params_.csum     =  (unsigned int)(parse_params_.bf[parse_params_.readSize-2]     );
						parse_params_.csum    |=  (unsigned int)(parse_params_.bf[parse_params_.readSize-1] << 8);
						parse_params_.csumComp = parse_params_.fletcher16( (unsigned char *)parse_params_.bf, 
																(short)(parse_params_.readSize - OPTIM_MESSAGE_CSUM_SIZE) );

						if( parse_params_.csum == parse_params_.csumComp )
						{
							//hal.uartE->printf("csum\n");
							switch (parse_params_.readID)
							{
								case ID_REPEATER_HEARTBEAT:
								case ID_BAD_AP_LINK:
									memcpy(&rc_heartbeat_, parse_params_.bf, sizeof(rc_repeater_heartbeat_ref));
									hrt_beat_rcvd_++; //increment our recvs
										#if 0
											//printf("hrt rcv\n");
										#endif
									break;

								case ID_EXTERNAL_AP_RC_OUT:
									//memcpy(&rcOut_,bf,sizeof(external_ap_rc_out_ref));
									break;

								case ID_EXTERNAL_AP_MIXER_OUT:
									break;
								
								case ID_MESSAGE_BATT_CELL:
									memcpy(&battery_in_, parse_params_.bf, sizeof(MsgBattCell_ref));
									battery_updated_ = true;
									hal.uartD->printf("batt recv\n");
									break;
								case ID_MESSAGE_BATT_CELL_SPLIT:
									memcpy(&battery_split_in_, parse_params_.bf, sizeof(MsgBattCellSplit_ref));
									battery_updated_ = true;
									battery_split_updated_ = true;
									hal.uartD->printf("batt split recv\n");
									break;
								case ID_MESSAGE_ANALOG_DATA:
									memcpy(&analog_in_,parse_params_.bf,sizeof(MsgAnalogData_ref));
									analog_updated_ = true;
									break;

								case ID_MESSAGE_THERMISTERS_DATA:
									memcpy(&thermisters_in, parse_params_.bf, sizeof(MsgThermistersData_ref));
									thermister_updated_ = true;
									break;
                                                                    
								default:{
									parse_params_.unknownReadID++;
									state.badIDs = parse_params_.unknownReadID;
									hal.uartD->printf("bad read id\n");
								}break;
							}

							//packetsRecvd

						}else
						{
							parse_params_.badChecksums++;
							state.badChecksums = parse_params_.badChecksums;
							//bad checksum!!
							hal.uartD->printf("bad ck\n");

						}
						
						iData += parse_params_.readSize; //move forward by readSize

					}else
					{
						//half pkt read -- move on
						dataDone = 1;
					}

				}else
				{ 
					// invalid message size -> move forward
					iData++;
				}

			}else{
				// buffer(idata) != sync
				iData++;
			}

		}//end of while parse loop

	if(iData > 0){ 
		// only need to shift buffer if we read anything
		parse_params_.bytesInBuffer = parse_params_.bytesInBuffer - iData;
		memmove( parse_params_.datalinkBuffer, &(parse_params_.datalinkBuffer[iData]), parse_params_.bytesInBuffer );
	}


	}//end of nullptr = uart

}


void AP_OA_Multi_Repeater::checkTemperatures(void){
	// should make a param for max temperature allowed? TBD....

	/* at this point we have confirmed that we have rcvd temp msg -> check for hottness*/
	for(int i=0; i< NUM_THERMISTERS; i++){
		if(thermisters_in.tempF[i] > MAX_TEMPERATURE_THERMISTER){
			max_temperature_reached_ = true;
			//an also use _INFO/_NOTICE
			// gcs().send_text(MAV_SEVERITY_WARNING, "Battery %d is %s %.2fV used %.0f mAh", i + 1, type_str,(double)voltage(i), (double)state[i].consumed_mah);
		}
	}


}

void AP_OA_Multi_Repeater::checkArmConnects(void){
	/* Jumping into this function we have already determined that we have received arm_connect data to check*/

	// assuing one analog measurement per arm
	// grab from ap what vehicle config we dealing with..asssuming high value for analog is 'CONNECTED"
	// ARM CONNECT VALUES [-1= NOT ENABLED, 0= DISCONNECT , 1= CONNECT]
	
	arm_disconnected_ = false;

	for(int i=0; i<MAX_ANALOG_CAPACITY;i++){

		//grab new data
		arm_connects[i] = analog_in_.data[i];

		//we really only care if any arms are in a disconnected state
		if(arm_connects[i] == ARDUINO_ARM_DISCONNECTED){
			arm_disconnected_ = true;
		}

	}
}

bool AP_OA_Multi_Repeater::updateBatteryHealth(void){
	/*these only get called if the param_function is set to want battery data*/
	
	bool isBatteryHealthy = false; //output of function

	/////////////////////////////////////////////////////////////
	///////////// CHECK FOR UPDATE
	if(battery_updated_){
		battery_health_cntr_ 	= 0;
		battery_updated_ 		= false;
		isBatteryHealthy 		= true;

		//populate state of battery
		if(battery_split_updated_){
			for(int j=0; j<30; j++){
				state.batt_cells.cells[j] = battery_split_in_.cell[j];
			}
		}else{
			for(int j=0; j<NUMBATT*NUMCELL; j++){
				state.batt_cells.cells[j] = battery_in_.cell[j];
				hal.uartD->printf("cell %d = %d \n",j, battery_in_.cell[j]);
			}
		}


		if(true == send_battery_health_warning_){
			send_battery_health_warning_ = false;
			// send something to ground saying battery data back
			gcs().send_text(MAV_SEVERITY_NOTICE, "Regained Comms with oaBattery");
			//an also use _INFO
		}

	}else{
		battery_health_cntr_++;
	}
	///////////////////////////////////////////////////////////////

	if(battery_health_cntr_ > WARN_BATT_COUNT && send_battery_health_warning_ == false){
		send_battery_health_warning_ = true;
		// send something to ground saying battery data gone
		gcs().send_text(MAV_SEVERITY_WARNING, "Lost Comms with oaBattery");
	}

	return isBatteryHealthy;

}

bool AP_OA_Multi_Repeater::updateAnalogHealth(void){

	bool isAnalogHealthy = false;
	/////////////////////////////////////////////////////////////
	///////////// CHECK FOR UPDATE
	if(analog_updated_){
		analog_health_cntr_ 	= 0;
		analog_updated_ 		= false;
		isAnalogHealthy 		= true;

		for(int j=0; j<MAX_ANALOG_CAPACITY; j++){
			state.ardu_analog_data[j] = analog_in_.data[j];
		}

		if(true == send_analog_health_warning_){
			send_analog_health_warning_ = false;
			// send something to ground saying analog data back
			gcs().send_text(MAV_SEVERITY_NOTICE, "Regained Comms with oaAnalog");
		}

	}else{
		analog_health_cntr_++;
	}
	///////////////////////////////////////////////////////////////

	if(analog_health_cntr_ > WARN_ANAL_COUNT && send_analog_health_warning_ == false){
		send_analog_health_warning_ = true;
		// send something to ground saying analog data gone
		gcs().send_text(MAV_SEVERITY_WARNING, "Lost Comms with oaAnalog");
	}

	return isAnalogHealthy;

}

bool AP_OA_Multi_Repeater::updateTemperatureHealth(void){

	bool isTemperatureHealthy = false;
	/////////////////////////////////////////////////////////////
	///////////// CHECK FOR UPDATE
	if(thermister_updated_){
		temp_health_cntr_ 		= 0;
		thermister_updated_ 	= false;
		isTemperatureHealthy 	= true;

		for(int j=0; j<NUM_THERMISTERS; j++){
			state.temperatures[j] = thermisters_in.tempF[j];
		}

		if(true == send_temperature_health_warning_){
			send_temperature_health_warning_ = false;
			// send something to ground saying temp data back
			// TBD
		}

	}else{
		temp_health_cntr_++;
	}
	///////////////////////////////////////////////////////////////

	if(temp_health_cntr_ > WARN_TEMP_COUNT && send_temperature_health_warning_ == false){
		send_temperature_health_warning_ = true;
		// send something to ground saying temp data gone
		// TBD
	}

	return isTemperatureHealthy;

}

