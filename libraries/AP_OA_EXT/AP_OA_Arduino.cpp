#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_OA_Arduino.h"
#include <GCS_MAVLink/GCS.h> 					/*to send data to GCS */

#if 0
#include <stdio.h> //for sim TBD
#endif

extern const AP_HAL::HAL& hal;

/* const inits the oa-units not called till detect() returns true*/

/* TBD -> (1) functions to monitor health of datastream for temp/analog/battery..if any stop updating we want to know  */

AP_OA_Arduino::AP_OA_Arduino(optimAero::optimAero_State &_state,
				AP_OA_Params &_params,
				uint8_t serial_instance) :
	AP_OA_Backend(_state,_params)
{

	const AP_SerialManager &serial_manager = AP::serialmanager();
	uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_optimAero, serial_instance);

	if(uart != nullptr){
		uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_optimAero, serial_instance));
	} //tbd might want this a param

	parse_params_.bytesInBuffer = 0; //make sure it knows its zero
	
	/*updates these if we have recvd them from arduino*/
	battery_updated_ 		= false;
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

}

/*detect if connected*/

bool AP_OA_Arduino::detect(uint8_t serial_instance)
{
	return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_optimAero, serial_instance) != nullptr;
}

void AP_OA_Arduino::updateFast(void)
{
	//uart->write("Hello\n");
	checkForData();
}

void AP_OA_Arduino::update10Hz(void){

	if(params.function | MASK_ARDUINO_ARMS){

		if(updateAnalogHealth()){
			checkArmConnects(); 	//only worth checking if data is healthy...
		}
	}

}
void AP_OA_Arduino::updateSlow(void){
	
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

}

void AP_OA_Arduino::populateBuffer(OA_Parser &parser_in){
	/*sim function only*/

	if(parser_in.bytesInBuffer > 0 && parse_params_.bytesInBuffer < OA_PARSER_MAX_BUFFER_SIZE)
	{
		// populate a buffer if data avail and buffer container size is not full

		for(int i=0; i<parser_in.bytesInBuffer; i++){
			parse_params_.datalinkBuffer[parse_params_.bytesInBuffer] = parser_in.datalinkBuffer[i];
			#if 0
				//printf("byte[%d] = %02X \n",parse_params_.bytesInBuffer,parse_params_.datalinkBuffer[parse_params_.bytesInBuffer]);
			#endif
			parse_params_.bytesInBuffer++;
		}
		
		memset (parser_in.datalinkBuffer,0,parser_in.bytesInBuffer);
		parser_in.bytesInBuffer = 0;
	}
}


void AP_OA_Arduino::checkForData(void){

	if(nullptr != uart ){
		// uart setup good to go

		while(uart->available() > 0 && parse_params_.bytesInBuffer < OA_PARSER_MAX_BUFFER_SIZE)
		{
			// populate a buffer if data avail and buffer container size is not full
			parse_params_.datalinkBuffer[parse_params_.bytesInBuffer] = uart->read();
			// hal.uartE->printf("byte[%d] = %02X ,",parse_params_.bytesInBuffer,parse_params_.datalinkBuffer[parse_params_.bytesInBuffer]);
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
				#if 0
					//printf("sync, id(%d),size(%d) \n",parse_params_.readID,parse_params_.readSize);
				#endif

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
							//printf("csum\n");
							
							switch (parse_params_.readID)
							{
								case ID_MESSAGE_BATT_CELL:
									memcpy(&battery_in_, parse_params_.bf, sizeof(MsgBattCell_ref));
									battery_updated_ = true;
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
								}break;
							}

							//packetsRecvd

						}else
						{
							parse_params_.badChecksums++;
							state.badChecksums = parse_params_.badChecksums;
							//bad checksum!!
							#if 0
								printf("bad ck\n");
							#endif
						}
						
						iData += parse_params_.readSize; //move forward by readSize
					}else
					{
						//half pkt read -- move on -- as more bytes come in we will eventaully do a cksum and throw out those bytes
						#if 0
							printf("half pkt %d\n ", iData);
						#endif
						// if this happens and on next iteration move the buffer certain amt - move forward by header size
						dataDone = 1;
					}

				}else
				{ 
					#if 0
						printf("invalide msg size\n");
					#endif
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
		//printf("memmove%d\n",iData);

	}
	

	}//end of nullptr = uart

}

void AP_OA_Arduino::checkTemperatures(void){
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

void AP_OA_Arduino::checkArmConnects(void){
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

bool AP_OA_Arduino::updateBatteryHealth(void){
	/*these only get called if the param_function is set to want battery data*/
	
	bool isBatteryHealthy = false; //output of function

	/////////////////////////////////////////////////////////////
	///////////// CHECK FOR UPDATE
	if(battery_updated_){
		battery_health_cntr_ 	= 0;
		battery_updated_ 		= false;
		isBatteryHealthy 		= true;

		//populate state of battery
		for(int j=0; j<NUMBATT*NUMCELL; j++){
			state.batt_cells.cells[j] = battery_in_.cell[j];
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

bool AP_OA_Arduino::updateAnalogHealth(void){

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

bool AP_OA_Arduino::updateTemperatureHealth(void){

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