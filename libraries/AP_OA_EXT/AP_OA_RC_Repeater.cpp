#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_OA_RC_Repeater.h"
#include <GCS_MAVLink/GCS.h> 					/*to send data to GCS */

#if 0
#include <stdio.h>
#endif
/*
    // initialise pointers to serial ports
    state[1].uart = hal.uartC;  // serial1, uartC, normally telem1
    state[2].uart = hal.uartD;  // serial2, uartD, normally telem2
    state[3].uart = hal.uartB;  // serial3, uartB, normally 1st GPS
    state[4].uart = hal.uartE;  // serial4, uartE, normally 2nd GPS
    state[5].uart = hal.uartF;  // serial5
    state[6].uart = hal.uartG;  // serial6
    state[7].uart = hal.uartH;  // serial7*/
	//        SerialProtocol_optimAero = 24,              //custom optimAero


extern const AP_HAL::HAL& hal;

/* const inits the oa-units not called till detect() returns true*/

AP_OA_RC_Repeater::AP_OA_RC_Repeater(optimAero::optimAero_State &_state,
				AP_OA_Params &_params,
				uint8_t serial_instance) :
	AP_OA_Backend(_state,_params)
{

	const AP_SerialManager &serial_manager = AP::serialmanager();
	uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_optimAero, serial_instance);

	if(uart != nullptr){
		uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_optimAero, serial_instance));
	} //tbd might want this a param

	motor_init = false;
	if(nullptr != vehicle_motors ){
		vehicle_motors 	= AP_Motors::get_singleton();
		motor_init 		= true;
	}

	parse_params_.bytesInBuffer = 0; //make sure it knows its zero
	hrt_beat_rcvd_ 				= 0; //number of hrt beats recevd since last check
	missed_hearts_ 				= 0;

}

/*detect if connected*/

bool AP_OA_RC_Repeater::detect(uint8_t serial_instance)
{
	return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_optimAero, serial_instance) != nullptr;
}

void AP_OA_RC_Repeater::updateFast(void)
{
	/* main function that gets called by ArduCopter at some rate*/
		
	/* need something to listen for rc repeater heartbeat */
	/* need something to send commands to rc repeater heartbeat */
	/* something to log rc repeater state */
	
	//uart->write("Hello\n");
	checkForData();
}

void AP_OA_RC_Repeater::update10Hz(void){

	motor_out_.sync 		= OPTIM_MESSAGE_SYNC;
	motor_out_.messageID 	= ID_EXTERNAL_AP_RC_OUT;
	motor_out_.messageSize 	= sizeof(external_ap_rc_out_ref);
	motor_out_.count++;
	motor_out_.csum 		= parse_params_.fletcher16((unsigned char *)&motor_out_, motor_out_.messageSize - OPTIM_MESSAGE_CSUM_SIZE);

	if(nullptr != uart){
		parse_params_.optim_serialWrite(uart, (uint8_t *)&motor_out_, motor_out_.messageSize);
	}
}
void AP_OA_RC_Repeater::updateSlow(void){

	if(hrt_beat_rcvd_ < HRT_RATE_HZ*CHECK_HEART_RATE_HZ){
		missed_hearts_++;
	}else{
		missed_hearts_ = 0;
	}

	if(missed_hearts_ > MISSED_HEARTS_LIMIT){
		gcs().send_text(MAV_SEVERITY_WARNING,"EXT RC DOWN");
	}
}
void AP_OA_RC_Repeater::populateBuffer(OA_Parser &parser_in){
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


void AP_OA_RC_Repeater::getRcOut(void){

	for(int i=0;i<EXTERNAL_AP_NUM_MOT; i++){
		motor_out_.rc[i] = 	hal.rcout->read(i);
	}
	motor_out_.motor_enable = hal.util->safety_switch_state(); //read state of switch
}

void AP_OA_RC_Repeater::checkForData(void){

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
							//hal.uartE->printf("bad ck\n");

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

