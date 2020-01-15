
/*
  simulator connector for ardupilot optimAero version of Optim
*/

#include "SIM_Optim.h"

#include <stdio.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

Optim::Optim(const char *frame_str) :
    Aircraft(frame_str),
    last_timestamp(0),
    socket_sitl{true}
{
    fprintf(stdout, "Starting SITL Optim\n");

    fprintf(stdout, "starting optimAero communications\n");
    serial.dataSource           = 1;
    serial.useOptimAeroComms    = 1;
    serial.connectTo            = (char *)"127.0.1.1";
    serial.useSock              = 1;
    serial.useTcp               = 0;
    serial.portNum              = 6003;
    serial.setRemotePort        = 1; //set the remote port instead of having it assigned
    serial.remotePortRequest    = 6002;
    serial.init                 = 1;

    fprintf(stdout, "starting usb output comms\n");
    usb_out_.dataSource         = 1;
    usb_out_.useOptimAeroComms  = 0;
    usb_out_.useSerial          = 1;
    usb_out_.nameFmt            = "/dev/ttyUSB%d";
    usb_out_.name               = "/dev/ttyUSB1";
    usb_out_.baud               = 460800;   //we are gonna need to suppport 460800/230400
    usb_out_.useTcp             = 0;
    usb_out_.useSock            = 0;
    usb_out_.port               = 2;
    usb_out_.termios            = "8n1\0";
    usb_out_.init               = 0;
    fprintf(stdout, "end usb output comms\n");


    for(int i=0;i<8; i++){
        current_cell_.cells[i] = 4000; //millivolts
    }
    
    sendSim_serial          = false;
    buffer_counter          = 0;
    first_sent_sim_buffer   = 0; //a value to track our first sent buffer to simulated ardu

    int8_t gerp = MASK_ARDUINO_BATT | MASK_ARDUINO_ARMS | MASK_ARDUINO_TEMPERATURE ; // 7
    printf("gerp is %d \n",gerp);
    gerp = MASK_ARDUINO_BATT | MASK_ARDUINO_ARMS  ; // 3 
    //printf("gerp is %d \n",gerp);
    gerp = MASK_ARDUINO_BATT   ; // 1
    //printf("gerp is %d \n",gerp);
    gerp =  MASK_ARDUINO_ARMS  ; // 2
    //printf("gerp is %d\n",gerp);

    send_once                   = true;
    increment_arm_disconnect_   = 0;
    SEND_ARDUINO_               = true;
    hrt_cntr_                   = 0;

    battery_data.cell[0]    = 3300;
    battery_data.cell[1]    = 1300;
    battery_data.cell[2]    = 2300;
    battery_data.cell[3]    = 3300;
    battery_data.cell[4]    = 3300;
    battery_data.cell[5]    = 3300;
    battery_data.cell[6]    = 3300;
    battery_data.cell[7]    = 3300;
    battery_data.cell[8]    = 3300;
    battery_data.cell[9]    = 3300;

    battery_data.cell[10]    = 300;
    battery_data.cell[11]    = 3300;
    battery_data.cell[12]    = 3300;
    battery_data.cell[13]    = 3300;
    battery_data.cell[14]    = 3300;
    battery_data.cell[15]    = 3300;
    battery_data.cell[16]    = 3300;
    battery_data.cell[17]    = 3300;
    battery_data.cell[18]    = 3300;
    battery_data.cell[19]    = 3300;

    battery_data.cell[20]    = 800;
    battery_data.cell[21]    = 3300;
    battery_data.cell[22]    = 3300;
    battery_data.cell[23]    = 3300;
    battery_data.cell[24]    = 3300;
    battery_data.cell[25]    = 3300;
    battery_data.cell[26]    = 3300;
    battery_data.cell[27]    = 3300;
    battery_data.cell[28]    = 3300;


}

/*
  Create and set in/out socket
*/
void Optim::set_interface_ports(const char* address, const int port_in, const int port_out)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // Optim keeps sending us packets. Not strictly necessary but
    // useful for debugging
    if (!socket_sitl.bind("0.0.0.0", port_in)) {
        fprintf(stderr, "SITL: socket in bind failed on sim in : %d  - %s\n", port_in, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    printf("Bind %s:%d for SITL in\n", "127.0.0.1", port_in);
    socket_sitl.reuseaddress();
    socket_sitl.set_blocking(false);

    _Optim_address = address;
    _Optim_port = port_out;
    printf("Setting Optim interface to %s:%d \n", _Optim_address, _Optim_port);
}

/*
  decode and send servos
*/
void Optim::send_servos(const struct sitl_input &input)
{
    servo_packet pkt;
    // should rename servo_command
    // 16 because struct sitl_input.servos is 16 large in SIM_Aircraft.h
    for (unsigned i = 0; i < 16; ++i)
    {
      pkt.motor_speed[i] = (input.servos[i]-1000) / 1000.0f;
    }
    socket_sitl.sendto(&pkt, sizeof(pkt), _Optim_address, _Optim_port);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void Optim::recv_fdm(const struct sitl_input &input)
{
    fdm_packet pkt;

    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    while (socket_sitl.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        send_servos(input);
        // Reset the timestamp after a long disconnection, also catch Optim reset
        if (get_wall_time_us() > last_wall_time_us + Optim_TIMEOUT_US) {
            last_timestamp = 0;
        }
    }

    const double deltat = pkt.timestamp - last_timestamp;  // in seconds
    if (deltat < 0) {  // don't use old packet
        time_now_us += 1;
        return;
    }
    // get imu stuff
    accel_body = Vector3f(static_cast<float>(pkt.imu_linear_acceleration_xyz[0]),
                          static_cast<float>(pkt.imu_linear_acceleration_xyz[1]),
                          static_cast<float>(pkt.imu_linear_acceleration_xyz[2]));

    gyro = Vector3f(static_cast<float>(pkt.imu_angular_velocity_rpy[0]),
                    static_cast<float>(pkt.imu_angular_velocity_rpy[1]),
                    static_cast<float>(pkt.imu_angular_velocity_rpy[2]));

    // compute dcm from imu orientation
    Quaternion quat(static_cast<float>(pkt.imu_orientation_quat[0]),
                    static_cast<float>(pkt.imu_orientation_quat[1]),
                    static_cast<float>(pkt.imu_orientation_quat[2]),
                    static_cast<float>(pkt.imu_orientation_quat[3]));
    quat.rotation_matrix(dcm);

    velocity_ef = Vector3f(static_cast<float>(pkt.velocity_xyz[0]),
                           static_cast<float>(pkt.velocity_xyz[1]),
                           static_cast<float>(pkt.velocity_xyz[2]));

    position = Vector3f(static_cast<float>(pkt.position_xyz[0]),
                        static_cast<float>(pkt.position_xyz[1]),
                        static_cast<float>(pkt.position_xyz[2]));

    airspeed = static_cast<float>(pkt.airspeed); 
    /*not sure what difference is between airspeed and airspeed_pitot */
    airspeed_pitot = airspeed;


    // auto-adjust to simulation frame rate
    time_now_us += static_cast<uint64_t>(deltat * 1.0e6);

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(static_cast<float>(1.0/deltat));
    }
    last_timestamp = pkt.timestamp;

}

/*
  Drain remaining data on the socket to prevent phase lag.
 */
void Optim::drain_sockets()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    errno = 0;
    do {
        received = socket_sitl.recv(buf, buflen, 0);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
                fprintf(stderr, "error recv on socket in: %s \n",
                        strerror(errno));
            }
        } else {
            // fprintf(stderr, "received from control socket: %s\n", buf);
        }
    } while (received > 0);

}

/*
  update the Optim simulation by one time step
 */
void Optim::update(const struct sitl_input &input)
{
    send_servos(input);
    recv_fdm(input);
    update_position();

    time_advance();
    // update magnetic field
    update_mag_field_bf();
    drain_sockets();
    update_slow_hz();
}

void Optim::update_slow_hz(void){

        const uint32_t now = AP_HAL::millis();
    #if defined(__CYGWIN__) || defined(__CYGWIN64__)
        if (now < 10000) {
            // don't send lidar reports until 10s after startup. This
            // avoids a windows threading issue with non-blocking sockets
            // and the initial wait on uartA
            return;
        }
    #endif


    // send a  messages at 1  Hz
    if (now - send_report_last_ms >= (1000/1) ) {
        send_report_last_ms = now;

        optimAero *oa_ = AP::oa();

        for(uint8_t i = 0; i< oa_->num_oa_connections(); i++)
        {
            typeOA = oa_->get_type(i);
            if(typeOA == optimAero::optimAero_TYPE_ARDUINO || typeOA == optimAero::optimAero_TYPE_MULTI)
            {

                #if 0
                //we could potentially have battery data
                //optimAero::ardu_cells curr_cells = oa_->get_batt_cells();
                // cheap way of setting battery
                current_cell_.cells[0] += 1;
                if(current_cell_.cells[0]> 4400){
                    current_cell_.cells[0] = 3300;
                }
                oa_->set_batt_cells(current_cell_);
                #endif


                if(first_sent_sim_buffer == 0 && now > 9000){//15000
                    first_sent_sim_buffer   = now;
                    sendSim_serial          = true;
                    printf("sending oa data now == %d\n",now);
                }

                if(sendSim_serial){
                    
                    //TBD GET TIME to only stop sending during a given interval
                    //if( (now - first_sent_sim_buffer) <= (10000) || (now - first_sent_sim_buffer) >= (100000) )
                    //{

                        if(SEND_ARDUINO_)
                        {
                            populateBufferOut();
                            oa_->sendBuffer(data2send_);
                        }
                    //}

                }


            }else if(typeOA == optimAero::optimAero_TYPE_EXTRC)
            {
                if(!SEND_ARDUINO_)
                {   
                    hrt_cntr_++;
                    if(hrt_cntr_>=10){
                        populateBufferOut();
                        oa_->sendBuffer(data2send_);
                        hrt_cntr_ = 0;
                        printf("send hrt\n");
                    }
                }
            }else{/*nothing*/}
        }

    }

}
void Optim:: write2Buffer(uint8_t* ptr, uint32_t len){

    #if 0
    uint32_t i;
    for(i=0; i<len;i++){
        data2send_.datalinkBuffer[i] = ptr[i];
    }
    data2send_.bytesInBuffer = len;
    //printf("len is %d\n",data2send_.bytesInBuffer);
    #endif

    uint32_t i;
    for(i=0; i<len;i++){
        if(data2send_.bytesInBuffer < DATALINK_BUFFER_SIZE){
            data2send_.datalinkBuffer[data2send_.bytesInBuffer++] = ptr[i];
        }else{
            printf("buff full\n");
        }
    }
}

void Optim::populateBufferOut(){
    
    // ONLY SEND ONE BUFFER TYPE OUT AT A TIME__AKA not arduino and ext RC together as module aint smart enough to split
    if(SEND_ARDUINO_)
    {
        /*battery data*/
        battery_data.sync           = OPTIM_MESSAGE_SYNC;
        battery_data.messageID      = ID_MESSAGE_BATT_CELL;
        battery_data.messageSize    = sizeof( MsgBattCell_ref);
        battery_data.count          = buffer_counter++;
        /*for(int i=0;i<NUMCELL*NUMBATT;i++){
            battery_data.cell[i]    = 3300;
        } */   
        
        //battery_data.cell[0]        = 3300 + increment_arm_disconnect_;
        battery_data.align          = 0;
        battery_data.csum = data2send_.fletcher16((uint8_t *)&battery_data, battery_data.messageSize-OPTIM_MESSAGE_CSUM_SIZE);
        //now batery data populated..put into datalinkbuffer
        write2Buffer((uint8_t *)&battery_data, battery_data.messageSize);
        printf("sent battery\n");

        /*analog data*/
        increment_arm_disconnect_++;

        analog_data.sync            = OPTIM_MESSAGE_SYNC;
        analog_data.messageID       = ID_MESSAGE_ANALOG_DATA;
        analog_data.messageSize     = sizeof(MsgAnalogData_ref);
        analog_data.count           = buffer_counter;

        if(increment_arm_disconnect_ > 100){

            for(int i=0; i<MAX_ANALOG_CAPACITY;i++){
                analog_data.data[i]     = ARDUINO_ARM_CONNECTED;
            }
            analog_data.data[2]         = ARDUINO_ARM_DISCONNECTED;
            if(increment_arm_disconnect_ > 200){
                increment_arm_disconnect_   = 0;
                printf("arm disconnect\n");
            }

        }else{

            for(int i=0; i<MAX_ANALOG_CAPACITY;i++){
                analog_data.data[i]     = ARDUINO_ARM_CONNECTED;
            }
        }

        analog_data.csum = data2send_.fletcher16((uint8_t *)&analog_data, analog_data.messageSize-OPTIM_MESSAGE_CSUM_SIZE);
        write2Buffer((uint8_t *)&analog_data, analog_data.messageSize);
    }
    else
    {
        external_hrt_beat_.sync         = OPTIM_MESSAGE_SYNC;
        external_hrt_beat_.messageID    = ID_REPEATER_HEARTBEAT;
        external_hrt_beat_.messageSize  = sizeof(rc_repeater_heartbeat_ref);
        external_hrt_beat_.count++;
        external_hrt_beat_.armStatus    = 0;
        external_hrt_beat_.badChecksum  = 0;
        external_hrt_beat_.missedAP     = 0;
        external_hrt_beat_.csum         = data2send_.fletcher16((uint8_t *)&external_hrt_beat_, external_hrt_beat_.messageSize-OPTIM_MESSAGE_CSUM_SIZE);
        write2Buffer((uint8_t *)&external_hrt_beat_,external_hrt_beat_.messageSize);
    }
  
    
    
}

}  // namespace SITL
