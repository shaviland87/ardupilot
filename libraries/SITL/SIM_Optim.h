/*
  simulator connection for ardupilot version of Optim
*/

#pragma once

#include "SIM_Aircraft.h"
#include <AP_HAL/utility/Socket.h>
////////////////////////////
#include "checksum.h"
#include "serial.h"
#include "ether2.h"
//////////////////////////////////////////////////////////////////
#define USE_PSEUDO_INV          false

#include <AP_OA_EXT/optimAero.h>


namespace SITL {

/*
  Optim simulator
 */
class Optim : public Aircraft {
public:
    Optim(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Optim(frame_str);
    }

    /*  Create and set in/out socket for Optim simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:
    /*
      packet sent to Optim
     */
    struct servo_packet {
      // size matches sitl_input upstream
      float motor_speed[16];
    };

    /*
      reply packet sent from Optim to ArduPilot
     */
    struct fdm_packet {
      double timestamp;  // in seconds
      double imu_angular_velocity_rpy[3];
      double imu_linear_acceleration_xyz[3];
      double imu_orientation_quat[4];
      double velocity_xyz[3];
      double position_xyz[3];
      double airspeed;
    };

    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);
    void drain_sockets();

    double last_timestamp;

    SocketAPM socket_sitl;
    const char *_Optim_address = "127.0.0.1";
    int _Optim_port = 9002;
    static const uint64_t Optim_TIMEOUT_US = 5000000;

    struct serialPort_ref usb_out_;
    struct serialPort_ref serial;
    struct quadArduMessageHeader_ref header;

        
    uint32_t send_report_last_ms;
    void update_slow_hz(void);
    void check_oa_sim_ports(void);
    AP_Int8 typeOA;
    optimAero::ardu_cells current_cell_;

    bool sendSim_serial;
    void populateBufferOut();
    void write2Buffer(uint8_t* ptr, uint32_t len);
    OA_Parser data2send_;
    unsigned char buffer_counter;
    uint32_t first_sent_sim_buffer; 
    MsgBattCell_ref battery_data;
    MsgThermistersData_ref thermister_data;
    MsgAnalogData_ref analog_data;
    rc_repeater_heartbeat_ref external_hrt_beat_;
    unsigned char hrt_cntr_; 
    bool send_once;
    unsigned int increment_arm_disconnect_;
    bool SEND_ARDUINO_;


};

}  // namespace SITL
