

#ifndef CHECKSUM_H_
#define CHECKSUM_H_

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

//these are added for rand
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------
#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define NTAB 32
#define NDIV (1+(IM-1)/NTAB)
#define EPS 1.2e-7
#define RNMX (1.0 - EPS)


#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------
double rande( void );
double randne( void );


unsigned int fletcher16( unsigned char *data, int len );
unsigned int datalinkCheckSumCompute( unsigned char *buf, int byteCount );
void datalinkCheckSumEncode( unsigned char *buf, int byteCount );

#define DATALINK_SYNC1 0xa3
#define DATALINK_SYNC2 0xb2
#define DATALINK_SYNC3 0xc1


#define DJI_MESSAGE_HDR_SIZE 3
#define DJI_MESSAGE_MAX_SIZE 60
#define DJI_MESSAGE_CSUM_SIZE 2
#define DJI_MESSAGE_SYNC 0xa1

#define OPTIM_MESSAGE_SYNC 0xa1
#define OPTIM_MESSAGE_CSUM_SIZE 2
#define SAS_MESSAGE_HDR_SIZE 3
#define OPTIM_MESSAGE_HDR_SIZE 3

#define DATALINK_BUFFER_SIZE 256

#define OPTIM_MESSAGE_MAX_SIZE 120 //updated for MsgBattCell
#define SAS_MESSAGE_MAX_SIZE 60

struct quadArduMessageHeader_ref {
  unsigned char sync     ;
  unsigned char messageID   ;
  unsigned char messageSize ;
};

struct datalinkHeader_ref{
    unsigned char sync1; //= DATALINK_SYNC0;
    unsigned char sync2; //= DATALINK_SYNC1;
    unsigned char sync3; //= DATALINK_SYNC2;
    unsigned char spare; //= 0;
    int messageID;
    int messageSize;
    unsigned int hcsum;
    unsigned int csum;
};


#define OPTIM_AERO_MESSAGE_FDM 111
struct optimAeroMessageFDM_ref { //if we use 8 byte(double) needs to be 8 byte aligned
    unsigned char sync;
    unsigned char messageID ;
    unsigned char messageSize;
    unsigned char count ;
    unsigned int align1;

    double timestamp;
    float imuAngularVelocityRPY[3];    /// \brief IMU angular velocity
    float imuLinearAccelerationXYZ[3]; /// \brief IMU linear acceleration
    float imuOrientationQuat[4];       /// \brief IMU quaternion orientation
    float velocityXYZ[3];              /// \brief Model velocity in NED frame
    float positionXYZ[3];              /// \brief Model position in NED frame

    unsigned int align2;
    unsigned short align ;
    unsigned short csum ;

};



#define OPTIM_AERO_MOTOR_CMD 112
struct optimAeroMotorCmd_ref {
  unsigned char sync;
  unsigned char messageID;
  unsigned char messageSize;
  unsigned char count;

  unsigned short pwm[16];
  unsigned short csum;
};
 


#define SAS_MESSAGE_GAIN_SET 114
#define SAS_MESSAGE_GAIN_GET 115
struct sasMsgGain_ref {
  unsigned char sync;
  unsigned char messageID;
  unsigned char messageSize;
  unsigned char count;

  float rollGainMan;
  float pitchGainMan;
  float yawGainMan;
  float rollGainAuto;
  float pitchGainAuto;
  float yawGainAuto;

  unsigned short align;
  unsigned short csum;
};


#define QUAD_ARDU_MESSAGE_GPS 117
struct quadArduMessageGps_ref {
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;

    unsigned char gpsInstance  ;//which gps did data come from ;
    unsigned char numSats  ;    //number of satellites ;
    unsigned char status ;
    unsigned char align  ;
    int   position_LLA[3];      // (rad*10^7, rad*10^7, cm) I think;
    float velocity[3]  ;        // (m/s) NED ;
    float speedAccuracy  ;
    float horizontalAccuracy;
    float verticalAccuracy;

    unsigned short align2;
    unsigned short csum ;
};


#define QUAD_ARDU_MESSAGE_STATUS 118
struct quadArduMessageStatus_ref {
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;

    unsigned int itime_gust_rate_feedback   ;
    unsigned int itime_gust_accumulate_imu  ;
    unsigned int num_unhealthy_accel        ;
    unsigned int num_unhealthy_gyro         ;
    unsigned int num_commands_received      ;

    unsigned short csum;
};


#define QUAD_ARDU_MESSAGE_COMPASS 119
    struct quadArduMessageCompass_ref {
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;
    float field[3]  ;//mag field;

    unsigned short align;
    unsigned short csum;
}; //[4,4,4,4,2,2] -packed good

#define QUAD_ARDU_MESSAGE_BARO 120
struct quadArduMessageBaro_ref {
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;
    float pressure   ;//(pascals) static pressure;
    float temperature  ;// (C) temperature;

    unsigned short align;
    unsigned short csum ;
};


#define QUAD_ARDU_MESSAGE_COMPASS2 125
    struct quadArduMessageCompass2_ref {
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;
    float field[3]  ;//mag field;

    unsigned short align;
    unsigned short csum;
}; //[4,4,4,4,2,2] -packed good

// optim setting message is sent to/back from ROS - only when ROS sends it - ROS will send based on a click button
#define OPTIM_SETTINGS 35
struct optimSettings_ref {
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;

    unsigned char use_f_damping;
    unsigned char use_m_damping;
    unsigned char use_wind;
    unsigned char reset_vehicle;

    unsigned char reset_pid;
    unsigned char gerp[3]; //for alignment

    float body_f_damping[3]; // body coord damping terms on accel
    float body_m_damping[3]; // body coord damping terms on ang.accel
    float fuse_inertia[3];   // body inertia of fuselage kg-m2
    float fuse_mass;         // mass of fuse in kg
    float prop_z_inertia;    // propeller z-inertia
    float prop_mass;         // propeller mass
    float wing_area;         // area of wing in m2
    float motor_Tc_up;       // motor time constant speed up
    float motor_Tc_down;     // motor time const down
    float maxOmega;          // max angular velo of rotor
    float rotorSlowdown;     // rotor slowdown factor
    float rotorDragCoeff;    // rotor drag coeff
    float rotorRollingCoeff; // rotor rolling moment
    float motorConstant;
    float momentConstant;
    float motorConstantKF;
    float pid[5];            // kp,ki,kd,imax,imin
    float torque_max;        // torque max

    float wind_mag;
    float wind_var;

    unsigned short align;
    unsigned short csum;

};

#define OPTIM_VEHICLE 36
// optim vehicle message is for sending debug data to ros for viewing/plotting/saving
// should this message be configurable to send at different rates? thinking std 100hz or 50Hz
struct optimVehicle_ref{
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;

    float ardu_time;            // useful for taking log data from ardupilot and aligning with ros logs
    float body_vel[3];          // velocity of body in m/s
    float body_accel[3];        // accel of body in m/s/s
    float body_ang_vel[3];      // ang velo of body deg/s
    float body_ang_acc[3];      // ang accel of body deg/s/s
    float sampling_time_;       // gazebo sampling time

    unsigned short align;
    unsigned short csum;

};


#define OPTIM_ROTOR 37
struct optimRotor_ref{
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;

    float rotor_velo[9];        // rotors velocity rad/s
    float rotor_acc[9];         // rotors ang accel rad/s/s

    unsigned short align;
    unsigned short csum;

};

#define OPTIM_DEBUG 38
struct optimDebug_ref{
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;

    float debug_data[16];

    unsigned short align;
    unsigned short csum;

};

#define OPTIM_MOTOR_CMD 39
struct optimMotorCmd_ref{
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;

    float motor_cmd[4]; // roll,pitch,yaw, throttle commands that are sent to motor mixing

    unsigned short align;
    unsigned short csum;

};

#define OPTIM_PID_OUT 40
struct optimPIDout_ref{
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;

    float pid_accel_z[4]; //ff, p,i,d
    float p_z[2]; // proportional on z-pos,vel-z
    float p_xy;
    float pid_xy[6]; //px,ix,dx,py,iy,dy
    float pid_roll[4];
    float pid_pitch[4];
    float pid_yaw[4];

    unsigned short align;
    unsigned short csum;

};

#define OPTIM_ATTITUDE_TARGET 144
struct optimAttTarget_ref{
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;

    float att[3];
    float att_des[3];

    unsigned short align;
    unsigned short csum;
};

#define OPTIM_RPM_MESSAGE 65
struct optimRPMMessage_ref {
	unsigned char sync;
	unsigned char messageID;
	unsigned char messageSize;
	unsigned char count;
   

	float rpm;  // range is 0 to 65535

    unsigned short gerp;
	unsigned short csum;
};

#define OPTIM_VEHICLE_DATA2 168
struct optimVehicleData2_ref{
    unsigned char sync;
    unsigned char messageID;
    unsigned char messageSize;
    unsigned char count;

    float posNWU[3];
    float attitude[3];
    float windNWU[3];

    unsigned short align;
    unsigned short csum;
};

#define OPTIM_VEHICLE_DATA3 183
struct optimVehicleData3_ref {  
    unsigned char sync;
    unsigned char messageID ;
    unsigned char messageSize;
    unsigned char count ;

    float pos[3];
    float velo[3];                      /// \brief vel ned orientation
    float gyro[3];                      /// \brief IMU linear acceleration
    float accel[3];                      /// \brief IMU angular velocity

    unsigned short align ;
    unsigned short csum ;

};



#define OPTIM_DATALINK_HEALTH 66
struct optimDatalinkHealth_ref{
    unsigned char   sync;
    unsigned char   messageID;
    unsigned char   messageSize;
    unsigned char   count;
    
    unsigned short  badChecksums; 
    unsigned short  unknownIDs;
    unsigned short  halfPackets;
    unsigned short  invalidMessageSize;
    unsigned int    packetsRecv;

    unsigned short  unknownESCID;
    unsigned short  csum;

};


#define OPTIM_TOYO_RPM 67
struct optimToyoRPM_ref{
      unsigned char   delimiter;      //  0x55
      unsigned char   messageID;      //  0x01
      unsigned char   messageSize;    //
      unsigned char   count;

      float rpm[8];

      unsigned short gerp;
      unsigned char align;
      unsigned char csum;
};


#define OPTIM_TOYO_SYNC_SEND        0x55 
#define OPTIM_TOYO_SYNC_RECV        0x55
#define OPTIM_TOYO_CRC_ERR          0x01  // LSB = 1  ->0x01 or 0000 0001 = dec 1
#define OPTIM_TOYO_MOTOR_ENABLE     0x01  // LSB = 1  = 0x01 or 0000 0001 = dec 1
#define OPTIM_TOYO_HDR_SIZE         4     // delimeter, id, size ,count

#define OPTIM_TOYO_CSUM_SIZE        1     // using crc8
#define OPTIM_TOYO_MAX_MSG_SIZE     140
#define OPTIM_TOYO_WARNING_SEC      3     //every X seconds check for warnings
// MOTOR 1
#define OPTIM_TOYO_ESC_MOTOR_1A     0x1A
#define OPTIM_TOYO_ESC_MOTOR_1B     0x1B
// MOTOR 2
#define OPTIM_TOYO_ESC_MOTOR_2A     0x2A
#define OPTIM_TOYO_ESC_MOTOR_2B     0x2B
// MOTOR 3
#define OPTIM_TOYO_ESC_MOTOR_3A     0x3A
#define OPTIM_TOYO_ESC_MOTOR_3B     0x3B
// MOTOR 4
#define OPTIM_TOYO_ESC_MOTOR_4A     0x4A
#define OPTIM_TOYO_ESC_MOTOR_4B     0x4B
// MOTOR 5
#define OPTIM_TOYO_ESC_MOTOR_5A     0x5A
#define OPTIM_TOYO_ESC_MOTOR_5B     0x5B
// MOTOR 6
#define OPTIM_TOYO_ESC_MOTOR_6A     0x6A
#define OPTIM_TOYO_ESC_MOTOR_6B     0x6B
// MOTOR 7
#define OPTIM_TOYO_ESC_MOTOR_7A     0x7A
#define OPTIM_TOYO_ESC_MOTOR_7B     0x7B
// MOTOR 8
#define OPTIM_TOYO_ESC_MOTOR_8A     0x8A
#define OPTIM_TOYO_ESC_MOTOR_8B     0x8B
// MOTOR 9
#define OPTIM_TOYO_ESC_MOTOR_9A     0x9A
#define OPTIM_TOYO_ESC_MOTOR_9B     0x9B

// MOTOR 10
#define OPTIM_TOYO_ESC_MOTOR_10A     0x1C
#define OPTIM_TOYO_ESC_MOTOR_10B     0x1D

#define OPTIM_TOYO_MAX_MOTOR_CMDS 9
#define OPTIM_TOYO_SWAN_CFG 5
#define OPTIM_BRAIN_RC_MAX  1965
#define OPTIM_BRAIN_RC_MIN   85
#define OPTIM_BRAIN_RC_RANGE 1880
#define OPTIM_BRAIN_RC_SCALE 0.5319f

#define GND_MOTOR_STOP  0x01
#define SBUS_REC_STOP   0x02
#define SBUS_TEL_STOP   0x04
#define GROUND_LOST     0x08
#define MB_PIX_STOP     0x20
#define PIX_CB_STOP     0x40
#define GND_CB_STOP     0x80
#define ENABLE_TOYO_MTR 0x01

/* TBD - NOTE THESE IN ALL FUTURE FIRMWARE SENDS SO WE CAN TRACK THIS */

#define TOYO_19_VERSION_NOBATT 1 // eva version 02/14/18
#define TOYO_16_VERSION 1 // eva version 10/31/18
#define TOYO_17_VERSION 0 // version for new WIFI system
#define TOYO_EVA_ON 1   //if EVA NOT ON - swan on

#if TOYO_19_VERSION_NOBATT == 0
    #define OPTIM_TOYO_COMP_ID          4     // component id comes in at fifth byte
#else
    #define OPTIM_TOYO_COMP_ID          5     // component id comes in at 6th byte
#endif

/* //version 1.5
struct optimPix2Brain_ref{
   unsigned char    delimiter;                                 //  0x55 
   unsigned char    control_flag;                              //  MOTOR ENABLE (LSB 1 = 0x01)
   unsigned short   motor_cmds[OPTIM_TOYO_MAX_MOTOR_CMDS];     //  range 0 - 1000
   unsigned short   brake[2];                                  //  range 0 - 1000
   unsigned char    count;
   unsigned char    csum;
};*/

//version 1.6

 //this version was for eva
#if TOYO_16_VERSION == 1
    #define OPTIM_TOYO_PIX2BRAIN_MSG 0xBB
    struct optimPix2Brain_ref{
    unsigned char    delimiter;                                 //  0x55 
    unsigned char    deviceID;                                  //  0x01
    unsigned char    count;
    unsigned char    control_flag;                              //  MOTOR ENABLE (LSB 1 = 0x01)
    unsigned short   motor_cmds[OPTIM_TOYO_MAX_MOTOR_CMDS];     //  range 0 - 1000
    unsigned short   brake[2];                                  //  range 0 - 1000
    unsigned char    align;
    unsigned char    csum;
    };
#endif

#if TOYO_17_VERSION == 1
    #define OPTIM_TOYO_PIX2BRAIN_MSG 0xBB
    struct optimPix2Brain_ref{
    unsigned char    delimiter;                                 //  0x55 
    unsigned char    deviceID;                                  //  0x01
    unsigned char    count;
    unsigned char    control_flag;                        //  MOTOR ENABLE (LSB 1 = 0x01) (0x20 for MB-pixhawk timeout)
    unsigned short   motor_cmds[OPTIM_TOYO_SWAN_CFG];     //  range 0 - 1000
    unsigned char    align;
    unsigned char    csum;
    };
#endif

#define OPTIM_TOYO_BRAINRC_MSG 0x03
struct optimBrainRC_ref{
   unsigned char    delimiter;                                 //  0x55 
   unsigned char    deviceID;                                  //  0x01
   unsigned char    msgLen;
   unsigned char    count;
   unsigned char    headAlign;
   unsigned char    rc_flag; 
   // 0x01 - grd esc enable, 02 sbus rec fail, 04 - sbus rec lost link, 
   // 0x08 lost ground rcvr comms, 0x20 mb-pix comms timeout, 0x40 pix-cb comms timeout, 0x80 ground-cb timeout
   unsigned short   rc_cmds[16];      //values 0 2047                          //  range 0 - 1000
   unsigned char    align;
   unsigned char    csum;
};

#if TOYO_19_VERSION_NOBATT == 0
#define OPTIM_TOYO_MB2PIX_MSG  0x01
struct optimMB2PixMsg_Ref{
    unsigned char   delimiter;      //  0x55
    unsigned char   messageID;      //  0x01
    unsigned char   messageSize;    //  0x1E = 30 bytes
    unsigned char   count;

    unsigned char   escID;
    unsigned char   statusFlag;     // LSB = 1 if crc error on their side, or 0x80 for controller crc error
    unsigned char   controlByte;
    unsigned char   errorByte;
    unsigned char   throttleSetpoint;
    unsigned char   tempBatt;       //temp in celcius
    unsigned char   tempMotor;      //temp of motor
    unsigned char   tempMC;         //temp of microcontroller
    
    unsigned short  pwmMC;          // PWM microcontroller
    unsigned short  voltMC;         // microcontroller voltage (V)
    unsigned short  currBatt;       // battery current (A)
    unsigned short  rpmMotor;       // rpm of motor
    unsigned short  bmsStatus;      // BMS status word?
    unsigned short  currMotor;      // Motor current (A)
    unsigned short  capacityBatt;   // Battery capacity overall
    unsigned short  operationalTime;//minutes    
    
    unsigned char   align;
    unsigned char   CRC8;
};
#else
#define OPTIM_TOYO_MB2PIX_MSG  0x01
/*new message without geiger batteries */
struct optimMB2PixMsg_Ref{
    unsigned char   delimiter;      //  0x55
    unsigned char   messageID;      //  0x01
    unsigned char   messageSize;    //  0x1E = 30 bytes
    unsigned char   count;

    unsigned char   align;
    unsigned char   escID;
    unsigned char   statusFlag;     // LSB = 1 if crc error on their side, or 0x80 for controller crc error
    unsigned char   controlByte;
    unsigned char   errorByte;
    unsigned char   throttleSetpoint;
    unsigned char   tempMotor;      //temp of motor
    unsigned char   tempMC;         //temp of microcontroller
    
    unsigned short  pwmMC;          // PWM microcontroller
    unsigned short  voltMC;         // microcontroller voltage (V)
    unsigned short  rpmMotor;       // rpm of motor
    unsigned short  currMotor;      // Motor current (A)
    unsigned short  operationalTime;//minutes    
    
    unsigned char   tempBatt;
    unsigned char   CRC8;
};
#endif

#define OPTIM_TOYO_MB2PIX_MSG2 0x02
#if TOYO_17_VERSION == 1
    #define MONITOR_BRAIN_WATCHDOG 101 // we read at 100hz ~ equiv to one second
    //swan config
    struct optimMB2PixMsg2_Ref{
        unsigned char   delimiter;      //  0x55
        unsigned char   messageID;      //  0x02
        unsigned char   messageSize;    //  0x42
        unsigned char   count;

        unsigned char   surfID_1;       //0x01
        unsigned char   surfStatus_1;   // 8 bits of flags to consider
        short           surfPos_1;

        unsigned char   surfID_2;       // 0x02
        unsigned char   surfStatus_2;   // 8 bits of flags to consider
        short           surfPos_2;

        unsigned char   surfID_3;       //0x03
        unsigned char   surfStatus_3;   // 8 bits of flags to consider
        short           surfPos_3;

        unsigned char   surfID_4;       //0x04
        unsigned char   surfStatus_4;   // 8 bits of flags to consider
        short           surfPos_4;

        unsigned char   surfID_5;       //0x05
        unsigned char   surfStatus_5;   // 8 bits of flags to consider for motor

        unsigned short  rpm;
        unsigned short  battery_voltage;    //multiply by 6 for scaling
        unsigned char   brain_temp;         //2's compliment 
        unsigned char   right_wing_temp;    //2's comp
        unsigned char   left_Wing_temp;     //2s comp
        unsigned char   control_brain_comms_status;
        unsigned char   align;
        unsigned char   csum;
    };
#endif

#if TOYO_16_VERSION == 1
    struct optimMB2PixMsg2_Ref{
        unsigned char   delimiter;      //  0x55
        unsigned char   messageID;      //  0x02
        unsigned char   messageSize;    //  0x42
        unsigned char   count;

        unsigned char   surfID_1;
        unsigned char   surfStatus_1;   // 8 bits of flags to consider
        short           surfPos_1;

        unsigned char   surfID_2;
        unsigned char   surfStatus_2;   // 8 bits of flags to consider
        short           surfPos_2;

        unsigned char   surfID_3;
        unsigned char   surfStatus_3;   // 8 bits of flags to consider
        short           surfPos_3;

        unsigned char   surfID_4;
        unsigned char   surfStatus_4;   // 8 bits of flags to consider
        short           surfPos_4;

        unsigned char   surfID_5;
        unsigned char   surfStatus_5;   // 8 bits of flags to consider
        short           surfPos_5;

        unsigned char   surfID_6;
        unsigned char   surfStatus_6;   // 8 bits of flags to consider
        short           surfPos_6;

        unsigned char   surfID_7;
        unsigned char   surfStatus_7;   // 8 bits of flags to consider
        short           surfPos_7;

        unsigned char   surfID_8;
        unsigned char   surfStatus_8;   // 8 bits of flags to consider
        short           surfPos_8;

        unsigned char   surfID_9;
        unsigned char   surfStatus_9;   // 8 bits of flags to consider
        short           surfPos_9;  

        unsigned char   surfID_10;
        unsigned char   surfStatus_10;   // 8 bits of flags to consider
        short           surfPos_10;

        unsigned char   surfID_11;
        unsigned char   surfStatus_11;   // 8 bits of flags to consider
        short           surfPos_11;

        unsigned char   surfID_12;
        unsigned char   surfStatus_12;   // 8 bits of flags to consider
        short           surfPos_12;

        unsigned char   surfID_13;
        unsigned char   surfStatus_13;   // 8 bits of flags to consider
        short           surfPos_13;

        unsigned char   surfID_14;
        unsigned char   surfStatus_14;   // 8 bits of flags to consider
        short           surfPos_14;

        unsigned short  avionics_voltage;

        unsigned char   align;
        unsigned char   CRC8;          
    };
#endif


struct optimToyoHalfPkt1_ref{
    unsigned char   delimiter;      //  0x55
    unsigned char   messageID;      //  0x01
    unsigned char   messageSize;    //  0x1E = 30 bytes
    unsigned char   count;

    unsigned char   escID;
    unsigned char   statusFlag;     // LSB = 1 if crc error on their side
    unsigned char   controlByte;
    unsigned char   errorByte;
    unsigned char   throttleSetpoint;
    unsigned char   tempBatt;       //temp in celcius
    unsigned char   tempMotor;      //temp of motor
    unsigned char   tempMC;         //temp of microcontroller
};

struct optimToyoHalfPkt2_ref{
    unsigned short  pwmMC;          // PWM microcontroller
    unsigned short  voltMC;         // microcontroller voltage (V)
    unsigned short  currBatt;       // battery current (A)
    unsigned short  rpmMotor;       // rpm of motor
    unsigned short  bmsStatus;      // BMS status word?
    unsigned short  currMotor;      // Motor current (A)
    unsigned short  capacityBatt;   // Battery capacity overall
    unsigned short  operationalTime;//minutes    
    
    unsigned char   align;
    unsigned char   CRC8;
};

struct toyoBattTemp_ref{
    unsigned char temp[18];
};

struct toyoBattCap_ref{
    unsigned short cap[18];
};

struct toyoBattCurr_ref{
    unsigned short curr[18];
};

struct serialPort_ref {
  int dataSource ;//= 0 ;//0;//=off,1;//=on,2;//=SITL,3;//=playback,4;//=worker;
  int init ;//= 1 ;// ;
  int port;//=1 ;//port number (1 lowest);
  int baud ;//= 9600 ;// ;
  char *  termios ;//= "8n1\0" ;// 8|7, n|o|e, 1, default is 8n1 ;
  char *  nameFmt ;//= "/dev/ttyS%d\0                 " ;//port name format (posix);
  char *  name    ;//= "\0                  " ;//port name (posix);
  char *  connectTo ;//= "hostname\0                                       ";// host name of machine to connect to;
  char   useOptimAeroComms;
  char   setRemotePort; //boolean to set remote port
  int    remotePortRequest; // port number requested to be set
  int    portNum   ;//= 0 ;// port number;
  int    portType  ;//= 1 ;// port type specifier for DSP, 0-fifo port, 1-direct uart;
  int useSerial    ;//= 1 ;// Default is to use the hardware serial - used to change modes;
  int useSock      ;//= 0 ;// Use Berkeley Sockets - used to change modes;
  int useTcp       ;//= 0 ;// Use tcp instead of udp - currently not supported;
  int connectionMode ;//= 0 ;// The current connection mode (defaults to none (off)) (1;//=serial,2;//=UDP,3;//=TCP,4;//=SPI);
  int newConnectionMode ;//= 0 ;// Set to a mode to change modes (1;//=serial,2;//=UDP,3;//=TCP);
  int connectState ;//= 0 ;// state of tcp connection;
  double connectRetry ;//= 1.0 ;// connect retry frequency;
  double lastConnectRetry ;//= 0.0 ;// last time connection was retried;
  int portIsOpen ;//= 0 ;// Is port open flag;
  int bytesread ;//= 0 ;// ;
  int fd ;//= 0;// File Descriptor;
  unsigned long dropwrite ;//= 0 ;// bytes dropped while writing;
  unsigned long dropread  ;//= 0 ;// bytes dropped while reading;
  unsigned long sent ;//= 0      ;// sent bytes;
  unsigned long received  ;//= 0 ;// received bytes;
  int blocking     ;//= 0 ;// make this a blocking port ;
  int blockingRead ;//= 0 ;// make this a blocking read port;
  double blockTimeout ;//= 0.1 ;// s, -1 ;//= block for ever, timeout for select timeout;
  int blockDeadlineExpired   ;//= 0   ;// nondim, blocking read timed out;
  void * serialDevice ;//= 0;// serial device pointer;
  void * sockDevice    ;//= 0;// bsd device pointer;
  char  myname[46] ;//= "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" ;// myhostname;   char  myname[46] ;
  int myport ;//= 0 ;// myport;
  char remotename[46] ;//= "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" ;// remotename;
  int remoteport ;//= 0 ;// remoteport;
  unsigned char bufferView[8] ;//= {0,0,0,0,0,0,0,0} ;// ;
  unsigned char buffer[1024*100] ;//= {0} ;// ;
};



#ifdef __cplusplus
}
#endif

#endif /* CHECKSUM_H_ */
