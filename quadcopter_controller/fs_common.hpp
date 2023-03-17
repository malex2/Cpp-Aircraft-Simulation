//
//  fs_common.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 2/26/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#ifndef fs_common_hpp
#define fs_common_hpp

#include <stdio.h>
#include <math.h>

// MACROS
//#define SIMULATION
#define IMU
#define GPS
#define BAROMETER
//#define PWM
//#define CONTROLS
//#define GROUND_DETECTION
#define NAVIGATION
//#define TELEMETRY
//#define PRINT
//#define UBX_PRINT

#ifdef SIMULATION
    #include "model_mapping.hpp"
    #include "arduino_class_models.hpp"
    #include "utilities.hpp"
    #include <iostream>
    #include <iomanip>
    #include <fstream>
    typedef std::string String;
    #define FS_Serial SimulationSerial

   class ModelMap;
   class Utilities;

   #define DEBUG_PRINT
#else
    #include "arduino.h"
    #include <EEPROM.h>
    #ifdef max
        #undef max
        #undef min
    #endif
    #define LEDPIN LED_BUILTIN

    //#ifdef TEENSYDUINO
      #define FS_Serial HardwareSerial
      #define DEBUG_PRINT
      #include "Wire.h"

      #include <SPI.h>
      #include <nRF24L01.h>
      #include <RF24.h>
    //#else //ARDUINO
    //  #include <SoftwareSerial.h>
    //  #define FS_Serial SoftwareSerial
    //#endif
#endif

// Forward References
struct IMUtype;
struct BarometerType;
struct GpsType;
struct NavType;
struct ControlType;

// Types
enum I2C_Error_Code {I2C_0_SUCCESS, I2C_1_DATA_TOO_LONG, I2C_2_NACK_ADDRESS, I2C_3_NACK_DATA, I2C_4_OTHER, I2C_5_TIMEOUT};
enum FS_Timing_Type {hz1, hz50, hz100, hz200, hz800, printRoutine, nRoutines};
enum FS_TM_READ_STATE {FS_TM_HEADER, TM_TYPE_HEADER, READ_TM_LENGTH, READ_TM_BUFFER, CALC_TM_CHECKSUM, TM_READ_COMPLETE};
enum TM_MSG_TYPE {FS_TM, FS_PRINT};
enum channelType {THROTTLE_CHANNEL, ROLL_CHANNEL, PITCH_CHANNEL, YAW_CHANNEL, nChannels};

struct TWO_BYTE_DATA
{
    TWO_BYTE_DATA(bool debug = false) { debug_print = debug; clear(); }
    
    byte         data[2];
    unsigned int index;
    bool         debug_print;
    
    uint16_t value();
    void swap();
    void clear();
};

// Mass
#define Mass 0.5
#define Ixx  0.0196
#define Iyy  0.0196
#define Izz  0.0264

// Propeller Layout
#define PropDistance 0.1

// Conversions
#define degree2radian M_PI/180.0
#define radian2degree 180.0/M_PI
#define hz2rps        2.0*M_PI
#define rpm2rps       2.0*M_PI/60.0
#define inv_sqrt2     1.0/sqrt(2.0)
#define inv_sqrt3     1.0/sqrt(3.0)

// Constants
#define RE       6371e+3
#define GMe      3.9857e+14
#define Gravity  9.80665
#define Weight   Mass*Gravity
#define EARTH_a  6378137.0
#define EARTH_a2 EARTH_a*EARTH_a
#define EARTH_b  6356752.3142
#define EARTH_b2 EARTH_b*EARTH_b
#define EARTH_e2 1.0 - (EARTH_b2)/(EARTH_a2)
const double dtPad = 1e-10;

// Pulse In Pins
#define THROTTLEPIN  7  // CH3
#define ROLLPIN      8  // CH4
#define PITCHPIN     12 // CH5
#define YAWPIN       13 // CH6

// Pulse Out Pins
#define T1PIN  6
#define T2PIN  9
#define T3PIN  10
#define T4PIN  11

// High Dynamics
#define highRate  5.0*degree2radian
#define highAccel 12.0

// GPS Pins
#define GPSRXPIN 2 // Rcv GPS msgs, connect to GPS TX
#define GPSTXPIN 3 // Txmit GPS msgs, connect to GPS RX

// IMU Pins
#define IMUSDAPIN A4
#define IMUSCLPIN A5

// PWM
#define PWMMIN    1000
#define PWMMINRPM 1100
#define PWMMAX    2000

#define MINRPM 500*rpm2rps
#define MAXRPM 6440*rpm2rps
#define dMIN   4.0*MINRPM*MINRPM
#define dMAX   4.0*MAXRPM*MAXRPM

// Min/Max Limits
#define AltHoldVelEnter 0.06  // m/s
#define AltHoldVelExit  0.061 // m/s
#define AltError        1.0   // m

#define MAXVELOCITY  1.5 // m/s - 5 ft/s
#define MAXROLL      25.0 * degree2radian // rad
#define MAXPITCH     25.0 * degree2radian // rad
#define MAXYAWRATE   360.0 * degree2radian // rad/s
#define MINTHROTTLE  dMIN
#define MAXTHROTTLE  0.85*dMAX
#define minPWMIncr   5.0

// Thrust Constants
#define        MAXTHRUST   4.22
#define        TorqueRatio 0.003

// Thrust Coefficients
#define KT     MAXTHRUST/(MAXRPM*MAXRPM)
#define KH     TorqueRatio * KT
#define dTo    Weight/(KT)

// Dynamics Shorthand
#define KVX    -KT*dTo/(Mass)
#define KVY    KT*dTo/(Mass)
#define KVZ    -KT/(Mass)

#define KROLL  KT*PropDistance/(Ixx)
#define KPITCH KT*PropDistance/(Iyy)
#define KYAW   KH/(Izz)

// IMU Addresses
#define MPU_ADDR   0x68
#define PWR_MGMT_1 0x6B // PWR_MGMT_1 register
#define GYRO_REG   0x1B // Gryo register
#define ACC_REG    0x1C // Accelerometer register
#define ACC_OUT    0x3B // (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
#define INT        0x37
#define INT_ENABLE 0x38

// Baramoter Addresses
#define BMP180_ADDR                0x77 // 7-bit address
#define BMP180_REG_CONTROL         0xF4
#define BMP180_REG_RESULT          0xF6
#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE0   0x34
#define BMP180_COMMAND_PRESSURE1   0x74
#define BMP180_COMMAND_PRESSURE2   0xB4
#define BMP180_COMMAND_PRESSURE3   0xF4
#define AC1_ADDR                   0xAA
#define AC2_ADDR                   0xAC
#define AC3_ADDR                   0xAE
#define AC4_ADDR                   0xB0
#define AC5_ADDR                   0xB2
#define AC6_ADDR                   0xB4
#define VB1_ADDR                   0xB6
#define VB2_ADDR                   0xB8
#define MB_ADDR                    0xBA
#define MC_ADDR                    0xBC
#define MD_ADDR                    0xBE

// GPS Addresses
#define UBX_HEADER_1   0xB5
#define UBX_HEADER_2   0x62

#define NMEA           0xF0
#define NMEA_GGA       0x00
#define NMEA_GLL       0x01
#define NMEA_GSA       0x02
#define NMEA_GSV       0x03
#define NMEA_RMC       0x04
#define NMEA_VTG       0x05

#define UBX_NAV        0x01
#define UBX_NAV_POSLLH 0x02
#define UBX_NAV_STATUS 0x03
#define UBX_NAV_DOP    0x04
#define UBX_NAV_VELNED 0x12
#define UBX_NAV_SOL    0x06

#define UBX_RXM        0x02

#define UBX_INF        0x04

#define UBX_ACK        0x05
#define UBX_ACK_ACK    0x01
#define UBX_ACK_NAK    0x00

#define UBX_CFG        0x06
#define UBX_CFG_MSG    0x01
#define UBX_CFG_NAV5   0x24

#define UBX_MON        0x0A

#define UBX_AID        0x0B
#define UBX_AID_ALM    0x30
#define UBX_AID_DATA   0x10
#define UBX_AID_EPH    0x31
#define UBX_AID_HUI    0x02
#define UBX_AID_INI    0x01
#define UBX_AID_REQ    0x00

#define UBX_TIM        0x0D

#define UBX_CFG_ON_OFF_SHORT_SIZE 3
#define UBX_CFG_ON_OFF_LONG_SIZE  8

#define UBX_MSG_HEADER_SIZE       2
#define UBX_MSG_CLASS_ID_SIZE     2
#define UBX_MSG_LENGTH_SIZE       2
#define UBX_MSG_CHECKSUM_SIZE     2

// Serial Fifo
#define MAX_SERIAL_FIFO_SIZE 800
#define NRF24L01_BUFFER_SIZE 32

// Classes
class FS_FIFO {
public:
    FS_FIFO();
    FS_FIFO(FS_Serial* serialIO);
    FS_FIFO(RF24* tmIO);
    ~FS_FIFO();
    
    void begin(uint32_t baud_rate);
    
    // Update
    void update_fifo();
    
    // serialIO to Flight Software
    byte read();
    unsigned short available();
    unsigned short get_read_count() { return read_byte_count; }
    unsigned short get_max_read_buffer_length()   { return max_read_buffer_length; }
    unsigned short get_read_fifo_overflow_count() { return read_fifo_overflow_count; }
    
    // Flight Software to Serial IO
    unsigned short write(byte val);
    unsigned short write(byte* val, unsigned long length);
    bool           write_available();
    unsigned short get_write_count() { return write_byte_count; };
    unsigned short  get_write_buffer_length() { return write_buffer_length; }
    unsigned short  get_max_write_buffer_length() { return max_write_buffer_length; }
    unsigned short get_write_fifo_overflow_count()   { return write_fifo_overflow_count; }
    unsigned short get_write_buffer_overflow_count() { return write_buffer_overflow_count; }
    
    bool read_fifo_full();
    bool write_fifo_full();
    
    // Debug
    void display_read_buffer();
    void display_write_buffer();
private:
    byte read_val;
    byte read_buffer[MAX_SERIAL_FIFO_SIZE];
    unsigned short read_buffer_index;
    unsigned short read_buffer_length;
    unsigned short read_byte_count;
    unsigned short max_read_buffer_length;
    unsigned short read_fifo_overflow_count;
    
    byte write_buffer[MAX_SERIAL_FIFO_SIZE];
    unsigned short  write_buffer_index;
    unsigned short  write_buffer_length;
    unsigned short write_byte_count;
    unsigned short  max_write_buffer_length;
    double baud_dt;
    double prevWriteTime;
    unsigned short write_fifo_overflow_count;
    unsigned short write_buffer_overflow_count;
    
    byte temp_nrf24L01_buffer[NRF24L01_BUFFER_SIZE];
    FS_Serial* serialIO;
    RF24*      nrf24L01;
    
    void reset_read_buffer();
    void reset_write_buffer();
};

// Time
double getTime();
#ifdef SIMULATION
    void delay(int ms_delay);
#endif

// Memory
void write_eeprom(unsigned int address, byte val);
void write_eeprom(unsigned int address, byte* val, unsigned int size);

void read_eeprom(unsigned int address, byte* val);
void read_eeprom(unsigned int address, byte* val, unsigned int size);

// Math Support
double errorToVariance(double maxError);
double varianceToError(double variance);
void crossProduct(double *cross, double *a, double *b);

// Printing
void display(const char* val, int printMode = DEC);
template<typename TempType>
void display(TempType val, int printMode = DEC);
void display(I2C_Error_Code val, int printMode = DEC);
FS_FIFO* get_print_fifo();
unsigned int sizeof_char(const char* str);
void FsCommon_performSerialIO();

// LED
void LEDon();
void LEDoff();

#ifdef SIMULATION
    void FsCommon_setSimulationModels(ModelMap* pMap);
#endif

uint16_t FsCommon_computeChecksum(const void* data, unsigned int len);
bool FsCommon_checkChecksum(uint16_t buffer_checksum, const void* data, unsigned int len);

// Groups of 32 bytes, 8 bytes each
// uint32_t for large types
// uint16_t
// byte or char for flags
// 59 x 32 bits = 1888 bits = 236 (bytes)
#define TM_HEADER       0xA0
#define TM_TM_HEADER    0xB1
#define TM_PRINT_HEADER 0xC2
#define FS_TM_HEADER_SIZE 2
#define FS_TM_LENGTH_SIZE 2
#define FS_TM_CHECKSUM_SIZE 2

#define TM_PRINT
struct FS_Telemetry_Type
{
    FS_Telemetry_Type() { clear(); disable_all_printing(); }
    
    enum TM_PRINT_TYPE {TIMING_TM, IMU_TM, BARO_TM, GPS_TM, NAV_TM, CONTROLS_TM, N_TM_PRINT_TYPE};
    bool print_bool[N_TM_PRINT_TYPE];
    bool any_print()
    {
        for (int i=0; i<N_TM_PRINT_TYPE; i++)
        { if (print_bool[i]) return true; }
        return false;
    }
    
    void enable_all_printing()
    {
        for (int i=0; i<N_TM_PRINT_TYPE; i++)
        { enable_printing((TM_PRINT_TYPE) i); }
    }
    
    void disable_all_printing()
    {
        for (int i=0; i<N_TM_PRINT_TYPE; i++)
        { disable_printing((TM_PRINT_TYPE) i); }
    }
    
    void enable_printing(TM_PRINT_TYPE i_TM)  { print_bool[i_TM] = true;}
    void disable_printing(TM_PRINT_TYPE i_TM) { print_bool[i_TM] = false; }
    
    struct data_t
    {
        uint32_t fs_time = 0;
        
        // IMU
        uint32_t imu_timestamp = 0;
        int32_t gyro_x = 0;
        int32_t gyro_y = 0;
        int32_t gyro_z = 0;
        int32_t accel_x = 0;
        int32_t accel_y = 0;
        int32_t accel_z = 0;
        
        // Baro
        uint32_t baro_timestamp = 0;
        uint16_t pressure = 0; //mbar
        int16_t  temperature = 0;
        int32_t  baro_altitude = 0;
        
        // GPS
        uint32_t gps_timestamp = 0;
        uint32_t gps_receiver_time = 0;
        uint32_t gps_ttff = 0;
        int32_t  gps_lat = 0;
        int32_t  gps_lon = 0;
        int32_t  gps_alt = 0;
        int32_t  gps_pos_east = 0;
        int32_t  gps_pos_north = 0;
        int32_t  gps_pos_up = 0;
        int32_t  gps_vel_n = 0;
        int32_t  gps_vel_e = 0;
        int32_t  gps_vel_d = 0;
        uint32_t gps_horiz_pos_acc = 0;
        uint32_t gps_vert_pos_acc = 0;
        uint32_t gps_speed_acc = 0;
        uint32_t gps_rcvd_msg_count = 0;
        uint32_t gps_read_fifo_overflow_count = 0;
        
        // Nav
        uint32_t nav_timestamp = 0;
        int32_t  roll = 0;
        int32_t  pitch = 0;
        int32_t  yaw = 0;
        int32_t  nav_vel_n = 0;
        int32_t  nav_vel_e = 0;
        int32_t  nav_vel_d = 0;
        int32_t  nav_pos_n = 0;
        int32_t  nav_pos_e = 0;
        int32_t  nav_alt = 0;
        int32_t  accel_bias_x = 0;
        int32_t  accel_bias_y = 0;
        int32_t  accel_bias_z = 0;
        int32_t  gyro_bias_x = 0;
        int32_t  gyro_bias_y = 0;
        int32_t  gyro_bias_z = 0;
        int32_t  gravity = 0;
        int32_t  accelerometer_pitch = 0;
        int32_t  accelerometer_roll = 0;
        uint32_t ins_update_count = 0;
        uint32_t baro_filter_count = 0;
        uint32_t gps_filter_count = 0;
        uint32_t accel_filter_count = 0;
        
        // Controls
        uint32_t ctrl_timestamp = 0;
        uint16_t throttle_channel_pwm = 0;
        uint16_t roll_channel_pwm = 0;
        uint16_t pitch_channel_pwm = 0;
        uint16_t yaw_channel_pwm = 0;
        int32_t  VLLxCmd = 0;
        int32_t  VLLyCmd = 0;
        int32_t  VLLzCmd = 0;
        int32_t  rollCmd = 0;
        int32_t  pitchCmd = 0;
        int32_t  yawRateCmd = 0;
        uint16_t prop1_pwm = 0;
        uint16_t prop2_pwm = 0;
        uint16_t prop3_pwm  = 0;
        uint16_t prop4_pwm = 0;
        
        // Status
        byte gps_fixOk = 0;
        byte numSV = 0;
        byte baroI2CErrorCode = 0;
        byte imuI2CErrorCode = 0;
        byte ctrl_mode = 0;
        byte onGround = 0;
        byte takeOff = 0;
        byte crashLand = 0;
        uint32_t tm_timeout_count = 0;
        uint16_t max_tm_rate = 0;
        uint16_t hz1_avg_rate = 0;
        uint16_t hz50_avg_rate = 0;
        uint16_t hz100_avg_rate = 0;
        uint16_t hz200_avg_rate = 0;
        uint16_t hz800_avg_rate = 0;

    } __attribute__((packed)) data;
    
    // Divide by scale when encoding
    // Multiply by scale when decoding
    struct scale_t
    {
        double fs_time = 1.0e-3;
        double imu_timestamp = 1.0e-3;
        double gyro_x = 1.0e-7;
        double gyro_y = 1.0e-7;
        double gyro_z = 1.0e-7;
        double accel_x = 1.0e-2;
        double accel_y = 1.0e-2;
        double accel_z = 1.0e-2;
        
        double baro_timestamp = 1.0e-3;
        double pressure = 1.0e-2;
        double temperature = 1.0e-2;
        double baro_altitude = 1.0e-3;
        
        double gps_timestamp = 1.0e-3;
        double gps_receiver_time = 1.0e-3;
        double gps_ttff = 1.0e-3;
        double gps_lat = 1.0e-7;
        double gps_lon = 1.0e-7;
        double gps_alt = 1.0e-3;
        double gps_pos_east = 1.0e-3;
        double gps_pos_north = 1.0e-3;
        double gps_pos_up = 1.0e-3;
        double gps_vel_n = 1.0e-3;
        double gps_vel_e = 1.0e-3;
        double gps_vel_d = 1.0e-3;
        double gps_horiz_pos_acc = 1.0e-3;
        double gps_vert_pos_acc = 1.0e-3;
        double gps_speed_acc = 1.0e-3;
        double gps_rcvd_msg_count = 1.0;
        double gps_read_fifo_overflow_count = 1.0;
        
        double nav_timestamp = 1.0e-3;
        double roll = 1.0e-7;
        double pitch = 1.0e-7;
        double yaw = 1.0e-7;
        double nav_vel_n = 1.0e-3;
        double nav_vel_e = 1.0e-3;
        double nav_vel_d = 1.0e-3;
        double nav_pos_n = 1.0e-3;
        double nav_pos_e = 1.0e-3;
        double nav_alt = 1.0e-3;
        double accel_bias_x = 1.0e-2;
        double accel_bias_y = 1.0e-2;
        double accel_bias_z = 1.0e-2;
        double gyro_bias_x = 1.0e-7;
        double gyro_bias_y = 1.0e-7;
        double gyro_bias_z = 1.0e-7;
        double gravity = 1.0e-2;
        double accelerometer_pitch = 1.0e-7;
        double accelerometer_roll = 1.0e-7;
        double ins_update_count = 1.0;
        double baro_filter_count = 1.0;
        double gps_filter_count = 1.0;
        double accel_filter_count = 1.0;
        
        double ctrl_timestamp = 1.0e-3;
        double throttle_channel_pwm = 1.0;
        double roll_channel_pwm = 1.0;
        double pitch_channel_pwm = 1.0;
        double yaw_channel_pwm = 1.0;
        double VLLxCmd = 1.0e-3;
        double VLLyCmd = 1.0e-3;
        double VLLzCmd = 1.0e-3;
        double rollCmd = 1.0e-7;
        double pitchCmd = 1.0e-7;
        double yawRateCmd = 1.0e-7;
        double prop1_pwm = 1.0;
        double prop2_pwm = 1.0;
        double prop3_pwm = 1.0;
        double prop4_pwm = 1.0;
        
        double gps_fixOk = 1.0;
        double numSV = 1.0;
        double baroI2CErrorCode = 1.0;
        double imuI2CErrorCode = 1.0;
        double ctrl_mode = 1.0;
        double onGround = 1.0;
        double takeOff = 1.0;
        double crashLand = 1.0;
        
        double tm_timeout_count = 1.0;
        double max_tm_rate = 0.1;
        double hz1_avg_rate = 0.1;
        double hz50_avg_rate = 0.1;
        double hz100_avg_rate = 0.1;
        double hz200_avg_rate = 0.1;
        double hz800_avg_rate = 0.1;
    } scale;
    
    void clear() { memset(&data, 0, sizeof(data)); }
    
    void print()
    {
#ifdef TM_PRINT
        if (any_print())
        {
            Serial.print("FS Time: ");
            Serial.println(data.fs_time * scale.fs_time);
            Serial.print("\n");
        }
        
        if (print_bool[TIMING_TM])
        {
            Serial.print("FS Rates = [1hz, 50hz, 100hz, 200hz, 800hz]: [");
            Serial.print(data.hz1_avg_rate * scale.hz1_avg_rate); Serial.print(", ");
            Serial.print(data.hz50_avg_rate * scale.hz50_avg_rate); Serial.print(", ");
            Serial.print(data.hz100_avg_rate * scale.hz100_avg_rate); Serial.print(", ");
            Serial.print(data.hz200_avg_rate * scale.hz200_avg_rate); Serial.print(", ");
            Serial.print(data.hz800_avg_rate * scale.hz800_avg_rate); Serial.println("]");
            
            Serial.print("[max_tm_rate, tm_timeout_count]: ["); Serial.println(data.max_tm_rate * scale.max_tm_rate);
            Serial.print(", "); Serial.println(data.tm_timeout_count * scale.tm_timeout_count);
            Serial.print("]\n");
        }
        
        if (print_bool[IMU_TM])
        {
            Serial.print("IMU I2C code: ");
            Serial.println(data.imuI2CErrorCode * scale.imuI2CErrorCode);
            
            Serial.print("gyro (deg/s): [");
            Serial.print(data.gyro_x * scale.gyro_x * radian2degree); Serial.print(", ");
            Serial.print(data.gyro_y * scale.gyro_y * radian2degree); Serial.print(", ");
            Serial.print(data.gyro_z * scale.gyro_z * radian2degree); Serial.println("]");
            
            Serial.print("accel (g): [");
            Serial.print(data.accel_x * scale.accel_x / (Gravity)); Serial.print(", ");
            Serial.print(data.accel_y * scale.accel_y / (Gravity)); Serial.print(", ");
            Serial.print(data.accel_z * scale.accel_z / (Gravity)); Serial.println("]");
            Serial.print("\n");
        }
        
        if (print_bool[BARO_TM])
        {
            Serial.print("Baro I2C code: ");
            Serial.println(data.baroI2CErrorCode * scale.baroI2CErrorCode);
            
            Serial.print("Baro [t, pres, temp, alt]: [");
            Serial.print(data.baro_timestamp * scale.baro_timestamp); Serial.print(", ");
            Serial.print(data.pressure * scale.pressure); Serial.print(", ");
            Serial.print(data.temperature * scale.temperature); Serial.print(", ");
            Serial.print(data.baro_altitude * scale.baro_altitude); Serial.println("]");
            Serial.print("\n");
        }

        if (print_bool[GPS_TM])
        {
            Serial.print("GPS [t, rcv_t, ttff]: [");
            Serial.print(data.gps_timestamp * scale.gps_timestamp); Serial.print(", ");
            Serial.print(data.gps_receiver_time * scale.gps_receiver_time); Serial.print(", ");
            Serial.print(data.gps_ttff * scale.gps_ttff); Serial.println("]");

            Serial.print("GPS LLH: [");
            Serial.print(data.gps_lat * scale.gps_lat); Serial.print(", ");
            Serial.print(data.gps_lon * scale.gps_lon); Serial.print(", ");
            Serial.print(data.gps_alt * scale.gps_alt); Serial.println("]");

            Serial.print("GPS Pos NEU: [");
            Serial.print(data.gps_pos_north * scale.gps_pos_north); Serial.print(", ");
            Serial.print(data.gps_pos_east * scale.gps_pos_east); Serial.print(", ");
            Serial.print(data.gps_pos_up * scale.gps_pos_up); Serial.println("]");
            
            Serial.print("GPS Vel NED: [");
            Serial.print(data.gps_vel_n * scale.gps_vel_n); Serial.print(", ");
            Serial.print(data.gps_vel_e * scale.gps_vel_e); Serial.print(", ");
            Serial.print(data.gps_vel_d * scale.gps_vel_d); Serial.println("]");
            
            Serial.print("GPS Acc [horiz, ver, speed]: [");
            Serial.print(data.gps_horiz_pos_acc * scale.gps_horiz_pos_acc); Serial.print(", ");
            Serial.print(data.gps_vert_pos_acc * scale.gps_vert_pos_acc); Serial.print(", ");
            Serial.print(data.gps_speed_acc * scale.gps_speed_acc); Serial.println("]");

            Serial.print("GPS Acc [rcvd_msg, fifo_overflow_count]: [");
            Serial.print(data.gps_rcvd_msg_count * scale.gps_rcvd_msg_count); Serial.print(", ");
            Serial.print(data.gps_read_fifo_overflow_count * scale.gps_read_fifo_overflow_count); Serial.println("]");
     
            Serial.print("GPS Acc [gps_fixOk, numSV]: [");
            Serial.print(data.gps_fixOk * scale.gps_fixOk); Serial.print(", ");
            Serial.print(data.numSV * scale.numSV); Serial.println("]");
            Serial.print("\n");
        }
        
        if (print_bool[NAV_TM])
        {
            Serial.print("Nav Euler: [");
            Serial.print(data.roll * scale.roll * radian2degree); Serial.print(", ");
            Serial.print(data.pitch * scale.pitch * radian2degree); Serial.print(", ");
            Serial.print(data.yaw * scale.yaw * radian2degree); Serial.println("]");
            
            Serial.print("Nav Vel NED: [");
            Serial.print(data.nav_vel_n * scale.nav_vel_n); Serial.print(", ");
            Serial.print(data.nav_vel_e * scale.nav_vel_e); Serial.print(", ");
            Serial.print(data.nav_vel_d * scale.nav_vel_d); Serial.println("]");
            
            Serial.print("Nav Pos NEU: [");
            Serial.print(data.nav_pos_n * scale.nav_pos_n); Serial.print(", ");
            Serial.print(data.nav_pos_e * scale.nav_pos_e); Serial.print(", ");
            Serial.print(data.nav_alt * scale.nav_alt); Serial.println("]");
            
            Serial.print("Nav Accel Bias (g): [");
            Serial.print(data.accel_bias_x * scale.accel_bias_x / (Gravity)); Serial.print(", ");
            Serial.print(data.accel_bias_y * scale.accel_bias_y / (Gravity)); Serial.print(", ");
            Serial.print(data.accel_bias_z * scale.accel_bias_z / (Gravity)); Serial.println("]");
            
            Serial.print("Nav Gyro Bias (deg/s): [");
            Serial.print(data.gyro_bias_x * scale.gyro_bias_x * radian2degree); Serial.print(", ");
            Serial.print(data.gyro_bias_y * scale.gyro_bias_y * radian2degree); Serial.print(", ");
            Serial.print(data.gyro_bias_z * scale.gyro_bias_z * radian2degree); Serial.println("]");
            
            Serial.print("Nav [accel_roll, accel_pitch, gravity]: [");
            Serial.print(data.accelerometer_roll * scale.accelerometer_roll * radian2degree); Serial.print(", ");
            Serial.print(data.accelerometer_pitch * scale.accelerometer_pitch * radian2degree); Serial.print(", ");
            Serial.print(data.gravity * scale.gravity); Serial.println("]");
            
            Serial.print("Nav [ins, accel, baro, gps] filer count: [");
            Serial.print(data.ins_update_count * scale.ins_update_count); Serial.print(", ");
            Serial.print(data.accel_filter_count * scale.accel_filter_count); Serial.print(", ");
            Serial.print(data.baro_filter_count * scale.baro_filter_count); Serial.print(", ");
            Serial.print(data.gps_filter_count * scale.gps_filter_count); Serial.println("]");
            Serial.print("\n");
        }

        if (print_bool[CONTROLS_TM])
        {
            Serial.print("PWM In: [");
            Serial.print(data.throttle_channel_pwm * scale.throttle_channel_pwm); Serial.print(", ");
            Serial.print(data.roll_channel_pwm * scale.roll_channel_pwm); Serial.print(", ");
            Serial.print(data.pitch_channel_pwm * scale.pitch_channel_pwm); Serial.print(", ");
            Serial.print(data.yaw_channel_pwm * scale.yaw_channel_pwm); Serial.println("]");
            
            Serial.print("Prop PWM: [");
            Serial.print(data.prop1_pwm * scale.prop1_pwm); Serial.print(", ");
            Serial.print(data.prop2_pwm * scale.prop2_pwm); Serial.print(", ");
            Serial.print(data.prop3_pwm * scale.prop3_pwm); Serial.print(", ");
            Serial.print(data.prop4_pwm * scale.prop4_pwm); Serial.println("]");
            
            Serial.print("Ctrl Euler Cmd: [");
            Serial.print(data.rollCmd * scale.rollCmd * radian2degree); Serial.print(", ");
            Serial.print(data.pitchCmd * scale.pitchCmd * radian2degree); Serial.print(", ");
            Serial.print(data.yawRateCmd * scale.yawRateCmd * radian2degree); Serial.println("]");
            
            Serial.print("Ctrl Vel Cmd: [");
            Serial.print(data.VLLxCmd * scale.VLLxCmd); Serial.print(", ");
            Serial.print(data.VLLyCmd * scale.VLLyCmd); Serial.print(", ");
            Serial.print(data.VLLzCmd * scale.VLLzCmd); Serial.println("]");
            
            Serial.print("Ctrl [mode, onGround, takeOff, crashLand]: [");
            Serial.print(data.ctrl_mode * scale.ctrl_mode); Serial.print(", ");
            Serial.print(data.onGround * scale.onGround); Serial.print(", ");
            Serial.print(data.takeOff * scale.takeOff); Serial.print(", ");
            Serial.print(data.crashLand * scale.crashLand); Serial.println("]");
            Serial.print("\n");
        }
#endif
    }
};

#endif /* fs_common_hpp */
