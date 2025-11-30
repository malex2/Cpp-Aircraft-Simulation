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
#define SIMULATION
#define IMU
#define GPS
#define BAROMETER
#define PWM
#define CONTROLS
//#define THRUST_ESTIMATOR
#define GROUND_DETECTION
#define NAVIGATION
//#define TELEMETRY
//#define PRINT
//#define UBX_PRINT
//#define TM_PRINT

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

   static Teensy4Pins FS_pins;
   #define pinMode FS_pins.pinMode
   #define digitalWrite FS_pins.digitalWrite

   #define DEBUG_PRINT
#else
    #include "arduino.h"
    #include <EEPROM.h>
    #ifdef max
        #undef max
        #undef min
    #endif
      #define LEDPIN 13

      #define FS_Serial HardwareSerial
      #include "Wire.h"

      //#define DEBUG_PRINT
      //#include <SPI.h>
      #include <nRF24L01.h>
      #include <RF24.h>
#endif

// Forward References
struct IMUtype;
struct BarometerType;
struct GpsType;
struct NavType;
struct ControlType;

// Types
enum I2C_Error_Code {I2C_0_SUCCESS, I2C_1_DATA_TOO_LONG, I2C_2_NACK_ADDRESS, I2C_3_NACK_DATA, I2C_4_OTHER, I2C_5_TIMEOUT};
enum FS_Timing_Type {hz1, hz50, hz100, hz200, hz400, hz800, printRoutine, nRoutines};
enum FS_TM_READ_STATE {FS_TM_HEADER, TM_TYPE_HEADER, READ_TM_LENGTH, READ_TM_BUFFER, CALC_TM_CHECKSUM, TM_READ_COMPLETE};
enum TM_MSG_TYPE {FS_TM_IMU, FS_TM_BARO, FS_TM_GPS, FS_TM_NAV_HIGHRATE, FS_TM_NAV_LOWRATE, FS_TM_CONTROLS, FS_TM_STATUS, FS_PRINT, N_TM_MSGS};
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
const double IMUtoBody[3] = {1.0, -1.0, -1.0};

// Pulse In Pins - TODO
#define THROTTLEPIN  7  // CH3
#define ROLLPIN      8  // CH4
#define PITCHPIN     12 // CH5
#define YAWPIN       13 // CH6

// Pulse Out Pins - TODO
#define T1PIN  6
#define T2PIN  9
#define T3PIN  10
#define T4PIN  11

// High Dynamics
#define highRate  5.0*degree2radian
#define highAccel 12.0

// Telemetry Pins
#define      TM_ENPIN  5 // APC220 - Set high to enable
#define      TM_SETPIN 6 // APC220 - Set low to go into settings mode
#define      APC220_CONFIG_SIZE 19
#define      APC220_GSFK_RATE 10
#define      APC220_BAUD_RATE 14
const double APC220_SETTING_DELAY   = 2.0/1000.0; // 1ms + 1ms buffer
const double APC220_SETTING_TIMEOUT = 300.0/1000.0; // 200ms expected + 100ms buffer

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

// Velocity accuracy for velocity control
#define VELSTDLIM 0.3

// Yaw accuracy for ground align
#define YAWSTDLIM 5.0 * degree2radian

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

// Printing
void display(const char* val, int printMode = DEC);
template<typename TempType>
void display(TempType val, int printMode = DEC);
void display(I2C_Error_Code val, int printMode = DEC);

struct SensorErrorType {
    double sum;
    double mean;
    double max;
    double min;
    double std;
    double sumSqr;
    double variance;
    unsigned int count;
    
    SensorErrorType() { reset(); }
    
    void reset()
    {
        sum = 0.0;
        sumSqr = 0.0;
        mean = 0.0;
        max = -999999.0;
        min = 999999.0;
        std = 0.0;
        variance = 0.0;
        count = 0;
    }
    
    void update(double x)
    {
        sum += x;
        sumSqr += x*x;
        
        if (x > max) { max =  x; }
        if (x < min) { min =  x; }
        
        count++;
    }
    
    void compute()
    {
        mean     = sum/count;
        variance = sumSqr/count - mean*mean;
        std      = sqrt(variance);
    }
    
    void print()
    {
        display("[sum, sumSqr, mean, std, variance, min, max, count] = [");
        display(sum); display(", ");
        display(sumSqr); display(", ");
        display(mean); display(", ");
        display(std); display(", ");
        display(variance); display(", ");
        display(min); display(", ");
        display(max); display(", ");
        display(count); display("]\n");
        
    }
};

// Serial Fifo
#define MAX_SERIAL_FIFO_SIZE 800
#define NRF24L01_BUFFER_SIZE 32

// Classes
class FS_FIFO {
public:
    FS_FIFO(const char* name="");
    FS_FIFO(FS_Serial* serialIO, const char* name="");
    FS_FIFO(RF24* tmIO, const char* name="");
    ~FS_FIFO();
    
    void begin(uint32_t baud_rate);
    void end();
    void set_kwrite(double kwrite);
    
    // Update
    void update_fifo();
    
    // serialIO to Flight Software
    byte read();
    unsigned short available();
    unsigned short get_read_count() { return read_byte_count; }
    unsigned short get_max_read_buffer_length()   { return max_read_buffer_length; }
    unsigned long  get_read_fifo_overflow_count() { return read_fifo_overflow_count; }
    
    // Flight Software to Serial IO
    unsigned short write(byte val);
    unsigned short write(const byte* val, unsigned long length);
    bool           write_available();
    unsigned long  get_write_count() { return write_byte_count; };
    unsigned short get_write_buffer_length() { return write_buffer_length; }
    unsigned short get_max_write_buffer_length() { return max_write_buffer_length; }
    unsigned long  get_write_fifo_overflow_count()   { return write_fifo_overflow_count; }
    unsigned long  get_write_buffer_overflow_count() { return write_buffer_overflow_count; }

    bool read_fifo_full();
    bool write_fifo_full();
    
    // Debug
    void display_read_buffer();
    void display_write_buffer();
private:
    const char* fifo_name;
    
    bool baud_begin;
    
    byte read_val;
    byte read_buffer[MAX_SERIAL_FIFO_SIZE];
    unsigned short read_buffer_index;
    unsigned short read_buffer_length;
    unsigned long read_byte_count;
    unsigned short max_read_buffer_length;
    unsigned long read_fifo_overflow_count;
    
    byte write_buffer[MAX_SERIAL_FIFO_SIZE];
    unsigned short  write_buffer_index;
    unsigned short  write_buffer_length;
    unsigned long write_byte_count;
    unsigned short  max_write_buffer_length;
    double baud_dt;
    double prevWriteTime;
    double kwrite;
    unsigned long write_fifo_overflow_count;
    unsigned long write_buffer_overflow_count;
    
    byte temp_nrf24L01_buffer[NRF24L01_BUFFER_SIZE];
    FS_Serial* serialIO;
    RF24*      nrf24L01;
    
    void reset_read_buffer();
    void reset_write_buffer();
};

// Time
double getTime();

// Memory
void write_eeprom(unsigned int address, byte val);
void write_eeprom(unsigned int address, byte* val, unsigned int size);

void read_eeprom(unsigned int address, byte* val);
void read_eeprom(unsigned int address, byte* val, unsigned int size);

// Math Support
double errorToVariance(double maxError);
double varianceToError(double variance);
void crossProduct(double *cross, const double *a, const double *b);

// FIFO
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
// byte or char for flags
// 59 x 32 bits = 1888 bits = 236 (bytes)
#define TM_HEADER              0xA0
#define TM_IMU_HEADER          0xA1
#define TM_BARO_HEADER         0xA2
#define TM_GPS_HEADER          0xA3
#define TM_NAV_HIGHRATE_HEADER 0xA4
#define TM_NAV_LOWRATE_HEADER  0xA5
#define TM_CONTROL_HEADER      0xA6
#define TM_STATUS_HEADER       0xA7
#define TM_PRINT_HEADER        0xA8
#define FS_TM_HEADER_SIZE   2
#define FS_TM_LENGTH_SIZE   2
#define FS_TM_CHECKSUM_SIZE 2

struct FS_Telemetry_Type
{
    FS_Telemetry_Type() { clear(); disable_all_printing(); }
    
    bool print_bool[N_TM_MSGS];
    bool any_print()
    {
        for (int i=0; i<N_TM_MSGS; i++)
        { if (print_bool[i]) return true; }
        return false;
    }
    
    void enable_all_printing()
    {
        for (int i=0; i<N_TM_MSGS; i++)
        { enable_printing((TM_MSG_TYPE) i); }
    }
    
    void disable_all_printing()
    {
        for (int i=0; i<N_TM_MSGS; i++)
        { disable_printing((TM_MSG_TYPE) i); }
    }
    
    void enable_printing(TM_MSG_TYPE i_TM)  { print_bool[i_TM] = true;}
    void disable_printing(TM_MSG_TYPE i_TM) { print_bool[i_TM] = false; }
    
    // IMU
    struct imu_data_t
    {
        uint32_t fs_time = 0;
        uint32_t imu_timestamp = 0;
        int32_t gyro_x = 0;
        int32_t gyro_y = 0;
        int32_t gyro_z = 0;
        int32_t accel_x = 0;
        int32_t accel_y = 0;
        int32_t accel_z = 0;
        
    } __attribute__((packed)) imu_data;
    
    // Divide by scale when encoding
    // Multiply by scale when decoding
    struct imu_scale_t
    {
        double fs_time = 1.0e-3;
        double imu_timestamp = 1.0e-3;
        double gyro_x = 1.0e-7;
        double gyro_y = 1.0e-7;
        double gyro_z = 1.0e-7;
        double accel_x = 1.0e-2;
        double accel_y = 1.0e-2;
        double accel_z = 1.0e-2;
        
    } __attribute__((packed)) imu_scale;
    
    // BARO
    struct baro_data_t
    {
        uint32_t baro_timestamp = 0;
        uint16_t pressure = 0; //mbar
        int16_t  temperature = 0;
        int32_t  baro_altitude = 0;
        
    }  __attribute__((packed)) baro_data;
    
    struct baro_scale_t
    {
        double baro_timestamp = 1.0e-3;
        double pressure = 1.0e-2;
        double temperature = 1.0e-2;
        double baro_altitude = 1.0e-3;
        
    }  __attribute__((packed)) baro_scale;
    
    // GPS
    struct gps_data_t
    {
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
        
    }  __attribute__((packed)) gps_data;
    
    struct gps_scale_t
    {
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
        
    }  __attribute__((packed)) gps_scale;
    
    // NAV Highrate
    struct nav_highrate_data_t
    {
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
        int32_t  accelerometer_pitch = 0;
        int32_t  accelerometer_roll = 0;
        
    }  __attribute__((packed)) nav_highrate_data;
    
    struct nav_highrate_scale_t
    {
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
        double accelerometer_pitch = 1.0e-7;
        double accelerometer_roll = 1.0e-7;
        
    }  __attribute__((packed)) nav_highrate_scale;

    // NAV Lowrate
    struct nav_lowrate_data_t
    {
        uint32_t nav_timestamp = 0;
        int32_t  accel_bias_x = 0;
        int32_t  accel_bias_y = 0;
        int32_t  accel_bias_z = 0;
        int32_t  gyro_bias_x = 0;
        int32_t  gyro_bias_y = 0;
        int32_t  gyro_bias_z = 0;
        int32_t  gravity = 0;
        uint32_t ins_update_count = 0;
        uint32_t baro_filter_count = 0;
        uint32_t gps_filter_count = 0;
        uint32_t accel_filter_count = 0;
        
    }  __attribute__((packed)) nav_lowrate_data;
    
    struct nav_lowrate_scale_t
    {
        double nav_timestamp = 1.0e-3;
        double accel_bias_x = 1.0e-2;
        double accel_bias_y = 1.0e-2;
        double accel_bias_z = 1.0e-2;
        double gyro_bias_x = 1.0e-7;
        double gyro_bias_y = 1.0e-7;
        double gyro_bias_z = 1.0e-7;
        double gravity = 1.0e-2;
        double ins_update_count = 1.0;
        double baro_filter_count = 1.0;
        double gps_filter_count = 1.0;
        double accel_filter_count = 1.0;
        
    }  __attribute__((packed)) nav_lowrate_scale;
    
    // CONTROL
    struct control_data_t
    {
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
        
    }  __attribute__((packed)) control_data;
    
    struct control_scale_t
    {
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
        
    }  __attribute__((packed)) control_scale;
    
    // STATUS
    struct status_data_t
    {
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
        uint16_t hz400_avg_rate = 0;
        uint16_t hz800_avg_rate = 0;
        
    }  __attribute__((packed)) status_data;
    
    struct status_scale_t
    {
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
        double hz400_avg_rate = 0.1;
        double hz800_avg_rate = 0.1;
        
    }  __attribute__((packed)) status_scale;
    
    void clear()
    {
        memset(&imu_data, 0, sizeof(imu_data));
        memset(&baro_data, 0, sizeof(baro_data));
        memset(&gps_data, 0, sizeof(gps_data));
        memset(&nav_highrate_data, 0, sizeof(nav_highrate_data));
        memset(&nav_lowrate_data, 0, sizeof(nav_lowrate_data));
        memset(&control_data, 0, sizeof(control_data));
        memset(&status_data, 0, sizeof(status_data));
    }
    
    void print()
    {
        if (any_print())
        {
            Serial.print("FS Time: ");
            Serial.println(imu_data.fs_time * imu_scale.fs_time);
            Serial.print("\n");
        }
        
        if (print_bool[FS_TM_IMU])
        {
            Serial.print("IMU I2C code: ");
            Serial.println(status_data.imuI2CErrorCode * status_scale.imuI2CErrorCode);
            
            Serial.print("gyro (deg/s): [");
            Serial.print(imu_data.gyro_x * imu_scale.gyro_x * radian2degree); Serial.print(", ");
            Serial.print(imu_data.gyro_y * imu_scale.gyro_y * radian2degree); Serial.print(", ");
            Serial.print(imu_data.gyro_z * imu_scale.gyro_z * radian2degree); Serial.println("]");
            
            Serial.print("accel (g): [");
            Serial.print(imu_data.accel_x * imu_scale.accel_x / (Gravity)); Serial.print(", ");
            Serial.print(imu_data.accel_y * imu_scale.accel_y / (Gravity)); Serial.print(", ");
            Serial.print(imu_data.accel_z * imu_scale.accel_z / (Gravity)); Serial.println("]");
            Serial.print("\n");
        }
        
        if (print_bool[FS_TM_BARO])
        {
            Serial.print("Baro I2C code: ");
            Serial.println(status_data.baroI2CErrorCode * status_scale.baroI2CErrorCode);
            
            Serial.print("Baro [t, pres, temp, alt]: [");
            Serial.print(baro_data.baro_timestamp * baro_scale.baro_timestamp); Serial.print(", ");
            Serial.print(baro_data.pressure * baro_scale.pressure); Serial.print(", ");
            Serial.print(baro_data.temperature * baro_scale.temperature); Serial.print(", ");
            Serial.print(baro_data.baro_altitude * baro_scale.baro_altitude); Serial.println("]");
            Serial.print("\n");
        }

        if (print_bool[FS_TM_GPS])
        {
            Serial.print("GPS [t, rcv_t, ttff]: [");
            Serial.print(gps_data.gps_timestamp * gps_scale.gps_timestamp); Serial.print(", ");
            Serial.print(gps_data.gps_receiver_time * gps_scale.gps_receiver_time); Serial.print(", ");
            Serial.print(gps_data.gps_ttff * gps_scale.gps_ttff); Serial.println("]");

            Serial.print("GPS LLH: [");
            Serial.print(gps_data.gps_lat * gps_scale.gps_lat); Serial.print(", ");
            Serial.print(gps_data.gps_lon * gps_scale.gps_lon); Serial.print(", ");
            Serial.print(gps_data.gps_alt * gps_scale.gps_alt); Serial.println("]");

            Serial.print("GPS Pos NEU: [");
            Serial.print(gps_data.gps_pos_north * gps_scale.gps_pos_north); Serial.print(", ");
            Serial.print(gps_data.gps_pos_east * gps_scale.gps_pos_east); Serial.print(", ");
            Serial.print(gps_data.gps_pos_up * gps_scale.gps_pos_up); Serial.println("]");
            
            Serial.print("GPS Vel NED: [");
            Serial.print(gps_data.gps_vel_n * gps_scale.gps_vel_n); Serial.print(", ");
            Serial.print(gps_data.gps_vel_e * gps_scale.gps_vel_e); Serial.print(", ");
            Serial.print(gps_data.gps_vel_d * gps_scale.gps_vel_d); Serial.println("]");
            
            Serial.print("GPS Acc [horiz, ver, speed]: [");
            Serial.print(gps_data.gps_horiz_pos_acc * gps_scale.gps_horiz_pos_acc); Serial.print(", ");
            Serial.print(gps_data.gps_vert_pos_acc * gps_scale.gps_vert_pos_acc); Serial.print(", ");
            Serial.print(gps_data.gps_speed_acc * gps_scale.gps_speed_acc); Serial.println("]");

            Serial.print("GPS Acc [rcvd_msg, fifo_overflow_count]: [");
            Serial.print(gps_data.gps_rcvd_msg_count * gps_scale.gps_rcvd_msg_count); Serial.print(", ");
            Serial.print(gps_data.gps_read_fifo_overflow_count * gps_scale.gps_read_fifo_overflow_count); Serial.println("]");
     
            Serial.print("GPS Acc [gps_fixOk, numSV]: [");
            Serial.print(status_data.gps_fixOk * status_scale.gps_fixOk); Serial.print(", ");
            Serial.print(status_data.numSV * status_scale.numSV); Serial.println("]");
            Serial.print("\n");
        }
        
        if (print_bool[FS_TM_NAV_HIGHRATE])
        {
            Serial.print("Nav Euler: [");
            Serial.print(nav_highrate_data.roll * nav_highrate_scale.roll * radian2degree); Serial.print(", ");
            Serial.print(nav_highrate_data.pitch * nav_highrate_scale.pitch * radian2degree); Serial.print(", ");
            Serial.print(nav_highrate_data.yaw * nav_highrate_scale.yaw * radian2degree); Serial.println("]");
            
            Serial.print("Nav Vel NED: [");
            Serial.print(nav_highrate_data.nav_vel_n * nav_highrate_scale.nav_vel_n); Serial.print(", ");
            Serial.print(nav_highrate_data.nav_vel_e * nav_highrate_scale.nav_vel_e); Serial.print(", ");
            Serial.print(nav_highrate_data.nav_vel_d * nav_highrate_scale.nav_vel_d); Serial.println("]");
            
            Serial.print("Nav Pos NEU: [");
            Serial.print(nav_highrate_data.nav_pos_n * nav_highrate_scale.nav_pos_n); Serial.print(", ");
            Serial.print(nav_highrate_data.nav_pos_e * nav_highrate_scale.nav_pos_e); Serial.print(", ");
            Serial.print(nav_highrate_data.nav_alt * nav_highrate_scale.nav_alt); Serial.println("]");
            
            Serial.print("Nav [accel_roll, accel_pitch]: [");
            Serial.print(nav_highrate_data.accelerometer_roll * nav_highrate_scale.accelerometer_roll * radian2degree); Serial.print(", ");
            Serial.print(nav_highrate_data.accelerometer_pitch * nav_highrate_scale.accelerometer_pitch * radian2degree); Serial.println("]");
            Serial.print("\n");
        }

        if (print_bool[FS_TM_NAV_LOWRATE])
        {
            Serial.print("Nav Accel Bias (g): [");
            Serial.print(nav_lowrate_data.accel_bias_x * nav_lowrate_scale.accel_bias_x / (Gravity)); Serial.print(", ");
            Serial.print(nav_lowrate_data.accel_bias_y * nav_lowrate_scale.accel_bias_y / (Gravity)); Serial.print(", ");
            Serial.print(nav_lowrate_data.accel_bias_z * nav_lowrate_scale.accel_bias_z / (Gravity)); Serial.println("]");
            
            Serial.print("Nav Gyro Bias (deg/s): [");
            Serial.print(nav_lowrate_data.gyro_bias_x * nav_lowrate_scale.gyro_bias_x * radian2degree); Serial.print(", ");
            Serial.print(nav_lowrate_data.gyro_bias_y * nav_lowrate_scale.gyro_bias_y * radian2degree); Serial.print(", ");
            Serial.print(nav_lowrate_data.gyro_bias_z * nav_lowrate_scale.gyro_bias_z * radian2degree); Serial.println("]");
            
            Serial.print("gravity: ");
            Serial.print(nav_lowrate_data.gravity * nav_lowrate_scale.gravity); Serial.println("]");
            
            Serial.print("Nav [ins, accel, baro, gps] filer count: [");
            Serial.print(nav_lowrate_data.ins_update_count * nav_lowrate_scale.ins_update_count); Serial.print(", ");
            Serial.print(nav_lowrate_data.accel_filter_count * nav_lowrate_scale.accel_filter_count); Serial.print(", ");
            Serial.print(nav_lowrate_data.baro_filter_count * nav_lowrate_scale.baro_filter_count); Serial.print(", ");
            Serial.print(nav_lowrate_data.gps_filter_count * nav_lowrate_scale.gps_filter_count); Serial.println("]");
            Serial.print("\n");
        }
        
        if (print_bool[FS_TM_CONTROLS])
        {
            Serial.print("PWM In: [");
            Serial.print(control_data.throttle_channel_pwm * control_scale.throttle_channel_pwm); Serial.print(", ");
            Serial.print(control_data.roll_channel_pwm * control_scale.roll_channel_pwm); Serial.print(", ");
            Serial.print(control_data.pitch_channel_pwm * control_scale.pitch_channel_pwm); Serial.print(", ");
            Serial.print(control_data.yaw_channel_pwm * control_scale.yaw_channel_pwm); Serial.println("]");
            
            Serial.print("Prop PWM: [");
            Serial.print(control_data.prop1_pwm * control_scale.prop1_pwm); Serial.print(", ");
            Serial.print(control_data.prop2_pwm * control_scale.prop2_pwm); Serial.print(", ");
            Serial.print(control_data.prop3_pwm * control_scale.prop3_pwm); Serial.print(", ");
            Serial.print(control_data.prop4_pwm * control_scale.prop4_pwm); Serial.println("]");
            
            Serial.print("Ctrl Euler Cmd: [");
            Serial.print(control_data.rollCmd * control_scale.rollCmd * radian2degree); Serial.print(", ");
            Serial.print(control_data.pitchCmd * control_scale.pitchCmd * radian2degree); Serial.print(", ");
            Serial.print(control_data.yawRateCmd * control_scale.yawRateCmd * radian2degree); Serial.println("]");
            
            Serial.print("Ctrl Vel Cmd: [");
            Serial.print(control_data.VLLxCmd * control_scale.VLLxCmd); Serial.print(", ");
            Serial.print(control_data.VLLyCmd * control_scale.VLLyCmd); Serial.print(", ");
            Serial.print(control_data.VLLzCmd * control_scale.VLLzCmd); Serial.println("]");
            
            Serial.print("Ctrl [mode, onGround, takeOff, crashLand]: [");
            Serial.print(status_data.ctrl_mode * status_scale.ctrl_mode); Serial.print(", ");
            Serial.print(status_data.onGround * status_scale.onGround); Serial.print(", ");
            Serial.print(status_data.takeOff * status_scale.takeOff); Serial.print(", ");
            Serial.print(status_data.crashLand * status_scale.crashLand); Serial.println("]");
            Serial.print("\n");
        }
        
         if (print_bool[FS_TM_STATUS])
         {
             Serial.print("FS Rates = [1hz, 50hz, 100hz, 200hz, 400hz, 800hz]: [");
             Serial.print(status_data.hz1_avg_rate * status_scale.hz1_avg_rate); Serial.print(", ");
             Serial.print(status_data.hz50_avg_rate * status_scale.hz50_avg_rate); Serial.print(", ");
             Serial.print(status_data.hz100_avg_rate * status_scale.hz100_avg_rate); Serial.print(", ");
             Serial.print(status_data.hz200_avg_rate * status_scale.hz200_avg_rate); Serial.print(", ");
             Serial.print(status_data.hz400_avg_rate * status_scale.hz400_avg_rate); Serial.print(", ");
             Serial.print(status_data.hz800_avg_rate * status_scale.hz800_avg_rate); Serial.println("]");
             
             Serial.print("[max_tm_rate, tm_timeout_count]: [");
             Serial.print(status_data.max_tm_rate * status_scale.max_tm_rate); Serial.print(", ");
             Serial.println(status_data.tm_timeout_count * status_scale.tm_timeout_count);
             Serial.print("]\n");
         }
        
    }
};

#endif /* fs_common_hpp */
