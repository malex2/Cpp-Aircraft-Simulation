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
//#define IMU
#define GPS
//#define BAROMETER
//#define PWM
//#define CONTROLS
//#define GROUND_DETECTION
//#define NAVIGATION
#define PRINT
//#define UBX_PRINT

#ifdef SIMULATION
    #include "model_mapping.hpp"
    #include "arduino_class_models.hpp"
    #include "utilities.hpp"
    #include <iostream>
    #include <iomanip>
    #include <fstream>
    typedef std::string String;
#else
    #include "arduino.h"
    #include <EEPROM.h>
    #include "Wire.h"
    #include <SoftwareSerial.h>
    #ifdef max
        #undef max
        #undef min
    #endif
    #define LEDPIN LED_BUILTIN
#endif

enum I2C_Error_Code {I2C_0_SUCCESS, I2C_1_DATA_TOO_LONG, I2C_2_NACK_ADDRESS, I2C_3_NACK_DATA, I2C_4_OTHER, I2C_5_TIMEOUT};

// Forward References
struct IMUtype;
struct NavType;
struct ControlType;
struct GpsType;

class ModelMap;
class Utilities;

// Types
enum channelType {THROTTLE_CHANNEL, ROLL_CHANNEL, PITCH_CHANNEL, YAW_CHANNEL, nChannels};

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

#define UBX_BUFFER_MAX_SIZE       100
#define UBX_CFG_ON_OFF_SHORT_SIZE 3
#define UBX_CFG_ON_OFF_LONG_SIZE  8

#define UBX_MSG_HEADER_SIZE       2
#define UBX_MSG_CLASS_ID_SIZE     2
#define UBX_MSG_LENGTH_SIZE       2
#define UBX_MSG_CHECKSUM_SIZE     2

// Serial Fifo
#define MAX_SERIAL_FIFO_SIZE 500

// Classes
class FS_FIFO {
public:
    FS_FIFO(HardwareSerial* serialIO);
    ~FS_FIFO();
    
    void begin(int baud_rate);
    
    // Update
    void update_fifo();
    
    // serialIO to Flight Software
    byte read();
    int available();
    unsigned long read_count() { return read_byte_count; }
    unsigned int get_max_read_buffer_length() { return max_read_buffer_length; }
    
    // Flight Software to Serial IO
    int write(byte val);
    int write(byte* val, int length);
    int write_available();
    unsigned long write_count() { return write_byte_count; };
    unsigned int get_max_write_buffer_length() { return max_write_buffer_length; }
    
    // Debug
    void display_read_buffer();
    void display_write_buffer();
private:
    byte read_val;
    byte read_buffer[MAX_SERIAL_FIFO_SIZE];
    unsigned int  read_buffer_index;
    unsigned int  read_buffer_length;
    unsigned long read_byte_count;
    unsigned int  max_read_buffer_length;
    
    byte write_buffer[MAX_SERIAL_FIFO_SIZE];
    unsigned int  write_buffer_index;
    unsigned int  write_buffer_length;
    unsigned long write_byte_count;
    unsigned int  max_write_buffer_length;
    double baud_rate;
    double prevWriteTime;
    
    HardwareSerial* serialIO;
    
    bool read_fifo_full();
    bool write_fifo_full();
    
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
template<typename TempType>
void display(TempType val);
template<typename TempType>
void display(TempType val, int printMode);
void display(I2C_Error_Code val);

// LED
void LEDon();
void LEDoff();

#ifdef SIMULATION
    void FsCommon_setSimulationModels(ModelMap* pMap);
#endif

#endif /* fs_common_hpp */
