//
//  arduino_class_models.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 2/26/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#ifndef arduino_class_models_hpp
#define arduino_class_models_hpp

#include <stdio.h>
#include <iostream>
#include "model_mapping.hpp"
#include "barometer_model.hpp"

enum pinMode {INPUT, OUTPUT};
enum PrintMode {BIN, OCT, DEC, HEX};
const unsigned int LEDPIN = 0;

class SimulationWire {
public:
    SimulationWire();
    
    void begin();
    void beginTransmission(int deviceIn);
    
    void write(int info);
    short read();
    void requestFrom(int address, int numBytes, bool request);
    int available();
    int endTransmission(bool restart = true);
    
    
    void wire_setSimulationModels(ModelMap* pMap, bool print_wire = false);
private:
    class IMUModelBase*       pIMU;
    class AtmosphereModel*    pAtmo;
    class BarometerModelBase* pBaro;
    
    bool active;
    
    // attached I2C devices
    enum deviceType {MPU6050, BMP180, nDevices};
    int devices[nDevices];
    bool deviceAwake[nDevices];
    bool foundDevice;
    deviceType device;
    
    int address;
    bool addressLatch;
    
    // buffer
    byte buffer[100];
    int bufferSize;
    int iBuffer;
    void clearBuffer();
    
    //MPU6050 addresses
    static const int MPU_ADDR   = 0x68;
    static const int PWR_MGMT_1 = 0x6B; // PWR_MGMT_1 register
    static const int GYRO_REG   = 0x1B; // Gryo register
    static const int ACC_REG    = 0x1C; // Accelerometer register
    static const int ACC_OUT    = 0x3B; // (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    static const int INT        = 0x37;
    static const int INT_ENABLE = 0x38;
    
    double MPU6050_gyroByteToLSB(int byte);
    double MPU6050_accByteToLSB(int byte);
    
    //BMP180 addresses
    static const int BMP180_ADDR                = 0x77;
    static const int BMP180_REG_CONTROL         = 0xF4;
    static const int BMP180_REG_RESULT          = 0xF6;
    static const int BMP180_COMMAND_TEMPERATURE = 0x2E;
    static const int BMP180_COMMAND_PRESSURE0   = 0x34;
    static const int BMP180_COMMAND_PRESSURE1   = 0x74;
    static const int BMP180_COMMAND_PRESSURE2   = 0xB4;
    static const int BMP180_COMMAND_PRESSURE3   = 0xF4;
    static const int AC1_ADDR = 0xAA;
    static const int AC2_ADDR = 0xAC;
    static const int AC3_ADDR = 0xAE;
    static const int AC4_ADDR = 0xB0;
    static const int AC5_ADDR = 0xB2;
    static const int AC6_ADDR = 0xB4;
    static const int VB1_ADDR = 0xB6;
    static const int VB2_ADDR = 0xB8;
    static const int MB_ADDR  = 0xBA;
    static const int MC_ADDR  = 0xBC;
    static const int MD_ADDR  = 0xBE;
    
    bmp180::requestStateType bmp180LastRequest;
    
    void BMP180_request2bytes(bmp180::calibrationType calVal, int numBytes);
    
    bool print;
};

class Servo {
public:
    Servo();
    
    void attach(int pinIn);
    
    void writeMicroseconds(unsigned long pwm);
    
    double read();
    
    void servo_setSimulationModels(ModelMap* pMap);
private:
    class ActuatorModelBase* pAct;
    
    // Types
    enum motorNumberType {T1, T2, T3, T4, nMotors};
    
    // Pins
    const unsigned int T1PIN = 6;
    const unsigned int T2PIN = 9;
    const unsigned int T3PIN = 10;
    const unsigned int T4PIN = 11;
    const int PWMMIN = 1000;
    const int PWMMAX = 2000;
    
    // Variables
    unsigned int motorPins[nMotors];
    motorNumberType motorNumber;
    bool foundMotor;
    double throttle;
    
    // Internal Functions
    double mapToValue(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax, double valueMin, double valueMax);
};

class ArduinoSerial {
public:
    ArduinoSerial();
    
    void begin(long baudRate);
};

class SoftwareSerial {
public:
    SoftwareSerial();
    SoftwareSerial(int rx_pin, int tx_pin);

    void begin(int baud);
    byte read();
    int write(byte val);
    int write(byte* val, int length);
    int available();
    
    // slave read/write
    byte slave_read();
    int slave_write(byte val);
    int slave_write(byte* val, int length);
    int slave_available();
    
    // simulation support
    int getRXPin()    { return rx_pin; }
    int getTXPin()    { return tx_pin; }
    int getBaudRate() { return baud_rate; }
    void display_rx_buffer();
    void display_tx_buffer();
    
    void serial_setSimulationModels(ModelMap* pMap, std::string model, bool print = false);
private:
    static const int max_buffer_size = 64;
    static const int max_gps_neo_buffer_size = 700;
    int baud_rate;
    bool baud_begin;
    byte rx_buffer[max_gps_neo_buffer_size];
    int rx_buffer_index;
    int rx_buffer_length;
    int rx_pin;
    
    byte tx_buffer[max_gps_neo_buffer_size];
    int tx_buffer_index;
    int tx_buffer_length;
    int tx_pin;
  
    GenericSensorModel* pModel;
    bool print_serial;
};

void pinMode(int pin, pinMode mode);

#define HardwareSerial SoftwareSerial
extern SimulationWire Wire;
extern SoftwareSerial Serial1;

#endif /* arduino_class_models_hpp */
