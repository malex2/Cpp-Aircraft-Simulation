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

typedef unsigned char byte;
#define LEDPIN 0
enum pinMode {INPUT, OUTPUT};

class SimulationWire {
public:
    SimulationWire();
    
    void begin();
    void beginTransmission(int deviceIn);
    
    void write(int info);
    byte read();
    void requestFrom(int address, int numBytes, bool request);
    void endTransmission(bool restart);
    
    bool available() { return true; }
    
    void wire_setSimulationModels(ModelMap* pMap);
private:
    class IMUModelBase*    pIMU;
    class AtmosphereModel* pAtmo;
    
    bool active;
    
    // attached I2C devices
    enum deviceType {MPU6050, nDevices};
    int devices[nDevices];
    int deviceAwake[nDevices];
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

void pinMode(int pin, pinMode mode);

#endif /* arduino_class_models_hpp */
