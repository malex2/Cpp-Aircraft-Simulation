//
//  arduino_class_models.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 2/26/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "arduino_class_models.hpp"
#include "imu_model.hpp"
#include "atmosphere_model.hpp"
#include "actuator_model.hpp"

SimulationWire::SimulationWire()
{
    active       = false;
    foundDevice  = false;
    addressLatch = false;
    address      = -1;
    
    bufferSize = 0;
    iBuffer    = 0;
    
    pIMU  = 0;
    pAtmo = 0;
    
    devices[MPU6050] = MPU_ADDR;
    device = nDevices;
    
    for (int iDevice = MPU6050; iDevice != nDevices; iDevice++)
    {
        deviceAwake[iDevice] = false;
    }
    
}

void SimulationWire::begin()
{
     active = true;
}

void SimulationWire::beginTransmission(int deviceIn)
{
    if (!active) { return; }
    
    for (int iDevice = MPU6050; iDevice != nDevices; iDevice++)
    {
        if (!foundDevice && address == devices[iDevice])
        {
            device = static_cast<deviceType> (iDevice);
            foundDevice = true;
            std::cout << "Device found: " << device << std::endl;
        }
    }
}

void SimulationWire::write(int info)
{
    if (!active) { return; }
    
    if (!foundDevice)
    {
        std::cout << "Writing before address set!" << std::endl;
        return;
    }
    
    if (!addressLatch)
    {
        address = info;
        addressLatch = true;
        std::cout << "Address change: " << address << std::endl;
        return;
    }
    else
    {
        std::cout << "Writing to address " << address << ": " << info << std::endl;
    }
    
    switch (device) {
        case nDevices :
            std::cout << "No device for writing!" << std::endl;
            break;
            
        case MPU6050 :
            if (address == PWR_MGMT_1 )
            {
                if (info == 0) { deviceAwake[device] = true; }
            }
                    
            else if (address == GYRO_REG )
            {
                double LSBdps = MPU6050_gyroByteToLSB(info);
                if (pIMU) pIMU->setLSBdps(LSBdps);
            }
            
            else if (address == ACC_REG)
            {
                double LSBg = MPU6050_accByteToLSB(info);;
                if (pIMU) pIMU->setLSBg(LSBg);
            }
            break;
    }
}

byte SimulationWire::read()
{
    if (!active) { return 0; }
    
    int val = buffer[iBuffer];
    iBuffer++;
    return val;
}

void SimulationWire::requestFrom(int address, int numBytes, bool end)
{
    if (!active) { return; }
    
    switch (device) {
        case nDevices :
            std::cout << "No device to requestFrom!" << std::endl;
            break;
            
        case MPU6050 :
            
            if(address == ACC_OUT)
            {
                int rawGyro[3] = {0,0,0};
                int rawAcc[3]  = {0,0,0};
                int rawTemp    = 0;
                
                if (pIMU)
                {
                    for (int i=0; i<3; i++)
                    {
                        rawGyro[i] = static_cast<int> ( pIMU->getGyroscope()[i] );
                        rawAcc[i]  = static_cast<int> ( pIMU->getAccelerometer()[i] );
                    }
                }
                
                if (pAtmo)
                {
                    double tempC = pAtmo->getAir()[AtmosphereModel::temp] - 273.15;
                    rawTemp = static_cast<int> ( (tempC - 36.53) * 349 );
                }
                
                buffer[0] = (byte) (rawAcc[0] & 0xFF);
                buffer[1] = (byte) ((rawAcc[0] >> 8) & 0xFF);
                buffer[2] = (byte) (rawAcc[1] & 0xFF);
                buffer[3] = (byte) ((rawAcc[1] >> 8) & 0xFF);
                buffer[4] = (byte) (rawAcc[2] & 0xFF);
                buffer[5] = (byte) ((rawAcc[2] >> 8) & 0xFF);
                
                buffer[6] = (byte) (rawTemp & 0xFF);
                buffer[7] = (byte) ((rawTemp >> 8) & 0xFF);
                
                buffer[8] = (byte) (rawGyro[0] & 0xFF);
                buffer[9] = (byte) ((rawGyro[0] >> 8) & 0xFF);
                buffer[10] = (byte) (rawGyro[1] & 0xFF);
                buffer[11] = (byte) ((rawGyro[1] >> 8) & 0xFF);
                buffer[12] = (byte) (rawGyro[2] & 0xFF);
                buffer[13] = (byte) ((rawGyro[2] >> 8) & 0xFF);
                
                bufferSize = 14;
                
                // check
                int check = buffer[4] << 8 | buffer[5];
                std::cout << "Buffer check: ";
                std::cout << rawAcc[2]  << " ";
                std::cout << check << std::endl;
                
            }
            break;
    }

    if (end)
    {
        endTransmission(true);
    }
}

void SimulationWire::endTransmission(bool restart)
{
    if (!active) { return; }
    
    if (restart)
    {
        device = nDevices;
        foundDevice = false;
        addressLatch = false;
        std::cout << "Device released: " << device << std::endl;
    }
}

void SimulationWire::clearBuffer()
{
    if (!active) { return; }
    
    for (int i=0; i<bufferSize; i++)
    {
        buffer[i] = 0;
    }
    bufferSize = 0;
    iBuffer = 0;
}

double SimulationWire::MPU6050_gyroByteToLSB(int byte)
{
    double LSBdps = 131;
    if      (byte == 0b00000000) { LSBdps = 131;  }
    else if (byte == 0b00001000) { LSBdps = 65.5; }
    else if (byte == 0b00010000) { LSBdps = 32.8; }
    else if (byte == 0b00011000) { LSBdps = 16.4; }
    else { std::cout << "Invalid LSPdps byte: " << byte << std::endl; }
    return LSBdps;
}

double SimulationWire::MPU6050_accByteToLSB(int byte)
{
    double LSBg = 16483;
    if      (byte == 0b00000000) { LSBg = 16483; }
    else if (byte == 0b00001000) { LSBg = 8192;  }
    else if (byte == 0b00010000) { LSBg = 4096;  }
    else if (byte == 0b00011000) { LSBg = 2048;  }
    else { std::cout << "Invalid LSBg byte: " << byte << std::endl; }
    return LSBg;
}

void SimulationWire::wire_setSimulationModels(ModelMap* pMap)
{
    if (pMap)
    {
        pIMU  = (IMUModelBase*)    pMap->getModel("IMUModel");
        pAtmo = (AtmosphereModel*) pMap->getModel("AtmosphereModel");
    }
}

Servo::Servo()
{
    motorPins[T1] = T1PIN;
    motorPins[T2] = T2PIN;
    motorPins[T3] = T3PIN;
    motorPins[T4] = T4PIN;
    
    pAct = 0;
    throttle = 0.0;
    foundMotor = false;
}

void Servo::attach(int pinIn)
{
    // Find Channel
    for (int iMotor = T1; iMotor != nMotors; iMotor++)
    {
        if (!foundMotor && pinIn == motorPins[iMotor])
        {
            motorNumber = static_cast<motorNumberType> (iMotor);
            foundMotor = true;
        }
    }
    
    if (!foundMotor)
    {
        std::cout << "Servo::attach, motor not found." << std::endl;
    }
}

void Servo::writeMicroseconds(unsigned long pwm)
{
    throttle = mapToValue(pwm, PWMMIN, PWMMAX, 0.0, 1.0);
    if (pAct && foundMotor)
    {
        pAct->setCommands(throttle, motorNumber);
    }
}

double Servo::read()
{
    return throttle;
}

double Servo::mapToValue(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax, double valueMin, double valueMax)
{
    // y - y1 = m * (x - x1)
    double slope = (valueMax-valueMin) / static_cast<double>(pwmMax-pwmMin);
    return slope*(pwm-pwmMin) + valueMin;
}

void Servo::servo_setSimulationModels(ModelMap* pMap)
{
    if (pMap)
    {
        pAct  = (ActuatorModelBase*) pMap->getModel("ActuatorModel");
    }
}

ArduinoSerial::ArduinoSerial() { }

void ArduinoSerial::begin(long baudRate) { }

void pinMode(int pin, enum pinMode mode) { }
