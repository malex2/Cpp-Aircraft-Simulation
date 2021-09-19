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
    pBaro = 0;
    
    devices[MPU6050] = MPU_ADDR;
    devices[BMP180]  = BMP180_ADDR;
    device = nDevices;
    
    for (int iDevice = MPU6050; iDevice != nDevices; iDevice++)
    {
        deviceAwake[iDevice] = false;
    }
    
    bmp180LastRequest = bmp180::NOREQUEST;
    
    print = false;
}

void SimulationWire::begin()
{
     active = true;
}

void SimulationWire::beginTransmission(int deviceIn)
{
    if (!active) { return; }
    
    if (foundDevice && device != deviceIn)
    {
        std::cout << "Cannot transmit to " << deviceIn << ", currently in use by " << device << std::endl;
    }
    
    for (int iDevice = MPU6050; iDevice != nDevices; iDevice++)
    {
        if (!foundDevice && deviceIn == devices[iDevice])
        {
            device = static_cast<deviceType> (iDevice);
            foundDevice = true;
            if (print) { std::cout << "Device found: " << device << std::endl; }
        }
    }
    
    // bmp180 always awake once connected
    if (device == BMP180 && !deviceAwake[device]) { deviceAwake[device] = true; }
}

void SimulationWire::write(int info)
{
    if (!active) { return; }
    
    if (!foundDevice)
    {
        if (print) { std::cout << "Writing before address set!" << std::endl; }
        return;
    }
    
    if (!addressLatch)
    {
        address = info;
        addressLatch = true;
        if (print) { std::cout << "Address change: " << address << std::endl; }
        return;
    }
    else
    {
        if (print) { std::cout << "Writing to address " << address << ": " << info << std::endl; }
    }
    
    switch (device) {
        case nDevices :
            std::cout << "No device for writing!" << std::endl;
            break;
            
        case MPU6050 :
            if (address == PWR_MGMT_1)
            {
                if (info == 0) { deviceAwake[device] = true; }
            }
                    
            else if (address == GYRO_REG)
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
            
        case BMP180:
            if (address == BMP180_REG_CONTROL)
            {
                if (info == BMP180_COMMAND_TEMPERATURE)
                {
                    static_cast<bmp180*> (pBaro)->startTemperatureReading();
                }
                else if (info == BMP180_COMMAND_PRESSURE0)
                {
                    static_cast<bmp180*> (pBaro)->setPressureNoise(0);
                    static_cast<bmp180*> (pBaro)->startPressureReading();
                }
                else if (info == BMP180_COMMAND_PRESSURE1)
                {
                    static_cast<bmp180*> (pBaro)->setPressureNoise(1);
                    static_cast<bmp180*> (pBaro)->startPressureReading();
                }
                else if (info == BMP180_COMMAND_PRESSURE2)
                {
                    static_cast<bmp180*> (pBaro)->setPressureNoise(2);
                    static_cast<bmp180*> (pBaro)->startPressureReading();
                }
                else if (info == BMP180_COMMAND_PRESSURE3)
                {
                    static_cast<bmp180*> (pBaro)->setPressureNoise(3);
                    static_cast<bmp180*> (pBaro)->startPressureReading();
                }
            }
            break;
    }
}

short SimulationWire::read()
{
    if (!active) { return 0; }
    
    short val = buffer[iBuffer];
    iBuffer++;
    if (iBuffer == bufferSize) { clearBuffer(); }
    
    return val;
}

void SimulationWire::requestFrom(int deviceWrite, int numBytes, bool restart)
{
    if (!active) { return; }
    
    if (!addressLatch) { std::cout << "Requesting from unknown I2C address" << std::endl; return; }
    
    if (deviceWrite != devices[device] )
    {
        std::cout << "Cannot request from " << deviceWrite << ", currently latched to " << device << "." << std::endl;
    }
    
    switch (device) {
        case nDevices :
            std::cout << "No device to requestFrom!" << std::endl;
            break;
            
        case MPU6050 :
            
            if(address == ACC_OUT)
            {
                short rawGyro[3] = {0,0,0};
                short rawAcc[3]  = {0,0,0};
                short rawTemp    = 0;
                
                if (pIMU)
                {
                    for (int i=0; i<3; i++)
                    {
                        rawGyro[i] = static_cast<short> ( pIMU->getGyroscope()[i] );
                        rawAcc[i]  = static_cast<short> ( pIMU->getAccelerometer()[i] );
                    }
                    // rawTemp = static_cast<short>  ( pIMU->getTemperature() );
                }
                
                if (pAtmo)
                {
                    double tempC = pAtmo->getAir()[AtmosphereModel::temp] - 273.15;
                    rawTemp = static_cast<short> ( (tempC - 36.53) * 349 );
                }
                
                buffer[0] = (byte) ((rawAcc[0] & 0xFF00) >> 8);
                buffer[1] = (byte) (rawAcc[0] & 0x00FF);
                buffer[2] = (byte) ((rawAcc[1] & 0xFF00) >> 8);
                buffer[3] = (byte) (rawAcc[1] & 0x00FF);
                buffer[4] = (byte) ((rawAcc[2] & 0xFF00) >> 8);
                buffer[5] = (byte) (rawAcc[2] & 0x00FF);
                
                buffer[6] = (byte) ((rawTemp & 0xFF00) >> 8);
                buffer[7] = (byte) (rawTemp & 0x00FF);
                
                buffer[8] = (byte)  ((rawGyro[0] & 0xFF00) >> 8);
                buffer[9] = (byte)  (rawGyro[0] & 0x00FF);
                buffer[10] = (byte) ((rawGyro[1] & 0xFF00) >> 8);
                buffer[11] = (byte) (rawGyro[1] & 0x00FF);
                buffer[12] = (byte) ((rawGyro[2] & 0xFF00) >> 8);
                buffer[13] = (byte) (rawGyro[2] & 0x00FF);
                
                bufferSize = numBytes;
                
                // check
                //short check = buffer[4] << 8 | buffer[5];
                //std::cout << "Buffer check: ";
                //std::cout << rawAcc[2]  << " ";
                //std::cout << check << std::endl;
            }
            break;
            
        case BMP180:
            
            if (address == BMP180_REG_RESULT)
            {
                bmp180LastRequest = static_cast<bmp180*> (pBaro)->getLastRequest();
                
                if (bmp180LastRequest == bmp180::TEMPERATURE)
                {
                    int rawTemp = 0;
                    
                    if (pBaro) { rawTemp = pBaro->getTemperature(); }

                    buffer[0] = (byte) ((rawTemp & 0xFF00) >> 8);
                    buffer[1] = (byte) (rawTemp & 0x00FF);
                    
                    bufferSize = numBytes;
                }
                else if (bmp180LastRequest == bmp180::PRESSURE)
                {
                    float rawPres = pBaro->getPressure();
                    short intRawPres = (short) rawPres;
                    
                    buffer[0] = (byte) ((intRawPres & 0xFF00) >> 8);
                    buffer[1] = (byte) (intRawPres & 0x00FF);
                    buffer[2] = (byte) ( (rawPres-intRawPres)*256.0 );
                    
                    bufferSize = numBytes;
                }
            }
            else if (address == AC1_ADDR)
            {
                BMP180_request2bytes(bmp180::AC1, numBytes);
            }
            else if (address == AC2_ADDR)
            {
                BMP180_request2bytes(bmp180::AC2, numBytes);
            }
            else if (address == AC3_ADDR)
            {
                BMP180_request2bytes(bmp180::AC3, numBytes);
            }
            else if (address == AC4_ADDR)
            {
                BMP180_request2bytes(bmp180::AC4, numBytes);
            }
            else if (address == AC5_ADDR)
            {
                BMP180_request2bytes(bmp180::AC5, numBytes);
            }
            else if (address == AC6_ADDR)
            {
                BMP180_request2bytes(bmp180::AC6, numBytes);
            }
            else if (address == VB1_ADDR)
            {
                BMP180_request2bytes(bmp180::VB1, numBytes);
            }
            else if (address == VB2_ADDR)
            {
                BMP180_request2bytes(bmp180::VB2, numBytes);
            }
            else if (address == MB_ADDR)
            {
                BMP180_request2bytes(bmp180::MB, numBytes);
            }
            else if (address == MC_ADDR)
            {
                BMP180_request2bytes(bmp180::MC, numBytes);
            }
            else if (address == MD_ADDR)
            {
                BMP180_request2bytes(bmp180::MD, numBytes);
            }
            break;
    }

    endTransmission(restart);
}


int SimulationWire::available()
{
    return bufferSize-iBuffer;
}

int SimulationWire::endTransmission(bool restart)
{
    if (!active) { return 4; }
    if (!foundDevice) { return 2;}
    
    if (restart)
    {
        if (print) { std::cout << "Device released: " << device << std::endl; }
        device = nDevices;
        foundDevice = false;
        addressLatch = false;
    }
    
    return 0;
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

void SimulationWire::BMP180_request2bytes(bmp180::calibrationType calVal, int numBytes)
{
    int rawCal = ( static_cast<bmp180*> (pBaro)->getCalibrationParameter()[calVal] );
    buffer[0] = (byte) ((rawCal & 0xFF00) >> 8);
    buffer[1] = (byte) (rawCal & 0x00FF);
    bufferSize = numBytes;
}

void SimulationWire::wire_setSimulationModels(ModelMap* pMap, bool print_wire)
{
    print = print_wire;
    if (pMap)
    {
        pIMU  = (IMUModelBase*)       pMap->getModel("IMUModel");
        pAtmo = (AtmosphereModel*)    pMap->getModel("AtmosphereModel");
        pBaro = (BarometerModelBase*) pMap->getModel("BarometerModel");
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

SimulationWire Wire = SimulationWire();
