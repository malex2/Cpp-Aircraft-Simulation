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
#include "gps_model.hpp"
#include "time.hpp"

#include <fstream>

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
    double LSBdps = 131.0;
    if      (byte == 0b00000000) { LSBdps = 131.0;  }
    else if (byte == 0b00001000) { LSBdps = 65.5; }
    else if (byte == 0b00010000) { LSBdps = 32.8; }
    else if (byte == 0b00011000) { LSBdps = 16.4; }
    else { std::cout << "Invalid LSPdps byte: " << byte << std::endl; }
    return LSBdps;
}

double SimulationWire::MPU6050_accByteToLSB(int byte)
{
    double LSBg = 16483.0;
    if      (byte == 0b00000000) { LSBg = 16483.0; }
    else if (byte == 0b00001000) { LSBg = 8192.0;  }
    else if (byte == 0b00010000) { LSBg = 4096.0;  }
    else if (byte == 0b00011000) { LSBg = 2048.0;  }
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

SimulationSerial::SimulationSerial(int rx_pin, int tx_pin) : enable_pin(true), config_pin(false)
{
    baud_rate  = 0;
    baud_begin = false;
    this->rx_pin = rx_pin;
    this->tx_pin = tx_pin;
    pModel = 0;
    print_serial = false;
}

SimulationSerial::~SimulationSerial()
{
    end();
}

void SimulationSerial::begin(int baud)
{
    rx_buffer = new byte[max_buffer_size];
    reset_rx_buffer();
    
    tx_buffer = new byte[max_buffer_size];
    reset_tx_buffer();
    
    baud_rate = baud;
    baud_begin = true;
}

void SimulationSerial::end()
{
    if (rx_buffer)
    {
        reset_rx_buffer();
        delete[] rx_buffer; rx_buffer = 0;
    }
    
    if (tx_buffer)
    {
        reset_tx_buffer();
        delete[] tx_buffer; tx_buffer = 0;
    }
    
    baud_rate = 0;
    baud_begin = false;
}

template<typename TempType>
void SimulationSerial::print(TempType val, PrintMode printMode)
{
    if (printMode == HEX)
    {
        std::cout << std::hex << std::setfill('0') << std::setw(2) << val;
        std::cout << std::dec;
    }
    else
    {
        std::cout << val;
    }
}

template<typename TempType>
void SimulationSerial::println(TempType val, PrintMode printMode)
{
    print(val, printMode);
    std::cout << std::endl;
}

byte SimulationSerial::read()
{
    byte val;
    if (!IsEnabled()) { return 0; }
    
    if (available())
    {
        val = rx_buffer[rx_buffer_index];
        rx_buffer_index++;
        if (!available()) { reset_rx_buffer(); }
        return val;
    }
    else
    {
        return 0;
    }
}

byte SimulationSerial::slave_read()
{
    byte val;
    if (!IsEnabled()) { return 0; }
    
    if (slave_available())
    {
        val = tx_buffer[tx_buffer_index];
        tx_buffer_index++;
        if (!slave_available()) { reset_tx_buffer(); }
        return val;
    }
    else
    {
        return 0;
    }
}

int SimulationSerial::write(byte val)
{
    if (!IsEnabled()) { return 0; }
/*
    if (config_pin.HW_Pins)
    {
        std::cout << serial_type << " pin " << config_pin.pin << " (" << config_pin.HW_Pins->serial_type << ") ";
        std::cout << config_pin.HW_Pins->periphialDigitalRead(config_pin.pin) << std::endl;
    }
    */
    if (IsInConfig())
    {
        return handleConfig(val);
    }
    
    if (rx_pin == 0 && tx_pin == 0)
    {
        std::cout << val;
        return 1;
    }
    else if (tx_buffer_length < max_buffer_size)
    {
        tx_buffer[tx_buffer_length] = val;
        tx_buffer_length++;
        return 1;
    }
    else
    {
        return 0;
    }
}

int SimulationSerial::write(const byte* val, int length)
{
    int tx_count;
    if (!IsEnabled()) { return 0; }
    
    tx_count = 0;
    for (int i = 0; i < length; i++)
    {
        tx_count += write(val[i]);
    }
    return tx_count;
}

int SimulationSerial::slave_write(byte val)
{
    if (!IsEnabled()) { return 0; }
    
    if (rx_buffer_length < max_buffer_size)
    {
        rx_buffer[rx_buffer_length] = val;
        rx_buffer_length++;
        return 1;
    }
    else
    {
        return 0;
    }
}

int SimulationSerial::slave_write(const byte* val, int length)
{
    int rx_count;
    if (!IsEnabled()) { return 0; }
    
    rx_count = 0;
    for (int i = 0; i < length; i++)
    {
        rx_count += slave_write(val[i]);
    }
    return rx_count;
}

int SimulationSerial::available()
{
    if (!IsEnabled()) { return 0; }
    else { return (rx_buffer_length - rx_buffer_index); }
}

int SimulationSerial::availableForWrite()
{
    if (!IsEnabled()) { return 0; }
    else { return (max_buffer_size - tx_buffer_length); }
}

int SimulationSerial::slave_available()
{
    if (!IsEnabled()) { return 0; }
    else { return (tx_buffer_length - tx_buffer_index); }
}

int SimulationSerial::slave_availableForWrite()
{
    if (!IsEnabled()) { return 0; }
    return (max_buffer_size - rx_buffer_length);
}

int SimulationSerial::handleConfig(byte val)
{
    return 1;
}

void SimulationSerial::reset_rx_buffer()
{
    memset(rx_buffer, 0, max_buffer_size);
    rx_buffer_index  = 0;
    rx_buffer_length = 0;
}

void SimulationSerial::reset_tx_buffer()
{
    memset(tx_buffer, 0, max_buffer_size);
    tx_buffer_index  = 0;
    tx_buffer_length = 0;
}

void SimulationSerial::serial_setSimulationModels(ModelMap* pMap, std::string model, bool print)
{
    print_serial = print;
    
    if (pMap)
    {
        pModel = (GenericSensorModel*) pMap->getModel(model);
        if (pModel)
        {
            pModel->setSerialIO(this);
            pModel = 0;
        }
        else
        {
            std::cout << "SimulationSerial::serial_setSimulationModels: cannot find " << model << " model." << std::endl;
        }
    }
}

void SimulationSerial::serial_setTwoHardwareSerialModels(ModelMap* pMap, std::string model, std::string periphialType, bool print)
{
    print_serial = print;
    
    if (pMap)
    {
        pModel = (GenericSensorModel*) pMap->getModel(model);
        if (pModel)
        {
            static_cast<TwoHardwareSerial*>(pModel)->setSerialIO(this, periphialType);
            pModel = 0;
        }
        else
        {
            std::cout << "SimulationSerial::serial_setSimulationModels: cannot find " << model << " model." << std::endl;
        }
    }
}

void SimulationSerial::display_rx_buffer()
{
    if (!available()) { return; }
    std::cout << "SimulationSerial::rx_buffer [" << rx_buffer_index << "-" << rx_buffer_length << "]: ";
    for (int i = rx_buffer_index; i < rx_buffer_length; i++)
    {
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +rx_buffer[i];
    }
    std::cout << std::dec << std::endl;
}

void SimulationSerial::display_tx_buffer()
{
    if (!slave_available()) { return; }
    std::cout << "SimulationSerial::tx_buffer [" << tx_buffer_index << "-" << tx_buffer_length << "]: ";
    for (int i = tx_buffer_index; i < tx_buffer_length; i++)
    {
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +tx_buffer[i];
    }
    std::cout << std::dec << std::endl;
}

void SimulationSerial::serial_setEnablePin(SimulationPins* HW_Pins, int pin, pinStateType activeState)
{
    enable_pin.HW_Pins = HW_Pins;
    enable_pin.pin     = pin;
    enable_pin.activeState = activeState;
}

void SimulationSerial::serial_setConfigPin(SimulationPins* HW_Pins, int pin, pinStateType activeState)
{
    config_pin.HW_Pins = HW_Pins;
    config_pin.pin     = pin;
    config_pin.activeState = activeState;
}

void SimulationSerial::serial_setPeriphialEnablePin(ModelMap* pMap, std::string model, SimulationPins* HW_Pins, int pin, pinStateType activeState)
{
    if (pMap)
    {
        pModel = (GenericSensorModel*) pMap->getModel(model);
        if (pModel)
        {
            static_cast<TwoHardwareSerial*>(pModel)->GetPeriphial()->serial_setEnablePin(HW_Pins, pin, activeState);
            pModel = 0;
        }
        else
        {
            std::cout << "SimulationSerial::serial_setPeriphialEnablePin: cannot find " << model << " model." << std::endl;
        }
    }
}

void SimulationSerial::serial_setPeriphialConfigPin(ModelMap* pMap, std::string model, SimulationPins* HW_Pins, int pin, pinStateType activeState)
{
    if (pMap)
    {
        pModel = (GenericSensorModel*) pMap->getModel(model);
        if (pModel)
        {
            static_cast<TwoHardwareSerial*>(pModel)->GetPeriphial()->serial_setConfigPin(HW_Pins, pin, activeState);
            pModel = 0;
        }
        else
        {
            std::cout << "SimulationSerial::serial_setPeriphialConfigPin: cannot find " << model << " model." << std::endl;
        }
    }
}

bool SimulationSerial::IsEnabled()
{
    return baud_begin && enable_pin.InMode();
}

bool SimulationSerial::IsInConfig()
{
    return config_pin.InMode();
}

bool SimulationSerial::SerialPinType::InMode()
{
    if (HW_Pins) { return HW_Pins->periphialDigitalRead(pin) == activeState; }
    else { return default_InMode; }
}

int APC220RadioSerial::handleConfig(byte val)
{
    if (config_count > 1)
    {
        // config_count=2 -> config_rsp[4]
        //{'W','R',' ','4','3','4','0','0','0',' ','3',' ','9',' ','3',' ', '0', '\r', '\n'} 19
        //->
        //{'P','A','R','A',' ','4','3','4','0','0','0',' ','3',' ','9',' ','3',' ', '0', '\r', '\n'} 21
        config_rsp[config_count+2] = val;
    }
    
    if (config_count == 18)
    {
        slave_write(config_rsp, 21);
        config_count = 0;
    }
    else
    {
        config_count++;
    }

    return 1;
}

template void SimulationSerial::print(const char*, PrintMode printMode);
template void SimulationSerial::print(char, PrintMode printMode);
template void SimulationSerial::print(std::string, PrintMode printMode);
template void SimulationSerial::print(short, PrintMode printMode);
template void SimulationSerial::print(unsigned short, PrintMode printMode);
template void SimulationSerial::print(int, PrintMode printMode);
template void SimulationSerial::print(unsigned int, PrintMode printMode);
template void SimulationSerial::print(long, PrintMode printMode);
template void SimulationSerial::print(unsigned long, PrintMode printMode);
template void SimulationSerial::print(double, PrintMode printMode);
template void SimulationSerial::print(byte, PrintMode printMode);

template void SimulationSerial::println(const char*, PrintMode printMode);
template void SimulationSerial::println(char, PrintMode printMode);
template void SimulationSerial::println(std::string, PrintMode printMode);
template void SimulationSerial::println(short, PrintMode printMode);
template void SimulationSerial::println(unsigned short, PrintMode printMode);
template void SimulationSerial::println(int, PrintMode printMode);
template void SimulationSerial::println(unsigned int, PrintMode printMode);
template void SimulationSerial::println(long, PrintMode printMode);
template void SimulationSerial::println(unsigned long, PrintMode printMode);
template void SimulationSerial::println(double, PrintMode printMode);
template void SimulationSerial::println(byte, PrintMode printMode);

TwoHardwareSerial::TwoHardwareSerial(ModelMap *pMapInit, bool debugFlagIn)
{
    pMap = pMapInit;
    debugFlag = false;
}

bool TwoHardwareSerial::update()
{
    if (!MCU1.init && !MCU2.init) { return false; }

    MCU1.update(MCU2);
    MCU2.update(MCU1);
        
    return true;
}

void TwoHardwareSerial::setSerialIO(SimulationSerial* pSerial, std::string periphialType)
{
    if      (!MCU1.init) { MCU1.set(pSerial, periphialType); }
    else if (!MCU2.init) { MCU2.set(pSerial, periphialType); }
    else    { std::cout << "TwoHardwareSerial - Attempt to set more than 2 MCU's!" << std::endl; }
}

SimulationSerial* TwoHardwareSerial::GetPeriphial()
{
    if (MCU2.init) { return MCU2.GetPeriphial(); }
    else if (MCU1.init) { return MCU1.GetPeriphial(); }
    else { return NULL; }
}

TwoHardwareSerial::MCUIO::MCUIO()
{
    init = false;
    pMCUSerial=0;
    pMCUperiphial=0;
    mcu_rx_fail_count = 0;
    mcu_tx_fail_count = 0;
    periphial_rx_fail_count = 0;
    periphial_tx_fail_count = 0;
    
}

void TwoHardwareSerial::initialize()
{
    pTime  = (Time*) pMap->getModel("Time");
}

TwoHardwareSerial::MCUIO::~MCUIO()
{
    if (pMCUperiphial)
    {
        delete pMCUperiphial;
        pMCUperiphial=0;
    }
}

void TwoHardwareSerial::MCUIO::update(MCUIO &otherMCU)
{
    int n_write_attempt;
    int n_write;
    
    if (pMCUperiphial && pMCUSerial->getBaudRate() > 0)
    {
        pMCUperiphial->begin(pMCUSerial->getBaudRate());
    }
    
    // Read MCU into periphial buffer (MCU tx -> periph tx)
    if (init)
    {
        n_write_attempt = pMCUSerial->slave_available();
        n_write = 0;
        while (pMCUSerial->slave_available())
        {
            n_write += pMCUperiphial->write(pMCUSerial->slave_read());
        }
        if (n_write < n_write_attempt)
        {
            mcu_tx_fail_count++;
        }
    
        // Read MCU periphial into otherMCU periphial (periph tx -> otherPeriph rx)
        n_write_attempt = pMCUperiphial->slave_available();
        n_write = 0;
        while (pMCUperiphial->slave_available())
        {
            if (otherMCU.init)
            {
                n_write += otherMCU.pMCUperiphial->slave_write(pMCUperiphial->slave_read());
            }
            else { pMCUperiphial->slave_read(); }
        }
        if (n_write < n_write_attempt)
        {
            periphial_tx_fail_count++;
            otherMCU.periphial_rx_fail_count++;
        }
    }
    
    // Read otherMCU periphial into otherMCU (otherPeriph rx -> otherMCU rx)
    if (otherMCU.init)
    {
        n_write_attempt = otherMCU.pMCUperiphial->available();
        n_write = 0;
        while (otherMCU.pMCUperiphial->available())
        {
            n_write += otherMCU.pMCUSerial->slave_write(otherMCU.pMCUperiphial->read());
        }
        if (n_write < n_write_attempt)
        {
            otherMCU.mcu_rx_fail_count++;
        }
    }
}

void TwoHardwareSerial::MCUIO::set(SimulationSerial* pSerial, std::string periphialType)
{
    pMCUSerial = pSerial;
    if (periphialType == "Arduino_Nano")
    {
        pMCUperiphial = new ArduinoSerial(pMCUSerial->getTXPin(),pMCUSerial->getRXPin());
        init = true;
    }
    else if (periphialType == "Teensy3_series")
    {
        pMCUperiphial = new Teensy3Serial(pMCUSerial->getTXPin(),pMCUSerial->getRXPin());
        init = true;
    }
    else if (periphialType == "Teensy4_series")
    {
        pMCUperiphial = new Teensy4Serial(pMCUSerial->getTXPin(),pMCUSerial->getRXPin());
        init = true;
    }
    else if (periphialType == "GPS_Neo6m")
    {
        pMCUperiphial = new GPSNeo6MSerial(pMCUSerial->getTXPin(),pMCUSerial->getRXPin());
        init = true;
    }
    else if (periphialType == "HC05_Bluetooth")
    {
        pMCUperiphial = new BluetoothHC05Serial(pMCUSerial->getTXPin(),pMCUSerial->getRXPin());
        init = true;
    }
    else if (periphialType == "APC220Radio")
    {
        pMCUperiphial = new APC220RadioSerial(pMCUSerial->getTXPin(),pMCUSerial->getRXPin());
        init = true;
    }
}

ArduinoEEPROM::ArduinoEEPROM()
{
    size = 1080;
    new_file = false;
    
    if (!std::ifstream(eeprom_file))
    {
        std::ofstream temp(eeprom_file);
        temp.close();
        new_file = true;
    }
    memoryfile.open(eeprom_file, std::ios::in | std::ios::out);
    
    memoryfile.seekg(0, std::ios::end);
    file_length = memoryfile.tellg();
    memoryfile.clear();
    memoryfile.seekg(0, std::ios::beg);
    
    reset_eeprom();
}

ArduinoEEPROM::~ArduinoEEPROM()
{
    if (memoryfile.is_open())
    {
        memoryfile.close();
    }
}

byte ArduinoEEPROM::read(unsigned int address)
{
    byte val = 0;
    
    if (memoryfile.fail()) { return val; }
    
    memoryfile.seekg(address);
    memoryfile >> val;
    
    return val;
}

void ArduinoEEPROM::write(unsigned int address, byte val)
{
    if (memoryfile.fail()) { return; }
    
    memoryfile.seekp(address);
    memoryfile << val;
    memoryfile.flush();
}

void ArduinoEEPROM::reset_eeprom()
{
    if (memoryfile.fail() || (!new_file && file_length == size)) { return; }
    
    byte val = 48;
    for (int i = 0; i < size; i++)
    {
        memoryfile << val;
    }
    memoryfile.flush();
}

SimulationPins::SimulationPins()
{
    digitalPins = new DigitalPinType[max_digital_pins];
}

SimulationPins::~SimulationPins()
{
    if (digitalPins)  { delete[] digitalPins; digitalPins = 0; }
}

void SimulationPins::pinMode(unsigned int pin, pinModeType mode)
{
    digitalPins[pin].pin_mode = mode;
}

pinStateType SimulationPins::digitalRead(unsigned int pin)
{
    if (digitalPins[pin].pin_mode == INPUT)
    {
        return digitalPins[pin].pin_state;
    }
    else
    {
        std::cout << "SimulationPins(" << serial_type << ")::digitalRead - Unable to read from OUTPUT Pin " << pin << "!" << std::endl;
        return LOW;
    }
}

void SimulationPins::digitalWrite(unsigned int pin, pinStateType state)
{
    if (digitalPins[pin].pin_mode == OUTPUT)
    {
        digitalPins[pin].pin_state = state;
    }
    else
    {
        std::cout << "SimulationPins(" << serial_type << ")::digitalWrite - Unable to write to INPUT Pin " << pin << "!" << std::endl;
    }
}

void SimulationPins::periphialDigitalWrite(unsigned int pin, pinStateType state)
{
    if (digitalPins[pin].pin_mode == INPUT)
    {
        digitalPins[pin].pin_state = state;
    }
    else
    {
        std::cout << "SimulationPins(" << serial_type << ")::periphialDigitalWrite - Unable to write to OUTPUT Pin " << pin << "!" << std::endl;
    }
}

pinStateType SimulationPins::periphialDigitalRead(unsigned int pin)
{
    if (digitalPins[pin].pin_mode == OUTPUT)
    {
        return digitalPins[pin].pin_state;
    }
    else
    {
        std::cout << "SimulationPins(" << serial_type << ")::periphialDigitalRead - Unable to read from INPUT Pin " << pin << "!" << std::endl;
        return LOW;
    }
}

std::map<std::string, SimulationNRF24L01::buffer_type> SimulationNRF24L01::buffer_map;
std::map<std::string, bool> SimulationNRF24L01::buffer_empty_map;

SimulationNRF24L01::SimulationNRF24L01(unsigned int CE_PIN, unsigned int CSN_PIN)
{
    active = false;
    listening = false;
    address = "";
}

void SimulationNRF24L01::begin()
{
    active = true;
}

void SimulationNRF24L01::openWritingPipe(const byte* address_byte)
{
    address = reinterpret_cast<const char*> (address_byte);
    if (!address_open(address))
    {
        buffer_type this_buffer;
        buffer_map[address] = this_buffer;
        buffer_empty_map[address] = true;
    }
    writing_pipe_map[address] = true;
}

void SimulationNRF24L01::openReadingPipe(byte number , const byte* address_byte)
{
    address = reinterpret_cast<const char*> (address_byte);
    if (!address_open(address))
    {
        buffer_type this_buffer;
        buffer_map[address] = this_buffer;
        buffer_empty_map[address] = true;
    }
    writing_pipe_map[address] = false;
}

void SimulationNRF24L01::stopListening()
{
    listening = false;
}

void SimulationNRF24L01::startListening()
{
    listening = true;
}

bool SimulationNRF24L01::write(byte* buffer, unsigned int size)
{
    if (address != "" && buffer_empty() && !listening)
    {
        for(unsigned int i=0; i<size; i++)
        { buffer_map[address].buffer[i] = buffer[i]; }
        //memcpy(buffer_map[address].buffer, buffer, size);
        buffer_empty_map[address] = false;
        //std::cout << "NRF25L01 write(): "; print_buffer(address);
        return true;
    }
    return false;
}

void SimulationNRF24L01::read(byte* buffer, unsigned int size)
{
    if (listening)
    {
        //std::cout << "NRF25L01 read(): "; print_buffer(address);
        //memcpy(buffer, buffer_map[address].buffer, size);
        for(unsigned int i=0; i<size; i++)
        { buffer[i] = buffer_map[address].buffer[i]; }
        memset(buffer_map[address].buffer, 0, buffer_size);
        buffer_empty_map[address] = true;
    }
}

bool SimulationNRF24L01::available()
{
    return (listening && !buffer_empty());
}

void SimulationNRF24L01::setPALevel(rf24_pa_dbm_e rf_level)
{
    // Do nothing //
}

void SimulationNRF24L01::setDataRate( rf24_datarate_e data_rate )
{
    int rate = 0;
    if (data_rate == RF24_1MBPS)
    {
        rate = 1000000;
    }
    else if (data_rate == RF24_2MBPS)
    {
          rate = 2000000;
    }
    else if (data_rate == RF24_250KBPS)
    {
        rate = 250000;
    }
    
    if (rate != 0 && address != "" && buffer_map.find(address) != buffer_map.end())
    {
        buffer_map[address].rate = rate;
    }
}

bool SimulationNRF24L01::address_open(std::string address)
{
    bool map_empty = true;
    for(std::map<std::string, buffer_type>::iterator it = buffer_map.begin(); it != buffer_map.end(); it++)
    {
        if (map_empty) { map_empty = false; }
        if (it->first == address) { return true; }
    }
    return !map_empty;
}

bool SimulationNRF24L01::buffer_empty()
{
    if (address == "" || !address_open(address)) { return true; }
    return buffer_empty_map[address];
}

void SimulationNRF24L01::print_buffer(std::string address)
{
    if (!address_open(address)) { return; }
    std::cout << std::dec << "SimulationNRF25L01 buffer (" << address << "): ";
    for (int i=0; i < buffer_map[address].payload_size; i++)
    {
        std::cout << std::hex << std::setfill('0') << std::setw(2) << (int) buffer_map[address].buffer[i];
    }
    std::cout << std::dec << std::endl;
}

void SimulationNRF24L01::print_buffer_empty()
{
    std::cout << "Printing buffer empty map" << std::endl;
    int i=0;
    for(std::map<std::string, bool>::iterator it = buffer_empty_map.begin(); it != buffer_empty_map.end(); it++)
    {
        std::cout << ++i << ") [address, buffer_size] = [" << it->first << ", " << it->second << "]" << std::endl;
    }
}

void SimulationNRF24L01::print_buffer_map()
{
    std::cout << "Printing buffer map" << std::endl;
    int i=0;
    for(std::map<std::string, buffer_type>::iterator it = buffer_map.begin(); it != buffer_map.end(); it++)
    {
        std::cout << ++i << ") [address, buffer_size] = [" << it->first << ", " << it->second.payload_size << "]" << std::endl;
    }
}

void SimulationNRF24L01::print_writing_map()
{
    std::cout << "Printing writing map" << std::endl;
    int i=0;
    for(std::map<std::string, bool>::iterator it = writing_pipe_map.begin(); it != writing_pipe_map.end(); it++)
    {
        std::cout << ++i << ") [address, writing_bool] = [" << it->first << ", " << it->second << "]" << std::endl;
    }
}

void delay(int timeMS) { }

SimulationSerial Serial = SimulationSerial();
HardwareSerial Serial1 = HardwareSerial(0,1);
HardwareSerial Serial2 = HardwareSerial(7,8);
SimulationWire Wire = SimulationWire();
