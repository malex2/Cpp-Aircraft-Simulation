//
//  fs_imu.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 2/26/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "fs_imu.hpp"
#ifdef SIMULATION
    #include "imu_model.hpp"
#else
    #include "Wire.h"
#endif

// IMU data
IMUtype IMUdata;

// Booleans
bool imu_setup  = false;
# ifdef SIMULATION
    bool print_wire = false;
    bool directIMU  = false; // Use direct IMU and not I2C
#endif

// IMU Sensitivity
const float accSensitivityLSB[nAccSensitivity] = {16483.0, 8192.0, 4096.0, 2048.0};
const float gyroSensitivityLSB[nAccSensitivity] = {131.0, 65.5, 32.8, 16.4};
const int sensitivityByte[nAccSensitivity] = {0b00000000, 0b00001000, 0b00010000, 0b00011000};
float LSBg;
float LSBdps;

// Simulation
#ifdef SIMULATION
    SimulationWire Wire;
    class IMUModelBase;
    IMUModelBase* pIMUmodel = 0;
#endif

void FsImu_setupIMU(accSensitivityType accSensitivity, gyroSensitivityType gyroSensitivity)
{
    LSBg   = accSensitivityLSB[accSensitivity];
    LSBdps = gyroSensitivityLSB[gyroSensitivity];
    
    display("Setting Up IMU I2C...\n");
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(PWR_MGMT_1); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    
    // Gyro range
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_REG); // Gryo register
    Wire.write( sensitivityByte[gyroSensitivity] );
    Wire.endTransmission(true);
    
    // Accelerometer range
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACC_REG); // Accelerometer register
    Wire.write( sensitivityByte[accSensitivity] );
    Wire.endTransmission(true);
    
    imu_setup = true;
}

void FsImu_performIMU( double &imuDt )
{
    if (!imu_setup) { return; }
    
    readIMU();
    updateDelta(imuDt);
    IMUdata.timestamp = getTime();
}

void readIMU()
{
    // Get Raw Values
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACC_OUT); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers
    
    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    if ( Wire.available() )
    {
        IMUdata.accel[0]  = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
        IMUdata.accel[1]  = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
        IMUdata.accel[2]  = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
        IMUdata.temp      = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
        IMUdata.gyro[0]   = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
        IMUdata.gyro[1]   = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
        IMUdata.gyro[2]   = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
    }
    else
    {
        display("No I2C\n");
    }
    
    // Convert to Units
    float toBody[3] = {1.0, -1.0, -1.0};
    for (int i=0; i<3; i++)
    {
#ifdef SIMULATION
        if (directIMU)
        {
            IMUdata.accel[i] = pIMUmodel->getAccelerometer()[i];
            IMUdata.gyro[i] = pIMUmodel->getGyroscope()[i];
        }
        
        // Adjust for negative
        short maxByte = 32767;
        if (IMUdata.accel[i] > maxByte) { IMUdata.accel[i] = IMUdata.accel[i] - 2*(maxByte+1); }
        if (IMUdata.gyro[i] > maxByte)  { IMUdata.gyro[i] = IMUdata.gyro[i] - 2*(maxByte+1); }
#endif
        IMUdata.accel[i] = toBody[i]*IMUdata.accel[i]/LSBg;
        IMUdata.gyro[i]  = toBody[i]*IMUdata.gyro[i]/LSBdps;
    }
}

void updateDelta( double &imuDt )
{
    for (int i=0; i<3; i++)
    {
        IMUdata.dTheta[i] += IMUdata.gyro[i] * imuDt;
        IMUdata.dVelocity[i] += IMUdata.accel[i] * imuDt;
    }

#ifdef SIMULATION
    if (pIMUmodel) { pIMUmodel->deltaIMU(imuDt); }
#endif
}

IMUtype* FsImu_getIMUdata()
{
    return &IMUdata;
}

void FsImu_zeroDelta(bool zero)
{
    if (zero)
    {
        for (int i=0; i<3; i++)
        {
            IMUdata.dTheta[i]    = 0.0;
            IMUdata.dVelocity[i] = 0.0;
        }
    }

#ifdef SIMULATION
    if (pIMUmodel) { pIMUmodel->reset(); }
#endif
}

#ifdef SIMULATION
void FsImu_setSimulationModels(ModelMap* pMap)
{
    Wire.wire_setSimulationModels(pMap, print_wire);
    pIMUmodel = (IMUModelBase*) pMap->getModel("IMUModel");
}
#endif
