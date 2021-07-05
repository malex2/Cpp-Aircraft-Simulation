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
#include "utilities.hpp"
Utilities util;
#endif

// IMU data
IMUtype IMUdata;

bool imu_setup  = false;
double imuDt    = 1.0/800.0;
bool print_wire = false;
bool directIMU  = true; // Use direct IMU and not I2C
bool bodySum    = true; // Sum delta velocities in a previous body frame under a duration of rotation, then transform into the new body frame
double R[3][3];
double dVelocity[3];
double dVelcoity_preBody[3];

// IMU Sensitivity
double accSensitivityLSB[nAccSensitivity] = {16483, 8192, 4096, 2048};
int accSensitivityWrite;
double LSBg;

double gyroSensitivityLSB[nAccSensitivity] = {131, 65.5, 32.8, 16.4};
int gyroSensitivityWrite;
double LSBdps;

int sensitivityByte[nAccSensitivity] = {0b00000000, 0b00001000, 0b00010000, 0b00011000};

// Simulation
#ifdef SIMULATION
SimulationWire Wire;
#endif
class IMUModelBase;
IMUModelBase* pIMUmodel = 0;

void FsImu_setupIMU(accSensitivityType accSensitivity, gyroSensitivityType gyroSensitivity)
{
    accSensitivityWrite  = sensitivityByte[accSensitivity];
    gyroSensitivityWrite = sensitivityByte[gyroSensitivity];
    
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
    Wire.write(gyroSensitivityWrite);
    Wire.endTransmission(true);
    
    // Accelerometer range
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACC_REG); // Accelerometer register
    Wire.write(accSensitivityWrite);
    Wire.endTransmission(true);
    
    for (int i=0; i<3; i++)
    {
        dVelocity[i] = 0.0;
        dVelcoity_preBody[i] = 0.0;
        for (int j=0; j<3; j++)
        {
            if (i == j) { R[i][j] = 1.0; }
            else { R[i][j] = 0.0; }
        }
    }
    
    imu_setup = true;
}

void FsImu_performIMU()
{
    if (!imu_setup) { return; }
    
    readIMU();
    updateDelta();
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
    for (int i=0; i<3; i++)
    {
#ifdef SIMULATION
        if (directIMU)
        {
            IMUdata.accel[i] = pIMUmodel->getAccelerometer()[i];
            IMUdata.gyro[i] = pIMUmodel->getGyroscope()[i];
        }
#endif
        // Protect against negative
        short maxByte = 32767;
        if (IMUdata.accel[i] > maxByte) { IMUdata.accel[i] = IMUdata.accel[i] - 2*(maxByte+1); }
        if (IMUdata.gyro[i] > maxByte)  { IMUdata.gyro[i] = IMUdata.gyro[i] - 2*(maxByte+1); }
        
        IMUdata.accel[i] /= LSBg;
        IMUdata.gyro[i]  /= LSBdps;
    }
}

void updateRotation()
{
    if (bodySum)
    {
       R[0][0] = 1;
       R[0][1] = IMUdata.dTheta[1]*IMUdata.dTheta[0]*deg2rad2-IMUdata.dTheta[2]*deg2rad;
       R[0][2] = IMUdata.dTheta[1]*deg2rad + IMUdata.dTheta[0]*IMUdata.dTheta[2]*deg2rad2;
       R[1][0] = IMUdata.dTheta[2]*deg2rad;
       R[1][1] = IMUdata.dTheta[0]*IMUdata.dTheta[1]*IMUdata.dTheta[2]*deg2rad2*deg2rad;
       R[1][2] = -IMUdata.dTheta[0]*deg2rad + IMUdata.dTheta[1]*IMUdata.dTheta[2]*deg2rad2;
       R[2][0] = -IMUdata.dTheta[1]*deg2rad;
       R[2][1] = IMUdata.dTheta[0]*deg2rad;
       R[2][2] = 1;
    }
}

void updateDelta()
{
    IMUdata.dCount += imuDt;
    for (int i=0; i<3; i++)
    {
        IMUdata.dTheta[i] += IMUdata.gyro[i] * imuDt;
        dVelocity[i] = IMUdata.accel[i] * imuDt;
    }
    
    updateRotation();
    
    // Sum delta velocities in previous body frame
    dVelcoity_preBody[0] = dVelcoity_preBody[0] + R[0][0]*dVelocity[0] + R[0][1]*dVelocity[1] + R[0][2]*dVelocity[2];
    dVelcoity_preBody[1] = dVelcoity_preBody[1] + R[1][0]*dVelocity[0] + R[1][1]*dVelocity[1] + R[1][2]*dVelocity[2];
    dVelcoity_preBody[2] = dVelcoity_preBody[2] + R[2][0]*dVelocity[0] + R[2][1]*dVelocity[1] + R[2][2]*dVelocity[2];
    
    if (pIMUmodel)
    pIMUmodel->deltaIMU(imuDt);
}

IMUtype* FsImu_getIMUdata()
{
    // Compute delta velocity in new body frame
    IMUdata.dVelocity[0] = R[0][0]*dVelcoity_preBody[0] + R[1][0]*dVelcoity_preBody[1] + R[2][0]*dVelcoity_preBody[2];
    IMUdata.dVelocity[1] = R[0][1]*dVelcoity_preBody[0] + R[1][1]*dVelcoity_preBody[1] + R[2][1]*dVelcoity_preBody[2];
    IMUdata.dVelocity[2] = R[0][2]*dVelcoity_preBody[0] + R[1][2]*dVelcoity_preBody[1] + R[2][2]*dVelcoity_preBody[2];
    
    IMUdata.timestamp = getTime();
    return &IMUdata;
}

void FsImu_zeroDelta(bool zero)
{
    if (zero)
    {
        IMUdata.dCount = 0;
        for (int i=0; i<3; i++)
        {
            IMUdata.dTheta[i]    = 0.0;
            IMUdata.dVelocity[i] = 0.0;
            dVelocity[i] = 0.0;
            dVelcoity_preBody[i] = 0.0;
        }
    }

    if (pIMUmodel)
        pIMUmodel->reset();
        
}

void FsImu_setSimulationModels(ModelMap* pMap)
{
#ifdef SIMULATION
    Wire.wire_setSimulationModels(pMap, print_wire);
    pIMUmodel = (IMUModelBase*) pMap->getModel("IMUModel");
#endif
}
