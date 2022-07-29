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
#endif

// IMU data
IMUtype IMUdata;

// Booleans
bool imu_setup = false;
bool printI2C  = false;
bool startDelta = false;
# ifdef SIMULATION
    bool print_wire = false;
    bool directRawIMU  = false; // Use direct raw IMU and not I2C
    bool directUnitIMU = false; // Use direct unitized IMU instead of LSB conversion
#endif

// IMU Sensitivity
const float accSensitivityLSB[nAccSensitivity] = {16483.0, 8192.0, 4096.0, 2048.0};
const float gyroSensitivityLSB[nGyroSensitivity] = {131.0, 65.5, 32.8, 16.4};
const int sensitivityByte[nAccSensitivity] = {0b00000000, 0b00001000, 0b00010000, 0b00011000};
float LSBg;
float LSBdps;

// Raw IMU measurements
short rawAccel[3];
short rawGyro[3];
short rawTemp;
short errorCodeIMU;

float toBody[3] = {1.0, -1.0, -1.0};

double accelSumSqr = 0.0;
double gyroSumSqr = 0.0;

// Simulation
#ifdef SIMULATION
    class IMUModelBase;
    IMUModelBase* pIMUmodel = 0;
#endif

void FsImu_setupIMU(accSensitivityType accSensitivity, gyroSensitivityType gyroSensitivity)
{
    LSBg   = accSensitivityLSB[accSensitivity];
    LSBdps = gyroSensitivityLSB[gyroSensitivity];
    
    IMUdata.gyroQuantizationError_dps = 1.0/LSBdps;
    IMUdata.accelQuantizationError_g = 1.0/LSBg;

    display("Setting Up IMU I2C...\n");
    
    Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(PWR_MGMT_1); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    errorCodeIMU = Wire.endTransmission(true);
    
    // Gyro range
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_REG); // Gryo register
    Wire.write( sensitivityByte[gyroSensitivity] );
    errorCodeIMU = Wire.endTransmission(true);
    
    // Accelerometer range
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACC_REG); // Accelerometer register
    Wire.write( sensitivityByte[accSensitivity] );
    errorCodeIMU = Wire.endTransmission(true);
    
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
    static bool noI2Cprint = false;
    
    // Get Raw Values
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACC_OUT); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    errorCodeIMU = Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    if ( Wire.available() )
    {
        rawAccel[0]  = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
        rawAccel[1]  = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
        rawAccel[2]  = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
        rawTemp      = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
        rawGyro[0]   = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
        rawGyro[1]   = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
        rawGyro[2]   = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
    }
    else if (!noI2Cprint)
    {
        display("No I2C.\n");
        noI2Cprint = true;
    }
    
    // Convert to Units
    for (int i=0; i<3; i++)
    {
#ifdef SIMULATION
        if (directRawIMU)
        {
            rawAccel[i] = pIMUmodel->getAccelerometer()[i];
            rawGyro[i]  = pIMUmodel->getGyroscope()[i];
        }
#endif
        IMUdata.accel[i] = (double) toBody[i]*rawAccel[i]/LSBg * Gravity;
        IMUdata.gyro[i]  = (double) toBody[i]*rawGyro[i]/LSBdps * degree2radian;
  
#ifdef SIMULATION
        if (directUnitIMU)
        {
            IMUdata.accel[i] = toBody[i]*pIMUmodel->getAccelerometerGs()[i] * Gravity;
            IMUdata.gyro[i]  = toBody[i]*pIMUmodel->getGyroscopeDps()[i] * degree2radian;
        }
#endif
        
        accelSumSqr = accelSumSqr + IMUdata.accel[i]*IMUdata.accel[i];
        gyroSumSqr  = gyroSumSqr  + IMUdata.gyro[i]*IMUdata.gyro[i];
    }
    accelSumSqr = sqrt(accelSumSqr);
    gyroSumSqr  = sqrt(gyroSumSqr);
    
    if ((gyroSumSqr > highRate)  || (accelSumSqr > highAccel))
    {
        IMUdata.highDynamics = true;
    }
    else
    {
        IMUdata.highDynamics = false;
    }
    
    accelSumSqr = 0.0;
    gyroSumSqr  = 0.0;
}

void updateDelta( double &imuDt )
{
    // Must call FsImu_zeroDelta() at least once before accumulating delta IMU data
    if (!startDelta) { return; }
    
    for (int i = 0; i < 3; i++)
    {
        IMUdata.dTheta[i]    += IMUdata.gyro[i] * imuDt;
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

void FsImu_zeroDelta()
{
    if (!startDelta) { startDelta = true; }
    
    for (int i=0; i<3; i++)
    {
        IMUdata.dTheta[i]    = 0.0;
        IMUdata.dVelocity[i] = 0.0;
    }

#ifdef SIMULATION
    if (pIMUmodel) { pIMUmodel->reset(); }
#endif
}

void printI2CErrors(bool printBool)
{
    printI2C = printBool;
}

#ifdef SIMULATION
void FsImu_setSimulationModels(ModelMap* pMap)
{
    Wire.wire_setSimulationModels(pMap, print_wire);
    pIMUmodel = (IMUModelBase*) pMap->getModel("IMUModel");
}
#endif
