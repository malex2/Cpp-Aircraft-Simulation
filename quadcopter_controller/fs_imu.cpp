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
    #include "dynamics_model.hpp"
#endif

// IMU data
IMUtype IMUdata;

// Booleans
bool imu_setup = false;
bool printI2C  = false;
bool startDelta = false;
bool I2C_data_available = false;
bool I2C_data_valid = false;
bool imu_apply_corrections = false;
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

double gyro_dt[3];
double vel_dt[3];

// Coning/Sculling Corrections
double coning_part1[3];
double dconing[3];
double sculling_part1a[3];
double sculling_part1b[3];
double sculling_part2a[3];
double sculling_part2b[3];

// Simulation
#ifdef SIMULATION
    DynamicsModel* pDYNModel = 0;
    IMUModelBase* pIMUmodel = 0;
#endif

void FsImu_setupIMU(accSensitivityType accSensitivity, gyroSensitivityType gyroSensitivity)
{
#ifdef IMU
    LSBg   = accSensitivityLSB[accSensitivity];
    LSBdps = gyroSensitivityLSB[gyroSensitivity];
    
    IMUdata.gyroQuantizationError_dps = 1.0/LSBdps;
    IMUdata.accelQuantizationError_g = 1.0/LSBg;

    display("Setting Up IMU I2C...\n");
    
    Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(PWR_MGMT_1); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    IMUdata.errorCodeIMU = (I2C_Error_Code) Wire.endTransmission(true);
    
    // Gyro range
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_REG); // Gryo register
    Wire.write( sensitivityByte[gyroSensitivity] );
    IMUdata.errorCodeIMU = (I2C_Error_Code) Wire.endTransmission(true);
    
    // Accelerometer range
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACC_REG); // Accelerometer register
    Wire.write( sensitivityByte[accSensitivity] );
    IMUdata.errorCodeIMU = (I2C_Error_Code) Wire.endTransmission(true);
    
    for (int i = 0; i < 3; i++)
    {
        gyro_dt[i] = 0.0;
        vel_dt[i] = 0.0;
    }
    
    imu_setup = true;
#endif
}

void FsImu_performIMU( double &imuDt )
{
#ifdef IMU
    if (!imu_setup) { return; }
    
    readIMU();
    updateDelta(imuDt);
    IMUdata.timestamp = getTime();
#endif
}

void readIMU()
{
    // Get Raw Values
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACC_OUT); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    IMUdata.errorCodeIMU = (I2C_Error_Code) Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    if ( Wire.available() )
    {
        rawAccel[0]  = (((int16_t) Wire.read()) << 8) | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
        rawAccel[1]  = (((int16_t) Wire.read()) << 8) | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
        rawAccel[2]  = (((int16_t) Wire.read()) << 8) | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
        rawTemp      = (((int16_t) Wire.read()) << 8) | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
        rawGyro[0]   = (((int16_t) Wire.read()) << 8) | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
        rawGyro[1]   = (((int16_t) Wire.read()) << 8) | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
        rawGyro[2]   = (((int16_t) Wire.read()) << 8) | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

        I2C_data_available = true;
        I2C_data_valid = rawAccel[0] != 0 | rawAccel[1] != 0 | rawAccel[2] != 0 | rawGyro[0] != 0 | rawGyro[1] != 0 | rawGyro[2] != 0;
    }
    else
    {
        I2C_data_available = false;
        I2C_data_valid = false;
    }
    
    IMUdata.IMUgood = (IMUdata.errorCodeIMU == I2C_0_SUCCESS) & I2C_data_available & I2C_data_valid;
    
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
        IMUdata.accel[i] = (double) IMUtoBody[i]*rawAccel[i]/LSBg * Gravity;
        IMUdata.gyro[i]  = (double) IMUtoBody[i]*rawGyro[i]/LSBdps * degree2radian;
  
#ifdef SIMULATION
        if (directUnitIMU)
        {
            IMUdata.accel[i] = IMUtoBody[i]*pIMUmodel->getAccelerometerGs()[i] * Gravity;
            IMUdata.gyro[i]  = IMUtoBody[i]*pIMUmodel->getGyroscopeDps()[i] * degree2radian;
        }
#endif
    }
}

void updateDelta( double &imuDt )
{
    // Must call FsImu_zeroDelta() at least once before accumulating delta IMU data
    if (!startDelta) { return; }
    
    for (int i = 0; i < 3; i++)
    {
        if (imu_apply_corrections)
        {
            coning_part1[i] = 0.5 * (IMUdata.dTheta[i] + gyro_dt[i]/6.0);
            sculling_part1a[i] = IMUdata.dTheta[i] + gyro_dt[i]/6.0;
            sculling_part2a[i] = IMUdata.dVelocity[i] + vel_dt[i]/6.0;
        }
        
        gyro_dt[i] = IMUdata.gyro[i]  * imuDt;
        vel_dt[i]  = IMUdata.accel[i] * imuDt;
        
        IMUdata.dTheta[i]    += gyro_dt[i];
        IMUdata.dVelocity[i] += vel_dt[i];
    }
    
    if (imu_apply_corrections)
    {
        // Coning
        crossProduct(dconing, coning_part1, gyro_dt);
        IMUdata.coningCorrection[0] += dconing[0];
        IMUdata.coningCorrection[1] += dconing[1];
        IMUdata.coningCorrection[2] += dconing[2];
        
        // Sculling
        crossProduct(sculling_part1b, sculling_part1a, vel_dt);
        crossProduct(sculling_part2b, sculling_part2a, gyro_dt);
        IMUdata.scullingCorrection[0] += 0.5 * (sculling_part1b[0] + sculling_part2b[0]);
        IMUdata.scullingCorrection[1] += 0.5 * (sculling_part1b[1] + sculling_part2b[1]);
        IMUdata.scullingCorrection[2] += 0.5 * (sculling_part1b[2] + sculling_part2b[2]);
    }
    
#ifdef SIMULATION
    if (pDYNModel) { pDYNModel->deltaIMU(imuDt); }
    if (pIMUmodel) { pIMUmodel->deltaIMU(imuDt); }
#endif
}

const IMUtype* FsImu_getIMUdata()
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
        
        IMUdata.coningCorrection[i]   = 0.0;
        IMUdata.scullingCorrection[i] = 0.0;
        
        gyro_dt[i] = 0.0;
        vel_dt[i] = 0.0;
    }

#ifdef SIMULATION
    if (pDYNModel) { pDYNModel->resetIMU(); }
    if (pIMUmodel) { pIMUmodel->reset(); }
#endif
}

bool FsImu_IMUGood()
{
    return IMUdata.IMUgood;
}

void FsImu_setCorrectionsFlag(bool flag)
{
    imu_apply_corrections = flag;
}

void printI2CErrors(bool printBool)
{
    printI2C = printBool;
}

#ifdef SIMULATION
void FsImu_setSimulationModels(ModelMap* pMap)
{
    Wire.wire_setSimulationModels(pMap, print_wire);
    pDYNModel = (DynamicsModel*) pMap->getModel("DynamicsModel");
    pIMUmodel = (IMUModelBase*) pMap->getModel("IMUModel");
}
#endif
