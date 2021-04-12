//
//  fs_imu.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 2/26/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#ifndef fs_imu_hpp
#define fs_imu_hpp

#include "fs_common.hpp"

// Types
struct IMUtype {
    double accel[3];
    double dVelocity[3];
    
    double gyro[3];
    double dTheta[3];
    
    double temp;
    
    double    dCount;
    double timestamp;
    
    IMUtype()
    {
        for (int i=0; i<3; i++)
        {
            accel[i]     = 0;
            dVelocity[i] = 0;
            gyro[i]      = 0;
            dTheta[i]    = 0;
        }
        temp = 0;
        dCount = 0;
        timestamp = 0;
    }
};

enum accSensitivityType  {accSensitivity_2g, accSensitivity_4g, accSensitivity_8g, accSensitivity_16g, nAccSensitivity};
enum gyroSensitivityType {gyroSensitivity_250dps, gyroSensitivity_500dps, gyroSensitivity_1000dps, gyroSensitivity_2000dps, nGyroSensitivity};

// I2C Registers
const int MPU_ADDR   = 0x68;
const int PWR_MGMT_1 = 0x6B; // PWR_MGMT_1 register
const int GYRO_REG   = 0x1B; // Gryo register
const int ACC_REG    = 0x1C; // Accelerometer register
const int ACC_OUT    = 0x3B; // (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
const int INT        = 0x37;
const int INT_ENABLE = 0x38;

// External Functions
void FsImu_performIMU();
void FsImu_setupIMU(accSensitivityType accSensitivity, gyroSensitivityType gyroSensitivity);
IMUtype* FsImu_getIMUdata();
void FsImu_zeroDelta(bool zero);

// Internal Functions
void readIMU();
void groundCalibration();
void updateDelta();

void FsImu_setSimulationModels(ModelMap* pMap);

#endif /* fs_imu_hpp */
