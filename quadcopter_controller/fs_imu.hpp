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
    
    double temperature;
    double timestamp;
    
    IMUtype()
    {
        for (int i=0; i<3; i++)
        {
            accel[i]     = 0.0;
            dVelocity[i] = 0.0;
            gyro[i]      = 0.0;
            dTheta[i]    = 0.0;
        }
        temperature = 0.0;
        timestamp   = 0.0;
    }
};

enum accSensitivityType  {accSensitivity_2g, accSensitivity_4g, accSensitivity_8g, accSensitivity_16g, nAccSensitivity};
enum gyroSensitivityType {gyroSensitivity_250dps, gyroSensitivity_500dps, gyroSensitivity_1000dps, gyroSensitivity_2000dps, nGyroSensitivity};

// External Functions
void FsImu_performIMU( double &imuDt );
void FsImu_setupIMU(accSensitivityType accSensitivity, gyroSensitivityType gyroSensitivity);
IMUtype* FsImu_getIMUdata();
void FsImu_zeroDelta();

// Internal Functions
void readIMU();
void groundCalibration();
void updateDelta( double &imuDt );

#ifdef SIMULATION
    void FsImu_setSimulationModels(ModelMap* pMap);
#endif

#endif /* fs_imu_hpp */
