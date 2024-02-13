//
//  imu_model.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/28/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "imu_model.hpp"

 #include "initial_conditions.hpp"
 #include "model_mapping.hpp"
 #include "rotate_frame.hpp"
 #include "dynamics_model.hpp"
 #include "atmosphere_model.hpp"
 #include "time.hpp"
 
IMUModelBase::IMUModelBase(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pAtmo   = NULL;
    pRotate = NULL;
    pTime   = NULL;
    pMap    = pMapInit;
    
    // units to LSB
    LSBdps = 0.0;
    LSBg   = 0.0;
    LSBuT  = 0.0;
    refGravity = 9.80665;
    //perfectSensor = false;
    imuReady = false;
    onlyGravity = false;
    
    // Noise
    util.initArray(gyroBias, 0.0, 3);
    util.initArray(gyroNoiseMax, 0.0, 3);
    util.initArray(gyroError, 0.0, 3);
    
    util.initArray(accBias, 0.0, 3);
    util.initArray(accNoiseMax, 0.0, 3);
    util.initArray(accError, 0.0, 3);

    util.initArray(magBias, 0.0, 3);
    util.initArray(magNoiseMax, 0.0, 3);
    util.initMatrix(*SoftIronMatrix, 0.0, 3, 3);
    util.initArray(magWithSoftIron, 0.0, 3);
    util.initArray(magError, 0.0, 3);
    
    util.initArray(gyroSensor, 0.0, 3);
    util.initArray(accSensor, 0.0, 3);
    util.initArray(magSensor, 0.0, 3);
 
    util.initArray(gyroInUnits, 0.0, 3);
    util.initArray(accInUnits, 0.0, 3);
    util.initArray(magInUnits, 0.0, 3);
    
    util.setUnitClassArray(bodyRates, zero_init, radiansPerSecond, 3);
    util.setUnitClassArray(eulerAngles, zero_init, degrees, 3);
    util.initArray(bodyAcc, 0.0, 3);
    util.initArray(bodyAngularAcc, 0.0, 3);
    gravity          = 9.81;
    magneticStrength = 55.0;
    inclination      = 70.0;
    
    util.initArray(gyroIMU, 0.0, 3);
    
    util.initArray(accTangentBody, 0.0, 3);
    util.initArray(accNormalBody, 0.0, 3);
    util.setUnitClassArray(velNormalBody, zero_init, metersPerSecond, 3);
    util.initArray(accRotationBody, 0.0, 3);
    util.initArray(accTotalBody, 0.0, 3);
    util.initArray(gravityBody, 0.0, 3);
    util.initArray(gravityIMU, 0.0, 3);
    util.initArray(unitGravityBody, 0.0, 3);
    util.initArray(accIMUnoGravity, 0.0, 3);
    util.initArray(accIMU, 0.0, 3);
    
    util.initArray(magNED, 0.0, 3);
    util.initArray(magBody, 0.0, 3);
    util.initArray(magIMU, 0.0, 3);

    util.setUnitClassArray(sensorFrameEuler, zero_init, degrees, 3);
    sensorFrameEuler[0].val = 180.0;
    sensorFrameEuler[1].val = 0.0;
    sensorFrameEuler[2].val = 0.0;
    
    util.initArray(dTheta, 0.0, 3);
    util.initArray(dVelocity, 0.0, 3);
    sumTime     = 0.0;
    resetPeriod = dynamicsInterval_init;
    
    util.setUnitClassArray(sensorFramePosition, zero_init, meters, 3);
    
    debugFlag = debugFlagIn;
    
    //pMap->addLogVar("IMU accNoGravity[0] (m/s/s)", &accIMUnoGravity[0], savePlot, 2);
    //pMap->addLogVar("IMU accNoGravity[1] (m/s/s)", &accIMUnoGravity[1], savePlot, 2);
    //pMap->addLogVar("IMU accNoGravity[2] (m/s/s)", &accIMUnoGravity[2], savePlot, 2);
    
    //pMap->addLogVar("IMU gravityIMU[0] (m/s/s)", &gravityIMU[0], savePlot, 2);
    //pMap->addLogVar("IMU gravityIMU[1] (m/s/s)", &gravityIMU[1], savePlot, 2);
    //pMap->addLogVar("IMU gravityIMU[2] (m/s/s)", &gravityIMU[2], savePlot, 2);
    
    //pMap->addLogVar("IMU acc[0] (m/s/s)", &accIMU[0], savePlot, 2);
    //pMap->addLogVar("IMU acc[1] (m/s/s)", &accIMU[1], savePlot, 2);
    //pMap->addLogVar("IMU acc[2] (m/s/s)", &accIMU[2], savePlot, 2);
    
    //pMap->addLogVar("IMU gyro x Error (deg/s)", &gyroError[0], savePlot, 2);
    //pMap->addLogVar("IMU gyro y Error (deg/s)", &gyroError[1], savePlot, 2);
    //pMap->addLogVar("IMU gyro z Error (deg/s)", &gyroError[2], savePlot, 2);
    //pMap->addLogVar("IMU acc x Error (g)", &accError[0], savePlot, 2);
    //pMap->addLogVar("IMU acc y Error (g)", &accError[1], savePlot, 2);
    //pMap->addLogVar("IMU acc z Error (g)", &accError[2], savePlot, 2);
    //pMap->addLogVar("IMU mag x Error (uT)", &magError[0], savePlot, 2);
    //pMap->addLogVar("IMU mag y Error (uT)", &magError[1], savePlot, 2);
    //pMap->addLogVar("IMU mag z Error (uT)", &magError[2], savePlot, 2);
    
    //pMap->addLogVar("IMU gyro x (deg/s)", &gyroInUnits[0], savePlot, 2);
    //pMap->addLogVar("IMU gyro y (deg/s)", &gyroInUnits[1], savePlot, 2);
    //pMap->addLogVar("IMU gyro z (deg/s)", &gyroInUnits[2], savePlot, 2);
    //pMap->addLogVar("IMU acc x (g)", &accInUnits[0], savePlot, 2);
    //pMap->addLogVar("IMU acc y (g)", &accInUnits[1], savePlot, 2);
    //pMap->addLogVar("IMU acc z (g)", &accInUnits[2], savePlot, 2);
    //pMap->addLogVar("IMU tangential acc x (g)", &accTangentBody[0], savePlot, 2);
    //pMap->addLogVar("IMU tangential acc y (g)", &accTangentBody[1], savePlot, 2);
    //pMap->addLogVar("IMU tangential acc z (g)", &accTangentBody[2], savePlot, 2);
    //pMap->addLogVar("IMU normal acc x (g)", &accNormalBody[0], savePlot, 2);
    //pMap->addLogVar("IMU normal acc y (g)", &accNormalBody[1], savePlot, 2);
    //pMap->addLogVar("IMU normal acc z (g)", &accNormalBody[2], savePlot, 2);
    //pMap->addLogVar("IMU deltaPitch", &dTheta[1], savePlot, 3);
    //pMap->addLogVar("dTheta X", &dTheta[0], savePlot, 2);
    //pMap->addLogVar("dTheta Y", &dTheta[1], savePlot, 2);
    //pMap->addLogVar("dTheta Z", &dTheta[2], savePlot, 2);
    //pMap->addLogVar("dVelocity X", &dVelocity[0], savePlot, 2);
    //pMap->addLogVar("dVelocity Y", &dVelocity[1], savePlot, 2);
    //pMap->addLogVar("dVelocity Z", &dVelocity[2], savePlot, 2);
    //pMap->addLogVar("dVelocity Dyn X", &dVelocity_dyn[0], savePlot, 2);
    //pMap->addLogVar("dVelocity Dyn Y", &dVelocity_dyn[1], savePlot, 2);
    //pMap->addLogVar("dVelocity Dyn z", &dVelocity_dyn[2], savePlot, 2);
    //pMap->addLogVar("IntSumTime", &sumTime, printSavePlot, 3);
    
    //pMap->addLogVar("accel_roll", &accel_roll, savePlot, 2);
    //pMap->addLogVar("accel_pitch", &accel_pitch, savePlot, 2);
    
    //pMap->addLogVar("lin_accel_roll", &lin_accel_roll, savePlot, 2);
    //pMap->addLogVar("lin_accel_pitch", &lin_accel_pitch, savePlot, 2);
    
    //pMap->addLogVar("lin_accel_roll_error", &lin_accel_roll_error, savePlot, 2);
    //pMap->addLogVar("lin_accel_pitch_error", &lin_accel_pitch_error, savePlot, 2);
    
    //pMap->addLogVar("IMU mag x (uT)", &magInUnits[0], savePlot, 2);
    //pMap->addLogVar("IMU mag y (uT)", &magInUnits[1], savePlot, 2);
    //pMap->addLogVar("IMU mag z (uT)", &magInUnits[2], savePlot, 2);
    
    //pMap->addLogVar("IMU gyro x (raw)", &gyroSensor[0], savePlot, 2);
    //pMap->addLogVar("IMU gyro y (raw)", &gyroSensor[1], savePlot, 2);
    //pMap->addLogVar("IMU gyro z (raw)", &gyroSensor[2], savePlot, 2);
    //pMap->addLogVar("IMU acc x (raw)", &accSensor[0], savePlot, 2);
    //pMap->addLogVar("IMU acc y (raw)", &accSensor[1], savePlot, 2);
    //pMap->addLogVar("IMU acc z (raw)", &accSensor[2], savePlot, 2);
    //pMap->addLogVar("IMU mag x (raw)", &magSensor[0], savePlot, 2);
    //pMap->addLogVar("IMU mag y (raw)", &magSensor[1], savePlot, 2);
    //pMap->addLogVar("IMU mag z (raw)", &magSensor[2], savePlot, 2);
}

void IMUModelBase::initialize(void)
{
    pDyn    = (DynamicsModel*)   pMap->getModel("DynamicsModel");
    pAtmo   = (AtmosphereModel*) pMap->getModel("AtmosphereModel");
    pRotate = (RotateFrame*)     pMap->getModel("RotateFrame");
    pTime   = (Time*)            pMap->getModel("Time");
    
    for (int i=0; i<3; i++)
    {
        gyroNoise[i] = gyroNoiseMax[i];
        accNoise[i] = accNoiseMax[i];
    }
}

bool IMUModelBase::update(void)
{
    util.setUnitClassUnit(bodyRates, radiansPerSecond, 3);
    
    // Update Models
    pRotate->update();
    pAtmo->update();
    
    // Update States
    util.setUnitClassArray(bodyRates  , pDyn->getBodyRates()  , radiansPerSecond, 3);
    util.setUnitClassArray(eulerAngles, pDyn->getEulerAngles(), radians, 3);
    util.setArray(bodyAcc    , pDyn->getAccBody()    , 3);
    util.setArray(bodyAngularAcc, pDyn->getbodyAngularAcc(), 3);
    gravity = pAtmo->getGravity();
    util.setArray(gravityBody, pAtmo->getGravityBody(), 3);
    //magneticStrength = pAtmo->getMagneticFieldStrength();
    //inclination      = pDyn->getInclination();
    
    // Update Units
    util.setUnitClassUnit(bodyRates, degreesPerSecond, 3);
    util.setArray(bodyRatesPrint, bodyRates, 3);
    
    // Update IMU measurements
    gyroscopeModel();
    accelerometerModel();
    magnetometerModel();
    
    accelerometerAttitude();
    
    imuReady = true;
    
    if (debugFlag)
    {
        // Gyroscope
        util.print(bodyRates, 3, "Body Rates");
        util.print(gyroIMU, 3, "True IMU Gyroscope");
        util.print(gyroError, 3, "Gyroscope Errors");
        util.print(gyroInUnits, 3, "Gyroscope In Units");
        util.print(gyroSensor, 3, "Gyroscope Raw");
        
        // Accelerometer
        util.print(bodyAcc, 3, "Body Acceleration");
        util.print(accTangentBody, 3, "Body Tangent Acceleration");
        util.print(accNormalBody, 3, "Body Normal Acceleration");
        util.print(accTotalBody, 3, "Body Total Acceleration");
        util.print(gravityIMU, 3, "Gravity In IMU Frame");
        util.print(accIMUg, 3, "True IMU Accelerometer");
        util.print(accError, 3, "Accelerometer Errors");
        util.print(accInUnits, 3, "Acclerometer In Units");
        util.print(accSensor, 3, "Accelerometer Raw");
        
        // Integrals
        util.print(dTheta, 3, "dTheta");
        util.print(dVelocity, 3, "dVelocity");
        
        // Magnetometer
        util.print(magNED, 3, "NED Magnetic Field");
        util.print(magIMU, 3, "True IMU Magnetometer");
        util.print(magError, 3, "Magnetometer Errors");
        util.print(magWithSoftIron, 3, "Magnetometer With Soft Iron");
        util.print(magInUnits, 3, "Magnetometer In Units");
        util.print(magSensor, 3, "Magnetometer Raw");
    }
    
    return true;
}

void IMUModelBase::gyroscopeModel(void)
{
    pRotate->bodyToImu(gyroIMU, bodyRates);
    
    // Add Error
    for(int i=0; i<3; i++) { gyroError[i] = gyroNoise[i] + gyroBias[i]; }
    
    if (perfectSensor)
    {
        util.setArray(gyroInUnits, gyroIMU, 3);
    }
    else
    {
        util.vAdd(gyroInUnits, gyroIMU, gyroError, 3);
    }
    
    // Raw Body Rate
    util.setArray(gyroSensor, gyroInUnits, 3);
    util.vgain(gyroSensor, LSBdps, 3);
}

void IMUModelBase::accelerometerModel(void)
{
    // A_imu = A_body + cross(angleAcc, r) + angleRate x (angleRate x r);
    // A_imu = A_body +     A_tangent      +         A_normal
    
    // Tangent Acceleration
    util.crossProduct(accTangentBody, bodyAngularAcc, sensorFramePosition);
    
    // Normal Acceleration
    util.crossProduct(velNormalBody, bodyRates, sensorFramePosition);
    util.crossProduct(accNormalBody, bodyRates, velNormalBody);
    
    // Total Body Acceleration
    util.vAdd(accRotationBody, accTangentBody, accNormalBody, 3);
    util.vAdd(accTotalBody, bodyAcc, accRotationBody, 3);
    
    // Gravity
    pRotate->bodyToImu(gravityIMU, gravityBody);
    
    // Gravity relative to free-fall
    util.vgain(gravityIMU, -1.0, 3);
    
    // IMU Acceleration
    // Acceleration in IMU frame
    pRotate->bodyToImu(accIMUnoGravity, accTotalBody);
    
    // Add gravity
    util.vAdd(accIMU, accIMUnoGravity, gravityIMU, 3);
    
    if (onlyGravity) { util.setArray(accIMU, gravityIMU, 3); }
    
    // Acceleration in g's
    util.setArray(accIMUg, accIMU, 3);
    util.vgain(accIMUg, 1.0/refGravity, 3);
    
    // Add Error
    //util.vAdd(accError, accBias, randomNoiseModel(accNoiseMax), 3);
    for(int i=0; i<3; i++) { accError[i] = accNoise[i] + accBias[i]; }
    
    if (perfectSensor)
    {
        util.setArray(accInUnits, accIMUg, 3);
    }
    else
    {
        util.vAdd(accInUnits, accIMUg, accError, 3);
    }
    
    // Raw Acceleration
    util.setArray(accSensor, accInUnits, 3);
    util.vgain(accSensor, LSBg, 3);
}

void IMUModelBase::deltaIMU(double dt)
{
    double accelBody[3];
    double gyroBody[3];
    
    pRotate->imuToBody(gyroBody, getGyroscopeDps());
    pRotate->imuToBody(accelBody, getAccelerometerGs());
    util.vgain(accelBody, refGravity, 3);
    
    
    for (int i=0; i<3; i++)
    {
        dTheta[i]    += gyroBody[i]*dt;
        dVelocity[i] += accelBody[i]*dt;
    }
    util.vSubtract(dVelocity_dyn, pDyn->getVelBody(), preVelBody, 3);
    sumTime += dt;
}

void IMUModelBase::reset()
{
    util.setArray(preVelBody, pDyn->getVelBody(), 3);
    util.setArray(dTheta, zero_init, 3);
    util.setArray(dVelocity, zero_init, 3);
    util.setArray(dVelocity_dyn, zero_init, 3);
    sumTime = 0.0;
}

void IMUModelBase::magnetometerModel(void)
{
    // Earth magnetic field
    magNED[0] = magneticStrength*cos(inclination*util.deg2rad);
    magNED[1] = 0;
    magNED[2] = magneticStrength*sin(inclination*util.deg2rad);
    
    // Rotate Earth magnetic field into sensor frame
    pRotate->NEDToBody(magBody, magNED);
    pRotate->bodyToImu(magIMU, magBody);

    // Add Error
    util.vAdd(magError, magBias, randomNoiseModel(magNoiseMax), 3);
    
    if (perfectSensor)
    {
        util.setArray(magInUnits, magIMU, 3);
    }
    else
    {
        util.mmult(magWithSoftIron, *SoftIronMatrix, magIMU, 3, 1);
        util.vAdd(magInUnits, magWithSoftIron, magError, 3);
    }
    
    // Raw Magnetic Field
    util.setArray(magSensor, magInUnits, 3);
    util.vgain(magSensor, LSBg, 3);
}

void IMUModelBase::accelerometerAttitude()
{
    double accel[3];
    double Myz;
    
    util.setArray(accel, gravityBody, 3);
    util.vgain(accel, -1.0, 3); // Relative to free-fall
    util.setArray(unitGravityBody, accel, 3);
    util.unitVector(unitGravityBody, 3);
    
    Myz = sqrt(accel[1]*accel[1] + accel[2]*accel[2]);
    accel_roll = -atan2(accel[1], -accel[2]) / util.deg2rad;
    accel_pitch = atan2(accel[0], Myz) / util.deg2rad;

    
    // Linear Model Around 0,0
    // lin_accel_roll = -1.0*accel[1]/refGravity / util.deg2rad;
    // lin_accel_pitch = accel[0]/refGravity / util.deg2rad;
    double accel_with_bias[3];
    double bias[3];
    double M, M2;
    double Myz2;
    double M2Myz;
    double a[3], b[3], x0[2], dx[2], du[6], A[2][6];
    
    bias[0] = accBias[0]*refGravity;
    bias[1] = accBias[1]*refGravity;
    bias[2] = accBias[2]*refGravity;
    util.vAdd(accel_with_bias, accel, bias, 3);
    
    a[0] = bias[0];
    a[1] = bias[1];
    a[2] = bias[2] - gravity;
    
    b[0] = bias[0];
    b[1] = bias[1];
    b[2] = bias[2];

    M     = sqrt((a[0]-b[0])*(a[0]-b[0])+ (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
    M2    = M*M;
    Myz   = sqrt((a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
    Myz2  = Myz*Myz;
    M2Myz = M2*Myz;
    
    x0[0] = atan((a[1] - b[1])/(a[2] - b[2]));
    x0[1] = atan((a[0] - b[0])/Myz);

    du[0] = accel[0] - a[0];
    du[1] = accel[1] - a[1];
    du[2] = accel[2] - a[2];
    du[3] = 0.0 - b[0];
    du[4] = 0.0 - b[1];
    du[5] = 0.0 - b[2];
    
    A[0][0] = 0.0;
    A[0][1] = (a[2] - b[2])/Myz2;
    A[0][2] = -(a[1] - b[1])/Myz2;
    A[0][3] = -A[0][0];
    A[0][4] = -A[0][1];
    A[0][5] = -A[0][2];
    
    A[1][0] = Myz/M2;
    A[1][1] = -((a[0] - b[0])*(a[1] - b[1]))/M2Myz;
    A[1][2] = -((a[0] - b[0])*(a[2] - b[2]))/M2Myz;
    A[1][3] = -A[1][0];
    A[1][4] = -A[1][1];
    A[1][5] = -A[1][2];
    
    util.mmult(dx, *A, du, 2, 6);
    
    lin_accel_roll  = (x0[0] + dx[0]) / util.deg2rad;
    lin_accel_pitch = (x0[1] + dx[1]) / util.deg2rad;
    
    //std::cout << pTime->getSimTime() << ") [Roll, Pitch] = [" << accel_roll << " " << lin_accel_roll << "], ";
    //std::cout << "[" << accel_pitch << " " << lin_accel_pitch << "]" << std::endl;
    //util.print(du, 6, "du");
    //util.print(*A, 2, 6, "A");

    lin_accel_roll_error  = lin_accel_roll  - accel_roll;
    lin_accel_pitch_error = lin_accel_pitch - accel_pitch;
    
    // Bias testing
    double a0[3] = {0.0, 0.0, -gravity};
    double b0[3] = {0.0, 0.0, 0.0};
    double Myz0  = sqrt((a0[1]-b0[1])*(a0[1]-b0[1]) + (a0[2]-b0[2])*(a0[2]-b0[2]));
    
    double a1[3] = {0.0, 0.0, -gravity};
    double b1[3] = {0.08*gravity, 0.0, 0.0};
    double Myz1  = sqrt((a1[1]-b1[1])*(a1[1]-b1[1]) + (a1[2]-b1[2])*(a1[2]-b1[2]));
    double x1[2];
    
    x0[0] = atan((a0[1] - b0[1])/(a0[2] - b0[2])) / util.deg2rad;
    x0[1] = atan((a0[0] - b0[0])/Myz0) / util.deg2rad;
    
    x1[0] = atan((a1[1] - b1[1])/(a1[2] - b1[2])) / util.deg2rad;
    x1[1] = atan((a1[0] - b1[0])/Myz1) / util.deg2rad;
    
    double diff[2];
    double lin_diff[2];
    double d[6];
    diff[0] = x1[0] - x0[0];
    diff[1] = x1[1] - x0[1];
    for (int i = 0; i < 3; i++)
    {
        d[i] = a1[i] - a0[i];
        d[i+3] = b1[i] - b0[i];
    }
    util.mmult(lin_diff, *A, d, 2, 6);
    util.vgain(lin_diff, 1.0/util.deg2rad, 3);
}

QuadcopterIMUModel::QuadcopterIMUModel(ModelMap *pMapInit, bool debugFlagIn) : IMUModelBase(pMapInit, debugFlagIn)
{
    // LSB
    LSBdps = 32.8;
    LSBg   = 4096;
    LSBuT  = 16;
    
    // Noise
    gyroBias[0] = 0.02;  // deg/s
    gyroBias[1] = -0.08; // deg/s
    gyroBias[2] = 0.05;  // deg/s
    gyroNoiseMax[0] = 0.23;  // deg/s
    gyroNoiseMax[1] = 0.27;  // deg/s
    gyroNoiseMax[2] = 0.26;  // deg/s
    
    accBias[0] = 0.01;     // g's
    accBias[1] = -0.03;    // g's
    accBias[2] = 0.08;     // g's
    accNoiseMax[0] = 0.02; // g's
    accNoiseMax[1] = 0.02; // g's
    accNoiseMax[2] = 0.02; // g's

    magBias[0] = 1.0;
    magBias[1] = -2.0;
    magBias[2] = -23.0;
    magNoiseMax[0] = 0.5;
    magNoiseMax[1] = 1.0;
    magNoiseMax[2] = 0.5;
    SoftIronMatrix[0][0] = 1.0; SoftIronMatrix[0][1] = 0.0; SoftIronMatrix[0][2] = 0.0;
    SoftIronMatrix[1][0] = 0.0; SoftIronMatrix[1][1] = 1.0; SoftIronMatrix[1][2] = 0.0;
    SoftIronMatrix[2][0] = 0.0; SoftIronMatrix[2][1] = 0.0; SoftIronMatrix[2][2] = 1.0;

    magneticStrength = 55;
    inclination = 70;
    
    resetPeriod = 1.0/50.0;
    
    onlyGravity = false;
}
