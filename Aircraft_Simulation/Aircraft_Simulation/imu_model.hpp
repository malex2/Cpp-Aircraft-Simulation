//
//  imu_model.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/28/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef imu_model_hpp
#define imu_model_hpp

#include "generic_model.hpp"

class IMUModelBase : public GenericModel
{
public:
    // Constructor
    IMUModelBase(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    // getters
    double* getGyroscope(void)     { return gyroSensor; }
    double* getAccelerometer(void) { return accSensor; }
    double* getMagnetometer(void)  { return magSensor; }
    
    AngleType<double>* getIMUEuler(void) { return sensorFrameEuler; }
    bool getIMUReady(void)               { return imuReady; }
    
    // setters
    void setPerfectSensor(bool input) { perfectSensor = input; }
    void setLSBdps(double LSBdps_in)  { LSBdps = LSBdps_in;    }
    void setLSBg(double LSBg_in)      { LSBg   = LSBg_in;      }
    void setLSBuT(double LSBuT_in)    { LSBuT = LSBuT_in;      }
    void setIMUReady(bool readyIn)    { imuReady = readyIn;    }
protected:
    // Functions
    void gyroscopeModel(void);
    void accelerometerModel(void);
    void magnetometerModel(void);
    double* noiseModel(double* maxNoise);
    double noiseModel(double maxNoise);
    
    // units to LSB
    double LSBdps; // degress per second to LSB
    double LSBg;   // g's to to LSB
    double LSBuT;  // micro Tesla to LSB
    double refGravity;
    bool perfectSensor;
    bool imuReady;
    
    // _Body   = IMU value without errors in body frame
    // _IMU    = IMU value without errors in IMU frame
    // _Error  = IMU errors in IMU frame (bias + noise)
    // _InUnit = _IMU + _Error
    // _Sensor = _InUnit / LSB;
    
    // accInUnits = accIMU + accError
    // accIMU = accInUnits / LSBg
    // Noise
    double noise[3];
    
    double gyroBias[3];
    double gyroNoiseMax[3];
    double gyroError[3];
    
    double accBias[3];
    double accNoiseMax[3];
    double accError[3];
    
    double magBias[3];
    double magNoiseMax[3];
    double SoftIronMatrix[3][3];
    double magWithSoftIron[3]; // True reading biased by SoftIronMatrix
    double magError[3];
    
    // Outputs in LSB
    double gyroSensor[3];
    double accSensor[3];
    double magSensor[3];
    
    // Outputs in Units
    double gyroInUnits[3];
    double accInUnits[3];
    double magInUnits[3];
    
    // Internal Variables
    // From Other Models
    AngleRateType<double> bodyRates[3];
    AngleType<double> eulerAngles[3];
    double bodyAcc[3];
    double bodyAngularAcc[3];
    double gravity;
    double magneticStrength;
    double inclination;
    
    // IMU processing variables
    //double gyroBody[3];
    double gyroIMU[3];
    
    //double accBody[3];
    double accTangentBody[3];
    double accNormalBody[3];
    SpeedType<double> velNormalBody[3];
    double accRotationBody[3];
    double accTotalBody[3];
    double gravityNED[3];
    double gravityBody[3];
    double gravityIMU[3];
    double accIMU[3];
    
    double magNED[3];
    double magBody[3];
    double magIMU[3];
    
    AngleType<double>    sensorFrameEuler[3]; // Euler Angles from body frame to sensor frame
    DistanceType<double> sensorFramePosition[3]; // Position of sensor frame with respect to the body frame
    
    // Print Variables
    double bodyRatesPrint[3];
    
    // Classes
    class DynamicsModel   *pDyn;
    class AtmosphereModel *pAtmo;
    class RotateFrame     *pRotate;
    class Time            *pTime;
};

class QuadcopterIMUModel : public IMUModelBase
{
public:
    // Constructor
    QuadcopterIMUModel(ModelMap *pMapInit, bool debugFlagIn = false);
};

#endif /* imu_model_hpp */
