//
//  Coordinate Transformation.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/17/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef RotateFrame_hpp
#define RotateFrame_hpp

#include "generic_model.hpp"

class RotateFrame : public GenericModel {
 
public:
    // Constructor
    RotateFrame(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update rotations
    virtual bool update(void);
    
    // NED
    void bodyToNED(float *NEDFrame, float *bodyFrame);
    template<typename valType, template<typename T> class unitType>
    void bodyToNED(unitType<valType> *NEDFrame, unitType<valType> *bodyFrame);
    
    void NEDToBody(float *bodyFrame, float *NEDFrame);
    template<typename valType, template<typename T> class unitType>
    void NEDToBody(unitType<valType> *bodyFrame, unitType<valType> *NEDFrame);
 
    // Local Level
    void bodyToLL(float *LLFrame, float *bodyFrame);
    template<typename valType, template<typename T> class unitType>
    void bodyToLL(unitType<valType> *LLFrame, unitType<valType> *bodyFrame);
    
    void LLToBody(float *bodyFrame, float *LLFrame);
    template<typename valType, template<typename T> class unitType>
    void LLToBody(unitType<valType> *bodyFrame, unitType<valType> *LLFrame);
    
    // Sensors
    void imuToBody(float *bodyFrame, float *imuFrame);
    void bodyToImu(float *imuFrame, float *bodyFrame);
    
    // Wind
    void windToBody(float *bodyFrame, float *windFrame);
    void bodyToWind(float *windFrame, float *bodyFrame);
    
    // Angle Rates
    void eulerRateToBodyRate(AngleRateType<float> *bodyRates, AngleRateType<float> *eulerRates);
    void bodyRateToEulerRate(AngleRateType<float> *eulerRates, AngleRateType<float> *bodyRates);

private:
    
    // Classes
    class DynamicsModel *pDyn;
    class AeroModelBase *pAero;
    
    // Variables
    AngleType<float> imuFrame[3];
    AngleType<float> aeroEuler[3];
    AngleType<float> eulerAngles[3];
    
    float q_B_NED[4];     // NED to body rotation
    float q_NED_B[4];     // Body to NED rotation
    
    float R_B_NED[3][3]; // NED to body matrix (for reference)
    float R_NED_B[3][3]; // Body to NED matrix (for reference)
    
    float q_B_LL[4];     // NED to body rotation
    float q_LL_B[4];     // Body to NED rotation
    
    float R_B_LL[3][3];  // LL to body (for reference)
    float R_LL_B[3][3];  // Body to LL (for reference)
    
    float T_B_imu[3][3]; // IMU to body matrix
    float T_imu_B[3][3]; // Body to IMU matrix
    
    float R_B_W[3][3]; // Wind to body matrix
    float R_W_B[3][3]; // Body to wind matrix
    
    float L_B_E[3][3]; // Euler rates to body rates
    float L_E_B[3][3]; // Body rates to Euler rates
};

#endif /* RotateFrame_hpp */
