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
    void bodyToNED(double *NEDFrame, double *bodyFrame);
    template<typename valType, template<typename T> class unitType>
    void bodyToNED(unitType<valType> *NEDFrame, unitType<valType> *bodyFrame);
    
    void NEDToBody(double *bodyFrame, double *NEDFrame);
    template<typename valType, template<typename T> class unitType>
    void NEDToBody(unitType<valType> *bodyFrame, unitType<valType> *NEDFrame);
 
    // Local Level
    void bodyToLL(double *LLFrame, double *bodyFrame);
    template<typename valType, template<typename T> class unitType>
    void bodyToLL(unitType<valType> *LLFrame, unitType<valType> *bodyFrame);
    
    void LLToBody(double *bodyFrame, double *LLFrame);
    template<typename valType, template<typename T> class unitType>
    void LLToBody(unitType<valType> *bodyFrame, unitType<valType> *LLFrame);
    
    // Sensors
    void imuToBody(double *bodyFrame, double *imuFrame);
    void bodyToImu(double *imuFrame, double *bodyFrame);
    
    // Wind
    void windToBody(double *bodyFrame, double *windFrame);
    void bodyToWind(double *windFrame, double *bodyFrame);
    
    // Angle Rates
    void eulerRateToBodyRate(AngleRateType<double> *bodyRates, AngleRateType<double> *eulerRates);
    void bodyRateToEulerRate(AngleRateType<double> *eulerRates, AngleRateType<double> *bodyRates);

private:
    
    // Classes
    class DynamicsModel *pDyn;
    class AeroModelBase *pAero;
    
    // Variables
    AngleType<double> imuFrame[3];
    AngleType<double> aeroEuler[3];
    AngleType<double> eulerAngles[3];
    
    double q_B_NED[4];     // NED to body rotation
    double q_NED_B[4];     // Body to NED rotation
    
    double R_B_NED[3][3]; // NED to body matrix (for reference)
    double R_NED_B[3][3]; // Body to NED matrix (for reference)
    
    double q_B_LL[4];     // NED to body rotation
    double q_LL_B[4];     // Body to NED rotation
    
    double R_B_LL[3][3];  // LL to body (for reference)
    double R_LL_B[3][3];  // Body to LL (for reference)
    
    double T_B_imu[3][3]; // IMU to body matrix
    double T_imu_B[3][3]; // Body to IMU matrix
    
    double R_B_W[3][3]; // Wind to body matrix
    double R_W_B[3][3]; // Body to wind matrix
    
    double L_B_E[3][3]; // Euler rates to body rates
    double L_E_B[3][3]; // Body rates to Euler rates
};

#endif /* RotateFrame_hpp */
