//
//  fs_thrust_estimator.hpp
//  Aircraft_Simulation
//
//  Created by Alex McLean on 9/16/23.
//  Copyright © 2023 Alexander McLean. All rights reserved.
//

#ifndef fs_thrust_estimator_hpp
#define fs_thrust_estimator_hpp

#include "fs_common.hpp"

struct ThrustEstimatorType {
    double deltaAngle[3];
    double bodyRate[3];
    double bodyAngAccel[3];
    double deltaVz;
    double az;
    double motorRPM[4];
    double motorRPMcmd[4];
    
    double motorThrustEst[4];
    double bodyThrustEst[3];
    double bodyMomentEst[3];
    
    double tau_est;
    double KT_est;
    double torqueRatio_est;
    
    double imuResidual[4];
};

// Setup
void FsThrustEstimator_setup();

// Perform
void rpmUpdate(double &dt);
void FsThrustEstimator_perform(double &dt);
void thrust_update();
void state_update(double &dt);
void variance_update(double &dt);
#ifdef SIMULATION
    void update_truth_errors();
#endif

// Setters
void FsThrustEstimator_setMotorPWMCmd(const double* pwmCmd);
void FsThrustEstimator_performIMUupdate(const double* imuDeltaAngle, const double * imuDeltaVelocity);
void FsThrustEstimator_setGravity(const double* gravity);
void FsThrustEstimator_setVelocity(const double* velocity);

#ifdef SIMULATION
    void FsThrustEstimator_setSimulationModels(ModelMap* pMap);
#endif

// Getters
const ThrustEstimatorType* FsThrustEstimator_getThrustEstimatorData();
#ifdef SIMULATION
const ThrustEstimatorType* FsThrustEstimator_getThrustEstimatorError();
#endif

// Support
double pwmToRPM(double pwm);
void reset_delta();
void update_variables();

#endif /* fs_thrust_estimator_hpp */
