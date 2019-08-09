//
//  Coordinate Transformation.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/17/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef RotateFrame_hpp
#define RotateFrame_hpp

#include <stdio.h>
#include <stdio.h>
#include <math.h>
#include <iostream>

#endif /* Coordinate_Transformation_hpp */

// Create functions that use quaternions
class RotateFrame
{

public:
    void RotateInertialToBody(float phi,float theta,float psi, float *pFrame1, float *pFrame2);
    void RotateBodyToInertial(float phi,float theta,float psi, float *pFrame1, float *pFrame2);
    void RotateAngularRatesInertialToBody(float roll,float pitch,float *omegaB,float *omegaI);
    void RotateAngularRatesBodyToInertial(float roll,float pitch,float *omegaB,float *omegaI);
private:
    float ctheta;
    float cpsi;
    float cphi;
    float stheta;
    float spsi;
    float sphi;
    float ttheta;
    float sectheta;
};



