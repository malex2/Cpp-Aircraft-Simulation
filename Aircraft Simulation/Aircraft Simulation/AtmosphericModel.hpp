//
//  Atmosphere.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/28/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef AtmosphericModel_hpp
#define AtmosphericModel_hpp

#include <stdio.h>

#endif /* Atmosphere_hpp */

class AtmosphericModel
{

public:
    void gravityModel(float m, float elevation, float alt, float* FgravityNED);

private:
    float radius;
    const float Rearth  = 6371e+3;
    const float GM      = 3.9857e+14;

};


