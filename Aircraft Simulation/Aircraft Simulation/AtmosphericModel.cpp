//
//  Atmosphere.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/28/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#include "AtmosphericModel.hpp"

void AtmosphericModel::gravityModel(float m, float elevation, float alt, float* FgravityNED){

radius = Rearth+elevation+alt;

FgravityNED[0] = 0;
FgravityNED[1] = 0;
FgravityNED[2] = GM*m/(radius*radius);
}
