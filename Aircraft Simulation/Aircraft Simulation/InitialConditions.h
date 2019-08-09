//
//  InitialConditions.h
//  Aircraft Simulation
//
//  Created by Alexander McLean on 8/9/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef InitialConditions_h
#define InitialConditions_h

#endif // InitialConditions_h

float m          = 3;
float inertia[4] = {0.1,0.2,0.3,0.002};
float elevation  = 0;

float printInterval = 1;
float dynamicsInterval = 0.001;

float xInitial    = 0;
float yInitial    = 0;
float altInitial  = 5;

float xdotInitial = 0;
float ydotInitial = 0;
float zdotInitial = 0;

float zInitial = -altInitial;

float pqr[3];
float omega[3];
float posNED[3];
float velBody[3];
float velNED[3];

