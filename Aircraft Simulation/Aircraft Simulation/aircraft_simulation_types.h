//
//  aircraft_simulation_types.h
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/6/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef aircraft_simulation_types_h
#define aircraft_simulation_types_h

// Controller input mode
//enum inputModeType {none, keyboard, serial, table};

// Actuator types
//enum actuatorType  {de, da, dr, dT};

// Aero types
enum coeffType {iCd, iCL, iCX, iCY, iCZ, iCl, iCm, iCn, nCoeff};
enum coeffDervType {constant, du, dalpha, dbeta, dbp, dbq, dbr, delevator, daileron, drudder, dthrottle, nDeriv};

// Atmosphere types

#endif /* aircraft_simulation_types_h */
