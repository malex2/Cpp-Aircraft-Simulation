//
//  general_model.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 6/12/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef general_model_hpp
#define general_model_hpp

#include <stdio.h>

#include "utilities.hpp"
#include "model_mapping.hpp"

class GenericModel {
protected:
    // Classes
    Utilities util;
    ModelMap  *pMap;
    
    bool debugFlag;
    
    float dt;
    
public:
    // Constructor
    GenericModel();
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void) { };
    
    // Update forces and moments
    virtual bool update(void) { return true; };
};

class GenericForceModel : public GenericModel {
protected:
    // Forces and moments
    float LLForce[3];
    float bodyForce[3];
    float bodyMoment[3];
    
public:
    // Constructor
    GenericForceModel();
    
    // Return forces and moments
    float* getForce(void) { return bodyForce; }
    float* getMoment(void) { return bodyMoment; }
};

// Example class
class ExampleModel : public GenericForceModel
{
public:
    // Constructor
    ExampleModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
private:
    class DynamicsModel *pDyn;
    class RotateFrame   *pRotate;
    class Time          *pTime;
};

#endif /* general_model_hpp */
