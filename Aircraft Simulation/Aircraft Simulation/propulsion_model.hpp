//
//  propulsion_model.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 6/28/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef propulsion_model_hpp
#define propulsion_model_hpp

#include "generic_model.hpp"

class PropulsionModel : public GenericForceModel
{
public:
    // Constructor
    PropulsionModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    float getThrottle(void)             { return throttle; }
    void  setThrottle(float throttleIn) { throttle = throttleIn; }
    
private:
    class DynamicsModel *pDyn;
    class RotateFrame   *pRotate;
    
    const float maxThrust = 3; // 0.7*m*g = 0.7*0.43*9.81 = 3 N
    float throttle;
};

#endif /* propulsion_model_hpp */
