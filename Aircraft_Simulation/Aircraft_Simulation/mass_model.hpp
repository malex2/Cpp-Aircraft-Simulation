//
//  mass_model.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/30/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef mass_model_hpp
#define mass_model_hpp

#include "generic_model.hpp"

class MassModelBase : public GenericForceModel
{
public:
    // Constructor
    MassModelBase(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
private:
    class DynamicsModel *pDyn;
    class RotateFrame   *pRotate;
    class Time          *pTime;
};

#endif /* mass_model_hpp */
