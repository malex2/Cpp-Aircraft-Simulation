//
//  DataLogger.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 5/31/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef DataLogger_hpp
#define DataLogger_hpp

#include <stdio.h>
#include "generic_model.hpp"

class DataLogger : public GenericModel
{
    std::string savefile;
    std::ofstream outfile;
    
public:
    DataLogger(ModelMap *pMapInit, bool debugFlag = false, std::string filename = "output.csv");
    
    ~DataLogger(void);
    
    void printLog(void);
    
    void saveLog(void);
    
    void plotLog(void);
    
};

#endif /* DataLogger_hpp */
