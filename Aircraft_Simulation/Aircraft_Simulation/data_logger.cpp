//
//  DataLogger.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 5/31/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//
#include <iomanip>
#include <fstream>

#include "model_mapping.hpp"
#include "data_logger.hpp"
#include "initial_conditions.hpp"

DataLogger::DataLogger(ModelMap *pMapInit, bool debugFlagIn, std::string filename)
{
    pMap = pMapInit;
    debugFlag = debugFlagIn;
    
    if (saveOutput)
    {
        outfile.open(filename);
    
        if ( outfile.fail() )
        {
            std::cout << "Failed to open output file:" << savefile << "." << std::endl;
        }
    }
}

DataLogger::~DataLogger(void)
{
    if (saveOutput) { outfile.close(); }
}

void DataLogger::printLog(void)
{
    static bool firstTime = true;
    
    // Print header
    if (firstTime)
    {
        for (int itr = 0; itr < pMap->getLogVarSize(printVar); itr++)
        {
            std::cout << pMap->getLogVarName(itr, printVar) <<" ";
        }
        std::cout << std::endl;
        firstTime = false;
    }
    
    // Print Variables
    for (iLog logIter = pMap->getFirstLogVar(printVar); logIter != pMap->getLogVarEnd(printVar); logIter++)
    {
        std::cout << std::fixed << std::setprecision(3) << *pMap->getLogVar(logIter) <<" ";
    }
    std::cout << std::endl;
}

void DataLogger::saveLog(void)
{
    static bool firstTime = true;
    
    if ( !outfile.fail() )
    {
        // Print header
        if (firstTime)
        {
            for (int itr = 0; itr < pMap->getLogVarSize(saveVar); itr++)
            {
                outfile << pMap->getLogVarName(itr, saveVar) <<",";
            }
            outfile << std::endl;
            firstTime = false;
        }
    
        // Print Variables
        for (iLog logIter = pMap->getFirstLogVar(saveVar); logIter != pMap->getLogVarEnd(saveVar); logIter++)
        {
            outfile << std::fixed << std::setprecision(6) << *pMap->getLogVar(logIter) <<",";
        }
        outfile << std::endl;
    }
}

