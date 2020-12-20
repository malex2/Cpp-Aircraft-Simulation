//
//  model_mapping.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 6/13/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "model_mapping.hpp"
#include "generic_model.hpp"

ModelMap::ModelMap(bool debugFlagIn)
{
    debugFlag = debugFlagIn;
    iPrintNames = 0;
}

void ModelMap::addModel(std::string key, GenericModel *pModel, mapType curMap)
{
    // Add to model map
    if (debugFlag) { std::cout << "Adding " << key << " to model map." << std::endl; }
    modelMapping.insert( std::pair<std::string , GenericModel*> (key, pModel) );
    
    // Add to force models
    if (curMap == forceModel)
    {
        if (debugFlag) { std::cout << "Adding " << key << " to force model map." << std::endl; }
        forceModels.insert( std::pair<std::string , GenericForceModel*> (key, (GenericForceModel*) pModel) );
    }
}

void ModelMap::addLogVar(std::string name, double *var, const mapType *mappings, int nMappings)
{
    for (int curMap = 0; curMap < nMappings; curMap++)
    {
        if (mappings[curMap] == printVar )
        {
            if (debugFlag) { std::cout << "Adding " << iPrintNames << " " << name << " to print list." << std::endl; }
            printMapping.insert( std::pair<int, double*> (iPrintNames, var) );
            printNames.push_back(name);
            iPrintNames++;
        }
    
        if (mappings[curMap] == saveVar )
        {
            if (debugFlag) { std::cout << "Adding " << iSaveNames << " " << name << " to save list." << std::endl; }
            saveMapping.insert( std::pair<int, double*> (iSaveNames, var) );
            saveNames.push_back(name);
            iSaveNames++;
        }
    
        if (mappings[curMap] == plotVar )
        {
            if (debugFlag) { std::cout << "Adding " << iPlotNames << " " << name << " to plot list." << std::endl; }
            plotMapping.insert( std::pair<int, double*> (iPlotNames, var) );
            plotNames.push_back(name);
            iPlotNames++;
        }
    }
}

// Get model from key
GenericModel* ModelMap::getModel(std::string key)
{
     if (debugFlag) std::cout << "Getting Model: " << modelMapping.find(key)->first << std::endl;
    return modelMapping.find(key)->second;
}

GenericForceModel* ModelMap::getForceModel(std::string key)
{
    if (debugFlag) std::cout << "Getting Force Model: " << forceModels.find(key)->first << std::endl;
    return forceModels.find(key)->second;
}

std::string ModelMap::getLogVarName(int itr, mapType curMap)
{
    std::string returnVal;// = NULL;
    
    if (curMap == printVar )
    {
        if (debugFlag) std::cout << "Getting Print Var: " << printNames[itr] << std::endl;
        returnVal = printNames[itr];
    }
    
    if (curMap == saveVar )
    {
        if (debugFlag) std::cout << "Getting Print Var: " << saveNames[itr] << std::endl;
        returnVal = saveNames[itr];
    }
    
    if (curMap == plotVar )
    {
        if (debugFlag) std::cout << "Getting Print Var: " << plotNames[itr] << std::endl;
        returnVal = plotNames[itr];
    }
    
    return returnVal;
}

// Get model from itr
GenericModel* ModelMap::getModel(iModel itr)
{
    if (debugFlag) std::cout << "Getting Model: " << itr->first << std::endl;
    return itr->second;
}

GenericForceModel* ModelMap::getForceModel(iForceModel itr)
{
    if (debugFlag) std::cout << "Getting Force Model: " << itr->first << std::endl;
    return itr->second;
}

double* ModelMap::getLogVar(iLog itr)
{
    if (debugFlag) std::cout << "Getting Variable: " << itr->first << std::endl;
    return itr->second;
}

iLog ModelMap::getFirstLogVar(mapType curMap)
{
    iLog returnVal;
    
    if (curMap == printVar )
    {
        returnVal = printMapping.begin();
    }
    
    if (curMap == saveVar )
    {
        returnVal = saveMapping.begin();
    }
    
    if (curMap == plotVar )
    {
        returnVal = plotMapping.begin();
    }
    
    return returnVal;
}

iLog ModelMap::getLogVarEnd(mapType curMap)
{
    iLog returnVal;
    
    if (curMap == printVar )
    {
        returnVal = printMapping.end();
    }
    
    if (curMap == saveVar )
    {
        returnVal = saveMapping.end();
    }
    
    if (curMap == plotVar )
    {
        returnVal = plotMapping.end();
    }
    
    return returnVal;
}

std::size_t ModelMap::getLogVarSize(mapType curMap)
{
    std::size_t returnVal = 0;
    
    if (curMap == printVar )
    {
        returnVal = printNames.size();
    }
    
    if (curMap == saveVar )
    {
        returnVal = saveNames.size();
    }
    
    if (curMap == plotVar )
    {
        returnVal = plotNames.size();
    }
    
    return returnVal;
}
