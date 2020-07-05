//
//  model_mapping.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 6/13/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef model_mapping_hpp
#define model_mapping_hpp

#include <iostream>
#include <map>
#include <vector>

// Foward declare classes
class GenericModel;
class GenericForceModel;

// Typedef's
typedef std::map<std::string, GenericModel*>::iterator iModel;
typedef std::map<std::string, GenericForceModel*>::iterator iForceModel;
typedef std::map<int, float*>::iterator iLog;

typedef enum mapType {genericModel, forceModel, printVar, saveVar, plotVar} mapType;

const mapType printSave[2] = { printVar, saveVar };
const mapType printPlot[2] = { printVar, plotVar };
const mapType savePlot[2]  = { saveVar, plotVar };
const mapType printSavePlot[3] = { printVar, saveVar, plotVar };

class ModelMap {
public :
    // Constructor
    ModelMap(bool debugFlagIn = false);
    
    // Add class pointer to map
    void addModel(std::string key, GenericModel *pModel, mapType curMap = genericModel);
    void addLogVar(std::string name, float *var, const mapType *mappings, int nMappings);
    
    // Get pointer from key
    GenericModel*      getModel(std::string key);
    GenericForceModel* getForceModel(std::string key);
    
    // Get pointer from itr
    GenericModel*      getModel(iModel itr);
    GenericForceModel* getForceModel(iForceModel itr);
    float*             getLogVar(iLog itr);
    
    // Get first key in model
    iModel      getFirstModel(void)           { return modelMapping.begin(); }
    iForceModel getFirstModel(mapType curMap) { return forceModels.begin(); }
    iLog        getFirstLogVar(mapType curMap);
    
    // Get last key in model
    iModel      getModelEnd(void)           { return modelMapping.end(); }
    iForceModel getModelEnd(mapType curMap) { return forceModels.end(); }
    iLog        getLogVarEnd(mapType curMap);
    std::size_t getLogVarSize(mapType curMap);
    
    // Get key name from iterator
    std::string getModelName(iModel itr)      { return itr->first; }
    std::string getModelName(iForceModel itr) { return itr->first; }
    std::string getLogVarName(int itr, mapType curMap);
    
private :
    bool debugFlag;
    
    std::map<std::string, GenericModel*> modelMapping;
    std::map<std::string, GenericForceModel*> forceModels;
    std::map<int, float*> printMapping;
    std::map<int, float*> saveMapping;
    std::map<int, float*> plotMapping;
    
    std::vector<std::string> printNames;
    std::vector<std::string> saveNames;
    std::vector<std::string> plotNames;
    
    int iPrintNames, iSaveNames, iPlotNames;
};

#endif /* model_mapping_hpp */
