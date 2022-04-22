//  Created by Alexander McLean on 08/09/19.//  Copyright © 2019 Alexander McLean. All rights reserved.#include <iostream>#include <chrono>#include <math.h>#include <fstream>#include <algorithm>#include "transfer_function_utilities.hpp"#include "utilities.hpp"#include "initial_conditions.hpp"#include "model_mapping.hpp"#include "data_logger.hpp"#include "time.hpp"#include "rotate_frame.hpp"#include "ground_model.hpp"#include "imu_model.hpp"#include "barometer_model.hpp"#include "dynamics_model.hpp"#include "atmosphere_model.hpp"#include "propulsion_model.hpp"#include "aero_model.hpp"#include "actuator_model.hpp"#include "flight_software.hpp"//#include "performance.hpp"int main() {    // Setup and initialize models    Utilities util;    ModelMap map(false);     DataLogger         dataLog(&map, false, savefile);    Time               time(&map);    DynamicsModel      dynamicsModel(&map, false);    RotateFrame        rotate(&map);    QuadcopterIMUModel imu(&map, false);    bmp180             barometer(&map, false);    //RCPropulsionModel  propulsionModel(&map, false);    QuadcopterPropulsionModel  propulsionModel(&map, false);    //RCActuatorModel    actuatorModel(&map, false);    QuadcopterActuatorModel    actuatorModel(&map, false);    //RCAeroModel        aeroModel(&map);    QuadcopterAeroModel        aeroModel(&map);    AtmosphereModel    atmosphereModel(&map, false);    //RCGroundModel      groundModel(&map, false);    QuadcopterGroundModel      groundModel(&map, false);    //Performance        performance(&map);        map.addModel("Time", &time);    map.addModel("DynamicsModel", &dynamicsModel ,forceModel);    map.addModel("RotateFrame", &rotate);    map.addModel("IMUModel", &imu);    map.addModel("BarometerModel", &barometer);    map.addModel("ActuatorModel", &actuatorModel);    map.addModel("PropulsionModel", &propulsionModel, forceModel);    map.addModel("AeroModel", &aeroModel, forceModel);    map.addModel("AtmosphereModel", &atmosphereModel, forceModel);    map.addModel("GroundModel", &groundModel, forceModel);    //map.addModel("Performance", &performance);        // Prep loop    const int nModels = 8;    bool continueSim[nModels];    std::fill_n(continueSim, nModels, true);        int simLoopCount = 0;    int dynLoopCount = 0;        dynamicsModel.initialize();   // body angles, position, and velocity    aeroModel.initialize();       // aero angles    rotate.initialize();          // rotation matrices    imu.initialize();    barometer.initialize();    atmosphereModel.initialize(); // Re, Mach, air properties        actuatorModel.initialize();    // set propulsion throttle    propulsionModel.initialize();  //    groundModel.initialize();      // set contact point loading        // Set flags    imu.setPerfectSensor(false);    barometer.setPerfectSensor(false);    flightSoftware_setMapPointer(&map);        while( util.all(continueSim, nModels) )    {        imu.setIMUReady(false);                if ( time.performDynamics() )        {            continueSim[0] = rotate.update(); // Update rotation matrices and quaternions                    // Update forces            continueSim[1] = atmosphereModel.update(); // Compute gravity body forces            continueSim[3] = actuatorModel.update();   // Compute actuator position (must be before propulsion and aero)            continueSim[2] = propulsionModel.update(); // Compute propulsion forces (must be before aero)            continueSim[4] = aeroModel.update();       // Compute aero body forces            continueSim[5] = groundModel.update();     // Compute ground body forces                        // Update dynamics            continueSim[6] = dynamicsModel.update(); // Compute position, velocity, attitude, and angular rates from forces and moments                        // Update hardware models            imu.update();            barometer.update();                        dynLoopCount++;            //std::cout<<"dynamics "<<dynLoopCount<<std::endl;        }                // Update GNC        mainFlightSoftware();                if (time.performPrint() && printOutput)   { dataLog.printLog(); }        if (time.performSave() && saveOutput)     { dataLog.saveLog(); }                // Update time        continueSim[7] = time.update();                simLoopCount++;        //std::cout<<"sim "<<simLoopCount<<std::endl;    }    return 0;}