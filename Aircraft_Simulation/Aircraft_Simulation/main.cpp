//  Created by Alexander McLean on 08/09/19.//  Copyright © 2019 Alexander McLean. All rights reserved.#include <iostream>#include <chrono>#include <math.h>#include <fstream>#include <algorithm>#include "transfer_function_utilities.hpp"#include "utilities.hpp"#include "initial_conditions.hpp"#include "model_mapping.hpp"#include "data_logger.hpp"#include "time.hpp"#include "rotate_frame.hpp"#include "ground_model.hpp"#include "imu_model.hpp"#include "barometer_model.hpp"#include "gps_model.hpp"#include "dynamics_model.hpp"#include "atmosphere_model.hpp"#include "propulsion_model.hpp"#include "aero_model.hpp"#include "actuator_model.hpp"#include "telemetry_monitor_model.hpp"#include "flight_software.hpp"#include "ground_station.hpp"//#include "performance.hpp"int main() {    // Setup and initialize models    Utilities util;    ModelMap map(false);    DataLogger         dataLog(&map, false, savefile);    Time               time(&map);    DynamicsModel      dynamicsModel(&map, false);    RotateFrame        rotate(&map);    QuadcopterIMUModel imu(&map, false);    bmp180             barometer(&map, false);    GPSNeo6m           gps(&map, false);    //RCPropulsionModel  propulsionModel(&map, false);    QuadcopterPropulsionModel  propulsionModel(&map, false);    //RCActuatorModel    actuatorModel(&map, false);    QuadcopterActuatorModel    actuatorModel(&map, false);    //RCAeroModel        aeroModel(&map);    QuadcopterAeroModel        aeroModel(&map);    AtmosphereModel    atmosphereModel(&map, false);    //RCGroundModel      groundModel(&map, false);    QuadcopterGroundModel      groundModel(&map, false);    //Performance        performance(&map);    QuadcopterTelemetry telemetry(&map, false);    TwoHardwareSerial    FS_GS_Interface(&map, false);        map.addModel("Time", &time);    map.addModel("DynamicsModel", &dynamicsModel ,forceModel);    map.addModel("RotateFrame", &rotate);    map.addModel("IMUModel", &imu);    map.addModel("BarometerModel", &barometer);    map.addModel("GPSModel", &gps);    map.addModel("ActuatorModel", &actuatorModel);    map.addModel("PropulsionModel", &propulsionModel, forceModel);    map.addModel("AeroModel", &aeroModel, forceModel);    map.addModel("AtmosphereModel", &atmosphereModel, forceModel);    map.addModel("GroundModel", &groundModel, forceModel);    map.addModel("Telemetry", &telemetry);    map.addModel("FS_GS_Interface", &FS_GS_Interface);    //map.addModel("Performance", &performance);    // Prep loop    const int nModels = 8;    bool continueSim[nModels];    std::fill_n(continueSim, nModels, true);        int simLoopCount = 0;    int dynLoopCount = 0;        dynamicsModel.initialize();   // body angles, position, and velocity    aeroModel.initialize();       // aero angles    rotate.initialize();          // rotation matrices    imu.initialize();    barometer.initialize();    gps.initialize();    atmosphereModel.initialize(); // Re, Mach, air properties        actuatorModel.initialize();    // set propulsion throttle    propulsionModel.initialize();  //    groundModel.initialize();      // set contact point loading        telemetry.initialize();    FS_GS_Interface.initialize();        // Set flags    imu.setPerfectSensor(false);    barometer.setPerfectSensor(false);    gps.setPerfectSensor(false);        flightSoftware_setMapPointer(&map);    groundStation_setMapPointer(&map);    while( util.all(continueSim, nModels) )    {        // Update time        continueSim[7] = time.update();        if (time.performPrint() && printOutput)   { dataLog.printLog(); }        if (time.performSave() && saveOutput)     { dataLog.saveLog(); }                imu.setIMUReady(false);                if ( time.performDynamics() )        {            continueSim[0] = rotate.update(); // Update rotation matrices and quaternions                    // Update forces            continueSim[1] = atmosphereModel.update(); // Compute gravity body forces            continueSim[3] = actuatorModel.update();   // Compute actuator position (must be before propulsion and aero)            continueSim[2] = propulsionModel.update(); // Compute propulsion forces (must be before aero)            continueSim[4] = aeroModel.update();       // Compute aero body forces            continueSim[5] = groundModel.update();     // Compute ground body forces                        // Update dynamics            continueSim[6] = dynamicsModel.update(); // Compute position, velocity, attitude, and angular rates from forces and moments                        // Update hardware models            imu.update();            barometer.update();            gps.update();            telemetry.update();                        dynLoopCount++;        }                // Update Serial Model FIFOs        gps.updateSerial();        FS_GS_Interface.update();                // Update Software        mainFlightSoftware();        mainGroundStation();                simLoopCount++;    }    return 0;}