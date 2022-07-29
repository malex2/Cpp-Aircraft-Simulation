//
//  fs_gps.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 5/1/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#ifndef fs_gps_hpp
#define fs_gps_hpp

#include "fs_common.hpp"

enum GPS_FIX_TYPE {NO_FIX, DEAD_RECKON, FIX2D, FIX3D, GPS_DEAD_RECKON, TIME_ONLY};

#ifdef SIMULATION
    enum GPS_MSG {GGA, GLL, GSA, GSV, RMC, VTG, NAVSTAT, POSLLH, VELNED, NMESSAGES};
#endif

struct GpsType {
    int input_msg_id;
    
    double posLLH[3];
    double posECEF[3];
    double ref_posECEF[3];
    double posENU[3];
    double alt_msl;
    double horizPosAcc;
    double vertPosAcc;
    bool   positionValid;
    
    double velNED[3];
    double heading;
    double speedAcc;
    double hdgAcc;
    bool   velocityValid;
    
    double timestamp;
    GPS_FIX_TYPE gpsFix;
    bool gpsGood;
    
    #ifdef SIMULATION
        int msgRates[NMESSAGES];
    #endif
    
    GpsType() {
        posLLH[0]      = 0.0;
        posLLH[1]      = 0.0;
        posLLH[2]      = 0.0;
        posECEF[0]     = 0.0;
        posECEF[1]     = 0.0;
        posECEF[2]     = 0.0;
        ref_posECEF[0] = 0.0;
        ref_posECEF[1] = 0.0;
        ref_posECEF[2] = 0.0;
        posENU[0]      = 0.0;
        posENU[1]      = 0.0;
        posENU[2]      = 0.0;
        alt_msl        = 0.0;
        horizPosAcc    = 0.0;
        vertPosAcc     = 0.0;
        positionValid  = false;
        
        velNED[0]     = 0.0;
        velNED[1]     = 0.0;
        velNED[2]     = 0.0;
        heading       = 0.0;
        speedAcc      = 0.0;
        hdgAcc        = 0.0;
        velocityValid = false;
        
        timestamp = 0.0;
        gpsFix    = NO_FIX;
        gpsGood   = false;
        
        #ifdef SIMULATION
            for (int i=0; i<NMESSAGES; i++)
            {
                if (i <= VTG) { msgRates[i] = 1; }
                else { msgRates[i] = 0; }
            }
        #endif
    }
};

void LLHtoECEF(double* ECEF, double* LLH);
void FsGPS_setupGPS(int baudRate);
void FsGPS_performGPS();

// Getters
bool FsGPS_GPSgood();
void FsGPS_resetPositionValid();
void FsGPS_resetVelocityValid();
GpsType* FsGPS_getGPSdata();

#ifdef SIMULATION
    void FsGPS_setSimulationModels(ModelMap* pMap);
#endif

#endif /* fs_gps_hpp */
