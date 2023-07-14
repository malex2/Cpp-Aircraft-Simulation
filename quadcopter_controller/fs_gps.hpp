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
#include "fs_gps_ubx_message_types.h"

enum GPS_FIX_TYPE {NO_FIX, DEAD_RECKON, FIX2D, FIX3D, GPS_DEAD_RECKON, TIME_ONLY};

#ifdef SIMULATION
    enum GPS_MSG {GGA, GLL, GSA, GSV, RMC, VTG, NAVSTAT, NAVDOP, POSLLH, VELNED, NMESSAGES};
#endif

struct GpsAidType {
    UBX_MSG_TYPES::UBX_MSG_AID_INI aidInit;
    UBX_MSG_TYPES::UBX_MSG_AID_ALM almanac[UBX_MSG_TYPES::NUM_SV];
    UBX_MSG_TYPES::UBX_MSG_AID_EPH ephemeris[UBX_MSG_TYPES::NUM_SV];
    UBX_MSG_TYPES::UBX_MSG_AID_HUI gpsHealth;
    static const int max_sv_alm = 20;
    static const int max_sv_eph = 12;
    unsigned int     n_sv_alm;
    unsigned int     n_sv_eph;
    bool             health_rcvd;
    bool             aid_rcvd;
    bool             alm_rcvd[UBX_MSG_TYPES::NUM_SV];
    bool             eph_rcvd[UBX_MSG_TYPES::NUM_SV];
    unsigned int     sv_list_alm[max_sv_alm];
    unsigned int     sv_list_eph[max_sv_eph];
    unsigned int     week_num;
    double           tow_hr;
    bool             aid_ready;
    bool             aid_saved;
    
    GpsAidType() {
        n_sv_alm   = 0;
        n_sv_eph   = 0;
        aid_rcvd   = false;
        memset(alm_rcvd, false, sizeof(alm_rcvd));
        memset(eph_rcvd, false, sizeof(eph_rcvd));
        week_num   = 0;
        tow_hr     = 0.0;
        aid_ready  = false;
        aid_saved  = false;
        memset(sv_list_alm, 0, sizeof(sv_list_alm));
        memset(sv_list_eph, 0, sizeof(sv_list_eph));
    }
};

struct GpsType {
    int input_msg_id;
    
    double posLLH[3];
    double posECEF[3];
    double ref_posECEF[3];
    double posENU[3];
    double alt_3dFixBias;
    double alt_msl;
    double horizPosAcc;
    double vertPosAcc;
    bool   positionValid;
    
    double velNED[3];
    double heading;
    double speedAcc;
    double hdgAcc;
    bool   velocityValid;
    
    double       timestamp;
    GPS_FIX_TYPE gpsFix;
    int          fixOk;
    int          numSV;
    bool         gpsGood;
    double       ttff;
    
    unsigned long rcvdByteCount;
    unsigned long rcvdMsgCount;
    unsigned long fifoReadCount;
    unsigned long fifoWriteCount;
    unsigned int  fifoMaxReadLength;
    unsigned int  fifoMaxWriteLength;
    unsigned long fifoReadOverflowCount;
    unsigned long fifoWriteOverflowCount;
    unsigned long bufferWriteOverflowCount;
    
    
    struct DOP_t {
        double gDOP;
        double pDOP;
        double tDOP;
        double vDOP;
        double hDOP;
        double nDOP;
        double eDOP;
    } DOP;

    double GPStimestamp;
    double dt_GPS_FS;
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
        alt_3dFixBias  = 0.0;
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
        fixOk     = 0;
        numSV     = 0;
        ttff      = 0.0;
        
        rcvdByteCount = 0;
        rcvdMsgCount  = 0;
        gpsGood       = false;
        GPStimestamp  = 0.0;
        dt_GPS_FS     = 0.0;
        
        fifoReadCount       = 0;
        fifoWriteCount      = 0;
        fifoMaxReadLength   = 0;
        fifoMaxWriteLength  = 0;
        fifoReadOverflowCount    = 0;
        fifoWriteOverflowCount   = 0;
        bufferWriteOverflowCount = 0;
        
        #ifdef SIMULATION
            for (int i=0; i<NMESSAGES; i++)
            {
                if (i <= VTG) { msgRates[i] = 1; }
                else { msgRates[i] = 0; }
            }
        #endif
    }
};

void FsGPS_setupGPS(int baudRate, bool fs_allow2DFix);
void init_GPS_messages();
void FsGPS_performGPS();
void FsGPS_performSerialIO();

void FsGPS_requestAidningInfo();

void update_gps_data();
void LLHtoECEF(double* ECEF, double* LLH);

// Getters
bool FsGPS_GPSgood();
void FsGPS_resetPositionValid();
void FsGPS_resetVelocityValid();
const GpsType* FsGPS_getGPSdata();

#ifdef SIMULATION
    void FsGPS_setSimulationModels(ModelMap* pMap);
#endif

#endif /* fs_gps_hpp */
