//
//  fs_gps.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 5/1/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#include "fs_gps.hpp"
#include "fs_gps_ubx_io.hpp"
#include "fs_controls.hpp"
#include "fs_navigation.hpp"

GpsType GpsData;
bool gps_setup    = false;
bool positionInit = false;
bool first3DFix   = true;
bool allow2DFix   = false;
int gpsFix_init   = 0;

double d_posECEF[3];
double ref_slon;
double ref_clon;
double ref_slat;
double ref_clat;

#define gpsIO Serial1
FS_FIFO gpsFIFO(&gpsIO);
UBX_MSG ubx_msg(&gpsFIFO);

# ifdef SIMULATION
    bool print_serial = false;
#endif

void FsGPS_setupGPS(int baudRate, bool fs_allow2DFix)
{
#ifdef GPS
    allow2DFix = fs_allow2DFix;
    gpsFIFO.begin(baudRate);
    //ubx_msg.write_nmea_off_long();
    ubx_msg.write_nmea_off();
    ubx_msg.write_ubx_off();
    ubx_msg.write_ubx_on();
    ubx_msg.get_nav_config();
    ubx_msg.set_nav_config();
    ubx_msg.get_nav_config();
    
    d_posECEF[0] = 0.0;
    d_posECEF[1] = 0.0;
    d_posECEF[2] = 0.0;
    ref_slon     = 0.0;
    ref_clon     = 0.0;
    ref_slat     = 0.0;
    ref_clat     = 0.0;
    
    display(getTime());
    display(") GPS setup.\n");
    gps_setup = true;
#endif
}

void FsGPS_performGPS()
{
#ifdef GPS
    if (!gps_setup) { return; }
    
    GpsData.fifoReadCount  = gpsFIFO.read_count();
    GpsData.fifoWriteCount = gpsFIFO.write_count();
    
    int rcvd = ubx_msg.data_available();
    if (rcvd > 0)
    {
        GpsData.rcvdByteCount += rcvd;
        GpsData.rcvdMsgCount += ubx_msg.read(&GpsData);
        
        if ((GpsData.gpsFix == FIX2D && allow2DFix) || GpsData.gpsFix == FIX3D) { GpsData.gpsGood = true; }
        else { GpsData.gpsGood = false; }
        
        if (!GpsData.gpsGood)
        {
            GpsData.positionValid = false;
            GpsData.velocityValid = false;
        }
        
        // Compute ECEF Position
        if (GpsData.positionValid)
        {
            // ECEF Position
            LLHtoECEF(GpsData.posECEF, GpsData.posLLH);
            
            if (!positionInit)
            {
                GpsData.ref_posECEF[0] = GpsData.posECEF[0];
                GpsData.ref_posECEF[1] = GpsData.posECEF[1];
                GpsData.ref_posECEF[2] = GpsData.posECEF[2];
                ref_slat = sin(GpsData.posLLH[0]*degree2radian);
                ref_clat = cos(GpsData.posLLH[0]*degree2radian);
                ref_clon = cos(GpsData.posLLH[1]*degree2radian);
                ref_slon = sin(GpsData.posLLH[1]*degree2radian);
                gpsFix_init = GpsData.gpsFix;
                positionInit = true;
            }
            
            if (GpsData.gpsFix == FIX3D && first3DFix)
            {
                if (gpsFix_init == FIX2D)
                {
                    GpsData.alt_3dFixBias = FsNavigation_getNavAlt() - GpsData.posLLH[2];
                }
                else if (!FsControls_onGround())
                {
                    GpsData.alt_3dFixBias = FsNavigation_getNavAlt();
                }
                first3DFix = false;
            }
            
            // ECEF to ENU
            d_posECEF[0] = GpsData.posECEF[0] - GpsData.ref_posECEF[0];
            d_posECEF[1] = GpsData.posECEF[1] - GpsData.ref_posECEF[1];
            d_posECEF[2] = GpsData.posECEF[2] - GpsData.ref_posECEF[2];
            
            GpsData.posENU[0] = -ref_slon         *d_posECEF[0] +  ref_clon         *d_posECEF[1];
            GpsData.posENU[1] = -ref_slat*ref_clon*d_posECEF[0] + -ref_slat*ref_slon*d_posECEF[1] + ref_clat*d_posECEF[2];
            GpsData.posENU[2] =  ref_clat*ref_clon*d_posECEF[0] +  ref_clat*ref_slon*d_posECEF[1] + ref_slat*d_posECEF[2];
            GpsData.posENU[2] += GpsData.alt_3dFixBias;
        }
        GpsData.timestamp = getTime();
        GpsData.dt_FS_GPS = GpsData.timestamp - GpsData.GPStimestamp;
    }
#endif
}

void FsGPS_performSerialIO()
{
#ifdef GPS
    gpsFIFO.update_fifo();
#endif
}

void LLHtoECEF(double* ECEF, double* LLH)
{
    double lat = LLH[0] * degree2radian;
    double lon = LLH[1] * degree2radian;
    double alt = LLH[2];
    double slat = sin(lat);
    double clat = cos(lat);
    double clon = cos(lon);
    double slon = sin(lon);

    double N = (EARTH_a)/( sqrt(1.0 - (EARTH_e2)*slat*slat) );
    
    ECEF[0] = (N+alt)*clat*clon;
    ECEF[1] = (N+alt)*clat*slon;
    ECEF[2] = ((EARTH_b2)/(EARTH_a2)*N + alt)*slat;
}

bool FsGPS_GPSgood()
{
    return GpsData.gpsGood;
}

void FsGPS_resetPositionValid() { GpsData.positionValid = false; }
void FsGPS_resetVelocityValid() { GpsData.velocityValid = false; }

GpsType* FsGPS_getGPSdata()
{
    return &GpsData;
}

#ifdef SIMULATION
void FsGPS_setSimulationModels(ModelMap* pMap)
{
    gpsIO.serial_setSimulationModels(pMap, "GPSModel", print_serial);
}
#endif
