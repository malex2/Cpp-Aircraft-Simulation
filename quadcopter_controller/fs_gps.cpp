//
//  fs_gps.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 5/1/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#include "fs_gps.hpp"
#include "fs_gps_ubx_io.hpp"

GpsType GpsData;
bool gps_setup    = false;
bool positionInit = false;
long rcvdCount    = 0;

double d_posECEF[3];
double slon;
double clon;
double slat;
double clat;

SoftwareSerial gpsIO(GPSTXPIN, GPSRXPIN);
UBX_MSG ubx_msg(&gpsIO);

# ifdef SIMULATION
    bool print_serial = false;
#endif

void FsGPS_setupGPS(int baudRate)
{
    gpsIO.begin(baudRate);
    ubx_msg.write_nmea_off_long();
    ubx_msg.write_ubx_off();
    ubx_msg.write_ubx_on();
    
    d_posECEF[0] = 0.0;
    d_posECEF[1] = 0.0;
    d_posECEF[2] = 0.0;
    slon         = 0.0;
    clon         = 0.0;
    slat         = 0.0;
    clat         = 0.0;
    
    gps_setup = true;
}

void FsGPS_performGPS()
{
    if (!gps_setup) { return; }
    
    int rcvd = ubx_msg.data_available();
    if (rcvd > 0)
    {
        rcvdCount += rcvd;
        ubx_msg.read(&GpsData);
        
        if (GpsData.gpsFix != FIX3D)
        {
            GpsData.positionValid = false;
            GpsData.velocityValid = false;
        }
        
        if (GpsData.gpsFix == FIX3D) { GpsData.gpsGood = true; }
        else { GpsData.gpsGood = false; }
        
        // Compute ECEF Position
        if (GpsData.positionValid && GpsData.input_msg_id==UBX_MSG::NAV_POSLLH)
        {
            // ECEF Position
            LLHtoECEF(GpsData.posECEF, GpsData.posLLH);
            
            if (!positionInit)
            {
                GpsData.ref_posECEF[0] = GpsData.posECEF[0];
                GpsData.ref_posECEF[1] = GpsData.posECEF[1];
                GpsData.ref_posECEF[2] = GpsData.posECEF[2];
                positionInit = true;
            }
            
            // ECEF to ENU
            d_posECEF[0] = GpsData.posECEF[0] - GpsData.ref_posECEF[0];
            d_posECEF[1] = GpsData.posECEF[1] - GpsData.ref_posECEF[1];
            d_posECEF[2] = GpsData.posECEF[2] - GpsData.ref_posECEF[2];
            
            GpsData.posENU[0] = -slon     *d_posECEF[0] +  clon     *d_posECEF[1];
            GpsData.posENU[1] = -slat*clon*d_posECEF[0] + -slat*slon*d_posECEF[1] + clat*d_posECEF[2];
            GpsData.posENU[2] =  clat*clon*d_posECEF[0] +  clat*slon*d_posECEF[1] + slat*d_posECEF[2];
        }
        
        GpsData.timestamp = getTime();
    }
}

void LLHtoECEF(double* ECEF, double* LLH)
{
    double lat = LLH[0] * degree2radian;
    double lon = LLH[1] * degree2radian;
    double alt = LLH[2];
    slat = sin(lat);
    clat = cos(lat);
    clon = cos(lon);
    slon = sin(lon);

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
