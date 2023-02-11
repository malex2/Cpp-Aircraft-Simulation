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

GpsType    GpsData;
GpsAidType AidData;

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

unsigned long n_gps_msgs[UBX_MSG_TYPES::NUBXCLASSES][UBX_MSG_TYPES::NUBXIDS];

# ifdef SIMULATION
    bool print_serial = false;
#endif

void FsGPS_setupGPS(int baudRate, bool fs_allow2DFix)
{
#ifdef GPS
    allow2DFix = fs_allow2DFix;
    gpsFIFO.begin(baudRate);
    
    // Configure Messsages
    ubx_msg.write_nmea_off();
    ubx_msg.write_ubx_off();
    ubx_msg.write_ubx_on();
    
    // Set dynmaic mode
    ubx_msg.set_nav_config();
    ubx_msg.get_nav_config();
    
    // Poll needed almanac data
    ubx_msg.init_aid_request();
    
    d_posECEF[0] = 0.0;
    d_posECEF[1] = 0.0;
    d_posECEF[2] = 0.0;
    ref_slon     = 0.0;
    ref_clon     = 0.0;
    ref_slat     = 0.0;
    ref_clat     = 0.0;
    
    for(int i = 0; i < UBX_MSG_TYPES::NUBXCLASSES; i++)
    {
        for (int j = 0; j < UBX_MSG_TYPES::NUBXIDS; j++)
        {
            n_gps_msgs[i][j] = 0;
        }
    }
    
    display(getTime());
    display(") GPS setup.\n");
    gps_setup = true;
#endif
}

void FsGPS_performGPS()
{
#ifdef GPS
    if (!gps_setup) { return; }
    
    GpsData.fifoReadCount  = gpsFIFO.get_read_count();
    GpsData.fifoWriteCount = gpsFIFO.get_write_count();

    GpsData.fifoMaxReadLength  = gpsFIFO.get_max_read_buffer_length();
    GpsData.fifoMaxWriteLength = gpsFIFO.get_max_write_buffer_length();
    
    //display("[gpsFIFO] = ["); display(gpsFIFO.available()); display("]\n");
    
    if (gpsFIFO.get_read_fifo_overflow_count() > GpsData.fifoReadOverflowCount)
    {
        GpsData.fifoReadOverflowCount = gpsFIFO.get_read_fifo_overflow_count();
        display("FsGPS - Read FIFO Overflow!!!\n");
    }
    
    if (gpsFIFO.get_write_fifo_overflow_count() > GpsData.fifoWriteOverflowCount)
    {
        GpsData.fifoWriteOverflowCount = gpsFIFO.get_write_fifo_overflow_count();
        display("FsGPS - Write FIFO Overflow!!!\n");
    }
    if (gpsFIFO.get_write_buffer_overflow_count() > GpsData.bufferWriteOverflowCount)
    {
        GpsData.bufferWriteOverflowCount = gpsFIFO.get_write_buffer_overflow_count();
        display("FsGPS - Write Buffer Overflow!!!\n");
    }
    
    int rcvd = ubx_msg.data_available();
    if (rcvd > 0)
    {
        //gpsFIFO.display_read_buffer();
        
        GpsData.rcvdByteCount += rcvd;
        GpsData.rcvdMsgCount  += ubx_msg.read();
        update_gps_data();
        
        if (GpsData.fixOk == 1 &&
            ((GpsData.gpsFix == FIX2D && allow2DFix) || GpsData.gpsFix == FIX3D))
        {
            GpsData.gpsGood = true;
        }
        else
        {
            GpsData.gpsGood = false;
        }
        
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
        GpsData.dt_GPS_FS = GpsData.GPStimestamp - GpsData.timestamp;
    }
#endif
}

void FsGPS_performSerialIO()
{
#ifdef GPS
    gpsFIFO.update_fifo();
#endif
}

void FsGPS_requestAidningInfo()
{
#ifdef GPS
    ubx_msg.request_aid_data();
#endif
}

void update_gps_data()
{
    for (int msg_class = 0; msg_class < UBX_MSG_TYPES::NUBXCLASSES; msg_class++)
    {
        for (int msg_id = 0; msg_id < UBX_MSG_TYPES::N_UBX_MSG_ID[msg_class]; msg_id++)
        {
            if (ubx_msg.get_n_rcvd(msg_class, msg_id) > n_gps_msgs[msg_class][msg_id])
            {
                n_gps_msgs[msg_class][msg_id] = ubx_msg.get_n_rcvd(msg_class, msg_id);
                
                if (msg_class == UBX_MSG_TYPES::NAV && msg_id == UBX_MSG_TYPES::NAV_POSLLH)
                {
                    GpsData.posLLH[0]   = ubx_msg.get_posLLH().data.latitude           * ubx_msg.get_posLLH().scale.latitude;
                    GpsData.posLLH[1]   = ubx_msg.get_posLLH().data.longitude          * ubx_msg.get_posLLH().scale.longitude;
                    GpsData.posLLH[2]   = ubx_msg.get_posLLH().data.altitude_ellipsoid * ubx_msg.get_posLLH().scale.altitude_ellipsoid;
                    GpsData.alt_msl     = ubx_msg.get_posLLH().data.altitude_msl       * ubx_msg.get_posLLH().scale.altitude_msl;
                    GpsData.horizPosAcc = ubx_msg.get_posLLH().data.horizontalAccuracy * ubx_msg.get_posLLH().scale.horizontalAccuracy;
                    GpsData.vertPosAcc  = ubx_msg.get_posLLH().data.verticalAccuracy   * ubx_msg.get_posLLH().scale.verticalAccuracy;
                    GpsData.positionValid = true;
                }
                else if (msg_class == UBX_MSG_TYPES::NAV && msg_id == UBX_MSG_TYPES::NAV_VELNED)
                {
                    GpsData.velNED[0] = ubx_msg.get_velNED().data.velN            * ubx_msg.get_velNED().scale.velN;
                    GpsData.velNED[1] = ubx_msg.get_velNED().data.velE            * ubx_msg.get_velNED().scale.velE;
                    GpsData.velNED[2] = ubx_msg.get_velNED().data.velD            * ubx_msg.get_velNED().scale.velD;
                    GpsData.heading   = ubx_msg.get_velNED().data.heading         * ubx_msg.get_velNED().scale.heading;
                    GpsData.speedAcc  = ubx_msg.get_velNED().data.speedAccuracy   * ubx_msg.get_velNED().scale.speedAccuracy;
                    GpsData.hdgAcc    = ubx_msg.get_velNED().data.headingAccuracy * ubx_msg.get_velNED().scale.headingAccuracy;
                    GpsData.velocityValid = true;
                }
                else if (msg_class == UBX_MSG_TYPES::NAV && msg_id == UBX_MSG_TYPES::NAV_STATUS)
                {
                    GpsData.ttff         = ubx_msg.get_navStatus().data.ttff * ubx_msg.get_navStatus().scale.ttff;
                    GpsData.GPStimestamp = ubx_msg.get_navStatus().data.msss * ubx_msg.get_navStatus().scale.msss;
                    GpsData.gpsFix       = (GPS_FIX_TYPE) ubx_msg.get_navStatus().data.gpsFix;
                    GpsData.fixOk        = (ubx_msg.get_navStatus().data.flags >> UBX_MSG_TYPES::GPSFixOk) & 1;
                    //int GPSFixOk = (ubx_msg.get_navStatus().data.flags >> 0) & 1;
                    //int DiffSoln = (ubx_msg.get_navStatus().data.flags >> 1) & 1;
                    //int WKNSET   = (ubx_msg.get_navStatus().data.flags >> 2) & 1;
                    //int TOWSET   = (ubx_msg.get_navStatus().data.flags >> 3) & 1;
                    //display(GPSFixOk); display(" "); display(DiffSoln); display(" "); display(WKNSET); display(" "); display(TOWSET); display("\n");
                }
                else if (msg_class == UBX_MSG_TYPES::NAV && msg_id == UBX_MSG_TYPES::NAV_DOP)
                {
                    GpsData.DOP.gDOP = ubx_msg.get_navDOP().data.gDOP * ubx_msg.get_navDOP().scale.gDOP;
                    GpsData.DOP.pDOP = ubx_msg.get_navDOP().data.pDOP * ubx_msg.get_navDOP().scale.pDOP;
                    GpsData.DOP.tDOP = ubx_msg.get_navDOP().data.tDOP * ubx_msg.get_navDOP().scale.tDOP;
                    GpsData.DOP.vDOP = ubx_msg.get_navDOP().data.vDOP * ubx_msg.get_navDOP().scale.vDOP;
                    GpsData.DOP.hDOP = ubx_msg.get_navDOP().data.hDOP * ubx_msg.get_navDOP().scale.hDOP;
                    GpsData.DOP.nDOP = ubx_msg.get_navDOP().data.nDOP * ubx_msg.get_navDOP().scale.nDOP;
                    GpsData.DOP.eDOP = ubx_msg.get_navDOP().data.eDOP * ubx_msg.get_navDOP().scale.eDOP;
                }
                else if (msg_class == UBX_MSG_TYPES::NAV && msg_id == UBX_MSG_TYPES::NAV_SOL)
                {
                    GpsData.numSV = (int) ubx_msg.get_navSol().data.numSV;
                }
                else if (msg_class == UBX_MSG_TYPES::CFG && msg_id == UBX_MSG_TYPES::CFG_NAV5)
                {
                    display("NAV5 mask:\n");
                    display((int)(ubx_msg.get_configNav5().data.mask >> UBX_MSG_TYPES::NavConfig_dyn) & 1); display(" ");
                    display((int)(ubx_msg.get_configNav5().data.mask >> UBX_MSG_TYPES::NavConfig_minEl) & 1); display(" ");
                    display((int)(ubx_msg.get_configNav5().data.mask >> UBX_MSG_TYPES::NavConfig_fixMode) & 1); display(" ");
                    display((int)(ubx_msg.get_configNav5().data.mask >> UBX_MSG_TYPES::NavConfig_drLim) & 1); display(" ");
                    display((int)(ubx_msg.get_configNav5().data.mask >> UBX_MSG_TYPES::NavConfig_posMask) & 1); display(" ");
                    display((int)(ubx_msg.get_configNav5().data.mask >> UBX_MSG_TYPES::NavConfig_timeMask) & 1); display(" ");
                    display((int)(ubx_msg.get_configNav5().data.mask >> UBX_MSG_TYPES::NavConfig_staticHoldMeask) & 1); display(" ");
                    display((int)(ubx_msg.get_configNav5().data.mask >> UBX_MSG_TYPES::NavConfig_dgpsMask) & 1); display("\n");
                    
                    display("NAV5 data:\n");
                    display((int) ubx_msg.get_configNav5().data.dynModel); display(" ");
                    display((int) ubx_msg.get_configNav5().data.fixMode);  display(" ");
                    display(ubx_msg.get_configNav5().data.fixedAlt                 * ubx_msg.get_configNav5().scale.fixedAlt); display(" ");
                    display(ubx_msg.get_configNav5().data.fixedAltVar              * ubx_msg.get_configNav5().scale.fixedAltVar); display(" ");
                    display(ubx_msg.get_configNav5().data.minElv                   * ubx_msg.get_configNav5().scale.minElv); display(" ");
                    display(((int) ubx_msg.get_configNav5().data.drLimit)          * ubx_msg.get_configNav5().scale.drLimit); display(" ");
                    display(ubx_msg.get_configNav5().data.pDop                     * ubx_msg.get_configNav5().scale.pDop); display(" ");
                    display(ubx_msg.get_configNav5().data.tDop                     * ubx_msg.get_configNav5().scale.tDop); display(" ");
                    display(ubx_msg.get_configNav5().data.pAcc                     * ubx_msg.get_configNav5().scale.pAcc); display(" ");
                    display(ubx_msg.get_configNav5().data.tAcc                     * ubx_msg.get_configNav5().scale.tAcc); display(" ");
                    display(((int) ubx_msg.get_configNav5().data.staticHoldThresh) * ubx_msg.get_configNav5().scale.staticHoldThresh); display(" ");
                    display(((int) ubx_msg.get_configNav5().data.dgpsTimeOut)      * ubx_msg.get_configNav5().scale.dgpsTimeOut); display("\n");
                }
                else if (msg_class == UBX_MSG_TYPES::ACK && msg_id == UBX_MSG_TYPES::ACK_NAK)
                {
                    display("Message not acknowledged! [classID, msgID] = [");
                    display(ubx_msg.get_ack().data.clsID); display(", ");
                    display(ubx_msg.get_ack().data.msgID); display("]\n");
                }
                else if (msg_class == UBX_MSG_TYPES::AID && msg_id == UBX_MSG_TYPES::AID_INI)
                {
                    int pos_valid           = (int) ((ubx_msg.get_aidInit().data.flags >> UBX_MSG_TYPES::INI_POS) & 1);
                    int time_valid          = (int) ((ubx_msg.get_aidInit().data.flags >> UBX_MSG_TYPES::INI_TIME) & 1);
                    int clock_drift_valid   = (int) ((ubx_msg.get_aidInit().data.flags >> UBX_MSG_TYPES::INI_clockD) & 1);
                    int use_time_pulse      = (int) ((ubx_msg.get_aidInit().data.flags >> UBX_MSG_TYPES::INI_tp) & 1);
                    int clock_freq_valid    = (int) ((ubx_msg.get_aidInit().data.flags >> UBX_MSG_TYPES::INI_clockF) & 1);
                    int pos_in_lla          = (int) ((ubx_msg.get_aidInit().data.flags >> UBX_MSG_TYPES::INI_lla) & 1);
                    int alt_invalid         = (int) ((ubx_msg.get_aidInit().data.flags >> UBX_MSG_TYPES::INI_altInv) & 1);
                    int mark_rcv_before_msg = (int) ((ubx_msg.get_aidInit().data.flags >> UBX_MSG_TYPES::INI_prevTm) & 1);
                    
                    display("AID_INI: \n");
                    display("\t[pos_valid, time_valid, alt_invalid] = [");
                    display(pos_valid); display(", ");
                    display(time_valid); display(", ");
                    display(alt_invalid); display("]\n");
                    
                    display("\t[clock_drift_valid, use_time_pulse, clock_freq_valid, mark_rcv_before_msg] = [");
                    display(clock_drift_valid); display(", ");
                    display(use_time_pulse); display(", ");
                    display(clock_freq_valid); display(", ");
                    display(mark_rcv_before_msg); display("]\n");
                    
                    if (pos_in_lla)
                    {
                        display("\tAID LLH = [");
                        display((double) ubx_msg.get_aidInit().data.ecefXOrLat * ubx_msg.get_aidInit().scale.Lat); display(",");
                        display((double) ubx_msg.get_aidInit().data.ecefYOrLon * ubx_msg.get_aidInit().scale.Lon); display(",");
                        display((double) ubx_msg.get_aidInit().data.ecefZOrAlt * ubx_msg.get_aidInit().scale.Alt); display("]\n");
                    }
                    else
                    {
                        display("\tAID ECEF = [");
                        display((double) ubx_msg.get_aidInit().data.ecefXOrLat * ubx_msg.get_aidInit().scale.ecefX); display(",");
                        display((double) ubx_msg.get_aidInit().data.ecefYOrLon * ubx_msg.get_aidInit().scale.ecefY); display(",");
                        display((double) ubx_msg.get_aidInit().data.ecefZOrAlt * ubx_msg.get_aidInit().scale.ecefZ); display("]\n");
                    }
                    display("\tPosAcc = "); display(ubx_msg.get_aidInit().data.posAcc * ubx_msg.get_aidInit().scale.posAcc); display("\n");
                    
                    display("\t[Week Num, TOW, TOWns] = [");
                    display(ubx_msg.get_aidInit().data.wn); display(", ");
                    display((double) ubx_msg.get_aidInit().data.tow * ubx_msg.get_aidInit().scale.tow); display(", ");
                    display(ubx_msg.get_aidInit().data.towNs); display("]\n");
                    
                    display("\tTimeAcc = ");
                    display((double) ubx_msg.get_aidInit().data.tAccMs * ubx_msg.get_aidInit().scale.tAccMs);
                    display(" + ");
                    display(ubx_msg.get_aidInit().data.tAccNs);
                    display(" ns\n");
                    
                    if (pos_valid && time_valid && !AidData.aid_rcvd && GpsData.fixOk)
                    {
                        display("Aid Init Rcvd.\n");
                        ubx_msg.get_aidInit().print();
                        //memcpy(&AidData.aidInit, ubx_msg.get_aidInit_p(), sizeof(AidData.aidInit));
                        AidData.aid_rcvd = true;
                    }
                    
                    //uint16_t tmCfg            = 0;
                    //int32_t  clkDOrFreq       = 0;
                    //uint32_t clkDAccOrFreqAcc = 0;
                }
                else if (msg_class == UBX_MSG_TYPES::AID && msg_id == UBX_MSG_TYPES::AID_ALM)
                {
                    for (int i_sv = 0; i_sv < UBX_MSG_TYPES::NUM_SV; i_sv++)
                    {
                        if(ubx_msg.get_almanac()[i_sv].data.week != UBX_MSG_TYPES::ALM_WEEK_INVALID && !AidData.alm_rcvd[i_sv] && GpsData.fixOk)
                        {
                            AidData.sv_list_alm[AidData.n_sv_alm] = i_sv;
                            //memcpy(&AidData.almanac[AidData.n_sv_alm], &ubx_msg.get_almanac()[i_sv].data, sizeof(AidData.almanac[AidData.n_sv_alm]));
                            AidData.alm_rcvd[i_sv] = true;
                            AidData.n_sv_alm++;
                            
                            display("Received Almanac Data: ");
                            display(ubx_msg.get_almanac()[i_sv].data.svid);
                            display("\n");
                            ubx_msg.get_almanac()[i_sv].print();
                        }
                    }
                }
                else if (msg_class == UBX_MSG_TYPES::AID && msg_id == UBX_MSG_TYPES::AID_EPH)
                {
                    for (int i_sv = 0; i_sv < UBX_MSG_TYPES::NUM_SV; i_sv++)
                    {
                        if(ubx_msg.get_ephemeris()[i_sv].data.how != UBX_MSG_TYPES::EPH_HOW_INVALID && !AidData.eph_rcvd[i_sv] && GpsData.fixOk)
                        {
                            AidData.sv_list_eph[AidData.n_sv_eph] = i_sv;
                            //memcpy(&AidData.ephemeris[AidData.n_sv_eph], &ubx_msg.get_ephemeris()[i_sv].data, sizeof(AidData.ephemeris[AidData.n_sv_eph]));
                            AidData.eph_rcvd[i_sv] = true;
                            AidData.n_sv_eph++;
                            
                            display("Received Ephemeris Data: ");
                            display(ubx_msg.get_ephemeris()[i_sv].data.svid);
                            display("\n");
                            ubx_msg.get_ephemeris()[i_sv].print();
                        }
                    }
                }
                else if (msg_class == UBX_MSG_TYPES::AID && msg_id == UBX_MSG_TYPES::AID_HUI)
                {
                    if (!AidData.health_rcvd && GpsData.fixOk)
                    {
                        //memcpy(&AidData.gpsHealth, ubx_msg.get_gpsHealth_p(), sizeof(AidData.gpsHealth));
                        AidData.health_rcvd = true;
                        
                        display("Received GPS Health Data\n");
                        ubx_msg.get_gpsHealth().print();
                    }
                    
                    
                }
            }
        }
    }
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
