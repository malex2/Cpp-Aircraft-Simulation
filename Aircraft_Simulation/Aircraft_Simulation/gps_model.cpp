//
//  gps_model.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 5/7/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#include "gps_model.hpp"
#include "dynamics_model.hpp"
#include "time.hpp"
#include "rotate_frame.hpp"
#include "arduino_class_models.hpp"
#include "fs_gps.hpp"
#include "fs_gps_ubx_io.hpp"

int GPSModelBase::MessageType::nGPSInputMessages = 0;
int GPSModelBase::MessageType::nGPSOutputMessages = 0;

GPSModelBase::GPSModelBase(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pTime   = NULL;
    pRotate = NULL;
    
    pMap = pMapInit;
    debugFlag = debugFlagIn;
   
    //pMap->addLogVar("gps_velN", &gps_velNED[0], savePlot, 2);
    //pMap->addLogVar("gps_velE", &gps_velNED[1], savePlot, 2);
    //pMap->addLogVar("gps_velD", &gps_velNED[2], savePlot, 2);
    //pMap->addLogVar("gps_speed", &gps_speed, savePlot, 2);
    //pMap->addLogVar("gps_groundSpeed", &gps_groundSpeed, savePlot, 2);
    //pMap->addLogVar("gps_heading", &gps_heading, savePlot, 2);
    //pMap->addLogVar("gps_lat", &gps_lat, savePlot, 2);
    //pMap->addLogVar("gps_lon", &gps_lon, savePlot, 2);
    //pMap->addLogVar("gps_alt", &gps_posLLH[2], savePlot, 2);
    //pMap->addLogVar("gps_altMSL", &gps_altMSL, savePlot, 2);
    //pMap->addLogVar("gps_timeofweek", &gps_timeofweek, savePlot, 2);
    //pMap->addLogVar("gps_posECEF X", &gps_posECEF[0], savePlot, 2);
    //pMap->addLogVar("gps_posECEF Y", &gps_posECEF[1], savePlot, 2);
    //pMap->addLogVar("gps_posECEF Z", &gps_posECEF[2], savePlot, 2);
    //pMap->addLogVar("gps_horizPosAcc", &horizPosAcc, savePlot, 2);
    //pMap->addLogVar("gps_vertPosAcc", &vertPosAcc, savePlot, 2);
    //pMap->addLogVar("gps_velAcc", &velAcc, savePlot, 2);
    //pMap->addLogVar("horizError", &horizError, savePlot, 2);
    //pMap->addLogVar("horizDir", &horizDir_Deg, savePlot, 2);
    //pMap->addLogVar("vertError", &vertError, savePlot, 2);
    //pMap->addLogVar("posErrorN", &posErrorNED[0], savePlot, 2);
    //pMap->addLogVar("posErrorE", &posErrorNED[1], savePlot, 2);
    //pMap->addLogVar("posErrorD", &posErrorNED[2], savePlot, 2);
    //pMap->addLogVar("posErrorECEF X", &posErrorECEF[0], savePlot, 2);
    //pMap->addLogVar("posErrorECEF Y", &posErrorECEF[1], savePlot, 2);
    //pMap->addLogVar("posErrorECEF Z", &posErrorECEF[2], savePlot, 2);
    //pMap->addLogVar("velError", &velError, savePlot, 2);
    //pMap->addLogVar("velErrorYaw", &velErrorYaw_Deg, savePlot, 2);
    //pMap->addLogVar("velErrorPitch", &velErrorPitch_Deg, savePlot, 2);
    //pMap->addLogVar("velErrorN", &velErrorNED[0], savePlot, 2);
    //pMap->addLogVar("velErrorE", &velErrorNED[1], savePlot, 2);
    //pMap->addLogVar("velErrorD", &velErrorNED[2], savePlot, 2);
    
    //pMap->addLogVar("GPS sendTime", &sendTime, savePlot, 2);
    //pMap->addLogVar("GPS sendCount", &d_sendCount, printSavePlot, 3);
    //pMap->addLogVar("GPS receiveCount", &d_receiveCount, printSavePlot, 3);

    util.initArray(gps_velNED, 0.0, 3);
    util.initArray(gps_posECEF, 0.0, 3);
    util.initArray(gps_posLLH, 0.0, 3);
    util.initArray(gps_velNE, 0.0, 2);
    gps_altMSL       = 0.0;
    gps_timeofweek   = 0.0;
    gps_speed        = 0.0;
    gps_groundSpeed  = 0.0;
    gps_heading      = 0.0;
    gps_acquire_time = 0.0;
    
    gps_lat           = 0.0;
    gps_lon           = 0.0;
    horizError        = 0.0;
    horizDir_Deg      = 0.0;
    vertError         = 0.0;
    velError          = 0.0;
    velErrorYaw_Deg   = 0.0;
    velErrorPitch_Deg = 0.0;
    d_sendCount       = 0.0;
    d_receiveCount    = 0.0;
    util.initArray(posErrorNED, 0.0, 3);
    util.initArray(posErrorECEF, 0.0, 3);
    util.initArray(velErrorNED, 0.0, 3);

    horizPosAcc = 0.0;
    vertPosAcc  = 0.0;
    velAcc      = 0.0;
    
    perfectSensor      = false;
    iGPSInputMessage   = 0;
    nGPSInputMessages  = 0;
    nGPSOutputMessages = 0;
    sendCount          = 0;
    receiveCount       = 0;

    for (int i=0; i<maxOutputMessages; i++)
    {
        pOutputMessages[i] = NULL;
    }
    
    for (int i=0; i<maxInputMessages; i++)
    {
        pInputMessages[i] = new MessageType(0.0, 0.0, true);
    }
    pGPSIO = NULL;
}

GPSModelBase::~GPSModelBase()
{
    for (int i = 0; i < maxOutputMessages; i++)
    {
        if (pOutputMessages[i] != NULL)
        {
            delete pOutputMessages[i];
        }
    }
    
    for (int i = 0; i < maxInputMessages; i++)
    {
        if (pInputMessages[i] != NULL)
        {
            delete pInputMessages[i];
        }
    }
    
    if (pGPSIO) { delete pGPSIO; }
}

void GPSModelBase::initialize()
{
    pDyn    = (DynamicsModel*) pMap->getModel("DynamicsModel");
    pTime   = (Time*) pMap->getModel("Time");
    pRotate = (RotateFrame*) pMap->getModel("RotateFrame");
    
    horizPosNoise.setNoise(horizPosAcc);
    horizPosNoiseDir.setNoise(M_PI);
    vertPosNoise.setNoise(vertPosAcc);
    velNoise.setNoise(velAcc);
    velNoiseYaw.setNoise(M_PI);
    velNoisePitch.setNoise(M_PI/2.0);
    
    nGPSInputMessages  = GPSModelBase::MessageType::nGPSInputMessages;
    nGPSOutputMessages = GPSModelBase::MessageType::nGPSOutputMessages;
}

bool GPSModelBase::update()
{
    updatePositionError();
    updateVelocityError();
    gps_timeofweek = pTime->getGPSTimeOfWeek();
    
    if (perfectSensor)
    {
        util.setArray(gps_posECEF, pDyn->getPosECEF(), 3);
        util.setArray(gps_velNED, pDyn->getVelNED(), 3);
        gps_altMSL = pDyn->getAltMSL();
    }
    else
    {
        util.vAdd(gps_posECEF, pDyn->getPosECEF(), posErrorECEF, 3);
        util.vAdd(gps_velNED, pDyn->getVelNED(), velErrorNED, 3);
        gps_altMSL = pDyn->getAltMSL() - posErrorNED[2];
    }
    util.ECEFtoLLH(gps_posLLH, gps_posECEF, 0.0);
    
    gps_velNE[0]    = gps_velNED[0];
    gps_velNE[1]    = gps_velNED[1];
    gps_speed       = util.mag(gps_velNED, 3);
    gps_groundSpeed = util.mag(gps_velNE, 2);
    gps_heading     = atan2(gps_velNE[1], gps_velNE[0]) / util.deg2rad;
    
    readInputIO();
    readInputMessages();
    decodeInputMessages();
    
    constructOutputMessages();
    writeOutputMessages();
    writeOutputIO();
    
    // Log Vars
    gps_lat = gps_posLLH[0] / util.deg2rad;
    gps_lon = gps_posLLH[1] / util.deg2rad;
    d_sendCount    = (double) sendCount;
    d_receiveCount = (double) receiveCount;
    
    return true;
}

void GPSModelBase::updatePositionError()
{
    double posErrorPE[3];   // error in position error frame
    double horizDir;
    double euler_PE_NED[3]; // Euler rotation from NED to position error frame
    double R_PE_NED[3][3];  // Rotation matrix from NED to position error frame
    double R_NED_PE[3][3];  // Rotation matrix from position error frame to NED
    double R_NED_ECEF[3][3];
    double R_ECEF_NED[3][3];
    
    // update horizontal position noise and direction
    horizPosNoise.updateNoise();
    horizPosNoiseDir.updateNoise();
    horizError = fabs( horizPosNoise.getNoise() );
    horizDir   = horizPosNoiseDir.getNoise();
    
    // update vertical position noise
    vertPosNoise.updateNoise();
    vertError  = vertPosNoise.getNoise();
    
    // position error vector with X in direction of horizontal error and Z in down direction
    posErrorPE[0] = horizError;
    posErrorPE[1] = 0.0;
    posErrorPE[2] = vertError;
    
    // rotation from error frame to NED
    // horizontal error is yawed in random direction
    euler_PE_NED[0] = 0.0;
    euler_PE_NED[1] = 0.0;
    euler_PE_NED[2] = horizDir;
    util.setupRotation(*R_PE_NED, euler_PE_NED);
    util.mtran(*R_NED_PE, *R_PE_NED, 3, 3);
    
    util.dcmECEFtoNED(*R_NED_ECEF, pDyn->getPosLLH());
    util.mtran(*R_ECEF_NED, *R_NED_ECEF, 3, 3);
    
    // transform position error into ECEF frame
    util.mmult(posErrorNED, *R_NED_PE, posErrorPE, 3, 3);
    util.mmult(posErrorECEF, *R_ECEF_NED, posErrorNED, 3, 3);
    
    // print variable
    horizDir_Deg = horizDir / util.deg2rad;
}

void GPSModelBase::updateVelocityError()
{
    double velErrorVE[3];
    double yawDir;
    double pitchDir;
    double euler_VE_NED[3];
    double R_VE_NED[3][3];
    double R_NED_VE[3][3];
    
    // update noise
    velNoise.updateNoise();
    velNoiseYaw.updateNoise();
    velNoisePitch.updateNoise();
    
    velError = fabs( velNoise.getNoise() );
    yawDir   = velNoiseYaw.getNoise();
    pitchDir = velNoisePitch.getNoise();
    
    // setup error
    velErrorVE[0] = velError;
    velErrorVE[1] = 0.0;
    velErrorVE[2] = 0.0;
    
    // rotation from error frame to NED
    // velocity error is yawed and pitched in a random direction
    euler_VE_NED[0] = 0.0;
    euler_VE_NED[1] = pitchDir;
    euler_VE_NED[2] = yawDir;
    util.setupRotation(*R_VE_NED, euler_VE_NED);
    util.mtran(*R_NED_VE, *R_VE_NED, 3, 3);

    // transform velocity error into NED frame
    util.mmult(velErrorNED, *R_NED_VE, velErrorVE, 3, 3);
    
    // print variables
    velErrorYaw_Deg = yawDir / util.deg2rad;
    velErrorPitch_Deg = pitchDir / util.deg2rad;
}

void GPSModelBase::readInputIO()
{
    if (pSerialIO && !pGPSIO)
    {
        pGPSIO = new SoftwareSerial(pSerialIO->getTXPin(), pSerialIO->getRXPin());
        pGPSIO->begin( pSerialIO->getBaudRate() );
    }
    
    if (pGPSIO)
    {
        while (pSerialIO->slave_available())
        {
            pGPSIO->slave_write(pSerialIO->slave_read());
        }
        //pGPSIO->display_rx_buffer();
    }
}

void GPSModelBase::readInputMessages()
{
    if (pGPSIO)
    {
        if (iGPSInputMessage > maxInputMessages-1)
        {
            std::cout << "GPSModelBase::readInputMessages(): Input Msg Overflow! Dropping message." << std::endl;
            return;
        }
        
        int nAvailable = pGPSIO->available();
        if (nAvailable > 0)
        {
            pInputMessages[iGPSInputMessage]->buffer_length = nAvailable;
            pInputMessages[iGPSInputMessage]->buffer = new byte[nAvailable];
            for (int i = 0; i < nAvailable; i++)
            {
                pInputMessages[iGPSInputMessage]->buffer[i] = pGPSIO->read();
            }
            pInputMessages[iGPSInputMessage]->prevUpdateTime = pTime->getSimTime();
        
            iGPSInputMessage++;
        }
    }
}

void GPSModelBase::decodeInputMessages()
{
    while(iGPSInputMessage > 0)
    {
        iGPSInputMessage--;
        receiveCount++;
        receiveTime = pTime->getSimTime();
        if (debugFlag)
        {
            std::cout << "GPSModelBase::decodeInputMessages() - Received Msg " << receiveCount;
            std::cout << " (" << pInputMessages[iGPSInputMessage]->prevUpdateTime << "s): ";
            for (int i = 0; i < pInputMessages[iGPSInputMessage]->buffer_length; i++)
            {
                std::cout << std::hex << std::setfill('0') << std::setw(2) << +pInputMessages[iGPSInputMessage]->buffer[i];
            }
            std::cout<<std::endl;
        }
        
        pInputMessages[iGPSInputMessage]->reset();
    }
}

void GPSModelBase::constructOutputMessages() { } // Update in derived class

void GPSModelBase::writeOutputMessages()
{
    for (int i = 0; i < nGPSOutputMessages; i++)
    {
        // Initialize messages
        if (pTime->getSimTime() >= pOutputMessages[i]->updateOffset && !pOutputMessages[i]->init)
        {
            pOutputMessages[i]->init = true;
            pOutputMessages[i]->prevUpdateTime = pTime->getSimTime();
        }
        
        // Send periodic messages
        if (pOutputMessages[i]->updateRate == 0.0 || pOutputMessages[i]->buffer == 0)
        {
            pOutputMessages[i]->send = false;
        }
        else if (pTime->getSimTime() >= pOutputMessages[i]->prevUpdateTime + 1.0/pOutputMessages[i]->updateRate)
        {
            pOutputMessages[i]->send = true;
        }
        
        // Send messages that are ready
        if (pGPSIO && pOutputMessages[i]->send)
        {
            if (debugFlag) { std::cout << "Sending message " << i+1 << std::endl; }
            
            pGPSIO->write(pOutputMessages[i]->buffer, pOutputMessages[i]->buffer_length);
            pOutputMessages[i]->reset();
            pOutputMessages[i]->prevUpdateTime = pTime->getSimTime();
            sendCount++;
            sendTime = pTime->getSimTime();
        }
    }
}

void GPSModelBase::writeOutputIO()
{
    if (pGPSIO)
    {
        while (pGPSIO->slave_available())
        {
            pSerialIO->slave_write(pGPSIO->slave_read());
        }
    }
}


GPSNeo6m::GPSNeo6m(ModelMap *pMapInit, bool debugFlagIn)  : GPSModelBase(pMapInit, debugFlagIn),
GPSFixLookup(GPSFixTimes, GPSFixValues, GPSFixLength),
HorizAccLookup(HorizAccTimes, HorizAccValues, HorizAccLength),
VertAccLookup(VertAccTimes, VertAccValues, VertAccLength),
VelAccLookup(VelAccTimes, VelAccValues, VelAccLength)
{
    horizPosAcc      = HorizAccLookup.update(0.0);
    vertPosAcc       = VertAccLookup.update(0.0);
    velAcc           = VelAccLookup.update(0.0);
    last_3dFixAlt    = 0.0;
    last_3dFixMSL    = 0.0;
    last_3dFixVD     = 0.0;
    
    initUBX  = false;
    pUBX     = NULL;
    pUBXFIFO = NULL;
    gpsData  = new GpsType();

    if (readGPSFile)
    {
        gps_msg_file.open(gps_file);
        gps_msg_file_begin = gps_msg_file.tellg();
        if (gps_msg_file.fail())
        {
            std::cout << "Failed to open output file: " << gps_file << "." << std::endl;
        }
        
        double d_msg = 0.001;
        double msg_time = 0.0;
        for (int i=0; i<maxOutputMessages; i++)
        {
            pOutputMessages[i] = new MessageType(0.0, msg_time);
            pOutputMessages[i]->updateRate = 1.0;
            msg_time += d_msg;
        }
    }
    else
    {
        pOutputMessages[NAV_STATUS] = new MessageType(0.0, 0.0);
        pOutputMessages[NAV_DOP]    = new MessageType(0.0, 0.001);
        pOutputMessages[NAV_POSLLH] = new MessageType(0.0, 0.002);
        pOutputMessages[NAV_VELNED] = new MessageType(0.0, 0.004);
    }
}

GPSNeo6m::~GPSNeo6m()
{
    if (pUBX)
    {
        pUBX = 0;
        delete pUBX;
    }
    
    if (pUBXFIFO)
    {
        pUBXFIFO = 0;
        delete pUBXFIFO;
    }
    
    delete gpsData;
    
    if (readGPSFile) { gps_msg_file.close(); }
}

void GPSNeo6m::updateSerial()
{
    if (pUBXFIFO)
    {
        pUBXFIFO->update_fifo();
    }
}

void GPSNeo6m::updatePositionError()
{
    horizPosAcc = HorizAccLookup.update(pTime->getSimTime());
    vertPosAcc  = VertAccLookup.update(pTime->getSimTime());
    if (HorizAccLookup.valueChange()) { horizPosNoise.setNoise(horizPosAcc); }
    if (VertAccLookup.valueChange())  { vertPosNoise.setNoise(vertPosAcc); }
    
    Base::updatePositionError();
}

void GPSNeo6m::updateVelocityError()
{
    velAcc = VelAccLookup.update(pTime->getSimTime());
    if (VelAccLookup.valueChange()) { velNoise.setNoise(velAcc); }
    
    Base::updateVelocityError();
}

void GPSNeo6m::readInputMessages()
{
    if (!initUBX && pGPSIO)
    {
        pUBXFIFO = new FS_FIFO(pGPSIO);
        pUBX     = new UBX_MSG(pUBXFIFO);
        
        pUBXFIFO->begin(pGPSIO->getBaudRate());
        initUBX = true;
    }

    if (pUBX)
    {
        int rcvd = pUBX->read(gpsData);
        receiveCount += rcvd;
        receiveTime = pTime->getSimTime();
    }
}

void GPSNeo6m::decodeInputMessages()
{
    if (!readGPSFile)
    {
        if (pOutputMessages[NAV_STATUS]->updateRate != gpsData->msgRates[NAVSTAT])
        {
            pOutputMessages[NAV_STATUS]->updateRate = gpsData->msgRates[NAVSTAT];
        }
        if (pOutputMessages[NAV_DOP]->updateRate != gpsData->msgRates[NAVDOP])
        {
            pOutputMessages[NAV_DOP]->updateRate = gpsData->msgRates[NAVDOP];
        }
        if (pOutputMessages[NAV_POSLLH]->updateRate != gpsData->msgRates[POSLLH])
        {
            pOutputMessages[NAV_POSLLH]->updateRate = gpsData->msgRates[POSLLH];
        }
        if (pOutputMessages[NAV_VELNED]->updateRate != gpsData->msgRates[VELNED])
        {
            pOutputMessages[NAV_VELNED]->updateRate = gpsData->msgRates[VELNED];
        }
    }
}

void GPSNeo6m::constructOutputMessages()
{
    if (gps_msg_file.fail())
    {
        //std::cout << pTime->getSimTime() << ") Failed to open GPS file" << std::endl;
    }
    
    if (readGPSFile && !gps_msg_file.fail())
    {
        bool done     = false;
        int msg_count = 0;
        int gpsbyte   = 0;
        UBX_MSG::MSG_PACKET ubx_msg;
        int i_msg     = 0;
        bool look_for_next_msg = true;
        std::streampos ubx_msg_begin = gps_msg_file.tellg();
        std::streampos ubx_msg_idx;
        
        while(!done && !gps_msg_file.eof())
        {
            ubx_msg_idx = gps_msg_file.tellg();
            gps_msg_file >> gpsbyte;

            if (look_for_next_msg && (byte) gpsbyte == UBX_HEADER_1)
            //if ((byte) gpsbyte == UBX_HEADER_1)
            {
                msg_count = 0;
                file_info.check_for_sent_messages(pOutputMessages);
                i_msg = file_info.get_available_msg_idx();
                ubx_msg_begin = ubx_msg_idx;
                look_for_next_msg = false;
                if (i_msg == -1) { done = true; gps_msg_file.seekg(ubx_msg_begin); break; }
                else { pOutputMessages[i_msg]->buffer = new byte[100]; }
            }
            else if (!look_for_next_msg && (byte) gpsbyte == UBX_HEADER_1)
            {
                look_for_next_msg = true;
                msg_count = 0;
            }
            else if (!look_for_next_msg)
            {
                msg_count++;
            }
            
            if (msg_count == 2) { ubx_msg.msg_class_id.data[0] = (byte) gpsbyte; }
            if (msg_count == 3)
            {
                ubx_msg.msg_class_id.data[1] = (byte) gpsbyte;
                ubx_msg.determine_input_msg_id();
                bool msg_stored = file_info.store_message(ubx_msg.msg_class, ubx_msg.msg_id);
                if (!msg_stored)
                {
                    pOutputMessages[i_msg]->reset();
                    look_for_next_msg = true;
                    gps_msg_file.seekg(ubx_msg_begin);
                    done = true;
                    break;
                }
            }
            if (msg_count == 4) { ubx_msg.msg_length.data[0] = (byte) gpsbyte; }
            if (msg_count == 5)
            {
                ubx_msg.msg_length.data[1] = (byte) gpsbyte;
                ubx_msg.determine_input_msg_length();
                pOutputMessages[i_msg]->buffer_length = ubx_msg.buffer_length + UBX_MSG_HEADER_SIZE
                + UBX_MSG_CLASS_ID_SIZE
                + UBX_MSG_LENGTH_SIZE
                + UBX_MSG_CHECKSUM_SIZE;
            }
            if (!look_for_next_msg)
            {
                pOutputMessages[i_msg]->buffer[msg_count] = (byte) gpsbyte;
                
                if (msg_count > 5 && msg_count == pOutputMessages[i_msg]->buffer_length-1)
                {
                    look_for_next_msg = true;
                    if (debugFlag)
                    {
                        std::cout << std::dec << "pOutputMessages[" << i_msg << "] " << pOutputMessages[i_msg]->buffer_length << ": ";
                        for (int i=0; i < pOutputMessages[i_msg]->buffer_length; i++)
                        {
                            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int) pOutputMessages[i_msg]->buffer[i];
                        }
                        std::cout << std::dec << std::endl << std::endl;
                    }
                }
            }
        }
    }
    else if (!readGPSFile)
    {
        UBX_MSG_TYPES::UBX_MSG_NAV_POSLLH posLLH;
        UBX_MSG_TYPES::UBX_MSG_NAV_VELNED velNED;
        UBX_MSG_TYPES::UBX_MSG_NAV_STATUS navStatus;
        UBX_MSG_TYPES::UBX_MSG_NAV_DOP    navDOP;
        double alt;
        double msl;
        double velD;
        
        // GPS fix status
        int gpsFix = GPSFixLookup.update(pTime->getSimTime());
        
        // Set GPS acquired
        if (gps_acquire_time == 0.0 && (gpsFix==2 || gpsFix==3)) { gps_acquire_time = pTime->getSimTime(); }
        
        // Set last 3D Fix Values
        if (gpsFix == 3)
        {
            last_3dFixAlt = gps_posLLH[2];
            last_3dFixMSL = gps_altMSL;
            last_3dFixVD  = gps_velNED[2];
        }
        alt  = last_3dFixAlt;
        msl  = last_3dFixMSL;
        velD = last_3dFixVD;
        
        if (gpsFix > 0)
        {
            // Generate Lat/Lon/Alt Message
            posLLH.data.iTOW               = gps_timeofweek             / posLLH.scale.iTOW;
            posLLH.data.latitude           = gps_posLLH[0]/util.deg2rad / posLLH.scale.latitude;
            posLLH.data.longitude          = gps_posLLH[1]/util.deg2rad / posLLH.scale.longitude;
            posLLH.data.altitude_ellipsoid = alt                        / posLLH.scale.altitude_ellipsoid;
            posLLH.data.altitude_msl       = msl                        / posLLH.scale.altitude_msl;
            posLLH.data.horizontalAccuracy = horizPosAcc                / posLLH.scale.horizontalAccuracy;
            posLLH.data.verticalAccuracy   = vertPosAcc                 / posLLH.scale.verticalAccuracy;
            
            // Generate NED Velocity Message
            velNED.data.iTOW            = gps_timeofweek  / velNED.scale.iTOW;
            velNED.data.velN            = gps_velNED[0]   / velNED.scale.velN;
            velNED.data.velE            = gps_velNED[1]   / velNED.scale.velE;
            velNED.data.velD            = velD            / velNED.scale.velD;
            velNED.data.speed           = gps_speed       / velNED.scale.speed;
            velNED.data.gSpeed          = gps_groundSpeed / velNED.scale.gSpeed;
            velNED.data.heading         = gps_heading     / velNED.scale.heading;
            velNED.data.speedAccuracy   = velAcc          / velNED.scale.speedAccuracy;
            velNED.data.headingAccuracy = 0.5             / velNED.scale.headingAccuracy;
            
            
            // Start NAV Status Message
            navStatus.data.iTOW    = gps_timeofweek  / navStatus.scale.iTOW;
            navStatus.data.gpsFix  = gpsFix;
            navStatus.data.ttff    = gps_acquire_time / navStatus.scale.ttff;

            navStatus.data.flags |= UBX_MSG_TYPES::GPSFixOk << 1;
        }
        else
        {
            navStatus.data.flags |= UBX_MSG_TYPES::GPSFixOk << 0;
        }
        
        // Complete NAV Status Message
        navStatus.data.flags |= UBX_MSG_TYPES::DiffSoln << 0;
        navStatus.data.flags |= UBX_MSG_TYPES::WKNSET << 0;
        navStatus.data.flags |= UBX_MSG_TYPES::TOWSET << 0;
        navStatus.data.fixStat = 0x00;
        navStatus.data.flags2  = 0x00;
        navStatus.data.msss    = pTime->getSimTime() / navStatus.scale.msss;
        
        // Store output message
        if (debugFlag) { navStatus.print(); }
        encodeOutputMessage(NAV_STATUS, &navStatus.data, sizeof(navStatus.data));
      
        if (debugFlag) { navDOP.print(); }
        encodeOutputMessage(NAV_DOP, &navDOP.data, sizeof(navDOP.data));
        
        if (debugFlag) { posLLH.print(); }
        encodeOutputMessage(NAV_POSLLH, &posLLH.data, sizeof(posLLH.data));
        
        if (debugFlag) { velNED.print(); }
        encodeOutputMessage(NAV_VELNED, &velNED.data, sizeof(velNED.data));
    }
}

int GPSNeo6m::encodeOutputMessage(int gpsMsgType, void* buffer, int buffer_length)
{
    UBX_MSG::MSG_PACKET ubx_msg;
    
    // Determine msg_id
    if (gpsMsgType == NAV_STATUS)
    {
        ubx_msg.msg_class = UBX_MSG_TYPES::NAV;
        ubx_msg.msg_id    = UBX_MSG_TYPES::NAV_STATUS;
    }
    else if (gpsMsgType == NAV_DOP)
    {
        ubx_msg.msg_class = UBX_MSG_TYPES::NAV;
        ubx_msg.msg_id    = UBX_MSG_TYPES::NAV_DOP;
    }
    else if (gpsMsgType == NAV_POSLLH)
    {
        ubx_msg.msg_class = UBX_MSG_TYPES::NAV;
        ubx_msg.msg_id    = UBX_MSG_TYPES::NAV_POSLLH;
    }
    else if (gpsMsgType == NAV_VELNED)
    {
        ubx_msg.msg_class = UBX_MSG_TYPES::NAV;
        ubx_msg.msg_id    = UBX_MSG_TYPES::NAV_VELNED;
    }
    else
    {
        return 0;
    }
    
    ubx_msg.determine_output_msg_id();
    
    // Copy data
    ubx_msg.buffer_length = buffer_length;
    memcpy(ubx_msg.buffer, buffer, buffer_length);
    
    // Compute message length and checksum
    ubx_msg.determine_output_msg_length();
    ubx_msg.calc_checksum();
    
    // Fill in output message
    pOutputMessages[gpsMsgType]->buffer_length = ubx_msg.buffer_length
    + UBX_MSG_HEADER_SIZE
    + UBX_MSG_CLASS_ID_SIZE
    + UBX_MSG_LENGTH_SIZE
    + UBX_MSG_CHECKSUM_SIZE;
    
    if (pOutputMessages[gpsMsgType]->buffer == 0)
    {
        pOutputMessages[gpsMsgType]->buffer = new byte[pOutputMessages[gpsMsgType]->buffer_length];
    }
    
    // ubx header
    int idx = 0;
    pOutputMessages[gpsMsgType]->buffer[0] = UBX_HEADER_1;
    pOutputMessages[gpsMsgType]->buffer[1] = UBX_HEADER_2;
    
    // class ID
    idx += UBX_MSG_HEADER_SIZE;
    memcpy(pOutputMessages[gpsMsgType]->buffer+idx, &ubx_msg.msg_class_id.data, UBX_MSG_CLASS_ID_SIZE);
    
    // message length
    idx += UBX_MSG_CLASS_ID_SIZE;
    memcpy(pOutputMessages[gpsMsgType]->buffer+idx, &ubx_msg.msg_length.data, UBX_MSG_LENGTH_SIZE);
    
    // message content
    idx += UBX_MSG_LENGTH_SIZE;
    memcpy(pOutputMessages[gpsMsgType]->buffer+idx, ubx_msg.buffer, ubx_msg.buffer_length);
    
    // checksum
    idx += ubx_msg.buffer_length;
    memcpy(pOutputMessages[gpsMsgType]->buffer+idx, &ubx_msg.msg_checksum.data, UBX_MSG_CHECKSUM_SIZE);
    
    if (debugFlag)
    {
        
        std::cout << std::dec << "ubx_msg.buffer " << ubx_msg.buffer_length << ": ";
        for (int i=0; i < ubx_msg.buffer_length; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int) ubx_msg.buffer[i];
        }
        std::cout << std::endl;
        
        std::cout << std::dec << "pOutputMessages[" << gpsMsgType << "] " << pOutputMessages[gpsMsgType]->buffer_length << ": ";
        for (int i=0; i < pOutputMessages[gpsMsgType]->buffer_length; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int) pOutputMessages[gpsMsgType]->buffer[i];
        }
        std::cout << std::endl << std::endl;
    }
    
    return 1;
}

GPSNeo6m::gps_file_info::gps_file_info()
{
    duplicate_class[0] = UBX_MSG_TYPES::ACK;
    duplicate_id[0]    = UBX_MSG_TYPES::ACK_ACK;
    
    duplicate_class[1] = UBX_MSG_TYPES::ACK;
    duplicate_id[1]    = UBX_MSG_TYPES::ACK_NAK;
    
    duplicate_class[2] = UBX_MSG_TYPES::UNKNOWN_MSG_CLASS;
    duplicate_id[2]    = UBX_MSG_TYPES::UNKNOWN_MSG_ID;
    
    duplicate_class[3] = UBX_MSG_TYPES::UNKNOWN_MSG_CLASS;
    duplicate_id[3]    = UBX_MSG_TYPES::UNKNOWN_MSG_ID;
    
    duplicate_class[4] = UBX_MSG_TYPES::UNKNOWN_MSG_CLASS;
    duplicate_id[4]    = UBX_MSG_TYPES::UNKNOWN_MSG_ID;
    
    clear();
}

int GPSNeo6m::gps_file_info::get_available_msg_idx()
{
    int idx = -1;
    for (int i = 0; i < maxOutputMessages; i++)
    {
        if (prev_msg_class[i] == UBX_MSG_TYPES::UNKNOWN_MSG_CLASS ||  prev_msg_id[i] == UBX_MSG_TYPES::UNKNOWN_MSG_ID)
        {
            idx = i;
            break;
        }
    }
    return idx;
}

bool GPSNeo6m::gps_file_info::store_message(int msg_class, int msg_id)
{
    bool msg_already_stored = false;
    bool msg_stored         = false;
    bool msg_valid = msg_class != UBX_MSG_TYPES::UNKNOWN_MSG_CLASS && msg_id != UBX_MSG_TYPES::UNKNOWN_MSG_ID;

    for (int i = 0; i < maxOutputMessages; i++)
    {
        if (prev_msg_class[i] == msg_class &&  prev_msg_id[i] == msg_id)
        {
            msg_already_stored = true;
            if (!allow_duplicate(msg_class, msg_id))
            {
                break;
            }
        }
        
        if (msg_valid && (allow_duplicate(msg_class, msg_id) || !msg_already_stored) && prev_msg_class[i] == UBX_MSG_TYPES::UNKNOWN_MSG_CLASS && prev_msg_id[i] == UBX_MSG_TYPES::UNKNOWN_MSG_ID)
        {
            prev_msg_class[i] = msg_class;
            prev_msg_id[i]    = msg_id;
            msg_stored        = true;
            break;
        }
    }
    return msg_stored;
}

bool GPSNeo6m::gps_file_info::allow_duplicate(int msg_class, int msg_id)
{
    bool allow = false;
    for (int i = 0; i < n_duplicate; i++)
    {
        if (duplicate_class[i] == msg_class && duplicate_id[i] == msg_id)
        {
            allow = true;
            break;
        }
    }
    
    return allow;
}

void GPSNeo6m::gps_file_info::check_for_sent_messages(MessageType* pOutputMessages[maxOutputMessages])
{
    for (int i = 0; i < maxOutputMessages; i++)
    {
        if (pOutputMessages[i]->buffer == 0 && (prev_msg_class[i] != UBX_MSG_TYPES::UNKNOWN_MSG_CLASS || prev_msg_id[i] != UBX_MSG_TYPES::UNKNOWN_MSG_ID))
        {
            prev_msg_class[i] = UBX_MSG_TYPES::UNKNOWN_MSG_CLASS;
            prev_msg_id[i]    = UBX_MSG_TYPES::UNKNOWN_MSG_ID;
            //free_message(prev_msg_class[i], prev_msg_id[i]);
        }
    }
}

bool GPSNeo6m::gps_file_info::free_message(int msg_class, int msg_id)
{
    bool msg_removed = false;
    for (int i = 0; i < maxOutputMessages; i++)
    {
        if (prev_msg_class[i] == msg_class &&  prev_msg_id[i] == msg_id)
        {
            prev_msg_class[i] = UBX_MSG_TYPES::UNKNOWN_MSG_CLASS;
            prev_msg_id[i]    = UBX_MSG_TYPES::UNKNOWN_MSG_ID;
            msg_removed = true;
            break;
        }
    }
    return msg_removed;
}

void GPSNeo6m::gps_file_info::clear()
{
    for (int i = 0; i < maxOutputMessages; i++)
    {
        prev_msg_class[i] = UBX_MSG_TYPES::UNKNOWN_MSG_CLASS;
        prev_msg_id[i]    = UBX_MSG_TYPES::UNKNOWN_MSG_ID;
    }
    i_msg = 0;
}
