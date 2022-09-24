//
//  fs_gps_ubx_io.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 5/1/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#include "fs_gps_ubx_io.hpp"
#include "fs_gps.hpp"

using namespace UBX_MSG_TYPES;

UBX_MSG::UBX_MSG(FS_FIFO* gpsFIFO)
{
    this->gpsFIFO = gpsFIFO;
    gpsbyte = 0;
    clear_input_buffer();
}

UBX_MSG::~UBX_MSG()
{
    gpsFIFO = 0;
    clear_input_buffer();
    output_msg.clear();
}

void UBX_MSG::write_nmea_off()
{
    byte temp_msg[UBX_CFG_ON_OFF_SHORT_SIZE];
    temp_msg[0] = NMEA;
    temp_msg[2] = 0x00;
    
    // Clear GGA
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = NMEA_GGA;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear GLL
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = NMEA_GLL;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear GSA
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = NMEA_GSA;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear GSV
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = NMEA_GSV;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear RMC
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = NMEA_RMC;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear VTG
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = NMEA_VTG;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
}

void UBX_MSG::write_nmea_off_long()
{
    byte temp_msg[UBX_CFG_ON_OFF_LONG_SIZE];
    temp_msg[0] = NMEA;
    temp_msg[2] = 0x00;
    temp_msg[3] = 0x00;
    temp_msg[4] = 0x00;
    temp_msg[5] = 0x00;
    temp_msg[6] = 0x00;
    temp_msg[7] = 0x01;
    
    // Clear GGA
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_LONG_SIZE;
    temp_msg[1]              = NMEA_GGA;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear GLL
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_LONG_SIZE;
    temp_msg[1]              = NMEA_GLL;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);

    // Clear GSA
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_LONG_SIZE;
    temp_msg[1]              = NMEA_GSA;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear GSV
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_LONG_SIZE;
    temp_msg[1]              = NMEA_GSV;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear RMC
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_LONG_SIZE;
    temp_msg[1]              = NMEA_RMC;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear VTG
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_LONG_SIZE;
    temp_msg[1]              = NMEA_VTG;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
}

void UBX_MSG::write_ubx_off()
{
    byte temp_msg[UBX_CFG_ON_OFF_SHORT_SIZE];
    temp_msg[0] = UBX_NAV;
    temp_msg[2] = 0x00;

    // Clear NAV-POSLLH
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = UBX_NAV_POSLLH;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear NAV-STATUS
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = UBX_NAV_STATUS;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear NAV-DOP
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = UBX_NAV_DOP;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear NAV-VELNED
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = UBX_NAV_VELNED;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Clear NAV-SOL
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = UBX_NAV_SOL;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
}

void UBX_MSG::write_ubx_on()
{
    byte temp_msg[UBX_CFG_ON_OFF_SHORT_SIZE];
    temp_msg[0] = UBX_NAV;
    temp_msg[2] = 0x01;

    // Write NAV-POSLLH
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = UBX_NAV_POSLLH;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Write NAV-STATUS
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = UBX_NAV_STATUS;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Write NAV-DOP
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = UBX_NAV_DOP;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Write NAV-VELNED
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = UBX_NAV_VELNED;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
    
    // Write NAV-SOL
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    temp_msg[1]              = UBX_NAV_SOL;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
}

void UBX_MSG::set_nav_config()
{
    memset(&configNav5, 0x00, sizeof(configNav5));
    
    // Set dynamic model to airborne
    configNav5.data.dynModel = UBX_MSG_TYPES::Airbone_1g;
    configNav5.data.mask |= 1 << NavConfig_dyn;
    
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_NAV5;
    output_msg.buffer_length = sizeof(configNav5.data);
    output_msg.set_buffer(&configNav5.data, output_msg.buffer_length);
    write(&output_msg);
}

void UBX_MSG::get_nav_config()
{
    output_msg.msg_class = CFG;
    output_msg.msg_id    = CFG_NAV5;
    write(&output_msg);
}

int UBX_MSG::data_available()
{
    return (gpsFIFO->available());
}

int UBX_MSG::read(GpsType* gpsData)
{
    int nRcvd = 0;
    while (gpsFIFO->available() > 0 || input_msg.state == COMPLETE)
    {
        if (input_msg.state != COMPLETE)
        { gpsbyte = gpsFIFO->read(); }
        
        #ifdef UBX_PRINT
            display(gpsbyte);
            display("\n");
        #endif
        
        switch (input_msg.state)
        {
            case HEADER1 :
                if (gpsbyte == (byte) UBX_HEADER_1)
                {
                    input_msg.state = HEADER2;
                }
                break;
                
            case HEADER2 :
                if (gpsbyte == (byte) UBX_HEADER_2)
                {
                    input_msg.state = MSG_TYPE;
                }
                else
                {
                    clear_input_buffer();
                    display("UBX_MSG::read - Invalid Header\n");
                }
                break;
                
            case MSG_TYPE :
                input_msg.msg_class_id.data[input_msg.msg_class_id.index] = gpsbyte;
                input_msg.msg_class_id.index++;
                if (input_msg.msg_class_id.index == UBX_MSG_CLASS_ID_SIZE)
                {
                    input_msg.determine_input_msg_id();
                    if (input_msg.msg_id != UNKNOWN_MSG_ID)
                    {
                        input_msg.state = MSG_LENGTH;
                    }
                    else
                    {
                        clear_input_buffer();
                        display("UBX_MSG::read - Invalid Class ID\n");
                    }
                }
                break;
                
            case MSG_LENGTH :
                input_msg.msg_length.data[input_msg.msg_length.index] = gpsbyte;
                input_msg.msg_length.index++;
                if (input_msg.msg_length.index == UBX_MSG_LENGTH_SIZE)
                {
                    input_msg.determine_input_msg_length();
                    if (input_msg.buffer_length != 0)
                    {
                        input_msg.state = UPDATE_MSG;
                    }
                    else
                    {
                        clear_input_buffer();
                        display("UBX_MSG::read - [");
                        display(input_msg.msg_class);
                        display(" ");
                        display(input_msg.msg_id);
                        display("] Invalid MSG Length\n");
                    }
                }
                break;
                
            case UPDATE_MSG :
                input_msg.buffer[input_msg.buffer_index] = gpsbyte;
                input_msg.buffer_index++;
                if (input_msg.buffer_index >= input_msg.buffer_length)
                {
                    input_msg.state = CALC_CHECKSUM;
                }
                break;
                
            case CALC_CHECKSUM :
                input_msg.msg_checksum.data[input_msg.msg_checksum.index] = gpsbyte;
                input_msg.msg_checksum.index++;
                if (input_msg.msg_checksum.index == UBX_MSG_CHECKSUM_SIZE)
                {
                    input_msg.validate_checksum();
                    if (input_msg.checksum_valid)
                    {
                        input_msg.state = COMPLETE;
                    }
                    else
                    {
                        display("UBX_MSG::read - [");
                        display(input_msg.msg_class);
                        display(" ");
                        display(input_msg.msg_id);
                        display("] Invalid Checksum\n");
                        clear_input_buffer();
                    }
                }
                break;
                
            case COMPLETE :
                if (decode(input_msg.msg_class, input_msg.msg_id, input_msg.buffer, input_msg.buffer_length) == 1)
                {
                    update_gps_data(input_msg.msg_class, input_msg.msg_id, gpsData);
                    nRcvd++;
                }
                else
                {
                    display("UBX_MSG::read - [");
                    display(input_msg.msg_class);
                    display(" ");
                    display(input_msg.msg_id);
                    display("] Invalid Decode\n");
                }
                clear_input_buffer();
                break;
                
            default :
                // do nothing
                break;
        }
    }
    return nRcvd;
}

void UBX_MSG::write(MSG_PACKET* output_msg)
{
    // UBX Header
    gpsFIFO->write(UBX_HEADER_1);
    gpsFIFO->write(UBX_HEADER_2);
    
    // Msg Class and ID
    output_msg->determine_output_msg_id();
    gpsFIFO->write(output_msg->msg_class_id.data, UBX_MSG_CLASS_ID_SIZE);

    // Msg Length
    output_msg->determine_output_msg_length();
    gpsFIFO->write(output_msg->msg_length.data, UBX_MSG_LENGTH_SIZE);
    
    // Write Message
    gpsFIFO->write(output_msg->buffer, output_msg->buffer_length);
    
    // Checksum
    output_msg->calc_checksum();
    gpsFIFO->write(output_msg->msg_checksum.data, UBX_MSG_CHECKSUM_SIZE);
    
    // Clear message and buffer
    output_msg->clear();
}

int UBX_MSG::decode(int msg_class, int msg_id, byte* buffer, unsigned short buffer_length)
{
    int valid_decode = 0;
    if (msg_class == NAV && msg_id == NAV_POSLLH)
    {
        if (buffer_length == sizeof(posLLH.data))
        {
            memcpy(&posLLH.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_class == NAV && msg_id == NAV_VELNED)
    {
        if (buffer_length == sizeof(velNED.data))
        {
            memcpy(&velNED.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_class == NAV && msg_id == NAV_STATUS)
    {
        if (buffer_length == sizeof(navStatus.data))
        {
            memcpy(&navStatus.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_class == NAV && msg_id == NAV_DOP)
    {
        if (buffer_length == sizeof(navDOP.data))
        {
            memcpy(&navDOP.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_class == NAV && msg_id == NAV_SOL)
    {
        if (buffer_length == sizeof(navSol.data))
        {
            memcpy(&navSol.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_class == CFG && msg_id == CFG_NAV5)
    {
        if (buffer_length == sizeof(configNav5.data))
        {
            memcpy(&configNav5.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
#ifdef SIMULATION
    else if (msg_class == CFG && msg_id == CFG_MSG)
    {
        if (buffer_length == sizeof(configMsgShort.data))
        {
            memcpy(&configMsgShort.data, buffer, buffer_length);
            valid_decode = 1;
        }
        else if (buffer_length == sizeof(configMsgLong.data))
        {
            memcpy(&configMsgLong.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
#endif
    return valid_decode;
}

int UBX_MSG::update_gps_data(int msg_class, int msg_id, GpsType* gpsData)
{
    if (gpsData == NULL) { display("gpsData = NULL\n"); return 0; }
    
    gpsData->input_msg_id = msg_id;
    
    if (msg_class == NAV && msg_id == NAV_POSLLH)
    {
        gpsData->posLLH[0]   = posLLH.data.latitude * posLLH.scale.latitude;
        gpsData->posLLH[1]   = posLLH.data.longitude * posLLH.scale.longitude;
        gpsData->posLLH[2]   = posLLH.data.altitude_geod * posLLH.scale.altitude_geod;
        gpsData->alt_msl     = posLLH.data.altitude_msl * posLLH.scale.altitude_msl;
        gpsData->horizPosAcc = posLLH.data.horizontalAccuracy * posLLH.scale.horizontalAccuracy;
        gpsData->vertPosAcc  = posLLH.data.verticalAccuracy * posLLH.scale.verticalAccuracy;
        gpsData->positionValid = true;
        return 1;
    }
    else if (msg_class == NAV && msg_id == NAV_VELNED)
    {
        gpsData->velNED[0] = velNED.data.velN * velNED.scale.velN;
        gpsData->velNED[1] = velNED.data.velE * velNED.scale.velE;
        gpsData->velNED[2] = velNED.data.velD * velNED.scale.velD;
        gpsData->heading   = velNED.data.heading * velNED.scale.heading;
        gpsData->speedAcc  = velNED.data.speedAccuracy * velNED.scale.speedAccuracy;
        gpsData->hdgAcc    = velNED.data.headingAccuracy * velNED.scale.headingAccuracy;
        gpsData->velocityValid = true;
        return 1;
    }
    else if (msg_class == NAV && msg_id == NAV_STATUS)
    {
        gpsData->ttff         = navStatus.data.ttff * navStatus.scale.ttff;
        gpsData->GPStimestamp = navStatus.data.msss * navStatus.scale.msss;
        gpsData->gpsFix       = (GPS_FIX_TYPE) navStatus.data.gpsFix;
        gpsData->fixOk        = (navStatus.data.flags >> 0) & 1;
        //int GPSFixOk = (navStatus.data.flags >> 0) & 1;
        //int DiffSoln = (navStatus.data.flags >> 1) & 1;
        //int WKNSET   = (navStatus.data.flags >> 2) & 1;
        //int TOWSET   = (navStatus.data.flags >> 3) & 1;
        //display(GPSFixOk); display(" "); display(DiffSoln); display(" "); display(WKNSET); display(" "); display(TOWSET); display("\n");
        return 1;
    }
    else if (msg_class == NAV && msg_id == NAV_DOP)
    {
        gpsData->DOP.gDOP = navDOP.data.gDOP * navDOP.scale.gDOP;
        gpsData->DOP.pDOP = navDOP.data.pDOP * navDOP.scale.pDOP;
        gpsData->DOP.tDOP = navDOP.data.tDOP * navDOP.scale.tDOP;
        gpsData->DOP.vDOP = navDOP.data.vDOP * navDOP.scale.vDOP;
        gpsData->DOP.hDOP = navDOP.data.hDOP * navDOP.scale.hDOP;
        gpsData->DOP.nDOP = navDOP.data.nDOP * navDOP.scale.nDOP;
        gpsData->DOP.eDOP = navDOP.data.eDOP * navDOP.scale.eDOP;
        return 1;
    }
    else if (msg_class == NAV && msg_id == NAV_SOL)
    {
        gpsData->numSV = (int) navSol.data.numSV;
        return 1;
    }
    else if (msg_class == CFG && msg_id == CFG_NAV5)
    {
        display("NAV5 mask:\n");
        display((int)(configNav5.data.mask >> NavConfig_dyn) & 1); display(" ");
        display((int)(configNav5.data.mask >> NavConfig_minEl) & 1); display(" ");
        display((int)(configNav5.data.mask >> NavConfig_fixMode) & 1); display(" ");
        display((int)(configNav5.data.mask >> NavConfig_drLim) & 1); display(" ");
        display((int)(configNav5.data.mask >> NavConfig_posMask) & 1); display(" ");
        display((int)(configNav5.data.mask >> NavConfig_timeMask) & 1); display(" ");
        display((int)(configNav5.data.mask >> NavConfig_staticHoldMeask) & 1); display(" ");
        display((int)(configNav5.data.mask >> NavConfig_dgpsMask) & 1); display("\n");
        
        display("NAV5 data:\n");
        display((int) configNav5.data.dynModel); display(" ");
        display((int) configNav5.data.fixMode);  display(" ");
        display(configNav5.data.fixedAlt                 * configNav5.scale.fixedAlt); display(" ");
        display(configNav5.data.fixedAltVar              * configNav5.scale.fixedAltVar); display(" ");
        display(configNav5.data.minElv                   * configNav5.scale.minElv); display(" ");
        display(((int) configNav5.data.drLimit)          * configNav5.scale.drLimit); display(" ");
        display(configNav5.data.pDop                     * configNav5.scale.pDop); display(" ");
        display(configNav5.data.tDop                     * configNav5.scale.tDop); display(" ");
        display(configNav5.data.pAcc                     * configNav5.scale.pAcc); display(" ");
        display(configNav5.data.tAcc                     * configNav5.scale.tAcc); display(" ");
        display(((int) configNav5.data.staticHoldThresh) * configNav5.scale.staticHoldThresh); display(" ");
        display(((int) configNav5.data.dgpsTimeOut)      * configNav5.scale.dgpsTimeOut); display("\n");
    }
#ifdef SIMULATION
    else if (msg_class == CFG && msg_id == CFG_MSG)
    {
        byte msgClass;
        byte msgID;
        int rate;
        if (configMsgShort.data.msgClass!=0 && configMsgShort.data.msgID!=0)
        {
            msgClass = configMsgShort.data.msgClass;
            msgID    = configMsgShort.data.msgID;
            rate     = (int) configMsgShort.data.rate;
        }
        else
        {
            msgClass = configMsgLong.data.msgClass;
            msgID    = configMsgLong.data.msgID;
            rate     = (int) configMsgLong.data.rateTgt2; // UART port
        }
        
        if (msgClass == (byte) NMEA && msgID == (byte) NMEA_GGA)
        {
            gpsData->msgRates[GGA] = rate;
            return 1;
        }
        else if (msgClass == (byte) NMEA && msgID == (byte) NMEA_GLL)
        {
            gpsData->msgRates[GLL] = rate;
            return 1;
        }
        else if (msgClass == (byte) NMEA && msgID == (byte) NMEA_GSA)
        {
            gpsData->msgRates[GSA] = rate;
            return 1;
        }
        else if (msgClass == (byte) NMEA && msgID == (byte) NMEA_GSV)
        {
            gpsData->msgRates[GSV] = rate;
            return 1;
        }
        else if (msgClass == (byte) NMEA && msgID == (byte) NMEA_RMC)
        {
            gpsData->msgRates[RMC] = rate;
            return 1;
        }
        else if (msgClass == (byte) NMEA && msgID == (byte) NMEA_VTG)
        {
            gpsData->msgRates[VTG] = rate;
            return 1;
        }
        else if (msgClass == (byte) UBX_NAV && msgID == (byte) UBX_NAV_STATUS)
        {
            gpsData->msgRates[NAVSTAT] = rate;
            return 1;
        }
        else if (msgClass == (byte) UBX_NAV && msgID == (byte) UBX_NAV_DOP)
        {
            gpsData->msgRates[NAVDOP] = rate;
            return 1;
        }
        else if (msgClass == (byte) UBX_NAV && msgID == (byte) UBX_NAV_POSLLH)
        {
            gpsData->msgRates[POSLLH] = rate;
            return 1;
        }
        else if (msgClass == (byte) UBX_NAV && msgID == (byte) UBX_NAV_VELNED)
        {
            gpsData->msgRates[VELNED] = rate;
            return 1;
        }
        return 0;
    }
#endif
    return 0;
}

void UBX_MSG::clear_input_buffer()
{
    input_msg.clear();
}

// TWO_BYTE_DATA //
void UBX_MSG::TWO_BYTE_DATA::swap()
{
    byte prev_data[2];
    memcpy(prev_data, data, 2);
    data[0] = prev_data[1];
    data[1] = prev_data[0];
    
    if (debug_print)
    {
        display("TWO_BYTE_DATA::swap(), prev_data: ");
        display(+prev_data[0], HEX);
        display(+prev_data[1], HEX);
        display(+data[0], HEX);
        display(+data[1], HEX);
        display("\n");
    }
}

void UBX_MSG::TWO_BYTE_DATA::clear()
{
    memset(data, 0x00, 2);
    if (debug_print)
    {
        display("TWO_BYTE_DATA::clear(), data: ");
        display(+data[0], HEX);
        display(+data[1], HEX);
        display("\n");
    }
    index = 0;
}

// MSG_PACKET //
void UBX_MSG::MSG_PACKET::determine_input_msg_id()
{
    for (int class_id = 0; class_id < NUBXCLASSES; class_id++)
    {
        if (msg_class_id.data[0] == (byte) UBX_MSG_CLASS[class_id])
        {
            msg_class = class_id;
            for (int i_msg_id = 0; i_msg_id < N_UBX_MSG_ID[msg_class]; i_msg_id++)
            {
                if (msg_class_id.data[1] == (byte) UBX_MSG_ID[msg_class][i_msg_id])
                {
                    msg_id = i_msg_id;
                    break;
                }
            }
            break;
        }
    }
}

void UBX_MSG::MSG_PACKET::determine_input_msg_length()
{
    if (debug_print)
    {
        display("MSG_PACKET::determine_input_msg_length(), msg_length.data: ");
        display(+msg_length.data[0], HEX);
        display(+msg_length.data[1], HEX);
        display("\n");
        display("buffer_length before: ");
        display(buffer_length);
    }
    
    memcpy(&buffer_length, msg_length.data, 2);
    
    if (debug_print)
    {
        display(", buffer_length after: ");
        display(buffer_length);
        display("\n");
        display("MSG_PACKET::determine_input_msg_length(), msg_length.data buffer_length after hex: ");
        display(+buffer_length, HEX);
        display("\n");
    }
}

void UBX_MSG::MSG_PACKET::determine_output_msg_id()
{
    if (msg_class == UNKNOWN_MSG_CLASS || msg_id == UNKNOWN_MSG_ID) { return; }
    
    msg_class_id.data[0] = UBX_MSG_CLASS[msg_class];
    msg_class_id.data[1] = UBX_MSG_ID[msg_class][msg_id];
    
    if (debug_print)
    {
        display("MSG_PACKET::determine_output_msg_id(), msg_class_id.data: ");
        display(+msg_class_id.data[0], HEX);
        display(+msg_class_id.data[1], HEX);
        display("\n");
    }
}

void UBX_MSG::MSG_PACKET::determine_output_msg_length()
{
    
    if (debug_print) { display("MSG_PACKET::determine_output_msg_length(), buffer_length: "); display(buffer_length); }
    
    memcpy(msg_length.data, &buffer_length, 2);
    
    if (debug_print)
    {
        display(", msg_length.data: ");
        display(+msg_length.data[0], HEX);
        display(+msg_length.data[1], HEX);
        display("\n");
    }
}

void UBX_MSG::MSG_PACKET::calc_checksum()
{
    if (debug_print)
    {
        display("MSG_PACKET::calc_checksum, msg_checksum.data before: ");
        display(+msg_checksum.data[0], HEX);
        display(+msg_checksum.data[1], HEX);
        display("\n");
    }
        
    compute_checksum(msg_checksum.data[0], msg_checksum.data[1]);
    
    if (debug_print)
    {
        display("MSG_PACKET::calc_checksum, msg_checksum.data after: ");
        display(+msg_checksum.data[0], HEX);
        display(+msg_checksum.data[1], HEX);
        display("\n");
    }
}

void UBX_MSG::MSG_PACKET::compute_checksum(byte & ck_a, byte & ck_b)
{
    int n = buffer_length + UBX_MSG_CLASS_ID_SIZE + UBX_MSG_LENGTH_SIZE;
    byte buffer_aug[n];
    memset(buffer_aug, 0x00, n);
    
    if (debug_print)
    {
        display("MSG_PACKET::compute_checksum, buffer_aug before: ");
        for (int i=0; i<n; i++) { display(+buffer_aug[i], HEX); }
    }
    
    memcpy(buffer_aug, msg_class_id.data, UBX_MSG_CLASS_ID_SIZE);
    if (debug_print)
    {
        display(", buffer_aug class_id: ");
        for (int i=0; i<n; i++) { display(+buffer_aug[i], HEX); }
    }
    
    memcpy(buffer_aug+UBX_MSG_CLASS_ID_SIZE, msg_length.data, UBX_MSG_LENGTH_SIZE);
    if (debug_print)
    {
        display(", buffer_aug msg_length: ");
        for (int i=0; i<n; i++) { display(+buffer_aug[i], HEX); }
    }
    
    memcpy(buffer_aug+UBX_MSG_CLASS_ID_SIZE+UBX_MSG_CLASS_ID_SIZE, buffer, buffer_length);
    if (debug_print)
    {
        display(", buffer_aug after: ");
        for (int i=0; i<n; i++) { display(+buffer_aug[i], HEX); }
    }
    
    ck_a = 0;
    ck_b = 0;
    for (int i = 0; i < n; i++)
    {
        ck_a = ck_a + buffer_aug[i];
        ck_b = ck_b + ck_a;
    }
    
    if (debug_print)
    {
        display(", ck_a/ck_b: ");
        display(+ck_a, HEX);
        display("/");
        display(+ck_b, HEX);
        display("\n");
    }
}

void UBX_MSG::MSG_PACKET::validate_checksum()
{
    byte ck_a;
    byte ck_b;
    compute_checksum(ck_a, ck_b);
    
    if (debug_print)
    {
        display("MSG_PACKET::validate_checksum, ck_a/ck_b: ");
        display(+ck_a, HEX);
        display("/");
        display(+ck_b, HEX);

        display(", msg_checksum.data[0]/[1]: ");
        display(+msg_checksum.data[0], HEX);
        display("/");
        display(+msg_checksum.data[1], HEX);
        display("\n");
    }
    
    if ( (msg_checksum.data[0] == ck_a) && (msg_checksum.data[1] == ck_b) )
    {
        checksum_valid = true;
    }
}

void UBX_MSG::MSG_PACKET::set_buffer(byte* input, int length)
{
    if (debug_print) { display("set_buffer: "); }
    for (int i = 0; i < length; i++)
    {
        if (debug_print) { display(+input[i], HEX); }
        buffer[i] = input[i];
    }
    if (debug_print) { display("\n"); }
}

void UBX_MSG::MSG_PACKET::set_buffer(void* input, int length)
{
    memcpy(buffer, input, length);
    
    if (debug_print) { display("set_buffer: "); }
    for (int i = 0; i < length; i++)
    {
        if (debug_print) { display(+buffer[i], HEX); }
    }
    if (debug_print) { display("\n"); }
}

void UBX_MSG::MSG_PACKET::print()
{
    for (int i = 0; i < buffer_length; i++)
    {
        display(buffer[i], HEX);
    }
    display("\n");
}

void UBX_MSG::MSG_PACKET::clear()
{
    msg_class_id.clear();
    msg_length.clear();
    msg_checksum.clear();
    memset(buffer, 0x00, UBX_BUFFER_MAX_SIZE);
    buffer_length  = 0;
    buffer_index   = 0;
    msg_id         = UNKNOWN_MSG_ID;
    msg_class      = UNKNOWN_MSG_CLASS;
    state          = HEADER1;
    checksum_valid = false;
}
