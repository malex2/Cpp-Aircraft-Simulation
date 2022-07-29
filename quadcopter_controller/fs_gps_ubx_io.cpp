//
//  fs_gps_ubx_io.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 5/1/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#include "fs_gps_ubx_io.hpp"
#include "fs_gps.hpp"

UBX_MSG::UBX_MSG(SoftwareSerial* gpsIOin)
{
    gpsIO   = gpsIOin;
    gpsbyte = 0;
    length  = UBX_BUFFER_MAX_SIZE;
    clear_buffer();
}

void UBX_MSG::write_nmea_off()
{
    unsigned short buffer_length = 3;
    
    // Clear GGA
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_GGA,0x00};
    write(&output_msg);
    
    // Clear GLL
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_GLL,0x00};
    write(&output_msg);
    
    // Clear GSA
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_GSA,0x00};
    write(&output_msg);
    
    // Clear GSV
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_GSV,0x00};
    write(&output_msg);
    
    // Clear RMC
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_RMC,0x00};
    write(&output_msg);
    
    // Clear VTG
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_VTG,0x00};
    write(&output_msg);
}

void UBX_MSG::write_nmea_off_long()
{
    unsigned short buffer_length = 8;
    
    // Clear GGA
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_GGA,0x00,0x00,0x00,0x00,0x00,0x01};
    write(&output_msg);
    
    // Clear GLL
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_GLL,0x00,0x00,0x00,0x00,0x00,0x01};
    write(&output_msg);
    
    // Clear GSA
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_GSA,0x00,0x00,0x00,0x00,0x00,0x01};
    write(&output_msg);
    
    // Clear GSV
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_GSV,0x00,0x00,0x00,0x00,0x00,0x01};
    write(&output_msg);
    
    // Clear RMC
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_RMC,0x00,0x00,0x00,0x00,0x00,0x01};
    write(&output_msg);
    
    // Clear VTG
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {NMEA,NMEA_VTG,0x00,0x00,0x00,0x00,0x00,0x01};
    write(&output_msg);
}

void UBX_MSG::write_ubx_off()
{
    unsigned short buffer_length = 8;
    
    // Clear NAV-POSLLH
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {UBX_NAV,UBX_NAV_POSLLH,0x00,0x00,0x00,0x00,0x00,0x00};
    write(&output_msg);
    
    // Clear NAV-STATUS
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {UBX_NAV,UBX_NAV_STATUS,0x00,0x00,0x00,0x00,0x00,0x00};
    write(&output_msg);
  
    // Clear NAV-VELNED
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {UBX_NAV,UBX_NAV_VELNED,0x00,0x00,0x00,0x00,0x00,0x00};
    write(&output_msg);
    
    // Disable UBX
    //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9,
    //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off
}

void UBX_MSG::write_ubx_on()
{
    unsigned short buffer_length = 8;
    
    // Clear NAV-POSLLH
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {UBX_NAV,UBX_NAV_POSLLH,0x00,0x01,0x00,0x00,0x00,0x00};
    write(&output_msg);
    
    // Clear NAV-STATUS
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {UBX_NAV,UBX_NAV_STATUS,0x00,0x01,0x00,0x00,0x00,0x00};
    write(&output_msg);
    
    // Clear NAV-VELNED
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = buffer_length;
    output_msg.buffer        = new byte[buffer_length] {UBX_NAV,UBX_NAV_VELNED,0x00,0x01,0x00,0x00,0x00,0x00};
    write(&output_msg);
    
    // Enable UBX
    //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
    //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on
}

int UBX_MSG::data_available()
{
    return (gpsIO->available());
}

int UBX_MSG::read(GpsType* gpsData)
{
    input_msg.state = HEADER1;
    int nRcvd = 0;
    while (gpsIO->available() > 0 || input_msg.state == COMPLETE)
    {
        if (input_msg.state != COMPLETE)
        { gpsbyte = gpsIO->read(); }
        
        //std::cout << std::hex << std::setfill('0') << std::setw(2) << +gpsbyte << std::endl;
        
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
                    input_msg.state = HEADER1;
                }
                break;
                
            case MSG_TYPE :
                input_msg.msg_class_id.data[input_msg.msg_class_id.index] = gpsbyte;
                input_msg.msg_class_id.index++;
                if (input_msg.msg_class_id.index == UBX_MSG_CLASS_ID_SIZE)
                {
                    input_msg.determine_input_msg_id();
                    if (input_msg.msg_id != NONE)
                    {
                        input_msg.state = MSG_LENGTH;
                    }
                    else
                    {
                        input_msg.state = HEADER1;
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
                        input_msg.buffer = new byte[input_msg.buffer_length];
                        input_msg.state = UPDATE_MSG;
                    }
                    else
                    {
                        input_msg.state = HEADER1;
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
                        input_msg.state = HEADER1;
                    }
                }
                break;
                
            case COMPLETE :
                decode(input_msg.msg_id, input_msg.buffer, input_msg.buffer_length);
                update_gps_data(input_msg.msg_id, gpsData);
                input_msg.clear();
                input_msg.state = HEADER1;
                nRcvd++;
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
    gpsIO->write(UBX_HEADER_1);
    gpsIO->write(UBX_HEADER_2);
    
    // Msg Class and ID
    output_msg->determine_output_msg_id();
    gpsIO->write(output_msg->msg_class_id.data, UBX_MSG_CLASS_ID_SIZE);

    // Msg Length
    output_msg->determine_output_msg_length();
    gpsIO->write(output_msg->msg_length.data, UBX_MSG_LENGTH_SIZE);
    
    // Write Message
    gpsIO->write(output_msg->buffer, output_msg->buffer_length);
    
    // Checksum
    output_msg->calc_checksum();
    gpsIO->write(output_msg->msg_checksum.data, UBX_MSG_CHECKSUM_SIZE);
    
    output_msg->clear();
    // Clear buffer
    clear_buffer();
}

int UBX_MSG::decode(int msg_id, byte* buffer, unsigned short buffer_length)
{
    int valid_decode = 0;
    if (msg_id == NAV_POSLLH)
    {
        if (buffer_length == sizeof(posLLH.data))
        {
            memcpy(&posLLH.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_id == NAV_VELNED)
    {
        if (buffer_length == sizeof(velNED.data))
        {
            memcpy(&velNED.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_id == NAV_STATUS)
    {
        if (buffer_length == sizeof(navStatus.data))
        {
            memcpy(&navStatus.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
#ifdef SIMULATION
    else if (msg_id == CFG_MSG)
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
#endif
    }
    return valid_decode;
}

int UBX_MSG::update_gps_data(int msg_id, GpsType* gpsData)
{
    if (gpsData == NULL) {return 0;}
    
    gpsData->input_msg_id = msg_id;
    
    if (msg_id == NAV_POSLLH)
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
    else if (msg_id == NAV_VELNED)
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
    else if (msg_id == NAV_STATUS)
    {
        gpsData->gpsFix = (GPS_FIX_TYPE) navStatus.data.gpsFix;
        return 1;
    }
#ifdef SIMULATION
    else if (msg_id == CFG_MSG)
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
#endif
        return 0;
    }

    return 0;
}

void UBX_MSG::clear_buffer()
{
    memset(buffer, UBX_BUFFER_MAX_SIZE, 0x00);
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
        std::cout << "TWO_BYTE_DATA::swap(), prev_data: " << std::hex << std::setfill('0') << std::setw(2) << +prev_data[0];
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +prev_data[1];
        std::cout << ", data: " << std::hex << std::setfill('0') << std::setw(2) << +data[0];
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +data[1] << std::endl;
    }
}

void UBX_MSG::TWO_BYTE_DATA::clear()
{
    memset(data, 2, 0x00);
    if (debug_print)
    {
        std::cout << "TWO_BYTE_DATA::clear(), data: " << std::hex << std::setfill('0') << std::setw(2) << +data[0];
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +data[1] << std::endl;
    }
    index = 0;
}

// MSG_PACKET //
void UBX_MSG::MSG_PACKET::determine_input_msg_id()
{
    if (msg_class_id.data[0] == (byte) UBX_NAV)
    {
        if (msg_class_id.data[1] == (byte) UBX_NAV_STATUS)
        {
            msg_id = NAV_STATUS;
        }
        else if (msg_class_id.data[1] == (byte) UBX_NAV_POSLLH)
        {
            msg_id = NAV_POSLLH;
        }
        else if (msg_class_id.data[1] == (byte) UBX_NAV_VELNED)
        {
            msg_id = NAV_VELNED;
        }
    }
    else if(msg_class_id.data[0] == (byte) UBX_CFG)
    {
        if (msg_class_id.data[1] == (byte) UBX_CFG_MSG)
        {
            msg_id = CFG_MSG;
        }
    }
}

void UBX_MSG::MSG_PACKET::determine_input_msg_length()
{
    if (debug_print)
    {
        std::cout << "MSG_PACKET::determine_input_msg_length(), msg_length.data: " << std::hex << std::setfill('0') << std::setw(2) << +msg_length.data[0];
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +msg_length.data[1] << std::endl;
        std::cout << ", buffer_length before: " << buffer_length;
    }
    
    memcpy(&buffer_length, msg_length.data, 2);
    
    if (debug_print)
    {
        std::cout << ", buffer_length after: " << buffer_length;
        std::cout << "MSG_PACKET::determine_input_msg_length(), msg_length.data buffer_length after hex: " << std::hex << std::setfill('0') << std::setw(4) << +buffer_length << std::endl;
    }
}

void UBX_MSG::MSG_PACKET::determine_output_msg_id()
{
    if (msg_id == CFG_MSG)
    {
        msg_class_id.data[0] = (byte) UBX_CFG;
        msg_class_id.data[1] = (byte) UBX_CFG_MSG;
    }
    else if (msg_id == NAV_STATUS)
    {
        msg_class_id.data[0] = (byte) UBX_NAV;
        msg_class_id.data[1] = (byte) UBX_NAV_STATUS;
    }
    else if (msg_id == NAV_POSLLH)
    {
        msg_class_id.data[0] = (byte) UBX_NAV;
        msg_class_id.data[1] = (byte) UBX_NAV_POSLLH;
    }
    else if (msg_id == NAV_VELNED)
    {
        msg_class_id.data[0] = (byte) UBX_NAV;
        msg_class_id.data[1] = (byte) UBX_NAV_VELNED;
    }

    if (debug_print)
    {
        std::cout << "MSG_PACKET::determine_output_msg_id(), msg_class_id.data: " << std::hex << std::setfill('0') << std::setw(2) << +msg_class_id.data[0];
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +msg_class_id.data[1] << std::endl;
    }
}

void UBX_MSG::MSG_PACKET::determine_output_msg_length()
{
    if (debug_print) { std::cout << "MSG_PACKET::determine_output_msg_length(), buffer_length: " << buffer_length; }
    
    memcpy(msg_length.data, &buffer_length, 2);
    
    if (debug_print)
    {
        std::cout << ", msg_length.data: " << std::hex << std::setfill('0') << std::setw(2) << +msg_length.data[0];
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +msg_length.data[1] << std::endl;
    }
}

void UBX_MSG::MSG_PACKET::calc_checksum()
{
    if (debug_print)
    {
        std::cout << "MSG_PACKET::calc_checksum, msg_checksum.data before: " << std::hex << std::setfill('0') << std::setw(2) << +msg_checksum.data[0];
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +msg_checksum.data[1] << std::endl;
    }
        
    compute_checksum(msg_checksum.data[0], msg_checksum.data[1]);
    
    if (debug_print)
    {
        std::cout << "MSG_PACKET::calc_checksum, msg_checksum.data after: " << std::hex << std::setfill('0') << std::setw(2) << +msg_checksum.data[0];
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +msg_checksum.data[1] << std::endl;
    }
}

void UBX_MSG::MSG_PACKET::compute_checksum(byte & ck_a, byte & ck_b)
{
    int n = buffer_length + UBX_MSG_CLASS_ID_SIZE + UBX_MSG_LENGTH_SIZE;
    byte buffer_aug[n];
    for (int i = 0; i < n; i++)
    { buffer_aug[i] = 0x00; }
    
    if (debug_print)
    {
        std::cout << "MSG_PACKET::compute_checksum, buffer_aug before: ";
        for (int i=0; i<n; i++) { std::cout << std::hex << std::setfill('0') << std::setw(2) << +buffer_aug[i]; }
    }
    
    memcpy(buffer_aug, msg_class_id.data, UBX_MSG_CLASS_ID_SIZE);
    if (debug_print)
    {
        std::cout << ", buffer_aug class_id: ";
        for (int i=0; i<n; i++) { std::cout << std::hex << std::setfill('0') << std::setw(2) << +buffer_aug[i]; }
    }
    
    memcpy(buffer_aug+UBX_MSG_CLASS_ID_SIZE, msg_length.data, UBX_MSG_LENGTH_SIZE);
    if (debug_print)
    {
        std::cout << ", buffer_aug msg_length: ";
        for (int i=0; i<n; i++) { std::cout << std::hex << std::setfill('0') << std::setw(2) << +buffer_aug[i]; }
    }
    
    memcpy(buffer_aug+UBX_MSG_CLASS_ID_SIZE+UBX_MSG_CLASS_ID_SIZE, buffer, buffer_length);
    if (debug_print)
    {
        std::cout << ", buffer_aug after: ";
        for (int i=0; i<n; i++) { std::cout << std::hex << std::setfill('0') << std::setw(2) << +buffer_aug[i]; }
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
        std::cout << ", ck_a/ck_b: " << std::hex << std::setfill('0') << std::setw(2) << +ck_a << "/";
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +ck_b << std::endl;
    }
}

void UBX_MSG::MSG_PACKET::validate_checksum()
{
    byte ck_a;
    byte ck_b;
    compute_checksum(ck_a, ck_b);
    
    if (debug_print)
    {
        std::cout << "MSG_PACKET::validate_checksum, ck_a/ck_b: " << std::hex << std::setfill('0') << std::setw(2) << +ck_a << "/";
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +ck_b;
    
        std::cout << ", msg_checksum.data[0]/[1]: " << std::hex << std::setfill('0') << std::setw(2) << +msg_checksum.data[0] << "/";
        std::cout << std::hex << std::setfill('0') << std::setw(2) << +msg_checksum.data[1] << std::endl;
    }
    
    if ( (msg_checksum.data[0] == ck_a) && (msg_checksum.data[1] == ck_b) )
    {
        checksum_valid = true;
    }
}

void UBX_MSG::MSG_PACKET::clear()
{
    msg_class_id.clear();
    msg_length.clear();
    msg_checksum.clear();
    if (buffer)
    {
        delete [] buffer;
        buffer = 0;
    }
    buffer_length = 0;
    buffer_index  = 0;
    msg_id = NONE;
    state  = READY;
    checksum_valid = false;
}
