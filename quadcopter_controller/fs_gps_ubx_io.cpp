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

UBX_MSG::UBX_MSG(FS_FIFO* gpsFIFO, bool debug_input_in, bool debug_output_in) : 
input_msg(debug_input_in), output_msg(debug_output_in)
{
    this->gpsFIFO = gpsFIFO;
    gpsbyte = 0;
    
    for(int i = 0; i < UBX_MSG_TYPES::NUBXCLASSES; i++)
    {
        for (int j = 0; j < UBX_MSG_TYPES::NUBXIDS; j++)
        {
            n_rcvd[i][j] = 0;
        }
    }
    
    request_nav5 = false;
    request_aid_poll = false;
}

UBX_MSG::~UBX_MSG()
{
    gpsFIFO = nullptr;
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
    memset(&configNav5, 0, sizeof(configNav5));
    
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

void UBX_MSG::init_aid_request()
{
    byte temp_msg[UBX_CFG_ON_OFF_SHORT_SIZE];
    temp_msg[0] = UBX_AID;
    temp_msg[1] = UBX_AID_REQ;
    temp_msg[2] = 0x01;
    
    // Write AID-REQ
    output_msg.msg_class     = CFG;
    output_msg.msg_id        = CFG_MSG;
    output_msg.buffer_length = UBX_CFG_ON_OFF_SHORT_SIZE;
    output_msg.set_buffer(temp_msg, output_msg.buffer_length);
    write(&output_msg);
}

void UBX_MSG::request_aid_data()
{
    output_msg.msg_class = AID;
    output_msg.msg_id    = AID_DATA;
    write(&output_msg);
}

int UBX_MSG::data_available()
{
    return (gpsFIFO->available());
}

int UBX_MSG::read()
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
                    input_msg.clear();
                    display("UBX_MSG::read - Invalid Header\n");
                }
                break;
                
            case MSG_TYPE :
                input_msg.msg_class_id.data[input_msg.msg_class_id.index] = gpsbyte;
                input_msg.msg_class_id.index++;
                if (input_msg.msg_class_id.index == UBX_MSG_CLASS_ID_SIZE)
                {
                    input_msg.determine_input_msg_id();
                    if (input_msg.msg_class != UNKNOWN_MSG_CLASS && input_msg.msg_id != UNKNOWN_MSG_ID)
                    {
                        input_msg.state = MSG_LENGTH;
                    }
                    else
                    {
                        input_msg.clear();
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
                    if (input_msg.buffer_length > 0)
                    {
                        input_msg.delete_buffer();
                        input_msg.buffer = new byte[input_msg.buffer_length];
                        input_msg.state = UPDATE_MSG;
                    }
                    else
                    {
                        input_msg.state = CALC_CHECKSUM;
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
                        byte ck_a;
                        byte ck_b;
                        input_msg.compute_checksum(ck_a, ck_b);
                        display(getTime());
                        display(") UBX_MSG::read, invalid checksum ");
                        display(+ck_a, HEX); display(+ck_b, HEX); display(". ");
                        input_msg.print();
                        input_msg.clear();
                    }
                }
                break;
                
            case COMPLETE :
                if (decode(input_msg.msg_class, input_msg.msg_id, input_msg.buffer, input_msg.buffer_length))
                {
                    n_rcvd[input_msg.msg_class][input_msg.msg_id]++;
                    nRcvd++;
                }
                else
                {
                    display("UBX_MSG::read, invalid decode. ");
                    input_msg.print();
                    /*
                    display("UBX_MSG::read - [");
                    display(input_msg.msg_class);
                    display(" ");
                    display(input_msg.msg_id);
                    display("] Invalid Decode\n");
                    */
                }
                input_msg.clear();
                break;
                
            default :
                // do nothing
                break;
        }
    }
    return nRcvd;
}

unsigned int UBX_MSG::write(MSG_PACKET* output_msg)
{
    unsigned int n_write = 0;
    
    // UBX Header
    n_write += gpsFIFO->write(UBX_HEADER_1);
    n_write += gpsFIFO->write(UBX_HEADER_2);
    
    // Msg Class and ID
    output_msg->determine_output_msg_id();
    n_write += gpsFIFO->write(output_msg->msg_class_id.data, UBX_MSG_CLASS_ID_SIZE);

    // Msg Length
    output_msg->determine_output_msg_length();
    n_write += gpsFIFO->write(output_msg->msg_length.data, UBX_MSG_LENGTH_SIZE);
    
    // Write Message
    n_write += gpsFIFO->write(output_msg->buffer, output_msg->buffer_length);
    
    // Checksum
    output_msg->calc_checksum();
    n_write += gpsFIFO->write(output_msg->msg_checksum.data, UBX_MSG_CHECKSUM_SIZE);
    //gpsFIFO->display_write_buffer();
    
    // Clear message and buffer
    output_msg->clear();
    
    return n_write;
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
    else if (msg_class == CFG && msg_id == CFG_NAV5)
    {
        if (buffer_length == 0)
        {
            request_nav5 = true;
            valid_decode = 1;
        }
        else if (buffer_length == sizeof(configNav5.data))
        {
            memcpy(&configNav5.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_class == ACK && msg_id == ACK_ACK)
    {
        if (buffer_length == sizeof(ack.data))
        {
            memcpy(&ack.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_class == ACK && msg_id == ACK_NAK)
    {
        if (buffer_length == sizeof(nak.data))
        {
            memcpy(&nak.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_class == AID && msg_id == AID_DATA)
    {
        if (buffer_length == 0)
        {
            request_aid_poll = true;
            valid_decode = 1;
        }
    }
    else if (msg_class == AID && msg_id == AID_INI)
    {
        if (buffer_length == sizeof(aidInit.data))
        {
            memcpy(&aidInit.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_class == AID && msg_id == AID_HUI)
    {
        if (buffer_length == sizeof(gpsHealth.data))
        {
            memcpy(&gpsHealth.data, buffer, buffer_length);
            valid_decode = 1;
        }
    }
    else if (msg_class == AID && msg_id == AID_ALM)
    {
        uint32_t svid = 0;
        memcpy(&svid, buffer, sizeof(svid));
        
        //display("Decoding Almanac ");
        //display(svid); display(" ");

        if (buffer_length == sizeof(almanacInvalid.data))
        {
            //memcpy(&almanacInvalid.data, buffer, buffer_length);
            memcpy(&almanac[svid-1].data, buffer, buffer_length);
            //display("invalid: \n");
            //almanac[svid-1].print();
            valid_decode = 1;
        }
        else if (buffer_length == sizeof(almanac[0].data))
        {
            memcpy(&almanac[svid-1].data, buffer, buffer_length);
            //display("valid: \n");
            //almanac[svid-1].print();
            valid_decode = 1;
        }
    }
    else if (msg_class == AID && msg_id == AID_EPH)
    {
        uint32_t svid = 0;
        memcpy(&svid, buffer, sizeof(svid));
        
        //display("Decoding Ephemeris ");
        //display(svid); display(" ");
        
        //display(buffer_length);
        //display(" ");
        //display(sizeof(ephemerisInvalid.data));
        //display(" ");
        //display(sizeof(ephemeris[0].data));
        //display("\n");
        
        if (buffer_length == sizeof(ephemerisInvalid.data))
        {
            //memcpy(&ephemerisInvalid.data, buffer, buffer_length);
            memcpy(&ephemeris[svid-1].data, buffer, buffer_length);
            //display("invalid: \n");
            //ephemeris[svid-1].print();
            valid_decode = 1;
        }
        else if (buffer_length == sizeof(ephemeris[0].data))
        {
            memcpy(&ephemeris[svid-1].data, buffer, buffer_length);
            //display("valid: \n");
            //ephemeris[svid-1].print();
            valid_decode = 1;
        }
    }
    return valid_decode;
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
                    return;
                }
            }
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
    memset(buffer_aug, 0, n);
    
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
    
    if (buffer)
    { memcpy(buffer_aug+UBX_MSG_CLASS_ID_SIZE+UBX_MSG_CLASS_ID_SIZE, buffer, buffer_length); }
    
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

void UBX_MSG::MSG_PACKET::set_buffer(byte* input, unsigned int length)
{
    delete_buffer();
    buffer = new byte[length];
    memcpy(buffer, input, length);
    
    if (debug_print)
    {
        display("set_buffer: ");
        for (int i = 0; i < length; i++)
        {
            display(+buffer[i], HEX);
        }
        display("\n");
    }
}

void UBX_MSG::MSG_PACKET::set_buffer(void* input, unsigned int length)
{
    delete_buffer();
    buffer = new byte[length];
    memcpy(buffer, input, length);
    
    if (debug_print)
    {
        display("set_buffer: ");
        for (int i = 0; i < length; i++)
        {
            display(+buffer[i], HEX);
        }
        display("\n");
    }
}

void UBX_MSG::MSG_PACKET::print()
{
    display("["); display(msg_class); display(" "); display(msg_id); display("], ");
    display(buffer_length); display(" - ");
    display(+UBX_HEADER_1, HEX);
    display(+UBX_HEADER_2, HEX);
    display(+msg_class_id.data[0], HEX);
    display(+msg_class_id.data[1], HEX);
    display(+msg_length.data[0], HEX);
    display(+msg_length.data[1], HEX);
    for (int i = 0; i < buffer_length; i++)
    {
        display(+buffer[i], HEX);
    }
    display(+msg_checksum.data[0], HEX);
    display(+msg_checksum.data[1], HEX);
    display("\n");
}

void UBX_MSG::MSG_PACKET::clear()
{
    msg_class_id.clear();
    msg_length.clear();
    msg_checksum.clear();
    delete_buffer();
    buffer_length  = 0;
    buffer_index   = 0;
    msg_id         = UNKNOWN_MSG_ID;
    msg_class      = UNKNOWN_MSG_CLASS;
    state          = HEADER1;
    checksum_valid = false;
}

void UBX_MSG::MSG_PACKET::delete_buffer()
{
    if (buffer != nullptr)
    {
        delete[] buffer;
        buffer = nullptr;
    }
}
