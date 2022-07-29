//
//  fs_gps_ubx_io.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 5/1/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#ifndef fs_gps_ubx_io_hpp
#define fs_gps_ubx_io_hpp

#include "fs_common.hpp"

class UBX_MSG
{
public:
    UBX_MSG(SoftwareSerial* gpsIOin);
    
    enum UBX_MSG_ID {NONE, CFG_MSG, NAV_STATUS, NAV_POSLLH, NAV_VELNED};
    enum UBX_IO_STATE {READY, HEADER1, HEADER2, MSG_TYPE, MSG_LENGTH, UPDATE_MSG, CALC_CHECKSUM, COMPLETE, INVALID};
    
    void write_nmea_off();
    void write_nmea_off_long();
    void write_ubx_off();
    void write_ubx_on();
    
    int data_available();
    int read(GpsType* gpsData = NULL);
    
    // Data Types
    struct TWO_BYTE_DATA
    {
        TWO_BYTE_DATA(bool debug = false) { debug_print = debug; clear(); }
        
        byte         data[2];
        unsigned int index;
        bool         debug_print;
        
        void swap();
        void clear();
    };
    
    struct MSG_PACKET
    {
        MSG_PACKET(bool debug = false) { debug_print = debug; buffer = 0; clear(); }
        
        TWO_BYTE_DATA  msg_class_id;
        TWO_BYTE_DATA  msg_length;
        TWO_BYTE_DATA  msg_checksum;
        byte*          buffer;
        unsigned int   buffer_index;
        unsigned short buffer_length;
        bool           checksum_valid;
        int            msg_id;
        UBX_IO_STATE   state;
        bool           debug_print;
        
        void determine_input_msg_id();
        void determine_input_msg_length();
        
        void determine_output_msg_id();
        void determine_output_msg_length();
        
        void calc_checksum();
        void compute_checksum(byte & ck_a, byte & ck_b);
        void validate_checksum();
        
        void clear();
    };
    
    // UBX Data Types
    struct UBX_MSG_NAV_POSLLH
    {
        struct data_t
        {
            uint32_t iTOW               = 0;
            int32_t longitude                   = 0;
            int32_t latitude                    = 0;
            int32_t altitude_geod               = 0;
            int32_t altitude_msl                = 0;
            uint32_t horizontalAccuracy = 0;
            uint32_t verticalAccuracy   = 0;
        } data;
        
        struct scale_t
        {
            double iTOW               = 1.0e-3;
            double longitude          = 1.0e-7;
            double latitude           = 1.0e-7;
            double altitude_geod      = 1.0e-3;
            double altitude_msl       = 1.0e-3;
            double horizontalAccuracy = 1.0e-3;
            double verticalAccuracy   = 1.0e-3;
        } scale;
        
        void print()
        {
            std::cout << std::dec << "UBX_MSG_NAV_POSLLH (data): ";
            std::cout << "iTOW " << data.iTOW << ", ";
            std::cout << "longitude " << data.longitude << ", ";
            std::cout << "latitude " << data.latitude << ", ";
            std::cout << "altitude_geod " << data.altitude_geod << ", ";
            std::cout << "altitude_msl " << data.altitude_msl << ", ";
            std::cout << "horizontalAccuracy " << data.horizontalAccuracy << ", ";
            std::cout << "verticalAccuracy " << data.verticalAccuracy << ", ";
            std::cout << std::endl;
            
            std::cout << "UBX_MSG_NAV_POSLLH (bytes) " << sizeof(data) << ": ";
            for (int i=0; i < sizeof(data); i++)
            {
                std::cout << std::hex << std::setfill('0') << std::setw(2) << +((char*)&data)[i];
            }
            std::cout << std::endl;
        }
    };
    
    struct UBX_MSG_NAV_VELNED
    {
        struct data_t
        {
            uint32_t iTOW            = 0;
            int32_t velN                     = 0;
            int32_t velE                     = 0;
            int32_t velD                     = 0;
            uint32_t speed           = 0;
            uint32_t gSpeed          = 0;
            int32_t heading                  = 0;
            uint32_t speedAccuracy   = 0;
            uint32_t headingAccuracy = 0;
        } data;
        
        struct scale_t
        {
            double iTOW            = 1.0e-3;
            double velN            = 1.0e-2;
            double velE            = 1.0e-2;
            double velD            = 1.0e-2;
            double speed           = 1.0e-2;
            double gSpeed          = 1.0e-2;
            double heading         = 1.0e-5;
            double speedAccuracy   = 1.0e-2;
            double headingAccuracy = 1.0e-5;
        } scale;
        
        void print()
        {
            std::cout << std::dec << "UBX_MSG_NAV_VELNED (data): ";
            std::cout << "iTOW " << data.iTOW << ", ";
            std::cout << "velN " << data.velN << ", ";
            std::cout << "velE " << data.velE << ", ";
            std::cout << "velD " << data.velD << ", ";
            std::cout << "speed " << data.speed << ", ";
            std::cout << "gSpeed " << data.gSpeed << ", ";
            std::cout << "heading " << data.heading << ", ";
            std::cout << "speedAccuracy " << data.speedAccuracy << ", ";
            std::cout << "headingAccuracy " << data.headingAccuracy << ", ";
            std::cout << std::endl;
            
            std::cout << "UBX_MSG_NAV_VELNED (bytes) " << sizeof(data) << ": ";
            for (int i=0; i < sizeof(data); i++)
            {
                std::cout << std::hex << std::setfill('0') << std::setw(2) << +((char*)&data)[i];
            }
            std::cout << std::endl;
        }
    };
    
    struct UBX_MSG_NAV_STATUS
    {
        struct data_t
        {
            uint32_t iTOW = 0;
            byte gpsFix        = 0;
            char flags         = 0;
            char fixStat       = 0;
            char flags2        = 0;
            uint32_t ttff = 0;
            uint32_t msss = 0;
        } data;
        
        struct scale_t
        {
            double iTOW    = 1.0e-3;
            double gpsFix  = 1.0;
            double flags   = 1.0;
            double fixStat = 1.0;
            double flags2  = 1.0;
            double ttff    = 1.0e-3;
            double msss    = 1.0e-3;
        } scale;
        
        void print()
        {
            std::cout << std::dec << "UBX_MSG_NAV_STATUS (data): ";
            std::cout << "iTOW " << data.iTOW << ", ";
            std::cout << "gpsFix " << (int) data.gpsFix << ", ";
            std::cout << "flags " << (int) data.flags << ", ";
            std::cout << "fixStat " << (int) data.fixStat << ", ";
            std::cout << "flags2 " <<(int)  data.flags2 << ", ";
            std::cout << "ttff " << data.ttff << ", ";
            std::cout << "msss " << data.msss << ", ";
            std::cout << std::endl;
            
            std::cout << "UBX_MSG_NAV_STATUS (bytes) " << sizeof(data) << ": ";
            for (int i=0; i < sizeof(data); i++)
            {
                std::cout << std::hex << std::setfill('0') << std::setw(2) << +((char*)&data)[i];
            }
            std::cout << std::endl;
        }
    };
    
    struct UBX_MSG_CFG_MSG_SHORT
    {
        struct data_t
        {
            byte msgClass = 0;
            byte msgID    = 0;
            byte rate     = 0;
        } data;
        
        struct scale_t
        {
            double msgClass = 1.0;
            double msgID    = 1.0;
            double rate     = 1.0;
        } scale;
    };
    
    struct UBX_MSG_CFG_MSG_LONG
    {
        struct data_t
        {
            byte msgClass = 0;
            byte msgID    = 0;
            byte rateTgt1 = 0;
            byte rateTgt2 = 0;
            byte rateTgt3 = 0;
            byte rateTgt4 = 0;
            byte rateTgt5 = 0;
            byte rateTgt6 = 0;
        } data;
        
        struct scale_t
        {
            double msgClass = 1.0;
            double msgID    = 1.0;
            double rateTgt1 = 1.0;
            double rateTgt2 = 1.0;
            double rateTgt3 = 1.0;
            double rateTgt4 = 1.0;
            double rateTgt5 = 1.0;
            double rateTgt6 = 1.0;
        } scale;
    };
    
private:
    SoftwareSerial* gpsIO;
    
    MSG_PACKET input_msg;
    MSG_PACKET output_msg;
    
    UBX_MSG_NAV_POSLLH    posLLH;
    UBX_MSG_NAV_VELNED    velNED;
    UBX_MSG_NAV_STATUS    navStatus;
    UBX_MSG_CFG_MSG_SHORT configMsgShort;
    UBX_MSG_CFG_MSG_LONG  configMsgLong;
    
    unsigned short length;
    bool checksum_valid;
    
    byte gpsbyte;
    byte buffer[UBX_BUFFER_MAX_SIZE];
    
    // functions
    void write(MSG_PACKET* output_msg);
    int decode(int msg_id, byte* buffer, unsigned short buffer_length);
    int update_gps_data(int msg_id, GpsType* gpsData);
    void clear_buffer();
};

#endif /* fs_gps_ubx_io_hpp */
