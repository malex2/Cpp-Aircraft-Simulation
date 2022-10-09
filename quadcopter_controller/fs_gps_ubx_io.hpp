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
#include "fs_gps_ubx_message_types.h"

class UBX_MSG {
public:
    UBX_MSG(FS_FIFO* gpsFIFO);
    ~UBX_MSG();
    
    enum UBX_IO_STATE {READY, HEADER1, HEADER2, MSG_TYPE, MSG_LENGTH, UPDATE_MSG, CALC_CHECKSUM, COMPLETE, INVALID};
    
    void write_nmea_off();
    void write_nmea_off_long();
    void write_ubx_off();
    void write_ubx_on();
    
    void set_nav_config();
    void get_nav_config();
    
    void init_aid_request();
    void request_aid_data();
    
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
        MSG_PACKET(bool debug = false) { debug_print = debug; clear(); }
        ~MSG_PACKET() { clear(); }
        
        TWO_BYTE_DATA  msg_class_id;
        TWO_BYTE_DATA  msg_length;
        TWO_BYTE_DATA  msg_checksum;
        byte           buffer[UBX_BUFFER_MAX_SIZE];
        unsigned short buffer_index;
        unsigned short buffer_length;
        bool           checksum_valid;
        int            msg_class;
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
        
        void set_buffer(byte* input, int length);
        void set_buffer(void* input, int length);
        void print();
        void clear();
    };
private:
    FS_FIFO* gpsFIFO;
    
    MSG_PACKET input_msg;
    MSG_PACKET output_msg;
    
    UBX_MSG_TYPES::UBX_MSG_NAV_POSLLH      posLLH;
    UBX_MSG_TYPES::UBX_MSG_NAV_VELNED      velNED;
    UBX_MSG_TYPES::UBX_MSG_NAV_STATUS      navStatus;
    UBX_MSG_TYPES::UBX_MSG_NAV_DOP         navDOP;
    UBX_MSG_TYPES::UBX_MSG_NAV_SOL         navSol;
    UBX_MSG_TYPES::UBX_MSG_CFG_MSG_SHORT   configMsgShort;
    UBX_MSG_TYPES::UBX_MSG_CFG_MSG_LONG    configMsgLong;
    UBX_MSG_TYPES::UBX_MSG_CFG_NAV5        configNav5;
    UBX_MSG_TYPES:: UBX_MSG_ACK            ack;
    UBX_MSG_TYPES::UBX_MSG_ACK             nak;
    UBX_MSG_TYPES::UBX_MSG_AID_INI         aidInit;
    UBX_MSG_TYPES::UBX_MSG_AID_ALM_POLL_SV almanacPollSv;
    UBX_MSG_TYPES::UBX_MSG_AID_ALM_INVALID almanacInvalid;
    UBX_MSG_TYPES::UBX_MSG_AID_ALM         almanac;
    UBX_MSG_TYPES::UBX_MSG_AID_EPH_POLL_SV ephemerisPollSv;
    UBX_MSG_TYPES::UBX_MSG_AID_EPH_INVALID ephemerisInvalid;
    UBX_MSG_TYPES::UBX_MSG_AID_EPH         ephemeris;
    UBX_MSG_TYPES::UBX_MSG_AID_HUI         gpsHealth;
    
    byte gpsbyte;
    
    // functions
    void write(MSG_PACKET* output_msg);
    int decode(int msg_class, int msg_id, byte* buffer, unsigned short buffer_length);
    int update_gps_data(int msg_class, int msg_id, GpsType* gpsData);
    void clear_input_buffer();
};

#endif /* fs_gps_ubx_io_hpp */
