//
//  telemetry_monitor_model.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 1/8/23.
//  Copyright © 2023 Alexander McLean. All rights reserved.
//

#ifndef telemetry_monitor_model_hpp
#define telemetry_monitor_model_hpp

#include "generic_model.hpp"
#include "fs_common.hpp"

class TelemetryMonitorBase : public GenericSensorModel {
    
public:
    // Constructor
    TelemetryMonitorBase(ModelMap *pMapInit, bool debugFlagIn = false);
    ~TelemetryMonitorBase();
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update rotations
    virtual bool update(void);
    
    // Data Types
    enum Telemetry_IO_Type {NONE, BLUETOOTH_HC05};
    
protected:
    class Time* pTime;
    
    void read_serialIO();
    virtual void process_inputs();
    virtual void process_outputs();
    void write_serialIO();
    
    SimulationSerial* pTMIO;
    Telemetry_IO_Type tm_io_type;
    SerialBuffer      read_buffer;
    byte              read_byte;
    
    int           sendCount;
    unsigned long sendByteCount;
    double        sendTime;
    int           receiveCount;
    unsigned long receiveByteCount;
    double        receiveTime;
    unsigned long readFailCount;
};

class QuadcopterTelemetry : public TelemetryMonitorBase {
public:
    QuadcopterTelemetry(ModelMap *pMapInit, bool debugFlagIn = false);
    virtual void initialize();
    
private:
    typedef TelemetryMonitorBase Base;
    static const unsigned short TM_HEADERS[N_TM_MSGS];
    
    virtual void process_inputs();
    virtual void process_outputs();
    bool set_msg_type(byte read_byte);
    void update_TM_data();
    void print_to_screen(const byte* buffer, unsigned int length);
    void clear_read_buffers();
    
    FS_Telemetry_Type TM;
    TM_MSG_TYPE FS_msg_type;
    TWO_BYTE_DATA tm_buffer_length;
    TWO_BYTE_DATA tm_checksum;
};
#endif /* telemetry_monitor_model_hpp */
