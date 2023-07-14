//
//  fs_telemetry.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 12/30/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#ifndef fs_telemetry_hpp
#define fs_telemetry_hpp

#include "fs_common.hpp"

enum telemetryStateType {commandSend, sendingTM, doneSendingTM, configureTM};
enum configStateType    {commandConfig, configuringTM, doneConfiguringTM};

struct TM_Message_Info {
    TM_Message_Info() { reset(); }
    
    uint16_t header;
    uint16_t checksum;
    uint16_t size;
    uint16_t sent_bytes;
    
    bool header_sent;
    bool done;
    
    byte* buffer;
    
    void reset()
    {
        header = 0;
        checksum = 0;
        size = 0;
        sent_bytes = 0;
        buffer = 0;
        
        header_sent = false;
        done = false;
    }
    
    void print()
    {
        if (buffer)
        {
            display(+header);
            display(+size);
            for (int i=0; i<size; i++)
            {
                display(+buffer[i]);
            }
            display(+checksum);
        }
    }
};

struct TMType {
    unsigned long tm_timeout_count;
    unsigned long tm_failed_write_count;
    unsigned long tm_write_fifo_full_count;
    unsigned long tm_write_msg_count;
    unsigned long tm_write_byte_count;
    unsigned long tm_rcv_msg_count;
    unsigned long tm_rcv_byte_count;
    double tm_start_time;
    double max_tm_rate;
    telemetryStateType tmState;
    configStateType configState;
    
    TMType() {
        tm_timeout_count = 0;
        tm_failed_write_count = 0;
        tm_write_fifo_full_count = 0;
        tm_write_msg_count = 0;
        tm_rcv_msg_count = 0;
        tm_write_byte_count = 0;
        tm_rcv_byte_count = 0;
        tm_start_time = 0.0;
        max_tm_rate = 0.0;
        tmState = doneSendingTM;
        configState = doneConfiguringTM;
    }
};

void FsTelemetry_setupTelemetry(int baud_rate);
void FsTelemetry_sendTelemetry();
void FsTelemetry_configureTelemetry();
void FsTelemetry_performTelemetry(double &tmDt);
void FsTelemetry_performSerialIO();

void APC220_configProcessing();
void telemetryProcessing(double &tmDt);
void finishSendingTelemetry(double &tmDt);

void APC220_SetAwake();
void APC220_SetSleep();
void APC220_SetSettingMode();
void APC220_SetRunningMode();

void FsTelemetry_setTelemetryData(const IMUtype* pIMUIn, const BarometerType* pBaroIn, const GpsType* pGPSIn, const NavType* pNavIn, const ControlType* pCtrlIn);
void FsTelemetry_setTimingPointer(const double* pTiming);
const TMType* FsTelemetry_getTMdata();

#ifdef SIMULATION
   void FsTelemetry_setSimulationModels(ModelMap* pMap);
#endif
#endif /* fs_telemetry_hpp */
