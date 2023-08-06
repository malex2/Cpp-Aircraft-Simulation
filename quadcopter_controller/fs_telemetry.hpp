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
            display("\n");
        }
    }
};

struct TMType {
    unsigned long tm_timeout_count;
    unsigned long tm_failed_write_count;
    unsigned long tm_write_fifo_full_count;
    unsigned long tm_write_msg_count[N_TM_MSGS];
    unsigned long tm_write_byte_count;
    unsigned long tm_max_write_byte_count;
    unsigned long tm_rcv_msg_count;
    unsigned long tm_rcv_byte_count;
    double tm_start_time[N_TM_MSGS];
    double max_tm_rate[N_TM_MSGS];
    telemetryStateType tmState[N_TM_MSGS];
    configStateType configState;
    
    TMType() {
        tm_timeout_count = 0;
        tm_failed_write_count = 0;
        tm_write_fifo_full_count = 0;
        tm_rcv_msg_count = 0;
        tm_write_byte_count = 0;
        tm_max_write_byte_count = 0;
        tm_rcv_byte_count = 0;
        configState = doneConfiguringTM;
        
        for (unsigned int tm_id = FS_TM_IMU; tm_id < N_TM_MSGS; tm_id++)
        {
            tm_write_msg_count[tm_id] = 0;
            tm_start_time[tm_id] = 0.0;
            max_tm_rate[tm_id] = 0.0;
            tmState[tm_id] = doneSendingTM;
        }
    }
};

void FsTelemetry_setupTelemetry(int baud_rate, float* msg_rates = NULL);
void FsTelemetry_sendTelemetry(TM_MSG_TYPE tm_msg_id);
void FsTelemetry_configureTelemetry();
void FsTelemetry_performTelemetry(double &tmDt);
void FsTelemetry_performSerialIO();

void checkPacketSizes(int baud_rate, float* msg_rates);
void APC220_configProcessing();
void telemetryProcessing(double &tmDt);
void finishSendingTelemetry(unsigned int tm_msg_id, double &tmDt);

void updateIMUTM(TM_Message_Info* pTMMSG);
void updateBaroTM(TM_Message_Info* pTMMSG);
void updateGPSTM(TM_Message_Info* pTMMSG);
void updateNavHighrateTM(TM_Message_Info* pTMMSG);
void updateNavLowrateTM(TM_Message_Info* pTMMSG);
void updateControlTM(TM_Message_Info* pTMMSG);
void updateStatusTM(TM_Message_Info* pTMMSG);
void updatePrintTM(TM_Message_Info* pTMMSG);
void APC220_UpdateConfig();
void APC220_SetAwake();
void APC220_SetSleep();
void APC220_SetSettingMode();
void APC220_SetRunningMode();

void FsTelemetry_resetMaxWriteCounter();
void FsTelemetry_setTelemetryData(const IMUtype* pIMUIn, const BarometerType* pBaroIn, const GpsType* pGPSIn, const NavType* pNavIn, const ControlType* pCtrlIn);
void FsTelemetry_setTimingPointer(const double* pTiming);
const TMType* FsTelemetry_getTMdata();

#ifdef SIMULATION
   void FsTelemetry_setSimulationModels(ModelMap* pMap);
#endif
#endif /* fs_telemetry_hpp */
