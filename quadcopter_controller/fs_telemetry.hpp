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

enum telemetryStateType {commandSend, sendingTM, doneSendingTM};

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

void FsTelemetry_setupTelemetry(int baud_rate);
void FsTelemetry_performTelemetry(double &tmDt);
void FsTelemetry_performSerialIO();

void FsTelemetry_sendTelemetry();
void finishSendingTelemetry(double &tmDt);

void FsTelemetry_setTelemetryData(IMUtype* pIMUIn, BarometerType* pBaroIn, GpsType* pGPSIn, NavType* pNavIn, ControlType* pCtrlIn);
void FsTelemetry_setTimingPointer(double* pTiming);

#ifdef SIMULATION
   void FsTelemetry_setSimulationModels(ModelMap* pMap);
#endif
#endif /* fs_telemetry_hpp */
