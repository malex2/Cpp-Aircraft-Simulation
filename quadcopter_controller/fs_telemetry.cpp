//
//  fs_telemetry.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 12/30/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#include "fs_telemetry.hpp"
#include "fs_imu.hpp"
#include "fs_barometer.hpp"
#include "fs_gps.hpp"
#include "fs_navigation.hpp"
#include "fs_controls.hpp"

const IMUtype* pIMU = 0;
const BarometerType* pBaro = 0;
const GpsType* pGPS = 0;
const NavType* pNav = 0;
const ControlType* pCtrl = 0;
const double* pFSTiming = 0;

TMType TMData;
FS_Telemetry_Type TM;
FS_FIFO* pPrintFIFO = 0;
#define tmIO Serial2 // [RX,TX] = [7,8]
FS_FIFO telemetryFIFO(&tmIO);

double configStateTime;
bool configStateSetup;

byte temp_TM_buffer[MAX_SERIAL_FIFO_SIZE];
unsigned int temp_buffer_size;
TM_Message_Info TM_info;
TM_Message_Info print_info;
TM_Message_Info* pSendInfo = 0;
byte print_buffer_copy[MAX_SERIAL_FIFO_SIZE];

void FsTelemetry_setupTelemetry(int baud_rate)
{
#ifdef TELEMETRY
    pPrintFIFO = get_print_fifo();
    
    temp_buffer_size = 0;
    memset(print_buffer_copy, 0, MAX_SERIAL_FIFO_SIZE);
    memset(temp_TM_buffer, 0, MAX_SERIAL_FIFO_SIZE);
    
    TMData.tmState = doneSendingTM;
    TMData.configState = doneConfiguringTM;
    
    configStateTime = 0.0;
    configStateSetup = false;
    
#ifndef SIMULATION
    pinMode(TM_ENPIN, OUTPUT);
    pinMode(TM_SETPIN, OUTPUT);
#endif
    APC220_SetAwake();
    APC220_SetRunningMode();
    
    // Begin telemetry port
    telemetryFIFO.begin(baud_rate);
#endif
}

void FsTelemetry_sendTelemetry()
{
#ifdef TELEMETRY
    if (TMData.tmState == doneSendingTM)
    {
        display("Start TM: ");
        display(getTime());
        display("\n");
        TMData.tmState = commandSend;
    }
    else if (TMData.tmState != configureTM)
    {
        TMData.tm_timeout_count++;
    }
#endif
}

void FsTelemetry_configureTelemetry()
{
#ifdef TELEMETRY
    TMData.tmState = configureTM;
    TMData.configState = commandConfig;
    display("Configure telemetry commanded\n");
#endif
}

void FsTelemetry_performTelemetry(double &tmDt)
{
#ifdef TELEMETRY
    if (telemetryFIFO.get_write_buffer_overflow_count() > 0)
    {
        display("TM write overflow: ");
        display(telemetryFIFO.get_write_buffer_overflow_count());
        display("\n");
    }
    TMData.tm_write_byte_count = telemetryFIFO.get_write_count();
    TMData.tm_rcv_byte_count = telemetryFIFO.get_read_count();
    
    APC220_configProcessing();
    telemetryProcessing(tmDt);
#endif
}

void APC220_configProcessing()
{
    if (TMData.tmState != configureTM) { return; }
    
    if (TMData.configState == commandConfig)
    {
        if (!configStateSetup)
        {
            display("Preparing TM Config\n");
            APC220_SetSettingMode();
            configStateTime = getTime();
            configStateSetup = true;
        }
        
        if (getTime() - configStateTime > APC220_SETTING_DELAY)
        {
            configStateSetup = false;
            TMData.configState = configuringTM;
        }
    }
    
    if (TMData.configState == configuringTM)
    {
        
        if (!configStateSetup)
        {
            display("Sending TM Config\n");
            Serial.write(FsTelemetry_APC220_CONFIG, APC220_CONFIG_SIZE);
            telemetryFIFO.write(FsTelemetry_APC220_CONFIG, APC220_CONFIG_SIZE);
            configStateTime = getTime();
            configStateSetup = true;
        }
        
        static int TM_config_rsp_count = 0;
        if (telemetryFIFO.available())
        {
            Serial.write(telemetryFIFO.read());
            TM_config_rsp_count++;
        }

        if (TM_config_rsp_count == 21 || getTime() - configStateTime > (double) APC220_SETTING_TIMEOUT)
        {
            APC220_SetRunningMode();
            configStateSetup = false;
            TMData.configState = doneConfiguringTM;
            TMData.tmState = doneSendingTM;
            TM_config_rsp_count = 0;
            display("TM Config Set!\n");
        }
    }
}

void telemetryProcessing(double &tmDt)
{
    if (TMData.tmState == commandSend)
    {
        //display("Command Send\n");
        // Set telemetry
        TM.data.fs_time = getTime() / TM.scale.fs_time;
        
        TM.data.imu_timestamp = pIMU->timestamp / TM.scale.imu_timestamp;
        TM.data.gyro_x = pIMU->gyro[0] / TM.scale.gyro_x;
        TM.data.gyro_y = pIMU->gyro[1] / TM.scale.gyro_y;
        TM.data.gyro_z = pIMU->gyro[2] / TM.scale.gyro_z;
        TM.data.accel_x = pIMU->accel[0] / TM.scale.accel_x;
        TM.data.accel_y = pIMU->accel[1] / TM.scale.accel_y;
        TM.data.accel_z = pIMU->accel[2] / TM.scale.accel_z;
        
        TM.data.baro_timestamp = pBaro->timestamp / TM.scale.baro_timestamp;
        TM.data.pressure = pBaro->pressure / TM.scale.pressure;
        TM.data.temperature = pBaro->temperature / TM.scale.temperature;
        TM.data.baro_altitude = pBaro->altitude / TM.scale.baro_altitude;
        
        TM.data.gps_timestamp = pGPS->timestamp/ TM.scale.gps_timestamp;
        TM.data.gps_receiver_time = pGPS->GPStimestamp / TM.scale.gps_receiver_time;
        TM.data.gps_ttff = pGPS->ttff / TM.scale.gps_ttff;
        TM.data.gps_lat = pGPS->posLLH[0] / TM.scale.gps_lat;
        TM.data.gps_lon = pGPS->posLLH[1] / TM.scale.gps_lon;
        TM.data.gps_alt = pGPS->posLLH[2] / TM.scale.gps_alt;
        TM.data.gps_pos_east = pGPS->posENU[0] / TM.scale.gps_pos_east;
        TM.data.gps_pos_north = pGPS->posENU[1] / TM.scale.gps_pos_north;
        TM.data.gps_pos_up = pGPS->posENU[2] / TM.scale.gps_pos_up;
        TM.data.gps_vel_n = pGPS->velNED[0] / TM.scale.gps_vel_n;
        TM.data.gps_vel_e = pGPS->velNED[1] / TM.scale.gps_vel_e;
        TM.data.gps_vel_d = pGPS->velNED[2] / TM.scale.gps_vel_d;
        TM.data.gps_horiz_pos_acc = pGPS->horizPosAcc/ TM.scale.gps_horiz_pos_acc;
        TM.data.gps_vert_pos_acc = pGPS->vertPosAcc / TM.scale.gps_vert_pos_acc;
        TM.data.gps_speed_acc = pGPS->speedAcc / TM.scale.gps_speed_acc;
        TM.data.gps_rcvd_msg_count = pGPS->rcvdMsgCount / TM.scale.gps_rcvd_msg_count;
        TM.data.gps_read_fifo_overflow_count = pGPS->fifoReadOverflowCount / TM.scale.gps_read_fifo_overflow_count;
        
        TM.data.nav_timestamp = pNav->timestamp / TM.scale.nav_timestamp;
        TM.data.roll = pNav->eulerAngles[0] / TM.scale.roll;
        TM.data.pitch = pNav->eulerAngles[1] / TM.scale.pitch;
        TM.data.yaw = pNav->eulerAngles[2] / TM.scale.yaw;
        TM.data.nav_vel_n = pNav->velNED[0] / TM.scale.nav_vel_n;
        TM.data.nav_vel_e = pNav->velNED[1] / TM.scale.nav_vel_e;
        TM.data.nav_vel_d = pNav->velNED[2] / TM.scale.nav_vel_d;
        TM.data.nav_pos_n = pNav->position[0] / TM.scale.nav_pos_n;
        TM.data.nav_pos_e = pNav->position[1] / TM.scale.nav_pos_e;
        TM.data.nav_alt = pNav->position[2] / TM.scale.nav_alt;
        TM.data.accel_bias_x = pNav->accBias[0]  / TM.scale.accel_bias_x;
        TM.data.accel_bias_y = pNav->accBias[1] / TM.scale.accel_bias_y;
        TM.data.accel_bias_z = pNav->accBias[2] / TM.scale.accel_bias_z;
        TM.data.gyro_bias_x = pNav->gyroBias[0] / TM.scale.gyro_bias_x;
        TM.data.gyro_bias_y = pNav->gyroBias[1] / TM.scale.gyro_bias_y;
        TM.data.gyro_bias_z = pNav->gyroBias[2] / TM.scale.gyro_bias_z;
        TM.data.gravity = pNav->gravity / TM.scale.gravity;
        TM.data.accelerometer_pitch = pNav->accel_pitch / TM.scale.accelerometer_pitch;
        TM.data.accelerometer_roll = pNav->accel_roll / TM.scale.accelerometer_roll;
        TM.data.ins_update_count = pNav->updateCount[INS] / TM.scale.ins_update_count;
        TM.data.baro_filter_count = pNav->updateCount[BaroUpdate] / TM.scale.baro_filter_count;
        TM.data.gps_filter_count = pNav->updateCount[GPSUpdate]/ TM.scale.gps_filter_count;
        TM.data.accel_filter_count = pNav->updateCount[AccelUpdate] / TM.scale.accel_filter_count;
        
        TM.data.ctrl_timestamp = pCtrl->timestamp / TM.scale.ctrl_timestamp;
        TM.data.throttle_channel_pwm = pCtrl->pwmCmd[THROTTLE_CHANNEL] / TM.scale.throttle_channel_pwm;
        TM.data.roll_channel_pwm = pCtrl->pwmCmd[ROLL_CHANNEL]/ TM.scale.roll_channel_pwm;
        TM.data.pitch_channel_pwm = pCtrl->pwmCmd[PITCH_CHANNEL] / TM.scale.pitch_channel_pwm;
        TM.data.yaw_channel_pwm = pCtrl->pwmCmd[YAW_CHANNEL] / TM.scale.yaw_channel_pwm;
        TM.data.VLLxCmd = pCtrl->VLLxCmd / TM.scale.VLLxCmd;
        TM.data.VLLyCmd = pCtrl->VLLyCmd / TM.scale.VLLyCmd;
        TM.data.VLLzCmd = pCtrl->VLLzCmd / TM.scale.VLLzCmd;
        TM.data.rollCmd = pCtrl->rollCmd / TM.scale.rollCmd;
        TM.data.pitchCmd = pCtrl->pitchCmd / TM.scale.pitchCmd;
        TM.data.yawRateCmd = pCtrl->yawRateCmd / TM.scale.yawRateCmd;
        TM.data.prop1_pwm = pCtrl->TPWM[0] / TM.scale.prop1_pwm;
        TM.data.prop2_pwm = pCtrl->TPWM[1] / TM.scale.prop2_pwm;
        TM.data.prop3_pwm  = pCtrl->TPWM[2] / TM.scale.prop3_pwm;
        TM.data.prop4_pwm = pCtrl->TPWM[3] / TM.scale.prop4_pwm;
        
        TM.data.gps_fixOk = pGPS->fixOk / TM.scale.gps_fixOk;
        TM.data.numSV = pGPS->numSV / TM.scale.numSV;
        TM.data.baroI2CErrorCode = pBaro->errorCodeBaro/ TM.scale.baroI2CErrorCode;
        TM.data.imuI2CErrorCode = pIMU->errorCodeIMU / TM.scale.imuI2CErrorCode;
        TM.data.ctrl_mode = pCtrl->mode / TM.scale.ctrl_mode;
        TM.data.onGround = pCtrl->onGround / TM.scale.onGround;
        TM.data.takeOff = pCtrl->takeOff / TM.scale.takeOff;
        TM.data.crashLand = pCtrl->crashLand / TM.scale.crashLand;
        
        TM.data.max_tm_rate = TMData.max_tm_rate / TM.scale.max_tm_rate;
        TM.data.tm_timeout_count = TMData.tm_timeout_count / TM.scale.tm_timeout_count;
        TM.data.hz1_avg_rate = (1.0/pFSTiming[hz1]) / TM.scale.hz1_avg_rate;
        TM.data.hz50_avg_rate = (1.0/pFSTiming[hz50]) / TM.scale.hz50_avg_rate;
        TM.data.hz100_avg_rate = (1.0/pFSTiming[hz100]) / TM.scale.hz100_avg_rate;
        TM.data.hz200_avg_rate = (1.0/pFSTiming[hz200]) / TM.scale.hz200_avg_rate;
        TM.data.hz800_avg_rate = (1.0/pFSTiming[hz800]) / TM.scale.hz800_avg_rate;

        // Prepare for sending telemetry
        TM_info.reset();
        TM_info.size = sizeof(TM.data);
        TM_info.header = (byte) TM_TM_HEADER << 8 | (byte) TM_HEADER;
        TM_info.buffer = (byte*) &TM.data;
        TM_info.checksum = FsCommon_computeChecksum(TM_info.buffer, TM_info.size);
        
        print_info.reset();
        print_info.size = pPrintFIFO->available();
        int i=0; while(pPrintFIFO->available()) { print_buffer_copy[i] = pPrintFIFO->read(); i++; }
        print_info.header = (byte) TM_PRINT_HEADER << 8 | (byte) TM_HEADER;
        print_info.buffer = print_buffer_copy;
        print_info.checksum = FsCommon_computeChecksum(print_info.buffer, print_info.size);
        
        memset(temp_TM_buffer, 0, MAX_SERIAL_FIFO_SIZE);
        pSendInfo = &TM_info;
        
        TMData.tm_start_time = getTime();
        TMData.tmState = sendingTM;
    }
    
    if (TMData.tmState == sendingTM)
    {
        // Write buffer empty and ready for more data
        if (pSendInfo && !telemetryFIFO.write_fifo_full())
        {
            // Determine how much telemetry needs to be sent
            unsigned short header_tail_size = 0;
            unsigned int available_fifo_size = MAX_SERIAL_FIFO_SIZE - telemetryFIFO.get_write_buffer_length();
            temp_buffer_size = pSendInfo->size - pSendInfo->sent_bytes;
            if (temp_buffer_size > available_fifo_size) { temp_buffer_size = available_fifo_size; }
            
            // Send header
            if (!pSendInfo->header_sent)
            {
                //pSendInfo->print();
                header_tail_size = FS_TM_HEADER_SIZE + FS_TM_LENGTH_SIZE;
                if (temp_buffer_size + header_tail_size > available_fifo_size) { temp_buffer_size = available_fifo_size - header_tail_size; }
                
                memcpy(temp_TM_buffer, &pSendInfo->header, FS_TM_HEADER_SIZE);
                memcpy(temp_TM_buffer+FS_TM_HEADER_SIZE, &pSendInfo->size, FS_TM_LENGTH_SIZE);
                memcpy(temp_TM_buffer+FS_TM_HEADER_SIZE+FS_TM_LENGTH_SIZE, pSendInfo->buffer, temp_buffer_size);
                
                pSendInfo->sent_bytes += temp_buffer_size;
                pSendInfo->header_sent = true;
            }
            // Send TM
            else if (pSendInfo->sent_bytes < pSendInfo->size)
            {
                memcpy(temp_TM_buffer, pSendInfo->buffer+pSendInfo->sent_bytes, temp_buffer_size);
                pSendInfo->sent_bytes += temp_buffer_size;
            }
            // Send checksum
            if (pSendInfo->sent_bytes == pSendInfo->size && available_fifo_size-temp_buffer_size-header_tail_size >= FS_TM_CHECKSUM_SIZE)
            {
                memcpy(temp_TM_buffer+header_tail_size+temp_buffer_size, &pSendInfo->checksum, FS_TM_CHECKSUM_SIZE);
                header_tail_size += FS_TM_CHECKSUM_SIZE;
                pSendInfo->done = true;
            }
            
            // Check sending
            unsigned int sent_bytes = telemetryFIFO.write(temp_TM_buffer, temp_buffer_size+header_tail_size);
            if(sent_bytes != temp_buffer_size+header_tail_size)
            {
                TMData.tm_failed_write_count++;
            }
            memset(temp_TM_buffer, 0, MAX_SERIAL_FIFO_SIZE);
        }
        else
        {
            TMData.tm_write_fifo_full_count++;
        }
        // Manually manage what telemetry to send
        if (TM_info.done && !print_info.done)
        {
            if (print_info.size > 0) { pSendInfo = &print_info; }
            else { finishSendingTelemetry(tmDt); }
        }
        else if (print_info.done) { finishSendingTelemetry(tmDt); }
    }
}

void finishSendingTelemetry(double &tmDt)
{
    pSendInfo = 0;
    TMData.tmState = doneSendingTM;
    TMData.tm_write_msg_count++;
    if (getTime() == TMData.tm_start_time) { TMData.max_tm_rate = 1.0/tmDt; }
    else { TMData.max_tm_rate = 1.0/(getTime() - TMData.tm_start_time); }
    
    display("Start/End/Rate TM: ");
    display(TMData.tm_start_time);
    display("/");
    display(getTime());
    display("/");
    display(TMData.max_tm_rate);
    display("\n");
}

void APC220_SetAwake()
{
#ifndef SIMULATION
    digitalWrite(TM_ENPIN, HIGH);
#endif
}

void APC220_SetSleep()
{
#ifndef SIMULATION
    digitalWrite(TM_ENPIN, LOW);
#endif
}

void APC220_SetSettingMode()
{
#ifndef SIMULATION
    digitalWrite(TM_SETPIN, LOW);
#endif
}

void APC220_SetRunningMode()
{
#ifndef SIMULATION
    digitalWrite(TM_SETPIN, HIGH);
#endif
}

void FsTelemetry_performSerialIO()
{
#ifdef TELEMETRY
    telemetryFIFO.update_fifo();
#endif
}

void FsTelemetry_setTelemetryData(const IMUtype* pIMUIn, const BarometerType* pBaroIn, const GpsType* pGPSIn, const NavType* pNavIn, const ControlType* pCtrlIn)
{
    pIMU  = pIMUIn;
    pBaro = pBaroIn;
    pGPS  = pGPSIn;
    pNav  = pNavIn;
    pCtrl = pCtrlIn;
}

void FsTelemetry_setTimingPointer(const double* pTiming)
{
    pFSTiming = pTiming;
}

const TMType* FsTelemetry_getTMdata()
{
    return &TMData;
}

#ifdef SIMULATION
void FsTelemetry_setSimulationModels(ModelMap* pMap)
{
    if (pMap)
    {
        tmIO.serial_setTwoHardwareSerialModels(pMap, "FS_GS_Interface", "APC220Radio");
    }
}
#endif
