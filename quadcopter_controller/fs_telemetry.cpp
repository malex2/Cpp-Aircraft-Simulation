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

//#define TM_DIRECT_READ
//#define TM_DIRECT_WRITE

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

TM_Message_Info TM_MSGS[N_TM_MSGS];
void (*updateTM[N_TM_MSGS])(TM_Message_Info* pTMMSG) =
{updateIMUTM, updateBaroTM, updateGPSTM, updateNavHighrateTM, updateNavLowrateTM, updateControlTM, updateStatusTM, updatePrintTM};

void FsTelemetry_setupTelemetry(int baud_rate, float* msg_rates)
{
#ifdef TELEMETRY
    pPrintFIFO = get_print_fifo();
    
    temp_buffer_size = 0;
    memset(print_buffer_copy, 0, MAX_SERIAL_FIFO_SIZE);
    memset(temp_TM_buffer, 0, MAX_SERIAL_FIFO_SIZE);
    
    TMData.configState = doneConfiguringTM;
    configStateTime = 0.0;
    configStateSetup = false;
    
    checkPacketSizes(baud_rate, msg_rates);
    
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

void checkPacketSizes(int baud_rate, float* msg_rates)
{
    int bits_per_second = 0;
    int uart_protocal_bits = 2; // start/stop bit
    if (msg_rates)
    {
        for (unsigned int tm_id=0; tm_id<N_TM_MSGS; tm_id++)
        {
            int bytes_per_msg = 0;
            if (tm_id == FS_TM_IMU) { bytes_per_msg = sizeof(TM.imu_data); }
            else if (tm_id == FS_TM_BARO) { bytes_per_msg = sizeof(TM.baro_data); }
            else if (tm_id == FS_TM_GPS) { bytes_per_msg = sizeof(TM.gps_data); }
            else if (tm_id == FS_TM_NAV_HIGHRATE) { bytes_per_msg = sizeof(TM.nav_highrate_data); }
            else if (tm_id == FS_TM_NAV_LOWRATE) { bytes_per_msg = sizeof(TM.nav_lowrate_data); }
            else if (tm_id == FS_TM_CONTROLS) { bytes_per_msg = sizeof(TM.control_data); }
            else if (tm_id == FS_TM_STATUS) { bytes_per_msg = sizeof(TM.status_data); }
            
            int bits_per_msg = bytes_per_msg * 8 * msg_rates[tm_id];
            bits_per_second += bits_per_msg  + bits_per_msg * uart_protocal_bits;
        }
    }
    
    display("Serial Bitrate: ");
    display(baud_rate);
    display("\n");
    
    display("TM Configured bits/sec: ");
    display(bits_per_second);
    display("\n");
}

void FsTelemetry_sendTelemetry(TM_MSG_TYPE tm_msg_id)
{
#ifdef TELEMETRY
    if (TMData.tmState[tm_msg_id] == doneSendingTM)
    {
        TMData.tmState[tm_msg_id] = commandSend;
    }
    else if (TMData.configState != doneConfiguringTM)
    {
        TMData.tm_timeout_count++;
    }
#endif
}

void FsTelemetry_configureTelemetry()
{
#ifdef TELEMETRY
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
    
    if (telemetryFIFO.get_max_write_buffer_length() > TMData.tm_max_write_byte_count)
    { TMData.tm_max_write_byte_count = telemetryFIFO.get_max_write_buffer_length(); }
    
    APC220_configProcessing();
    telemetryProcessing(tmDt);
#endif
}

void APC220_configProcessing()
{
    if (TMData.configState == doneConfiguringTM) { return; }
    
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
#ifndef TM_DIRECT_WRITE
            telemetryFIFO.write(FsTelemetry_APC220_CONFIG, APC220_CONFIG_SIZE);
#else
            tmIO.write(FsTelemetry_APC220_CONFIG, APC220_CONFIG_SIZE);
#endif
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
            TM_config_rsp_count = 0;
            display("TM Config Set!\n");
        }
    }
}

void telemetryProcessing(double &tmDt)
{
    // Determine what telemetry is ready to send
    for (unsigned int tm_id = FS_TM_IMU; tm_id < N_TM_MSGS; tm_id++)
    {
        if (!pSendInfo && TMData.tmState[tm_id] == commandSend)
        {
            updateTM[tm_id](&TM_MSGS[tm_id]);
            TMData.tm_start_time[tm_id] = getTime();
            if (TM_MSGS[tm_id].size > 0)
            {
                pSendInfo = &TM_MSGS[tm_id];
                TMData.tmState[tm_id] = sendingTM;
            }
            else
            {
                finishSendingTelemetry(tm_id, tmDt);
            }
            break;
        }
    }
    
    // Send telemetry
    if (pSendInfo)
    {
        // Write buffer empty and ready for more data
        if (!telemetryFIFO.write_fifo_full())
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
        
        // Finish sending TM
        for (unsigned int tm_id = FS_TM_IMU; tm_id < N_TM_MSGS; tm_id++)
        {
            if (TMData.tmState[tm_id] == sendingTM && pSendInfo->done)
            {
                finishSendingTelemetry(tm_id, tmDt);
            }
        }
    } // pSendInfo
}

void finishSendingTelemetry(unsigned int tm_id, double &tmDt)
{
    pSendInfo = 0;
    TMData.tmState[tm_id] = doneSendingTM;
    TMData.tm_write_msg_count[tm_id]++;
    if (getTime() == TMData.tm_start_time[tm_id]) { TMData.max_tm_rate[tm_id] = 1.0/tmDt; }
    else { TMData.max_tm_rate[tm_id] = 1.0/(getTime() - TMData.tm_start_time[tm_id]); }
    
    if (tm_id == FS_PRINT)
    {
        memset(print_buffer_copy, 0, MAX_SERIAL_FIFO_SIZE);
    }
}

void updateIMUTM(TM_Message_Info* pTMMSG)
{
    TM.imu_data.fs_time = getTime() / TM.imu_scale.fs_time;
    TM.imu_data.imu_timestamp = pIMU->timestamp / TM.imu_scale.imu_timestamp;
    TM.imu_data.gyro_x = pIMU->gyro[0] / TM.imu_scale.gyro_x;
    TM.imu_data.gyro_y = pIMU->gyro[1] / TM.imu_scale.gyro_y;
    TM.imu_data.gyro_z = pIMU->gyro[2] / TM.imu_scale.gyro_z;
    TM.imu_data.accel_x = pIMU->accel[0] / TM.imu_scale.accel_x;
    TM.imu_data.accel_y = pIMU->accel[1] / TM.imu_scale.accel_y;
    TM.imu_data.accel_z = pIMU->accel[2] / TM.imu_scale.accel_z;
    
    pTMMSG->reset();
    pTMMSG->size = sizeof(TM.imu_data);
    pTMMSG->header = (byte) TM_IMU_HEADER << 8 | (byte) TM_HEADER;
    pTMMSG->buffer = (byte*) &TM.imu_data;
    pTMMSG->checksum = FsCommon_computeChecksum(pTMMSG->buffer, pTMMSG->size);
}

void updateBaroTM(TM_Message_Info* pTMMSG)
{
    TM.baro_data.baro_timestamp = pBaro->timestamp / TM.baro_scale.baro_timestamp;
    TM.baro_data.pressure = pBaro->pressure / TM.baro_scale.pressure;
    TM.baro_data.temperature = pBaro->temperature / TM.baro_scale.temperature;
    TM.baro_data.baro_altitude = pBaro->altitude / TM.baro_scale.baro_altitude;
    
    pTMMSG->reset();
    pTMMSG->size = sizeof(TM.baro_data);
    pTMMSG->header = (byte) TM_BARO_HEADER << 8 | (byte) TM_HEADER;
    pTMMSG->buffer = (byte*) &TM.baro_data;
    pTMMSG->checksum = FsCommon_computeChecksum(pTMMSG->buffer, pTMMSG->size);
}

void updateGPSTM(TM_Message_Info* pTMMSG)
{
    TM.gps_data.gps_timestamp = pGPS->timestamp/ TM.gps_scale.gps_timestamp;
    TM.gps_data.gps_receiver_time = pGPS->GPStimestamp / TM.gps_scale.gps_receiver_time;
    TM.gps_data.gps_ttff = pGPS->ttff / TM.gps_scale.gps_ttff;
    TM.gps_data.gps_lat = pGPS->posLLH[0] / TM.gps_scale.gps_lat;
    TM.gps_data.gps_lon = pGPS->posLLH[1] / TM.gps_scale.gps_lon;
    TM.gps_data.gps_alt = pGPS->posLLH[2] / TM.gps_scale.gps_alt;
    TM.gps_data.gps_pos_east = pGPS->posENU[0] / TM.gps_scale.gps_pos_east;
    TM.gps_data.gps_pos_north = pGPS->posENU[1] / TM.gps_scale.gps_pos_north;
    TM.gps_data.gps_pos_up = pGPS->posENU[2] / TM.gps_scale.gps_pos_up;
    TM.gps_data.gps_vel_n = pGPS->velNED[0] / TM.gps_scale.gps_vel_n;
    TM.gps_data.gps_vel_e = pGPS->velNED[1] / TM.gps_scale.gps_vel_e;
    TM.gps_data.gps_vel_d = pGPS->velNED[2] / TM.gps_scale.gps_vel_d;
    TM.gps_data.gps_horiz_pos_acc = pGPS->horizPosAcc/ TM.gps_scale.gps_horiz_pos_acc;
    TM.gps_data.gps_vert_pos_acc = pGPS->vertPosAcc / TM.gps_scale.gps_vert_pos_acc;
    TM.gps_data.gps_speed_acc = pGPS->speedAcc / TM.gps_scale.gps_speed_acc;
    TM.gps_data.gps_rcvd_msg_count = pGPS->rcvdMsgCount / TM.gps_scale.gps_rcvd_msg_count;
    TM.gps_data.gps_read_fifo_overflow_count = pGPS->fifoReadOverflowCount / TM.gps_scale.gps_read_fifo_overflow_count;
    
    pTMMSG->reset();
    pTMMSG->size = sizeof(TM.gps_data);
    pTMMSG->header = (byte) TM_GPS_HEADER << 8 | (byte) TM_HEADER;
    pTMMSG->buffer = (byte*) &TM.gps_data;
    pTMMSG->checksum = FsCommon_computeChecksum(pTMMSG->buffer, pTMMSG->size);
}

void updateNavHighrateTM(TM_Message_Info* pTMMSG)
{
    TM.nav_highrate_data.nav_timestamp = pNav->timestamp / TM.nav_highrate_scale.nav_timestamp;
    TM.nav_highrate_data.roll = pNav->eulerAngles[0] / TM.nav_highrate_scale.roll;
    TM.nav_highrate_data.pitch = pNav->eulerAngles[1] / TM.nav_highrate_scale.pitch;
    TM.nav_highrate_data.yaw = pNav->eulerAngles[2] / TM.nav_highrate_scale.yaw;
    TM.nav_highrate_data.nav_vel_n = pNav->velNED[0] / TM.nav_highrate_scale.nav_vel_n;
    TM.nav_highrate_data.nav_vel_e = pNav->velNED[1] / TM.nav_highrate_scale.nav_vel_e;
    TM.nav_highrate_data.nav_vel_d = pNav->velNED[2] / TM.nav_highrate_scale.nav_vel_d;
    TM.nav_highrate_data.nav_pos_n = pNav->position[0] / TM.nav_highrate_scale.nav_pos_n;
    TM.nav_highrate_data.nav_pos_e = pNav->position[1] / TM.nav_highrate_scale.nav_pos_e;
    TM.nav_highrate_data.nav_alt = pNav->position[2] / TM.nav_highrate_scale.nav_alt;
    TM.nav_highrate_data.accelerometer_pitch = pNav->accel_pitch / TM.nav_highrate_scale.accelerometer_pitch;
    TM.nav_highrate_data.accelerometer_roll = pNav->accel_roll / TM.nav_highrate_scale.accelerometer_roll;
    
    pTMMSG->reset();
    pTMMSG->size = sizeof(TM.nav_highrate_data);
    pTMMSG->header = (byte) TM_NAV_HIGHRATE_HEADER << 8 | (byte) TM_HEADER;
    pTMMSG->buffer = (byte*) &TM.nav_highrate_data;
    pTMMSG->checksum = FsCommon_computeChecksum(pTMMSG->buffer, pTMMSG->size);
}

void updateNavLowrateTM(TM_Message_Info* pTMMSG)
{
    TM.nav_lowrate_data.nav_timestamp = pNav->timestamp / TM.nav_lowrate_scale.nav_timestamp;
    TM.nav_lowrate_data.accel_bias_x = pNav->accBias[0]  / TM.nav_lowrate_scale.accel_bias_x;
    TM.nav_lowrate_data.accel_bias_y = pNav->accBias[1] / TM.nav_lowrate_scale.accel_bias_y;
    TM.nav_lowrate_data.accel_bias_z = pNav->accBias[2] / TM.nav_lowrate_scale.accel_bias_z;
    TM.nav_lowrate_data.gyro_bias_x = pNav->gyroBias[0] / TM.nav_lowrate_scale.gyro_bias_x;
    TM.nav_lowrate_data.gyro_bias_y = pNav->gyroBias[1] / TM.nav_lowrate_scale.gyro_bias_y;
    TM.nav_lowrate_data.gyro_bias_z = pNav->gyroBias[2] / TM.nav_lowrate_scale.gyro_bias_z;
    TM.nav_lowrate_data.gravity = pNav->gravity / TM.nav_lowrate_scale.gravity;
    TM.nav_lowrate_data.ins_update_count = pNav->updateCount[INS] / TM.nav_lowrate_scale.ins_update_count;
    TM.nav_lowrate_data.baro_filter_count = pNav->updateCount[BaroUpdate] / TM.nav_lowrate_scale.baro_filter_count;
    TM.nav_lowrate_data.gps_filter_count = pNav->updateCount[GPSUpdate]/ TM.nav_lowrate_scale.gps_filter_count;
    TM.nav_lowrate_data.accel_filter_count = pNav->updateCount[AccelUpdate] / TM.nav_lowrate_scale.accel_filter_count;
    
    pTMMSG->reset();
    pTMMSG->size = sizeof(TM.nav_lowrate_data);
    pTMMSG->header = (byte) TM_NAV_LOWRATE_HEADER << 8 | (byte) TM_HEADER;
    pTMMSG->buffer = (byte*) &TM.nav_lowrate_data;
    pTMMSG->checksum = FsCommon_computeChecksum(pTMMSG->buffer, pTMMSG->size);
}

void updateControlTM(TM_Message_Info* pTMMSG)
{
    TM.control_data.ctrl_timestamp = pCtrl->timestamp / TM.control_scale.ctrl_timestamp;
    TM.control_data.throttle_channel_pwm = pCtrl->pwmCmd[THROTTLE_CHANNEL] / TM.control_scale.throttle_channel_pwm;
    TM.control_data.roll_channel_pwm = pCtrl->pwmCmd[ROLL_CHANNEL]/ TM.control_scale.roll_channel_pwm;
    TM.control_data.pitch_channel_pwm = pCtrl->pwmCmd[PITCH_CHANNEL] / TM.control_scale.pitch_channel_pwm;
    TM.control_data.yaw_channel_pwm = pCtrl->pwmCmd[YAW_CHANNEL] / TM.control_scale.yaw_channel_pwm;
    TM.control_data.VLLxCmd = pCtrl->VLLxCmd / TM.control_scale.VLLxCmd;
    TM.control_data.VLLyCmd = pCtrl->VLLyCmd / TM.control_scale.VLLyCmd;
    TM.control_data.VLLzCmd = pCtrl->VLLzCmd / TM.control_scale.VLLzCmd;
    TM.control_data.rollCmd = pCtrl->rollCmd / TM.control_scale.rollCmd;
    TM.control_data.pitchCmd = pCtrl->pitchCmd / TM.control_scale.pitchCmd;
    TM.control_data.yawRateCmd = pCtrl->yawRateCmd / TM.control_scale.yawRateCmd;
    TM.control_data.prop1_pwm = pCtrl->TPWM[0] / TM.control_scale.prop1_pwm;
    TM.control_data.prop2_pwm = pCtrl->TPWM[1] / TM.control_scale.prop2_pwm;
    TM.control_data.prop3_pwm  = pCtrl->TPWM[2] / TM.control_scale.prop3_pwm;
    TM.control_data.prop4_pwm = pCtrl->TPWM[3] / TM.control_scale.prop4_pwm;
    
    pTMMSG->reset();
    pTMMSG->size = sizeof(TM.control_data);
    pTMMSG->header = (byte) TM_CONTROL_HEADER << 8 | (byte) TM_HEADER;
    pTMMSG->buffer = (byte*) &TM.control_data;
    pTMMSG->checksum = FsCommon_computeChecksum(pTMMSG->buffer, pTMMSG->size);
}

void updateStatusTM(TM_Message_Info* pTMMSG)
{
    TM.status_data.gps_fixOk = pGPS->fixOk / TM.status_scale.gps_fixOk;
    TM.status_data.numSV = pGPS->numSV / TM.status_scale.numSV;
    TM.status_data.baroI2CErrorCode = pBaro->errorCodeBaro/ TM.status_scale.baroI2CErrorCode;
    TM.status_data.imuI2CErrorCode = pIMU->errorCodeIMU / TM.status_scale.imuI2CErrorCode;
    TM.status_data.ctrl_mode = pCtrl->mode / TM.status_scale.ctrl_mode;
    TM.status_data.onGround = pCtrl->onGround / TM.status_scale.onGround;
    TM.status_data.takeOff = pCtrl->takeOff / TM.status_scale.takeOff;
    TM.status_data.crashLand = pCtrl->crashLand / TM.status_scale.crashLand;
    
    TM.status_data.max_tm_rate = TMData.max_tm_rate[FS_TM_IMU] / TM.status_scale.max_tm_rate;
    TM.status_data.tm_timeout_count = TMData.tm_timeout_count / TM.status_scale.tm_timeout_count;
    TM.status_data.hz1_avg_rate = (1.0/pFSTiming[hz1]) / TM.status_scale.hz1_avg_rate;
    TM.status_data.hz50_avg_rate = (1.0/pFSTiming[hz50]) / TM.status_scale.hz50_avg_rate;
    TM.status_data.hz100_avg_rate = (1.0/pFSTiming[hz100]) / TM.status_scale.hz100_avg_rate;
    TM.status_data.hz200_avg_rate = (1.0/pFSTiming[hz200]) / TM.status_scale.hz200_avg_rate;
    TM.status_data.hz600_avg_rate = (1.0/pFSTiming[hz600]) / TM.status_scale.hz600_avg_rate;
    
    pTMMSG->reset();
    pTMMSG->size = sizeof(TM.status_data);
    pTMMSG->header = (byte) TM_STATUS_HEADER << 8 | (byte) TM_HEADER;
    pTMMSG->buffer = (byte*) &TM.status_data;
    pTMMSG->checksum = FsCommon_computeChecksum(pTMMSG->buffer, pTMMSG->size);
}

void updatePrintTM(TM_Message_Info* pTMMSG)
{
    pTMMSG->reset();
    pTMMSG->size = pPrintFIFO->available();
    int i=0; while(pPrintFIFO->available()) { print_buffer_copy[i] = pPrintFIFO->read(); i++; }
    pTMMSG->header = (byte) TM_PRINT_HEADER << 8 | (byte) TM_HEADER;
    pTMMSG->buffer = print_buffer_copy;
    pTMMSG->checksum = FsCommon_computeChecksum(pTMMSG->buffer, pTMMSG->size);
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
    #ifdef TM_DIRECT_READ
    static int TM_config_rsp_count = 0;
    while( tmIO.available() )
    {
        Serial.write(tmIO.read());
        TM_config_rsp_count++;
    }
    
    if (TM_config_rsp_count == 21)
    {
        APC220_SetRunningMode();
        configStateSetup = false;
        TMData.configState = doneConfiguringTM;
        TM_config_rsp_count = 0;
        display("tmIO) TM Config Set!\n");
    }
    else
    {
        TM_config_rsp_count = 0;
    }
    #endif
    
    telemetryFIFO.update_fifo();
#endif
}

void FsTelemetry_resetMaxWriteCounter()
{
    TMData.tm_max_write_byte_count = 0;
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
