//
//  telemetry_monitor_model.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 1/8/23.
//  Copyright © 2023 Alexander McLean. All rights reserved.
//

#include "telemetry_monitor_model.hpp"
#include "arduino_class_models.hpp"
#include "time.hpp"

TelemetryMonitorBase::TelemetryMonitorBase(ModelMap *pMapInit, bool debugFlagIn)
{
    pTime   = NULL;
    pMap = pMapInit;
    
    //pMap->addLogVar("xForce", &bodyForce[0], printSavePlot, 3);
    
    debugFlag = debugFlagIn;
    
    sendCount = 0;
    sendByteCount = 0;
    sendTime = 0.0;
    receiveCount = 0;
    receiveByteCount = 0;
    receiveTime = 0.0;
    readFailCount = 0;
    
    tm_io_type = NONE;
    pTMIO = 0;
}
TelemetryMonitorBase::~TelemetryMonitorBase()
{
    if (pTMIO) { delete pTMIO; }
}

void TelemetryMonitorBase::initialize(void)
{
    pTime = (Time*) pMap->getModel("Time");
}

bool TelemetryMonitorBase::update(void)
{
    if (pSerialIO)
    {
        read_serialIO();
        process_inputs();
        process_outputs();
        write_serialIO();
    }
    
    return true;
}

void TelemetryMonitorBase::read_serialIO()
{
    if (!pSerialIO) { return; }
    
    if (pSerialIO->getBaudRate() != 0 && !pTMIO)
    {
        if (tm_io_type == BLUETOOTH_HC05)
        {
            pTMIO = (SimulationSerial*) new BluetoothHC05Serial(pSerialIO->getTXPin(), pSerialIO->getRXPin());
        }
        else
        {
            pTMIO = new SimulationSerial(pSerialIO->getTXPin(), pSerialIO->getRXPin());
        }
        pTMIO->begin( pSerialIO->getBaudRate() );
    }
    
    // Take pSerialIO write buffer and transfer to read buffer
    if (pSerialIO && pTMIO)
    {
        int n_attempt = pSerialIO->slave_available();
        int n_write = 0;
        while (pSerialIO->slave_available())
        {
            n_write += pTMIO->slave_write(pSerialIO->slave_read());
        }
        receiveByteCount += n_write;
        if (n_write < n_attempt)
        {
            std::cout << pTime->getSimTime() << "TelemetryMonitorBase::read_serialIO() - SerialIO failed to write to TMIO!!" << std::endl;
        }
    }
}

void TelemetryMonitorBase::process_inputs() {}
void TelemetryMonitorBase::process_outputs() {}

void TelemetryMonitorBase::write_serialIO()
{
    // Take pTMIO wiret buffer and transfer it to pSerialIO read buffer
    if (pSerialIO && pTMIO)
    {
        int n_write = 0;
        if (pTMIO->slave_available() && pTime->getSimTime() > sendTime+1.0/pTMIO->getBaudRate())
        {
            //pTMIO->display_tx_buffer();
            sendTime = pTime->getSimTime();
            n_write += pSerialIO->slave_write(pTMIO->slave_read());
            
            //std::cout << "[TMIO, SerialIO] = [" << pTMIO->slave_available()  << ", " << pSerialIO->available() << "]" << std::endl;
            if (pTMIO->slave_available() && n_write < 1)
            {
                std::cout << pTime->getSimTime() << ") TelemetryMonitorBase::write_serialIO() - TMIO failed to write to SerialIO!!" << std::endl;
            }
        }
        sendByteCount += n_write;
    }
}

const unsigned short QuadcopterTelemetry::TM_HEADERS[N_TM_MSGS] =
{TM_IMU_HEADER, TM_BARO_HEADER, TM_GPS_HEADER, TM_NAV_HIGHRATE_HEADER, TM_NAV_LOWRATE_HEADER, TM_CONTROL_HEADER, TM_STATUS_HEADER, TM_PRINT_HEADER};

QuadcopterTelemetry::QuadcopterTelemetry(ModelMap *pMapInit, bool debugFlagIn)  : TelemetryMonitorBase(pMapInit, debugFlagIn)
{
    
}

void QuadcopterTelemetry::initialize()
{
    tm_io_type = BLUETOOTH_HC05;
    Base::initialize();
    TM.enable_all_printing();
}

void QuadcopterTelemetry::process_inputs()
{
    if (!pTMIO) { return; }
    
    while (pTMIO->available() || read_buffer.state==SerialBuffer::READ_COMPLETE)
    {
        if (read_buffer.state != SerialBuffer::READ_COMPLETE)
        { read_byte = pTMIO->read(); }
        
        switch (read_buffer.state)
        {
            case SerialBuffer::HEADER1 :
                if (read_byte == (byte) TM_HEADER)
                {
                    read_buffer.state = SerialBuffer::HEADER2;
                }
                break;
                
            case SerialBuffer::HEADER2 :
                if (set_msg_type(read_byte))
                {
                    read_buffer.state = SerialBuffer::READ_LENGTH;
                }
                else
                {
                    clear_read_buffers();
                    readFailCount++;
                }
                break;
                
            case SerialBuffer::READ_LENGTH :
                tm_buffer_length.data[tm_buffer_length.index++] = read_byte;
                if (tm_buffer_length.index >= 2)
                {
                    read_buffer.state = SerialBuffer::READ_BUFFER;
                    read_buffer.new_buffer(tm_buffer_length.value());
                }
                break;
                
            case SerialBuffer::READ_BUFFER :
                read_buffer.set_buffer(read_byte);
                if (read_buffer.buffer_full())
                {
                    read_buffer.state = SerialBuffer::CALC_CHECKSUM;
                }
                
                break;
                
            case SerialBuffer::CALC_CHECKSUM :
                tm_checksum.data[tm_checksum.index++] = read_byte;
                if (tm_checksum.index >= 2)
                {
                    if ( FsCommon_checkChecksum(tm_checksum.value(), read_buffer.get_buffer(), tm_buffer_length.value()) )
                    {
                        read_buffer.state = SerialBuffer::READ_COMPLETE;
                    }
                    else
                    {
                        clear_read_buffers();
                        readFailCount++;
                    }
                }
                
                break;
                
            case SerialBuffer::READ_COMPLETE :
                if (FS_msg_type == FS_PRINT)
                {
                    print_to_screen(read_buffer.get_buffer(), read_buffer.buffer_size());
                }
                else
                {
                    update_TM_data();
                }
                
                clear_read_buffers();
                break;
            default :
                // do nothing
                break;
        }
    }
}

bool QuadcopterTelemetry::set_msg_type(byte read_byte)
{
    for (unsigned int tm_id = FS_TM_IMU; tm_id < N_TM_MSGS; tm_id++)
    {
        if (read_byte == (byte) TM_HEADERS[tm_id])
        {
            FS_msg_type = (TM_MSG_TYPE) tm_id;
            return true;
        }
    }
    
    return false;
}

void QuadcopterTelemetry::update_TM_data()
{
    bool valid = false;
    
    if(FS_msg_type == FS_TM_IMU && tm_buffer_length.value() == sizeof(TM.imu_data))
    { memcpy(&TM.imu_data, read_buffer.get_buffer(), tm_buffer_length.value()); valid = true; }
    
    else if (FS_msg_type == FS_TM_BARO && tm_buffer_length.value() == sizeof(TM.baro_data))
    { memcpy(&TM.baro_data, read_buffer.get_buffer(), tm_buffer_length.value()); valid = true; }
    
    else if (FS_msg_type == FS_TM_GPS && tm_buffer_length.value() == sizeof(TM.gps_data))
    { memcpy(&TM.gps_data, read_buffer.get_buffer(), tm_buffer_length.value()); valid = true; }
    
    else if (FS_msg_type == FS_TM_NAV_HIGHRATE && tm_buffer_length.value() == sizeof(TM.nav_highrate_data))
    { memcpy(&TM.nav_highrate_data, read_buffer.get_buffer(), tm_buffer_length.value()); valid = true; }
    
    else if (FS_msg_type == FS_TM_NAV_LOWRATE && tm_buffer_length.value() == sizeof(TM.nav_lowrate_data))
    { memcpy(&TM.nav_lowrate_data, read_buffer.get_buffer(), tm_buffer_length.value()); valid = true; }
    
    else if (FS_msg_type == FS_TM_CONTROLS && tm_buffer_length.value() == sizeof(TM.control_data))
    { memcpy(&TM.control_data, read_buffer.get_buffer(), tm_buffer_length.value()); valid = true; }
    
    else if (FS_msg_type == FS_TM_STATUS && tm_buffer_length.value() == sizeof(TM.status_data))
    { memcpy(&TM.status_data, read_buffer.get_buffer(), tm_buffer_length.value()); valid = true; }
    
    if (valid)
    {
        TM.disable_all_printing();
        TM.enable_printing(FS_msg_type);
        TM.print();
    }
}

void QuadcopterTelemetry::process_outputs()
{
    if (pTMIO && (pTime->getSimTime() - sendTime) > 0.5 &&  pTime->getSimTime() < 5)
    {
        pTMIO->write(0x46);
        pTMIO->write(0x53);
    }
}

void QuadcopterTelemetry::print_to_screen(const byte* buffer, unsigned int length)
{
    for (int i=0; i<length; i++)
    {
        std::cout << buffer[i];
    }
}

void QuadcopterTelemetry::clear_read_buffers()
{
    tm_buffer_length.clear();
    tm_checksum.clear();
    read_buffer.clear();
}
