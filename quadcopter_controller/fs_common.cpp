//
//  fs_common.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 2/26/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "fs_common.hpp"

#ifdef SIMULATION
    #include "time.hpp"
#endif

// Classes
#ifdef SIMULATION
    class Time* pTime = 0;
#endif

FS_FIFO::FS_FIFO(HardwareSerial* serialIO)
{
    this->serialIO   = serialIO;
    read_byte_count  = 0.0;
    write_byte_count = 0.0;
    prevWriteTime = 0.0;
    baud_rate     = 0.0;
    reset_read_buffer();
    reset_write_buffer();
}

FS_FIFO::~FS_FIFO()
{
    serialIO = 0;
    reset_read_buffer();
    reset_write_buffer();
}

void FS_FIFO::begin(int baud_rate)
{
    if (serialIO)
    {
        serialIO->begin(baud_rate);
        this->baud_rate = 1.0/baud_rate;
        prevWriteTime = getTime();
    }
}

void FS_FIFO::update_fifo()
{
    if (!serialIO || baud_rate==0.0) { return ;}
    
    // Read from SerialIO
    while (serialIO->available())
    {
        if (!read_fifo_full())
        {
            read_buffer[read_buffer_length] = serialIO->read();
            read_buffer_length++;
            read_byte_count++;
        }
    }
    
    // Write to SerialIO
    if (write_available() &&  getTime() - prevWriteTime >= baud_rate)
    {
        serialIO->write(write_buffer[write_buffer_index]);
        write_buffer_index++;
        prevWriteTime = getTime();
        if (!write_available()) { reset_write_buffer(); }
        write_byte_count++;
    }
    
}

// Read from SerilaIO
byte FS_FIFO::read()
{
    if (available())
    {
        read_val = read_buffer[read_buffer_index];
        read_buffer_index++;
        if (!available()) { reset_read_buffer(); }
    }
    else
    {
        read_val = 0x00;
    }
    
    return read_val;
}

int FS_FIFO::available()
{
    return (read_buffer_length - read_buffer_index);
}

bool FS_FIFO::read_fifo_full()
{
    return (read_buffer_length >= (int) (MAX_SERIAL_FIFO_SIZE));
}

// Write to SerialIO
int FS_FIFO::write(byte write_val)
{
    if (!write_fifo_full())
    {
        write_buffer[write_buffer_length] = write_val;
        write_buffer_length++;
        return 1;
    }
    else
    {
        return 0;
    }
}

int FS_FIFO::write(byte* write_val, int length)
{
    int tx_count = 0;
    for (int i = 0; i < length; i++)
    {
        tx_count += write(write_val[i]);
    }
    return tx_count;
}

int FS_FIFO::write_available()
{
    return (write_buffer_length - write_buffer_index);
}

bool FS_FIFO::write_fifo_full()
{
    return (write_buffer_length >= (int) (MAX_SERIAL_FIFO_SIZE));
}

void FS_FIFO::reset_read_buffer()
{
    memset(read_buffer, 0x00, MAX_SERIAL_FIFO_SIZE);
    read_buffer_index  = 0;
    read_buffer_length = 0;
}

void FS_FIFO::reset_write_buffer()
{
    memset(write_buffer, 0x00, MAX_SERIAL_FIFO_SIZE);
    write_buffer_index  = 0;
    write_buffer_length = 0;
}

void FS_FIFO::display_read_buffer()
{
    display("FS_FIFO::read_buffer [");
    display(read_buffer_index);
    display("-");
    display(read_buffer_length);
    display("]: ");
    for (int i = read_buffer_index; i < read_buffer_length; i++)
    {
        display(+read_buffer[i], HEX);
    }
    display("\n");
}

void FS_FIFO::display_write_buffer()
{
    display("FS_FIFO::write_buffer [");
    display(write_buffer_index);
    display("-");
    display(write_buffer_length);
    display("]: ");
    for (int i = write_buffer_index; i < write_buffer_length; i++)
    {
        display(+write_buffer[i], HEX);
    }
    display("\n");
}


// Time
double getTime()
{
#ifdef SIMULATION
    if (pTime) { return pTime->getSimTime(); }
    else { return 0.0; }
#else
    return micros() / 1000000.0;
#endif
}

#ifdef SIMULATION
void delay(int ms_delay)
{
    // do nothing
}
#endif

// Errors
double errorToVariance(double maxError)
{
    double std;
    double variance;
    
    // max error is 3 standard deviations
    std = maxError / 3.0;
    // variance is std^2
    variance = std*std;
    
    return variance;
}

void crossProduct(double *cross, double *a, double *b)
{
    *(cross+0) = a[1]*b[2] - a[2]*b[1];
    *(cross+1) = a[2]*b[0] - a[0]*b[2];
    *(cross+2) = a[0]*b[1] - a[1]*b[0];
}

// LED
void LEDon()
{
#ifndef SIMULATION
    digitalWrite(LEDPIN, HIGH);
#endif
}
void LEDoff()
{
#ifndef SIMULATION
    digitalWrite(LEDPIN, LOW);
#endif
}

// Printing
template<typename TempType>
void display(TempType val)
{
#ifdef SIMULATION
    std::cout << val;
#else
    Serial.print(val);
#endif
}

template<typename TempType>
void display(TempType val, int printMode)
{
#ifdef SIMULATION
    if (printMode == HEX)
    {
        std::cout << std::hex << std::setfill('0') << std::setw(2) << val;
        std::cout << std::dec;
    }
    else
    {
        display(val);
    }
#else
    Serial.print(val, printMode);
#endif
}

void display(I2C_Error_Code val)
{
    if (val == I2C_0_SUCCESS)
    {
        display("0 - success.\n");
    }
    else if (val == I2C_1_DATA_TOO_LONG)
    {
        display("1 - data too long to fit in transmit buffer.\n");
    }
    else if (val == I2C_2_NACK_ADDRESS)
    {
        display("2 - received NACK on transmit of data.\n");
    }
    else if (val == I2C_3_NACK_DATA)
    {
        display("3 - received NACK on transmit of data.\n");
    }
    else if (val == I2C_4_OTHER)
    {
        display("4 - other error.\n");
    }
    else if (val == I2C_5_TIMEOUT)
    {
        display("5 - timeout.\n");
    }
    else
    {
        display("unkown error.\n");
    }
}

template void display(const char*);
template void display(char);
template void display(String);
template void display(short);
template void display(unsigned short);
template void display(int);
template void display(unsigned int);
template void display(long);
template void display(unsigned long);
template void display(double);
template void display(byte);

template void display(short, int printMode);
template void display(unsigned short, int printMode);
template void display(int, int printMode);
template void display(unsigned int, int printMode);
template void display(long, int printMode);
template void display(unsigned long, int printMode);
template void display(double, int printMode);
template void display(byte, int printMode);

#ifdef SIMULATION
void FsCommon_setSimulationModels(ModelMap* pMap)
{
    if (pMap) { pTime = (Time*) pMap->getModel("Time"); }
}
#endif
