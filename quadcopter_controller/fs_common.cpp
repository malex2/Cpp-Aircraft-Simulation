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
    ArduinoEEPROM EEPROM;
#endif

// TWO_BYTE_DATA //
void TWO_BYTE_DATA::swap()
{
    byte prev_data[2];
    memcpy(prev_data, data, 2);
    data[0] = prev_data[1];
    data[1] = prev_data[0];
#ifdef DEBUG_PRINT
    if (debug_print)
    {
        display("TWO_BYTE_DATA::swap(), prev_data: ");
        display(+prev_data[0], HEX);
        display(+prev_data[1], HEX);
        display(+data[0], HEX);
        display(+data[1], HEX);
        display("\n");
    }
#endif
}

void TWO_BYTE_DATA::clear()
{
    memset(data, 0, 2);
    index = 0;
#ifdef DEBUG_PRINT
    if (debug_print)
    {
        display("TWO_BYTE_DATA::clear(), data: ");
        display(+data[0], HEX);
        display(+data[1], HEX);
        display("\n");
    }
#endif
}

uint16_t TWO_BYTE_DATA::value()
{
    uint16_t val;
    memcpy(&val, data, 2);
    return val;
}

FS_FIFO::FS_FIFO()
{
    serialIO = nullptr;
    nrf24L01 = nullptr;
    baud_begin = false;
    read_byte_count  = 0.0;
    write_byte_count = 0.0;
    prevWriteTime = 0.0;
    baud_dt       = 0.0;
    max_read_buffer_length    = 0;
    read_fifo_overflow_count  = 0;
    write_fifo_overflow_count = 0;
    write_buffer_overflow_count = 0;
    max_write_buffer_length     = 0;
    reset_read_buffer();
    reset_write_buffer();
    memset(temp_nrf24L01_buffer, 0, NRF24L01_BUFFER_SIZE);
}

FS_FIFO::FS_FIFO(FS_Serial* serialIO)
{
    this->serialIO  = serialIO;
    nrf24L01 = nullptr;
    baud_begin = false;
    read_byte_count  = 0.0;
    write_byte_count = 0.0;
    prevWriteTime = 0.0;
    baud_dt       = 0.0;
    max_read_buffer_length    = 0;
    read_fifo_overflow_count  = 0;
    write_fifo_overflow_count = 0;
    write_buffer_overflow_count = 0;
    max_write_buffer_length     = 0;
    reset_read_buffer();
    reset_write_buffer();
    memset(temp_nrf24L01_buffer, 0, NRF24L01_BUFFER_SIZE);
}

FS_FIFO::FS_FIFO(RF24* tmIO)
{
    serialIO = nullptr;
    nrf24L01 = tmIO;
    baud_begin = false;
    read_byte_count  = 0.0;
    write_byte_count = 0.0;
    prevWriteTime = 0.0;
    baud_dt       = 0.0;
    max_read_buffer_length    = 0;
    read_fifo_overflow_count  = 0;
    write_fifo_overflow_count = 0;
    write_buffer_overflow_count = 0;
    max_write_buffer_length     = 0;
    reset_read_buffer();
    reset_write_buffer();
    memset(temp_nrf24L01_buffer, 0, NRF24L01_BUFFER_SIZE);
}

FS_FIFO::~FS_FIFO()
{
    end();
}

void FS_FIFO::begin(uint32_t baud_rate)
{
    if (serialIO != nullptr)
    {
        serialIO->begin(baud_rate);
    }
    if (nrf24L01 != nullptr)
    {
        nrf24L01->begin();
    }
    this->baud_dt = 1.0/baud_rate;
    prevWriteTime = getTime();
    baud_begin = true;
}

void FS_FIFO::end()
{
    reset_read_buffer();
    reset_write_buffer();
    memset(temp_nrf24L01_buffer, 0, NRF24L01_BUFFER_SIZE);
    baud_dt = 0.0;
    baud_begin = false;
    if (serialIO) { serialIO->end(); }
}

void FS_FIFO::update_fifo()
{
    if (!baud_begin) { return; }
    
    if (baud_dt == 0.0)
    {
        // Copy write buffer into read buffer
        if (write_available() && !read_fifo_full())
        {
            memcpy(read_buffer+read_buffer_length, write_buffer, write_buffer_length);
            read_buffer_length += write_buffer_length;
            reset_write_buffer();
        }
    }

    else if (serialIO != nullptr)
    {
        // Read from SerialIO
        while (serialIO->available() && !read_fifo_full())
        {
            read_buffer[read_buffer_length] = serialIO->read();
            read_buffer_length++;
            read_byte_count++;
            if (read_buffer_length > max_read_buffer_length) { max_read_buffer_length = read_buffer_length; }
            
            // Detect buffer overflow
            if (read_fifo_full()) { read_fifo_overflow_count++; }
        }
    
        // Write to SerialIO
        //if (write_available() &&  getTime() - prevWriteTime >= baud_dt)
        while (write_available() && serialIO->availableForWrite())
        {
            int n_write = serialIO->write(write_buffer[write_buffer_index]);
            write_buffer_index++;
            write_byte_count++;
            prevWriteTime = getTime();
        
            if (n_write < 1) { write_buffer_overflow_count++; }
  
            // Reset write buffer once all data written
            if (!write_available()) { reset_write_buffer(); }
        }
    }
    
    else if (nrf24L01 != nullptr)
    {
        // Read from NRF24L01
        if (nrf24L01->available() && !read_fifo_full())
        {
            nrf24L01->read(read_buffer+read_buffer_length, (unsigned int) NRF24L01_BUFFER_SIZE);
            read_buffer_length += (unsigned int) NRF24L01_BUFFER_SIZE;
            read_byte_count += (unsigned int) NRF24L01_BUFFER_SIZE;
        }

        // Write to NRF24L01
        if (write_available() &&  getTime() - prevWriteTime >= baud_dt)
        {
            unsigned int write_length = write_buffer_length - write_buffer_index;
            if (write_length > NRF24L01_BUFFER_SIZE) { write_length = NRF24L01_BUFFER_SIZE; }
            memcpy(temp_nrf24L01_buffer, write_buffer+write_buffer_index, write_length);
            
            bool write_ok = nrf24L01->write(temp_nrf24L01_buffer, (unsigned int) NRF24L01_BUFFER_SIZE);
            if (write_ok)
            {
                write_buffer_index += (unsigned int) write_length;
                write_byte_count += (unsigned int) write_length;
                prevWriteTime = getTime();
            }
            else
            {
                write_buffer_overflow_count++;
            }

            memset(temp_nrf24L01_buffer,0,NRF24L01_BUFFER_SIZE);
            // Reset write buffer once all data written
            if (!write_available()) { reset_write_buffer(); }
        }
    }
}

// Read from SerilaIO
byte FS_FIFO::read()
{
    if (available())
    {
        //std::cout << "FS_FIFO read(): read_buffer[" << read_buffer_index << "] = ";
        //std::cout << std::hex << std::setfill('0') << std::setw(2) << +read_val;
        //std::cout << std::dec << std::endl;
        
        read_val = read_buffer[read_buffer_index];
        read_buffer_index++;
        
        // Reset read buffer once read complete
        if (!available()) { reset_read_buffer(); }
    }
    else
    {
        read_val = 0;
    }
    
    return read_val;
}

unsigned short FS_FIFO::available()
{
    return (read_buffer_length - read_buffer_index);
}

bool FS_FIFO::read_fifo_full()
{
    if (serialIO != nullptr)
    {
        return (read_buffer_length >= (int) (MAX_SERIAL_FIFO_SIZE));
    }
    else if (nrf24L01 != nullptr)
    {
        return (read_buffer_length + (int) NRF24L01_BUFFER_SIZE > (int) MAX_SERIAL_FIFO_SIZE);
    }
    else
    {
        return (read_buffer_length + write_buffer_length > (int) MAX_SERIAL_FIFO_SIZE);
    }
    return false;
}

// Write to SerialIO
unsigned short FS_FIFO::write(byte write_val)
{
    if (!write_fifo_full())
    {
        //std::cout << "FS_FIFO write(): write_buffer[" << write_buffer_length << "] = ";
        //std::cout << std::hex << std::setfill('0') << std::setw(2) << +write_val;
        //std::cout << std::dec << std::endl;
        
        write_buffer[write_buffer_length] = write_val;
        write_buffer_length++;
        if (write_buffer_length > max_write_buffer_length) { max_write_buffer_length = write_buffer_length; }
        return 1;
    }
    else
    {
        write_fifo_overflow_count++;
        return 0;
    }
}

unsigned short FS_FIFO::write(const byte* write_val, unsigned long length)
{
    unsigned short tx_count = 0;
    for (int i = 0; i < length; i++)
    {
        tx_count += write(write_val[i]);
    }
    return tx_count;
}

bool FS_FIFO::write_available()
{
    /*
    if (serialIO != nullptr)
    {
        std::cout << "write_available() serialIO = [" << write_buffer_index << ", " << write_buffer_length << "]" << std::endl;
    }
    else if (nrf24L01 != nullptr)
    {
        std::cout << "write_available() nrf24L01 = [" << write_buffer_index << ", " << write_buffer_length << "]" << std::endl;
    }
    else
    {
        std::cout << "write_available() = [" << write_buffer_index << ", " << write_buffer_length << "]" << std::endl;
    }
     */
    return (write_buffer_index < write_buffer_length);
}

bool FS_FIFO::write_fifo_full()
{
    return (write_buffer_length >= (int) (MAX_SERIAL_FIFO_SIZE));
}

void FS_FIFO::reset_read_buffer()
{
    memset(read_buffer, 0, MAX_SERIAL_FIFO_SIZE);
    read_buffer_index  = 0;
    read_buffer_length = 0;
}

void FS_FIFO::reset_write_buffer()
{
    memset(write_buffer, 0, MAX_SERIAL_FIFO_SIZE);
    write_buffer_index  = 0;
    write_buffer_length = 0;
}

void FS_FIFO::display_read_buffer()
{
#ifdef DEBUG_PRINT
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
#endif
}

void FS_FIFO::display_write_buffer()
{
#ifdef DEBUG_PRINT
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
#endif
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

// Memory
void write_eeprom(unsigned int address, byte val)
{
    EEPROM.write(address, val);
}

void write_eeprom(unsigned int address, byte* val, unsigned int size)
{
    for (unsigned int i = 0; i < size; i++)
    {
        write_eeprom(address+i, val[i]);
    }
}

void read_eeprom(unsigned int address, byte* val)
{
    *val = EEPROM.read(address);
}

void read_eeprom(unsigned int address, byte* val, unsigned int size)
{
    for (unsigned int i = 0; i < size; i++)
    {
        read_eeprom(address+i, val+i);
    }
}

// Errors
double errorToVariance(double maxError)
{
    double std;
    double variance;
    
    // max error is 3 standard deviations
    std = maxError / 3.0;
    
    // variance = std^2
    variance = std*std;
    
    return variance;
}

double varianceToError(double variance)
{
    double std;
    double error;
    
    if (variance<0.0) { variance = -1.0*variance; }
    std = sqrt(variance);
    
    // max error is 3 std
    error = 3.0*std;
    
    return error;
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
FS_FIFO FS_print_fifo;
void display(const char* val, int printMode)
{
#ifdef TM_PRINT
    FS_print_fifo.write((byte*) val, sizeof_char(val));
#else
#ifdef SIMULATION
    std::cout << val;
#else
    Serial.print(val);
#endif
#endif
}

template<typename TempType>
void display(TempType val, int printMode)
{
#ifdef TM_PRINT
    String str;
   // Turn into character array
   #ifdef SIMULATION
    if (printMode == HEX) { str = std::to_string((byte) val); }
    else { str = std::to_string(val); }
   #else
    if (printMode == HEX) {}
    else { str = String((byte) val,6); }
   #endif
    FS_print_fifo.write((byte*) str.c_str(), str.length());
#else
#ifdef SIMULATION
    if (printMode == HEX)
    {
        std::cout << val;
        //std::cout << std::hex << std::setfill('0') << std::setw(2) << val;
        //std::cout << std::dec;
    }
    else
    {
        std::cout << val;
    }
#else
    Serial.print(val, printMode);
#endif
#endif
}

void display(I2C_Error_Code val, int printMode)
{
#ifdef TELEMETRY
    FS_print_fifo.write((byte*) &val, sizeof(val));
#else
    if (val == I2C_0_SUCCESS)
    {
        display("0 - success.");
    }
    else if (val == I2C_1_DATA_TOO_LONG)
    {
        display("1 - data too long to fit in transmit buffer.");
    }
    else if (val == I2C_2_NACK_ADDRESS)
    {
        display("2 - received NACK on transmit of data.");
    }
    else if (val == I2C_3_NACK_DATA)
    {
        display("3 - received NACK on transmit of data.");
    }
    else if (val == I2C_4_OTHER)
    {
        display("4 - other error.");
    }
    else if (val == I2C_5_TIMEOUT)
    {
        display("5 - timeout.");
    }
    else
    {
        display("unkown error.");
    }
#endif
}

template void display(char, int printMode);
template void display(short, int printMode);
template void display(unsigned short, int printMode);
template void display(int, int printMode);
template void display(unsigned int, int printMode);
template void display(long, int printMode);
template void display(unsigned long, int printMode);
template void display(double, int printMode);
template void display(byte, int printMode);

FS_FIFO* get_print_fifo()
{
    return &FS_print_fifo;
}

unsigned int sizeof_char(const char* str)
{
    unsigned int max_length = 1000;
    unsigned int size = 0;
    while(size < max_length && str[size] != '\0')
    {
        size++;
    }

    if (size == max_length) { size = 0; }
    return size;
}

void FsCommon_performSerialIO()
{
#ifdef TELEMETRY
    FS_print_fifo.update_fifo();
#endif
}

#ifdef SIMULATION
void FsCommon_setSimulationModels(ModelMap* pMap)
{
    if (pMap) { pTime = (Time*) pMap->getModel("Time"); }
}
#endif

uint16_t FsCommon_computeChecksum(const void* buffer, unsigned int len)
{
    // fletcher16 algorithm
    // Found by solving for c1 overflow:
    // n > 0 and n * (n+1) / 2 * (2^8-1) < (2^32-1).
    
    byte* data = (byte*) buffer;
    uint32_t c0, c1;
    for (c0 = c1 = 0; len > 0; ) {
        size_t blocklen = len;
        if (blocklen > 5802) {
            blocklen = 5802;
        }
        len -= blocklen;
        do {
            c0 = c0 + *data++;
            c1 = c1 + c0;
        } while (--blocklen);
        c0 = c0 % 255;
        c1 = c1 % 255;
    }
    return (c1 << 8 | c0);
}

bool FsCommon_checkChecksum(uint16_t buffer_checksum, const void* data, unsigned int len)
{
    uint16_t check_checksum = FsCommon_computeChecksum(data, len);
    return (check_checksum == buffer_checksum);
}
