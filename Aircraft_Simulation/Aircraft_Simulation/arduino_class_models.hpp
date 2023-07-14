//
//  arduino_class_models.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 2/26/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#ifndef arduino_class_models_hpp
#define arduino_class_models_hpp

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "model_mapping.hpp"
#include "barometer_model.hpp"

enum pinMode {INPUT, OUTPUT};
enum PrintMode {BIN, OCT, DEC, HEX};
const unsigned int LEDPIN = 0;

class SimulationWire {
public:
    SimulationWire();
    
    void begin();
    void beginTransmission(int deviceIn);
    
    void write(int info);
    short read();
    void requestFrom(int address, int numBytes, bool request);
    int available();
    int endTransmission(bool restart = true);
    
    
    void wire_setSimulationModels(ModelMap* pMap, bool print_wire = false);
private:
    class IMUModelBase*       pIMU;
    class AtmosphereModel*    pAtmo;
    class BarometerModelBase* pBaro;
    
    bool active;
    
    // attached I2C devices
    enum deviceType {MPU6050, BMP180, nDevices};
    int devices[nDevices];
    bool deviceAwake[nDevices];
    bool foundDevice;
    deviceType device;
    
    int address;
    bool addressLatch;
    
    // buffer
    byte buffer[100];
    int bufferSize;
    int iBuffer;
    void clearBuffer();
    
    //MPU6050 addresses
    static const int MPU_ADDR   = 0x68;
    static const int PWR_MGMT_1 = 0x6B; // PWR_MGMT_1 register
    static const int GYRO_REG   = 0x1B; // Gryo register
    static const int ACC_REG    = 0x1C; // Accelerometer register
    static const int ACC_OUT    = 0x3B; // (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    static const int INT        = 0x37;
    static const int INT_ENABLE = 0x38;
    
    double MPU6050_gyroByteToLSB(int byte);
    double MPU6050_accByteToLSB(int byte);
    
    //BMP180 addresses
    static const int BMP180_ADDR                = 0x77;
    static const int BMP180_REG_CONTROL         = 0xF4;
    static const int BMP180_REG_RESULT          = 0xF6;
    static const int BMP180_COMMAND_TEMPERATURE = 0x2E;
    static const int BMP180_COMMAND_PRESSURE0   = 0x34;
    static const int BMP180_COMMAND_PRESSURE1   = 0x74;
    static const int BMP180_COMMAND_PRESSURE2   = 0xB4;
    static const int BMP180_COMMAND_PRESSURE3   = 0xF4;
    static const int AC1_ADDR = 0xAA;
    static const int AC2_ADDR = 0xAC;
    static const int AC3_ADDR = 0xAE;
    static const int AC4_ADDR = 0xB0;
    static const int AC5_ADDR = 0xB2;
    static const int AC6_ADDR = 0xB4;
    static const int VB1_ADDR = 0xB6;
    static const int VB2_ADDR = 0xB8;
    static const int MB_ADDR  = 0xBA;
    static const int MC_ADDR  = 0xBC;
    static const int MD_ADDR  = 0xBE;
    
    bmp180::requestStateType bmp180LastRequest;
    
    void BMP180_request2bytes(bmp180::calibrationType calVal, int numBytes);
    
    bool print;
};

class Servo {
public:
    Servo();
    
    void attach(int pinIn);
    
    void writeMicroseconds(unsigned long pwm);
    
    double read();
    
    void servo_setSimulationModels(ModelMap* pMap);
private:
    class ActuatorModelBase* pAct;
    
    // Types
    enum motorNumberType {T1, T2, T3, T4, nMotors};
    
    // Pins
    const unsigned int T1PIN = 6;
    const unsigned int T2PIN = 9;
    const unsigned int T3PIN = 10;
    const unsigned int T4PIN = 11;
    const int PWMMIN = 1000;
    const int PWMMAX = 2000;
    
    // Variables
    unsigned int motorPins[nMotors];
    motorNumberType motorNumber;
    bool foundMotor;
    double throttle;
    
    // Internal Functions
    double mapToValue(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax, double valueMin, double valueMax);
};

class SimulationSerial {
public:
    SimulationSerial(int rx_pin=0, int tx_pin=0);
    ~SimulationSerial();
    
    void begin(int baud);
    byte read();
    int write(byte val);
    int write(const byte* val, int length);
    int available();
    int availableForWrite();
    
    template<typename TempType>
    void print(TempType val, PrintMode printMode = DEC);
    template<typename TempType>
    void println(TempType val, PrintMode printMode = DEC);
    
    // slave read/write
    byte slave_read();
    int slave_write(byte val);
    int slave_write(const byte* val, int length);
    int slave_available();
    int slave_availableForWrite();
    
    // simulation support
    int getRXPin()    { return rx_pin; }
    int getTXPin()    { return tx_pin; }
    int getBaudRate() { return baud_rate; }
    void display_rx_buffer();
    void display_tx_buffer();
    
    void serial_setSimulationModels(ModelMap* pMap, std::string model, bool print = false);
    void serial_setTwoHardwareSerialModels(ModelMap* pMap, std::string model, std::string serialType, bool print = false);
protected:
    void reset_rx_buffer();
    void reset_tx_buffer();
    
    static const int max_arduino_buffer_size = 64;
    static const int max_teensy3_series_buffer_size = 8;
    static const int max_teensy4_series_buffer_size = 4;
    static const int max_gps_neo6m_buffer_size = 720;
    static const int max_HC05Bluetooth_buffer_size = 100;
    static const int max_APC220Radio_buffer_size = 256;
    
    std::string serial_type = "unknown";
    int max_buffer_size = max_arduino_buffer_size;
    int baud_rate;
    bool baud_begin;
    byte* rx_buffer;
    int rx_buffer_index;
    int rx_buffer_length;
    int rx_pin;
    
    byte* tx_buffer;
    int tx_buffer_index;
    int tx_buffer_length;
    int tx_pin;
  
    GenericSensorModel* pModel;
    bool print_serial;
};

class ArduinoSerial : public SimulationSerial {
public:
    ArduinoSerial(int rx_pin, int tx_pin) : SimulationSerial(rx_pin, tx_pin)
    {
        serial_type = "Arduino_Nano";
        max_buffer_size = max_arduino_buffer_size;
    }
};

class Teensy3Serial : public SimulationSerial {
public:
    Teensy3Serial(int rx_pin, int tx_pin) : SimulationSerial(rx_pin, tx_pin)
    {
        serial_type = "Teensy3_series";
        max_buffer_size = max_teensy3_series_buffer_size;
    }
};

class Teensy4Serial : public SimulationSerial {
public:
    Teensy4Serial(int rx_pin, int tx_pin) : SimulationSerial(rx_pin, tx_pin)
    {
        serial_type = "Teensy4_series";
        max_buffer_size = max_teensy4_series_buffer_size;
    }
};

class GPSNeo6MSerial : public SimulationSerial {
public:
    GPSNeo6MSerial(int rx_pin, int tx_pin) : SimulationSerial(rx_pin, tx_pin)
    {
        serial_type = "GPS_Neo6m";
        max_buffer_size = max_gps_neo6m_buffer_size;
    }
};

class BluetoothHC05Serial : public SimulationSerial {
public:
    BluetoothHC05Serial(int rx_pin, int tx_pin) : SimulationSerial(rx_pin, tx_pin)
    {
        serial_type = "HC05_Bluetooth";
        max_buffer_size = max_HC05Bluetooth_buffer_size;
    }
};

class APC220RadioSerial : public SimulationSerial {
public:
    APC220RadioSerial(int rx_pin, int tx_pin) : SimulationSerial(rx_pin, tx_pin)
    {
        serial_type = "APC220Radio";
        max_buffer_size = max_APC220Radio_buffer_size;
    }
};

class TwoHardwareSerial : public GenericSensorModel {
public:
    TwoHardwareSerial(ModelMap *pMapInit, bool debugFlagIn = false);
    
    virtual void initialize();
    virtual bool update();
    virtual void setSerialIO(SimulationSerial* pSerial, std::string periphialType);
private:
    class Time* pTime;
    
    struct MCUIO {
        MCUIO();
        ~MCUIO();
        
        SimulationSerial* pMCUSerial;
        SimulationSerial* pMCUperiphial;
        bool init;
        unsigned int mcu_rx_fail_count;
        unsigned int mcu_tx_fail_count;
        unsigned int periphial_rx_fail_count;
        unsigned int periphial_tx_fail_count;
        
        void set(SimulationSerial* pSerial, std::string periphialType);
        void update(MCUIO &otherMCU);
    };
    
    MCUIO MCU1;
    MCUIO MCU2;
};

class ArduinoEEPROM {
public:
    ArduinoEEPROM();
    ~ArduinoEEPROM();
    
    byte read(unsigned int address);
    void write(unsigned int address, byte val);
    
private:
    unsigned int size;
    bool new_file;
    std::__1::streamoff file_length;
    std::string filename;
    std::fstream memoryfile;
    
    void reset_eeprom();
};

typedef enum { RF24_1MBPS, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;
typedef enum { RF24_PA_MIN , RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e;

class SimulationNRF24L01 {
public:
    SimulationNRF24L01(unsigned int CE_PIN, unsigned int CSN_PIN);
    void begin();
    void openWritingPipe(const byte* address);
    void openReadingPipe(byte number , const byte* address);
    void stopListening();
    void startListening();
    bool write(byte* buffer, unsigned int size);
    void read(byte* buffer, unsigned int size);
    bool available();
    
    void setPALevel(rf24_pa_dbm_e rf_level);
    void setDataRate( rf24_datarate_e data_rate );
    
    static const byte address_size = 5;
    static const byte buffer_size  = 32;
    
    struct buffer_type {
        byte buffer[buffer_size];
        unsigned int rate;
        unsigned int payload_size;
        buffer_type() {
            memset(buffer, 0, buffer_size);
            rate =  250000;
            payload_size = buffer_size;
        }
    };
    
    // Buffer shared by all instances
    static std::map<std::string, buffer_type> buffer_map;
    static std::map<std::string, bool> buffer_empty_map;
    
    //int getPayloadSize();
    //enableAckPayload();
    //setRetries(5, 5);
    //writeAckPayload(1, &sent_1, sizeof(sent_1))
private:
    bool address_open(std::string address);
    bool buffer_empty();
    
    // Info local to this instance
    std::map<std::string, bool> writing_pipe_map;
    
    bool active;
    bool listening;
    std::string address;
    
    void print_buffer(std::string address);
    void print_buffer_map();
    void print_buffer_empty();
    void print_writing_map();
};

void pinMode(int pin, pinMode mode);
void delay(int timeMS);

#define RF24 SimulationNRF24L01
#define SoftwareSerial ArduinoSerial
#define HardwareSerial Teensy4Serial

extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern SimulationSerial Serial;
extern SimulationWire Wire;
extern ArduinoEEPROM EEPROM;

#endif /* arduino_class_models_hpp */
