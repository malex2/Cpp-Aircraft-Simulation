//
//  ground_station.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 12/29/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#include "ground_station.hpp"

const byte BLUETOOTH_ACKNOWLEDGE_KEYa = 0x46; // hex for F
const byte BLUETOOTH_ACKNOWLEDGE_KEYb = 0x53; // hex for S (FS = 18003)
const byte test_msg[] = "Flight SW Test Message";
byte test_msg_checksum[2];
byte test_msg_length[2];
byte ble_init_key[2];

bool GS_initialized = false;

// Telemetry
int GS_TMBaudRate;
SoftwareSerial GS_tmIO(TELEMETRY_RADIO_RXPIN, TELEMETRY_RADIO_TXPIN);
double prevWriteTime_TM;
double baudDt_TM;
bool GS_configure_TM;
bool TM_setting_sent;
const unsigned char APC220_CONFIG[19] = {'W','R',' ','4','3','4','0','0','0',' ','3',' ','9',' ','0',' ', '0', '\r', '\n'};
unsigned int TM_config_rsp_count;

// Bluetooth
int GS_Bluetooth_Baudrate;
bool bluetooth_ack;
#ifdef BLUETOOTH
   SoftwareSerial bluetoothIO(BLUETOOTH_RXPIN, BLUETOOTH_TXPIN);
#else
   #define bluetoothIO Serial
#endif
double prevWriteTime_BLE;
double baudDt_BLE;

#ifdef GROUND_STATION_SIMULATION
    class Time* GS_pTime = 0;
#endif

// **********************************************************************
// Initialize
// **********************************************************************
void GS_initialize()
{
    // Telemetry Settings
    GS_TMBaudRate    = 9600;
    baudDt_TM        = 1.0/GS_TMBaudRate;
    prevWriteTime_TM = 0.0;
    GS_APC220_SetAwake();
    GS_APC220_SetRunningMode();
    GS_configure_TM = false;
    TM_setting_sent = false;
    TM_config_rsp_count = 0;
#ifndef GROUND_STATION_SIMULATION
    pinMode(TELEMETRY_RADIO_ENPIN, OUTPUT);
    pinMode(TELEMETRY_RADIO_SETPIN, OUTPUT);
#endif
    
    // Bluetooth Settings
    GS_Bluetooth_Baudrate = 9600;
    baudDt_BLE            = 1.0/GS_Bluetooth_Baudrate;
    prevWriteTime_BLE     = 0.0;
    
#ifndef GROUND_STATION_SIMULATION
    pinMode(TELEMETRY_RADIO_RXPIN, INPUT);
    pinMode(TELEMETRY_RADIO_TXPIN, OUTPUT);
    #ifdef BLUETOOTH
       pinMode(BLUETOOTH_RXPIN, INPUT);
       pinMode(BLUETOOTH_TXPIN, OUTPUT);
    #endif
#endif
    
    // Begin Serial
    GS_tmIO.begin(GS_TMBaudRate);
    bluetoothIO.begin(GS_Bluetooth_Baudrate);
    bluetooth_ack = false;
#ifdef PRINT_BLUETOOTH
    bluetooth_ack = true;
#endif
    
    memset(ble_init_key, 0, 2);
    uint16_t length = sizeof(test_msg);
    memcpy(test_msg_length, &length, 2);
    GS_computeChecksum(test_msg_checksum, test_msg, sizeof(test_msg));
}

// **********************************************************************
// Main Ground Station Software
// **********************************************************************
void mainGroundStation()
{
    if (!GS_initialized)
    {
        GS_initialize();
        GS_initialized = true;
    }
    
    // Handle Telemetry
    GS_PerformTelemetry();
    
    // Handle Bluetooth
    GS_PerformBluetooth();
}

void GS_PerformTelemetry()
{
#ifdef TEST_MODE
    if (GS_getTime()-prevWriteTime_BLE > 2.0)
    {
        bluetoothIO.write(TM_HEADER);
        bluetoothIO.write(TM_PRINT_HEADER);
        bluetoothIO.write(test_msg_length, 2);
        bluetoothIO.write(test_msg, sizeof(test_msg));
        bluetoothIO.write(test_msg_checksum, 2);
        prevWriteTime_BLE = GS_getTime();
    }
#else
    if (GS_configure_TM && !TM_setting_sent)
    {
        GS_APC220_SetSettingMode();
        delay(2);
        Serial.println("Sending TM Config");
        Serial.write(APC220_CONFIG, 19);
        GS_tmIO.write(APC220_CONFIG, 19);
        TM_setting_sent = true;
    }
    
    if (GS_tmIO.available() && (GS_getTime()-prevWriteTime_BLE) >= baudDt_BLE)
    {
        if (GS_configure_TM && TM_setting_sent)
        {
           Serial.write(GS_tmIO.read());
           TM_config_rsp_count++;
        }
        else if (bluetooth_ack)
        {
            bluetoothIO.println(GS_tmIO.read());
            //bluetoothIO.write(GS_tmIO.read());
        }
        else
        {
            Serial.print("Dumping: ");
            Serial.println(GS_tmIO.read());
        }
        
        prevWriteTime_BLE = GS_getTime();
        
        if (GS_configure_TM && TM_setting_sent && TM_config_rsp_count == 21)
        {
            Serial.println("TM Config Set!");
            GS_APC220_SetRunningMode();
            TM_config_rsp_count = 0;
            GS_configure_TM = false;
            TM_setting_sent = false;
        }
    }
#endif
}

void GS_PerformBluetooth()
{
    if (bluetoothIO.available() && (GS_getTime()-prevWriteTime_TM) >= baudDt_TM)
    {
        byte tm_byte = bluetoothIO.read();
        if (bluetooth_ack)
        {
            GS_tmIO.write(tm_byte);
            prevWriteTime_TM = GS_getTime();
        }
        else
        {
            if (!ble_init_key[0] && tm_byte == BLUETOOTH_ACKNOWLEDGE_KEYa)
            {
                ble_init_key[0] = tm_byte;
            }
            else if (ble_init_key[0] && !ble_init_key[1] && tm_byte == BLUETOOTH_ACKNOWLEDGE_KEYb)
            {
                ble_init_key[1] = tm_byte;
            }
            if (ble_init_key[0] && ble_init_key[1])
            {
                if (ble_init_key[0] == BLUETOOTH_ACKNOWLEDGE_KEYa && ble_init_key[1] == BLUETOOTH_ACKNOWLEDGE_KEYb)
                {
                    bluetooth_ack = true;
                }
                memset(ble_init_key, 0, 2);
            }
        }
    }
}

void GS_computeChecksum(byte* checksum, const void* buffer, unsigned int len)
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
    
    uint16_t checksum_int = (c1 << 8 | c0);
    memcpy(checksum, &checksum_int, sizeof(checksum_int));
}

// Time
double GS_getTime()
{
#ifdef GROUND_STATION_SIMULATION
    if (GS_pTime) { return GS_pTime->getSimTime(); }
    else { return 0.0; }
#else
    return micros() / 1000000.0;
#endif
}

// APC220 Telemetry
//#define TELEMETRY_RADIO_ENPIN  7 // APC220 - Set high to enable
//#define TELEMETRY_RADIO_SETPIN 8 // APC220 - Set low to go into settings mode
void GS_APC220_SetAwake()
{
#ifndef GROUND_STATION_SIMULATION
    digitalWrite(TELEMETRY_RADIO_ENPIN, HIGH);
#endif
}

void GS_APC220_SetSleep()
{
#ifndef GROUND_STATION_SIMULATION
    digitalWrite(TELEMETRY_RADIO_ENPIN, LOW);
#endif
}

void GS_APC220_SetSettingMode()
{
#ifndef GROUND_STATION_SIMULATION
    digitalWrite(TELEMETRY_RADIO_SETPIN, LOW);
#endif
}

void GS_APC220_SetRunningMode()
{
#ifndef GROUND_STATION_SIMULATION
    digitalWrite(TELEMETRY_RADIO_SETPIN, HIGH);
#endif
}

double GS_read_voltage(int analog)
{
    double V = 5.0*analog/1023.0;
    return V;
}

#ifdef GROUND_STATION_SIMULATION
void groundStation_setMapPointer(ModelMap* pMap)
{
    if (pMap)
    {
        GS_pTime = (Time*) pMap->getModel("Time");
        GS_tmIO.serial_setTwoHardwareSerialModels(pMap, "FS_GS_Interface", "APC220Radio");
        bluetoothIO.serial_setSimulationModels(pMap, "Telemetry", false);
    }
}
#endif
