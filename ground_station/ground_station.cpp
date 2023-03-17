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
bool bluetooth_ack;

// Telemetry
int GS_TMBaudRate;
SoftwareSerial GS_tmIO(TELEMETRY_RADIO_RXPIN, TELEMETRY_RADIO_TXPIN);
double prevWriteTime_TM;
double baudDt_TM;

// Bluetooth
int GS_Bluetooth_Baudrate;
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
    
    // Bluetooth Settings
    GS_Bluetooth_Baudrate = 9600;
    baudDt_BLE            = 1.0/GS_Bluetooth_Baudrate;
    prevWriteTime_BLE     = 0.0;
    
    // Begin Serial
    GS_tmIO.begin(GS_TMBaudRate);
    bluetoothIO.begin(GS_Bluetooth_Baudrate);
    
#ifdef BLUETOOTH
    bluetooth_ack = false;
#else
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

#ifdef TEST_MODE
    if (GS_getTime()-prevWriteTime > 2.0)
    {
        bluetoothIO.write(TM_HEADER);
        bluetoothIO.write(TM_PRINT_HEADER);
        bluetoothIO.write(test_msg_length, 2);
        bluetoothIO.write(test_msg, sizeof(test_msg));
        bluetoothIO.write(test_msg_checksum, 2);
        prevWriteTime = GS_getTime();
    }
#else
    if (GS_tmIO.available() && (GS_getTime()-prevWriteTime_BLE) >= baudDt_BLE)
    {
        if (bluetooth_ack)
        {
            bluetoothIO.write(GS_tmIO.read());
        }
        else
        {
            GS_tmIO.read();
        }
        
        prevWriteTime_BLE = GS_getTime();
    }
#endif
    if (bluetoothIO.available() && (GS_getTime()-prevWriteTime_TM) >= baudDt_TM)
    {
        if (bluetooth_ack)
        {
            GS_tmIO.write(bluetoothIO.read());
        }
        else
        {
            if (!ble_init_key[0])
            {
                ble_init_key[0] = bluetoothIO.read();
            }
            else if (ble_init_key[0] && ! ble_init_key[1])
            {
                ble_init_key[1] = bluetoothIO.read();
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
        
        prevWriteTime_TM = GS_getTime();
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
