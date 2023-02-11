//
//  ground_station.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 12/29/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#include "ground_station.hpp"

// RF Constants
const byte TM_FS_address[5] = {'0', '0', '0', '0', '1'};
const byte TM_GS_address[5] = {'0', '0', '0', '0', '2'};

const byte BLUETOOTH_ACKNOWLEDGE_KEYa = 0x46; // hex for F
const byte BLUETOOTH_ACKNOWLEDGE_KEYb = 0x53; // hex for S (FS = 18003)
const byte test_msg[] = "Flight SW Test Message";
byte test_msg_checksum[2];
byte test_msg_length[2];
byte key[2];


bool GS_initialized = false;
bool bluetooth_ack;

// Telemetry
rf24_datarate_e GS_TMDataRate;
rf24_pa_dbm_e GS_TMPowerLevel;
byte tm_buffer[NRF24L01_BUFFER_SIZE];
unsigned short tm_buffer_idx;
RF24 GS_tmIO(ARDUINO_NANO_CE, ARDUINO_NANO_CSN);

// Bluetooth
int GS_Bluetooth_Baudrate;
SoftwareSerial bluetoothIO(BLUETOOTH_RXPIN, BLUETOOTH_TXPIN);

#ifdef GROUND_STATION_SIMULATION
    class Time* GS_pTime = 0;
#endif

// **********************************************************************
// Initialize
// **********************************************************************
void GS_initialize()
{
    // Telemetry Settings
    GS_TMDataRate = RF24_250KBPS; //RF24_1MBPS, RF24_2MBPS, RF24_250KBPS
    GS_TMPowerLevel = RF24_PA_MIN; //RF24_PA_MIN , RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    GS_Bluetooth_Baudrate = 9600;
    
    GS_tmIO.begin();
    GS_tmIO.openReadingPipe(0, TM_FS_address);
    GS_tmIO.setPALevel(GS_TMPowerLevel);
    GS_tmIO.setDataRate(GS_TMDataRate);
    GS_tmIO.startListening();
    
    bluetoothIO.begin(GS_Bluetooth_Baudrate);
    
    tm_buffer_idx = 0;
    memset(tm_buffer, 0, NRF24L01_BUFFER_SIZE);
    
    bluetooth_ack = false;
    memset(key, 0, 2);

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
    static double prevTime = 0.0;
    if (GS_getTime()-prevTime > 2.0)
    {
        bluetoothIO.write(TM_HEADER);
        bluetoothIO.write(TM_PRINT_HEADER);
        bluetoothIO.write(test_msg_length, 2);
        bluetoothIO.write(test_msg, sizeof(test_msg));
        bluetoothIO.write(test_msg_checksum, 2);
        prevTime = GS_getTime();
    }
#else
    // Read from NRF24L01
    if (GS_tmIO.available())
    {
        GS_tmIO.read(tm_buffer, (unsigned int) NRF24L01_BUFFER_SIZE);
        
        if (bluetooth_ack)
        {
            bluetoothIO.write(tm_buffer, NRF24L01_BUFFER_SIZE);
            
        }
        memset(tm_buffer, 0, NRF24L01_BUFFER_SIZE);
    }
#endif
    while (bluetoothIO.available())
    {
        if (bluetooth_ack)
        {
            //std::cout << "ground_station tm_buffer[" << tm_buffer_idx << "] = ";
            tm_buffer[tm_buffer_idx++] = bluetoothIO.read();
            //std::cout << std::hex << std::setfill('0') << std::setw(2) << +tm_buffer[tm_buffer_idx-1];
            //std::cout << std::dec << std::endl;
            if (!bluetoothIO.available() || tm_buffer_idx >= (unsigned int) NRF24L01_BUFFER_SIZE)
            {
                // Do nothing with received data for now
                //GS_tmIO.write(tm_buffer, NRF24L01_BUFFER_SIZE);
                memset(tm_buffer, 0, NRF24L01_BUFFER_SIZE);
                tm_buffer_idx = 0;
            }
        }
        else
        {
            if (!key[0])
            {
                key[0] = bluetoothIO.read();
            }
            else if (key[0] && ! key[1])
            {
                key[1] = bluetoothIO.read();
            }
            if (key[0] && key[1])
            {
                if (key[0] == BLUETOOTH_ACKNOWLEDGE_KEYa && key[1] == BLUETOOTH_ACKNOWLEDGE_KEYb)
                {
                    bluetooth_ack = true;
                }
                memset(key, 0, 2);
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

#ifdef GROUND_STATION_SIMULATION
void groundStation_setMapPointer(ModelMap* pMap)
{
    if (pMap)
    {
        GS_pTime = (Time*) pMap->getModel("Time");
        bluetoothIO.serial_setSimulationModels(pMap, "Telemetry", false);
    }
}
#endif
