//
//  ground_station.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 12/29/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#include "ground_station.hpp"
bool GS_initialized = false;

// Telemetry
int GS_TMBaudRate;
SoftwareSerial GS_tmIO(TELEMETRY_RADIO_RXPIN, TELEMETRY_RADIO_TXPIN);
double prevWriteTime_TM;
double baudDt_TM;
bool GS_configure_TM;
bool TM_setting_sent;
unsigned char APC220_CONFIG[19] = {'W','R',' ','4','3','4','0','0','0',' ','3',' ','9',' ','3',' ', '0', '\r', '\n'};
unsigned int TM_config_rsp_count;
unsigned long TM_rcvd_count;
unsigned long TM_sent_count;

// Bluetooth
int GS_Bluetooth_Baudrate;
SoftwareSerial bluetoothIO(BLUETOOTH_RXPIN, BLUETOOTH_TXPIN);
double prevWriteTime_BLE;
double baudDt_BLE;
unsigned long BLE_rcvd_count;
unsigned long BLE_sent_count;

#ifdef GROUND_STATION_SIMULATION
    class Time* GS_pTime = 0;
#endif

// **********************************************************************
// Initialize
// **********************************************************************
void GS_initialize()
{
    // Pins
    pinMode(TELEMETRY_RADIO_ENPIN, OUTPUT);
    pinMode(TELEMETRY_RADIO_SETPIN, OUTPUT);

    pinMode(TELEMETRY_RADIO_RXPIN, INPUT);
    pinMode(TELEMETRY_RADIO_TXPIN, OUTPUT);
    
    pinMode(BLUETOOTH_RXPIN, INPUT);
    pinMode(BLUETOOTH_TXPIN, OUTPUT);
    
    // Telemetry Settings
    GS_TMBaudRate    = 9600;
    baudDt_TM        = 1.0/GS_TMBaudRate;
    prevWriteTime_TM = 0.0;
    GS_configure_TM = true;
    TM_setting_sent = false;
    TM_config_rsp_count = 0;
    TM_rcvd_count = 0;
    TM_sent_count = 0;
    
    GS_APC220_SetAwake();
    GS_APC220_SetRunningMode();
    
    // Set APC220 config
    unsigned char gsfk_val = '3';
    unsigned char baud_val = '3';
    switch (GS_TMBaudRate) {
        case 1200:  { baud_val='0'; gsfk_val='1'; break; }
        case 2400:  { baud_val='1'; gsfk_val='1'; break; }
        case 4800:  { baud_val='2'; gsfk_val='2'; break; }
        case 9600:  { baud_val='3'; gsfk_val='3'; break; }
        case 19200: { baud_val='4'; gsfk_val='4'; break; }
        case 38400: { baud_val='5'; gsfk_val='4'; break; }
        case 57600: { baud_val='6'; gsfk_val='4'; break; }
        default:    { baud_val='3'; gsfk_val='3'; break; }
    }
    APC220_CONFIG[APC220_BAUD_RATE] = baud_val;
    APC220_CONFIG[APC220_GSFK_RATE] = gsfk_val;
    
    // Bluetooth Settings
    GS_Bluetooth_Baudrate = 9600;
    baudDt_BLE            = 1.0/GS_Bluetooth_Baudrate;
    prevWriteTime_BLE     = 0.0;
    BLE_rcvd_count        = 0;
    BLE_sent_count        = 0;
    
    // Begin Serial
    if (GS_configure_TM) { GS_tmIO.begin(9600); }
    else { GS_tmIO.begin(GS_TMBaudRate); }
    
    bluetoothIO.begin(GS_Bluetooth_Baudrate);
    
    Serial.begin(9600);
    
    GS_tmIO.listen();
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
    GS_tmIO.listen();
    GS_PerformTelemetry();
    
    // Handle Bluetooth
    //bluetoothIO.listen();
    GS_PerformBluetooth();
    
    // Prints
    GS_DebugPrints();
}

void GS_PerformTelemetry()
{
    // Configure APC220
    if (GS_configure_TM && !TM_setting_sent)
    {
        GS_APC220_SetSettingMode();
        delay(2);
        Serial.println("Sending TM Config");
        Serial.write(APC220_CONFIG, 19);
        GS_tmIO.write(APC220_CONFIG, 19);
        TM_setting_sent = true;
    }
    
    // Read Telemetry
    while(GS_tmIO.available())
    {
        if (GS_configure_TM && TM_setting_sent)
        {
           Serial.write(GS_tmIO.read());
           TM_config_rsp_count++;
        }
        else
        {
            byte tm_byte = GS_tmIO.read();
            BLE_sent_count += bluetoothIO.write(tm_byte);
            TM_rcvd_count++;
#ifdef PRINT_TM
            Serial.println(tm_byte);
#endif
            prevWriteTime_BLE = GS_getTime();
        }
        
        
        if (GS_configure_TM && TM_setting_sent && TM_config_rsp_count == 21)
        {
            GS_tmIO.end();
            GS_tmIO.begin(GS_TMBaudRate);
            Serial.println("TM Config Set!");
            GS_APC220_SetRunningMode();
            TM_config_rsp_count = 0;
            GS_configure_TM = false;
            TM_setting_sent = false;
        }
    }
}

void GS_PerformBluetooth()
{
    while (bluetoothIO.available())
    {
        TM_sent_count += GS_tmIO.write(bluetoothIO.read());
        BLE_rcvd_count++;
        prevWriteTime_TM = GS_getTime();
    }
}

void GS_DebugPrints()
{
#ifdef PRINT_ANALOG0
    static double prev_A0 = GS_getTime();
    double volt_A0 = GS_read_voltage(A0);
    if (GS_getTime() - prev_A0 > 1.0)
    {
        Serial.print("A0: ");
        Serial.println(volt_A0);
        prev_A0 = GS_getTime();
    }
#endif
    
#ifdef PRINT_ANALOG1
    static double prev_A1 = GS_getTime();
    double volt_A1 = GS_read_voltage(A1);
    if (GS_getTime() - prev_A1 > 1.0)
    {
        Serial.print("A1: ");
        Serial.println(volt_A1);
        prev_A1 = GS_getTime();
    }
#endif
    
#ifdef PRINT_IO_STATUS
    static double prev_status = GS_getTime();
    if (GS_getTime() - prev_status > 1.0)
    {
        Serial.print("[TM_rcvd, TM_write, BLE_rvcd, BLE_write] = [");
        Serial.print(TM_rcvd_count); Serial.print(", ");
        Serial.print(TM_sent_count); Serial.print(", ");
        Serial.print(BLE_rcvd_count); Serial.print(", ");
        Serial.print(BLE_sent_count); Serial.println("]");
        
        prev_status = GS_getTime();
    }
#endif
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
void GS_APC220_SetAwake()
{
    digitalWrite(TELEMETRY_RADIO_ENPIN, HIGH);
}

void GS_APC220_SetSleep()
{
    digitalWrite(TELEMETRY_RADIO_ENPIN, LOW);
}

void GS_APC220_SetSettingMode()
{
    digitalWrite(TELEMETRY_RADIO_SETPIN, LOW);
}

void GS_APC220_SetRunningMode()
{
    digitalWrite(TELEMETRY_RADIO_SETPIN, HIGH);
}

double GS_read_voltage(int pin)
{
#ifndef GROUND_STATION_SIMULATION
    int analog = analogRead(pin);
    double V = 5.0*analog/1023.0;
    return V;
#else
    return 0.0;
#endif
}

#ifdef GROUND_STATION_SIMULATION
void groundStation_setMapPointer(ModelMap* pMap)
{
    if (pMap)
    {
        GS_pTime = (Time*) pMap->getModel("Time");
        
        GS_tmIO.serial_setTwoHardwareSerialModels(pMap, "FS_GS_Interface", "APC220Radio");
        GS_tmIO.serial_setPeriphialEnablePin(pMap, "FS_GS_Interface", &GS_pins, TELEMETRY_RADIO_ENPIN, HIGH);
        GS_tmIO.serial_setPeriphialConfigPin(pMap, "FS_GS_Interface", &GS_pins, TELEMETRY_RADIO_SETPIN, LOW);
        
        bluetoothIO.serial_setSimulationModels(pMap, "Telemetry", false);
    }
}
#endif
