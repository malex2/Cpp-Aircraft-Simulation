//
//  ground_station.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 12/29/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#ifndef ground_station_hpp
#define ground_station_hpp

#define GROUND_STATION_SIMULATION
//#define BLUETOOTH
#define PRINT_BLUETOOTH
//#define TEST_MODE

#ifdef GROUND_STATION_SIMULATION
   #include "arduino_class_models.hpp"
   #include "time.hpp"
#else
   #include "arduino.h"
   #include <SoftwareSerial.h>
   //#include <SPI.h>
   //#include <nRF24L01.h>
   //#include <RF24.h>
#endif

// SPI Pints
//#define ARDUINO_NANO_CE 9
//#define ARDUINO_NANO_CSN 10
//#define NRF24L01_BUFFER_SIZE 32

// Bluetooth Pins
#define BLUETOOTH_RXPIN 2 // Rcv Bluetooth msgs, connect to Bluetooth TX
#define BLUETOOTH_TXPIN 3 // Txmit Bluetooth msgs, connect to Bluetooth RX

#define TELEMETRY_RADIO_RXPIN 4 // Rcv Bluetooth msgs, connect to Telemetry TX
#define TELEMETRY_RADIO_TXPIN 5 // Txmit Bluetooth msgs, connect to Telemetry RX
#define TELEMETRY_RADIO_ENPIN  7 // APC220 - Set high to enable
#define TELEMETRY_RADIO_SETPIN 8 // APC220 - Set low to go into settings mode
#define ANALOG_EN_READ A0
#define ANALOG_SET_READ A1

#define TM_HEADER       0xA0
#define TM_TM_HEADER    0xB1
#define TM_PRINT_HEADER 0xC2

void GS_initialize();
void mainGroundStation();
void GS_PerformTelemetry();
void GS_PerformBluetooth();
void GS_computeChecksum(byte* checksum, const void* data, unsigned int len);

#ifdef GROUND_STATION_SIMULATION
   class ModelMap;
   void groundStation_setMapPointer(ModelMap* pMapInit);
#endif

double GS_getTime();

void GS_APC220_SetAwake();
void GS_APC220_SetSleep();
void GS_APC220_SetSettingMode();
void GS_APC220_SetRunningMode();

double GS_read_voltage(int analog);
#endif /* ground_station_hpp */
