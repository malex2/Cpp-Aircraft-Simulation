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
#define TELEMETRY
#define BLUETOOTH
//#define TEST_MODE

#ifdef GROUND_STATION_SIMULATION
   #include "arduino_class_models.hpp"
    #include "time.hpp"
#else
   #include <SPI.h>
   #include <nRF24L01.h>
   #include <RF24.h>
#endif

// SPI Pints
#define ARDUINO_NANO_CE 9
#define ARDUINO_NANO_CSN 10
#define TEENSY40_CE 4
#define TEENSY40_CSN 2
#define NRF24L01_BUFFER_SIZE 32

// Bluetooth Pins
#define BLUETOOTH_RXPIN 2 // Rcv Bluetooth msgs, connect to Bluetooth TX
#define BLUETOOTH_TXPIN 3 // Txmit Bluetooth msgs, connect to Bluetooth RX

#define TM_HEADER       0xA0
#define TM_TM_HEADER    0xB1
#define TM_PRINT_HEADER 0xC2

void GS_initialize();
void mainGroundStation();

void GS_computeChecksum(byte* checksum, const void* data, unsigned int len);

#ifdef GROUND_STATION_SIMULATION
   class ModelMap;
   void groundStation_setMapPointer(ModelMap* pMapInit);
#endif

double GS_getTime();

#endif /* ground_station_hpp */
