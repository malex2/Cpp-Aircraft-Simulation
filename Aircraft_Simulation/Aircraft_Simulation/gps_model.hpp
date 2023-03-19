//
//  gps_model.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 5/7/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#ifndef gps_model_hpp
#define gps_model_hpp

#include <iostream>
#include <fstream>
#include "generic_model.hpp"

class GPSModelBase : public GenericSensorModel
{
public:
    // Constructor
    GPSModelBase(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Deconstructor
    ~GPSModelBase();
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    virtual void updateSerial() { };
    virtual void setPerfectSensor(bool input) { perfectSensor = input; }
    
    // Data Types
    enum GPS_Type {NONE, GPS_NEO6M};
    
protected:
    class DynamicsModel* pDyn;
    class Time* pTime;
    class SimulationSerial* pGPSIO;
    
    double gps_velNED[3];
    double gps_posECEF[3];
    double gps_posLLH[3];
    double gps_altMSL;
    double gps_timeofweek;
    double gps_velNE[2];
    double gps_speed;
    double gps_groundSpeed;
    double gps_heading;
    double gps_acquire_time;
    
    // Log variables
    double gps_lat;
    double gps_lon;
    double horizError;
    double horizDir_Deg;
    double vertError;
    double posErrorNED[3];
    double posErrorECEF[3];
    double velError;
    double velErrorYaw_Deg;
    double velErrorPitch_Deg;
    double velErrorNED[3];
    double d_sendCount;
    double d_receiveCount;
    double d_sendByteCount;
    double d_receiveByteCount;
    
    // Error Inputs
    double horizPosAcc;
    double vertPosAcc;
    double velAcc;
    
    // Error generators
    guassianNoiseM horizPosNoise;
    uniformNoiseM  horizPosNoiseDir;
    guassianNoiseM vertPosNoise;
    guassianNoiseM velNoise;
    uniformNoiseM  velNoiseYaw;
    uniformNoiseM  velNoisePitch;
    
    // Sensor Variables
    bool perfectSensor;
    
    // Serial Messaging
    const static int maxOutputMessages = 10;
    const static int maxInputMessages = 10;
    GPS_Type gps_name;
    
    struct MessageType {
        const static short max_buffer_size = 300;
        double updateRate;
        double updateOffset;
        double prevUpdateTime;
        bool   send;
        bool   init;
        bool   waitingForBuffer;
        byte   buffer[max_buffer_size];
        int    buffer_length;
        bool   isInputMsg;
        static int nGPSInputMessages;
        static int nGPSOutputMessages;
        MessageType(double updateRate = 0.0, double updateOffset = 0.0, bool isInputMsg = false)
        {
            this->updateRate   = updateRate;
            this->updateOffset = updateOffset;
            prevUpdateTime     = 0.0;
            send               = false;
            init               = false;
            waitingForBuffer   = false;
            memset(buffer, 0, sizeof(buffer));
            buffer_length      = 0;
            this->isInputMsg   = isInputMsg;
            
            if (isInputMsg) { nGPSInputMessages++; }
            else { nGPSOutputMessages++; }
        }
        
        void reset()
        {
            if (buffer_length > max_buffer_size) { std::cout << "MessageType allocated more than buffer length!!" << std::endl; }
            send = false;
            waitingForBuffer = false;
            memset(buffer, 0, sizeof(buffer));
            buffer_length = 0;
        }
        
        void print(double time=0.0, int i_msg=-1)
        {
            std::cout << time << std::dec << ") pOutputMessages[" << i_msg << "] " << buffer_length << ": ";
            for (int i=0; i < buffer_length; i++)
            {
                std::cout << std::hex << std::setfill('0') << std::setw(2) << (int) buffer[i];
            }
            std::cout << std::dec << std::endl << std::endl;
        }
    };
    
    MessageType* pOutputMessages[maxOutputMessages];
    MessageType* pInputMessages[maxInputMessages];
    int iGPSInputMessage;
    int nGPSInputMessages;
    int nGPSOutputMessages;
    
    int           sendCount;
    unsigned long sendByteCount;
    double        sendTime;
    int           receiveCount;
    unsigned long receiveByteCount;
    double        receiveTime;
    
    // Functions
    virtual void updatePositionError();
    virtual void updateVelocityError();
    virtual void readInputIO();
    virtual void readInputMessages();
    virtual void decodeInputMessages();
    virtual void constructOutputMessages();
    virtual void writeOutputMessages();
    virtual void writeOutputIO();
};

class GPSNeo6m : public GPSModelBase
{
public:
    GPSNeo6m(ModelMap *pMapInit, bool debugFlagIn = false);
    ~GPSNeo6m();
    
    virtual void initialize();
    virtual void updateSerial();
private:
    typedef GPSModelBase Base;
    class   FS_FIFO* pUBXFIFO;
    class   UBX_MSG* pUBX;
    struct  GpsType* gpsData;
    
    bool initUBX;
    enum GPSOutputMessgaes {NAV_STATUS, NAV_DOP, NAV_POSLLH, NAV_VELNED, NGPSMESSAGES};
    double last_3dFixAlt;
    double last_3dFixMSL;
    double last_3dFixVD;
    
    struct gps_file_info {
        gps_file_info();
        
        static const int n_duplicate = 5;
        int duplicate_class[n_duplicate];
        int duplicate_id[n_duplicate];
        
        int prev_msg_class[maxOutputMessages];
        int prev_msg_id[maxOutputMessages];
        int i_msg;
        
        int  get_available_msg_idx();
        
        bool store_message(int msg_class, int msg_id);
        bool allow_duplicate(int msg_class, int msg_id);
        bool free_message(int msg_class, int msg_id);
        void check_for_sent_messages(MessageType* pOutputMessages[maxOutputMessages]);
        void clear();
    } file_info;
    
    // Lake Eola Park
    // Fix = 0, [horiz, vert, speed] = [16875.61 11932.86 20.00]
    // Fix = 3, [horiz, vert, speed] = [42.22 230.59 3.38]
 
    // Adeline
    // Fix = 2, [horiz, vert] = [15-30, 65]
    static const int GPSFixLength  = 3;
    int GPSFixTimes[GPSFixLength]  = {0, 5, 20};
    int GPSFixValues[GPSFixLength] = {3, 3, 3};
    LookupTable<int> GPSFixLookup;
    
    static const int HorizAccLength = 3;
    double HorizAccTimes[HorizAccLength]  = {0.0, 5.0, 300.0};
    double HorizAccValues[HorizAccLength] = {10.0, 10.0, 10.0};
    LookupTable<double> HorizAccLookup;
    
    static const int VertAccLength = 3;
    double VertAccTimes[VertAccLength]  = {0.0, 5.0, 300.0};
    double VertAccValues[VertAccLength] = {10.0, 10.0, 10.0};
    LookupTable<double> VertAccLookup;
    
    static const int VelAccLength = 3;
    double VelAccTimes[VelAccLength]  = {0.0, 5.0, 300.0};
    double VelAccValues[VelAccLength] = {2.0, 2.0 , 0.1};
    LookupTable<double> VelAccLookup;
    
    virtual void updatePositionError();
    virtual void updateVelocityError();
    virtual void readInputMessages();
    virtual void decodeInputMessages();
    virtual void constructOutputMessages();
    int encodeOutputMessage(int gpsMsg, void* buffer, int buffer_length);
    
    std::ifstream gps_msg_file;
    std::streampos gps_msg_file_begin;
};

#endif /* gps_model_hpp */
