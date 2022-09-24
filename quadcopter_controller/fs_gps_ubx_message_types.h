//
//  fs_gps_ubx_message_types.h
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 8/28/22.
//  Copyright © 2022 Alexander McLean. All rights reserved.
//

#ifndef fs_gps_ubx_message_types_h
#define fs_gps_ubx_message_types_h
#include "fs_common.hpp"

// UBX Messages Types
namespace UBX_MSG_TYPES {
    static const int UNKNOWN_MSG_ID    = -1;
    static const int UNKNOWN_MSG_CLASS = -1;
    
    // Navigation Dynamic Modes
    enum DynamicPlatformModel {Portable=0, Stationary=2, Pedestrian, Automotive, Sea, Airbone_1g, Airbone_2g, Airbone_4g};
    enum NavConfigMask {NavConfig_dyn, NavConfig_minEl, NavConfig_fixMode, NavConfig_drLim, NavConfig_posMask, NavConfig_timeMask, NavConfig_staticHoldMeask, NavConfig_dgpsMask};
    // Messages
    enum UBX_MSG_CLASS_ID {NAV, RXM, INF, ACK, CFG, MON, AID, TIM, NUBXCLASSES};
    enum UBX_MSG_NAV_ID   {NAV_STATUS, NAV_DOP, NAV_POSLLH, NAV_VELNED, NAV_SOL, NNAVMESSAGES};
    enum UBX_MSG_ACK_ID   {ACK_ACK, ACK_NAK, NACKMESSAGES};
    enum UBX_MSG_CFG_ID   {CFG_MSG, CFG_NAV5, NCFGMESSAGES};
    
    static const byte UBX_MSG_CLASS[NUBXCLASSES] = {UBX_NAV, UBX_RXM, UBX_INF, UBX_ACK, UBX_CFG, UBX_MON, UBX_AID, UBX_TIM};
    static const int N_UBX_MSG_ID[NUBXCLASSES]   = {NNAVMESSAGES, 0, 0, NACKMESSAGES, NCFGMESSAGES, 0, 0, 0};
    
    static const byte UBX_MSG_ID[NUBXCLASSES][5] = {
        {UBX_NAV_STATUS, UBX_NAV_DOP , UBX_NAV_POSLLH, UBX_NAV_VELNED, UBX_NAV_SOL}, // NAV
        {0x00          , 0x00        , 0x00          , 0x00          , 0x00       }, // RXM
        {0x00          , 0x00        , 0x00          , 0x00          , 0x00       }, // INF
        {UBX_ACK_ACK   , UBX_ACK_NAK , 0x00          , 0x00          , 0x00       }, // ACK
        {UBX_CFG_MSG   , UBX_CFG_NAV5, 0x00          , 0x00          , 0x00       }, // CFG
        {0x00          , 0x00        , 0x00          , 0x00          , 0x00       }, // MON
        {0x00          , 0x00        , 0x00          , 0x00          , 0x00       }, // AID
        {0x00          , 0x00        , 0x00          , 0x00          , 0x00       }  // TIM
    };
    
    /*---------------------------------------------------*/
    /*------------------NAV_POSLLH-----------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_NAV_POSLLH
    {
        struct data_t
        {
            uint32_t iTOW               = 0;
            int32_t  longitude          = 0;
            int32_t  latitude           = 0;
            int32_t  altitude_geod      = 0;
            int32_t  altitude_msl       = 0;
            uint32_t horizontalAccuracy = 0;
            uint32_t verticalAccuracy   = 0;
        } data;
        
        struct scale_t
        {
            double iTOW               = 1.0e-3;
            double longitude          = 1.0e-7;
            double latitude           = 1.0e-7;
            double altitude_geod      = 1.0e-3;
            double altitude_msl       = 1.0e-3;
            double horizontalAccuracy = 1.0e-3;
            double verticalAccuracy   = 1.0e-3;
        } scale;
        
        void print()
        {
            display("UBX_MSG_NAV_POSLLH (data): ");
            display("iTOW "); display(data.iTOW); display(", ");
            display("longitude "); display(data.longitude); display(", ");
            display("latitude "); display(data.latitude); display(", ");
            display("altitude_geod "); display(data.altitude_geod); display(", ");
            display("altitude_msl "); display(data.altitude_msl); display(", ");
            display("horizontalAccuracy "); display(data.horizontalAccuracy); display(", ");
            display("verticalAccuracy "); display(data.verticalAccuracy); display(", ");
            display("\n");
            
            display("UBX_MSG_NAV_POSLLH (bytes) "); display(sizeof(data)); display(": ");
            for (int i=0; i < sizeof(data); i++)
            {
                display( +((char*)&data)[i], HEX);
                //std::cout << std::hex << std::setfill('0') << std::setw(2) << +((char*)&data)[i];
            }
            display("\n");
        }
    };
    
    /*---------------------------------------------------*/
    /*------------------NAV_VELNED-----------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_NAV_VELNED
    {
        struct data_t
        {
            uint32_t iTOW            = 0;
            int32_t  velN            = 0;
            int32_t  velE            = 0;
            int32_t  velD            = 0;
            uint32_t speed           = 0;
            uint32_t gSpeed          = 0;
            int32_t  heading         = 0;
            uint32_t speedAccuracy   = 0;
            uint32_t headingAccuracy = 0;
        } data;
        
        struct scale_t
        {
            double iTOW            = 1.0e-3;
            double velN            = 1.0e-2;
            double velE            = 1.0e-2;
            double velD            = 1.0e-2;
            double speed           = 1.0e-2;
            double gSpeed          = 1.0e-2;
            double heading         = 1.0e-5;
            double speedAccuracy   = 1.0e-2;
            double headingAccuracy = 1.0e-5;
        } scale;
        
        void print()
        {
            display("UBX_MSG_NAV_VELNED (data): ");
            display("iTOW "); display(data.iTOW); display(", ");
            display("velN "); display(data.velN); display(", ");
            display("velE "); display(data.velE); display(", ");
            display("velD "); display(data.velD); display(", ");
            display("speed "); display(data.speed); display(", ");
            display("gSpeed "); display(data.gSpeed); display(", ");
            display("heading "); display(data.heading); display(", ");
            display("speedAccuracy "); display(data.speedAccuracy); display(", ");
            display("headingAccuracy "); display(data.headingAccuracy); display(", ");
            display("\n");
            
            display("UBX_MSG_NAV_VELNED (bytes) "); display(sizeof(data)); display(": ");
            for (int i=0; i < sizeof(data); i++)
            {
                display( +((char*)&data)[i], HEX);
                //std::cout << std::hex << std::setfill('0') << std::setw(2) << +((char*)&data)[i];
            }
            display("\n");
        }
    };
    
    /*---------------------------------------------------*/
    /*------------------NAV_STATUS-----------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_NAV_STATUS
    {
        struct data_t
        {
            uint32_t iTOW    = 0;
            byte     gpsFix  = 0;
            char     flags   = 0;
            char     fixStat = 0;
            char     flags2  = 0;
            uint32_t ttff    = 0;
            uint32_t msss    = 0;
        } data;
        
        struct scale_t
        {
            double iTOW    = 1.0e-3;
            double gpsFix  = 1.0;
            double flags   = 1.0;
            double fixStat = 1.0;
            double flags2  = 1.0;
            double ttff    = 1.0e-3;
            double msss    = 1.0e-3;
        } scale;
        
        void print()
        {
            display("UBX_MSG_NAV_STATUS (data): ");
            display("iTOW "); display(data.iTOW); display(", ");
            display("gpsFix "); display((int) data.gpsFix); display(", ");
            display("flags "); display((int) data.flags); display(", ");
            display("fixStat "); display((int) data.fixStat); display(", ");
            display("flags2 "); display((int)  data.flags2); display(", ");
            display("ttff "); display(data.ttff); display(", ");
            display("msss "); display(data.msss); display(", ");
            display("\n");
            
            display("UBX_MSG_NAV_STATUS (bytes) "); display(sizeof(data)); display(": ");
            for (int i=0; i < sizeof(data); i++)
            {
                display( +((char*)&data)[i], HEX);
                //std::cout << std::hex << std::setfill('0') << std::setw(2) << +((char*)&data)[i];
            }
            display("\n");
        }
    };
    
    /*---------------------------------------------------*/
    /*-------------------NAV_DOP-------------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_NAV_DOP
    {
        struct data_t
        {
            uint32_t iTOW    = 0;
            uint16_t gDOP    = 0; // Geometric DOP
            uint16_t pDOP    = 0; // Position DOP
            uint16_t tDOP    = 0; // Time DOP
            uint16_t vDOP    = 0; // Vertical DOP
            uint16_t hDOP    = 0; // Horizontal DOP
            uint16_t nDOP    = 0; // Northing DOP
            uint16_t eDOP    = 0; // Easting DOP
        } __attribute__((packed)) data;
        
        struct scale_t
        {
            double iTOW    = 1.0e-3;
            double gDOP    = 0.01;
            double pDOP    = 0.01;
            double tDOP    = 0.01;
            double vDOP    = 0.01;
            double hDOP    = 0.01;
            double nDOP    = 0.01;
            double eDOP    = 0.01;
        } scale;
        
        void print()
        {
            display("UBX_MSG_NAV_DOP (data): ");
            display("iTOW "); display(data.iTOW); display(", ");
            display("gDOP "); display((int) data.gDOP); display(", ");
            display("pDOP "); display((int) data.pDOP); display(", ");
            display("tDOP "); display((int) data.tDOP); display(", ");
            display("vDOP "); display((int) data.vDOP); display(", ");
            display("nDOP "); display((int) data.nDOP); display(", ");
            display("eDOP "); display((int) data.eDOP); display(", ");
            display("\n");
            
            display("UBX_MSG_NAV_DOP (bytes) "); display(sizeof(data)); display(": ");
            for (int i=0; i < sizeof(data); i++)
            {
                display( +((char*)&data)[i], HEX);
                //std::cout << std::hex << std::setfill('0') << std::setw(2) << +((char*)&data)[i];
            }
            display("\n");
        }
    };
    
    /*---------------------------------------------------*/
    /*-------------------NAV_SOL-------------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_NAV_SOL
    {
        struct data_t
        {
            uint32_t iTOW      = 0;
            int32_t  fTOW      = 0;
            int16_t  week      = 0;
            byte     gpsFix    = 0;
            byte     flags     = 0;
            int32_t  ecefX     = 0;
            int32_t  ecefY     = 0;
            int32_t  ecefZ     = 0;
            uint32_t pAcc      = 0;
            int32_t  ecefVX    = 0;
            int32_t  ecefVY    = 0;
            int32_t  ecefVZ    = 0;
            int32_t  sAcc      = 0;
            uint16_t pDOP      = 0;
            byte     reserved1 = 0;
            byte     numSV     = 0;
            uint32_t reserved2 = 0;
        } data;
        
        struct scale_t
        {
            double iTOW      = 1.0e-3;
            double fTOW      = 1.0e-9;
            double week      = 1.0;
            double gpsFix    = 1.0;
            double flags     = 1.0;
            double ecefX     = 0.01;
            double ecefY     = 0.01;
            double ecefZ     = 0.01;
            double pAcc      = 0.01;
            double ecefVX    = 0.01;
            double ecefVY    = 0.01;
            double ecefVZ    = 0.01;
            double sAcc      = 0.01;
            double pDOP      = 0.01;
            double reserved1 = 1.0;
            double numSV     = 1.0;
            double reserved2 = 1.0;
        } scale;
        
        void print()
        {
            display("UBX_MSG_NAV_SOL (data): ");
            display("iTOW ");   display(data.iTOW); display(", ");
            display("fTOW ");   display(data.fTOW); display(", ");
            display("week ");   display(data.week); display(", ");
            display("gpsFix "); display((int) data.gpsFix); display(", ");
            display("flags ");  display((int) data.flags); display(", ");
            display("ecefX ");  display(data.ecefX); display(", ");
            display("ecefY ");  display(data.ecefY); display(", ");
            display("ecefZ ");  display(data.ecefZ); display(", ");
            display("pAcc ");   display(data.pAcc); display(", ");
            display("ecefVX "); display(data.ecefVX); display(", ");
            display("ecefVY "); display(data.ecefVY); display(", ");
            display("ecefVZ "); display(data.ecefVZ); display(", ");
            display("sAcc ");   display(data.sAcc); display(", ");
            display("pDOP ");   display(data.pDOP); display(", ");
            display("numSV ");  display((int) data.numSV); display(", ");
            display("\n");
            
            display("UBX_MSG_NAV_SOL (bytes) "); display(sizeof(data)); display(": ");
            for (int i=0; i < sizeof(data); i++)
            {
                display( +((char*)&data)[i], HEX);
                //std::cout << std::hex << std::setfill('0') << std::setw(2) << +((char*)&data)[i];
            }
            display("\n");
        }
    };
    
    /*---------------------------------------------------*/
    /*-----------------CFG_MSG_SHORT---------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_CFG_MSG_SHORT
    {
        struct data_t
        {
            byte msgClass = 0;
            byte msgID    = 0;
            byte rate     = 0;
        } data;
        
        struct scale_t
        {
            double msgClass = 1.0;
            double msgID    = 1.0;
            double rate     = 1.0;
        } scale;
    };
    
    /*---------------------------------------------------*/
    /*-----------------CFG_MSG_LONG----------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_CFG_MSG_LONG
    {
        struct data_t
        {
            byte msgClass = 0;
            byte msgID    = 0;
            byte rateTgt1 = 0;
            byte rateTgt2 = 0;
            byte rateTgt3 = 0;
            byte rateTgt4 = 0;
            byte rateTgt5 = 0;
            byte rateTgt6 = 0;
        } data;
        
        struct scale_t
        {
            double msgClass = 1.0;
            double msgID    = 1.0;
            double rateTgt1 = 1.0;
            double rateTgt2 = 1.0;
            double rateTgt3 = 1.0;
            double rateTgt4 = 1.0;
            double rateTgt5 = 1.0;
            double rateTgt6 = 1.0;
        } scale;
    };
    
    /*---------------------------------------------------*/
    /*-----------------CFG_NAV5---------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_CFG_NAV5
    {
        struct data_t
        {
            uint16_t mask             = 0;
            byte     dynModel         = 0;
            byte     fixMode          = 0;
            int32_t  fixedAlt         = 0;
            uint32_t fixedAltVar      = 0;
            int8_t   minElv           = 0;
            byte     drLimit          = 0;
            uint16_t pDop             = 0;
            uint16_t tDop             = 0;
            uint16_t pAcc             = 0;
            uint16_t tAcc             = 0;
            byte     staticHoldThresh = 0;
            byte     dgpsTimeOut      = 0;
            uint32_t reserved2        = 0;
            uint32_t reserved3        = 0;
            uint32_t reserved4        = 0;
            
        } data;
        
        struct scale_t
        {
            double mask             = 1.0;
            double dynModel         = 1.0;
            double fixMode          = 1.0;
            double fixedAlt         = 0.01;
            double fixedAltVar      = 0.0001;
            double minElv           = 1.0;
            double drLimit          = 1.0;
            double pDop             = 0.1;
            double tDop             = 0.1;
            double pAcc             = 1.0;
            double tAcc             = 1.0;
            double staticHoldThresh = 0.01;
            double dgpsTimeOut      = 1.0;
            double reserved2        = 1.0;
            double reserved3        = 1.0;
            double reserved4        = 1.0;
        } scale;
    };
}

#endif /* fs_gps_ubx_message_types_h */
