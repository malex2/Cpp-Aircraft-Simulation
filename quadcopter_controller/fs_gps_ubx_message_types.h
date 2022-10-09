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
    static const int ALM_WEEK_INVALID  = 0;
    
    // Navigation Dynamic Modes
    enum DynamicPlatformModel {Portable=0, Stationary=2, Pedestrian, Automotive, Sea, Airbone_1g, Airbone_2g, Airbone_4g};
    enum NavConfigMask {NavConfig_dyn, NavConfig_minEl, NavConfig_fixMode, NavConfig_drLim, NavConfig_posMask, NavConfig_timeMask, NavConfig_staticHoldMeask, NavConfig_dgpsMask};
    enum NavStatusFlag {GPSFixOk, DiffSoln, WKNSET, TOWSET};
    enum AidInitFlags {INI_POS, INI_TIME, INI_clockD, INI_tp, INI_clockF, INI_lla, INI_altInv, INI_prevTm};
    // Messages
    enum UBX_MSG_CLASS_ID {NAV, RXM, INF, ACK, CFG, MON, AID, TIM, NUBXCLASSES};
    enum UBX_MSG_NAV_ID   {NAV_STATUS, NAV_DOP, NAV_POSLLH, NAV_VELNED, NAV_SOL, NNAVMESSAGES};
    enum UBX_MSG_ACK_ID   {ACK_ACK, ACK_NAK, NACKMESSAGES};
    enum UBX_MSG_CFG_ID   {CFG_MSG, CFG_NAV5, NCFGMESSAGES};
    enum UBX_MSG_AID_ID   {AID_INI, AID_ALM, AID_EPH, AID_HUI, AID_DATA, AID_REQ, NAIDMESSAGES};
    
    static const byte UBX_MSG_CLASS[NUBXCLASSES] = {UBX_NAV, UBX_RXM, UBX_INF, UBX_ACK, UBX_CFG, UBX_MON, UBX_AID, UBX_TIM};
    static const int N_UBX_MSG_ID[NUBXCLASSES]   = {NNAVMESSAGES, 0, 0, NACKMESSAGES, NCFGMESSAGES, 0, NAIDMESSAGES, 0};
    
    static const byte UBX_MSG_ID[NUBXCLASSES][6] = {
        {UBX_NAV_STATUS, UBX_NAV_DOP , UBX_NAV_POSLLH, UBX_NAV_VELNED, UBX_NAV_SOL , 0x00       }, // NAV
        {0x00          , 0x00        , 0x00          , 0x00          , 0x00        , 0x00       }, // RXM
        {0x00          , 0x00        , 0x00          , 0x00          , 0x00        , 0x00       }, // INF
        {UBX_ACK_ACK   , UBX_ACK_NAK , 0x00          , 0x00          , 0x00        , 0x00       }, // ACK
        {UBX_CFG_MSG   , UBX_CFG_NAV5, 0x00          , 0x00          , 0x00        , 0x00       }, // CFG
        {0x00          , 0x00        , 0x00          , 0x00          , 0x00        , 0x00       }, // MON
        {UBX_AID_INI   , UBX_AID_ALM , UBX_AID_EPH   , UBX_AID_HUI   , UBX_AID_DATA, UBX_AID_REQ}, // AID
        {0x00          , 0x00        , 0x00          , 0x00          , 0x00        , 0x00       }  // TIM
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
            int32_t  altitude_ellipsoid = 0;
            int32_t  altitude_msl       = 0;
            uint32_t horizontalAccuracy = 0;
            uint32_t verticalAccuracy   = 0;
        } data;
        
        struct scale_t
        {
            double iTOW               = 1.0e-3;
            double longitude          = 1.0e-7;
            double latitude           = 1.0e-7;
            double altitude_ellipsoid = 1.0e-3;
            double altitude_msl       = 1.0e-3;
            double horizontalAccuracy = 1.0e-3;
            double verticalAccuracy   = 1.0e-3;
        } scale;
        
        void clear() { memset(&data, 0, sizeof(data)); }
        
        void print()
        {
            display("UBX_MSG_NAV_POSLLH (data): ");
            display("iTOW "); display(data.iTOW); display(", ");
            display("longitude "); display(data.longitude); display(", ");
            display("latitude "); display(data.latitude); display(", ");
            display("altitude_ellipsoid "); display(data.altitude_ellipsoid); display(", ");
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
        
        void clear() { memset(&data, 0, sizeof(data)); }
        
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
        
        void clear() { memset(&data, 0, sizeof(data)); }
        
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
        
        void clear() { memset(&data, 0, sizeof(data)); }
        
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
        
        void clear() { memset(&data, 0, sizeof(data)); }
        
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
        
        void clear() { memset(&data, 0, sizeof(data)); }
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
        
        void clear() { memset(&data, 0, sizeof(data)); }
    };
    
    
    /*---------------------------------------------------*/
    /*--------------------ACK_ACK------------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_ACK
    {
        struct data_t
        {
            byte clsID = 0;
            byte msgID = 0;
        } data;
        
        struct scale_t
        {
            double clsID = 1.0;
            double msgID = 1.0;
        } scale;
        
        void clear() { memset(&data, 0, sizeof(data)); }
    };
    
    /*---------------------------------------------------*/
    /*--------------------CFG_NAV5-----------------------*/
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
        
        void clear() { memset(&data, 0, sizeof(data)); }
    };
    
    /*---------------------------------------------------*/
    /*---------------------AID_INI-----------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_AID_INI
    {
        struct data_t
        {
            int32_t  ecefXOrLat       = 0;
            int32_t  ecefYOrLon       = 0;
            int32_t  ecefZOrAlt       = 0;
            uint32_t posAcc           = 0;
            uint16_t tmCfg            = 0;
            uint16_t wn               = 0;
            uint32_t tow              = 0;
            int32_t  towNs            = 0;
            uint32_t tAccMs           = 0;
            uint32_t tAccNs           = 0;
            int32_t  clkDOrFreq       = 0;
            uint32_t clkDAccOrFreqAcc = 0;
            uint32_t flags            = 0;
        } data;
        
        struct scale_t
        {
            double ecefX            = 1.0e-2;
            double ecefY            = 1.0e-2;
            double ecefZ            = 1.0e-2;
            double Lat              = 1.0e-7;
            double Lon              = 1.0e-7;
            double Alt              = 1.0e-2;
            double posAcc           = 1.0e-2;
            double tmCfg            = 1.0;
            double wn               = 1.0;
            double tow              = 1.0e-3;
            double towNs            = 1.0e-9;
            double tAccMs           = 1.0e-3;
            double tAccNs           = 1.0e-9;
            double clkD             = 1.0e-9;
            double Freq             = 1.0e2;
            double clkDAcc          = 1.0e-9;
            double FreqAcc          = 1.0;
            double flags            = 1.0;
        } scale;
        
        void clear() { memset(&data, 0, sizeof(data)); }
    };
    
    /*---------------------------------------------------*/
    /*--------------------AID_ALM------------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_AID_ALM_POLL_SV
    {
        struct data_t
        {
            byte svid = 0;
        } data;
        
        struct scale_t
        {
            double svid = 1.0;
        } scale;
        
        void clear() { memset(&data, 0, sizeof(data)); }
    };
    
    struct UBX_MSG_AID_ALM_INVALID
    {
        struct data_t
        {
            uint32_t svid = 0;
            uint32_t week = 0;
        } data;
        
        struct scale_t
        {
            double svid = 1.0;
            double week = 1.0;
        } scale;
        
        void clear() { memset(&data, 0, sizeof(data)); }
    };
    
    struct UBX_MSG_AID_ALM
    {
        struct data_t
        {
            uint32_t svid    = 0;
            uint32_t week    = 0;
            uint32_t dwrd[8] = {0};
        } data;
        
        struct scale_t
        {
            double svid    = 1.0;
            double week    = 1.0;
            double dwrd[8] = {1.0};
        } scale;
        
        void clear() { memset(&data, 0, sizeof(data)); }
    };
    
    /*---------------------------------------------------*/
    /*--------------------AID_EPH------------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_AID_EPH_POLL_SV
    {
        struct data_t
        {
            byte svid = 0;
        } data;
        
        struct scale_t
        {
            double svid = 1.0;
        } scale;
        
        void clear() { memset(&data, 0, sizeof(data)); }
    };
    
    struct UBX_MSG_AID_EPH_INVALID
    {
        struct data_t
        {
            uint32_t svid = 0;
            uint32_t how  = 0;
        } data;
        
        struct scale_t
        {
            double svid = 1.0;
            double how  = 1.0;
        } scale;
        
        void clear() { memset(&data, 0, sizeof(data)); }
    };
    
    struct UBX_MSG_AID_EPH
    {
        struct data_t
        {
            uint32_t svid    = 0;
            uint32_t how     = 0;
            uint32_t sf1d[8] = {0};
            uint32_t sf2d[8] = {0};
            uint32_t sf3d[8] = {0};
        } data;
        
        struct scale_t
        {
            double svid    = 1.0;
            double how     = 1.0;
            double sf1d[8] = {1.0};
            double sf2d[8] = {1.0};
            double sf3d[8] = {1.0};
        } scale;
        
        void clear() { memset(&data, 0, sizeof(data)); }
    };
    /*---------------------------------------------------*/
    /*-------------------AID_HUI-------------------------*/
    /*---------------------------------------------------*/
    struct UBX_MSG_AID_HUI
    {
        struct data_t
        {
            uint32_t health   = 0;
            double   utcA0    = 0.0;
            double   utcA1    = 0.0;
            int32_t  utcTOW   = 0;
            int16_t  utcWNT   = 0;
            int16_t  utcLS    = 0;
            int16_t  utcWNF   = 0;
            int16_t  utcDN    = 0;
            int16_t  utcLSF   = 0;
            int16_t  utcSpare = 0;
            float    klobA0   = 0.0;
            float    klobA1   = 0.0;
            float    klobA2   = 0.0;
            float    klobA3   = 0.0;
            float    klobB0   = 0.0;
            float    klobB1   = 0.0;
            float    klobB2   = 0.0;
            float    klobB3   = 0.0;
            uint32_t flags    = 0;
        } __attribute__((packed)) data;
        
        struct scale_t
        {
            double health   = 1.0;
            double utcA0    = 1.0;
            double utcA1    = 1.0;
            double utcTOW   = 1.0;
            double utcWNT   = 1.0;
            double utcLS    = 1.0;
            double utcWNF   = 1.0;
            double utcDN    = 1.0;
            double utcLSF   = 1.0;
            double utcSpare = 1.0;
            double klobA0   = 1.0;
            double klobA1   = 1.0;
            double klobA2   = 1.0;
            double klobA3   = 1.0;
            double klobB0   = 1.0;
            double klobB1   = 1.0;
            double klobB2   = 1.0;
            double klobB3   = 1.0;
            double flags    = 1.0;
        } scale;
        
        void clear() { memset(&data, 0, sizeof(data)); }
        
        void print()
        {
            display("UBX_MSG_AID_HUI (data): ");
            display("health "); display((int) data.health); display(", ");
            display("utcA0 "); display((int) data.utcA0); display(", ");
            display("utcA1 "); display((int) data.utcA1); display(", ");
            display("utcTOW "); display((int) data.utcTOW); display(", ");
            display("utcWNT "); display((int) data.utcWNT); display(", ");
            display("utcLS "); display((int) data.utcLS); display(", ");
            display("utcWNF "); display((int) data.utcWNF); display(", ");
            display("\n");
            
            display("UBX_MSG_AID_HUI (bytes) "); display(sizeof(data)); display(": ");
            for (int i=0; i < sizeof(data); i++)
            {
                display( +((char*)&data)[i], HEX);
            }
            display("\n");
        }
    };
}

#endif /* fs_gps_ubx_message_types_h */
