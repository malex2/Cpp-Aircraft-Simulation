//
//  flight_software.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/30/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "flight_software.hpp"
#include <math.h>

#ifdef SIMULATION
    #include "imu_model.hpp"
    #include "time.hpp"
    #include "model_mapping.hpp"
#else
    #include "Wire.h" // This library allows you to communicate with I2C devices.
#endif

const double deg2rad = 180.0/M_PI;

// Arduino Specific Variables
// I2C Registers
const int MPU_ADDR   = 0x68;
const int PWR_MGMT_1 = 0x6B; // PWR_MGMT_1 register
const int GYRO_REG   = 0x1B; // Gryo register
const int ACC_REG    = 0x1C; // Accelerometer register
const int ACC_OUT    = 0x3B; // (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]

uint baudRate;
bool intiailized     = false;
bool doneCalibrating = false;
bool printAngles     = false;

class IMUModelBase* pIMU;
class Time*         pTime;
class ModelMap*     pMap;

// Enumerations
enum {imuRoutine, attitudeFilterRoutine, controlRoutine, nRoutines};
enum accSensitivityType  {accSensitivity_2g, accSensitivity_4g, accSensitivity_8g, accSensitivity_16g, nAccSensitivity};
enum gyroSensitivityType {gyroSensitivity_250dps, gyroSensitivity_500dps, gyroSensitivity_1000dps, gyroSensitivity_2000dps, nGyroSensitivity};

double accSensitivityLSB[nAccSensitivity]  = {16483, 8192, 4096, 2048};
double gyroSensitivityLSB[nGyroSensitivity] = {131, 65.5, 32.8, 16.4};

char sensitivityByte[nAccSensitivity] = {0b00000000, 0b00001000, 0b00010000, 0b00011000};

// Time keeping (s)
double runTime;
double prevTime[nRoutines];
double trueRoutineDelay[nRoutines];
const double routineDelays[nRoutines] = {0.05, 0.05, 0.05};

// IMU
accSensitivityType  accSensitivity;
gyroSensitivityType gyroSensitivity;
char accSensitivityWrite;
char gyroSensitivityWrite;
bool newIMUmeasurement = false;

double accBias[3];
double gyroBias[3];
double magBias[3];

double accelerometerData[3];
double gyroscopeData[3];

double accCal[3];
double gyroCal[3];
double temperature;

double LSBdps;
double LSBg;
double LSBuT;

// Sensor Fusion (deg)
double attitude[3];
double attitudeDeg[3];
double compFilterGain;
double maxFilterAcc;

// Attitde Keeping (deg)
double desiredAttitude[3];
double throttle;

void initialize(void)
{
    // Change These As Desired
    compFilterGain  = 0.1;
    accSensitivity  = accSensitivity_8g;
    gyroSensitivity = gyroSensitivity_1000dps;
    baudRate        = 38400;
    maxFilterAcc    = 1.1;
    printAngles     = false;
    
    // Setup
    getSimulationModels();
    setupIMU();
}

bool mainFlightSoftware(void)
{
    if (!intiailized)
    {
        initialize();
        intiailized = true;
    }
    
#ifdef SIMULATION
    if (pTime) { runTime = pTime->getSimTime(); }
#else
    runTime = millis() / 1000.0;
#endif
    
    // Get IMU Values
    if (runTime-prevTime[imuRoutine] >= routineDelays[imuRoutine])
    {
        getImuData(accelerometerData, gyroscopeData, &temperature);
        prevTime[imuRoutine] = runTime;
        
        // Calibrate Sensors
        doneCalibrating = groundCalibration(accBias, gyroBias, accelerometerData, gyroscopeData);
    }

    if ( doneCalibrating )
    {
        for (int i=0; i<3; i++)
        {
            accCal[i]  = accelerometerData[i] - accBias[i];
            gyroCal[i] = gyroscopeData[i] - gyroBias[i];
        }
        
        if (runTime-prevTime[attitudeFilterRoutine] >= routineDelays[attitudeFilterRoutine])
        {
            atittudeFilter(attitude, accCal, gyroCal);
            prevTime[attitudeFilterRoutine] = runTime;
        }
        
        if (runTime-prevTime[controlRoutine] >= routineDelays[controlRoutine])
        {
            attitudeControl(attitude);
            prevTime[controlRoutine] = runTime;
        }
    }
    // Get IMU Data
    return true;
}

void getImuData(double* acc, double* gyro, double* temperature)
{
    // Get Raw Values
#ifdef SIMULATION
    if (pIMU)
    {
        for (int i=0; i<3; i++)
        {
            acc[i]  = pIMU->getAccelerometer()[i];
            gyro[i] = pIMU->getGyroscope()[i];
        }
    }
#else
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACC_OUT); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers
    
    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    *(acc+0)       = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    *(acc+1)       = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    *(acc+2)       = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    *(temperature) = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
    *(gyro+0)      = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    *(gyro+1)      = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
    *(gyro+2)      = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
#endif
    
    // Convert to Units
    for (int i=0; i<3; i++)
    {
        *(acc+i)  /= LSBg;
        *(gyro+i) /= LSBdps;
    }
    
    newIMUmeasurement = true;
}

bool groundCalibration(double* accBias, double* gyroBias, double* acc, double* gyro)
{
    static bool doneCalibrating = false;
    static int iSample   = 0;
    const int  maxSample = 256;
    
    static double accSum[3];
    static double gyroSum[3];
    
    if ( !doneCalibrating )
    {
        accSum[0]  += acc[0];
        accSum[1]  += acc[1];
        accSum[2]  += acc[2] - 1.0;
        gyroSum[0] += gyro[0];
        gyroSum[1] += gyro[1];
        gyroSum[2] += gyro[2];
        iSample++;

        if (iSample >= maxSample)
        {
            for (int i=0; i<3; i++)
            {
                *(accBias+i)  = accSum[i]/maxSample;
                *(gyroBias+i) = gyroSum[i]/maxSample;
            }
            printStatement("Done calibrating IMU\n");
        
            printStatement("accBias:\t");
            printStatement( *(accBias+0) );
            printStatement("\t");
            printStatement( *(accBias+1) );
            printStatement("\t");
            printStatement( *(accBias+2) );
            printStatement("\n");
    
            printStatement("gyroBias:\t");
            printStatement( *(gyroBias+0) );
            printStatement("\t");
            printStatement( *(gyroBias+1) );
            printStatement("\t");
            printStatement( *(gyroBias+2) );
            printStatement("\n");
        
            doneCalibrating = true;
        }
    }
    
    return doneCalibrating;
}

void atittudeFilter(double* attitude, double* acc, double* gyro)
{
    // Variables
    double sr;
    double cr;
    double sp;
    double tp;
    double secp;
    double eulerRates[3];
    
    double eulerPredGyro[3];
    double eulerPredAcc[3];
    double aMag;
    
    static double curTime;
    static double prevTime;
    
    prevTime = curTime;
    curTime = runTime;
    
    sr   = sin(attitude[0]);
    cr   = cos(attitude[0]);
    sp   = cos(attitude[1]);
    tp   = tan(attitude[1]);
    secp = 1.0/cos(attitude[1]);
    
    // Euler rates from gyroscope
    /*  -------------------------------------------------------------------------------
     * Body rates to euler rates matrix transformation
     *   euler_rates   =         L               x   body_rates
     *  [ roll_rate  ]   [ 1   sr*tp    cr*sp  ]    [ body_rate_x ]
     *  [ pitch_rate ] = [ 0    cr       -sr   ] x  [ body_rate_y ]
     *  [ yaw_rate   ]   [ 0  sr*secp  cr*secp ]    [ body_rate_z ]
     * -------------------------------------------------------------------------------
     */
    eulerRates[0] = ( gyro[0] - sr*tp*gyro[1]   -  cr*sp*gyro[2]   ) * deg2rad;
    eulerRates[1] = (         - cr*gyro[1]      +  sr*gyro[2]      ) * deg2rad;
    eulerRates[2] = (         - sr*secp*gyro[1] -  cr*secp*gyro[2] ) * deg2rad;
    
    // Use gyro to compute angles
    //eulerPredGyro[0] = attitude[0] + eulerRates[0] * routineDelays[attitudeFilterRoutine];
    //eulerPredGyro[1] = attitude[1] + eulerRates[1] * routineDelays[attitudeFilterRoutine];
    //eulerPredGyro[2] = attitude[2] + eulerRates[2] * routineDelays[attitudeFilterRoutine];
    eulerPredGyro[0] = attitude[0] + eulerRates[0] * (curTime-prevTime);
    eulerPredGyro[1] = attitude[1] + eulerRates[1] * (curTime-prevTime);
    eulerPredGyro[2] = attitude[2] + eulerRates[2] * (curTime-prevTime);
    
    // Use accelerometer to measure angles
    eulerPredAcc[0] = atan2( acc[1], acc[2] );
    eulerPredAcc[1] = atan2( acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2]) );
    
    // Acceleration magnitude
    aMag = sqrt( acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2] );
    
    // Estimate angles
    if (aMag < maxFilterAcc)
    {
        attitude[0]  = eulerPredGyro[0]  + compFilterGain * (eulerPredAcc[0] - eulerPredGyro[0]);
        attitude[1]  = eulerPredGyro[1]  + compFilterGain * (eulerPredAcc[1] - eulerPredGyro[1]);
    }
    else
    {
        attitude[0] = eulerPredGyro[0];
        attitude[1] = eulerPredGyro[1];
    }
    attitude[2]  = eulerPredGyro[2];
    
    for (int i=0; i<3; i++)
    {
        attitudeDeg[i] = attitude[i] / deg2rad;
    }
    
    if (printAngles)
    {
        printStatement("(");
        printStatement(runTime);
        printStatement(") ");
        printStatement("Roll Pitch Yaw: ");
        printStatement( attitude[0]/deg2rad );
        printStatement(" ");
        printStatement( attitude[1]/deg2rad );
        printStatement(" ");
        printStatement( attitude[2]/deg2rad );
        printStatement("\n");
    }
}

void attitudeControl(double* attitude)
{
    
}

void initializeVariables(void)
{
    for (int i=0; i<nRoutines; i++) { prevTime[i] = 0.0; }
}

void getSimulationModels(void)
{
# ifdef SIMULATION
    if(pMap)
    {
        pIMU  = (IMUModelBase*) pMap->getModel("IMUModel");
        pTime = (Time*)         pMap->getModel("Time");
    }
    else
    {
        printStatement("Warning: Could not find map models.");
        printStatement("\n");
    }
#endif
}

void setupIMU(void)
{
    LSBdps = gyroSensitivityLSB[gyroSensitivity];
    gyroSensitivityWrite = sensitivityByte[gyroSensitivity];
    
    LSBg   = accSensitivityLSB[accSensitivity];
    accSensitivityWrite = sensitivityByte[accSensitivity];
    
#ifdef SIMULATION
    if (pIMU)
    {
        pIMU->setLSBdps(LSBdps);
        pIMU->setLSBg(LSBg);
    }
#else
    Serial.begin(baudRate); //38400
    Wire.begin();
    
    Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(PWR_MGMT_1); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_REG); // Gryo register
    Wire.write(gyroSensitivityWrite); // Gyro to full range
    Wire.endTransmission(true);
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACC_REG); // Accelerometer register
    Wire.write(accSensitivityWrite); // Accelerometer to +/- 8g
    Wire.endTransmission(true);
#endif
}

void flightSoftware_setMapPointer(ModelMap* pMapInit)
{
    pMap = pMapInit;
    
    pMap->addLogVar("FS Acc X" , &accelerometerData[0], savePlot, 2);
    pMap->addLogVar("FS Acc Y" , &accelerometerData[1], savePlot, 2);
    pMap->addLogVar("FS Acc Z" , &accelerometerData[2], savePlot, 2);
    pMap->addLogVar("FS Gyro X", &gyroscopeData[0], savePlot, 2);
    pMap->addLogVar("FS Gyro Y", &gyroscopeData[1], savePlot, 2);
    pMap->addLogVar("FS Gyro X", &gyroscopeData[2], savePlot, 2);
    
    pMap->addLogVar("FS Acc X Bias" , &accBias[0], savePlot, 2);
    pMap->addLogVar("FS Acc Y Bias" , &accBias[1], savePlot, 2);
    pMap->addLogVar("FS Acc Z Bias" , &accBias[2], savePlot, 2);
    pMap->addLogVar("FS Gyro X Bias", &gyroBias[0], savePlot, 2);
    pMap->addLogVar("FS Gyro Y Bias", &gyroBias[1], savePlot, 2);
    pMap->addLogVar("FS Gyro X Bias", &gyroBias[2], savePlot, 2);
    
    pMap->addLogVar("FS Roll Cmd" , &desiredAttitude[0], savePlot, 2);
    pMap->addLogVar("FS Pitch Cmd", &desiredAttitude[1], savePlot, 2);
    pMap->addLogVar("FS Yaw Cmd"  , &desiredAttitude[2], savePlot, 2);
    
    pMap->addLogVar("FS Roll" , &attitudeDeg[0], printSavePlot, 3);
    pMap->addLogVar("FS Pitch", &attitudeDeg[1], printSavePlot, 3);
    pMap->addLogVar("FS Yaw"  , &attitudeDeg[2], printSavePlot, 3);
}

void printStatement(std::string val)
{
#ifdef SIMULATION
    std::cout << val;
#else
    Serial.print(val);
#endif
}

template<typename TempType>
void printStatement(TempType val)
{
#ifdef SIMULATION
    std::cout << val;
#else
    Serial.print(val);
#endif
}

template void printStatement(int);
template void printStatement(float);
template void printStatement(double);
