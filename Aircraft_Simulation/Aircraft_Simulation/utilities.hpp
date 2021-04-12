//
//  utilities.hpp
//  Utilities
//
//  Created by Alexander McLean on 6/5/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef utilities_hpp
#define utilities_hpp

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <termios.h>

// Mac key enum
enum keyType {
    rightArrow = -125, // -17 -100 -125
    leftArrow = -126,  // -17 -100 -126
    downArrow = -127,  // -17 -100 -127
    upArrow = -128,    // -17 -100 -128
    wKey = 119,
    aKey = 97,
    sKey = 115,
    dKey = 100
};

// distance enum
typedef enum DistanceListType
{inches, feet, yards, miles, nauticalmiles, millimeters, centimeters, meters, kilometers, numDistanceTypes} DistanceListType;

// speed enum
typedef enum SpeedListType
{feetPerSecond, milesPerHour, knots, metersPerSecond, kilometersPerHour, numSpeedTypes} SpeedListType;

// angle enum
typedef enum AngleListType
{radians, degrees, numAngleTypes} AngleListType;

typedef enum AngleRateListType
{radiansPerSecond, degreesPerSecond, RPM, numAngleRateTypes} AngleRateListType;

class UnitConversions {
public:
    // distance conversions
    const double in2m  = 1.0/39.37;  // inches to meters
    const double ft2m  = 1.0/3.2808; // feet to meters
    const double yd2m  = 0.9144;   // yards to meters
    const double mi2m  = 1609.344; // miles to meters
    const double nmi2m = 1852;     // nautical miles to meters
    const double mm2m  = 0.001;    // millimeters to meters
    const double cm2m  = 0.01;     // centimeters to meters
    const double km2m  = 1000;     // kilometers to meters
    
    // speed conversions
    const double fps2mps   = 1/3.2808;   // feet per second to meters per second
    const double mph2mps   = 0.44704;    // miles per hour to meters per second
    const double knots2mps = 0.5144444;  // knots (nautical miles per hour) to meters per second
    const double kph2mps   = 1/3.6;      // kilometers per hour to meters per second
    
    // angle conversions
    const double deg2rad = M_PI/180; // degrees to radians
    
    // angle rate conversions
    const double dps2rps = deg2rad;
    const double rpm2rps = 2*M_PI/60;
    
};

template<typename unitType, typename valType>
class Conversions : public UnitConversions {
    
public:

    valType val;
    
    // Accessors
    void convertUnit(unitType newUnit, bool convertValue = true )
    {
        if (convertValue) { val = performConversion(currentUnit, newUnit); }
        currentUnit = newUnit;
    }
    
    unitType getUnit(void) { return currentUnit; }
    
    const char* printUnit(void) { return printTemp[currentUnit]; }
    
protected:
    // Prevent using this class on its own
    //Conversions();
    
    // Variables
    unitType currentUnit;
    unitType desiredUnit;
    
    // Conversion arrays
    const double distanceArray[numDistanceTypes] = {in2m, ft2m, yd2m, mi2m, nmi2m, mm2m, cm2m, 1.0, km2m};
    const double speedArray[numSpeedTypes] = {fps2mps, mph2mps, knots2mps, 1.0, kph2mps};
    const double angleArray[numAngleTypes] = {1.0, deg2rad};
    const double angleRateArray[numAngleRateTypes] = {1.0, dps2rps, rpm2rps};
    const double *tempArray;
    
    // Conversion functions
    valType performConversion(unitType currentUnit, unitType desiredUnit)
    {
        return val * tempArray[currentUnit] / tempArray[desiredUnit];
    }
    
    // Converion printing
    const char *printDistance[numDistanceTypes] = {"in", "ft", "yd", "mi", "nmi", "mm", "cm", "m", "km"};
    const char *printSpeed[numSpeedTypes] = {"ft/s", "mi/h", "knots", "m/s", "km/h"};
    const char *printAngle[numAngleTypes] = {"rad", "deg"};
    const char *printAngleRate[numAngleRateTypes] = {"rad/s", "deg/s", "rpm"};
    const char **printTemp;
    
};

template<typename valType>
class DistanceType : public Conversions<DistanceListType, valType> {
    
public:
    
    using Conversions<DistanceListType,valType>::val;
    using Conversions<DistanceListType,valType>::currentUnit;
    using Conversions<DistanceListType,valType>::desiredUnit;
    using Conversions<DistanceListType,valType>::tempArray;
    using Conversions<DistanceListType,valType>::distanceArray;
    using Conversions<DistanceListType,valType>::printTemp;
    using Conversions<DistanceListType,valType>::printDistance;
    using Conversions<DistanceListType,valType>::performConversion;
    
    // Constructor
    DistanceType(valType initialValue = 0, DistanceListType initialUnit = meters)
    {
        val         = initialValue;
        currentUnit = initialUnit;
        tempArray   = distanceArray;
        printTemp   = &printDistance[0];
    }
    
    // Conversions
    valType in(void)
    {
        desiredUnit = inches;
        return performConversion(currentUnit, desiredUnit);
    }
    
    valType ft(void)
    {
        desiredUnit = feet;
        return performConversion(currentUnit, desiredUnit);
    }

    valType yd(void)
    {
        desiredUnit = yards;
        return performConversion(currentUnit, desiredUnit);
    }
 
    valType mi(void)
    {
        desiredUnit = miles;
        return performConversion(currentUnit, desiredUnit);
    }

    valType nmi(void)
    {
        desiredUnit = nauticalmiles;
        return performConversion(currentUnit, desiredUnit);
    }
    
    valType mm(void)
    {
        desiredUnit = millimeters;
        return performConversion(currentUnit, desiredUnit);
    }

    valType cm(void)
    {
        desiredUnit = centimeters;
        return performConversion(currentUnit, desiredUnit);
    }
    
    valType m(void)
    {
        desiredUnit = meters;
        return performConversion(currentUnit, desiredUnit);
    }
    
    valType km(void)
    {
        desiredUnit = kilometers;
        return performConversion(currentUnit, desiredUnit);
    }
    
};

template<typename valType>
class SpeedType : public Conversions<SpeedListType, valType> {
    
public:
    using Conversions<SpeedListType,valType>::val;
    using Conversions<SpeedListType,valType>::currentUnit;
    using Conversions<SpeedListType,valType>::desiredUnit;
    using Conversions<SpeedListType,valType>::tempArray;
    using Conversions<SpeedListType,valType>::speedArray;
    using Conversions<SpeedListType,valType>::printTemp;
    using Conversions<SpeedListType,valType>::printSpeed;
    using Conversions<SpeedListType,valType>::performConversion;
    
    // Constructor
    SpeedType(valType initialValue = 0, SpeedListType initialUnit = metersPerSecond)
    {
        val         = initialValue;
        currentUnit = initialUnit;
        tempArray   = speedArray;
        printTemp   = &printSpeed[0];
    }
    
    // Conversions
    valType fps(void)
    {
        desiredUnit = feetPerSecond;
        return performConversion(currentUnit, desiredUnit);
    }
    
    valType mph(void)
    {
        desiredUnit = milesPerHour;
        return performConversion(currentUnit, desiredUnit);
    }
    
    valType kts(void)
    {
        desiredUnit = knots;
        return performConversion(currentUnit, desiredUnit);
    }
    
    valType mps(void)
    {
        desiredUnit = metersPerSecond;
        return performConversion(currentUnit, desiredUnit);
    }
    
    valType kph(void)
    {
        desiredUnit = kilometersPerHour;
        return performConversion(currentUnit, desiredUnit);
    }
    
};

template<typename valType>
class AngleType : public Conversions<AngleListType, valType> {
    
 public:
    using Conversions<AngleListType,valType>::val;
    using Conversions<AngleListType,valType>::currentUnit;
    using Conversions<AngleListType,valType>::desiredUnit;
    using Conversions<AngleListType,valType>::tempArray;
    using Conversions<AngleListType,valType>::angleArray;
    using Conversions<AngleListType,valType>::printTemp;
    using Conversions<AngleListType,valType>::printAngle;
    using Conversions<AngleListType,valType>::performConversion;
    
    // Constructor
    AngleType(valType initialValue = 0, AngleListType initialUnit = degrees)
    {
        val         = initialValue;
        currentUnit = initialUnit;
        tempArray   = angleArray;
        printTemp   = &printAngle[0];
    }
    
    // Conversions
    valType rad(void)
    {
        desiredUnit = radians;
        return performConversion(currentUnit, desiredUnit);
    }
    
    valType deg(void)
    {
        desiredUnit = degrees;
        return performConversion(currentUnit, desiredUnit);
    }
    
};

template<typename valType>
class AngleRateType : public Conversions<AngleRateListType, valType> {
    
public:
    using Conversions<AngleRateListType,valType>::val;
    using Conversions<AngleRateListType,valType>::currentUnit;
    using Conversions<AngleRateListType,valType>::desiredUnit;
    using Conversions<AngleRateListType,valType>::tempArray;
    using Conversions<AngleRateListType,valType>::angleRateArray;
    using Conversions<AngleRateListType,valType>::printTemp;
    using Conversions<AngleRateListType,valType>::printAngleRate;
    using Conversions<AngleRateListType,valType>::performConversion;
    
    // Constructor
    AngleRateType(valType initialValue = 0, AngleRateListType initialUnit = degreesPerSecond)
    {
        val         = initialValue;
        currentUnit = initialUnit;
        tempArray   = angleRateArray;
        printTemp   = &printAngleRate[0];
    }
    
    // Conversions
    valType rps(void)
    {
        desiredUnit = radiansPerSecond;
        return performConversion(currentUnit, desiredUnit);
    }
    
    valType dps(void)
    {
        desiredUnit = degreesPerSecond;
        return performConversion(currentUnit, desiredUnit);
    }

    valType rpm(void)
    {
        desiredUnit = RPM;
        return performConversion(currentUnit, desiredUnit);
    }
    
};

class MonitorMacInput {
public:
    // Constructor
    MonitorMacInput(void);
    
    // Deconstructor
    ~MonitorMacInput(void);
    
    // Monitor input
    int monitorInput(void);
    
private:
    struct termios oldSettings; // current Mac terminal settings
    struct termios newSettings; // new Mac terminal settings
    bool error;
};

class Delay {
public:
    // Constructor
    Delay(int *pCounter, float counterRate, float sDelay, bool debug = false);
    
    // Functions
    template<typename TempType>
    void delayValue(TempType *value, TempType newValue);
    
    // Getters
    bool getChangeApplied(void) { return changeApplied; }
    
    // Setters
    void setDelay(float inputDelay) { sDelay = inputDelay; }
    void setRate(float inputRate)   { counterRate = inputRate; }
    
private:
    // Settings
    int   *pCounter;     // pointer to counter
    float counterRate;   // rate of counter
    float sDelay;        // delay between changes in seconds
    
    // Internal variables
    bool  timerStarted;  // variable change sensed and time is counting
    bool  changeApplied; // variable change has been applied
    float timer;         // amount of time in current state
    int   countStart;    // counter value when timer started
    
    bool debug;
    
};

class FirstOrderFilter {
public:
    // xcmd/x = k/(tau*s + 1)
    FirstOrderFilter(double x_in = 0.0, double dt_in = 0.0, double tau_in = 0.001, double gain_in = 1.0)
    {
        dt   = dt_in;
        tau  = tau_in;
        k    = gain_in;
        
        xprev = x_in;
        x     = x_in;
        xdot  = 0.0;
        maxrate = std::numeric_limits<double>::infinity();
    }
    
    void setInitialValue(double x_in)   { x = x_in; }
    void setTimeConstant(double tau_in) { tau = tau_in; }
    void setGain(double gain_in)        { k = gain_in; }
    void setDt(double dt_in)            { dt = dt_in; }
    void setMaxRate(double maxrate_in)  { maxrate = maxrate_in; }
    
    void setValue(double xcmd)
    {
        xprev = x;
        if (tau != 0) { xdot  = (k*xcmd - x)/tau; }
        else          { xdot = 0;}
        
        if (xdot > maxrate)  { xdot = maxrate; }
        if (xdot < -maxrate) { xdot = -maxrate; }
        
        x  = xprev + xdot*dt;
        //std::cout<<xcmd<<std::endl;
        //std::cout<<x<<"="<<xprev<<"+"<<xdot<<"*"<<dt<<std::endl;
    }
    
    // Getters and Setters
    
    double getFilterValue(void) { return x; }
    double getFilterRate(void)  { return xdot; }
    
private:
    double xprev;
    double x;
    double xdot;
    
    double dt;
    double tau;
    double k;
    double maxrate;
};

class MassUtilities {
    
public:
    //MassUtilities::MassUtilities
    
};

class Utilities : public UnitConversions {
    
public:
    // Constants
    const double zeroTolerance = 0.0001;
    
    // Print statements
    template<typename TempType>
    void print(TempType *array, int arrayLength);

    template<typename TempType>
    void print(TempType *array, int arrayLength, std::string name);
    
    template<typename TempType>
    void print(TempType *array, int arrayLength, const char *name);
    
    template<typename valType, template<typename T> class unitType>
    void print(unitType<valType> *array, int arrayLength);
    
    template<typename valType, typename unitListType, template<typename T> class unitType>
    void print(unitType<valType> *array, unitListType unit, int arrayLength);
    
    template<typename valType, template<typename T> class unitType>
    void print(unitType<valType> *array, int arrayLength, const char *name);

    template<typename valType, typename unitListType, template<typename T> class unitType>
    void print(unitType<valType> *array, unitListType unit, int arrayLength, const char *name);
    
    template<typename TempType>
    void print(TempType *matrix, int nrows, int ncols);
    
    template<typename TempType>
    void print(TempType *matrix, int nrows, int ncols, const char *name);
    
    // Array initialization
    template<typename TempType>
    void initArray(TempType *array, TempType val, int arrayLength);
    
    template<typename TempType>
    void initMatrix(TempType *matrix, TempType val, int nrow, int ncol);
    
    template<typename TempType>
    void setArray(TempType *array, TempType *setArray, int arrayLength);
  
    template<typename TempType>
    void setArray(TempType *array, const TempType *setArray, int arrayLength);
    
    template<typename valType, template<typename T> class unitType>
    void setArray(valType *array, unitType<valType> *setArray, int arrayLength);
    
    template<typename valType, template<typename T> class unitType>
    void setArray(unitType<valType> *array, unitType<valType> *setArray, int arrayLength);
    
    template<typename valType, typename unitListType, template<typename T> class unitType>
    void setUnitClassUnit(unitType<valType> *unitClass, unitListType unit, int arrayLength);
    
    template<typename valType, typename unitListType, template<typename T> class unitType>
    void setUnitClassArray(unitType<valType> *unitClass, valType *setArray, unitListType initUnit, int arrayLength);
  
    template<typename valType, typename unitListType, template<typename T> class unitType>
    void setUnitClassArray(unitType<valType> *unitClass, const valType *setArray, unitListType initUnit, int arrayLength);
    
    template<typename TempType>
    void setMatrix(TempType *matrix, const TempType *setMat, int nrow, int ncol);
    
    // Booleans
    bool any(bool *boolArray, int lengthArray);
    
    bool all(bool *boolArray, int lengthArray);
    
    // Statistics
    template<typename TempType>
    TempType max(TempType val1, TempType val2);
    
    template<typename TempType>
    TempType max(TempType *array, int arrayLength);

    template<class TempType>
    TempType min(TempType val1, TempType val2);

    template<typename TempType>
    TempType min(TempType *array, int arrayLength);
    
    template<typename TempType>
    TempType mean(TempType *array, int arrayLength);
    
    template<typename TempType>
    TempType stdev(TempType *array, int arrayLength);
    
    // Vector math
    template<typename TempType>
    void vAdd(TempType *result, TempType *array1, TempType *array2, int arrayLength);
    
    template<typename valType, template<typename T> class unitType>
    void vAdd(unitType<valType> *result, unitType<valType> *array1, valType *array2, int arrayLength);
    
    template<typename valType, template<typename T> class unitType>
    void vAdd(unitType<valType> *result, unitType<valType> *array1, unitType<valType> *array2, int arrayLength);
    
    template<typename TempType>
    void vSubtract(TempType *result, TempType *array1, TempType *array2, int arrayLength);
 
    template<typename valType, template<typename T> class unitType>
    void vSubtract(unitType<valType> *result, unitType<valType> *array1, unitType<valType> *array2, int arrayLength);
    
    template<typename TempType>
    void crossProduct(TempType *cross, TempType *a, TempType *b);
 
    template
    <typename valType,
    template<typename T1> class unitType1,
    template<typename T2> class unitType2,
    template<typename T3> class unitType3>
    void crossProduct(unitType1<valType> *cross, unitType2<valType> *a, unitType3<valType> *b); //#TODO

    template <typename valType, template<typename T1> class unitType>
    void crossProduct(valType *cross, unitType<valType> *a, valType *b);
  
    template <typename valType, template<typename T1> class unitType>
    void crossProduct(valType *cross, valType *a, unitType<valType> *b);
  
    template
    <typename valType,
    template<typename T1> class unitType1,
    template<typename T2> class unitType2>
    void crossProduct(valType *cross, unitType1<valType> *a, unitType2<valType> *b);
    
    template<typename TempType>
    TempType dotProduct(TempType *a, TempType *b, int n);
  
    template<typename valType, template<typename T> class unitType>
    valType dotProduct(unitType<valType> *a, unitType<valType> *b, int n); //#TODO
    
    template<typename TempType>
    TempType mag(TempType *vec, int n);
 
    template<typename valType, template<typename T> class unitType>
    valType mag(unitType<valType> *vec, int n); //#TODO
    
    template<typename TempType>
    void unitVector(TempType *vec, int n);
 
    template<typename valType, template<typename T> class unitType>
    void unitVector(unitType<valType> *vec, int n); //#TODO
    
    template<typename TempType>
    void vgain(TempType *vec, TempType gain, int n);
 
    template<typename valType, template<typename T> class unitType>
    void vgain(unitType<valType> *vec, valType gain, int n); //#TODO
    
    template<typename TempType>
    TempType interpolate(TempType *xvec, TempType *yvec, TempType x, int n, bool extrapolate = false, bool print = false);
    
    template<typename TempType>
    TempType interpolate(const TempType *xvec, const TempType *yvec, TempType x, int n, bool extrapolate = false, bool print = false);
    
    // Matrix math
    template<typename TempType>
    void mmult(TempType *result, TempType *matrix, TempType *vector, int nrow, int ncol);

    template<typename valType, template<typename T> class unitType>
    void mmult(valType *result, valType *matrix, unitType<valType> *vector, int nrow, int ncol);
    
    template<typename valType, template<typename T> class unitType>
    void mmult(unitType<valType> *result, valType *matrix, unitType<valType> *vector, int nrow, int ncol);
    
    template<typename TempType>
    void mmult(TempType *result, TempType *A, int nrows1, int ncols1, TempType *B, int nrows2, int ncols2);
    
    template<typename TempType>
    void mtran(TempType *matrix_t, TempType *matrix, int nrow_t, int ncol_t);
    
    template<typename TempType>
    void mtran(TempType *matrix_t, const TempType *matrix, int nrow_t, int ncol_t);
    
    template<typename TempType>
    void mgain(TempType *matrix, TempType gain, int nrow, int ncol);
    
    template<typename TempType>
    void LUdecomp(TempType *x, TempType *A, TempType *b, int n);
    
    // Euler math
    template<typename TempType>
    void setupRotation(TempType *R, TempType *euler_angles);

    template<typename valType>
    void setupRotation(valType *R, AngleType<valType> *euler_angles);
    
    template<typename TempType>
    void setupEulerRateToBodyRate(TempType *L, TempType *euler_angles);

    template<typename valType>
    void setupEulerRateToBodyRate(valType *L, AngleType<valType> *euler_angles);
    
    template<typename TempType>
    void setupBodyRateToEulerRate(TempType *L, TempType *euler_angles);
    
    template<typename valType>
    void setupBodyRateToEulerRate(valType *L, AngleType<valType> *euler_angles);
    
    // Quaternion math
    template<typename TempType>
    void initQuaternion(TempType *q, TempType angle, TempType *axis);
    
    template<typename TempType>
    void quatToVec(TempType *vec, TempType *q);
    
    template<typename valType, template<typename T> class unitType>
    void quatToVec(unitType<valType> *vec, valType *q);
    
    template<typename TempType>
    void vecToQuat(TempType *q, TempType *vec);
  
    template<typename valType, template<typename T> class unitType>
    void vecToQuat(valType *q, unitType<valType> *vec);
    
    template<typename TempType>
    void quaternionProduct(TempType *product, TempType *q1, TempType *q2);
    
    template<typename TempType>
    void quaternionConjugate(TempType *q_conj, TempType *q);
    
    template<typename TempType>
    void quaternionRotation(TempType *result, TempType *q, TempType *v);
    
    template<typename TempType>
    void quaternionTransformation(TempType *result, TempType *quaternion, TempType *v);
    
    template<typename valType, template<typename T> class unitType>
    void quaternionTransformation(unitType<valType> *result, valType *quaternion, unitType<valType> *v);
  
    template<typename valType, template<typename T> class unitType>
    void quaternionTransformation(valType *result, valType *quaternion, unitType<valType> *v);
    
    template<typename TempType>
    void eulerToQuaternion(TempType *q, TempType *euler);

    template<typename valType>
    void eulerToQuaternion(valType *q, AngleType<valType> *euler);
    
    template<typename TempType>
    void quaternionToEuler(TempType *euler, TempType *q);
    
    template<typename valType>
    void quaternionToEuler(AngleType<valType> *euler, valType *q);
};

#endif /* utilities_hpp */