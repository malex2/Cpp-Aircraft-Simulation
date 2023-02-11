//
//  utilities.cpp
//  Utilities
//
//  Created by Alexander McLean on 6/5/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "utilities.hpp"

MonitorMacInput::MonitorMacInput(void)
{
    error = false;
    
    // get old settings
    tcgetattr(0, &oldSettings);
    
    // configure terminal
    newSettings = oldSettings;
    
    newSettings.c_lflag &= ~ICANON; // disable line-at-a-time input
    newSettings.c_lflag &= ~ECHO;   // disable echo
    newSettings.c_cc[VMIN]  = 0;    // don't wait for characters
    newSettings.c_cc[VTIME] = 0;    // minimum wait time
    
    // check for errors
    if ( tcsetattr(0, TCSAFLUSH, &newSettings) != 0)
    {
        std::cout << "error code: " << stderr;
        std::cout << "Unable to set terminal mode" << std::endl;
        error = true;
    }
}

MonitorMacInput::~MonitorMacInput(void)
{
    // restore old settings
    tcsetattr( 0, TCSAFLUSH, &oldSettings );
}

int MonitorMacInput::monitorInput(void)
{
    if (!error)
    {
        char input;
    
        // return '\0' when no character available
        if ( read(STDIN_FILENO, &input, 1) == 0 ) { return '\0'; }
        else { /*tcflush(STDIN_FILENO, TCIOFLUSH);*/ return input; }
    }
    return '\0';
}

Delay::Delay(int *pCounterIn, float counterRateIn, float sDelayIn, bool debugIn)
{
    pCounter    = pCounterIn;
    counterRate = counterRateIn;
    sDelay      = sDelayIn;
    debug       = debugIn;
    
    timerStarted  = false;
    changeApplied = false;
}

template<typename TempType>
void Delay::delayValue(TempType *value, TempType newValue)
{
    // Change sensed
    if ( (newValue) != (*value) )
    {
        // start timer
        if (!timerStarted)
        {
            countStart    = *pCounter;
            timer         = 0;
            changeApplied = false;
            timerStarted  = true;
            if (debug)
            {
                std::cout << "Change sensed (" << countStart << "): " << *value;
                std::cout << " to "<< newValue;
                std::cout << " (delay: " << sDelay << "s)" << std::endl;
            }
        }
        else
        {
            timer  = (float) (*pCounter - countStart)/(counterRate);
            if (debug) { std::cout << "timer: " << timer << " (" << *pCounter << ")" << std::endl; }
        }
        
        // Apply value change after delay passed
        if (timer >= sDelay)
        {
            (*value) = newValue;
            if (debug)
            {
                std::cout << *value << " applied";
                std::cout << " (delay: " << sDelay << "s)" << std::endl;
            }
            changeApplied = true;
            timerStarted  = false;
        }
    }
    
    // No change, stop timer
    else
    {
        timerStarted  = false;
        changeApplied = false;
    }
}


/* -------------------- General -------------------- */
template<typename TempType>
void Utilities::print(TempType *array, int arrayLength, std::string name)
{
   std::cout << name << std::endl;
    
    for (int i = 0; i < arrayLength; i++)
    {
        std::cout << array[i] << " ";
    }
    std::cout << std::endl << std::endl;
}

template<typename TempType>
void Utilities::print(TempType *array, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        std::cout << array[i] << " ";
    }
    std::cout << std::endl << std::endl;
}

template<typename TempType>
void Utilities::print(TempType *array, int arrayLength, const char *name)
{
    if (name != nullptr ) { std::cout << name << std::endl; }
    
    for (int i = 0; i < arrayLength; i++)
    {
        std::cout << array[i] << " ";
    }
    std::cout << std::endl << std::endl;
}

template<typename valType, template<typename T> class unitType>
void Utilities::print(unitType<valType> *array, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        std::cout << array[i].val << " " << array[i].printUnit() << " ";
    }
    std::cout << std::endl << std::endl;
}

template<typename valType, typename unitListType, template<typename T> class unitType>
void Utilities::print(unitType<valType> *array, unitListType unit, int arrayLength)
{
    unitListType oldUnit;
    
    for (int i = 0; i < arrayLength; i++)
    {
        oldUnit = array[i].getUnit();
        array[i].convertUnit(unit);
        std::cout << array[i].val << " " << array[i].printUnit() << " ";
        array[i].convertUnit(oldUnit);
    }
    std::cout << std::endl << std::endl;
}

template<typename valType, template<typename T> class unitType>
void Utilities::print(unitType<valType> *array, int arrayLength, const char *name)
{
    if (name != nullptr ) { std::cout << name << std::endl; }
    
    for (int i = 0; i < arrayLength; i++)
    {
        std::cout << array[i].val << " " << array[i].printUnit() << " ";
    }
    std::cout << std::endl << std::endl;
}

template<typename valType, typename unitListType, template<typename T> class unitType>
void Utilities::print(unitType<valType> *array, unitListType unit, int arrayLength, const char *name)
{
    unitListType oldUnit;
    
    if (name != nullptr ) { std::cout << name << std::endl; }
    
    for (int i = 0; i < arrayLength; i++)
    {
        oldUnit = array[i].getUnit();
        array[i].convertUnit(unit);
        std::cout << array[i].val << " " << array[i].printUnit() << " ";
        array[i].convertUnit(oldUnit);
    }
    std::cout << std::endl << std::endl;
}

template<typename TempType>
void Utilities::print(TempType *matrix, int nrows, int ncols)
{
    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < ncols; j++)
        {
            std::cout << *(matrix+i*ncols+j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

template<typename TempType>
void Utilities::print(TempType *matrix, int nrows, int ncols, const char *name)
{
    if (name != nullptr ) { std::cout << name << std::endl; }
    
    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < ncols; j++)
        {
            std::cout << *(matrix+i*ncols+j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

template<typename TempType>
void Utilities::initArray(TempType *array, TempType val, int arrayLength)
{
    for (int i = 0; i <arrayLength; i++)
    {
        array[i] = val;
    }
}

template<typename TempType>
void Utilities::setArray(TempType *array, TempType *setArray, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        array[i] = setArray[i];
    }
}

template<typename TempType>
void Utilities::setArray(TempType *array, const TempType *setArray, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        array[i] = setArray[i];
    }
}

template<typename valType, template<typename T> class unitType>
void Utilities::setArray(valType *array, unitType<valType> *setArray, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        array[i] = setArray[i].val;
    }
}

template<typename valType, template<typename T> class unitType>
void Utilities::setArray(unitType<valType> *array, unitType<valType> *setArray, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        array[i].val = setArray[i].val;
    }
}

template<typename valType, typename unitListType, template<typename T> class unitType>
void Utilities::setUnitClassUnit(unitType<valType> *unitClass, unitListType unit, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        unitClass[i].convertUnit(unit);
    }
}

template<typename valType, typename unitListType, template<typename T> class unitType>
void Utilities::setUnitClassArray(unitType<valType> *unitClass, valType *setArray, unitListType initUnit, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        unitClass[i].val = setArray[i];
        unitClass[i].convertUnit(initUnit, false);
    }
}

template<typename valType, typename unitListType, template<typename T> class unitType>
void Utilities::setUnitClassArray(unitType<valType> *unitClass, const valType *setArray, unitListType initUnit, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        unitClass[i].val = setArray[i];
        unitClass[i].convertUnit(initUnit, false);
    }
}

template<typename TempType>
void Utilities::initMatrix(TempType *matrix, TempType val, int nrow, int ncol)
{
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < ncol; j++)
        {
            *(matrix + i*ncol + j) = val; // matrix[i][j]
        }
        
    }
}

template<typename TempType>
void Utilities::setMatrix(TempType *matrix, const TempType *setMat, int nrow, int ncol)
{
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < ncol; j++)
        {
            *(matrix+i*ncol+j) = *(setMat+i*ncol+j);
        }
    }
}


/* -------------------- Statistics -------------------- */
template<typename TempType>
TempType Utilities::max(TempType val1, TempType val2)
{
    if (val1 >= val2) { return val1; }
    else              { return val2; }
}

template<typename TempType>
TempType Utilities::max(TempType *array, int arrayLength)
{
    TempType maximum = -std::numeric_limits<TempType>::max();
    for (int i = 0; i < arrayLength; i++)
    {
        if (array[i] > maximum) { maximum = array[i]; }
    }
    return maximum;
}

template<typename TempType>
TempType Utilities::min(TempType val1, TempType val2)
{
    if (val1 <= val2) { return val1; }
    else              { return val2; }
}

template<typename TempType>
TempType Utilities::min(TempType *array, int arrayLength)
{
    TempType minimum = std::numeric_limits<TempType>::max();
    for (int i = 0; i < arrayLength; i++)
    {
        if (array[i] < minimum) { minimum = array[i]; }
    }
    return minimum;
}

template<typename TempType>
TempType Utilities::mean(TempType *array, int arrayLength)
{
    // Initialize variables
    TempType avg;
    TempType sum = 0;
    
    if (arrayLength > 1)
    {
        // Compute sum of all values
        for(int i = 0; i < arrayLength; i++){
            sum += array[i];
        }
        
        // Compute mean
        avg = sum/arrayLength;
    }
    
    else { avg = array[0]; }
    
    return avg;
}

template<typename TempType>
TempType Utilities::stdev(TempType *array, int arrayLength)
{
    // Initialize variables
    TempType variance, avg, std, sum = 0;
    
    if (arrayLength > 1)
    {
        // Compute mean
        avg = mean(array, arrayLength);
        
        // Compute standard deviation
        for(int i = 0; i < arrayLength; i++){
            sum += (array[i] - avg)*(array[i] - avg);
        }
        
        variance = sum/(arrayLength-1.0);
        
        std = sqrt(variance);
    }
    else {std = 0;}
    
    return std;
}

bool Utilities::any(bool *boolArray, int lengthArray)
{
    bool flag = false;
    
    for (int i = 0; i < lengthArray; i++)
    {
        flag = flag || boolArray[i];
    }
    
    return flag;
}

bool Utilities::all(bool *boolArray, int lengthArray)
{
    bool flag = true;
    
    for (int i = 0; i < lengthArray; i++)
    {
        flag = flag && boolArray[i];
    }
    
    return flag;
}

/* -------------------- Vector Math -------------------- */
template<typename TempType>
void Utilities::vAdd(TempType *result, TempType *array1, TempType *array2, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        result[i] = array1[i] + array2[i];
    }
}

template<typename valType, template<typename T> class unitType>
void Utilities::vAdd(unitType<valType> *result, unitType<valType> *array1, valType *array2, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        result[i].val = array1[i].val + array2[i];
        
        if ( result[i].getUnit() != array1[i].getUnit() )
        {
            printf("Utilities::vAdd - Warning, result[%d] unit of %s does not match array1[%d] unit of %s\n",i, result[i].printUnit(), i, array1[i].printUnit());
        }
    }
}

template<typename valType, template<typename T> class unitType>
void Utilities::vAdd(unitType<valType> *result, unitType<valType> *array1, unitType<valType> *array2, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        result[i].val = array1[i].val + array2[i].val;
        
        if ( array1[i].getUnit() != array2[i].getUnit() )
        {
            printf("Utilities::vAdd - Warning, array1[%d] unit of %s does not match array2[%d] unit of %s\n",i, array1[i].printUnit(), i, array2[i].printUnit());
        }
        
        if ( result[i].getUnit() != array1[i].getUnit() )
        {
            printf("Utilities::vAdd - Warning, result[%d] unit of %s does not match array1[%d] unit of %s\n",i, result[i].printUnit(), i, array1[i].printUnit());
        }
        
        if ( result[i].getUnit() != array2[i].getUnit() )
        {
            printf("Utilities::vAdd - Warning, result[%d] unit of %s does not match array2[%d] unit of %s\n",i, result[i].printUnit(), i, array2[i].printUnit());
        }
    }
}

template<typename TempType>
void Utilities::vSubtract(TempType *result, TempType *array1, TempType *array2, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        result[i] = array1[i] - array2[i];
    }
}

template<typename valType, template<typename T> class unitType>
void Utilities::vSubtract(unitType<valType> *result, unitType<valType> *array1, unitType<valType> *array2, int arrayLength)
{
    for (int i = 0; i < arrayLength; i++)
    {
        result[i].val = array1[i].val - array2[i].val;
        
        if ( array1[i].getUnit() != array2[i].getUnit() )
        {
            printf("Utilities::vSubtract - Warning, array1[%d] unit of %s does not match array2[%d] unit of %s\n",i, array1[i].printUnit(), i, array2[i].printUnit());
        }
        
        if ( result[i].getUnit() != array1[i].getUnit() )
        {
            printf("Utilities::vSubtract - Warning, result[%d] unit of %s does not match array1[%d] unit of %s\n",i, result[i].printUnit(), i, array1[i].printUnit());
        }
        
        if ( result[i].getUnit() != array2[i].getUnit() )
        {
            printf("Utilities::vAdd - Warning, result[%d] unit of %s does not match array2[%d] unit of %s\n",i, result[i].printUnit(), i, array2[i].printUnit());
        }
    }
}

template<typename TempType>
void Utilities::crossProduct(TempType *cross, TempType *a, TempType *b)
{
    *(cross+0) = a[1]*b[2] - a[2]*b[1];
    *(cross+1) = a[2]*b[0] - a[0]*b[2];
    *(cross+2) = a[0]*b[1] - a[1]*b[0];
}

template <typename valType, template<typename T1> class unitType>
void Utilities::crossProduct(valType *cross, unitType<valType> *a, valType *b)
{
    *(cross+0) = a[1].val*b[2] - a[2].val*b[1];
    *(cross+1) = a[2].val*b[0] - a[0].val*b[2];
    *(cross+2) = a[0].val*b[1] - a[1].val*b[0];
}

template
<typename valType,
template<typename T1> class unitType1,
template<typename T2> class unitType2,
template<typename T3> class unitType3>
void Utilities::crossProduct(unitType1<valType> *cross, unitType2<valType> *a, unitType3<valType> *b)
{
    cross[0].val = a[1].val*b[2].val  - a[2].val*b[1].val;
    cross[1].val = a[2].val*b[0].val  - a[0].val*b[2].val;
    cross[2].val = a[0].val*b[1].val  - a[1].val*b[0].val;
}

template <typename valType, template<typename T1> class unitType>
void Utilities::crossProduct(valType *cross, valType *a, unitType<valType> *b)
{
    *(cross+0) = a[1]*b[2].val - a[2]*b[1].val;
    *(cross+1) = a[2]*b[0].val - a[0]*b[2].val;
    *(cross+2) = a[0]*b[1].val - a[1]*b[0].val;
}

template
<typename valType,
template<typename T1> class unitType1,
template<typename T2> class unitType2>
void Utilities::crossProduct(valType *cross, unitType1<valType> *a, unitType2<valType> *b)
{
    cross[0] = a[1].val*b[2].val  - a[2].val*b[1].val;
    cross[1] = a[2].val*b[0].val  - a[0].val*b[2].val;
    cross[2] = a[0].val*b[1].val  - a[1].val*b[0].val;
}

template<typename TempType>
TempType Utilities::dotProduct(TempType *a, TempType *b, int n)
{
    TempType dot = 0;
    for (int i = 0; i < n; i++)
    {
        dot += a[i]*b[i];
    }
    
    return dot;
}

template<typename TempType>
TempType Utilities::mag(TempType *vec, int n)
{
    TempType mag = 0;
    for (int i = 0; i <n; i++)
    {
        mag += vec[i]*vec[i];
    }
    mag = sqrt(mag);
    
    return mag;
}

template<typename valType, template<typename T> class unitType>
valType Utilities::mag(unitType<valType> *vec, int n)
{
    valType mag = 0;
    for (int i = 0; i <n; i++)
    {
        mag += vec[i].val*vec[i].val;
    }
    mag = sqrt(mag);
    
    return mag;
}

template<typename TempType>
void Utilities::unitVector(TempType *vec, int n)
{
    // Get magnitude
    TempType mag = Utilities::mag(vec, n);
    
    if (mag == 0) return;
    
    // Make vec a unit vector
    for (int i = 0; i < n; i++)
    {
        vec[i] = vec[i]/mag;
    }
}

template<typename TempType>
void Utilities::vgain(TempType *vec, TempType gain, int n)
{
    for (int i = 0; i < n; i++)
    {
        *(vec+i) *= gain;
    }
}

template<typename TempType>
TempType Utilities::interpolate(TempType *xvec, TempType *yvec, TempType x, int n, bool extrapolate, bool print)
{
    int itr = -1;
    
    TempType y;
    TempType slope;
    TempType x1, x2, y1, y2;
    
    enum range {above, below, good};
    range lookupRange = good;
    
    bool cont = true;
    
    if (x < xvec[0])
    {
        cont = false;
        itr = 1;
        if (print) { std::cout << "Warning: " << x << " is below range" << std::endl; }
        lookupRange = below;
    }
    
    // Find where x falls in xvec
    while (cont)
    {
        itr ++;
        if (x <= xvec[itr]) { cont = false; }
        else if (itr > n-1)
        {
            cont = false;
            itr  = n-1;
            if (print) { std::cout << "Warning: " << x << " is above range" << std::endl; }
            lookupRange = above;
            
        }
    }
    
    if (lookupRange == below && !extrapolate)
    {
        y = yvec[0];
    }
    
    else if (lookupRange == above && !extrapolate)
    {
        y = yvec[n-1];
    }
    
    else
    {
        // Bound range (x1, y1), (x2, y2)
        x1 = xvec[itr-1];
        x2 = xvec[itr];
        y1 = yvec[itr-1];
        y2 = yvec[itr];
    
        // (y - y1) = m(x - x1)
        slope = (y2 - y1)/(x2 - x1);
        y = slope * (x - x1) + y1;
    }
    
    return y;
}

template<typename TempType>
TempType Utilities::interpolate(const TempType *xvec, const TempType *yvec, TempType x, int n, bool extrapolate, bool print)
{
    int itr = -1;
    
    TempType y;
    TempType slope;
    TempType x1, x2, y1, y2;
    
    enum range {above, below, good};
    range lookupRange = good;
    
    bool cont = true;
    
    if (x < xvec[0])
    {
        cont = false;
        itr = 1;
        if (print) { std::cout << "Warning: " << x << " is below range" << std::endl; }
        lookupRange = below;
    }
    
    // Find where x falls in xvec
    while (cont)
    {
        itr ++;
        if (x <= xvec[itr]) { cont = false; }
        else if (itr > n-1)
        {
            cont = false;
            itr  = n-1;
            if (print) { std::cout << "Warning: " << x << " is above range" << std::endl; }
            lookupRange = above;
            
        }
    }
    
    if (lookupRange == below && !extrapolate)
    {
        y = yvec[0];
    }
    
    else if (lookupRange == above && !extrapolate)
    {
        y = yvec[n-1];
    }
    
    else
    {
        // Bound range (x1, y1), (x2, y2)
        x1 = xvec[itr-1];
        x2 = xvec[itr];
        y1 = yvec[itr-1];
        y2 = yvec[itr];
        
        // (y - y1) = m(x - x1)
        slope = (y2 - y1)/(x2 - x1);
        y  = slope * (x - x1) + y1;
    }
    
    return y;
}

/* --------------------  Polynomial Math -------------------- */
template<typename TempType>
bool Utilities::solveQuadratic(TempType a, TempType b, TempType c, TempType* solnReal, TempType* solnImag)
{
    // solnReal[0] & solnImag[0] is + solution
    // solnReal[0] & solnImag[0] is - solution
    
    bool realSolution = true;
    TempType sqrtVal;
    TempType part1;
    TempType part2;
    
    // Part 1: -b/(2*a)
    // Part 2: sqrt(b^2 - 4*a*c)/(2*a)
    part1 = -b/(2*a);
    sqrtVal = b*b - 4*a*c;
    if (sqrtVal < 0)
    {
        part2 = sqrt(-sqrtVal)/(2*a);
        realSolution = false;
    }
    else
    {
        part2 = sqrt(sqrtVal)/(2*a);
    }
    
    if (realSolution)
    {
        solnReal[0] = part1 + part2;
        solnReal[1] = part1 - part2;
        solnImag[0] = 0.0;
        solnImag[1] = 0.0;
    }
    else
    {
        solnReal[0] = part1;
        solnReal[1] = part1;
        solnImag[0] = part2;
        solnImag[1] = -part2;
    }
    
    return realSolution;
}

/* -------------------- Matrix Math -------------------- */
template<typename TempType>
void Utilities::mmult(TempType* result, TempType* matrix, TempType* vector, int nrow, int ncol)
{
    for(int i = 0; i < nrow; i++)
    {
        // zero sum
        *(result+i) = 0;
        
        //result[i] = sum( matrix[i][1:n] )*vector[j]
        for(int j = 0; j < ncol; j++)
        {
            TempType matrix_ij = *(matrix + i*ncol + j);// matrix[i][j]
            TempType vector_j  = *(vector + j);         // vector[j]
            *(result+i) += matrix_ij*vector_j;
        }
    }
}

template<typename valType, template<typename T> class unitType>
void Utilities::mmult(valType *result, valType *matrix, unitType<valType> *vector, int nrow, int ncol)
{
    for(int i = 0; i < nrow; i++)
    {
        // zero sum
        *(result+i) = 0;
        
        //result[i] = sum( matrix[i][1:n] )*vector[j]
        for(int j = 0; j < ncol; j++)
        {
            valType matrix_ij = *(matrix + i*ncol + j);// matrix[i][j]
            valType vector_j  = vector[j].val;         // vector[j]
            *(result+i) += matrix_ij*vector_j;
        }
    }
}

template<typename valType, template<typename T> class unitType>
void Utilities::mmult(unitType<valType>  *result, valType *matrix, unitType<valType> *vector, int nrow, int ncol)
{
    for(int i = 0; i < nrow; i++)
    {
        // zero sum
        result[i].val = 0;
        
        //result[i] = sum( matrix[i][1:n] )*vector[j]
        for(int j = 0; j < ncol; j++)
        {
            valType matrix_ij = *(matrix + i*ncol + j);// matrix[i][j]
            valType vector_j  = vector[j].val;         // vector[j]
            result[i].val += matrix_ij*vector_j;
        }
    }
}

template<typename TempType>
void Utilities::mmult(TempType* result, TempType* A, int nrows1, int ncols1, TempType* B, int nrows2, int ncols2)
{
    for(int i = 0; i < nrows1; i++)
    {
        for(int j = 0; j < ncols2; j++)
        {
            // zero sum
            *(result+i*ncols2+j) = 0;
            
            for(int k=0;k<ncols1;k++)
            {
                int Arow = i, Acol = k, Brow = k, Bcol = j;
                TempType Aik = *(A+Arow*ncols1+Acol); // A[i][k]
                TempType Bkj = *(B+Brow*ncols2+Bcol); // B[k][j]
                *(result+i*ncols2+j) += Aik*Bkj;      //C[i][j] = A[i][k]*B[k][j]
            }
        }
    }
}

template<typename TempType>
void Utilities::mtran(TempType *matrix_t, TempType *matrix, int nrow_t, int ncol_t)
{
    for(int i=0;i<nrow_t;i++)
    {
        for(int j=0;j<ncol_t;j++)
        {
            *(matrix_t+i*ncol_t+j) = *(matrix+j*nrow_t+i);
        }
    }
}

template<typename TempType>
void Utilities::mtran(TempType *matrix_t, const TempType *matrix, int nrow_t, int ncol_t)
{
    for(int i=0;i<nrow_t;i++)
    {
        for(int j=0;j<ncol_t;j++)
        {
            *(matrix_t+i*ncol_t+j) = *(matrix+j*nrow_t+i);
        }
    }
}

template<typename TempType>
void Utilities::mgain(TempType *matrix, TempType gain, int nrow, int ncol)
{
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < ncol; j++)
        {
            *(matrix+i*ncol+j) *= gain;
        }
    }
}

template<typename TempType>
int Utilities::mrank(TempType *matrix, int nrow, int ncol)
{
    TempType mik;
    TempType reduceEcl[nrow][ncol];
    int nFullRank;
    int nRank;
    TempType sumRow;
    TempType temp;
    
    // Find smallest dimension
    if (nrow < ncol) { nFullRank = nrow; }
    else { nFullRank = ncol; }
    
    print(matrix, nrow, ncol, "A:");
    
    // Copy matrix
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < ncol; j++)
        {
            reduceEcl[i][j] = *(matrix+i*ncol+j);
        }
    }
    
    // Go through rows, starting at second row
    for (int irow = 1; irow < nrow; irow++)
    {
        for (int k = 0; k < irow; k++)
        {
            if (reduceEcl[k][k] > zeroTolerance || reduceEcl[k][k] < -zeroTolerance)
            { mik = reduceEcl[irow][k]/reduceEcl[k][k]; }
            else { mik = 0; }

            for (int icol = k; icol < ncol; icol++)
            {
                reduceEcl[irow][icol] -= reduceEcl[k][icol] * mik;
            }
        }
        //print(*reduceEcl, nrow, ncol, "reduced echelon form:");
        // Condition reduceEcl matrix with zero's on bottom
    }
    
    print(*reduceEcl, nrow, ncol, "reduced echelon form:");
    
    nRank = nrow;
    for (int irow = 0; irow < nrow; irow++)
    {
        sumRow = 0;
        
        for (int icol = 0; icol < ncol; icol++)
        {
            temp = reduceEcl[irow][icol];
            if ( reduceEcl[irow][icol] < 0) { temp *= -1; }
            sumRow += temp;
        }
        if (sumRow < zeroTolerance) { nRank--; }
    }
    print(&nRank, 1, "Rank:");
    return nRank;
}

template<typename TempType>
void Utilities::minv(TempType *matrix_inv, TempType *matrix, int n)
{
    TempType b[n];
    TempType x[n];
    for (int i = 0; i < n; i++)
    {
        b[i] = 0.0;
        x[i] = 0.0;
    }
    
    for (int icol = 0; icol < n; icol++)
    {
        // matrix_inv[1:n,icol] = A\b
        // where b[icol] = 1 and b[!icol] = 0
        
        b[icol] = 1.0;
        LUdecomp(x, matrix, b, n);
        for (int irow = 0; irow < n; irow++)
        {
            *(matrix_inv+irow*n+icol) = x[irow];
        }
        b[icol] = 0.0;
    }
}

template<typename TempType>
void Utilities::LUdecomp(TempType *x, TempType *A, TempType *b, int n)
{
    // Initialize variables
    TempType L[n][n];
    TempType U[n][n];
    TempType xstar[n];
    TempType sum;
    
    // Initialize L and U
    for(int i=0;i<n;i++){
        L[i][0] = *(A+i*n+0); //L[i][0] = A[i][0]
        U[i][i] = 1; // U diagonals = 1
        if(i > 0) {
            U[0][i] = *(A+0*n+i)/L[0][0]; // U[0][i] = A[0][i]/L[0][0]
        }
    }
    
    // Compute full L and U matrices
    for(int i=1;i<n;i++){
        // Compute col i of L
        for(int j=i;j<n;j++){
            // j is row of L, col of U
            // i is row of U, col of L
            sum = 0;
            for(int k=0;k<i;k++){
                sum += L[j][k]*U[k][i];
            }
            L[j][i] = *(A+j*n+i) - sum;
        }
        // Compute row i of U
        for(int j=i+1;j<n;j++){
            sum = 0;
            for(int k=0;k<i;k++){
                sum += L[i][k]*U[k][j];
            }
            U[i][j] = (*(A+i*n+j) - sum)/L[i][i];
        }
    }
    
    // Foward substitution L*xstar = b, solve for xstar
    xstar[0] = b[0]/L[0][0];
    for(int i=1;i<n;i++){
        sum = 0;
        for(int k=0;k<i;k++){
            sum += L[i][k]*xstar[k];
        }
        xstar[i] = (b[i] - sum)/L[i][i];
    }
    
    // Backward substitution U*x = xstar, solve for x
    x[n-1] = xstar[n-1]; //x[n-1] = xstar[n-1]/U[n-1][n-1];
    for (int i=n-2;i>-1;i--){
        sum = 0;
        for(int k=i+1;k<n;k++){
            sum += U[i][k]*x[k];
        }
        x[i] = xstar[i] - sum; //x[i] = (xstar[i] - sum)/U[i][i];
    }
}


template<typename TempType>
TempType Utilities::checkOrthonormal(TempType *A, int n)
{
    TempType Ainv[n][n];
    TempType Atran[n][n];
    TempType Adiff[n][n];
    TempType sumDiff;
    
    minv(*Ainv, A, n);
    mtran(*Atran, A, n, n);
    
    sumDiff = 0.0;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            Adiff[i][j] = Ainv[i][j] - Atran[i][j];
            sumDiff += Adiff[i][j];
        }
    }
    
    return sumDiff;
}


/* -------------------- Euler Math -------------------- */
template<typename TempType>
void Utilities::setupRotation(TempType *R, TempType *eulerAngles)
{
    TempType cr, cp, cy;
    TempType sr, sp, sy;
    
    cr = cos(eulerAngles[0]);
    cp = cos(eulerAngles[1]);
    cy = cos(eulerAngles[2]);
    
    sr = sin(eulerAngles[0]);
    sp = sin(eulerAngles[1]);
    sy = sin(eulerAngles[2]);
    
    *(R + 0*3 + 0) = cp*cy;            *(R + 0*3 + 1) = cp*sy;             *(R + 0*3 + 2) = -sp;
    *(R + 1*3 + 0) = cy*sp*sr - cr*sy; *(R + 1*3 + 1) = cr*cy + sp*sr*sy;  *(R + 1*3 + 2) = cp*sr;
    *(R + 2*3 + 0) = cy*sp*cr + sr*sy; *(R + 2*3 + 1) = -sr*cy + sp*cr*sy; *(R + 2*3 + 2) = cp*cr;
}

template<typename valType>
void Utilities::setupRotation(valType *R, AngleType<valType> *eulerAngles)
{
    valType cr, cp, cy;
    valType sr, sp, sy;
    
    cr = cos( eulerAngles[0].rad() );
    cp = cos( eulerAngles[1].rad() );
    cy = cos( eulerAngles[2].rad() );
    
    sr = sin( eulerAngles[0].rad() );
    sp = sin( eulerAngles[1].rad() );
    sy = sin( eulerAngles[2].rad() );
    
    *(R + 0*3 + 0) = cp*cy;            *(R + 0*3 + 1) = cp*sy;             *(R + 0*3 + 2) = -sp;
    *(R + 1*3 + 0) = cy*sp*sr - cr*sy; *(R + 1*3 + 1) = cr*cy + sp*sr*sy;  *(R + 1*3 + 2) = cp*sr;
    *(R + 2*3 + 0) = cy*sp*cr + sr*sy; *(R + 2*3 + 1) = -sr*cy + sp*cr*sy; *(R + 2*3 + 2) = cp*cr;
}

template<typename TempType>
void Utilities::dcmToEuler(TempType *euler, TempType *dcm)
{
    int n = 3;
    TempType dcm12 = *(dcm + 1*n + 2);
    TempType dcm22 = *(dcm + 2*n + 2);
    TempType dcm02 = *(dcm + 0*n + 2);
    TempType dcm01 = *(dcm + 0*n + 1);
    TempType dcm00 = *(dcm + 0*n + 0);

    euler[0] = atan(dcm12/dcm22);
    euler[1] = -asin(dcm02);
    euler[2] = atan(dcm01/dcm00);
}

template<typename valType>
void Utilities::dcmToEuler(AngleType<valType> *euler, valType *dcm)
{
    bool isDeg = euler[0].getUnit() == degrees;
    
    if (isDeg) { setUnitClassUnit(euler, radians, 3); }
    
    euler[0].val = atan(dcm[1][2]/dcm[2][2]);
    euler[1].val = -asin(dcm[0][2]);
    euler[2].val = atan(dcm[0][1]/dcm[0][0]);
    
    if (isDeg) { setUnitClassUnit(euler, degrees, 3); }
}

template<typename TempType>
void Utilities::setupEulerRateToBodyRate(TempType *L, TempType *eulerAngles)
{
    TempType cr, cp;
    TempType sr, sp;
    
    cr = cos(eulerAngles[0]);
    cp = cos(eulerAngles[1]);
    
    sr = sin(eulerAngles[0]);
    sp = sin(eulerAngles[1]);
    
    *(L + 0*3 + 0) = 1; *(L + 0*3 + 1) =   0; *(L + 0*3 + 2) =   -sp;
    *(L + 1*3 + 0) = 0; *(L + 1*3 + 1) =  cr; *(L + 1*3 + 2) = cp*sr;
    *(L + 2*3 + 0) = 0; *(L + 2*3 + 1) = -sr; *(L + 2*3 + 2) = cp*cr;
}

template<typename valType>
void Utilities::setupEulerRateToBodyRate(valType *L, AngleType<valType> *eulerAngles)
{
    valType cr, cp;
    valType sr, sp;
    
    cr = cos( eulerAngles[0].rad() );
    cp = cos( eulerAngles[1].rad() );
    
    sr = sin( eulerAngles[0].rad() );
    sp = sin( eulerAngles[1].rad() );
    
    *(L + 0*3 + 0) = 1; *(L + 0*3 + 1) =   0; *(L + 0*3 + 2) =   -sp;
    *(L + 1*3 + 0) = 0; *(L + 1*3 + 1) =  cr; *(L + 1*3 + 2) = cp*sr;
    *(L + 2*3 + 0) = 0; *(L + 2*3 + 1) = -sr; *(L + 2*3 + 2) = cp*cr;
}

template<typename TempType>
void Utilities::setupBodyRateToEulerRate(TempType *L, TempType *eulerAngles)
{
    TempType cr = cos(eulerAngles[0]);     // cos(roll)
    TempType sr = sin(eulerAngles[0]);     // sin(roll)
    TempType tp = tan(eulerAngles[1]);     // tan(pitch)
    TempType secp = 1.0/cos(eulerAngles[1]);  // sec(pitch)
    
    *(L +0*3 +0) =  1.0;   *(L +0*3 +1) =  sr*tp  ;   *(L +0*3 +2) =  cr*tp  ;
    *(L +1*3 +0) =  0.0;   *(L +1*3 +1) =    cr   ;   *(L +1*3 +2) =   -sr   ;
    *(L +2*3 +0) =  0.0;   *(L +2*3 +1) =  sr*secp;   *(L +2*3 +2) =  cr*secp;
}

template<typename valType>
void Utilities::setupBodyRateToEulerRate(valType *L, AngleType<valType> *eulerAngles)
{
    valType cr = cos( eulerAngles[0].rad()  );     // cos(roll)
    valType sr = sin( eulerAngles[0].rad()  );     // sin(roll)
    valType tp = tan( eulerAngles[1].rad()  );     // tan(pitch)
    valType secp = 1.0/cos( eulerAngles[1].rad() );  // sec(pitch)
    
    *(L +0*3 +0) =  1.0;   *(L +0*3 +1) =  sr*tp  ;   *(L +0*3 +2) =  cr*tp  ;
    *(L +1*3 +0) =  0.0;   *(L +1*3 +1) =    cr   ;   *(L +1*3 +2) =   -sr   ;
    *(L +2*3 +0) =  0.0;   *(L +2*3 +1) =  sr*secp;   *(L +2*3 +2) =  cr*secp;
}

/* -------------------- Quaternion Math -------------------- */
template<typename TempType>
void Utilities::initQuaternion(TempType *q, TempType angle, TempType *axis)
{
    TempType c2 = cos(angle/2);
    TempType s2 = sin(angle/2);
    
    q[0] = c2;
    q[1] = axis[0]*s2;
    q[2] = axis[1]*s2;
    q[3] = axis[2]*s2;
}

template<typename TempType>
void Utilities::quatToVec(TempType *vec, TempType *q)
{
    vec[0] = q[1];
    vec[1] = q[2];
    vec[2] = q[3];
}

template<typename valType, template<typename T> class unitType>
void Utilities::quatToVec(unitType<valType> *vec, valType *q)
{
    vec[0].val = q[1];
    vec[1].val = q[2];
    vec[2].val = q[3];
}

template<typename TempType>
void Utilities::vecToQuat(TempType *q, TempType *vec)
{
    q[0] = 0;
    q[1] = vec[0];
    q[2] = vec[1];
    q[3] = vec[2];
}

template<typename valType, template<typename T> class unitType>
void Utilities::vecToQuat(valType *q, unitType<valType> *vec)
{
    q[0] = 0;
    q[1] = vec[0].val;
    q[2] = vec[1].val;
    q[3] = vec[2].val;
}

template<typename TempType>
void Utilities::quaternionProduct(TempType *product, TempType *q1, TempType *q2)
{
    product[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    product[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    product[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    product[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

template<typename TempType>
void Utilities::quaternionConjugate(TempType *q_conj, TempType *q)
{
    q_conj[0] =  q[0];
    q_conj[1] = -q[1];
    q_conj[2] = -q[2];
    q_conj[3] = -q[3];
}

template<typename TempType>
void Utilities::quaternionRotation(TempType *w, TempType *quaternion, TempType *v)
{
    // w = q'*v*q
    // w = (q0^2 - |q|^2)*v + 2*dot(q,v)*q + 2*q0*cross(q,v)
    // w = k1*v1 + k2*q + k3*cross_q_v
    
    TempType q0, q[3];
    TempType cross_q_v[3];
    TempType k1, k2, k3;
    
    // Seperate q
    q0   = quaternion[0];
    q[0] = quaternion[1];
    q[1] = quaternion[2];
    q[2] = quaternion[3];
    
    // k1 = (q0^2 - |q|^2)
    k1 = q0*q0 - Utilities::dotProduct(q, q, 3);
    
    // k2 = 2*dot(q,v)
    k2 = 2*Utilities::dotProduct(q, v, 3);
    
    // k3 = 2*q0
    k3 = 2*q0;
    Utilities::crossProduct(cross_q_v, q, v);
    
    for (int i = 0; i < 3; i++)
    {
        w[i] = k1*v[i] + k2*q[i] + k3*cross_q_v[i];
    }
}

template<typename TempType>
void Utilities::quaternionTransformation(TempType *w, TempType *quaternion, TempType *v)
{
    // w = q'*v*q
    // w = (q0^2 - |q|^2)*v + 2*dot(q,v)*q + 2*q0*cross(q,v)
    // w = k1*v + k2*q + k3*cross_q_v
    
    TempType q0, q[3], q_conj[4];
    TempType cross_q_v[3];
    TempType k1, k2, k3;
    
    // Perform using conjugate to do a transformation instaed of a rotation
    Utilities::quaternionConjugate(q_conj, quaternion);
    
    // Seperate q
    q0   = q_conj[0];
    q[0] = q_conj[1];
    q[1] = q_conj[2];
    q[2] = q_conj[3];
    
    // k1 = (q0^2 - |q|^2)
    k1 = q0*q0 - Utilities::dotProduct(q, q, 3);
    
    // k2 = 2*dot(q,v)
    k2 = 2*Utilities::dotProduct(q, v, 3);
    
    // k3 = 2*q0
    k3 = 2*q0;
    Utilities::crossProduct(cross_q_v, q, v);

    for (int i = 0; i < 3; i++)
    {
        w[i] = k1*v[i] + k2*q[i] + k3*cross_q_v[i];
    }
}

template<typename valType, template<typename T> class unitType>
void Utilities::quaternionTransformation(unitType<valType> *wUnit, valType *quaternion, unitType<valType> *vUnit)
{
    valType v[3], w[3];
    Utilities::setArray(v, vUnit, 3);
    Utilities::quaternionTransformation(w, quaternion, v);
    Utilities::setUnitClassArray(wUnit, w, wUnit[0].getUnit(), 3);
}

template<typename valType, template<typename T> class unitType>
void Utilities::quaternionTransformation(valType *w, valType *quaternion, unitType<valType> *vUnit)
{
    valType v[3];
    Utilities::setArray(v, vUnit, 3);
    Utilities::quaternionTransformation(w, quaternion, v);
}

template<typename TempType>
void Utilities::eulerToQuaternion(TempType *q, TempType *euler)
{
    TempType cr2 = cos(euler[0]/2.0);
    TempType cp2 = cos(euler[1]/2.0);
    TempType cy2 = cos(euler[2]/2.0);
    
    TempType sr2 = sin(euler[0]/2.0);
    TempType sp2 = sin(euler[1]/2.0);
    TempType sy2 = sin(euler[2]/2.0);
    
    q[0] = cr2*cp2*cy2 + sr2*sp2*sy2;
    q[1] = sr2*cp2*cy2 - cr2*sp2*sy2;
    q[2] = cr2*sp2*cy2 + sr2*cp2*sy2;
    q[3] = cr2*cp2*sy2 - sr2*sp2*cy2;
    
    Utilities::unitVector(q, 4);
}

template<typename valType>
void Utilities::eulerToQuaternion(valType *q, AngleType<valType> *euler)
{
    valType cr2 = cos(euler[0].rad() / 2.0);
    valType cp2 = cos(euler[1].rad() / 2.0);
    valType cy2 = cos(euler[2].rad() / 2.0);
    
    valType sr2 = sin(euler[0].rad() / 2.0);
    valType sp2 = sin(euler[1].rad() / 2.0);
    valType sy2 = sin(euler[2].rad() / 2.0);
    
    q[0] = cr2*cp2*cy2 + sr2*sp2*sy2;
    q[1] = sr2*cp2*cy2 - cr2*sp2*sy2;
    q[2] = cr2*sp2*cy2 + sr2*cp2*sy2;
    q[3] = cr2*cp2*sy2 - sr2*sp2*cy2;
    
    Utilities::unitVector(q, 4);
}

template<typename TempType>
void Utilities::quaternionToEuler(TempType *euler, TempType *q)
{
    euler[0] = atan2(2*q[2]*q[3] + 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
    euler[1] = asin(-2*q[1]*q[3] + 2*q[0]*q[2]);
    euler[2] = atan2(2*q[1]*q[2] + 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
}

template<typename valType>
void Utilities::quaternionToEuler(AngleType<valType> *euler, valType *q)
{
    AngleListType oldUnit[3];
    
    // Convert to radians
    for (int i = 0; i < 3; i++)
    {
        oldUnit[i] = euler[i].getUnit();
        euler[i].convertUnit(radians);
    }
    
    // Store value in radians
    euler[0].val = atan2(2*q[2]*q[3] + 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
    euler[1].val = asin(-2*q[1]*q[3] + 2*q[0]*q[2]);
    euler[2].val = atan2(2*q[1]*q[2] + 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
    
    // Convert to degress
    for (int i = 0; i < 3; i++)
    {
        if ( oldUnit[i] == degrees ) { euler[i].convertUnit(degrees); }
    }

}

template<typename TempType>
void Utilities::quaternionToDcm(TempType *dcm, TempType *q)
{
    TempType q0 = q[0];
    TempType q1 = q[1];
    TempType q2 = q[2];
    TempType q3 = q[3];
    
    int n = 3;
    *(dcm+0*n+0) = q3*q3 + q0*q0 - q1*q1 - q2*q2;
    *(dcm+0*n+1) = 2.0*(q0*q1 + q2*q3);
    *(dcm+0*n+2) = 2.0*(q0*q2 - q1*q3);
    
    *(dcm+1*n+0) = 2.0*(q0*q1 - q2*q3);
    *(dcm+1*n+1) = q3*q3 - q0*q0 + q1*q1 - q2*q2;
    *(dcm+1*n+2) = 2.0*(q1*q2 + q0*q3);
    
    *(dcm+2*n+0) = 2.0*(q0*q2 + q1*q3);
    *(dcm+2*n+1) = 2.0*(q1*q2 - q0*q3);
    *(dcm+2*n+2) = q3*q3 - q0*q0 - q1*q1 + q2*q2;
}

template<typename TempType>
void Utilities::dcmToQuaternion(TempType *q, TempType *dcm)
{
    int n = 3;
    TempType dcm00 = *(dcm+0*n+0);
    TempType dcm11 = *(dcm+1*n+1);
    TempType dcm22 = *(dcm+2*n+2);
    TempType dcm12 = *(dcm+1*n+2);
    TempType dcm21 = *(dcm+2*n+1);
    TempType dcm20 = *(dcm+2*n+0);
    TempType dcm02 = *(dcm+0*n+2);
    TempType dcm01 = *(dcm+0*n+1);
    TempType dcm10 = *(dcm+1*n+0);
    
    q[0] = 0.5*sqrt(dcm00 + dcm11 + dcm22 + 1.0);
    q[1] = (dcm12 - dcm21)/(4.0*q[0]);
    q[2] = (dcm20 - dcm02)/(4.0*q[0]);
    q[3] = (dcm01 - dcm10)/(4.0*q[0]);
}

// ECEF/LLH Math
template<typename TempType>
void Utilities::LLHtoECEF(TempType* ECEF, TempType* LLH)
{
    TempType lat = LLH[0];
    TempType lon = LLH[1];
    TempType alt = LLH[2];
    TempType slat = sin(lat);
    TempType clat = cos(lat);
    TempType clon = cos(lon);
    TempType slon = sin(lon);
    TempType N = EARTHCONSTANTS::a/( sqrt(1.0 - EARTHCONSTANTS::e2*slat*slat) );

    ECEF[0] = (N+alt)*clat*clon;
    ECEF[1] = (N+alt)*clat*slon;
    ECEF[2] = (EARTHCONSTANTS::b2/EARTHCONSTANTS::a2*N + alt)*slat;
}

template<typename TempType>
void Utilities::ECEFtoLLH(TempType* LLH, TempType* ECEF)
{
    TempType X = ECEF[0];
    TempType Y = ECEF[1];
    TempType Z = ECEF[2];
    TempType Z2 = Z*Z;
    
    TempType ePrime = (EARTHCONSTANTS::a2 - EARTHCONSTANTS::b2)/EARTHCONSTANTS::b2;
    TempType p = sqrt(X*X + Y*Y);
    TempType F = 54.0*EARTHCONSTANTS::b2*Z2;
    TempType G = p*p + (1.0 - EARTHCONSTANTS::e2)*Z2 - EARTHCONSTANTS::e2*(EARTHCONSTANTS::a2 - EARTHCONSTANTS::b2);
    TempType c = EARTHCONSTANTS::e2*EARTHCONSTANTS::e2*F*p*p/(G*G*G);
    TempType s = pow(1.0+c+sqrt(c*c+2*c), 1/3);
    TempType k = s + 1.0 + 1.0/s;
    TempType P = F/(3.0*k*k*G*G);
    TempType Q = sqrt(1.0 + 2.0*EARTHCONSTANTS::e2*EARTHCONSTANTS::e2*P);
    
    TempType r0_1 = -P*EARTHCONSTANTS::e2*p/(1.0+Q);
    TempType r0_2 = sqrt(0.5*EARTHCONSTANTS::a2*(1.0+1.0/Q) - P*(1.0-EARTHCONSTANTS::e2)*Z2/(Q*(1.0+Q)) - 0.5*P*p*p);
    TempType r0 = r0_1 + r0_2;
    
    TempType U = sqrt(pow(p-EARTHCONSTANTS::e2*r0,2) + Z2);
    TempType V = sqrt(pow(p-EARTHCONSTANTS::e2*r0,2) + (1.0-EARTHCONSTANTS::e2)*Z2);
    TempType z0 = EARTHCONSTANTS::b2*Z/(EARTHCONSTANTS::a*V);
    
    LLH[0] = atan((Z + ePrime*ePrime*z0)/p);
    LLH[1] = atan2(Y, X);
    LLH[2] = U*(1.0 - EARTHCONSTANTS::b2/(EARTHCONSTANTS::a*V));
}

template<typename TempType>
void Utilities::ECEFtoLLH(TempType* LLH, TempType* ECEF, TempType k_init)
{
    // phi = lat
    // lambda = lon
    
    TempType c;
    TempType c_num;
    TempType c_den;
    
    TempType k0;
    TempType k;
    TempType k_num;
    TempType k_den;
    TempType k_prev;
    
    TempType X = ECEF[0];
    TempType Y = ECEF[1];
    TempType Z = ECEF[2];
    TempType p = sqrt(X*X + Y*Y);
    
    k0 = 1.0/(1.0 - EARTHCONSTANTS::e2);
    if (k_init == 0) { k = k0; }
    else { k = k_init; }
    
    const int maxiter = 100;
    int iter = 0;
    TempType k_diff = 100;
    TempType e = 1e-20;
    
    while(k_diff > e && iter < maxiter)
    {
        k_prev = k;
        c_num = p*p + (1 - EARTHCONSTANTS::e2)*Z*Z*k*k;
        c_den = EARTHCONSTANTS::a*EARTHCONSTANTS::e2;
        c = pow(c_num, 1.5)/c_den;

        k_num = p*p + (1 - EARTHCONSTANTS::e2)*Z*Z*k_prev*k_prev*k_prev;
        k_den = c - p*p;
        k = 1.0 + k_num/k_den;
        
        k_diff = k - k_prev;
        if (k_diff < 0) { k_diff *= -1.0; }
        iter++;
    }
    
    LLH[0] = atan(k*Z/p);
    LLH[1] = atan2(Y, X);
    LLH[2] = 1.0/EARTHCONSTANTS::e2*(1.0/k - 1.0/k0)*sqrt(p*p + Z*Z*k*k);
}

template<typename TempType>
void Utilities::dcmECEFtoNED(TempType* dcm, TempType* LLH)
{
    TempType lat = LLH[0];
    TempType lon = LLH[1];
    TempType slat = sin(lat);
    TempType clat = cos(lat);
    TempType slon = sin(lon);
    TempType clon = cos(lon);
    
    TempType dcm_ECEFtoENU[3][3];
    initMatrix(*dcm_ECEFtoENU, (TempType) 0.0, 3, 3);
    
    TempType dcm_ENUtoNED[3][3] = {
        {0.0, 1.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 0.0, -1.0}
    };

    dcm_ECEFtoENU[0][0] = -slon;
    dcm_ECEFtoENU[0][1] = clon;
    dcm_ECEFtoENU[0][2] = 0.0;
    
    dcm_ECEFtoENU[1][0] = -slat*clon;
    dcm_ECEFtoENU[1][1] = -slat*slon;
    dcm_ECEFtoENU[1][2] = clat;
    
    dcm_ECEFtoENU[2][0] = clat*clon;
    dcm_ECEFtoENU[2][1] = clat*slon;
    dcm_ECEFtoENU[2][2] = slat;
    
    //dcmECEFtoNED = dcm_ENUtoNED*dcm_ECEFtoENU
    initMatrix(dcm, (TempType) 0.0, 3, 3);
    mmult(dcm, *dcm_ENUtoNED, 3, 3, *dcm_ECEFtoENU, 3, 3);
}

bool Utilities::isBigEndian()
{
    unsigned short a = 0x1234;
    if (*((unsigned char *)&a)==0x12)
    {
        //std::cout << "BIG_ENDIAN" << std::endl;
        return true;
    }
    else
    {
        //std::cout << "LITTLE_ENDIAN" << std::endl;
        return false;
    }
}
/* --------------------  Template Definitions -------------------- */
template void Utilities::print(int*, int , std::string);
template void Utilities::print(float*, int , std::string);
template void Utilities::print(double*, int , std::string);
template void Utilities::print(const double*, int , std::string);

template void Utilities::print(int* , int);
template void Utilities::print(float* , int);
template void Utilities::print(double* , int);

template void Utilities::print(int* , int, const char*);
template void Utilities::print(float* , int, const char*);
template void Utilities::print(double* , int, const char*);
template void Utilities::print(const double* , int, const char*);

template void Utilities::print(DistanceType<int>* , int);
template void Utilities::print(DistanceType<float>* , int);
template void Utilities::print(DistanceType<double>* , int);

template void Utilities::print(DistanceType<int>* , int, const char*);
template void Utilities::print(DistanceType<float>* , int, const char*);
template void Utilities::print(DistanceType<double>* , int, const char*);

template void Utilities::print(SpeedType<int>* , int);
template void Utilities::print(SpeedType<float>* , int);
template void Utilities::print(SpeedType<double>* , int);

template void Utilities::print(SpeedType<int>* , int, const char*);
template void Utilities::print(SpeedType<float>* , int, const char*);
template void Utilities::print(SpeedType<double>* , int, const char*);

template void Utilities::print(AngleType<int>* , int);
template void Utilities::print(AngleType<float>* , int);
template void Utilities::print(AngleType<double>* , int);

template void Utilities::print(AngleType<float>*, AngleListType, int);
template void Utilities::print(AngleType<double>*, AngleListType, int);

template void Utilities::print(AngleType<int>* , int, const char*);
template void Utilities::print(AngleType<float>* , int, const char*);
template void Utilities::print(AngleType<double>* , int, const char*);

template void Utilities::print(AngleType<float>*, AngleListType, int, const char*);
template void Utilities::print(AngleType<double>*, AngleListType, int, const char*);

template void Utilities::print(AngleRateType<int>* , int);
template void Utilities::print(AngleRateType<float>* , int);
template void Utilities::print(AngleRateType<double>* , int);

template void Utilities::print(AngleRateType<float>*, AngleRateListType, int);
template void Utilities::print(AngleRateType<double>*, AngleRateListType, int);

template void Utilities::print(AngleRateType<int>* , int, const char*);
template void Utilities::print(AngleRateType<float>* , int, const char*);
template void Utilities::print(AngleRateType<double>* , int, const char*);

template void Utilities::print(AngleRateType<float>*, AngleRateListType, int, const char*);
template void Utilities::print(AngleRateType<double>*, AngleRateListType, int, const char*);

template void Utilities::print(int*, int, int);
template void Utilities::print(float*, int, int);
template void Utilities::print(double*, int, int);

template void Utilities::print(int*, int, int, const char*);
template void Utilities::print(float*, int, int, const char*);
template void Utilities::print(double*, int, int, const char*);

template void Utilities::initArray(bool*, bool, int);
template void Utilities::initArray(int*, int, int);
template void Utilities::initArray(float*, float, int);
template void Utilities::initArray(double*, double, int);

template void Utilities::setArray(float*, float*, int);
template void Utilities::setArray(double*, double*, int);

template void Utilities::setArray(float*, const float*, int);
template void Utilities::setArray(double*, const double*, int);

template void Utilities::setArray(float*, DistanceType<float>*, int);
template void Utilities::setArray(float*, SpeedType<float>*, int);
template void Utilities::setArray(float*, AngleType<float>*, int);
template void Utilities::setArray(float*, AngleRateType<float>*, int);

template void Utilities::setArray(double*, DistanceType<double>*, int);
template void Utilities::setArray(double*, SpeedType<double>*, int);
template void Utilities::setArray(double*, AngleType<double>*, int);
template void Utilities::setArray(double*, AngleRateType<double>*, int);

template void Utilities::setArray(DistanceType<float>*, DistanceType<float>*, int);
template void Utilities::setArray(SpeedType<float>*, SpeedType<float>*, int);
template void Utilities::setArray(AngleType<float>*, AngleType<float>*, int);
template void Utilities::setArray(AngleRateType<float>*, AngleRateType<float>*, int);

template void Utilities::setArray(DistanceType<double>*, DistanceType<double>*, int);
template void Utilities::setArray(SpeedType<double>*, SpeedType<double>*, int);
template void Utilities::setArray(AngleType<double>*, AngleType<double>*, int);
template void Utilities::setArray(AngleRateType<double>*, AngleRateType<double>*, int);

template void Utilities::setUnitClassUnit(AngleType<float>*, AngleListType, int);
template void Utilities::setUnitClassArray(AngleType<float>*, float*, AngleListType ,int);
template void Utilities::setUnitClassArray(AngleType<float>*, const float*, AngleListType ,int);

template void Utilities::setUnitClassUnit(AngleType<double>*, AngleListType, int);
template void Utilities::setUnitClassArray(AngleType<double>*, double*, AngleListType ,int);
template void Utilities::setUnitClassArray(AngleType<double>*, const double*, AngleListType ,int);

template void Utilities::setUnitClassUnit(DistanceType<float>*, DistanceListType, int);
template void Utilities::setUnitClassArray(DistanceType<float>*, float*, DistanceListType ,int);
template void Utilities::setUnitClassArray(DistanceType<float>*, const float*, DistanceListType ,int);

template void Utilities::setUnitClassUnit(DistanceType<double>*, DistanceListType, int);
template void Utilities::setUnitClassArray(DistanceType<double>*, double*, DistanceListType ,int);
template void Utilities::setUnitClassArray(DistanceType<double>*, const double*, DistanceListType ,int);

template void Utilities::setUnitClassUnit(SpeedType<float>*, SpeedListType, int);
template void Utilities::setUnitClassArray(SpeedType<float>*, float*, SpeedListType ,int);
template void Utilities::setUnitClassArray(SpeedType<float>*, const float*, SpeedListType ,int);

template void Utilities::setUnitClassUnit(SpeedType<double>*, SpeedListType, int);
template void Utilities::setUnitClassArray(SpeedType<double>*, double*, SpeedListType ,int);
template void Utilities::setUnitClassArray(SpeedType<double>*, const double*, SpeedListType ,int);

template void Utilities::setUnitClassUnit(AngleRateType<float>*, AngleRateListType, int);
template void Utilities::setUnitClassArray(AngleRateType<float>*, float*, AngleRateListType ,int);
template void Utilities::setUnitClassArray(AngleRateType<float>*, const float*, AngleRateListType ,int);

template void Utilities::setUnitClassUnit(AngleRateType<double>*, AngleRateListType, int);
template void Utilities::setUnitClassArray(AngleRateType<double>*, double*, AngleRateListType ,int);
template void Utilities::setUnitClassArray(AngleRateType<double>*, const double*, AngleRateListType ,int);

template void Utilities::initMatrix(float*, float, int, int);
template void Utilities::initMatrix(double*, double, int, int);

template void Utilities::setMatrix(float*, const float*, int, int);
template void Utilities::setMatrix(double*, const double*, int, int);

template void Delay::delayValue(bool *value, bool newValue);
template void Delay::delayValue(float *value, float newValue);
template void Delay::delayValue(double *value, double newValue);

template int Utilities::max(int, int);
template float Utilities::max(float, float);
template double Utilities::max(double, double);

template int Utilities::max(int*, int);
template float Utilities::max(float*, int);
template double Utilities::max(double*, int);

template int Utilities::min(int, int);
template float Utilities::min(float, float);
template double Utilities::min(double, double);

template int Utilities::min(int*, int);
template float Utilities::min(float*, int);
template double Utilities::min(double*, int);

template int Utilities::mean(int*, int);
template float Utilities::mean(float*, int);
template double Utilities::mean(double*, int);

template int Utilities::stdev(int*, int);
template float Utilities::stdev(float*, int);
template double Utilities::stdev(double*, int);

template void Utilities::vAdd(float*, float*, float*, int);
template void Utilities::vAdd(double*, double*, double*, int);

template void Utilities::vAdd(DistanceType<float>*, DistanceType<float>*, float*, int);
template void Utilities::vAdd(SpeedType<float>*, SpeedType<float>*, float*, int);
template void Utilities::vAdd(AngleType<float>*, AngleType<float>*, float*, int);
template void Utilities::vAdd(AngleRateType<float>*, AngleRateType<float>*, float*, int);

template void Utilities::vAdd(DistanceType<double>*, DistanceType<double>*, double*, int);
template void Utilities::vAdd(SpeedType<double>*, SpeedType<double>*, double*, int);
template void Utilities::vAdd(AngleType<double>*, AngleType<double>*, double*, int);
template void Utilities::vAdd(AngleRateType<double>*, AngleRateType<double>*, double*, int);

template void Utilities::vAdd(SpeedType<float>*, SpeedType<float>*, SpeedType<float>*, int);
template void Utilities::vAdd(SpeedType<double>*, SpeedType<double>*, SpeedType<double>*, int);

template void Utilities::vSubtract(float*, float*, float*, int);
template void Utilities::vSubtract(double*, double*, double*, int);

template void Utilities::vSubtract(SpeedType<float>*, SpeedType<float>*, SpeedType<float>*, int);
template void Utilities::vSubtract(SpeedType<double>*, SpeedType<double>*, SpeedType<double>*, int);

template void Utilities::crossProduct(int*, int*, int*);
template void Utilities::crossProduct(float*, float*, float*);
template void Utilities::crossProduct(double*, double*, double*);

template void Utilities::crossProduct(float*, DistanceType<float>*, float*);
template void Utilities::crossProduct(float*, AngleRateType<float>*, float*);

template void Utilities::crossProduct(double*, DistanceType<double>*, double*);
template void Utilities::crossProduct(double*, AngleRateType<double>*, double*);

template void Utilities::crossProduct(SpeedType<float>*, AngleRateType<float>*, DistanceType<float>*);
template void Utilities::crossProduct(SpeedType<double>*, AngleRateType<double>*, DistanceType<double>*);

template void Utilities::crossProduct(double*, double*, DistanceType<double>*);

template void Utilities::crossProduct(double*, SpeedType<double>*, DistanceType<double>*);
template void Utilities::crossProduct(double*, SpeedType<double>*, AngleRateType<double>*);
template void Utilities::crossProduct(double*, AngleRateType<double>*, SpeedType<double>*);

template int Utilities::dotProduct(int* , int* , int);
template float Utilities::dotProduct(float* , float* , int);
template double Utilities::dotProduct(double* , double* , int);

template int Utilities::mag(int*, int);
template float Utilities::mag(float* , int);
template double Utilities::mag(double* , int);

template float Utilities::mag(DistanceType<float> *vec, int n);
template float Utilities::mag(SpeedType<float> *vec, int n);
template float Utilities::mag(AngleRateType<float> *vec, int n);

template double Utilities::mag(DistanceType<double> *vec, int n);
template double Utilities::mag(SpeedType<double> *vec, int n);
template double Utilities::mag(AngleRateType<double> *vec, int n);

template void Utilities::unitVector(int* , int);
template void Utilities::unitVector(float* , int);
template void Utilities::unitVector(double* , int);

template void Utilities::vgain(int* , int, int);
template void Utilities::vgain(float* , float, int);
template void Utilities::vgain(double* , double, int);

template float Utilities::interpolate(float*, float*, float, int, bool, bool);
template double Utilities::interpolate(double*, double*, double, int, bool, bool);

template float Utilities::interpolate(const float*, const float*, float, int, bool, bool);
template double Utilities::interpolate(const double*, const double*, double, int, bool, bool);

template bool Utilities::solveQuadratic(double, double, double, double*, double*);

template void Utilities::mmult(int* , int* , int* , int, int);
template void Utilities::mmult(float* , float* , float* , int, int);
template void Utilities::mmult(double* , double* , double* , int, int);

template void Utilities::mmult(float* ,float* ,AngleRateType<float>* , int, int);
template void Utilities::mmult(double* ,double* ,AngleRateType<double>* , int, int);

template void Utilities::mmult(double* ,double* ,SpeedType<double>* , int, int);

template void Utilities::mmult(DistanceType<float>* ,float* ,DistanceType<float>* , int, int);
template void Utilities::mmult(SpeedType<float>* ,float* ,SpeedType<float>* , int, int);
template void Utilities::mmult(AngleRateType<float>* ,float* ,AngleRateType<float>* , int, int);

template void Utilities::mmult(DistanceType<double>* ,double* ,DistanceType<double>* , int, int);
template void Utilities::mmult(SpeedType<double>* ,double* ,SpeedType<double>* , int, int);
template void Utilities::mmult(AngleRateType<double>* ,double* ,AngleRateType<double>* , int, int);

template void Utilities::mmult(int*, int* , int, int, int* , int, int);
template void Utilities::mmult(float*, float* , int, int, float* , int, int);
template void Utilities::mmult(double*, double* , int, int, double* , int, int);

template void Utilities::mtran(int* , int* ,int ,int);
template void Utilities::mtran(float* , float* ,int ,int);
template void Utilities::mtran(double* , double* ,int ,int);

template void Utilities::mtran(float* , const float* ,int ,int);
template void Utilities::mtran(double* , const double* ,int ,int);

template void Utilities::mgain(int* , int, int, int);
template void Utilities::mgain(float* , float, int, int);
template void Utilities::mgain(double* , double, int, int);

template int Utilities::mrank(double* matrix, int nrow, int ncol);

template void Utilities::minv(float* matrix_inv, float* matrix, int n);
template void Utilities::minv(double* matrix_inv, double* matrix, int n);

template void Utilities::LUdecomp(int*, int* , int* , int);
template void Utilities::LUdecomp(float*, float* , float* , int);
template void Utilities::LUdecomp(double*, double* , double* , int);

template double Utilities::checkOrthonormal(double *A, int n);

template void Utilities::setupRotation(int* , int*);
template void Utilities::setupRotation(float* , float*);
template void Utilities::setupRotation(double* , double*);

template void Utilities::setupRotation(int*, AngleType<int>*);
template void Utilities::setupRotation(float*, AngleType<float>*);
template void Utilities::setupRotation(double*, AngleType<double>*);

template void Utilities::dcmToEuler(float*, float*);
template void Utilities::dcmToEuler(double*, double*);

template void Utilities::setupEulerRateToBodyRate(int* , int*);
template void Utilities::setupEulerRateToBodyRate(float* , float*);
template void Utilities::setupEulerRateToBodyRate(double* , double*);

template void Utilities::setupEulerRateToBodyRate(int* , AngleType<int>*);
template void Utilities::setupEulerRateToBodyRate(float* , AngleType<float>*);
template void Utilities::setupEulerRateToBodyRate(double* , AngleType<double>*);

template void Utilities::setupBodyRateToEulerRate(int* , int*);
template void Utilities::setupBodyRateToEulerRate(float* , float*);
template void Utilities::setupBodyRateToEulerRate(double* , double*);

template void Utilities::setupBodyRateToEulerRate(int* , AngleType<int>*);
template void Utilities::setupBodyRateToEulerRate(float* , AngleType<float>*);
template void Utilities::setupBodyRateToEulerRate(double* , AngleType<double>*);

template void Utilities::initQuaternion(float *q, float angle, float *axis);
template void Utilities::initQuaternion(double *q, double angle, double *axis);

template void Utilities::quatToVec(int* , int*);
template void Utilities::quatToVec(float* , float*);
template void Utilities::quatToVec(double* , double*);

template void Utilities::quatToVec(DistanceType<float>*, float*);
template void Utilities::quatToVec(SpeedType<float>*, float*);
template void Utilities::quatToVec(AngleRateType<float>*, float*);

template void Utilities::quatToVec(DistanceType<double>*, double*);
template void Utilities::quatToVec(SpeedType<double>*, double*);
template void Utilities::quatToVec(AngleRateType<double>*, double*);

template void Utilities::vecToQuat(float*, DistanceType<float>*);
template void Utilities::vecToQuat(float*, SpeedType<float>*);
template void Utilities::vecToQuat(float*, AngleRateType<float>*);

template void Utilities::vecToQuat(double*, DistanceType<double>*);
template void Utilities::vecToQuat(double*, SpeedType<double>*);
template void Utilities::vecToQuat(double*, AngleRateType<double>*);

template void Utilities::vecToQuat(int* , int*);
template void Utilities::vecToQuat(float* , float*);
template void Utilities::vecToQuat(double* , double*);

template void Utilities::quaternionProduct(int* , int* , int*);
template void Utilities::quaternionProduct(float* , float* , float*);
template void Utilities::quaternionProduct(double* , double* , double*);

template void Utilities::quaternionConjugate(int*, int*);
template void Utilities::quaternionConjugate(float*, float*);
template void Utilities::quaternionConjugate(double*, double*);

template void Utilities::quaternionRotation(int* , int*, int*);
template void Utilities::quaternionRotation(float* , float*, float*);
template void Utilities::quaternionRotation(double* , double*, double*);

template void Utilities::quaternionTransformation(int* , int* , int*);
template void Utilities::quaternionTransformation(float* , float* , float*);
template void Utilities::quaternionTransformation(double* , double* , double*);

template void Utilities::quaternionTransformation(DistanceType<float>*, float*, DistanceType<float>*);
template void Utilities::quaternionTransformation(SpeedType<float>*, float*, SpeedType<float>*);

template void Utilities::quaternionTransformation(DistanceType<double>*, double*, DistanceType<double>*);
template void Utilities::quaternionTransformation(SpeedType<double>*, double*, SpeedType<double>*);

template void Utilities::quaternionTransformation(double *w, double *quaternion, AngleRateType<double> *vUnit);

template void Utilities::eulerToQuaternion(int* , int*);
template void Utilities::eulerToQuaternion(float* , float*);
template void Utilities::eulerToQuaternion(double* , double*);

template void Utilities::eulerToQuaternion(int*, AngleType<int>*);
template void Utilities::eulerToQuaternion(float*, AngleType<float>*);
template void Utilities::eulerToQuaternion(double*, AngleType<double>*);

template void Utilities::quaternionToEuler(int* , int*);
template void Utilities::quaternionToEuler(float* , float*);
template void Utilities::quaternionToEuler(double* , double*);

template void Utilities::quaternionToEuler(AngleType<int>* , int*);
template void Utilities::quaternionToEuler(AngleType<float>* , float*);
template void Utilities::quaternionToEuler(AngleType<double>* , double*);

template void Utilities::quaternionToDcm(double*, double*);

template void Utilities::dcmToQuaternion(double*, double*);

template void Utilities::LLHtoECEF(float* ECEF, float* LLH);
template void Utilities::LLHtoECEF(double* ECEF, double* LLH);

template void Utilities::ECEFtoLLH(float* LLH, float* ECEF);
template void Utilities::ECEFtoLLH(double* LLH, double* ECEF);

template void Utilities::ECEFtoLLH(float* LLH, float* ECEF, float k_init);
template void Utilities::ECEFtoLLH(double* LLH, double* ECEF, double k_init);

template void Utilities::dcmECEFtoNED(float* dcm, float* LLH);
template void Utilities::dcmECEFtoNED(double* dcm, double* LLH);
