//
//  transfer_function_utilities.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 7/16/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#ifndef transfer_function_utilities_hpp
#define transfer_function_utilities_hpp

#include <stdio.h>
#include "utilities.hpp"

class tf {
public:
    
    tf(double* numIn, int numSize, double* denIn, int denSize);
    
    void setFile(std::string filenameIn) { stepFilename = filenameIn; }
    void print();
    void poles();
    void zeros();
    void dampfreq();
    void step(double input = 1.0, double tFinal = 10.0);
private:
    static const int maxOrder = 10;
    int numOrder;
    int denOrder;
    double num[maxOrder];
    double den[maxOrder];
    std::string stepFilename;
    void printPolynomial(double* ploynomial, int size);
};

#endif /* transfer_function_utilities_hpp */
