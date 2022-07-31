//
//  transfer_function_utilities.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 7/16/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "transfer_function_utilities.hpp"

tf::tf(double* numIn, int numSize, double* denIn, int denSize)
{
    numOrder = numSize-1;
    denOrder = denSize-1;
    
    for (int i=0; i<maxOrder; i++)
    {
        if (i < numSize)
        {
            num[i] = numIn[i];
        }
        
        if (i < denSize)
        {
            den[i] = denIn[i];
        }
    }
    
    stepFilename = "step.csv";
}

void tf::dampfreq()
{
    if (denOrder == 2)
    {
        if (den[0] != 1)
        {
            den[1] /= den[0];
            den[2] /= den[0];
            den[0] /= den[0];
        }
        // s^2 = 2*zeta*wn + wn^2
        double wn = sqrt(fabs(den[2]));
        double zeta = den[1]/(2*wn);
        
        std::cout << "wn: " << wn/(2*M_PI) << " hz " << std::endl;
        std::cout << "zeta: " << zeta << std::endl;
    }
}

void step(double input, double tFinal)
{
    //bool save = false;
    //bool print = false;
    
    double tInitial = 0.0;
    double dt = 1e-3;
    double time = tInitial;
    
    //
    // U(s) = 1/s
    //
    // Y(s)    n1*s + n0
    // --- =   ------------------
    // U(s)     d2*s^2 + d1*s + d0
    //
    // d2*ddy + d1*dy + d0*y = n1*du + n0*u
    //
    //

    while(time < tFinal)
    {
        
        time += dt;
    }
}

void tf::print()
{
    printPolynomial(num, numOrder);
    std::cout << "--------------" << std::endl;
    printPolynomial(den, denOrder);
}

void tf::printPolynomial(double* ploynomial, int size)
{
    int printCount = 0;
    int order = 0;
    for (int i=0; i<size; i++)
    {
        order = size-i;
        // Skip if zero
        if (ploynomial[i] != 0.0)
        {
            // Print sign and number
            if (printCount != 0)
            {
                if (ploynomial[i] > 0.0) { std::cout << "+ "; }
                else { std::cout << "- "; }
                std::cout << fabs( ploynomial[i] );
            }
            else
            {
                std::cout << ploynomial[i];
            }
            
            // Print s
            if (order == 1)
            {
                std::cout << "s ";
            }
            else if (order != 0)
            {
                std::cout << "s^" << order << " ";
            }
            printCount++;
        }
    }
    std::cout<<std::endl;
}
