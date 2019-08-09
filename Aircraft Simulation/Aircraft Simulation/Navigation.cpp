//
//  Navigation.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/17/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#include "Navigation.hpp"
#include <iostream>
/*
using namespace std;

int main(int argc, const char * argv[]) {
    
    float xo = 0;
    float vo = 0;
    
    float a1  = 1;
    float a2 = 0;
    float a3 = -1;
    float a1Time = 0;
    float a2Time = 30;
    float a3Time = 170;
    
    float aTrue = 0;
    float vTrue = vo;
    float xTrue = xo;
    
    float aIMU = 0;
    float vIMU = vo;
    float xIMU = xo;
    
    float xGPS = 0;
    float xprevGPS = 0;
    float vGPS = vo;
    
    
    float dt = 1;
    
    
    float initialTime = 0;
    float finalTime = 200;
    
    float imuUpdateTime = 1;
    float gpsUpdateTime = 3;
    float oneOverGpsUpdateTime = 1/gpsUpdateTime;
    float printTime     = 6;
    
    float imuUpdateTimeSquared = imuUpdateTime*imuUpdateTime;
    float timeSinceLastPrint = 0;
    float timeSinceLastlastImuUpdate = 0;
    float timeSinceLastlastGpsUpdate = 0;
    
    for(float time=initialTime;time<=finalTime;time+=dt)
    {
        if (time >= a3Time) {aTrue = a3;}
        else if (time >=a2Time) {aTrue = a2;}
        else  {aTrue = a1;}
        
        if (timeSinceLastlastImuUpdate >= imuUpdateTime) {
            timeSinceLastlastImuUpdate=0;
            aIMU = aTrue;
            vIMU = vIMU + aIMU*imuUpdateTime;
            xIMU = xIMU + vIMU*imuUpdateTime + 0.5*aIMU*imuUpdateTimeSquared;
            cout<<"t="<<time<<" xIMU="<<xIMU<<" vIMU="<<vIMU<<" aIMU="<<aIMU<<endl;
        }
        
        if (timeSinceLastlastGpsUpdate >= gpsUpdateTime) {
            timeSinceLastlastGpsUpdate=0;
            xGPS = xTrue;
            vGPS = (xGPS - xprevGPS)*oneOverGpsUpdateTime - 0.5*aIMU*gpsUpdateTime;
            xprevGPS = xGPS;
            cout<<"t="<<time<<" xGPS="<<xGPS<<" vGPS="<<vGPS<<endl;
        }
        
        if (timeSinceLastPrint >= printTime) {
            timeSinceLastPrint=0;
            cout<<"t="<<time<<" xTrue="<<xTrue<<" vTrue="<<vTrue<<" aTure="<<aTrue<<"\n"<<endl;
            cout<<rand() % 1<<endl;
        }
        
        vTrue = vTrue + aTrue*dt;
        xTrue = xTrue + vTrue*dt + 0.5*aTrue*dt*dt;
        
        timeSinceLastlastImuUpdate += dt;
        timeSinceLastlastGpsUpdate += dt;
        timeSinceLastPrint += dt;
        
    }
    return 0;
}
*/
