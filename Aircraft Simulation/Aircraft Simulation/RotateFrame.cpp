//
//  Coordinate Transformation.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/17/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//
// Rotates a 3D vector
//      RotateInertialToBody - Rotates a vector from inertial frame to body frame representation
//      RotateBodyToInertial - Rotates a vector from body frame to inertial frame representation

#include "RotateFrame.hpp"

void RotateFrame::RotateInertialToBody(float phi,float theta,float psi, float *FrameBody, float *FrameInertial){
     ctheta = cos(theta);
     cpsi   = cos(psi);
     cphi   = cos(phi);
     stheta = sin(theta);
     spsi   = sin(psi);
     sphi   = sin(phi);
    
     float RBI[3][3] = { {ctheta*cpsi               , ctheta*spsi               ,     -stheta  },
                         {cpsi*stheta*sphi-cphi*spsi, cphi*cpsi+stheta*sphi*spsi, ctheta*sphi  },
                         {cpsi*stheta*cphi+sphi*spsi,-sphi*cpsi+stheta*cphi*spsi, ctheta*cphi }};
    
    for (int i=0;i<3;i++)
    {
        FrameBody[i] = 0;
        for (int j=0;j<3;j++)
        {
            FrameBody[i] = FrameBody[i] + RBI[i][j] * FrameInertial[j];
        }
    }
}

void RotateFrame::RotateBodyToInertial(float phi,float theta,float psi, float *FrameBody, float *FrameInertial){
    ctheta = cos(theta);
    cpsi   = cos(psi);
    cphi   = cos(phi);
    stheta = sin(theta);
    spsi   = sin(psi);
    sphi   = sin(phi);
    
    float RIB[3][3] = { {ctheta*cpsi ,cpsi*stheta*sphi-cphi*spsi ,cpsi*stheta*cphi+sphi*spsi   },
                         {ctheta*spsi ,cphi*cpsi+stheta*sphi*spsi ,-sphi*cpsi+stheta*cphi*spsi },
                         {-stheta     ,ctheta*sphi                ,ctheta*cphi                }};
    
    for (int i=0;i<3;i++)
    {
        FrameInertial[i] = 0;
        for (int j=0;j<3;j++)
        {
            FrameInertial[i] = FrameInertial[i] + RIB[i][j] * FrameBody[j];
        }
    }
}

void RotateFrame::RotateAngularRatesInertialToBody(float phi,float theta,float *omegaBody,float *omegaInertial){
    ctheta = cos(theta);
    cphi   = cos(phi);
    stheta = sin(theta);
    sphi   = sin(phi);
    ttheta = tan(theta);
    sectheta = 1/cos(theta);
    
 float LIB[3][3] = {{1, sphi*ttheta  , cphi*ttheta  },
                    {0, cphi         , -sphi        },
                    {0, sphi*sectheta,cphi*sectheta}};
    for (int i=0;i<3;i++)
    {
        omegaBody[i] = 0;
        for (int j=0;j<3;j++)
        {
            omegaBody[i] = omegaBody[i] + LIB[i][j] * omegaInertial[j];
        }
    }
}

void RotateFrame::RotateAngularRatesBodyToInertial(float phi,float theta,float *omegaBody,float *omegaInertial){
    ctheta = cos(theta);
    cphi   = cos(phi);
    stheta = sin(theta);
    sphi   = sin(phi);
    ttheta = tan(theta);
    sectheta = 1/cos(theta);
    
    float LBI[3][3] = {{1, 0     , -stheta      },
                       {0, cphi  , ctheta*sphi  },
                       {0, -sphi , ctheta*cphi }};
    for (int i=0;i<3;i++)
    {
        omegaInertial[i] = 0;
        for (int j=0;j<3;j++)
        {
            omegaInertial[i] = omegaInertial[i] + LBI[i][j] * omegaBody[j];
        }
    }
}
