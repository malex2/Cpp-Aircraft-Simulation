//
//  AeroModel.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/20/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef AeroModel_hpp
#define AeroModel_hpp

#include "generic_model.hpp"

class AeroModelBase : public GenericForceModel
{
public:
    // Constructor
    AeroModelBase(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    // Getters
    AngleType<float>* getAeroEuler(void) { return aeroEuler; };
    
protected:
    // Class references
    class DynamicsModel   *pDyn;
    class RotateFrame     *pRotate;
    class PropulsionModel *pProp;
    class ActuatorModel   *pAct;
    class AtmosphereModel *pAtmo;
    class Time            *pTime;
    class GroundModel     *pGnd;
    
    // Internal Update Functions
    
    // Update alpha and beta
    void updateAeroAngles(void);
    
    // Override this definition in most cases
    virtual void updateAeroCoeffs(void);
    
    // Use this base class definition in most cases
    virtual void updateAeroForces(void);
    
    // Variables
    AngleType<float> aeroEuler[3]; // {0, alpha, -beta}
    AngleType<float> alpha, beta;

    // Print variables
    float aeroForce[3];       // Lift, Drag, Side Force
    float bodyForceCoeff[3];  // CX, CY, CZ
    float bodyMomentCoeff[3]; // Cl, Cm, Cn
    float betaPrint, alphaPrint;
    
    // Aero Matrix
    enum coeffType {iCd, iCL, iCX, iCY, iCZ, iCl, iCm, iCn, nCoeff};
    enum coeffDervType {constant, du, dalpha, dbeta, dp, dq, dr, delevator, daileron, drudder, dthrottle, nDeriv};
    
    float aeroMatrix[nCoeff][nDeriv];
    float coeffs[nCoeff];
    float inputs[nDeriv];
    
    float Cdarray[nDeriv];
    float CLarray[nDeriv];
    float CXarray[nDeriv];
    float CYarray[nDeriv];
    float CZarray[nDeriv];
    float Clarray[nDeriv];
    float Cmarray[nDeriv];
    float Cnarray[nDeriv];
    
    // Note: CX & CZ can substitute CL and Cd derivatives
    // [ Cd ]   [ Cdo  Cdu Cda Cdb Cdp Cdq Cdr Cdde Cdda Cddr CddT ]   [ 1      ]
    // [ CL ]   [ CLo  CLu CLa CLb CLp CLq CLr CLde CLda CLdr CLdT ]   [ u      ]
    // [ CX ]   [ CXo  CXu CXa CXb CXp CXq CXr CXde CXda CXdr CXdT ]   [ alpha  ]
    // [ CY ] = [ CYo  CYu CYa CYb CYp CYq CYr CYde CYda CYdr CYdT ] x [ beta   ]
    // [ CZ ]   [ CZo  CZu CZa CZb CZp CZq CZr CZde CZda CZdr CZdT ]   [ p      ]
    // [ Cl ]   [ Clo  Clu Cla Clb Clp Clq Clr Clde Clda Cldr CldT ]   [ q      ]
    // [ Cm ]   [ Cmo  Cmu Cma Cmb Cmp Cmq Cmr Cmde Cmda Cmdr CmdT ]   [ r      ]
    // [ Cn ]   [ Cno  Cnu Cna Cnb Cnp Cnq Cnr Cnde Cnda Cndr CndT ]   [ de     ]
    //                                                                 [ da     ]
    //                                                                 [ dr     ]
    //                                                                 [ dT     ]
    
    //coefficients =                 aeroMatrix                     x    inputs
};

class SimpleRCAeroModel : public AeroModelBase
{
public:
    // Constructor
    SimpleRCAeroModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    typedef AeroModelBase Base;
    
    // Use Base initialize
    //virtual void initialize(void);
    
    // Use Base update
    //virtual bool update(void);
    
private:
    float dCd_dCL;

    // Update drag coefficients
    virtual void updateAeroCoeffs(void);
};

class RCAeroModel : public AeroModelBase
{
public:
    // Constructor
    RCAeroModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    typedef AeroModelBase Base;
    
    // Use Base initialize
    //virtual void initialize(void);
    
    // Use Base update
    //virtual bool update(void);
    
private:
    
    virtual void updateAeroCoeffs(void);
    
    // Note: CX & CZ can substitute CL and Cd derivatives
    // [ Cd ]   [ Cdo  0   0   0   0   0   0   0    0    0    0    ]   [ 1      ]
    // [ CL ]   [ CLo  0   CLa 0   0   CLq 0   0    0    0    0    ]   [ u      ]
    // [ CX ]   [ 0    CXu CXa 0   0   0   0   CXde CXda CXdr 0    ]   [ alpha  ]
    // [ CY ] = [ 0    0   0   CYb CYp 0   CYr 0    CYda CYdr 0    ] x [ beta   ]
    // [ CZ ]   [ 0    CZu 0   0   0   0   0   CZde CZda CZdr 0    ]   [ p      ]
    // [ Cl ]   [ 0    0   0   Clb Clp 0   Clr 0    Clda Cldr 0    ]   [ q      ]
    // [ Cm ]   [ Cmo  0   Cma 0   0   Cmq 0   Cmde Cmda Cmdr 0    ]   [ r      ]
    // [ Cn ]   [ 0    0   0   Cnb Cnp 0   Cnr 0    Cnda Cndr 0    ]   [ de     ]
    //                                                                 [ da     ]
    //                                                                 [ dr     ]
    //                                                                 [ dT     ]
    
    // Aerodynamic coefficients as a function of Reynolds number
    const static int nReynolds = 8;
    const float reynoldsVec[nReynolds] = {72492.35967, 79016.24268, 84086.99144, 91413.98715, 106452.9941, 126196.9146, 177280.9713, 275866.5754};
    
    enum longitudinalCoefficients {lookupCLo, lookupCdo, lookupCXu, lookupCXa, lookupCZu, lookupCLa, lookupCLq, lookupCmo, lookupCma, lookupCmq};
    const float AeroTable_LongitudinalT[nReynolds][10] {
        //CLo, Cdo ,    CXu    ,   CXa   ,    CZu     ,  CLa  ,  CLq  ,    Cmo    ,   Cma   ,   Cmq
        {0.31, 0.04, -0.25289  , 0.58102 , -0.034381  , 3.7883, 6.3436, 0.12744635, -1.5562 , -9.4643},
        {0.31, 0.04, -0.17556  , 0.45587 , -0.012854  , 3.9413, 6.4242, 0.0820698 , -1.3519 , -9.3511},
        {0.31, 0.04, -0.13593  , 0.39016 , -0.0058827 , 4.0162, 6.4782, 0.06193474, -1.2391 , -9.2764},
        {0.31, 0.04, -0.096662 , 0.32141 , -0.0014787 , 4.0878, 6.5454, 0.04438665, -1.1164 , -9.1863},
        {0.31, 0.04, -0.5214   , 0.23267 , 0.00060243 , 4.1653, 6.6501, 0.01808996, -0.95027, -9.0499},
        {0.31, 0.04, -0.026271 , 0.16717 , 0.00039715 , 4.2072, 6.7408, 0.02221337, -0.82203, -8.9323},
        {0.31, 0.04, -0.0069296, 0.092156, -0.00034931, 4.2345, 6.8574, 0.02221371, -0.67029, -8.7783},
        {0.31, 0.04, -0.0017353, 0.048239, -0.00041963, 4.2385, 6.9307, 0.02591836, -0.57985, -8.6788}
    };
    float AeroTable_Longitudinal[10][nReynolds];
    
    enum lateralCoefficients {lookupCYb, lookupCYp, lookupCYr, lookupClb, lookupClp, lookupClr, lookupCnb, lookupCnp, lookupCnr};
    const float AeroTable_LateralT[nReynolds][9] {
        // CYb   ,      CYp   ,   CYr  ,    Clb    ,    Clp  ,  Clr    ,   Cnb  ,    Cnp    ,   Cnr
        {-0.3394 , 0.093687   , 0.37337, 0.0073338 , -0.36929, 0.3314  , 0.16463, -0.2437   , -0.18339},
        {-0.34742, 0.070075   , 0.38685, -0.0049111, -0.37514, 0.2856  , 0.17192, -0.19782  , -0.19381},
        {-0.35111, 0.055992   , 0.39291, -0.012252 , -0.3785 , 0.25965 , 0.17523, -0.17088  , -0.19849},
        {-0.35447, 0.039953   , 0.39825, -0.02064  , -0.38231, 0.23086 , 0.17819, -0.14042  , -0.20256},
        {-0.35792, 0.017328   , 0.40325, -0.032513 , -0.38775, 0.19107 , 0.18102, -0.09769  , -0.20625},
        {-0.35968, -0.00059002, 0.40526, -0.04194  , -0.39218, 0.15978 , 0.18222, -0.063944 , -0.20752},
        {-0.36076, -0.021952  , 0.40553, -0.053196 , -0.39764, 0.12224 , 0.18251, -0.023697 , -0.20722},
        {-0.3609 , -0.034615  , 0.40461, -0.059872 , -0.40098, 0.099661, 0.1821 , 0.00021518, -0.20612}
    };
    float AeroTable_Lateral[9][nReynolds];
    
    enum controlCoefficients {lookupCmde, lookupCYda, lookupClda, lookupCnda, lookupCYdr, lookupCldr, lookupCndr};
    const float AeroTable_ControlT[nReynolds][7] {
        // Cmde ,   CYda  ,   Clda ,   Cnda  ,   CYdr ,   Cldr  ,   Cndr
        {-1.1968, -0.13034, 0.40029, 0.065812, 0.24698, 0.027361, -0.13426},
        {-1.1871, -0.13034, 0.40029, 0.065812, 0.24698, 0.027361, -0.13426},
        {-1.1826, -0.13034, 0.40029, 0.065812, 0.24698, 0.027361, -0.13426},
        {-1.1782, -0.13034, 0.40029, 0.065812, 0.24698, 0.027361, -0.13426},
        {-1.1722, -0.13034, 0.40029, 0.065812, 0.24698, 0.027361, -0.13426},
        {-1.1669, -0.13034, 0.40029, 0.065812, 0.28305, 0.033462, -0.15261},
        {-1.1588, -0.13337, 0.39877, 0.067916, 0.28305, 0.033462, -0.15261},
        {-1.1525, -0.13337, 0.39877, 0.067916, 0.28305, 0.033462, -0.15261}
    };
    float AeroTable_Control[7][nReynolds];
    
    // Control coefficients as a function of control deflection
    const static int nDe = 8;
    const float deVec[nDe] = {-15, -10, -7.5, -5, -2.5, 0, 2, 3};
    
    enum elevatorCoefficients {lookupCXde, lookupCZde};
    const float elevatorTableT[nDe][2] {
        //  CXde  ,   CZde
        {-0.12147 , -0.48619},
        {-0.10166 , -0.49146},
        {-0.089333, -0.49477},
        {-0.074767, -0.49847},
        {-0.053137, -0.50309},
        {-0.034996, -0.50583},
        {-0.012103, -0.50743},
        {0.0021437, -0.50725}
    };
    float elevatorTable[2][nDe];
    
    const static int nDa = 7;
    const float daVec[nDa] = {-15, -10, -5, 0, 5, 10, 15};
    
    enum aileronCoefficients {lookupCXda, lookupCZda, lookupCmda};
    const float aileronTableT[nDa][3] {
        // CXda   ,    CZda   ,    Cmda
        {0.15757  , -0.020987 , -0.0097151},
        {0.10848  , -0.014852 , -0.0077044},
        {0.062139 , -0.011088 , -0.011012 },
        {-0.003965, 0.00016137, 0         },
        {-0.062139, 0.014852  , 0.011012  },
        {-0.10848 , 0.014852  , 0.0077044 },
        {-0.15757 , 0.020987  , 0.0097151 }
    };
    float aileronTable[3][nDa];
    
    const static int nDr = 7;
    const float drVec[nDr] = {-15, -10, -5, 0, 5, 10, 15};
    
    enum rudderCoefficients {lookupCXdr, lookupCZdr, lookupCmdr};
    const float rudderTableT[nDr][3] {
        //  CXdr   ,    CZdr  ,    Cmdr
        {0.74095   , -0.40558 , -0.34345 },
        {-0.015745 , 0.08039  , 0.12488  },
        {0.0094144 , -0.045505, -0.067536},
        {0         , 0        , 0        },
        {-0.0094144, 0.045505 , 0.067536 },
        {0.015745  , -0.08039 , -0.12488 },
        {-0.74095  , 0.40558  , 0.34345  },
    };
    float rudderTable[3][nDr];
};

#endif /* AeroModel_hpp */
