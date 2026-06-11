/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#include "TestController.h"

using namespace std;
using namespace Eigen;
using namespace RobotJoint;

TestController::TestController()
{
         TestController(0);
}

TestController::TestController(int x)
{
        switch(x)
        {
                case 0:
                        A.resize(2,2);
                        B.resize(2,1);
                        C.resize(2,1);
                        A<<0, 1, -5.0, -1.0;
                        B<<0, 1;
                        C<<1, 0;                        
                        break;
                case 1:
                        break;
                case 2:
                        break;
        }
}

bool TestController::TestAll(bool verbose)
{
        cout<<"Testing PD controller()                                 ";
        if(TestPDFeedback()) cout<<"Pass \n";
        else cout<<"Fail \n";
        
        cout<<"Testing PD_P_Obs_Controller()                           ";
        if(TestPD_P_Obs_Controller(verbose)) cout<<"Pass \n";
        else cout<<"Fail \n";
        if(verbose)        cout<<"   ....check TestPD_P_Obs_Controller.dat logfile        "<<endl;

                RJReal dt = 0.001;
                A.resize(3,3);
                A<<
                        1,      dt,      dt*dt/2.0,
                        0,      1,      dt,
                        0,      0,      1;
                B.resize(3,1);
                B<<
                        dt*dt*dt/6.0,
                        dt*dt/2.0,
                        dt;
                C.resize(1,3);
                C<<     
                        1,      0,      -0.48/9.81;
                Q.resize(3,3);
                Q<<
                        1,      0,      0,
                        0,      0,      0,
                        0,      0,      0;
                R.resize(1,1);
                R(0,0) = 1E-6;        
        
                RJMatrixX GInom = RJMatrixX::Zero(1,1);
                RJMatrixX GXnom = RJMatrixX::Zero(1,3);
                GInom<<
                        851.106;
                #ifndef USE_FLOAT
                GXnom<<
                        382004, 85919,  318.513;
                #endif
                #ifdef USE_FLOAT
                GXnom<<
                        382018, 85903.9,  318.513;                        
                #endif
                cout<<"Testing CalcPreviewGain()                               ";
                 CalcPreviewGain(2000);
                 if( Compare(GInom,GI,1.0) && Compare(GXnom,GX,1.0) ) cout<<"Pass \n";
                 else cout<<"Fail \n";
                 if(verbose)
                 {
                        cout<< "GI: "<<endl<<GI<<endl;
                        cout<< "GX: "<<endl<<GX<<endl;
                        cout << "Writing Preview Gain to TestPreviewGain.dat ..."<<endl;
                        Log("TestPreviewGain.dat",preview_gain);
                 }

}

bool TestController::TestPDFeedback()
{
//         return false;
        RJVectorX x    = RJVectorX::Zero(2);
        RJVectorX xnew = RJVectorX::Zero(2);
        RJVectorX xref = RJVectorX::Zero(2);
        RJVectorX u;
        RJReal dt = 0.001;
        
        xref<<1, 0;
        
        Gains.resize(1,2);
        Gains<< 100, 1;
        
        for(int t=0;t<20000;t++)
        {
                u = Feedback(xref, x);
                xnew = Predict(x, u, dt);
                x = xnew;
        }
        
        return true;
}

bool TestController::TestPD_P_Obs_Controller(bool verbose)
{
//         return false;
        RJVectorX x    = RJVectorX::Zero(2);
        RJVectorX xnew = RJVectorX::Zero(2);
        RJVectorX xref = RJVectorX::Zero(2);
        RJVectorX u(1);
        RJReal dt = 0.001;
        RJReal disturbance = 2.0;
        
        xref<<1, 0;
        
        Gains.resize(1,2);
        Gains<< 100.0, 0.1;        
        
        for(int t=0;t<20000;t++)
        {

                u(0) = PD_P_Obs_Controller(1.0, 1.0, 1.0, x(0), 5.0 /*x_ref*/,
                        1.0, 10.0
//                                            ,999.0,-999.0,0.35,-.35 //torque/velocity limits
                        );
                u(0) += disturbance;
                xnew = Predict(x, u, dt);
                x = xnew;
                if (verbose) Log("TestPD_P_Obs_Controller.dat",t*0.001,x);
        }
        
        if(fabs(x(0)-5.0)<0.001) return true;
        else                     return false;
}