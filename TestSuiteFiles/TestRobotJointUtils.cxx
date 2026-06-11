/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#include "TestRobotJointUtils.h"

using namespace std;
using namespace Eigen;
using namespace RobotJoint;

TestRobotJointUtils::TestRobotJointUtils()
{
//         Pnom.resize(4,4);
        setNominalValues();
}

int TestRobotJointUtils::setNominalValues()
{

        SolveQpnom1.resize(4);
        SolveQpnom1 << 2.916,    2.916,    4.763,    3.402;
        Pnom <<
        1.4236, 0.2827, -0.2700, 0.1664,
        0.2827, 0.0790, -0.0591, 0.0193,
        -0.270, 0.0591, 0.5216,  0.0312,
        0.1664, 0.0193, 0.0312,  0.0593;
        
        Knom << 56.54,   15.8, -11.82,   3.86;
        
        return RJ_OK;
}

bool TestRobotJointUtils::TestAll(bool verbose)
{
        try {
                RJMatrixX A = RJMatrixX::Zero(2,2);
                RJMatrixX C = RJMatrixX::Zero(2,2);
                RJVectorX B = RJVectorX::Zero(2);
                RJVectorX D = RJVectorX::Zero(2);
                RJVectorX SolveQpResult = RJVector4::Zero(4);
                RJVectorX SolveTnVP = RJVector4::Zero(4);
                
                A << 2.0, 0, 0, 2.0;// This is the general form form for the cost function f(x)=(x-xref)^2
                B << -2.0*7.0, -2.0*7.0; //General form is 2*[x1ref x2ref]'
                C << 1.0, 5.0/7.0, 1.0, 7.0/5.0;
                D << 5.0, 7.0;
                        
                // A = 2*A; // To compensate for the fact that solve_qp expects 0.5*A
                cout<<"Testing SolveQp()                                       ";
                SolveQpResult = SolveQp(A, B, C, D);
                if(Compare<RJVectorX>(SolveQpResult,SolveQpnom1)) cout<<"Pass \n";
                else cout<<"Fail \n";
                if(verbose) cout<<SolveQpResult<<"\n";
                
   
                 cout<<"Testing QP_Thiel_VanDePanne()                           ";
                 SolveTnVP = QP_Thiel_VanDePanne(A, B, C, D);
//                  return true;
                 if(Compare<RJVectorX>(SolveTnVP,SolveQpnom1)) cout<<"Pass \n";
                 else cout<<"Fail \n";
                 if(verbose) cout<<SolveTnVP<<"\n";
                
                MatrixXd TT = MatrixXd::Zero(8,8);
                TT<<
                0.943963 ,    -0.826976,    0.490882 ,    0.914667 ,  -0.0651457 ,  -0.0231449 ,      0.3115 ,  -0.0628043 ,
                0.0940634,      1.36687,    0.0271662,    -0.858671,    -0.164688,    0.0249399,     0.673703,    -0.215294,
                        0,            0,     0.716471,     0.526606,    -0.134603,   0.00689323,     0.568781,    -0.240151,
                        0,            0,   -0.0647176,     0.972658,    0.0025934,    -0.209805,     0.279498,      0.91912,
                        0,            0,            0,            0,      1.00459, -0.000190437,   -0.0185337,   0.00591858,
                        0,            0,            0,            0,  1.88869e-05,      1.00447,  -0.00592517,   -0.0185132,
                        0,            0,            0,            0,            0,            0,     0.995489, -0.000208106,
                        0,            0,            0,            0,            0,            0,  4.25062e-09,     0.995489;
                VectorXcd eignom = VectorXcd::Zero(8);
                eignom<< 
                complex<double>(1.15542, 0.1818),
                complex<double>(1.15542, -0.1818),
                complex<double>(0.84464, 0.1329),
                complex<double>(0.84464, -0.1329),
                complex<double>(1.00453, 0),
                complex<double>(1.00453, 0),
                complex<double>(0.99548, 0),         
                complex<double>(0.99548, 0);
                        
                cout<<"Testing OrdEig()                                        ";
                VectorXcd eigr;
                eigr = ordeig(TT);
                if(Compare<VectorXcd>(eignom,eigr)) cout<<"Pass \n";
                else cout<<"Fail \n";
                if(verbose) cout<<eigr<<"\n";                
        
                cout<<"Testing RiccatiSolve()                                  ";
                RJMatrixX P;
                A.resize(4,4);
                A<<
                        0,    1.0000,         0,         0,
                  -2.0000,   -2.0000,    2.0000,         0,
                        0,         0,         0,    1.0000,
                  10.0000,         0,  -10.0000,   -0.4000;
                B.resize(4,1);
                B<<          0,
                        0.2000,
                             0,
                             0;
                C.resize(2,4);
                C<<
                        1,      0,      0,      0,
                        0,      0,      1,      0;
                RJMatrixX Q(4,4);
                Q<<
                        1,     0,     0,     0,
                        0,     0,     0,     0,
                        0,     0,     1,     0,
                        0,     0,     0,     0;
                RJMatrixX R(1,1);
                R<<
                1E-3;
                P = RiccatiSolve(A, B, Q, R);
                        if(Compare(P,Pnom,0.3)) cout<<"Pass \n";
                        else cout<<"Fail \n";
                        if(verbose) cout<<P<<"\n";
                
                cout<<"Testing LQR()                                           ";
                RJMatrixX K = LQR(A, B, Q, R);
                RJVector4 K2 = K.row(0);
                
                        if(Compare(K2,Knom,0.1)) cout<<"Pass \n";
                        else cout<<"Fail \n";
                        if(verbose) cout<<K<<"\n";
                
                RJReal dt = 0.001;
                A.resize(3,3);
                A<<
                        1,      dt,      dt*dt/2,
                        0,      1,      dt,
                        0,      0,      1;
                B.resize(3,1);
                B<<
                        dt*dt*dt/6,
                        dt*dt/2,
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
                R(0,0) = 1E-3;
                RJMatrix3 Pdnom;
                Pdnom<<
                        632.956,        200,    31.6228,
                        200,         94.8685,    20,
                        31.6228,        20,     6.32456;

                cout<<"Testing RiccatiSolveDiscrete()                          ";
                RJMatrix3 Pd = RiccatiSolveDiscrete(A, B, Q, R);
                        if(Compare(Pdnom,Pd,0.1)) cout<<"Pass \n";
                        else cout<<"Fail \n";
                        if(verbose) cout<<Pd<<"\n";

                std::vector<RJMatrixX> preview_gain;
                RJMatrixX GI = RJMatrixX::Zero(1,1);
                RJMatrixX GX = RJMatrixX::Zero(1,3);
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
                R(0,0) = 1E-6;
                cout<<"Testing CalcPreviewGain()                               ";
                 CalcPreviewGain(A, B, C, Q, R, 2000, preview_gain, GI, GX);
                 if( Compare(GInom,GI,5.0) && Compare(GXnom,GX,5.0) ) cout<<"Pass \n";
                 else cout<<"Fail \n";
                 if(verbose)
                 {
                        cout<< "GI: "<<endl<<GI<<endl;
                        cout<< "GX: "<<endl<<GX<<endl;
                        cout << "Writing Preview Gain to TestPreviewGain.dat ..."<<endl;
                        Log("TestPreviewGain.dat",preview_gain);
                 }
        }
        catch(const char* n)
        {
                        cout << "Fatal Exception: "<< n <<endl;
                        exit(1);
        }

        return RJ_OK;
}
