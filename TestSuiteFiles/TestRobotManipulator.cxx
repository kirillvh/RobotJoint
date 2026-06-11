/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#include "TestRobotManipulator.h"

using namespace std;
using namespace Eigen;
using namespace RobotJoint;

TestRobotManipulator::TestRobotManipulator()
{
         TestRobotManipulator(1); // default case - construt with 6 DOF manipulator
}

TestRobotManipulator::TestRobotManipulator(int TestManipulatorIdc)
{
        RJMatrix3 inertia;
        RJVector4 compos;
        RJReal m;
        RJReal a;
        RJReal alpha;
        RJReal d;
        RJReal theta;
        RJReal roty;
        RJReal gear_rat=1.0;
        RJReal static_friction=1.7; 
        RJReal viscosity=0.3;
        JointLink *J;

        TestManipulatorId = TestManipulatorIdc;

        setNominalValues();
    
        if(TestManipulatorId==0)
        { // 2-link mode
                /*
                Link | a | alpha | d | theta | m |   com   | inertia
                0    | 5 | 0     | 0 |   0   | 2 |   [ 1 ] | [5 0 0 ]  
                     |           |           |       [ 0 ] | [0 1 0 ]  
                     |           |           |       [ 0 ] | [0 0 1 ]  
                1    | 3 | 0     | 0 |   0   | 11|   [ 1 ] | [1 0 0 ]  
                     |           |           |       [ 0 ] | [0 7 0 ]  
                     |           |           |       [ 0 ] | [0 0 1 ]  
                */

                //0-1
                inertia<<5,0,0,0,1,0,0,0,1;
//                 inertia<<0,0,0,0,0,0,0,0,0;
                m=2; 
                alpha=0;
                a=5;
                d=0;
                theta=0;
                roty=0;
//                 compos<<0,0,0,m;
                compos<<5,0,0,m;

                //J = new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,false);
                addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,false));

                //0-1
                inertia<<0,0,0,0,7,0,0,0,1;
//                 inertia<<0,0,0,0,0,0,0,0,0;
                m=11;
                alpha=0;
                a=3;
                d=0;
                theta=0;
                roty=0;
//                 compos<<0,0,0,m;
                compos<<3,0,0,m;
                addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,false) );

        }
        else 
        {
                //0-1
                inertia<<0.06672,0.0000,0.00002077,0.0000,0.02308,0.0000,0.00002077,0.000,0.08422;
                m=3.731/2.0; //### make this half the full value because the same JointLink is counted on left and right legs
                // m=0.0; // for testing the fixed case scenario
                alpha=0;
                a=-0.085;
                // a=0.0;
                d=-0.246;
                theta=PI/2.0;
                roty=0;

                compos<<-0.000,0.0,-0.018,m;

                addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,true) );
                
                //1-2
                inertia<<0.01683,0.00007964,0.0001713,0.00007964,0.1231,-0.001589,0.0001713,-0.001589,0.009480;
                m=2.4;
                alpha=PI/2.0;
                a=-0.0;
                d=-0.0;
                theta=0;
                roty=0;

                compos<<-0.009,0.029,0.091,m;


                addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,false) );

                //2-3
                inertia<<0.002336,-1.386E-6,-0.0003392,-1.386E-6,0.002506,0.00001707,-0.0003392,0.00001707,0.001115;
                m=1.241;
                alpha=PI/2.0; // 2011-07-12 changes from +PI/2.0
                a=0.0;
                d=-0.0;
                theta=-PI/2.0;
                roty=0;


                compos<<-0.015,0.000,-0.018,m;

                addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,false) );

                //3-4
                inertia<<0.01209,0.0005635,0.0002864,0.0005635,0.04118,-0.0001803,0.0002864,-0.0001803,0.03728;
                m=4.212;
                alpha=0;
                a=0.3;
                d=-0.0;
                theta=0;
                roty=0;


                compos<<0.175,0.016,0.015,m;

                addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,false) );

                //4-5
                m=3.101;
                alpha=0;
                a=0.3;
                d=-0.0;
                theta=0;
                roty=0;
                inertia<<0.006879,0.0001232,0.0004393,0.0001232,0.03529,-0.00008200,0.0004394,-0.00008199,0.03195;


                compos<<0.099,0.003,0.012,m;

                addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,false) );

                //5-6
                m=1.385;
//                 m=0;
                alpha=-PI/2.0;
                a=0.0;
                d=-0.0;
                theta=0;
                roty=0;
                inertia<<0.002224,0.0006932,0.0004036,0.0006932,0.002590,0.0002898,0.0004037,0.0002898,0.003401;


                // compos,0.3,0,0;
                compos<<-0.023,-0.016,0.013,m;

                addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,false) );

                //6-rf
                m=1.868;
// m=0;
                alpha=0;
                a=0.11;
//                 a=0.0;
                d=-0.0;
                theta=0;
                roty=-PI/2.0;
                inertia<<0.005236,0.00006792,-0.002636,0.00006790,0.01024,-0.0001132,-0.002636,-0.0001132,0.007148;

                compos<<0.053,0.001,-0.010,m;
//                 compos<<0.0,0.00,-0.00,m;

                //testing
                // m=1.385;
                // alpha=-PI/2.0;
                // a=0.0;
                // d=-0.0;
                // theta=0;
                // roty=0;
                //testing


                addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,false) );
                
                RJVectorX q = RJVectorX::Zero(6);
                setAngles(q);
        }
}

int TestRobotManipulator::setNominalValues()
{
        
        if(TestManipulatorId==0) // 2-link planar case
        {
                
        Jacnom.resize(6,2);
        COMJacnom.resize(6,2);
        COMJacwrtToolnom.resize(6,2);
                
        DOFnom = 2;                
                /*
                [ c12 -s12 0 (a1c1 + a2c12)]
                [ s12  c12 0 (a1s1 + a2s12)]
                [ 0     0  1      0        ]
                [ 0     0  0      1        ]
                */
        Tnom << 1,0,0,8,
                0,1,0,0,
                0,0,1,0,
                0,0,0,1;
        /*
        (2*(1;0;0)+11*([0;1;0]+[5;0;0]))/13
        */
        Vnom << (2.0*1.0+11.0*5.0)/13.0,11.0*1.0/13.0,0.0,13.0;
        
        /*
        [ (-a1s1-a2s12) -a2s12]
        [ ( a1c1+a2c12)  a2c12]
        [ 0                0  ]
        [ 0                0  ]
        [ 0                0  ]
        [ 1                1  ]
        */
        Jacnom << 0.0,0.0,
                  8.0,3.0,
                  0.0,0.0,
                  0.0,0.0,
                  0.0,0.0,
                  1.0,1.0;        
                            
        //COMJacnom << ;
                             
        //COMJacwrtToolnom << ;
        }
        else
        {
        Jacnom.resize(6,6); 
        COMJacnom.resize(6,6);
        COMJacwrtToolnom.resize(6,6);

        DOFnom = 6;
        Tnom << 1,1.61554e-15,-2.5638e-30,1.00971e-15,
                -1.61554e-15,1,-1.61554e-15,-0.085,
                0,1.61554e-15,1,-0.956,
                0,0,0,1;        
                
        Vnom << -0.00348938,-0.0848865,-0.454407,16.0725;                
        
        Jacnom << -1.14704e-15, 1.14704e-15,        0.71,         0.41,         0.11,  1.7771e-16,
                  1.14704e-15 ,        0.71, 3.01912e-35,  1.74343e-35,  4.67751e-36,        0.11,
                            0 , 1.14704e-15, 1.14704e-15,  6.62372e-16,   1.7771e-16,   1.7771e-16,
                            0 ,           1, 2.61002e-30,  2.61002e-30,  2.61002e-30,            1,
                            0 ,-1.61554e-15,          -1,          -1 ,           -1, -1.61554e-15,
                            1 , 1.61554e-15, -1.61554e-15, -1.61554e-15, -1.61554e-15,  1.61554e-15;        
                            
        COMJacnom <<   0.00975231,  4.14979e-16,     0.248458,    0.0839974,   0.00417788,  9.76372e-18,
                   -0.00348938,     0.248458,   -3.604e-18,  3.16997e-18,  4.10507e-18,   0.00615984,
                             0,   -0.0084084,   0.00223083,  -0.00196217,  -0.00254099,  0.000116223,
                             0,            1,  2.61002e-30,  2.61002e-30,  2.61002e-30,            1,
                             0, -1.61554e-15,           -1,           -1,           -1, -1.61554e-15,
                             1,  1.61554e-15, -1.61554e-15, -1.61554e-15, -1.61554e-15,  1.61554e-15;                            
                             
        COMJacwrtToolnom << 0.00963886, -3.09857e-15,    -0.963135,    -0.827596,    -0.607416, -1.95658e-15,
                           -0.00697875,    -0.963135,  1.56005e-15,  1.35463e-15,  1.00079e-15,    -0.605434,
                          -1.12745e-17,  -0.00829494,  -0.00125855,  -0.00545155,  -0.00603036,  0.000229678,
                           -2.5638e-30,            1, -1.61554e-15, -1.61554e-15, -1.61554e-15,            1,
                          -1.61554e-15, -3.23109e-15,           -1,           -1,           -1, -3.23109e-15,
                                     1,  1.61554e-15, -3.23109e-15, -3.23109e-15, -3.23109e-15,  1.61554e-15;                             
        }
        return RJ_OK;
}

bool TestRobotManipulator::TestAll(bool verbose)
{
        RJVectorX q = RJVectorX::Zero(DOFnom);
        if(TestManipulatorId==0){
        q(0) = PI*0.0;
        q(1) = PI*0.0;
                //  RNE : Experimental 
        RJVectorX dq = RJVectorX::Zero(DOFnom);
        dq(0) = 1.0;
        RJVectorX ddq = RJVectorX::Zero(DOFnom);
        RJVector6 Fextern = RJVector6::Zero();
        cout<<"Testing RNE method                                       ";
        cout << "RNE "<<RNE(q, dq, ddq, Fextern)<<endl;
/*        if(jsense == jsensenom) cout<<"Pass \n";
        else cout<<"Fail \n";  */      

        //temporary testing function follows from here onwards. TODO make it into a proper unit test
        cout << endl;
        
                RJVector2 velVsfrictionTau;
                RJVectorX simtau = RJVectorX::Zero(2);
                RJVector6 nil = RJVector6::Zero();
                RJVector2 nil2 = RJVector2::Zero();
                RJVectorX simmotor(2);
                RJVectorX simq = RJVectorX::Zero(2);
                RJVectorX simdq = RJVectorX::Zero(2);
                
                simq<< 1.0,1.0;
                update(simq); //do it two times to get the differentiators to catch up
                update(simq);
                update(simq);
//                 update(simq);
//                 simmotor=frictionIdentification(0, frictionTauVsVel, true);
        for(int i=0;i<15000;i++)
        {
                simmotor=frictionIdentification(0, velVsfrictionTau);
                Log("frictionID.dat",Samples2Sec(i), simmotor);
                Log("frictionID2.dat",Samples2Sec(i), velVsfrictionTau);
                simtau=RNE(getq(),getdq(),getddq(),nil);
//                 simtau=RNE(getq(),getdq(),nil2,nil);
                simdq(0) += 0.001*(1.0/getJoint(0)->getInertia()(2,2))*(simtau(0)+simmotor(0)+JointLinks[0]->staticFriction(dqi(0)));
                simdq(1) += 0.001*(1.0/getJoint(1)->getInertia()(2,2))*(simtau(1)+simmotor(1));
//                 simq(0) += 0.001*0.1*(simtau(0)+simmotor(0));
//                 simq(1) += 0.001*0.1*(simtau(1)+simmotor(1));
                simq(0) += 0.001*simdq(0);
                simq(1) += 0.001*simdq(1);
                
                update(simq);
                
                frictionIdentification_holdData(velVsfrictionTau);
                Log("frictionID3.dat",Samples2Sec(i), simq);
//                 cout<<"getJoint(0)->getInertia()(2,2))  "<<getJoint(0)->getInertia()(2,2)<<endl;
//                 cout<<"JointControllers[0]->getInertia()  "<<JointControllers[0].getInertia()<<endl;
        }
        cout << "identified paramaters " <<frictionIdentification_calculate().transpose();
        }
        else {
        _angles << 0 , 0 , 0.5*PI , 0 , 0 , 0;
//         _angles << 0 , 0.5*PI , 0 , 0 , 0 , -0.5*PI;
//         
//         RJVector3 ptest;
//         ptest << 0, 0, 1;
// //         cout<<"ptemp "<<BaseVector2ToolFrame(ptest,_angles)<<endl;
//         cout<<"ptemp "<<ROT(fk(_angles)).transpose()*ptest<<endl;
        
        RJVectorX nil = RJVectorX::Zero(6);
	RJVector6 nil2 = RJVector6::Zero();
        cout<<"Testing rne"<<endl;    
        RJVectorX ttemp = RNE(_angles, nil, nil, nil2);
        cout<<ttemp.transpose()<<endl;
        cout<<"xxx "<<getCOM(_angles,3)<<endl;
        cout<<"xxx "<<getCOM(_angles,2)<<endl;

	int DOF;
	cout<<"Testing RobotManipulator::getDOF()			";
	DOF = getDOF();
	if(DOF == DOFnom) cout<<"Pass \n";
	else cout<<"Fail \n";
	if(verbose) cout<<DOF<<"\n";

	RJMatrix4 T;

	cout<<"Testing RobotManipulator::fk()				";
	T = fk(q);
	if(matCompare4(T,Tnom)) cout<<"Pass \n";
	else cout<<"Fail \n";
	if(verbose) cout<<T<<"\n";
	
	RJVector4 V;

	cout<<"Testing RobotManipulator::getCOM()			";
	V =getCOM(q);
	if(vecCompare4(V,Vnom)) cout<<"Pass \n";
	else cout<<"Fail \n";
	if(verbose) cout<<V<<"\n";

	RJMatrixX Jac(6,6);

	cout<<"Testing RobotManipulator::getJacobian()			";
	Jac = getJacobian(q);
	if(matCompare(Jac,Jacnom)) cout<<"Pass \n";
	else cout<<"Fail \n";
	if(verbose)                cout<<Jac<<"\n";

	RJMatrixX COMJac(6,6);


	cout<<"Testing RobotManipulator::getCOMJacobian()		";
	COMJac = getCOMJacobian(q);
	if(matCompare(COMJac,COMJacnom)) cout<<"Pass \n";
	else cout<<"Fail \n";
	if(verbose) cout<<COMJac<<"\n";


	RJMatrixX COMJacwrtTool(6,6);



	cout<<"Testing RobotManipulator::getCOMJacobianWRTToolFrame()	";
	COMJacwrtTool = getCOMJacobianWRTFootFrame(q);
	if(matCompare(COMJacwrtTool,COMJacwrtToolnom)) cout<<"Pass \n";
	else cout<<"Fail \n";
	if(verbose) cout<<COMJacwrtTool<<"\n";

// 	int jsense,jsensenom;
// 	jsensenom = 0;

	cout<<"Testing fk() vel. vs. getJacobian vel.			";
	if(test_kinematic_jacobian(verbose)) cout<<"Pass \n";
	else cout<<"Fail \n";

	cout<<"Testing getCOM() vel. vs. getCOMJacobian vel.		";
	if(test_COM_jacobian(verbose)) cout<<"Pass \n";
	else cout<<"Fail \n";

	cout<<"Testing vels. of getcom vs getCOMJac. wrt2Tool		";
	if(test_COM_jacobian_wrt2tool(verbose)) cout<<"Pass \n";
	else cout<<"Fail \n";	
	
	cout<<"Testing fk() vel. vs. getJacobianWRTFootFrame vel        ";
	if(test_kinematic_jacobianWRTFootFrame(true)) cout<<"Pass \n";
	else cout<<"Fail \n";

        cout<<"Testing GRFC_TF() - Passthrough without limits          ";
        RJVectorX anglesin(6);
        anglesin << 0 , 0 , 0.3 , -0.6, 0.3, 0;
        update(anglesin);
        RJVectorX torquesin(6);
        torquesin << 1.0 , 2.0 , 3.3 , -4.6, 5.3, 6.0;
        RJVectorX fsens = RJVectorX::Zero(6);
        torquesin << 1.0 , 2.0 , 3.3 , -4.6, 5.3, 6.0;
        RJMatrixX maxwrench(6,2);
        maxwrench << 9999.9, -9999.9,
                     9999.9, -9999.9,
                     9999.9, -9999.9,
                     9999.9, -9999.9,
                     9999.9, -9999.9,
                     9999.9, -9999.9;
        Vector6b axes_to_use;
        axes_to_use   << true, false, false, true, true, false;
        VectorXb joints_to_use(6);
        joints_to_use << false, false, false, true, true, true;
        RJVectorX torques_result;
        
        torques_result = GRFC_TF(torquesin, fsens, maxwrench, axes_to_use,  joints_to_use, _sampling_frequency*0.05);
        if(Compare(torques_result, torquesin)) cout<<"Pass \n";
        else cout<<"Fail \n";   
        if(verbose) cout << "GRFC_TF() output : "<<torques_result<<endl;
        
//         cout<<"Testing GRFC_TF() - Passthrough with limits              ";
//         maxwrench(0,0) =  30;        maxwrench(0,1) = -30;
//         maxwrench(4,0) =  5;        maxwrench(4,1) = -5;
//         fsens<<30,0,0,6,5,0;
//         for(int i=0; i<100; i++){
//         torques_result = GRFC_TF(torquesin, fsens, maxwrench, axes_to_use,  joints_to_use, _sampling_frequency*0.05);
//         }

        cout<<"Testing CLIK_IK()  -- manipulator pos. mode              ";
        RJVector3 xref= TRANS(fk());
        xref(0)+=0.05;
        RJMatrix3 difdes;
        //rotate around x by 0.1 rad
        double rotx = 0.1;
        difdes << 1.0 , 0 , 0,
                    0 , cos(rotx), -sin(rotx),
                    0 , sin(rotx), cos(rotx);
        RJQuaternion oriref(ROT(fk())*difdes);
//         oriref.w() =  oriref.w();
        oriref.normalize();
        RJVectorX CLIK_angles = CLIK_IK(xref, oriref,IK_Kinematic,200,1.0);
        cout<<"clik angles"<<endl<<CLIK_angles.transpose()<<endl;
        cout<<"old x"<<endl<<TRANS(fk(_angles)).transpose()<<endl;
        cout<<"clik x"<<endl<<TRANS(fk(CLIK_angles)).transpose()<<endl;
        cout<<"old ori"<<endl<<ROT(fk(_angles))<<endl;
        cout<<"clik ori"<<endl<<ROT(fk(CLIK_angles))<<endl;
        
        cout<<"Testing CLIK_IK()  -- COM pos. mode                      ";
        xref= getCOM().segment(0,3);
        xref(0)+=0.05;
        difdes;
        //use previous declaration of the orientation
        oriref.normalize();
                CLIK_angles = CLIK_IK(xref, oriref,IK_COM,200,1.0);
        cout<<"clik angles"<<endl<<CLIK_angles.transpose()<<endl;
        cout<<"old x"<<endl<<getCOM(_angles).segment(0,3).transpose()<<endl;
        cout<<"clik x"<<endl<<getCOM(CLIK_angles).segment(0,3).transpose()<<endl;
        cout<<"old ori"<<endl<<ROT(fk(_angles))<<endl;
        cout<<"clik ori"<<endl<<ROT(fk(CLIK_angles))<<endl;
        
        cout<<"Testing CLIK_IK()  -- COM pos. wrt foot mode             ";
/*        RJVector3 c = getCOM(_angles).head(3);
        xref = BaseVector2ToolFrame(c, _angles);        
        xref(0)+=0.05;
        //use previous declaration of the orientation
        RJQuaternion oriref2(ROT(fk())*difdes);
        oriref2.normalize();
        CLIK_angles = CLIK_IK(xref, oriref2,IK_COMwrtFoot,200,1.0);
        cout<<"clik angles"<<endl<<CLIK_angles.transpose()<<endl;
        cout<<"old x"<<endl<<BaseVector2ToolFrame(c, _angles).transpose()<<endl;
        c = getCOM(CLIK_angles).head(3);
        cout<<"clik x"<<endl<<BaseVector2ToolFrame(c, CLIK_angles).transpose()<<endl;
        cout<<"old ori"<<endl<<ROT(fk(_angles))<<endl;
        cout<<"clik ori"<<endl<<ROT(fk(CLIK_angles))<<endl;*/    

        }
	
        return RJ_OK;
}

bool TestRobotManipulator::test_kinematic_jacobian(bool verbose)
/** @brief Debugging function to test the correctness of the kinematic Jacobian
*/
{
        RJVectorX angles(_DOF); 
        RJVectorX angles_old(_DOF);
        RJVectorX dq(_DOF);
        RJVector3 x = RJVector3::Zero();
        RJVector3 x_old = RJVector3::Zero();
        RJVector3 dx = RJVector3::Zero();
        RJVector3 dx_jacobian = RJVector3::Zero();
        RJVector3 error = RJVector3::Zero();
        RJReal t;
        RJReal tstep=0.000001;
        bool start=true;
	RJReal total_error;

        for(t=9.98;t<10.0;t+=tstep)
        {
                //shift current values to old values
                if(!start)
                {
                angles_old=angles;
                x_old=x;
                }
                //generate new joint angles
                for(int i=0;i<_DOF;i++)
                {
                        angles(i)=sin(t*i);
                }      
            
                //calculate new position
                x=fk(angles).block<3,1>(0,3);
      
		//special initilization thing
/*		if(!start)
		{
			angles_old=angles;
			x_old=x;
		} */    
		//calculate joint and cartesian velocities
		dx=(x-x_old)/tstep;
		dq=(angles-angles_old)/tstep;
		dx_jacobian=(getJacobian(angles)*dq).block<3,1>(0,0);
		//if(verbose)
		//{
  		//cout<<"Testing RobotManipulator::Jac | dx "<< dx.transpose() <<endl; // dx and dx_jac should be quite simmilar 
										       // otherwise theres a mistake somewhere
		//cout<<"Testing RobotManipulator::Jac | dx_jac "<< dx_jacobian.transpose() <<endl;
		//}
     	 	//compare results
     	  	error(0)+=sqrt((dx(0)-dx_jacobian(0))*(dx(0)-dx_jacobian(0)));
     	  	error(1)+=sqrt((dx(1)-dx_jacobian(1))*(dx(1)-dx_jacobian(1)));
     	  	error(2)+=sqrt((dx(2)-dx_jacobian(2))*(dx(2)-dx_jacobian(2)));

     	 	if(start)
		{
			error = RJVector3::Zero();
			start=false;
		}      
    	} 
	total_error = sqrt(error(0)*error(0) + error(1)*error(1) + error(2)*error(2));

	if(verbose)
	{
     		cout<<"Total error "<<  total_error <<endl;
	}

	if(total_error < 1.0) return true;
	else return false;
	}

bool TestRobotManipulator::test_kinematic_jacobianWRTFootFrame(bool verbose)
/** @brief Debugging function to test the correctness of the kinematic Jacobian
*/
{
        RJVectorX angles(_DOF); 
        RJVectorX angles_old(_DOF);
        RJVectorX dq(_DOF);
        RJVector3 x = RJVector3::Zero();
        RJVector3 x_old = RJVector3::Zero();
        RJVector3 dx = RJVector3::Zero();
        RJVector3 dx_jacobian = RJVector3::Zero();
        RJVector3 error = RJVector3::Zero();
        RJReal t;
        RJReal tstep=0.000001;
        bool start=true;
	RJReal total_error;

        for(t=9.98;t<10.0;t+=tstep)
        {
                //shift current values to old values
                if(!start)
                {
                angles_old=angles;
                x_old=x;
                }
                //generate new joint angles
                for(int i=0;i<_DOF;i++)
                {
                        angles(i)=sin(t*i);
                }      
            
                //calculate new position
                x=ROT(fk(angles)).transpose()*-TRANS(fk(angles));
      
		//special initilization thing
/*		if(!start)
		{
			angles_old=angles;
			x_old=x;
		} */    
		//calculate joint and cartesian velocities
		dx=(x-x_old)/tstep;
		dq=(angles-angles_old)/tstep;
		dx_jacobian=(getJacobianWRTFootFrame(angles)*dq).block<3,1>(0,0);
		if(verbose)
		{
  		cout<<"Testing RobotManipulator::Jac | dx "<< dx.transpose() <<endl; // dx and dx_jac should be quite simmilar 
										       // otherwise theres a mistake somewhere
		cout<<"Testing RobotManipulator::Jac | dx_jac "<< dx_jacobian.transpose() <<endl;
		}
     	 	//compare results
     	  	error(0)+=sqrt((dx(0)-dx_jacobian(0))*(dx(0)-dx_jacobian(0)));
     	  	error(1)+=sqrt((dx(1)-dx_jacobian(1))*(dx(1)-dx_jacobian(1)));
     	  	error(2)+=sqrt((dx(2)-dx_jacobian(2))*(dx(2)-dx_jacobian(2)));

     	 	if(start)
		{
			error = RJVector3::Zero();
			start=false;
		}      
    	} 
	total_error = sqrt(error(0)*error(0) + error(1)*error(1) + error(2)*error(2));

	if(verbose)
	{
     		cout<<"Total error "<<  total_error <<endl;
	}

	if(total_error < 1.0) return true;
	else return false;
}

bool TestRobotManipulator::test_COM_jacobian(bool verbose)
/** @brief Debugging function to test the correctness of the COM Jacobian
*/
{
        RJVectorX angles(_DOF); 
        RJVectorX angles_old(_DOF);
        RJVectorX dq(_DOF);
        RJVector3 x = RJVector3::Zero();
        RJVector3 x_old = RJVector3::Zero();
        RJVector3 dx = RJVector3::Zero();
        RJVector3 dx_jacobian = RJVector3::Zero();
        RJVector3 error = RJVector3::Zero();
        RJReal t;
        RJReal tstep=0.000001;
        bool start=true;
	RJReal total_error;

        for(t=9.98;t<10.0;t+=tstep)
        {
                //shift current values to old values
                if(!start)
                {
                angles_old=angles;
                x_old=x;
                }
                //generate new joint angles
                for(int i=0;i<_DOF;i++)
                {
                        angles(i)=sin(t*i);
                }      
            
                //calculate new position
                x=getCOM(angles).head(3);
      
		//special initilization thing
/*		if(!start)
		{
			angles_old=angles;
			x_old=x;
		}  */   
		//calculate joint and cartesian velocities
		dx=(x-x_old)/tstep;
		dq=(angles-angles_old)/tstep;
		dx_jacobian=(getCOMJacobian(angles)*dq).block<3,1>(0,0);
		//if(verbose)
		//{
  		//cout<<"Testing RobotManipulator::Jac | dx "<< dx.transpose() <<endl; // dx and dx_jac should be quite simmilar 
										       // otherwise theres a mistake somewhere
		//cout<<"Testing RobotManipulator::Jac | dx_jac "<< dx_jacobian.transpose() <<endl;
		//}
     	 	//compare results
     	  	error(0)+=sqrt((dx(0)-dx_jacobian(0))*(dx(0)-dx_jacobian(0)));
     	  	error(1)+=sqrt((dx(1)-dx_jacobian(1))*(dx(1)-dx_jacobian(1)));
     	  	error(2)+=sqrt((dx(2)-dx_jacobian(2))*(dx(2)-dx_jacobian(2)));

     	 	if(start)
		{
			error = RJVector3::Zero();
			start=false;
		}      
    	} 
	total_error = sqrt(error(0)*error(0) + error(1)*error(1) + error(2)*error(2));

	if(verbose)
	{
     		cout<<"Total error "<<  total_error <<endl;
	}

	if(total_error < 1.0) return true;
	else return false;
	}

bool TestRobotManipulator::test_COM_jacobian_wrt2tool(bool verbose)
/** @brief Debugging function to test the correctness of the COM Jacobian
*/
{
        RJVectorX angles = RJVectorX::Zero(_DOF); 
        RJVectorX angles_old(_DOF);
        RJVectorX dq(_DOF);
        RJVector3 x = RJVector3::Zero();
        RJVector3 x_old = RJVector3::Zero();
        RJVector3 dx = RJVector3::Zero();
        RJVector3 dx_jacobian = RJVector3::Zero();
        RJVector3 error = RJVector3::Zero();
	RJVector3 temp;
        RJReal t;
        RJReal tstep=0.000001;
        bool start=true;
	RJReal total_error;

        for(t=9.98;t<10.0;t+=tstep)
        {
                //shift current values to old values
                if(!start)
                {
                angles_old=angles;
                x_old=x;
                }
                //generate new joint angles
                for(int i=0;i<_DOF;i++)
                {
                        angles(i)=sin(t*i);
                }      
//                         angles(4)=sin(t*1);
//                         
//                         
//                         angles(4)=PI*0.5 + 0.00001;
//                         angles_old(4)=PI*0.5 - 0.0001;
            
                //calculate new position
		temp = getCOM(angles).head(3);
                x=BaseVector2ToolFrame(temp,angles);
                
//                 temp = getCOM(angles_old).head(3);
//                 x_old=BaseVector2ToolFrame(temp,angles_old);
      
		//special initilization thing
/*		if(!start)
		{
			angles_old=angles;
			x_old=x;
		}  */   
		//calculate joint and cartesian velocities
		dx=(x-x_old)/tstep;
		dq=(angles-angles_old)/tstep;
		dx_jacobian=(getCOMJacobianWRTFootFrame(angles)*dq).block<3,1>(0,0);
		if(verbose)
		{
  		cout<<"Testing RobotManipulator::Jac | dx "<< dx.transpose() <<endl; // dx and dx_jac should be quite simmilar 
										     // otherwise theres a mistake somewhere
		cout<<"Testing RobotManipulator::Jac | dx_jac "<< dx_jacobian.transpose() <<endl;
//                 cout<<"x_old "<<x_old<<endl;
//                 cout<<"x "<<x<<endl;
//                 cout<<"comjacwrtFoot "<<getCOMJacobianWRTFootFrame(angles)<<endl;
//                 cout<<"angles_old "<<angles_old<<endl;
//                 cout<<"dq "<<dq<<endl;
		}
     	 	//compare results
     	  	error(0)+=sqrt((dx(0)-dx_jacobian(0))*(dx(0)-dx_jacobian(0)));
     	  	error(1)+=sqrt((dx(1)-dx_jacobian(1))*(dx(1)-dx_jacobian(1)));
     	  	error(2)+=sqrt((dx(2)-dx_jacobian(2))*(dx(2)-dx_jacobian(2)));

     	 	if(start)
		{
			error = RJVector3::Zero();
			start=false;
		}      
    	} 
	total_error = sqrt(error(0)*error(0) + error(1)*error(1) + error(2)*error(2));

	if(verbose)
	{
     		cout<<"Total error "<<  total_error <<endl;
                cout<<"Sum error "<<  error.transpose() <<endl;
	}

	if(total_error < 1.0) return true;
	else return false;
}
