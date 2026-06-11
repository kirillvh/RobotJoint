/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#include "TestRobot.h"

using namespace std;
using namespace Eigen;
using namespace RobotJoint;

TestRobot::TestRobot()
{
         TestRobot(0);
}

TestRobot::TestRobot(int x)
{
// if(x==0)
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
        RJReal static_friction=0.0; 
        RJReal viscosity=0.0;

        RJVectorX qleft  = RJVectorX::Zero(6);
        RJVectorX qright = RJVectorX::Zero(6);
        RJVectorX qtrunk = RJVectorX::Zero(1);        

        //in a real application the manipulator should be initilized first then added at the end
        addManipulator(RJ_LeftLeg, new RobotManipulator(RJ_LeftLeg, qleft));
        addManipulator(RJ_RightLeg, new RobotManipulator(RJ_RightLeg, qright));
//         addManipulator(RJ_Trunk, new RobotManipulator(RJ_Trunk, qtrunk));        
    
        
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

        compos<<-0.000,0.0,-0.018, m;

        getManipulator(RJ_RightLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, true) );
        
        //1-2
        inertia,0.01683,0.00007964,0.0001713,0.00007964,0.1231,-0.001589,0.0001713,-0.001589,0.009480;
        m=2.4;
        alpha=PI/2.0;
        a=-0.0;
        d=-0.0;
        theta=0;
        roty=0;

        compos<<-0.009,0.029,0.091, m;


        getManipulator(RJ_RightLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );

        //2-3
        inertia<<0.002336,-1.386E-6,-0.0003392,-1.386E-6,0.002506,0.00001707,-0.0003392,0.00001707,0.001115;
        m=1.241;
        alpha=PI/2.0; // 2011-07-12 changes from +PI/2.0
        a=0.0;
        d=-0.0;
        theta=-PI/2.0;
        roty=0;


        compos<<-0.015,0.000,-0.018, m;

        getManipulator(RJ_RightLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );

        //3-4
        inertia<<0.01209,0.0005635,0.0002864,0.0005635,0.04118,-0.0001803,0.0002864,-0.0001803,0.03728;
        m=4.212;
        alpha=0;
        a=0.3;
        d=-0.0;
        theta=0;
        roty=0;


        compos<<0.175,0.016,0.015, m;

        getManipulator(RJ_RightLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );

        //4-5
        m=3.101;
        alpha=0;
        a=0.3;
        d=-0.0;
        theta=0;
        roty=0;
        inertia<<0.006879,0.0001232,0.0004393,0.0001232,0.03529,-0.00008200,0.0004394,-0.00008199,0.03195;


        compos<<0.099,0.003,0.012, m;

        getManipulator(RJ_RightLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );

        //5-6
        m=1.385;
        alpha=-PI/2.0;
        a=0.0;
        d=-0.0;
        theta=0;
        roty=0;
        inertia<<0.002224,0.0006932,0.0004036,0.0006932,0.002590,0.0002898,0.0004037,0.0002898,0.003401;


        // compos<<0.3<<0<<0;
        compos<<-0.023,-0.016,0.013,m;

        getManipulator(RJ_RightLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );

        //6-rf
        m=1.868;
        alpha=0;
        a=0.11;
        d=-0.0;
        theta=0;
        roty=-PI/2.0;
        inertia<<0.005236,0.00006792,-0.002636,0.00006790,0.01024,-0.0001132,-0.002636,-0.0001132,0.007148;

        compos<<0.053,0.001,-0.010, m;

        //testing
        // m=1.385;
        // alpha=-PI/2.0;
        // a=0.0;
        // d=-0.0;
        // theta=0;
        // roty=0;
        //testing


        getManipulator(RJ_RightLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );
        
        
        // cerr<<"im3l a"<<endl;
        //0-7
        inertia<<0.06672,0.0000,0.00002077,0.0000,0.02308,0.0000,0.00002077,0.000,0.08422;
        m=3.731/2.0; //### make this half the full value because the same JointLink is counted on left and right legs
        alpha=0;
        a=0.085;
        // a=0.0;
        d=-0.246;
        theta=PI/2.0;
        roty=0;

        compos<<-0.000,0.0,-0.018, m;

        getManipulator(RJ_LeftLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, true));
        //JointLinks.push_back(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, m,true) );

        //7-8
        m=2.4;
        alpha=PI/2.0;
        a=-0.0;
        d=-0.0;
        theta=0;
        roty=0;


        inertia<<0.01683,0.00007974,0.0001708,0.00007974,0.1231,-0.001589,0.0001708,-0.001589,0.009480;


        compos<<0.009,0.029,0.091, m;
        getManipulator(RJ_LeftLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );
        //8-9
        //inertia<<0.002336<<1.392E-6<<0.0003393<<1.392E-6<<0.002506<<0.00001731<<0.0003393<<0.00001731<<0.001115;
        //////////////////////
        inertia<<0.01683,0.00007964,0.0001713,0.00007964,0.1231,-0.001589,0.0001713,-0.001589,0.009480;
        m=1.241;
        alpha=PI/2.0; //2011-07-12 changed from +PI/2.0
        a=0.0;
        d=-0.0;
        theta=-PI/2.0;
        roty=0;

        compos<<0.015,0.000,-0.018, m;
        getManipulator(RJ_LeftLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );
        //9-10
        inertia<<0.01238,0.0005252,-0.0004428,0.0005254,0.04278,0.0001722,-0.0004427,0.0001722,0.03881;

        m=4.212;
        alpha=0;
        a=0.3;
        d=-0.0;
        theta=0;
        roty=0;

        compos<<0.175,0.016,-0.015, m;
        getManipulator(RJ_LeftLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );
        //10-11
        inertia<<0.006879,0.0001228,-0.0004393,0.0001227,0.03519,0.00008233,-0.0004394,0.00008233,0.03195;

        m=3.101;
        alpha=0;
        a=0.3;
        d=-0.0;
        theta=0;
        roty=0;

        compos<<0.099,0.003,-0.012, m;
        getManipulator(RJ_LeftLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );

        //11-12
        inertia<<0.002224,0.0006926,-0.0004043,0.0006926,0.002590,-0.0002893,-0.0004043,-0.0002894,0.003400;

        m=1.385;
        alpha=-PI/2.0;
        a=0.0;
        d=-0.0;
        theta=0;
        roty=0;

        compos<<-0.023,-0.016,-0.013, m;
        getManipulator(RJ_LeftLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );
        //12-lf
        inertia<<0.005240,-0.00009460,-0.002636,-0.00009460,0.01024,0.00006550,-0.002636,0.00006550,0.007152;

        m=1.868;
        alpha=0;
        a=0.11;
        d=-0.0;
        theta=0;
        roty=-PI/2.0;

        compos<<0.053,-0.001,-0.010, m;
        getManipulator(RJ_LeftLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );

//         //0-13
//         inertia<<0.06672,0.0000,0.00002077,0.0000,0.02308,0.0000,0.00002077,0.000,0.08422;
//         m=5.87;//3.731; //### make this zero so that the mass of this JointLink is only counted once from the right leg
//         // m=0.0; // for testing the fixed case scenario
//         alpha=0;
//         a=0.00;
//         // a=0.0;
//         d=0.0;
//         theta=0;
//         roty=0;
// 
//         compos<<-0.000,0.0,0.0862, m;
// 
//         getManipulator(RJ_Trunk)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia, false) );
}

}

bool TestRobot::TestAll(bool verbose)
{
//         addMovementPhase(DoubleSup, 1.0/* time diff*/, 0.0/*Left Foot Pos x*/, 
//                          0.085/*Left Foot Pos y*/, 0.0, -0.085);
        addMovementPhase(DoubleSup, 1.0/*time diff*/, 0.0/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.0, -0.085);
        addMovementPhase(LeftSup, 1.0/*time diff*/, 0.0/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.0, -0.085);
        addMovementPhase(DoubleSup, 1.0/*time diff*/, 0.0/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.1, -0.085);
        addMovementPhase(RightSup, 1.0/*time diff*/, 0.0/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.1, -0.085);
        addMovementPhase(DoubleSup, 1.0/*time diff*/, 0.2/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.1, -0.085);
        addMovementPhase(LeftSup, 1.0/*time diff*/, 0.2/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.1, -0.085);
        
        addMovementPhase(DoubleSup, 1.0/*time diff*/, 0.2/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.3, -0.085);
        addMovementPhase(RightSup, 1.0/*time diff*/, 0.2/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.3, -0.085);
        addMovementPhase(DoubleSup, 1.0/*time diff*/, 0.4/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.3, -0.085);        
        addMovementPhase(LeftSup, 1.0/*time diff*/, 0.4/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.3, -0.085);
        addMovementPhase(DoubleSup, 1.0/*time diff*/, 0.4/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.5, -0.085);        
        addMovementPhase(RightSup, 1.0/*time diff*/, 0.4/*Left Foot Pos x*/, 
                         0.085/*Left Foot Pos y*/, 0.5, -0.085);
        
        cout<<"Testing MakeZMPTrjectory()                              ";
        RJMatrixX ZMPTrj = MakeZMPTrajectory();
//         cout << ZMPTrj<<endl;
//         RJMatrixX ZMPTrj = RJMatrixX::Zero(2,2);
        Log("ZMPTrj.dat", ZMPTrj);
        cout<< "Check ZMPTrj.dat"<<endl;
        cout<<"Testing MakeCOMTrjectory()                              ";
        RJMatrixX COMTrj =  MakeCOMTrajectory(ZMPTrj);
//         Log("COMTrj.dat",COMTrj.block(0,0,COMTrj.rows(),1));
        Log("COMTrj.dat",COMTrj);
        cout<< "Check COMTrj.dat"<<endl;
        cout<<"Testing CL_IK()                                         ";
// cout<<COMTrj<<endl;
        cout << "TODO Robot::Clik not fully implemented, not fully tested"<<endl;
//         CL_IK(RJVector3(0,0,0.3), RJQuaternion(RJMatrix3::Identity()), RJVector3(0,0.3,0.2), RJQuaternion(RJMatrix3::Identity()));        
        
        cout<<"Testing Scaled COMJacobian()                            ";
      //  if( test_COM(verbose) ) cout<<"Pass \n";
        //else cout<<"Fail \n";
        
        
        cout<<"Testing MakeJointTrajectory()                           ";
        MakeJointTrajectory(COMTrj);
	//testing COM velocity predicition
// 	cout<<"Testing COMvelprediction                                ";
// 	#define dtsim 0.001
// 	for(double tsim = 0; tsim < 100; tsim += dtsim)
// 	{
// 		RJVectorX q_new = getManipulator(RJ_LeftLeg)->getq();
// 		q_new(2) = sin(tsim);
// 		q_new(0) = 0;
// 		q_new(1) = 0;
// 		q_new(3) = -sin(tsim);
// 		q_new(4) = 0;
// 		q_new(5) = 0;
// 
// 		RJVectorXMap qrobot;
// // 		qrobot.insert(make_pair(RJ_RightLeg,q_new));
// 		qrobot.insert(make_pair(RJ_LeftLeg,q_new));
// // 		RJVectorX qtrunk = RJVectorX::Zero(1);
// // 		qrobot.insert(make_pair(RJ_Trunk,qtrunk));
// // 		getManipulator(RJ_LeftLeg)->update(q_trunk);
// 		update(qrobot);
// 		static RJVector3 oldarm = RJVector3::Zero();
// 		RJVector3 newarm= ROT(getManipulator(RJ_LeftLeg)->fk(q_new)).transpose()*-TRANS(getManipulator(RJ_LeftLeg)->fk(q_new));
// 		RJVector3 dq_arm = (newarm - oldarm)/dtsim;
// 		oldarm = newarm;
// 		static RJVector3 oldcom = RJVector3::Zero();
// 		RJVector3 newcom = BaseVector2ToolFrame(getCOM().head<3>(),RJ_LeftLeg);
// 		RJVector3 comvelS = (newcom - oldcom)/dtsim;
// 		oldcom = newcom;
// 
// 
// 		RJVectorX J_arm = RJVectorX::Zero(6);
// 		RJVectorX q_leg = getManipulator(RJ_LeftLeg)->getq();
// 		J_arm << cos(q_leg(2) + q_leg(3) + q_leg(4)), 0, sin(q_leg(2) + q_leg(3) + q_leg(4)), 0, 0, 0;
// 
// 		RJMatrixX J_combined_left = getManipulator(RJ_LeftLeg)->getCOMJacobianWRTFootFrame()*getManipulator(RJ_LeftLeg)->getInvJacobianWRTFootFrame()*J_arm;
// // 		RJMatrixX J_combined_right = getCOMJacobianWRTFootFrame(RJ_RightLeg)*getManipulator(RJ_RightLeg)->getInvJacobianWRTFootFrame()*J_arm;
// 	
// 		RJVectorX comvelP = J_combined_left*dq_arm(0);// + J_combined_right*dq_arm(0);
// 		RJVectorX dq(6);
// 		dq<<0,0,cos(tsim),-cos(tsim),0,0;
// 		comvelP = getManipulator(RJ_LeftLeg)->getCOMJacobianWRTFootFrame()*dq;
// 
// 		if(tsim>dtsim){
// 		Log("comP.dat",tsim, comvelP);
// 		Log("comS.dat",tsim, comvelS);
// 		}
// 	}
        
//         (SupportPhase phase,RJReal start, RJReal end, RJReal footAposX, RJReal footAposY, RJReal footBposX, RJReal footBposY)

}


bool TestRobot::test_COM(bool verbose)
/** @brief Debugging function to test the correctness of the COM Jacobian
*/
{
        RJVectorXMap angles;
        RJVectorXMap angles_old;
//         RJVectorX angles_old(getManipulator(RJ_RightLeg)->getDOF());
        RJVectorX dq(getManipulator(RJ_RightLeg)->getDOF());
        RJVector3 x = RJVector3::Zero();
        RJVector3 x_old = RJVector3::Zero();
        RJVector3 dx = RJVector3::Zero();
        RJVector3 dx_jacobian = RJVector3::Zero();
        RJVector3 error = RJVector3::Zero();
        RJVector3 temp;
        RJMatrixX jac;
        RJReal t;
        RJReal tstep=0.000001;
        bool start=true;
        RJReal total_error;
        ManipMapType::iterator it;
        
        RJVectorX qleft  = RJVectorX::Zero(6);
        RJVectorX qright = RJVectorX::Zero(6);
        RJVectorX qtrunk = RJVectorX::Zero(1);
        
        angles.insert(make_pair(RJ_RightLeg,qright));
        angles.insert(make_pair(RJ_LeftLeg,qleft));
        angles.insert(make_pair(RJ_Trunk,qtrunk));
        
        angles_old.insert(make_pair(RJ_RightLeg,qright));
        angles_old.insert(make_pair(RJ_LeftLeg,qleft));
        angles_old.insert(make_pair(RJ_Trunk,qtrunk));
        
        

//         RJVectorX::Zero(getManipulator(RJ_RightLeg)->getDOF())
//         for(it=Manipulators.begin(); it!=Manipulators.end(); it++) angles.insert(make_pair());
        for(it=Manipulators.begin(); it!=Manipulators.end(); it++) angles[it->first] = it->second->getAngles();
        for(it=Manipulators.begin(); it!=Manipulators.end(); it++) angles_old[it->first] = it->second->getAngles();
        
        for(t=9.98;t<10.0;t+=tstep)
        {
                //shift current values to old values
                if(!start)
                {
                angles_old=angles;
                x_old=x;
                }
                //generate new joint angles
                for(int i=0;i<getManipulator(RJ_RightLeg)->getDOF();i++)
                {
                        angles[RJ_RightLeg](i)=sin(t*i);
                }      
            
                //calculate new position
                temp = getCOM(angles).head(3);
                x = temp;
//                 x=BaseVector2ToolFrame(temp, angles[RJ_RightLeg], RJ_RightLeg);
      
                //special initilization thing
/*              if(!start)
                {
                        angles_old=angles;
                        x_old=x;
                }  */   
                //calculate joint and cartesian velocities
                dx=(x-x_old)/tstep;
                dq=(angles[RJ_RightLeg]-angles_old[RJ_RightLeg])/tstep;
                jac = getCOMJacobian(RJ_RightLeg, angles[RJ_RightLeg]);
                dx_jacobian=(jac*dq).block<3,1>(0,0);
                if(verbose)
                {
              cout<<"Testing RobotManipulator::Jac | dx "<< dx.transpose() <<endl; // dx and dx_jac should be quite simmilar 
// //                                                                                     otherwise theres a mistake somewhere
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
                cout<<"Sum error "<<  error.transpose() <<endl;
        }

        if(total_error < 1.0) return true;
        else return false;
}
