/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#include "TestRobotManipulator.h"
#include "TestRobotAlgo.h"
#include "TestController.h"
#include "TestRobotJointUtils.h"
#include "RobotJointUtils.h"
#include "TestRobot.h"
#include <iostream>

#include <iostream>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;
using namespace RobotJoint;



main(int argc, char **argv)
{
	bool verbose = false;
	bool all = false;
	bool robotmanipulator = false;
        bool controller = false;
        bool robotalgo = false;
        bool robotjointutils = false;
        bool robot = false;
        int TestRobotType = 1;

	for (int i=0; i<argc; i++)
	{
		if(strcmp( argv[i], "--verbose") == 0) verbose = true;
		if(strcmp( argv[i], "--robotmanipulator") == 0) robotmanipulator = true;
		if(strcmp( argv[i], "--robotalgo") == 0) robotalgo = true;
		if(strcmp( argv[i], "--robot") == 0) robot = true;
		if(strcmp( argv[i], "--controller") == 0) controller = true;
		if(strcmp( argv[i], "--all") == 0) all = true;
		if(strcmp( argv[i], "--2link-planar") == 0) TestRobotType = 0;
                if(strcmp( argv[i], "--robotjointutils") == 0) robotjointutils = true;
		if(strcmp( argv[i], "--help") == 0)
                {
                        cout<<"--all                    =       Tests all classes"<<std::endl;
                        cout<<"--2link-planar           =       Uses 2-link planar manipulator in tests(default is 6-DOF)"<<std::endl;
                        cout<<"--robotmanipulator       =       Tests RobotManipulator class"<<std::endl;
                        cout<<"--robotalgo              =       Tests RobotAlgo class"<<std::endl;
                        cout<<"--controller             =       Tests Controller class"<<std::endl;
                        cout<<"--robotjointutils        =       Tests functions in RobotJointUtils file"<<std::endl;
                        cout<<"--verbose                =       Displays results of tests"<<std::endl;
                }
	}
        
        try
        {
                if(all || robotmanipulator)
                {
        //                Vector4d test=Vector4d::Ones();
        //                Matrix4d t=Matrix4d::Ones();
        //                cout<<t*test;
                        TestRobotManipulator Manip(TestRobotType);
                        cout<<"Testing getCOM() vel. vs. getCOMJacobian vel.            \n";
                        cout<<"=============  Testing RobotManipulator =================\n";
                        Manip.TestAll(verbose);
                }
                
                if(all || robotalgo)
                {
        //                Vector4d test=Vector4d::Ones();
        //                Matrix4d t=Matrix4d::Ones();
        //                cout<<t*test;
                        TestRobotAlgo RobotAlg(2);
                        cout<<"=============  Testing RobotAlgo        =================\n";
                        RobotAlg.TestAll(verbose);
                        //RobotAlg.test_update_state_x();
                }
                
                if(all || controller)
                {
        //                Vector4d test=Vector4d::Ones();
        //                Matrix4d t=Matrix4d::Ones();
        //                cout<<t*test;
                        cout<<"=============  Testing Controller       =================\n";
                        TestController c(0);
                        c.TestAll(verbose);
                }
                if(all || robotjointutils)
                {
                        cout<<"=============  Testing RobotJointUtils  =================\n";
                        TestRobotJointUtils u;
                        u.TestAll(verbose);
                }
                if(all || robot)
                {
                        cout<<"=============  Testing Robot            =================\n";
                        TestRobot u(0);
                        u.TestAll(verbose);
                }
        }
        catch(const char* n)
        {
                        cout << "Fatal Exception: "<< n <<endl;
                        exit(1);
        }
}