/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#include "TestRobotAlgo.h"

using namespace std;
using namespace Eigen;
using namespace RobotJoint;

TestRobotAlgo::TestRobotAlgo()
{
        state_id = addState();
        t_sampling = 0.00001;
        setSamplingPeriod(t_sampling);
        setNominalValues();
}

int TestRobotAlgo::Reset()
{
        //clear
        state_id=0;
        t_sampling=0;
        state_memory.clear();
        state_memory_prev.clear();
        state_memory_prev_prev.clear();
        //reinitilize
        state_id = addState();
        t_sampling = 0.00001;
        setSamplingPeriod(t_sampling);
        return 0;
}

TestRobotAlgo::TestRobotAlgo(int x)
{
        state_id = addState();
        t_sampling = 0.00001;
        setSamplingPeriod(t_sampling);
        setNominalValues();
}

int TestRobotAlgo::setNominalValues()
{
        SolveQpnom1.resize(4);
        SolveQpnom1 << 2.91667, 2.91667, 25.1806, -17.0139;

        return RJ_OK;
}

bool TestRobotAlgo::TestAll(bool verbose)
{
        cout<<"Testing RobotAlgo::update_state_x()                     ";
	if(test_update_state_x()) cout<<"Pass \n";
	else cout<<"Fail \n";
	
        cout<<"Testing RobotAlgo::update_state_x_poly()                ";
        if(test_update_state_x_poly()) cout<<"Pass \n";
        else cout<<"Fail \n";

        Reset();
        cout<<"Testing RobotAlgo::update_state_dx()                    ";
        if(test_update_state_dx()) cout<<"Pass \n";
        else cout<<"Fail \n";
        
        Reset();
        cout<<"Testing RobotAlgo::update_state_ddx()                   ";
        if(test_update_state_ddx()) cout<<"Pass \n";
        else cout<<"Fail \n";
        
        Reset();
        cout<<"Testing RobotAlgo::Filter()                             ";
        RJMatrixX FilterTest = RJMatrixX::Zero(10000,2);
        for(int i=0; i<1000; i++)
        {
                FilterTest(i,0) = 10.0;
                FilterTest(i,1) = Filter("TestFilter",10.0,50.0 /*Hz*/);
        }
        for(int i=1000; i<10000; i++)
        {
                FilterTest(i,0) = sin(((i-1000)/1000.0)*(i-1000)*3.14/500.0);
                FilterTest(i,1) = Filter("TestFilter",sin(((i-1000)/1000.0)*(i-1000)*3.14/500.0),50 /*Hz*/);
        }
        Log("TestFilter.dat", FilterTest);
        cout << "Check Log \"TestFilter.dat\""<<endl;

        return RJ_OK;
}

bool TestRobotAlgo::test_update_state_x()
{
        Vector3d x,xnom;
        double t;
        
        //set inital values
        this->state_memory[state_id](1)=PI*cos(0);
        this->state_memory[state_id](0)=sin(0);
        this->state_memory_prev[state_id](1)=PI*cos(0);
        this->state_memory_prev[state_id](0)=sin(0);
        
        for(int i=0; i<20000;i++)
        {
                t = i*t_sampling;
                x = update_state_x(state_id, sin(PI*t), t_sampling);
                xnom << sin(PI*t) , PI*cos(PI*t) , -PI*PI*sin(PI*t);
                if(i>10)
                {
                        if(!Compare<Vector3d>(x,xnom,0.1)) return false;
                }
                
        }
        
        return true;
}


bool TestRobotAlgo::test_update_state_x_poly()
{
        Vector3d x,xnom;
        double t;
        
        //set inital values
        this->state_memory[state_id](1)=PI*cos(0);
        this->state_memory[state_id](0)=sin(0);
        this->state_memory_prev[state_id](1)=PI*cos(0);
        this->state_memory_prev[state_id](0)=sin(0);
        
        for(int i=0; i<20000;i++)
        {
                t = i*t_sampling;
                x = update_state_x_poly(state_id, sin(PI*t), t_sampling);
                xnom << sin(PI*t) , PI*cos(PI*t) , -PI*PI*sin(PI*t);
//                 cout<<"xnom "<<xnom.transpose()<<endl;
//                 cout<<"x "<<x.transpose()<<endl;
                if(i>10)
                {
                        if(!Compare<Vector3d>(x,xnom,0.1)) return false;
                }
                
        }
        
        return true;
}

bool TestRobotAlgo::test_update_state_dx()
{
        Vector3d x,xnom;
        double t;
        
        for(int i=0; i<20000;i++)
        {
                t = i*t_sampling;
                x = update_state_dx(state_id, PI*cos(PI*t), t_sampling);
                xnom << sin(PI*t) , PI*cos(PI*t) , -PI*PI*sin(PI*t);
                if(i>2)
                {
                        if(!Compare<Vector3d>(x,xnom,0.1)) return false;
                }
                
        }
        
        return true;
}

bool TestRobotAlgo::test_update_state_ddx()
{
        Vector3d x,xnom;
        double t;
        
        //set inital values
        this->state_memory[state_id](1)=PI*cos(0);
        this->state_memory[state_id](0)=sin(0);
        
        for(int i=0; i<20000;i++)
        {
                t = i*t_sampling;
                x = update_state_ddx(state_id, -PI*PI*sin(PI*t), t_sampling);
                xnom << sin(PI*t) , PI*cos(PI*t) , -PI*PI*sin(PI*t);
                if(i>2)
                {
                        if(!Compare<Vector3d>(x,xnom,0.1)) return false;
                }
                
                //RJVector3 update_state_dx(int id, int element, RJReal x, RJReal dt);
                //RJVector3 update_state_ddx(int id, int element, RJReal x, RJReal dt);
        }
        
        return true;
}