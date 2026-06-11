/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#ifndef TESTROBOTALGO_H
#define TESTROBOTALGO_H
#include "RobotAlgo.h"
#include "RJointTest.h"
#include "Eigen/Dense"
#include "Eigen/StdVector"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXcd;
using Eigen::VectorXcd;

namespace RobotJoint
{
        class TestRobotAlgo : public RobotAlgo , public RJointTest
        {
                public:
                        TestRobotAlgo();
                        TestRobotAlgo(int);
                        int Reset();                
                        int setNominalValues();
                        bool TestAll(bool verbose);
                        bool test_update_state_x();
                        bool test_update_state_x_poly();
                        bool test_update_state_dx();
                        bool test_update_state_ddx();
                
                        int state_id;
                        RJReal t_sampling;
                        RJVector4 SolveQpnom1;
        };
}

#endif 
