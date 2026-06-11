/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#ifndef TESTROBOT_H
#define TESTROBOT_H
#include <string>
#include <assert.h>
#define EPS 0.0000000000000001;
#define PI 3.14159265358979
#include "Robot.h"
#include "RJointTest.h"
#include "RJointTypes.h"
#include "JointLink_rev.h"
#include <iostream>
#include "Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXcd;
using Eigen::VectorXcd;

namespace RobotJoint
{
        class TestRobot : public Robot , public RJointTest
        {
                public:
                        TestRobot();
                        TestRobot(int);
                        bool TestAll(bool verbose);
                        bool test_COM(bool verbose);
                        //This redefines the new operator for SSE support(allows for 16-bit boundry alignment)
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
}

#endif 
