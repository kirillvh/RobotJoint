/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#ifndef TESTROBOTMANIPULATOR_H
#define TESTROBOTMANIPULATOR_H
#include <string>
#include <assert.h>
#define EPS 0.0000000000000001;
#define PI 3.14159265358979
#include "RobotManipulator.h"
#include "RJointTest.h"
#include "RJointTypes.h"
#include "JointLink_rev.h"
#include <iostream>
#include "Eigen/Dense"


// using Eigen::RJMatrixX;
using Eigen::VectorXd;
using Eigen::MatrixXcd;
using Eigen::VectorXcd;

namespace RobotJoint
{

        class TestRobotManipulator : public RobotManipulator , public RJointTest
        {
                public:
                        TestRobotManipulator();
                        TestRobotManipulator(int TestManipulatorIdc);                
                        bool TestAll(bool verbose);
                        bool test_kinematic_jacobian(bool verbose);
			bool test_kinematic_jacobianWRTFootFrame(bool verbose);
                        bool test_COM_jacobian(bool verbose);
                        bool test_COM_jacobian_wrt2tool(bool verbose);
                        int setNominalValues();
                        int TestManipulatorId; // 0=2-link, default=6DOF
                        
                        RJMatrix4 Tnom;
                        RJVector4 Vnom;
                        RJMatrixX Jacnom, COMJacnom, COMJacwrtToolnom;
                        int DOFnom;
                        //This redefines the new operator for SSE support(allows for 16-bit boundry alignment)
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

}
#endif 
