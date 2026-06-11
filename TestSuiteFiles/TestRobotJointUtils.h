/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#ifndef TESTROBOTJOINTUTILS_H
#define TESTROBOTJOINTUTILS_H
#include <string>
#include <assert.h>
#define EPS 0.0000000000000001;
#define PI 3.14159265358979
#include "RJointTest.h"
#include "RJointTypes.h"
#include "RobotJointUtils.h"
#include <iostream>
#include "Eigen/Dense"
#include <exception>


// using Eigen::RJMatrixX;
// using Eigen::RJVectorX;
// using Eigen::MatrixXcd;
// using Eigen::VectorXcd;

namespace RobotJoint
{
        class TestRobotJointUtils : public RJointTest, public RobotAlgo
        {
                public:
                        TestRobotJointUtils();            
                        bool TestAll(bool verbose);
                        int setNominalValues();
                
                        RJVectorX SolveQpnom1;
                        RJMatrix4 Tnom;
                        RJVector4 Vnom;
                        RJMatrixX Jacnom, COMJacnom, COMJacwrtToolnom;
                        RJMatrix4 Pnom;
                        RJVector4 Knom;
                        int DOFnom;
                        //This redefines the new operator for SSE support(allows for 16-bit boundry alignment)
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
        };
}


#endif 
