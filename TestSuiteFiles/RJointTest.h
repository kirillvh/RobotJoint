/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#ifndef RJOINTTEST_H
#define RJOINTTEST_H

#include <string>
#include <assert.h>
#define EPS 0.0000000000000001;
#define PI 3.14159265358979

#include <iostream>
#include "RobotAlgo.h"
#include "Eigen/Dense"
#include <Eigen/Core>

// using Eigen::RJMatrixX;
// using Eigen::RJMatrix4;
using Eigen::VectorXd;
// using Eigen::RJVector2;
// using Eigen::RJVector3;
// using Eigen::RJVector4;
using Eigen::MatrixXcd;
using Eigen::VectorXcd;

namespace RobotJoint
{
        class RJointTest
        {
                public:
                        RJointTest() {};
                        bool matCompare4(RJMatrix4 a, RJMatrix4 b, double thresh = 0.001);
                        bool matCompare3(RJMatrix3 a, RJMatrix3 b, double thresh = 0.001);
                        bool matCompare (RJMatrixX a, RJMatrixX b, double thresh = 0.001);
                        bool vecCompare2(RJVector2 a, RJVector2 b, double thresh = 0.001);
                        bool vecCompare3(RJVector3 a, RJVector3 b, double thresh = 0.001);
                        bool vecCompare4(RJVector4 a, RJVector4 b, double thresh = 0.001);
                        virtual bool TestAll(bool verbosei) = 0;
                        
                        
                        template <typename DerivedA,typename DerivedB>
                        bool Compare(const Eigen::MatrixBase<DerivedA>& p1,const Eigen::MatrixBase<DerivedB>& p2, double thresh=0.001)
                        {
                                if( (p1-p2).squaredNorm() >thresh) return false;
                                else return true;
                        }
                        //This redefines the new operator for SSE support(allows for 16-bit boundry alignment)
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                        
        };
}

#endif 
