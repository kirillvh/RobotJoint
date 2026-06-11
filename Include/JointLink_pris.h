/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#ifndef JointLinkpris_H
#define JointLinkpris_H
#include <string>
#include "JointLink.h"
#define EPS 0.0000000000000001;
#define PI 3.14159265358979
#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
// using Eigen::RJMatrix4;
// using Eigen::RJMatrix3;
using Eigen::VectorXd;
// using Eigen::RJVector4;
// using Eigen::RJVector3;

namespace RobotJoint
{
        /**
        *       @class JointLink_rev JointLink_rev.h "JointLink_rev.h"
        *       @brief Revolute joint class
        */
        class JointLink_pris : public JointLink
        {
                public:
                        JointLink_pris();
                        JointLink_pris(RJReal &ai, RJReal &alphai, RJReal &di, RJReal &thetai, RJReal &rot_yi, 
                        RJReal &gear_rati, RJReal &static_frictioni, RJReal &viscosityi, RJVector4 &com_posi, 
                        RJMatrix3 &inertiai, bool immobilei);
                        virtual int transform(const RJReal &qi); //define new transforms
                        virtual int transform();
                        JointType get_joint_type(){return Prismatic;};
                        RJReal get_m();
                        bool get_immobile();
                        RJVector4 get_dh_param();
                        //This redefines the new operator for SSE support(allows for 16-bit boundry alignment)
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
                private:
                        RJReal a,alpha,d,theta,rot_y; 
                        RJReal gear_rat,static_friction,viscosity;
        };
}
#endif
