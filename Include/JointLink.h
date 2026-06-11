/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#ifndef JointLink_H
#define JointLink_H
#include <string>

#define EPS 0.0000000000000001;
#define PI 3.14159265358979
// #include <vector>
#include "Eigen/StdVector"

#include <iostream>
#include "Eigen/Dense"
#include "RJointTypes.h"

using Eigen::MatrixXd;
// using Eigen::RJMatrix4;
// using Eigen::RJMatrix3;
using Eigen::VectorXd;
// using Eigen::RJVector4;
// using Eigen::RJVector3;

enum JointType
{
        Revolute        =       0,
        Prismatic       =       1,
};

namespace RobotJoint
{
        /**
        *       @class JointLink JointLink.h "JointLink.h"
        *       @brief Basic virtual class for joints.
        */
        class JointLink
        {
                public:
                        JointLink();
                        virtual int transform();
                        virtual int transform(const RJReal &param);
                        int update(RJReal &param);
                        int get_id();
                        int set_id(int idi);
                        RJReal getTopLimit(){return TopLimit;}
                        RJReal getBottomLimit(){return BottomLimit;}
                        bool setTopLimit(RJReal x){TopLimit = x; return true;}
                        bool setBottomLimit(RJReal x){BottomLimit = x; return true;}

                        std::string get_description();
                        virtual JointType get_joint_type() = 0;
                        bool get_immobile();
                        RJReal get_inertia_rotaxis();
                        RJReal getMass();
                        int setMass(RJReal &mi);
                        int setInertia(const RJMatrix3 &i);
                        RJMatrix3 getInertia();
                        RJReal get_value();
                        int set_q(const RJReal &q);
                        RJReal get_q();

                        RJMatrix4 get_t();
                        RJMatrix4 get_t(RJReal &qi);
                        RJMatrix3 get_r();
                        RJMatrix3 get_r(const RJReal &qi);
                        RJVector3 get_p();
                        RJVector3 get_p(const RJReal &qi);
                        RJVector4 get_pc();
                        RJVector4 get_pc(const RJReal &qi);
                        
                        void setStaticFrictionParam(const RJVector4 &x){static_friction_param=x;}
                        RJVector4 getStaticFrictionParam(){return static_friction_param;}
                        RJReal staticFriction(RJReal dx);
                        
                        
                        RJMatrix4 T;
                        //This redefines the new operator for SSE support(allows for 16-bit boundry alignment)
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                protected:
                        RJVector4 com_pos;
                        RJReal q;
                        RJReal TopLimit, BottomLimit;
                        RJMatrix3 inertia;
                        bool immobile;
                        int JointLink_id; // TODO deprecated?
                        RJReal gear_rat;
                        RJVector4 static_friction_param;
			static int _time_index; // since RobotJoint classes inherit from this base class, this variable becomes global by making it static. This way all RobotJoint classes have access to the same time variable
                        
        };
}
#endif
