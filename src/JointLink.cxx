/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/


#include "JointLink.h"
using namespace std;
using namespace Eigen;
using namespace RobotJoint;

JointLink::JointLink()
{
        JointLink_id=0;
        T=RJMatrix4::Zero();
//         joint_type=0;
        immobile=false; 
        q=0;
        com_pos=RJVector4::Zero();
        inertia=RJMatrix3::Zero();
}

int JointLink::transform()
/*! @brief Set the rotation matrix R and the vector p.
z is theta in dh
x is alpha in dh
dh rotation is: first theta then alpha
mydh is 1. theta rot 2. alpha rot 3. y rot
*/
{
	T = RJMatrix4::Identity();
	return 0;
}

int JointLink::transform(const RJReal &qi)
/*! @brief Set the rotation matrix R and the vector p.
z is theta in dh
x is alpha in dh
dh rotation is: first theta then alpha
mydh is 1. theta rot 2. alpha rot 3. y rot
*/
{
	T = RJMatrix4::Identity();
	return 0;
}

int JointLink::update(RJReal &qi)
{
        this->transform(qi);
        q = qi;
}


RJMatrix4 JointLink::get_t(){
        this->transform();
        return T;
}

RJMatrix4 JointLink::get_t(RJReal &qi)
/** @brief returns the transformation matrix of this JointLink
*/
{
        // immobile JointLinks dont have a joint angle
        if(!get_immobile())     this->transform(qi);
        else
        {
                RJReal temp = 0;
                this->transform(temp);
        }
        return T;
}

bool JointLink::get_immobile()
{
        return immobile; 
}

int JointLink::get_id()
{
        return JointLink_id; 
}

int JointLink::set_id(int idi)
{
        JointLink_id=idi;
        return 0;
}

RJMatrix3 JointLink::getInertia()
{
        return inertia;
}

RJReal JointLink::getMass()
{
        return com_pos(3);
}

int JointLink::setMass(RJReal &mi)
{
        com_pos(3)=mi;
}

int JointLink::setInertia(const RJMatrix3 &i)
{
        inertia=i;
}

RJMatrix3 JointLink::get_r()
{
        RJReal temp = 0;
        return get_r(temp);
}

RJMatrix3 JointLink::get_r(const RJReal &qi)
/**
  @brief get the rotation matrix given some angle
*/
{
	transform(qi);	
	return T.block<3,3>(0,0);
}

RJVector3 JointLink::get_p()
{
        RJReal temp = 0;
        return get_p(temp);
}

RJVector3 JointLink::get_p(const RJReal &qi)
/**
  @brief get the position vector given some angle
*/
{
        transform(qi);
        return T.block<3,1>(0,3);
}

RJVector4 JointLink::get_pc()
{ //### compos bug, com is given wrt to JointLink frame, rotating by JointLink frame again gives wrong result
        return com_pos;
}

RJVector4 JointLink::get_pc(const RJReal &qi)
/**
@brief get the position vector of the COM
*/
{
	RJMatrix3 rot;
	RJVector4 ret;
	rot <<	cos(qi),-sin(qi),0
		,sin(qi),cos(qi),0
		,0      ,0      ,1.0;

	ret.head(3) = rot*(com_pos.head(3));
	ret(3) = com_pos(3);
	return ret;
}

RJReal JointLink::get_q()
/**
@brief Return joint position
*/
{
	return q;
}

int JointLink::set_q(const RJReal &qi)
/**
@brief set joint position 
*/
{ 
	q=qi;
	return 0;
}

RJReal JointLink::staticFriction(RJReal dx)
/**     @brief calculates friction force based on a static friction model
        @param[in] dx velocity of the joint
*/
{
// #define staticFrictionVelThreshold 0.0000005
#define staticFrictionVelThreshold 0.05
//#define staticFrictionVelThreshold 0.0
        if(dx > staticFrictionVelThreshold)
        {
                return static_friction_param(0) + static_friction_param(1)*dx;
        }
        else if(dx < -staticFrictionVelThreshold)
        {
                return static_friction_param(2) + static_friction_param(3)*dx;
        }
        else
	{
		double m=(static_friction_param(0)-static_friction_param(2))/(2.0*staticFrictionVelThreshold);
		return m*dx;
	}

return 0;
}