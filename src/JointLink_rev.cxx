/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#include "JointLink_rev.h"
using namespace std;
using namespace Eigen;
using namespace RobotJoint;


JointLink_rev::JointLink_rev()
/**
  @brief Default constructor.
*/
{
	T = RJMatrix4::Zero();
// 	joint_type=0;
	immobile=0;
	gear_rat=0;
        static_friction_param = RJVector4::Zero();
	q=0;
	inertia = RJMatrix3::Zero();
	com_pos = RJVector4::Zero();
}

JointLink_rev::JointLink_rev(RJReal &ai, RJReal &alphai, RJReal &di, RJReal &thetai, RJReal &rot_yi, RJReal &gear_rati, RJReal &static_frictioni, RJReal &viscosityi, RJVector4& com_posi, RJMatrix3& inertiai, bool immobilei)
/**
  @brief Constructor.
*/
{
	immobile=false;
	rot_y=rot_yi;
	q=0;
	a=ai;
	alpha=alphai;
	d=di;
	theta=thetai;
	gear_rat=gear_rati;
        static_friction_param(0)=static_frictioni;
        static_friction_param(1)=viscosityi;
        static_friction_param(2)=-static_frictioni;        
        static_friction_param(3)=-viscosityi;
	com_pos=com_posi;
	inertia=inertiai;
	immobile=immobilei;
        TopLimit = PI; // i.e no limits if not set
        BottomLimit = -PI;
}

JointLink_rev::JointLink_rev(RJReal &ai, RJReal &alphai, RJReal &di, RJReal &thetai, RJReal &rot_yi, RJReal &gear_rati, RJReal &static_frictioni, RJReal &viscosityi, RJVector4& com_posi, RJMatrix3& inertiai, bool immobilei, RJReal topLim, RJReal botLim)
/**
  @brief Constructor.
*/
{
        immobile=false;
        rot_y=rot_yi;
        q=0;
        a=ai;
        alpha=alphai;
        d=di;
        theta=thetai;
        gear_rat=gear_rati;
        static_friction_param(0)=static_frictioni;
        static_friction_param(1)=viscosityi;
        static_friction_param(2)=-static_frictioni;        
        static_friction_param(3)=-viscosityi;
        com_pos=com_posi;
        inertia=inertiai;
        immobile=immobilei;
        TopLimit = topLim;
        BottomLimit = botLim;
}

int JointLink_rev::transform(){
 return transform(q) ;
}

int JointLink_rev::transform(const RJReal &qi)
/** @brief Set the rotation matrix R and the vector p.
z is theta in dh
x is alpha in dh
dh rotation is: first theta then alpha
mydh is 1. theta rot 2. alpha rot 3. y rot
*/

{
	
	RJReal ct, st, ca, sa,cy,cx,sy,sx,cz,sz,thetar;
	//total angle will be the joints offset plus rotational angle
	thetar = qi + theta;
	//precalculating and storing these values saves the cpu from reculculating them
	ct = cos(thetar);
	st = sin(thetar);
	ca = cos(alpha);//ca = R(3,3);
	sa = sin(alpha);//sa = R(3,2);
	cy = cos(rot_y);
	sy = sin(rot_y);
	//### TODO can save a bit of memory by getting rid of this duplicatioon
	sx = sa;
	cx = ca;
	sz = st;
	cz = ct;
/*
         R(1,1) = ct;
         R(2,1) = st;
         R(1,2) = -ca*st;
         R(2,2) = ca*ct;
         R(1,3) = sa*st;
         R(2,3) = -sa*ct;
*/
	T(0,0) = cz*cy-sz*sx*sy;//ct;
	T(1,0) = sz*cy+cz*sx*sy;//st;
	T(2,0) = -cx*sy;//0.0;
	T(3,0) = 0.0;
	T(0,1) = -sz*cx;//-ca*st;
	T(1,1) = cz*cx;//ca*ct;
	T(2,1) = sx;//sa;
	T(3,1) = 0.0;
	T(0,2) = cz*sy+sz*sx*cy;//sa*st;
	T(1,2) = sz*sy-cz*sx*cy;//-sa*ct;
	T(2,2) = cx*cy;//ca;
	T(3,2) = 0.0;
	T(0,3) = a*ct;
	T(1,3) = a*st;
	T(2,3) = d;
	T(3,3) = 1.0;

	return 0;
}



bool JointLink_rev::get_immobile()
{
	return immobile; 
}

// RJReal JointLink_rev::get_m(){
//   return m;
// }

// int JointLink_rev::get_joint_type(){
//   return joint_type;
// }

RJVector4 JointLink_rev::get_dh_param()
{
	RJVector4 temp;
	temp<<a,alpha,d,theta,rot_y;
	return temp;
}