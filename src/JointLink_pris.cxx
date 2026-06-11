/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#include "JointLink_pris.h"
using namespace std;
using namespace Eigen;
using namespace RobotJoint;


JointLink_pris::JointLink_pris()
/**
  @brief Default constructor.
*/
{
        T = RJMatrix4::Zero();
//         joint_type=0;
        immobile=0;
        gear_rat=0;
        static_friction=0;
        viscosity=0;
        q=0;
        inertia = RJMatrix3::Zero();
        com_pos = RJVector4::Zero();
}

JointLink_pris::JointLink_pris(RJReal &ai, RJReal &alphai, RJReal &di, RJReal &thetai, RJReal &rot_yi, RJReal &gear_rati, RJReal &static_frictioni, RJReal &viscosityi, RJVector4& com_posi, RJMatrix3& inertiai, bool immobilei)
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
        static_friction=static_frictioni;
        viscosity=viscosityi;
        com_pos=com_posi;
        inertia=inertiai;
        immobile=immobilei;
}

int JointLink_pris::transform(){
 return transform(q) ;
}

int JointLink_pris::transform(const RJReal &qi)
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

        T(0,0) = 1.0;
        T(1,0) = 0.0;
        T(2,0) = 0.0;
        T(3,0) = 0.0;
        T(0,1) = 0.0;
        T(1,1) = 1.0;
        T(2,1) = 0.0;
        T(3,1) = 0.0;
        T(0,2) = 0.0;
        T(1,2) = 0.0;
        T(2,2) = 1.0;
        T(3,2) = 0.0;
        T(0,3) = a + qi;
        T(1,3) = 0;
        T(2,3) = d;
        T(3,3) = 1.0;

        return 0;
}



bool JointLink_pris::get_immobile()
{
        return immobile; 
}

// RJReal JointLink_pris::get_m(){
//   return m;
// }

// int JointLink_pris::get_joint_type(){
//   return joint_type;
// }

RJVector4 JointLink_pris::get_dh_param()
{
        RJVector4 temp;
        temp<<a,alpha,d,theta,rot_y;
        return temp;
}