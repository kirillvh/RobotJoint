#include "Robot.h"
#include "JointLink_rev.h"
#include "RJointTypes.h"

using namespace std;
using namespace RobotJoint;
using namespace Eigen;

int main()
{
        //First declare the data types
        Robot TestRobot;
        Matrix3d inertia;
        Vector4d compos;
        VectorXd angles(2);
        double m;
        double a;
        double alpha;
        double d;
        double theta;
        double roty;
        double gear_rat=1.0;
        double static_friction=0.0; 
        double viscosity=0.0;
        
        //add one manipulator to the robot
        
        TestRobot.addManipulator(RJ_LeftLeg ,new RobotManipulator(RJ_LeftLeg ) );
        
        /* Next declare a 2 DOF manipulator with the following 
        Link Paramaters (a, alpha, d, theta), m = mass,
        com = center of mass, inertia = moment of inertia
        Link | a | alpha | d | theta | m |   com   | inertia
        0    | 5 | 0     | 0 |   0   | 2 |   [ 1 ] | [5 0 0 ]  
             |   |       |   |       |   |   [ 0 ] | [0 1 0 ]  
             |   |       |   |       |   |   [ 0 ] | [0 0 1 ]  
        1    | 3 | 0     | 0 |   0   | 11|   [ 0 ] | [1 0 0 ]  
             |   |       |   |       |   |   [ 1 ] | [0 7 0 ]  
             |   |       |   |       |   |   [ 0 ] | [0 0 1 ]  
        */

        //0-1
        inertia<<5,0,0,0,1,0,0,0,1;
        m=2; 
        alpha=0;
        a=5;
        d=0;
        theta=0;
        roty=0;
        compos<<1,0,0,m;

        //add link 1
        TestRobot.getManipulator(RJ_LeftLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,false));

        //0-1
        inertia<<0,0,0,0,7,0,0,0,1;
        m=11;
        alpha=0;
        a=3;
        d=0;
        theta=0;
        roty=0;
        compos<<0,1,0,m;
        
        //add link 2
        TestRobot.getManipulator(RJ_LeftLeg)->addJoint(new JointLink_rev(a, alpha, d, theta, roty, gear_rat, static_friction, viscosity, compos, inertia,false) );

        //define the joint angles
        angles<< 0, 0;
        
        //update the robot
        TestRobot.getManipulator(RJ_LeftLeg)->update(angles);
        
        //now quantaties like the COM, Jacobian, etc can be accessed
        cout<<" Center of mass is:" <<endl
        <<TestRobot.getManipulator(RJ_LeftLeg)->getCOM()<<endl;
        
        cout<<" Jacobian is:" <<endl
        <<TestRobot.getManipulator(RJ_LeftLeg)->getJacobian()<<endl;
        
        //TODO demonstrate multiple manipulators, preview control based COM trajectory generation, etc.
        
        return 0;
}