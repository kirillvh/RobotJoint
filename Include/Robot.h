/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#ifndef RoboJointRobot_H
#define RoboJointRobot_H
#include <string>
#include <assert.h>
#define EPS 0.0000000000000001;
#define PI 3.14159265358979
// #include <vector>
#include "Eigen/StdVector"
#include <map>
#include <iostream>
#include <fstream>

#include "RJointTypes.h"
#include "RobotManipulator.h"

#include "RobotAlgo.h"
#include "Controller.h"

#include <iostream>
#include "Eigen/Dense"

/**
*       @page tutorial Tutorial Page 1.
*       @section initilizing_manipulators Initilizing manipulators
*       @code
*       RobotJoint::JointLink_rev joint();
*       RobotJoint::RobotManipulator manip(sampling_time);
*       manip.addJoint(joint);
*       @endcode
*/

// using Eigen::RJMatrixX;
// using Eigen::RJVectorX;
using Eigen::MatrixXcd;
using Eigen::VectorXcd;

namespace RobotJoint
{
        /**
        *       @class Robot Robot.h "Robot.h"
        *       @brief A robot is a collection of manipulators with trajectory generation functions
        */
        class Robot : public RobotAlgo {
                public:
                        Robot();
                        ~Robot();
                        int update(const RJVectorXMap &Angles);
//                         RJVector4 getCOM(RobotAlgo R);//wrt to base TODO ... deprecated ???
                        RJVector4 getCOM(const RJVectorXMap &Angles);//wrt to base
                        RJVector4 getCOM(bool calculate=false);//wrt to base
			RJVector3 getCOMvel();
			RJVector3 getCOMvelWRTFootFrame();
			RJVector3 getCOMvelWRTFootFrame2();
			RJVector3 getCOMvel(RJVectorXMap q, RJVectorXMap dq);
                        RJMatrixX getJacobian(const RoboJointVars &var);
                        RJMatrixX getCOMJacobian(const RoboJointVars &var, const RJVectorX &q);
                        RJMatrixX getCOMJacobian(const RoboJointVars &var);
                        RJMatrixX getCOMJacobianWRTFootFrame(const RoboJointVars &var);
                        RJMatrixX getCOMJacobianWRTFootFrame(const RoboJointVars &var, const RJVectorX &q);

                        RJMatrixX getInvJacobian(const RoboJointVars &var); 
                        RJMatrixX getInvCOMJacobian(const RoboJointVars &var);
                        RJMatrixX getInvCOMJacobianWRTFootFrame(const RoboJointVars &var);

                        RJMatrixX _ScaleCOMJacobian(RJMatrixX &jac, const RoboJointVars &manip_name);
                        RJVector3 BaseVector2ToolFrame(const RJVector3 &v, const RJVectorX &angles, const RoboJointVars &manip_name);
                        RJVector3 BaseVector2ToolFrame(const RJVector3 &v, const RoboJointVars &manip_name);
                        
                        RJMatrixX MakeZMPTrajectory(int from = -1, int to = -1);
                        RJMatrixX MakeCOMTrajectory(const RJMatrixX &ZMPTrj);
                        std::vector<RJVectorXMap > MakeJointTrajectory(const RJMatrixX &COMTrj);
                        
                        int addMovementPhase(LegPhase phase,RJReal start, RJReal end, RJReal footAposX, RJReal footAposY, RJReal footBposX, RJReal footBposY);
                        int addMovementPhase(LegPhase phase,RJReal dt, RJReal footAposX, RJReal footAposY, RJReal footBposX, RJReal footBposY);

                        RJVectorXMap CL_IK(const RJVector3Map &XRef, const RJQuaternionMap &OriRef,  const std::map<RoboJointVars, bool> &use_com);
                        RJVectorXMap CL_IK(const RJVector3 &LeftLeg2COM, const RJQuaternion &LeftLeg2COMOrientation, const RJVector3 &LeftLeg2RightLeg, const RJQuaternion &LeftLeg2RightLegOrientation, int maxIter=1000, double epsError=1E-6);
                        
                        bool isSingleSupport(SupportPhase &x)
                        {
                                if((x.Phase==LeftSup) || (x.Phase==RightSup)) return true;
                                else return false;
                        }
                        bool isDoubleSupport(SupportPhase &x)
                        {
                                if(x.Phase==DoubleSup) return true;
                                else return false;
                        }
                        bool isLeftSupport(SupportPhase &x)
                        {
                                if(x.Phase==LeftSup) return true;
                                else return false;
                        }
                        bool isRightSupport(SupportPhase &x)
                        {
                                if(x.Phase==RightSup) return true;
                                else return false;
                        }

// 			//Hardware Abstraction functions - Should be implemented in child classes
// 			virtual RJVectorX HAL_ReadJointAngles(){ };
// 			virtual RJVectorX HAL_WriteJointTorques(){ };
                        
                        //Robots deriving from this class should implement servo's with appropriate paramaters
                        virtual RJVectorX positionControl(RJVectorX posRef){}; 
                        
                        int getDOF();
                        RJVectorX getAngles(const RoboJointVars &var){ return _Angles[var];}
                        RJVectorXMap getAngles(){ return _Angles;}
                        RJReal getMass(bool calculate);
                        
                        //accessors
                        RobotManipulator* getManipulator(RoboJointVars var);
                        int addManipulator(RoboJointVars name, RobotManipulator* manip);
                        //This redefines the new operator for SSE support(allows for 16-bit boundry alignment)
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                        protected:
                        std::map<RoboJointVars,RobotManipulator*> Manipulators; //Pointers deleted in destructor, TODO change container of pointers to smart pointers once project is moved to C++11
                        std::map<RoboJointVars,RJMatrixX> COMJac;
                        std::map<RoboJointVars,RJMatrixX> iCOMJac;
                        std::map<RoboJointVars,RJMatrixX> COMJacWRT2Tool;
                        std::map<RoboJointVars,RJMatrixX> iCOMJacWRT2Tool;
                        RJVectorX COM;
                        std::vector<SupportPhase> SupportPhases;
                        Controller COMPreviewController;
                        RJReal mass;
                        RJReal preffered_com_height;
                        RJVectorXMap _Angles;
                };
                typedef std::map<RoboJointVars,RobotManipulator*> ManipMapType;
}

#endif // SAMPLEPD_H
