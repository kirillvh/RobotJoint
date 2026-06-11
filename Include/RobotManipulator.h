/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#ifndef ROBOTMANIPULATOR_H
#define ROBOTMANIPULATOR_H



#include <string>
#include <assert.h>
#define EPS 0.0000000000000001;
#define PI 3.14159265358979
#include "JointLink.h"
//#include <vector>
//#include <map>
#include <fstream>
#include "RobotAlgo.h"
#include "RJointTypes.h"
#include "Controller.h"

#include <iostream>
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "Eigen/StdVector"

#ifdef RJ_THREADED   
        #include <thread>
#endif
// #include <pthread.h>
// using Eigen::RJMatrixX;
// using Eigen::RJMatrix4;
// using Eigen::RJVectorX;
// using Eigen::RJVector4;
using Eigen::MatrixXcd;
using Eigen::VectorXcd;

/*
Some considerations on caching results for faster computation time.

1. Some functions like getJacobian use other functions [FK()] so if FK is already computed for
a set of joint angles. Then getJacobian should use the cached FK() results
2. Some functions (like some inverse kine. functions) may want to try out Jacobians with
many different angle values. In this case getJacobian shouldnt use cached FK() results.
3. Higher up in the library, getJacobian might get called many times. In this case the
results of getJacobian should also be cached(not just caching the stuff used to calculate the Jacobian)

In all cases there should be a way to differentiate between cached code paths and non-cached ones.
*/
typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;
typedef Eigen::Matrix<bool, 6, 1>              Vector6b;

namespace RobotJoint
{
        enum IKMode
        {
                IK_Kinematic       =       0,
                IK_COM             =       1,
                IK_COMwrtFoot      =       2,
//                 IK_COMwrtFootExt   =       3,
        };
        
//         struct update_thread_data
//         {
//                 int thread_id;
//                 RJVectorX angles;
//         };

        /** 
        *       @class RobotManipulator RobotManipulator.h "RobotManipulator.h"
        *       @brief Robot's algorithim class
        *       Simple serial manipulator class that provides forward kinematics, inverse kinematics, Jacobian calculation(both Center 
        *       of Mass and Kinematic Jacobians) and COM calculation.
        */
        class RobotManipulator : public RobotAlgo
        {
                public:
                RobotManipulator();
                ~RobotManipulator();
                RobotManipulator(RoboJointVars name);
                RobotManipulator(RoboJointVars name, RJVectorX angles);
                //### TODO Handle jacobian updates for manipulators with less than 6 joints
                int update(const RJVectorX &q);
                void update2(const RJVectorX &q);
//                 void *updateThreaded(void *threadarg);
                //virtual int update(RobotAlgo &R);
                //forward kine
                RJVector4 getCOM(const RJVectorX &angles, int k=0,bool use_cached=false, 
                bool memorize=false);
                RJVector4 getCOM(int k=0);  
                // returns transform matrix
                RJMatrix4 fk(const RJVectorX &angles ,int from , int to , bool use_com=false, 
                bool memorize=Dont_Remember ); 
                RJMatrix4 fk(const RJVectorX &angles, bool memorize=Dont_Remember);
                RJMatrix4 fk(int from, int to); 
                RJMatrix4 fk(); 

                // Jacobians
                RJMatrixX getJacobian(const RJVectorX &angles, bool use_cached=Dont_Remember);
                RJMatrixX getJacobian();
                RJMatrixX getCOMJacobian(const RJVectorX &angles, bool use_cached = false);
                RJMatrixX getCOMJacobian();
                RJMatrixX getCOMJacobianWRTFootFrame(const RJVectorX &angles, bool use_cached = false);
                RJMatrixX getCOMJacobianWRTFootFrame();
                RJMatrixX getCOMJacobianWRTFootFrame(const RJVector4 &COM, const RJVectorX &angles);
                RJMatrixX getCOMJacobianWRTFootFrame(const RJVector4 &COM);
                
                RJMatrixX getCOMJacobian(const RJVector4 &COM, const RJVectorX &angles, bool use_cached = false);
                
                RJMatrixX getJacobianWRTFootFrame();
		RJMatrixX getJacobianWRTFootFrame(RJVectorX q, RJVector3 offest = RJVector3::Zero());

		RJMatrixX getInvJacobianWRTFootFrame();
                RJMatrixX getInvJacobian();
                RJMatrixX getInvJacobian(const RJVectorX &qi);
                RJMatrixX getInvCOMJacobian();
                RJMatrixX getInvCOMJacobian(const RJVectorX &qi);
                RJMatrixX getInvCOMJacobianWRTFootFrame();
                RJMatrixX getInvCOMJacobianWRTFootFrame(const RJVectorX &qi);

                //Experimental functions
                RJVectorX RNE(const RJVectorX &q, const RJVectorX &dq, const RJVectorX & ddq, const RJVector6 &Fextern = RJVector6::Zero() );
                RJVectorX GravityTorque(const RJVectorX &q);
		RJMatrixX InertiaMatrix(const RJVectorX &q);
//                 RJVectorX CLIK_IK(const RJVector3 &XRef, const RJQuaternion &OrientationRef,bool comIK=false, int max_runs = 50, RJReal error_tol = 0.001);
                RJVectorX CLIK_IK(const RJVector3 &XRef, const RJQuaternion &OrientationRef, IKMode mode, int max_runs, RJReal error_eps);
                
                RJVectorX GRFC_TF(const RJVectorX &Torques, const RJVectorX &TipForceSens, const RJMatrixX &MaxWrench);
                RJVectorX GRFC_TF(const RJVectorX &Torques, const RJVectorX &TipForceSens, const RJMatrixX &MaxWrench, const Vector6b &axes_to_use,  const VectorXb &joints_to_use, const RJReal &fil_freq);
		RJVectorX GRFC_TF_ankle(const RJVectorX &Torques, const RJVectorX &TipForceSens, const RJMatrixX &MaxWrench, const Vector6b &axes_to_use,  const VectorXb &joints_to_use, const RJReal &fil_freq);
                RJVectorX frictionIdentification(int jointIndex, RJVector2 &velVsfrictionTau, bool reset = false);
                void frictionIdentification_holdData(const RJVector2 velVsfrictionTau , RJReal reject_limit = 0.05);
                RJVector4 frictionIdentification_calculate();
                RJVectorX getFrictionTorque();
                RJVectorX getFrictionTorque(const RJVectorX &dq);
        
                //accessors/mutators
                int setAngles(const RJVectorX &qi);
                RJVectorX getAngles(){return _angles;}
                RJVectorX getq(){return qi;} //TODO fix this redundancy
                RJVectorX getdq(){return dqi;}
                RJVectorX getddq(){return ddqi;}
                int addJoint(JointLink* newlink);
                JointLink* getJoint(int i){ return JointLinks.at(i);}
                int getDOF();
                const int getNumberOfLinks(){return JointLinks.size();}
                //utils
                RJReal get_total_mass(int k=0);
                RoboJointVars getName();
                RJVector3 BaseVector2ToolFrame(const RJVector3 &v);
                RJVector3 BaseVector2ToolFrame(const RJVector3 &v, const RJVectorX &angles);
        
                //This redefines the new operator for SSE support(allows for 16-bit boundry alignment)
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
                protected:  
                std::vector<JointLink*>			                        JointLinks; //TODO support smart pointers when moving to c++11
                RJMatrix4				                        FK;
                std::vector<RJMatrix4, Eigen::aligned_allocator<RJMatrix4> >	FK_accumulated;
                RJVector4			                        	COM;
                std::vector<RJVector4, Eigen::aligned_allocator<RJVector4> >	COM_accumulated;
                RJMatrixX 		                        		Jac;
                RJMatrixX 			                        	iJac; 
                RJMatrixX 			                        	COMJac;
                RJMatrixX 		                        		iCOMJac; 
                RJMatrixX 			                        	COMJac_wrt2foot;
                RJMatrixX 		                        		iCOMJac_wrt2foot; 
                RJVectorX LimitedMotorForce;
                std::vector<Controller, Eigen::aligned_allocator<Controller> >  JointControllers;
                
                //internal joint states
                RJVectorX                                                       qi;
                RJVectorX                                                       dqi;
                RJVectorX                                                       ddqi;
                
                int                                                             frictionIDTime;
                int                                                             currFrictionIdJoint;
                RJVectorX                                                       prevFrictionIDTorque;
                
                std::vector<RJVector2, Eigen::aligned_allocator<RJVector2> >    positive_friction_data;
                std::vector<RJVector2, Eigen::aligned_allocator<RJVector2> >    negative_friction_data;
                
                
                //### todo, cache the mass and partial mass for COM and COMJacobian calculation
                /** @brief mass of robot in Kg*/
                RJReal	totalmass;
		RJReal _gravity;

                /** @brief joint angles */
                RJVectorX _angles; //TODO deprecate and replace with qi
                /** @brief name tag of this manipulator*/
                RoboJointVars _ManipName;
                /** @brief internal count of the degrees of freedom */
                int _DOF;
                bool RobotManipulator_start;
        };
        typedef std::vector<JointLink*> JointLinksType;
}

#endif
