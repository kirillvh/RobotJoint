/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#ifndef Controller_H
#define Controller_H



#include <string>
#include <assert.h>
#define EPS 0.0000000000000001;
#define PI 3.14159265358979

//#include <vector>
//#include <map>
#include <fstream>
#include "RobotAlgo.h"
#include "RJointTypes.h"

#include <iostream>
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "Eigen/StdVector"



// using Eigen::RJMatrixX;
// using Eigen::RJVectorX;
// using Eigen::MatrixXcd;
// using Eigen::VectorXcd;

/*
Design considerations:
1. Need to study control theory some more to see if its possible to provide PID(set point complicates things otherwise manually setting gains would work) & LQR with disturbance observer(something like "Robust Control Using A State Space Disturbance Observer" Seung-Hi Lee and Chung Choo Chung) via a single framework that enable both SISO and MIMO
2. Need to implement some simple SISO PID and disturbance observer stuff as a fall back to the above point

should consider that LQR is a sort of MIMO where the state variables are intertwined where as multiple SISO PID's are 
independent of each other. So its more atomic to consider MIMO LQR and SISO PID
*/

namespace RobotJoint
{
        enum DisturbanceObsState{
                DO_Sensor_Mode  = 0, 
                DO_Model_Mode   = 1
        };
        enum {
                proportional = 0,
                derivative   = 1,
                integral     = 2
        };
        enum {
                position         = 0,
                velocity         = 1,
                acceleration     = 2
        };

        /** 
        @class Controller Controller.h "Controller.h"
        @brief Controller class
        This class stores the actuator states in a matrix "X" which also represents the relationship between the states via its structure.
        The rows of "X" represent its variables while its columns represent the derivatives of these variables(1st column is 0th derivative ,
        2nd column represnts the 1st derivative, etc)
        This class calculates a "Gains" matrix which is then used for feedback. This is done via methods such as LQR or PolePlacement.
        This class updates the state by taking in a matrix representing the measured states. Then by also taking in a matrix representing the 
        measureable states it determines which states need to be estimated.
        */
        class Controller : public RobotAlgo
        {
                public:
                Controller();
		void setupMemberData();
                Controller(const double &sampling_period);
                Controller(const RJReal &Jn, const double &sampling_period);
                Controller(const RJMatrixX &Ai, const RJMatrixX &Bi, const RJMatrixX &Ci, const RJMatrixX &Di, const RJMatrixX &Qi, const RJMatrixX &Ri);
                int update(RJMatrixX measured, RJMatrixX measurable); //updates the measurable states and estimates the non-measureable ones
                //update(measured_x_states, measured_dx_states, measured_ddx_states);
                bool SolveRicattiBrute(RJMatrixX &A, RJMatrixX &B, RJMatrixX &Q, RJMatrixX &R, RJMatrixX &P,int itterations);
        
                RJVectorX Observe(RJMatrixX x); //Estimates the full state from sensor data
                RJVectorX Feedback(RJVectorX x_ref, RJVectorX x); //calculates the actuator ref. by multiplying the state error by the gain
                RJVectorX Predict(RJVectorX x, RJVectorX u, double dt); //predicts the future state based on current state, actuator command and elapsed time
                int CalcPreviewGain(const  int &preview_window_size);
                RJMatrixX PreviewControl(const RJMatrixX &ZMPRef);
                RJVectorX DisturbanceObserver(); //### STUB 
                RJVectorX LQR(); //### STUB 
                
                //simple classical control tools
                RJReal P_feedback(RJReal gain, RJReal x, RJReal xref=0);
		RJReal P_feedback_vel(RJReal gain, RJReal dx, RJReal dxref=0);
		RJReal PI_feedback(RJReal gain, RJReal integ_gain, RJReal x, RJReal xref=0);
                RJReal PD_feedback(RJReal gain_p, RJReal gain_d, RJReal x, RJReal xref=0, RJReal dxref=0);
		RJReal PID_feedback(RJReal gain_p, RJReal gain_d, RJReal gain_i, RJReal x, RJReal xref=0, RJReal dxref=0);
                RJReal PD_feedback2(RJReal gain_p, RJReal gain_d, RJReal x, RJReal xref, RJReal dx, RJReal dxref=0);
                RJReal Disturbance_Observer(RJReal tau_ref, RJReal Jn, RJReal LPF_cutoff,
                        RJReal tau_lim_pos = std::numeric_limits<RJReal>::max(),
                        RJReal tau_lim_neg = -std::numeric_limits<RJReal>::max()
                        );
                RJReal Disturbance_Observer2(RJReal tau_ref, RJReal ddx, RJReal Jn, RJReal LPF_cutoff, std::string name = "std",
                        RJReal tau_lim_pos = std::numeric_limits<RJReal>::max(),
                        RJReal tau_lim_neg = -std::numeric_limits<RJReal>::max()
                        );
                RJReal Disturbance_Observer3(RJReal tau_ref, RJReal ddx, RJReal LPF_cutoff,
                        RJReal tau_lim_pos = std::numeric_limits<RJReal>::max(),
                        RJReal tau_lim_neg = -std::numeric_limits<RJReal>::max()
                        );
                RJReal PD_P_Obs_Controller(RJReal gain_x, RJReal gain_dx, RJReal gain_dx2, RJReal x, RJReal x_ref,
                        RJReal Jn, RJReal LPF_cutoff, 
                        RJReal tau_lim_pos = std::numeric_limits<RJReal>::max(),
                        RJReal tau_lim_neg = -std::numeric_limits<RJReal>::max(), 
                        RJReal vel_lim_pos = std::numeric_limits<RJReal>::max(), 
                        RJReal vel_lim_neg = -std::numeric_limits<RJReal>::max(), RJReal forward_compensation=0
                        );
                RJReal PD_P_Obs_Controller2(RJReal gain_x, RJReal gain_dx, RJReal gain_dx2, RJReal x, RJReal x_ref, RJReal dx, RJReal ddx,
                        RJReal Jn, RJReal LPF_cutoff, 
                        RJReal tau_lim_pos = std::numeric_limits<RJReal>::max(),
                        RJReal tau_lim_neg = -std::numeric_limits<RJReal>::max(), 
                        RJReal vel_lim_pos = std::numeric_limits<RJReal>::max(), 
                        RJReal vel_lim_neg = -std::numeric_limits<RJReal>::max()
                        );                        
                RJReal PD_P_Obs_Controller3(RJReal gain_x, RJReal gain_dx, RJReal gain_dx2, RJReal x, RJReal x_ref, RJReal dx, RJReal ddx,
                        RJReal LPF_cutoff, 
                        RJReal tau_lim_pos = std::numeric_limits<RJReal>::max(),
                        RJReal tau_lim_neg = -std::numeric_limits<RJReal>::max(), 
                        RJReal vel_lim_pos = std::numeric_limits<RJReal>::max(), 
                        RJReal vel_lim_neg = -std::numeric_limits<RJReal>::max()
                        );    
                RJReal velocityController(RJReal x, RJReal dxref, RJReal gain_dx2, RJReal Jn, RJReal LPF_cutoff, RJReal tau_lim_pos, RJReal tau_lim_neg);
                RJReal slidingModeController(RJReal x, RJReal dx, RJReal alpha, RJReal beta, RJReal dither, RJReal c);
                
                
        
                //accessors
                int setGains(RJVectorX x);
                RJVectorX getGains();
                RJMatrixX getGI(){return GI;};
                RJMatrixX getGX(){return GX;};
                RJMatrixX getGD(int i){ return preview_gain.at(i);}
                RJMatrixX getA(){return A;};
                RJMatrixX setA(const RJMatrixX &x){A = x;};
                RJMatrixX getB(){return B;};
                RJMatrixX setB(const RJMatrixX &x){B = x;};
                RJMatrixX getC(){return C;};
                RJMatrixX setC(const RJMatrixX &x){C = x;};
                RJMatrixX getD(){return D;};
                RJMatrixX setD(const RJMatrixX &x){D = x;};
                RJMatrixX getQ(){return Q;};
                RJMatrixX setQ(const RJMatrixX &x){Q = x;};
                RJMatrixX getR(){return R;};
                RJMatrixX setR(const RJMatrixX &x){R = x;};
                
                RJReal getInertia(){return In;}
                RJReal getInertia_Master(){return In_master;}
                RJReal getInertia_Slave(){return In_slave;}
                RJReal setInertia(RJReal x){In = x;}
                RJReal setInertia_Master(RJReal x){In_master = x;}
                RJReal setInertia_Slave(RJReal x){In_slave = x;}
                
                void setTauRefPrev(RJReal x){tau_ref_prev = x;}
                void addFeedForward(RJReal x){tau_ref_prev += x;}
                
                RJVector2 bilateralController(RJReal kp, RJReal kd, RJReal kf, RJReal xm, RJReal xs);
                RJVector2 bilateralController(RJReal kp, RJReal kd, RJReal kf, RJReal xm, RJReal xs, RJReal fm, RJReal fs);                
                RJVector2 bilateralController(RJReal kp, RJReal kd, RJReal kf, const RJVector2 &x_acc, const RJVector2 &dx_acc, const RJVector2 &ddx_acc, const RJVector2 &f_acc);
		RJVector2 bilateralController2(RJReal xm, RJReal xs, RJReal kp, RJReal kf, RJReal kv, RJReal Ns, RJVector2 &VelLimit, const RJVector3 &DeadZone, RJVector2 &f_acc, RJVector2 &dx_acc, RJVector2 &dx_acc_ref);
		void setBilateralTauRefPrev(RJVector2 newtauref);
		RJVector2 bilateralControllerForceLimited_part1(RJReal xm, RJReal xs, RJReal kp, RJReal kf, RJReal kv, RJReal Ns, RJVector2 &VelLimit, const RJVector3 &DeadZone, RJVector2 &f_acc, RJVector2 &dx_acc, RJVector2 &dx_acc_ref, const RJVector2 &f_sens);
		RJVector2 bilateralControllerForceLimited_part2(RJVector2 &limited_modal_ref, RJReal damping_const, RJReal kv);
		RJVector2 bilateralControllerForceLimited_part3(RJVector2 &limited_modal_ref, RJReal kv, RJVector2 &tau_ref_comm, RJVector2 &tau_ref_diff, RJVector2 &dis_hat, RJVector2 tau_prev);

		RJVector2 bilateralControllerForceVelLimited(const RJVector2 &x_acc, const RJVector2 &dx_acc, const RJVector2 &ddx_acc, RJVector2 &f_acc, RJReal kp, RJReal kf, RJReal kv,const RJVector2 &VelLimit, const RJVector3 &DeadZone, RJReal force_limit);

        
                //This redefines the new operator for SSE support(allows for 16-bit boundry alignment)
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
                protected:  
                RJMatrixX A,B,C,D,Q,R; //Matrices that describe the state space
                RJMatrixX GI, GX;        //Preview control's gains
                std::vector<RJMatrixX> preview_gain;
                RJMatrixX X; // The state and its derivatives
                RJMatrixX Gains; // Encodes Kp, Ki, Kd internally
                RJReal In; //nominal inertia
                RJReal In_master; //nominal inertia of master in bilateral control case
                RJReal In_slave; //nominal inertia of slave in bilateral control case
                double sum_error;
                int state_id;
                bool Disturbance_observer_start;
                RJReal tau_ref_prev;
                RJMatrix2 Hadamard;
                RJMatrix2 iHadamard;
                bool bilateralController_start;
                RJVector2 bilateralController_prev_tau;
		RJReal integrator;
		RJVector2 dx_modal_ref_forceLimitedPart2;

		RJVector2 bilateral_tau_ref_prev; //bilateral controllers previous actuator force(needed for disturbance observer)

		//### delete below line, its just a hack
		RJVector2 x_acc,f_sensi, dx_acci;

        };
}
#endif
