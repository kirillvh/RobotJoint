/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#include "Controller.h"

#define xm_STATE_ID 0
#define xs_STATE_ID 1
#define x_diff_STATE_ID 2

using namespace std;
using namespace Eigen;
using namespace RobotJoint;

void Controller::setupMemberData()
{
	dx_modal_ref_forceLimitedPart2 = RJVector2::Zero();
        addState(0); //bilateral controller states, xm    = 0
        addState(0); //                            xs     = 1
        addState(0); //                            x_diff = 2
        Hadamard<<1.0, 1.0, 1.0, -1.0;
        iHadamard<<0.5, 0.5, 0.5, -0.5;
        In_master = 1.0;
        In_slave = 1.0;
	integrator = 0;
	bilateral_tau_ref_prev = RJVector2::Zero();
	setSamplingPeriod(0.001);
}

Controller::Controller() : Disturbance_observer_start(true), tau_ref_prev(0), bilateralController_start(true)
{
	Controller::setupMemberData();
}

Controller::Controller(const double &sampling_period) : Disturbance_observer_start(true), tau_ref_prev(0), bilateralController_start(true)
{
	Controller::setupMemberData();
        setSamplingPeriod(sampling_period);
}

Controller::Controller(const RJReal &Jn, const double &sampling_period) : Disturbance_observer_start(true), tau_ref_prev(0), bilateralController_start(true)
{
	Controller::setupMemberData();
        In = Jn;
        setSamplingPeriod(sampling_period);
}


Controller::Controller(const RJMatrixX &Ai, const RJMatrixX &Bi, const RJMatrixX &Ci, const RJMatrixX &Di, const RJMatrixX &Qi, const RJMatrixX &Ri) : Disturbance_observer_start(true), tau_ref_prev(0), bilateralController_start(true)
{
	Controller::setupMemberData();
        A = Ai;
        B = Bi;
        C = Ci;
        D = Di;
        Q = Qi;
        R = Ri;
}

/** \brief Attempts to solve Matrix Ricatti equation with brute force
This method is computationally inefficent and might not converge to a global optimum. Better to use the Newton method.
*/
bool SolveRicattiBrute(RJMatrixX &A, RJMatrixX &B, RJMatrixX &Q, RJMatrixX &R, RJMatrixX &P,int itterations)
{
        RJMatrixX Rinv(R.rows(),R.cols());
        RJMatrixX temp(R.rows(),R.cols());
        RJMatrixX P_old(Q.rows(),Q.cols());
        
        P=RJMatrixX::Zero(Q.rows(),Q.cols());
        
        //itterate the Ricatti equation to solve for P in a brute force fashion
        for (int i=0;i<itterations;i++)
        {
                temp = R+B.transpose()*P*B;
                pseudoInverse(temp,Rinv);
                P=A.transpose()*P*A.transpose()-(A.transpose()*P*B)*(Rinv)*(B.transpose()*P*A) + Q;
                //At'*P*At-(At'*P*Bt)*(1.0/(R+Bt'*P*Bt))*(Bt'*P*At)+Qt;
        }
        
        //check if P converged
        if ((P-P_old).sum()> 1.0) return false;
        else                      return true;
}

int Controller::update(RJMatrixX measured, RJMatrixX measurable)
{

}

RJVectorX Controller::Observe(RJMatrixX y)
/**     @brief constructs the full state of a variable up to 2nd order using differntiators/integrators
        y= 3 X #DOF (1 for SISO)
        i.e y = [ [0;1;0] [1;0;0]] will consider the input data of first variable to be velocity based and the second variable to be position based. Then integrators and differentiators will be used to complete the unkown position, velcoity and acceleration states.
*/
{
        //first estimate the states with the first observable state
//         for(int i=0;i<y.rows();i++)
//         {
//                 if (y())
//                 if(Obs_Type(i,position)>0)     this->update_state_x(state_id,i,y(i,position), _sampling_period);
//                         else if(Obs_Type(i,velocity)>0)        this->update_state_dx(state_id,i,y(i,1), _sampling_period);
//                                 else if(Obs_Type(i,acceleration)>0)        this->update_state_ddx(state_id,i,y(i,2), _sampling_period);
//         }
//         
//         //next overwrite the estimated states with the measured states. Consider Kalman filter for sensor fusion later
//         for(int i=0;i<y.size();i++)
//         {
//                 if(Obs_Type(i,0)>0)     this->state_memory[state_id](i,0) = y(i,0);
//                         else if(Obs_Type(i,1)>0)        this->state_memory[state_id](i,1) = y(i,1);
//                                 else if(Obs_Type(i,2)>0)        this->state_memory[state_id](i,2) = y(i,2);
//         }
}

RJVectorX Controller::Feedback(RJVectorX x_ref, RJVectorX x)
/** @brief Multiply error by gains
*/
{
        RJVectorX u(Gains.rows());
        u = -Gains*(x-x_ref);
        return u;
}

RJVectorX Controller::Predict(RJVectorX x, RJVectorX u, double dt)
{
        RJVectorX newx(x.rows());
        newx = dt*(A*x +B*u) + x;
        return newx;
}

int Controller::setGains(RJVectorX x)
{
        Gains = x;
        return RJ_OK;
}

RJVectorX Controller::getGains()
{
        return Gains;
}

RJMatrixX Controller::PreviewControl(const RJMatrixX &OutputRef)
/**     @brief Generates a state trajectory using preview control
* Applies  u[i] = -Gx*x[i] - Gi*sum{j=0_to_i}(x[j]-outputref[j])
*                 - sum{j=i_to_i+preview_window_size}(Gd[j]*outputref[j])
*/
{
        RJMatrixX ret(OutputRef.rows(),A.cols());
        RJVectorX XState = RJVectorX::Zero(A.rows());
        RJVectorX XStateRef = RJVectorX::Zero(A.rows());
        RJVectorX Y(C.rows());
        RJVectorX U(B.cols());
        RJVectorX SumOutputError(C.rows());
        RJVectorX OutputError(C.rows());        
        
        for(int i=0; i<OutputRef.rows(); i++)
        {
                U = RJVectorX::Zero(U.rows());
                for (int j = 0; j<preview_gain.size(); j++)
                {
                        if(i+j < OutputRef.size()) U -= preview_gain[j]*OutputRef.row(i+j);
                        else U -= preview_gain[j]*OutputRef.row(OutputRef.size()-1);
                } // TODO need a efficent multiply accumulate method around here
                for (int j = 0; j<3; j++)
                {
                        U(0,0) = U(0,0) - GX(0,j)*(XState(j) - XStateRef(j));
                }    
                //-GX.cwiseProduct(XState)
                U +=  GI*SumOutputError;

                XState = A*XState + B*U;
                Y = C*XState;
                OutputError = -Y + OutputRef.row(i);
                SumOutputError += OutputError;
                XStateRef = RJVectorX::Zero(3);

                ret(i,0) = XState(0);
                ret(i,1) = XState(1);
                ret(i,2) = Y(0,0);
        }
        return ret;
}

/**
*       \brief Designs Preview gains and feedback gains using the method by Katayama et al.
* Dimensions:  A - n*n , B - n*r , C - p*n
* Ref: "Design of an optimal controller for a discrete-time system subjected to previewable demand" by Touru Katayama, Takahira Ohki, Toshio Inoue and Tomoyuki Kato
*/
int Controller::CalcPreviewGain(const  int &preview_window_size)
{
        if (A.rows() != A.cols()) throw "CalcPreviewGain: A is not symetric!";
        if (A.rows() != B.rows()) throw "CalcPreviewGain: A & B dont have equal number of rows!";
        if (Q.rows() != Q.cols()) throw "CalcPreviewGain: Q is not symetric!";
        if (Q.rows() != A.rows()) throw "CalcPreviewGain: A & B dont have equal number of rows!";
    
        int n = A.rows();
        int r = B.cols();
        int p = C.rows();
        RJMatrixX CB = C*B;
        RJMatrixX CA = C*A;
        RJMatrixX Ip = RJMatrixX::Ones(p,p);//Identity
        RJMatrixX Btilda = RJMatrixX::Zero(CB.rows()+B.rows(),B.cols());
        RJMatrixX Ftilda = RJMatrixX::Zero(CA.rows()+A.rows(),A.cols());
        RJMatrixX Itilda = RJMatrixX::Zero(CA.rows()+A.rows(),Ip.cols());
        RJMatrixX Atilda = RJMatrixX::Zero(Itilda.rows(),Itilda.cols()+Ftilda.cols());
        RJMatrixX Qtilda = RJMatrixX::Zero(Atilda.rows(),Atilda.cols());       

        Btilda.topLeftCorner(CB.rows(),CB.cols())  = CB;
        Btilda.bottomLeftCorner(B.rows(),B.cols()) = B;

        Ftilda.topLeftCorner(CA.rows(),CA.cols())  = CA;
        Ftilda.bottomLeftCorner(A.rows(),A.cols()) = A;

        Itilda.topLeftCorner(Ip.rows(), Ip.cols()) = Ip;

        Atilda.topLeftCorner (Itilda.rows(),Itilda.cols()) = Itilda;
        Atilda.topRightCorner(Ftilda.rows(),Ftilda.cols()) = Ftilda;

//         Qtilda.topLeftCorner(Q.rows(),Q.cols()) = Q; // implement Qx     Qtilda=[Qe 0;0 Qx]
//      ### HACK need to properly take care of this matrix
        Qtilda(0, 0) = 1.0;

        RJMatrixX Ktilda = RiccatiSolveDiscrete(Atilda,Btilda, Qtilda, R);
        RJMatrixX part0 = (R+Btilda.transpose()*Ktilda*Btilda);
        RJMatrixX part1;
        pseudoInverse(part0 , part1);
        RJMatrixX part3 = part1*Btilda.transpose();
        RJMatrixX part2 = part3*Ktilda;
        GI = part2*Itilda;
        GX = part2*Ftilda;
        
        //now generate preview gain
        preview_gain.push_back(-GI);
        RJMatrixX Actilda = Atilda - Btilda*part2*Atilda;
        RJMatrixX Xl = -1*Actilda.transpose()*Ktilda*Itilda;   
        for(int i=0; i<preview_window_size;i++)
        {
                preview_gain.push_back(part3*Xl);
                Xl = Actilda.transpose()*Xl;
        }
        return 0;
}

RJReal Controller::P_feedback(RJReal gain, RJReal x, RJReal xref)
/**     @brief Proportional feedback for position signal

*/
{
	xref = quantize(xref);
        return -gain*(x - xref);
}

RJReal Controller::P_feedback_vel(RJReal gain, RJReal dx, RJReal dxref)
/**     @brief Proportional feedback for velocity signal

*/
{
//  	dxref = quantize_vel(dxref);
        return -gain*(dx - dxref);
}

RJReal Controller::PI_feedback(RJReal gain, RJReal integ_gain, RJReal x, RJReal xref)
{
// 	double quantization_level = 0.001533981;
	xref = quantize(xref);
	integrator += x - xref;
        return -gain*(x - xref)-integ_gain*integrator;
}

RJReal Controller::PD_feedback(RJReal gain_p, RJReal gain_d, RJReal x, RJReal xref, RJReal dxref)
/**     @brief Proportional derivative feedback
        Velocity is estimated using the base class's(RobotAlgo) "update_state_x" method which uses discrete differentiation internally

*/
{
        Vector3d state = update_state_x_poly(x, _sampling_period);
//  	xref = quantize(xref);
        return -gain_p*(x - xref) - gain_d*(state(1) - dxref);
}

RJReal Controller::PID_feedback(RJReal gain_p, RJReal gain_d, RJReal gain_i, RJReal x, RJReal xref, RJReal dxref)
{
        Vector3d state = update_state_x_poly(x, _sampling_period);
 	xref = quantize(xref);
	dxref = quantize_vel(dxref);
	integrator += x - xref;
        return -gain_p*(x - xref) - gain_d*(state(1) - dxref) - gain_i*integrator;
}

RJReal Controller::PD_feedback2(RJReal gain_p, RJReal gain_d, RJReal x, RJReal xref, RJReal dx, RJReal dxref)
/**     @brief Proportional derivative feedback
        Both the position and the velocity are explicitly provided to this function

*/
{
// 	xref = quantize(xref);
        return -gain_p*(x - xref) - gain_d*(dx - dxref);
}

RJReal Controller::Disturbance_Observer(RJReal tau_ref, RJReal Jn, RJReal LPF_cutoff, RJReal tau_lim_pos, RJReal tau_lim_neg)
/**     @brief Applies the torque disturbance observer
        MUST CALL PD_feedback before using this function so that the state can be calculated!
        Filtering is applied with the base class's(RobotAlgo) builtin butterworth filter.
*/
{
        if(Disturbance_observer_start)
        {
                Disturbance_observer_start  = false;
                addFilter("D_Obs",LPF_cutoff);
        }
        
        double x,y;
        RJReal tau_dis_hat,tau_dis;
        
        tau_dis = -tau_ref_prev + Jn*getState_ddx();
        tau_dis_hat = Filter("D_Obs", tau_dis, LPF_cutoff);
        
        tau_ref_prev = tau_ref - tau_dis_hat;
// 	tau_ref_prev = floor(tau_ref_prev/0.01206)*0.01206;//makes it worse!
// 	tau_ref_prev = tau_ref;
        tau_ref_prev = limit(tau_ref_prev,tau_lim_pos, tau_lim_neg);
        return tau_ref_prev;
}

RJReal Controller::Disturbance_Observer2(RJReal tau_ref, RJReal ddx, RJReal Jn, RJReal LPF_cutoff, std::string name, RJReal tau_lim_pos, RJReal tau_lim_neg)
/**     @brief Applies the torque disturbance observer
        This version of the Disturbance_Observer function takes the acceleration response (ddx) as paramater.
*/
{
//         if(Disturbance_observer_start)
	
//         {
//                 Disturbance_observer_start  = false;
//                 addFilter("D_Obs"+name,LPF_cutoff);
//         }
        
        double x,y;
        RJReal tau_dis_hat,tau_dis;
        
        tau_dis = -tau_ref + Jn*ddx;
        tau_dis_hat = Filter("D_Obs"+name, tau_dis, LPF_cutoff); //will auto create if its a new filter
        
        return tau_dis_hat;
}

RJReal Controller::Disturbance_Observer3(RJReal tau_ref, RJReal ddx, RJReal LPF_cutoff, RJReal tau_lim_pos, RJReal tau_lim_neg)
/**     @brief Applies the torque disturbance observer
        This version of the Disturbance_Observer function takes the acceleration response (ddx) as paramater.
        Also this version of the Disturbance_Observer function uses the internal value of the inertia In.
*/
{
        return Disturbance_Observer2(tau_ref, ddx, In, LPF_cutoff,"std", tau_lim_pos, tau_lim_neg);
}

RJReal Controller::PD_P_Obs_Controller(RJReal gain_x, RJReal gain_dx, RJReal gain_dx2, RJReal x, RJReal x_ref, RJReal Jn, RJReal LPF_cutoff, RJReal tau_lim_pos, RJReal tau_lim_neg, RJReal vel_lim_pos, RJReal vel_lim_neg, RJReal forward_compensation)
/**     @brief Cascade of PD controller(vel. ref. generation), P control(tau ref. generation) and disturbance observer
        @param[in] gain_x Proportional gain used in velocity reference generation
        @param[in] gain_dx Derivative gain used in velocity reference generation
        @param[in] gain_dx2 Proportional gain used in torque reference generation
        @param[in] x Position response
        @param[in] x_ref Position reference
        @param[in] Jn nominal mass/inertia
        @param[in] gain_x Proportional gain used in velocity reference generation
	@param[in] forward_compensation forward compensation term
*/
{
        RJReal dxref = PD_feedback(gain_x, gain_dx, x, x_ref);	
        dxref = limit(dxref,vel_lim_pos, vel_lim_neg);
        RJReal tauref = P_feedback_vel(gain_dx2, getState_dx(), dxref);
	RJReal tauref2 = tauref;
// 	return tauref;
        tauref = Disturbance_Observer(tauref, Jn, LPF_cutoff, tau_lim_pos, tau_lim_neg)+forward_compensation;
// 	double frictionval = 0.083;
// 	if(getState_dx()>0.0) tauref += frictionval;
// 	else if(getState_dx()<-0.0) tauref += -frictionval;
// 	else if(getState_dx()==0.0)
// 	{
// 		if(tauref2>0) 	tauref += frictionval;
// 		else if(tauref2<0)tauref += -frictionval;
// 	}
	return tauref;
}
/*
	double frictionval = 0.083;
	if(getState_dx()>0.0) dxref += frictionval;
	else if(getState_dx()<-0.0) dxref += -frictionval;
	else if(getState_dx()==0.0)
	{
		if(dxref>0) 	dxref += frictionval;
		else if(dxref<0)dxref += -frictionval;
	}
	return dxref;
*/

RJReal Controller::PD_P_Obs_Controller2(RJReal gain_x, RJReal gain_dx, RJReal gain_dx2, RJReal x, RJReal x_ref, RJReal dx, RJReal ddx, RJReal Jn, RJReal LPF_cutoff, RJReal tau_lim_pos, RJReal tau_lim_neg, RJReal vel_lim_pos, RJReal vel_lim_neg)
/**     @brief Cascade of PD controller(vel. ref. generation), P control(tau ref. generation) and disturbance observer

        This version of this function takes the current position, velocity and acceleration as paramaters

        @param[in] gain_x Proportional gain used in velocity reference generation
        @param[in] gain_dx Derivative gain used in velocity reference generation
        @param[in] gain_dx2 Proportional gain used in torque reference generation
        @param[in] x Position response
        @param[in] x_ref Position reference
        @param[in] Jn nominal mass/inertia
        @param[in] gain_x Proportional gain used in velocity reference generation
*/
{
        RJReal dxref = PD_feedback(gain_x, gain_dx, x, x_ref);
        dxref = limit(dxref,vel_lim_pos, vel_lim_neg);
        RJReal tauref = P_feedback_vel(gain_dx2, dx, dxref);
        return Disturbance_Observer2(tauref, ddx, Jn, LPF_cutoff, "std", tau_lim_pos, tau_lim_neg);
}

RJReal Controller::PD_P_Obs_Controller3(RJReal gain_x, RJReal gain_dx, RJReal gain_dx2, RJReal x, RJReal x_ref, RJReal dx, RJReal ddx,  RJReal LPF_cutoff, RJReal tau_lim_pos, RJReal tau_lim_neg, RJReal vel_lim_pos, RJReal vel_lim_neg)
/**     @brief Cascade of PD controller(vel. ref. generation), P control(tau ref. generation) and disturbance observer

        This version of this function takes the current position, velocity and acceleration as paramaters.
        Also this version of this function uses the internal value of the inertia In.

        @param[in] gain_x Proportional gain used in velocity reference generation
        @param[in] gain_dx Derivative gain used in velocity reference generation
        @param[in] gain_dx2 Proportional gain used in torque reference generation
        @param[in] x Position response
        @param[in] x_ref Position reference
        @param[in] gain_x Proportional gain used in velocity reference generation
*/
{
        return PD_P_Obs_Controller2(gain_x, gain_dx, gain_dx2, x, x_ref, dx, ddx, In, LPF_cutoff, tau_lim_pos, tau_lim_neg, vel_lim_pos, vel_lim_neg);
}





RJReal Controller::velocityController(RJReal x, RJReal dxref, RJReal gain_dx2, RJReal Jn, RJReal LPF_cutoff, RJReal tau_lim_pos, RJReal tau_lim_neg)
/**     @brief Cascade of PD controller(vel. ref. generation), P control(tau ref. generation) and disturbance observer
        @param[in] gain_dx2 Proportional gain used in torque reference generation
        @param[in] x Position response
        @param[in] dx_ref Velocity reference
        @param[in] Jn nominal mass/inertia
*/
{
        update_state_x(x, _sampling_period);
        RJReal tauref = P_feedback_vel(gain_dx2, getState_dx(), dxref);
        return Disturbance_Observer(tauref, Jn, LPF_cutoff, tau_lim_pos, tau_lim_neg);
}

RJReal Controller::slidingModeController(RJReal x, RJReal dx, RJReal alpha, RJReal beta, RJReal dither, RJReal c)
/**     @brief Sliding mode controller
 *      @param[in] x position error
 *      @param[in] dx velocity
 *      @param[in] alpha gain applied if above hyperplane
 *      @param[in] beta gain applied if below hyperplane
 *      @param[in] dither 
 *      @param[in] c gradient of control surface s=c*x1+x2
 * 
 * Applies tau=mu*x + dither*sgn(s), where s=c*x1+x2, mu=alpha if s*x1>0 otherwise beta
 * for stability alpha>c(b-c) and beta < c(b-c) where b is damping. Also Kf > |disturbance_max| 
 * */
{
        RJReal s = c*x+dx;
        RJReal mu;
        if(sgn(s*x)>0.0) mu = alpha;
        else           mu = beta;
        
        return mu*x + dither*sgn(s);
}

RJVector2 Controller::bilateralController(RJReal kp, RJReal kd, RJReal kf, RJReal xm, RJReal xs, RJReal fm, RJReal fs)
/**     @brief Bilateral force/position control - calculates acctuator velocity internally
        @param[in] xm master position
        @param[in] xs slave position
        @param[in] fm master force
        @param[in] fs slave force
        @returns [torque_{ref}^{master} ; torque_{ref}^{slave}]
*/
{
        RJVector2 x_acc(xm, xs*1.0);
        RJVector2 f_acc(fm, fs);
        
        update_state_x_poly(xm_STATE_ID, xm, _sampling_period); //calculate velocity and acceleration using position data
        update_state_x_poly(xs_STATE_ID, xs, _sampling_period); //calculate velocity and acceleration using position data
        if(bilateralController_start)
        {
                //update the state two more times to get the differentiators to work correctly
                update_state_x_poly(xm_STATE_ID, xm, _sampling_period); //calculate velocity and acceleration using position data
                update_state_x_poly(xs_STATE_ID, xs, _sampling_period); //calculate velocity and acceleration using position data
                update_state_x_poly(xm_STATE_ID, x_acc(0), _sampling_period); //calculate velocity and acceleration using position data
                update_state_x_poly(xs_STATE_ID, x_acc(1), _sampling_period); //calculate velocity and acceleration using position data
                bilateralController_start = false;
        }
        
        RJVector2 dx_acc(getState_dx(xm_STATE_ID), getState_dx(xs_STATE_ID));
	RJVector2 ddx_acc(getState_ddx(xm_STATE_ID), getState_ddx(xs_STATE_ID));
        
        return bilateralController(kp, kd, kf, x_acc, dx_acc, ddx_acc, f_acc);
}

RJVector2 Controller::bilateralController(RJReal kp, RJReal kd, RJReal kf, RJReal xm, RJReal xs)
/**     @brief Bilateral force/position control - calculates acctuator force & velocity internally
        @param[in] xm master position
        @param[in] xs slave position
        @param[in] fm master force
        @param[in] fs slave force
        @returns [torque_{ref}^{master} ; torque_{ref}^{slave}]
*/
{
//         RJVector2 x_acc(xm, xs);
        RJVector2 f_acc;	
        
        update_state_x(xm_STATE_ID, xm, _sampling_period); //calculate velocity and acceleration using position data
        update_state_x(xs_STATE_ID, xs, _sampling_period); //calculate velocity and acceleration using position data      
        if(bilateralController_start)
        {
                //update the state two more times to get the differentiators to work correctly
                update_state_x(xm_STATE_ID, xm, _sampling_period); //calculate velocity and acceleration using position data
                update_state_x(xs_STATE_ID, xs, _sampling_period); //calculate velocity and acceleration using position data
                update_state_x(xm_STATE_ID, xm, _sampling_period); //calculate velocity and acceleration using position data
                update_state_x(xs_STATE_ID, xs, _sampling_period); //calculate velocity and acceleration using position data
                bilateralController_start = false;
        }
        

        f_acc(0) = -Disturbance_Observer2(bilateralController_prev_tau(0), Filter("Master_facc",getState_ddx(xm_STATE_ID),50.0), In_master, 0.1*_sampling_frequency,"master-RFOB");        
        f_acc(1) = 0;//-Disturbance_Observer2(bilateralController_prev_tau(1), Filter("Slave_facc", getState_ddx(xs_STATE_ID), 50.0/*Hz*/), In_slave, 0.050*_sampling_frequency/*Hz*/,"slave-RFOB");        
// cout<<"facc " <<f_acc.transpose()<<endl;
	RJVector2 x_acc(xm, xs);
        RJVector2 dx_acc(getState_dx(xm_STATE_ID), getState_dx(xs_STATE_ID));
	RJVector2 ddx_acc(getState_ddx(xm_STATE_ID), getState_ddx(xs_STATE_ID));

        bilateralController_prev_tau = bilateralController(kp, kd, kf, x_acc, dx_acc, ddx_acc, f_acc);
        return bilateralController_prev_tau;
}

RJVector2 Controller::bilateralController(RJReal kp, RJReal kd, RJReal kf, const RJVector2 &x_acc, const RJVector2 &dx_acc, const RJVector2 &ddx_acc, const RJVector2 &f_acc)
/**     @brief Bilateral force/position control
        @param[in] x_acc [xm;xs] i.e vector containing master and slave positions
        @returns [torque_{ref}^{master} ; torque_{ref}^{slave}]
        
        Experimental!!!
*/
{	
        RJVector2 x_modal, f_modal, dx_modal, ddx_acc_ref, ddx_modal, tau_ref, dis_hat;
        x_modal = 0.5*Hadamard*x_acc; // [x_common; x_differential]
        dx_modal = 0.5*Hadamard*dx_acc; // [dx_common; dx_differential]
        f_modal = Hadamard*f_acc; // [f_common; f_differential]

        
        //generatre common mode and differntial mode acceleration by applying position and force controllers
        //the following implements the goal of xm - xs = 0, i.e zero differential mode position
        ddx_modal(1) = PD_feedback2(kp, kd, x_modal(1), 0, dx_modal(1), 0);
        //the following implements the goal of fm + fs = 0, i.e zero common mode force
        ddx_modal(0) = -kf*f_modal(0);
        
        //now transform the modal trasformation into the acctuator's space
        ddx_acc_ref = iHadamard*ddx_modal;

	//velocity control transplant
        ddx_acc_ref(0) = P_feedback_vel(10.0, dx_acc(0),  ddx_acc_ref(0)/*should be dx_acc_ref*/);
	ddx_acc_ref(1) = P_feedback_vel(750.0, Filter("Slave_Vel", dx_acc(1), 200.0),  ddx_acc_ref(1)/*should be dx_acc_ref*/);
        //next generate the required torques
        tau_ref(0) = In_master*ddx_acc_ref(0);
        tau_ref(1) = In_slave*ddx_acc_ref(1);
// 	double slave_pos_ref = 0.5*ddx_acc_ref(1)*_sampling_period*_sampling_period*

// 	//debug
// 	tau_ref(0) = In_master*PD_feedback2(kp, kd, x_acc(0), 0, dx_acc(0), 0);
// 	tau_ref(1) = In_slave*PD_feedback2(kp, kd, x_acc(1), 0, dx_acc(1), 0);

	_quantization_level = 0.3;
// 	tau_ref = quantize(tau_ref);
// 	ddx = quantize(ddx);

	dis_hat(0) = Disturbance_Observer2(bilateral_tau_ref_prev(0),ddx_acc(0), In_master, 0.01*_sampling_frequency, "master");
	dis_hat(1) = Disturbance_Observer2(
	quantize(bilateral_tau_ref_prev(1)),
	quantize(Filter("Slave_Acc", ddx_acc(1), 50.0))
	, In_slave*1.0, 0.05*_sampling_frequency, "slave");
//                 RJReal Disturbance_Observer2(RJReal tau_ref, RJReal ddx, RJReal Jn, RJReal LPF_cutoff, std::string name = "std",
//                         RJReal tau_lim_pos = std::numeric_limits<RJReal>::max(),
//                         RJReal tau_lim_neg = -std::numeric_limits<RJReal>::max()
//                         );
	tau_ref -= dis_hat;

        bilateral_tau_ref_prev(0) = tau_ref(0);
        bilateral_tau_ref_prev(1) = tau_ref(1);
	
        
        return tau_ref;
}
