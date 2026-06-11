/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#include "RobotAlgo.h"

using namespace std;
using namespace Eigen;
using namespace RobotJoint;

double RobotAlgo::_time = 0;
int RobotAlgo::_itime = 0;

RobotAlgo::RobotAlgo(){
	_sampling_period    = 0.001  ; //[s]
	_sampling_frequency = 1000.0 ; //[Hz]
// 	_time = 0;
	setTime(0);
        CalculatePolyConst();
	_quantization_level=1E-9;
	startup=true;
}

void RobotAlgo::setSamplingPeriod(double sampling_period)
{
        _sampling_period    = sampling_period  ; //[s]
        _sampling_frequency = 1.0 / sampling_period ; //[Hz]
        CalculatePolyConst();
}

void RobotAlgo::CalculatePolyConst()
{
        PolyMat X;
        for(int i = 0; i < POLY_DEPTH; i++)
        {
                for(int j = 0; j < POLY_ORDER; j++)
                {
                        if (j==0) X(i,j) = 1.0;
			else      X(i,j) = pow(-_sampling_period * i, j);
                }
        }
        PolyConst = (X.transpose() * X).inverse()*X.transpose();
}

int RobotAlgo::Sec2Samples(RJReal sec){
 return sec/_sampling_period;
}

RJReal RobotAlgo::Samples2Sec(int samples){
  return samples*_sampling_period;
}

// int RobotAlgo::addState(int size_rows, int size_cols)
// {
//         MatrixXd temp=MatrixXd::Zero(size_rows*size_cols,3);
//         state_memory.push_back(temp);
//         state_memory_prev.push_back(temp);
//         state_memory_prev_prev.push_back(temp);
//         return state_memory_prev.size()-1; //returns the index
// }
int RobotAlgo::addState()
{
        Vector3d temp=Vector3d::Zero();
        PolyDepthVec temp2=PolyDepthVec::Zero();
        state_memory.push_back(temp);
        state_memory_prev.push_back(temp);
        state_memory_prev_prev.push_back(temp);
        state_memory_poly.push_back(temp2);
	startup2.push_back(true);
        return state_memory_prev.size()-1; //returns the index
}


int RobotAlgo::addState(std::string id)
{
        Vector3d temp=Vector3d::Zero();
        PolyDepthVec temp2=PolyDepthVec::Zero();
	this->state_memory_map_poly.insert(make_pair(id,temp2 ) );
	this->state_memory_str.insert(make_pair(id,temp) );
	this->state_memory_prev_str.insert(make_pair(id,temp) );
	this->state_memory_prev_prev_str.insert(make_pair(id,temp) );
//         state_memory.push_back(temp);
//         state_memory_prev.push_back(temp);
//         state_memory_prev_prev.push_back(temp);
        return state_memory_map_poly.size()-1; //returns the index
}

int RobotAlgo::addState(double x)
{
        Vector3d temp=Vector3d::Zero();
        PolyDepthVec temp2=PolyDepthVec::Zero();
        temp(0) = x;
        state_memory.push_back(temp);
        state_memory_prev.push_back(temp);
        state_memory_prev_prev.push_back(temp);
        state_memory_poly.push_back(temp2);
	startup2.push_back(true);
        return state_memory_prev.size()-1; //returns the index
}


void RobotAlgo::update_state_memory(std::string id, RJVector3 v)
{
	state_memory_prev_prev_str[id](0) = state_memory_prev_str[id](0);
	state_memory_prev_prev_str[id](1) = state_memory_prev_str[id](1);
	state_memory_prev_prev_str[id](2) = state_memory_prev_str[id](2);
	
	state_memory_prev_str[id](0) = state_memory_str[id](0);
	state_memory_prev_str[id](1) = state_memory_str[id](1);
	state_memory_prev_str[id](2) = state_memory_str[id](2);
	
	state_memory_str[id](0) = v(0);
	state_memory_str[id](1) = v(1);
	state_memory_str[id](2) = v(2);
}

void RobotAlgo::update_state_memory(int id, RJVector3 v)
{
	state_memory_prev_prev[id](0) = state_memory_prev[id](0);
	state_memory_prev_prev[id](1) = state_memory_prev[id](1);
	state_memory_prev_prev[id](2) = state_memory_prev[id](2);
	
	state_memory_prev[id](0) = state_memory[id](0);
	state_memory_prev[id](1) = state_memory[id](1);
	state_memory_prev[id](2) = state_memory[id](2);
	
	state_memory[id](0) = v(0);
	state_memory[id](1) = v(1);
	state_memory[id](2) = v(2);
}

void RobotAlgo::update_state_memory(RJVector3 v)
{
	state_memory_simple_prev_prev(0) = state_memory_simple_prev(0);
	state_memory_simple_prev_prev(1) = state_memory_simple_prev(1);
	state_memory_simple_prev_prev(2) = state_memory_simple_prev(2);
	
	state_memory_simple_prev(0) = state_memory_simple(0);
	state_memory_simple_prev(1) = state_memory_simple(1);
	state_memory_simple_prev(2) = state_memory_simple(2);
	
	state_memory_simple(0) = v(0);
	state_memory_simple(1) = v(1);
	state_memory_simple(2) = v(2);
}


/** \brief calculates velocity using pseudo differentiation

*/
double RobotAlgo::pseudo_diff_velocity(double x, double x_old, double dt)
{
        return (x - x_old)/dt;
}

/** \brief calculates acceleration using pseudo acceleration

*/
double RobotAlgo::pseudo_diff_acceleration(double x, double x_old, double x_old_old, double dt)
{
        return (x - 2.0*x_old + x_old_old)/(dt*dt);
}


/** \brief integrates a variable using the verlet method
*/
double RobotAlgo::integ_verlet(double x, double dx, double ddx, double dt)
{
        return x + dx*dt + ddx*0.5*dt*dt;
}

/** \brief integrates a variable using the euler method
*/
double RobotAlgo::integ_euler(double x, double dx, double dt)
{
        return x + dx*dt;
}

Vector3d RobotAlgo::update_state_x(double x, double dt)
/**     @brief updates state position, velocity & acceleration based on position data and its 1st and 2nd derivatives.
        This function uses a built-in state which can only handle one variable but doesnt need to be setup prior to usage.
*/
{
        Vector3d v;

	if(startup)
	{
        v(0) = x;
        v(1) = 0;
        v(2) = 0;
	update_state_memory(v);
	update_state_memory(v);
	startup = false;
	}
	else
	{
        v(0) = x;
        v(1) = pseudo_diff_velocity(x, state_memory_simple(0), dt);
        v(2) = pseudo_diff_acceleration(x, state_memory_simple(0), state_memory_simple_prev(0), dt);
	}

// 	cout<<"vel "<<v(1)<<" ; x "<<x<<"; state_memory_simple(0) "<<state_memory_simple(0)<<endl;
        
        //update states        
        update_state_memory(v);
        
        return v;
}

Vector3d RobotAlgo::update_state_x_poly(double x, double dt)
/**     @brief updates state position, velocity & acceleration based on polynomial fitting, extrapolation & differentiation

*/
{
        Vector3d v;
        
        //first shift the new value into state_memory_poly
	if(startup)
	{
		for(int i = POLY_DEPTH - 1; i > 0; i--) state_memory_simple_poly(i) = x;
		startup = false;
	}

        for(int i = POLY_DEPTH - 1; i > 0; i--) state_memory_simple_poly(i) = state_memory_simple_poly(i-1);
        state_memory_simple_poly(0) = x;
        
        //next calculate the new polynomial coefficents via least squares fitting
        PolyOrderVec a = PolyConst * state_memory_simple_poly;
        
        //   y = a(0) +  a(1)*t^1 +   a(2)*t^2 +   a(3)*t^3 +   a(4)*t^4 +   a(5)*t^5
        //  dy =         a(1)     + 2*a(2)*t^1 + 3*a(3)*t^2 + 4*a(4)*t^3 + 5*a(5)*t^4
        // ddy =                  + 2*a(2)     + 6*a(3)*t^1 +12*a(4)*t^2 +30*a(5)*t^3
        // next substitute t=0 to get the velocity and acceleration at the current time
        v(0) = x;
        v(1) = a(1);
        v(2) = 2.0*a(2);
        
//         //update states        
        update_state_memory(v);
        
        return v;
}

Vector3d RobotAlgo::update_state_x(int id, double x, double dt)
/**     @brief updates state position, velocity & acceleration based on position data and its 1st and 2nd derivatives.

*/
{
        Vector3d v;

	if(!startup2[id])
	{
        v(0) = x;
        v(1) = pseudo_diff_velocity(x, state_memory[id](0), dt);
        v(2) = pseudo_diff_acceleration(x, state_memory[id](0), state_memory_prev[id](0), dt);
	}
	else
	{
        v(0) = x;
        v(1) = 0;
        v(2) = 0;
	update_state_memory(id,v);
	update_state_memory(id,v);
	startup2[id] = false;
	}
        
        //update states        
        update_state_memory(id,v);
        
        return v;
}

Vector3d RobotAlgo::update_state_x_poly(int id, double x, double dt)
/**     @brief updates state position, velocity & acceleration based on polynomial fitting, extrapolation & differentiation

*/
{
        Vector3d v;
        
        //first shift the new value into state_memory_poly
	if(!startup2[id])
	{
        for(int i = POLY_DEPTH - 1; i > 0; i--) state_memory_poly[id](i) = state_memory_poly[id](i-1);
        state_memory_poly[id](0) = x;
	}
	else
	{
		for(int i = POLY_DEPTH - 1; i > 0; i--) state_memory_poly[id](i) = x;
		state_memory_poly[id](0) = x;
		startup2[id] = false;
	}
        
        //next calculate the new polynomial coefficents via least squares fitting
        PolyOrderVec a = PolyConst * state_memory_poly[id];
        
        //   y = a(0) +  a(1)*t^1 +   a(2)*t^2 +   a(3)*t^3 +   a(4)*t^4 +   a(5)*t^5
        //  dy =         a(1)     + 2*a(2)*t^1 + 3*a(3)*t^2 + 4*a(4)*t^3 + 5*a(5)*t^4
        // ddy =                  + 2*a(2)     + 6*a(3)*t^1 +12*a(4)*t^2 +30*a(5)*t^3
        // next substitute t=0 to get the velocity and acceleration at the current time
        v(0) = x;
        v(1) = a(1);
        v(2) = 2.0*a(2);
        
//         //update states        
        update_state_memory(id,v);
        
        return v;
}


//TODO fix this function, its somehow bugged use update_state_x(std::string id, ...) meanwhile
Vector3d RobotAlgo::update_state_x_poly(std::string id, double x, double dt)
/**     @brief updates state position, velocity & acceleration based on polynomial fitting, extrapolation & differentiation

*/
{
        Vector3d v;
        StateIter Iter = state_memory_map_poly.find(id);
        //If the state doesnt exist then create it
        if(Iter==state_memory_map_poly.end())
        {
                addState(id);
                Iter = state_memory_map_poly.find(id); //should be able to find the log now

		RJASSERT(Iter != state_memory_map_poly.end(), "RobotAlgo::update_state_x_poly, can't find state  "<<id<<endl);
		//initilize the state
		for(int i = POLY_DEPTH - 1; i > 0; i--) Iter->second(i) = x;
		Iter->second(0) = x;
        }

// 	if(!startup2[id])
// 	{
        for(int i = POLY_DEPTH - 1; i > 0; i--) Iter->second(i) = Iter->second(i-1);
        Iter->second(0) = x;
// 	}
// 	else
// 	{

// 		startup2[id] = false;
// 	}
        
        //next calculate the new polynomial coefficents via least squares fitting
        PolyOrderVec a = PolyConst * Iter->second;
        
        //   y = a(0) +  a(1)*t^1 +   a(2)*t^2 +   a(3)*t^3 +   a(4)*t^4 +   a(5)*t^5
        //  dy =         a(1)     + 2*a(2)*t^1 + 3*a(3)*t^2 + 4*a(4)*t^3 + 5*a(5)*t^4
        // ddy =                  + 2*a(2)     + 6*a(3)*t^1 +12*a(4)*t^2 +30*a(5)*t^3
        // next substitute t=0 to get the velocity and acceleration at the current time
        v(0) = x;
        v(1) = a(1);
        v(2) = 2.0*a(2);
        
//         //update states        
         update_state_memory(id,v);
        
        return v;
}

Vector3d RobotAlgo::update_state_x(std::string id, double x, double dt)
/**     @brief updates state position, velocity & acceleration based on position data and its 1st and 2nd derivatives.

*/
{
        Vector3d v;
        StateStrIter Iter = state_memory_str.find(id);
        //If the state doesnt exist then create it
        if(Iter==state_memory_str.end())
        {
                addState(id);
//                 Iter = state_memory_map_poly.find(id); //should be able to find the log now

		RJASSERT(Iter != state_memory_map_poly.end(), "RobotAlgo::update_state_x_poly, can't find state  "<<id<<endl);
		//initilize the state
        	v(0) = x;
        	v(1) = 0;
        	v(2) = 0;
		update_state_memory(id,v);
		update_state_memory(id,v);
        }
        v(0) = x;
        v(1) = pseudo_diff_velocity(x, state_memory_str[id](0), dt);
        v(2) = pseudo_diff_acceleration(x, state_memory_str[id](0), state_memory_prev_str[id](0), dt);
        //update state
        update_state_memory(id,v);
        return v;
}


Vector3d RobotAlgo::update_state_dx(double dx, double dt)
/**     @brief updates state position, velocity & acceleration based on velocity data, its 1st derivatives and integral.
        This function uses a built-in state which can only handle one variable but doesnt need to be setup prior to usage.
*/
{
        Vector3d v;
	if(startup)
	{
        v(0) = 0;
        v(1) = dx;
        v(2) = 0,
	update_state_memory(v);
	update_state_memory(v);
	startup = false;
	}
	else
	{
        v(0) = integ_euler(state_memory_simple(0), dx, dt);
        v(1) = dx;
        v(2) = pseudo_diff_acceleration(state_memory_simple(0), state_memory_simple_prev(0), state_memory_simple_prev_prev(0), dt);
	}
        
        //update states
        update_state_memory(v);
        
        return v;
}

Vector3d RobotAlgo::update_state_dx(int id, double dx, double dt)
/**     @brief updates state position, velocity & acceleration based on velocity data, its 1st derivatives and integral.

*/
{
        Vector3d v;
	if(startup2[id])
	{
        v(0) = 0;
        v(1) = dx;
        v(2) = 0;
	update_state_memory(id,v);
	update_state_memory(id,v);
	startup2[id] = false;
	}
	else
	{
        v(0) = integ_euler(state_memory[id](0), dx, dt);
        v(1) = dx;
        v(2) = pseudo_diff_acceleration(state_memory[id](0), state_memory_prev[id](0), state_memory_prev_prev[id](0), dt);	
	}
        
        //update states
        update_state_memory(id,v);
        
        return v;
}

Vector3d RobotAlgo::update_state_ddx(double ddx, double dt)
/**     @brief updates state position, velocity & acceleration based on acceleration data and its integrals.
        This function uses a built-in state which can only handle one variable but doesnt need to be setup prior to usage.
*/
{
        Vector3d v;
	if(startup)
	{
        //integrate velocity using euler method
        v(1) = 0;
        //integrate position using verlet method
        v(0) = 0;
        v(2) = ddx;
	update_state_memory(v);
	update_state_memory(v);
	startup = false;
	}
	else
	{
        //integrate velocity using euler method
        v(1) = integ_euler(state_memory_simple(1), ddx, dt);
        //integrate position using verlet method
        v(0) = integ_verlet(state_memory_simple(0), v(1), ddx, dt);
        v(2) = ddx;
	}
        
        //update states
        update_state_memory(v);
        
        return v;
}

Vector3d RobotAlgo::update_state_ddx(int id, double ddx, double dt)
/**     @brief updates state position, velocity & acceleration based on acceleration data and its integrals.

*/
{
        Vector3d v;
	if(startup2[id])
	{
        //integrate velocity using euler method
        v(1) = 0;
        //integrate position using verlet method
        v(0) = 0;
        v(2) = ddx;
	startup2[id] = false;
        update_state_memory(id,v);
        update_state_memory(id,v);
	}
	else
	{
        //integrate velocity using euler method
        v(1) = integ_euler(state_memory[id](1), ddx, dt);
        //integrate position using verlet method
        v(0) = integ_verlet(state_memory[id](0), v(1), ddx, dt);
        v(2) = ddx;	
	}
        
        //update states
        update_state_memory(id,v);
        
        return v;
}

/**
*       \brief Adds a new filestream to the 'Logs' map
*/
bool RobotAlgo::addLog(std::string LogName)
{
        this->Logs.insert(make_pair(LogName,new ofstream(LogName.c_str()) ) ); 
        return true;
}

/**
*       \brief Writes data with a timestamp to file
*/
bool RobotAlgo::Log(std::string LogName, RJReal data1)
{
        LogIter Iter = Logs.find(LogName);
        //If the log doesnt exist then create it
        if(Iter==Logs.end())
        {
                addLog(LogName);
                Iter = Logs.find(LogName); //should be able to find the log now
        }
        
        (*Iter->second) << getTime() << data1;
        
        return true;
}

bool RobotAlgo::Log(std::string LogName, std::vector<RJMatrixX> data1)
{
        LogIter Iter = Logs.find(LogName);
        //If the log doesnt exist then create it
        if(Iter==Logs.end())
        {
                addLog(LogName);
                Iter = Logs.find(LogName); //should be able to find the log now
        }
        
        for(int i=0; i<data1.size(); i++)
        {
                (*Iter->second) << i <<" "<<data1[i]<<endl;
        }

        
        return true;
}
// template <typename Derived>
// bool RobotAlgo::Log(std::string LogName,const EigenBase<Derived>& data)
// {
//         LogIter Iter = Logs.find(LogName);
//         //If the log doesnt exist then create it
//         if(Iter==Logs.end())
//         {
//                 addLog(LogName);
//                 Iter = Logs.find(LogName); //should be able to find the log now
//         }
//         
//         for(int i=0; i<data.rows(); i++)
//         {
//                 (*Iter->second) << i <<" "<<data.row(i) <<endl;
//         }
// 
//         
//         return true;
// }

// bool RobotAlgo::Log(std::string LogName, const RJMatrixX &data1)
// {
//         LogIter Iter = Logs.find(LogName);
//         //If the log doesnt exist then create it
//         if(Iter==Logs.end())
//         {
//                 addLog(LogName);
//                 Iter = Logs.find(LogName); //should be able to find the log now
//         }
//         
//         for(int i=0; i<data1.rows(); i++)
//         {
//                 (*Iter->second) << i <<" "<<data1.row(i) <<endl;
//         }
// 
//         
//         return true;
// }

bool RobotAlgo::addPoly(std::string id)
{
        this->poly_coef.insert(make_pair(id, std::vector<RJReal>(10) ) ); 
        return true;
}

bool RobotAlgo::poly5thMakeCoefficents(std::string id, RJReal tstart, RJReal tend, RJReal xin, RJReal xout, RJReal dxin, RJReal dxout, RJReal ddxin, RJReal ddxout)
{
        RJReal dt = tend - tstart;
        
        PolyIter poly_coefIter = poly_coef.find(id);
        //If this polynomial doesnt exist then create it
        if(poly_coefIter==poly_coef.end())
        {
                addPoly(id);
                poly_coefIter = poly_coef.find(id); //should be able to find the log now
        }

        
        poly_coefIter->second[0] = xin;
        poly_coefIter->second[1] = dxin;
        poly_coefIter->second[2] = ddxin/2.0;
        poly_coefIter->second[3] = (20.0*xout-20.0*xin-(8.0*dxout+12.0*dxin)*dt-(3.0*ddxin-ddxout)*pow(dt,2.0))/(2.0*pow(dt,3.0));
        poly_coefIter->second[4] = (30.0*xin-30.0*xout+(14.0*dxout+16.0*dxin)*dt +(3.0*ddxin-2.0*ddxout)*pow(dt,2.0))/(2.0*pow(dt,4.0));
        poly_coefIter->second[5] = (12.0*xout-12.0*xin-(6.0*dxout+6.0*dxin)*dt-(ddxin-ddxout)*pow(dt,2.0))/(2.0*pow(dt,5.0));
        
        poly_coefIter->second[6] = tstart;
        poly_coefIter->second[7] = tend;
        poly_coefIter->second[8] = xin;
        poly_coefIter->second[9] = xout;
        
        return true;
}

RJReal RobotAlgo::poly5th(std::string id, RJReal time)
{
        PolyIter poly_coefIter = poly_coef.find(id);
        //If this polynomial doesnt exist then create it
        if(poly_coefIter==poly_coef.end())
        {
                throw "RobotAlgo::poly5th Polynomail not found, first create it!";
        }
        
        if(time>=poly_coefIter->second[6] && time<poly_coefIter->second[7])
        {
                time -= poly_coefIter->second[6];
                return (poly_coefIter->second[0] + poly_coefIter->second[1]*time
                + poly_coefIter->second[2]*pow(time,2) + poly_coefIter->second[3]*pow(time,3)
                + poly_coefIter->second[4]*pow(time,4) + poly_coefIter->second[5]*pow(time,5) );
        }
        else if(time >= poly_coefIter->second[7])  return poly_coefIter->second[9];
        else if(time < poly_coefIter->second[6])   return poly_coefIter->second[8];
}

RJReal RobotAlgo::Filter(const std::string &FilterName, RJReal xin, RJReal cutoff)
{
        FilterIter Iter = Filters.find(FilterName);
        //If the log doesnt exist then create it
        if(Iter==Filters.end())
        {
                addFilter(FilterName, cutoff);
                Iter = Filters.find(FilterName); //should be able to find the filter now
        }
        
        //Implementing a Direct Form II filter
        //https://en.wikipedia.org/wiki/Digital_filter
        RJReal ret;
        RJReal w = xin - Iter->second[3]*Iter->second[5] - Iter->second[4]*Iter->second[6];
                
        ret = Iter->second[0]*w + Iter->second[1]*Iter->second[5] + Iter->second[2]*Iter->second[6];
        
        //update the delays
        Iter->second[6] = Iter->second[5];
        Iter->second[5] = w;
        
        return ret;
}

bool RobotAlgo::addFilter(const std::string &id, RJReal cutoff)
/** @brief adds a filter and designs it(low pass)
    @param cutoff Cutoff frequency in Hertz, must be less than half the sampling frequency
A simple 1st order butterworth filter is designed with the method 
in "High-Order Digital Parametric Equalizer Design" by High-Order Digital Parametric Equalizer Design

The structure of the Filters map implements a Digital Biquad filter:
        [b0 b1 b2 a1 a2 delay1 delay2 delay3 delay4]
          0  1  2  3  4  5         6      7    8
*/
{
        if (cutoff>0.5*_sampling_frequency) throw "RobotAlgo::addFilter() Filter frequency greater than Nyquist frequency. Filtering algorithm will be unstable!";
        
        std::vector<RJReal> FilterParams(8);
        #define Filter_N 1
        #define Filter_g 1.0
        #define Filter_g0 0.05
        #define Filter_eps 2.00756
        
        RJReal delta_omega_div2 = 3.14159 * cutoff * _sampling_period;
        RJReal Filter_beta = 0.49811*tan(delta_omega_div2); 
        RJReal Filter_D0 = Filter_beta + 1.0; 
        
        FilterParams[0] = (Filter_g*Filter_beta + Filter_g0) / Filter_D0;
        FilterParams[1] = (Filter_g*Filter_beta - Filter_g0) / Filter_D0;
        FilterParams[2] = 0.0;
        FilterParams[3] = (Filter_beta - 1.0) / Filter_D0;
        FilterParams[4] = 0.0;
        FilterParams[5] = 0.0;
        FilterParams[6] = 0.0;
        FilterParams[7] = 0.0;
//         FilterParams[8] = 0.0;
        
        this->Filters.insert(make_pair(id, FilterParams ) ); 
        
        return true;
}


double RobotAlgo::getState_x()
{	
	return state_memory_simple(0);
}
double RobotAlgo::getState_dx()
{
	//only return a value if its above the quantization threshold
// 	if(fabs(state_memory_simple(1)) > _quantization_level*_sampling_frequency)
	return state_memory_simple(1);
// 	else return 0;
}
double RobotAlgo::getState_ddx()
{
	//only return a value if its above the quantization threshold
// 	if(fabs(state_memory_simple(2)) > _quantization_level*_sampling_frequency*_sampling_frequency)
	return state_memory_simple(2);
// 	else return 0;
}

double RobotAlgo::getState_x(std::string id)
{
	StateStrIter itr = state_memory_str.find(id);
	if(itr==state_memory_str.end()) throw "getState_x("+id+") failed, state not found";
	if(fabs(itr->second(0))>1E-10) return itr->second(0);
	else return 0;
}
double RobotAlgo::getState_dx(std::string id)
{
	StateStrIter itr = state_memory_str.find(id);
	if(itr==state_memory_str.end()) throw "getState_dx("+id+") failed, state not found";
	if(fabs(itr->second(1))>1E-10) return itr->second(1);
	else return 0;
}
double RobotAlgo::getState_ddx(std::string id)
{
	StateStrIter itr = state_memory_str.find(id);
	if(itr==state_memory_str.end()) throw "getState_ddx("+id+") failed, state not found";
	if(fabs(itr->second(2))>1E-10) return itr->second(2);
	else return 0;
}

double RobotAlgo::getState_x(int id)
{
	if(fabs(state_memory[id](0))>1E-10) return state_memory[id](0);
	else return 0;
}
double RobotAlgo::getState_dx(int id)
{
	if(fabs(state_memory[id](1))>1E-10) return state_memory[id](1);
	else return 0;
}
double RobotAlgo::getState_ddx(int id)
{
	if(fabs(state_memory[id](2))>1E-10) return state_memory[id](2);
	else return 0;
}

void RobotAlgo::PI2minusPI(RJVectorX &x)
{
        for (int i=0;i!=x.rows();++i)
        {
//                 x(i) = fmod(x(i),PI);
                if (x(i)>0)
                        x(i) = fmod(x(i)+PI, 2.0*PI)-PI;
                else
                        x(i) = fmod(x(i)-PI, 2.0*PI)+PI;
        }

}
