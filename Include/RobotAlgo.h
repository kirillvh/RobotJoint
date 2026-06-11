/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#ifndef RobotAlgo_H
#define RobotAlgo_H
/**
* @mainpage RobotJoint
* RobotJoint is a robotics library specilizing in biped robot motion control and planning. It focuses on speed, good documentation and elegant * code and powerful algorithims. High speed is achived by using the matrix Eigen template library, SSE optimizations, caching 
* calculated results and multithreading(todo...). RobotJoint implements the following notable functions(work in progress).
* <ul>
* <li>ZMP Preview Control to generate a center of mass trajectory </li>
* <li>Newton Euler Inverse Dynamics</li>
* <li>Center of mass Jacobians</li>
* <li>more to do...</li>
* </ul> 
*/


#include <assert.h>
#define EPS 0.0000000000000001;
#define PI 3.14159265358979
// #include <vector>
#include "Eigen/StdVector"
#include <map>
#include "RJointTypes.h"

#include <memory>

#define want_thread
#include "RoboJointInclude.h"
#include "RobotJointUtils.h"

#include "Eigen/Dense"

#include <iostream>
#include <fstream>
#include <string>

//hard coded polynomial interpolation paramaters. TODO consider paramaterizing this
#define POLY_ORDER 4 //order of the polynomial to use
#define POLY_DEPTH 10 //number of datasamples to consider in the least squares fit of the polynomial
typedef Eigen::Matrix<RJReal, POLY_DEPTH, 1> PolyDepthVec;
typedef Eigen::Matrix<RJReal, POLY_ORDER, 1> PolyOrderVec;
typedef Eigen::Matrix<RJReal, POLY_DEPTH, POLY_ORDER> PolyMat;
typedef Eigen::Matrix<RJReal, POLY_ORDER, POLY_DEPTH> PolyMat2;
// using Eigen::RJMatrixX;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
// using Eigen::RJVector4;
using Eigen::MatrixXcd;
using Eigen::VectorXcd;

/**
* RobotJoint is a robotics library specilizing in biped robot motion control and planning. It focuses on speed, good documentation and elegant * code and powerful algorithims. High speed is achived by using the matrix Eigen template library, SSE optimizations, caching 
* calculated results and multithreading(todo...). RobotJoint implements the following notable functions(work in progress).
* <ul>
* <li>ZMP Preview Control to generate a center of mass trajectory </li>
* <li>Newton Euler Inverse Dynamics</li>
* <li>Center of mass Jacobians</li>
* <li>more to do...</li>
* </ul> 
*/
namespace RobotJoint{
        
        enum ValueMemory {
                Dont_Remember	= 0,
                Remember	= 1,
        };
        
        typedef std::map<std::string,std::ofstream*>::iterator LogIter;
        typedef std::map<std::string, std::vector< RJReal > >::iterator PolyIter;
        typedef std::map<std::string, std::vector< RJReal > >::iterator FilterIter;
        typedef std::map<std::string, PolyDepthVec, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, PolyDepthVec> > >::iterator StateIter;
	typedef std::map<std::string, Vector3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Vector3d> > >::iterator StateStrIter;

        /**
        *       @class RobotAlgo RobotAlgo.h "RobotAlgo.h"
        *       @brief Base Class of RobotJoint Objects
        */
        class RobotAlgo { // tolua_export
                protected:  
                RJReal _sampling_period;
                RJReal _sampling_frequency;
                Vector3d state_memory_simple; // [x;dx;ddx]
                Vector3d state_memory_simple_prev;
                Vector3d state_memory_simple_prev_prev;
		PolyDepthVec state_memory_simple_poly;

		//standard place to store pos, vel, acc data by integer id
                std::vector<Vector3d, Eigen::aligned_allocator<Vector3d> > state_memory;
                std::vector<Vector3d, Eigen::aligned_allocator<Vector3d> > state_memory_prev;
                std::vector<Vector3d, Eigen::aligned_allocator<Vector3d> > state_memory_prev_prev;

		//standard place to store pos, vel, acc data by string id
                std::map<std::string, Vector3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Vector3d> > > state_memory_str;
                std::map<std::string, Vector3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Vector3d> > > state_memory_prev_str;
                std::map<std::string, Vector3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Vector3d> > > state_memory_prev_prev_str;
                
                std::vector<Eigen::Matrix<RJReal, POLY_DEPTH, 1>, Eigen::aligned_allocator<PolyDepthVec> > state_memory_poly; //TODO state_memory_prev_prev and state_memory_simple_prev_prev and state_memory_poly should be merged
                PolyMat2 PolyConst; //stores the constant (X^T * X)^-1 * X^T, where X is the Vandermonde matrix

// std::map<int, Eigen::Vector4f, std::less<int>,
// Eigen::aligned_allocator<std::pair<const string, Eigen::PolyDepthVec> > >


		std::map<std::string, PolyDepthVec, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, PolyDepthVec> > > state_memory_map_poly;
                
		//these are static so that they are globaly accesible from all RobotJoint classes(since such classes should inherit from this base class)
                static double _time;  //time in decimal notation
                static int    _itime; //time in samples
                std::vector<RJVector4> FilterParams;
                std::map<std::string,std::ofstream*> Logs;
                
                std::map<std::string, std::vector< RJReal > > poly_coef;
                std::map<std::string, std::vector< RJReal > > Filters;
		double _quantization_level;
		bool startup;
		std::vector<bool> startup2;

                public:
                RobotAlgo();
                virtual int update(VectorXd &x){};
                void setSamplingPeriod(double sampling_period);
		RJReal getSamplingPeriod(){return _sampling_period;}
                void CalculatePolyConst();
		void Tick(){_itime++; _time=_sampling_period*_itime;}
		int getTimeInt(){return _itime;}
		void setTimeInt(int x){_itime = x;}
		double getTime(){return _time;}
		void setTime(double x){_time = x;}
                //accessors
                // tolua_begin  
                int Sec2Samples(RJReal sec);
                RJReal Samples2Sec(int samples);
		double getQuantizationLevel(){return _quantization_level;}
		double setQuantizationLevel(double ql){_quantization_level = ql;}
		inline double quantize(RJReal x){return floor(x/_quantization_level)*_quantization_level;}
		inline double quantize_vel(RJReal x){return floor(x*_sampling_period/(_quantization_level))*(_quantization_level/_sampling_period);}

		void update_state_memory(int id, RJVector3 v);
		void update_state_memory(RJVector3 v);
		void update_state_memory(std::string id, RJVector3 v);
                Vector3d update_state_x(double x, double dt);
		Vector3d update_state_x(std::string id, double x, double dt);
		Vector3d update_state_x_poly(double x, double dt);
                Vector3d update_state_dx(double x, double dt);
                Vector3d update_state_ddx(double x, double dt);
                
                Vector3d update_state_x(int id, double x, double dt);
                Vector3d update_state_x_poly(int id, double x, double dt);
		Vector3d update_state_x_poly(std::string id, double x, double dt);
                Vector3d update_state_dx(int id, double x, double dt);
                Vector3d update_state_ddx(int id, double x, double dt);
        
                int addState();
		int addState(std::string id);
                int addState(double x);
//                 int addState(int size_rows, int size_cols);
                
                double getState_x();
                double getState_dx();
                double getState_ddx();

                double getState_x(std::string id);
                double getState_dx(std::string id);
                double getState_ddx(std::string id);
                
                double getState_x(int id);
		double getState_dx(int id);
		double getState_ddx(int id);
//                 double getState_dx(int id){ return state_memory[id](1);}
//                 double getState_ddx(int id){ return state_memory[id](2);}
        
                //NOTE: The Intergrators and differentiators dont work well with float precission
                //integrators
                double integ_euler(double x, double dx, double dt);
                double integ_verlet(double x, double dx, double ddx, double dt);

                //differentiators
                double pseudo_diff_velocity(double x, double x_old, double dt);
                double pseudo_diff_acceleration(double x, double x_old, double x_old_old, double dt);        

                // logging functions
                bool addLog(std::string LogName);
                bool Log(std::string LogName, RJReal data1);
                bool Log(std::string LogName, std::vector<RJMatrixX> data1);
//                 bool Log(std::string LogName, const RJMatrixX &data1);
                template <typename Derived>
                bool Log(std::string LogName,const Eigen::MatrixBase<Derived>& data)
                {
                        LogIter Iter = Logs.find(LogName);
                        //If the log doesnt exist then create it
                        if(Iter==Logs.end())
                        {
                                addLog(LogName);
                                Iter = Logs.find(LogName); //should be able to find the log now
                        }
                        
                        for(int i=0; i<data.rows(); i++)
                        {
                                (*Iter->second) << i <<" "<<data.row(i) <<std::endl;
                        }

                        
                        return true;
                }
                
                template <typename Derived>
                bool Log(std::string LogName,double time, const Eigen::MatrixBase<Derived>& data)
                {
                        LogIter Iter = Logs.find(LogName);
                        //If the log doesnt exist then create it
                        if(Iter==Logs.end())
                        {
                                addLog(LogName);
                                Iter = Logs.find(LogName); //should be able to find the log now
                        }
                        
                        (*Iter->second) << time;
                        for(int i=0; i<data.rows(); i++)
                        {
                                (*Iter->second) <<" "<<data.row(i);
                        }
                        (*Iter->second)<<std::endl;
                        
                        return true;
                }
                // filtering functions
                bool addFilter(const std::string &id, RJReal cutoff);
                RJReal Filter(const std::string &FilterName, RJReal xin, RJReal cutoff);
                
                //polynomial function
                bool poly5thMakeCoefficents(std::string id, RJReal tstart, RJReal tend, RJReal xin, RJReal xout, RJReal dxin=0, RJReal dxout=0, RJReal ddxin=0, RJReal ddxout=0);
                RJReal poly5th(std::string id,RJReal time);
                bool addPoly(std::string id);
		void PI2minusPI(RJVectorX &x);
                //This redefines the new operator for SSE support(allows for 16-bit boundry alignment)
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                
        };
        // tolua_end
}

#endif // SAMPLEPD_H
