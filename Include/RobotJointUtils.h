/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#ifndef ROBOTJOINTUTILS_H
#define ROBOTJOINTUTILS_H
#include "Eigen/SVD"
#include <iostream>
#include "Eigen/Dense"
// #include <Eigen/Eigenvalues> 
// #include <vector>
#include "Eigen/StdVector"
#include <bitset>
#include <exception>
#include "RJointTypes.h"

// #define limit(x,y,up,down) if(x>up) y = up; else if(x<down) y = down; else y = x;

using Eigen::MatrixXd;
// using Eigen::RJVectorX;
// using Eigen::RJMatrixXc;
using Eigen::VectorXcd;

namespace RobotJoint
{
        //LAPACK: reorder the real Schur factorization  of a real matrix                          
        extern "C" void dtrsen_(const char *job, const char *compq, int *select, int *n, double *t, int *ldt, double *q, int *ldq, double *wr, double *wi, int *m, double *s, double *sep, double *work, int *lwork, int *iwork, int *liwork, int *info);

                       

        template<typename _Matrix_Type_>
        bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, RJReal
        epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
        {
                if(a.rows()<a.cols())
                        return false;

                Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeFullU |Eigen::ComputeFullV);
        
                typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(),
                a.rows()) * svd.singularValues().array().abs().maxCoeff();

                result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() >
                tolerance).select(svd.singularValues().array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();
        }
    
        template<typename _Matrix_Type_>
        bool pseudoInverseNonSquare(const _Matrix_Type_ &a, _Matrix_Type_ &result, RJReal
        epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
        {
                if(a.rows()<a.cols())
                        return false;

                Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeFullU |Eigen::ComputeFullV);
        
                typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(),
                a.rows()) * svd.singularValues().array().abs().maxCoeff();

                _Matrix_Type_ sigma = _Matrix_Type_::Zero(a.cols(), a.rows());
                sigma.block(0, 0, a.cols(), a.cols())<<_Matrix_Type_(_Matrix_Type_(
                (svd.singularValues().array().abs() >
                tolerance).select(svd.singularValues().array().inverse(), 0) ).asDiagonal());

                result = (svd.matrixV() * sigma) * svd.matrixU().adjoint();
        }

        template< typename T>
        inline T limit(const T &x ,const T &up,const T &down)
        {
                if(x>up) return up;
                else if(x<down) return down;
                else return x;
        }
        
        template< typename T>
        inline T sgn(const T &x)
        {
                if(x>0.0) return 1.0;
                else return -1.0;
        }

        template< typename T>
	inline bool isFloatEqual(const T x, const T y)
	{
		if(fabs(x-y) < std::numeric_limits<T>::epsilon()*2) return true;
		else return false;
	}

        //quadratic programing stuff
        RJVectorX SolveQp(const RJMatrixX &A,const RJVectorX &B,const RJMatrixX &C,const RJVectorX &D);
        RJVectorX QP_Thiel_VanDePanne(const RJMatrixX &A,const RJVectorX &B,const RJMatrixX &C,const RJVectorX &D);
        bool BelowHyperPlanes(const RJVectorX &x,const RJMatrixX &C,const RJVectorX &D);
        

        RJMatrixX RiccatiSolve(const RJMatrixX &A,const  RJMatrixX &B,const  RJMatrixX &Q,const  RJMatrixX &R);
        RJMatrixX RiccatiSolveDiscrete(const RJMatrixX &A,const  RJMatrixX &B,const  RJMatrixX &Q,const  RJMatrixX &R);
        RJMatrixX LQR(const RJMatrixX &A,const  RJMatrixX &B,const  RJMatrixX &Q,const  RJMatrixX &R);
        RJMatrixX LQRDiscrete(const RJMatrixX &A, const RJMatrixX &B, const RJMatrixX &Q,const  RJMatrixX &R);
        int CalcPreviewGain(const RJMatrixX &A, const RJMatrixX &B, const RJMatrixX &C, const RJMatrixX &Q, const RJMatrixX &R, const int &preview_window_size, std::vector<RJMatrixX> &preview_gain, RJMatrixX &GI, RJMatrixX &GX);
        VectorXcd ordeig(const MatrixXd &t);
        bool OrdSchur(MatrixXd& T,MatrixXd& Q, int * SELECT);
}
#endif