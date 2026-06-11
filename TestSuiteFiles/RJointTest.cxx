/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#include "RJointTest.h"

using namespace std;
using namespace Eigen;
using namespace RobotJoint;

bool RJointTest::matCompare4(RJMatrix4 a, RJMatrix4 b, double thresh)
{
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			if(fabs(a(i,j) - b(i,j)) > thresh) return false;
			if ( std::isnan( a(i,j) ) )             return false;
                        if ( std::isnan( b(i,j) ) )             return false;
                        if ( std::isinf( a(i,j) ) )             return false;
                        if ( std::isinf( b(i,j) ) )             return false;
		}
	}
	return true;
}

bool RJointTest::matCompare3(RJMatrix3 a, RJMatrix3 b, double thresh)
{
        for(int i = 0; i < 3; i++)
        {
                for(int j = 0; j < 3; j++)
                {
                        if(fabs(a(i,j) - b(i,j)) > thresh) return false;
                        if ( std::isnan( a(i,j) ) )             return false;
                        if ( std::isnan( b(i,j) ) )             return false;
                        if ( std::isinf( a(i,j) ) )             return false;
                        if ( std::isinf( b(i,j) ) )             return false;
                }
        }
        return true;
}

bool RJointTest::matCompare(RJMatrixX a, RJMatrixX b, double thresh)
{
	for(int i = 0; i < a.cols(); i++)
	{
		for(int j = 0; j < a.rows(); j++)
		{
			if(fabs(a(i,j) - b(i,j)) > thresh) return false;
                        if ( std::isnan( a(i,j) ) )             return false;
                        if ( std::isnan( b(i,j) ) )             return false;
                        if ( std::isinf( a(i,j) ) )             return false;
                        if ( std::isinf( b(i,j) ) )             return false;
		}
	}
	return true;
}

bool RJointTest::vecCompare2(RJVector2 a, RJVector2 b, double thresh)
{
        for(int i = 0; i < 2; i++)
        {
                if(fabs(a(i) - b(i)) > thresh) return false;
                if ( std::isnan( a(i) ) )           return false;
                if ( std::isnan( b(i) ) )           return false;
                if ( std::isinf( a(i) ) )           return false;
                if ( std::isinf( b(i) ) )           return false;
        }
        return true;
}

bool RJointTest::vecCompare3(RJVector3 a, RJVector3 b, double thresh)
{
        for(int i = 0; i < 3; i++)
        {
                if(fabs(a(i) - b(i)) > thresh) return false;
                if ( std::isnan( a(i) ) )           return false;
                if ( std::isnan( b(i) ) )           return false;
                if ( std::isinf( a(i) ) )           return false;
                if ( std::isinf( b(i) ) )           return false;
        }
        return true;
}

bool RJointTest::vecCompare4(RJVector4 a, RJVector4 b, double thresh)
{
	for(int i = 0; i < 4; i++)
	{
		if(fabs(a(i) - b(i)) > thresh) return false;
                if ( std::isnan( a(i) ) )           return false;
                if ( std::isnan( b(i) ) )           return false;
                if ( std::isinf( a(i) ) )           return false;
                if ( std::isinf( b(i) ) )           return false;
	}
	return true;
}

// template <typename T>
// bool RJointTest::vecCompareA(T &a, T &b, double thresh)
// {
//         int n;
//         n = a.size();
//         for(int i = 0; i < n; i++)
//         {
//                 if(fabs(a(i) - b(i)) > thresh) return false;
//                 if ( std::isnan( a(i) ) )           return false;
//                 if ( std::isnan( b(i) ) )           return false;
//                 if ( std::isinf( a(i) ) )           return false;
//                 if ( std::isinf( b(i) ) )           return false;
//         }
//         return true;
// }