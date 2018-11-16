#pragma once
#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
//#include <bits/stdc++.h>
//#include <Mtransform.h>

namespace RoboticsMath 
{
       // typedef Eigen::Matrix<double, 4, 4> Matrix4d;
	typedef Eigen::Matrix<double, 6, 6> Matrix6d; 
        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        typedef Eigen::Matrix<double, 7, 1> Vector7d;
	typedef Eigen::Matrix<double, 8, 1> Vector8d;


        double deg2rad(double degrees);
        double sgn(double x);
        double vectornorm(Eigen::Vector3d v);
        Eigen::Matrix3d orthonormalize(Eigen::Matrix3d R);
        Eigen::Matrix4d assembleTransformation(Eigen::Matrix3d Rot, Eigen::Vector3d Trans);
        RoboticsMath::Vector7d collapseTransform(Eigen::Matrix4d T);
        Eigen::Matrix3d quat2rotm(Eigen::Vector4d Quat);
        Eigen::Vector4d rotm2quat(Eigen::Matrix3d R);

        void getCofactor(double A[6][6], double temp[6][6], int p, int q, int n);
        double determinant(double A[6][6], int n);
        void adjoint(double A[6][6], double adj[6][6]);
        void inverse(double A[6][6], double inverse[6][6]);

        Eigen::Matrix3d hat3(Eigen::Vector3d v);
        RoboticsMath::Matrix6d Adjoint_p_q(Eigen::Vector3d p, Eigen::Vector4d q);
        Eigen::Matrix4d inverseTransform(Eigen::Matrix4d T);
        Eigen::Vector4d slerp(Eigen::Vector4d qa, Eigen::Vector4d qb, double t);
        Eigen::Matrix<double,4,Eigen::Dynamic> quatInterp(Eigen::Matrix<double, 4,Eigen::Dynamic> refQuat, Eigen::VectorXd refArcLengths, Eigen::VectorXd interpArcLengths);
}
