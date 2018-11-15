#ifndef MEDLABTYPES_H
#define MEDLABTYPES_H

#endif // MEDLABTYPES_H
#pragma once
#include <Eigen/Dense>
#include <vector>
#include "Tube.h"
#include "BasicFunctions.h"
#include "Utility.h"
#include "spline.h"
#include "RoboticsMath.h"

namespace medlab
{
        typedef std::tuple < CTR::Tube< CTR::Functions::constant_fun< Eigen::Vector2d > >,
                             CTR::Tube< CTR::Functions::constant_fun< Eigen::Vector2d > >,
                             CTR::Tube< CTR::Functions::constant_fun< Eigen::Vector2d > > > Cannula3; // CTR3Robot architecture
        typedef CTR::Functions::constant_fun<Eigen::Vector2d> CurvFun;
        typedef std::tuple < CTR::Tube<CurvFun>, CTR::Tube<CurvFun>, CTR::Tube<CurvFun> > CannulaT;
        typedef CTR::Tube<CTR::Functions::constant_fun<CTR::Vector<2>::type> > TubeType;
        typedef CTR::DeclareOptions < CTR::Option::ComputeJacobian, CTR::Option::ComputeGeometry, CTR::Option::ComputeStability, CTR::Option::ComputeCompliance>::options OType;

        struct CTR3RobotParams {

                // Material Properties
                double E;
                double G;

                // Tube 1 Geometry
                double L1;
                double Lt1;
                double k1;
                double OD1;
                double ID1;

                // Tube 2 Geometry
                double L2;
                double Lt2;
                double k2;
                double OD2;
                double ID2;

                // Tube 3 Geometry
                double L3;
                double Lt3;
                double k3;
                double OD3;
                double ID3;
        };

        struct CTR3KinematicsInputVector { // format of the state vector fed into Kinematics_with_dense_output()
                Eigen::Vector3d PsiL;
                Eigen::Vector3d Beta;
                Eigen::Vector3d Ftip;
                Eigen::Vector3d Ttip;
        };

        struct InterpRet {      // Interpolated CTR3 Backbone
                Eigen::VectorXd s;
                Eigen::MatrixXd p;
                Eigen::MatrixXd q;
        };

        struct WeightingRet { // Joint Limits Weighting Matrix (Configuration Dependent)
                RoboticsMath::Matrix6d W;
                Eigen::Vector3d dh;
        };

        struct KinOut {  // used by ResolvedRates.init() -> Online Resolved Rates Loop
                Eigen::Vector3d p;
                Eigen::Vector4d q;
                Eigen::Vector3d alpha;
                Eigen::Vector3d psiBeta;
                RoboticsMath::Matrix6d jBody;		// Body Jacobian
        };

}
