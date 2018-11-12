#pragma once
#include <Eigen/Dense> // this includes Eigen/Geometry
//#include <QDebug>
//#include <QString>
//#include <QVector>
#include <vector>

#include "Kinematics.h"
#include "BasicFunctions.h"
#include "Tube.h"
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

	struct CTR3ModelStateVector { // format of the state vector fed into Kinematics_with_dense_output()
		Eigen::Vector3d psiL_;
		Eigen::Vector3d beta_;
		Eigen::Vector3d fTip_;
		Eigen::Vector3d tTip_;
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
		Eigen::Vector3d p_;
		Eigen::Vector4d q_;
		Eigen::Vector3d alpha_;
		Eigen::Vector3d psiBeta_;
		RoboticsMath::Matrix6d jBody_;		// Body Jacobian
	};

}

class CTR3Robot
{

public:
	CTR3Robot();
	~CTR3Robot();
	void init();

	bool SetCannulaName(std::string cannula);
	std::string GetCannulaName();

	bool SetCannula(const medlab::CTR3RobotParams params); // setting a pointer is better practice? shared pointers?
	medlab::Cannula3 GetCannula(); // should this return the current cannula tuple or just params? 

	bool SetCurrStateVector(medlab::CTR3ModelStateVector stateVector);
	medlab::CTR3ModelStateVector GetCurrStateVector();

	bool SetCurrQVec(RoboticsMath::Vector6d qVec);
	RoboticsMath::Vector6d GetCurrQVec();

	bool SetInterpolatedBackbone(medlab::InterpRet interpolatedBackbone);
	medlab::InterpRet GetInterpolatedBackbone();

	medlab::KinOut callKinematicsWithDenseOutput(RoboticsMath::Vector6d);
	Eigen::MatrixXd forwardKinematics(auto kin);

	medlab::InterpRet interpolateBackbone(Eigen::VectorXd sRef, Eigen::MatrixXd poseDataRef, int nPts);

private:

	std::string cannulaName_;
	medlab::Cannula3 cannula_;  // cannula object fed to kinematics call
	medlab::CTR3RobotParams currCannulaParams_; // params that define the cannula3
	medlab::CTR3ModelStateVector currStateVector_; // state vector fed to kinematics call //TODO: store full q_vector and function to condense
	RoboticsMath::Vector6d currQVec_; // condensed state vector [psiL, beta]
	RoboticsMath::Vector6d qHome_; // home configuration (joint space [psiL, beta])
	medlab::InterpRet currInterpolatedBackbone_; // interpolated cannula
	int lastPosInterp_;
	medlab::KinOut currKinematics_;  // kinout from kinematics call, transformed into base frame & interpolated

};
