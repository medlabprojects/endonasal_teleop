#pragma once
#include "CTR3Robot.h"
#include "RoboticsMath.h"

class ResolvedRatesController {

public:

	enum class LIMIT_FLAG
	{
		NONE, VELOCITY_T1_ROT, VELOCITY_T1_TRANS  //TODO: add the rest & implement
	};

	ResolvedRatesController();  // setup
	~ResolvedRatesController();
	void init(std::string Name);		// go online
	RoboticsMath::Matrix6d step(); // output of this should be joint positions [alpha, beta]
	//bool SetGains(double ScaleFactor, double LambdaTracking, double LambdaDamping, double LambdaJL);
	bool SetTrackingGain(double LambdaTracking);
	bool SetDampingGain(double LambdaDamping);
	bool SetJointLimitsGain(double LambdaJL);
	//bool SetInputDeviceTransform(Matrix4d TRegistration); //TODO: all preprocessing of input device twist should be done outside of this class

	//std::vector<ResolvedRatesController::LIMIT_FLAG> GetLimitFlags(); //TODO: implement this

private:

	RoboticsMath::Matrix6d JCur_{};
	RoboticsMath::Matrix6d WTracking_{};
	RoboticsMath::Matrix6d WDamping_{};
	RoboticsMath::Matrix6d WJointLims_{};
	medlab::InterpRet InterpolatedBackboneCur_;
	std::vector<ResolvedRatesController::LIMIT_FLAG> limitFlags_{};
	double rosLoopRate_{};

	RoboticsMath::Vector6d saturateJointVelocities(RoboticsMath::Vector6d delta_qx, int node_freq);
	RoboticsMath::Vector6d transformBetaToX(RoboticsMath::Vector6d qbeta, Eigen::Vector3d L);
	RoboticsMath::Vector6d transformXToBeta(RoboticsMath::Vector6d qx, Eigen::Vector3d L);
	double dhFunction(double xmin, double xmax, double x);
	medlab::WeightingRet computeWeightingMatrix(Eigen::Vector3d x, Eigen::Vector3d dhPrev, Eigen::Vector3d L, double lambda);
	Eigen::Vector3d limitBetaValsSimple(Eigen::Vector3d x_in, Eigen::Vector3d L);
	Eigen::Vector3d limitBetaValsBimanualAlgorithm(Eigen::Vector3d Beta_in, Eigen::Vector3d L_in);
	RoboticsMath::Vector6d scaleInputDeviceVelocity(RoboticsMath::Vector6d desTwistDelta);

};
