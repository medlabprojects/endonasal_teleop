#pragma once
#include <QObject>
#include "CTR3Robot.h"
#include "RoboticsMath.h"
#include "PhantomOmniRos.h"
#include <vector>

class ResolvedRatesController : public QObject
{
  Q_OBJECT

public:

  enum class LIMIT_FLAG
  {
    VELOCITY_T1_ROT, VELOCITY_T1_TRANS,
    VELOCITY_T2_ROT, VELOCITY_T2_TRANS,
    VELOCITY_T3_ROT, VELOCITY_T3_TRANS,
    T1_BACK, T1_FRONT,
    T2_BACK, T2_FRONT,
    T3_BACK, T3_FRONT //TODO: implement these
  };

  ResolvedRatesController(medlab::Cannula3 cannula,medlab::CTR3RobotParams params,
                          RoboticsMath::Vector6d qHome, Eigen::Matrix4d baseFrame);  // setup
  ~ResolvedRatesController();
  void init();
  bool SetTrackingGain(double LambdaTracking);
  bool SetDampingGain(double LambdaDamping);
  bool SetJointLimitsGain(double LambdaJL);
  std::vector<ResolvedRatesController::LIMIT_FLAG> GetLimitFlags(){return currentLimitFlags_;} //TODO: implement this
  CTR3Robot GetRobot();
public slots:
  RoboticsMath::Vector6d step(RoboticsMath::Vector6d desTwist); // online loop

private:
  CTR3Robot robot_;
  std::vector<ResolvedRatesController::LIMIT_FLAG> currentLimitFlags_;
  RoboticsMath::Matrix6d JCur_;
  double lambdaTracking_;
  Eigen::Matrix4d WTracking_;
  double lambdaDamping_;
  RoboticsMath::Matrix6d WDamping_;
  double lambdaJL_;
  Eigen::Vector3d dhPrev_;
  RoboticsMath::Matrix6d WJointLims_;
  RoboticsMath::Matrix6d WStability_;
  medlab::InterpRet InterpolatedBackboneCur_;
  double rosLoopRate_;
  bool instabilityAvoidance_; // using instability avoidance?

  double vMax_;
  double vMin_;
  double eMax_;
  double eMin_;
  double convergeRadius_;


  RoboticsMath::Vector6d saturateJointVelocities(RoboticsMath::Vector6d delta_qx, int node_freq);
  RoboticsMath::Vector6d transformBetaToX(RoboticsMath::Vector6d qbeta, Eigen::Vector3d L);
  RoboticsMath::Vector6d transformXToBeta(RoboticsMath::Vector6d qx, Eigen::Vector3d L);
  double dhFunction(double xmin, double xmax, double x);
  medlab::WeightingRet computeJLWeightingMatrix(Eigen::Vector3d x, Eigen::Vector3d dhPrev, Eigen::Vector3d L);
  Eigen::Vector3d limitBetaValsSimple(Eigen::Vector3d x_in, Eigen::Vector3d L);
  Eigen::Vector3d limitBetaValsBimanualAlgorithm(Eigen::Vector3d Beta_in, Eigen::Vector3d L_in);
  RoboticsMath::Vector6d scaleInputDeviceVelocity(RoboticsMath::Vector6d desTwistDelta);
  double computeVMag(Eigen::Vector3d e);

};
