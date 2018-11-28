#include "PhantomOmniRos.h"
#include <QObject>
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <endonasal_teleop/OmniState.h>
#include <std_msgs/Float32.h>
#include <Mtransform.h>
#include "RoboticsMath.h"


using namespace Eigen;

PhantomOmniRos::PhantomOmniRos(std::string nodeName):
  nodeName_(nodeName),
  scaleFactor_(0.30),
  pose_(Eigen::Matrix4d::Zero()),
  posePrev_(Eigen::Matrix4d::Zero())
{
  qRegisterMetaType< RoboticsMath::Vector6d >();
}

bool PhantomOmniRos::init()
{
  // create publisher
  std::string topicForceScaling = "/" + nodeName_+ "/force_scaling";
  pubSetForceScaling_ = nh_.advertise<std_msgs::Float32>(topicForceScaling.c_str(),1);

  // create subscriber
  std::string topicState = "/" + nodeName_ + "/state";
  subState_ = nh_.subscribe(topicState.c_str(), 1, &PhantomOmniRos::omniCallback, this);
}


void PhantomOmniRos::omniCallback(const endonasal_teleop::OmniState &msg)
{
  // copy transform from message and remap as a Matrix4f
  float omniTransform[16];
  std::copy(std::begin(msg.transform), std::end(msg.transform), std::begin(omniTransform));
  Eigen::Map< Eigen::Matrix<float,4,4, ColMajor> > poseTemp(omniTransform);
  pose_ = poseTemp.cast<double>();

  std::cout << "pose_ = " << pose_ << std::endl;

  // update button states
  bool buttonState1Prev = buttonState1_;
  bool buttonState2Prev = buttonState2_;
  buttonState1_ = msg.button1;
  buttonState2_ = msg.button2;

  // check if either button was just pressed
  if(buttonState1_ != buttonState1Prev){
    emit(omniButtonStateChanged(1, buttonState1_));
  }
  if(buttonState2_ != buttonState2Prev){
    emit(omniButtonStateChanged(2, buttonState2_));
  }

}

RoboticsMath::Vector6d PhantomOmniRos::twistUpdate()
{
  // This uses pose_ and posePrev_ to update desTwist

  // compute change
  Eigen::Matrix4d omniDeltaOmniPenCoords = Mtransform::Inverse(posePrev_)*pose_;

  // convert [mm] to [m] and scale down by omniScaleFactor
  omniDeltaOmniPenCoords.block(0,3,3,1) = omniDeltaOmniPenCoords/1.0E3;
  omniDeltaOmniPenCoords.block(0,3,3,1) = scaleFactor_*omniDeltaOmniPenCoords.block(0,3,3,1);

  // ????
  Eigen::Matrix4d RposePrev_ = RoboticsMath::assembleTransformation(posePrev_.block(0,0,3,3), Eigen::Vector3d::Zero());
  Eigen::Matrix4d omniDeltaOmniBaseCoords = (Mtransform::Inverse(RposePrev_.transpose())*omniDeltaOmniPenCoords*RposePrev_.transpose());

  Eigen::Matrix3d Rd = pose_.block<3,3>(0,0);
  Eigen::Matrix3d Rc = posePrev_.block<3,3>(0,0);
  Eigen::Matrix3d Re = Rd*Rc.transpose();
  Re = RoboticsMath::orthonormalize(Re);

  // desTwist should use OmniBaseCoords for linar velocity, and OmniPenCoords for angular velocity
  RoboticsMath::Vector6d desTwist;
  desTwist(0) = omniDeltaOmniBaseCoords(0,3); // vx
  desTwist(1) = omniDeltaOmniBaseCoords(1,3); // vy
  desTwist(2) = omniDeltaOmniBaseCoords(2,3); // vz
  desTwist(3) = omniDeltaOmniPenCoords(2,1);  // wx
  desTwist(4) = omniDeltaOmniPenCoords(0,2);  // wy
  desTwist(5) = omniDeltaOmniPenCoords(1,0);  // wz

//  RoboticsMath::Vector6d omniTwistOmniPenCoords;
//  omniTwistOmniPenCoords(0) = omniDeltaOmniPenCoords(0,3); // vx
//  omniTwistOmniPenCoords(1) = omniDeltaOmniPenCoords(1,3); // vy
//  omniTwistOmniPenCoords(2) = omniDeltaOmniPenCoords(2,3); // vz
//  omniTwistOmniPenCoords(3) = omniDeltaOmniPenCoords(2,1); // wx
//  omniTwistOmniPenCoords(4) = omniDeltaOmniPenCoords(0,2); // wy
//  omniTwistOmniPenCoords(5) = omniDeltaOmniPenCoords(1,0); // wz

//  RoboticsMath::Vector6d omniTwistOmniBaseCoords;
//  omniTwistOmniBaseCoords(0) = omniDeltaOmniBaseCoords(0,3);
//  omniTwistOmniBaseCoords(1) = omniDeltaOmniBaseCoords(1,3);
//  omniTwistOmniBaseCoords(2) = omniDeltaOmniBaseCoords(2,3);
//  omniTwistOmniBaseCoords(3) = omniDeltaOmniBaseCoords(2,1);
//  omniTwistOmniBaseCoords(4) = omniDeltaOmniBaseCoords(0,2);
//  omniTwistOmniBaseCoords(5) = omniDeltaOmniBaseCoords(1,0);

//  RoboticsMath::Vector6d desTwist;
//  desTwist.head(3) = omniTwistOmniBaseCoords.head(3);
//  desTwist.tail(3) = omniTwistOmniPenCoords.tail(3);

  // update posePrev_
  posePrev_ = pose_;

  emit newTwist(desTwist);
  return desTwist;
}
