#pragma once

#ifndef PHANTOMOMNIROS_H
#define PHANTOMOMNIROS_H

#include <QObject>
#include <Eigen/Dense>
#include "geometry_msgs/Pose.h"
#include "endonasal_teleop/OmniState.h"
#include "RoboticsMath.h"
#include <ros/ros.h>


Q_DECLARE_METATYPE(RoboticsMath::Vector6d)

class PhantomOmniRos  : public QObject
{
  Q_OBJECT

public:
  PhantomOmniRos(std::string nodeName);
  bool init(void);
  void setScaleFactor(double scale) {scaleFactor_ = scale;}
  double getScaleFactor(void) {return scaleFactor_;}

public slots:
  RoboticsMath::Vector6d twistUpdate(void); // creates a desired twist based on pose_ and posePrev_
  void omniCallback(const endonasal_teleop::OmniState &msg);
  bool resetTwist(void) {posePrev_ = pose_;}

signals:
  void omniButtonStateChanged(int button, bool currentState);
  void newTwist(RoboticsMath::Vector6d twist);

private:
  std::string nodeName_;
  ros::NodeHandle nh_;
  bool connected_ = false; // true when connected to ROS
  ros::Publisher pubSetForceScaling_; // sets haptic damping force in [Newtons per m/s]
  ros::Subscriber subState_; // current Omni state
  Eigen::Matrix4d pose_;  // current Omni pose
  Eigen::Matrix4d posePrev_; // pose_ when twistUpdate() was last called
  double scaleFactor_; // = 0.30;
  bool buttonState1_ = false;
  bool buttonState2_ = false;

};

#endif // PHANTOMOMNIROS_H
