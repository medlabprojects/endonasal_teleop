#pragma once

#ifndef PHANTOMOMNIROS_H
#define PHANTOMOMNIROS_H

#include <Eigen/Dense>

class PhantomOmniRos
{
public:
  PhantomOmniRos();

  void omniCallback(const geometry_msgs::Pose &msg);
  void setScaleFactor(double scale) {scaleFactor_ = scale;}
  double getScaleFactor(void) {return scaleFactor_;}

private:
  Eigen::Matrix4d pose_;  // current Omni pose
  Eigen::Matrix4d Tregs_; // mapping from Omni frame to desired frame
  Eigen::Matrix4d OmniDeltaOmniCoords_;
  Eigen::Matrix4d prevOmniInv_;
  Eigen::Matrix4d omniDeltaCannulaCoords_;
  Eigen::Matrix4d omniFrameAtClutch_;
  double scaleFactor_; // = 0.30;
  bool buttonState1_;
  bool buttonState2_;
};

#endif // PHANTOMOMNIROS_H
