#include "PhantomOmniRos.h"
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include "RoboticsMath.h"


PhantomOmniRos::PhantomOmniRos():
  scaleFactor_(0.30)
{

}


void PhantomOmniRos::omniCallback(const geometry_msgs::Pose &msg)
{
  // create rotation matrix from quaternion
  Eigen::Matrix3d R = Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z).toRotationMatrix();

  // assemble transformation matrix
  pose_ = RoboticsMath::assembleTransformation(R, Eigen::Vector3d(msg.x, msg.y, msg.z));
}
