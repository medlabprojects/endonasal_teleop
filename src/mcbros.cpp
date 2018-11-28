#include "mcbros.h"
#include <QObject>
#include <QTimer>
#include <stdint.h>
#include <QString>
#include <QVector>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include "medlab_motor_control_board/McbEncoders.h"
#include "medlab_motor_control_board/McbEncoderCurrent.h"
#include "medlab_motor_control_board/McbStatus.h"
#include "medlab_motor_control_board/EnableMotor.h"
#include "medlab_motor_control_board/McbGains.h"


McbRos::McbRos()
  : connected_(false)
{
  // register data types used in signals/slots
  qRegisterMetaType< QVector<bool> >();
  qRegisterMetaType< medlab_motor_control_board::McbEncoderCurrent >();
}

McbRos::~McbRos()
{
  // shutdown node
  nh_.shutdown();
}

bool McbRos::init(std::string nodeName)
{
  nodeName_ = nodeName;

  // setup pubEnableRos_
  std::string topicEnableRosControl = "/" + nodeName_+ "/enable_ros_control";
  pubEnableRos_ = nh_.advertise<std_msgs::Bool>(topicEnableRosControl.c_str(),1);

  // setup pubEnableAllMotors_
  std::string topicEnableAllMotors = "/" + nodeName_+ "/enable_all_motors";
  pubEnableAllMotors_ = nh_.advertise<std_msgs::Bool>(topicEnableAllMotors.c_str(),1);

  // setup pubEnableMotor_
  std::string topicEnableMotor = "/" + nodeName_+ "/enable_motor";
  pubEnableMotor_ = nh_.advertise<medlab_motor_control_board::EnableMotor>(topicEnableMotor.c_str(),1);

  // setup pubGetStatus_
  std::string topicGetStatus = "/" + nodeName_+ "/get_status";
  pubGetStatus_ = nh_.advertise<std_msgs::Empty>(topicGetStatus.c_str(),1);

  // setup pubZeroSingle_
  std::string topicZeroSingle = "/" + nodeName_+ "/encoder_zero_single";
  pubZeroSingle_ = nh_.advertise<std_msgs::UInt8>(topicZeroSingle.c_str(),1);

  // setup pubZeroAll_
  std::string topicZeroAll = "/" + nodeName_+ "/encoder_zero_all";
  pubZeroAll_ = nh_.advertise<std_msgs::Empty>(topicZeroAll.c_str(),1);

  // setup pubEncoderCommand_
  std::string topicEncoderCommand = "/" + nodeName_+ "/encoder_command";
  pubEncoderCommand_ = nh_.advertise<medlab_motor_control_board::McbEncoders>(topicEncoderCommand.c_str(),1);

  // setup pubSetGains_
  std::string topicSetGains = "/" + nodeName_+ "/set_gains";
  pubSetGains_ = nh_.advertise<medlab_motor_control_board::McbGains>(topicSetGains.c_str(),1);

  // setup subStatus_
  std::string topicStatus = "/" + nodeName + "/status";
  subStatus_ = nh_.subscribe(topicStatus.c_str(), 1, &McbRos::callbackSubStatus, this);

  // setup subEncoderCurrent_
  std::string topicEncoderCurrent = "/" + nodeName_+ "/encoder_current";
  subEncoderCurrent_ = nh_.subscribe(topicEncoderCurrent.c_str(), 1, &McbRos::callbackSubEncoderCurrent, this);

  // setup subLimitSwitchEvent_
  std::string topicLimitSwitchEvent = "/" + nodeName_+ "/limit_switch_event";
  subLimitSwitchEvent_ = nh_.subscribe(topicLimitSwitchEvent.c_str(), 1, &McbRos::callbackSubLimitSwitchEvent, this);
}

bool McbRos::enableRosControl(bool cmd)
{
  std_msgs::Bool msg;
  msg.data = cmd;
  pubEnableRos_.publish(msg);
}

bool McbRos::enableAllMotors(bool cmd)
{
  std_msgs::Bool msg;
  msg.data = cmd;
  pubEnableAllMotors_.publish(msg);
}

bool McbRos::enableMotor(uint8_t motor, bool cmd)
{
  bool success = false;

  if(connected_ && (motor<getNumMotors()) && (motor>-1)){
    medlab_motor_control_board::EnableMotor msg;
    msg.motor = motor;
    msg.enable = cmd;
    pubEnableMotor_.publish(msg);

    success = true;
  }

  return success;
}

bool McbRos::requestStatus()
{
  bool success = false;

  if(connected_){
    // send request
    std_msgs::Empty msg;
    pubGetStatus_.publish(msg);

    success = true;
  }

  return success;
}

bool McbRos::setDesiredPosition(int motor, int32_t position)
{
  bool success = false;

  if(!connected_){
    return success;
  }

  if((motor<getNumMotors()) && (motor>-1)){
    // assemble message
    medlab_motor_control_board::McbEncoders msg;
    msg.count = encoderCurrent_.desired; // ensure we keep others the same
    msg.count[motor] = position;

    // publish
    setDesiredPosition(msg);

    success = true;
  }
  return success;
}

QVector<float> McbRos::getEfforts(void)
{
  QVector<float> efforts;
  for(int ii=0; ii<6; ii++){
    efforts.push_back(currentStatus_.control_effort[ii]);
  }

  return efforts;
}

bool McbRos::setDesiredPosition(medlab_motor_control_board::McbEncoders desiredPositions)
{
  pubEncoderCommand_.publish(desiredPositions);
}

bool McbRos::zeroCurrentPosition(uint8_t motor)
{
  bool success = false;

  if((motor<getNumMotors()) && (motor>-1)){
    std_msgs::UInt8 msg;
    msg.data = motor;
    pubZeroSingle_.publish(msg);

    success = true;
  }

  return success;
}

bool McbRos::zeroCurrentPositions(void)
{
  bool success = false;

  if(connected_ && isRosControlEnabled()){
    std_msgs::Empty msg;
    pubZeroAll_.publish(msg);

    success = true;
  }

  return success;
}

bool McbRos::setGains(quint8 motor, double p, double i, double d)
{
  bool success = false;

  if(connected_ && (motor < getNumMotors())){
    medlab_motor_control_board::McbGains msg;

    msg.motor = motor;
    msg.p = p;
    msg.i = i;
    msg.d = d;

    pubSetGains_.publish(msg);

    success = true;
  }

  return success;
}

double McbRos::getP(uint8_t motor)
{
  double p = 0.0;

  if(connected_ && (motor<getNumMotors())){
    p = currentStatus_.p[motor];
  }

  return p;
}

double McbRos::getI(uint8_t motor)
{
  double i = 0.0;

  if(connected_ && (motor<getNumMotors())){
    i = currentStatus_.i[motor];
  }

  return i;
}

double McbRos::getD(uint8_t motor)
{
  double d = 0.0;

  if(connected_ && (motor<getNumMotors())){
    d = currentStatus_.d[motor];
  }

  return d;
}

int McbRos::getNumMotors(void)
{
  int numMotors = 0;

  if(connected_){
    numMotors = currentStatus_.number_modules;
  }

  return numMotors;
}

QString McbRos::getIp(void)
{
  QString ip = "Not Connected";

  if(connected_){
    // convert IP address to a string
    ip =  QString::number(currentStatus_.ip[0]) + "."
        + QString::number(currentStatus_.ip[1]) + "."
        + QString::number(currentStatus_.ip[2]) + "."
        + QString::number(currentStatus_.ip[3]);
  }

  return ip;
}

QString McbRos::getMac(void)
{
  QString mac = "Not Connected";

  if(connected_){
    // convert IP address to a hex string
    mac = QString::number(currentStatus_.mac[0],16).toUpper().rightJustified(2,'0') + ":"
        + QString::number(currentStatus_.mac[1],16).toUpper().rightJustified(2,'0') + ":"
        + QString::number(currentStatus_.mac[2],16).toUpper().rightJustified(2,'0') + ":"
        + QString::number(currentStatus_.mac[3],16).toUpper().rightJustified(2,'0') + ":"
        + QString::number(currentStatus_.mac[4],16).toUpper().rightJustified(2,'0') + ":"
        + QString::number(currentStatus_.mac[5],16).toUpper().rightJustified(2,'0');
  }

  return mac;
}

bool McbRos::isRosControlEnabled()
{
  return currentControlState_;
}

bool McbRos::isMotorEnabled(uint8_t motor)
{
  if(isRosControlEnabled() && (motor<6)){
    return currentStatus_.motor_enabled[motor];
  }
  else{
    return false; // FIX: should probably return -1
  }
}

void McbRos::callbackSubEncoderCurrent(const medlab_motor_control_board::McbEncoderCurrent::ConstPtr& msg)
{
  // store measured/desired encoder positions
  encoderCurrent_ = *msg;

  emit newPositions(encoderCurrent_);
}

void McbRos::callbackSubStatus(const medlab_motor_control_board::McbStatus::ConstPtr &msg)
{
  // emit signal if we were previously disconnected
  if(!connected_){
    connected_ = true;
    emit connectionEstablished();
  }

  // save previous status and control state
  medlab_motor_control_board::McbStatus previousStatus = currentStatus_;
  bool previousControlState = currentControlState_;

  // update status and control state
  currentStatus_ = *msg;
  std::string currentStateString = currentStatus_.current_state;
  currentControlState_ = !currentStateString.compare("ROS Control"); // compare() returns 0 if strings are equal

  // emit signal if control state has changed
  if(currentControlState_ != previousControlState){
    emit controlStateChanged(currentControlState_);
  }

  // emit signal(s) if any motor states have changed
  for(int ii=0; ii<getNumMotors(); ii++){
    if(currentStatus_.motor_enabled[ii] != previousStatus.motor_enabled[ii]){
      emit motorStateChanged(ii);
    }
  }

  emit newStatus();
}

void McbRos::callbackSubLimitSwitchEvent(const medlab_motor_control_board::EnableMotor::ConstPtr &msg)
{
  emit limitSwitchEvent(msg->motor, msg->enable);
}
