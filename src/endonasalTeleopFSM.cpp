#include "endonasalTeleopFSM.h"
#include <endonasal_teleop/matrix8.h>
#include <endonasal_teleop/OmniState.h>
#include "mcbros.h"
#include "PhantomOmniRos.h"

#include <QMap>

// --> INIT state
//    --> at the end, segue to IDLE
// --> IDLE State
//    --> listen for event triggers to SIM or ACTIVE
//    --> user can change RR gains
// --> ACTIVE State
//    --> enable motors and listen to input devices
//    --> listen for trigger back to IDLE
// --> SIM State
//    --> disable motors and listen to input devices
//    --> listen for trigger back to IDLE
//

// Parameter Namespace
std::string paramNamespace = "EndonasalRobot";

// Resolved Rates Controllers
QVector<ResolvedRatesController *> controllers;

// Omni devices
QVector<PhantomOmniRos *> omnis;

// Motorboards
QVector<McbRos *> mcb;

// Visualization
ros::Publisher needle_pub;

TeleopState stepStateMachine(TeleopState nextState)
{
  switch (nextState)
  {
  case TeleopState::INIT:
    return runInit();

  case TeleopState::IDLE:
    return runIdle();

  case TeleopState::ACTIVE:
    return runActive();

  case TeleopState::SIMULATION:
    return runSimulation();

  default:
    return runError();

  }
}

TeleopState runInit()
{
  transitionToInitState();
  while(loopInitState()){};
  return transitionFromInitState();
}
TeleopState runIdle()
{
  transitionToIdleState();
  while(loopIdleState()){};
  return transitionFromIdleState();
}
TeleopState runActive()
{
  transitionToActiveState();
  while(loopActiveState()){};
  return transitionFromActiveState();
}
TeleopState runSimulation()
{
  transitionToSimulationState();
  while(loopSimulationState()){};
  return transitionFromSimulationState();
}
TeleopState runError()
{
  // alert user
  // turn off motors
  // shutdown ROS stuff

  while(1);
}

//// INIT
/// ---------------------------------------------------------------
bool transitionToInitState()
{
  // Create node
  ros::NodeHandle node;

  // PUBLISHERS
  ros::Publisher needle_pub = node.advertise<endonasal_teleop::matrix8>("needle_position",10);

  ///////////////////////
  // Query param server
  ///////////////////////


  // Init GUI/Rviz

  // Init RR controllers

}
bool loopInitState()
{
  bool allDevicesOnline = false;

  // Check for online devices
  while(!allDevicesOnline)
  {

  }
}
TeleopState transitionFromInitState()
{
  // Next state is IDLE
}

////IDLE
/// ---------------------------------------------------------------
bool transitionToIdleState()
{
  // Connect/Map User Input Devices
}
bool loopIdleState()
{
  // Process User Configuration
}
TeleopState transitionFromIdleState()
{
  // Disconnect user gui elements?
}

//// ACTIVE
/// ---------------------------------------------------------------
bool transitionToActiveState()
{
  ///////////////////////
  // Motor Control Boards
  ///////////////////////

  // determine number of MCBs
  std::string paramNumBoards = paramNamespace + "/MCB/NumberOfBoards";
  int numMcbs;
  ros::param::get(paramNumBoards.c_str(), numMcbs);

  for(int ii=0; ii<numMcbs; ii++){
    // get MCB node name
    std::string paramMcbName = paramNamespace + "/MCB/" + std::to_string(ii) + "/NodeName";
    std::string mcbName;
    ros::param::get(paramMcbName, mcbName);

    // create and initialize MCB
    McbRos* mcbTemp = new McbRos();
    mcb.push_back(mcbTemp);
    mcb[ii]->init(mcbName);
  }




  // Connect Input Devices to RR controllers
  // connect(*PhantomOmniRosObject*, SIGNAL(newTwist(RoboticsMath::Vector6d), this, SLOT(ResolvedRatesController::step(Roboticsmath::Vector6d desTwist))

  // MCB --> ROS Control State
}
bool loopActiveState()
{
  while(ros::ok()){
    // compute desired twist
    for(int ii=0; ii<omnis.size(); ii++){
      omnis[ii].twistUpdate();
    }


    // desiredJoints = RR.step(desTwist);
    // assemble motor messages


    ros::spinOnce();
  }


}
TeleopState transitionFromActiveState()
{
  // Disconnect controller?
}

/// SIMULATION
/// ---------------------------------------------------------------
bool transitionToSimulationState()
{
  // Connect Input Devices to RR controllers
  // MCB --> Ros Idle State
}
bool loopSimulationState()
{
  // If Ros::okay() spinOnce;
  // RR.step(desTwist);
}
TeleopState transitionFromSimulationState()
{
  // Disconnect controller?
}

