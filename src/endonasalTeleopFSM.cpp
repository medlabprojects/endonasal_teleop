#include "endonasalTeleopFSM.h"

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

bool transitionToInitState()
{

}

bool loopInitState()
{

}

TeleopState transitionFromInitState()
{

}

bool transitionToIdleState()
{

}

bool loopIdleState()
{

}

TeleopState transitionFromIdleState()
{

}

bool transitionToActiveState()
{

}

bool loopActiveState()
{

}

TeleopState transitionFromActiveState()
{

}

bool transitionToSimulationState()
{

}

bool loopSimulationState()
{

}

TeleopState transitionFromSimulationState()
{

}

