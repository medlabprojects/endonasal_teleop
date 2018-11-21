#pragma once

#include <QVector>
#include "ResolvedRatesController.h"

// Finite State Machine
enum class TeleopState {INIT, IDLE, ACTIVE, SIMULATION, ERROR};
TeleopState stepStateMachine(TeleopState nextState);
TeleopState runInit(void);
TeleopState runIdle(void);
TeleopState runActive(void);
TeleopState runSimulation(void);
TeleopState runError(void);

bool transitionToInitState(void);
bool loopInitState(void);
TeleopState transitionFromInitState(void);

bool transitionToIdleState(void);
bool loopIdleState(void);
TeleopState transitionFromIdleState(void);

bool transitionToActiveState(void);
bool loopActiveState(void);
TeleopState transitionFromActiveState(void);

bool transitionToSimulationState(void);
bool loopSimulationState(void);
TeleopState transitionFromSimulationState(void);


// ROS


// Resolved Rates Controllers
QVector<ResolvedRatesController> controllers;

// Callbacks
