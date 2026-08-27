/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"

class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    float _targetPos[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                            0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
    float _startPos[12];
    // 1000 steps (2s @ 500Hz) rises too fast for the real Go2's mass/inertia and
    // tends to overshoot into a tip before the stance gains can arrest it; slowing
    // the ramp cuts the dynamic loading during the transition.
    float _duration = 3000;   //steps, at the nominal (not guaranteed) loop rate
    float _percent = 0;       //%

    // _percent used to advance by a fixed 1/_duration every call to run(), which
    // assumes run() is actually called at the nominal rate. setProcessScheduler()
    // has no real-time priority here, so the loop's real period jitters under CPU
    // load (see the "waitTime ... is not enough" warnings) -- with a fixed-per-call
    // ramp that jitter makes the commanded position jump unevenly, and since dq is
    // always commanded as 0 (no feedforward velocity), every uneven jump reads as a
    // sudden position error and produces a torque spike. Driving _percent off wall
    // clock time and commanding the ramp's actual instantaneous velocity as dq fixes
    // both: the trajectory itself no longer depends on how often run() gets called.
    double _startTime = 0;
    double _durationSec = 0;

    int _debugCount = 0;
};

#endif  // FIXEDSTAND_H