/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_FixedStand.h"
#include "common/timeMarker.h"

State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"){}

void State_FixedStand::enter(){
    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }
    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
    }
    _ctrlComp->setAllStance();

    _percent = 0;
    _startTime = getTimeSecond();
    _durationSec = _duration * _ctrlComp->dt;
    _debugCount = 0;

    std::cout << "[FixedStand] enter: ramping over " << _durationSec << "s" << std::endl;
}

void State_FixedStand::run(){
    double elapsed = getTimeSecond() - _startTime;
    _percent = (float)(elapsed / _durationSec);
    _percent = _percent > 1 ? 1 : _percent;

    for(int j=0; j<12; j++){
        _lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos[j];
        // Feedforward the ramp's own velocity instead of always commanding 0: the
        // joint PD's damping term then resists deviation from the intended motion,
        // not the motion itself, which is what was producing torque spikes whenever
        // the (jittery, non-realtime) loop delivered an uneven position jump.
        _lowCmd->motorCmd[j].dq = _percent < 1 ?
            (_targetPos[j] - _startPos[j]) / (float)_durationSec : 0.0f;
    }

    // Debug: print tilt (1.0 = level, <0.5 = the 60 deg safety-trip threshold in
    // FSM::checkSafty) and FR_hip tracking error a few times a second so you can
    // see, in this terminal, whether a "jump" coincides with a big position error
    // (gains/torque issue) or shows up with tilt still near 1.0 (a timing/contact
    // issue rather than a standing-balance one).
    if(++_debugCount % 25 == 0){
        double tilt = _lowState->getRotMat()(2,2);
        double qErr = _lowCmd->motorCmd[0].q - _lowState->motorState[0].q;
        std::cout << "[FixedStand] t=" << elapsed << "s percent=" << _percent
                  << " tilt=" << tilt
                  << " FR_hip cmd_q=" << _lowCmd->motorCmd[0].q
                  << " actual_q=" << _lowState->motorState[0].q
                  << " err=" << qErr << std::endl;
    }
}

void State_FixedStand::exit(){
    _percent = 0;
}

FSMStateName State_FixedStand::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_X){
        return FSMStateName::FREESTAND;
    }
    else if(_lowState->userCmd == UserCommand::START){
        return FSMStateName::TROTTING;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        return FSMStateName::BALANCETEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_A){
        return FSMStateName::SWINGTEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_Y){
        return FSMStateName::STEPTEST;
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if(_lowState->userCmd == UserCommand::L2_Y){
        return FSMStateName::MOVE_BASE;
    }
#endif  // COMPILE_WITH_MOVE_BASE

#ifdef COMPILE_WITH_ROS2_MB
    else if(_lowState->userCmd == UserCommand::L2_Y){
        return FSMStateName::MOVE_BASE;
    }
#endif  // COMPILE_WITH_ROS2_MB
    else{
        return FSMStateName::FIXEDSTAND;
    }
}