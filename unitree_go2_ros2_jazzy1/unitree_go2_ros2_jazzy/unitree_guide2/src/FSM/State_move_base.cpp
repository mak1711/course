/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_MOVE_BASE

#include "FSM/State_move_base.h"
#include <algorithm>

// /cmd_vel came straight from a Twist message into the gait controller with no
// clamping at all -- so anything upstream (Nav2 misconfigured to an untested speed,
// a stray teleop command, a future caller) could command a speed nobody ever verified
// keeps the robot upright and it would just be obeyed. Confirmed the hard way: Nav2's
// Spin behavior briefly used an untested 0.6 rad/s (double the actually-tested-safe
// 0.3 rad/s ceiling -- see go2_navigation's nav2_params_junior.yaml) and the robot
// fell. That's fixed at the Nav2 config layer, but this is the last line of defense
// closest to the actual hardware/sim, so it gets its own hard clamp too.
static constexpr double kMaxLinearVel = 0.35;
static constexpr double kMaxAngularVel = 0.3;

State_move_base::State_move_base(CtrlComponents *ctrlComp)
    :State_Trotting(ctrlComp){
    _stateName = FSMStateName::MOVE_BASE;
    _stateNameString = "move_base";
    initRecv();
    
}

FSMStateName State_move_base::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::MOVE_BASE;
    }
}

void State_move_base::getUserCmd(){
    setHighCmd(_vx, _vy, _wz);
    ros::spinOnce();
}

void State_move_base::twistCallback(const geometry_msgs::Twist& msg){
    _vx = std::clamp(msg.linear.x, -kMaxLinearVel, kMaxLinearVel);
    _vy = std::clamp(msg.linear.y, -kMaxLinearVel, kMaxLinearVel);
    _wz = std::clamp(msg.angular.z, -kMaxAngularVel, kMaxAngularVel);
}

void State_move_base::initRecv(){
    _cmdSub = _nm.subscribe("/cmd_vel", 1, &State_move_base::twistCallback, this);
}

#endif  // COMPILE_WITH_MOVE_BASE

#ifdef COMPILE_WITH_ROS2_MB

#include "FSM/State_move_base.h"
#include <algorithm>

// See the COMPILE_WITH_MOVE_BASE block above for why these clamps exist: /cmd_vel had
// no velocity limiting at all before this, and an untested 0.6 rad/s Nav2 rotation
// command made the robot fall.
static constexpr double kMaxLinearVel = 0.35;
static constexpr double kMaxAngularVel = 0.3;

State_move_base::State_move_base(CtrlComponents *ctrlComp)
    :State_Trotting(ctrlComp){
    _stateName = FSMStateName::MOVE_BASE;
    _stateNameString = "move_base";
    _nm = rclcpp::Node::make_shared("state_mb");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 1
    );
    executor->add_node(_nm);
    executor_thread = std::thread([executor] (){
        executor->spin();
    });
    executor_thread.detach();
    initRecv();
}

FSMStateName State_move_base::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::MOVE_BASE;
    }
}

void State_move_base::getUserCmd(){
    setHighCmd(_vx, _vy, _wz);
}

void State_move_base::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    _vx = std::clamp(msg->linear.x, -kMaxLinearVel, kMaxLinearVel);
    _vy = std::clamp(msg->linear.y, -kMaxLinearVel, kMaxLinearVel);
    _wz = std::clamp(msg->angular.z, -kMaxAngularVel, kMaxAngularVel);
}

void State_move_base::initRecv(){
    std::cout << "Initialized cmd vel sub" << std::endl;
    _cmdSub = _nm->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&State_move_base::twistCallback, this, std::placeholders::_1));
}

#endif  // COMPILE_WITH_ROS2_MB