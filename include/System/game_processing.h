#pragma once
#include "System/struct.h"
#include "System/temp.h"
#include "System/accel_limited_control.h"
#include "System/define.h"

/// @brief halt中は出力0
/// @param output
void halt(RobotOutput *output)
{
    RobotOutput zero = {0};
    *output = zero;
}
/// @brief stop中は最大速度減
/// @param strategy_pc_command
/// @param world
/// @param output
void stopGame(const StrategyPcCommand *strategy_pc_command, const World *world, RobotController *robot_controller, RobotOutput *output)
{
    RobotOutput zero = {0};
    *output = zero;
    robot_controller->max_robot_vel = MAX_ROBOT_SPEED_GAME_STOP;
    // 目標値に向けて走行
    goTargetPose(&(strategy_pc_command->robot_goal_pose), &(world->robot_pose), robot_controller, &(output->velocity));
}

/// @brief 目標出力計算
/// @param game_state
/// @param strategy_pc_command
/// @param world 座標
/// @param output 目標出力
void calcOutput(const int *game_state, const StrategyPcCommand *strategy_pc_command, const World *world, RobotController *robot_controller, RobotOutput *output)
{
    int game_state_ = *game_state;
    if (checkRobotError())
    {
        game_state_ = true;
    }
    switch (game_state_)
    {
    case GAME_STATE_STOP:
        stopGame(strategy_pc_command, world, robot_controller, output);
        break;
    case GAME_STATE_INGAME:
        break;
    case GAME_STATE_OUR_BALL_PLACEMENT:
        break;
    case GAME_STATE_THEIR_BALL_PLACEMENT:
        break;
    case GAME_STATE_HALT:
        halt(output);
        break;
    default:
        halt(output);
        // printf("[ERROR] game state is invalid")
        break;
    }
};
