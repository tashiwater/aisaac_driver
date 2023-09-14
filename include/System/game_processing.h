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
    robot_controller->max_robot_vel = MAX_ROBOT_SPEED_GAME_STOP;
    // 目標値に向けて走行
    goTargetPose(&(strategy_pc_command->robot_goal_pose), &(world->robot_pose), robot_controller, &(output->velocity));
    //// actuatorは動かさない
    output->actuator_type = ACTION_NONE;
    output->actuator_value = 0;
}

void inGame(const StrategyPcCommand *strategy_pc_command, const World *world, RobotController *robot_controller, RobotOutput *output)
{
    switch (strategy_pc_command->ball_action)
    {
    case ACTION_NONE: // ただ目標値に移動
        robot_controller->max_robot_vel = MAX_ROBOT_SPEED_GAME_STOP;
        goTargetPose(&(strategy_pc_command->robot_goal_pose), &(world->robot_pose), robot_controller, &(output->velocity));

        break;

    default:
        break;
    }

    ////ドリブルする
    output->actuator_type = ACTION_DRIBLE;
    output->actuator_value = DRIBBLE_POWER;
    if (world->is_ball_detecting)
    {
        //// ボールを持っているときは、ドリブルで進む
        robot_controller->max_robot_velocity = DRIBBLE_VELOCITY;
    }
    else
    {
        ////ドリブルする
        output->actuator_type = DRIBLE;
        output->actuator_value = DRIBBLE_POWER;
        ////  ボールを持っていないときは、ボールを取りに行く
        //// どの角度でボールを取るか計算
        //// [TODO] フィールドの隅にいるときは後ろ向きで取る
        State2D ball_goal_vector = {0};
        diffState2D(&(strategy_pc_command->ball_goal_pose), &(world->ball_pose), &ball_goal_vector);
        float target_theta = atan2(ball_goal_vector.y, ball_goal_vector.x);
        goBallGetPose(target_theta, world, robot_controller, &(output->velocity));
        return
    }
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
    case GAME_STATE_OUR_BALL_PLACEMENT:
        inGame(strategy_pc_command, world, robot_controller, output);
        break;
    case GAME_STATE_THEIR_BALL_PLACEMENT:
        stopGame(strategy_pc_command, world, robot_controller, output);
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
