#pragma once
#include "System/struct.h"
#include "System/struct_tool.h"
#include "System/pid.h"
#include "System/tools.h"
#include "System/define.h"
/// @brief PIDで速度を計算。制動速度以下であることも確認する
/// @param max_accel 最大加速度
/// @param dead_band 制動をかける最小誤差。この値以下のときはPIDを優先
/// @param velocity_threshold 制動をかける最小速度. この値以下のときはPIDを優先
/// @param max_robot_vel 最大速度
/// @param target_position
/// @param current_position
/// @param current_velocity
/// @param pid
/// @return
float calcVelocityFromPID(float max_accel, float dead_band, float velocity_threshold, float target_position, float current_position, float current_velocity, _pid_t *pid)
{
    float error = target_position - current_position;
    float error_abs = fabs(error);
    float target_velocity = pidExecute(pid);
    float target_velocity_abs = fabs(target_velocity);

    // 制動可能速度以下か
    float v_limit = sqrt(2 * error_abs * max_accel);
    if (target_velocity_abs > v_limit && dead_band < error_abs && velocity_threshold < target_velocity_abs)
    {
        // Decelerates the robot when the distance between the current location and the target value is smaller than the braking distance.
        target_velocity_abs = v_limit;
    }
    return getSign(target_velocity) * target_velocity_abs;
}

float velocityConstraint(float max_accel, float dt, float max_velocity, float target_velocity, float current_velocity)
{
    // 加速度制約
    float result = target_velocity;
    float target_accel = target_velocity - current_velocity;
    float max_dv = max_accel * dt;
    if (fabs(target_accel) > max_dv)
    {
        result = current_velocity + getSign(target_accel) * max_dv;
    }

    // 最大速度制約
    if (fabs(result) > max_velocity)
    {
        result = max_velocity * getSign(result);
    }
    return result;
}
void calcVelocityFromStrategy(const StrategyPcCommand *strategy_pc_command, const World *world, RobotController *robot_controller, RobotOutput *output)
{
}

///// 最大加速度制約を二次元にかけた場合のコード。スマートにするために1次元の方を使う
// void calcXYVelocity(float max_accel, float dead_band, float velocity_threshold, float max_robot_vel,
//                     const State2D *error_vector, _pid_t *pid_x, _pid_t *pid_y, State2D *output_velocity)
// {
//     // Calculate the target vector in the world coordinate system
//     pid_x->error = error_vector->x;
//     pid_y->error = error_vector->y;
//     float error = xyNorm(error_vector);
//     State2D target_velocity = {0};

//     target_velocity.x = pidExecute(&pid_x);
//     target_velocity.y = pidExecute(&pid_y);
//     float target_velocity_norm = xyNorm(&target_velocity);

//     // 現在の速度と目標速度が乖離していないか確認
//     float braking_distance = target_velocity_norm * target_velocity_norm / (2 * max_accel);
//     if (error < braking_distance && dead_band < error && velocity_threshold < target_velocity_norm)
//     {
//         // Decelerates the robot when the distance between the current location and the target value is smaller than the braking distance.
//         float v_limit = sqrt(2 * error * max_accel);
//         // ロボットの指令速度が最大速度を超えていないか
//         xyMultiple(&target_velocity, v_limit / target_velocity_norm, &target_velocity);
//         target_velocity_norm = v_limit;
//     }
//     // robot velocity check
//     if (target_velocity_norm > max_robot_vel)
//     {
//         // the command speed of the robot is exceeding the maximum speed
//         xyMultiple(&target_velocity, max_robot_vel / target_velocity_norm, &target_velocity);
//         target_velocity_norm = max_robot_vel;
//     }
//     *output_velocity = target_velocity;
// }

// void calcAccelLimitedXY(float max_accel, float dt, const State2D *target_velocity, State2D *inout_velocity)
// {
//     State2D ret = *target_velocity;
//     State2D accel_vector = {0};
//     diffState2D(target_velocity, inout_velocity, &accel_vector);
//     float accel = xyNorm(&accel_vector);
//     float max_dv = max_accel * dt;
//     if (accel > max_dv)
//     {
//         ret.x = inout_velocity->x + accel_vector.x * max_dv / accel;
//         ret.y = inout_velocity->y + accel_vector.y * max_dv / accel;
//     }
//     *inout_velocity = ret;
// }

// /// @brief 台形加速を行う
// /// @param robot_goal_pose 目標位置
// /// @param current_pose 現在位置
// /// @param pid_x
// /// @param pid_y
// /// @param pid_theta
// /// @param inout_velocity input: 1ループ前の目標vx,vy,omega. output: 目標vx,vy,omega
// void goTargetPose(const State2D *target_pose, const State2D *current_pose,
//                   RobotController *robot_controller, State2D *inout_velocity)
// {
//     State2D error = {0}; // 偏差
//     diffState2D(target_pose, current_pose, &error);

//     //// xy速度を計算
//     State2D velocity_from_pid = {0};
//     calcVelocityFromPid(ROBOT_MAX_ACCEL, POSITION_CONTROL_DEAD_BAND, VEROSITY_CONTROL_THRESHOLD, robot_controller->max_robot_velocity,
//                         &error, &(robot_controller->pid_x), &(robot_controller->pid_y), &velocity_from_pid);
//     // 加速度制約
//     calcAccelLimitedXY(ROBOT_MAX_ACCEL, CONTROL_DT, &velocity_from_pid, inout_velocity);

//     //// 角速度計算
//     //// [WARN] x=theta, y=0とすることで、calcAccelLimitedXYを流用可能にしている
//     State2D error_for_omega = {0};
//     State2D omega_from_pid = {0};
//     _pid_t dummy = {0};
//     error_for_omega.x = angle_range_corrector_deg2(error.theta); //[-180, 180]に収める
//     calcVelocityFromPid(ROBOT_MAX__ROTATE_ACCELE, 0, 0, SAFETY_MACHINE_OMEGA,
//                         &error_for_omega, &(robot_controller->pid_theta), &dummy, &omega_from_pid);
//     // 加速度制約
//     State2D inout_omega = {0};
//     inout_omega.x = inout_velocity->theta;
//     calcAccelLimitedXY(ROBOT_MAX__ROTATE_ACCELE, CONTROL_DT, &omega_from_pid, &inout_omega);
//     inout_velocity->theta = inout_omega.x
// }
