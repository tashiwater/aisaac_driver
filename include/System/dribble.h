#pragma once
#include "System/accel_limited_control.h"
/// @brief ボールを取りに行く
/// @param target_theta ボールを取る際の機体の角度
/// @param world 現在位置
/// @param robot_controller
/// @param output_velocity 速度
void goBallGetPose(float target_theta, const World *world, RobotController *robot_controller, State2D *output_velocity)
{

    //// ロボット中心からドリブラー中心のベクトルを計算
    float robot_ball_con_distance = ROBOT_KICK_MIN_X;
    State2D robot2dribbler_vector = {0};
    robot2dribbler_vector.x = cosf(target_theta) * robot_ball_con_distance;
    robot2dribbler_vector.y = sinf(target_theta) * robot_ball_con_distance;

    // ボールを受け取る目標pose = ball_pose - robot2dribbler_vector
    State2D target_robot_pose = {0};
    diffState2D(&(world->ball_pose), &robot2dribbler_vector, &target_robot_pose);
    target_robot_pose.theta = target_theta;
    goTargetPose(&target_robot_pose, &(world->robot_pose), robot_controller, output_velocity);
}

void calcDribbleOutput(const World *world, const StrategyPcCommand *strategy_pc_command, const World *world, RobotController *robot_controller, RobotOutput *output)
{
    output->actuator_type = DRIBLE;
    output->actuator_value = DRIBBLE_POWER;
    if (world->is_ball_detecting)
    {
        robot_controller->max_robot_velocity = DRIBBLE_VELOCITY;
    }
    else
    {
        //// まずボールを取りに行く
        //// どの角度でボールを取るか
        State2D ball_goal_vector = {0};
        diffState2D(&(strategy_pc_command->ball_goal_pose), &(world->ball_pose), &ball_goal_vector);
        float target_theta = atan2(ball_goal_vector.y, ball_goal_vector.x);

        ////
        goBallGetPose(target_theta, world, robot_controller, &(output->velocity));
    }
}

bool wrap_dribble(struct coordinate_system robot, ball_state ball, wrap_motion_pid *pid, int32_t ball_target_allowable_error, bool dribble_advance)
{
    float ball_goal_ball_distance = norm((float)ball.ball_wx, (float)ball.ball_wy, (float)ball.dribble_target_ball_x, (float)ball.dribble_target_ball_y);
    float robot_ball_distance = norm((float)ball.ball_rx, (float)ball.ball_ry, 0, 0);
    if (robot_ball_distance <= 0.1 || ball_goal_ball_distance <= 0.1)
    {
        return false;
    }
    float robot_ball_con_distance = ROBOT_KICK_MIN_X;
    // ロボットがドリブルをはじめる時の目標地点
    float robot_kick_goal_x;
    float robot_kick_goal_y;
    if (dribble_advance)
    {
        robot_kick_goal_x = ball.ball_wx + robot_ball_con_distance * (ball.ball_wx - ball.dribble_target_ball_x) / ball_goal_ball_distance;
        robot_kick_goal_y = ball.ball_wy + robot_ball_con_distance * (ball.ball_wy - ball.dribble_target_ball_y) / ball_goal_ball_distance;
    }
    else
    {
        robot_kick_goal_x = ball.ball_wx - robot_ball_con_distance * (ball.ball_wx - ball.dribble_target_ball_x) / ball_goal_ball_distance;
        robot_kick_goal_y = ball.ball_wy - robot_ball_con_distance * (ball.ball_wy - ball.dribble_target_ball_y) / ball_goal_ball_distance;
    }
    float robot_robot_target_distance = norm(robot_kick_goal_x, robot_kick_goal_y, robot.wp.x, robot.wp.y);
    if (robot_robot_target_distance <= 50)
    {
        pid->robot_xy_con_flag = true;
        pid->goal_x = robot_kick_goal_x;
        pid->goal_y = robot_kick_goal_y;
        if (ball.dribble_target_ball_x - ball.ball_wx != 0 || ball.dribble_target_ball_y - ball.ball_wy != 0)
        {
            float w_goal_rad = atan2f(ball.dribble_target_ball_y - ball.ball_wy, ball.dribble_target_ball_x - ball.ball_wx);
            pid->robot_goal_theta = w_goal_rad * RAD_TO_DEG;
        }
        else
        {
            pid->robot_goal_theta = robot.wp.theta;
        }
        // 後退ドリブルの時は目標角が180度反転する
        if (!dribble_advance)
        {
            pid->robot_goal_theta = angle_range_corrector_deg2(pid->robot_goal_theta + 180);
        }
    }
    else
    {
        pid->robot_xy_con_flag = false;
        float different_rad = modifid_acosf(((robot_kick_goal_x - ball.ball_wx) * (robot.wp.x - ball.ball_wx) + (robot_kick_goal_y - ball.ball_wy) * (robot.wp.y - ball.ball_wy)) / (robot_ball_distance * robot_ball_con_distance));
        pid->cul_circumferential_error = robot_ball_con_distance * different_rad;
        pid->cul_radius_error = robot_ball_con_distance - robot_ball_distance;
        float unit_vec_radius_x = (robot.wp.x - ball.ball_wx) / robot_ball_distance;
        float unit_vec_radius_y = (robot.wp.y - ball.ball_wy) / robot_ball_distance;
        pid->ob_unit_vec_radius_x = (robot.wp.x - ball.ball_wx) / robot_ball_distance;
        pid->ob_unit_vec_radius_y = (robot.wp.y - ball.ball_wy) / robot_ball_distance;
        float rotate_flag = (robot_kick_goal_y - robot.wp.y) * (ball.ball_wx - robot.wp.x) - (robot_kick_goal_x - robot.wp.x) * (ball.ball_wy - robot.wp.y);
        float robot_wrap_omega = 0;
        float robot_wrap_d_theta, unit_vec_circumferential_x, unit_vec_circumferential_y;
        float robot_v = norm(robot.wv.vx, robot.wv.vy, 0, 0);
        if (0 < rotate_flag)
        {
            unit_vec_circumferential_x = pid->ob_unit_vec_radius_y;
            unit_vec_circumferential_y = -pid->ob_unit_vec_radius_x;
            if (robot_v != 0)
            {
                robot_wrap_omega = (robot.wv.vx * unit_vec_circumferential_x + robot.wv.vy * unit_vec_circumferential_y) / robot_ball_con_distance;
                // robot_wrap_d_theta = angle_range_corrector(robot_wrap_omega);
                if (0 < robot_wrap_omega)
                {
                    robot_wrap_omega = 0;
                }
            }
        }
        else
        {
            unit_vec_circumferential_x = -pid->ob_unit_vec_radius_y;
            unit_vec_circumferential_y = pid->ob_unit_vec_radius_x;
            if (robot_v != 0)
            {
                robot_wrap_omega = (robot.wv.vx * unit_vec_circumferential_x + robot.wv.vy * unit_vec_circumferential_y) / robot_ball_con_distance;
                // robot_wrap_d_theta = angle_range_corrector(robot_wrap_omega);
                if (robot_wrap_omega < 0)
                {
                    robot_wrap_omega = 0;
                }
            }
        }
        robot_wrap_d_theta = angle_range_corrector(robot_wrap_omega / CONTROL_FREQUENCY);
        pid->ob_unit_vec_circumferential_x = unit_vec_circumferential_x * cosf(robot_wrap_d_theta) - unit_vec_circumferential_y * sinf(robot_wrap_d_theta);
        pid->ob_unit_vec_circumferential_y = unit_vec_circumferential_x * sinf(robot_wrap_d_theta) + unit_vec_circumferential_y * cosf(robot_wrap_d_theta);
        pid->ob_unit_vec_radius_x = unit_vec_radius_x * cosf(robot_wrap_d_theta) - unit_vec_radius_y * sinf(robot_wrap_d_theta);
        pid->ob_unit_vec_radius_y = unit_vec_radius_x * sinf(robot_wrap_d_theta) + unit_vec_radius_y * cosf(robot_wrap_d_theta);
        pid->robot_goal_theta = robot.wp.theta + atan2f(ball.ball_ry, ball.ball_rx) * RAD_TO_DEG + 10 * robot_wrap_omega * RAD_TO_DEG / CONTROL_FREQUENCY;
    }
    // ドリブルを開始するかを判定
    bool dribble_start_flag = ball_dribble_check(robot, ball, ball_target_allowable_error, dribble_advance);
    return dribble_start_flag;
}

void robot_move_kick_drible()
{
    bool dribble_move;
    if (robot_ball_get)
    {
        // パスを受けてボールをこぼしたときの動作
        if (!ball_sensor_valid && fabs(ball_lost_counter - ball_get_counter) < 100 && (dribble_state || ball_kick_state))
        {
            if (!ball_spill_flag)
            {
                ball.spill_ball_get_x = coordinate_system.wp.x + 100 * cos(coordinate_system.wp.theta * DEG2RAD);
                ball.spill_ball_get_y = coordinate_system.wp.y + 100 * sin(coordinate_system.wp.theta * DEG2RAD);
                ball.spill_ball_get_theta = coordinate_system.wp.theta;
            }
            ball_spill_flag = true;
            go_worldXYTheta(&coordinate_system.wov, coordinate_system.wp, ball.spill_ball_get_x, ball.spill_ball_get_y, ball.spill_ball_get_theta);
        }
        // 目標値に向けてドリブル
        else if (dribble_state && (!dribble_conplete_flag || ball_placement_flag))
        {
            wrap_motion_pid pid;
            // 目標値に向かってドリブル
            dribble_move_start_flag = wrap_dribble(coordinate_system, ball, &pid, dribble_enabble_error, dribble_advance);
            if (dribble_move_start_flag && !dribble_conplete_flag)
            {
                max_robot_vel = 500;
                float ball_dribble_target_x = ball.dribble_target_ball_x - (BALL_DIAMETER / 2 + ROBOT_NOT_TOUCH_MIN_X) * cos(pid.robot_goal_theta * DEG2RAD);
                float ball_dribble_target_y = ball.dribble_target_ball_y - (BALL_DIAMETER / 2 + ROBOT_NOT_TOUCH_MIN_X) * sin(pid.robot_goal_theta * DEG2RAD);
                if (!dribble_advance)
                {
                    back_dribble_prepair_counter++;
                    max_robot_vel = 200;
                    if (back_dribble_prepair_counter < 500)
                    {
                        max_robot_vel = 100;
                        ball_dribble_target_x = ball.dribble_target_ball_x - (BALL_DIAMETER / 2 + ROBOT_NOT_TOUCH_MIN_X - 10) * cos(pid.robot_goal_theta * DEG2RAD);
                        ball_dribble_target_y = ball.dribble_target_ball_y - (BALL_DIAMETER / 2 + ROBOT_NOT_TOUCH_MIN_X - 10) * sin(pid.robot_goal_theta * DEG2RAD);
                    }
                }
                go_worldXYTheta(&coordinate_system.wov, coordinate_system.wp, ball_dribble_target_x, ball_dribble_target_y, pid.robot_goal_theta);
                float robot_vel = norm(coordinate_system.wov.vx, coordinate_system.wov.vy, 0, 0);
                float robot_vel_threshold = 300;
                if (ball_placement_flag)
                {
                    dribble_enabble_error = dribble_enabble_error / 2;
                    robot_vel_threshold = 100;
                }
                if (dribble_conplete_check(ball, dribble_enabble_error) && robot_vel < robot_vel_threshold)
                {
                    dribble_move_start_flag = false;
                    dribble_move = false;
                    dribble_conplete_flag = true;
                }
                else
                {
                    dribble_conplete_flag = false;
                }
            }
            else
            {
                back_dribble_prepair_counter = 0;
                if (dribble_conplete_flag && ball_placement_flag)
                {
                    max_robot_vel = 500;
                    // ball placement時はドリブル目標値に到達したらロボットが停止する
                    if (dribble_conplete_check(ball, dribble_enabble_error))
                    {
                        dribble_move_start_flag = false;
                        dribble_move = false;
                        dribble_conplete_flag = true;
                    }
                    else
                    {
                        dribble_conplete_flag = false;
                    }
                    float ball_dribble_target_x = ball.dribble_target_ball_x - (BALL_DIAMETER / 2 + ROBOT_NOT_TOUCH_MIN_X + 300) * cos(pid.robot_goal_theta * DEG2RAD) / 2;
                    float ball_dribble_target_y = ball.dribble_target_ball_y - (BALL_DIAMETER / 2 + ROBOT_NOT_TOUCH_MIN_X + 300) * sin(pid.robot_goal_theta * DEG2RAD) / 2;
                    go_worldXYTheta(&coordinate_system.wov, coordinate_system.wp, ball_dribble_target_x, ball_dribble_target_y, pid.robot_goal_theta);
                }
                else if (pid.robot_xy_con_flag)
                {
                    go_worldXYTheta(&coordinate_system.wov, coordinate_system.wp, pid.goal_x, pid.goal_y, pid.robot_goal_theta);
                }
                else
                {
                    max_robot_vel = 550;
                    wrap_control(&coordinate_system.wov, coordinate_system.wp, pid, pid.robot_goal_theta);
                }
            }
        }
        // 回り込みキック
        else if (ball_kick_state)
        {
            wrap_motion_pid pid;
            if (free_kick_flag)
            {
                max_robot_vel = 400;
                dribbler_power = 0;
                if (!free_kick_prepair)
                {
                    free_kick_prepair = free_kick_prepair_motion(coordinate_system, &pid, &free_kick_prepair_counter, &ball, ball_get_counter);
                    if (!free_kick_prepair)
                    {
                        max_robot_vel = 100;
                        dribble_move = false;
                        go_worldXYTheta(&coordinate_system.wov, coordinate_system.wp, pid.goal_x, pid.goal_y, pid.robot_goal_theta);
                    }
                }
                wrap_kick2(coordinate_system, ball, &pid, free_kick_flag, ball_target_allowable_error, ball_kick, &kick, &dribble_move);
                if (pid.robot_xy_con_flag)
                {
                    go_worldXYTheta(&coordinate_system.wov, coordinate_system.wp, pid.goal_x, pid.goal_y, pid.robot_goal_theta);
                }
                else
                {
                    wrap_control(&coordinate_system.wov, coordinate_system.wp, pid, pid.robot_goal_theta);
                }
            }
            else
            {
                max_robot_vel = 550;
                wrap_kick2(coordinate_system, ball, &pid, free_kick_flag, ball_target_allowable_error, ball_kick, &kick, &dribble_move);
                if (pid.robot_xy_con_flag)
                {
                    go_worldXYTheta(&coordinate_system.wov, coordinate_system.wp, pid.goal_x, pid.goal_y, pid.robot_goal_theta);
                }
                else
                {
                    wrap_control(&coordinate_system.wov, coordinate_system.wp, pid, pid.robot_goal_theta);
                }
            }
        }
        else
        {
            world_ob_vector_calculate(&coordinate_system.wov, coordinate_system.wp, coordinate_system.wop);
        }
    }
    else
    {
        // 目標値に向かって走行
        world_ob_vector_calculate(&coordinate_system.wov, coordinate_system.wp, coordinate_system.wop);
    }

    // ドリブラーの制御
    if ((((ball_kick_state && !free_kick_flag) || (dribble_state)) && ball_get_counter < 200) || dribble_move_start_flag)
    {
        dribble_move = true;
    }
    else
    {
        dribble_move = false;
    }
    if (dribble_move)
    {
        dribbler_power = 100;
        // dribbler_power = 0;
    }
    else
    {
        dribbler_power = 0;
    }
}
