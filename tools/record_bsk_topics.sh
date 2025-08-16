#!/bin/bash

ros2 bag record \
/leaderSc/bsk/out/hill_trans_state \
/leaderSc/bsk/out/hill_rot_state \
/leaderSc/bsk_mpc/setpoint_pose \
/leaderSc/bsk_mpc/predicted_path \
/leaderSc/bsk_mpc/vehicle_pose \
/leaderSc/bsk_mpc/vehicle_pose_ref \
/leaderSc/bsk_mpc/vehicle_velocity \
/leaderSc/bsk_mpc/vehicle_velocity_ref \
/leaderSc/bsk_mpc/vehicle_angular_velocity \
/leaderSc/bsk_mpc/vehicle_angular_velocity_ref \
/leaderSc/bsk/in/cmd_force \
/leaderSc/bsk/in/cmd_torque \
/leaderSc/bsk/out/thr_array_cmd_force \
/followerSc_1/bsk/out/hill_trans_state \
/followerSc_1/bsk/out/hill_rot_state \
/followerSc_1/bsk_mpc/setpoint_pose \
/followerSc_1/bsk_mpc/predicted_path \
/followerSc_1/bsk_mpc/vehicle_pose \
/followerSc_1/bsk_mpc/vehicle_pose_ref \
/followerSc_1/bsk_mpc/vehicle_velocity \
/followerSc_1/bsk_mpc/vehicle_velocity_ref \
/followerSc_1/bsk_mpc/vehicle_angular_velocity \
/followerSc_1/bsk_mpc/vehicle_angular_velocity_ref \
/followerSc_1/bsk/in/cmd_force \
/followerSc_1/bsk/in/cmd_torque \
/followerSc_1/bsk/out/thr_array_cmd_force \
/followerSc_2/bsk/out/hill_trans_state \
/followerSc_2/bsk/out/hill_rot_state \
/followerSc_2/bsk_mpc/setpoint_pose \
/followerSc_2/bsk_mpc/predicted_path \
/followerSc_2/bsk_mpc/vehicle_pose \
/followerSc_2/bsk_mpc/vehicle_pose_ref \
/followerSc_2/bsk_mpc/vehicle_velocity \
/followerSc_2/bsk_mpc/vehicle_velocity_ref \
/followerSc_2/bsk_mpc/vehicle_angular_velocity \
/followerSc_2/bsk_mpc/vehicle_angular_velocity_ref \
/followerSc_2/bsk/in/cmd_force \
/followerSc_2/bsk/in/cmd_torque \
/followerSc_2/bsk/out/thr_array_cmd_force

