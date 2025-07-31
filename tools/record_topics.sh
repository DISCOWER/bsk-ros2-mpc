#!/bin/bash

ros2 bag record \
/camera/camera/color/camera_info \
/camera/camera/color/image_raw \
/camera/camera/color/image_raw/compressed \
/camera/camera/color/image_raw/compressedDepth \
/camera/camera/color/image_raw/theora \
/camera/camera/color/metadata \
/crackle/bsk_mpc/predicted_path \
/crackle/bsk_mpc/setpoint_pose \
/crackle/bsk_mpc/vehicle_angular_velocity \
/crackle/bsk_mpc/vehicle_angular_velocity_ref \
/crackle/bsk_mpc/vehicle_pose \
/crackle/bsk_mpc/vehicle_pose_ref \
/crackle/bsk_mpc/vehicle_velocity \
/crackle/bsk_mpc/vehicle_velocity_ref \
/crackle/fmu/in/actuator_motors \
/crackle/fmu/in/offboard_control_mode \
/crackle/fmu/in/vehicle_visual_odometry \
/crackle/fmu/out/vehicle_angular_velocity \
/crackle/fmu/out/vehicle_attitude \
/crackle/fmu/out/vehicle_control_mode \
/crackle/fmu/out/vehicle_local_position \
/crackle/fmu/out/vehicle_status_v1 \
/crackle/odom \
/snap/bsk_mpc/predicted_path \
/snap/bsk_mpc/setpoint_pose \
/snap/bsk_mpc/vehicle_angular_velocity \
/snap/bsk_mpc/vehicle_angular_velocity_ref \
/snap/bsk_mpc/vehicle_pose \
/snap/bsk_mpc/vehicle_pose_ref \
/snap/bsk_mpc/vehicle_velocity \
/snap/bsk_mpc/vehicle_velocity_ref \
/snap/fmu/in/actuator_motors \
/snap/fmu/in/offboard_control_mode \
/snap/fmu/in/vehicle_visual_odometry \
/snap/fmu/out/actuator_motors \
/snap/fmu/out/vehicle_angular_velocity \
/snap/fmu/out/vehicle_attitude \
/snap/fmu/out/vehicle_control_mode \
/snap/fmu/out/vehicle_local_position \
/snap/fmu/out/vehicle_status_v1 \
/snap/odom
