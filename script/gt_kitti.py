import numpy as np
import re
import os
import csv
import ground_truth_tools as gtt

def gen_kitti(lidar_folder_path, timestamp_path, f_calib, left_camera_to_world_path, output_folder_path):
    sequence_output_path = output_folder_path + "/out.txt"
    nonsequence_output_path = output_folder_path + "/out2.txt"
    # for cont2
    ts_bin_path = output_folder_path + "/ts-lidar_bins.txt"
    ts_pose_path = output_folder_path + "/ts-sens_pose.txt"

    file_pathes = gtt.get_file_pathes_in_folder(lidar_folder_path)
    timestamps = gtt.read_timestamps(timestamp_path)

    left_camera_to_world_pose = gtt.read_pose_file_kitti(left_camera_to_world_path)

    left_cam_to_velo = np.hstack([np.identity(3), np.zeros((3, 1))])
    with open(f_calib, "r") as f1:
        for line in f1.readlines():
            segs = line.strip().split(" ")
            if segs and segs[0] == "Tr:":
                assert len(segs) == 13
                left_cam_to_velo = np.array(
                    [eval(x) for x in segs[1:]]).reshape((3, 4))
    T_leftcam_velod_ = np.vstack([left_cam_to_velo, np.array([[0, 0, 0, 1]])])
    
    frame_infos = {}
    for frame_index, pose_values in left_camera_to_world_pose.items():
        pose_matrix = np.array(pose_values[0]).reshape(4, 4)
        tmp_T_lc0_lc = pose_matrix
        T_w_velod = np.linalg.inv(
            T_leftcam_velod_) @ tmp_T_lc0_lc @ T_leftcam_velod_
        frame_info = gtt.ScanInfo(frame_index)
        frame_info.bin_file_path = file_pathes[frame_index]
        frame_info.timestamp = timestamps[frame_index]
        frame_info.pose = T_w_velod[:3, :] # Exclude the last row
        frame_infos[frame_index] = frame_info
    
    gtt.write_sequence_frame_infos(frame_infos, sequence_output_path)
    gtt.write_frame_infos(frame_infos, nonsequence_output_path)
    gtt.write_timestamp_pose(frame_infos, ts_pose_path)
    gtt.write_timestamp_id_bin(frame_infos, ts_bin_path)

    print("Sequence Output to:", sequence_output_path)
    print("Non-Sequence Output written to:", nonsequence_output_path)
    print("ts-sens_pose written to:", ts_pose_path)
    print("ts-lidar_bins written to:", ts_bin_path)


if __name__ == "__main__":
    # =============================== KITTI Odometry ====================================
    lidar_folder_path = "/media/zz/new/kitti_odometry/data_odometry_velodyne/dataset/sequences/06/velodyne"
    timestamp_path = "/media/zz/new/kitti_odometry/data_odometry_calib/dataset/sequences/06/times.txt"
    calib_path = "/media/zz/new/kitti_odometry/data_odometry_calib/dataset/sequences/06/calib.txt"
    pose_path = "/media/zz/new/kitti_odometry/data_odometry_labels/dataset/sequences/06/poses.txt"

    output_folder_path = "/media/zz/new/myMidImg/kitti_06"

    gen_kitti(lidar_folder_path, timestamp_path, calib_path, pose_path, output_folder_path)
