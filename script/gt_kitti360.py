import numpy as np
import ground_truth_tools as gtt
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d


def process_kitti360(lidar_folder_path, timestamp_path, calib_cam_to_velo_path, cam0_to_world_path, output_folder_path):
    sequence_output_path = output_folder_path + "/out.txt"
    nonsequence_output_path = output_folder_path + "/out2.txt"
    # for cont2
    ts_bin_path = output_folder_path + "/ts-lidar_bins.txt"
    ts_pose_path = output_folder_path + "/ts-sens_pose.txt"

    file_pathes = gtt.get_file_pathes_in_folder(lidar_folder_path)
    timestamps = gtt.read_timestamps_kitti360(timestamp_path)
    cam0_to_world_poses = gtt.read_pose_file_kitti360(cam0_to_world_path)
    print("cam0_to_world_poses has", len(cam0_to_world_poses))
    print("has", len(file_pathes), "bin files")
    print("has", len(timestamps), "timestamps")

    calib_cam_to_velo = gtt.read_calibration_file_kitti360(calib_cam_to_velo_path)
    print("calib_cam_to_velo:\n", calib_cam_to_velo)
    velo_to_cam0 = gtt.get_invers_transform(calib_cam_to_velo)
    print("velo_to_cam0:\n", velo_to_cam0)

    frame_infos = {}
    for frame_index, pose_values in cam0_to_world_poses.items():
        pose_matrix = np.array(pose_values[0]).reshape(4, 4)
        velo_to_world = np.dot(pose_matrix, velo_to_cam0)
        if (frame_index > len(file_pathes)) :
            continue
        frame_info = gtt.ScanInfo(frame_index)
        # frame_id of KITTI360 start from 1
        frame_info.bin_file_path = file_pathes[frame_index - 1]
        frame_info.timestamp = timestamps[frame_index - 1]
        frame_info.pose = velo_to_world[:3, :] # Exclude the last row
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
    lidar_folder_path = "/media/zz/new/kitti360/KITTI-360/data_3d_raw/2013_05_28_drive_0010_sync/velodyne_points/data"
    timestamp_path = "/media/zz/new/kitti360/KITTI-360/data_3d_raw/2013_05_28_drive_0010_sync/velodyne_points/timestamps.txt"
    calib_cam_to_velo_path = "/media/zz/new/kitti360/KITTI-360/calibration/calib_cam_to_velo.txt"
    cam0_to_world_path = "/media/zz/new/kitti360/KITTI-360/data_poses/2013_05_28_drive_0010_sync/cam0_to_world.txt"

    output_folder_path = "/media/zz/new/myMidImg/kitti360_10"

    process_kitti360(lidar_folder_path, timestamp_path,
                     calib_cam_to_velo_path, cam0_to_world_path, output_folder_path)