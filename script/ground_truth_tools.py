import os
import re
import numpy as np
from datetime import datetime, timedelta
# from mpl_toolkits.mplot3d import Axes3D


class ScanInfo:
    def __init__(self, scan_id, timestamp=0.0, bin_file_path="", pose=None):
        self.scan_id = scan_id
        self.timestamp = timestamp
        self.bin_file_path = bin_file_path
        self.pose = pose

    def __str__(self):
        pose_str = " ".join(
            f"{val:.6f}" for val in self.pose.flatten()) if self.pose is not None else ""
        return f"{self.scan_id} {self.timestamp:.6e} {pose_str} {self.bin_file_path}"

#                                timestamp


def calculate_relative_time(timestamps):
    first_time = timestamps[0]
    relative_times = [(t - first_time).total_seconds() for t in timestamps]
    return relative_times


def read_timestamps_kitti360(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        timestamps = []
        for line in lines:
            parts = line.strip().split('.')
            timestamp = datetime.strptime(parts[0], "%Y-%m-%d %H:%M:%S")
            microseconds = int(parts[1][:6])  # Round to 6 digits
            timestamps.append(timestamp + timedelta(microseconds=microseconds))
        timestamps = calculate_relative_time(timestamps)
        return timestamps


def read_timestamps(timestamps_path):
    with open(timestamps_path, "r") as timestamps_file:
        timestamps = [float(line.strip()) for line in timestamps_file]
    return timestamps

#                           parse .bin folder


def get_file_pathes_in_folder(folder_path):
    file_pathes = [
        entry.path
        for entry in os.scandir(folder_path)
        if entry.is_file()
    ]

    # Sort the file names numerically
    file_pathes.sort(key=lambda entry: (
        re.sub(r'\D', '', os.path.splitext(os.path.basename(entry))[0]), entry))
    return file_pathes

#                              pose and calibration


def read_pose_file_kitti360(file_path):
    poses = {}
    with open(file_path, 'r') as f:
        for line in f:
            values = line.strip().split()
            frame_index = int(values[0])
            matrix = np.array([list(map(float, values[1:]))])
            poses[frame_index] = matrix
    return poses


def read_pose_file_kitti(file_path):
    poses = {}
    frame_index = 0
    with open(file_path, 'r') as f:
        for line in f:
            values = line.strip().split()
            matrix = np.array([list(map(float, values[0:]))])
            matrix = np.hstack((matrix, np.array([0, 0, 0, 1]).reshape(-1, 4)))
            poses[frame_index] = matrix
            frame_index += 1
    return poses


def read_calibration_file_kitti360(file_path):
    with open(file_path, 'r') as f:
        values = list(map(float, f.readline().strip().split()))
        calib_matrix = np.array(values).reshape(3, -1)  # Reshape to 3xN matrix
    return calib_matrix


def get_invers_transform(calib_cam_to_velo):
    rotation = calib_cam_to_velo[:, :3]
    translation = calib_cam_to_velo[:, 3]
    velo_to_cam = np.hstack(
        (rotation.T, -np.dot(rotation.T, translation.reshape(-1, 1))))
    velo_to_cam = np.vstack((velo_to_cam, [0, 0, 0, 1]))
    return velo_to_cam


def interpolate_pose(pose1, pose2, t1, t2, t):
    alpha = (t - t1) / (t2 - t1)
    interpolated_pose = (1 - alpha) * pose1 + alpha * pose2
    return interpolated_pose

#                                 write


def write_timestamp_id_bin(timestamps, file_paths, output_file_path):
    output_lines = []
    for i, (file_path, timestamp) in enumerate(zip(file_paths, timestamps), start=1):
        output_lines.append(f"{timestamp:.6e} {i} {file_path}")

    with open(output_file_path, "w") as output_file:
        for line in output_lines:
            output_file.write(line + "\n")

    return output_lines


def write_frame_infos(frame_infos, output_path):
    with open(output_path, "w") as output_file:
        for scan_id, frame_info in frame_infos.items():
            output_file.write(str(frame_info) + "\n")

# Some dataset has discontinuous scan, such as KITTI360. In this dataset some stationary scans are discarded, so their scan_id may 'jump'. That cause problem when testing some method such as Scan Context, so we change these 'jumped' id into sequence for convience.
def write_sequence_frame_infos(frame_infos, output_path):
    with open(output_path, "w") as output_file:
        # since we don't care the origin scan_id, always start from 0
        sequence_id = 0
        for scan_id, frame_info in frame_infos.items():
            timestamp = frame_info.timestamp
            bin_file_path = frame_info.bin_file_path
            pose_str = " ".join(f"{val:.6f}" for val in frame_info.pose.flatten(
            )) if frame_info.pose is not None else ""
            output_file.write(
                f"{sequence_id} {timestamp:.6e} {pose_str} {bin_file_path}\n")
            sequence_id += 1


def write_timestamp_pose(frame_infos, output_path):
    with open(output_path, "w") as output_file:
        for scan_id, frame_info in frame_infos.items():
            timestamp = frame_info.timestamp
            pose_str = " ".join(f"{val:.6f}" for val in frame_info.pose.flatten(
            )) if frame_info.pose is not None else ""
            output_file.write(f"{timestamp:.6e} {pose_str}\n")


def write_timestamp_id_pose(frame_infos, output_path):
    with open(output_path, "w") as output_file:
        for scan_id, frame_info in frame_infos.items():
            timestamp = frame_info.timestamp
            pose_str = " ".join(f"{val:.6f}" for val in frame_info.pose.flatten(
            )) if frame_info.pose is not None else ""
            output_file.write(f"{timestamp:.6e} {scan_id} {pose_str}\n")


def write_timestamp_id_bin(frame_infos, output_path):
    with open(output_path, "w") as output_file:
        sequence_id = 0
        for scan_id, frame_info in frame_infos.items():
            timestamp = frame_info.timestamp
            bin_file_path = frame_info.bin_file_path
            output_file.write(f"{timestamp:.6e} {sequence_id} {bin_file_path}\n")
            sequence_id += 1
