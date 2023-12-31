import numpy as np
import math
import pr_tools as pr
import ground_truth_tools as gtt

def extract_rpy_translation(matrix):
    # Extract the rotation matrix (upper-left 3x3 part)
    rotation_matrix = matrix[:3, :3]
    
    # Calculate the translation vector (upper-right 3x1 part)
    translation_vector = matrix[:3, 3]
    
    # Calculate roll, pitch, and yaw (Euler angles) from the rotation matrix
    # Note: This assumes a specific order of rotations, typically ZYX
    # Yaw (rotation about the Z-axis)
    # Pitch (rotation about the Y-axis)
    # Roll (rotation about the X-axis)
    yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
    roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    
    return roll, pitch, yaw, translation_vector


names = ["kitti_00", "kitti_02", "kitti_05", "kitti_08", "kitti360_00", "kitti360_04", "kitti360_05", "kitti360_06", "kitti360_09"]

result_path = "/media/zz/new/myMidImg/pose_err_result/osk.txt"

with open(result_path, "w") as results_file:
    for dataset_name in names:
        file_gt_sens_poses = "/media/zz/new/myMidImg/" + dataset_name + "/out.txt"
        file_outcome = "/media/zz/new/myMidImg/result_osk/" + dataset_name + ".txt"

        tp_result = pr.find_true_positive_outcome(file_gt_sens_poses, file_outcome, thres_dist=15, thres_score= 0.25)
        gt_dic = pr.get_gt_sens_pose_dictionary(file_gt_sens_poses)

        translation_err_list = []  # List to store translation errors
        yaw_err_list = []          # List to store yaw errors

        for row in tp_result:
            query_scan_id, matching_scan_id, transformation_matrix = row
            gt_query = gt_dic.get(query_scan_id)
            gt_match = gt_dic.get(matching_scan_id)

            T_match_to_query_est = gtt.vector_to_transform(transformation_matrix)

            T_match_to_world = gtt.vector_to_transform(gt_match)
            T_query_to_world = gtt.vector_to_transform(gt_query)

            T_match_to_query = np.dot(gtt.get_invers_transform(T_query_to_world), T_match_to_world)

            err = np.dot(gtt.get_invers_transform(T_match_to_query), T_match_to_query_est)

            roll, pitch, yaw, translation_vector = extract_rpy_translation(err)

            translation_err = np.linalg.norm(translation_vector)
            yaw_err = np.abs(yaw * 180 / math.pi)

            # Append the errors to the respective lists
            translation_err_list.append(translation_err)
            yaw_err_list.append(yaw_err)

        # Calculate mean, standard deviation, and maximum
        mean_translation_err = np.mean(translation_err_list)
        std_translation_err = np.std(translation_err_list)
        max_translation_err = np.max(translation_err_list)

        mean_yaw_err = np.mean(yaw_err_list)
        std_yaw_err = np.std(yaw_err_list)
        max_yaw_err = np.max(yaw_err_list)

        results_file.write(f"{dataset_name} total: {len(translation_err_list)}\n")
        results_file.write(f"Translation Error Mean: {mean_translation_err}\n")
        results_file.write(f"Translation Error Std: {std_translation_err}\n")
        results_file.write(f"Translation Error Max: {max_translation_err}\n")
        results_file.write(f"Yaw Error Mean: {mean_yaw_err}\n")
        results_file.write(f"Yaw Error Std: {std_yaw_err}\n")
        results_file.write(f"Yaw Error Max: {max_yaw_err}\n")
        results_file.write("\n")  # Separate results for each dataset with a blank line

