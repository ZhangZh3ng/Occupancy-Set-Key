import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import KDTree
import pr_tools as pr

thres_dist = 15.0

if __name__ == "__main__":
    # Replace 'your_file_path.txt' with the path to your .txt file containing the poses
    # file_path = "/media/zz/new/cont2_mid/mulran-Riverside02/results/mulran_to_kitti/ts-sens_pose.txt"
    file_path = "/media/zz/new/cont2_gen/ts-sens_pose-kitti08_semantic.txt"
    file_outcome_osk = "/media/zz/new/myMidImg/search_result/kitti08.txt"

    plot_data, detected_loops = pr.get_pr_points_and_max_f1_loops(
        file_path, file_outcome_osk, thres_dist)

    poses = pr.get_gt_sens_poses(file_path)
    pr.plot_trajectory_with_loop_mark(poses, detected_loops, thres_dist)
