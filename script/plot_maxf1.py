import numpy as np
import matplotlib.pyplot as plt

import pr_tools as pr

file_gt_sens_poses = "/media/zz/new/myMidImg/kitti360_00/out.txt"
file_outcome = "/media/zz/new/myMidImg/result_cont2/kitti360_00.txt"
# file_outcome = "/media/zz/new/myMidImg/result_osk/kitti360_00.txt"
file_outcome = "/media/zz/new/myMidImg/result_sc/kitti360_00.txt"
maxf1_score = 0.774075
thres_dist = 15.0
thres_frame_dist = 150

outcome = pr.read_loop_detction_result(file_outcome)

print(len(outcome))
trajectory = pr.comput_maxf1_result(
    file_gt_sens_poses, outcome, maxf1_score, thres_dist, thres_frame_dist)

# Call the function to visualize the PR results
pr.visualize_pr_trajectory(trajectory, use_legend=False, use_grid=False, title="")
