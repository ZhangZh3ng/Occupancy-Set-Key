import numpy as np
import matplotlib.pyplot as plt

import pr_tools as pr

file_gt_sens_poses = "/media/zz/new/myMidImg/kitti_00/out.txt"
file_outcome = "/media/zz/new/myMidImg/result_osk/kitti_00.txt"
file_outcome2 = "/media/zz/new/myMidImg/result_cont2/kitti_00.txt"
# file_outcome = "/media/zz/new/myMidImg/result_sc/kitti360_00.txt"
maxf1_score = 0.28
thres_dist = 15.0
thres_frame_dist = 150

outcome = pr.read_loop_detction_result(file_outcome)
# outcome = pr.read_loop_detction_result_cont2(file_outcome2)

trajectory = pr.comput_maxf1_result(
    file_gt_sens_poses, outcome, maxf1_score, thres_dist, thres_frame_dist)

for entry in trajectory :
    val = entry[0]
    entry[0] = -entry[1]
    entry[1] = val

# Call the function to visualize the PR results
fig = pr.visualize_pr_trajectory(trajectory, use_legend=True, use_grid=False, title="")
plt.show()