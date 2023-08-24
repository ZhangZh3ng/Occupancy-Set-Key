import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.spatial import KDTree
import pr_tools as pr

if __name__ == "__main__":
    file_gt_sens_poses = "/media/zz/new/myMidImg/kitti360_00/out.txt"

    file_outcome_osk = "/media/zz/new/myMidImg/search_result/kitti360_00.txt"
    file_outcome_cont2 = "/media/zz/new/cont2_gen/outcome-kitti02_semantic.txt"
    file_outcome_sc = "/media/zz/new/myresult/sc-kitti02_semantic.txt"

    file_outcome_bow3d = "/media/zz/new/myMidImg/bow3d_results/kitti360_00.txt"

    thres_dist = 10.0
    thres_time = 30

    osk_result = pr.read_loop_detction_result(file_outcome_osk)
    bow3d_result = pr.read_loop_detction_result(file_outcome_bow3d)
    # cont2_result = pr.read_loop_detction_result_cont2(file_outcome_cont2)
    # sc_result = pr.read_loop_detction_result_sc(file_outcome_sc)

    pr_results = [
        pr.comput_pr_points(file_gt_sens_poses, osk_result, thres_dist, thres_time),
        pr.comput_pr_points(file_gt_sens_poses, bow3d_result, thres_dist, thres_time),
        # pr.comput_pr_points(file_gt_sens_poses, cont2_result, thres_dist, thres_time),
        # pr.comput_pr_points(file_gt_sens_poses, sc_result, thres_dist, thres_time)
    ]
    
    data_names = [
        "OSK",
        "bow3d"
        # "Cont2",
        # "SC"
    ]

    title = "kitti00 threshold = " + str(thres_dist)

    pr.plot_pr_curves(pr_results, data_names, title)