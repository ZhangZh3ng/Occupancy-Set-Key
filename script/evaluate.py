import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.spatial import KDTree
import pr_tools as pr

if __name__ == "__main__":
    file_gt_sens_poses = "/media/zz/new/myMidImg/kitti_08/out.txt"

    file_outcome_osk = "/media/zz/new/myMidImg/search_result/kitti_08_4.txt"
    file_outcome_cont2 = "/media/zz/new/cont2_gen/outcome-kitti08_semantic.txt"
    file_outcome_sc = "/media/zz/new/myresult/sc-kitti08_semantic.txt"
    file_outcome_iris = "/media/zz/new/myMidImg/iris_result/kitti_06.txt"

    file_outcome_bow3d = "/media/zz/new/myMidImg/bow3d_results/kitti360_00.txt"

    thres_dist = 15.0
    thres_frame_dist = 150

    osk_result = pr.read_loop_detction_result(file_outcome_osk)
    bow3d_result = pr.read_loop_detction_result(file_outcome_bow3d)
    iris_result = pr.read_loop_detction_result(file_outcome_iris)
    cont2_result = pr.read_loop_detction_result_cont2(file_outcome_cont2)
    sc_result = pr.read_loop_detction_result_sc(file_outcome_sc)

    pr_results = [
        pr.comput_pr_points(file_gt_sens_poses, osk_result, thres_dist, thres_frame_dist),
        # pr.comput_pr_points(file_gt_sens_poses, iris_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, cont2_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, sc_result, thres_dist, thres_frame_dist)
    ]
    
    data_names = [
        "OSK",
        # "iris",
        "Cont2",
        "SC"
    ]

    title = "kitti02 threshold = " + str(thres_dist)

    pr.plot_pr_curves(pr_results, data_names, title)