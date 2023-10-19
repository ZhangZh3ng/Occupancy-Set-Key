import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.spatial import KDTree
import pr_tools as pr

if __name__ == "__main__":
    file_gt_sens_poses = "/media/zz/new/myMidImg/kitti360_09/out.txt"

    file_outcome_osk = "/media/zz/new/myMidImg/result_osk/kitti360_09.txt"
    file_outcome_sc = "/media/zz/new/myMidImg/result_sc/kitti360_09.txt"
    file_outcome_iris = "/media/zz/new/myMidImg/result_iris/kitti360_09.txt"
    file_outcome_cont2 = "/media/zz/new/myMidImg/result_cont2/kitti360_09.txt"

    fig_save_folder = "/home/zz/桌面/fig"

    # file_outcome_std = "/media/zz/new/myMidImg/result_std/kitti_00.txt"
    # file_outcome_bow3d = "/media/zz/new/myMidImg/result_bow3d/kitti360_00.txt"

    dataset_name = "kitti360_09"
    thres_dist = 15.0
    thres_frame_dist = 150

    fig_name = dataset_name + "_" + str(int(thres_dist))
    fig_path = fig_save_folder + "/" + fig_name + ".png"
    print(fig_name)

    print("Dataset: " + dataset_name + " thresh_dist: " + str(thres_dist))

    osk_result = pr.read_loop_detction_result(file_outcome_osk)
    sc_result = pr.read_loop_detction_result(file_outcome_sc)
    iris_result = pr.read_loop_detction_result(file_outcome_iris)
    cont2_result = pr.read_loop_detction_result_cont2(file_outcome_cont2)

    # bow3d_result = pr.read_loop_detction_result(file_outcome_bow3d)
    # std_result = pr.read_loop_detction_result(file_outcome_std)

    pr_results = [
        pr.comput_pr_points(file_gt_sens_poses, osk_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, sc_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, iris_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, cont2_result, thres_dist, thres_frame_dist),
        # pr.comput_pr_points(file_gt_sens_poses, std_result, thres_dist, thres_frame_dist)
    ]
    
    data_names = [
        "OSK(Ours)",
        "SC",
        "Iris",
        "Cont2",
        # "STD"
    ]

    title = dataset_name + "  th:" + str(thres_dist) + "m"

    fig = pr.plot_pr_curves(pr_results, data_names, title)

    fig.savefig(fig_path)
    
    plt.show()