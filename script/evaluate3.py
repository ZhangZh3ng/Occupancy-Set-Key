import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.spatial import KDTree
import pr_tools as pr

if __name__ == "__main__":
    file_gt_sens_poses = "/media/zz/new/myMidImg/kitti_00/out.txt"

    file_outcome_osk = "/media/zz/new/myMidImg/result_osk/kitti_00_1.txt"
    file_outcome_sc = "/media/zz/new/myMidImg/result_osk/kitti_00_2.txt"
    file_outcome_iris = "/media/zz/new/myMidImg/result_osk/kitti_00_3.txt"
    file_outcome_cont2 = "/media/zz/new/myMidImg/result_osk/kitti_00_4.txt"
    file_outcome_5 = "/media/zz/new/myMidImg/result_osk/kitti_00_5.txt"

    fig_save_folder = "/home/zz/桌面/fig/multi"

    dataset_name = "kitti_00"
    thres_dist = 15.0
    thres_frame_dist = 150

    fig_name = dataset_name + "_" + str(int(thres_dist))
    fig_path = fig_save_folder + "/" + fig_name + ".eps"
    print(fig_name)

    print("Dataset: " + dataset_name + " thresh_dist: " + str(thres_dist))

    osk_result = pr.read_loop_detction_result(file_outcome_osk)
    sc_result = pr.read_loop_detction_result(file_outcome_sc)
    iris_result = pr.read_loop_detction_result(file_outcome_iris)
    cont2_result = pr.read_loop_detction_result(file_outcome_cont2)
    cont2_result2 = pr.read_loop_detction_result(file_outcome_5)

    pr_results = [
        pr.comput_pr_points(file_gt_sens_poses, osk_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, sc_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, iris_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, cont2_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, cont2_result2, thres_dist, thres_frame_dist)
    ]
    
    data_names = [
        "1",
        "2",
        "3",
        "4",
        "5"
    ]

    title = dataset_name + "  th:" + str(thres_dist) + "m"
    title = ""

    fig = pr.plot_pr_curves(pr_results, data_names, title,use_label_mark=True)

    fig.savefig(fig_path)
    
    plt.show()