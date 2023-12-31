import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.spatial import KDTree
import pr_tools as pr

if __name__ == "__main__":

    band_l = [4, 5, 6, 7, 8]
    band_b = [8, 10, 12, 14, 16]

    file_gt_sens_poses = "/media/zz/new/myMidImg/kitti_00/out.txt"
    thres_dist = 15.0
    thres_frame_dist = 150

    names = []
    for l in band_l :
        for b in band_b :
            file_outcome = "/media/zz/new/myMidImg/result_osk/kitti_00_l" + str(l) + "_b" + str(b) +".txt"
            # print(file_outcome)

            osk_result = pr.read_loop_detction_result(file_outcome)
            
            auc, maxf1 = pr.comput_auc_maxf1(file_gt_sens_poses, osk_result, thres_dist=thres_dist, thres_frame_dist=thres_frame_dist)
            print("l = %d b = %d: auc = %f maxf1 = %f" % (l, b, auc, maxf1))

    # names = {"kitti_00", "kitti_02", "kitti_05", "kitti_08", "kitti360_00", "kitti360_04", "kitti360_05", "kitti360_06", "kitti360_09"}

    # for dataset_name in names :
    #     file_gt_sens_poses = "/media/zz/new/myMidImg/" + dataset_name + "/out.txt"

    #     file_outcome_osk = "/media/zz/new/myMidImg/result_osk/" +dataset_name+ ".txt"
    #     file_outcome_sc = "/media/zz/new/myMidImg/result_sc/" + dataset_name+".txt"
    #     file_outcome_iris = "/media/zz/new/myMidImg/result_iris/"+dataset_name+".txt"
    #     file_outcome_cont2 = "/media/zz/new/myMidImg/result_cont2/"+dataset_name+".txt"

    #     fig_save_folder = "/home/zz/桌面/fig/no_title"

    #     # file_outcome_std = "/media/zz/new/myMidImg/result_std/kitti_00.txt"
    #     # file_outcome_bow3d = "/media/zz/new/myMidImg/result_bow3d/kitti360_00.txt"

    #     thres_dist = 15.0
    #     thres_frame_dist = 150

    #     fig_name = dataset_name + "_" + str(int(thres_dist))
    #     fig_path = fig_save_folder + "/" + fig_name + ".png"
    #     print(fig_name)

    #     print("Dataset: " + dataset_name + " thresh_dist: " + str(thres_dist))

    #     osk_result = pr.read_loop_detction_result(file_outcome_osk)
    #     sc_result = pr.read_loop_detction_result(file_outcome_sc)
    #     iris_result = pr.read_loop_detction_result(file_outcome_iris)
    #     cont2_result = pr.read_loop_detction_result_cont2(file_outcome_cont2)


    #     auc, maxf1 = pr.comput_auc_maxf1(file_gt_sens_poses, osk_result, thres_dist, thres_frame_dist)