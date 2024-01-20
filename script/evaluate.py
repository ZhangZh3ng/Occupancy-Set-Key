import matplotlib.pyplot as plt
import pr_tools as pr

if __name__ == "__main__":
    file_gt_sens_poses = "/media/zz/new/myMidImg/kitti_00/out.txt"

    file_outcome_osk = "/media/zz/new/myMidImg/result_osk/kitti_00_1.txt"
    file_outcome_sc = "/media/zz/new/myMidImg/result_sc/kitti_00.txt"
    file_outcome_iris = "/media/zz/new/myMidImg/result_iris/kitti_00.txt"
    file_outcome_cont2 = "/media/zz/new/myMidImg/result_cont2/kitti_00.txt"
    file_outcome_ot = "/media/zz/new/myMidImg/result_ot/kitti_00.txt"
    file_outcome_logg3d = "/media/zz/new/myMidImg/result_logg3d/kitti_00.txt"

    flag_save_result_image = True
    fig_save_folder = "/home/zz/桌面/fig/fig2"

    dataset_name = "kitti_00"
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
    ot_result = pr.read_loop_detction_result(file_outcome_ot)
    logg3d_result = pr.read_loop_detction_result(file_outcome_logg3d)

    pr_results = [
        pr.comput_pr_points(file_gt_sens_poses, osk_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, sc_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, iris_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, cont2_result, thres_dist, thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, ot_result, thres_dist,
        thres_frame_dist),
        pr.comput_pr_points(file_gt_sens_poses, logg3d_result, thres_dist,
        thres_frame_dist)
    ]
    
    data_names = [
        "OSK(Ours)",
        "SC",
        "Iris",
        "Cont2",
        "OT",
        "Logg3D"
    ]

    title = dataset_name + "  th:" + str(thres_dist) + "m"
    title = ""

    fig = pr.plot_pr_curves(pr_results, data_names, title, use_label_mark=True)

    if flag_save_result_image:
        fig.savefig(fig_path)
    
    plt.show()