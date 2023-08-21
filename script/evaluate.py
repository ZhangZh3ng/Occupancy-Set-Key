import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import matplotlib
import math
from scipy.spatial import KDTree
import pr_tools as pr

'''
Modified version from contour-context(https://github.com/lewisjiang/contour-context)
We modified our script to make it fit new outcome format.
PR curve is computed using protocol-3 in that paper.
'''


def get_gt_sens_poses(fpath_gt_sens_poses):
    """

    :return: 2d matrix, each row is a 12 dim elements
    """

    with open(fpath_gt_sens_poses, "r") as fp:
        lines = fp.readlines()

    res = []
    for line in lines:
        assert len(line.strip().split()) == 13
        res.append([eval(x) for x in line.strip().split()[1:]])

    return np.vstack(res)

def get_maxf1_idx(data):
    max_f1 = 0
    idx = -1
    max_pt = None
    for d in data:
        cur = 2 * d[0] * d[1] / (d[0] + d[1]) if (d[0] + d[1]) > 0 else 0

        if max_f1 < cur:
            max_f1 = cur
            idx = d[2]
            max_pt = d
    print("Max f1 point: ", max_pt)
    return max_f1, idx


class SimpleRMSE:
    def __init__(self):
        self.sum_sqs = 0
        self.sum_abs = 0
        self.cnt_sqs = 0

    def add_one_error(self, err_vec):
        self.cnt_sqs += 1
        tmp = 0
        for i in err_vec:
            tmp += i ** 2
        self.sum_sqs += tmp
        self.sum_abs += math.sqrt(tmp)

    def get_rmse(self):
        if self.cnt_sqs:
            return math.sqrt(self.sum_sqs / self.cnt_sqs)
        else:
            return -1

    def get_mean(self):
        if self.cnt_sqs:
            return self.sum_abs / self.cnt_sqs
        else:
            return -1


def get_points_ours(fp_gt_sens_poses, fp_outcome, thres_dist):
    print("In ours2")
    plots_data = []

    print(fp_gt_sens_poses)
    print(fp_outcome)
    pr_points = []

    # the sensor poses must be ordered by time/creation/acquisition
    gt_pose = get_gt_sens_poses(fp_gt_sens_poses)
    # gt_positive indicate if this scan has a positive loop pair
    gt_positive = np.zeros(gt_pose.shape[0])
    gt_points = gt_pose[:, [3, 7, 11]]
    tree = KDTree(gt_points)

    for i in range(gt_pose.shape[0]):
        near_points = tree.query_ball_point(gt_points[i, :], thres_dist)
        for j in near_points:
            if j < i - 150:
                gt_positive[i] = 1
                break

    with open(fp_outcome, "r") as f1:
        lines = f1.readlines()
        est = []
        for line in lines:
            line_info = line.strip().split()
            assert len(line_info) > 3

            pairing = line_info[1].split('-')
            idx_curr = int(pairing[0])

            # [score, if_find_tp, is_positive , id_curr]
            est_line = [eval(line_info[2]), 0, 0, idx_curr, -1]
            if pairing[1] != 'x':
                idx_best = int(pairing[1])
                if np.linalg.norm(gt_pose[idx_curr].reshape(3, 4)[:, 3] -
                                  gt_pose[idx_best].reshape(3, 4)[:, 3]) < thres_dist:
                    est_line[1] = 1
                    est_line[4] = idx_best

        # 3. if the overall is P
            est_line[2] = gt_positive[idx_curr]

            est.append(est_line)
            # print(est_line)

        est = np.vstack(est)
        est = est[(-est[:, 0]).argsort()]  # sort by score, larger is better

        tp = 0
        fp = 0
        for i in range(est.shape[0]):
            if est[i, 1]:
                tp += 1
            else:
                fp += 1

            fn = 0
            for j in range(i, est.shape[0]):
                if est[j, 2]:
                    fn += 1

            pr_points.append([tp / (tp + fn), tp / (tp + fp), est[i, 3]])

        # print(pr_points)
        points = np.vstack(pr_points)[:, 0:2]
        points = points[points[:, 0].argsort()]
        plots_data.append(points)

        # get max F1
        max_f1, f1_pose_idx = get_maxf1_idx(pr_points)
        print("Max F1 score: %f @%d " % (max_f1, int(f1_pose_idx)))

        # calc rmse for scores above max f1 sim
        sim_thres = eval(lines[int(f1_pose_idx)].split()[2])
        print("sim thres for Max F1 score: %f" % sim_thres)

        num_scan_has_lc = 0
        for i in gt_positive:
            if i == 1:
                num_scan_has_lc += 1
        print('total %d scans has positive loop pair' % num_scan_has_lc)

    return plots_data

def get_points_sc(fp_gt_sens_poses, fp_outcome, thres_dist):
    print("In ours2")
    plots_data = []

    print(fp_gt_sens_poses)
    print(fp_outcome)
    pr_points = []

    # the sensor poses must be ordered by time/creation/acquisition
    gt_pose = get_gt_sens_poses(fp_gt_sens_poses)
    # gt_positive indicate if this scan has a positive loop pair
    gt_positive = np.zeros(gt_pose.shape[0])
    gt_points = gt_pose[:, [3, 7, 11]]
    tree = KDTree(gt_points)

    for i in range(gt_pose.shape[0]):
        near_points = tree.query_ball_point(gt_points[i, :], thres_dist)
        for j in near_points:
            if j < i - 150:
                gt_positive[i] = 1
                break

    with open(fp_outcome, "r") as f1:
        lines = f1.readlines()
        est = []
        for line in lines:
            line_info = line.strip().split()
            assert len(line_info) > 3

            idx_curr = int(line_info[1])
            idx_best = int(line_info[2])
            score = float(line_info[3])

            # [score, if_find_tp, is_positive , id_curr]
            est_line = [score, 0, 0, idx_curr]

            if np.linalg.norm(gt_pose[idx_curr].reshape(3, 4)[:, 3] -
                              gt_pose[idx_best].reshape(3, 4)[:, 3]) < thres_dist:
                est_line[1] = 1

            # 3. if the overall is P
            est_line[2] = gt_positive[idx_curr]

            est.append(est_line)

        est = np.vstack(est)
        est = est[(-est[:, 0]).argsort()]  # sort by correlation, larger better

        tp = 0
        fp = 0
        for i in range(est.shape[0]):
            if est[i, 1]:
                tp += 1
            else:
                fp += 1

            fn = 0
            for j in range(i, est.shape[0]):
                if est[j, 2]:
                    fn += 1

            pr_points.append([tp / (tp + fn), tp / (tp + fp), est[i, 3]])

        # print(pr_points)
        points = np.vstack(pr_points)[:, 0:2]
        points = points[points[:, 0].argsort()]
        plots_data.append(points)

        # get max F1
        max_f1, f1_pose_idx = get_maxf1_idx(pr_points)
        print("Max F1 score: %f @%d " % (max_f1, int(f1_pose_idx)))

        # calc rmse for scores above max f1 sim
        sim_thres = eval(lines[int(f1_pose_idx)].split()[2])
        print("sim thres for Max F1 score: %f" % sim_thres)

        num_scan_has_lc = 0
        for i in gt_positive:
            if i == 1:
                num_scan_has_lc += 1
        print('total %d scans has positive loop pair' % num_scan_has_lc)

    return plots_data

if __name__ == "__main__":
    file_gt_sens_poses = "/media/zz/new/myMidImg/kitti_00/out.txt"

    file_outcome_osk = "/media/zz/new/myMidImg/search_result/kitti00.txt"
    file_outcome_cont2 = "/media/zz/new/cont2_gen/outcome-kitti02_semantic.txt"
    file_outcome_sc = "/media/zz/new/myresult/sc-kitti02_semantic.txt"

    thres_dist = 15.0
    thres_time = 30

    osk_result = pr.read_loop_detction_result(file_outcome_osk)
    # cont2_result = pr.read_loop_detction_result_cont2(file_outcome_cont2)
    # sc_result = pr.read_loop_detction_result_sc(file_outcome_sc)

    pr_results = [
        pr.comput_pr_points(file_gt_sens_poses, osk_result, thres_dist, thres_time),
        # pr.comput_pr_points(file_gt_sens_poses, cont2_result, thres_dist, thres_time),
        # pr.comput_pr_points(file_gt_sens_poses, sc_result, thres_dist, thres_time)
    ]
    
    data_names = [
        "OSK",
        # "Cont2",
        # "SC"
    ]

    title = "kitti00 threshold = 15"

    pr.plot_pr_curves(pr_results, data_names, title)