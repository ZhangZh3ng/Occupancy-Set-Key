import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree


def get_gt_sens_poses(fpath_gt_sens_poses):
    with open(fpath_gt_sens_poses, "r") as fp:
        lines = fp.readlines()

    timestamps = []
    frame_ids = []
    pose = []
    for line in lines:
        assert len(line.strip().split()) >= 14
        elements = line.strip().split()
        frame_ids.append(eval(elements[0]))
        timestamps.append(eval(elements[1]))
        pose.append([eval(x) for x in elements[2:14]])

    return np.vstack(pose), frame_ids, timestamps


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
    return max_f1, idx


def read_loop_detction_result(file_path):
    result = []
    with open(file_path, "r") as f1:
        lines = f1.readlines()
        for line in lines:
            line_info = line.strip().split()
            assert len(line_info) >= 3

            idx_curr = int(line_info[0])
            idx_best = int(line_info[1])
            score = float(line_info[2])
            coordinates = np.array([0, 0, 0])
            if (len(line_info) >= 14):
                coordinates[0] = float(line_info[6])
                coordinates[1] = float(line_info[10])
                coordinates[2] = float(line_info[14])
            dist = np.linalg.norm(coordinates)
            result.append([idx_curr, idx_best, score, dist])
    return result


def read_loop_detction_result_sc(file_path):
    result = []
    with open(file_path, "r") as f1:
        lines = f1.readlines()
        for line in lines:
            line_info = line.strip().split()
            assert len(line_info) > 3

            idx_curr = int(line_info[1])
            idx_best = int(line_info[2])
            score = float(line_info[3])
            result.append([idx_curr, idx_best, score])
    return result


def read_loop_detction_result_cont2(file_path):
    result = []
    with open(file_path, "r") as f1:
        lines = f1.readlines()
        for line in lines:
            line_info = line.strip().split()
            assert len(line_info) > 3

            pairing = line_info[1].split('-')
            idx_curr = int(pairing[0])
            score = eval(line_info[2])
            if pairing[1] != 'x':
                idx_best = int(pairing[1])
            else :
                idx_best = -1
            result.append([idx_curr, idx_best, score])
    return result


def comput_pr_points(fp_gt_sens_poses, outcome, thres_dist=10, thres_time=30):
    plots_data = []
    pr_points = []

    # the sensor poses must be ordered by time/creation/acquisition
    gt_pose, frame_ids, timestamps = get_gt_sens_poses(fp_gt_sens_poses)

    # gt_positive indicate if this scan has a positive loop pair
    gt_positive = np.zeros(gt_pose.shape[0])
    gt_points = gt_pose[:, [3, 7, 11]]
    tree = KDTree(gt_points)

    id_map = {}
    for index, element in enumerate(frame_ids):
        id_map[element] = index

    for i in range(gt_pose.shape[0]):
        near_points = tree.query_ball_point(gt_points[i, :], thres_dist)
        for j in near_points:
            if timestamps[j] < timestamps[i] - thres_time:
                gt_positive[i] = 1
                break

    est = []
    for idx_curr, idx_best, score, dist in outcome:
        idx_curr = id_map[idx_curr]
        if idx_best != -1:
            idx_best = id_map[idx_best]

        # [score, if_find_tp, is_positive , id_curr, id_match]
        est_line = [score, 0, 0, idx_curr, int(-1)]

        if np.linalg.norm(gt_pose[idx_curr].reshape(3, 4)[:, 3] -
                          gt_pose[idx_best].reshape(3, 4)[:, 3]) < thres_dist:
            est_line[1] = 1

        # 3. if the overall is P
        if dist < thres_dist:
            est_line[2] = gt_positive[idx_curr]
            est_line[4] = idx_best

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

    points = np.vstack(pr_points)[:, 0:2]
    points = points[points[:, 0].argsort()]
    plots_data.append(points)

    # get max F1
    max_f1, f1_pose_idx = get_maxf1_idx(pr_points)
    print("Max F1 score: %f @%d " % (max_f1, int(f1_pose_idx)))

    # calc rmse for scores above max f1 sim
    sim_thres = (outcome[int(f1_pose_idx)][2])
    print("sim thres for Max F1 score: %f" % sim_thres)

    num_scan_has_lc = 0
    for i in gt_positive:
        if i == 1:
            num_scan_has_lc += 1
    print('total %d scans has positive loop pair' % num_scan_has_lc)

    return plots_data


def get_pr_points(fp_gt_sens_poses, fp_outcome, thres_dist):
    plots_data = []

    print(fp_gt_sens_poses)
    print(fp_outcome)
    pr_points = []

    # the sensor poses must be ordered by time/creation/acquisition
    gt_pose, frame_ids, timestamps = get_gt_sens_poses(fp_gt_sens_poses)
    # gt_positive indicate if this scan has a positive loop pair
    gt_positive = np.zeros(gt_pose.shape[0])
    gt_points = gt_pose[:, [3, 7, 11]]
    tree = KDTree(gt_points)

    print(frame_ids)
    id_map = {}
    for index, element in enumerate(frame_ids):
        print(index, element)
        id_map[element] = index

    for i in range(gt_pose.shape[0]):
        near_points = tree.query_ball_point(gt_points[i, :], thres_dist)
        for j in near_points:
            if timestamps[j] < timestamps[i] - 30:
                gt_positive[i] = 1
                break

    with open(fp_outcome, "r") as f1:
        lines = f1.readlines()
        est = []
        for line in lines:
            line_info = line.strip().split()
            assert len(line_info) > 3

            idx_curr = int(line_info[0])
            idx_best = int(line_info[1])
            idx_curr = id_map[idx_curr]
            if idx_best != -1:
                idx_best = id_map[idx_best]
            score = float(line_info[2])

            # [score, if_find_tp, is_positive , id_curr, id_match]
            est_line = [score, 0, 0, idx_curr, int(-1)]

            if np.linalg.norm(gt_pose[idx_curr].reshape(3, 4)[:, 3] -
                              gt_pose[idx_best].reshape(3, 4)[:, 3]) < thres_dist:
                est_line[1] = 1

            # 3. if the overall is P
            est_line[2] = gt_positive[idx_curr]
            est_line[4] = idx_best

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


def get_pr_points_and_max_f1_loops(fp_gt_sens_poses, fp_outcome, thres_dist):
    pr_data = []

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
            if j < i - 300:  # block nearest 300 scans.
                gt_positive[i] = 1
                break

    with open(fp_outcome, "r") as f1:
        lines = f1.readlines()
        est = []
        for line in lines:
            line_info = line.strip().split()
            assert len(line_info) > 3

            idx_curr = int(line_info[0])
            idx_best = int(line_info[1])
            score = float(line_info[2])

            # [score, if_find_tp, is_positive , id_curr, id_match]
            est_line = [score, 0, 0, idx_curr, int(-1)]

            if np.linalg.norm(gt_pose[idx_curr].reshape(3, 4)[:, 3] -
                              gt_pose[idx_best].reshape(3, 4)[:, 3]) < thres_dist:
                est_line[1] = 1

            # 3. if the overall is P
            est_line[2] = gt_positive[idx_curr]
            est_line[4] = idx_best

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

            # [recall, precision, curr_id, score]
            pr_points.append(
                [tp / (tp + fn), tp / (tp + fp), est[i, 3], est[j, 0]])

        # print(pr_points)
        points = np.vstack(pr_points)[:, 0:2]
        points = points[points[:, 0].argsort()]
        pr_data.append(points)

        print(pr_points[0])

        # get max F1
        max_f1, f1_pose_idx = get_maxf1_idx(pr_points)
        print("Max F1 score: %f @%d " % (max_f1, int(f1_pose_idx)))

        # find corresponding similarity threshold of max F1
        sim_thres = eval(lines[int(f1_pose_idx)].split()[2])
        print("sim thres for Max F1 score: %f" % sim_thres)

        # get best match pairs
        detect_loop_pairs = []
        for i in range(est.shape[0]):
            if est[i, 0] > sim_thres:
                if (est[i, 2]):
                    detect_loop_pairs.append(
                        [1, int(est[i, 3]), int(est[i, 4])])
                else:
                    detect_loop_pairs.append(
                        [0, int(est[i, 3]), int(est[i, 4])])

        num_scan_has_lc = 0
        for i in gt_positive:
            if i == 1:
                num_scan_has_lc += 1
        print('total %d scans has positive loop pair' % num_scan_has_lc)

    return pr_data, detect_loop_pairs

# gt_poses: Nx12, each row is a tranformation matrix(upper 3x4 part)
# detected_loops: Mx3, [is_tp, curr_scan_id, match_scan_id]


def plot_trajectory_with_loop_mark(gt_poses, detected_loops, thres_dist):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    gt_positive = np.zeros(gt_poses.shape[0])
    gt_points = gt_poses[:, [3, 7, 11]]
    tree = KDTree(gt_points)

    for i in range(gt_poses.shape[0]):
        near_points = tree.query_ball_point(gt_points[i, :], thres_dist)
        for j in near_points:
            if j < i - 300:
                gt_positive[i] = 1
                break

    # We use z label indicate if has true loop in ground truth since
    # vertical movement often irrelevance in place recognition task.
    x, y, z = [], [], []
    for i in range(gt_poses.shape[0]):
        pose = gt_poses[i]
        x.append(pose[3])
        y.append(pose[7])
        if gt_positive[i]:
            z.append(1)
        else:
            z.append(0)

    # If TP draw a green line, if FP draw a red line
    for loop in detected_loops:
        color = 'r'
        if (loop[0]):
            color = 'g'

        id1 = loop[1]
        id2 = loop[2]
        ax.plot([x[id1], x[id2]], [y[id1], y[id2]], [z[id1], z[id2]], c=color)

    ax.plot(x, y, z)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_zlim(-1, 3)
    plt.show()


def plot_pr_curves(pr_datas, curve_names, image_title="outcome"):
    fig, axes = plt.subplots(1, 1, figsize=(18, 6))

    assert len(pr_datas) == len(curve_names)

    for i in range(1):
        ax = axes

        ax.set_xlabel('Recall')
        ax.set_ylabel('Precision')

        ax.set_xlim([0, 1.02])
        ax.set_ylim([0, 1.02])

        ax.set_title(image_title)
        ax.tick_params(axis="y", direction="in")
        ax.tick_params(axis="x", direction="in")

        used_names = []
        used_colors = []

        for j, data1 in enumerate(pr_datas):
            if data1[i].size == 0:
                continue
            ax.plot(data1[i][:, 0], data1[i][:, 1], color="C%d" % (9 - j))
            used_names.append(curve_names[j])
            used_colors.append("C%d" % (9 - j))

        ax.legend(used_names, loc=3)

    plt.show()
