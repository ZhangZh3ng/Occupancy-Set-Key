# Some of the code comes form contour-context(https://github.com/lewisjiang/contour-context)
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from matplotlib.collections import LineCollection


# input data formt: [frame_id(0), timestamp(1), pose(2-13)]


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

# Note: If algorithm didn't find a loop, we always set idx_best = 0 and score = 0


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
            else:
                idx_best = -1

            coordinates = np.array([0, 0, 0])
            if (len(line_info) >= 6):
                coordinates[0] = float(line_info[3])
                coordinates[1] = float(line_info[4])
                coordinates[2] = float(line_info[5])
            dist = np.linalg.norm(coordinates)
            result.append([idx_curr, idx_best, score, dist])
    return result


def comput_pr_points_ts(fp_gt_sens_poses, outcome, thres_dist=10, thres_time=30):
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


# Note: this function only work on sequence scan id, and the first scan id must be 0 that is consistent with our xxx.out.txt format
def comput_pr_points(fp_gt_sens_poses, outcome, thres_dist=10, thres_frame_dist=300):
    plots_data = []
    pr_points = []

    gt_pose, frame_ids, timestamps = get_gt_sens_poses(fp_gt_sens_poses)

    # gt_positive indicate if this scan has a positive loop pair
    gt_positive = np.zeros(gt_pose.shape[0])
    gt_points = gt_pose[:, [3, 7, 11]]
    tree = KDTree(gt_points)

    for i in range(gt_pose.shape[0]):
        near_points = tree.query_ball_point(gt_points[i, :], thres_dist)
        for j in near_points:
            if j < i - thres_frame_dist:
                gt_positive[i] = 1
                break

    est = []
    for idx_curr, idx_best, score, dist in outcome:
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


# Note: this function only work on sequence scan id, and the first scan id must be 0 that is consistent with our xxx.out.txt format
def comput_maxf1_result(fp_gt_sens_poses, outcome, maxf1_score, thres_dist=10, thres_frame_dist=300):
    result = []
    gt_pose, frame_ids, timestamps = get_gt_sens_poses(fp_gt_sens_poses)

    # gt_positive indicate if this scan has a positive loop pair
    gt_positive = np.zeros(gt_pose.shape[0])
    gt_points = gt_pose[:, [3, 7, 11]]
    tree = KDTree(gt_points)

    for i in range(gt_pose.shape[0]):
        near_points = tree.query_ball_point(gt_points[i, :], thres_dist)
        for j in near_points:
            if j < i - thres_frame_dist:
                gt_positive[i] = 1
                break

    for idx_curr, idx_best, score, dist in outcome:
        judgement_is_positive = True
        if score < maxf1_score:
            judgement_is_positive = False

        find_true_loop = True
        if np.linalg.norm(gt_pose[idx_curr].reshape(3, 4)[:, 3] -
                          gt_pose[idx_best].reshape(3, 4)[:, 3]) > thres_dist:
            find_true_loop = False

        has_true_loop = gt_positive[idx_curr]
        gt_point = gt_points[idx_curr]

        # True Positive: 0    has true loop, and find correct true loop
        # False Positive: 1   find a loop, but it is not true loop
        # True Negative: 2    no true loop, and report no loop
        # False Negative: 3   has true loop, but not find it
        type = 2
        if has_true_loop:
            if judgement_is_positive and find_true_loop:
                # has TP and find it
                type = 0
            else:
                # has TP but not find
                type = 3
        else:
            if judgement_is_positive:
                # no TP but report one positive
                type = 1

        this_result = [gt_point[0], gt_point[1], gt_point[2], type]
        result.append(this_result)

    return result


def get_pr_points_and_max_f1_loops(fp_gt_sens_poses, fp_outcome, thres_dist, thres_frame_dist=300):
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
            if j < i - thres_frame_dist:  # block nearest scans.
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


def plot_pr_curves(pr_datas, curve_names, image_title="outcome", use_legend=True):
    fig, axes = plt.subplots(1, 1, figsize=(12, 9))

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

        if use_legend :
            ax.legend(used_names, loc=3)

    return fig


def visualize_pr_trajectory(trajectory, title="", use_legend=True, use_grid=True):
    # Extract line segments and colors for LineCollection
    lines = []
    colors = []

    for i in range(len(trajectory) - 1):
        line_points = np.array([[trajectory[i][0], trajectory[i][1]], [
                               trajectory[i + 1][0], trajectory[i + 1][1]]])
        lines.append(line_points)
        colors.extend([trajectory[i][3]])

    # Define color map for the conditions
    condition_colors = {
        0: 'g',  # True Positive: Green
        1: 'r',  # False Positive: Red
        2: 'b',  # True Negative: Blue
        3: 'y'   # False Negative: Yellow
    }

    # Create a LineCollection
    line_segments = LineCollection(
        lines, colors=[condition_colors[color] for color in colors], linewidths=2)

    # Create a scatter plot for the endpoints (optional)
    endpoints_x = np.array([line[-1][0] for line in lines])
    endpoints_y = np.array([line[-1][1] for line in lines])
    plt.scatter(endpoints_x, endpoints_y, color=[
                condition_colors[color] for color in colors], s=10, alpha=0.8, linewidths=0.1)

    # Customize plot appearance
    plt.xlabel('X Coordinate (m)')
    plt.ylabel('Y Coordinate (m)')
    plt.title(title)
    plt.grid(use_grid)

    # Create legend for condition colors
    legend_labels = {
        0: 'True Positive',
        1: 'False Positive',
        2: 'True Negative',
        3: 'False Negative'
    }
    handles = [plt.Line2D([], [], marker='o', color='w',
                          markerfacecolor=condition_colors[i], label=legend_labels[i]) for i in range(4)]
    if (use_legend) :
        plt.legend(handles=handles, title='Conditions', loc='upper left')

    # Add LineCollection to the plot
    plt.gca().add_collection(line_segments)

    # Show plot
    plt.show()


def visualize_pr_trajectory_3d(trajectory, z_rate=0.1, use_legend=True):
    x_coords = []
    y_coords = []
    z_coords = []
    conditions = []
    id = 0
    for line in trajectory:
        x_coords.append(line[0])
        y_coords.append(line[1])
        # Compute z coordinate based on rate and id (row index)
        z_coords.append(z_rate * id)  
        conditions.append(line[3])
        id += 1

    condition_colors = {
        0: 'g',  # True Positive: Green
        1: 'r',  # False Positive: Red
        2: 'b',  # True Negative: Blue
        3: 'y'   # False Negative: Yellow
    }

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the trajectory as a colored line
    for i in range(len(x_coords) - 1):
        ax.plot(x_coords[i:i+2], y_coords[i:i+2], z_coords[i:i+2], color=condition_colors[conditions[i]], linewidth=2)

    ax.set_xlabel('X Coordinate (m)')
    ax.set_ylabel('Y Coordinate (m)')
    ax.set_zlabel('Timestamp (s)')
    ax.set_title('3D Trajectory Plot with Colored Path')

    # Create legend for condition colors
    legend_labels = {
        0: 'True Positive',
        1: 'False Positive',
        2: 'True Negative',
        3: 'False Negative'
    }
    handles = [plt.Line2D([], [], color=condition_colors[i], label=legend_labels[i]) for i in range(4)]

    if (use_legend) :
        plt.legend(handles=handles, title='Conditions', loc='upper left')

    plt.show()