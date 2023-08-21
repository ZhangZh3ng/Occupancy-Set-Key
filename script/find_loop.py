import numpy as np
import re
import os
import csv
import math
from scipy.spatial import KDTree

# to exclude scan near in time
thres_ts = 15.0
# if two scans' relative translation less than it, regard them as a loop
thres_dist = 5.0


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


def get_gt_sens_timestamp(fpath_gt_sens_poses) :
    res = []
    with open(fpath_gt_sens_poses, "r") as fp:
        lines = fp.readlines()

    for line in lines:
        row_data = line.strip().split()
        assert len(row_data) == 13
        res.append(float(row_data[0]))
    return res


'''
find all possible loop, not limit the nearest loop. When condition
thres_ts and thres_dist are fitted, we deem it as a loop and save it.
output file has the following format:

# curr_scan_id_0
lc_id_0 lc_T_curr (1x12, in kitti format)
lc_id_1 lc_T_curr
# curr_scan_id_1
lc_id_0 lc_T_curr
...
'''
def generate_loop_file(fp_pose, fp_result) :
    ts = get_gt_sens_timestamp(fp_pose)
    gt_pose = get_gt_sens_poses(fp_pose)

    gt_points = gt_pose[:, [3, 7, 11]]
    tree = KDTree(gt_points)

    num_scan_has_lc = 0
    num_total_valid_lc = 0

    with open(fp_result, "w") as fout:
        for i in range(gt_pose.shape[0]):
            has_write_this_id = 0
            near_points = tree.query_ball_point(gt_points[i, :], thres_dist)
            for j in near_points:
                if (ts[i] - ts[j]) < thres_ts:
                    continue

                if has_write_this_id == 0:
                    fout.write("@ " + str(i) + "\n")
                    has_write_this_id = 1
                    num_scan_has_lc += 1

                num_total_valid_lc += 1
                T0 = np.vstack(
                    [np.array(gt_pose[i]).reshape((3, 4)),
                     np.array([[0, 0, 0, 1]])])
                T1 = np.vstack(
                    [np.array(gt_pose[j]).reshape((3, 4)),
                     np.array([[0, 0, 0, 1]])])
                dT = np.linalg.inv(T1) @ T0
                tmp = dT.reshape(1, 16)[0, 0:12]
                n = "{:5d}".format(j)
                fout.write(n)
                for val in tmp :
                    n = "{:8.5f}".format(val)
                    fout.write(" " + n)
                fout.write("\n")
            print(str(i))

    print('search done, %d scans has loop, totally has %d loops' %
          (num_scan_has_lc, num_total_valid_lc))
    return
    

if __name__ == "__main__":
    fp_pose = "/media/zz/new/cont2_gen/ts-sens_pose-kitti08_sementic.txt"
    fp_result = "/media/zz/new/cont2_gen/loops-kitti08_sementic.txt"

    generate_loop_file(fp_pose, fp_result)