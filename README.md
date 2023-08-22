# Occupancy Set Key(OSK)

## Run on KITTI

1. In **script/gt_kitti.py**, set the *lidar_folder_path*(lidar .bin file's folder), *timestamp_path*, *calib_cam_to_velo_path*, *cam0_to_world_path* according to the dataset's path on your computer. Then create a folder and assign its path to *output_folder_path*. After these change run this python script, you will get some output file under *output_folder_path* including a *xxx/out.txt*, please copy the complete path of *xxx/out.txt* .
2. In **launch/run_osk.launch**, set the *lidar_info_path* using *xxx/out.txt* and set the *result_save_path* using a .txt file path.
3. In *shell/* run *run.sh*

## Run On KITTI360

1. The only difference with Run on KITTI is step 1, Please use *script/gt_kitti360.py* and set the four path according to your dataset's path

## Evaluation

In *script/evaluation.py*, set the *file_gt_sens_poses* using *xxx/out.txt* and set *file_outcome_osk* using the *result_save_path* then run this python script.