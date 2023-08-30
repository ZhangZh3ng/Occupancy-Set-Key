

---

# 结论

所有结论均基于kitti08中的测试结果

## LSH

* LSH在多线程情况下进入的计算开销较小，因为可以平摊到多线程中，相应的cpu占用率升高(50%)

* 关于LSH的一个比较稳定的参数是

  ```
  <param name="lsh_band_length" type="int" value="6" />
  <param name="lsh_band_num" type="int" value="10" />
  ```

  其中lsh_band_length非常关键，几乎不能让他小于4，并且尽量不让他大于8，否则将带来非常多的误匹配或难以匹配

* 想要仅通过增加lsh_band_num来提升召回率似乎并不好用，测试发现召回率并未发生明显变化

## z_leaf

* 垂直方向上的占有率非常重要，经过测试0.2是一个相当不错的值
* 测试发现0.5及以上时，结果非常差

## Occupancy Context

* 增大描述符感受半径可以获得效果提升，增大半径但减小分辨率也没有关系，依然有提升，这一点可能跟KITTI_08这段数据有关

# Check Points



---

## checkpoint 1:

kitti_08, 表现低于cont2

```
  <param name="xy_leaf_size" type="double" value="0.5" />
  <param name="z_leaf_size" type="double" value="0.2" />

  <param name="landmark_occupancy_threshold" type="int" value="4" />
  <param name="landmark_range_threshold" type="double" value="40" />
  <param name="landmark_mask_radius" type="double" value="3" />

  <param name="lsh_band_length" type="int" value="6" />
  <param name="lsh_band_num" type="int" value="10" />

  <param name="occupancy_context_max_range" type="double" value="20" />
  <param name="occupancy_context_redius_resolution" type="double" value="0.5" />
  <param name="occupancy_context_angle_resolution" type="double" value="6" />
  
  <param name="overlap_grid_size" type="double" value="1" />
  <param name="num_exclude_near_scan" type="int" value="150" />
```

效果：这个是原先最好的参数，召回率可以并且能够以相当大的概率计算出正确的相对位姿

时间：40ms



---

## check point 2:

kitti_08_c2 超越Cont2

kitti_00_c2 上也明显优于cont2

```
  <param name="xy_leaf_size" type="double" value="0.5" />
  <param name="z_leaf_size" type="double" value="0.2" />

  <param name="landmark_occupancy_threshold" type="int" value="4" />
  <param name="landmark_range_threshold" type="double" value="40" />
  <param name="landmark_mask_radius" type="double" value="4" />

  <param name="lsh_band_length" type="int" value="6" />
  <param name="lsh_band_num" type="int" value="10" />

  <param name="occupancy_context_max_range" type="double" value="30" />
  <param name="occupancy_context_redius_resolution" type="double" value="1" />
  <param name="occupancy_context_angle_resolution" type="double" value="6" />

  <param name="overlap_grid_size" type="double" value="1" />
  <param name="num_exclude_near_scan" type="int" value="150" />
```

效果：增大了感受范围，同时减低了半径方向上的分辨率，性能提高了

时间：24ms



# Testing versions



---

## v1

kitti_08_2:

```
  <param name="xy_leaf_size" type="double" value="1" />
  <param name="z_leaf_size" type="double" value="0.2" />

  <param name="landmark_occupancy_threshold" type="int" value="4" />
  <param name="landmark_range_threshold" type="double" value="40" />
  <param name="landmark_mask_radius" type="double" value="3" />

  <param name="lsh_band_length" type="int" value="6" />
  <param name="lsh_band_num" type="int" value="10" />

  <param name="occupancy_context_max_range" type="double" value="20" />
  <param name="occupancy_context_redius_resolution" type="double" value="0.5" />
  <param name="occupancy_context_angle_resolution" type="double" value="6" />

  <param name="overlap_grid_size" type="double" value="1" />
  <param name="num_exclude_near_scan" type="int" value="150" />
```

效果：不好，召回率非常低

速度：13ms

---

## v2

(已废弃):

```
  <param name="xy_leaf_size" type="double" value="1" />
  <param name="z_leaf_size" type="double" value="0.5" />

  <param name="landmark_occupancy_threshold" type="int" value="4" />
  <param name="landmark_range_threshold" type="double" value="40" />
  <param name="landmark_mask_radius" type="double" value="3" />

  <param name="lsh_band_length" type="int" value="6" />
  <param name="lsh_band_num" type="int" value="10" />

  <param name="occupancy_context_max_range" type="double" value="20" />
  <param name="occupancy_context_redius_resolution" type="double" value="0.5" />
  <param name="occupancy_context_angle_resolution" type="double" value="6" />

  <param name="overlap_grid_size" type="double" value="1" />
  <param name="num_exclude_near_scan" type="int" value="150" />
```

效果：不好，召回率几乎没有

时间：6ms

---

## v3

kitti_08_3:

```
  <param name="xy_leaf_size" type="double" value="0.5" />
  <param name="z_leaf_size" type="double" value="0.2" />

  <param name="landmark_occupancy_threshold" type="int" value="4" />
  <param name="landmark_range_threshold" type="double" value="40" />
  <param name="landmark_mask_radius" type="double" value="3" />

  <param name="lsh_band_length" type="int" value="3" />
  <param name="lsh_band_num" type="int" value="10" />

  <param name="occupancy_context_max_range" type="double" value="20" />
  <param name="occupancy_context_redius_resolution" type="double" value="0.5" />
  <param name="occupancy_context_angle_resolution" type="double" value="6" />

  <param name="overlap_grid_size" type="double" value="1" />
  <param name="num_exclude_near_scan" type="int" value="150" />
```

效果：还行，但整体来说不如cont2

时间：45ms

---

## v4

已废弃

```
  <param name="xy_leaf_size" type="double" value="0.5" />
  <param name="z_leaf_size" type="double" value="0.5" />

  <param name="landmark_occupancy_threshold" type="int" value="4" />
  <param name="landmark_range_threshold" type="double" value="40" />
  <param name="landmark_mask_radius" type="double" value="3" />

  <param name="lsh_band_length" type="int" value="4" />
  <param name="lsh_band_num" type="int" value="10" />

  <param name="occupancy_context_max_range" type="double" value="20" />
  <param name="occupancy_context_redius_resolution" type="double" value="0.5" />
  <param name="occupancy_context_angle_resolution" type="double" value="6" />

  <param name="overlap_grid_size" type="double" value="1" />
  <param name="num_exclude_near_scan" type="int" value="150" />
```

效果：一般

时间：10ms

---

## v5


```
<param name="xy_leaf_size" type="double" value="0.5" />
  <param name="z_leaf_size" type="double" value="0.5" />

  <param name="landmark_occupancy_threshold" type="int" value="2" />
  <param name="landmark_range_threshold" type="double" value="40" />
  <param name="landmark_mask_radius" type="double" value="4" />

  <param name="lsh_band_length" type="int" value="10" />
  <param name="lsh_band_num" type="int" value="20" />

  <param name="occupancy_context_max_range" type="double" value="20" />
  <param name="occupancy_context_redius_resolution" type="double" value="0.5" />
  <param name="occupancy_context_angle_resolution" type="double" value="6" />

  <param name="overlap_grid_size" type="double" value="1" />
  <param name="num_exclude_near_scan" type="int" value="150" />
```

效果：在寻找特征点时能够以一定概率找回，但无法给出相对位姿

时间：18ms

---
## v6

已废弃

```
<param name="xy_leaf_size" type="double" value="0.5" />
  <param name="z_leaf_size" type="double" value="0.5" />

  <param name="landmark_occupancy_threshold" type="int" value="2" />
  <param name="landmark_range_threshold" type="double" value="40" />
  <param name="landmark_mask_radius" type="double" value="4" />

  <param name="lsh_band_length" type="int" value="6" />
  <param name="lsh_band_num" type="int" value="20" />

  <param name="occupancy_context_max_range" type="double" value="20" />
  <param name="occupancy_context_redius_resolution" type="double" value="0.5" />
  <param name="occupancy_context_angle_resolution" type="double" value="6" />

  <param name="overlap_grid_size" type="double" value="1" />
  <param name="num_exclude_near_scan" type="int" value="150" />
```

效果：还可以，但召回率稍微低于checkpoint 1

时间：17ms

---

## v7

已废弃

```
<param name="xy_leaf_size" type="double" value="0.5" />
  <param name="z_leaf_size" type="double" value="0.5" />

  <param name="landmark_occupancy_threshold" type="int" value="2" />
  <param name="landmark_range_threshold" type="double" value="40" />
  <param name="landmark_mask_radius" type="double" value="4" />

  <param name="lsh_band_length" type="int" value="10" />
  <param name="lsh_band_num" type="int" value="100" />

  <param name="occupancy_context_max_range" type="double" value="20" />
  <param name="occupancy_context_redius_resolution" type="double" value="0.5" />
  <param name="occupancy_context_angle_resolution" type="double" value="6" />

  <param name="overlap_grid_size" type="double" value="1" />
  <param name="num_exclude_near_scan" type="int" value="150" />
```

效果：不如checkpoint，召回率低

