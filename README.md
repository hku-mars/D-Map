# D-Map: Occupancy Grid Mapping Framework without Ray-Casting for High-resolution LiDAR Sensors

## Introduction
**D-Map** is an efficient occupancy mapping framework for high-resolution LiDAR sensors. The framework introduces three main novelties to address the computational efficiency challenges of occupancy mapping.

- D-Map uses a depth image to determine the occupancy state of regions instead of the traditional ray-casting method. 
- D-Map utilizes an efficient on-tree update strategy on a tree-based map structure. These two techniques avoid redundant visits to small cells, significantly reducing the number of cells to be updated. 
- D-Map removes known cells from the map at each update by leveraging the low false alarm rate of LiDAR sensors. This approach not only enhances our framework's update efficiency by reducing map size but also endows it with an interesting decremental property.

## Framework Overview

<img src="documents/imgs/SystemOverview.png" style="zoom:50%;" />

The map structure of D-Map consists of two parts: the occupied map and the unknown map. The green block represents the pipeline of the occupancy update strategy. At each update, a depth image is rasterized from the incoming point clouds at the sensor pose. Subsequently, a 2-D segment tree is constructed on the depth image to enable efficient occupancy state determination. The cell extraction module retracts the unknown cells on the octree from the largest to the smallest size, projects them to the depth image, and determines their occupancy states. The cells determined as known are directly removed from the map, while the unknown ones remain, and the undetermined ones are split into smaller cells for further occupancy state determination. 

## Our Paper and Accompanying Video

- Our paper "Occupancy Grid Mapping without Ray-Casting for High-resolution Sensors" is accepted for publication in **Transactions on Robotics (TRO)**. [[Preprint]](https://arxiv.org/pdf/2307.08493.pdf)

- The video showcasing two real-world applications is available on [Youtube](https://youtu.be/m5QQPbkYYnA).


If you are using any code of this repo in your research, please cite our paper as following:
```
@misc{cai2023occupancy,
      title={Occupancy Grid Mapping without Ray-Casting for High-resolution LiDAR Sensors}, 
      author={Yixi Cai and Fanze Kong and Yunfan Ren and Fangcheng Zhu and Jiarong Lin and Fu Zhang},
      year={2023},
      eprint={2307.08493},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Build & Run demo
### 1. How to build this project
```bash
cd ~/catkin_ws/src
git clone git@github.com:hku-mars/D-Map.git
cd ..
catkin_make
source devel/setup.bash
```
### 2. Run rosbag examples
#### 2.1 Workshop Dataset
- Please download our Workshop dataset via [google drive](https://drive.google.com/file/d/1dbfx9w1tMUrPm7kloF0LSZVorRE8ruoZ/view?usp=sharing).
- Run the example launch with rosbag as follows:

```bash
roslaunch dmap DMap_example.launch
rosbag play workshop.bag
```
#### 2.2 Kitti Dataset
- We provide an example to run D-Map on spinning LiDARs using the 04 sequence in [Kitti Dataset for Odometry Evaluation](https://www.cvlibs.net/datasets/kitti/eval_odometry.php). Please download the rosbag from [google drive](https://drive.google.com/file/d/1AOFcfXf62vhC9g5V--TZQzzBuKPXLtsR/view?usp=sharing) which includes the velodyne laser data and ground truth odometry. 
- Run the velodyne launch with rosbag as follows:
```bash
roslaunch dmap DMap_velodyne.launch
rosbag play kitti_04.bag
```
## Guidelines for Parameters
The parameters for D-Map are included in the config yaml file and the launch file. We summarize the parameters as below with a brief description.
### LiDAR Sensor Configuration
 The configurations required are listed below:
- ```FOV_shape```: Set 0 for normal FoV shape (e.g., rotating scanning LiDARs); Set 1 for corn shape FoV (e.g., avia LiDAR).
- ```FOV_theta_range```: The horizontal field of view of LiDAR (Unit: degree).
- ```FOV_phi_range```: The vertical field of view of LiDAR (Unit: degree).
- ```FOV_depth```: The detection range of LiDAR (Unit: meter).
- ```sensor_res_hor```: The horizontal angular resolution of LiDAR (Unit: degree).
- ```sensor_res_vert```: The vertical angular resolution of LiDAR (Unit: degree).

Note: The parameters should be provided according to the LiDAR's manual sheet. Inappropriate settings might lead to unsatisfying mapping performance.

### Odometry & Point Cloud Input
- ```point_frame```: Set this param as ```body``` or ```world``` for point cloud provided in the body frame or in the world frame.
- ```lidar_topics```: The topic name for LiDAR point cloud.
- ```odom_topic```: The topic name for odometry.
- ```odom_topic_types```: Set this param as ```odometry``` for ros topic type of ```nav_msgs::Odometry``` or ```pose``` for ros topic type .```geometry_msgs::PoseStamped```.
- ```fixed_frame```: The frame_id to publish the visualization msgs (e.g., publishing octree and grid map).

### D-Map
- ```environment```: The bounding box for D-Map (in **yaml** file).
- ```map_res_min```: The map resolution for D-Map.
- ```map_res_init```: The initial cell size for occupancy state determination to avoid unnecessary queries (Unit: meter, see algorithm 2 in our paper). Default value is ```5.0```. We suggest using a smaller value for indoor environments (e.g., ```2.0```).
- ```full_ratio```: The threshold for observation completeness (See algorithm 1 in our paper). Default value is ```0.9```.
- ```depthmap_accuracy```: The relaxed factor $\gamma$ to trade off between efficiency and accuracy. (See Section IV-D of our paper). Default value is ```1.0``` for no relaxation.
- ```sliding_en``` and ```sliding_thres```: The switch and threshold for map region sliding. The map will slide when odometry moves beyond the sliding threshold from last move. 

### Auxiliary Params
- ```print_en```: Enable the full print of D-Map time consumption of each module. 
- ```log_map``` and ```log_name```: The switch and file name to output D-map.
- ```vis_en```: Enable rviz for visualization.

## Notes

D-Map requires a projection of point cloud onto a depth image. Please confirm that you have a correct depth image via visualization in rviz. You can check whether D-Map is working normally by:
- The depth image is correct (ros topic: ```/dmap_depthmap``` ). 
- At the first update, the shape of the octree map (ros topic: ```/dmap_octree``` ) is consistent to the shape of LiDAR FoV. 
- The octree map (ros topic: ```/dmap_octree``` ) and the grid map (ros topic: ```/dmap_gridmap``` ) show a consistent map.

## License
The source code of D-Map is released under [GPLv2](http://www.gnu.org/licenses/old-licenses/gpl-2.0.html) license. For commercial use, please contact Mr. Yixi CAI (<yixicai@connect.hku.hk>) or Dr. Fu ZHANG (<fuzhang@hku.hk>).
