# DDDMR LeGO LOAM BOR
This repo is based on the [LeGO-LOAM-BOR ](https://github.com/facontidavide/LeGO-LOAM-BOR), and has been modified for ground vehicles. We did not fork the repo due to significant modifications, variant features and frameworks are implemented.

The original authors deserve all the credits, we just stand on the shoulders of giants.

<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/mapping.gif" width="700" height="420"/>
</p>

The following features are provided and are different from original version:

- ROS2 version (Humble) of LeGO LOAM BOR, support online and offline(bag) mapping, IMU/Odometry is optional.
- Interactive mapping. Users can pause and resume mapping during bag playing, and change parameters accordingly.
- Pose graph visualization.
- Conditional loop closure mechanism for a more stable mapping result.
- The result is saved as a pose graph, users can leverage our pose graph editor to modify the result ex: manual loop closure.
- Our pose graph editor allow users to merge two pose graphs.
- The pose graph can be used in DDDMR MCL localization.

<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/legoloam_weiwuyin.png" width="700" height="320"/>
</p>

Watch more videos on Youtube of 3D navigation stack -> [DDDMobilerobot channel](https://www.youtube.com/@dddmobilerobot9169).

## RUN The Demo
### 1. Create docker image
The package runs in the docker, so we need to build the image first. We support both x64 (tested in intel NUC) and arm64 (tested in nvidia jetson jpack6).
```
cd ~
git clone https://github.com/dddmobilerobot/dddmr_navigation.git
cd ~/dddmr_navigation && git submodule update --init dddmr_docker src/dddmr_lego_loam_bor src/dddmr_rviz_tools
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```
### 2. Download bag files
To play SLAM, you will need to download bag file (4.0GB). To play pose graph editor, you will need to download pose graph folder (2.6MB).
```
cd ~/dddmr_navigation/src/dddmr_lego_loam_bor && ./download_files.bash
```
### 3. Run demo
#### Create a docker container
> [!NOTE]
> The following command will create an interactive docker container using the image we built. The we can launch the demo in the container.
```
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```
#### Play mapping using bag files in docker container
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch lego_loam_bor lego_loam_bag.launch
```
In the Rviz2, click resume to start mapping, and change the parameter accordingly during mapping.

<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/interactive_mapping_panel.png" width="700" height="420"/>
</p>

#### Play pose graph editor in docker container
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch lego_loam_bor pose_graph_editor.launch
```
## Cite *LeGO-LOAM*

Many thanks to *LeGO-LOAM* paper: 
```
@inproceedings{legoloam2018,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Tixiao Shan and Brendan Englot},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```
# Pose Graph Editor Tutorial
## Edit the First Pose Graph
### Open a pose graph folder
<p align="center">
<img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/pose_graph_editor/open_file.gif" width="700" height="320"/>
</p>

### Select first key frame

Press "shift" and select a pose, the selected key frame will be red
<p align="center">
<img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/pose_graph_editor/select_key_frame.gif" width="600" height="280"/>
</p>

### Select second key frame

Select a pose without press any key, the selected key frame will be green
<p align="center">
<img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/pose_graph_editor/select_second_key_frame.gif" width="600" height="280"/>
</p>

### Use ICP to find an edge between the first key frame and the second key frame

The buttons px+, px-, py+, py-, .... are used to fine tune the icp result. When the result is satified, click "Accept", an edge will be added
<p align="center">
<img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/pose_graph_editor/icp.gif" width="600" height="280"/>
</p>

****

## Edit the Second Pose Graph
### Open another pose graph folder
Tick the second radio button so you will be able to work on the second pose graph. All operations will be activated on the second pose graph.
<p align="center">
<img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/pose_graph_editor/second_pg_editor.gif" width="700" height="320"/>
</p>

****
## Merge two pose graphs
#### Switch to merge panel
The Merge ICP button will match the first key frame of each pose graph. i.e.: Merge red key frame of the first pose graph and the red key frame of the second pose graph.
After click accept merge, there will be a pop up window to ask thenew directory to save the merged result.
Once the result is saved, you can go back to pose graph panel to keep working on the merge result.
Steps:
1. Switch to the first pose graph, and pressed "shift" to select the first key frame of pose graph 1.
2. Switch to the second pose graph, and pressed "shift" to select the first key frame of pose graph 2.
3. Tick Merge visualization on the Rviz, then you will see two key frames. You can ICP them and fine tune the result.
4. Click accpet to save the result.
<p align="center">
<img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/pose_graph_editor/merge.gif" width="700" height="320"/>
</p>
