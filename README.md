# DDDMR LeGO LOAM BOR
This repo is based on the [LeGO-LOAM-BOR ](https://github.com/facontidavide/LeGO-LOAM-BOR), and has been modified for ground vehicles. We did not fork the repo due to significant modifications, variant features and frameworks are implemented.

The original authors deserve all the credits, we just stand on the shoulders of giants.

<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/mapping.gif" width="700" height="420"/>
</p>

The following features are provided and are different from original version:

- ROS2 version (Humble) of LeGO LOAM BOR, support online and offline(bag) mapping, IMU/Odometry is optional.
- Pose graph visualization.
- Conditional loop closure mechanism for a more stable mapping result.
- The result is saved as a pose graph, users can leverage our pose graph editor to modify the result ex: manual loop closure.
- Our pose graph editor allow users to merge two pose graphs.
- The pose graph can be used in DDDMR MCL localization.

<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/dddmr_lego_loam_bor/legoloam_weiwuyin.png" width="700" height="320"/>
</p>
Watch more videos on Youtube of 3D navigation stack -> [DDDMobilerobot](https://www.youtube.com/@dddmobilerobot9169)

## RUN The Demo

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
