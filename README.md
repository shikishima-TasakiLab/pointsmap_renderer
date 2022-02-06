# Pointsmap Renderer

## Docker Image

- pull
    ```bash
    docker pull shikishimatasakilab/pointsmap_renderer:melodic
    ```
- build
    ```bash
    ./docker/build.sh
    ```

## Start a Docker Container

1. Start a Docker container with the following command. 
    ```bash
    ./docker/run.sh
    ```

1. Build source code.
    ```bash
    catkin build
    source /workspace/devel/setup.bash
    ```

## How to use

### Configuration File

Write the following configuration to the YAML file.

Ex) sample.yaml
```yaml
voxel_size: 10.0
map_frameid: map
init_maps:
  - /path/to/pointsmap.pcd
```

|Key|Value|Default|
|----|--|------|
|`voxel_size`|The length of a side of the voxel that stores the point cloud maps. [m]|`10.0`|
|`init_maps`|List of point cloud map paths to be loaded at startup.|None|
|`map_frameid`|Coordinate system of the point cloud maps to be loaded at startup.|`map`|

### roslaunch

Load the YAML file and launch.

例) sample.launch
```xml
<launch>
  <node name="vgm_pub" pkg="pointsmap_renderer" type="voxelgridmap_publisher_node" output="screen">
    <rosparam command="load" file="$(find pointsmap_renderer)/config/sample.yaml"/>
  </node>
  <node name="iteration1" pkg="pointsmap_renderer" type="pointsmap_renderer_node">
    <param name="hz" value="5" />
    <param name="queue_size" value="2" />
    <param name="depth_min" value="1.0" />
    <param name="depth_max" value="80.0" />
  </node>
  <node name="iteration2" pkg="pointsmap_renderer" type="pointsmap_renderer_node">
    <param name="hz" value="5" />
    <param name="queue_size" value="2" />
    <param name="depth_min" value="1.0" />
    <param name="depth_max" value="80.0" />
  </node>
  <node name="iteration3" pkg="pointsmap_renderer" type="pointsmap_renderer_node">
    <param name="hz" value="5" />
    <param name="queue_size" value="2" />
    <param name="depth_min" value="1.0" />
    <param name="depth_max" value="80.0" />
  </node>
</launch>
```

## Overview of each node

### pointsmap_renderer/voxelgridmap_publisher_node

Convert the PCD files specified in `init_maps` or the maps in `/points_map` topic to a Voxel Grid Map and publish it.

- Publications:
  - `/voxel_grid_map` [pointsmap_renderer/VoxelGridMap]

- Subscriptions:
  - `/points_map` [sensor_msgs/PointCloud2]

- Params:
  - `voxel_size`: The length of a side of the voxel that stores the point cloud maps. [m] (`10.0`)
  - `init_maps`: List of point cloud map paths to be loaded at startup. (None)
  - `map_frameid`: Coordinate system of the point cloud maps to be loaded at startup. (`map`)

### pointsmap_renderer/pointsmap_renderer_node

Generate depth maps projected from point cloud maps from `~/camera_info`, `/voxel_grid_map` and TF, and publish them as `~/sparse_depth`.

- Publications:
  - `~/sparse_depth` [sensor_msgs/Image]

- Subscriptions:
  - `~/camera_info` [sensor_msgs/Image]
  - `/tf` [tf2_msgs/TFMessage]
  - `/tf_static` [tf2_msgs/TFMessage]
  - `/voxel_grid_map` [pointsmap_renderer/VoxelGridMap]

- Params:
  - `hz`: Frequency of publish. [Hz] (`5.0`)
  - `depth_min`: Minimum value for depth map [m] (`0.0`)
  - `depth_max`: Maximum value for depth map [m] (`INFINITY`)
  - `queue_size`: Queue size of `~/sparse_depth` (`2`)
