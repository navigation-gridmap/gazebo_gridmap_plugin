[![GitHub Action
Status](https://github.com/fmrico/gazebo_gridmap_plugin/workflows/main/badge.svg)](https://github.com/fmrico/gazebo_gridmap_plugin)

# gazebo_gridmap_plugin

This gazebo plugin creates and publishes to [ROS2 (Galactic)](https://docs.ros.org/en/galactic/index.html) a [gridmap](https://github.com/ANYbotics/grid_map) with elevation and occupancy.

## Usage:

```
   <plugin name='gridmap' filename='libgazebo_ros_gridmap.so'>
      <ros>
        <namespace>demo</namespace>
      </ros>
      <center_x>0.0</center_x>
      <center_y>0.0</center_y>
      <min_scan_x>-10.0</min_scan_x>
      <min_scan_y>-10.0</min_scan_y>
      <min_scan_z>-10.0</min_scan_z>
      <max_scan_x>10.0</max_scan_x>
      <max_scan_y>10.0</max_scan_y>
      <max_scan_z>10.0</max_scan_z>
      <min_height>0.2</min_height>
      <max_height>1.5</max_height>
      <resolution>0.1</resolution>
    </plugin>
```

## captures

![Captura de pantalla 2022-02-16 20:00:46](https://user-images.githubusercontent.com/3810011/154469799-23fbca08-cc6d-4d6c-8398-eb8dda94342b.png)
![Captura de pantalla 2022-02-16 19:59:42](https://user-images.githubusercontent.com/3810011/154469801-a73b0ada-d3db-439d-bc9e-1995696227d5.png)
![Captura de pantalla 2022-02-16 19:58:26](https://user-images.githubusercontent.com/3810011/154469802-2878e765-d718-4a81-b234-2b61c3b674c4.png)

