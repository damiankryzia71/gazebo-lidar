# Steps to simulate a LiDAR in Gazebo Classic and stream the binary data with GStreamer (PX4 simulation).

### 1. Insert the `lidar_gst_plugin.cpp` source file into the `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/` directory.
### 2. Modify `CMakeLists.txt`.
In the `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/` directory, add the following to the `CMakeLists.txt` file.

Under `if (GSTREAMER_FOUND)`, add:
```cmake
 add_library(lidar_gst_plugin SHARED src/lidar_gst_plugin.cpp)
```
And register the plugin:
```cmake
set(plugins
  ${plugins}
  gazebo_gst_camera_plugin
  modified_gst_plugin
  lidar_gst_plugin
)
```
