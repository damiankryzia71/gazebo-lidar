# Steps to simulate a 2D or 3D LiDAR in Gazebo Classic, stream and process the binary data using a GStreamer pipeline (PX4 Simulation).

### 1. Insert the appropriate `_lidar_gst_plugin.cpp` source file into the `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/` directory.
The plugin subscribes to the LiDAR topic created by Gazebo and creates a GStreamer pipeline to stream the LiDAR metadata and ranges as raw binary data.

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
### 3. Attach the LiDAR sensor to a simulated model.
The `2d_lidar.sdf` and `3d_lidar.sdf` files contain the sensor elements with the plugin attached. Adjust the pose, topic name, host, and ports as necessary.
Attach it to any model and rebuild the PX4 simulation using that model. The LiDAR data should now start streaming.

### 4. Confirm data is being streamed.
To confirm that the LiDAR data is being streamed, read the GStreamer pipeline by opening a new terminal and running (adjust the port as necessary):
```bash
gst-launch-1.0 -v udpsrc port=5601 caps="application/octet-stream" ! fakesink dump=true
```

### 5. Visualize the LiDAR data
Use the included `lidar_visualization_from_gst.cpp` file to visualize the point cloud data with OpenCV.
Use the included `CMakeLists.txt`.
Run the file with your specified UDP port number:
```bash
./lidar_visualization_from_gst 5601
```
