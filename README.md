# Steps to simulate a 2D LiDAR in Gazebo Classic, stream and process the binary data using a GStreamer pipeline (PX4 Simulation).
## The steps to achieve the same result with a 3D LiDAR can be found in the `main` branch of this repository.

### 1. Insert the `2d_lidar_gst_plugin.cpp` source file into the `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/` directory.
The plugin subscribes to the LiDAR topic created by Gazebo and creates a GStreamer pipeline to stream the LiDAR metadata and ranges as raw binary data.

### 2. Modify `CMakeLists.txt`.
In the `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/` directory, add the following to the `CMakeLists.txt` file.

Under `if (GSTREAMER_FOUND)`, add:
```cmake
 add_library(lidar_gst_plugin SHARED src/2d_lidar_gst_plugin.cpp)
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
The `2d_lidar.sdf` file contains the sensor element with the plugin attached. Adjust the pose, topic name, host, and ports as necessary.
Attach it to any model and rebuild the PX4 simulation using that model. The LiDAR data should now start streaming.
```sdf
<sensor name='lidar_sensor' type='gpu_ray'>
  <pose>0.0 0 -0.162 0 0 0</pose>
  <always_on>1</always_on>
  <visualize>1</visualize>
  <update_rate>15</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>1080</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
       </horizontal>
    </scan>
    <range>
      <min>0.05</min>
      <max>30</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_gst_plugin" filename="liblidar_gst_plugin.so">
    <topicName>/gazebo/default/typhoon_h480/cgo3_camera_link/lidar_sensor/scan</topicName>
    <udpHost>127.0.0.1</udpHost>
    <udpPorts>5600,5601</udpPorts>
  </plugin>
</sensor>
```

### 4. Confirm data is being streamed.
To confirm that the LiDAR data is being streamed, read the GStreamer pipeline by opening a new terminal and running (adjust the port as necessary):
```bash
gst-launch-1.0 -v udpsrc port=5602 caps="application/octet-stream" ! fakesink dump=true
```

### 5. Compute and visualize the point cloud data
Use the included `2d_lidar_visualization_from_gst.cpp` file to visualize the point cloud data with OpenCV.
Use the included `CMakeLists.txt`.

This program includes code to set up and read data from the GStreamer pipeline created by the Gazebo plugin.

This program also includes the `LoadPointCloudGst` function. This function uses the streamed metadata and ranges to compute 2D (X, Y) points and stores them in a `cv::Mat` object. This is an efficient way to store point cloud data and in this case we're using it for visualization.
```cpp
bool LoadPointCloudGst(cv::Mat &pcd, GstElement *appsink)
{
    GstSample *sample = nullptr;
    g_signal_emit_by_name(appsink, "pull-sample", &sample);
    if (!sample)
    {
        std::cerr << "Failed to pull sample." << std::endl;
        return false;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;

    if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
        std::cerr << "Failed to map buffer." << std::endl;
        gst_sample_unref(sample);
        return false;
    }

    const float *data = reinterpret_cast<const float*>(map.data);

    const int count = static_cast<int>(data[0]);
    const float angle_min = data[1];
    const float angle_step = data[2];
    const float *ranges = data + 3;

    std::vector<cv::Vec2f> validPoints;

    for (int i = 0; i < count; ++i)
    {
        float r = ranges[i];
        if (!std::isfinite(r) || r <= 0.05f || r > 50.0f)
            continue;

        float angle = angle_min + i * angle_step;
        float x = r * cosf(angle);
        float y = r * sinf(angle);

        validPoints.emplace_back(cv::Vec2f(x, y));
    }

    if (validPoints.empty())
    {
        std::cerr << "No valid LiDAR points found." << std::endl;
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return false;
    }

    pcd = cv::Mat(2, validPoints.size(), CV_32F);
    for (size_t i = 0; i < validPoints.size(); ++i)
    {
        pcd.at<float>(0, i) = validPoints[i][0];
        pcd.at<float>(1, i) = validPoints[i][1];
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    
    return true;
}
```
Run the file with your specified UDP port number:
```bash
./2d_lidar_visualization_from_gst 5602
```
