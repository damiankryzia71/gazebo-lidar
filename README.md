# Steps to simulate a 3D LiDAR in Gazebo Classic, stream and process the binary data using a GStreamer pipeline (PX4 Simulation).
## The steps to achieve the same result with a 2D LiDAR can be found in the `2D` branch of this repository.

### 1. Insert the `3d_lidar_gst_plugin.cpp` source file into the `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/` directory.
The plugin subscribes to the LiDAR topic created by Gazebo and creates a GStreamer pipeline to stream the LiDAR metadata and ranges as raw binary data.

### 2. Modify `CMakeLists.txt`.
In the `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/` directory, add the following to the `CMakeLists.txt` file.

Under `if (GSTREAMER_FOUND)`, add:
```cmake
 add_library(lidar_gst_plugin SHARED src/3d_lidar_gst_plugin.cpp)
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
The `3d_lidar.sdf` file contains the sensor element with the plugin attached. Adjust the pose, topic name, host, and ports as necessary.
Attach it to any model and rebuild the PX4 simulation using that model. The LiDAR data should now start streaming.
```sdf
      <sensor name='lidar_sensor' type='gpu_ray'>
        <pose>0.0 0 -0.162 0 0 0</pose>
        <always_on>1</always_on>
        <visualize>1</visualize>
        <update_rate>30</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>1080</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>8</samples>
              <resolution>1</resolution>
              <min_angle>-0.2618</min_angle>
              <max_angle>0.2618</max_angle>
            </vertical>
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
          <udpPorts>5602</udpPorts>
        </plugin>
      </sensor>
```

### 4. Confirm data is being streamed.
To confirm that the LiDAR data is being streamed, read the GStreamer pipeline by opening a new terminal and running (adjust the port as necessary):
```bash
gst-launch-1.0 -v udpsrc port=5602 caps="application/octet-stream" ! fakesink dump=true
```

### 5. Compute and visualize the point cloud data
Use the included `3d_lidar_visualization_from_gst.cpp` file to visualize the point cloud data with OpenCV.
Use the included `CMakeLists.txt`.

This program includes the `LoadPointCloudGst` function. This function uses the streamed metadata and ranges to compute 3D (X, Y, Z) points and stores them in a `cv::Mat` object. This is an efficient way to store point cloud data and in this case we're using it for visualization.
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

    const int count_h = static_cast<int>(data[0]);
    const int count_v = static_cast<int>(data[1]);
    const float angle_min_h = data[2];
    const float angle_step_h = data[3];
    const float angle_min_v = data[4];
    const float angle_step_v = data[5];
    const float *ranges = data + 6;

    std::vector<cv::Vec4f> validPoints;

    for (int v = 0; v < count_v; v++)
    {
        float angle_v = angle_min_v + v * angle_step_v;
        for (int h = 0; h < count_h; h++)
        {
            int i = v * count_h + h;
            float r = ranges[i];

            // Skip invalid ranges
            if (!std::isfinite(r) || r <= 0.05f || r > 200.0f)
                continue;

            float angle_h = angle_min_h + h * angle_step_h;

            float x = r * cosf(angle_v) * cosf(angle_h);
            float y = r * cosf(angle_v) * sinf(angle_h);
            float z = r * sinf(angle_v);

            // Skip points that would produce zero or invalid depth
            if (!std::isfinite(z) || fabs(z) < 1e-5f)
                continue;

            validPoints.emplace_back(cv::Vec4f(x, y, z, 1.0f));
        }
    }

    if (validPoints.empty())
    {
        std::cerr << "No valid LiDAR points found." << std::endl;
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return false;
    }

    // Convert to 4xN cv::Mat
    pcd = cv::Mat(4, validPoints.size(), CV_32F);
    for (size_t i = 0; i < validPoints.size(); ++i)
    {
        pcd.at<float>(0, i) = validPoints[i][0];
        pcd.at<float>(1, i) = validPoints[i][1];
        pcd.at<float>(2, i) = validPoints[i][2];
        pcd.at<float>(3, i) = validPoints[i][3];
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    
    return true;
}
```
Run the file with your specified UDP port number:
```bash
./3d_lidar_visualization_from_gst 5602
```
