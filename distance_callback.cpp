void msgCallback(ConstLaserScanStampedPtr &msg)
{
  const gazebo::msgs::LaserScan &scan = msg->scan();
  const int n = scan.ranges_size();

  GstBuffer *buffer = gst_buffer_new_allocate(nullptr, n * sizeof(float), nullptr);
  GstMapInfo map;

  if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE))
  {
    std::cerr << "Failed to map GStreamer buffer." << std::endl;
    gst_buffer_unref(buffer);
    return;
  }

  memcpy(map.data, scan.ranges().data(), n * sizeof(float));

  gst_buffer_unmap(buffer, &map);

  GstFlowReturn ret;
  g_signal_emit_by_name(this->appsrc, "push-buffer", buffer, &ret);
  gst_buffer_unref(buffer);

  if (ret != GST_FLOW_OK)
    std::cerr << "Failed to push buffer to appsrc\n";
}  
