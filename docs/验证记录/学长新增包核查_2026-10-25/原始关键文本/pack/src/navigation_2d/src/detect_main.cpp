/***************************************************************************************************************************
 *
 * Author: expload
 * Time: 2026.10
 * Description: 单机检测调试节点（detect.launch）。任务节点不需要它。
 ***************************************************************************************************************************/
#include "detect.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detectnet");
  ros::NodeHandle nh_private("~");

  Detect detect(argc, argv);
  if (!detect.ready())
    ROS_WARN("detectnet: inference is not ready; only the raw image will be published");

  std::string target_name;
  nh_private.param<std::string>("target_name", target_name, "");
  ROS_INFO("detectnet: annotating %s", target_name.empty() ? "all classes" : target_name.c_str());

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate rate(10.0);

  while (ros::ok())
  {
    const std::vector<Detect::Detection> detections = detect.infer();
    for (const auto &detection : detections)
    {
      if (target_name.empty() || detection.label == target_name)
      {
        ROS_INFO("detect: %-26s %.2f  box=(%.0f,%.0f %.0fx%.0f)", detection.label.c_str(),
                 detection.confidence, detection.box.x, detection.box.y, detection.box.width,
                 detection.box.height);
        detect.annotate(detection, cv::Scalar(0, 0, 255), "det_" + detection.label, 0.5);
      }
    }
    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
