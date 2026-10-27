// Compare the same camera JPEG through MPP/RGA and OpenCV, detecting RGB swaps,
// range/stride errors, throughput regressions and repeated decoder use.
#include <usb_cam/rockchip_mjpeg.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <fstream>
#include <iterator>
#include <chrono>
#include <iostream>
int main(int argc,char **argv) {
  std::cerr << "media compare starting" << std::endl;
  ros::init(argc,argv,"media_compare",ros::init_options::AnonymousName);
  ros::Time::init();
  if (argc!=2) return 2;
  std::ifstream in(argv[1],std::ios::binary);
  std::vector<unsigned char> jpeg((std::istreambuf_iterator<char>(in)),std::istreambuf_iterator<char>());
  cv::Mat reference=cv::imdecode(jpeg,cv::IMREAD_COLOR),rgb;
  if(reference.empty()) return 3;
  cv::cvtColor(reference,rgb,cv::COLOR_BGR2RGB);
  std::cerr << "reference " << rgb.cols << "x" << rgb.rows << std::endl;
  usb_cam::RockchipMjpeg decoder(rgb.cols,rgb.rows);
  cv::Mat output(rgb.size(),CV_8UC3),difference;
  auto start=std::chrono::steady_clock::now();
  for(int i=0;i<60;++i) if(!decoder.decode(jpeg.data(),jpeg.size(),reinterpret_cast<char*>(output.data))) return 4;
  double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  cv::absdiff(rgb,output,difference);
  cv::Scalar error=cv::mean(difference);
  std::cout<<"RGB_MAE="<<error<<" frames=60 seconds="<<seconds<<" fps="<<60/seconds<<std::endl;
  return (error[0]<12 && error[1]<12 && error[2]<12 && 60/seconds>20)?0:5;
}
