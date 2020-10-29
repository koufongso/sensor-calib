#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cstdio>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <fstream>
   
int main(int argc, char **argv){
  ros::init(argc,argv,"image_motion_detect");

  if(argc<2){
    std::cerr<<"Please enter a .bag file path"<<std::endl;
    exit(1);
  }
  
  int step = 0;
  float data =0;
  // sensor_msgs::CompressedImage::ConstPtr ptr_pre;
  sensor_msgs::Image::ConstPtr ptr_pre;
  uint32_t diff = 0;
  uint32_t diff_pre;

  std::ofstream myfile; 
  myfile.open((argc>=3 ? argv[2] : "example.txt"));
  rosbag::Bag bag;        
  bag.open(argv[1]);  // BagMode is Read by default
  
  for(rosbag::MessageInstance const m: rosbag::View(bag,rosbag::TopicQuery("/usb_cam/image_raw"))){

    // sensor_msgs::CompressedImage::ConstPtr ptr = m.instantiate<sensor_msgs::CompressedImage>();
    sensor_msgs::Image::ConstPtr ptr = m.instantiate<sensor_msgs::Image>();
    
    if(step==0){ // first data, no previous data
      ptr_pre = ptr;
    }else if (ptr != nullptr){ // has previous data, find diff
      diff = 0;
      for(int i=0; i<ptr->data.size();i++){
        uint32_t temp = ((uint8_t)ptr->data[i]-(uint8_t)ptr_pre->data[i]);
        diff += temp*temp;
      }
    }

    data = (float)diff;
    ptr_pre = ptr;
    // step>=2 ? data = (float)diff/diff_pre-1 : data = 0.0f; // first 2 step has no previous diff
    // diff_pre = diff;

    myfile << ptr->header.stamp<<" "<<data<<"\n";
    step++;
  }
  
  bag.close();
  myfile.close();

  return 0;
}
