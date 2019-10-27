#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <queue>

using namespace std;
using namespace cv;

queue <Mat> q;
Mat colorImage;
Mat delta_x, delta_y, delta_t;
Mat prev, next, result;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
    try
    {
        colorImage= cv_bridge::toCvShare(msg, "bgr8")->image;
        q.push(colorImage);
    
    if (q.size() > 2){
        q.pop();
       
        
        prev = q.front();
       
        next = q.back();
        delta_t = next - prev;
        
        imshow("view", delta_t);
        waitKey(30);
    }
  }
  
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_processing");
  ros::NodeHandle nh;
  namedWindow("view");
  startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
  ros::spin();
  destroyWindow("view");
}
