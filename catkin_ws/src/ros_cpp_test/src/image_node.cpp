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
#include "OpenCV.hpp"

using namespace std;
using namespace cv;

queue <Mat> __q;
Mat __src_color, __src_binary;
Mat __delta_x, __delta_y, __delta_t;
Mat __prev, __next, __result;

Mat __flow, __cflow;
UMat __gray, __prevgray, __uflow__;

void __line_detect();
void __image_delta_calculation();
void __optical_flow_calculation();
void __imageCallback(const sensor_msgs::ImageConstPtr& msg);


void __line_detect(){
  Mat __dst;
  float __result = line_follow_algorithm(__src_color, __dst);
  printf("line theta: %f\n", __result);
  imshow("line_detector", __dst);
}


void __image_delta_calculation(){
    __q.push(__src_color);

    if (__q.size() > 2){
        __q.pop();
        __prev = __q.front();
        __next = __q.back();
        __delta_t = __next - __prev;
        
        convertColor(__delta_t, __delta_t);
        imshow("image_delta", __delta_t);
      
    }
}


void __optical_flow_calculation(){
    // src_color, 즉, 원본의 사이즈가 크니, 영상 하단 중심을 roi로 잡고 자른 다음, 그 부분에 대해서만 연산을 실시한다.
    // 필요한 부분만 자르는 것을 추가하도록 합시다.

	cvtColor(__src_color, __gray, COLOR_BGR2GRAY);
	int height = __gray.rows;
    int width  = __gray.cols;
    
    float height_roi_ratio = 1.0f;
    float width_roi_ratio = 0.33f;
    
    int start_col = (int)(width/2.0f - width*width_roi_ratio/2.0f);
    int start_row = (int)(height - height*height_roi_ratio);
    
    
    Rect rect(start_col, start_row, width*width_roi_ratio, height*height_roi_ratio);
    __gray = __gray(rect);
    
   
    if( !__prevgray.empty() )
    {
        calcOpticalFlowFarneback(__prevgray, __gray, __uflow__, 0.5, 3, 15, 3, 5, 1.2, 0);
        cvtColor(__prevgray, __cflow, COLOR_GRAY2BGR);
        __uflow__.copyTo(__flow);
        drawOptFlowMap(__flow, __cflow, 16, 1.5, Scalar(0, 255, 0)); 
        imshow("optical_flow", __cflow); // 계산된 결과는 cflow에 담기게 된다 이를 활용하여 현재 속도를 구한다.
    }
    if(waitKey(1)>=0)
        return;
    std::swap(__prevgray, __gray);
}


void __imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
  try
  {
    __src_color= cv_bridge::toCvShare(msg, "bgr8")->image;
    convertColor(__src_color, __src_binary);
    __line_detect();
    //__image_delta_calculation();
    __optical_flow_calculation();
    waitKey(25);
  }
  
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_node");
  ros::NodeHandle __nh;
  namedWindow("optical_flow");
  namedWindow("line_detector");
  startWindowThread();
  image_transport::ImageTransport __it(__nh);
  image_transport::Subscriber __sub = __it.subscribe("camera/image_raw", 1, __imageCallback);
  ros::spin();
  destroyWindow("optical_flow");
  destroyWindow("line_detector");
}
