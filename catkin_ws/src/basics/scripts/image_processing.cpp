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
Mat delta;
Mat old_frame, old_gray;
Mat frame, gray;

vector<uchar> status;
vector<float> err;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    printf("testing\n");
    vector<Point2f> p0, p1;
    vector<Scalar> colors;
    RNG rng;
    for(int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(Scalar(r,g,b));
    }

    try
    {
        colorImage= cv_bridge::toCvShare(msg, "bgr8")->image;
        q.push(colorImage);
    
    if (q.size() > 2){
        q.pop();
       
        // delta = q.back() - q.front(); 
        // cvtColor(delta, gray, CV_RGB2GRAY );
        old_frame = q.front();
        cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
        goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, Mat(), 7, false, 0.04);
        Mat mask = Mat::zeros(old_frame.size(), old_frame.type());
        
        frame = q.back();
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(old_gray, gray, p0, p1, status, err, Size(15,15), 2, criteria);
        
        vector<Point2f> good_new;
        for(uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if(status[i] == 1) {
                good_new.push_back(p1[i]);
                // draw the tracks
                line(mask,p1[i], p0[i], colors[i], 2);
                circle(frame, p1[i], 5, colors[i], -1);
            }
        }
        Mat img;
        add(frame, mask, img);
       
        imshow("view", img);
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
