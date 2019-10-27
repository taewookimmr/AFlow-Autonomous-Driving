#include "ros/ros.h"
#include "std_msgs/String.h"
#include "JHPWMPCA9685.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <utility> // pair
#include <vector>  // vector

using namespace std;

#define PWM_FULL_REVERSE 204 // 1ms/20ms * 4096
#define PWM_NEUTRAL 330      // 1.61ms/20ms * 4096
#define PWM_FULL_FORWARD 409 // 2ms/20ms * 4096

#define PWM_MAX PWM_NEUTRAL + 50
#define PWM_MIN PWM_NEUTRAL - 50

#define STEERING_LEFT 210
#define STEERING_CENTER 290
#define STEERING_RIGHT 370

#define STEERING_CHANNEL 0
#define ESC_CHANNEL 1

PCA9685 *pca9685;

float PWM_dc = PWM_NEUTRAL;
float PWM_sv = STEERING_CENTER;
int selectedChannel = ESC_CHANNEL;
int switch_dc = 0;
int switch_sv = 0;

// function prototype start//////////////////////////////////////////////////////////////////
void init_pca9685();
void updatePWM(int selectedChannel);

void on_neutral();
void on_neutral2();
void on_decrement(int delta);
void on_increment(int delta);
void on_left(int delta);
void on_right(int delta);

void callback(const std_msgs::String::ConstPtr &msg);
void callback_person(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
void callback_lidar(const sensor_msgs::LaserScan::ConstPtr &msg);
void callback_lidar_twk(const sensor_msgs::LaserScan::ConstPtr &msg);
void callback_lidar_twk2(const sensor_msgs::LaserScan::ConstPtr &msg);
void callback_lidar_cho(const sensor_msgs::LaserScan::ConstPtr &msg);
int map_func(int angle_on_standard_coordinate);
// function prototype end////////////////////////////////////////////////////////////////////


typedef struct _Sign{
    int flag;
    int area;
} Sign;

Sign s_left, s_right, s_straight, s_start, s_stop, s_magnet, s_rocket, s_cloud, s_smile;

void init_pca9685()
{
    pca9685 = new PCA9685();
    int err = pca9685->openPCA9685();
    if (err < 0)
    {
        printf("Error: %d", pca9685->error);
    }
    printf("PCA9685 Device Address: 0x%02X\n", pca9685->kI2CAddress);
    pca9685->setAllPWM(0, 0);
    pca9685->reset();
    pca9685->setPWMFrequency(50);

    sleep(1);
    pca9685->setPWM(ESC_CHANNEL, 0, PWM_NEUTRAL);
    PWM_dc = PWM_NEUTRAL;
    updatePWM(ESC_CHANNEL);

    pca9685->setPWM(STEERING_CHANNEL, 0, STEERING_CENTER);
    PWM_sv = STEERING_CENTER;
    updatePWM(STEERING_CHANNEL);

    on_neutral();
}

void updatePWM(int selectedChannel)
{
    if (selectedChannel == ESC_CHANNEL)
    {
        pca9685->setPWM(selectedChannel, 0, PWM_dc);
    }
    else
    {
        pca9685->setPWM(selectedChannel, 0, PWM_sv);
    }
}

void on_neutral()
{
    PWM_dc = PWM_NEUTRAL;
    PWM_sv = STEERING_CENTER;
    updatePWM(ESC_CHANNEL);
    updatePWM(STEERING_CHANNEL);
}

void on_neutral2()
{
    PWM_dc = PWM_NEUTRAL;
    //PWM_sv = STEERING_CENTER ;
    updatePWM(ESC_CHANNEL);
    //updatePWM(STEERING_CHANNEL) ;
}

void on_decrement(int delta)
{
    if (PWM_dc > PWM_MIN)
    {
        PWM_dc -= delta;
        selectedChannel = ESC_CHANNEL;
        updatePWM(ESC_CHANNEL);
    }
}

void on_increment(int delta)
{
    if (PWM_dc < PWM_MAX)
    {
        PWM_dc += delta;
        selectedChannel = ESC_CHANNEL;
        updatePWM(ESC_CHANNEL);
    }
}

void on_right(int delta)
{

    if (PWM_sv == STEERING_RIGHT)
        return;
    PWM_sv += delta;
    selectedChannel = STEERING_CHANNEL;
    updatePWM(STEERING_CHANNEL);
}

void on_left(int delta)
{

    if (PWM_sv == STEERING_LEFT)
        return;
    PWM_sv -= delta;
    selectedChannel = STEERING_CHANNEL;
    updatePWM(STEERING_CHANNEL);
}

void callback(const std_msgs::String::ConstPtr &msg)
{

    ROS_INFO("testing: [%s]", msg->data.c_str());
    printf("DC: [%d], SV: [%d]", (int)PWM_dc, (int)PWM_sv);

    if (std::strcmp(msg->data.c_str(), "w") == 0)
    {
        PWM_dc = 340;
        selectedChannel = ESC_CHANNEL;
        updatePWM(ESC_CHANNEL);
    }
    else if (std::strcmp(msg->data.c_str(), "x") == 0)
    {
        on_decrement(1);
    }
    else if (std::strcmp(msg->data.c_str(), "d") == 0)
    {
        on_right(5);
    }
    else if (std::strcmp(msg->data.c_str(), "a") == 0)
    {
        on_left(5);
    }
    else if (std::strcmp(msg->data.c_str(), "s") == 0)
    {
        on_neutral();
    }
}

int i = 0; // what is this for?


void callback_lidar_twk2(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    int angle_start[] = {0, 30, 60, 90, 120, 150}; // 우측부터 반시계 방향으로 1,2,3,4,5,6 구역으로 나눔
    float max_radius = 4.0f;                       // 반경값이 0으로 인식될 시, 대신할 값
    float stop_radius = 0.9f;                      // 전방 1.8 meter 이내에 라이다 인식될 시, 멈추게 끔 하기 위해 필요한 변수

    vector<pair <float, float> > vec[6]; // divide front 180 by 6
    pair<float, float> point_avg[6];   // average point(coordinate) in each divided section

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 31; j++)
        {
            int angle_standard = angle_start[i] + j;    // conventional counter-clockwise(right(x축) 0, front(y축) 180, left 180)
            int angle_lidar = map_func(angle_standard); // (right 270, left 90)
            if (msg->ranges[angle_lidar] > 0.001)
            { // 반경값이 0으로 인식되지 않을 시에는 정상적으로 입력
                float x = msg->ranges[angle_lidar] * cos(angle_standard * M_PI / 180);
                float y = msg->ranges[angle_lidar] * sin(angle_standard * M_PI / 180);
                pair<float, float> point = make_pair(x, y);
                vec[i].push_back(point);
            }
            else
            { // 반경값이 0으로 인식되었을 때
                float x = max_radius * cos(angle_standard * M_PI / 180);
                float y = max_radius * sin(angle_standard * M_PI / 180);
                pair<float, float> point = make_pair(x, y);
                vec[i].push_back(point);
            }
        }
    }

    // right, average
    for (int i = 0; i < 3; i++)
    {
        vector<pair <float, float> >::iterator iter = vec[i].begin();
        float sum_x = 0.0f;
        float sum_y = 0.0f;
        int count = 0;
        for (iter = vec[i].begin(); iter != vec[i].end(); ++iter)
        {
            sum_x += (*iter).first;
            sum_y += (*iter).second;
            count++;
        }
        point_avg[i] = make_pair(sum_x / count, sum_y / count);
    }

    // left,  average
    for (int i = 3; i < 6; i++)
    {
        vector<pair <float, float> >::reverse_iterator riter = vec[i].rbegin();
        float sum_x = 0.0f;
        float sum_y = 0.0f;
        int count = 0;
        for (riter = vec[i].rbegin(); riter != vec[i].rend(); ++riter)
        {
            sum_x += (*riter).first;
            sum_y += (*riter).second;
            count++;
        }
        point_avg[i] = make_pair(sum_x / count, sum_y / count);
    }

    // front 확인용
    float avg_x = 0.0f;
    float avg_y = 0.0f;

    for (int i = 2; i < 4; i++)
    {
        avg_x += point_avg[i].first / 2;
        avg_y += point_avg[i].second / 2;
    }
    
    float frontal_distance = powf(avg_x * avg_x + avg_y * avg_y, 0.5f);
    printf("frontal_distance : %0.1f\n", frontal_distance);

    // front 상태 확인되었으면
    if (frontal_distance < stop_radius)
    {
        printf("watch out!\n");
        on_neutral2();
        return;
    }
    /*
    else{
        // 기본 주행 속도
        PWM_dc = 340;
        selectedChannel = ESC_CHANNEL;
        updatePWM(ESC_CHANNEL);
    }
    */

    // 실시간 방향 전환을 위한 부분
    avg_x = 0.0f;
    avg_y = 0.0f;
    // aiming 벡터, 전방 벡터를 기준으로 aiming 벡터가 어느 각도(부호 고려)만큼 벌어져 있는 지 확인하고 그 각도에 비례하여 steering하도록 하기 위함
    for (int i = 0; i < 6; i++)
    {
        avg_x += point_avg[i].first / 6;
        avg_y += point_avg[i].second / 6;
    }

    // 기준 벡터(전방 벡터)
    float standard_x = 0.0f;
    float standard_y = 1.0f;

    // 벡터 계산
    float inner_product = standard_x * avg_x + standard_y * avg_y;
    float outter_product = standard_x * avg_y - standard_y * avg_x;
    float abs_avg = powf(avg_x * avg_x + avg_y * avg_y, 0.5f);
    float abs_std = powf(standard_x * standard_x + standard_y * standard_y, 0.5f);

    float cosine = inner_product / (abs_avg * abs_std);
    float sine = outter_product / (abs_avg * abs_std); // 각도의 부호 확인용
    float theta = acos(cosine) * 180 / M_PI;           // 각도의 크기 확인용

    int is_base_algorithm = 0 ;
    int area_threshold = 4500;
    int area_magnet_threshold = 9000;
    
    
    printf("theta : %0.1f\n", theta);
    printf("sine : %0.1f\n", sine);
    
    if (s_left.flag == 0 && s_right.flag == 0 && s_straight.flag ==0 && s_start.flag == 0 && s_stop.flag == 0
        && s_magnet.flag == 0 && s_rocket.flag == 0 && s_cloud.flag == 0 && s_smile.flag == 0)
    {
    //if (true){
       is_base_algorithm = 1;
       
    } else {
    
        if (s_left.flag == 1) {
            printf("left sign area : %d\n", s_left.area);
            if(s_left.area < area_threshold){
                is_base_algorithm = 1;
            }else{
                printf("fuck off left\n");
                PWM_sv = STEERING_LEFT;
            }
        } 

        
        else if (s_right.flag == 1) {
            printf("right sign area : %d\n", s_right.area);
            if(s_right.area < area_threshold){
                is_base_algorithm = 1;
            }else{
                printf("fuck off right\n");
                PWM_sv = STEERING_RIGHT;
            }
        }
        
        else if (s_straight.flag == 1){
            printf("straight sign area : %d\n", s_straight.area);
            if(s_straight.area < area_threshold){
                is_base_algorithm = 1;
            }else{
                // PASS
            }
        }
        
        else if (s_start.flag == 1){
            printf("start sign area : %d\n", s_start.area);
            if(s_start.area < area_threshold){
                is_base_algorithm = 1;
            }else{
               PWM_dc = 340;
               selectedChannel = ESC_CHANNEL;
               updatePWM(ESC_CHANNEL);
            }
        }
        
        else if (s_stop.flag == 1) {
            printf("stop sign area : %d\n", s_stop.area);
            if(s_stop.area < area_threshold){
                is_base_algorithm = 1;
            }else{
                 printf("stop sign detected!!!!!!\n");
                 on_neutral2();
            }
        }
        
        else if (s_magnet.flag == 1) {
            printf("magnet sign area : %d\n", s_magnet.area);
            if(s_magnet.area > area_magnet_threshold){
                 printf("magnet is too close. so stot!!!!!!\n");
                 on_neutral2();
            }else{
                 printf("magnet sign detected!!!!!!\n");
                 is_base_algorithm = 1;
                 // is_base_algorithm 대신 magnetically following code가 돌아가도록 해야합니다.
            }
        }
        
        else if (s_rocket.flag == 1) {
            printf("rocket sign area : %d\n", s_rocket.area);
            if(s_rocket.area < area_threshold){
                 is_base_algorithm = 1;
            }else{
                 printf("rocket mode on!!!!\n");
                 is_base_algorithm = 1;
                 PWM_dc = 343;
                 selectedChannel = ESC_CHANNEL;
                 updatePWM(ESC_CHANNEL);
                 // 정해진 시간동안 가속이 유지되도록 하는 방법에 대해서 생각해보도록 
                 // 특정 함수를 만들고 , 그 함수를 정해진 시간동안 반복적으로 호출하면 문제 없음 
                 // 이러한 방식은 magnet, rocket, cloud, smile에 모두 적용 가
            }
        }
        
        else if (s_cloud.flag == 1) {
            printf("cloud sign area : %d\n", s_cloud.area);
            if(s_cloud.area < area_threshold){
                 is_base_algorithm = 1;
            }else{
                 printf("so fucking cloudy, so make u-turn\n");
                 on_neutral();
                 PWM_sv = STEERING_LEFT;
                 PWM_dc = 320;
                 selectedChannel = ESC_CHANNEL;
                 updatePWM(ESC_CHANNEL);
            }
        }
        
        else if (s_smile.flag == 1) {
            printf("smile sign area : %d\n", s_smile.area);
            if(s_smile.area < area_threshold){
                 is_base_algorithm = 1;
            }else{
                 printf("let's follow the yellow bricks!~!\n");
            }
        }
    }
    
    
    if (is_base_algorithm == 1){
       PWM_sv = STEERING_CENTER + (sine>=0.0f?-1:1) * theta * theta / 5.5;
    }

    selectedChannel = STEERING_CHANNEL;
    updatePWM(STEERING_CHANNEL);
}

int map_func(int angle_on_standard_coordinate)
{
    return 270 - angle_on_standard_coordinate;
}



void callback_darknet_ros(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    int count = 0;
    int n;
    n = msg->bounding_boxes.size();
    
    s_left.flag   = 0;
    s_left.area   = 0;
    
    s_right.flag  = 0;
    s_right.area  = 0;
    
    s_straight.flag  = 0;
    s_straight.area  = 0;
    
    s_start.flag = 0;
    s_start.area = 0;
    
    s_stop.flag   = 0;
    s_stop.area  = 0;
    
    s_magnet.flag   = 0;
    s_magnet.area  = 0;
    
    s_rocket.flag   = 0;
    s_rocket.area  = 0;
    
    s_cloud.flag   = 0;
    s_cloud.area  = 0;
    
    s_smile.flag   = 0;
    s_smile.area  = 0;
    
    for (int i = 0; i < n; i++)
    {
        if (std::strcmp(msg->bounding_boxes[i].Class.c_str(), "left") == 0)
        {
            s_left.flag = 1;
            s_left.area = (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)*(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin);
            break;
        }
        else if (std::strcmp(msg->bounding_boxes[i].Class.c_str(), "right") == 0)
        {
            s_right.flag = 1;
            s_right.area = (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)*(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin);
            break;
        }
        else if (std::strcmp(msg->bounding_boxes[i].Class.c_str(), "straight") == 0)
        {
            s_straight.flag = 1;
            s_straight.area = (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)*(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin);
            break;
        }
        else if (std::strcmp(msg->bounding_boxes[i].Class.c_str(), "start") == 0)
        {
            s_start.flag = 1;
            s_start.area = (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)*(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin);
            break;
        }
        else if (std::strcmp(msg->bounding_boxes[i].Class.c_str(), "stop") == 0)
        {
            s_stop.flag = 1;
            s_stop.area = (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)*(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin);
            break;
        }
        
        else if (std::strcmp(msg->bounding_boxes[i].Class.c_str(), "magnet") == 0)
        {
            s_magnet.flag = 1;
            s_magnet.area = (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)*(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin);
            break;
        }
        
        else if (std::strcmp(msg->bounding_boxes[i].Class.c_str(), "rocket") == 0)
        {
            s_rocket.flag = 1;
            s_rocket.area = (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)*(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin);
            break;
        }
        
        else if (std::strcmp(msg->bounding_boxes[i].Class.c_str(), "cloud") == 0)
        {
            s_cloud.flag = 1;
            s_cloud.area = (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)*(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin);
            break;
        }
        
        else if (std::strcmp(msg->bounding_boxes[i].Class.c_str(), "smile") == 0)
        {
            s_smile.flag = 1;
            s_smile.area = (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)*(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin);
            break;
        }
    }
}

int main(int argc, char **argv)
{
    init_pca9685();

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("keys", 1, callback);
    ros::Subscriber sub_darknet_ros = n.subscribe("darknet_ros/bounding_boxes", 1, callback_darknet_ros);
    ros::Subscriber sub_lidar = n.subscribe("scan", 1, callback_lidar_twk2);
    ros::spin();

    return 0;
}
