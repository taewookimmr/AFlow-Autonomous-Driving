#include "JHPWMPCA9685.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "darknet_ros_msgs/BoundingBoxes.h"


#define PWM_FULL_REVERSE 204 // 1ms/20ms * 4096
#define PWM_NEUTRAL 307      // 1.5ms/20ms * 4096
#define PWM_FULL_FORWARD 409 // 2ms/20ms * 4096

#define STEERING_LEFT   205
#define STEERING_CENTER 290
#define STEERING_RIGHT  375

#define STEERING_CHANNEL 0
#define ESC_CHANNEL 1

PCA9685 *pca9685;

float PWM_dc = PWM_NEUTRAL;
float PWM_sv = STEERING_CENTER;
int selectedChannel = ESC_CHANNEL;
int switch_dc = 0;
int switch_sv = 0;

void init_pca9685();
void updatePWM(int selectedChannel);

void on_neutral();
void on_decrement();
void on_increment();
void on_left();
void on_right();

void callback(const std_msgs::String::ConstPtr & msg);
void callback_lidar(const sensor_msgs::LaserScan::ConstPtr & msg);
void callback_person(const darknet_ros_msgs::BoundingBoxes::ConstPtr & msg);

int flag_p = 0;

void init_pca9685(){
    pca9685 = new PCA9685();
    int err = pca9685->openPCA9685();
    if (err < 0){
        printf("Error: %d", pca9685->error);
    }
    printf("PCA9685 Device Address: 0x%02X\n", pca9685->kI2CAddress);
    pca9685->setAllPWM(0,0);
    pca9685->reset();
    pca9685->setPWMFrequency(50);

    sleep(1) ;
    pca9685->setPWM(ESC_CHANNEL,0,PWM_NEUTRAL);
    PWM_dc = PWM_NEUTRAL;
    updatePWM(ESC_CHANNEL);

    pca9685->setPWM(STEERING_CHANNEL,0,STEERING_CENTER);
    PWM_sv = STEERING_CENTER;
    updatePWM(STEERING_CHANNEL);

    on_neutral();
}

void updatePWM(int selectedChannel){
    if (selectedChannel == ESC_CHANNEL){
        pca9685->setPWM(selectedChannel,0,PWM_dc);
    } else{
        pca9685->setPWM(selectedChannel,0,PWM_sv);
    }
}

void on_neutral(){
    PWM_dc = PWM_NEUTRAL;
    PWM_sv = STEERING_CENTER;
    updatePWM(ESC_CHANNEL);
    updatePWM(STEERING_CHANNEL);
}

void on_decrement(){
    PWM_dc --;
    selectedChannel = ESC_CHANNEL;
    updatePWM(ESC_CHANNEL);
}

void on_increment(){
    PWM_dc ++;
    selectedChannel = ESC_CHANNEL;
    updatePWM(ESC_CHANNEL);
}

void on_right(){
    PWM_sv ++;
    selectedChannel = STEERING_CHANNEL;
    updatePWM(STEERING_CHANNEL);
}

void on_left(){
    PWM_sv --;
    selectedChannel = STEERING_CHANNEL;
    updatePWM(STEERING_CHANNEL);
}

void callback(const std_msgs::String::ConstPtr & msg){
    ROS_INFO("testing: [%s]", msg->data.c_str());
    printf("DC: [%d], SV: [%d]", (int)PWM_dc, (int)PWM_sv);

    if(std::strcmp(msg->data.c_str(), "w") == 0){
            on_increment();
    }else if (std::strcmp(msg->data.c_str(), "x")== 0){
            on_decrement();
    }else if (std::strcmp(msg->data.c_str(), "d")== 0){
            on_right();
    }else if (std::strcmp(msg->data.c_str(), "a")== 0){
            on_left();
    }else if (std::strcmp(msg->data.c_str(), "s")== 0){
            on_neutral();
    }
}

void callback_lidar(const sensor_msgs::LaserScan::ConstPtr & msg){
    printf("distance : %0.1lf\n", msg->ranges[180]);
    if(msg->ranges[180] < 0.4 && msg->ranges[180] > 0){
        on_neutral();
    }
    else if(msg->ranges[180] >= 0.4 && flag_p == 1){
        printf("person ok!\n");
        on_neutral();
    }
    else {
        PWM_dc = 320;
        selectedChannel = ESC_CHANNEL;
        updatePWM(ESC_CHANNEL);
    }
}

void callback_person(const darknet_ros_msgs::BoundingBoxes::ConstPtr & msg){
    int count = 0;
    int n;
    n = msg->bounding_boxes.size();

    for(int i = 0; i < n; i++){
        if(std::strcmp(msg->bounding_boxes[i].Class.c_str(), "person") == 0){
            flag_p = 1;
            printf("flag_p change! %d\n", flag_p);
            return;
        }
    }
    flag_p = 0;
}

int main(int argc, char **argv){
    init_pca9685();

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("keys", 1, callback);
    ros::Subscriber sub_lidar = n.subscribe("scan", 1, callback_lidar);
    ros::Subscriber sub_person = n.subscribe("darknet_ros/bounding_boxes", 1, callback_person);
    ros::spin();

    return 0;
}
