#include "ros/ros.h"
#include "std_msgs/String.h"
#include "JHPWMPCA9685.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <utility>  // pair
#include <vector>   // vector


using namespace std;

#define PWM_FULL_REVERSE 204 // 1ms/20ms * 4096
#define PWM_NEUTRAL 330      // 1.61ms/20ms * 4096
#define PWM_FULL_FORWARD 409 // 2ms/20ms * 4096

#define PWM_MAX PWM_NEUTRAL+50
#define PWM_MIN PWM_NEUTRAL-50

#define STEERING_LEFT   210
#define STEERING_CENTER 290
#define STEERING_RIGHT  370

#define STEERING_CHANNEL 0
#define ESC_CHANNEL 1

PCA9685 *pca9685 ;

float PWM_dc = PWM_NEUTRAL ;
float PWM_sv = STEERING_CENTER ;
int selectedChannel = ESC_CHANNEL ;
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

void callback(const std_msgs::String::ConstPtr& msg);
void callback_person(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
void callback_lidar(const sensor_msgs::LaserScan::ConstPtr& msg);
void callback_lidar_twk(const sensor_msgs::LaserScan::ConstPtr& msg);
void callback_lidar_twk2(const sensor_msgs::LaserScan::ConstPtr& msg);
void callback_lidar_cho(const sensor_msgs::LaserScan::ConstPtr& msg);
int map_func(int angle_on_standard_coordinate);
// function prototype end////////////////////////////////////////////////////////////////////

int flag_p = 0;

void init_pca9685(){
    pca9685 = new PCA9685() ;
    int err = pca9685->openPCA9685();
    if (err < 0){
        printf("Error: %d", pca9685->error);
    }
    printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
    pca9685->setAllPWM(0,0) ;
    pca9685->reset() ;
    pca9685->setPWMFrequency(50) ;

    sleep(1) ;
    pca9685->setPWM(ESC_CHANNEL,0,PWM_NEUTRAL);
    PWM_dc = PWM_NEUTRAL ;
    updatePWM(ESC_CHANNEL) ;

    pca9685->setPWM(STEERING_CHANNEL,0,STEERING_CENTER);
    PWM_sv = STEERING_CENTER ;
    updatePWM(STEERING_CHANNEL) ;

    on_neutral();
}

void updatePWM(int selectedChannel){
    if (selectedChannel == ESC_CHANNEL){
        pca9685->setPWM(selectedChannel,0,PWM_dc) ;
    } else{
        pca9685->setPWM(selectedChannel,0,PWM_sv) ;
    }
}

void on_neutral(){
    PWM_dc = PWM_NEUTRAL ;
    PWM_sv = STEERING_CENTER ;
    updatePWM(ESC_CHANNEL) ;
    updatePWM(STEERING_CHANNEL) ;
}

void on_neutral2(){
    PWM_dc = PWM_NEUTRAL ;
    //PWM_sv = STEERING_CENTER ;
    updatePWM(ESC_CHANNEL) ;
    //updatePWM(STEERING_CHANNEL) ;
}

void on_decrement(int delta){
	if(PWM_dc > PWM_MIN){
        PWM_dc -= delta ;
        selectedChannel = ESC_CHANNEL;
        updatePWM(ESC_CHANNEL) ;
	}
}
void on_increment(int delta){
	if(PWM_dc < PWM_MAX){
        PWM_dc += delta ;
        selectedChannel = ESC_CHANNEL;
        updatePWM(ESC_CHANNEL) ;
	}
}
void on_right(int delta){

    if( PWM_sv == STEERING_RIGHT )
        return;
    PWM_sv += delta ;
    selectedChannel = STEERING_CHANNEL;
    updatePWM(STEERING_CHANNEL) ;
}
void on_left(int delta){

    if( PWM_sv == STEERING_LEFT )
        return;
    PWM_sv -= delta ;
    selectedChannel = STEERING_CHANNEL;
    updatePWM(STEERING_CHANNEL) ;
}

void callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("testing: [%s]", msg->data.c_str());
    printf("DC: [%d], SV: [%d]", (int)PWM_dc, (int)PWM_sv);

    if(std::strcmp(msg->data.c_str(), "w") == 0){
        on_increment(1);
    }else if (std::strcmp(msg->data.c_str(), "x")== 0){
        on_decrement(1);
    }else if (std::strcmp(msg->data.c_str(), "d")== 0){
        on_right(5);
    }else if (std::strcmp(msg->data.c_str(), "a")== 0){
        on_left(5);
    }else if (std::strcmp(msg->data.c_str(), "s")== 0){
        on_neutral();
    }
}

int i = 0; // what is this for?

void callback_lidar(const sensor_msgs::LaserScan::ConstPtr  &msg){
    printf("distance : %0.1lf\n",msg->ranges[90]);
    printf("distance : %0.1lf\n",msg->ranges[270]);
    printf("flag : %d\n\n",flag_p);

	if( msg->ranges[180] < 0.4 && msg->ranges[180] > 0)
		on_neutral();
	else if (msg->ranges[180] >= 0.4 && flag_p == 1){
		printf("person oK!\n");
		on_neutral();
	}
	else if (msg->ranges[180] == 0 && flag_p == 1){
		printf("person oK!\n");
		on_neutral();
	}		
	else{
		PWM_dc = 320;
        selectedChannel = ESC_CHANNEL;
        updatePWM(ESC_CHANNEL) ;
	}
}

void callback_lidar_cho(const sensor_msgs::LaserScan::ConstPtr  &msg){
	float l_avg,r_avg,f_avg;
	float f_sum = 0;
	float r_sum = 0;
	float l_sum = 0;

	for(int i =90;i<180;i++){
		if(msg->ranges[i] == 0)
			l_sum+= 3.5;
		else
			l_sum+=msg->ranges[i];
	}
	for(int i =180;i<270;i++){
		if(msg->ranges[i] == 0)
			r_sum+= 3.5;
		else		
			r_sum+=msg->ranges[i];
	}
	for(int i = 150;i < 210 ; i++){
		if(msg->ranges[i] == 0)
			f_sum+=3.5;
		else
			f_sum+=msg->ranges[i];
	}
	l_avg = l_sum / 90;
	r_avg = r_sum / 90;
	f_avg = f_sum / 60;

	printf("l avg : %f, r avg : %f f avg : %f, PWM_DC : %f\n ",l_avg,r_avg,f_avg,PWM_dc);

	if( f_avg < 0.8)
		on_neutral2();
	else if (f_avg >= 0.8 && flag_p == 1){
		printf("person oK!\n");
		on_neutral2();
	}	
	else{
		PWM_dc = 345;
		selectedChannel = ESC_CHANNEL;
		updatePWM(ESC_CHANNEL);

		if( l_avg > r_avg )
			on_left( (l_avg - r_avg) * 10 );
		else if( l_avg < r_avg )
			on_right( (r_avg - l_avg) * 10);
	}
}

void callback_lidar_twk(const sensor_msgs::LaserScan::ConstPtr  &msg){

        int angle_start[] = {0, 30, 60, 90, 120, 150}; // 우측부터 반시계 방향으로 1,2,3,4,5,6 구역으로 나눔
        float max_radius = 4.0f; // 반경값이 0으로 인식될 시, 대신할 값
        float stop_radius = 1.8f; // 전방 1.8 meter 이내에 라이다 인식될 시, 멈추게 끔 하기 위해 필요한 변수

        vector< pair<float, float> > vec[6]; // divide front 180 by 6
        pair<float, float> point_avg[6]; // average point(coordinate) in each divided section

        for(int i = 0; i < 6; i++){
            for(int j = 0; j < 31; j++){
                int angle_standard = angle_start[i] + j;    // conventional counter-clockwise(right(x축) 0, front(y축) 180, left 180)
                int angle_lidar = map_func(angle_standard); // (right 270, left 90)
                if (msg->ranges[angle_lidar] > 0.001){      // 반경값이 0으로 인식되지 않을 시에는 정상적으로 입력
                    float x = msg->ranges[angle_lidar] * cos (angle_standard * M_PI / 180);
                    float y = msg->ranges[angle_lidar] * sin (angle_standard * M_PI / 180);
                    pair<float, float> point = make_pair(x, y);
                    vec[i].push_back(point);
                }else{ // 반경값이 0으로 인식되었을 때
                    float x = max_radius * cos (angle_standard * M_PI / 180);
                    float y = max_radius * sin (angle_standard * M_PI / 180);
                    pair<float, float> point = make_pair(x, y);
                    vec[i].push_back(point);
                }
            }
        }

        // right, average
        for(int i =0; i < 3; i++){
            vector< pair<float, float> >::iterator iter = vec[i].begin();
            float sum_x = 0.0f;
            float sum_y = 0.0f;
            int count = 0;
            for(iter = vec[i].begin(); iter !=  vec[i].end(); ++iter ){
                sum_x += (*iter).first;
                sum_y += (*iter).second;
                count ++;
            }
            point_avg[i] = make_pair(sum_x/count, sum_y/count);
        }

        // left,  average
        for(int i = 3; i < 6; i++){
            vector< pair<float, float> >::reverse_iterator riter = vec[i].rbegin();
            float sum_x = 0.0f;
            float sum_y = 0.0f;
            int count = 0;
            for(riter = vec[i].rbegin(); riter !=  vec[i].rend(); ++riter ){
                sum_x += (*riter).first;
                sum_y += (*riter).second;
                count ++;
            }
            point_avg[i] = make_pair(sum_x/count, sum_y/count);
        }

        // front 확인용
        float avg_x = 0.0f;
        float avg_y = 0.0f;

        for (int i=2; i<4; i++){
            avg_x +=point_avg[i].first/2;
            avg_y +=point_avg[i].second/2;
        }
        float frontal_distance = powf(avg_x*avg_x + avg_y*avg_y, 0.5f);
        printf("frontal_distance : %0.1f\n", frontal_distance);


        if (frontal_distance < stop_radius){
            printf("watch out!\n");
            on_neutral2();
            return;
        }else{
            // 기본 주행 속도
            PWM_dc = 340;
            selectedChannel = ESC_CHANNEL;
            updatePWM(ESC_CHANNEL);
        }

        avg_x = 0.0f;
        avg_y = 0.0f;

        // 실시간 방향 전환을 위한 부분

        // aiming 벡터, 전방 벡터를 기준으로 aiming 벡터가 어느 각도(부호 고려)만큼 벌어져 있는 지 확인하고 그 각도에 비례하여 steering하도록 하기 위함
        for(int i=0; i<6; i++){
            avg_x +=point_avg[i].first/6;
            avg_y +=point_avg[i].second/6;
        }

        // 기준 벡터(전방 벡터)
        float standard_x = 0.0f;
        float standard_y = 1.0f;

        // 벡터 계산
        float inner_product = standard_x*avg_x + standard_y * avg_y;
        float outter_product = standard_x*avg_y-standard_y*avg_x;
        float abs_avg = powf(avg_x * avg_x + avg_y * avg_y, 0.5f);
        float abs_std = powf(standard_x * standard_x + standard_y * standard_y, 0.5f);

        float cosine = inner_product/(abs_avg * abs_std);
        float sine = outter_product/(abs_avg * abs_std); // 각도의 부호 확인용
        float theta = acos(cosine) * 180/M_PI; // 각도의 크기 확인용

        printf("theta : %0.1f\n", theta);
        printf("sine : %0.1f\n", sine);
        printf("cosine : %0.1f\n", cosine);

        if(sine >= 0.0f){
            // go left

            PWM_sv = STEERING_CENTER-theta*2.5 ;
            // 이 부분, theta에 대해 선형적(1차)으로 변하게 끔 해놨지만 일단,
            // 비선형적으로 변하게 수정해야한다.
            // theta가 클수록, 즉 방향 전환해야할 정도가 클 수록, PWM_sv를 크게 바꿔야 하기 때문에
            // STEERING_CENTER - theta*theta / constant 이런식으로 비선형적으로 구성하고 적절한 constant를 구해야한다고 생각
            // 아래  else 내부에 있는 부분도 마찬가지.
            selectedChannel = STEERING_CHANNEL;
            updatePWM(STEERING_CHANNEL) ;
        }else {
            // go right

            PWM_sv = STEERING_CENTER + theta*2.5 ;
            selectedChannel = STEERING_CHANNEL;
            updatePWM(STEERING_CHANNEL) ;
        }

}


void callback_lidar_twk2(const sensor_msgs::LaserScan::ConstPtr  &msg){

        int angle_start[] = {0, 30, 60, 90, 120, 150}; // 우측부터 반시계 방향으로 1,2,3,4,5,6 구역으로 나눔
        float max_radius = 4.0f; // 반경값이 0으로 인식될 시, 대신할 값
        float stop_radius = 0.9f; // 전방 1.8 meter 이내에 라이다 인식될 시, 멈추게 끔 하기 위해 필요한 변수

        vector< pair<float, float> > vec[6]; // divide front 180 by 6
        pair<float, float> point_avg[6]; // average point(coordinate) in each divided section

        for(int i = 0; i < 6; i++){
            for(int j = 0; j < 31; j++){
                int angle_standard = angle_start[i] + j;    // conventional counter-clockwise(right(x축) 0, front(y축) 180, left 180)
                int angle_lidar = map_func(angle_standard); // (right 270, left 90)
                if (msg->ranges[angle_lidar] > 0.001){      // 반경값이 0으로 인식되지 않을 시에는 정상적으로 입력
                    float x = msg->ranges[angle_lidar] * cos (angle_standard * M_PI / 180);
                    float y = msg->ranges[angle_lidar] * sin (angle_standard * M_PI / 180);
                    pair<float, float> point = make_pair(x, y);
                    vec[i].push_back(point);
                }else{ // 반경값이 0으로 인식되었을 때
                    float x = max_radius * cos (angle_standard * M_PI / 180);
                    float y = max_radius * sin (angle_standard * M_PI / 180);
                    pair<float, float> point = make_pair(x, y);
                    vec[i].push_back(point);
                }
            }
        }

        // right, average
        for(int i =0; i < 3; i++){
            vector< pair<float, float> >::iterator iter = vec[i].begin();
            float sum_x = 0.0f;
            float sum_y = 0.0f;
            int count = 0;
            for(iter = vec[i].begin(); iter !=  vec[i].end(); ++iter ){
                sum_x += (*iter).first;
                sum_y += (*iter).second;
                count ++;
            }
            point_avg[i] = make_pair(sum_x/count, sum_y/count);
        }

        // left,  average
        for(int i = 3; i < 6; i++){
            vector< pair<float, float> >::reverse_iterator riter = vec[i].rbegin();
            float sum_x = 0.0f;
            float sum_y = 0.0f;
            int count = 0;
            for(riter = vec[i].rbegin(); riter !=  vec[i].rend(); ++riter ){
                sum_x += (*riter).first;
                sum_y += (*riter).second;
                count ++;
            }
            point_avg[i] = make_pair(sum_x/count, sum_y/count);
        }

        // front 확인용
        float avg_x = 0.0f;
        float avg_y = 0.0f;

        for (int i=2; i<4; i++){
            avg_x +=point_avg[i].first/2;
            avg_y +=point_avg[i].second/2;
        }
        float frontal_distance = powf(avg_x*avg_x + avg_y*avg_y, 0.5f);
        printf("frontal_distance : %0.1f\n", frontal_distance);

        // front 상태 확인되었으면
        if (frontal_distance < stop_radius){
            printf("watch out!\n");
            on_neutral2();
            return;
        }else{
            // 기본 주행 속도
            PWM_dc = 340;
            selectedChannel = ESC_CHANNEL;
            updatePWM(ESC_CHANNEL);
        }

        // 90도 좌회전, 90도 우회전 상황에 대한 처리를 이 부분에서 하도록
        // 우선 직, 좌, 우에 대한 우선 순위를 매기도록 합시다.
        // p의 꼬리에서 출발하는 p턴을 생각해보면
        // 우회전 > 직진
        // 좌회전 > 우회전
        // 직진 > 좌회전
        // 일종의 가위바위보와 같은 구조이다.
        // 이를 구현하는 단계

        // 실시간 방향 전환을 위한 부분
        avg_x = 0.0f;
        avg_y = 0.0f;
        // aiming 벡터, 전방 벡터를 기준으로 aiming 벡터가 어느 각도(부호 고려)만큼 벌어져 있는 지 확인하고 그 각도에 비례하여 steering하도록 하기 위함
        for(int i=0; i<6; i++){
            avg_x +=point_avg[i].first/6;
            avg_y +=point_avg[i].second/6;
        }

        // 기준 벡터(전방 벡터)
        float standard_x = 0.0f;
        float standard_y = 1.0f;

        // 벡터 계산
        float inner_product = standard_x*avg_x + standard_y * avg_y;
        float outter_product = standard_x*avg_y-standard_y*avg_x;
        float abs_avg = powf(avg_x * avg_x + avg_y * avg_y, 0.5f);
        float abs_std = powf(standard_x * standard_x + standard_y * standard_y, 0.5f);

        float cosine = inner_product/(abs_avg * abs_std);
        float sine = outter_product/(abs_avg * abs_std); // 각도의 부호 확인용
        float theta = acos(cosine) * 180/M_PI; // 각도의 크기 확인용

        printf("theta : %0.1f\n", theta);
        printf("sine : %0.1f\n", sine);
        printf("cosine : %0.1f\n", cosine);

        if(sine >= 0.0f){
            // go left

            PWM_sv = STEERING_CENTER-theta*theta/5.5;
            // 이 부분, theta에 대해 선형적(1차)으로 변하게 끔 해놨지만 일단,
            // 비선형적으로 변하게 수정해야한다.
            // theta가 클수록, 즉 방향 전환해야할 정도가 클수록, PWM_sv를 크게 바꿔야 하기 때문에
            // STEERING_CENTER - theta*theta / constant 이런식으로 비선형적으로 구성하고 적절한 constant를 구해야한다고 생각
            // 아래  else 내부에 있는 부분도 마찬가지.
            selectedChannel = STEERING_CHANNEL;
            updatePWM(STEERING_CHANNEL) ;
        }else {
            // go right

            PWM_sv = STEERING_CENTER + theta*theta/5.5 ;
            selectedChannel = STEERING_CHANNEL;
            updatePWM(STEERING_CHANNEL) ;
        }

}


int map_func(int angle_on_standard_coordinate){
    return 270-angle_on_standard_coordinate;
}




void callback_person(const darknet_ros_msgs::BoundingBoxes::ConstPtr  &msg){
    int count = 0;
    int n;
    n = msg->bounding_boxes.size();
    //n = 0;
    for(int i = 0; i < n; i++){
       // printf("for ok\n");
        if (std::strcmp(msg->bounding_boxes[i].Class.c_str(), "person") == 0){
       //     count = 1;
			flag_p = 1;
			//printf("flag_p change! %d\n",flag_p);
        	return;
        }
    }
    //if (count == 1){
      //  printf("person ok\n");
       // on_neutral();
    //}
	
	//printf("flag_p change! not person !\n");
	flag_p = 0;

}


int main(int argc, char **argv){
    init_pca9685();

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("keys", 1, callback);
    ros::Subscriber sub_person = n.subscribe("darknet_ros/bounding_boxes", 1, callback_person);
    ros::Subscriber sub_lidar = n.subscribe("scan", 1, callback_lidar_twk2);
    ros::spin();
 
    return 0;
}
