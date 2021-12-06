#include "ros/ros.h"

#include <iostream>
#include <cstdio>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"
#include"geometry_msgs/Twist.h"
#include"std_msgs/Int16.h"

#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection2D.h>

//  for publish image topic
#include <image_transport/image_transport.h>

//ros::Publisher twist_pub;

cv::Mat subscribed_image;
geometry_msgs::Twist cmd_vel;
std_msgs::Int16 shake;

static const std::vector<std::string> labelMap = {
    "ball",        "house",      "meat",        "yasue",        "miyasaka"
};

//  detected object's Data struct
struct DetectionData {
    //  label id
    int id = -1;
    //  center x,y
    double center_x;
    double center_y;
    //  size x,y
    double size_x;
    double size_y;

    double score;
};

// detected objects
std::vector<DetectionData> ball_datas;
std::vector<DetectionData> house_datas;
std::vector<DetectionData> meat_datas;
std::vector<DetectionData> yasue_datas;
std::vector<DetectionData> miyasaka_datas;

int image_width , image_height;

void compare_scores(std::vector<DetectionData> _datas, int* _array_num, int* _count )
{
    double score_max = 0;
    *_array_num = 0; *_count = 0;
    for(auto&& x : _datas){
        if(score_max < x.score){
            score_max = x.score;
            *_array_num = *_count;
        }
        *_count = *_count + 1;
    }
}

double yaw_move(std::vector<DetectionData> _datas, int _array_num){
    double x = _datas[_array_num].center_x - (double)image_width/2;
    double yaw = -(x / image_width);
    if(yaw >= -0.05&&yaw <0){
        yaw = 0.0;
    } else if (yaw > 0 && yaw <= 0.05){
        yaw = 0.0;
    } else if (yaw > 0.7) {
        yaw = 0.7;
    } else if (yaw < -0.7) {
        yaw = -0.7;
    }


    return yaw/3;
}

int off_screen(double _yaw){
    int flg;
    if( _yaw > 0) {
        flg = 1;
    } else if( _yaw < 0){
        flg = 2;
    }
    return flg;
}

double ver_move(std::vector<DetectionData> _datas, int _array_num, double target_size, int _a){
    double size = _datas[_array_num].size_x * _datas[_array_num].size_y;
    double ver;

    if(target_size < size){
        ver = 0.05;
    } else if (target_size > size){
        ver = -target_size/size/_a;
    }

    if (ver < -0.70){
        ver = -0.70;
    } else if(-0.07<ver&&ver<0){
        ver = -0.07;
    }


    return ver;
}


void imageCallback(const sensor_msgs::Image& msg)
{
    try{
        subscribed_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        image_width = subscribed_image.cols;
        image_height = subscribed_image.rows;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int ofsc_flg = 0, flg = 0, shake_flg = 0, count1 = 0, count2 = 0;

void detectionCallback(const vision_msgs::Detection2DArray& msg)
{ 
    double yaw, ver;
    // Check if detections[] is empty or not
    if(!msg.detections.empty()) {
        //  insert dataros opencv
        ball_datas.clear();  //  clear buffer
        house_datas.clear();
        meat_datas.clear();
        yasue_datas.clear();
        miyasaka_datas.clear();
        for(vision_msgs::Detection2D _d : msg.detections) {  //  insert data
            DetectionData data = {(int)_d.results[0].id, _d.bbox.center.x, _d.bbox.center.x, _d.bbox.size_x, _d.bbox.size_y, _d.results[0].score};
            if (data.score > 0.80){ 
                if(data.id == 0){
                    ball_datas.push_back(data);
                } else if (data.id == 1){
                    house_datas.push_back(data);
                } else if (data.id == 2){
                    meat_datas.push_back(data);
                } else if (data.id == 3){
                    yasue_datas.push_back(data);
                } else if (data.id == 4){
                    miyasaka_datas.push_back(data);
                }
            }
        }
    }
    if(flg == 0){
        if(!ball_datas.empty()){
            int array_co = 0, array_num = 0.0;
            compare_scores(ball_datas, &array_num, &array_co);
            if(array_co != 0){
                double size = ball_datas[array_num].size_x * ball_datas[array_num].size_y;
                double x =  ball_datas[array_num].center_x - (double)image_width/2;
                yaw = yaw_move(ball_datas, array_num);
                ofsc_flg = off_screen(yaw);
                if (0.3 > yaw && yaw > -0.3){
                    ver = ver_move(ball_datas, array_num, 70000, 50);
                } else {
                    ver = 0;
                }

                shake.data = 0;

                if(70000<size && size<75000 && -50<x && x<50){
                    flg = 1;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg = 0;
                }
                ROS_INFO("%f, %f",size, x);
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.2;
            } else if(ofsc_flg==2){
                yaw = -0.2;
            } else if(ofsc_flg==3){
                yaw = 0.05;
            } else if(ofsc_flg==4){
                yaw = -0.05;
            } else {
                yaw = 0;
            }
            ver = 0;
        }
    } else if(flg == 1){
        if(shake_flg<10){
            shake.data = 1;
            shake_flg = 0;
        } else {
            shake.data = 0;
        }

        if(!ball_datas.empty()){
            count1=0;
            count2++;
        } else {
            count1++;
            count2=0;
        }

        if(count2 >= 100){
            flg=0;
            count2=0;
        }

        if(count1 >=50){
            flg=2;
            count1=0;
        } 
    } else if(flg == 2){
        if(!miyasaka_datas.empty()){
            int array_co = 0, array_num = 0.0;
            compare_scores(miyasaka_datas, &array_num, &array_co);
            if(array_co != 0){
                yaw = yaw_move(miyasaka_datas, array_num);
                ofsc_flg = off_screen(yaw);
                if (0.15 > yaw && yaw > -0.15){
                    ver = ver_move(miyasaka_datas, array_num, 50000, 50);
                } else {
                    ver = 0;
                }

                double size = miyasaka_datas[array_num].size_x * miyasaka_datas[array_num].size_y;
                double x =  miyasaka_datas[array_num].center_x - (double)image_width/2;
                if(50000<size && size<55000 && -50<x && x<50){
                    flg = 3;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg=0;
                }
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.2;
            } else if(ofsc_flg==2){
                yaw = -0.2;
            } else {
                yaw = 0.2;
            }
            ver = 0;
        }
    } else if(flg==3){
        count1 ++;
        if(count1<70){
            yaw = 0;
            ver = 0;
        } else if(count1<90){
            yaw = 0.0;
            ver = 0.2;
        } else if(count1<170){
            yaw = -0.3;
            ver = 0;
        } else {
            yaw = 0;
            ver = 0;
            flg = 0;
            count1 = 0;
        } 
    } else if(flg==4){
        if(!ball_datas.empty()){
            int array_co = 0, array_num = 0.0;
            compare_scores(ball_datas, &array_num, &array_co);
            if(array_co != 0){
                yaw = yaw_move(ball_datas, array_num);
                double size = ball_datas[array_num].size_x * ball_datas[array_num].size_y;
                double x =  ball_datas[array_num].center_x - (double)image_width/2;
                ofsc_flg = off_screen(yaw);
                if (0.3 > yaw && yaw > -0.3){
                    ver = ver_move(ball_datas, array_num, 80000, 50);
                } else {
                    ver = 0;
                }

                if(80000<size && size<90000 && -50<x && x<50){
                    flg = 5;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg = 0;
                }
                ROS_INFO("%f, %f",size, x);
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.2;
            } else if(ofsc_flg==2){
                yaw = -0.2;
            } else {
                yaw = 0;
            }
            ver = 0;
        }
    } else if(flg == 5){
        if(shake_flg<10){
            shake.data = 1;
            shake_flg = 0;
        } else {
            shake.data = 0;
        }

        if(!ball_datas.empty()){
            count1=0;
            count2++;
        } else {
            count1++;
            count2=0;
        }

        if(count2 >= 100){
            flg=4;
            count2=0;
        }

        if(count1 >=50){
            flg=6;
            count1=0;
        } 
    } else if(flg == 6){
        if(!meat_datas.empty()){
            int array_co = 0, array_num = 0.0;
            compare_scores(meat_datas, &array_num, &array_co);
            if(array_co != 0){
                yaw = yaw_move(meat_datas, array_num);
                ofsc_flg = off_screen(yaw);
                if (0.15 > yaw && yaw > -0.15){
                    ver = ver_move(meat_datas, array_num, 25000, 50);
                } else {
                    ver = 0;
                }

                double size = meat_datas[array_num].size_x * meat_datas[array_num].size_y;
                double x =  meat_datas[array_num].center_x - (double)image_width/2;
                if(24000<size && size<25000 && -50<x && x<50){
                    flg = 7;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg=0;
                }
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.2;
            } else if(ofsc_flg==2){
                yaw = -0.2;
            } else {
                yaw = 0.2;
            }
            ver = 0;
        }
    } else if(flg==7){
        count1 ++;
        if(count1<70){
            yaw = 0;
            ver = 0;
        } else if(count1<90){
            yaw = 0.0;
            ver = 0.2;
        } else if(count1<170){
            yaw = -0.3;
            ver = 0;
        } else {
            yaw = 0;
            ver = 0;
            flg = 8;
            count1 = 0;
        } 
    } else if(flg==8){
        if(!meat_datas.empty()){
            int array_co = 0, array_num = 0.0;
            compare_scores(meat_datas, &array_num, &array_co);
            if(array_co != 0){
                yaw = yaw_move(meat_datas, array_num);
                double size = meat_datas[array_num].size_x * meat_datas[array_num].size_y;
                double x =  meat_datas[array_num].center_x - (double)image_width/2;
                ofsc_flg = off_screen(yaw);
                if (0.3 > yaw && yaw > -0.3){
                    ver = ver_move(meat_datas, array_num, 60000, 50);
                } else {
                    ver = 0;
                }

                if(60000<size && size<65000 && -50<x && x<50){
                    flg = 9;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg = 0;
                }
                ROS_INFO("%f, %f",size, x);
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.2;
            } else if(ofsc_flg==2){
                yaw = -0.2;
            } else {
                yaw = 0;
            }
            ver = 0;
            ROS_INFO("a");
        }
    } else if(flg == 9){
        if(shake_flg<10){
            shake.data = 1;
            shake_flg = 0;
        } else {
            shake.data = 0;
        }

        if(!ball_datas.empty()){
            count1=0;
            count2++;
        } else {
            count1++;
            count2=0;
        }

        if(count2 >= 100){
            flg=8;
            count2=0;
        }

        if(count1 >=50){
            flg=10;
            count1=0;
        } 
    } else if(flg == 10){
        if(!house_datas.empty()){
            int array_co = 0, array_num = 0.0;
            compare_scores(house_datas, &array_num, &array_co);
            if(array_co != 0){
                yaw = yaw_move(house_datas, array_num);
                ofsc_flg = off_screen(yaw);
                if (0.15 > yaw && yaw > -0.15){
                    ver = ver_move(house_datas, array_num, 25000, 50);
                } else {
                    ver = 0;
                }

                double size = house_datas[array_num].size_x * house_datas[array_num].size_y;
                double x =  house_datas[array_num].center_x - (double)image_width/2;
                if(24000<size && size<25000 && -50<x && x<50){
                    flg = 11;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg=0;
                }
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.2;
            } else if(ofsc_flg==2){
                yaw = -0.2;
            } else {
                yaw = 0.2;
            }
            ver = 0;
        }
    } else if(flg==11){
        yaw=0;
        ver=0;
    }
    cmd_vel.angular.z = yaw;
    cmd_vel.linear.x = ver;
    //ROS_INFO("%d",flg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "dogrun_yolo");
    ros::NodeHandle pnh("~");

    ros::Subscriber imageSub = pnh.subscribe("/yolo_publisher/color/image", 10, imageCallback);
    ros::Subscriber detectionSub = pnh.subscribe("/yolo_publisher/color/yolo_detections", 10, detectionCallback);
    ros::Rate loop_rate(10);
    ros::Publisher twist_pub = pnh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Publisher dog_pub = pnh.advertise<std_msgs::Int16>("/flagger", 1000);
    while(ros::ok()){
        twist_pub.publish(cmd_vel);
        dog_pub.publish(shake);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}