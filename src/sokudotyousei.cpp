#include "ros/ros.h"

#include <iostream>
#include <cstdio>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"
#include"geometry_msgs/Twist.h"
#include"std_msgs/Int16.h"
#include<std_msgs/Bool.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection2D.h>

//  for publish image topic
#include <image_transport/image_transport.h>

//ros::Publisher twist_pub;

cv::Mat subscribed_image;
geometry_msgs::Twist cmd_vel;
std_msgs::Int16 shake;
std_msgs::Int16 shippo;
std_msgs::Bool nakigoe;

static const std::vector<std::string> labelMap = {
    "ball",        "house",      "meat"
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
    double yaw = -(x / image_width)/3;
    if(yaw >= -0.05&&yaw <0){
        yaw = -0.05;
    } else if (yaw > 0 && yaw <= 0.05){
        yaw = 0.05;
    } else if (yaw > 0.7) {
        yaw = 0.7;
    } else if (yaw < -0.7) {
        yaw = -0.7;
    }

    return yaw;
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

    if (ver < -0.50){
        ver = -0.50;
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

int ofsc_flg = 0, flg = 1, shake_flg = 0, count1 = 0, count2 = 0;

void detectionCallback(const vision_msgs::Detection2DArray& msg)
{ 
    double yaw, ver;
    // Check if detections[] is empty or not
    ball_datas.clear();  //  clear buffer
    house_datas.clear();
    meat_datas.clear();
    if(!msg.detections.empty()) {
        //  insert dataros opencv
        for(vision_msgs::Detection2D _d : msg.detections) {  //  insert data
            DetectionData data = {(int)_d.results[0].id, _d.bbox.center.x, _d.bbox.center.x, _d.bbox.size_x, _d.bbox.size_y, _d.results[0].score};
            if (data.score > 0.7){ 
                if(data.id == 0){
                    ball_datas.push_back(data);
                } else if (data.id == 1){
                    house_datas.push_back(data);
                } else if (data.id == 2){
                    meat_datas.push_back(data);
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

                ver = ver_move(ball_datas, array_num, 75000, 50);

                shake.data = 0;
                shake_flg = 0;
                shippo.data = 1;
                ROS_INFO("%f %f", x, size);

                if(20000<size && x<-100){
                    yaw =  0.05;
                } else if(20000<size && x>100){
                    yaw = -0.05;
                }

                if(60000<size && size<75000 && -100<x && x<100){
                    flg = 1;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg = 0;
                }
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.1;
            } else if(ofsc_flg==2){
                yaw = -0.1;
            } else {
                yaw = 0;
            }
            ver = 0;
        }
    } else if(flg==1){
        ver = 0.0;
        yaw = 0.0;
        shippo.data = 0;
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

        if(count1 >= 200){
            flg=2;
            count1=0;
        }
    } else if(flg==2){
        if(!meat_datas.empty()){
            int array_co = 0, array_num = 0.0;
            compare_scores(meat_datas, &array_num, &array_co);
            if(array_co != 0){
                double size = meat_datas[array_num].size_x * meat_datas[array_num].size_y;
                double x =  meat_datas[array_num].center_x - (double)image_width/2;
                yaw = yaw_move(meat_datas, array_num);
                ofsc_flg = off_screen(yaw);

                ver = ver_move(meat_datas, array_num, 75000, 50);

                shake.data = 0;
                shake_flg = 0;
                shippo.data = 1;
                ROS_INFO("%f %f", x, size);

                if(20000<size && x<-100){
                    yaw =  0.05;
                } else if(20000<size && x>100){
                    yaw = -0.05;
                }

                if(60000<size && size<75000 && -100<x && x<100){
                    flg = 3;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg = 0;
                }
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.1;
            } else if(ofsc_flg==2){
                yaw = -0.1;
            } else {
                yaw = 0;
            }
            ver = 0.1;
        }
    } else if(flg==3){
        count1 ++;
        if(count1<3){
            nakigoe.data = true;
            yaw = 0;
            ver = 0;
        }else if(count1<70){
            nakigoe.data = false;
            yaw = 0.0;
            ver = 0.0;
        } else if(count1<90){
            nakigoe.data = false;
            yaw = 0.0;
            ver = 0.2;
        } else if(count1<170){
            yaw = -0.3;
            ver = 0;
        } else {
            yaw = 0;
            ver = 0;
            flg = 4;
            count1 = 0;
        }
    } else if(flg == 4){
        if(!ball_datas.empty()){
            int array_co = 0, array_num = 0.0;
            compare_scores(ball_datas, &array_num, &array_co);
            if(array_co != 0){
                double size = ball_datas[array_num].size_x * ball_datas[array_num].size_y;
                double x =  ball_datas[array_num].center_x - (double)image_width/2;
                yaw = yaw_move(ball_datas, array_num);
                ofsc_flg = off_screen(yaw);

                ver = ver_move(ball_datas, array_num, 75000, 50);

                shake.data = 0;
                shake_flg = 0;
                shippo.data = 1;
                ROS_INFO("%f %f", x, size);

                if(20000<size && x<-100){
                    yaw =  0.05;
                } else if(20000<size && x>100){
                    yaw = -0.05;
                }

                if(60000<size && size<75000 && -100<x && x<100){
                    flg = 5;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg = 0;
                }
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.1;
            } else if(ofsc_flg==2){
                yaw = -0.1;
            } else {
                yaw = 0;
            }
            ver = 0;
        }
    } else if(flg==1){
        ver = 0.0;
        yaw = 0.0;
        shippo.data = 0;
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

        if(count1 >= 200){
            flg=2;
            count1=0;
        }
    } else if(flg==2){
        if(!meat_datas.empty()){
            int array_co = 0, array_num = 0.0;
            compare_scores(meat_datas, &array_num, &array_co);
            if(array_co != 0){
                double size = meat_datas[array_num].size_x * meat_datas[array_num].size_y;
                double x =  meat_datas[array_num].center_x - (double)image_width/2;
                yaw = yaw_move(meat_datas, array_num);
                ofsc_flg = off_screen(yaw);

                ver = ver_move(meat_datas, array_num, 75000, 50);

                shake.data = 0;
                shake_flg = 0;
                shippo.data = 1;
                ROS_INFO("%f %f", x, size);

                if(20000<size && x<-100){
                    yaw =  0.05;
                } else if(20000<size && x>100){
                    yaw = -0.05;
                }

                if(60000<size && size<75000 && -100<x && x<100){
                    flg = 3;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg = 0;
                }
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.1;
            } else if(ofsc_flg==2){
                yaw = -0.1;
            } else {
                yaw = 0;
            }
            ver = 0.1;
        }
    } else if(flg==3){
        count1 ++;
        if(count1<3){
            nakigoe.data = true;
            yaw = 0;
            ver = 0;
        }else if(count1<70){
            nakigoe.data = false;
            yaw = 0.0;
            ver = 0.0;
        } else if(count1<90){
            nakigoe.data = false;
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
    } else if(flg == 0){
        if(!ball_datas.empty()){
            int array_co = 0, array_num = 0.0;
            compare_scores(ball_datas, &array_num, &array_co);
            if(array_co != 0){
                double size = ball_datas[array_num].size_x * ball_datas[array_num].size_y;
                double x =  ball_datas[array_num].center_x - (double)image_width/2;
                yaw = yaw_move(ball_datas, array_num);
                ofsc_flg = off_screen(yaw);

                ver = ver_move(ball_datas, array_num, 75000, 50);

                shake.data = 0;
                shake_flg = 0;
                shippo.data = 1;
                ROS_INFO("%f %f", x, size);

                if(20000<size && x<-100){
                    yaw =  0.05;
                } else if(20000<size && x>100){
                    yaw = -0.05;
                }

                if(60000<size && size<75000 && -100<x && x<100){
                    flg = 1;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg = 0;
                }
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.1;
            } else if(ofsc_flg==2){
                yaw = -0.1;
            } else {
                yaw = 0;
            }
            ver = 0;
        }
    } else if(flg==1){
        ver = 0.0;
        yaw = 0.0;
        shippo.data = 0;
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

        if(count1 >= 200){
            flg=2;
            count1=0;
        }
    } else if(flg==2){
        if(!house_datas.empty()){
            int array_co = 0, array_num = 0.0;
            compare_scores(house_datas, &array_num, &array_co);
            if(array_co != 0){
                double size = house_datas[array_num].size_x * house_datas[array_num].size_y;
                double x =  house_datas[array_num].center_x - (double)image_width/2;
                yaw = yaw_move(house_datas, array_num);
                ofsc_flg = off_screen(yaw);

                ver = ver_move(meat_datas, array_num, 100000, 50);

                shake.data = 0;
                shake_flg = 0;
                shippo.data = 1;
                ROS_INFO("%f %f", x, size);

                if(30000<size && x<-100){
                    yaw =  0.05;
                } else if(30000<size && x>100){
                    yaw = -0.05;
                }

                if(house_datas[array_num].size_x > 200 && -100<x && x<100){
                    flg = 3;
                    yaw = 0;
                    ver = 0;
                    ofsc_flg = 0;
                }
            }
        } else {
            if(ofsc_flg==1){
                yaw = 0.1;
            } else if(ofsc_flg==2){
                yaw = -0.1;
            } else {
                yaw = 0.1;
            }
            ver = 0;
        }
    } else if(flg==3){
        count1 ++;
        if(count1<3){
            nakigoe.data = true;
            yaw = 0;
            ver = 0;
        }else if(count1<70){
            nakigoe.data = false;
            yaw = 0.0;
            ver = 0.0;
        } else if(count1<90){
            nakigoe.data = false;
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
    }
    cmd_vel.angular.z = yaw;
    cmd_vel.linear.x = ver;
    ROS_INFO("%d",flg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "dogrun_yolo");
    ros::NodeHandle pnh("~");

    ros::Subscriber imageSub = pnh.subscribe("/yolo_publisher/color/image", 10, imageCallback);
    ros::Subscriber detectionSub = pnh.subscribe("/yolo_publisher/color/yolo_detections", 10, detectionCallback);
    ros::Rate loop_rate(10);
    ros::Publisher twist_pub = pnh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Publisher dog_pub = pnh.advertise<std_msgs::Int16>("/flagger", 1000);
    ros::Publisher shippo_pub = pnh.advertise<std_msgs::Int16>("/shippo_flag", 1000);
    ros::Publisher bowwow_pub = pnh.advertise<std_msgs::Bool>("/bowwowflag", 1000);
    while(ros::ok()){
        twist_pub.publish(cmd_vel);
        dog_pub.publish(shake);
        shippo_pub.publish(shippo);
        bowwow_pub.publish(nakigoe);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}