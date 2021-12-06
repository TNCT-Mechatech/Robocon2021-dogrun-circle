#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include"geometry_msgs/Twist.h"
#include"std_msgs/Int16.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

using namespace sensor_msgs;
using namespace message_filters;
geometry_msgs::Twist cmd_vel;
std_msgs::Int16 shake;

int flg = 0, count1 = 0, count2 = 0,shake_flg = 0;
//赤い部分の検出
void red_recognize(cv::Mat& hsv_image , cv::Mat& thresh_image)
{
    int RlowH1 = 175;
    int RhighH1 = 179;

    int RlowH2 = 0;
    int RhighH2 = 5;

    int RlowS = 190;
    int RhighS = 255;

    int RlowV = 50;
    int RhighV =225;

    cv::Mat thresh_image1, thresh_image2;

    cv::inRange(hsv_image, cv::Scalar(RlowH1, RlowS, RlowV), cv::Scalar(RhighH1,RhighS,RhighV),thresh_image1);
    cv::inRange(hsv_image, cv::Scalar(RlowH2, RlowS, RlowV), cv::Scalar(RhighH2,RhighS,RhighV),thresh_image2);
    thresh_image = thresh_image1 | thresh_image2;
    cv::GaussianBlur(thresh_image, thresh_image, cv::Size(3,3), 0);
    cv::dilate(thresh_image, thresh_image, 0);
    cv::erode(thresh_image, thresh_image, 0);
}
//黄色い部分の検出
void yellow_recognize(cv::Mat& hsv_image , cv::Mat& thresh_image)
{
    int YlowH = 20;
    int YhighH = 30;

    int YlowS = 190;
    int YhighS = 255;

    int YlowV = 50;
    int YhighV = 255;
    cv::inRange(hsv_image, cv::Scalar(YlowH, YlowS, YlowV), cv::Scalar(YhighH, YhighS, YhighV),thresh_image);
    cv::GaussianBlur(thresh_image, thresh_image, cv::Size(3,3), 0);
    cv::dilate(thresh_image, thresh_image, 0);
    cv::erode(thresh_image, thresh_image, 0);
}
//円検出
std::vector<cv::Vec3f> circle_recognize(cv::Mat& rgb_image , cv::Mat& thresh_image){
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(
            thresh_image,
            circles,
            CV_HOUGH_GRADIENT,
            1,
            1000,
            100,
            10
        );
    if (circles.size() !=0)
    {
        for(int i=0; i < circles.size(); i++){
                cv::circle(rgb_image,
                    cv::Point((int)circles[i][0], (int)circles[i][1]),(int)circles[i][2], cv::Scalar(255, 0, 0),3);
                cv::circle(rgb_image,
                    cv::Point((int)circles[i][0], (int)circles[i][1]) , 3, cv::Scalar(0,255,0),cv::FILLED);
            }
    }

    return circles;
}

//callbacK関数
void callback(const ImageConstPtr& rgb_msg, const ImageConstPtr& depth_msg)
{
    cv::Mat rgb_image, depth_image, hsv_image, thresh_image;
    double x, distance, yaw, forward;
    try{
        rgb_image = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
        depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::cvtColor (rgb_image, hsv_image, CV_BGR2HSV);

    if(flg==3){
        count1 ++;
        if(count1<70){
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
        } else if(count1<90){
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = 0;
        }else if(count1<170){
            cmd_vel.linear.x=0;
            cmd_vel.angular.z = -0.3;
        } else {
            cmd_vel.linear.x=0;
            cmd_vel.angular.z=0;
            flg=0;
            count1=0;
        }
        red_recognize( hsv_image, thresh_image);
        ROS_INFO("%d",count1);
    }else if(flg==2){
        ROS_INFO("c");
        yellow_recognize( hsv_image, thresh_image);
        std::vector<cv::Vec3f> circles = circle_recognize( rgb_image, thresh_image);
        if(circles.size()!=0){
            distance = 0.001*depth_image.at<u_int16_t>((int)circles[0][1], (int)circles[0][0]);
            x = circles[0][0] - rgb_image.cols/2;
            yaw = -(x / rgb_image.cols);
            if(yaw >= -0.05&&yaw <0){
                yaw = -0.05;
            } else if (yaw > 0 && yaw <= 0.05){
                yaw = 0.05;
            }

            forward = (distance-0.50)/2;
            if(forward>0&&forward<=0.05){
                forward = 0.05;
            } else if (forward >= 0.70){
                forward = 0.70;
            }

            if(distance > 0 && distance < 0.50)
            {
                cmd_vel.linear.x = 0.05;
                cmd_vel.angular.z = yaw;
            } else if(distance == 0.50){
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = yaw;
            } else if(distance > 0.5){
                cmd_vel.linear.x = -forward;
                cmd_vel.angular.z = yaw;
            } else {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }

            if(distance>=0.5&&distance<=0.6&&x<=30&&x>=-30){
                cmd_vel.angular.z=0;
                cmd_vel.linear.x=0;
                flg=3;
            }

        } else {
            cmd_vel.angular.z = -0.2;
            cmd_vel.linear.x = 0;
            x=0;
            distance=0;
        }

        ROS_INFO("%.3f,%.3f",x,distance);

    } else if (flg==1){
        ROS_INFO("b");
        red_recognize( hsv_image, thresh_image);
        std::vector<cv::Vec3f> circles = circle_recognize( rgb_image, thresh_image);
        
        if (shake_flg<10){
            shake.data = 1;
            shake_flg++;
        } else {
            shake.data = 0;
        }

        if(circles.size() != 0){
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
        ROS_INFO("%d,%d",count1,count2);

    } else if (flg==0){
        ROS_INFO("a");
        red_recognize( hsv_image, thresh_image);
        std::vector<cv::Vec3f> circles = circle_recognize( rgb_image, thresh_image);
        if(circles.size()!=0){//円検出時
            distance = 0.001*depth_image.at<u_int16_t>((int)circles[0][1], (int)circles[0][0]);
            x = circles[0][0] - rgb_image.cols/2;
            shake.data = 0;
            shake_flg = 0;

            if(x<5 &&x >-5){
                x = 0;
            } 

            yaw = -(x / rgb_image.cols);
            if(yaw >= -0.05&&yaw <0){
                yaw = -0.05;
            } else if (yaw > 0 && yaw <= 0.05){
                yaw = 0.05;
            }

            forward = (distance-0.29)/2;
            if(forward>0&&forward<=0.08){
                forward = 0.08;
            } else if (forward >= 0.70){
                forward = 0.70;
            }

            if(distance > 0 && distance < 0.28){
                cmd_vel.linear.x = 0.05;
                cmd_vel.angular.z = 0;
            } else if (distance==0.28) {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = yaw;
            } else if (distance > 0.28) {
                cmd_vel.linear.x = -forward;
                cmd_vel.angular.z = yaw;
            } else {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z=0;
            }

            if(distance>=0.28&&distance<=0.31&&x<=20&&x>=-20){
                flg = 1;
                cmd_vel.linear.x=0;
                cmd_vel.angular.z=0;
            }

        } else {//円未検出時
            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
            distance = 0;
            x = 0;
        }
        //ROS_INFO("%d,%d,%.2f,%.2f",flg,count1,distance,x);
        //ROS_INFO("%d,%d",thresh_image.cols,thresh_image.rows);
    }
    //cv::resize(rgb_image, rgb_image, cv::Size(), 1.5, 1.5);
    //cv::resize(thresh_image, thresh_image, cv::Size(), 1.5, 1.5);
    cv::imshow("thresh",thresh_image);
    cv::imshow("rgb",rgb_image);
    //ROS_INFO("%d,%d,%.2f,%.2f",flg,count1,distance,x);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dogrun_circle");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Publisher dog_pub = nh.advertise<std_msgs::Int16>("flagger", 1000);
    message_filters::Subscriber<Image> rgb_sub(nh, "/camera/color/image_raw",10);
    message_filters::Subscriber<Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 10);
    typedef sync_policies::ExactTime<Image, Image> MySyncPolicy;

    while(ros::ok()){
        Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub);
        sync.registerCallback(boost::bind(&callback, _1, _2));
        twist_pub.publish(cmd_vel);
        dog_pub.publish(shake);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}