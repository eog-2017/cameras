#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

cv_bridge::CvImageConstPtr img_color, img_ir, img_depth;

void depth(const sensor_msgs::Image::ConstPtr& msg)
{
    img_depth = cv_bridge::toCvShare(msg);

    double max = 0.0, min=0.0;
    cv::minMaxLoc(img_depth->image, &min, &max, 0, 0);

    cv::Mat depth_normalized;
    img_depth->image.convertTo(depth_normalized, CV_32F, 1.0/max, 0)  ;
    //depth_normalized = 50 * depth_normalized;

    cv::imshow("depth in gray", depth_normalized);
    cv::waitKey(1);

    cv::Mat rgb_mapped(480,640,CV_32FC3);

    for(int i=0;i<480;i++)
    for(int j=0;j<640;j++)
    {
      float* pixel = (float*)(rgb_mapped.data+ rgb_mapped.step[0]*i + rgb_mapped.step[1]*j);
      float* pixel_depth = (float*)(img_depth->image.data+ img_depth->image.step[0]*i + img_depth->image.step[1]*j);

    pixel[0] = (pixel_depth[0]/max)*360.0;  //Hue varies from 0 to 360 degree, change multiplying factor to change color
    pixel[1]= 1.0f;                         //Saturation from 0.0 to 1.0
    pixel[2] = 0.8f;                        //Value from 0.0 to 1.0
    }

    cv::cvtColor(rgb_mapped,rgb_mapped, CV_HSV2BGR);
    cv::imshow("depth in HSV", rgb_mapped);

    cv::waitKey(1);
}

void ir(const sensor_msgs::Image::ConstPtr& msg)
{
    img_ir = cv_bridge::toCvShare(msg);

    double max = 0.0, min = 0.0;
    cv::minMaxLoc(img_ir->image, &min, &max, 0, 0);

    int w = img_ir->image.cols;
    int h = img_ir->image.rows;

    cv::Mat normalized(h, w, CV_32F);

    for(size_t y = 0 ; y < h ; ++y) {
        for(size_t x = 0 ; x < w ; ++x) {
                normalized.at<float>(y,x) = img_ir->image.at<uint16_t>(y,x)/max;
        }
    }

    cv::imshow("ir", normalized);
    cv::waitKey(1);
}

void color(const sensor_msgs::Image::ConstPtr& msg)
{
    img_color = cv_bridge::toCvShare(msg);

    cv::imshow("color", img_color->image);
    cv::waitKey(1);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Komal");

    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("camera/depth/image", 1, depth);

    //------------------------- IR and RGB images cannot be received at same time ------------------------//

//    ros::Subscriber sub2 = nh.subscribe("camera/ir/image_raw", 1, ir);
    ros::Subscriber sub3 = nh.subscribe("camera/rgb/image_color", 1, color);

    ros::spin();

    return 0;
}


