#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cout << "usage: " << argv[0] << " topic video_path" << std::endl;
        return 0;
    }
    ros::init(argc, argv, "myimage_pub");

    ros::NodeHandle nh;
    
    std::string topic = "myimage_raw";
    bool b_gray = false;

    if (argc >= 2)
        topic = argv[1];

    std::string video_path(argv[2]);

    image_transport::ImageTransport it(nh);

    image_transport::Publisher image_pub = it.advertise(topic, 1);

    cv::VideoCapture vc(video_path);

    if (!vc.isOpened()) {
        ROS_ERROR_STREAM("open video error");
        ros::shutdown();
    }

    cv_bridge::CvImagePtr frame;
    frame = boost::make_shared<cv_bridge::CvImage>();
    if (b_gray)
        frame->encoding = sensor_msgs::image_encodings::MONO8;
    else
        frame->encoding = sensor_msgs::image_encodings::BGR8;

    ros::Rate rate(10);
    cv::Mat img;
    while (1) {
        vc >> img;

        if (img.empty())
            continue;

        frame->image = img;

        if (!frame->image.empty()) {
            frame->header.stamp = ros::Time::now();
            image_pub.publish(frame->toImageMsg());
        }

        cv::imshow("Image", frame->image);

        if (cv::waitKey(1) == 27)
            break;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}
