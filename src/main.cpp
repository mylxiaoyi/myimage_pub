#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cout << "usage: " << argv[0] << " topic cameraId [gray]" << std::endl;
        return 0;
    }
    ros::init(argc, argv, "myimage_pub");

    ros::NodeHandle nh;
    
    int cameraId = 0;
    std::string topic = "myimage_raw";
    bool b_gray = false;

    if (argc >= 2)
        topic = argv[1];

    if (argc >= 3)
        cameraId = atoi(argv[2]);

    if (argc >= 4 && strncmp(argv[3], "gray", 4) == 0)
        b_gray = true;

    image_transport::ImageTransport it(nh);

    image_transport::Publisher image_pub = it.advertise(topic, 1);

    cv::VideoCapture vc(cameraId);

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

    cv::Mat img;
    while (1) {
        vc >> img;

        if (b_gray)
            cv::cvtColor(img, img, CV_BGR2GRAY);

        frame->image = img;

        if (!frame->image.empty()) {
            frame->header.stamp = ros::Time::now();
            image_pub.publish(frame->toImageMsg());
        }

        cv::imshow("Image", frame->image);

        if (cv::waitKey(1) == 27)
            break;
    }

    ros::shutdown();

    return 0;
}
