#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cout << "usage: " << argv[0] << " directory topic" << std::endl;
        return 0;
    }

    ros::init(argc, argv, "myimage_pub");

    ros::NodeHandle nh;
    ros::Rate loop_rate(5); 

    std::string file_path(argv[1]);
    std::string topic = argv[2];

    ROS_INFO_STREAM("file_path = " << file_path);
    ROS_INFO_STREAM("topic = " << topic);

    std::ifstream troj_in(file_path+"/trajectory.txt");
    if (!troj_in.is_open()) {
        ROS_ERROR_STREAM("can not open trajectory.txt");
    }

    image_transport::ImageTransport it(nh);

    image_transport::Publisher image_pub = it.advertise(topic, 10);
    tf::TransformBroadcaster br;

    cv_bridge::CvImagePtr frame;
    frame = boost::make_shared<cv_bridge::CvImage>();
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    cv::Mat img;
    std::string line;
    while (!troj_in.eof()) {
        std::getline(troj_in, line);
        if (line.empty()) {
            ROS_INFO_STREAM("end of trajectory");
            break;
        }

        std::stringstream ss(line);
        std::string image_name;
        float x, y, z;
        float wx, wy, wz, w;
        ss >> image_name >> x >> y >> z
            >> wx >> wy >> wz >> w;

        if (ros::ok()) {
            img = cv::imread(file_path + "/rgb/" + image_name + ".jpg");

            frame->image = img;

            if (!frame->image.empty()) {
                frame->header.stamp = ros::Time::now();
                frame->header.frame_id = "frame";
                image_pub.publish(frame->toImageMsg());

                tf::Transform transform(tf::Quaternion(wx, wy, wz, w), tf::Vector3(x, y, z));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "frame"));

                cv::imshow("Image", img);
            }

            if (cv::waitKey(1) == 27)
                break;

            loop_rate.sleep();
        }
    }

    ROS_INFO_STREAM("Done");
    ros::shutdown();

    return 0;
}
