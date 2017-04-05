#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(directory, "", "directory contains images");
DEFINE_string(topic, "/camera/image_raw", "topic published");
DEFINE_string(image_ext, ".jpg", "image extension");
DEFINE_bool(gray, false, "wheather to publish gray scale image");
DEFINE_bool(interactive, false, "wheather to interactive mannuly");

DECLARE_string(directory);
DECLARE_string(topic);
DECLARE_string(image_ext);
DECLARE_bool(gray);
DECLARE_bool(interactive);

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

//    CHECK_EQ(FLAGS_directory, "") << "please enter directory";

    ros::init(argc, argv, "myimage_pub");

    ros::NodeHandle nh;
    ros::Rate loop_rate(5); 
    boost::filesystem::path image_path(FLAGS_directory);

    std::string topic = FLAGS_topic;
    std::string image_ext = FLAGS_image_ext;
    //image_ext.replace(0, 1, ".");

    bool b_gray = FLAGS_gray;
    bool b_interactive = FLAGS_interactive;

    ROS_INFO("image_path = %s", image_path.string().c_str());
    ROS_INFO("topic      = %s", topic.c_str());
    ROS_INFO("image_ext  = %s", image_ext.c_str());
    ROS_INFO("b_gray     = %d", b_gray);

    image_transport::ImageTransport it(nh);

    image_transport::Publisher image_pub = it.advertise(topic, 10);

    cv_bridge::CvImagePtr frame;
    frame = boost::make_shared<cv_bridge::CvImage>();
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    std::vector<std::string> image_list;

    for (auto && x : boost::filesystem::directory_iterator(image_path))
        image_list.push_back(x.path().string());

    std::sort(image_list.begin(), image_list.end());

    ROS_INFO("image_list.size = %d", image_list.size());
    int count = 1;
    cv::Mat img;
    for (std::string image_name : image_list) {
        ROS_INFO("image_name = %s (%d/%d)", image_name.c_str(), count++, image_list.size());
        if (ros::ok()) {
            img = cv::imread(image_name);
            if (b_gray)
                cv::cvtColor(img, img, CV_BGR2GRAY);

            frame->image = img;

            if (!frame->image.empty()) {
                frame->header.stamp = ros::Time::now();
                image_pub.publish(frame->toImageMsg());
            }

            cv::imshow("Image", img);

            if (b_interactive)
            {
                if (cv::waitKey() == 27)
                    break;
            }
            else
            {
                if (cv::waitKey(1) == 27)
                    break;
            }

            loop_rate.sleep();
        }
    }

    ros::shutdown();

    return 0;
}
