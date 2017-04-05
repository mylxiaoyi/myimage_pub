#include <gflags/gflags.h>
#include <iostream>
#include <ros/ros.h>

#include "imagepublisher.h"

DEFINE_string(act, "camera", "publish images from camera");
DEFINE_int32(camId, 0, "cameraId used to publish images");
DEFINE_bool(gray, false, "publish gray images");
DEFINE_string(dir, "", "image dir name");
DEFINE_string(image_ext, "png", "extension of image files");
DEFINE_string(video, "", "video file to publish images");
DEFINE_string(topic, "/camera/image_raw", "topic name");

void showHelp(const char * name) {
    gflags::ShowUsageWithFlagsRestrict(name, "myimage_pub_node");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "myimage_pub_node");

    gflags::SetUsageMessage("\n"
                            "usage: myimage_pub_node\n"
                            "-topic topic_name\n"
                            "-act camera -camId cameraId -gray [true|false]\n"
                            "-act openni2 -gray [true|false]\n"
                            "-act dir -dir dir -image_ext [png|jpg] -gray [true|false]\n"
                            "-act video -video video_file -gray [true|false]\n"
                            );

    gflags::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_act.empty()) {
        std::cout << "FLAGS_act is empty" << std::endl;
        showHelp(argv[0]);
        return 0;
    }

    if (FLAGS_act == "camera")
        ImagePublisher::getInstance()->publishCamera(FLAGS_topic, FLAGS_camId, FLAGS_gray);
    else if (FLAGS_act == "dir")
        ImagePublisher::getInstance()->publishDir(FLAGS_topic, FLAGS_dir, FLAGS_image_ext, FLAGS_gray);
    else if (FLAGS_act == "video")
        ImagePublisher::getInstance()->publishVideo(FLAGS_topic, FLAGS_video, FLAGS_gray);
    else
        showHelp(argv[0]);

    ros::shutdown();

    std::cout << "Done" << std::endl;

    return 0;
}
