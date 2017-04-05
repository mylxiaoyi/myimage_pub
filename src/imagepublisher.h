#ifndef IMAGEPUBLISHER_H
#define IMAGEPUBLISHER_H

#include <ros/ros.h>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "zhelpers.hpp"

namespace cv
{
    class Mat;
}

class ImagePublisher
{
public:
    static ImagePublisher *getInstance();

    virtual ~ImagePublisher();

    void publishCamera(std::string topic = "/camera/image_raw",
                       int camId = 0, bool bGray = false);
    void publishDir(std::string topic = "/camera/image_raw",
                    std::string image_dir = "",
                    std::string image_ext = "png",
                    bool bGray = false);
    void publishVideo(std::string topic = "/camera/image_raw",
                      std::string video_file = "", bool bGray = false);

private:
    ImagePublisher();

    static ImagePublisher *instance;

    ros::NodeHandle nh;

    std::queue<cv::Mat> mat_queue;
    std::mutex imageMutex;
    std::condition_variable imageReady;

    bool bShouldStop;

    void publishImage();
    void publishZMQImage();

    zmq::context_t context;
    std::shared_ptr<zmq::socket_t> pPublisher;

    std::string convertMatToString(const cv::Mat & img);
};

#endif // IMAGEPUBLISHER_H
