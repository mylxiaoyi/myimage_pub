#include "imagepublisher.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <sstream>

ImagePublisher * ImagePublisher::instance = NULL;

ImagePublisher::ImagePublisher ()
    : bShouldStop(false), context(1)
{
    pPublisher.reset(new zmq::socket_t(context, ZMQ_PUB));
    pPublisher->bind("tcp://*:5563");
}

ImagePublisher::~ImagePublisher ()
{
}

ImagePublisher* ImagePublisher::getInstance ()
{
    if (instance == NULL) instance = new ImagePublisher;

    return instance;
}

void ImagePublisher::publishCamera (std::string topic, int camId, bool bGray)
{
    cv::VideoCapture vc(camId);

    if (!vc.isOpened()) {
        ROS_ERROR_STREAM("open video error");
        return ;
    }

    //std::thread t(&ImagePublisher::publishImage, this);
    std::thread t(&ImagePublisher::publishZMQImage, this);

    cv::Mat img;
    while (1) {
        vc >> img;

        if (bGray)
            cv::cvtColor(img, img, CV_BGR2GRAY);

        if (img.empty())
            continue;

        {
            std::unique_lock<std::mutex> lock(imageMutex);
            mat_queue.push(img.clone());
        }
        imageReady.notify_all();

        cv::imshow("Image", img);

        if (cv::waitKey(1) == 27)
            break;
    }

    bShouldStop = true;

    std::cout << "waiting publish thread to stop" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    t.join();

    std::cout << "Thread Done" << std::endl;
}

void ImagePublisher::publishDir (std::string topic, std::string image_dir,
                                 std::string image_ext, bool bGray)
{
}

void ImagePublisher::publishVideo(std::string topic, std::string video_file, bool bGray)
{

}

void ImagePublisher::publishImage()
{
    image_transport::ImageTransport it(nh);

    image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 1);

    bool bGray = false;

    cv_bridge::CvImagePtr frame;
    frame = boost::make_shared<cv_bridge::CvImage>();
    if (bGray)
        frame->encoding = sensor_msgs::image_encodings::MONO8;
    else
        frame->encoding = sensor_msgs::image_encodings::BGR8;

    while (!bShouldStop) {
        cv::Mat img;

        {

            std::unique_lock<std::mutex> lock(imageMutex);
            imageReady.wait_for(lock,
                            std::chrono::seconds(1),
                            [&] {return !this->mat_queue.empty();});

            img = mat_queue.front();
            mat_queue.pop();
        }

        if (img.empty())
            continue;

        frame->image = img;
        frame->header.stamp = ros::Time::now();
        image_pub.publish(frame->toImageMsg());
    }
}

void ImagePublisher::publishZMQImage()
{
    while (!bShouldStop)
    {
        cv::Mat img;
        {
            std::unique_lock<std::mutex> lock(imageMutex);
            imageReady.wait_for(lock,
                                std::chrono::seconds(1),
                                [&] {return !this->mat_queue.empty();});

            img = mat_queue.front();
            mat_queue.pop();
        }

        if (img.empty())
            continue;

        std::string img_str = convertMatToString(img);
        s_sendmore(*pPublisher, "/camera/image_raw");
        s_send(*pPublisher, img_str);
    }
}

std::string ImagePublisher::convertMatToString(const cv::Mat &img)
{
    if (img.empty())
        return std::string("");

    int rows, cols, channels;
    std::vector<unsigned char> data;

    rows = img.rows;
    cols = img.cols;
    channels = img.channels();
    data.assign((unsigned char*)img.data,
                (unsigned char*)img.data+rows*cols*channels);

    std::stringstream ss;
    cereal::BinaryOutputArchive oa(ss);
    oa(rows, cols, channels, data);

    return ss.str();
}
