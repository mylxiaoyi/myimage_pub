#include "imagesubscriber.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <sstream>

ImageSubscriber::ImageSubscriber () : context (1)
{
    pSubscriber.reset (new zmq::socket_t (context, ZMQ_SUB));
    pSubscriber->connect ("tcp://localhost:5563");

    pSubscriber->setsockopt (ZMQ_SUBSCRIBE, "/camera/image_raw", 1);
}

ImageSubscriber::~ImageSubscriber ()
{
}

void ImageSubscriber::subscribeImage ()
{
    while (true)
    {
        std::string topic = s_recv(*pSubscriber);
        std::string img_str = s_recv(*pSubscriber);
        cv::Mat img = convertStringToMat(img_str);
        if (img.empty())
            continue;

        cv::imshow("image", img);
        if (cv::waitKey(1) == 27)
            break;
    }
}

cv::Mat ImageSubscriber::convertStringToMat (const std::string& img_str)
{
    std::stringstream ss (img_str);
    cereal::BinaryInputArchive ia (ss);

    int rows, cols, channels;
    std::vector<unsigned char> data;
    ia(rows, cols, channels, data);

    if (channels == 1)
        return cv::Mat(rows, cols, CV_8UC1, data.data());
    else if (channels == 3)
        return cv::Mat(rows, cols, CV_8UC3, data.data());
    else
        return cv::Mat();
}
