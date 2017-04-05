#ifndef IMAGESUBSCRIBER_H
#define IMAGESUBSCRIBER_H

#include <memory>
#include "zhelpers.hpp"

namespace cv
{
    class Mat;
}

class ImageSubscriber
{
public:
    ImageSubscriber();
    virtual ~ImageSubscriber();

    void subscribeImage();

private:
    zmq::context_t context;
    std::shared_ptr<zmq::socket_t> pSubscriber;

    cv::Mat convertStringToMat(const std::string & img_str);
};

#endif // IMAGESUBSCRIBER_H
