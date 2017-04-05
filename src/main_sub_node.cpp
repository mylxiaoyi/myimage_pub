#include <iostream>
#include <ros/ros.h>
#include "imagesubscriber.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "myimage_sub_node");

   ImageSubscriber imageSub;
   imageSub.subscribeImage();

   ros::shutdown();

   std::cout << "Done" << std::endl;

   return 0;
}
