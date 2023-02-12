#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "adrc_process.h"


int main(int argc, char ** argv)
{   
    ros::init(argc, argv, "adrc");

    ros::NodeHandle nh_("~");

    AdrcProcess adrc_process = AdrcProcess();
    adrc_process.init(nh_);

    while(ros::ok)
    {
        ros::spinOnce();
    }

    return 0;
}
