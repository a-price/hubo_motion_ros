
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


void spacenav_callback(const sensor_msgs::JoyConstPtr joystick)
{
    std::cerr << "RECEIVED SPACENAV" << std::endl;

}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "spacenav_relay");

    ros::NodeHandle nh;

    ros::Subscriber spacenavJoy = nh.subscribe("spacenav/joy", 1,
                                               &spacenav_callback);


    ros::spin();
}

