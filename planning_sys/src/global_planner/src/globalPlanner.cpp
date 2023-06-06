#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <iostream>

#define RATE 1

int main(int argc, char** argv){
    std::cout<<"Starting..."<<std::endl;
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;

    // publish the time every second
    ros::Publisher time_pub = nh.advertise<rosgraph_msgs::Clock>("clock", 10);
    rosgraph_msgs::Clock clock_msg;
    ros::Rate loop_rate(RATE);

    while(ros::ok()){
        clock_msg.clock = ros::Time::now();
        time_pub.publish(clock_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}