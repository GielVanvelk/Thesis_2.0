#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include <stdlib.h>



int main(int argc, char **argv) {

ros::init(argc, argv, "talker");
ros::NodeHandle n;
ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("chatter", 100);
ros::Rate loop_rate(10);

  while(ros::ok()) {

    std_msgs::Int32 msg;
    msg.data = 10;
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    }
    return 0;
}
