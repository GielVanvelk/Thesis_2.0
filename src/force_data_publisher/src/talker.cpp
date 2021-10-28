#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include <stdlib.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float64MultiArray.h"
#include <vector>



int main(int argc, char **argv) {

ros::init(argc, argv, "talker");
ros::NodeHandle n;
ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("chatter", 1);
ros::Rate loop_rate(1000);

  while(ros::ok()) {

    std_msgs::Float64MultiArray arr;

    arr.data.clear();
    //for loop, pushing data in the size of the array
    for (int i = 0; i < 6; i++)
    {
        arr.data.push_back(10);
    }

		//Publish array
		chatter_pub.publish(arr);

    ros::spinOnce();

    loop_rate.sleep();
    }
    return 0;
}
