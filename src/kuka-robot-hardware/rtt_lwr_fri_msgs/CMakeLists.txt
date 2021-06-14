cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_fri_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(lwr_fri_msgs)
ros_generate_rtt_service_proxies(lwr_fri_msgs)

orocos_generate_package(
  DEPENDS lwr_fri_msgs
  DEPENDS_TARGETS rtt_roscomm 
)
