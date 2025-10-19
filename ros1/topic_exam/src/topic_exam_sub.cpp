#include "ros/ros.h"
#include "std_msgs/String.h"

void getCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
	ros::init(argc, argv, "topic_exam_subscriber");
	ros::NodeHandle n;

	ros::Subscriber topic_exam_sub = n.subscribe("topic_exam_message",1000,getCallback);
	ros::spin();

	return 0;
}
