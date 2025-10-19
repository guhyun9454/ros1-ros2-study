#include "ros/ros.h"
#include "service_exam/myservice.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_exam_client");
  if (argc != 3) {
    ROS_INFO("usage: service_exam_client num1 num2");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<service_exam::myservice>("my_service");
  service_exam::myservice srv;
  srv.request.a = atoi(argv[1]);
  srv.request.b = atoi(argv[2]);

  if (client.call(srv)) {
    ROS_INFO("Sum: %ld", (long int)srv.response.result);
  } else {
    ROS_ERROR("Failed to call service my_service");
    return 1;
  }

  return 0;
}
