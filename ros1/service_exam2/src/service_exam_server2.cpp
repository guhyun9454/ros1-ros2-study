#include "ros/ros.h"
#include "service_exam2/myservice.h"

#define PLUS 1
#define MINUS 2
#define MULTIPLICATION 3
#define DIVISION 4

int opt = PLUS;

bool server_callback(service_exam2::myservice::Request &req, service_exam2::myservice::Response &res)
{
    if (opt == PLUS) {
        res.result = req.a + req.b;
        ROS_INFO("request: x=%ld + y=%ld", (long int)req.a, (long int)req.b);
    }
    else if (opt == MINUS) {
        res.result = req.a - req.b;
        ROS_INFO("request: x=%ld - y=%ld", (long int)req.a, (long int)req.b);
    }
    else if (opt == MULTIPLICATION) {
        res.result = req.a * req.b;
        ROS_INFO("request: x=%ld * y=%ld", (long int)req.a, (long int)req.b);
    }
    else if (opt == DIVISION) {
        res.result = req.a / req.b;
        ROS_INFO("request: x=%ld / y=%ld", (long int)req.a, (long int)req.b);
    }
    ROS_INFO("sending back response: [%ld]", (long int)res.result);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_exam_server2");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //n.setParam("calculation_method", PLUS);

    ros::ServiceServer service = n.advertiseService("my_service", server_callback);
    ROS_INFO("my_service: ready");

    ros::Rate r(10);
    while (ros::ok())
    {
        pn.getParam("calculation_method", opt);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}