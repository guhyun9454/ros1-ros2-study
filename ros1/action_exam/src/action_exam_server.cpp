#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_exam/PrimenumberAction.h>


class PrimenumberAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<action_exam::PrimenumberAction> as_;
  std::string action_name_;
  action_exam::PrimenumberFeedback feedback_;
  action_exam::PrimenumberResult result_;

public:
  PrimenumberAction(std::string name) :
    as_(nh_, name, boost::bind(&PrimenumberAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  void executeCB(const action_exam::PrimenumberGoalConstPtr &goal)
  {
    feedback_.cur_num = 0;
    result_.sequence.clear();
    ROS_INFO("We will find the prime number from 2 to 30");
  
    bool success = true;
    for (int i = 2; i <= 30; ++i)    {
      if (as_.isPreemptRequested() || !ros::ok()){
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        success = false;
        break;
      }
  
      bool isPrime = true;
      for (int j = 2; j < i; ++j){
        if (i % j == 0){
          isPrime = false;
          break;
        }
      }
  
      if (isPrime){
        ROS_INFO("Find prime number : %d", i);
        result_.sequence.push_back(i);
        feedback_.cur_num = i;
        as_.publishFeedback(feedback_);
      }
  
      ros::Duration(0.1).sleep();
    }
  
    if (success){
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded(result_);
    }
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_exam_server");
  PrimenumberAction Primenumber("action_exam_action");
  ros::spin();
  return 0;
}
