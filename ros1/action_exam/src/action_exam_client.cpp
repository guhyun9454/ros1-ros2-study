#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "action_exam/PrimenumberAction.h"

int get_num = 0;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const action_exam::PrimenumberResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  for (int i = 0; i < get_num; i++) {
    ROS_INFO("%d: %d", i + 1, result->sequence[i]);
  }
}

void activeCb()
{
  ROS_INFO("Goal just went active");
}

void feedbackCb(const action_exam::PrimenumberFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback: current number is %d", feedback->cur_num);
  get_num++;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_exam_client");

  actionlib::SimpleActionClient<action_exam::PrimenumberAction> ac("action_exam_action", true);

  ROS_INFO("Waiting for action server to start");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  action_exam::PrimenumberGoal goal;
  if (argc != 2)
    goal.target = 20;
  else
    goal.target = (int)atoi(argv[1]);

  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  return 0;
}