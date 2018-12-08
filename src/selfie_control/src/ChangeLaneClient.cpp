#include <selfie_control/ChangeLaneAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<selfie_control::ChangeLaneAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "change_lane_client");
  Client client("change_lane", true); // true -> don't need ros::spin()
  client.waitForServer();
  ROS_INFO("Action server started");
  selfie_control::ChangeLaneGoal goal;
  goal.left_lane = true;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! We have changed lane");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}