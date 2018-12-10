#include <selfie_control/ChangeLaneAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <string>

typedef actionlib::SimpleActionClient<selfie_control::ChangeLaneAction> Client;

int main(int argc, char** argv)
{
  //create node
  ros::init(argc, argv, "demo_change_lane");
  ros::NodeHandle nh("~");
  std::string param;
  std::string lane;

  //get parameters
  nh.getParam("lane",lane);
  int lane_choice = 1;
  if (lane=="l"){
    ROS_INFO("Go left");
    lane_choice = 1;
  }
  else if (lane=="r"){
    ROS_INFO("Go right");
    lane_choice = 0;
  }
  else{
    ROS_INFO("Default = Go left");
    lane_choice = 1;
  }

  //create client for action server
  Client client("change_lane", true); // true -> don't need ros::spin()
  client.waitForServer();
  ROS_INFO("Action server started");

  selfie_control::ChangeLaneGoal goal;

  //set goal
  if (lane_choice == 0){
    goal.left_lane = false;
  }
  else{
    goal.left_lane = true;
  }
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! We have changed lane");
  
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}