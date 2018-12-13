#include <selfie_control/ChangeLaneAction.h> // Note: "Action" is appended
#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>
#include <string>

typedef actionlib::ActionClient<selfie_control::ChangeLaneAction> Client;

void action_status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

int main(int argc, char** argv)
{
  //create node
  ros::init(argc, argv, "demo_change_lane");
  ros::NodeHandle nh("~");
  ros::Subscriber status_subscriber = nh.subscribe("action_status", 10, &action_status_callback);
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
  Client client(nh,"change_lane"); 
  client.waitForActionServerToStart(ros::Duration(5,0));
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
  //client.waitForResult(ros::Duration(5.0));

  /*if (client.getState() == actionlib::ClientGoalState::SUCCEEDED)
    printf("Yay! We have changed lane");
  else if (client.getState() == actionlib::ClientGoalState::REJECTED)
    printf("No, we are on this lane. No change");
  if (client.getState() == actionlib::ClientGoalState::ACTIVE){
    printf("Aktywny");
  
  }*/

  //printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}

//callback from subscriber with information about position goal
void action_status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    //uint8_t status = msg->status;
    //ROS_INFO("Status: %d",status);
}