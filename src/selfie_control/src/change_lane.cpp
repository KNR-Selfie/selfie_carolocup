#include "ChangeLane.h"

bool start_maneuver = false;
float position_offset = 0.1f;

//function used in Action Server for change lane
void execute(Server::GoalHandle goal, ChangeLane *changelane);
//callback from subscriber with information about position goal
void position_offset_callback(const std_msgs::Float64::ConstPtr& msg);
// get msgs values from class and send it to the corresponding publischer
void send_publishers_msgs(ros::Publisher& target_publisher, ros::Publisher& left_turn_indicator_publisher, ros::Publisher& right_turn_indicator_publisher, float target_pos, bool left_indicator, bool right_indicator);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "change_lane");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    //geting paramets from nodehandle
    float lane_width = 1.0f, error_margin = 0.1f;
    pnh.getParam("lane_width",lane_width);
    pnh.getParam("error_margin",error_margin);
    //evaluation
    ROS_INFO("Width %f error %f", lane_width, error_margin);

    //Constructor for changelane class
    ChangeLane changelane(lane_width, error_margin);
    //subscriber creation
    ros::Subscriber position_offset_subscriber = nh.subscribe("position_offset", 10, &position_offset_callback);
    //publishers creation
    ros::Publisher target_publisher = nh.advertise<std_msgs::Float64>("target_offset", 100);
    ros::Publisher left_turn_indicator_publisher = nh.advertise<std_msgs::Bool>("left_turn_indicator", 100);
    ros::Publisher right_turn_indicator_publisher = nh.advertise<std_msgs::Bool>("right_turn_indicator", 100);
    
    //Action server creation
    Server::GoalHandle active_goal;
    Server server(nh, "change_lane", boost::bind(&execute,&server, _1,&changelane), false);
    server.start();
    ROS_INFO("Server ready");
    bool maneuver_status = false;
    while (ros::ok())
    {
        if (start_maneuver == true){
            changelane.position_offset = position_offset;
            //compute output values
            maneuver_status = changelane.process_target_position();

            if(maneuver_status == true){
                start_maneuver = false;

                //confirm success
                //active_goal.setSucceeded();
                //server.setSucceeded();
            }

            //sending msgs to topics
            send_publishers_msgs(target_publisher,left_turn_indicator_publisher, right_turn_indicator_publisher, changelane.target_position, changelane.left_turn_indicator, changelane.right_turn_indicator);
        }
        ros::spinOnce();
    }

    return 0;

}

void execute(Server::GoalHandle goal, ChangeLane* changelane)
{
    ROS_INFO("Chce zmienic pas");
    changelane->be_on_left_lane = goal.getGoal()->left_lane;
    start_maneuver = true;
    static bool goal_status = changelane->process_target_position();
    if (goal_status == false){
        start_maneuver = true;
        //goal.setAccepted();
        //as->setPreempted();
    }
    else if (goal_status == true){
        //goal.setRejected();
        //as->setAborted();
    }
}

//callback from subscriber with information about position goal
void position_offset_callback(const std_msgs::Float64::ConstPtr& msg)
{
    position_offset = msg->data;
}

void send_publishers_msgs(ros::Publisher& target_publisher, ros::Publisher& left_turn_indicator_publisher, ros::Publisher& right_turn_indicator_publisher, float target_pos, bool left_indicator, bool right_indicator){
    //msgs for publishers
    static std_msgs::Float64 target_msg;
    static std_msgs::Bool left_turn_indicator_msg;
    static std_msgs::Bool right_turn_indicator_msg;

    target_msg.data = target_pos;
    target_publisher.publish(target_msg);
    left_turn_indicator_msg.data = left_indicator;
    right_turn_indicator_msg.data = right_indicator;
    left_turn_indicator_publisher.publish(left_turn_indicator_msg);
    right_turn_indicator_publisher.publish(right_turn_indicator_msg);
}