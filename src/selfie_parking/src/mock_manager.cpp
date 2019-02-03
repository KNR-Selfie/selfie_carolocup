
#include <selfie_msgs/searchAction.h>
#include <selfie_msgs/parkingAction.h>

#include <selfie_msgs/PolygonArray.h>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <actionlib/client/simple_action_client.h>

#include <selfie_park/parkAction.h>
#include <geometry_msgs/Polygon.h>

enum parking_state{searching=0,  planning_failed=1, planning=2, parking=3};

class mockManager{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<selfie_park::parkAction> park_client_;
    actionlib::SimpleActionClient<selfie_msgs::searchAction> search_client_;
    ros::Publisher parking_state_pub;

    parking_state state;

    public:
    mockManager(const ros::NodeHandle &nh):
      nh_(nh), search_client_("search", true), park_client_("park", true)
    {
      //  parking_state_pub = nh_.advertise<std_msgs::Int16>("/parking_state", 10);
    }

    bool search(geometry_msgs::Polygon *place_found)
    {
        ROS_INFO("waiting for server to start");
        search_client_.waitForServer();
        ROS_INFO("put min lenght for a parking spot [float]: ");
        float x;
        std::cin >> x;

        ros::Duration(1).sleep();
        send_goal(x);
        ROS_INFO("goal sent");

        ros::Duration(3).sleep();

        bool finished = search_client_.waitForResult(ros::Duration(50.0));

        if(finished)
        {
            auto result = search_client_.getResult();
            *place_found = result->parking_spot;
         //   std::cout << ((*place_found).points.begin())->x << std::endl;
            ROS_INFO("place_received!!!");
        }
        else 
        {
            search_client_.cancelGoal();
        }
        return finished;
    }

    void send_goal(float dist)
    {
      selfie_msgs::searchGoal msg;
      msg.min_spot_lenght = dist;
      search_client_.sendGoal(msg);
    }

    bool park(geometry_msgs::Polygon &place_received)
    {
        ROS_INFO("waiting for park_server to start");
        park_client_.waitForServer();

        selfie_park::parkGoal msg;
        msg.parking_spot = place_received;
        msg.park = true;
        park_client_.sendGoal(msg);
        ROS_INFO("goal sent to park_server");

        bool finished = park_client_.waitForResult(ros::Duration(10.0));
        if(finished)
        {
            auto result = park_client_.getResult();
            ROS_INFO("parking_done!!!");
            return true;
        }
        else
        {
            ROS_INFO("parking took too long");
            ROS_FATAL("ABORTING parking");
            return false;
        }
        

    }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mock_manager");
    ros::NodeHandle nh;
    mockManager client(nh);

    geometry_msgs::Polygon place_found;

    if( client.search(&place_found))
    {
        ROS_WARN("place found!!");
        ROS_WARN("initialising parking manouver!!");
        bool park_done = client.park(place_found);
        if(park_done)
        {
            ROS_INFO("parking done correctly");
        }
        else
        {
            ROS_FATAL("PARKING FAILED");
        }
    }
    else
    {
        ROS_WARN("place not found in given time!");
    }

    ROS_INFO("mock_manager is being closed");
    return 0;

}