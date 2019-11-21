#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_mavros_takeoff/TakeOffAction.h>

using namespace std;

typedef actionlib::SimpleActionServer<uav_mavros_takeoff::TakeOffAction> Server;

// Variables
ros::Publisher pos_controller_goal_position_pub;

ros::Subscriber mavros_state_sub;
ros::Subscriber pos_provider_sub;

//**************** CALLBACKS *******************************************************************************

void mavros_state_cb(const mavros_msgs::State::ConstPtr &msg) {
}

void pos_provider_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
}

bool execute_cb(const uav_mavros_takeoff::TakeOffGoal::ConstPtr &goal, Server *as) {
	ROS_DEBUG("TAKEOFF - Executing TakeOff action..");
}

//*******************************************************************************************************************

int main(int argc, char **argv) {
	ros::init(argc, argv, "uav_mavros_takeoff");
	ros::NodeHandle nh;

	mavros_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, mavros_state_cb);
	pos_provider_sub = nh.subscribe<geometry_msgs::PoseStamped>("pos_provider", 10, pos_provider_cb);

	pos_controller_goal_position_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", 10);

	Server server(nh, "cmd/takeoff", boost::bind(&execute_cb, _1, &server), false);
	server.start();
	ros::spin();

	return 0;
}
