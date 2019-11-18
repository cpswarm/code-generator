#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <uav_mavros_takeoff/TakeOffAction.h>

using namespace std;

typedef actionlib::SimpleActionServer<uav_mavros_takeoff::TakeOffAction> Server;

// Variables
ros::Publisher goal_pos_pub;

ros::Subscriber state_sub;
ros::Subscriber local_pos_sub;

//**************** CALLBACKS *******************************************************************************

void localPosition_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
}

bool execute_cb(const uav_mavros_takeoff::TakeOffGoal::ConstPtr &goal, Server *as) {
	ROS_DEBUG("TAKEOFF - Executing Takeoff action..");
}

void state_cb(const mavros_msgs::State::ConstPtr &msg) {
}

//*******************************************************************************************************************

int main(int argc, char **argv) {
	ros::init(argc, argv, "takeoff_action");
	ros::NodeHandle nh;

	string state_topic = "mavros/state";
	state_sub = nh.subscribe<mavros_msgs::State>(state_topic, 10, state_cb);

	string pose_topic = "pos_provider/pose";
	local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(local_pos_topic, 1, localPosition_cb);

	string goal_position_topic = "pos_controller/goal_position";
	goal_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_pos_topic, 1);

	Server server(nh, "cmd/takeoff", boost::bind(&execute_cb, _1, &server), false);
	server.start();
	ros::spin();

	return 0;
}
