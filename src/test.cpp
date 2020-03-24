#include "CommonFunctions.h"
#include "Polynomial.h"
#include "ros/ros.h"

const double pi = 3.141592;

geometry_msgs::PoseStamped display;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory");		// Initialize node
	ros::NodeHandle n;				// Give this node a handle
	
	ros::Publisher line = n.advertise<nav_msgs::Path>("line",1);
	ros::Publisher waypoints = n.advertise<nav_msgs::Path>("waypoints",1);
	ros::Publisher animate = n.advertise<geometry_msgs::PoseStamped>("animated",1);


	// Create trajectory object
	nav_msgs::Path path;
	path.header.frame_id = "/map";
	path.poses.resize(5);
	path.poses[0].pose.position.x = 0;
	path.poses[0].pose.position.y = 0;
	path.poses[0].pose.position.z = 0;
	path.poses[0].header.stamp = ros::Time(0.0);
	path.poses[0].pose.orientation.w = 1;
	path.poses[0].pose.orientation.x = 0;
	path.poses[0].pose.orientation.y = 0;
	path.poses[0].pose.orientation.z = 0;

	path.poses[1].pose.position.x = 1;
	path.poses[1].pose.position.y = 0;
	path.poses[1].pose.position.z = 1;
	path.poses[1].header.stamp = ros::Time(2.0);
	path.poses[1].pose.orientation.w = cos(0.25*pi);
	path.poses[1].pose.orientation.x = 0;
	path.poses[1].pose.orientation.y = -sin(0.25*pi);
	path.poses[1].pose.orientation.z = 0;

	path.poses[2].pose.position.x = 2;
	path.poses[2].pose.position.y = 1;
	path.poses[2].pose.position.z = 0;
	path.poses[2].header.stamp = ros::Time(4.0);
	path.poses[2].pose.orientation.w = 1;
	path.poses[2].pose.orientation.x = 0;
	path.poses[2].pose.orientation.y = 0;
	path.poses[2].pose.orientation.z = 0;

	path.poses[3].pose.position.x = 1;
	path.poses[3].pose.position.y = 2;
	path.poses[3].pose.position.z = 1;
	path.poses[3].header.stamp = ros::Time(6.0);
	path.poses[3].pose.orientation.w = cos(0.5*pi);
	path.poses[3].pose.orientation.x = -sin(0.5*pi);
	path.poses[3].pose.orientation.y = 0;
	path.poses[3].pose.orientation.z = 0;

	path.poses[4].pose.position.x = 3;
	path.poses[4].pose.position.y = 1;
	path.poses[4].pose.position.z = 1;
	path.poses[4].header.stamp = ros::Time(8.0);
	path.poses[4].pose.orientation.w = cos(1.75*pi);
	path.poses[4].pose.orientation.x = 0;
	path.poses[4].pose.orientation.y = -sin(1.25*pi);
	path.poses[4].pose.orientation.z = 0;


	Polynomial trajectory(path);
	
	nav_msgs::Path visual = trajectory.genCartesianTrajectory(25); // Generate trajectory for visualization purposes

	
	ros::Rate loop_rate(100);					// Establish the loop frequency
	ros::Duration(5.0).sleep();					// Sleep for 5 seconds before beginning

	ros::Time start = ros::Time::now();
	double t = 0.0;
	while(ros::ok())
	{
		t = ros::Time::now().toSec() - start.toSec();			// Current time since beginning of loop
		trajectory::CartesianState state = trajectory.getCartesianState(t); // Get reference state
		display.pose = state.pose;
		display.header.frame_id = "/map";

		line.publish(visual);
		waypoints.publish(path);
		animate.publish(display);

		loop_rate.sleep();
	}

	return 0;					// No problems with main	
}
