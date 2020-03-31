#include "nav_msgs/Path.h"
#include "sensor_msgs/JointState.h"
#include "trajectory/CartesianState.h"
#include "trajectory/JointPath.h"

trajectory::CartesianState cartesianInterpolation(const double &s, const double &sd, const double &sdd, 
					          const nav_msgs::Path &input, const double &theta)
{
	trajectory::CartesianState state;						// Value to be returned

	// p(t) = (1-s(t))*p1 + s(t)*p2
	state.pose.position.x = (1-s)*input.poses[0].pose.position.x + s*input.poses[1].pose.position.x;
	state.pose.position.y = (1-s)*input.poses[0].pose.position.y + s*input.poses[1].pose.position.y;
	state.pose.position.z = (1-s)*input.poses[0].pose.position.z + s*input.poses[1].pose.position.z;

	// v(t) = (p2 - p1)*sd(t)
	state.vel.linear.x = (input.poses[1].pose.position.x - input.poses[0].pose.position.x)*sd;
	state.vel.linear.y = (input.poses[1].pose.position.y - input.poses[0].pose.position.y)*sd;
	state.vel.linear.z = (input.poses[1].pose.position.z - input.poses[0].pose.position.z)*sd;

	// a(t) = (p2 - p1)*sdd(t)
	state.vel.linear.x = (input.poses[1].pose.position.x - input.poses[0].pose.position.x)*sdd;
	state.vel.linear.y = (input.poses[1].pose.position.y - input.poses[0].pose.position.y)*sdd;
	state.vel.linear.z = (input.poses[1].pose.position.z - input.poses[0].pose.position.z)*sdd;


	// Orientation interpolation
	if(abs(theta) < 0.01)									// Angle between quaternions is less than 1 degree
	{
		state.pose.orientation = input.poses[0].pose.orientation;			// No need to interpolate

		state.vel.angular.x = 0;
		state.vel.angular.y = 0;
		state.vel.angular.z = 0;

		state.accel.angular.x = 0;
		state.accel.angular.y = 0;
		state.accel.angular.z = 0;
	}
	else
	{
		double a 	= (1-s)*theta;							// This assignment makes calcs easier
		double b 	= s*theta;							// This assignment makes calcs easier
		double denom 	= sin(theta);							// Denominator for slerp

		geometry_msgs::Quaternion q, qd, qdd, dqds;
		
		// Spherical linear interpolation of quaternion
		q.w = (sin(a)*input.poses[0].pose.orientation.w + sin(b)*input.poses[1].pose.orientation.w)/denom;
		q.x = (sin(a)*input.poses[0].pose.orientation.x + sin(b)*input.poses[1].pose.orientation.x)/denom;
		q.y = (sin(a)*input.poses[0].pose.orientation.y + sin(b)*input.poses[1].pose.orientation.y)/denom;
		q.z = (sin(a)*input.poses[0].pose.orientation.z + sin(b)*input.poses[1].pose.orientation.z)/denom;

		// Partial-derivative of SLERP
		dqds.w = (-cos(a)*input.poses[0].pose.orientation.w + cos(b)*input.poses[1].pose.orientation.w)*(theta/denom);
		dqds.x = (-cos(a)*input.poses[0].pose.orientation.x + cos(b)*input.poses[1].pose.orientation.x)*(theta/denom);
		dqds.y = (-cos(a)*input.poses[0].pose.orientation.y + cos(b)*input.poses[1].pose.orientation.y)*(theta/denom);
		dqds.z = (-cos(a)*input.poses[0].pose.orientation.z + cos(b)*input.poses[1].pose.orientation.z)*(theta/denom);

		// Quaternion velocity
		qd.w = dqds.w*sd;
		qd.x = dqds.x*sd;
		qd.y = dqds.y*sd;
		qd.z = dqds.z*sd;

		// Quaternion acceleration
		qdd.w = -q.w*theta*theta*sd + dqds.w*sdd;
		qdd.x = -q.x*theta*theta*sd + dqds.x*sdd;
		qdd.y = -q.y*theta*theta*sd + dqds.y*sdd;
		qdd.z = -q.z*theta*theta*sd + dqds.z*sdd;

		state.pose.orientation = q;

		// Convert quaternion velocity to angular velocity
		state.vel.angular.x = 2*(-q.x*qd.w + q.w*qd.x - q.z*qd.y + q.y*qd.z);
		state.vel.angular.y = 2*(-q.y*qd.w + q.z*qd.x + q.w*qd.y - q.x*qd.z);
		state.vel.angular.z = 2*(-q.z*qd.w - q.y*qd.x + q.x*qd.y + q.w*qd.z);

		// Convert quaternion acceleration to angular acceleration
		state.accel.angular.x = 2*(-q.x*qdd.w + q.w*qdd.x - q.z*qdd.y + q.y*qdd.z);
		state.accel.angular.y = 2*(-q.y*qdd.w + q.z*qdd.x + q.w*qdd.y - q.x*qdd.z);
		state.accel.angular.z = 2*(-q.z*qdd.w - q.y*qdd.x + q.x*qdd.y + q.w*qdd.z);
	}
	return state;
}

sensor_msgs::JointState jointInterpolation(const double &s, const double &sd, const double &sdd, const trajectory::JointPath &input)
{
		sensor_msgs::JointState state;					// To be returned

		state.position.resize(sizeof(input.point[0].position));
		state.velocity.resize(sizeof(input.point[0].position));
		state.effort.resize(sizeof(input.point[0].position));

		for(int i = 0; i < sizeof(input.point); i++)
		{
			state.position[i] = (1-s)*input.point[0].position[i] + s*input.point[1].position[i];
			state.velocity[i] = (input.point[1].position[i] - input.point[0].position[i])*sd;
			state.effort[i]   = (input.point[1].position[i] - input.point[0].position[i])*sdd; // This is acceleration, NOT torque
		}

		return state;
}
