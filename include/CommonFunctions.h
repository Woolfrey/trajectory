#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "string"					
#include "trajectory/CartesianState.h"
#include "trajectory/JointPath.h"
#include "vector"

trajectory::CartesianState cartesianInterpolation(const double &s, const double &sd, const double &sdd, 
							    const nav_msgs::Path &input, const double &theta);

sensor_msgs::JointState jointInterpolation(const double &s, const double &sd, const double &sdd, 
					   const trajectory::JointPath &input);

//void publishPath(int &argc, char **argv, const nav_msgs::Path &trajectory, const nav_msgs::Path &waypoints, std::string &name);
