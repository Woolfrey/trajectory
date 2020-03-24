#include "CommonFunctions.h"								// LERP, SLERP, and visualization
#include "Eigen/Dense"									// For Matrix.inverse() function
#include "nav_msgs/Path.h"								// Gives a set of poses
#include "ros/ros.h"									// Fundamental
#include "sensor_msgs/JointState.h"
#include "string"
#include "trajectory/CartesianState.h"							// Gives pose, velocity, acceleration	
#include "trajectory/JointPath.h"							// Inherets JointPosition.msg
#include "vector"

class Polynomial
{
	public:

		std::string frame_id;
	
		std::string name = "trajectory";					// A name for this object

		Polynomial(const nav_msgs::Path &input);				// Constructor for a Cartesian trajectory
		Polynomial(const trajectory::JointPath &input);				// Constructor for a joint trajectory

		sensor_msgs::JointState getJointState(const double &time);		// Get the joint state at the current time

		trajectory::CartesianState getCartesianState(const double &time); 	// Get the Cartesian state at the current time

		nav_msgs::Path genCartesianTrajectory(int Hz);				// Generate a trajectory at the given frequency

		void visualize(int &argc, char **argv);				

	private:
		double s, sd, sdd;							// Scalars for interpolation

		double angle;								// Angle between 2 quaternions (needed for SLERP)

		Eigen::MatrixXd a,b,c,d;						// Polynomial coefficients

		int n;									// No. of trajectories to create

		nav_msgs::Path cartesianPath;						// Waypoints in Cartesian space
		
		trajectory::JointPath jointPath;					// Waypoints in joint space

		void computeScalar(const double &time);					// Computes scalar for 2-point interpolation
		void genCubicSpline(Eigen::MatrixXd &points, Eigen::VectorXd &times);	// Generates coefficients for the cubic spline
		void genQuinticPolynomial(const double &t0, const double &tf);		// Generate coefficients for quintic polynomial

};
