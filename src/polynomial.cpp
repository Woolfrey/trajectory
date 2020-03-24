#include "Polynomial.h"
#include "CommonFunctions.h"

/********** CONSTRUCTOR(S) **********/

Polynomial::Polynomial(const nav_msgs::Path &input)
{
 	/*** This constructor creates a polynomial object in Cartesian space.***/

	this->n = input.poses.size() - 1;						// No. of trajectories = no. of points -1
	this->frame_id = input.header.frame_id;						

	if(this->n < 1)									// Not enough points to create a trajectory
	{
		throw "A minimum of 2 points is required to generate a trajectory.";	
		ROS_FATAL("A minimum of 2 points is required to generate a trajectory.");
	}
	else
	{
		this->cartesianPath = input;						// Commit the input to memory

		if(this->n == 1)							// Compute coefficients for a quintic polynomial
		{
			// Vector projection of first quaternion on to second
			double AB =  input.poses[0].pose.orientation.w*input.poses[1].pose.orientation.w + 
				     input.poses[0].pose.orientation.x*input.poses[1].pose.orientation.x +
				     input.poses[0].pose.orientation.y*input.poses[1].pose.orientation.y +
				     input.poses[0].pose.orientation.z*input.poses[1].pose.orientation.z;

			this->angle = acos(AB);						// Since quaternion is unit norm, no need to divide by magnitude
												
			genQuinticPolynomial(input.poses[0].header.stamp.toSec(), input.poses[1].header.stamp.toSec());
		}
		else									// Compute coefficients for n cubic splines
		{
			Eigen::MatrixXd p(7,this->n+1);					// Matrix of positions to be passed to cubic spline function
			Eigen::VectorXd t(this->n+1,1);					// Vector of times to be passed to cubic spline function
			for(int i = 0; i < this->n+1; i++)
			{
				p(0,i) = input.poses[i].pose.position.x;
				p(1,i) = input.poses[i].pose.position.y;
				p(2,i) = input.poses[i].pose.position.z;
				double theta = 2*acos(input.poses[i].pose.orientation.w);

				if(theta > 3.141592)					// Correct the angle to lie between -pi and pi
				{
					theta -= 2*3.141592;
				}

				p(3,i) = theta;
				if(abs(theta) < 0.01) 					// Less than 1 degree in size
				{
					p(4,i) = 0;					
					p(5,i) = 0;
					p(6,i) = 0;
				}
				else
				{
					p(4,i) = input.poses[i].pose.orientation.x/sin(0.5*theta);
					p(5,i) = input.poses[i].pose.orientation.y/sin(0.5*theta);
					p(6,i) = input.poses[i].pose.orientation.z/sin(0.5*theta);
				}

				t(i) = input.poses[i].header.stamp.toSec();		// Transfer over time information		
			}

			genCubicSpline(p, t);						// Now compute coefficients for cubic splines
		}
	}
}

Polynomial::Polynomial(const trajectory::JointPath &input)
{
	/*** This constructor creates a polynomial trajectory object in joint space ***/

	this->n = input.point.size() - 1;						// No. of trajecories = no. of points -1 
	
	if(this->n < 1)
	{
		ROS_FATAL("A minimum of 2 points is required to generate a trajectory.");
		throw "A minimum of 2 points is required to generate a trajectory.";	
	}
	else
	{
		this->jointPath = input;						// Commit to memory

		if(this->n == 1)
		{
			genQuinticPolynomial(input.point[0].header.stamp.toSec(), input.point[1].header.stamp.toSec());
		}
		else
		{
			int m = input.point[0].position.size();
			Eigen::MatrixXd p(m, this->n+1);
			Eigen::VectorXd t(this->n+1,1);
			for(int i = 0; i < this->n+1; i++)
			{
				for(int j = 0; j < this->n+1; j++) p(j,i) = input.point[i].position[j];
				t(i) = input.point[i].header.stamp.toSec();
			}
			genCubicSpline(p, t);
		}
	}
}

/********** FUNCTIONS **********/

void Polynomial::genQuinticPolynomial(const double &t0, const double &tf)
{
	/*** This function generates the coefficients for a quintic polynomial. ***/

	// Every dimension follows the same profile
	this->a.resize(1,1);
	this->b.resize(1,1);
	this->c.resize(1,1);

	double dt = this->cartesianPath.poses[this->n].header.stamp.toSec()
		  - this->cartesianPath.poses[0].header.stamp.toSec();

	if(dt <= 0)
	{
		throw "Failed to create a polynomial trajectory object. The start time is greater than or equal to the finishing time.";
	}

	this->a(0,0) = 6*pow(dt,-5);
	this->b(0,0) = -15*pow(dt,-4);
	this->c(0,0) = 10*pow(dt,-3);	
}

void Polynomial::genCubicSpline(Eigen::MatrixXd &points, Eigen::VectorXd &times)
{
	// 1. First compute the cubic spline properties

	Eigen::MatrixXd A(this->n+1, this->n+1);
	A.setZero();
	Eigen::MatrixXd B(this->n+1, this->n+1);
	B.setZero();

	double dt = times(1) - times(0);
	if(dt <= 0)
	{
		throw "Failed to create a polynomial trajectory object. Time t(1) is less than or equal to time t(0).";
	}

	A(0,0) = dt*dt/3;
	A(0,1) = dt*dt/6;
	B(0,0) = -1;
	B(0,1) = 1;

	dt = times(this->n) - times(this->n-1);
	if(dt <= 0)
	{
		throw "Failed to create a polynomial trajectory object. The final time t(n) is greater than or equal to the previous time t(n-1).";
	}
	
	A(this->n, this->n-1) 	= -1*dt*dt/6;
	A(this->n, this->n) 	= -1*dt*dt/3;
	B(this->n, this->n-1)	= -1;
	B(this->n, this->n)	= 1;

	for(int i=1; i < this->n; i++)
	{
		double dt1 = times(i) - times(i-1);
		double dt2 = times(i+1) - times(i);
		if(dt1 <= 0 || dt2 <= 0)
		{
			throw "Failed to create a polynomial trajectory object. The time steps are not in ascending order.";
		}

		A(i,i-1)   = dt1/6.0;
		A(i,i)     = (dt1 + dt2)/3.0;
		A(i,i+1)   = (dt2/6.0);
		B(i,i-1)   = 1.0/dt1;
		B(i,i)     = -1.0/dt1 - 1.0/dt2;
		B(i,i+1)   = 1.0/dt2;
	}

	Eigen::MatrixXd C = A.inverse()*B;						// Maps positions to accelerations

	// 2. Now solve explicitly for spline coefficients
	int m = points.rows();
	this->a.resize(m,this->n);
	this->b.resize(m,this->n);
	this->c.resize(m,this->n);
	this->d.resize(m,this->n);

	Eigen::VectorXd sdd(this->n+1);							// Vector of accelerations across time for 1 dimension

	for(int i=0; i < m; i++)
	{
		sdd = C*points.block(i,0,1,this->n+1).transpose();			// Accelerations for each waypoint

		for(int j = 0; j < this->n; j++)
		{
			dt = times(j+1) - times(j);
			this->a(i,j) = points(i,j);
			this->c(i,j) = 0.5*sdd(j);
			this->d(i,j) = (sdd(j+1)-sdd(j))/(6*dt);

			if(j == 0)
			{
				this->b(i,j) = 0;
			}			
			else if(j == this->n)
			{
				this->b(i,j) = -0.5*dt*(sdd(j+1) + sdd(j));
			}
			else
			{
				this->b(i,j) = (points(i,j+1) - points(i,j))/dt
						- dt*(sdd(j+1) + 2*sdd(j))/6;
			}
		}
	}
}

void Polynomial::computeScalar(const double &time)
{
	if(time < cartesianPath.poses[0].header.stamp.toSec())				// Trajectory not yet started
	{
		this->s   = 0;
		this->sd  = 0;
		this->sdd = 0;
	}
	else if(time > cartesianPath.poses[1].header.stamp.toSec())			// Trajectory finished
	{
		this->s   = 1;
		this->sd  = 0;
		this->sdd = 0;
	}
	else
	{
		this->s   =    this->a(0,0)*pow(time,5) +    this->b(0,0)*pow(time,4) +   this->c(0,0)*pow(time,3);
		this->sd  =  5*this->a(0,0)*pow(time,4) +  4*this->b(0,0)*pow(time,3) + 3*this->c(0,0)*pow(time,2);
		this->sdd = 20*this->a(0,0)*pow(time,3) + 12*this->b(0,0)*pow(time,2) + 6*this->c(0,0)*time;
	}
}

trajectory::CartesianState Polynomial::getCartesianState(const double &time)
{
	if(this->n == 1)								// 1 trajectory; must be a quintic polynomial
	{
		computeScalar(time);							// Compute scalar interpolation for elapsed time
		return cartesianInterpolation(this->s, this->sd, this->sdd, this->cartesianPath, this->angle); // Compute cartesian state for given time
	}
	else										// More than 1 trajectory; must be a cubic spline
	{

		trajectory::CartesianState state;					// To be returned

		if(time < this->cartesianPath.poses[0].header.stamp.toSec())		// Trajectory not yet started
		{

			state.pose = cartesianPath.poses[0].pose;			// Maintain start poses

			// Zero velocity
			state.vel.linear.x    = 0;
			state.vel.linear.y    = 0;
			state.vel.linear.z    = 0;
			state.vel.angular.x   = 0;
			state.vel.angular.y   = 0;
			state.vel.angular.z   = 0;

			// Zero acceleration
			state.accel.linear.x  = 0;
			state.accel.linear.y  = 0;
			state.accel.linear.z  = 0;
			state.accel.angular.x = 0;
			state.accel.angular.y = 0;
			state.accel.angular.z = 0;
		}
		else if(time > this->cartesianPath.poses[this->n].header.stamp.toSec())	// Trajectory finished
		{

			state.pose = cartesianPath.poses[this->n].pose;			// Maintain final pose

			// Zero velocity
			state.vel.linear.x    = 0;
			state.vel.linear.y    = 0;
			state.vel.linear.z    = 0;
			state.vel.angular.x   = 0;
			state.vel.angular.y   = 0;
			state.vel.angular.z   = 0;

			// Zero acceleration
			state.accel.linear.x  = 0;
			state.accel.linear.y  = 0;
			state.accel.linear.z  = 0;
			state.accel.angular.x = 0;
			state.accel.angular.y = 0;
			state.accel.angular.z = 0;
		}
		else
		{

			int j = 0;
			for(int i = this->n; i >= 0; i--)
			{
				if(time > this->cartesianPath.poses[i].header.stamp.toSec())
				{
					j = i;
					break;
				}
			}
			
			double dt = time - this->cartesianPath.poses[j].header.stamp.toSec();		// Time since the start of jth spline


			// Assign the pose
			state.pose.position.x = a(0,j) + b(0,j)*dt + c(0,j)*pow(dt,2) + d(0,j)*pow(dt,3);
			state.pose.position.y = a(1,j) + b(1,j)*dt + c(1,j)*pow(dt,2) + d(1,j)*pow(dt,3);
			state.pose.position.z = a(2,j) + b(2,j)*dt + c(2,j)*pow(dt,2) + d(2,j)*pow(dt,3);
			double theta 	      = a(3,j) + b(3,j)*dt + c(3,j)*pow(dt,2) + d(3,j)*pow(dt,3);
			double axis [3]       ={a(4,j) + b(4,j)*dt + c(4,j)*pow(dt,2) + d(4,j)*pow(dt,3),
						a(5,j) + b(5,j)*dt + c(5,j)*pow(dt,2) + d(5,j)*pow(dt,3),
						a(6,j) + b(6,j)*dt + c(6,j)*pow(dt,2) + d(6,j)*pow(dt,3)};

			// Normalize the axis before encoding in quaternion
			double norm = sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
			axis[0] *= norm;
			axis[1] *= norm;
			axis[2] *= norm;
		
			state.pose.orientation.w = cos(0.5*theta);
			state.pose.orientation.x = sin(0.5*theta)*axis[0];
			state.pose.orientation.y = sin(0.5*theta)*axis[1];
			state.pose.orientation.z = sin(0.5*theta)*axis[2];

			// Assign velocities; 3rd row is the norm of the angular velocity vector
			state.vel.linear.x  = b(0,j) + 2*c(0,j)*dt + 3*d(0,j)*dt*dt;
			state.vel.linear.y  = b(1,j) + 2*c(1,j)*dt + 3*d(1,j)*dt*dt;		
			state.vel.linear.z  = b(2,j) + 2*c(2,j)*dt + 3*d(2,j)*dt*dt;
			state.vel.angular.x = b(4,j) + 2*c(4,j)*dt + 3*d(4,j)*dt*dt;
			state.vel.angular.y = b(5,j) + 2*c(5,j)*dt + 3*d(5,j)*dt*dt;		
			state.vel.angular.z = b(6,j) + 2*c(6,j)*dt + 3*d(6,j)*dt*dt;

			// Assign accelerations; 3rd row is the norm of the angular acceleration vector
			state.accel.linear.x  = 2*c(0,j) + 6*d(0,j)*dt;
			state.accel.linear.y  = 2*c(1,j) + 6*d(1,j)*dt;	
			state.accel.linear.z  = 2*c(2,j) + 6*d(2,j)*dt;
			state.accel.angular.x = 2*c(4,j) + 6*d(4,j)*dt;
			state.accel.angular.y = 2*c(5,j) + 6*d(5,j)*dt;		
			state.accel.angular.z = 2*c(6,j) + 6*d(6,j)*dt;
		}
		return state;
	}
}

sensor_msgs::JointState Polynomial::getJointState(const double &time)			// Get the joint state at the current time
{
	if(this->n == 1)								// 1 trajectory; must be a quintic polynomial
	{
		computeScalar(time);	
		return jointInterpolation(s,sd,sdd,jointPath);						 	
	}
	else										// Must be a cubic spline
	{
		sensor_msgs::JointState state;						// To be returned

		int m = this->jointPath.point[0].position.size();
		state.position.resize(m);
		state.velocity.resize(m);
		state.effort.resize(m);

		if(time < this->jointPath.point[0].header.stamp.toSec())		// Trajectory not yet begun
		{
			state.position = this->jointPath.point[0].position;
			
			for(int i = 0; i < m; i++)
			{
				state.velocity[i] = 0;
				state.effort[i] = 0;					// N.B. this is acceleration, not torque
			}
		}
		else if (time > this->jointPath.point[this->n].header.stamp.toSec())	// Trajectory finished
		{
			state.position = this->jointPath.point[this->n].position;
			
			for(int i = 0; i < m; i++)
			{
				state.velocity[i] = 0;
				state.effort[i] = 0;					// N.B. this is acceleration, not torque
			}
		}
		else
		{
			int j;
			for(int i = this->n; i >= 0; i--)				// Count backwards
			{
				if(time > this->jointPath.point[i].header.stamp.toSec())
				{
					j = i;
					break;						
				}
			}

			double dt = time - this->jointPath.point[j].header.stamp.toSec();

			for(int i = 0; i < this->jointPath.point[j].position.size(); i++)
			{
				state.position[i] =   this->a(i,j) +   	this->b(i,j)*dt +   this->c(i,j)*pow(dt,2) + this->d(i,j)*pow(dt,3);
				state.velocity[i] =   			this->b(i,j) 	+ 2*this->c(i,j)*dt 	   + 3*this->d(i,j)*dt*dt;
				state.effort[i]   =		      			  2*this->c(i,j)           + 6*this->d(i,j)*dt;
			}
		}
	}
}

nav_msgs::Path Polynomial::genCartesianTrajectory(int Hz)
{
	// Generate a high-resolution path

	double t0 = this->cartesianPath.poses[0].header.stamp.toSec();			// Start time
	double tf = this->cartesianPath.poses[this->n].header.stamp.toSec();		// End time
	double dt = 1/(double)Hz;

	int steps = (int)((tf - t0)/dt);

	nav_msgs::Path	trajectory;							// To be returned

	trajectory.header.frame_id = this->frame_id;					// Assign frame id
	trajectory.poses.resize(steps);							// No. of poses to generate
	trajectory::CartesianState state;						// Temporary placeholder

	double t = 0.0;

	for(int i = 0; i < steps; i++)
	{
		t = t0 + dt*(double)i;
		state = getCartesianState(t);						// Interpolate along the trajectory
		trajectory.poses[i].pose = state.pose;					// Assign pose to array for visualization
		trajectory.poses[i].header.frame_id = this->frame_id;
		trajectory.poses[i].header.stamp = ros::Time(t);
	}
		
	return trajectory;
}
