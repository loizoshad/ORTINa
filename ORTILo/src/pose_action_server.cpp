#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "ORTILo/PoseAction.h"

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <ORTILo/StateVector.h>

#include <iostream>
//#include "math.h"
#include "cmath"

class PoseActionServer
{

protected:
  	ros::NodeHandle n;
	actionlib::SimpleActionServer<ORTILo::PoseAction> action_server;

	std::string action_name;
	ORTILo::PoseFeedback feedback;
	ORTILo::PoseResult result;

	ros::Subscriber actual_pose_sub;
	ros::Subscriber estimated_state_sub;
	ros::Subscriber path_plan_sub;
	ros::Publisher command_pose_pub;

	ORTILo::StateVector estimated_state;
	geometry_msgs::Pose current_pose;
	geometry_msgs::Pose goal_pose;
	std_msgs::Float64 local_error;
	//std_msgs::Float64 local_err_threshold;

public:

	PoseActionServer(std::string name): action_server(n, name, boost::bind(&PoseActionServer::actionCallback, this, _1), false), action_name(name)
	{
		initSubs();
		initPubs();
		action_server.start();
	}

	~PoseActionServer()
	{}

private:

	void initSubs()
	{
		ROS_INFO("Initializing Subscribers ...");
		actual_pose_sub = n.subscribe("/uav1/pose", 1, &PoseActionServer::actualPoseCb, this);
		estimated_state_sub = n.subscribe("/kalman_filter_estimated_state", 1, &PoseActionServer::estimatedStateCallback, this);
		ROS_INFO("All subscribers have been successfully initialized");
	}

	void initPubs()
	{	ROS_INFO("Initializing Publishers ...");
		command_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/lqi/command/single_pose", 1);
		ROS_INFO("All publishers have been successfully Initialized");
	}

	std_msgs::Float64 calcGlobalPoseError(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose)
	{
		std_msgs::Float64 tot_err;
		tot_err.data = 0.0;
		std_msgs::Float64 x_err;
		x_err.data = 0.0;
		std_msgs::Float64 y_err;
		y_err.data = 0.0;

		// Calculate the local error
		x_err.data = goal_pose.position.x - current_pose.position.x;
		y_err.data = goal_pose.position.y - current_pose.position.y;
		tot_err.data = std::sqrt(std::pow(x_err.data, 2)+std::pow(y_err.data,2));

		return tot_err;
	}

	std_msgs::Float64 calcLocalPoseError(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose)
	{
		std_msgs::Float64 tot_err;
		tot_err.data = 0.0;
		std_msgs::Float64 x_err;
		x_err.data = 0.0;
		std_msgs::Float64 y_err;
		y_err.data = 0.0;

		// Calculate the local error
		x_err.data = goal_pose.position.x - current_pose.position.x;
		y_err.data = goal_pose.position.y - current_pose.position.y;
		tot_err.data = std::sqrt(std::pow(x_err.data, 2)+std::pow(y_err.data,2));
		ROS_INFO("current_local_error = %f", tot_err.data);

		return tot_err;
	}

	void actualPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{	// You can use this for debugging purposes
		//current_pose = msg->pose;
	}


	void estimatedStateCallback(const ORTILo::StateVectorConstPtr &msg)
	{
		estimated_state.x = msg->x;
		estimated_state.y = msg->y;
		estimated_state.z = msg->z;
		estimated_state.phi = msg->phi;
		estimated_state.theta = msg->theta;
		estimated_state.psi = msg->psi;	

		current_pose.position.x = msg->x;
		current_pose.position.y = msg->y;
		current_pose.position.z = msg->z;
	}


	void actionCallback(const ORTILo::PoseGoalConstPtr &msg)
	{
	
		ros::Rate rate(100);

		bool success = true;

		int counter = 0;

		ROS_INFO("Processing ...");
		
		while(counter < (msg->goal.poses.size() - 1))
		{
			// Calculate current error
			feedback.error = calcGlobalPoseError(current_pose, msg->goal.poses[(msg->goal.poses.size() - 1)].pose);

			// Publish current error (give feedback)
			action_server.publishFeedback(feedback);

      		// Preemption
			if(action_server.isPreemptRequested() || !ros::ok())
      		{
				ROS_INFO("Action Server '%s': Preempted", action_name.c_str());
        		action_server.setPreempted();
        		success = false;
        		break;
			}

			/* 	Main Control Loop
					Check Local error
					if local_error > threshold then command same pose
					if local error <= threshold then command next pose
			*/

			goal_pose = msg->goal.poses[counter].pose;
			local_error = calcLocalPoseError(current_pose, goal_pose);

			if( ( local_error.data <= 0.3 ) || ( current_pose.position.x - goal_pose.position.x > 0.1 ) )
			{
				counter++;
				goal_pose = msg->goal.poses[counter].pose;
			}

			//publish to topic "/lqi/command/single_pose"
			geometry_msgs::PoseStamped goal_pose_stamped;
			goal_pose_stamped.pose = goal_pose;
			goal_pose_stamped.pose.position.z = 1.5;
			command_pose_pub.publish(goal_pose_stamped);

			// Set a timeout error
			ROS_INFO("counter = %d", counter);
		}

		// Check if the goal has been successfully achieved 
    	if(success)
    	{
      		result.status = "FINISHED";
      		ROS_INFO("Action Server '%s': Succeeded", action_name.c_str());
      		action_server.setSucceeded(result);
    	}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_action_server");

	// Create an object of the class
	PoseActionServer pose_obj("pose_action");

	ros::Rate rate(100);

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
