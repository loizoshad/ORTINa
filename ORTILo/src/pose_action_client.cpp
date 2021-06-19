#include "ros/ros.h"

#include "ORTILo/PoseAction.h"
#include "ORTILo/PoseActionGoal.h"

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include <iostream>
#include <string.h>
#include "cmath"



template<typename PublishT, typename SubscribeT>

class PublisherSubscriber
{
public:
  PublisherSubscriber(){}
  PublisherSubscriber(std::string publishTopicName, std::string subscribeTopicName, int queueSize)
  {
    publisherObject = nh.advertise<PublishT>(publishTopicName, queueSize);
    subscriberObject = nh.subscribe<SubscribeT>(subscribeTopicName, queueSize, &PublisherSubscriber::subscriberCallback, this);
  }

  void subscriberCallback(const typename SubscribeT::ConstPtr& msg);

protected:
  ros::Subscriber subscriberObject;
  ros::Publisher publisherObject;
  ros::NodeHandle nh;
};


template<>
void PublisherSubscriber<ORTILo::PoseActionGoal, nav_msgs::Path>::subscriberCallback(const nav_msgs::Path::ConstPtr& msg)
{
	ORTILo::PoseActionGoal goal;
	goal.goal.goal.poses = msg->poses;
	publisherObject.publish(goal);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "first_controller_node");

  ros::NodeHandle n;

  PublisherSubscriber<ORTILo::PoseActionGoal, nav_msgs::Path> topic_converter("/pose_action/goal", "/smoothedPath", 1);
  
  ros::spin();

  return 0;
}