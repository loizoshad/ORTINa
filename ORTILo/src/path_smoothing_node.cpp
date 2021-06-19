#include "path_smoothing_ros/cubic_spline_interpolator.h"
#include <tf/tf.h>
#include "ros/ros.h"
#include <string.h>
#include "std_msgs/String.h"

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
  nav_msgs::Path initial_path;
  int flag = 0;

};


template<>
void PublisherSubscriber<nav_msgs::Path, nav_msgs::Path>::subscriberCallback(const nav_msgs::Path::ConstPtr& msg)
{

  int pointsPerUnit, skipPoints;
  bool useEndConditions, useMiddleConditions;

  nh.param<int>("points_per_unit", pointsPerUnit, 5);
  nh.param<int>("skip_points", skipPoints, 0);
  nh.param<bool>("use_end_conditions", useEndConditions, false);
  nh.param<bool>("use_middle_conditions", useMiddleConditions, false);


  initial_path.poses = msg->poses;
  nav_msgs::Path path, smoothedPath;
  path.header.frame_id = "map";
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";


  for(int i = 0; i < initial_path.poses.size(); i++)
  { 
    pose.pose.position.x = initial_path.poses[i].pose.position.x;
    pose.pose.position.y = initial_path.poses[i].pose.position.y;
    pose.pose.position.z = initial_path.poses[i].pose.position.z;
    pose.pose.orientation.x = initial_path.poses[i].pose.orientation.x;
    pose.pose.orientation.y = initial_path.poses[i].pose.orientation.y;
    pose.pose.orientation.z = initial_path.poses[i].pose.orientation.z;
    pose.pose.orientation.w = initial_path.poses[i].pose.orientation.w;
  }


  // create a cubic spline interpolator
  path_smoothing::CubicSplineInterpolator csi("lala");
  // pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);

  initial_path.header.frame_id = "map";
  csi.interpolatePath(initial_path, smoothedPath);
  //csi.interpolatePath(path, smoothedPath);


  publisherObject.publish(smoothedPath);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_smoothing_ros_demo");
  ros::NodeHandle nh("~");
  ROS_INFO_STREAM("Namespace:" << nh.getNamespace());
  PublisherSubscriber<nav_msgs::Path, nav_msgs::Path> path_smoother("/smoothedPath", "/move_base/NavfnROS/plan", 1);

  ros::spin();

  return 0;
}