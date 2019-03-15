#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <vector>

// Define vector to store multiple goals
std::vector<std::vector<float> > goals_vector
{
  {-2.5, -3.5, 0.0, 0.0, 0.0, 0.0, 1.0},
  {6.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0},
};

// Define global publisher and subscriber
ros::Publisher marker_pub;
ros::Subscriber odom_sub;

// Define function to add/remove marker at certain location
void publish_marker(std::vector<float> current_goal, int action)
{
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "pickup_object";
  marker.id = 0;

  // Set the marker type. 
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  if (action == 1){ 
    marker.action = visualization_msgs::Marker::ADD;
  }
  else{
    marker.action = visualization_msgs::Marker::DELETE;
  }

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = current_goal.at(0);
  marker.pose.position.y = current_goal.at(1);
  marker.pose.position.z = current_goal.at(2);
  marker.pose.orientation.x = current_goal.at(3);
  marker.pose.orientation.y = current_goal.at(4);
  marker.pose.orientation.z = current_goal.at(5);
  marker.pose.orientation.w = current_goal.at(6);

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);
}

// Define function to calculate distance between robot pose and goal pose
double calc_dist(const nav_msgs::Odometry::ConstPtr& msg, std::vector<float> current_goal){
  double delta_x = msg->pose.pose.position.x - current_goal.at(0);
  double delta_y = msg->pose.pose.position.y - current_goal.at(1);
  double dist = sqrt(pow(delta_x,2)+pow(delta_y,2));
  return dist;
}

// Define function for odometry callback
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  static bool pickup = false;
  static bool dropoff = false;
  double dist_tolerance = 1.0; 
  double dist = 1000; 

  if (pickup && dropoff){
    publish_marker(goals_vector.at(1),1);
  }
  else if (pickup){
    dist = calc_dist(msg,goals_vector.at(1));
    if (dist <= dist_tolerance){
      dropoff = true;
      ROS_INFO("Object is dropped off");
    }
    publish_marker(goals_vector.at(1),0);
  }
  else{
    dist = calc_dist(msg,goals_vector.at(0));
    if (dist <= dist_tolerance){
      pickup = true;
      ROS_INFO("Object is picked up");
    }
    publish_marker(goals_vector.at(0),1);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  odom_sub = n.subscribe("odom",1000,odom_cb);
  ros::spin();
  return 0;
}
