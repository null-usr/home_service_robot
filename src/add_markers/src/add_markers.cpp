#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "nav_msgs/Odometry.h"
#include "pick_objects/FoundObject.h"

class Marker_SubscribeAndPublish
{
  public:
    Marker_SubscribeAndPublish()
    {
      // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
      service = n_.advertiseService("/add_markers/interact_object", &Marker_SubscribeAndPublish::interact_object_request, this);

      ROS_INFO("Ready to simulate picking up and dropping");

      //odom_sub for potential improvements
      //sub_ = n_.subscribe("odom", 100, &Marker_SubscribeAndPublish::odom_callback, this);

      marker_pub = 
        n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }

    bool marker_initialize()
    {
        // Set our initial shape type to be a cube
      shape = visualization_msgs::Marker::CUBE;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "basic_shapes";
      marker.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = -17.0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;


      marker.lifetime = ros::Duration();

      // Publish the marker
      while (marker_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return false;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }

      marker_pub.publish(marker);
      return true;
    }

    //could use this and hardcode locations but nah..
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      ROS_INFO("Seq: [%d]", msg->header.seq);
      ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
      ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
      
      //.... do something with the input and generate the output...
      //PUBLISHED_MESSAGE_TYPE output;
      //pub_.publish(output);
    }

    //ideal world, we'd use the odom position for the new location but 
    //we'll see..
    // callback for when an object pickup/dropoff point is reached
    bool interact_object_request(pick_objects::FoundObject::Request& req,
      pick_objects::FoundObject::Response& res)
    {
      ROS_INFO("FoundObject received - drop:%d", (int)req.drop);

      //drop will be 1 for dropping the object, 0 for picking up

      int val = (int)req.drop;

      switch(val)
      {
        case 0:
          //wait 5 seconds
          ros::Duration(5.0).sleep();

          //hide marker
          marker.action = visualization_msgs::Marker::DELETE;
          marker_pub.publish(marker);

          res.msg_feedback = "Virtual object picked up.";
        break;

        case 1:
          //wait 5 seconds
          ros::Duration(5.0).sleep();
          marker.action = visualization_msgs::Marker::ADD;
          //set new marker pose and publish
          marker.pose.position.x = -17.0;
          marker.pose.position.y = 7;
          marker_pub.publish(marker);
          res.msg_feedback = "Virtual object dropped.";
        break;
      }

      // Return a response message
      ROS_INFO_STREAM(res.msg_feedback);

      return true;
    }

  private:
    ros::NodeHandle n_; 
    ros::Publisher marker_pub;
    ros::ServiceServer service;
    ros::Subscriber sub_;
    uint32_t shape;
    visualization_msgs::Marker marker;
};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");
    Marker_SubscribeAndPublish SAP_marker;
    if(!SAP_marker.marker_initialize())
    {
      ROS_WARN_ONCE("Failed to initialize add_markers");
    }
    



    // Cycle between different shapes
    /*switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }*/

    /*while(ros::ok())
        r.sleep();*/

    ros::spin();
 // }
}
