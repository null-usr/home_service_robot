#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "pick_objects/FoundObject.h"

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Define a global client that can request services
ros::ServiceClient client;

//this function calls the interact_object service to interact with the object
void interact_object(int drop)
{
    pick_objects::FoundObject srv;
    srv.request.drop = drop;

    if(!client.call(srv))
        ROS_ERROR("Failed to call service interact_object");
}

int main(int argc, char** argv)
{
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from interact_object
    client = n.serviceClient<pick_objects::FoundObject>("/add_markers/interact_object");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //going to try to reuse the same goal, here's the first movement ===========================================

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = -17;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending virtual object location.");
    ac.sendGoal(goal);

    ROS_INFO("Moving to object...");
    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, we've found a virtual object to pick up!");
    else
    {
        //if our initial movement fails, we've never made it to the virtual object to drop off
        ROS_INFO("Could not reach our virtual object...");
        return 0;
    }

    ROS_INFO("Picking up virtual object...");
    interact_object(0);
    //wait 5 seconds to pick up the object =============================================
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Virtual object picked up!");
    }


    //second movement ===========================================
    
    // set up the frame parameters
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = -17.0;
    goal.target_pose.pose.position.y = 7.0;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending drop-off location.");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();
    ROS_INFO("Moving to drop-off location.");

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, we've successfully delivered our object!");
        interact_object(1);
    }
    else
        ROS_INFO("We've failed to deliver our virtual object....");
    return 0;
}