#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int8.h>

#include <sstream>

#include "pick_objects/MoveToPose.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PickObjects {
  public:
    PickObjects(std::string the_frame_id = "map")
      : move_base_client_("move_base", true),
        frame_id_{the_frame_id}
    {
        // Publisher of a std_msgs::Int8 message specifying the robot's moving state
        // Finishied: 0
        // Moving:    1
        // Failed:    2
        move_state_pub_ = n_.advertise<std_msgs::Int8>("/move_state", 10);

        // Server for the /pick_objects/move_robot service using the HandleMoveRequest callback function
        move_robot_server_ = n_.advertiseService("/pick_objects/move_robot", &PickObjects::HandleMoveRequest, this);
    }

    // This method sends a goal request using the MoveBaseClient and checks if the action is successfully completed
    void HandleMoveRequest(
        pick_objects::MoveToPose::Request & req,
        pick_objects::MoveToPose::Response & res
    ) {
        // Create a goal object
        move_base_msgs::MoveBaseGoal goal;

        // set up the frame parameters
        goal.target_pose.header.frame_id = frame_id_;
        goal.target_pose.header.stamp = ros::Time::now();

        // Create a tf quaternion object to convert the yaw rotation input
        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, req.rot);

        // Define the position and orientation for the robot to reach
        goal.target_pose.pose.position.x = res.x;
        goal.target_pose.pose.position.y = res.y;
        tf2::convert(q_rot, goal.target_pose.pose.orientation);

        // Keep waiting in 5 sec intervals for move_base action server to come up
        while(!move_base_client_.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        // Send the goal position and orientation for the robot to reach
        std::ostringstream goal_oss;
        goal_oss << "(" << res.x << ", " << res.y << ", " << res.rot <<  ")";
        ROS_INFO_STREAM("Sending goal: " + goal_oss.str());
        move_base_client_.sendGoal(goal);

        // Publish the current state as "moving" (1)
        std_msgs::Int8 move_state;
        move_state.data = 1;
        move_state_pub_.publish(move_state);

        // Wait an infinite time for the results
        move_base_client_.waitForResult();

        // Check if the robot rached its goal
        if(move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO_STREAM("The base moved to goal: " + goal_oss.str());

            // Set current state as "finished" (0)
            move_state.data = 0;
        } else {
            ROS_INFO_STREAM("The base failed to move to goal: " + goal_oss.str());

            // Set current state as "failed" (2)
            move_state.data = 1;
        }

        // Publish the current state
        move_state_pub_.publish(move_state);
    }

  private:
    ros::NodeHandle n_;

    // The move_base action client
    MoveBaseClient move_base_client_;

    // The server for moving the robot
    ros::ServiceServer move_robot_server_;

    // A publisher to inform whether the robot is moving
    ros::Publisher move_state_pub_;

    // The fixed frame (can be specified via constructor argument, defaults to "map")
    std::string const frame_id_;
};  // class PickObjects

int main(int argc, char * * argv){
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    // Create an object of the PickObjects class
    PickObjects POObject;

    ROS_INFO("Ready to pick objects");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
