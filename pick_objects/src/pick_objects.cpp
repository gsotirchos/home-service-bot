#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Int8.h>

#include <sstream>

#include "pick_objects/MarkerPose.h"


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PickObjects {
  public:
    PickObjects(std::string the_frame_id = "map")
      : move_base_client_("move_base", true),
        frame_id_{the_frame_id}
    {
        // Publisher of a std_msgs::Int8 message specifying the robot's moving state
        // Finished: 0
        // Failed:   1
        // Moving:   2
        robot_state_pub_ = n_.advertise<std_msgs::Int8>("/robot_state", 10);

        // Server for the /pick_objects/move_robot service using the HandleMoveRequest callback function
        move_robot_server_ = n_.advertiseService(
            "/pick_objects/move_robot",
            &PickObjects::HandleMoveRequest,
            this
        );

        ROS_INFO("Ready to pick objects");
    }

    // This method sends a goal request using the MoveBaseClient and checks if the action is successfully completed
    bool HandleMoveRequest(
        pick_objects::MarkerPose::Request & req,
        pick_objects::MarkerPose::Response & res
    ) {
        // Keep waiting in 5 sec intervals for move_base action server to come up
        while(!move_base_client_.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        // Create a goal object
        move_base_msgs::MoveBaseGoal goal;

        // Set up the frame parameters
        goal.target_pose.header.frame_id = frame_id_;
        goal.target_pose.header.stamp = ros::Time::now();

        // Create a tf quaternion object to convert the yaw rotation input
        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, req.rot);
        q_rot.normalize();

        // Define the position and orientation for the robot to reach
        goal.target_pose.pose.position.x = req.x;
        goal.target_pose.pose.position.y = req.y;
        goal.target_pose.pose.position.z = 0.5;
        tf2::convert(q_rot, goal.target_pose.pose.orientation);

        // Set the response message
        std::ostringstream goal_oss;
        goal_oss << "(" << req.x << ", " << req.y << ")";
        res.msg_feedback = "Sending goal " + goal_oss.str() + " to move_base";
        ROS_INFO_STREAM(res.msg_feedback);

        // Send the goal position and orientation for the robot to reach
        move_base_client_.sendGoal(
            goal,
            boost::bind(&PickObjects::DoneCallback, this, _1, _2),
            MoveBaseClient::SimpleActiveCallback(),
            MoveBaseClient::SimpleFeedbackCallback()
        );

        // Publish the current state as "moving" (2)
        std_msgs::Int8 robot_state;
        robot_state.data = 2;
        robot_state_pub_.publish(robot_state);

        return true;
    }

    // Callback to inform about the current robot state, via the /robot_state topic, once the action is finished 
    void DoneCallback(
        actionlib::SimpleClientGoalState const & state,
        move_base_msgs::MoveBaseResult::ConstPtr const & result
    ) {
        std_msgs::Int8 robot_state;

        // Check if the robot rached its goal
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO_STREAM("The base moved to goal successfully");

            // Set current robot's state as "finished" (0)
            robot_state.data = 0;
        } else {
            ROS_INFO_STREAM("The base failed to move to goal");

            // Set current state as "failed" (1)
            robot_state.data = 1;
        }

        // Publish the current state
        robot_state_pub_.publish(robot_state);
    }

  private:
    ros::NodeHandle n_;

    // The move_base action client
    MoveBaseClient move_base_client_;

    // The server for moving the robot
    ros::ServiceServer move_robot_server_;

    // A publisher to inform whether the robot is moving
    ros::Publisher robot_state_pub_;

    // The fixed frame (can be specified via constructor argument, defaults to "map")
    std::string const frame_id_;

};  // class PickObjects

int main(int argc, char * * argv){
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    // Create an object of the PickObjects class
    PickObjects POObject;

    // Handle ROS communication events
    ros::spin();

    return 0;
}
