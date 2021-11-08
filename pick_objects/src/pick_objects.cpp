#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>

#include <sstream>

#include "pick_objects/MarkerPose.h"
#include "pick_objects/utils.h"


// Typedef for a SimpleActionClient for sending goal requests to the move_base server
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

class PickObjects {
  public:
    PickObjects(std::string the_frame_id = "map")
      : move_base_action_client_("move_base", true),
        frame_id_{the_frame_id},
        robot_state_{0},
        last_marker_state_{0},
        marker_{}
    {
        // Publisher of a std_msgs::Int8 message indicating the robot's current moving state
        robot_state_pub_ = n_.advertise<std_msgs::Int8>("/pick_objects/robot_state", 10);

        // Subscriber to the /add_markers/marker_state topic receiving updates on the marker's state and acting accordingly via the HandleMarkerState_ callback function
        marker_state_sub_ = n_.subscribe(
            "/add_markers/marker_state",
            10,
            &PickObjects::HandleMarkerState_,
            this
        );

        // Subscriber to the /visualization_marker topic receiving the published marker storing it via the HandleMarker_ callback function
        marker_sub_ = n_.subscribe(
            "/visualization_marker",
            1,
            &PickObjects::HandleMarker_,
            this
        );

        // Server for the /pick_objects/move_robot service using the HandleMoveRequest callback function
        move_robot_server_ = n_.advertiseService(
            "/pick_objects/move_robot",
            &PickObjects::HandleMoveRequest_,
            this
        );

        ROS_INFO("Ready to pick objects");
    }

  private:
    ros::NodeHandle n_;
    ros::Publisher robot_state_pub_;
    ros::Subscriber marker_state_sub_;
    ros::Subscriber marker_sub_;
    ros::ServiceServer move_robot_server_;
    MoveBaseActionClient move_base_action_client_;

    // The fixed frame (can be specified via constructor argument, defaults to "map")
    std::string const frame_id_;

    // The robot's current moving state
    int robot_state_;

    // The last marker state ("finished", "pickup", or "dropoff"),
    // published by the /add_markers node
    int last_marker_state_;

    // The marker published by the /add_markers node
    visualization_msgs::Marker marker_;

    // Callback to act according to the published marker's node by the add_markers node
    void HandleMarkerState_(std_msgs::Int8 const & msg) {
        last_marker_state_ = msg.data;

        // If the robot is not moving and this is a pickup marker, then move the robot
        if (robot_state_ != pick_objects::RobotState::MOVING
                && last_marker_state_ == add_markers::MarkerState::PICKUP) {
            SendGoal_(marker_.pose);
        }
    }

    // Callback to store the marker published by the add_markers node
    void HandleMarker_(visualization_msgs::Marker const & msg) {
        marker_ = msg;
        marker_.pose.position.z = 0;
    }

    // This method sends a goal request using the MoveBaseActionClient and checks if the action is successfully completed
    bool HandleMoveRequest_(
        pick_objects::MarkerPose::Request & req,
        pick_objects::MarkerPose::Response & res
    ) {
        // Set the response message
        std::ostringstream goal_oss;
        goal_oss << "(" << req.x << ", " << req.y << ")";
        res.msg_feedback = "Sending goal " + goal_oss.str() + " to move_base";
        ROS_INFO_STREAM(res.msg_feedback);

        // Create a tf quaternion object to convert the yaw rotation input
        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, req.rot);
        q_rot.normalize();

        geometry_msgs::Pose target_pose;
        target_pose.position.x = req.x;
        target_pose.position.y = req.y;
        target_pose.position.z = 0;
        tf2::convert(q_rot, target_pose.orientation);

        SendGoal_(target_pose);

        return true;
    }

    // Method to send a goal for a given pose
    void SendGoal_(geometry_msgs::Pose target_pose) {
        // Keep waiting in 5 sec intervals for move_base action server to come up
        while(!move_base_action_client_.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        // Create a goal object
        move_base_msgs::MoveBaseGoal goal;

        // Set up the frame parameters
        goal.target_pose.header.frame_id = frame_id_;
        goal.target_pose.header.stamp = ros::Time::now();

        // Define the position and orientation for the robot to reach
        goal.target_pose.pose = target_pose;

        // Send the goal position and orientation for the robot to reach
        move_base_action_client_.sendGoal(
            goal,
            boost::bind(&PickObjects::DoneCallback_, this, _1, _2),
            boost::bind(&PickObjects::ActiveCallback_, this),
            MoveBaseActionClient::SimpleFeedbackCallback()
        );
    }

    // Callback to update current robot state, once the action is started
    void ActiveCallback_() {
        // Set current robot's state as "moving"
        SetRobotState_(pick_objects::RobotState::MOVING);
    }

    // Callback to update current robot state, once the action is finished
    void DoneCallback_(
        actionlib::SimpleClientGoalState const & state,
        move_base_msgs::MoveBaseResult::ConstPtr const & result
    ) {
        // Check if the robot rached its goal
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO_STREAM("The base moved to goal successfully");

            // Set current robot's state as "finished"
            SetRobotState_(pick_objects::RobotState::FINISHED);
        } else {
            ROS_INFO_STREAM("The base failed to move to goal");

            // Set current state as "failed"
            SetRobotState_(pick_objects::RobotState::FAILED);
        }
    }

    // Method to set and publish the robot's state
    void SetRobotState_(int state) {
        robot_state_ = state;
        std_msgs::Int8 robot_state;
        robot_state.data = state;
        robot_state_pub_.publish(robot_state);
    }
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
