#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <sstream>

#include "pick_objects/MarkerPose.h"
#include "pick_objects/utils.h"


class AddMarkers {
  public:
    AddMarkers(std::string the_frame_id = "/map")
      : frame_id_{the_frame_id},
        marker_state_{0},
        last_robot_state_{0}
    {
        // Publisher for visualization_msgs::Marker messages, for sending markers to RViz
        marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        // Publisher of a std_msgs::Int8 message indicating the marker's current state
        marker_state_pub_ = n_.advertise<std_msgs::Int8>("/add_markers/marker_state", 10);

        // Subscriber to the /pick_objects/robot_state topic receiving updates on the robot's moving state and acting accordingly via the HandleRobotState callback function
        robot_state_sub_ = n_.subscribe(
            "/pick_objects/robot_state",
            10,
            &AddMarkers::HandleRobotState_,
            this
        );

        // Subscriber to the /move_base/goal topic for storing the last issued goal
        robot_goal_sub_ = n_.subscribe(
            "/move_base/goal",
            10,
            &AddMarkers::HandleRobotGoal_,
            this
        );


        // Server for showing a new marker given its coordinates
        show_marker_server_ = n_.advertiseService(
            "/add_markers/show_marker",
            &AddMarkers::ShowMarker_,
            this
        );

        // Server for the /pick_objects/hide_all_markers service for hiding all shown markers
        hide_marker_server_ = n_.advertiseService(
            "/add_markers/hide_marker",
            &AddMarkers::HideMarker_,
            this
        );

        ROS_INFO("Ready to add markers");
    }

  private:
    ros::NodeHandle n_;
    ros::Publisher marker_pub_;
    ros::Publisher marker_state_pub_;
    ros::Subscriber robot_state_sub_;
    ros::Subscriber robot_goal_sub_;
    ros::ServiceServer show_marker_server_;
    ros::ServiceServer hide_marker_server_;

    // The fixed frame (can be specified via constructor argument, defaults to "/map")
    std::string const frame_id_;

    // The current marker state
    int marker_state_;

    // The robot's last moving state ("finished", "failed", or "moving"),
    // published by the pick_objects node
    int last_robot_state_;

    // The robot's last issued move_base goal
    // published by the move_base node
    move_base_msgs::MoveBaseGoal last_robot_goal_;

    // Callback to act according to the published robot's moving state by the pick_objects node
    void HandleRobotState_(std_msgs::Int8 const & msg) {
        last_robot_state_ = msg.data;

        // If the robot is s not moving
        if (last_robot_state_ != pick_objects::RobotState::MOVING) {
            // ...and this is a dropoff marker, then show the marker
            if (marker_state_ == add_markers::MarkerState::DROPOFF) {
                visualization_msgs::Marker marker;
                marker.pose = last_robot_goal_.target_pose.pose;
                marker.pose.position.z = 0.2;
                marker.action = visualization_msgs::Marker::ADD;
                PublishMarker_(marker);

                // Set the current marker's state to "finished"
                SetMarkerState_(add_markers::MarkerState::FINISHED);
            // ...and this is a pickup marker, then hide the marker
            } else if (marker_state_ == add_markers::PICKUP) {
                visualization_msgs::Marker marker;
                marker.action = visualization_msgs::Marker::DELETEALL;
                PublishMarker_(marker);

                // Set the current marker's state to "dropoff"
                SetMarkerState_(add_markers::MarkerState::DROPOFF);
            }
        }
    }

    // Callback to store the goal issued to move_base
    void HandleRobotGoal_(move_base_msgs::MoveBaseActionGoal const & msg) {
        last_robot_goal_ = msg.goal;
    }

    // Method to show a marker
    bool ShowMarker_(
        pick_objects::MarkerPose::Request & req,
        pick_objects::MarkerPose::Response & res
    ) {
        std::ostringstream marker_oss;
        marker_oss << "(" << req.x << ", " << req.y << ")";
        res.msg_feedback = "Showing marker " + marker_oss.str();
        ROS_INFO_STREAM(res.msg_feedback);

        // Create a marker object
        visualization_msgs::Marker marker;

        // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Create a tf quaternion object to convert the yaw rotation input
        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, req.rot);
        q_rot.normalize();

        // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = req.x;
        marker.pose.position.y = req.y;
        marker.pose.position.z = 0.2;
        tf2::convert(q_rot, marker.pose.orientation);

        PublishMarker_(marker);

        // Set the current marker's state to "pickup"
        SetMarkerState_(add_markers::MarkerState::PICKUP);

        return true;
    }

    // Method to hide all shown markers
    bool HideMarker_(
        std_srvs::Trigger::Request & req,
        std_srvs::Trigger::Response & res
    ) {
        res.success = true;
        res.message = "Hiding marker";
        ROS_INFO_STREAM(res.message);

        // Create a marker object
        visualization_msgs::Marker marker;

        // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::DELETEALL;

        PublishMarker_(marker);

        return true;
    }

    // Method to publish a marker
    void PublishMarker_(visualization_msgs::Marker & marker) const {
        // Check if anyone's listening, at 1 sec intervals
        while (marker_pub_.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return;
            }

            ROS_WARN_ONCE("Please create a subscriber to the marker");
            ros::Duration(1.0).sleep();
        }

        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker. This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "goal_markers";
        marker.id = 0;

        // Set the shape type
        marker.type = visualization_msgs::Marker::SPHERE;

        // Set the scale of the marker
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Set the color
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8;

        // Set infinite lifetime
        marker.lifetime = ros::Duration();

        // Publish the marker
        marker_pub_.publish(marker);
    }

    // Method to set and publish the marker's state
    void SetMarkerState_(int state) {
        marker_state_ = state;
        std_msgs::Int8 marker_state;
        marker_state.data = state;
        marker_state_pub_.publish(marker_state);
    }
};  // class AddMarkers


int main(int argc, char * * argv) {
    ros::init(argc, argv, "add_markers");

    // Create an AddMarkers object
    AddMarkers AMObject;

    // Handle ROS communication events
    ros::spin();

    return 0;
}
