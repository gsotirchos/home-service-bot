#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h>

#include "pick_objects/MarkerPose.h"


class AddMarkers {
  public:
    AddMarkers(std::string the_frame_id = "/map")
      : frame_id_{the_frame_id},
        last_robot_state{0}
    {
        // Publisher for visualization_msgs::Marker type messages with queue size of 1
        marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        // Subscriber on the /robot_state topic receoving updates on the robot's moving state and storing it in the last_robot_state varable via the StoreMoveState callback function
        move_state_sub_ = n_.subscribe(
            "/robot_state",
            10,
            &AddMarkers::StoreMoveState,
            this
        );

        // Server for showing a new marker given its coordinates
        show_marker_server_ = n_.advertiseService(
            "/add_markers/show_marker",
            &AddMarkers::ShowMarker,
            this
        );

        // Server for the /pick_objects/hide_all_markers service for hiding all shown markers
        hide_markers_server_ = n_.advertiseService(
            "/add_markers/hide_all_markers",
            &AddMarkers::HideAllMarkers,
            this
        );

        ROS_INFO("Ready to add markers");
    }

    // The robot's last received moving state
    int last_robot_state;

    // Callback to store the received robot's moving state
    void StoreMoveState(std_msgs::Int8 const & msg) {
        last_robot_state = msg.data;
    }

    // Method to show a marker
    bool ShowMarker(
        pick_objects::MarkerPose::Request & req,
        pick_objects::MarkerPose::Response & res
    ) {
        // Create a marker object
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker. This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "goal_markers";
        marker.id = 0;

        // Set the shape type
        marker.type = visualization_msgs::Marker::SPHERE;

        // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Create a tf quaternion object to convert the yaw rotation input
        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, req.rot);
        q_rot.normalize();

        // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = req.x;
        marker.pose.position.y = req.y;
        marker.pose.position.z = 0.5;
        tf2::convert(q_rot, marker.pose.orientation);

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

        std::ostringstream marker_oss;
        marker_oss << "(" << req.x << ", " << req.y << ")";
        res.msg_feedback = "Showing marker " + marker_oss.str();
        ROS_INFO_STREAM(res.msg_feedback);

        PublishMarker_(marker);

        return true;
    }

    // Method to hide all shown markers
    bool HideAllMarkers(
        std_srvs::Trigger::Request & req,
        std_srvs::Trigger::Response & res
    ) {
        // Create a marker object
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();

        // Set the namespace.
        marker.ns = "goal_markers";

        // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::DELETEALL;

        res.success = true;
        res.message = "Hiding all markers";
        ROS_INFO_STREAM(res.message);

        PublishMarker_(marker);

        return true;
    }

  private:
    ros::NodeHandle n_;
    ros::Publisher marker_pub_;
    ros::Subscriber move_state_sub_;
    ros::ServiceServer show_marker_server_;
    ros::ServiceServer hide_markers_server_;

    // The fixed frame (can be specified via constructor argument, defaults to "/map")
    std::string const frame_id_;

    // Method to perform the publishing of a created marker
    void PublishMarker_(visualization_msgs::Marker & marker) const {
        // Check if anyone's listening, at 1 sec intervals
        while (marker_pub_.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return;
            }

            ROS_WARN_ONCE("Please create a subscriber to the marker");
            ros::Duration(1.0).sleep();
        }

        // Publish the marker
        marker_pub_.publish(marker);
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
