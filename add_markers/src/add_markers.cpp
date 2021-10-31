#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>

#include <map>
#include <vector>

#include "pick_objects/MoveToPose.h"

class AddMarkers {
  public:
    AddMarkers(std::string the_frame_id = "/map")
      : frame_id_{the_frame_id},
        last_move_state{0}
    {
        // Publisher for visualization_msgs::Marker type messages with queue size of 1
        marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        // Subscriber on the /move_state topic receoving updates on the robot's moving state and storing it in the last_move_state varable via the StoreMoveState callback function
        move_state_sub_ = n_.subscribe(
            "/move_state",
            10,
            &AddMarkers::StoreMoveState,
            this
        );

        // Client capable of requesting services from /pick_objects/move_robot
        move_robot_client_ = n_.serviceClient<pick_objects::MoveToPose>(
            "/pick_objects/move_robot"
        );
    }

    // The robot's last received moving state
    int last_move_state;

    // Callback to store the received robot's moving state
    void StoreMoveState(std_msgs::Int8 const & msg) {
        last_move_state = msg.data;
    }

    // Method to move the robot to a marker (specified by its ID)
    bool MoveRobot(int id) {
        // Request move to marker's pose
        pick_objects::MoveToPose srv;
        srv.request.pose = poses_[id];
        srv.request.pose.position.z = 0;  // in case the marker was higher (just for demonstration)

        // Call the move_robot service and pass the requested pose
        if (!move_robot_client_.call(srv)) {
        ROS_ERROR("Failed to call service move_robot");
            return 0;
        } else {
            return 1;
        }
    }

    // Method to create a new marker entry, store its pose and return its ID
    int New(float x, float y, float rot) {
        // Create a geometry_msgs/Pose object to properly store the pose
        geometry_msgs::Pose pose;

        // Create a tf quaternion object to convert the yaw rotation input
        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, rot);
        q_rot.normalize();

        // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.5;
        tf2::convert(q_rot, pose.orientation);

        int id = poses_.size();
        poses_.push_back(pose);

        return id;
    }

    // Method to show a marker specified by its ID
    int Show(int id) const {
        // Create a marker object
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker. This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "goal_markers";
        marker.id = id;

        // Set the shape type
        marker.type = visualization_msgs::Marker::SPHERE;

        // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose = poses_[id];

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

        std::ostringstream publish_text;
        publish_text << "Showing marker: ["
            << marker.id << "] ("
            << marker.pose.position.x << ", "
            << marker.pose.position.y <<  ")";

        publish_marker_(marker, publish_text);

        return marker.id;
    }

    // Method to delete maker specified by its ID
    void Hide(int id) const {
        // Create a marker object
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker. This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "goal_markers";
        marker.id = id;

        // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::DELETE;

        std::ostringstream publish_text;
        publish_text << "Hiding marker: [" << id << "]";

        publish_marker_(marker, publish_text);
    }

    // Method to delete all shown markers
    void HideAll() const {
        // Create a marker object
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker. This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "goal_markers";

        // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::DELETEALL;

        std::ostringstream publish_text;
        publish_text << "Hiding all markers";

        publish_marker_(marker, publish_text);
    }

  private:
    ros::NodeHandle n_;
    ros::Publisher marker_pub_;
    ros::Subscriber move_state_sub_;
    ros::ServiceClient move_robot_client_;

    // Map to store the markers' IDs and poses
    std::vector<geometry_msgs::Pose> poses_;

    // The fixed frame (can be specified via constructor argument, defaults to "/map")
    std::string const frame_id_;

    // Method to perform the publishing of a created marker
    void publish_marker_(
        visualization_msgs::Marker & marker,
        std::ostringstream & text_oss
    ) const {
        // Check if anyone's listening, at 1 sec intervals
        while (marker_pub_.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return;
            }

            ROS_WARN_ONCE("Please create a subscriber to the marker");
            ros::Duration(1.0).sleep();
        }

        // Publish the marker
        ROS_INFO_STREAM(text_oss.str());
        marker_pub_.publish(marker);
    }
};  // class AddMarkers


int main(int argc, char * * argv) {
    ros::init(argc, argv, "add_markers");

    // Create an AddMarkers object
    AddMarkers AMObject;

    // Perform the pickup
    int pickup_marker = AMObject.New(3.0, 0.0, -1.0);
    AMObject.Show(pickup_marker);
    if (AMObject.MoveRobot(pickup_marker)) {
        AMObject.Hide(pickup_marker);
        sleep(5);
    } else {
        return 1;
    }

    // Perform the dropoff
    int dropoff_marker = AMObject.New(3.0, 2.0, -1.0);
    if (AMObject.MoveRobot(dropoff_marker)) {;
        AMObject.Show(dropoff_marker);
    } else {
        return 1;
    }

    return 0;
}
