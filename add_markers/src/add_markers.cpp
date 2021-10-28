#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class AddMarkers {
  public:
    AddMarkers(std::string the_frame_id = "/map")
      : frame_id_{the_frame_id}
    {
        // Create a publisher for visualization_msgs::Marker type messages with queue size of 1
        marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }

    void Add(float x, float y, float rot, int id) const {
        // Create a marker object
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker. This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "goal_markers";
        marker.id = id;

        // Set the shape type to be a cube
        marker.type = visualization_msgs::Marker::CUBE;

        // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Create a tf quaternion object to convert the yaw rotation input
        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, rot);

        // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        tf2::convert(q_rot, marker.pose.orientation);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // Set the lifetime
        marker.lifetime = ros::Duration();

        std::ostringstream publish_text;
        publish_text << "Adding marker: " << id << "(" << x << ", " << y << ", " << rot <<  ")";

        publish_marker_(marker, publish_text);
    }

    void Delete(int id) const {
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
        publish_text << "Deleting marker: " << id;

        publish_marker_(marker, publish_text);
    }

    void DeleteAll() const {
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
        publish_text << "Deleting all markers";

        publish_marker_(marker, publish_text);
    }


  private:
    ros::NodeHandle n_;
    ros::Publisher marker_pub_;

    // The fixed frame (can be specified via constructor argument, defaults to "/map")
    std::string const frame_id_;

    // This method performs the publishinf of a created marker
    void publish_marker_(
        visualization_msgs::Marker & marker,
        std::ostringstream & text_oss
    ) const {
        // Publish the marker
        while (marker_pub_.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return;
            }

            ROS_WARN_ONCE("Please create a subscriber to the marker");
            ros::Duration(1.0).sleep();
        }

        ROS_INFO_STREAM(text_oss.str());
        marker_pub_.publish(marker);
    }
};  // class AddMarkers

int main(int argc, char * * argv) {
    ros::init(argc, argv, "add_markers");

    AddMarkers addMarkers;

    addMarkers.Add(1.0, 0.0, -1.0, 10);
    sleep(5);


    addMarkers.Delete(10);
    sleep(5);

    addMarkers.Add(1.0, 1.0, -2.0, 20);
    sleep(5);

    addMarkers.DeleteAll();
}
