#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sstream>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PickObjects {
  public:
    PickObjects(std::string the_frame_id = "map")
      : client_("move_base", true),
        frame_id_{the_frame_id}
    {}

    // This method sends a goal request using the MoveBaseClient
    void SetGoal(float x, float y, float rot) {
        // Create a goal object
        move_base_msgs::MoveBaseGoal goal;

        // set up the frame parameters
        goal.target_pose.header.frame_id = frame_id_;
        goal.target_pose.header.stamp = ros::Time::now();

        // Create a tf quaternion object to convert the yaw rotation input
        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, rot);

        // Define the position and orientation for the robot to reach
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        tf2::convert(q_rot, goal.target_pose.pose.orientation);

        // Keep waiting in 5 sec intervals for move_base action server to come up
        while(!client_.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        // Send the goal position and orientation for the robot to reach
        std::ostringstream goal_oss;
        goal_oss << "(" << x << ", " << y << ", " << rot <<  ")";
        ROS_INFO_STREAM("Sending goal: " + goal_oss.str());
        client_.sendGoal(goal);

        // Wait an infinite time for the results
        client_.waitForResult();

        // Check if the robot rached its goal
        if(client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO_STREAM("The base moved to goal: " + goal_oss.str());
        } else {
            ROS_INFO_STREAM("The base failed to move to goal: " + goal_oss.str());
        }
    }

  private:
    // The action client
    MoveBaseClient client_;

    // The fixed frame (can be specified via constructor argument, defaults to "map")
    std::string const frame_id_;
};  // class PickObjects

int main(int argc, char * * argv){
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");

    PickObjects pickObjects;
    pickObjects.SetGoal(1.0, 0.0, -1);
    pickObjects.SetGoal(2.0, 0.0, -3.1420);

    return 0;
}
